//! The p2util socket (C `writePhotonII` / `readPhotonII`).
//!
//! One `drvAsynIPPort` to the procServ that runs p2util. The port actor
//! serialises every operation, so a write-read is atomic against any other
//! command even though both the driver actor and the acquisition task use this
//! handle.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynStatus, OctetStep};
use epics_rs::asyn::sync_io::SyncIOHandle;

use crate::types::MAX_MESSAGE_SIZE;

/// What a p2util exchange can fail with.
#[derive(Debug)]
pub enum ChannelError {
    /// The read timed out — normal while p2util is still exposing.
    Timeout,
    /// Socket failure.
    Transport(AsynError),
}

impl std::fmt::Display for ChannelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout => write!(f, "timeout"),
            Self::Transport(e) => write!(f, "{e}"),
        }
    }
}

impl From<AsynError> for ChannelError {
    fn from(e: AsynError) -> Self {
        if e.status() == AsynStatus::Timeout {
            Self::Timeout
        } else {
            Self::Transport(e)
        }
    }
}

/// The command channel to p2util.
#[derive(Clone)]
pub struct P2Util {
    handle: Arc<SyncIOHandle>,
}

impl P2Util {
    pub fn new(handle: SyncIOHandle) -> Self {
        Self {
            handle: Arc::new(handle),
        }
    }

    /// Send one command line and return p2util's reply (C `writePhotonII`,
    /// which is `asynOctetSyncIO::writeRead`: flush, write, read one line).
    /// The reply is what the read left in C's `fromPhotonII_`, also when it
    /// failed; `None` when no read ran and the buffer kept the last reply.
    pub fn write_read(
        &self,
        command: &str,
        timeout: Duration,
    ) -> (Option<String>, Result<(), ChannelError>) {
        let (reply, status) = self
            .handle
            .with_timeout(timeout)
            .write_read(0, command.as_bytes(), MAX_MESSAGE_SIZE)
            .into_parts();
        let read = match &status {
            Ok(()) => true,
            Err(e) => e.octet_step() == Some(OctetStep::Read),
        };
        (
            read.then(|| decode(reply)),
            status.map_err(ChannelError::from),
        )
    }

    /// Read one line p2util sent unprompted (C `readPhotonII`), e.g. the
    /// "wrote N bytes to ..." message that follows every frame, a timeout's
    /// bytes included; `None` when the request never ran.
    pub fn read(&self, timeout: Duration) -> (Option<String>, Result<(), ChannelError>) {
        // C `readIt` leaves the buffer alone when queueLockPort fails
        // (asynOctetSyncIO.c:213-216), a missing port included.
        if let Err(e) = self.handle.port() {
            return (None, Err(e.into()));
        }
        let (line, status) = self
            .handle
            .with_timeout(timeout)
            .read_octet(0, MAX_MESSAGE_SIZE)
            .into_parts();
        let read = status.as_ref().map_or_else(|e| !e.never_ran(), |()| true);
        (
            read.then(|| decode(line)),
            status.map_err(ChannelError::from),
        )
    }
}

/// `fromPhotonII_` as C posts it: up to the NUL the read appended.
fn decode(data: Vec<u8>) -> String {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    String::from_utf8_lossy(&data[..end]).into_owned()
}

#[cfg(test)]
mod is_timeout_tests {
    use super::*;
    use epics_rs::asyn::user::AsynUser;

    /// Trailing blanks and the `\r` of an EOS a timeout split stay in the
    /// line C posts (asynInterposeEos.c:89, PhotonII.cpp:260).
    #[test]
    fn a_line_keeps_its_trailing_bytes() {
        assert_eq!(decode(b"wrote 3 \r".to_vec()), "wrote 3 \r");
        assert_eq!(decode(b"ok\0junk".to_vec()), "ok");
    }

    #[test]
    fn a_timeout_maps_to_timeout() {
        let e = AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        };
        assert!(matches!(ChannelError::from(e), ChannelError::Timeout));
    }

    /// A p2util port that takes every write and answers every read `ok`.
    struct Answers(epics_rs::asyn::port::PortDriverBase);

    impl epics_rs::asyn::port::PortDriver for Answers {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.0
        }
        fn io_write_octet(
            &mut self,
            _user: &mut AsynUser,
            data: &[u8],
        ) -> epics_rs::asyn::error::AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
            buf[..2].copy_from_slice(b"ok");
            OctetReadResult::ok(2, EomReason::empty())
        }
    }

    /// `writePhotonII` and `readPhotonII` are `pasynOctetSyncIO` writeRead and
    /// read (PhotonII.cpp:226, :249), which print their TRACEIO_DEVICE lines
    /// (asynOctetSyncIO.c:219-223, :259-266).
    #[test]
    fn p2util_io_prints_its_sync_io_lines() {
        use epics_rs::asyn::port::{InterfaceMask, PortDriverBase};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceIoMask, TraceMask};

        let name = "PII_LINES";
        let (rt, _) = create_port_runtime(
            &claim_port(name).unwrap(),
            Answers(PortDriverBase::new(
                name,
                1,
                Default::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            )),
            RuntimeConfig::default(),
        )
        .unwrap();
        let trace = rt.port_handle().trace();
        let path = std::env::temp_dir().join(format!("{name}_{}", std::process::id()));
        trace.set_trace_file(
            Some(name),
            TraceFile::File(Arc::new(std::sync::Mutex::new(
                std::fs::File::create(&path).unwrap(),
            ))),
        );
        trace.set_trace_mask(Some(name), TraceMask::IO_DEVICE);
        trace.set_trace_io_mask(Some(name), TraceIoMask::ASCII);
        let p2 = P2Util::new(SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            Duration::ZERO,
        ));
        assert!(p2.write_read("[cmd]", Duration::from_secs(1)).1.is_ok());
        assert!(p2.read(Duration::from_secs(1)).1.is_ok());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(text.contains("asynOctetSyncIO wrote:\n[cmd]"), "{text}");
        assert_eq!(
            text.matches("asynOctetSyncIO read:\nok").count(),
            2,
            "{text}"
        );
        std::mem::forget(rt);
    }

    #[test]
    fn a_real_error_maps_to_transport() {
        let e = AsynError::Status {
            status: AsynStatus::Error,
            message: "boom".into(),
        };
        assert!(matches!(ChannelError::from(e), ChannelError::Transport(_)));
    }
}
