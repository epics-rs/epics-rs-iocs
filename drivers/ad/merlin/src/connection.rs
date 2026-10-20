//! MPX framing over an asyn octet port (C `mpxConnection`).
//!
//! One `MpxConnection` owns one socket: the command channel and the data
//! channel each get their own. C shared a single `mpxConnection` object
//! between the two — `merlinTask` called `cmdConnection->mpxRead()` with the
//! *data* channel's asynUser, so a data frame overwrote the command channel's
//! response buffers and error code while the command thread could be mid
//! transaction.

use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::ad_core::driver::ADDriverBase;
use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;

use crate::protocol::{self, MpxError};
use crate::types::*;

/// Anything that can go wrong on an MPX channel.
#[derive(Debug)]
pub enum ChannelError {
    /// The socket read timed out — expected on an idle data channel.
    Timeout,
    /// Transport failure (socket closed, port disconnected, ...).
    Transport(AsynError),
    /// The peer spoke MPX badly, or reported an error code.
    Protocol(MpxError),
}

impl std::fmt::Display for ChannelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout => write!(f, "timeout"),
            Self::Transport(e) => write!(f, "{e}"),
            Self::Protocol(e) => write!(f, "{e}"),
        }
    }
}

impl ChannelError {
    /// The `asynStatus` C prints for this failure.
    pub(crate) fn code(&self) -> i32 {
        match self {
            Self::Timeout => AsynStatus::Timeout as i32,
            Self::Transport(e) => e.status() as i32,
            Self::Protocol(_) => AsynStatus::Error as i32,
        }
    }
}

/// What C's `mpxSet`/`mpxCommand` return to `writeInt32`/`writeFloat64`.
impl From<ChannelError> for AsynError {
    fn from(e: ChannelError) -> Self {
        match e {
            ChannelError::Transport(e) => e,
            ChannelError::Timeout => AsynError::Status {
                status: AsynStatus::Timeout,
                message: "timeout".into(),
            },
            ChannelError::Protocol(e) => AsynError::Status {
                status: AsynStatus::Error,
                message: e.to_string(),
            },
        }
    }
}

impl From<MpxError> for ChannelError {
    fn from(e: MpxError) -> Self {
        Self::Protocol(e)
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

/// A framed MPX channel on top of one asyn octet port.
pub struct MpxConnection {
    io: SyncIOHandle,
    max_frame: usize,
}

impl MpxConnection {
    pub fn new(io: SyncIOHandle, max_frame: usize) -> Self {
        Self { io, max_frame }
    }

    /// C prints these on the asynUser of the socket being read or written.
    pub(crate) fn trace(&self, msg: &str) {
        self.io.print(0, TraceMask::ERROR, format_args!("{msg}"));
    }

    /// Read exactly `count` more bytes into `out`. `Err(None)` is C's read
    /// that succeeded with zero bytes: the peer closed the socket.
    fn read_exact(
        &self,
        out: &mut Vec<u8>,
        count: usize,
        timeout: Duration,
    ) -> Result<(), Option<ChannelError>> {
        let want = out.len() + count;
        while out.len() < want {
            // C adds `nread` only on asynSuccess (mpxConnection.cpp:647-648).
            let (chunk, status) = self
                .io
                .with_timeout(timeout)
                .read_octet(0, want - out.len())
                .into_parts();
            status.map_err(|e| Some(e.into()))?;
            if chunk.is_empty() {
                return Err(None);
            }
            out.extend_from_slice(&chunk);
        }
        Ok(())
    }

    /// Read one MPX frame, discarding any bytes that precede the `MPX` magic
    /// so the channel re-syncs after an error or a server restart. `header`
    /// receives the frame header once it is complete (C `fromLabviewHeader`).
    ///
    /// C matched the magic with a counter it reset to zero on every mismatch,
    /// so junk ending in a prefix of the magic (`"...MMPX"`) desynchronised it
    /// permanently. This slides the window by one byte instead.
    fn read_frame_into(
        &self,
        header: &mut String,
        timeout: Duration,
    ) -> Result<Vec<u8>, ChannelError> {
        let timeout_s = timeout.as_secs_f64();
        let closed = || {
            ChannelError::Transport(AsynError::Status {
                status: AsynStatus::Disconnected,
                message: "peer closed the MPX socket".into(),
            })
        };
        // C prints `fromLabview`, which `mpxReadCmd` clears before every read
        // and which this connection never fills in between.
        let mut window = Vec::with_capacity(MPX_FRAME_HEADER_LEN);
        self.read_exact(&mut window, MPX_HEADER.len(), timeout)
            .map_err(|e| e.unwrap_or_else(closed))?;
        let mut junk = 0usize;
        while window != MPX_HEADER {
            window.remove(0);
            self.read_exact(&mut window, 1, timeout)
                .map_err(|e| e.unwrap_or_else(closed))?;
            junk += 1;
        }
        if junk > 0 {
            self.trace(&format!(
                "merlinDetector:mpxRead, status=0 {junk} bytes of leading garbage discarded before header  \n"
            ));
        }

        let rest = MPX_FRAME_HEADER_LEN - MPX_HEADER.len();
        match self.read_exact(&mut window, rest, timeout) {
            Ok(()) => {}
            Err(None) => {
                self.trace("merlinDetector:mpxRead, Header too short\n");
                return Err(closed());
            }
            Err(Some(e)) => {
                self.trace(&format!(
                    "merlinDetector:mpxRead, timeout={timeout_s:.6}, status={} received {} bytes\n\n",
                    e.code(),
                    window.len() - MPX_HEADER.len(), timeout_s = CFloat(timeout_s)
                ));
                return Err(e);
            }
        }
        *header = String::from_utf8_lossy(&window).into_owned();
        let body_len = match protocol::parse_frame_header(&window, self.max_frame) {
            Ok(n) => n,
            Err(e) => {
                if let MpxError::BadBodySize(n) = e {
                    self.trace(&format!(
                        "merlinDetector:mpxRead, frame size {n} not supported\n"
                    ));
                }
                return Err(e.into());
            }
        };

        // C returned asynSuccess on a short body read (`status = asynSuccess ?
        // asynError : status` always took the else branch), handing the caller
        // a truncated frame it believed was complete. This fails it instead.
        let mut body = Vec::with_capacity(body_len);
        if let Err(e) = self.read_exact(&mut body, body_len, timeout) {
            self.trace(&format!(
                "merlinDetector:mpxRead, timeout={timeout_s:.6}, status={} received {} bytes in MPX command body, expected {body_len}\n",
                e.as_ref().map_or(0, ChannelError::code),
                body.len(), timeout_s = CFloat(timeout_s)
            ));
            return Err(e.unwrap_or_else(closed));
        }
        Ok(body)
    }

    pub fn read_frame(&self, timeout: Duration) -> Result<Vec<u8>, ChannelError> {
        self.read_frame_into(&mut String::new(), timeout)
    }

    /// Send one command and read frames until the reply that echoes it comes
    /// back (C `mpxWriteRead`). Anything else on the wire is reported and
    /// dropped, which is how the channel recovers from a desynchronised
    /// server.
    ///
    /// The command lands on `ADStringToServer` whatever the write did, and
    /// every frame read lands on `ADStringFromServer`, stray ones included
    /// (mpxConnection.cpp:571/:763); C posts neither.
    fn transact(
        &self,
        ad: &mut ADDriverBase,
        kind: MpxKind,
        name: &str,
        value: Option<&str>,
        timeout: Duration,
    ) -> Result<Vec<u8>, ChannelError> {
        let msg = protocol::encode(kind, name, value)?;
        // C `mpxWrite` reports a failed write and still reads the reply.
        if let Err(e) = self.io.with_timeout(timeout).write_octet(0, msg.as_bytes()) {
            self.trace(&format!(
                "merlinDetector:mpxWrite, status={}, sent\n{msg}\n",
                ChannelError::from(e).code()
            ));
        }
        let _ = ad
            .port_base
            .set_string_param(ad.params.string_to_server, 0, msg.clone());

        loop {
            let mut header = String::new();
            let read = self.read_frame_into(&mut header, timeout);
            if let Ok(body) = &read {
                let from = format!("{header}{}", String::from_utf8_lossy(body));
                let _ = ad
                    .port_base
                    .set_string_param(ad.params.string_from_server, 0, from);
            }
            let (code, body) = match &read {
                Ok(body) if protocol::response_echoes(body, kind, name) => break read,
                Ok(body) => (0, String::from_utf8_lossy(body).into_owned()),
                Err(e) => (e.code(), String::new()),
            };
            self.trace(&format!(
                "merlinDetector:mpxReadCmd error, status={code} unexpected response from labview: '{header}{body}'\n"
            ));
            read?;
        }
    }

    /// `SET <name> <value>`.
    pub fn set(
        &self,
        ad: &mut ADDriverBase,
        name: &str,
        value: &str,
        timeout: Duration,
    ) -> Result<(), ChannelError> {
        let body = self.transact(ad, MpxKind::Set, name, Some(value), timeout)?;
        protocol::decode_ack(&body)?;
        Ok(())
    }

    /// `GET <name>`, returning the device's value field.
    pub fn get(
        &self,
        ad: &mut ADDriverBase,
        name: &str,
        timeout: Duration,
    ) -> Result<String, ChannelError> {
        let body = self.transact(ad, MpxKind::Get, name, None, timeout)?;
        Ok(protocol::decode_get(&body)?)
    }

    /// `CMD <name>`.
    pub fn command(
        &self,
        ad: &mut ADDriverBase,
        name: &str,
        timeout: Duration,
    ) -> Result<(), ChannelError> {
        let body = self.transact(ad, MpxKind::Cmd, name, None, timeout)?;
        protocol::decode_ack(&body)?;
        Ok(())
    }

    /// `GET <name>` parsed as a float, or `None` if the device refused.
    pub fn get_f64(&self, ad: &mut ADDriverBase, name: &str, timeout: Duration) -> Option<f64> {
        match self.get(ad, name, timeout) {
            Ok(v) => v.trim().parse().ok(),
            Err(_) => None,
        }
    }

    /// `GET <name>` parsed as an integer, or `None` if the device refused.
    pub fn get_i32(&self, ad: &mut ADDriverBase, name: &str, timeout: Duration) -> Option<i32> {
        match self.get(ad, name, timeout) {
            Ok(v) => v.trim().parse().ok(),
            Err(_) => None,
        }
    }
}

/// The command timeout every parameter write uses (C
/// `Labview_DEFAULT_TIMEOUT`).
pub fn default_timeout() -> Duration {
    Duration::from_secs_f64(LABVIEW_DEFAULT_TIMEOUT_SEC)
}

#[cfg(test)]
mod is_timeout_tests {
    use super::*;
    use epics_rs::asyn::user::AsynUser;

    #[test]
    fn a_timeout_maps_to_timeout() {
        let e = AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        };
        assert!(matches!(ChannelError::from(e), ChannelError::Timeout));
    }

    /// A Merlin command socket: each write queues the reply the fake rig
    /// sends, each read takes what is queued, up to its buffer.
    struct Mpx {
        base: epics_rs::asyn::port::PortDriverBase,
        queued: Vec<u8>,
    }

    impl epics_rs::asyn::port::PortDriver for Mpx {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(
            &mut self,
            _user: &mut AsynUser,
            data: &[u8],
        ) -> epics_rs::asyn::error::AsynResult<usize> {
            let body = b"GET,X,1,0";
            self.queued = format!("MPX,{:010},", body.len() + 1).into_bytes();
            self.queued.extend_from_slice(body);
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
            let n = buf.len().min(self.queued.len());
            buf[..n].copy_from_slice(&self.queued[..n]);
            self.queued.drain(..n);
            OctetReadResult::ok(n, EomReason::empty())
        }
    }

    /// `mpxWrite` and `mpxRead` are `pasynOctetSyncIO` write and read
    /// (mpxConnection.cpp:561, :617, :645, :701), which print their
    /// TRACEIO_DEVICE lines (asynOctetSyncIO.c:194-200, :219-223).
    #[test]
    fn a_transaction_prints_its_sync_io_lines() {
        use epics_rs::asyn::port::{InterfaceMask, PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceIoMask};

        let name = "MERLIN_LINES";
        let (rt, _) = create_port_runtime(
            &claim_port(name).unwrap(),
            Mpx {
                base: PortDriverBase::new(
                    name,
                    1,
                    Default::default(),
                    InterfaceMask::UNMIGRATED_ALL,
                    InterfaceMask::UNMIGRATED_INTERRUPTS,
                ),
                queued: Vec::new(),
            },
            RuntimeConfig::default(),
        )
        .unwrap();
        let trace = rt.port_handle().trace();
        let path = std::env::temp_dir().join(format!("{name}_{}", std::process::id()));
        trace.set_trace_file(
            Some(name),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(
                std::fs::File::create(&path).unwrap(),
            ))),
        );
        trace.set_trace_mask(Some(name), TraceMask::IO_DEVICE);
        trace.set_trace_io_mask(Some(name), TraceIoMask::ASCII);
        let conn = MpxConnection::new(
            SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::ZERO),
            1 << 20,
        );
        let mut ad = ADDriverBase::new(
            "MERLIN_LINES_DET",
            1,
            1 << 20,
            PortFlags::default(),
            InterfaceMask::UNMIGRATED_SUBCLASS,
            InterfaceMask::UNMIGRATED_SUBCLASS_INTERRUPTS,
        )
        .unwrap();
        assert_eq!(conn.get(&mut ad, "X", Duration::from_secs(1)).unwrap(), "1");
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(text.contains("asynOctetSyncIO wrote:\nMPX,"), "{text}");
        assert!(text.contains("asynOctetSyncIO read:\nMPX\n"), "{text}");
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
