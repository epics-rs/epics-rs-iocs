//! The p2util socket (C `writePhotonII` / `readPhotonII`).
//!
//! One `drvAsynIPPort` to the procServ that runs p2util. The port actor
//! serialises every operation, so a write-read is atomic against any other
//! command even though both the driver actor and the acquisition task use this
//! handle.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynStatus, OctetStep};
use epics_rs::asyn::request::OctetReadOp;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

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

    fn user(&self, timeout: Duration) -> AsynUser {
        AsynUser::new(0).with_addr(0).with_timeout(timeout)
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
        let port = match self.handle.port() {
            Ok(port) => port,
            Err(e) => return (None, Err(e.into())),
        };
        let (reply, status) = port
            .submit_octet_read_blocking(
                OctetReadOp::WriteRead {
                    data: command.as_bytes().to_vec(),
                    buf_size: MAX_MESSAGE_SIZE,
                },
                self.user(timeout),
            )
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
        let port = match self.handle.port() {
            Ok(port) => port,
            Err(e) => return (None, Err(e.into())),
        };
        let (line, status) = port
            .submit_octet_read_blocking(
                OctetReadOp::Read {
                    buf_size: MAX_MESSAGE_SIZE,
                },
                self.user(timeout),
            )
            .into_parts();
        let read = status.as_ref().map_or_else(|e| !e.never_ran(), |()| true);
        (
            read.then(|| decode(line)),
            status.map_err(ChannelError::from),
        )
    }
}

fn decode(data: Vec<u8>) -> String {
    String::from_utf8_lossy(&data).trim_end().to_string()
}

#[cfg(test)]
mod is_timeout_tests {
    use super::*;

    #[test]
    fn a_timeout_maps_to_timeout() {
        let e = AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        };
        assert!(matches!(ChannelError::from(e), ChannelError::Timeout));
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
