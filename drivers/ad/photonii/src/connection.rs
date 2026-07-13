//! The p2util socket (C `writePhotonII` / `readPhotonII`).
//!
//! One `drvAsynIPPort` to the procServ that runs p2util. The port actor
//! serialises every operation, so a write-read is atomic against any other
//! command even though both the driver actor and the acquisition task use this
//! handle.

use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
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
        match &e {
            AsynError::Status {
                status: AsynStatus::Timeout,
                ..
            } => Self::Timeout,
            _ => Self::Transport(e),
        }
    }
}

/// The command channel to p2util.
#[derive(Clone)]
pub struct P2Util {
    handle: PortHandle,
}

impl P2Util {
    pub fn new(handle: PortHandle) -> Self {
        Self { handle }
    }

    fn user(&self, timeout: Duration) -> AsynUser {
        AsynUser::new(0).with_addr(0).with_timeout(timeout)
    }

    /// Send one command line and return p2util's reply (C `writePhotonII`,
    /// which is `asynOctetSyncIO::writeRead`: flush, write, read one line).
    pub fn write_read(&self, command: &str, timeout: Duration) -> Result<String, ChannelError> {
        log::debug!("photonii -> {command}");
        let result = self.handle.submit_blocking(
            RequestOp::OctetWriteRead {
                data: command.as_bytes().to_vec(),
                buf_size: MAX_MESSAGE_SIZE,
                flush: true,
            },
            self.user(timeout),
        )?;
        let reply = decode(result.data.unwrap_or_default());
        log::debug!("photonii <- {reply}");
        Ok(reply)
    }

    /// Read one line p2util sent unprompted (C `readPhotonII`), e.g. the
    /// "wrote N bytes to ..." message that follows every frame.
    pub fn read(&self, timeout: Duration) -> Result<String, ChannelError> {
        let result = self.handle.submit_blocking(
            RequestOp::OctetRead {
                buf_size: MAX_MESSAGE_SIZE,
            },
            self.user(timeout),
        )?;
        Ok(decode(result.data.unwrap_or_default()))
    }
}

fn decode(data: Vec<u8>) -> String {
    String::from_utf8_lossy(&data).trim_end().to_string()
}
