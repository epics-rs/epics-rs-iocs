//! The PSLViewer socket (C `writeReadServer`).
//!
//! # Ownership
//!
//! The server speaks one command at a time, and several driver operations are
//! sequences of commands that must not interleave: `openCamera` is eight
//! commands, `getImage` is a `GetImage` followed by however many raw reads the
//! payload needs. C got that atomicity from the ADDriver mutex, which both the
//! parameter path and the acquisition thread held. Here the socket sits behind
//! [`PslServer`]'s own mutex, and a caller that needs a sequence to be atomic
//! holds one [`Session`] across it.
//!
//! **Invariant: no caller may touch the detector port's parameter library
//! while holding a `Session`.** The port actor takes the session lock inside
//! `write_int32`; a task that held the session and then waited on the actor
//! would close the cycle. The acquisition task therefore drops its session
//! before it publishes parameters or arrays.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::user::AsynUser;
use parking_lot::{Mutex, MutexGuard};

use crate::types::{GET_IMAGE_TIMEOUT, MAX_MESSAGE_SIZE, SERVER_TIMEOUT};

/// What an exchange with the server can fail with.
#[derive(Debug)]
pub enum ServerError {
    /// The server did not answer in time.
    Timeout,
    /// Socket failure.
    Transport(AsynError),
}

impl std::fmt::Display for ServerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout => write!(f, "timeout"),
            Self::Transport(e) => write!(f, "{e}"),
        }
    }
}

impl From<AsynError> for ServerError {
    fn from(e: AsynError) -> Self {
        // `status()` reads *through* a `PartialRead` carrier, so a timeout that
        // returned partial bytes (EOS interpose installed) is still recognized.
        if e.status() == AsynStatus::Timeout {
            Self::Timeout
        } else {
            Self::Transport(e)
        }
    }
}

pub type ServerResult<T> = Result<T, ServerError>;

/// The PSLViewer server socket.
#[derive(Clone)]
pub struct PslServer {
    io: Arc<Mutex<ServerIo>>,
}

impl PslServer {
    pub fn new(handle: PortHandle) -> Self {
        Self {
            io: Arc::new(Mutex::new(ServerIo { handle })),
        }
    }

    /// Take the socket for a sequence of commands that must not interleave
    /// with anyone else's.
    pub fn session(&self) -> Session<'_> {
        Session { io: self.io.lock() }
    }

    /// One command, one reply.
    pub fn command(&self, command: &str) -> ServerResult<String> {
        self.session().command(command)
    }
}

/// Exclusive use of the socket.
pub struct Session<'a> {
    io: MutexGuard<'a, ServerIo>,
}

impl Session<'_> {
    /// Send a command and read the reply as text (C `writeReadServer` with the
    /// default `PSL_SERVER_TIMEOUT`).
    pub fn command(&mut self, command: &str) -> ServerResult<String> {
        let bytes = self
            .io
            .write_read(command, MAX_MESSAGE_SIZE, SERVER_TIMEOUT)?;
        let reply = String::from_utf8_lossy(&bytes).trim_end().to_string();
        log::debug!("psl: '{command}' -> '{reply}'");
        Ok(reply)
    }

    /// Ask for a frame and read the first block of the answer: the text header
    /// and as much of the payload as arrived with it.
    ///
    /// The payload is binary, so the read must not be cut short or stripped by
    /// an input end-of-string; `OctetReadBinary` suppresses one if the IOC has
    /// configured it. C reconnects the socket before every command, so nothing
    /// can be left in the input buffer from the previous exchange.
    pub fn request_image(&mut self) -> ServerResult<Vec<u8>> {
        self.io
            .reconnect()
            .and_then(|()| self.io.write("GetImage", GET_IMAGE_TIMEOUT))
            .and_then(|()| self.io.read_binary(MAX_MESSAGE_SIZE, GET_IMAGE_TIMEOUT))
    }

    /// Read the next block of an image payload (C `pasynOctetSyncIO->read`
    /// inside the copy loop).
    pub fn read_image_block(&mut self, max_bytes: usize) -> ServerResult<Vec<u8>> {
        self.io.read_binary(max_bytes, GET_IMAGE_TIMEOUT)
    }
}

struct ServerIo {
    handle: PortHandle,
}

impl ServerIo {
    fn user(&self, timeout: Duration) -> AsynUser {
        AsynUser::new(0).with_addr(0).with_timeout(timeout)
    }

    /// The server expects a fresh connection for every command
    /// (C `pasynCommonSyncIO->disconnectDevice` + `connectDevice`).
    fn reconnect(&mut self) -> ServerResult<()> {
        self.handle
            .submit_blocking(RequestOp::Disconnect, self.user(SERVER_TIMEOUT))?;
        self.handle
            .submit_blocking(RequestOp::Connect, self.user(SERVER_TIMEOUT))?;
        Ok(())
    }

    fn write_read(
        &mut self,
        command: &str,
        buf_size: usize,
        timeout: Duration,
    ) -> ServerResult<Vec<u8>> {
        self.reconnect()?;
        let result = self.handle.submit_blocking(
            RequestOp::OctetWriteRead {
                data: command.as_bytes().to_vec(),
                buf_size,
                flush: true,
            },
            self.user(timeout),
        )?;
        Ok(result.data.unwrap_or_default())
    }

    fn write(&mut self, command: &str, timeout: Duration) -> ServerResult<()> {
        self.handle.submit_blocking(
            RequestOp::OctetWrite {
                data: command.as_bytes().to_vec(),
            },
            self.user(timeout),
        )?;
        Ok(())
    }

    fn read_binary(&mut self, buf_size: usize, timeout: Duration) -> ServerResult<Vec<u8>> {
        let result = self
            .handle
            .submit_blocking(RequestOp::OctetReadBinary { buf_size }, self.user(timeout))?;
        Ok(result.data.unwrap_or_default())
    }
}

#[cfg(test)]
mod is_timeout_tests {
    use super::*;
    use epics_rs::asyn::interpose::{EomReason, PartialOctetRead};

    /// A read that times out *after* transferring bytes arrives as
    /// `AsynError::PartialRead` when the EOS interpose is installed (this
    /// port's st.cmd uses `noProcessEos=0`). A bare `AsynError::Status`
    /// match missed it and misclassified the timeout as a transport fault.
    #[test]
    fn a_partial_read_wrapped_timeout_maps_to_timeout() {
        let e = AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        }
        .with_partial_read(PartialOctetRead {
            data: b"partial".to_vec(),
            eom_reason: EomReason::empty(),
        });
        assert!(matches!(ServerError::from(e), ServerError::Timeout));
    }

    #[test]
    fn a_real_error_maps_to_transport() {
        let e = AsynError::Status {
            status: AsynStatus::Error,
            message: "boom".into(),
        };
        assert!(matches!(ServerError::from(e), ServerError::Transport(_)));
    }
}
