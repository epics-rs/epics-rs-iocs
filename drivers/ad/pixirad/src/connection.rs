//! The box's TCP command channel (C `writeReadServer`).
//!
//! # Ownership
//!
//! Two threads send commands: the port actor (every parameter write) and the
//! status task (which switches the cooling off when the box gets too warm or
//! too damp). The socket therefore sits behind its own mutex.
//!
//! **Invariant: no caller may touch the detector port's parameter library while
//! holding the command lock.** The actor takes the lock inside `write_int32`,
//! so a task that held the lock and then waited on the actor would deadlock.
//! The status task reads every parameter it needs *before* it sends anything.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::user::AsynUser;
use parking_lot::Mutex;

use crate::types::{MAX_MESSAGE_SIZE, SERVER_TIMEOUT};

#[derive(Debug)]
pub enum ServerError {
    Timeout,
    Transport(AsynError),
    /// The box answered, but not with `GOT:` (C's "Error from server").
    Rejected(String),
}

impl std::fmt::Display for ServerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout => write!(f, "timeout"),
            Self::Transport(e) => write!(f, "{e}"),
            Self::Rejected(reply) => write!(f, "the box answered '{reply}'"),
        }
    }
}

impl From<AsynError> for ServerError {
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

pub type ServerResult<T> = Result<T, ServerError>;

/// What one exchange with the box produced.
pub struct Exchange {
    pub reply: String,
    pub result: ServerResult<()>,
}

/// The Pixirad command socket.
#[derive(Clone)]
pub struct PixiradServer {
    handle: Arc<Mutex<PortHandle>>,
}

impl PixiradServer {
    pub fn new(handle: PortHandle) -> Self {
        Self {
            handle: Arc::new(Mutex::new(handle)),
        }
    }

    /// Send a command and read the reply (C `writeReadServer`, including its
    /// `GOT:` check).
    pub fn command(&self, command: &str) -> Exchange {
        let user = AsynUser::new(0).with_addr(0).with_timeout(SERVER_TIMEOUT);
        let op = RequestOp::OctetWriteRead {
            data: command.as_bytes().to_vec(),
            buf_size: MAX_MESSAGE_SIZE,
            flush: true,
        };
        let outcome = self.handle.lock().submit_blocking(op, user);

        match outcome {
            Err(e) => Exchange {
                reply: String::new(),
                result: Err(e.into()),
            },
            Ok(result) => {
                let bytes = result.data.unwrap_or_default();
                let reply = String::from_utf8_lossy(&bytes).trim_end().to_string();
                log::debug!("pixirad: '{command}' -> '{reply}'");
                let result = if crate::protocol::reply_is_ok(command, &reply) {
                    Ok(())
                } else {
                    Err(ServerError::Rejected(reply.clone()))
                };
                Exchange { reply, result }
            }
        }
    }

    /// Drop the connection, wait, and take it again (C `systemReset`: the box
    /// is unreachable while it reboots, and reconnecting by hand is quicker
    /// than waiting for the auto-reconnect).
    pub fn reconnect_after(&self, delay: Duration) {
        let user = || AsynUser::new(0).with_addr(0).with_timeout(SERVER_TIMEOUT);
        {
            let handle = self.handle.lock();
            if let Err(e) = handle.submit_blocking(RequestOp::Disconnect, user()) {
                log::error!("pixirad: cannot disconnect the command port: {e}");
            }
        }
        std::thread::sleep(delay);
        let handle = self.handle.lock();
        if let Err(e) = handle.submit_blocking(RequestOp::Connect, user()) {
            log::error!("pixirad: cannot reconnect the command port: {e}");
        }
    }
}
