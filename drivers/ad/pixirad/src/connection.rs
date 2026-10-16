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
use epics_rs::asyn::trace::TraceMask;
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
        // `status()` reads *through* a `PartialRead` carrier, so a timeout that
        // returned partial bytes (EOS interpose installed) is still recognized.
        if e.status() == AsynStatus::Timeout {
            Self::Timeout
        } else {
            Self::Transport(e)
        }
    }
}

impl From<ServerError> for AsynError {
    fn from(e: ServerError) -> Self {
        match e {
            ServerError::Timeout => AsynError::Status {
                status: AsynStatus::Timeout,
                message: "timeout".into(),
            },
            ServerError::Transport(e) => e,
            // C's `status = asynError` on a reply without `GOT:`.
            e @ ServerError::Rejected(_) => AsynError::Status {
                status: AsynStatus::Error,
                message: e.to_string(),
            },
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
        let handle = self.handle.lock();
        let outcome = handle.submit_blocking(op, user);
        // C prints both lines on `pasynUserCommand_`: the command port's trace
        // mask gates them.
        let trace_error = |message: &str| {
            handle
                .trace()
                .output(handle.port_name(), TraceMask::ERROR, message);
        };

        match outcome {
            Err(e) => {
                let status = e.status();
                // A timeout is the read side giving up after the write went out.
                let wrote = if status == AsynStatus::Timeout {
                    command.len()
                } else {
                    0
                };
                trace_error(&format!(
                    "pixirad:writeReadServer, timeout={:.6}, status={} wrote {wrote} bytes, \
                     received 0 bytes\n",
                    SERVER_TIMEOUT.as_secs_f64(),
                    status as i32
                ));
                Exchange {
                    reply: String::new(),
                    result: Err(e.into()),
                }
            }
            Ok(result) => {
                let bytes = result.data.unwrap_or_default();
                let reply = String::from_utf8_lossy(&bytes).trim_end().to_string();
                let result = if crate::protocol::reply_is_ok(command, &reply) {
                    Ok(())
                } else {
                    trace_error(&format!(
                        "pixirad:writeReadServer unexpected response from server = {reply}"
                    ));
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
            let _ = handle.submit_blocking(RequestOp::Disconnect, user());
        }
        std::thread::sleep(delay);
        let handle = self.handle.lock();
        let _ = handle.submit_blocking(RequestOp::Connect, user());
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
