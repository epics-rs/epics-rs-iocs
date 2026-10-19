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
use epics_rs::asyn::request::{OctetReadOp, RequestOp};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
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
        if e.status() == AsynStatus::Timeout {
            Self::Timeout
        } else {
            Self::Transport(e)
        }
    }
}

/// C's `writeReadServer` returns only a status: the exchange runs on
/// `pasynUserServer_`, so the caller's `errorMessage` stays empty.
impl From<ServerError> for AsynError {
    fn from(e: ServerError) -> Self {
        AsynError::Status {
            status: match e {
                ServerError::Timeout => AsynStatus::Timeout,
                ServerError::Transport(e) => e.status(),
            },
            message: String::new(),
        }
    }
}

impl ServerError {
    /// The asynStatus C's `writeReadServer` would have returned.
    pub fn status(&self) -> i32 {
        match self {
            Self::Timeout => AsynStatus::Timeout as i32,
            Self::Transport(e) => e.status() as i32,
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
    pub fn new(handle: SyncIOHandle) -> Self {
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

    /// [`Self::command`] with the reply a failed read delivered.
    pub fn exchange(&self, command: &str) -> (String, ServerResult<()>) {
        self.session().exchange(command)
    }

    /// C `asynPrint(pasynUserServer_, ASYN_TRACE_ERROR, ...)`.
    pub fn trace_error(&self, message: &str) {
        let io = self.io.lock();
        io.handle
            .print(0, TraceMask::ERROR, format_args!("{message}"));
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
        let (reply, status) = self.exchange(command);
        status.map(|()| reply)
    }

    /// [`Self::command`] with the bytes a failed read delivered, which C's
    /// `fromServer_` holds; a failed write reads none.
    pub fn exchange(&mut self, command: &str) -> (String, ServerResult<()>) {
        let (bytes, status) = self
            .io
            .write_read(command, MAX_MESSAGE_SIZE, SERVER_TIMEOUT);
        if let Err(e) = &status {
            self.io.write_read_failed(e, command);
        }
        let reply = String::from_utf8_lossy(&bytes).trim_end().to_string();
        (reply, status)
    }

    /// Ask for a frame and read the first block of the answer: the text header
    /// and as much of the payload as arrived with it.
    ///
    /// The payload is binary, so the read must not be cut short or stripped by
    /// an input end-of-string; `OctetReadBinary` suppresses one if the IOC has
    /// configured it. C reconnects the socket before every command, so nothing
    /// can be left in the input buffer from the previous exchange.
    ///
    /// The block is what the read delivered, with the read's status: C's
    /// `getImage` parses `fromServer_` whatever `writeReadServer` returned
    /// (PSL.cpp:523-557). `Err` when no read ran.
    pub fn request_image(&mut self) -> ServerResult<(Vec<u8>, ServerResult<()>)> {
        self.io.reconnect();
        let read = self
            .io
            .write("GetImage", GET_IMAGE_TIMEOUT)
            .and_then(|()| self.io.read_binary(MAX_MESSAGE_SIZE, GET_IMAGE_TIMEOUT));
        let status = match &read {
            Ok((_, status)) => status.as_ref(),
            Err(e) => Err(e),
        };
        if let Err(e) = status {
            self.io.write_read_failed(e, "GetImage");
        }
        read
    }

    /// Read the next block of an image payload (C `pasynOctetSyncIO->read`
    /// inside the copy loop): what arrived, also when the read failed.
    pub fn read_image_block(&mut self, max_bytes: usize) -> (Vec<u8>, ServerResult<()>) {
        // A read that never ran leaves C's `nRead_` at the previous block's
        // count, which its error line prints: not copied.
        self.io
            .read_binary(max_bytes, GET_IMAGE_TIMEOUT)
            .unwrap_or_else(|e| (Vec::new(), Err(e)))
    }
}

struct ServerIo {
    handle: SyncIOHandle,
}

impl ServerIo {
    /// C `writeReadServer`'s line, on `pasynUserServer_`: the server port's
    /// trace mask gates it.
    fn write_read_failed(&self, e: &ServerError, command: &str) {
        self.handle.print(
            0,
            TraceMask::ERROR,
            format_args!(
                "PSL:writeReadServer, status={}, sent\n{command}\n",
                e.status()
            ),
        );
    }

    fn user(&self, timeout: Duration) -> AsynUser {
        AsynUser::new(0).with_addr(0).with_timeout(timeout)
    }

    /// The server expects a fresh connection for every command
    /// (C `pasynCommonSyncIO->disconnectDevice` + `connectDevice`). C ignores
    /// both statuses (PSL.cpp:304-305): a failed connect surfaces as the
    /// exchange's asynDisconnected.
    fn reconnect(&mut self) {
        if let Ok(port) = self.handle.port() {
            let _ = port.submit_blocking(RequestOp::Disconnect, self.user(SERVER_TIMEOUT));
            let _ = port.submit_blocking(RequestOp::Connect, self.user(SERVER_TIMEOUT));
        }
    }

    fn write_read(
        &mut self,
        command: &str,
        buf_size: usize,
        timeout: Duration,
    ) -> (Vec<u8>, ServerResult<()>) {
        self.reconnect();
        let port = match self.handle.port() {
            Ok(port) => port,
            Err(e) => return (Vec::new(), Err(e.into())),
        };
        let (reply, status) = port
            .submit_octet_read_blocking(
                OctetReadOp::WriteRead {
                    data: command.as_bytes().to_vec(),
                    buf_size,
                },
                self.user(timeout),
            )
            .into_parts();
        (reply, status.map_err(ServerError::from))
    }

    fn write(&mut self, command: &str, timeout: Duration) -> ServerResult<()> {
        self.handle.port()?.submit_blocking(
            RequestOp::OctetWrite {
                data: command.as_bytes().to_vec(),
            },
            self.user(timeout),
        )?;
        Ok(())
    }

    /// What the read delivered and its status; `Err` when it never ran.
    fn read_binary(
        &mut self,
        buf_size: usize,
        timeout: Duration,
    ) -> ServerResult<(Vec<u8>, ServerResult<()>)> {
        let (data, status) = self
            .handle
            .port()?
            .submit_octet_read_blocking(OctetReadOp::ReadBinary { buf_size }, self.user(timeout))
            .into_parts();
        match status {
            Err(e) if e.never_ran() => Err(e.into()),
            status => Ok((data, status.map_err(ServerError::from))),
        }
    }
}

#[cfg(test)]
mod reconnect_tests {
    use super::*;
    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;

    /// A refused connect fails the command with the exchange's
    /// asynDisconnected, as C's ignored `connectDevice` does.
    #[test]
    fn a_refused_connect_fails_the_command_disconnected() {
        let driver = DrvAsynIPPort::new("PSL_REFUSED_IP", "127.0.0.1:1").unwrap();
        let (runtime, _) = create_port_runtime(
            &claim_port("PSL_REFUSED_IP").unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let server = PslServer::new(SyncIOHandle::from_handle(
            runtime.port_handle().clone(),
            0,
            Duration::ZERO,
        ));
        let err = server.command("GetVersion").unwrap_err();
        assert_eq!(err.status(), AsynStatus::Disconnected as i32, "{err}");
    }
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
