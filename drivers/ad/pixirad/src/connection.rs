//! The box's TCP command channel (C `writeReadServer`).
//!
//! # Ownership
//!
//! The driver owns the one [`PixiradServer`] and [`PixiradServer::command`]
//! takes it by `&mut`, so every command — the status task's cooling-off
//! included — runs on the port actor through the driver's exchange, which
//! publishes it as C does.

use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynStatus, OctetStep};
use epics_rs::asyn::request::{OctetRead, OctetReadOp, RequestOp};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::types::{MAX_MESSAGE_SIZE, SERVER_TIMEOUT};

#[derive(Debug)]
pub enum ServerError {
    Timeout,
    Transport(AsynError),
    /// The box answered, but not with `GOT:` (C's "Error from server").
    Rejected(Vec<u8>),
}

impl std::fmt::Display for ServerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout => write!(f, "timeout"),
            Self::Transport(e) => write!(f, "{e}"),
            Self::Rejected(reply) => {
                write!(f, "the box answered '{}'", String::from_utf8_lossy(reply))
            }
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

/// C `writeReadServer` returns a bare status: the message stays on
/// `pasynUserCommand_` and never reaches the record's user, whose devAsyn
/// error line is therefore empty.
impl From<ServerError> for AsynError {
    fn from(e: ServerError) -> Self {
        let status = match e {
            ServerError::Timeout => AsynStatus::Timeout,
            ServerError::Transport(e) => e.status(),
            // C's `status = asynError` on a reply without `GOT:`.
            ServerError::Rejected(_) => AsynStatus::Error,
        };
        AsynError::Status {
            status,
            message: String::new(),
        }
    }
}

pub type ServerResult<T> = Result<T, ServerError>;

/// What one exchange with the box produced.
pub struct Exchange {
    /// `fromServer_` as a C string.
    pub reply: Vec<u8>,
    pub result: ServerResult<()>,
}

struct Channel {
    handle: SyncIOHandle,
    /// C `fromServer_`: the buffer the reply is read into, shared by every
    /// command, kept as the C string it holds.
    from_server: Vec<u8>,
}

/// The Pixirad command socket.
pub struct PixiradServer {
    channel: Channel,
}

impl PixiradServer {
    pub fn new(handle: SyncIOHandle) -> Self {
        Self {
            channel: Channel {
                handle,
                from_server: Vec::new(),
            },
        }
    }

    /// Send a command and read the reply (C `writeReadServer`, including its
    /// `GOT:` check).
    pub fn command(&mut self, command: &str) -> Exchange {
        let user = AsynUser::new(0).with_addr(0).with_timeout(SERVER_TIMEOUT);
        let op = OctetReadOp::WriteRead {
            data: command.as_bytes().to_vec(),
            buf_size: MAX_MESSAGE_SIZE,
        };
        let Channel {
            handle,
            from_server,
        } = &mut self.channel;
        let (bytes, status) = match handle.port() {
            Ok(port) => port.submit_octet_read_blocking(op, user),
            Err(e) => OctetRead::failed(e),
        }
        .into_parts();
        // C prints both lines on `pasynUserCommand_`: the command port's trace
        // mask gates them.
        let trace_error = |message: &str| {
            handle.print(0, TraceMask::ERROR, format_args!("{message}"));
        };

        match status {
            Err(e) => {
                // asynOctetSyncIO writeRead leaves the buffer alone unless it
                // got as far as the read, which follows a whole write and
                // (EOS interpose) stores what arrived, terminated
                // (asynInterposeEos.c:250).
                let (wrote, read) = if e.octet_step() == Some(OctetStep::Read) {
                    *from_server = crate::protocol::c_string(&bytes).to_vec();
                    (command.len(), bytes.len())
                } else {
                    (0, 0)
                };
                trace_error(&format!(
                    "pixirad:writeReadServer, timeout={:.6}, status={} wrote {wrote} bytes, \
                     received {read} bytes\n{}\n",
                    CFloat(SERVER_TIMEOUT.as_secs_f64()),
                    e.status() as i32,
                    String::from_utf8_lossy(from_server)
                ));
                Exchange {
                    reply: from_server.clone(),
                    result: Err(e.into()),
                }
            }
            Ok(()) => {
                let reply = crate::protocol::c_string(&bytes).to_vec();
                from_server.clone_from(&reply);
                let result = if crate::protocol::reply_is_ok(command, &reply) {
                    Ok(())
                } else {
                    trace_error(&format!(
                        "pixirad:writeReadServer unexpected response from server = {}\n",
                        String::from_utf8_lossy(&reply)
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
        let _ = self
            .channel
            .handle
            .port()
            .and_then(|port| port.submit_blocking(RequestOp::Disconnect, user()));
        std::thread::sleep(delay);
        let _ = self
            .channel
            .handle
            .port()
            .and_then(|port| port.submit_blocking(RequestOp::Connect, user()));
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

#[cfg(test)]
mod from_server_tests {
    use super::*;
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;

    /// Exchange 1 answers, 2 fails its write, 3 times out after part of a
    /// reply, 4 times out with nothing. Each text ends in a blank and runs on
    /// past a NUL.
    struct Scripted {
        base: PortDriverBase,
        exchange: usize,
    }

    fn timeout() -> AsynError {
        AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        }
    }

    impl PortDriver for Scripted {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.exchange += 1;
            if self.exchange == 2 {
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: "write failed".into(),
                });
            }
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            match self.exchange {
                1 => {
                    let reply = b"GOT: OK \0x";
                    buf[..reply.len()].copy_from_slice(reply);
                    OctetReadResult::ok(reply.len(), EomReason::EOS)
                }
                3 => {
                    let partial = b"GO \0x";
                    buf[..partial.len()].copy_from_slice(partial);
                    OctetReadResult {
                        status: Err(timeout()),
                        ..OctetReadResult::ok(partial.len(), EomReason::empty())
                    }
                }
                _ => OctetReadResult::failed(timeout()),
            }
        }
    }

    /// C `writeReadServer` publishes `fromServer_` whatever the status, as the
    /// C string it holds, and asynOctetSyncIO writes it only once the read
    /// starts.
    #[test]
    fn a_failed_exchange_replies_with_what_c_left_in_from_server() {
        let name = "PIXI_FROM_SERVER";
        let driver = Scripted {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            exchange: 0,
        };
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), driver, RuntimeConfig::default())
                .unwrap();
        let mut server = PixiradServer::new(SyncIOHandle::from_handle(
            runtime.port_handle().clone(),
            0,
            Duration::ZERO,
        ));
        let replies: Vec<Vec<u8>> = (0..4).map(|_| server.command("DAQ:! X").reply).collect();
        assert_eq!(replies, [&b"GOT: OK "[..], b"GOT: OK ", b"GO ", b""]);
    }

    /// C's failure line prints what each step moved and `fromServer_`
    /// (pixirad.cpp:906-910).
    #[test]
    fn a_failed_exchange_traces_nread_and_from_server() {
        use epics_rs::asyn::trace::TraceFile;
        let name = "PIXI_TRACE_FAIL";
        let driver = Scripted {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            exchange: 0,
        };
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), driver, RuntimeConfig::default())
                .unwrap();
        let handle = runtime.port_handle().clone();
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        handle.trace().set_trace_file(
            Some(name),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        let mut server = PixiradServer::new(SyncIOHandle::from_handle(handle, 0, Duration::ZERO));
        for _ in 0..4 {
            server.command("DAQ:! X");
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let line = |status: AsynStatus, wrote: usize, read: usize, from: &str| {
            format!(
                "pixirad:writeReadServer, timeout=1.000000, status={} wrote {wrote} bytes, \
                 received {read} bytes\n{from}\n",
                status as i32
            )
        };
        for expected in [
            line(AsynStatus::Error, 0, 0, "GOT: OK "),
            line(AsynStatus::Timeout, 7, 5, "GO "),
            line(AsynStatus::Timeout, 7, 0, ""),
        ] {
            assert!(text.contains(&expected), "{expected:?} not in {text:?}");
        }
    }
}
