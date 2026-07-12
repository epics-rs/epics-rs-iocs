//! The BIS command socket (C `writeBIS`).
//!
//! # Ownership
//!
//! The port actor is the only thing that sends a command. C sent the scan
//! command from the acquisition thread instead, and had to take the port lock
//! to do it; here the acquisition task asks the actor for a scan through a
//! parameter, so the socket has one owner and needs no lock at all.
//!
//! # Why the exchange runs on a thread of its own
//!
//! `PortHandle::submit_blocking` blocks with `tokio::task::block_in_place` when
//! it is called from inside a tokio runtime, and `block_in_place` panics unless
//! that runtime is multi-threaded. Both runtimes the framework builds for a
//! driver — the port actor's (`asyn-rs`, `runtime/port.rs`) and
//! `ad_core::runtime::run_thread_named`'s — are current-thread runtimes, so a
//! driver that calls `submit_blocking` from its own `write_int32` panics the
//! actor thread on the first command it sends.
//!
//! [`BisServer`] therefore owns a plain thread, with no tokio runtime of its
//! own, and the actor asks it for each exchange and waits: on a plain thread
//! `submit_blocking` takes its blocking-channel path and no `block_in_place` is
//! attempted. Waiting is what the C did too — it held the port lock across the
//! whole exchange.

use std::sync::mpsc::{Sender, channel};
use std::thread::JoinHandle;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::user::AsynUser;

use crate::types::MAX_MESSAGE_SIZE;

/// What one exchange with BIS produced. C published both sides of it whether or
/// not the write succeeded, so both are here whatever `result` says.
pub struct Exchange {
    pub reply: String,
    pub result: Result<(), AsynError>,
}

fn failed(message: &str) -> Exchange {
    Exchange {
        reply: String::new(),
        result: Err(AsynError::Status {
            status: AsynStatus::Error,
            message: message.into(),
        }),
    }
}

struct Request {
    command: String,
    timeout: Duration,
    reply: Sender<Exchange>,
}

pub struct BisServer {
    requests: Sender<Request>,
    #[allow(dead_code)] // the thread lives as long as the driver does
    worker: JoinHandle<()>,
}

impl BisServer {
    pub fn new(handle: PortHandle) -> Self {
        let (requests, rx) = channel::<Request>();
        let worker = std::thread::Builder::new()
            .name("BISCommand".into())
            .spawn(move || {
                while let Ok(request) = rx.recv() {
                    let exchange = exchange(&handle, &request.command, request.timeout);
                    let _ = request.reply.send(exchange);
                }
            })
            .expect("failed to spawn the BIS command thread");

        Self { requests, worker }
    }

    /// Send a command and wait for the reply.
    ///
    /// BIS terminates its replies with `]`, not with a newline; the input EOS
    /// the startup script sets on the command port is what cuts the reply, so
    /// nothing here depends on the reply's shape.
    pub fn command(&self, command: &str, timeout: Duration) -> Exchange {
        let (reply, replies) = channel::<Exchange>();
        let request = Request {
            command: command.to_string(),
            timeout,
            reply,
        };
        if self.requests.send(request).is_err() {
            return failed("the BIS command thread is not running");
        }
        replies
            .recv()
            .unwrap_or_else(|_| failed("the BIS command thread stopped"))
    }
}

fn exchange(handle: &PortHandle, command: &str, timeout: Duration) -> Exchange {
    let user = AsynUser::new(0).with_addr(0).with_timeout(timeout);
    let op = RequestOp::OctetWriteRead {
        data: command.as_bytes().to_vec(),
        buf_size: MAX_MESSAGE_SIZE,
        flush: true,
    };

    match handle.submit_blocking(op, user) {
        Err(e) => {
            log::error!("bruker: '{command}': {e}");
            Exchange {
                reply: String::new(),
                result: Err(e),
            }
        }
        Ok(result) => {
            let bytes = result.data.unwrap_or_default();
            let reply = String::from_utf8_lossy(&bytes).trim_end().to_string();
            log::debug!("bruker: '{command}' -> '{reply}'");
            Exchange {
                reply,
                result: Ok(()),
            }
        }
    }
}
