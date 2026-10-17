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

use epics_rs::asyn::error::{AsynError, AsynStatus, OctetStep};
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::types::MAX_MESSAGE_SIZE;

/// How C's `pasynOctetSyncIO->writeRead` in `writeBIS` left `fromBIS`.
pub enum Exchange {
    /// It failed before the read — not connected, flush or write — so
    /// `fromBIS` keeps the previous reply.
    NotSent(AsynError),
    /// The read ran and `fromBIS` holds what it received: the reply, or the
    /// bytes before a timeout or error (normally none).
    Read {
        reply: String,
        status: Result<(), AsynError>,
    },
}

fn failed(message: &str) -> Exchange {
    Exchange::NotSent(AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    })
}

/// C posts `fromBIS` up to its first NUL, whitespace included
/// (BISDetector.cpp:325).
pub(crate) fn reply_text(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
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
    pub fn new(handle: SyncIOHandle) -> Self {
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

fn exchange(io: &SyncIOHandle, command: &str, timeout: Duration) -> Exchange {
    let handle = match io.port() {
        Ok(handle) => handle,
        Err(e) => return Exchange::NotSent(e),
    };
    let user = AsynUser::new(0).with_addr(0).with_timeout(timeout);
    let op = RequestOp::OctetWriteRead {
        data: command.as_bytes().to_vec(),
        buf_size: MAX_MESSAGE_SIZE,
        flush: true,
    };

    match handle.submit_blocking(op, user) {
        Ok(result) => Exchange::Read {
            reply: reply_text(&result.data.unwrap_or_default()),
            status: Ok(()),
        },
        Err(e) if e.octet_step() == Some(OctetStep::Read) => Exchange::Read {
            reply: e
                .partial_read()
                .map(|p| reply_text(&p.data))
                .unwrap_or_default(),
            status: Err(e),
        },
        Err(e) => Exchange::NotSent(e),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_reply_is_cut_at_its_first_nul_and_keeps_its_whitespace() {
        assert_eq!(reply_text(b"[ACK] \r\n\0old"), "[ACK] \r\n");
    }
}
