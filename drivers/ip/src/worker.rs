//! Worker thread shared by the `ip` port drivers.
//!
//! The C device support performed every transaction from an asyn queue-request
//! callback, i.e. on the octet port's own thread. Here the port driver keeps a
//! plain `std::thread` worker: it owns the [`SyncIOHandle`] to the device's
//! octet port, polls the inputs on a period, and executes the commands the asyn
//! write handlers enqueue. The write handlers must not touch the handle
//! themselves — a blocking submit from the port-actor thread is not usable.

use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use epics_rs::asyn::sync_io::SyncIOHandle;

/// Largest device reply any of the `ip` devices sends (`devMPC.c` reads at most
/// 50 bytes, `devTPG261.c` 64).
const READ_BUF: usize = 128;

/// asyn `pasynUser->reason` for a raw octet read/write on the transport port.
const OCTET_REASON: usize = 0;

/// The device end of a port: a write/read pair over the octet port, framed by
/// the port's own input EOS (set in `st.cmd` with `asynOctetSetInputEos`).
pub struct Transport {
    io: SyncIOHandle,
}

impl Transport {
    pub fn new(io: SyncIOHandle) -> Self {
        Self { io }
    }

    /// Write `data` and read one EOS-delimited reply.
    pub fn write_read(&self, data: &[u8]) -> Result<String, String> {
        self.write(data)?;
        self.read()
    }

    pub fn write(&self, data: &[u8]) -> Result<(), String> {
        self.io
            .write_octet(OCTET_REASON, data)
            .map(|_| ())
            .map_err(|e| format!("write failed: {e}"))
    }

    /// Read one EOS-delimited reply and decode it as text. The `ip` devices all
    /// speak ASCII; a non-UTF-8 byte is a framing error, not data.
    pub fn read(&self) -> Result<String, String> {
        let raw = self
            .io
            .read_octet(OCTET_REASON, READ_BUF)
            .map_err(|e| format!("read failed: {e}"))?;
        String::from_utf8(raw).map_err(|e| format!("reply is not ASCII: {e}"))
    }
}

/// A device's worker: polled on a period, and fed the commands the record
/// write handlers enqueue.
pub trait DeviceWorker: Send + 'static {
    /// Command enqueued by an asyn write handler.
    type Command: Send + 'static;

    /// Read the device's inputs and publish them as parameters.
    fn poll(&mut self);

    /// Execute one queued command.
    fn handle(&mut self, command: Self::Command);
}

/// Run `worker` on its own thread until the command channel is dropped.
///
/// Commands are served as they arrive; `poll` runs whenever `period` has
/// elapsed since the last poll, so a busy command stream cannot starve it.
pub fn spawn<W: DeviceWorker>(
    name: &str,
    mut worker: W,
    rx: Receiver<W::Command>,
    period: Duration,
) -> JoinHandle<()> {
    let name = name.to_string();
    std::thread::Builder::new()
        .name(format!("ip-{name}"))
        .spawn(move || {
            let mut next_poll = Instant::now();
            loop {
                let now = Instant::now();
                if now >= next_poll {
                    worker.poll();
                    next_poll = Instant::now() + period;
                    continue;
                }
                match rx.recv_timeout(next_poll - now) {
                    Ok(command) => worker.handle(command),
                    Err(RecvTimeoutError::Timeout) => {
                        worker.poll();
                        next_poll = Instant::now() + period;
                    }
                    Err(RecvTimeoutError::Disconnected) => break,
                }
            }
            log::info!("ip: worker thread for '{name}' stopped");
        })
        .expect("failed to spawn the ip worker thread")
}
