//! Worker thread shared by the `ip` port drivers.
//!
//! The C device support performed every transaction from an asyn queue-request
//! callback, i.e. on the octet port's own thread. Here the port driver keeps a
//! plain `std::thread` worker: it owns the [`SyncIOHandle`] to the device's
//! octet port, polls the inputs on a period, and executes the commands the asyn
//! write handlers enqueue. The write handlers must not touch the handle
//! themselves — a blocking submit from the port-actor thread is not usable.

use std::fmt;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::{TraceManager, TraceMask};

/// Largest device reply any of the `ip` devices sends (`devMPC.c` reads at most
/// 50 bytes, `devTPG261.c` 64).
const READ_BUF: usize = 128;

/// asyn `pasynUser->reason` for a raw octet read/write on the transport port.
const OCTET_REASON: usize = 0;

/// The address C's device support connects at (`connectDevice(pasynUser, port, 0)`).
pub const OCTET_ADDR: i32 = 0;

/// The device end of a port: a write/read pair over the octet port, framed by
/// the port's own input EOS (set in `st.cmd` with `asynOctetSetInputEos`).
pub struct Transport {
    io: SyncIOHandle,
    trace: Arc<TraceManager>,
    port: String,
}

impl Transport {
    pub fn new(io: SyncIOHandle, trace: Arc<TraceManager>, port: &str) -> Self {
        Self {
            io,
            trace,
            port: port.to_string(),
        }
    }

    /// C's `asynPrint(pasynUser, ...)` on the device support's octet user.
    pub fn print(&self, mask: TraceMask, args: fmt::Arguments<'_>) {
        self.trace
            .print(Some(&self.port), Some(OCTET_ADDR), 0, mask, args);
    }

    /// Write `data` and read one EOS-delimited reply.
    pub fn write_read(&self, data: &[u8]) -> Result<String, String> {
        self.write(data).map_err(|e| format!("write failed: {e}"))?;
        self.read()
    }

    pub fn write(&self, data: &[u8]) -> AsynResult<()> {
        self.io.write_octet(OCTET_REASON, data).map(|_| ())
    }

    /// Read one EOS-delimited reply and decode it as text. The `ip` devices all
    /// speak ASCII; a non-UTF-8 byte is a framing error, not data.
    pub fn read(&self) -> Result<String, String> {
        let raw = self.read_bytes().map_err(|e| format!("read failed: {e}"))?;
        String::from_utf8(raw).map_err(|e| format!("reply is not ASCII: {e}"))
    }

    /// Read one EOS-delimited reply, undecoded.
    pub fn read_bytes(&self) -> AsynResult<Vec<u8>> {
        self.io.read_octet(OCTET_REASON, READ_BUF)
    }

    /// C's device-support callback: write `data`, then read whatever comes
    /// back, as one transaction. `None` when the write never ran — C's
    /// `queueRequest` callback never ran either, so nothing is printed. Once
    /// the write ran, a failed read is C's `nread == 0`: empty bytes.
    pub fn exchange(&self, data: &[u8]) -> Option<Vec<u8>> {
        match self.write(data) {
            Err(e) if e.never_ran() => None,
            _ => Some(self.read_bytes().unwrap_or_default()),
        }
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
        })
        .expect("failed to spawn the ip worker thread")
}

#[cfg(test)]
pub(crate) mod test_support {
    use std::path::PathBuf;
    use std::sync::Mutex;

    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::port::PortDriver;
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
    use epics_rs::asyn::trace::TraceFile;

    use super::*;

    /// A transport on an octet port that never connects, so the queue gate
    /// refuses every request before it runs. The trace goes to a file.
    pub fn refused_transport(port: &str) -> (Transport, PathBuf, PortRuntimeHandle) {
        transport_on(port, "127.0.0.1:1", false)
    }

    /// A transport on a device that accepts the connection and never answers,
    /// so every read times out after the write went out.
    pub fn silent_transport(port: &str) -> (Transport, PathBuf, PortRuntimeHandle) {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let _ = std::io::copy(&mut stream, &mut std::io::sink());
        });
        transport_on(port, &addr, true)
    }

    fn transport_on(
        port: &str,
        host: &str,
        auto_connect: bool,
    ) -> (Transport, PathBuf, PortRuntimeHandle) {
        let mut driver = DrvAsynIPPort::new(port, host).unwrap();
        driver.base_mut().init_auto_connect(auto_connect);
        let (runtime, _actor) = create_port_runtime(driver, RuntimeConfig::default()).unwrap();
        let handle = runtime.port_handle().clone();
        let path = std::env::temp_dir().join(format!("{port}-trace-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        handle
            .trace()
            .set_trace_file(Some(port), TraceFile::File(Arc::new(Mutex::new(file))));
        let io = SyncIOHandle::from_handle(handle.clone(), OCTET_ADDR, Duration::from_millis(50));
        (
            Transport::new(io, handle.trace().clone(), port),
            path,
            runtime,
        )
    }

    /// The trace file's lines, with the timestamp/port prefix cut.
    pub fn traced(path: &PathBuf) -> Vec<String> {
        let text = std::fs::read_to_string(path).unwrap();
        let _ = std::fs::remove_file(path);
        text.lines()
            .map(|l| l.split_once(' ').map_or(l, |(_, rest)| rest).to_string())
            .collect()
    }
}
