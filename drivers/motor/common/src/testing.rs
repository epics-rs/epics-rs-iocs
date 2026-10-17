//! Driver test rigs: a recording serial line, where every write is kept and
//! a read answers the last write through the test's `reply`, and a count of
//! the process's `motorPoller` threads.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

type Reply = Box<dyn Fn(&str) -> String + Send + Sync>;

struct Line {
    base: PortDriverBase,
    sent: Arc<Mutex<Vec<String>>>,
    reply: Reply,
}

impl PortDriver for Line {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let line = String::from_utf8_lossy(data).into_owned();
        self.sent.lock().unwrap().push(line);
        Ok(data.len())
    }
    fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        let last = self
            .sent
            .lock()
            .unwrap()
            .last()
            .cloned()
            .unwrap_or_default();
        let reply = (self.reply)(&last);
        let n = reply.len().min(buf.len());
        buf[..n].copy_from_slice(&reply.as_bytes()[..n]);
        Ok(n)
    }
}

/// The test's side of one port.
pub struct LineRig {
    sent: Arc<Mutex<Vec<String>>>,
}

impl LineRig {
    /// A port named `name`, and the handle a driver talks to it through.
    /// `reply` maps the last message written to what a read returns.
    pub fn new(
        name: &str,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
    ) -> (Self, SyncIOHandle) {
        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Line {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            sent: sent.clone(),
            reply: Box::new(reply),
        };
        let (rt, _jh) =
            create_port_runtime(&claim_port(name).unwrap(), line, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        (Self { sent }, io)
    }

    /// The messages written since the last call.
    pub fn sent(&self) -> Vec<String> {
        std::mem::take(&mut *self.sent.lock().unwrap())
    }
}

/// The `motorPoller` threads of this process; nextest runs each test in its
/// own.
#[cfg(target_os = "linux")]
pub fn pollers() -> usize {
    std::fs::read_dir("/proc/self/task")
        .unwrap()
        .filter_map(|t| std::fs::read_to_string(t.ok()?.path().join("comm")).ok())
        .filter(|name| name.trim_end() == "motorPoller")
        .count()
}

/// Run `cmd` with `args` as the shell does, on a thread of its own, and
/// return the `motorPoller` count once the first has started.
#[cfg(target_os = "linux")]
pub async fn pollers_after(
    cmd: epics_rs::base::server::iocsh::registry::CommandDef,
    args: Vec<epics_rs::base::server::iocsh::registry::ArgValue>,
) -> usize {
    use epics_rs::base::runtime::task::{BlockingBridge, sleep};
    use epics_rs::base::server::database::PvDatabase;
    use epics_rs::base::server::iocsh::registry::CommandContext;

    let bridge = BlockingBridge::capture();
    std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        cmd.handler.call(&args, &ctx).unwrap();
    })
    .join()
    .unwrap();
    for _ in 0..100 {
        if pollers() > 0 {
            break;
        }
        sleep(Duration::from_millis(10)).await;
    }
    sleep(Duration::from_millis(200)).await;
    pollers()
}
