//! A recording serial line for driver tests: every write is kept, and a read
//! answers the last write through the test's `reply`.

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
