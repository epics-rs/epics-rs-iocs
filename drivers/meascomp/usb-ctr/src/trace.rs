//! C `asynPrint(pasynUserSelf, ...)` for the code that runs outside
//! [`PortDriver`](epics_rs::asyn::port::PortDriver): the poller, the scaler
//! record's driver and the scan helpers they share.

use std::sync::{Arc, OnceLock};

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;

/// The port's own trace, bound once its runtime exists. Silent before that,
/// as `PortDriverBase::trace_print` is for a port not yet bound.
#[derive(Clone, Default)]
pub struct SelfTrace(Arc<OnceLock<PortHandle>>);

impl SelfTrace {
    pub fn bind(&self, port: PortHandle) {
        let _ = self.0.set(port);
    }

    pub fn print(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        if let Some(port) = self.0.get() {
            port.trace()
                .output(port.port_name(), mask, &args.to_string());
        }
    }
}
