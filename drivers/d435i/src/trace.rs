//! The driver's asyn trace lines. There is no C driver to copy, so the text is
//! this crate's own; the level follows the asyn mask (warnings at
//! `ASYN_TRACE_WARNING`, informational lines at `ASYN_TRACE_FLOW`).
//!
//! Lines go through the global trace manager by port name, so capability
//! discovery can print before the ports are registered.

use epics_rs::asyn::services::PortServices;
pub use epics_rs::asyn::trace::TraceMask;

pub fn print(port: &str, mask: TraceMask, msg: &str) {
    PortServices::global()
        .trace()
        .output_device(port, Some(0), 0, mask, msg);
}
