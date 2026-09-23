//! C drvMultiFunction's `asynPrint` lines, printed through the port's asyn
//! trace so `asynSetTraceMask` shows them as it does for the C driver.

use std::fmt;

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;

/// C `driverName` (drvMultiFunction.cpp:90).
pub(crate) const DRIVER: &str = "MultiFunction";

/// C `asynPrint(pasynUserSelf, mask, ...)` from the poller, outside the port's
/// actor. Formatted only when `mask` is on, since the poller runs it every
/// cycle.
pub(crate) fn print(handle: &PortHandle, mask: TraceMask, args: fmt::Arguments<'_>) {
    let trace = handle.trace();
    let port = handle.port_name();
    if trace.is_enabled(port, mask) {
        trace.output(port, mask, &args.to_string());
    }
}
