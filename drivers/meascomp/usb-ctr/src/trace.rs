//! C drvUSBCTR's `asynPrint` lines, printed through the port's asyn trace so
//! `asynSetTraceMask` shows them as it does for the C driver.

use std::fmt;

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;
use meascomp::error::ScanPosition;

/// C `driverName` (drvUSBCTR.cpp:42).
pub(crate) const DRIVER: &str = "USBCTR";

/// C `asynPrint(pasynUserSelf, mask, ...)` from outside the port's actor --
/// the poller and the scaler device support. Formatted only when `mask` is
/// on at `addr` (`None` for the port), since the poller runs it every cycle.
pub(crate) fn print(
    handle: &PortHandle,
    addr: Option<i32>,
    mask: TraceMask,
    args: fmt::Arguments<'_>,
) {
    let trace = handle.trace();
    let port = handle.port_name();
    let enabled = match addr {
        Some(addr) => trace.is_enabled_device(port, addr, mask),
        None => trace.is_enabled(port, mask),
    };
    if enabled {
        trace.output_device(port, addr, 0, mask, &args.to_string());
    }
}

/// C `readMCS`'s and `readScaler`'s trace of the scan-status call
/// (drvUSBCTR.cpp:748-750, :968-970), for the named function.
pub(crate) struct GetStatus<'a>(pub &'a str, pub ScanPosition);

impl fmt::Display for GetStatus<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let GetStatus(function, p) = self;
        write!(
            f,
            "{DRIVER}::{function} getStatus returned status={}, ctrStatus={}, ctrCount={}, \
             ctrIndex={}",
            p.code, p.status, p.total_count, p.index
        )
    }
}
