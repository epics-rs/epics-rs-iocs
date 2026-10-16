//! C `asynPrint(pasynUserSelf, ...)` and `MultiFunction::reportError` for
//! the driver, the poller and the scan helpers they share.

use std::sync::{Arc, OnceLock};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;
use meascomp::error::MeasCompError;
use uldaq_sys::UlError;

/// C `driverName`.
pub const DRIVER_NAME: &str = "MultiFunction";

/// C's `status` from one uldaq call.
pub fn status_of<T>(result: &meascomp::error::Result<T>) -> UlError {
    result.as_ref().err().map_or(0, |e| e.code)
}

/// C's `return (status==0) ? asynSuccess : asynError;`, which leaves the
/// user's error message unset.
pub fn asyn_status(status: i32) -> AsynResult<()> {
    if status == 0 {
        Ok(())
    } else {
        Err(AsynError::Status {
            status: AsynStatus::Error,
            message: String::new(),
        })
    }
}

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
                .print(Some(port.port_name()), None, 0, mask, args);
        }
    }

    /// C `reportError`: 0 is an IO_DRIVER "Info" line, -1 an ERROR line
    /// with no library text, anything else the uldaq code and its message.
    pub fn report_error(&self, err: UlError, function: &str, message: &str) -> UlError {
        match err {
            0 => self.print(
                TraceMask::IO_DRIVER,
                format_args!("{DRIVER_NAME}::{function} Info: {message}"),
            ),
            -1 => self.print(
                TraceMask::ERROR,
                format_args!("{DRIVER_NAME}::{function} Error: {message}"),
            ),
            _ => self.print(
                TraceMask::ERROR,
                format_args!(
                    "{DRIVER_NAME}::{function} Error: {message}, err={err} {}",
                    MeasCompError::from_code(err).message
                ),
            ),
        }
        err
    }

    /// [`Self::report_error`] on one uldaq call's result.
    pub fn report<T>(
        &self,
        result: &meascomp::error::Result<T>,
        function: &str,
        message: &str,
    ) -> UlError {
        self.report_error(status_of(result), function, message)
    }
}
