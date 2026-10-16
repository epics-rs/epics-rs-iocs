//! The driver's asyn trace lines, as eigerDetector.cpp's and eigerParam.cpp's
//! `ERR`/`FLOW` macros print them on `pasynUserSelf`.
//!
//! Lines go through the global trace manager by port name, so the parameter
//! layer can print before the port is registered — C's `mParams` prints from
//! the constructor too.

use std::sync::Arc;

use epics_rs::asyn::services::PortServices;
use epics_rs::asyn::trace::TraceMask;

#[derive(Clone)]
pub struct Trace {
    port: Arc<str>,
}

impl Trace {
    pub fn new(port: &str) -> Self {
        Self { port: port.into() }
    }

    fn out(&self, mask: TraceMask, msg: &str) {
        PortServices::global()
            .trace()
            .output_device(&self.port, Some(0), 0, mask, msg);
    }

    /// eigerDetector.cpp `ERR`/`ERR_ARGS`.
    pub fn err(&self, function: &str, msg: &str) {
        self.out(
            TraceMask::ERROR,
            &format!("eigerDetector::{function}: {msg}"),
        );
    }

    /// The `ASYN_TRACE_WARNING` lines `initializeTask`/`restartTask` print,
    /// with a single colon.
    pub fn warning(&self, function: &str, msg: &str) {
        self.out(
            TraceMask::WARNING,
            &format!("eigerDetector:{function}: {msg}"),
        );
    }

    /// eigerParam.cpp `ERR`/`ERR_ARGS`.
    pub fn param_err(&self, asyn_name: &str, function: &str, msg: &str) {
        self.out(
            TraceMask::ERROR,
            &format!("Param[{asyn_name}]::{function}: {msg}"),
        );
    }
}
