//! The driver's asyn trace lines, as eigerDetector.cpp's and eigerParam.cpp's
//! `ERR`/`FLOW` macros print them on `pasynUserSelf`.
//!
//! Lines go through the global trace manager by port name, so the parameter
//! layer can print before the port is registered — C's `mParams` prints from
//! the constructor too.

use std::fmt;
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

    /// Formatted only when the port's FLOW bit is on.
    fn flow_line(&self, line: fmt::Arguments) {
        PortServices::global()
            .trace()
            .print(Some(&self.port), Some(0), 0, TraceMask::FLOW, line);
    }

    /// eigerDetector.cpp `ERR`/`ERR_ARGS`.
    pub fn err(&self, function: &str, msg: &str) {
        self.out(
            TraceMask::ERROR,
            &format!("eigerDetector::{function}: {msg}\n"),
        );
    }

    /// eigerDetector.cpp `FLOW`/`FLOW_ARGS`.
    pub fn flow(&self, function: &str, msg: fmt::Arguments) {
        self.flow_line(format_args!("eigerDetector::{function}: {msg}\n"));
    }

    /// parseH5File's hand-written FLOW line, with a single colon
    /// (eigerDetector.cpp:1895-1897).
    pub fn flow_single_colon(&self, function: &str, msg: &str) {
        self.flow_line(format_args!("eigerDetector:{function}: {msg}\n"));
    }

    /// The `ASYN_TRACE_WARNING` lines `initializeTask`/`restartTask` print,
    /// with a single colon.
    pub fn warning(&self, function: &str, msg: &str) {
        self.out(
            TraceMask::WARNING,
            &format!("eigerDetector:{function}: {msg}\n"),
        );
    }

    /// eigerParam.cpp `ERR`/`ERR_ARGS`.
    pub fn param_err(&self, asyn_name: &str, function: &str, msg: &str) {
        self.out(
            TraceMask::ERROR,
            &format!("Param[{asyn_name}]::{function}: {msg}\n"),
        );
    }

    /// eigerParam.cpp `FLOW`/`FLOW_ARGS`.
    pub fn param_flow(&self, asyn_name: &str, function: &str, msg: fmt::Arguments) {
        self.flow_line(format_args!("Param[{asyn_name}]::{function}: {msg}\n"));
    }
}
