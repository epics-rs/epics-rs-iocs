//! `asynPrint` for the threads that run outside the port actor.

use std::sync::Arc;

use asyn_rs::services::PortServices;
use asyn_rs::trace::{TraceManager, TraceMask};

#[derive(Clone)]
pub(crate) struct Trace {
    port_name: String,
    manager: Arc<TraceManager>,
}

impl Trace {
    /// The trace configuration `create_port_runtime` binds the port to.
    pub fn new(port_name: &str) -> Self {
        Self {
            port_name: port_name.to_string(),
            manager: PortServices::global().trace().clone(),
        }
    }

    pub fn error(&self, msg: &str) {
        self.manager
            .output(&self.port_name, TraceMask::ERROR, &format!("{msg}\n"));
    }

    pub fn warning(&self, msg: &str) {
        self.manager
            .output(&self.port_name, TraceMask::WARNING, &format!("{msg}\n"));
    }
}
