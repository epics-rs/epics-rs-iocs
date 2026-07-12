//! What an `*Config` iocsh command hands back to the IOC: the live port plus
//! the worker thread driving it. The IOC keeps it alive for the process.

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::port::PortRuntimeHandle;

pub struct IpPortRuntime {
    pub runtime_handle: PortRuntimeHandle,
    _worker: std::thread::JoinHandle<()>,
}

impl IpPortRuntime {
    pub fn new(runtime_handle: PortRuntimeHandle, worker: std::thread::JoinHandle<()>) -> Self {
        Self {
            runtime_handle,
            _worker: worker,
        }
    }

    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }
}
