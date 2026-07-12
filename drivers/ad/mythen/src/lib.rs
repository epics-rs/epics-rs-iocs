//! Rust port of areaDetector `ADMythen` — a Dectris Mythen strip detector
//! driven over an asyn octet IP port with the M1K ASCII/binary command set.
//!
//! Source: `areaDetector/ADMythen/mythenApp/src/mythen.cpp`.
//!
//! The socket itself belongs to asyn, exactly as in C: st.cmd creates it with
//! `drvAsynIPPortConfigure` and sets the output EOS the detector needs, and
//! `mythenConfig` names that port.

use std::sync::Arc;

use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::asyn_record;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};

pub mod detector;
pub mod driver;
pub mod protocol;
pub mod task;
pub mod transport;

use crate::detector::Detector;
use crate::driver::MythenDriver;
use crate::transport::Transport;

/// Everything an IOC needs to keep alive and to wire plugins to.
pub struct MythenRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub pool: Arc<NDArrayPool>,
    pub output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    pub detector: Arc<Detector>,
    _task: std::thread::JoinHandle<()>,
}

impl MythenRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.output
    }
}

/// Create a Mythen detector port (C `mythenConfig`).
///
/// `ip_port_name` is the asyn octet port `drvAsynIPPortConfigure` created.
pub fn create_mythen_detector(
    port_name: &str,
    ip_port_name: &str,
    max_memory: usize,
) -> AsynResult<MythenRuntime> {
    let ip_port = asyn_record::get_port(ip_port_name).ok_or_else(|| AsynError::Status {
        status: AsynStatus::Error,
        message: format!("mythen: no asyn port named {ip_port_name}"),
    })?;
    let det = Arc::new(Detector::new(Transport::new(ip_port.handle)));

    let (start_tx, start_rx) = rt::command_channel::<()>(1);
    let driver = MythenDriver::new(port_name, det.clone(), max_memory, start_tx)?;

    let p = driver.p;
    let ad_params = driver.ad.params;
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(driver, RuntimeConfig::default());

    let output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let shared = Arc::new(task::Shared {
        handle: runtime_handle.port_handle().clone(),
        p,
        ad: ad_params,
        det: det.clone(),
        output: output.clone(),
    });
    let task = task::start(shared, start_rx);

    Ok(MythenRuntime {
        runtime_handle,
        pool,
        output,
        detector: det,
        _task: task,
    })
}
