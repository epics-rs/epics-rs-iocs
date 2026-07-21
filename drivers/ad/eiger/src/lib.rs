//! Rust port of areaDetector `ADEiger` — Dectris Eiger detectors driven over the
//! SIMPLON REST API (HTTP) plus the ZeroMQ image stream.
//!
//! Source: `areaDetector/ADEiger/eigerApp/src/{eigerDetector,restApi,eigerParam,streamApi}.cpp`.
//!
//! Every C dependency has a pure-Rust replacement: `libzmq` → `zeromq`,
//! `libhdf5` + the bslz4 HDF5 plugin → `hdf5-reader` + [`bslz4`], `libcurl` →
//! `ureq`.

use std::sync::Arc;

use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};

pub mod bslz4;
pub mod driver;
pub mod h5;
pub mod param;
pub mod params;
pub mod rest;
pub mod stream;
pub mod tasks;
pub mod tiff;

use crate::driver::EigerDriver;
use crate::params::Model;
use crate::rest::RestApi;
use crate::tasks::{Outputs, Signals};

/// The detector's REST port (C `mApi(serverHostname, 80)`).
const REST_PORT: u16 = 80;

/// Everything an IOC needs to keep alive and to wire plugins to.
pub struct EigerRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub pool: Arc<NDArrayPool>,
    pub outputs: Outputs,
    pub signals: Signals,
    task_handles: Vec<std::thread::JoinHandle<()>>,
}

impl EigerRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.outputs.main
    }

    /// The number of background tasks that are running.
    pub fn num_tasks(&self) -> usize {
        self.task_handles.len()
    }
}

/// Create an Eiger detector port (C `eigerDetectorConfig`).
pub fn create_eiger_detector(
    port_name: &str,
    hostname: &str,
    max_memory: usize,
) -> AsynResult<EigerRuntime> {
    create_eiger_detector_on_port(port_name, hostname, REST_PORT, max_memory)
}

/// [`create_eiger_detector`] against a REST port other than 80.
///
/// C hard-codes the port in the constructor's initialiser list
/// (`mApi(serverHostname, 80)`); this seam exists so a test can aim the driver
/// at a socket it controls.
pub fn create_eiger_detector_on_port(
    port_name: &str,
    hostname: &str,
    rest_port: u16,
    max_memory: usize,
) -> AsynResult<EigerRuntime> {
    let mut rest = RestApi::new(hostname, rest_port);

    // A detector that does not answer must not stop the IOC from booting, so
    // the version negotiation only logs and leaves the 1.6.0 bootstrap paths in
    // place — every later request against a dead detector fails anyway, and the
    // port comes up reporting itself disconnected.
    //
    // UPSTREAM DEFECT (restApi.cpp:262-274 + eigerDetector.cpp:2137): C throws
    // out of the `RestAPI` constructor here, `eigerDetectorConfig` does not
    // catch, and the IOC dies on an uncaught exception — where the very next
    // thing C's own constructor does (the `state` fetch below) is careful to
    // log and carry on.
    if let Err(e) = rest.negotiate_api_version() {
        log::error!(
            "eiger: cannot read the SIMPLON API version from {hostname}: {e}; the port is created \
             with the detector disconnected"
        );
    }
    let api = rest.api_version();

    // The model decides which parameters exist, so it has to be known before any
    // of them are created. C interleaves the two by fetching `description`
    // through a parameter it has just created; here the fetch is a plain GET.
    //
    // The version negotiation above is the probe: a detector that did not answer
    // it is marked disconnected, so this GET — and every other request the
    // constructor makes — fails at the client's gate instead of sitting out the
    // full 20 s timeout. Nothing here needs to know that; the gate is uniform.
    let description = rest
        .get_value(rest::Sys::DetConfig, "description")
        .unwrap_or_else(|e| {
            log::warn!("eiger: cannot read the detector description: {e}");
            String::new()
        });
    let model = Model::from_description(&description);
    log::info!("eiger: {hostname} is a {model:?} on SIMPLON API {api:?}");

    let (signals, ctl_rx, init_rx, restart_rx) = tasks::signals();

    // The sensor size is only known once the parameters have been fetched, so
    // the pool starts from the detector's own x/y_pixels_in_detector.
    let sensor_size = |param: &str| {
        rest.get_value(rest::Sys::DetConfig, param)
            .ok()
            .and_then(|v| v.parse::<i32>().ok())
            .unwrap_or(0)
    };
    let max_size_x = sensor_size("x_pixels_in_detector");
    let max_size_y = sensor_size("y_pixels_in_detector");

    let cfg = driver::EigerConfig {
        port_name: port_name.to_string(),
        api,
        model,
        max_size_x,
        max_size_y,
        max_memory,
    };
    let det = EigerDriver::new(cfg, rest, signals.clone())?;

    let p = det.p;
    let ad_params = det.ad.params;
    let ops = det.ops.clone();
    let pool = det.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(det, RuntimeConfig::default());

    let outputs = Outputs::new();
    let shared = tasks::shared(
        ops,
        runtime_handle.port_handle().clone(),
        p,
        ad_params,
        model,
        api,
        hostname.to_string(),
        outputs.clone(),
        &signals,
    );
    let task_handles = tasks::start(shared, ctl_rx, init_rx, restart_rx);

    Ok(EigerRuntime {
        runtime_handle,
        pool,
        outputs,
        signals,
        task_handles,
    })
}
