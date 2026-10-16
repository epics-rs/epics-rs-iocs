//! Rust port of areaDetector `ADEiger` — Dectris Eiger detectors driven over the
//! SIMPLON REST API (HTTP) plus the ZeroMQ image stream.
//!
//! Source: `areaDetector/ADEiger/eigerApp/src/{eigerDetector,restApi,eigerParam,streamApi}.cpp`.
//!
//! Every C dependency has a pure-Rust replacement: `libzmq` → `zeromq`, tinycbor → [`stream2`]'s parser,
//! `libhdf5` + the bslz4 HDF5 plugin → `hdf5-reader` + [`bslz4`], `libcurl` →
//! `ureq`.

use std::sync::Arc;

use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::asyn::error::AsynError;
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
pub mod stream2;
pub mod tasks;
pub mod tiff;
pub mod trace;

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

/// Why `eigerDetectorConfig` made no port.
#[derive(Debug)]
pub enum ConfigError {
    /// What C's `RestAPI` constructor throws (restApi.cpp:244-275); iocsh
    /// prints it as `C++ error: ...`.
    Rest(&'static str),
    Asyn(AsynError),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Rest(what) => f.write_str(what),
            Self::Asyn(e) => write!(f, "{e}"),
        }
    }
}

impl From<AsynError> for ConfigError {
    fn from(e: AsynError) -> Self {
        Self::Asyn(e)
    }
}

/// Create an Eiger detector port (C `eigerDetectorConfig`).
pub fn create_eiger_detector(
    port_name: &str,
    hostname: &str,
    max_memory: usize,
) -> Result<EigerRuntime, ConfigError> {
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
) -> Result<EigerRuntime, ConfigError> {
    // C builds `mApi` before its constructor body runs, so a detector that
    // cannot say which API it speaks leaves no driver at all.
    let rest = RestApi::open(hostname, rest_port).map_err(ConfigError::Rest)?;
    let api = rest.api_version();

    // The model decides which parameters exist, so it has to be known before any
    // of them are created. C interleaves the two by fetching `description`
    // through a parameter it has just created; here the fetch is a plain GET.
    let description = rest
        .get_value(rest::Sys::DetConfig, "description")
        .unwrap_or_default();
    let model = Model::from_description(&description);

    let (signals, receivers) = tasks::signals();

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

    let (runtime_handle, _) = create_port_runtime(det, RuntimeConfig::default())?;

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
    let task_handles = tasks::start(shared, receivers);

    Ok(EigerRuntime {
        runtime_handle,
        pool,
        outputs,
        signals,
        task_handles,
    })
}
