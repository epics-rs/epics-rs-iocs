//! Rust port of areaDetector `ADEiger` — Dectris Eiger detectors driven over the
//! SIMPLON REST API (HTTP) plus the ZeroMQ image stream.
//!
//! Source: `areaDetector/ADEiger/eigerApp/src/{eigerDetector,restApi,eigerParam,streamApi}.cpp`.
//!
//! Every C dependency has a pure-Rust replacement: `libzmq` → `zeromq`, tinycbor → [`stream2`]'s parser,
//! `libhdf5` + the bslz4 HDF5 plugin → `hdf5-reader` + [`bslz4`], `libcurl` →
//! `ureq`.

use std::sync::Arc;

use epics_rs::ad_core::driver::ADDriverBase;
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
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

use crate::driver::{EigerDriver, MAX_ASYN_ADDRESS};
use crate::params::Model;
use crate::rest::RestApi;
use crate::tasks::{Outputs, Signals};

/// The detector's REST port (C `mApi(serverHostname, 80)`).
const REST_PORT: u16 = 80;

/// Everything an IOC needs to keep alive and to wire plugins to.
pub struct EigerRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub pool: DriverPool,
    pub outputs: Outputs,
    pub signals: Signals,
    /// What C's `RestAPI` constructor threw (restApi.cpp:244-275), after the
    /// `ADDriver` base had registered the port; iocsh prints it as
    /// `C++ error: ...`. The port then has ADDriver's parameters only.
    pub thrown: Option<&'static str>,
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
    claim: &PortClaim,
    hostname: &str,
    max_memory: usize,
) -> AsynResult<EigerRuntime> {
    create_eiger_detector_on_port(claim, hostname, REST_PORT, max_memory)
}

/// [`create_eiger_detector`] against a REST port other than 80.
///
/// C hard-codes the port in the constructor's initialiser list
/// (`mApi(serverHostname, 80)`); this seam exists so a test can aim the driver
/// at a socket it controls.
pub fn create_eiger_detector_on_port(
    claim: &PortClaim,
    hostname: &str,
    rest_port: u16,
    max_memory: usize,
) -> AsynResult<EigerRuntime> {
    let port_name = claim.name();
    // C builds `mApi` after the ADDriver base constructor has registered the
    // port, and nothing unregisters it when `mApi` throws.
    let rest = match RestApi::open(hostname, rest_port) {
        Ok(rest) => rest,
        Err(thrown) => return create_inert(claim, max_memory, thrown),
    };
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

    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;

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
        pool.clone(),
        &signals,
    );
    let task_handles = tasks::start(shared, receivers);

    Ok(EigerRuntime {
        runtime_handle,
        pool,
        outputs,
        signals,
        thrown: None,
        task_handles,
    })
}

/// The port of a constructor that threw: the ADDriver base alone.
struct InertDriver {
    ad: ADDriverBase,
}

impl PortDriver for InertDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }
}

fn create_inert(
    claim: &PortClaim,
    max_memory: usize,
    thrown: &'static str,
) -> AsynResult<EigerRuntime> {
    let ad = ADDriverBase::new(
        claim.name(),
        MAX_ASYN_ADDRESS,
        0,
        0,
        max_memory,
        PortFlags {
            multi_device: true,
            ..ASYN_CANBLOCK
        },
    )?;
    let pool = ad.pool.clone();
    let (runtime_handle, _) =
        create_port_runtime(claim, InertDriver { ad }, RuntimeConfig::default())?;
    Ok(EigerRuntime {
        runtime_handle,
        pool,
        outputs: Outputs::new(),
        signals: tasks::signals().0,
        thrown: Some(thrown),
        task_handles: Vec::new(),
    })
}
