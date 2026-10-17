//! Rust port of areaDetector `ADMythen` — a Dectris Mythen strip detector
//! driven over an asyn octet IP port with the M1K ASCII/binary command set.
//!
//! Source: `areaDetector/ADMythen/mythenApp/src/mythen.cpp`.
//!
//! The socket itself belongs to asyn, exactly as in C: st.cmd creates it with
//! `drvAsynIPPortConfigure` and sets the output EOS the detector needs, and
//! `mythenConfig` names that port.

use std::sync::Arc;

use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::ad_core::runtime as rt;
use std::time::Duration;

use epics_rs::ad_core::driver::ADDriverBase;
use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;

pub mod detector;
pub mod driver;
pub mod protocol;
pub mod task;
pub mod transport;

use crate::detector::Detector;
use crate::driver::{MythenDriver, SENSOR_SIZE_X, SENSOR_SIZE_Y};
use crate::transport::Transport;

/// Everything an IOC needs to keep alive and to wire plugins to.
pub struct MythenRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub pool: DriverPool,
    pub output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    /// `None` when the IP port was missing and C's constructor returned
    /// before it created its parameters and its acquisition task.
    pub acquisition: Option<Acquisition>,
}

/// The detector and the acquisition task C's constructor ends by starting.
pub struct Acquisition {
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
    claim: &PortClaim,
    ip_port_name: &str,
    max_memory: usize,
) -> AsynResult<MythenRuntime> {
    let port_name = claim.name();
    let (meter, connected) =
        SyncIOHandle::connect_port(ip_port_name, 0, Duration::ZERO, InterfaceType::Octet);
    if let Err(e) = connected {
        // mythen.cpp:1300-1305 prints this on stdout and returns from the
        // constructor: the port exists with only ADDriver's parameters.
        println!(
            "mythen:mythen: error calling pasynOctetSyncIO->connect, status={}, error={}",
            e.status() as i32,
            e.message_display()
        );
        return create_unconnected(claim, max_memory);
    }
    let ip_port = meter.port()?.clone();
    let det = Arc::new(Detector::new(Transport::new(ip_port), port_name));

    let (start_tx, start_rx) = rt::command_channel::<()>(1);
    let driver = MythenDriver::new(port_name, det.clone(), max_memory, start_tx)?;

    let p = driver.p;
    let ad_params = driver.ad.params;
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(claim, driver, RuntimeConfig::default())?;

    let output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let shared = Arc::new(task::Shared {
        handle: runtime_handle.port_handle().clone(),
        p,
        ad: ad_params,
        det: det.clone(),
        output: output.clone(),
        pool: pool.clone(),
    });
    let task = task::start(shared, start_rx);

    Ok(MythenRuntime {
        runtime_handle,
        pool,
        output,
        acquisition: Some(Acquisition {
            detector: det,
            _task: task,
        }),
    })
}

/// The `mythen` C leaves registered when its IP port is missing: `ADDriver`
/// built, nothing of its own.
struct UnconnectedDriver {
    ad: ADDriverBase,
}

impl PortDriver for UnconnectedDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }
}

fn create_unconnected(claim: &PortClaim, max_memory: usize) -> AsynResult<MythenRuntime> {
    let ad = ADDriverBase::new(
        claim.name(),
        1,
        SENSOR_SIZE_X,
        SENSOR_SIZE_Y,
        max_memory,
        PortFlags {
            multi_device: true,
            ..ASYN_CANBLOCK
        },
        InterfaceMask::UNMIGRATED_SUBCLASS,
        InterfaceMask::UNMIGRATED_SUBCLASS,
    )?;
    let pool = ad.pool.clone();
    let (runtime_handle, _) =
        create_port_runtime(claim, UnconnectedDriver { ad }, RuntimeConfig::default())?;
    Ok(MythenRuntime {
        runtime_handle,
        pool,
        output: Arc::new(parking_lot::Mutex::new(NDArrayOutput::new())),
        acquisition: None,
    })
}
