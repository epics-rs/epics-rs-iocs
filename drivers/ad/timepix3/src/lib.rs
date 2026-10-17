//! Rust port of areaDetector `ADTimePix3` — an ASI TimePix3 detector behind an
//! ASI Serval server.
//!
//! Source: `areaDetector/ADTimePix3/tpx3App/src/` (`ADTimePix.cpp`,
//! `serval_http.cpp`, `serval_stream.cpp`, `histogram_io.cpp`, `mask_io.cpp`,
//! `acquire.cpp`, `network_client.cpp`, `img_accumulation.cpp`).
//!
//! Serval is driven over plain HTTP with JSON bodies (`cpr` + `nlohmann/json`
//! in C, `ureq` + `serde_json` here). The preview channels are *raw TCP*
//! streams of `JSON header \n binary payload` frames, not HTTP.

use std::sync::Arc;

use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};

pub mod accum;
mod adlog;
pub mod bpc_file;
pub mod calibration;
pub mod coordinator;
pub mod destination;
pub mod driver;
mod dtoa;
pub mod family;
pub mod http;
pub mod mask;
pub mod params;
pub mod rate;
pub mod serval;
pub mod state;
pub mod stream;
pub mod tasks;
pub mod window;
pub mod worker;

use crate::driver::TimePix3Driver;
use crate::http::ServalHttp;
use crate::state::{Command, Shared};

/// Everything an IOC has to keep alive and wire plugins to.
pub struct TimePix3Runtime {
    pub runtime_handle: PortRuntimeHandle,
    pub pool: DriverPool,
    pub outputs: tasks::Outputs,
    pub shared: Arc<Shared>,
    _threads: Vec<std::thread::JoinHandle<()>>,
}

impl TimePix3Runtime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    /// The NDArray output plugins on asyn address `addr` connect to.
    pub fn array_output(&self, addr: usize) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.outputs[addr]
    }
}

/// Create a TimePix3 port (C `ADTimePixConfig`, ADTimePix.cpp:1601).
///
/// `server_url` is the Serval endpoint, e.g. `http://localhost:8081`.
pub fn create_timepix3_detector(
    claim: &PortClaim,
    server_url: &str,
    max_memory: usize,
) -> AsynResult<TimePix3Runtime> {
    let port_name = claim.name();
    let http = Arc::new(ServalHttp::new(server_url));
    let shared = Arc::new(Shared::new());

    let (cmd_tx, cmd_rx) = rt::command_channel::<Command>(8);
    let driver = TimePix3Driver::new(port_name, http.clone(), shared.clone(), max_memory, cmd_tx)?;

    let p = driver.p;
    let ad_params = driver.ad.params;
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(claim, driver, RuntimeConfig::default())?;

    let outputs = tasks::new_outputs();
    let ctx = Arc::new(tasks::Ctx {
        handle: runtime_handle.port_handle().clone(),
        http,
        shared: shared.clone(),
        p,
        ad: ad_params,
        outputs: outputs.clone(),
        pool: pool.clone(),
    });
    let connected = tasks::initial_server_check_connection(&ctx);
    let threads = tasks::start(ctx, cmd_rx, connected);

    Ok(TimePix3Runtime {
        runtime_handle,
        pool,
        outputs,
        shared,
        _threads: threads,
    })
}
