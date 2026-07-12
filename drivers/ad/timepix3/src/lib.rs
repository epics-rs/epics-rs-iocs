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

use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};

pub mod accum;
pub mod driver;
pub mod http;
pub mod mask;
pub mod params;
pub mod serval;
pub mod state;
pub mod stream;
pub mod tasks;

use crate::driver::TimePix3Driver;
use crate::http::ServalHttp;
use crate::state::{Command, Shared};

/// Everything an IOC has to keep alive and wire plugins to.
pub struct TimePix3Runtime {
    pub runtime_handle: PortRuntimeHandle,
    pub pool: Arc<NDArrayPool>,
    pub output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    pub shared: Arc<Shared>,
    _threads: Vec<std::thread::JoinHandle<()>>,
}

impl TimePix3Runtime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.output
    }
}

/// Create a TimePix3 port (C `ADTimePixConfig`, ADTimePix.cpp:1601).
///
/// `server_url` is the Serval endpoint, e.g. `http://localhost:8081`.
pub fn create_timepix3_detector(
    port_name: &str,
    server_url: &str,
    max_memory: usize,
) -> AsynResult<TimePix3Runtime> {
    let http = Arc::new(ServalHttp::new(server_url));
    let shared = Arc::new(Shared::new());

    let (cmd_tx, cmd_rx) = rt::command_channel::<Command>(8);
    let driver = TimePix3Driver::new(port_name, http.clone(), shared.clone(), max_memory, cmd_tx)?;

    let p = driver.p;
    let ad_params = driver.ad.params;
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(driver, RuntimeConfig::default());

    let output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let ctx = Arc::new(tasks::Ctx {
        handle: runtime_handle.port_handle().clone(),
        http,
        shared: shared.clone(),
        p,
        ad: ad_params,
        output: output.clone(),
    });
    let threads = tasks::start(ctx, cmd_rx);

    Ok(TimePix3Runtime {
        runtime_handle,
        pool,
        output,
        shared,
        _threads: threads,
    })
}
