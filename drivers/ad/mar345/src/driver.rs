//! The `PortDriver` half of the mar345 port: `writeInt32` sets the shared `mode`
//! and signals the worker events, and the runtime wiring that spawns the single
//! `mar345Task` worker.
//!
//! marServer I/O is not done here — a `PortDriver` method runs inside the port
//! actor, whose runtime cannot block on a second port — so each branch only
//! updates `mode` (an atomic shared with the worker) and signals the
//! start / stop / abort events, exactly as C's `writeInt32` does.

use std::sync::Arc;
use std::sync::atomic::{AtomicI32, AtomicU64, Ordering};

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ImageMode};
use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use parking_lot::Mutex;

use crate::params::Mar345Params;
use crate::server::Server;
use crate::task::{Worker, start_task};
use crate::types::{EraseMode, Event, Mode, Resolution, ScanSize, Status, TriggerMode};

/// The maximum image plate dimension (345 mm at 0.10 mm), C's `dims[0/1]` and
/// the `ADMaxSizeX`/`ADMaxSizeY` the constructor writes.
const MAX_DIM: i32 = 3450;

pub struct Mar345Driver {
    pub ad: ADDriverBase,
    pub p: Mar345Params,
    /// C `startEventId` — wakes `mar345Task` to run the current `mode`.
    start: Arc<Event>,
    /// C `stopEventId` — ends an exposure early (does not abort).
    stop: Arc<Event>,
    /// C `abortEventId` — aborts the current operation.
    abort: Arc<Event>,
    /// C `mar345Mode_t mode`, shared with the worker.
    mode: Arc<AtomicI32>,
    /// C `pData` (mar345.cpp:741-747): allocated once and never read, so it
    /// stays charged on the pool for the driver's life.
    _raw_buffer: Option<DriverArray>,
}

/// The handles the driver shares with its worker thread.
struct DriverLinks {
    start: Arc<Event>,
    stop: Arc<Event>,
    abort: Arc<Event>,
    mode: Arc<AtomicI32>,
}

impl Mar345Driver {
    fn new(port_name: &str, max_memory: usize, links: DriverLinks) -> AsynResult<Self> {
        // C's ADDriver base takes no sensor size; ADMaxSizeX/Y are set to 3450
        // explicitly below, and ADSizeX/Y are left at the base default of 0.
        let mut ad = ADDriverBase::new(port_name, 1, 0, 0, max_memory, ASYN_CANBLOCK)?;
        let p = Mar345Params::create(&mut ad.port_base)?;

        let params = ad.params;
        let base = &mut ad.port_base;
        // C constructor defaults (mar345.cpp:745-766).
        base.set_int32_param(params.max_size_x, 0, MAX_DIM)?;
        base.set_int32_param(params.max_size_y, 0, MAX_DIM)?;
        base.set_string_param(params.base.manufacturer, 0, "MAR")?;
        base.set_string_param(params.base.model, 0, "345")?;
        base.set_int32_param(params.base.data_type, 0, NDDataType::Int16 as i32)?;
        base.set_int32_param(params.image_mode, 0, ImageMode::Single as i32)?;
        base.set_int32_param(params.trigger_mode, 0, TriggerMode::Internal as i32)?;
        base.set_float64_param(params.acquire_time, 0, 1.0)?;
        base.set_float64_param(params.acquire_period, 0, 0.0)?;
        base.set_int32_param(params.num_images, 0, 1)?;
        base.set_int32_param(p.erase_mode, 0, EraseMode::After as i32)?;
        base.set_int32_param(p.size, 0, ScanSize::S345 as i32)?;
        base.set_int32_param(p.res, 0, Resolution::R100 as i32)?;
        base.set_int32_param(p.num_erase, 0, 1)?;
        base.set_int32_param(p.num_erased, 0, 0)?;
        base.set_int32_param(p.erase, 0, 0)?;

        let raw_buffer = ad.pool.alloc(
            vec![
                NDDimension::new(MAX_DIM as usize),
                NDDimension::new(MAX_DIM as usize),
            ],
            NDDataType::Int16,
        );

        Ok(Self {
            ad,
            p,
            start: links.start,
            stop: links.stop,
            abort: links.abort,
            mode: links.mode,
            _raw_buffer: raw_buffer.ok(),
        })
    }
}

impl PortDriver for Mar345Driver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    /// C inherits `ADDriver::writeOctet`.
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        self.ad.write_octet(user, data)
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let function = user.reason;
        let params = self.ad.params;
        let p = self.p;

        // C: `setIntegerParam(function, value)` for every function.
        self.ad
            .port_base
            .set_int32_param(function, user.addr, value)?;

        let idle = self.mode.load(Ordering::SeqCst) == Mode::Idle as i32;
        let mut status = Ok(());
        if function == params.acquire {
            if value != 0 && idle {
                // Wake the mar345 task to acquire.
                self.mode.store(Mode::Acquire as i32, Ordering::SeqCst);
                self.start.signal();
            }
            if value == 0 && !idle {
                // Stop acquiring (ends exposure, does not abort).
                self.stop.signal();
            }
        } else if function == p.erase {
            if value != 0 && idle {
                self.mode.store(Mode::Erase as i32, Ordering::SeqCst);
                self.start.signal();
            }
        } else if function == p.change_mode {
            if value != 0 && idle {
                self.mode.store(Mode::Change as i32, Ordering::SeqCst);
                self.start.signal();
            }
        } else if function == p.abort {
            if value != 0 && !idle {
                // Abort operation.
                self.ad
                    .port_base
                    .set_int32_param(params.status, 0, Status::Aborting as i32)?;
                self.abort.signal();
            }
        } else if function < p.first() {
            // Base-class parameter: pool controls plus plain stores.
            status = self.ad.write_int32_pool(function, value).map(|_| ());
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "mar345:writeInt32: error, status={} function={function}, value={value}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("mar345:writeInt32: function={function}, value={value}"),
            ),
        }
        status
    }
}

impl ADDriver for Mar345Driver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------

pub struct Mar345Runtime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub mar345_params: Mar345Params,
    pool: DriverPool,
    array_output: Arc<Mutex<NDArrayOutput>>,
    /// Worker thread; kept alive for the IOC's lifetime.
    pub tasks: Vec<std::thread::JoinHandle<()>>,
}

impl Mar345Runtime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &DriverPool {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, sender: NDArraySender) {
        self.array_output.lock().add(sender);
    }
}

/// C `mar345Config`.
///
/// `server_port` must already exist (`drvAsynIPPortConfigure`), because C
/// connects to it with `pasynOctetSyncIO->connect` in the constructor.
pub fn create_mar345_detector(
    claim: &PortClaim,
    server_port: &str,
    max_memory: usize,
) -> Result<Mar345Runtime, String> {
    let port_name = claim.name();
    let server_entry = get_port(server_port).ok_or_else(|| {
        format!("marServer port '{server_port}' not found (call drvAsynIPPortConfigure first)")
    })?;
    let server_handle = server_entry.handle.clone();

    let start = Arc::new(Event::new());
    let stop = Arc::new(Event::new());
    let abort = Arc::new(Event::new());
    let mode = Arc::new(AtomicI32::new(Mode::Idle as i32));
    let acq_start = Arc::new(AtomicU64::new(0));

    let driver = Mar345Driver::new(
        port_name,
        max_memory,
        DriverLinks {
            start: start.clone(),
            stop: stop.clone(),
            abort: abort.clone(),
            mode: mode.clone(),
        },
    )
    .map_err(|e| format!("failed to create mar345 driver: {e}"))?;

    let ad_params = driver.ad.params;
    let mar345_params = driver.p;
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(claim, driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the mar345 port runtime: {e}"))?;
    let handle = runtime_handle.port_handle().clone();
    let array_output = Arc::new(Mutex::new(NDArrayOutput::new()));

    let server = Server::new(handle, server_handle, ad_params);
    let worker = Worker {
        server,
        p: mar345_params,
        start,
        stop,
        abort,
        mode,
        acq_start,
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
    };
    let tasks = vec![start_task(worker)];

    Ok(Mar345Runtime {
        runtime_handle,
        ad_params,
        mar345_params,
        pool,
        array_output,
        tasks,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn driver(name: &str, max_memory: usize) -> Mar345Driver {
        Mar345Driver::new(
            name,
            max_memory,
            DriverLinks {
                start: Arc::new(Event::new()),
                stop: Arc::new(Event::new()),
                abort: Arc::new(Event::new()),
                mode: Arc::new(AtomicI32::new(0)),
            },
        )
        .unwrap()
    }

    #[test]
    fn the_raw_buffer_is_charged_at_construction() {
        let d = driver("MAR345_RAW", 0);
        assert_eq!(d.ad.pool.num_alloc_buffers(), 1);
        assert_eq!(d.ad.pool.allocated_bytes(), 3450 * 3450 * 2);
    }

    #[test]
    fn a_pool_without_room_for_the_raw_buffer_still_builds_the_driver() {
        let d = driver("MAR345_RAW_REFUSED", 1000);
        assert_eq!(d.ad.pool.num_alloc_buffers(), 0);
    }
}
