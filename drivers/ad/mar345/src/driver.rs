//! The `PortDriver` half of the mar345 port: `writeInt32` sets the shared `mode`
//! and signals the worker events, exactly as C's does, and the runtime wiring
//! that spawns the single `mar345Task` worker. The one write that talks to
//! marServer is a detector shutter, inline as in C.

use std::sync::Arc;
use std::sync::atomic::{AtomicI32, AtomicU64, Ordering};
use std::time::Duration;

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ImageMode, ShutterMode};
use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use parking_lot::Mutex;

use crate::params::Mar345Params;
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
    /// C `pasynUserServer`: the user of the marServer octet port, shared with
    /// the worker.
    pub(crate) server: Arc<SyncIOHandle>,
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
    server: Arc<SyncIOHandle>,
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
            server: links.server,
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

        // C: `status = setIntegerParam(function, value)` for every function,
        // on list 0; it stands unless the base class replaces it.
        let mut status = self.ad.port_base.set_int32_param(function, 0, value);

        let idle = self.mode.load(Ordering::SeqCst) == Mode::Idle as i32;
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
            status = ADDriver::ad_write_int32(self, user, value);
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

    /// `mar345::setShutter` (mar345.cpp:372-397).
    fn set_shutter(&mut self, open: bool) {
        let mode = self
            .ad
            .port_base
            .get_int32_param(self.ad.params.shutter_mode, 0)
            .unwrap_or(0);
        if ShutterMode::from_i32(mode) == Some(ShutterMode::DetectorOnly) {
            self.set_detector_shutter(open);
            return;
        }
        self.ad.set_shutter(open);
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
/// C connects to `server_port` with `pasynOctetSyncIO->connect` in the
/// constructor and ignores the status (mar345.cpp:740): a port that does not
/// exist leaves a user whose every marServer exchange fails.
pub fn create_mar345_detector(
    claim: &PortClaim,
    server_port: &str,
    max_memory: usize,
) -> Result<Mar345Runtime, String> {
    let port_name = claim.name();
    let server_handle = Arc::new(
        SyncIOHandle::connect_port(server_port, 0, Duration::ZERO, InterfaceType::Octet).0,
    );

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
            server: server_handle.clone(),
        },
    )
    .map_err(|e| format!("failed to create mar345 driver: {e}"))?;

    let ad_params = driver.ad.params;
    let mar345_params = driver.p;
    let pool = driver.ad.pool.clone();
    let owner = driver.ad.owner.clone();

    let (runtime_handle, _) = create_port_runtime(claim, driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the mar345 port runtime: {e}"))?;
    let handle = runtime_handle.port_handle().clone();
    let array_output = Arc::new(Mutex::new(NDArrayOutput::new()));

    let worker = Worker {
        port: handle,
        server_port: server_handle,
        ad: ad_params,
        p: mar345_params,
        from: String::new(),
        start,
        stop,
        abort,
        mode,
        acq_start,
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
        owner,
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
pub(crate) mod tests {
    use super::*;
    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;
    use std::io::Read;

    /// An IP port `{name}_IP` on a server that logs what it receives.
    pub(crate) fn fake_server(name: &str) -> Arc<Mutex<String>> {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let log = Arc::new(Mutex::new(String::new()));
        let seen = log.clone();
        std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut buf = [0u8; 512];
            while let Ok(n @ 1..) = stream.read(&mut buf) {
                seen.lock().push_str(&String::from_utf8_lossy(&buf[..n]));
            }
        });
        let ip = format!("{name}_IP");
        let claim = claim_port(&ip).unwrap();
        let (runtime, _) = create_port_runtime(
            &claim,
            DrvAsynIPPort::new(&ip, &addr).unwrap(),
            RuntimeConfig::default(),
        )
        .unwrap();
        claim
            .publish(runtime.port_handle().clone(), &InterruptGate::opened())
            .unwrap();
        std::mem::forget(runtime);
        log
    }

    /// The user of the fake server port `{name}_IP`.
    pub(crate) fn server_user(name: &str) -> SyncIOHandle {
        SyncIOHandle::connect_port(
            &format!("{name}_IP"),
            0,
            Duration::ZERO,
            InterfaceType::Octet,
        )
        .0
    }

    /// A driver on a fake server, outside any port runtime.
    fn driver(name: &str, max_memory: usize) -> Mar345Driver {
        fake_server(name);
        Mar345Driver::new(
            name,
            max_memory,
            DriverLinks {
                start: Arc::new(Event::new()),
                stop: Arc::new(Event::new()),
                abort: Arc::new(Event::new()),
                mode: Arc::new(AtomicI32::new(0)),
                server: Arc::new(server_user(name)),
            },
        )
        .unwrap()
    }

    /// ADDriver.cpp:96-98 calls the virtual setShutter, which mar345
    /// overrides to drive a detector shutter through marServer, waiting out
    /// the delay before ADShutterStatus (:372-396): all of it inside the write.
    #[test]
    fn a_detector_shutter_is_done_when_the_write_returns() {
        let name = "MAR345_SHUTTER";
        let log = fake_server(name);
        let det =
            create_mar345_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        let (h, ad) = (det.port_handle(), det.ad_params);
        h.write_float64_blocking(ad.shutter_open_delay, 0, 0.2)
            .unwrap();
        h.write_int32_blocking(ad.shutter_mode, 0, ShutterMode::DetectorOnly as i32)
            .unwrap();
        let start = std::time::Instant::now();
        h.write_int32_blocking(ad.shutter_control, 0, 1).unwrap();
        assert!(start.elapsed().as_secs_f64() >= 0.2, "the open delay");
        assert_eq!(h.read_int32_blocking(ad.shutter_status, 0).unwrap(), 1);
        assert_eq!(
            h.get_string_param_blocking(ad.string_to_server, 0, 256)
                .unwrap(),
            b"COMMAND SHUTTER OPEN"
        );
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20);
        while !log.lock().contains("COMMAND SHUTTER OPEN") {
            assert!(
                std::time::Instant::now() < deadline,
                "never reached marServer"
            );
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        std::mem::forget(det);
    }

    /// mar345.cpp:602: setIntegerParam(function, value) is list 0 whatever
    /// the address.
    #[test]
    fn a_write_at_another_address_sets_list_zero() {
        let mut d = driver("MAR345_LIST0", 0);
        let reason = d.p.num_erase;
        let mut user = AsynUser::new(reason).with_addr(1);
        d.write_int32(&mut user, 3).unwrap();
        assert_eq!(d.ad.port_base.get_int32_param(reason, 0).unwrap(), 3);
    }

    /// A non-detector mode is ADDriver::setShutter inside the write
    /// (ADDriver.cpp:43-47), with no shutter thread to reach.
    #[test]
    fn an_epics_shutter_is_set_inside_the_write() {
        let mut d = driver("MAR345_EPICS_SHUTTER", 0);
        let ad = d.ad.params;
        d.ad.port_base
            .set_int32_param(ad.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        d.ad_write_int32(&AsynUser::new(ad.shutter_control).with_addr(0), 1)
            .unwrap();
        assert_eq!(
            d.ad.port_base
                .get_int32_param(ad.shutter_control_epics, 0)
                .unwrap(),
            1
        );
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

    /// C ignores the marServer `pasynOctetSyncIO->connect` status
    /// (mar345.cpp:740): with no server port configured the detector port
    /// still exists.
    #[test]
    fn a_detector_whose_server_port_is_missing_still_boots() {
        let rt = create_mar345_detector(&claim_port("MAR345_NONE").unwrap(), "MAR345_NONE_SRV", 0)
            .expect("the detector port");
        let sync = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(5));
        sync.write_int32(rt.ad_params.num_images, 3).unwrap();
        assert_eq!(sync.read_int32(rt.ad_params.num_images).unwrap(), 3);
    }
}
