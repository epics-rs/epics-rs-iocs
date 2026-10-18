//! The `PortDriver` half of the marCCD port: `writeInt32` / `writeFloat64`
//! run their marServer round trips inline, as `marCCD.cpp` does under the
//! driver lock, through the server helpers in [`crate::server`].

use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::time::Duration;

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::write_tail::{OwnList, WriteTail};
use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ImageMode, ShutterMode};
use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use parking_lot::Mutex;

use crate::params::MarccdParams;
use crate::protocol::{
    cmd_set_bin, cmd_set_frameshift, cmd_set_gating, cmd_set_readout_mode, cmd_set_stability,
};
use crate::task::{Worker, start_det_task, start_image_task};
use crate::types::{
    Event, FrameType, TASK_ACQUIRE, TASK_STATUS_EXECUTING, TASK_STATUS_QUEUED, TriggerMode,
    test_task_status,
};

const INT32_TAIL: WriteTail = WriteTail {
    error: "marCCD:writeInt32: error, ",
    done: "marCCD:writeInt32: ",
    list: OwnList::Zero,
};

const FLOAT64_TAIL: WriteTail = WriteTail {
    error: "marCCD:writeFloat64: error, ",
    done: "marCCD:writeFloat64: ",
    list: OwnList::Zero,
};

/// C `DRIVER_VERSION`.
const DRIVER_VERSION: &str = "2.3.0";

pub struct MarccdDriver {
    pub ad: ADDriverBase,
    pub p: MarccdParams,
    /// C `pasynUserServer`; it has no port when the startup script named a
    /// marServer port that does not exist.
    pub(crate) server: SyncIOHandle,
    /// C `serverMode` (1 = marCCD, 2 = high speed).
    pub(crate) server_mode: i32,
    /// C `fromServer`.
    pub(crate) from: String,
    /// C `startEventId`.
    start: Arc<Event>,
    /// C `stopEventId`.
    stop: Arc<Event>,
    /// C `pData` (marCCD.cpp:1556-1560): allocated once and never read, so it
    /// stays charged on the pool for the driver's life.
    _raw_buffer: Option<DriverArray>,
}

/// The handles the driver shares with its worker threads.
struct DriverLinks {
    server: SyncIOHandle,
    start: Arc<Event>,
    stop: Arc<Event>,
}

impl MarccdDriver {
    fn new(port_name: &str, max_memory: usize, links: DriverLinks) -> AsynResult<Self> {
        // marCCDConfig passes no sensor size; C's ADDriver base leaves
        // ADMaxSizeX/Y at 0 and `getConfig` fills them from the server.
        let mut ad = ADDriverBase::new(
            port_name,
            1,
            0,
            0,
            max_memory,
            ASYN_CANBLOCK,
            InterfaceMask::UNMIGRATED_SUBCLASS,
            InterfaceMask::UNMIGRATED_SUBCLASS_INTERRUPTS,
        )?;
        let p = MarccdParams::create(&mut ad.port_base)?;

        Ok(Self {
            ad,
            p,
            server: links.server,
            server_mode: 1,
            from: String::new(),
            start: links.start,
            stop: links.stop,
            _raw_buffer: None,
        })
    }

    /// The end of C's constructor (marCCD.cpp:1542-1580): `getServerMode`,
    /// `getConfig`, `getState`, the `pData` allocation and the defaults.
    /// `false` where C returns before creating the threads.
    fn init(&mut self) -> bool {
        if self.get_server_mode() != AsynStatus::Success {
            let msg = format!(
                "marCCD:marCCD: error calling getServerMode for server port {}",
                self.server.port().map_or("", PortHandle::port_name)
            );
            self.trace(TraceMask::ERROR, &msg);
            return false;
        }
        self.get_config();
        self.get_state();
        let dims = vec![
            NDDimension::new(self.get_i32(self.ad.params.max_size_x) as usize),
            NDDimension::new(self.get_i32(self.ad.params.max_size_y) as usize),
        ];
        self._raw_buffer = self.ad.pool.alloc(dims, NDDataType::Int16).ok();
        if self.set_defaults().is_err() {
            println!("marCCD: unable to set camera parameters");
            return false;
        }
        true
    }

    /// C constructor defaults (marCCD.cpp:1563-1574).
    fn set_defaults(&mut self) -> AsynResult<()> {
        let (params, p) = (self.ad.params, self.p);
        let base = &mut self.ad.port_base;
        base.set_string_param(params.base.manufacturer, 0, "MAR")?;
        base.set_string_param(params.base.model, 0, "CCD")?;
        base.set_int32_param(params.base.data_type, 0, NDDataType::Int16 as i32)?;
        base.set_int32_param(params.image_mode, 0, ImageMode::Single as i32)?;
        base.set_int32_param(params.trigger_mode, 0, TriggerMode::Internal as i32)?;
        base.set_float64_param(params.acquire_time, 0, 1.0)?;
        base.set_float64_param(params.acquire_period, 0, 0.0)?;
        base.set_int32_param(params.num_images, 0, 1)?;
        base.set_int32_param(p.overlap, 0, 0)?;
        base.set_string_param(params.base.driver_version, 0, DRIVER_VERSION)?;
        base.set_float64_param(p.tiff_timeout, 0, 20.0)
    }
}

impl PortDriver for MarccdDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let function = user.reason;
        let params = self.ad.params;
        let p = self.p;

        // C: `getIntegerParam(ADAcquire, &acquiring)` (the value *before* the
        // set) then `setIntegerParam(function, value)`.
        let acquiring = self
            .ad
            .port_base
            .get_int32_param(params.acquire, 0)
            .unwrap_or(0);

        INT32_TAIL.write_int32(self, user, value, |det, user, mut status| {
            if function == params.acquire {
                let state = det.get_state();
                if value != 0
                    && test_task_status(
                        state,
                        TASK_ACQUIRE,
                        TASK_STATUS_QUEUED | TASK_STATUS_EXECUTING,
                    ) == 0
                {
                    // Kill any stale stop event, then wake the marCCD task.
                    det.stop.try_wait();
                    det.start.signal();
                }
                if value == 0 && acquiring != 0 {
                    // The inline exposure deadline replaces C's epicsTimerCancel.
                    det.stop.signal();
                }
            } else if function == params.bin_x || function == params.bin_y {
                // marCCDServer only reports the binning after the next image, so
                // C does not read it back here.
                let bin_x = det.get_i32(params.bin_x);
                let bin_y = det.get_i32(params.bin_y);
                det.write_server(&cmd_set_bin(bin_x, bin_y));
            } else if function == p.gate_mode && det.server_mode == 2 {
                det.write_server(&cmd_set_gating(value));
                det.get_config();
            } else if function == p.readout_mode && det.server_mode == 2 {
                det.write_server(&cmd_set_readout_mode(value));
                det.get_config();
            } else if function == p.frame_shift {
                det.write_server(&cmd_set_frameshift(value));
                det.get_config();
            } else if function == params.read_status {
                if value != 0 {
                    det.get_state();
                }
            } else if function == params.base.write_file {
                // Raw frames are saved uncorrected.
                let frame_type = det.get_i32(params.frame_type);
                let corrected_flag = i32::from(frame_type != FrameType::Raw as i32);
                det.save_file(corrected_flag, true);
            } else if function < p.first() {
                status = ADDriver::ad_write_int32(det, user, value);
            }
            status
        })
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let function = user.reason;
        let p = self.p;

        FLOAT64_TAIL.write_float64(self, user, value, |det, user, mut status| {
            if function == p.stability {
                det.write_server(&cmd_set_stability(value));
                det.get_config();
            } else if function < p.gate_mode {
                // ADDriver::writeFloat64 is asynPortDriver's (marCCD.cpp:1344).
                status = det.ad.port_base.write_float64_param(user, value);
            }
            status
        })
    }

    /// C inherits `ADDriver::writeOctet`.
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        self.ad.write_octet(user, data)
    }
}

impl ADDriver for MarccdDriver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }

    /// `marCCD::setShutter` (marCCD.cpp:712-750).
    fn set_shutter(&mut self, open: bool) {
        let mode = self.get_i32(self.ad.params.shutter_mode);
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

pub struct MarccdRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub marccd_params: MarccdParams,
    pool: DriverPool,
    array_output: Arc<Mutex<NDArrayOutput>>,
    /// Worker threads; kept alive for the IOC's lifetime.
    pub tasks: Vec<std::thread::JoinHandle<()>>,
}

impl MarccdRuntime {
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

/// C `marCCDConfig`.
///
/// C connects to `server_port` with `pasynOctetSyncIO->connect` in the
/// constructor; when that fails the constructor returns with the port
/// registered but no defaults and no threads (marCCD.cpp:1534-1540).
pub fn create_marccd_detector(
    claim: &PortClaim,
    server_port: &str,
    max_memory: usize,
) -> Result<MarccdRuntime, String> {
    let port_name = claim.name();
    let (server, connected) =
        SyncIOHandle::connect_port(server_port, 0, Duration::ZERO, InterfaceType::Octet);

    let start = Arc::new(Event::new());
    let stop = Arc::new(Event::new());

    let driver = MarccdDriver::new(
        port_name,
        max_memory,
        DriverLinks {
            server,
            start: start.clone(),
            stop: stop.clone(),
        },
    )
    .map_err(|e| format!("failed to create marCCD driver: {e}"))?;
    if connected.is_err() {
        driver.trace(
            TraceMask::ERROR,
            &format!(
                "marCCD:marCCD: error calling pasynOctetSyncIO->connect for server port {server_port}"
            ),
        );
    }

    let ad_params = driver.ad.params;
    let marccd_params = driver.p;
    let pool = driver.ad.pool.clone();
    let owner = driver.ad.owner.clone();

    let (runtime_handle, _) = create_port_runtime(claim, driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the marCCD port runtime: {e}"))?;
    let handle = runtime_handle.port_handle().clone();
    let array_output = Arc::new(Mutex::new(NDArrayOutput::new()));

    let acq_start = Arc::new(AtomicU64::new(0));
    // `image_event` (C `imageEventId`) is signalled by the det task in overlap
    // mode and awaited by the image task.
    let image_ev = Arc::new(Event::new());
    let worker = || Worker {
        port: handle.clone(),
        start: start.clone(),
        stop: stop.clone(),
        image_event: image_ev.clone(),
        acq_start: acq_start.clone(),
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
        owner: owner.clone(),
    };

    let started = connected.is_ok()
        && handle
            .with_driver_blocking::<MarccdDriver, _>(MarccdDriver::init)
            .map_err(|e| format!("marCCD init failed: {e}"))?;
    let tasks = if started {
        vec![start_det_task(worker()), start_image_task(worker())]
    } else {
        Vec::new()
    };

    Ok(MarccdRuntime {
        runtime_handle,
        ad_params,
        marccd_params,
        pool,
        array_output,
        tasks,
    })
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::time::{Duration, Instant};

    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::error::AsynError;
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;

    /// A marServer that answers each command with `reply(command)` and logs
    /// what it was sent.
    pub(crate) fn fake_server(
        name: &str,
        reply: fn(&str) -> &'static str,
    ) -> Arc<Mutex<Vec<String>>> {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let log = Arc::new(Mutex::new(Vec::new()));
        let seen = log.clone();
        std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut buf = [0u8; 512];
            while let Ok(n) = stream.read(&mut buf) {
                if n == 0 {
                    break;
                }
                let command = String::from_utf8_lossy(&buf[..n]).into_owned();
                // Only a `get_*` waits for a reply, so it ends its chunk.
                let last = command
                    .rfind("get_")
                    .map_or(&command[..], |i| &command[i..]);
                let answer = reply(last);
                seen.lock().push(command);
                if !answer.is_empty() && stream.write_all(answer.as_bytes()).is_err() {
                    break;
                }
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

    /// A driver on a silent fake server, outside any port runtime.
    fn driver(name: &str) -> MarccdDriver {
        fake_server(name, |_| "");
        let (server, connected) = SyncIOHandle::connect_port(
            &format!("{name}_IP"),
            0,
            Duration::ZERO,
            InterfaceType::Octet,
        );
        connected.unwrap();
        let links = DriverLinks {
            server,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
        };
        MarccdDriver::new(name, 0, links).unwrap()
    }

    /// An octet port whose every write and read fails with `status`.
    struct RefusingPort {
        base: PortDriverBase,
        status: AsynStatus,
    }

    impl PortDriver for RefusingPort {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, _data: &[u8]) -> AsynResult<usize> {
            Err(AsynError::Status {
                status: self.status,
                message: "refused".into(),
            })
        }
        fn io_read_octet(&mut self, _user: &AsynUser, _buf: &mut [u8]) -> AsynResult<usize> {
            Err(AsynError::Status {
                status: self.status,
                message: "refused".into(),
            })
        }
    }

    /// C `writeServer` / `readServer` return `pasynOctetSyncIO`'s own status
    /// (marCCD.cpp:494-535), not a Timeout-or-Error subset.
    #[test]
    fn server_io_keeps_the_octet_ports_status() {
        for status in [
            AsynStatus::Timeout,
            AsynStatus::Overflow,
            AsynStatus::Error,
            AsynStatus::Disconnected,
            AsynStatus::Disabled,
        ] {
            let name = format!("MARCCD_REFUSING_{status:?}");
            let (server, _) = create_port_runtime(
                &claim_port(&name).unwrap(),
                RefusingPort {
                    base: PortDriverBase::new(
                        &name,
                        1,
                        Default::default(),
                        InterfaceMask::UNMIGRATED_ALL,
                        InterfaceMask::UNMIGRATED_ALL,
                    ),
                    status,
                },
                RuntimeConfig::default(),
            )
            .unwrap();
            let links = DriverLinks {
                server: SyncIOHandle::from_handle(server.port_handle().clone(), 0, Duration::ZERO),
                start: Arc::new(Event::new()),
                stop: Arc::new(Event::new()),
            };
            let mut det = MarccdDriver::new(&format!("{name}_DET"), 0, links).unwrap();
            assert_eq!(
                det.write_server("x") as i32,
                status as i32,
                "write {status:?}"
            );
            assert_eq!(
                det.read_server(0.1) as i32,
                status as i32,
                "read {status:?}"
            );
            std::mem::forget(server);
        }
    }

    /// marCCD.cpp:1467 adds asynEnumMask to ADDriver's interrupt mask, and
    /// the port refuses a client outside it (`getInterruptPvt`,
    /// asynManager.c:2492-2497): the enum callbacks must register.
    #[test]
    fn an_enum_interrupt_client_registers() {
        use epics_rs::asyn::interrupt::InterruptFilter;
        let d = driver("MARCCD_ENUMINT");
        let refused =
            d.ad.port_base
                .interrupts
                .register_interrupt_user(
                    InterruptFilter {
                        iface: Some(InterfaceType::Enum),
                        ..Default::default()
                    },
                    0,
                )
                .err()
                .map(|e| e.message_display().to_string());
        assert_eq!(refused, None);
    }

    /// marCCD.cpp:1261, :1336: setIntegerParam / setDoubleParam(function,
    /// value) are list 0 whatever the address.
    #[test]
    fn a_write_at_another_address_sets_list_zero() {
        let mut d = driver("MARCCD_LIST0");
        let (int, float) = (d.p.series_file_digits, d.p.tiff_timeout);
        d.write_int32(&mut AsynUser::new(int).with_addr(1), 3)
            .unwrap();
        d.write_float64(&mut AsynUser::new(float).with_addr(1), 2.5)
            .unwrap();
        assert_eq!(d.ad.port_base.get_int32_param(int, 0).unwrap(), 3);
        assert_eq!(d.ad.port_base.get_float64_param(float, 0).unwrap(), 2.5);
    }

    /// marCCD.cpp:1316-1319, :1352-1355 print asynParamWrongType for a write
    /// of the other type.
    #[test]
    fn a_refused_write_prints_the_param_librarys_status() {
        use epics_rs::asyn::services::PortServices;
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let port = "MARCCD_STATUS";
        let mut d = driver(port);
        let manager = PortServices::global().trace().clone();
        let log = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = std::fs::File::create(&log).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        let user = |reason| {
            let mut user = AsynUser::new(reason);
            user.trace = Some(UserTrace {
                manager: manager.clone(),
                port: port.into(),
            });
            user
        };
        let (float, int) = (d.p.tiff_timeout, d.p.gate_mode);
        assert!(d.write_int32(&mut user(float), 1).is_err());
        assert!(d.write_float64(&mut user(int), 1.0).is_err());
        let text = std::fs::read_to_string(&log).unwrap();
        let _ = std::fs::remove_file(&log);
        for line in [
            format!("marCCD:writeInt32: error, status=8 function={float}, value=1"),
            format!("marCCD:writeFloat64: error, status=8 function={int}, value=1.000000"),
        ] {
            assert!(text.contains(&line), "{line}: {text}");
        }
    }

    /// Build the detector on `name`'s fake server, then send it a `set_bin`.
    fn after_init(name: &str, reply: fn(&str) -> &'static str) -> (MarccdRuntime, Vec<String>) {
        let log = fake_server(name, reply);
        let det =
            create_marccd_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        det.port_handle()
            .write_int32_blocking(det.ad_params.bin_x, 0, 1)
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(20);
        while !log.lock().iter().any(|c| c.starts_with("set_bin")) {
            assert!(Instant::now() < deadline, "set_bin never arrived");
            std::thread::sleep(Duration::from_millis(10));
        }
        let sent = log.lock().clone();
        (det, sent)
    }

    /// How many times `command` reached the server.
    pub(crate) fn sent_count(log: &Mutex<Vec<String>>, command: &str) -> usize {
        log.lock().iter().map(|c| c.matches(command).count()).sum()
    }

    /// [`sent_count`] once it reaches `n` or 5 s pass. The fake server logs a
    /// command after its read returns, so an unanswered write can complete
    /// before its entry lands.
    pub(crate) fn wait_sent(log: &Mutex<Vec<String>>, command: &str, n: usize) -> usize {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let count = sent_count(log, command);
            if count >= n || Instant::now() >= deadline {
                return count;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    /// marCCD.cpp:1250-1357 talks to marServer inside writeInt32 /
    /// writeFloat64, and ADDriver.cpp:96-98 reaches setShutter from there
    /// (:723-743): every round trip is done when the write returns.
    #[test]
    fn server_round_trips_finish_inside_the_write() {
        let name = "MARCCD_INLINE";
        let log = fake_server(name, |c| {
            let reply = if c.starts_with("get_mode") {
                "2"
            } else if c.starts_with("get_size") {
                "64,32"
            } else if c.starts_with("get_bin") {
                "1,1"
            } else if c.starts_with("get_stability") {
                "2.5"
            } else if c.starts_with("get_") {
                "0"
            } else {
                return "";
            };
            // Longer than a queued worker could beat the next check by.
            std::thread::sleep(Duration::from_millis(100));
            reply
        });
        let det =
            create_marccd_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        let (h, ad, p) = (det.port_handle(), det.ad_params, det.marccd_params);

        h.write_int32_blocking(ad.bin_x, 0, 2).unwrap();
        let to_server = h.get_string_param_blocking(ad.string_to_server, 0, 256);
        assert_eq!(to_server.unwrap(), b"set_bin,2,1");

        let states = sent_count(&log, "get_state");
        h.write_int32_blocking(ad.read_status, 0, 1).unwrap();
        assert_eq!(sent_count(&log, "get_state"), states + 1, "ADReadStatus");
        h.write_int32_blocking(ad.acquire, 0, 0).unwrap();
        assert_eq!(sent_count(&log, "get_state"), states + 2, "ADAcquire");

        for (reason, value, command) in [
            (p.gate_mode, 1, "set_gating,1"),
            (p.readout_mode, 2, "set_readout_mode,2"),
            (p.frame_shift, 3, "set_frameshift,3"),
        ] {
            let configs = sent_count(&log, "get_stability");
            h.write_int32_blocking(reason, 0, value).unwrap();
            assert_eq!(sent_count(&log, command), 1, "{command}");
            assert_eq!(sent_count(&log, "get_stability"), configs + 1, "{command}");
        }
        h.write_float64_blocking(p.stability, 0, 1.0).unwrap();
        assert_eq!(sent_count(&log, "set_stability,1.000000"), 1);
        assert_eq!(h.read_float64_blocking(p.stability, 0).unwrap(), 2.5);

        h.write_int32_blocking(ad.base.write_file, 0, 1).unwrap();
        assert_eq!(sent_count(&log, "writefile,"), 1);

        h.write_float64_blocking(ad.shutter_open_delay, 0, 0.2)
            .unwrap();
        h.write_int32_blocking(ad.shutter_mode, 0, ShutterMode::DetectorOnly as i32)
            .unwrap();
        h.write_int32_blocking(ad.shutter_control, 0, 1).unwrap();
        assert_eq!(h.read_int32_blocking(ad.shutter_status, 0).unwrap(), 1);
        assert_eq!(wait_sent(&log, "shutter,1", 1), 1);
        std::mem::forget(det);
    }

    /// A non-detector mode is ADDriver::setShutter inside the write
    /// (ADDriver.cpp:43-47), with no worker to reach.
    #[test]
    fn an_epics_shutter_is_set_inside_the_write() {
        let mut d = driver("MARCCD_EPICS_SHUTTER");
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
    fn a_failed_get_server_mode_ends_the_init() {
        let (_det, sent) = after_init("MARCCD_BAD_MODE", |_| "7");
        assert_eq!(sent[0], "get_mode");
        assert!(sent[1].starts_with("set_bin"), "{sent:?}");
    }

    /// marCCD.cpp:1543-1549 returns from the constructor at a failed
    /// getServerMode, before the defaults (:1563-1574) and the threads.
    #[test]
    fn a_failed_get_server_mode_skips_the_defaults_and_the_tasks() {
        let (det, _) = after_init("MARCCD_BAD_MODE_DEFAULTS", |_| "7");
        let h = det.port_handle();
        let manufacturer = h.get_string_param_blocking(det.ad_params.base.manufacturer, 0, 256);
        assert!(
            matches!(manufacturer, Err(AsynError::ParamUndefined(_))),
            "{manufacturer:?}"
        );
        assert!(det.tasks.is_empty());
    }

    /// marCCD.cpp:1534-1540 returns at the failed server connect with the port
    /// registered: no defaults, no threads, and writes still land.
    #[test]
    fn a_missing_server_port_leaves_the_port_up_without_defaults() {
        let det = create_marccd_detector(&claim_port("MARCCD_NONE").unwrap(), "MARCCD_NONE_IP", 0)
            .expect("the detector port");
        assert!(det.tasks.is_empty());
        let h = det.port_handle();
        let manufacturer = h.get_string_param_blocking(det.ad_params.base.manufacturer, 0, 256);
        assert!(
            matches!(manufacturer, Err(AsynError::ParamUndefined(_))),
            "{manufacturer:?}"
        );
        h.write_int32_blocking(det.ad_params.num_images, 0, 3)
            .unwrap();
        assert_eq!(
            h.read_int32_blocking(det.ad_params.num_images, 0).unwrap(),
            3
        );
    }

    #[test]
    fn the_raw_buffer_is_charged_after_get_config() {
        let (det, _) = after_init("MARCCD_RAW", |c| {
            if c.starts_with("get_mode") {
                "1"
            } else if c.starts_with("get_size") {
                "64,32"
            } else if c.starts_with("get_bin") {
                "2,2"
            } else if c.starts_with("get_") {
                "0"
            } else {
                ""
            }
        });
        assert_eq!(det.pool().num_alloc_buffers(), 1);
        assert_eq!(det.pool().allocated_bytes(), 128 * 64 * 2);
    }

    /// ADDriver::writeFloat64 is asynPortDriver's (marCCD.cpp:1344), whose
    /// IO_DRIVER line precedes marCCD's.
    #[test]
    fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let port = "MARCCD_BASE_F64";
        let mut d = driver(port);
        let manager = epics_rs::asyn::services::PortServices::global()
            .trace()
            .clone();
        let path = std::env::temp_dir().join(format!("marccd-trace-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        let gain = d.ad.params.gain;
        let mut user = AsynUser::new(gain).with_addr(0);
        user.trace = Some(UserTrace {
            manager,
            port: port.into(),
        });
        d.write_float64(&mut user, 2.0).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let base =
            format!("asynPortDriver:writeFloat64: function={gain}, name=GAIN, value=2.000000");
        let own = format!("marCCD:writeFloat64: function={gain}, value=2.000000");
        let at = |line: &str| {
            text.find(line)
                .unwrap_or_else(|| panic!("{line}: {text:?}"))
        };
        assert!(at(&base) < at(&own));
    }

    /// reviewer5: C's marCCDTask holds the driver lock across acquireFrame's
    /// "wait for acquisition to actually start" poll (marCCD.cpp:776-781,
    /// epicsThreadSleep under lock), so a client's BinX write cannot reach
    /// marServer until the task unlocks.
    #[test]
    fn r5_a_client_write_waits_out_the_acquire_poll() {
        let name = "MARCCD_R5_LOCK";
        // marState 0 forever: the exposure never reports started.
        let log = fake_server(name, |c| {
            if c.starts_with("get_mode") {
                "1"
            } else if c.starts_with("get_size") {
                "64,32"
            } else if c.starts_with("get_bin") {
                "1,1"
            } else if c.starts_with("get_") {
                "0"
            } else {
                ""
            }
        });
        let det =
            create_marccd_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        let (h, ad) = (det.port_handle().clone(), det.ad_params);
        h.write_int32_blocking(ad.acquire, 0, 1).unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        while sent_count(&log, "start") == 0 {
            assert!(Instant::now() < deadline, "start never sent");
            std::thread::sleep(Duration::from_millis(10));
        }
        let writer = h.clone();
        std::thread::spawn(move || {
            let _ = writer.write_int32_blocking(ad.bin_x, 0, 2);
        });
        std::thread::sleep(Duration::from_secs(2));
        assert_eq!(
            sent_count(&log, "set_bin"),
            0,
            "set_bin reached marServer mid-poll"
        );
        std::mem::forget(det);
    }
}
