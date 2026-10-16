//! The background threads: the connection poll, the acquisition poll and the
//! three TCP stream workers (port of `connectionPollThread`,
//! `timePixCallback` and the `img`/`prvImg`/`prvHst` worker threads).

use std::io::Read;
use std::net::TcpStream;
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute};
use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool};
use epics_rs::ad_core::params::ad_driver::ADDriverParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interrupt::InterruptValue;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::trace::TraceMask;
use serde_json::Value;

use crate::accum::{RangeState, Update, clamp_average_to_u32};
use crate::adlog;
use crate::coordinator::Phase;
use crate::driver::TimePix3Driver;
use crate::family::DetectorFamily;
use crate::http::{ServalHttp, TIMEOUT_POLL};
use crate::params::TimePixParams;
use crate::serval;
use crate::state::{Command, Shared};
use crate::stream::{Channel, DecodeError, Frame, FrameDecoder, ImageLimits, parse_tcp_path};

/// C's `connectionPollPeriodSec_` (ADTimePix.cpp:1471).
const CONNECTION_POLL: Duration = Duration::from_secs(5);
/// C's `timePixCallback` poll period (acquire.cpp:430).
const ACQUISITION_POLL: Duration = Duration::from_millis(10);
/// How long a stream worker waits before retrying a connection.
const RECONNECT_WAIT: Duration = Duration::from_millis(200);
/// A read timeout the C worker does not set at all (UPSTREAM DEFECT,
/// network_client.cpp:157): C blocks in `recv` forever and relies on another
/// thread closing the socket underneath it, which is a use-after-close race.
const READ_TIMEOUT: Duration = Duration::from_secs(2);
/// C's `accumulationDelay` (histogram_io.cpp:963).
const PRV_HST_PAUSED_WAIT: Duration = Duration::from_millis(100);

/// Every parameter of the streaming channels lives at address 0: C's
/// `doCallbacks*Array(..., 0)` (serval_stream.cpp:975, histogram_io.cpp:541)
/// and its two-argument `setIntegerParam`/`setDoubleParam` calls all target
/// address 0, and every record in the templates binds `ADDR=0`. The port's
/// eight addresses carry the per-chip DACs and the eight *NDArray* streams,
/// not these parameters.
const ADDR: i32 = 0;

/// C `NDARRAY_ADDR_PRVIMG_THRESHOLD0` / `NDARRAY_ADDR_IMG_THRESHOLD0`
/// (ADTimePix.h:907, :919): the PrvImg and Img images' NDArray addresses.
const PRV_IMG_ADDR: usize = 0;
const IMG_ADDR: usize = 1;

pub struct Ctx {
    pub handle: PortHandle,
    pub http: Arc<ServalHttp>,
    pub shared: Arc<Shared>,
    pub p: TimePixParams,
    pub ad: ADDriverParams,
    /// One NDArray output per asyn address (C `doCallbacksGenericPointer(...,
    /// addr)`), index = address.
    pub outputs: Outputs,
    pub pool: DriverPool,
}

pub type Outputs = Arc<[Arc<parking_lot::Mutex<NDArrayOutput>>]>;

/// A fresh output for each of the port's `MAX_ADDR` addresses.
pub fn new_outputs() -> Outputs {
    (0..crate::driver::MAX_ADDR)
        .map(|_| Arc::new(parking_lot::Mutex::new(NDArrayOutput::new())))
        .collect()
}

impl Ctx {
    async fn set(&self, addr: i32, updates: Vec<ParamSetValue>) {
        let _ = self.handle.set_params_and_notify(addr, updates).await;
    }

    fn trace(&self, mask: TraceMask, line: &str) {
        self.handle
            .trace()
            .output(self.handle.port_name(), mask, line);
    }

    async fn set_int(&self, reason: usize, addr: i32, value: i32) {
        self.set(
            addr,
            vec![ParamSetValue::new(reason, addr, ParamValue::Int32(value))],
        )
        .await;
    }

    /// C `setInteger64Param` + `callParamCallbacks`.
    async fn set_int64(&self, reason: usize, addr: i32, value: i64) {
        self.set(
            addr,
            vec![ParamSetValue::new(reason, addr, ParamValue::Int64(value))],
        )
        .await;
    }

    /// C `doCallbacksGenericPointer(pArray, NDArrayData, addr)`: stamp the
    /// array's epicsTS (`updateTimeStamp`), then hand it to the plugins on
    /// `addr`.
    async fn publish_at(&self, addr: usize, mut array: DriverArray, time_stamp: f64) {
        array.time_stamp = time_stamp;
        array.timestamp = EpicsTimestamp::now();
        ArrayPublisher::new(self.outputs[addr].clone())
            .publish(array)
            .await;
    }

    /// The parameter half of C `updatePrvHstRangeState` /
    /// `updateImgRangeState` (ADTimePix.cpp:1329-1334, :1344-1349).
    async fn post_range(&self, saturated: usize, status: usize, range: Option<RangeState>) {
        let Some(range) = range else { return };
        self.set(
            ADDR,
            vec![
                ParamSetValue::new(
                    saturated,
                    ADDR,
                    ParamValue::Int32(i32::from(range != RangeState::Ok)),
                ),
                ParamSetValue::new(
                    status,
                    ADDR,
                    ParamValue::Octet(range.name().as_bytes().into()),
                ),
            ],
        )
        .await;
    }

    /// C `doCallbacksInt64Array` from the stream threads (serval_stream.cpp:975,
    /// histogram_io.cpp:541/605): the array goes to the asynInt64Array
    /// subscribers only, is not stored, and is not gated on interruptAccept
    /// (asynPortDriver.cpp:1829-1866).
    fn post_int64_array(&self, reason: usize, addr: i32, data: Vec<i64>) {
        self.handle.interrupts().notify(InterruptValue {
            reason,
            addr,
            value: ParamValue::Int64Array(data.into()),
            timestamp: SystemTime::now(),
            iface: Some(InterfaceType::Int64Array),
            ..Default::default()
        });
    }
}

/// Start every background thread. They run until the process exits.
pub fn start(
    ctx: Arc<Ctx>,
    cmd_rx: rt::CommandReceiver<Command>,
) -> Vec<std::thread::JoinHandle<()>> {
    let (acq_tx, acq_rx) = rt::command_channel::<()>(1);

    let c = ctx.clone();
    let connection = rt::run_thread_named("tpx3ConnectionPoll", move || async move {
        connection_poll(c, cmd_rx, acq_tx).await;
    });

    let c = ctx.clone();
    let acquisition = rt::run_thread_named("tpx3Acquisition", move || async move {
        acquisition_poll(c, acq_rx).await;
    });

    let mut threads = vec![connection, acquisition];
    for (name, channel) in [
        ("tpx3PrvImgWorker", StreamKind::PrvImg),
        ("tpx3ImgWorker", StreamKind::Img),
        ("tpx3PrvHstWorker", StreamKind::PrvHst),
    ] {
        let c = ctx.clone();
        threads.push(rt::run_thread_named(name, move || async move {
            stream_worker(c, channel).await;
        }));
    }
    threads
}

// ---------------------------------------------------------------------------
// Connection poll (C `connectionPollThread`, serval_http.cpp:318)
// ---------------------------------------------------------------------------

async fn connection_poll(
    ctx: Arc<Ctx>,
    mut cmd_rx: rt::CommandReceiver<Command>,
    acq_tx: rt::CommandSender<()>,
) {
    let mut was_up = false;
    loop {
        let command = tokio::time::timeout(CONNECTION_POLL, cmd_rx.recv())
            .await
            .unwrap_or(Some(Command::RefreshConnection));
        let Some(command) = command else { return };

        match command {
            Command::AcquisitionStarted => {
                let _ = acq_tx.try_send(());
                continue;
            }
            Command::RefreshStatus => {
                refresh_dashboard(&ctx).await;
                refresh_detector(&ctx).await;
                refresh_measurement_config(&ctx).await;
                continue;
            }
            Command::RefreshDetector => {
                refresh_detector(&ctx).await;
                continue;
            }
            Command::PushProcessedImg => {
                push_processed_img(&ctx).await;
                ctx.set_int(ctx.p.write_processed_img, ADDR, 0).await;
                continue;
            }
            Command::PushProcessedHst => {
                push_processed_hst(&ctx).await;
                ctx.set_int(ctx.p.write_processed_hst, ADDR, 0).await;
                continue;
            }
            Command::RefreshConnection => {}
        }

        let up = check_connection(&ctx).await;
        // A 0 -> 1 edge re-reads everything (C `refreshOnReconnect`,
        // serval_http.cpp:340).
        //
        // UPSTREAM DEFECT (ADTimePix.cpp:1579): C never initialises
        // `lastServalConnected_`/`lastDetConnected_`, so whether the first
        // successful poll runs the reconnect refresh depends on uninitialised
        // memory. Here the edge starts from "down", so the first success
        // always refreshes.
        if up && !was_up {
            refresh_dashboard(&ctx).await;
            refresh_detector(&ctx).await;
            refresh_measurement_config(&ctx).await;
        }
        was_up = up;
    }
}

/// C `getMeasurementConfig`, which the Health scan and the reconnect refresh
/// run (ADTimePix.cpp:560, serval_http.cpp:297-298).
async fn refresh_measurement_config(ctx: &Arc<Ctx>) {
    let _ = ctx
        .handle
        .with_driver::<TimePix3Driver, _>(|d| {
            let _ = d.get_measurement_config();
        })
        .await;
}

/// C `checkConnection` (serval_http.cpp:274).
async fn check_connection(ctx: &Arc<Ctx>) -> bool {
    let p = ctx.p;
    let reply = ctx.http.get_json(serval::DASHBOARD, TIMEOUT_POLL);
    let (serval_ok, det_ok, det_type, code) = match &reply {
        Ok(dashboard) => {
            let detector = dashboard.get("Detector").unwrap_or(&Value::Null);
            let det_ok = !detector.is_null();
            let det_type = if det_ok {
                serval::json_to_string(detector.get("DetectorType").unwrap_or(&Value::Null))
            } else {
                "null".to_string()
            };
            (true, det_ok, det_type, 200)
        }
        Err(e) => (false, false, "null".to_string(), e.code()),
    };

    let mut updates = vec![
        ParamSetValue::new(p.http_code, 0, ParamValue::Int32(code)),
        ParamSetValue::new(
            p.serval_connected,
            0,
            ParamValue::Int32(i32::from(serval_ok)),
        ),
        ParamSetValue::new(p.det_connected, 0, ParamValue::Int32(i32::from(det_ok))),
        ParamSetValue::new(
            p.det_type,
            0,
            ParamValue::Octet(det_type.clone().into_bytes().into()),
        ),
    ];
    connection_status(ctx, serval_ok, det_ok, &mut updates);
    if det_ok {
        updates.push(ParamSetValue::new(
            ctx.ad.base.model,
            0,
            ParamValue::Octet(det_type.into_bytes().into()),
        ));
    }
    if let Ok(dashboard) = &reply {
        serval_version(ctx, dashboard, &mut updates);
        disk_space(ctx, dashboard, &mut updates);
    }
    ctx.set(0, updates).await;
    serval_ok && det_ok
}

/// C `updateStatusFromConnection` (serval_http.cpp:251-285): a connected
/// detector reports the acquisition phase.
fn connection_status(
    ctx: &Arc<Ctx>,
    serval_ok: bool,
    det_ok: bool,
    updates: &mut Vec<ParamSetValue>,
) {
    let (message, status) = match (serval_ok, det_ok) {
        (true, true) => {
            let acquisition = ctx.shared.acquisition.snapshot();
            let status = match acquisition.phase {
                Phase::Starting | Phase::Running | Phase::Stopping => ADStatus::Acquire,
                Phase::Fault => ADStatus::Error,
                Phase::Idle => ADStatus::Idle,
            };
            (acquisition.connected_status_message(), status)
        }
        (false, false) => ("SERVAL and detector disconnected", ADStatus::Disconnected),
        (false, true) => ("SERVAL disconnected", ADStatus::Disconnected),
        (true, false) => ("Detector disconnected", ADStatus::Disconnected),
    };
    updates.push(ParamSetValue::new(
        ctx.ad.status_message,
        0,
        ParamValue::Octet(message.as_bytes().to_vec().into()),
    ));
    updates.push(ParamSetValue::new(
        ctx.ad.status,
        0,
        ParamValue::Int32(status as i32),
    ));
}

/// C `updateServalVersionFromDashboard` (serval_http.cpp:133).
fn serval_version(ctx: &Arc<Ctx>, dashboard: &Value, updates: &mut Vec<ParamSetValue>) {
    let Some(server) = dashboard.get("Server").filter(|s| s.is_object()) else {
        return;
    };
    let mut octet = |reason: usize, v: &str| {
        updates.push(ParamSetValue::new(
            reason,
            0,
            ParamValue::Octet(v.as_bytes().to_vec().into()),
        ));
    };
    if let Some(ver) = server.get("SoftwareVersion").map(serval::json_to_string)
        && !ver.is_empty()
    {
        octet(ctx.ad.base.sdk_version, &ver);
        octet(ctx.ad.base.manufacturer, "ASI");
    }
    if let Some(ts) = server.get("SoftwareTimestamp").map(serval::json_to_string)
        && !ts.is_empty()
    {
        octet(ctx.p.fw_time_stamp, &ts);
    }
}

/// The first disk in `Server.DiskSpace`, which is empty until raw file writing
/// is configured (C getDashboard, serval_http.cpp:388).
fn disk_space(ctx: &Arc<Ctx>, dashboard: &Value, updates: &mut Vec<ParamSetValue>) {
    let p = ctx.p;
    let Some(disk) = dashboard.pointer("/Server/DiskSpace/0") else {
        return;
    };
    if let Some(v) = disk.get("WriteSpeed").and_then(Value::as_f64) {
        updates.push(ParamSetValue::new(p.write_speed, 0, ParamValue::Float64(v)));
    }
    if let Some(v) = disk.get("DiskLimitReached").and_then(as_bool_or_int) {
        updates.push(ParamSetValue::new(p.l_lim_reached, 0, ParamValue::Int32(v)));
    }
}

fn as_bool_or_int(v: &Value) -> Option<i32> {
    match v {
        Value::Bool(b) => Some(i32::from(*b)),
        Value::Number(n) => n.as_i64().map(|i| i32::try_from(i).unwrap_or(0)),
        _ => None,
    }
}

/// `getDashboard`'s int64 fields.
async fn refresh_dashboard(ctx: &Arc<Ctx>) {
    let p = ctx.p;
    if let Ok(dashboard) = ctx.http.get_json(serval::DASHBOARD, TIMEOUT_POLL) {
        let mut updates = Vec::new();
        serval_version(ctx, &dashboard, &mut updates);
        ctx.set(0, updates).await;
        if let Some(disk) = dashboard.pointer("/Server/DiskSpace/0") {
            if let Some(v) = disk.get("FreeSpace").and_then(Value::as_i64) {
                ctx.set_int64(p.free_space, 0, v).await;
            }
            if let Some(v) = disk.get("LowerLimit").and_then(Value::as_i64) {
                ctx.set_int64(p.lower_limit, 0, v).await;
            }
        }
    }
}

/// C `getDetector` (serval_http.cpp:1293).
async fn refresh_detector(ctx: &Arc<Ctx>) {
    let p = ctx.p;
    let detector = match ctx.http.get_json(serval::DETECTOR, TIMEOUT_POLL) {
        Ok(d) => d,
        Err(_) => {
            ctx.set_int(p.det_connected, 0, 0).await;
            return;
        }
    };

    let mut updates = Vec::new();
    let mut octet = |reason: usize, v: Option<&Value>| {
        if let Some(v) = v {
            updates.push(ParamSetValue::new(
                reason,
                0,
                ParamValue::Octet(serval::json_to_string(v).into_bytes().into()),
            ));
        }
    };
    octet(p.iface_name, detector.pointer("/Info/IfaceName"));
    octet(p.sw_version, detector.pointer("/Info/SW_version"));
    octet(p.fw_version, detector.pointer("/Info/FW_version"));
    octet(
        ctx.ad.base.firmware_version,
        detector.pointer("/Info/FW_version"),
    );
    // UPSTREAM DEFECT (serval_http.cpp:1190): C sets ADSerialNumber from
    // `Info.SW_version` — the Serval software version, not a serial number —
    // and the line that would have used the chipboard ID sits commented out
    // right above it. The chipboard ID is the detector's identity.
    octet(
        ctx.ad.base.serial_number,
        detector.pointer("/Info/Boards/0/ChipboardId"),
    );
    octet(p.boards_id, detector.pointer("/Info/Boards/0/ChipboardId"));
    octet(p.boards_ip, detector.pointer("/Info/Boards/0/IpAddress"));
    octet(p.boards2_id, detector.pointer("/Info/Boards/1/ChipboardId"));
    octet(p.boards2_ip, detector.pointer("/Info/Boards/1/IpAddress"));
    for (n, reason) in [
        (0, p.boards_ch1),
        (1, p.boards_ch2),
        (2, p.boards_ch3),
        (3, p.boards_ch4),
    ] {
        octet(
            reason,
            detector.pointer(&format!("/Info/Boards/0/Chips/{n}")),
        );
    }
    for (n, reason) in [
        (0, p.boards_ch5),
        (1, p.boards_ch6),
        (2, p.boards_ch7),
        (3, p.boards_ch8),
    ] {
        octet(
            reason,
            detector.pointer(&format!("/Info/Boards/1/Chips/{n}")),
        );
    }
    octet(p.tdc, detector.pointer("/Config/Tdc"));

    let mut int = |reason: usize, v: Option<&Value>| {
        if let Some(v) = v.and_then(as_bool_or_int) {
            updates.push(ParamSetValue::new(reason, 0, ParamValue::Int32(v)));
        }
    };
    let pix_count = detector
        .pointer("/Info/PixCount")
        .and_then(Value::as_i64)
        .unwrap_or(0);
    let rows = detector
        .pointer("/Info/NumberOfRows")
        .and_then(Value::as_i64)
        .unwrap_or(0);
    int(p.pix_count, detector.pointer("/Info/PixCount"));
    int(p.row_len, detector.pointer("/Info/RowLen"));
    int(p.number_of_chips, detector.pointer("/Info/NumberOfChips"));
    int(p.number_of_rows, detector.pointer("/Info/NumberOfRows"));
    int(p.mpx_type, detector.pointer("/Info/MpxType"));
    int(p.supp_acq_modes, detector.pointer("/Info/SuppAcqModes"));
    int(p.max_pulse_count, detector.pointer("/Info/MaxPulseCount"));
    int(p.fan1_pw_m, detector.pointer("/Config/Fan1PWM"));
    int(p.fan2_pw_m, detector.pointer("/Config/Fan2PWM"));
    int(p.bias_volt, detector.pointer("/Config/BiasVoltage"));
    int(p.bias_enable, detector.pointer("/Config/BiasEnabled"));
    int(p.trigger_in, detector.pointer("/Config/TriggerIn"));
    int(p.trigger_out, detector.pointer("/Config/TriggerOut"));
    int(p.n_triggers, detector.pointer("/Config/nTriggers"));
    int(p.periph_clk80, detector.pointer("/Config/PeriphClk80"));
    int(
        p.external_reference_clock,
        detector.pointer("/Config/ExternalReferenceClock"),
    );
    int(p.log_level, detector.pointer("/Config/LogLevel"));

    // UPSTREAM DEFECT (serval_http.cpp:1198): C computes
    // `ADMaxSizeX = PixCount / NumberOfRows` with no guard — a detector that
    // reports `NumberOfRows: 0` (or omits it, which nlohmann turns into 0)
    // raises SIGFPE and takes the IOC down.
    if rows > 0 {
        updates.push(ParamSetValue::new(
            ctx.ad.max_size_x,
            0,
            ParamValue::Int32(i32::try_from(pix_count / rows).unwrap_or(0)),
        ));
        updates.push(ParamSetValue::new(
            ctx.ad.max_size_y,
            0,
            ParamValue::Int32(i32::try_from(rows).unwrap_or(0)),
        ));
    }

    let mut float = |reason: usize, v: Option<&Value>| {
        if let Some(v) = v.and_then(Value::as_f64) {
            updates.push(ParamSetValue::new(reason, 0, ParamValue::Float64(v)));
        }
    };
    float(p.clock_readout, detector.pointer("/Info/ClockReadout"));
    float(p.max_pulse_height, detector.pointer("/Info/MaxPulseHeight"));
    float(p.max_pulse_period, detector.pointer("/Info/MaxPulsePeriod"));
    float(p.timer_max_val, detector.pointer("/Info/TimerMaxVal"));
    float(p.timer_min_val, detector.pointer("/Info/TimerMinVal"));
    float(p.timer_step, detector.pointer("/Info/TimerStep"));
    float(p.clock_timepix, detector.pointer("/Info/ClockTimepix"));
    float(p.exposure_time, detector.pointer("/Config/ExposureTime"));
    float(
        ctx.ad.acquire_time,
        detector.pointer("/Config/ExposureTime"),
    );
    float(p.trigger_period, detector.pointer("/Config/TriggerPeriod"));
    float(
        ctx.ad.acquire_period,
        detector.pointer("/Config/TriggerPeriod"),
    );
    float(p.trigger_delay, detector.pointer("/Config/TriggerDelay"));
    float(
        p.global_timestamp_interval,
        detector.pointer("/Config/GlobalTimestampInterval"),
    );

    update_detector_family(ctx, &detector, &mut updates);

    // The detector orientation drives the mask geometry, so it also goes into
    // the shared state the mask code reads.
    if let Some(o) = detector
        .pointer("/Layout/DetectorOrientation")
        .and_then(Value::as_str)
        .and_then(serval::orientation_index)
    {
        ctx.shared.set_orientation(o);
        updates.push(ParamSetValue::new(
            p.detector_orientation,
            0,
            ParamValue::Int32(o),
        ));
    }

    health(ctx, &detector, &mut updates);
    ctx.set(0, updates).await;

    // The per-address readbacks: the DACs, the chip temperature and the chip
    // layout, one address per chip (C `fetchDacs`, serval_http.cpp:790), plus
    // the VDD/AVDD rails — three per SPIDR board, so addresses 0-2 are the
    // first board and 3-5 the second (serval_http.cpp:730-757).
    let temps = chip_temperatures(&detector);
    let rails = rail_voltages(&detector);
    let chips = detector.pointer("/Chips").and_then(Value::as_array);
    let layout = detector
        .pointer("/Layout/Original/Chips")
        .and_then(Value::as_array);
    for chip in 0..crate::driver::MAX_ADDR {
        let addr = i32::try_from(chip).unwrap_or(0);
        let mut updates = Vec::new();
        if let Some(c) = chips.and_then(|c| c.get(chip)) {
            for (name, index) in crate::driver::dac_params(&p) {
                if let Some(v) = c.pointer(&format!("/DACs/{name}")).and_then(Value::as_i64) {
                    updates.push(ParamSetValue::new(
                        index,
                        addr,
                        ParamValue::Int32(i32::try_from(v).unwrap_or(0)),
                    ));
                }
            }
            updates.push(ParamSetValue::new(
                p.adjust,
                addr,
                ParamValue::Int32(adjust_code(c)),
            ));
            updates.push(ParamSetValue::new(
                p.chip_n_temperature,
                addr,
                ParamValue::Int32(temps.get(chip).copied().unwrap_or(0)),
            ));
        }
        if let Some(c) = layout.and_then(|l| l.get(chip)) {
            updates.push(ParamSetValue::new(
                p.layout,
                addr,
                ParamValue::Octet(serval::json_to_string(c).into_bytes().into()),
            ));
        }
        if let Some(&(vdd, avdd)) = rails.get(chip) {
            updates.push(ParamSetValue::new(
                p.chip_nv_dd,
                addr,
                ParamValue::Float64(vdd),
            ));
            updates.push(ParamSetValue::new(
                p.chip_na_vd_d,
                addr,
                ParamValue::Float64(avdd),
            ));
        }
        if !updates.is_empty() {
            ctx.set(addr, updates).await;
        }
    }
}

/// `Detector.Health` is a single object on Serval 3 and an array — one entry
/// per SPIDR board — on Serval 4. Both shapes carry the same keys, so the
/// scalars come from the first board.
/// C `updateDetectorFamily` (ADTimePix.cpp:2107-2127), with the arguments
/// `getDetector` passes it (serval_http.cpp:1351-1363).
fn update_detector_family(ctx: &Ctx, detector: &Value, updates: &mut Vec<ParamSetValue>) {
    let p = ctx.p;
    let mpx_type = detector
        .pointer("/Info/MpxType")
        .and_then(Value::as_i64)
        .and_then(|v| i32::try_from(v).ok())
        .unwrap_or(0);
    let chip_type = detector
        .pointer("/Info/ChipType")
        .map(serval::json_to_string)
        .unwrap_or_default();
    let chipboard_id = detector
        .pointer("/Info/Boards/0/ChipboardId")
        .map(serval::json_to_string)
        .unwrap_or_default();
    let family = DetectorFamily::detect(mpx_type, &chip_type, &chipboard_id);
    let caps = family.capabilities();
    let int = |reason: usize, v: i32| ParamSetValue::new(reason, 0, ParamValue::Int32(v));
    updates.push(ParamSetValue::new(
        p.chip_type,
        0,
        ParamValue::Octet(chip_type.as_bytes().into()),
    ));
    updates.push(int(p.detector_family, family as i32));
    updates.push(int(p.cap_tdc, caps.supports_tdc.into()));
    updates.push(int(p.cap_tof_hist, caps.supports_tof_histogram.into()));
    updates.push(int(p.cap_dual_preview, caps.supports_dual_preview.into()));
    updates.push(int(
        p.cap_img_thresholds,
        caps.supports_image_thresholds.into(),
    ));

    if ctx.shared.note_family(family) {
        apply_family_defaults(&p, family, updates);
        ctx.trace(
            TraceMask::FLOW,
            &adlog::log(
                adlog::UPDATE_DETECTOR_FAMILY,
                &format!(
                    "Detector family {} (MpxType={mpx_type} ChipType={chip_type} ChipboardId={chipboard_id})",
                    family.name()
                ),
            ),
        );
    }
}

/// C `applyFamilyDefaults` (ADTimePix.cpp:2155-2175): Medipix3 reads two
/// thresholds in 12-bit high-gain mode.
fn apply_family_defaults(
    p: &TimePixParams,
    family: DetectorFamily,
    updates: &mut Vec<ParamSetValue>,
) {
    if family != DetectorFamily::Mpx3 {
        return;
    }
    for reason in [p.img_ths, p.img1_ths, p.prv_img_ths, p.prv_img1_ths] {
        updates.push(ParamSetValue::new(
            reason,
            0,
            ParamValue::Octet(b"0,1"[..].into()),
        ));
    }
    for (reason, v) in [
        (p.prv_img_format, 3),
        (p.prv_img1_format, 3),
        (p.gain_mode, 1),
        (p.pixel_depth, 12),
        (p.i_delay0, 15),
        (p.i_delay1, 15),
        (p.i_delay2, 15),
        (p.i_delay3, 10),
    ] {
        updates.push(ParamSetValue::new(reason, 0, ParamValue::Int32(v)));
    }
}

fn health_blocks(detector: &Value) -> Vec<&Value> {
    match detector.get("Health") {
        Some(Value::Array(a)) => a.iter().collect(),
        Some(h @ Value::Object(_)) => vec![h],
        _ => Vec::new(),
    }
}

/// `Health[*].ChipTemperatures`, flattened: on a multi-board detector each
/// board lists only its own chips, so board 1's first entry is global chip 4.
fn chip_temperatures(detector: &Value) -> Vec<i32> {
    health_blocks(detector)
        .iter()
        .filter_map(|b| b.get("ChipTemperatures").and_then(Value::as_array))
        .flatten()
        .map(|t| {
            let v = t
                .as_i64()
                .unwrap_or_else(|| t.as_f64().unwrap_or(0.0).round() as i64);
            i32::try_from(v).unwrap_or(0)
        })
        .collect()
}

/// The `(VDD, AVDD)` rails by asyn address: three rails per board, board 0 at
/// addresses 0-2 and board 1 at 3-5. A board that is absent reads 0 V.
fn rail_voltages(detector: &Value) -> Vec<(f64, f64)> {
    let blocks = health_blocks(detector);
    let rail = |board: usize, key: &str, i: usize| -> f64 {
        blocks
            .get(board)
            .and_then(|b| b.get(key))
            .and_then(Value::as_array)
            .and_then(|a| a.get(i))
            .and_then(Value::as_f64)
            .unwrap_or(0.0)
    };
    (0..6)
        .map(|addr| {
            let (board, i) = (addr / 3, addr % 3);
            (rail(board, "VDD", i), rail(board, "AVDD", i))
        })
        .collect()
}

/// `Chips[n].Adjust`: absent on Serval 3, an array on Serval 4. C reports the
/// shape rather than the value (serval_http.cpp:820-829) and so does this.
fn adjust_code(chip: &Value) -> i32 {
    match chip.get("Adjust") {
        None | Some(Value::Null) => -1,
        Some(Value::Array(_)) => -2,
        Some(_) => -3,
    }
}

/// The `Detector.Health` scalars, all at address 0.
fn health(ctx: &Arc<Ctx>, detector: &Value, updates: &mut Vec<ParamSetValue>) {
    let p = ctx.p;
    let blocks = health_blocks(detector);
    let Some(h) = blocks.first() else {
        return;
    };
    for (key, reason) in [
        ("LocalTemperature", p.local_temp),
        ("FPGATemperature", p.fp_ga_temp),
        ("Fan1Speed", p.fan1_speed),
        ("Fan2Speed", p.fan2_speed),
        ("BiasVoltage", p.bias_voltage),
    ] {
        if let Some(v) = h.get(key).and_then(Value::as_f64) {
            updates.push(ParamSetValue::new(reason, 0, ParamValue::Float64(v)));
        }
    }
    if let Some(v) = h.get("Humidity").and_then(Value::as_i64) {
        updates.push(ParamSetValue::new(
            p.humidity,
            0,
            ParamValue::Int32(i32::try_from(v).unwrap_or(0)),
        ));
    }
    // The JSON arrays, verbatim: one string per rail set, and one for the chip
    // temperatures, merged across boards.
    for (key, reason) in [("VDD", p.vd_d), ("AVDD", p.av_dd)] {
        let merged: Vec<&Value> = blocks.iter().filter_map(|b| b.get(key)).collect();
        if let Some(first) = merged.first() {
            let value = if merged.len() > 1 {
                serval::json_to_string(&Value::Array(merged.iter().map(|v| (*v).clone()).collect()))
            } else {
                serval::json_to_string(first)
            };
            updates.push(ParamSetValue::new(
                reason,
                0,
                ParamValue::Octet(value.into_bytes().into()),
            ));
        }
    }
    let temps: Vec<Value> = chip_temperatures(detector)
        .into_iter()
        .map(Value::from)
        .collect();
    if !temps.is_empty() {
        updates.push(ParamSetValue::new(
            p.chip_temperature,
            0,
            ParamValue::Octet(
                serval::json_to_string(&Value::Array(temps))
                    .into_bytes()
                    .into(),
            ),
        ));
    }
}

// ---------------------------------------------------------------------------
// Acquisition poll (C `timePixCallback`, acquire.cpp:538-686)
// ---------------------------------------------------------------------------

async fn acquisition_poll(ctx: Arc<Ctx>, mut start_rx: rt::CommandReceiver<()>) {
    while start_rx.recv().await.is_some() {
        // The start transaction commits, or rolls back, after it wakes this
        // monitor; this thread has nothing else to run while it waits.
        if !ctx
            .shared
            .acquisition
            .wait_for_running_for(Duration::from_millis(5000))
        {
            continue;
        }

        // C's first GET and its in-loop polls print different lines.
        let mut first = true;
        let stop = loop {
            let m = match poll_measurement(&ctx, first) {
                Ok(m) => m,
                Err(reason) => break Some((true, reason)),
            };
            first = false;
            let updates = measurement_updates(&ctx.p, &m)
                .into_iter()
                .map(|(reason, value)| ParamSetValue::new(reason, 0, value))
                .collect();
            ctx.set(0, updates).await;

            if m.is_idle() {
                break Some((false, ""));
            }
            if !ctx.shared.acquisition.monitoring() {
                break None;
            }
            tokio::time::sleep(ACQUISITION_POLL).await;
        };

        // C `stopAcquisition` from the monitor thread (acquire.cpp:582-683);
        // the driver owns the transition, so it runs there.
        if let Some((fault, reason)) = stop {
            let _ = ctx
                .handle
                .with_driver::<TimePix3Driver, _>(move |d| {
                    let _ = d.stop_acquisition(fault, reason);
                })
                .await;
        }
    }
}

/// C `publishMeasurementSnapshot` (acquire.cpp:40-60) and
/// `updateMeasurementStatusFromJson` (acquire.cpp:83-98).
pub(crate) fn measurement_updates(
    p: &TimePixParams,
    m: &serval::StatusSnapshot,
) -> Vec<(usize, ParamValue)> {
    let mut updates = Vec::new();
    for (v, reason) in [
        (m.pixel_event_rate, p.pel_rate),
        (m.tdc1_event_rate, p.tdc1_rate),
        (m.tdc2_event_rate, p.tdc2_rate),
    ] {
        if let Some(v) = v {
            updates.push((reason, ParamValue::Int32(v)));
        }
    }
    if let Some(v) = m.start_date_time {
        updates.push((p.start_time, ParamValue::Int64(v)));
    }
    for (v, reason) in [(m.elapsed_time, p.elapsed_time), (m.time_left, p.time_left)] {
        if let Some(v) = v {
            updates.push((reason, ParamValue::Float64(v)));
        }
    }
    for (v, reason) in [
        (m.frame_count, p.frame_count),
        (m.dropped_frames, p.dropped_frames),
    ] {
        if let Some(v) = v {
            updates.push((reason, ParamValue::Int32(v)));
        }
    }
    if let Some(status) = &m.status {
        let index = match status.as_str() {
            "DA_STARTING" => 0,
            "DA_RECORDING" => 1,
            "DA_STOPPING" => 2,
            "DA_IDLE" | "DA_STOPPED" => 3,
            _ => -1,
        };
        updates.push((p.status, ParamValue::Octet(status.as_bytes().into())));
        updates.push((p.pipeline_state, ParamValue::Int32(index)));
    }
    updates
}

/// One `GET /measurement`, printing C `timePixCallback`'s line for a failure;
/// the error is the reason the acquisition rolls back with.
fn poll_measurement(ctx: &Ctx, first: bool) -> Result<serval::StatusSnapshot, &'static str> {
    match ctx.http.get(serval::MEASUREMENT, TIMEOUT_POLL) {
        Ok(body) => serval::parse_status_response(&body).map_err(|e| {
            let what = if first { "measurement" } else { "poll" };
            ctx.trace(
                TraceMask::ERROR,
                &adlog::err(
                    adlog::TIME_PIX_CALLBACK,
                    &format!("timePixCallback: invalid {what} response: {}", e.message()),
                ),
            );
            if first {
                "Invalid measurement response; acquisition rolled back"
            } else {
                "Invalid measurement JSON; acquisition rolled back"
            }
        }),
        Err(e) => {
            let url = format!("{}{}", ctx.http.base_url(), serval::MEASUREMENT);
            let line = if first {
                adlog::http_failure("timePixCallback GET /measurement", "GET", &url, &e)
                    .map(|l| (TraceMask::ERROR, l))
            } else {
                adlog::http_warning("timePixCallback poll GET /measurement", "GET", &url, &e)
                    .map(|l| (TraceMask::WARNING, l))
            };
            if let Some((mask, line)) = line {
                ctx.trace(mask, &line);
            }
            Err(if first {
                "Measurement HTTP error; acquisition rolled back"
            } else {
                "Measurement poll HTTP error; acquisition rolled back"
            })
        }
    }
}

// ---------------------------------------------------------------------------
// Stream workers (C `imgWorkerThread`, `prvImgWorkerThread`,
// `prvHstWorkerThread`)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StreamKind {
    PrvImg,
    Img,
    PrvHst,
}

impl StreamKind {
    fn channel(self) -> Channel {
        match self {
            Self::PrvHst => Channel::Histogram,
            _ => Channel::Image,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::PrvImg => "PrvImg",
            Self::Img => "Img",
            Self::PrvHst => "PrvHst",
        }
    }

    /// The C functions whose `__PRETTY_FUNCTION__` the image channels' asyn
    /// lines carry: connect, worker thread, header parse. C's PrvHst code
    /// prints with `printf`/`fprintf` instead, so it has none.
    fn c_functions(self) -> Option<[&'static str; 3]> {
        match self {
            Self::PrvImg => Some([
                "void ADTimePix::prvImgConnect()",
                "void ADTimePix::prvImgWorkerThread()",
                "bool ADTimePix::processPrvImgDataLine(char*, char*, size_t)",
            ]),
            Self::Img => Some([
                "void ADTimePix::imgConnect()",
                "void ADTimePix::imgWorkerThread()",
                "bool ADTimePix::processImgDataLine(char*, char*, size_t)",
            ]),
            Self::PrvHst => None,
        }
    }

    fn path(self, s: &Shared) -> Option<String> {
        let paths = s.stream_paths();
        match self {
            Self::PrvImg => paths.prv_img,
            Self::Img => paths.img,
            Self::PrvHst => paths.prv_hst,
        }
    }
}

/// One worker: connect while an acquisition runs, decode frames, publish them.
///
/// C creates these threads inside `acquireStart` and never joins the one that
/// stops itself (UPSTREAM DEFECT, acquire.cpp:625-628: `epicsThreadMustJoin` is
/// skipped when the callback thread *is* the caller, leaking a joinable thread
/// on every DA_IDLE stop). Here each worker is created once and parks between
/// acquisitions.
async fn stream_worker(ctx: Arc<Ctx>, kind: StreamKind) {
    loop {
        if !ctx.shared.acquisition.monitoring() {
            tokio::time::sleep(RECONNECT_WAIT).await;
            continue;
        }
        let Some(path) = kind.path(&ctx.shared) else {
            tokio::time::sleep(RECONNECT_WAIT).await;
            continue;
        };
        let Some((host, port)) = parse_tcp_path(&path) else {
            // A file:// channel has no stream to read.
            tokio::time::sleep(RECONNECT_WAIT).await;
            continue;
        };

        let name = kind.name();
        if kind.c_functions().is_none() {
            println!("PrvHst worker thread: Attempting to connect to {host}:{port}");
        }
        let stream = match TcpStream::connect((host.as_str(), port)) {
            Ok(s) => s,
            Err(_) => {
                let msg = format!("{name} TCP failed to connect to {host}:{port}");
                match kind.c_functions() {
                    Some([connect, ..]) => ctx.trace(TraceMask::ERROR, &adlog::err(connect, &msg)),
                    None => {
                        eprintln!("ERROR | ADTimePix::prvHstConnect: {msg}");
                        println!(
                            "PrvHst worker thread: Connection failed, will retry in {:.1} seconds",
                            RECONNECT_WAIT.as_secs_f64()
                        );
                    }
                }
                tokio::time::sleep(RECONNECT_WAIT).await;
                continue;
            }
        };
        let _ = stream.set_read_timeout(Some(READ_TIMEOUT));
        let msg = format!("{name} TCP connected to {host}:{port}");
        match kind.c_functions() {
            Some([connect, ..]) => ctx.trace(TraceMask::IO_DRIVER, &adlog::log(connect, &msg)),
            None => {
                println!("{msg}");
                println!("PrvHst worker thread: Connection successful, starting data reception");
            }
        }
        read_stream(&ctx, kind, stream).await;
    }
}

async fn read_stream(ctx: &Arc<Ctx>, kind: StreamKind, mut stream: TcpStream) {
    let mut decoder = FrameDecoder::new(kind.channel());
    let mut buf = vec![0u8; 256 * 1024];

    while ctx.shared.acquisition.monitoring() {
        // C leaves the histogram socket unread while accumulation is off
        // (histogram_io.cpp:1018-1023).
        if kind == StreamKind::PrvHst && !prv_hst_accumulating(ctx).await {
            tokio::time::sleep(PRV_HST_PAUSED_WAIT).await;
            continue;
        }
        match stream.read(&mut buf) {
            Ok(0) => {
                println!("{} TCP connection closed by peer", kind.name());
                return;
            }
            Ok(n) => decoder.push(&buf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => {
                let msg = format!("{} TCP socket error: {}", kind.name(), strerror(&e));
                match kind.c_functions() {
                    Some([_, worker, _]) => {
                        ctx.trace(TraceMask::IO_DRIVER, &adlog::log(worker, &msg))
                    }
                    None => println!("{msg}"),
                }
                return;
            }
        }

        loop {
            if kind.channel() == Channel::Image {
                decoder.set_image_limits(image_limits(ctx).await);
            }
            match decoder.next_frame() {
                Ok(Some(frame)) => handle_frame(ctx, kind, frame).await,
                Ok(None) => break,
                Err(e) => {
                    // The payload length is unknown, so the stream cannot be
                    // resynchronised: drop the connection and start over.
                    decode_failed(ctx, kind, &e);
                    return;
                }
            }
        }
    }
}

/// C `ADTimePixPrvHstAccumulationEnable`, which gates the PrvHst worker and
/// both of its frame handlers.
async fn prv_hst_accumulating(ctx: &Ctx) -> bool {
    ctx.handle
        .read_int32(ctx.p.prv_hst_accumulation_enable, 0)
        .await
        .unwrap_or(0)
        != 0
}

/// C `detectorImageFrameLimits` from the detector params
/// (serval_stream.cpp:520-527).
async fn image_limits(ctx: &Ctx) -> ImageLimits {
    let int = |reason| async move { ctx.handle.read_int32(reason, 0).await.unwrap_or(0) };
    ImageLimits::detector(
        int(ctx.ad.max_size_x).await,
        int(ctx.ad.max_size_y).await,
        int(ctx.p.pix_count).await,
    )
}

/// The line C prints for a header it cannot use.
fn decode_failed(ctx: &Ctx, kind: StreamKind, e: &DecodeError) {
    match (kind.c_functions(), e) {
        (Some([_, worker, _]), DecodeError::NoNewline) => ctx.trace(
            TraceMask::IO_DRIVER,
            &adlog::log(
                worker,
                &format!(
                    "{} TCP buffer full without finding newline, resetting",
                    kind.name()
                ),
            ),
        ),
        (None, DecodeError::NoNewline) => {
            println!("PrvHst TCP buffer full without finding newline, resetting");
        }
        (Some([_, _, line]), DecodeError::Json(m)) => ctx.trace(
            TraceMask::ERROR,
            &adlog::err(line, &format!("JSON parse error: {m}")),
        ),
        (None, DecodeError::Json(m)) => {
            eprintln!("ERROR | ADTimePix::processPrvHstDataLine: JSON parse error in PrvHst: {m}");
        }
        // The worker's payload-size resolver refuses the header before the
        // line processor sees it (serval_stream.cpp:532-535, :606-611).
        (Some([_, worker, _]), DecodeError::Image(e)) => ctx.trace(
            TraceMask::ERROR,
            &adlog::err(
                worker,
                &format!(
                    "{} rejected TCP stream: invalid JSON frame header: {e}",
                    kind.name()
                ),
            ),
        ),
        (None, DecodeError::BinSize(n)) => {
            eprintln!("ERROR | ADTimePix::processPrvHstDataLine: Invalid bin size: {n}");
        }
        // Each channel's decoder only produces its own header's errors.
        _ => {}
    }
}

/// C `strerror(errno)`: the OS text without Rust's " (os error N)" suffix.
fn strerror(e: &std::io::Error) -> String {
    let text = e.to_string();
    match text.find(" (os error ") {
        Some(end) => text[..end].to_string(),
        None => text,
    }
}

async fn handle_frame(ctx: &Arc<Ctx>, kind: StreamKind, frame: Frame) {
    match (kind, frame) {
        (StreamKind::PrvImg, Frame::Image(mut f)) => {
            let pixels = std::mem::take(&mut f.pixels);
            let Some(mut array) = image_array(
                ctx,
                &f,
                pixels,
                &adlog::err(
                    adlog::PROCESS_PRV_IMG_DATA_LINE,
                    "Failed to allocate NDArray or NDArray has no data pointer",
                ),
            ) else {
                return;
            };
            let p = ctx.p;
            let rate = frame_rate(
                ctx,
                f.frame_number,
                |r| &mut r.prv_img,
                adlog::PROCESS_PREVIEW_JSONIMAGE_LINE,
                "PrvImg ",
            );
            ctx.set(
                ADDR,
                geometry(ctx, &f)
                    .into_iter()
                    .chain([
                        ParamSetValue::new(
                            p.prv_img_frame_number,
                            ADDR,
                            ParamValue::Int32(f.frame_number),
                        ),
                        ParamSetValue::new(
                            p.prv_img_time_at_frame,
                            ADDR,
                            ParamValue::Float64(f.time_at_frame),
                        ),
                        ParamSetValue::new(
                            p.prv_img_threshold_id,
                            ADDR,
                            ParamValue::Int32(f.threshold_id),
                        ),
                        ParamSetValue::new(
                            p.prv_img_integration_size,
                            ADDR,
                            ParamValue::Int32(f.integration_size),
                        ),
                    ])
                    .chain(rate.map(|r| {
                        ParamSetValue::new(p.prv_img_acq_rate, ADDR, ParamValue::Float64(r))
                    }))
                    .collect(),
            )
            .await;
            attach_attributes(ctx, &mut array, &f).await;
            publish_image(ctx, PRV_IMG_ADDR, array).await;
        }
        (StreamKind::Img, Frame::Image(f)) => {
            let Some(mut array) = image_array(
                ctx,
                &f,
                f.pixels.clone(),
                &adlog::err(
                    adlog::PROCESS_IMG_DATA_LINE,
                    "Failed to allocate NDArray or NDArray has no data pointer",
                ),
            ) else {
                return;
            };
            let p = ctx.p;
            // processImgDataLine counts every Img frame, NDArrayCallbacks or
            // not, before getAttributes (serval_stream.cpp:880-883).
            let counter = ctx
                .handle
                .read_int32(ctx.ad.base.array_counter, 0)
                .await
                .unwrap_or(0)
                + 1;
            let rate = frame_rate(
                ctx,
                f.frame_number,
                |r| &mut r.img,
                adlog::PROCESS_IMG_DATA_LINE,
                "Img ",
            );
            ctx.set(
                ADDR,
                geometry(ctx, &f)
                    .into_iter()
                    .chain([
                        ParamSetValue::new(
                            ctx.ad.base.array_counter,
                            ADDR,
                            ParamValue::Int32(counter),
                        ),
                        ParamSetValue::new(
                            p.img_frame_number,
                            ADDR,
                            ParamValue::Int32(f.frame_number),
                        ),
                        ParamSetValue::new(
                            p.img_threshold_id,
                            ADDR,
                            ParamValue::Int32(f.threshold_id),
                        ),
                        ParamSetValue::new(
                            p.img_time_at_frame,
                            ADDR,
                            ParamValue::Float64(f.time_at_frame),
                        ),
                    ])
                    .chain(
                        rate.map(|r| {
                            ParamSetValue::new(p.img_acq_rate, ADDR, ParamValue::Float64(r))
                        }),
                    )
                    .collect(),
            )
            .await;
            attach_attributes(ctx, &mut array, &f).await;

            // processImgFrame runs after getAttributes, on threshold 0 only and
            // only while ImgAccumulationEnable is set (serval_stream.cpp:948-982).
            let enabled = ctx
                .handle
                .read_int32(p.img_accumulation_enable, 0)
                .await
                .unwrap_or(0)
                != 0;
            if enabled && f.threshold_id != 1 {
                let update = ctx
                    .shared
                    .img
                    .lock()
                    .add_shaped(&f.pixels, &[f.width, f.height]);
                let (counts, frames) = {
                    let img = ctx.shared.img.lock();
                    (img.total_counts(), img.frame_count())
                };
                ctx.set(
                    ADDR,
                    vec![ParamSetValue::new(
                        p.img_image_frame,
                        ADDR,
                        ParamValue::Int32Array(
                            f.pixels
                                .iter()
                                .map(|&v| v as i32)
                                .collect::<Vec<i32>>()
                                .into(),
                        ),
                    )],
                )
                .await;
                ctx.set_int64(
                    p.img_total_counts,
                    ADDR,
                    i64::try_from(counts).unwrap_or(i64::MAX),
                )
                .await;

                publish_accumulation(
                    ctx,
                    &update,
                    p.img_image_data,
                    ADDR,
                    p.img_image_sum_n_frames,
                    ADDR,
                )
                .await;
                // processImgFrame hands the running sum to the plugins on address 2
                // and the window sum, when it updates, on 3; C does not check
                // NDArrayCallbacks here (serval_stream.cpp:1257-1343).
                let uid = i32::try_from(frames).unwrap_or(i32::MAX);
                publish_img_sum(ctx, 2, f.width, f.height, update.running, uid).await;
                if let Some(sum_n) = update.sum_n {
                    publish_img_sum(ctx, 3, f.width, f.height, sum_n, uid).await;
                }
            }
            publish_image(ctx, IMG_ADDR, array).await;
        }
        (StreamKind::PrvHst, Frame::Histogram(h)) => {
            // histogram_io.cpp:165-170, :425-433: a frame read before
            // accumulation was turned off is dropped whole.
            if !prv_hst_accumulating(ctx).await {
                return;
            }
            let p = ctx.p;
            // histogram_io.cpp:251-292 tracks the rate; :574 posts it every frame.
            let hst_rate = ctx
                .shared
                .rates
                .lock()
                .prv_hst
                .frame(h.frame_number, EpicsTimestamp::now().as_f64())
                .rate;
            let update = ctx.shared.hst.lock().add(&h.counts);
            let (frames, counts) = {
                let hst = ctx.shared.hst.lock();
                (hst.frame_count(), hst.total_counts())
            };

            ctx.set(
                ADDR,
                vec![
                    ParamSetValue::new(
                        p.prv_hst_time_at_frame,
                        ADDR,
                        ParamValue::Float64(h.time_at_frame),
                    ),
                    ParamSetValue::new(
                        p.prv_hst_frame_bin_size,
                        ADDR,
                        ParamValue::Int32(i32::try_from(h.counts.len()).unwrap_or(i32::MAX)),
                    ),
                    ParamSetValue::new(
                        p.prv_hst_frame_bin_width,
                        ADDR,
                        ParamValue::Int32(h.bin_width),
                    ),
                    ParamSetValue::new(
                        p.prv_hst_frame_bin_offset,
                        ADDR,
                        ParamValue::Int32(h.bin_offset),
                    ),
                    ParamSetValue::new(
                        p.prv_hst_frame_count,
                        ADDR,
                        ParamValue::Int32(i32::try_from(frames).unwrap_or(i32::MAX)),
                    ),
                    ParamSetValue::new(p.prv_hst_acq_rate, ADDR, ParamValue::Float64(hst_rate)),
                    ParamSetValue::new(
                        p.prv_hst_histogram_frame,
                        ADDR,
                        ParamValue::Int32Array(
                            h.counts
                                .iter()
                                .map(|&v| v as i32)
                                .collect::<Vec<i32>>()
                                .into(),
                        ),
                    ),
                ],
            )
            .await;
            ctx.set(
                ADDR,
                vec![ParamSetValue::new(
                    p.prv_hst_histogram_time_ms,
                    ADDR,
                    ParamValue::Float64Array(h.time_axis_ms().into()),
                )],
            )
            .await;
            ctx.set_int64(
                p.prv_hst_total_counts,
                ADDR,
                i64::try_from(counts).unwrap_or(i64::MAX),
            )
            .await;

            publish_accumulation(
                ctx,
                &update,
                p.prv_hst_histogram_data,
                ADDR,
                p.prv_hst_histogram_sum_n_frames,
                ADDR,
            )
            .await;

            // histogram_io.cpp:720-830: every frame also goes to the plugins on
            // addresses 4-7, the window sum only on the frames it was updated.
            if ctx
                .handle
                .read_int32(ctx.ad.base.array_callbacks, 0)
                .await
                .unwrap_or(0)
                != 0
            {
                let time_ms = h.time_axis_ms();
                let arrays = HstArrays {
                    window: update.sum_n.map(|w| (w, update.frames_summed as u64)),
                    running: update.running,
                    frame_count: frames,
                    current: Some(h.counts),
                    time_ms,
                };
                publish_hst_arrays(ctx, arrays, SumOutput::Sum).await;
            }
        }
        // The decoder is created for the channel, so this cannot happen.
        _ => {}
    }
}

/// C's PrvImg/Img rate block (serval_stream.cpp:248-283, :899-936): the
/// rate to post, when a second has passed since the last post, after the
/// frame-loss line.
fn frame_rate(
    ctx: &Ctx,
    frame_number: i32,
    tracker: fn(&mut crate::rate::Rates) -> &mut crate::rate::RateTracker,
    func: &str,
    tag: &str,
) -> Option<f64> {
    let o =
        tracker(&mut ctx.shared.rates.lock()).frame(frame_number, EpicsTimestamp::now().as_f64());
    if let Some((expected, got)) = o.lost {
        ctx.trace(
            TraceMask::IO_DRIVER,
            &adlog::log(
                func,
                &format!(
                    "{tag}frame loss detected! Expected frame {expected}, got frame {got} (lost {} frames)",
                    got.wrapping_sub(expected)
                ),
            ),
        );
    }
    o.due.then_some(o.rate)
}

async fn publish_accumulation(
    ctx: &Arc<Ctx>,
    update: &Update,
    sum_param: usize,
    sum_addr: i32,
    sum_n_param: usize,
    sum_n_addr: i32,
) {
    ctx.post_int64_array(
        sum_param,
        sum_addr,
        update
            .running
            .iter()
            .map(|&v| i64::try_from(v).unwrap_or(i64::MAX))
            .collect(),
    );
    if let Some(sum_n) = &update.sum_n {
        ctx.post_int64_array(
            sum_n_param,
            sum_n_addr,
            sum_n
                .iter()
                .map(|&v| i64::try_from(v).unwrap_or(i64::MAX))
                .collect(),
        );
    }
}

/// What the PrvHst NDArrays on addresses 4-7 are built from, copied out of
/// the accumulator under its lock (C holds `prvHstMutex_` throughout).
struct HstArrays {
    running: Vec<u64>,
    frame_count: u64,
    /// The window sum and its frame count, when address 4 is published.
    window: Option<(Vec<u64>, u64)>,
    current: Option<Vec<u32>>,
    time_ms: Vec<f64>,
}

/// C `processedImgOutputType` / `processedHstOutputType`: 0 publishes sums
/// as NDUInt64, 1 per-frame averages as NDUInt32.
#[derive(Clone, Copy, PartialEq, Eq)]
enum SumOutput {
    Sum,
    Average,
}

impl SumOutput {
    /// C treats anything but 1 as 0 (ADTimePix.cpp:1156, serval_stream.cpp:1447).
    async fn read(ctx: &Ctx, reason: usize) -> Self {
        match ctx.handle.read_int32(reason, 0).await.unwrap_or(0) {
            1 => Self::Average,
            _ => Self::Sum,
        }
    }

    /// The `addNumericRangeAttributes` range and units of these arrays.
    fn range_units(self) -> (&'static str, &'static str) {
        match self {
            Self::Sum => ("uint64", "counts"),
            Self::Average => ("uint32", "counts/frame"),
        }
    }
}

/// Which accumulation an array is built from, for its range-state latch.
#[derive(Clone, Copy)]
enum Accum {
    Img,
    PrvHst,
}

impl Accum {
    fn get(self, shared: &Shared) -> &parking_lot::Mutex<crate::accum::Accumulator> {
        match self {
            Self::Img => &shared.img,
            Self::PrvHst => &shared.hst,
        }
    }

    fn range_params(self, p: &TimePixParams) -> (usize, usize) {
        match self {
            Self::Img => (p.img_range_saturated, p.img_range_status),
            Self::PrvHst => (p.prv_hst_range_saturated, p.prv_hst_range_status),
        }
    }

    fn saturated(self, shared: &Shared) -> bool {
        self.get(shared).lock().range() != RangeState::Ok
    }
}

/// C `addNumericRangeAttributes` (ADTimePix.cpp:1352).
fn add_range_attributes(array: &mut DriverArray, range: &str, units: &str, saturated: bool) {
    for (name, desc, value) in [
        (
            "ADTimePixNumericRange",
            "Published numeric range",
            NDAttrValue::String(range.into()),
        ),
        (
            "ADTimePixCountUnits",
            "Count value units",
            NDAttrValue::String(units.into()),
        ),
        (
            "ADTimePixRangeSaturated",
            "Range saturation occurred",
            NDAttrValue::Int32(i32::from(saturated)),
        ),
    ] {
        array.attributes.add(NDAttribute::new_static(
            name,
            desc,
            NDAttrSource::Driver,
            value,
        ));
    }
}

/// C's `PrvHstTimeBin0Ms`/`PrvHstTimeBinStepMs`/`PrvHstNumBins` trio.
fn add_time_attributes(array: &mut DriverArray, time_ms: &[f64]) {
    let t0 = time_ms.first().copied().unwrap_or(0.0);
    let step = if time_ms.len() > 1 {
        time_ms[1] - time_ms[0]
    } else {
        0.0
    };
    for (name, desc, value) in [
        (
            "PrvHstTimeBin0Ms",
            "First bin center (ms)",
            NDAttrValue::Float64(t0),
        ),
        (
            "PrvHstTimeBinStepMs",
            "Bin center spacing (ms)",
            NDAttrValue::Float64(step),
        ),
        (
            "PrvHstNumBins",
            "Number of bins",
            NDAttrValue::Int32(i32::try_from(time_ms.len()).unwrap_or(i32::MAX)),
        ),
    ] {
        array.attributes.add(NDAttribute::new_static(
            name,
            desc,
            NDAttrSource::Driver,
            value,
        ));
    }
}

/// A sum as published on address 4 or 5: the sum itself, or its per-frame
/// average clamped to u32 with any clamp latched as `OutputClamped`.
async fn sum_buffer(
    ctx: &Ctx,
    accum: Accum,
    sum: &[u64],
    frames: u64,
    output: SumOutput,
) -> NDDataBuffer {
    match output {
        SumOutput::Sum => NDDataBuffer::U64(sum.to_vec()),
        SumOutput::Average => {
            let mut clamped = false;
            let avg = sum
                .iter()
                .map(|&v| {
                    let (a, c) = clamp_average_to_u32(v, frames);
                    clamped |= c;
                    a
                })
                .collect();
            if clamped {
                let posted = accum
                    .get(&ctx.shared)
                    .lock()
                    .report_range(RangeState::OutputClamped);
                let (saturated, status) = accum.range_params(&ctx.p);
                ctx.post_range(saturated, status, posted).await;
            }
            NDDataBuffer::U32(avg)
        }
    }
}

/// The PrvHst NDArrays in C's order: running sum on 5, bin times on 7, the
/// current frame on 6 and the window sum on 4 (ADTimePix.cpp:1209-1305,
/// histogram_io.cpp:765-830). Every array carries one `time_stamp`, taken
/// before the first is built.
async fn publish_hst_arrays(ctx: &Ctx, a: HstArrays, output: SumOutput) {
    let bins = a.running.len();
    if bins == 0 {
        return;
    }
    let time_stamp = EpicsTimestamp::now().as_f64();
    let dims = || vec![NDDimension::new(bins)];
    let (range, units) = output.range_units();

    let data = sum_buffer(ctx, Accum::PrvHst, &a.running, a.frame_count.max(1), output).await;
    if let Ok(mut array) = ctx.pool.wrap(dims(), data) {
        let _ = ctx.pool.owner().get_attributes(&mut array).await;
        add_time_attributes(&mut array, &a.time_ms);
        let saturated = Accum::PrvHst.saturated(&ctx.shared);
        add_range_attributes(&mut array, range, units, saturated);
        ctx.publish_at(5, array, time_stamp).await;
    }

    if let Ok(mut array) = ctx.pool.wrap(dims(), NDDataBuffer::F64(a.time_ms.clone())) {
        let _ = ctx.pool.owner().get_attributes(&mut array).await;
        ctx.publish_at(7, array, time_stamp).await;
    }

    if let Some(current) = a.current.filter(|c| c.len() == bins)
        && let Ok(mut array) = ctx.pool.wrap(dims(), NDDataBuffer::U32(current))
    {
        let _ = ctx.pool.owner().get_attributes(&mut array).await;
        add_range_attributes(&mut array, "uint32", "counts", false);
        ctx.publish_at(6, array, time_stamp).await;
    }

    if let Some((window, frames)) = a.window.filter(|(w, _)| w.len() >= bins) {
        let data = sum_buffer(ctx, Accum::PrvHst, &window[..bins], frames.max(1), output).await;
        if let Ok(mut array) = ctx.pool.wrap(dims(), data) {
            let _ = ctx.pool.owner().get_attributes(&mut array).await;
            add_time_attributes(&mut array, &a.time_ms);
            let saturated = Accum::PrvHst.saturated(&ctx.shared);
            add_range_attributes(&mut array, range, units, saturated);
            ctx.publish_at(4, array, time_stamp).await;
        }
    }
}

/// C `pushProcessedHstToPlugins` (ADTimePix.cpp:1150): the accumulated
/// PrvHst state on addresses 4-7 on demand, as sums or per-frame averages.
/// The time axis is rebuilt from the last frame's bin offset and width.
async fn push_processed_hst(ctx: &Ctx) {
    let output = SumOutput::read(ctx, ctx.p.processed_hst_output_type).await;
    if ctx
        .handle
        .read_int32(ctx.ad.base.array_callbacks, 0)
        .await
        .unwrap_or(0)
        == 0
    {
        return;
    }
    let offset = ctx
        .handle
        .read_int32(ctx.p.prv_hst_frame_bin_offset, 0)
        .await
        .unwrap_or(0);
    let width = ctx
        .handle
        .read_int32(ctx.p.prv_hst_frame_bin_width, 0)
        .await
        .unwrap_or(0);
    let arrays = {
        let hst = ctx.shared.hst.lock();
        let buffered = hst.buffered_frames() as u64;
        HstArrays {
            running: hst.running().to_vec(),
            frame_count: hst.frame_count(),
            window: (buffered > 0).then(|| (hst.window_sum(), buffered)),
            current: hst.current_frame().map(<[u32]>::to_vec),
            time_ms: crate::stream::time_axis_ms(offset, width, hst.running().len()),
        }
    };
    publish_hst_arrays(ctx, arrays, output).await;
}

/// One of processImgFrame's NDUInt64 sums on address 2 or 3.
async fn publish_img_sum(
    ctx: &Ctx,
    addr: usize,
    width: usize,
    height: usize,
    sum: Vec<u64>,
    unique_id: i32,
) {
    let Ok(mut array) = ctx.pool.wrap(
        vec![NDDimension::new(width), NDDimension::new(height)],
        NDDataBuffer::U64(sum),
    ) else {
        return;
    };
    array.unique_id = unique_id;
    let _ = ctx.pool.owner().get_attributes(&mut array).await;
    let saturated = Accum::Img.saturated(&ctx.shared);
    add_range_attributes(&mut array, "uint64", "counts", saturated);
    ctx.publish_at(addr, array, EpicsTimestamp::now().as_f64())
        .await;
}

/// C `pushProcessedImgToPlugins` (serval_stream.cpp:1441): the accumulated
/// Img running sum on address 2 and window sum on 3, as sums or per-frame
/// averages.
async fn push_processed_img(ctx: &Ctx) {
    let output = SumOutput::read(ctx, ctx.p.processed_img_output_type).await;
    if ctx
        .handle
        .read_int32(ctx.ad.base.array_callbacks, 0)
        .await
        .unwrap_or(0)
        == 0
    {
        return;
    }
    let (running, frames, window, shape) = {
        let img = ctx.shared.img.lock();
        let buffered = img.buffered_frames() as u64;
        (
            img.running().to_vec(),
            img.frame_count(),
            (buffered > 0).then(|| (img.window_sum(), buffered)),
            img.shape().to_vec(),
        )
    };
    if running.is_empty() {
        return;
    }
    let dims = || shape.iter().map(|&n| NDDimension::new(n)).collect();
    let (range, units) = output.range_units();

    let data = sum_buffer(ctx, Accum::Img, &running, frames.max(1), output).await;
    if let Ok(mut array) = ctx.pool.wrap(dims(), data) {
        let _ = ctx.pool.owner().get_attributes(&mut array).await;
        let saturated = Accum::Img.saturated(&ctx.shared);
        add_range_attributes(&mut array, range, units, saturated);
        ctx.publish_at(2, array, EpicsTimestamp::now().as_f64())
            .await;
    }

    if let Some((window, frames)) = window.filter(|(w, _)| w.len() >= running.len()) {
        let data = sum_buffer(ctx, Accum::Img, &window[..running.len()], frames, output).await;
        if let Ok(mut array) = ctx.pool.wrap(dims(), data) {
            let _ = ctx.pool.owner().get_attributes(&mut array).await;
            let saturated = Accum::Img.saturated(&ctx.shared);
            add_range_attributes(&mut array, range, units, saturated);
            ctx.publish_at(3, array, EpicsTimestamp::now().as_f64())
                .await;
        }
    }
}

/// Push a preview frame down the plugin chain as an NDArray. C stamps it with
/// Serval's `frameNumber`, not NDArrayCounter (serval_stream.cpp:604, :1467).
/// The geometry both image streams post before getAttributes
/// (serval_stream.cpp:224-233, :862-875).
fn geometry(ctx: &Ctx, f: &crate::stream::ImageFrame) -> [ParamSetValue; 7] {
    use crate::stream::PixelFormat;
    use epics_rs::ad_core::color::NDColorMode;
    use epics_rs::ad_core::ndarray::NDDataType;

    let (data_type, bytes) = match f.format {
        PixelFormat::UInt8 => (NDDataType::UInt8, 1),
        PixelFormat::UInt16 => (NDDataType::UInt16, 2),
        PixelFormat::UInt32 => (NDDataType::UInt32, 4),
    };
    let width = i32::try_from(f.width).unwrap_or(i32::MAX);
    let height = i32::try_from(f.height).unwrap_or(i32::MAX);
    let size = i32::try_from(f.width * f.height * bytes).unwrap_or(i32::MAX);
    let ad = &ctx.ad;
    let int = |reason, v| ParamSetValue::new(reason, ADDR, ParamValue::Int32(v));
    [
        int(ad.size_x, width),
        int(ad.base.array_size_x, width),
        int(ad.size_y, height),
        int(ad.base.array_size_y, height),
        int(ad.base.data_type, data_type as u8 as i32),
        int(ad.base.color_mode, NDColorMode::Mono as i32),
        int(ad.base.array_size, size),
    ]
}

/// The frame's NDArray, allocated before anything is posted, as C does
/// whether or not NDArrayCallbacks is on (serval_stream.cpp:177-183,
/// :808-814).
fn image_array(
    ctx: &Ctx,
    f: &crate::stream::ImageFrame,
    pixels: Vec<u32>,
    refused: &str,
) -> Option<DriverArray> {
    use crate::stream::PixelFormat;
    // C allocates the NDArray in the header's storage type
    // (serval_stream.cpp:157-159, :793-795).
    let data = match f.format {
        PixelFormat::UInt8 => NDDataBuffer::U8(pixels.into_iter().map(|p| p as u8).collect()),
        PixelFormat::UInt16 => NDDataBuffer::U16(pixels.into_iter().map(|p| p as u16).collect()),
        PixelFormat::UInt32 => NDDataBuffer::U32(pixels),
    };
    let array = ctx.pool.wrap(
        vec![NDDimension::new(f.width), NDDimension::new(f.height)],
        data,
    );
    if array.is_err() {
        ctx.trace(TraceMask::ERROR, refused);
    }
    array.ok()
}

/// C hands the frame the getAttributes set plus ThresholdID, and no
/// ColorMode attribute (serval_stream.cpp:236-240, :291-296, :886-890,
/// :942-947); C ignores the getAttributes status.
async fn attach_attributes(ctx: &Ctx, array: &mut DriverArray, f: &crate::stream::ImageFrame) {
    array.unique_id = f.frame_number;
    array.timestamp = EpicsTimestamp::now();
    array.time_stamp = array.timestamp.as_f64();
    let _ = ctx.pool.owner().get_attributes(array).await;
    array.attributes.add(NDAttribute::new_static(
        "ThresholdID",
        "Serval jsonimage thresholdID",
        NDAttrSource::Driver,
        NDAttrValue::Int32(f.threshold_id),
    ));
}

async fn publish_image(ctx: &Ctx, addr: usize, array: DriverArray) {
    if ctx
        .handle
        .read_int32(ctx.ad.base.array_callbacks, 0)
        .await
        .unwrap_or(0)
        != 0
    {
        ArrayPublisher::new(ctx.outputs[addr].clone())
            .publish(array)
            .await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn chip_temperatures_merge_across_boards() {
        // Serval 4: Health is an array and each board lists only its own chips,
        // so board 1's first temperature is global chip 4.
        let d = json!({"Health": [
            {"ChipTemperatures": [30, 31, 32, 33]},
            {"ChipTemperatures": [40, 41, 42, 43]},
        ]});
        assert_eq!(chip_temperatures(&d), vec![30, 31, 32, 33, 40, 41, 42, 43]);

        // Serval 3: Health is one object with a flat array.
        let d = json!({"Health": {"ChipTemperatures": [30.4, 31.6]}});
        assert_eq!(chip_temperatures(&d), vec![30, 32]);

        assert!(chip_temperatures(&json!({})).is_empty());
    }

    #[test]
    fn rails_map_three_per_board() {
        let d = json!({"Health": [
            {"VDD": [1.5, 4.0e-4, 0.6], "AVDD": [1.4, 3.0e-4, 0.5]},
            {"VDD": [2.5, 5.0e-4, 0.7], "AVDD": [2.4, 6.0e-4, 0.8]},
        ]});
        let r = rail_voltages(&d);
        assert_eq!(r.len(), 6);
        assert_eq!(r[0], (1.5, 1.4));
        assert_eq!(r[2], (0.6, 0.5));
        assert_eq!(r[3], (2.5, 2.4));
        assert_eq!(r[5], (0.7, 0.8));

        // One board: the second board's addresses read 0 V.
        let d = json!({"Health": {"VDD": [1.5, 4.0e-4, 0.6], "AVDD": [1.4, 3.0e-4, 0.5]}});
        let r = rail_voltages(&d);
        assert_eq!(r[0], (1.5, 1.4));
        assert_eq!(r[3], (0.0, 0.0));
    }

    fn idle_ctx(name: &str) -> Ctx {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let http = Arc::new(ServalHttp::new("http://127.0.0.1:1"));
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let driver =
            crate::driver::TimePix3Driver::new(name, http.clone(), shared.clone(), 0, cmd_tx)
                .unwrap();
        let (p, ad) = (driver.p, driver.ad.params);
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        Ctx {
            handle,
            http,
            shared,
            p,
            ad,
            outputs: new_outputs(),
            pool: DriverPool::standalone(0),
        }
    }

    fn int_set(updates: &[ParamSetValue], reason: usize) -> Option<i32> {
        updates.iter().rev().find_map(|u| match u {
            ParamSetValue::Value {
                reason: r,
                value: ParamValue::Int32(v),
                ..
            } if *r == reason => Some(*v),
            _ => None,
        })
    }

    fn octet_set(updates: &[ParamSetValue], reason: usize) -> Option<String> {
        updates.iter().rev().find_map(|u| match u {
            ParamSetValue::Value {
                reason: r,
                value: ParamValue::Octet(v),
                ..
            } if *r == reason => Some(String::from_utf8_lossy(v).into_owned()),
            _ => None,
        })
    }

    /// A connected detector reports the acquisition phase, not "OK"
    /// (serval_http.cpp:251-275, acquisition_coordinator.cpp:143-160).
    #[test]
    fn a_connected_detector_reports_the_acquisition_phase() {
        let ctx = Arc::new(idle_ctx("TPX_CONN_PHASE"));
        let coordinator = &ctx.shared.acquisition;
        assert!(coordinator.begin_start());
        assert!(coordinator.note_resource(crate::coordinator::Resource::RemoteMeasurement));
        assert!(coordinator.note_resource(crate::coordinator::Resource::MonitorThread));
        assert!(coordinator.commit_start());
        let mut updates = Vec::new();
        connection_status(&ctx, true, true, &mut updates);
        assert_eq!(
            octet_set(&updates, ctx.ad.status_message).as_deref(),
            Some("Acquisition running")
        );
        assert_eq!(
            int_set(&updates, ctx.ad.status),
            Some(ADStatus::Acquire as i32)
        );

        coordinator.begin_stop(true);
        coordinator.finish_stop(true, true);
        let mut updates = Vec::new();
        connection_status(&ctx, true, true, &mut updates);
        assert_eq!(
            octet_set(&updates, ctx.ad.status_message).as_deref(),
            Some("Acquisition fault; retry acquisition")
        );
        assert_eq!(
            int_set(&updates, ctx.ad.status),
            Some(ADStatus::Error as i32)
        );
    }

    /// The family and its capabilities are published on every detector read;
    /// the Medipix3 defaults only when the family first appears or changes
    /// (ADTimePix.cpp:2107-2127, :2155-2175).
    #[test]
    fn the_family_defaults_are_applied_when_the_family_changes() {
        let ctx = idle_ctx("TPX_FAMILY");
        let p = ctx.p;
        let mpx3 = serde_json::json!({"Info": {"MpxType": 5, "Boards": [{"ChipboardId": "41"}]}});
        let mut u = Vec::new();
        update_detector_family(&ctx, &mpx3, &mut u);
        assert_eq!(int_set(&u, p.detector_family), Some(2));
        assert_eq!(int_set(&u, p.cap_img_thresholds), Some(1));
        assert_eq!(int_set(&u, p.cap_tdc), Some(0));
        assert_eq!(int_set(&u, p.pixel_depth), Some(12));
        assert_eq!(int_set(&u, p.i_delay3), Some(10));

        let mut u = Vec::new();
        update_detector_family(&ctx, &mpx3, &mut u);
        assert_eq!(int_set(&u, p.detector_family), Some(2));
        assert_eq!(int_set(&u, p.pixel_depth), None);

        let tpx3 = serde_json::json!({"Info": {"ChipType": "tpx3"}});
        let mut u = Vec::new();
        update_detector_family(&ctx, &tpx3, &mut u);
        assert_eq!(int_set(&u, p.detector_family), Some(1));
        assert_eq!(int_set(&u, p.cap_tdc), Some(1));
        assert_eq!(int_set(&u, p.pixel_depth), None);
        assert_eq!(ctx.shared.family(), DetectorFamily::Tpx3);
    }

    #[tokio::test]
    async fn preview_sums_reach_io_intr_subscribers() {
        use epics_rs::asyn::interrupt::InterruptFilter;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let http = Arc::new(ServalHttp::new("http://127.0.0.1:1"));
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        // The port's interruptAccept is never opened: C's doCallbacksArray
        // does not consult it.
        let driver =
            crate::driver::TimePix3Driver::new("TPXT", http.clone(), shared.clone(), 0, cmd_tx)
                .unwrap();
        let (p, ad) = (driver.p, driver.ad.params);
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port("TPXT").unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = runtime.port_handle().clone();
        let ctx = Ctx {
            handle: handle.clone(),
            http,
            shared,
            p,
            ad,
            outputs: new_outputs(),
            pool: DriverPool::standalone(0),
        };

        let (_sub, mut rx) = handle.interrupts().register_interrupt_user(
            InterruptFilter {
                reason: Some(p.prv_hst_histogram_data),
                ..Default::default()
            },
            1024,
        );
        ctx.post_int64_array(p.prv_hst_histogram_data, 0, vec![3, 1, 4]);
        let v = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .expect("no callback")
            .unwrap();
        let ParamValue::Int64Array(a) = v.value else {
            panic!("expected an Int64Array, got {:?}", v.value);
        };
        assert_eq!(&a[..], [3, 1, 4]);
        assert_eq!(v.iface, Some(InterfaceType::Int64Array));
    }

    #[test]
    fn adjust_reports_the_shape() {
        assert_eq!(adjust_code(&json!({})), -1);
        assert_eq!(adjust_code(&json!({"Adjust": null})), -1);
        assert_eq!(adjust_code(&json!({"Adjust": [1, 2]})), -2);
        assert_eq!(adjust_code(&json!({"Adjust": 7})), -3);
    }

    #[tokio::test]
    async fn an_image_carries_the_serval_frame_number() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let http = Arc::new(ServalHttp::new("http://127.0.0.1:1"));
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let driver =
            crate::driver::TimePix3Driver::new("TPXUID", http.clone(), shared.clone(), 0, cmd_tx)
                .unwrap();
        let (p, ad) = (driver.p, driver.ad.params);
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port("TPXUID").unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let outputs = new_outputs();
        let (tx, mut rx) = ndarray_channel("TPXUID_SINK", 4);
        outputs[PRV_IMG_ADDR].lock().add(tx);
        let (tx, mut rx_img) = ndarray_channel("TPXUID_IMG_SINK", 4);
        outputs[IMG_ADDR].lock().add(tx);
        let ctx = Arc::new(Ctx {
            handle: runtime.port_handle().clone(),
            http,
            shared,
            p,
            ad,
            outputs,
            pool: DriverPool::standalone(0),
        });
        ctx.set_int(ad.base.array_callbacks, 0, 1).await;

        for (kind, frame_number) in [(StreamKind::PrvImg, 41), (StreamKind::Img, 97)] {
            let frame = Frame::Image(crate::stream::ImageFrame {
                width: 2,
                height: 1,
                format: crate::stream::PixelFormat::UInt16,
                frame_number,
                time_at_frame: 0.0,
                threshold_id: 0,
                integration_size: 0,
                pixels: vec![1, 2],
            });
            handle_frame(&ctx, kind, frame).await;
            let rx = match kind {
                StreamKind::Img => &mut rx_img,
                _ => &mut rx,
            };
            let array = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .expect("no array")
                .unwrap();
            assert_eq!(array.unique_id, frame_number);
        }
        std::mem::forget(runtime);
    }

    /// C allocates both image streams' NDArrays in the header's storage type
    /// (serval_stream.cpp:157-159, :793-795).
    #[tokio::test]
    async fn an_image_is_published_in_its_header_storage_type() {
        use crate::stream::PixelFormat;
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let http = Arc::new(ServalHttp::new("http://127.0.0.1:1"));
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let driver =
            crate::driver::TimePix3Driver::new("TPXDTYPE", http.clone(), shared.clone(), 0, cmd_tx)
                .unwrap();
        let (p, ad) = (driver.p, driver.ad.params);
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port("TPXDTYPE").unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let outputs = new_outputs();
        let (tx, mut rx) = ndarray_channel("TPXDTYPE_SINK", 4);
        outputs[PRV_IMG_ADDR].lock().add(tx);
        let (tx, mut rx_img) = ndarray_channel("TPXDTYPE_IMG_SINK", 4);
        outputs[IMG_ADDR].lock().add(tx);
        let ctx = Arc::new(Ctx {
            handle: runtime.port_handle().clone(),
            http,
            shared,
            p,
            ad,
            outputs,
            pool: DriverPool::standalone(0),
        });
        ctx.set_int(ad.base.array_callbacks, 0, 1).await;

        for kind in [StreamKind::PrvImg, StreamKind::Img] {
            for format in [PixelFormat::UInt8, PixelFormat::UInt16, PixelFormat::UInt32] {
                let frame = Frame::Image(crate::stream::ImageFrame {
                    width: 2,
                    height: 1,
                    format,
                    frame_number: 1,
                    time_at_frame: 0.0,
                    threshold_id: 0,
                    integration_size: 0,
                    pixels: vec![1, 200],
                });
                handle_frame(&ctx, kind, frame).await;
                let rx = match kind {
                    StreamKind::Img => &mut rx_img,
                    _ => &mut rx,
                };
                let array = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                    .await
                    .expect("no array")
                    .unwrap();
                let ok = match (format, &array.data) {
                    (PixelFormat::UInt8, NDDataBuffer::U8(v)) => *v == [1, 200],
                    (PixelFormat::UInt16, NDDataBuffer::U16(v)) => *v == [1, 200],
                    (PixelFormat::UInt32, NDDataBuffer::U32(v)) => *v == [1, 200],
                    _ => false,
                };
                assert!(ok, "{kind:?} {format:?}: {:?}", array.data.data_type());
            }
        }
        std::mem::forget(runtime);
    }

    /// C runs `getAttributes` on both image streams' frames and adds no
    /// ColorMode attribute (serval_stream.cpp:292, :943).
    #[tokio::test]
    async fn an_image_carries_the_attributes_file_set() {
        use epics_rs::ad_core::attributes::NDAttrValue;
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::user::AsynUser;

        let http = Arc::new(ServalHttp::new("http://127.0.0.1:1"));
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut driver =
            crate::driver::TimePix3Driver::new("TPXATTR", http.clone(), shared.clone(), 0, cmd_tx)
                .unwrap();
        let (p, ad, pool) = (driver.p, driver.ad.params, driver.ad.pool.clone());
        // ADTimePix::writeOctet never chains to ADDriver::writeOctet, so the
        // set is loaded through the base the way asynNDArrayDriver would.
        driver
            .ad
            .write_octet(
                &AsynUser::new(ad.base.attributes_file),
                br#"<Attributes><Attribute name="Exposure" type="PARAM" source="ACQ_TIME" datatype="DOUBLE"/></Attributes>"#,
            )
            .unwrap();
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port("TPXATTR").unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let outputs = new_outputs();
        let (tx, mut rx) = ndarray_channel("TPXATTR_SINK", 4);
        outputs[PRV_IMG_ADDR].lock().add(tx);
        let (tx, mut rx_img) = ndarray_channel("TPXATTR_IMG_SINK", 4);
        outputs[IMG_ADDR].lock().add(tx);
        let ctx = Arc::new(Ctx {
            handle: runtime.port_handle().clone(),
            http,
            shared,
            p,
            ad,
            outputs,
            pool,
        });
        ctx.set_int(ad.base.array_callbacks, 0, 1).await;
        ctx.set(
            0,
            vec![ParamSetValue::new(
                ad.acquire_time,
                0,
                ParamValue::Float64(0.25),
            )],
        )
        .await;

        for kind in [StreamKind::PrvImg, StreamKind::Img] {
            let frame = Frame::Image(crate::stream::ImageFrame {
                width: 2,
                height: 1,
                format: crate::stream::PixelFormat::UInt16,
                frame_number: 1,
                time_at_frame: 0.0,
                threshold_id: 0,
                integration_size: 0,
                pixels: vec![1, 2],
            });
            handle_frame(&ctx, kind, frame).await;
            let rx = match kind {
                StreamKind::Img => &mut rx_img,
                _ => &mut rx,
            };
            let array = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .expect("no array")
                .unwrap();
            assert_eq!(
                array.attributes.get("Exposure").map(|a| &a.value),
                Some(&NDAttrValue::Float64(0.25))
            );
            // C adds ThresholdID after getAttributes
            // (serval_stream.cpp:292-295, :943-946).
            assert_eq!(
                array.attributes.get("ThresholdID").map(|a| &a.value),
                Some(&NDAttrValue::Int32(0))
            );
            assert_eq!(array.attributes.len(), 2);
        }

        // The header's threshold reaches the attribute and the stream's
        // params (serval_stream.cpp:245-246, :894).
        for kind in [StreamKind::PrvImg, StreamKind::Img] {
            let frame = Frame::Image(crate::stream::ImageFrame {
                width: 2,
                height: 1,
                format: crate::stream::PixelFormat::UInt16,
                frame_number: 2,
                time_at_frame: 0.0,
                threshold_id: 1,
                integration_size: 7,
                pixels: vec![1, 2],
            });
            handle_frame(&ctx, kind, frame).await;
            let rx = match kind {
                StreamKind::Img => &mut rx_img,
                _ => &mut rx,
            };
            let array = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .expect("no array")
                .unwrap();
            assert_eq!(
                array.attributes.get("ThresholdID").map(|a| &a.value),
                Some(&NDAttrValue::Int32(1))
            );
        }
        let int = |reason| ctx.handle.read_int32(reason, 0);
        assert_eq!(int(p.prv_img_threshold_id).await.unwrap(), 1);
        assert_eq!(int(p.prv_img_integration_size).await.unwrap(), 7);
        assert_eq!(int(p.img_threshold_id).await.unwrap(), 1);
        std::mem::forget(runtime);
    }

    /// C posts the frame geometry, and on Img the bumped NDArrayCounter, before
    /// getAttributes; PrvImg leaves the counter alone and Img counts with
    /// NDArrayCallbacks off (serval_stream.cpp:224-233, :862-883).
    #[tokio::test]
    async fn the_frame_geometry_and_counter_precede_get_attributes() {
        use epics_rs::ad_core::attributes::NDAttrValue;
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::user::AsynUser;

        let http = Arc::new(ServalHttp::new("http://127.0.0.1:1"));
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut driver =
            crate::driver::TimePix3Driver::new("TPXGEOM", http.clone(), shared.clone(), 0, cmd_tx)
                .unwrap();
        let (p, ad, pool) = (driver.p, driver.ad.params, driver.ad.pool.clone());
        driver
            .ad
            .write_octet(
                &AsynUser::new(ad.base.attributes_file),
                br#"<Attributes>
                <Attribute name="SizeX" type="PARAM" source="ARRAY_SIZE_X" datatype="INT"/>
                <Attribute name="DataType" type="PARAM" source="DATA_TYPE" datatype="INT"/>
                <Attribute name="Bytes" type="PARAM" source="ARRAY_SIZE" datatype="INT"/>
                <Attribute name="Counter" type="PARAM" source="ARRAY_COUNTER" datatype="INT"/>
                </Attributes>"#,
            )
            .unwrap();
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port("TPXGEOM").unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let outputs = new_outputs();
        let (tx, mut rx) = ndarray_channel("TPXGEOM_SINK", 4);
        outputs[PRV_IMG_ADDR].lock().add(tx);
        let (tx, mut rx_img) = ndarray_channel("TPXGEOM_IMG_SINK", 4);
        outputs[IMG_ADDR].lock().add(tx);
        let ctx = Arc::new(Ctx {
            handle: runtime.port_handle().clone(),
            http,
            shared,
            p,
            ad,
            outputs,
            pool,
        });
        let frame = |format| {
            Frame::Image(crate::stream::ImageFrame {
                width: 3,
                height: 1,
                format,
                frame_number: 1,
                time_at_frame: 0.0,
                threshold_id: 0,
                integration_size: 0,
                pixels: vec![1, 2, 3],
            })
        };
        let attr = |a: &epics_rs::ad_core::ndarray::NDArray, name| {
            a.attributes.get(name).map(|a| a.value.clone())
        };
        let int = |reason| ctx.handle.read_int32(reason, 0);

        // Counted with NDArrayCallbacks off.
        ctx.set_int(ad.base.array_callbacks, 0, 0).await;
        handle_frame(
            &ctx,
            StreamKind::Img,
            frame(crate::stream::PixelFormat::UInt16),
        )
        .await;
        assert_eq!(int(ad.base.array_counter).await.unwrap(), 1);

        ctx.set_int(ad.base.array_callbacks, 0, 1).await;
        handle_frame(
            &ctx,
            StreamKind::PrvImg,
            frame(crate::stream::PixelFormat::UInt32),
        )
        .await;
        let array = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .expect("no array")
            .unwrap();
        assert_eq!(attr(&array, "SizeX"), Some(NDAttrValue::Int32(3)));
        assert_eq!(attr(&array, "DataType"), Some(NDAttrValue::Int32(5)));
        assert_eq!(attr(&array, "Bytes"), Some(NDAttrValue::Int32(12)));
        assert_eq!(attr(&array, "Counter"), Some(NDAttrValue::Int32(1)));

        handle_frame(
            &ctx,
            StreamKind::Img,
            frame(crate::stream::PixelFormat::UInt16),
        )
        .await;
        let array = tokio::time::timeout(Duration::from_secs(5), rx_img.recv())
            .await
            .expect("no array")
            .unwrap();
        assert_eq!(attr(&array, "DataType"), Some(NDAttrValue::Int32(3)));
        assert_eq!(attr(&array, "Bytes"), Some(NDAttrValue::Int32(6)));
        assert_eq!(attr(&array, "Counter"), Some(NDAttrValue::Int32(2)));
        assert_eq!(int(ad.size_x).await.unwrap(), 3);
        assert_eq!(int(ad.base.color_mode).await.unwrap(), 0);
        std::mem::forget(runtime);
    }

    /// processImgFrame accumulates after getAttributes, so an Img frame's
    /// TPX3_IMG_TOTAL_COUNTS attribute is the total before that frame
    /// (serval_stream.cpp:942-982).
    #[tokio::test]
    async fn an_img_frame_is_accumulated_after_get_attributes() {
        use epics_rs::ad_core::attributes::NDAttrValue;
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::user::AsynUser;

        let http = Arc::new(ServalHttp::new("http://127.0.0.1:1"));
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut driver = crate::driver::TimePix3Driver::new(
            "TPXACCORD",
            http.clone(),
            shared.clone(),
            0,
            cmd_tx,
        )
        .unwrap();
        let (p, ad, pool) = (driver.p, driver.ad.params, driver.ad.pool.clone());
        driver
            .ad
            .write_octet(
                &AsynUser::new(ad.base.attributes_file),
                br#"<Attributes><Attribute name="Total" type="PARAM" source="TPX3_IMG_TOTAL_COUNTS" datatype="INT64"/></Attributes>"#,
            )
            .unwrap();
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port("TPXACCORD").unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let outputs = new_outputs();
        let (tx, mut rx) = ndarray_channel("TPXACCORD_SINK", 4);
        outputs[IMG_ADDR].lock().add(tx);
        let ctx = Arc::new(Ctx {
            handle: runtime.port_handle().clone(),
            http,
            shared,
            p,
            ad,
            outputs,
            pool,
        });
        ctx.set_int(ad.base.array_callbacks, 0, 1).await;
        ctx.set_int(p.img_accumulation_enable, 0, 1).await;
        for (frame_number, before) in [(1, 0), (2, 3)] {
            let frame = Frame::Image(crate::stream::ImageFrame {
                width: 2,
                height: 1,
                format: crate::stream::PixelFormat::UInt16,
                frame_number,
                time_at_frame: 0.0,
                threshold_id: 0,
                integration_size: 0,
                pixels: vec![1, 2],
            });
            handle_frame(&ctx, StreamKind::Img, frame).await;
            let array = tokio::time::timeout(Duration::from_secs(5), rx.recv())
                .await
                .expect("no array")
                .unwrap();
            assert_eq!(
                array.attributes.get("Total").map(|a| &a.value),
                Some(&NDAttrValue::Int64(before))
            );
        }
        std::mem::forget(runtime);
    }

    /// processImgDataLine accumulates only threshold-0 frames, and only while
    /// ImgAccumulationEnable is set (serval_stream.cpp:949-951).
    #[tokio::test]
    async fn img_accumulation_takes_enabled_threshold_0_frames_only() {
        let ctx = Arc::new(idle_ctx("TPX_IMG_GATE"));
        let frame = |threshold_id| {
            Frame::Image(crate::stream::ImageFrame {
                width: 2,
                height: 1,
                format: crate::stream::PixelFormat::UInt16,
                frame_number: 1,
                time_at_frame: 0.0,
                threshold_id,
                integration_size: 0,
                pixels: vec![1, 2],
            })
        };
        let total = || ctx.shared.img.lock().total_counts();
        handle_frame(&ctx, StreamKind::Img, frame(1)).await;
        assert_eq!(total(), 0);
        handle_frame(&ctx, StreamKind::Img, frame(0)).await;
        assert_eq!(total(), 3);
        ctx.set_int(ctx.p.img_accumulation_enable, 0, 0).await;
        handle_frame(&ctx, StreamKind::Img, frame(0)).await;
        assert_eq!(total(), 3);
    }

    /// C `pushProcessedHstToPlugins` hands the accumulated PrvHst state to the
    /// plugins on addresses 4-7: sums as NDUInt64 for output type 0, averages
    /// as NDUInt32 for type 1 (ADTimePix.cpp:1150-1305).
    #[tokio::test]
    async fn processed_hst_push_arrives_on_addresses_4_to_7() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;

        let ctx = Arc::new(idle_ctx("TPX_HST_PUSH"));
        let mut sinks = Vec::new();
        for addr in 4..=7 {
            let (tx, rx) = ndarray_channel(&format!("TPX_HST_PUSH_{addr}"), 4);
            ctx.outputs[addr].lock().add(tx);
            sinks.push(rx);
        }
        ctx.set_int(ctx.ad.base.array_callbacks, 0, 1).await;
        ctx.set_int(ctx.p.prv_hst_frame_bin_offset, 0, 0).await;
        ctx.set_int(ctx.p.prv_hst_frame_bin_width, 0, 2).await;
        {
            let mut hst = ctx.shared.hst.lock();
            hst.add(&[1, u32::MAX]);
            hst.add(&[3, u32::MAX]);
        }

        let mut recv = async |i: usize| {
            tokio::time::timeout(Duration::from_secs(5), sinks[i].recv())
                .await
                .expect("no array")
                .unwrap()
        };

        push_processed_hst(&ctx).await;
        let sum4 = recv(0).await;
        let sum5 = recv(1).await;
        let frame6 = recv(2).await;
        let time7 = recv(3).await;
        let both = 2 * u64::from(u32::MAX);
        assert!(matches!(&sum4.data, NDDataBuffer::U64(v) if *v == vec![4, both]));
        assert!(matches!(&sum5.data, NDDataBuffer::U64(v) if *v == vec![4, both]));
        assert!(matches!(&frame6.data, NDDataBuffer::U32(v) if *v == vec![3, u32::MAX]));
        let step = 2.0 * crate::stream::TDC_CLOCK_PERIOD_SEC * 1e3;
        assert!(matches!(&time7.data, NDDataBuffer::F64(v) if *v == vec![0.0, step]));
        assert_eq!(
            sum5.attributes
                .get("ADTimePixNumericRange")
                .map(|a| &a.value),
            Some(&NDAttrValue::String("uint64".into()))
        );
        assert_eq!(
            sum5.attributes.get("PrvHstNumBins").map(|a| &a.value),
            Some(&NDAttrValue::Int32(2))
        );

        ctx.set_int(ctx.p.processed_hst_output_type, 0, 1).await;
        push_processed_hst(&ctx).await;
        let avg4 = recv(0).await;
        let avg5 = recv(1).await;
        assert!(matches!(&avg4.data, NDDataBuffer::U32(v) if *v == vec![2, u32::MAX]));
        assert!(matches!(&avg5.data, NDDataBuffer::U32(v) if *v == vec![2, u32::MAX]));
        assert_eq!(
            avg5.attributes.get("ADTimePixCountUnits").map(|a| &a.value),
            Some(&NDAttrValue::String("counts/frame".into()))
        );
    }

    /// Every stream posts its acquisition rate from the frame numbers it
    /// receives (serval_stream.cpp:248-283, :899-936, histogram_io.cpp:574).
    #[tokio::test]
    async fn each_stream_posts_its_acquisition_rate() {
        let ctx = Arc::new(idle_ctx("TPX_ACQ_RATE"));
        let p = ctx.p;
        for frame_number in [1, 2] {
            for kind in [StreamKind::PrvImg, StreamKind::Img] {
                let frame = Frame::Image(crate::stream::ImageFrame {
                    width: 2,
                    height: 1,
                    format: crate::stream::PixelFormat::UInt16,
                    frame_number,
                    time_at_frame: 0.0,
                    threshold_id: 0,
                    integration_size: 0,
                    pixels: vec![1, 2],
                });
                handle_frame(&ctx, kind, frame).await;
            }
            let frame = Frame::Histogram(crate::stream::HistogramFrame {
                bin_width: 2,
                bin_offset: 1,
                frame_number,
                time_at_frame: 0.0,
                counts: vec![1, 2],
            });
            handle_frame(&ctx, StreamKind::PrvHst, frame).await;
            std::thread::sleep(Duration::from_millis(5));
        }
        for reason in [p.prv_img_acq_rate, p.img_acq_rate, p.prv_hst_acq_rate] {
            let rate = ctx.handle.read_float64(reason, 0).await.unwrap();
            assert!(rate > 0.0, "{reason}: {rate}");
        }
        // A stop clears the trackers, so the next first frame starts over.
        ctx.shared.rates.lock().reset();
        let o = ctx.shared.rates.lock().prv_hst.frame(9, 1.0);
        assert_eq!(o.rate, 0.0);
    }

    /// C neither reads the histogram socket nor handles a frame while
    /// PrvHstAccumulationEnable is 0 (histogram_io.cpp:165-170, :1018-1023):
    /// a frame sent meanwhile waits in the socket and is taken on re-enable.
    #[tokio::test(flavor = "multi_thread")]
    async fn prv_hst_accumulation_off_leaves_the_socket_unread() {
        use std::io::Write;

        let ctx = Arc::new(idle_ctx("TPX_HST_GATE"));
        let enable = ctx.p.prv_hst_accumulation_enable;
        ctx.set_int(enable, 0, 0).await;
        assert!(ctx.shared.acquisition.begin_start());
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let _ = client.set_read_timeout(Some(Duration::from_millis(50)));
        let (mut peer, _) = listener.accept().unwrap();
        let reader = tokio::spawn({
            let ctx = ctx.clone();
            async move { read_stream(&ctx, StreamKind::PrvHst, client).await }
        });

        let mut bytes = br#"{"binSize":2,"binWidth":1,"binOffset":0,"frameNumber":1}"#.to_vec();
        bytes.push(b'\n');
        bytes.extend_from_slice(&[0, 0, 0, 1, 0, 0, 0, 2]);
        peer.write_all(&bytes).unwrap();
        tokio::time::sleep(Duration::from_millis(400)).await;
        assert_eq!(ctx.shared.hst.lock().frame_count(), 0);

        ctx.set_int(enable, 0, 1).await;
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while ctx.shared.hst.lock().frame_count() == 0 {
            assert!(std::time::Instant::now() < deadline, "frame never taken");
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        drop(peer);
        reader.await.unwrap();
    }

    /// C `processPrvHstFrame` hands every PrvHst frame to the plugins on
    /// addresses 4-7 as well (histogram_io.cpp:720-830).
    #[tokio::test]
    async fn a_prv_hst_frame_arrives_on_addresses_4_to_7() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;

        let ctx = Arc::new(idle_ctx("TPX_HST_FRAME"));
        let mut sinks = Vec::new();
        for addr in 4..=7 {
            let (tx, rx) = ndarray_channel(&format!("TPX_HST_FRAME_{addr}"), 4);
            ctx.outputs[addr].lock().add(tx);
            sinks.push(rx);
        }
        ctx.set_int(ctx.ad.base.array_callbacks, 0, 1).await;
        for counts in [vec![1, 2], vec![3, 4]] {
            let frame = Frame::Histogram(crate::stream::HistogramFrame {
                bin_width: 2,
                bin_offset: 1,
                frame_number: 1,
                time_at_frame: 0.0,
                counts,
            });
            handle_frame(&ctx, StreamKind::PrvHst, frame).await;
        }

        let mut got = Vec::new();
        for rx in &mut sinks {
            let mut last = None;
            for _ in 0..2 {
                last = Some(
                    tokio::time::timeout(Duration::from_secs(5), rx.recv())
                        .await
                        .expect("no array")
                        .unwrap(),
                );
            }
            got.push(last.unwrap());
        }
        assert!(matches!(&got[0].data, NDDataBuffer::U64(v) if *v == vec![4, 6]));
        assert!(matches!(&got[1].data, NDDataBuffer::U64(v) if *v == vec![4, 6]));
        assert!(matches!(&got[2].data, NDDataBuffer::U32(v) if *v == vec![3, 4]));
        let t = crate::stream::TDC_CLOCK_PERIOD_SEC * 1e3;
        assert!(matches!(&got[3].data, NDDataBuffer::F64(v) if *v == vec![t, 3.0 * t]));
    }

    /// C `processImgFrame` hands every Img frame's running sum to the plugins
    /// on address 2 and the window sum on 3, without checking NDArrayCallbacks
    /// (serval_stream.cpp:1257-1343); `pushProcessedImgToPlugins` sends the
    /// same sums, or their per-frame averages, on demand (:1441-1546).
    #[tokio::test]
    async fn img_sums_arrive_on_addresses_2_and_3() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;

        let ctx = Arc::new(idle_ctx("TPX_IMG_SUMS"));
        let mut sinks = Vec::new();
        for addr in 2..=3 {
            let (tx, rx) = ndarray_channel(&format!("TPX_IMG_SUMS_{addr}"), 4);
            ctx.outputs[addr].lock().add(tx);
            sinks.push(rx);
        }
        for pixels in [vec![1, 2], vec![3, 4]] {
            let frame = Frame::Image(crate::stream::ImageFrame {
                width: 2,
                height: 1,
                format: crate::stream::PixelFormat::UInt16,
                frame_number: 7,
                time_at_frame: 0.0,
                threshold_id: 0,
                integration_size: 0,
                pixels,
            });
            handle_frame(&ctx, StreamKind::Img, frame).await;
        }
        let mut recv = async |i: usize| {
            tokio::time::timeout(Duration::from_secs(5), sinks[i].recv())
                .await
                .expect("no array")
                .unwrap()
        };
        for i in 0..2 {
            let (sum, sum_n) = (recv(0).await, recv(1).await);
            assert_eq!(sum.unique_id, i + 1);
            assert_eq!(sum_n.unique_id, i + 1);
        }
        // The second frame's pair was the last one sent.
        ctx.set_int(ctx.ad.base.array_callbacks, 0, 1).await;
        push_processed_img(&ctx).await;
        let (sum, sum_n) = (recv(0).await, recv(1).await);
        assert!(matches!(&sum.data, NDDataBuffer::U64(v) if *v == vec![4, 6]));
        assert!(matches!(&sum_n.data, NDDataBuffer::U64(v) if *v == vec![4, 6]));
        assert_eq!(sum.dims.len(), 2);

        ctx.set_int(ctx.p.processed_img_output_type, 0, 1).await;
        push_processed_img(&ctx).await;
        let (avg, avg_n) = (recv(0).await, recv(1).await);
        assert!(matches!(&avg.data, NDDataBuffer::U32(v) if *v == vec![2, 3]));
        assert!(matches!(&avg_n.data, NDDataBuffer::U32(v) if *v == vec![2, 3]));
        assert_eq!(
            avg.attributes.get("ADTimePixCountUnits").map(|a| &a.value),
            Some(&NDAttrValue::String("counts/frame".into()))
        );
    }
}
