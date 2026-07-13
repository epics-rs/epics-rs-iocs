//! The background threads: the connection poll, the acquisition poll and the
//! three TCP stream workers (port of `connectionPollThread`,
//! `timePixCallback` and the `img`/`prvImg`/`prvHst` worker threads).

use std::io::Read;
use std::net::TcpStream;
use std::sync::Arc;
use std::time::Duration;

use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::params::ad_driver::ADDriverParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{ParamSetValue, RequestOp};
use epics_rs::asyn::user::AsynUser;
use serde_json::Value;

use crate::accum::Update;
use crate::http::{ServalHttp, TIMEOUT_POLL};
use crate::params::TimePixParams;
use crate::serval;
use crate::state::{Command, Shared};
use crate::stream::{Channel, Frame, FrameDecoder, parse_tcp_path};

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

/// Every parameter of the streaming channels lives at address 0: C's
/// `doCallbacks*Array(..., 0)` (serval_stream.cpp:975, histogram_io.cpp:541)
/// and its two-argument `setIntegerParam`/`setDoubleParam` calls all target
/// address 0, and every record in the templates binds `ADDR=0`. The port's
/// eight addresses carry the per-chip DACs and the eight *NDArray* streams,
/// not these parameters.
const ADDR: i32 = 0;

pub struct Ctx {
    pub handle: PortHandle,
    pub http: Arc<ServalHttp>,
    pub shared: Arc<Shared>,
    pub p: TimePixParams,
    pub ad: ADDriverParams,
    pub output: Arc<parking_lot::Mutex<NDArrayOutput>>,
}

impl Ctx {
    async fn set(&self, addr: i32, updates: Vec<ParamSetValue>) {
        if let Err(e) = self.handle.set_params_and_notify(addr, updates).await {
            log::error!("timepix3: parameter update failed: {e}");
        }
    }

    async fn set_int(&self, reason: usize, addr: i32, value: i32) {
        self.set(
            addr,
            vec![ParamSetValue::new(reason, addr, ParamValue::Int32(value))],
        )
        .await;
    }

    /// `PortHandle` has no `Int64` parameter setter, so the request goes in
    /// directly; the driver's default `write_int64` stores it and fires the
    /// callbacks, which is exactly C's `setInteger64Param` +
    /// `callParamCallbacks`.
    async fn set_int64(&self, reason: usize, addr: i32, value: i64) {
        let user = AsynUser::new(reason).with_addr(addr);
        if let Err(e) = self
            .handle
            .submit_async(RequestOp::Int64Write { value }, user)
            .await
        {
            log::error!("timepix3: int64 parameter update failed: {e}");
        }
    }

    async fn set_int64_array(&self, reason: usize, addr: i32, data: Vec<i64>) {
        let user = AsynUser::new(reason).with_addr(addr);
        if let Err(e) = self
            .handle
            .submit_async(RequestOp::Int64ArrayWrite { data }, user)
            .await
        {
            log::error!("timepix3: int64 array update failed: {e}");
        }
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
                ctx.shared.set_acquiring(true);
                if acq_tx.try_send(()).is_err() {
                    log::error!("timepix3: the acquisition poll is not running");
                }
                continue;
            }
            Command::AcquisitionStopped => {
                ctx.shared.set_acquiring(false);
                continue;
            }
            Command::RefreshStatus => {
                refresh_detector(&ctx).await;
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
            refresh_detector(&ctx).await;
        }
        was_up = up;
    }
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
        ParamSetValue::new(p.det_type, 0, ParamValue::Octet(det_type.clone())),
        ParamSetValue::new(
            ctx.ad.status,
            0,
            ParamValue::Int32(if serval_ok && det_ok {
                if ctx.shared.acquiring() {
                    ADStatus::Acquire as i32
                } else {
                    ADStatus::Idle as i32
                }
            } else {
                ADStatus::Disconnected as i32
            }),
        ),
    ];
    if det_ok {
        updates.push(ParamSetValue::new(
            ctx.ad.base.model,
            0,
            ParamValue::Octet(det_type),
        ));
    }
    if let Ok(dashboard) = &reply {
        disk_space(ctx, dashboard, &mut updates);
    }
    ctx.set(0, updates).await;
    serval_ok && det_ok
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

/// C `getDetector` (serval_http.cpp:1162) + `getDashboard`'s int64 fields.
async fn refresh_detector(ctx: &Arc<Ctx>) {
    let p = ctx.p;

    if let Ok(dashboard) = ctx.http.get_json(serval::DASHBOARD, TIMEOUT_POLL)
        && let Some(disk) = dashboard.pointer("/Server/DiskSpace/0")
    {
        if let Some(v) = disk.get("FreeSpace").and_then(Value::as_i64) {
            ctx.set_int64(p.free_space, 0, v).await;
        }
        if let Some(v) = disk.get("LowerLimit").and_then(Value::as_i64) {
            ctx.set_int64(p.lower_limit, 0, v).await;
        }
    }

    let detector = match ctx.http.get_json(serval::DETECTOR, TIMEOUT_POLL) {
        Ok(d) => d,
        Err(e) => {
            log::warn!("timepix3: GET /detector: {e}");
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
                ParamValue::Octet(serval::json_to_string(v)),
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
                ParamValue::Octet(serval::json_to_string(c)),
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
            updates.push(ParamSetValue::new(reason, 0, ParamValue::Octet(value)));
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
            ParamValue::Octet(serval::json_to_string(&Value::Array(temps))),
        ));
    }
}

// ---------------------------------------------------------------------------
// Acquisition poll (C `timePixCallback`, acquire.cpp:416)
// ---------------------------------------------------------------------------

async fn acquisition_poll(ctx: Arc<Ctx>, mut start_rx: rt::CommandReceiver<()>) {
    while start_rx.recv().await.is_some() {
        let p = ctx.p;
        ctx.set(
            0,
            vec![
                ParamSetValue::new(
                    ctx.ad.status,
                    0,
                    ParamValue::Int32(ADStatus::Acquire as i32),
                ),
                ParamSetValue::new(ctx.ad.acquire, 0, ParamValue::Int32(1)),
            ],
        )
        .await;

        while ctx.shared.acquiring() {
            match ctx.http.get_json(serval::MEASUREMENT, TIMEOUT_POLL) {
                Ok(m) => {
                    let mut updates = Vec::new();
                    if let Some(v) = m.pointer("/Info/Status").and_then(Value::as_str) {
                        updates.push(ParamSetValue::new(
                            p.status,
                            0,
                            ParamValue::Octet(v.to_string()),
                        ));
                    }
                    for (ptr, reason) in [
                        ("/Info/ElapsedTime", p.elapsed_time),
                        ("/Info/TimeLeft", p.time_left),
                        ("/Info/PixelEventRate", p.pel_rate),
                        ("/Info/TdcEventRate", p.tdc1_rate),
                    ] {
                        if let Some(v) = m.pointer(ptr).and_then(Value::as_f64) {
                            updates.push(ParamSetValue::new(reason, 0, ParamValue::Float64(v)));
                        }
                    }
                    for (ptr, reason) in [
                        ("/Info/FrameCount", p.frame_count),
                        ("/Info/DroppedFrames", p.dropped_frames),
                    ] {
                        if let Some(v) = m.pointer(ptr).and_then(Value::as_i64) {
                            updates.push(ParamSetValue::new(
                                reason,
                                0,
                                ParamValue::Int32(i32::try_from(v).unwrap_or(i32::MAX)),
                            ));
                        }
                    }
                    if let Some(v) = m.pointer("/Info/StartDateTime").and_then(Value::as_str) {
                        updates.push(ParamSetValue::new(
                            p.start_time,
                            0,
                            ParamValue::Octet(v.to_string()),
                        ));
                    }
                    ctx.set(0, updates).await;

                    if !serval::measurement_is_running(&m) {
                        break;
                    }
                }
                Err(e) => {
                    // UPSTREAM DEFECT (acquire.cpp:514-518): C `continue`s on an
                    // HTTP error without sleeping, so a Serval that goes away
                    // mid-acquisition leaves this thread spinning at 100% CPU.
                    log::warn!("timepix3: GET /measurement: {e}");
                }
            }
            tokio::time::sleep(ACQUISITION_POLL).await;
        }

        ctx.shared.set_acquiring(false);
        ctx.set(
            0,
            vec![
                ParamSetValue::new(ctx.ad.acquire, 0, ParamValue::Int32(0)),
                ParamSetValue::new(ctx.ad.status, 0, ParamValue::Int32(ADStatus::Idle as i32)),
            ],
        )
        .await;
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
        if !ctx.shared.acquiring() {
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

        let stream = match TcpStream::connect((host.as_str(), port)) {
            Ok(s) => s,
            Err(e) => {
                log::warn!("timepix3: {kind:?}: connecting to {host}:{port}: {e}");
                tokio::time::sleep(RECONNECT_WAIT).await;
                continue;
            }
        };
        if let Err(e) = stream.set_read_timeout(Some(READ_TIMEOUT)) {
            log::warn!("timepix3: {kind:?}: cannot set the read timeout: {e}");
        }
        log::info!("timepix3: {kind:?} stream connected to {host}:{port}");
        read_stream(&ctx, kind, stream).await;
        log::info!("timepix3: {kind:?} stream closed");
    }
}

async fn read_stream(ctx: &Arc<Ctx>, kind: StreamKind, mut stream: TcpStream) {
    let mut decoder = FrameDecoder::new(kind.channel());
    let mut buf = vec![0u8; 256 * 1024];

    while ctx.shared.acquiring() {
        match stream.read(&mut buf) {
            Ok(0) => return, // The peer closed the stream.
            Ok(n) => decoder.push(&buf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => {
                log::warn!("timepix3: {kind:?}: read: {e}");
                return;
            }
        }

        loop {
            match decoder.next_frame() {
                Ok(Some(frame)) => handle_frame(ctx, kind, frame).await,
                Ok(None) => break,
                Err(e) => {
                    // The payload length is unknown, so the stream cannot be
                    // resynchronised: drop the connection and start over.
                    log::error!("timepix3: {kind:?}: {e}");
                    return;
                }
            }
        }
    }
}

async fn handle_frame(ctx: &Arc<Ctx>, kind: StreamKind, frame: Frame) {
    match (kind, frame) {
        (StreamKind::PrvImg, Frame::Image(f)) => {
            let p = ctx.p;
            ctx.set(
                ADDR,
                vec![
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
                ],
            )
            .await;
            publish_image(ctx, f.width, f.height, f.pixels).await;
        }
        (StreamKind::Img, Frame::Image(f)) => {
            let p = ctx.p;
            let update = ctx.shared.img.lock().add(&f.pixels);
            let counts = ctx.shared.img.lock().total_counts();

            ctx.set(
                ADDR,
                vec![
                    ParamSetValue::new(p.img_frame_number, ADDR, ParamValue::Int32(f.frame_number)),
                    ParamSetValue::new(
                        p.img_time_at_frame,
                        ADDR,
                        ParamValue::Float64(f.time_at_frame),
                    ),
                    ParamSetValue::new(
                        p.img_image_frame,
                        ADDR,
                        ParamValue::Int32Array(
                            f.pixels
                                .iter()
                                .map(|&v| v as i32)
                                .collect::<Vec<i32>>()
                                .into(),
                        ),
                    ),
                ],
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
            publish_image(ctx, f.width, f.height, f.pixels).await;
        }
        (StreamKind::PrvHst, Frame::Histogram(h)) => {
            let p = ctx.p;
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
        }
        // The decoder is created for the channel, so this cannot happen; a
        // mismatch means the channel's Format PV disagrees with the port.
        (kind, _) => log::error!("timepix3: {kind:?}: the stream carries the wrong frame type"),
    }
}

async fn publish_accumulation(
    ctx: &Arc<Ctx>,
    update: &Update,
    sum_param: usize,
    sum_addr: i32,
    sum_n_param: usize,
    sum_n_addr: i32,
) {
    ctx.set_int64_array(
        sum_param,
        sum_addr,
        update
            .running
            .iter()
            .map(|&v| i64::try_from(v).unwrap_or(i64::MAX))
            .collect(),
    )
    .await;
    if let Some(sum_n) = &update.sum_n {
        ctx.set_int64_array(
            sum_n_param,
            sum_n_addr,
            sum_n
                .iter()
                .map(|&v| i64::try_from(v).unwrap_or(i64::MAX))
                .collect(),
        )
        .await;
    }
}

/// Push a preview frame down the plugin chain as an NDArray.
async fn publish_image(ctx: &Arc<Ctx>, width: usize, height: usize, pixels: Vec<u32>) {
    if ctx
        .handle
        .read_int32(ctx.ad.base.array_callbacks, 0)
        .await
        .unwrap_or(0)
        == 0
    {
        return;
    }
    let counter = ctx
        .handle
        .read_int32(ctx.ad.base.array_counter, 0)
        .await
        .unwrap_or(0)
        + 1;
    ctx.set_int(ctx.ad.base.array_counter, 0, counter).await;

    let mut attributes = epics_rs::ad_core::attributes::NDAttributeList::new();
    attributes.add(epics_rs::ad_core::attributes::NDAttribute::new_static(
        "ColorMode",
        "Color Mode",
        epics_rs::ad_core::attributes::NDAttrSource::Driver,
        epics_rs::ad_core::attributes::NDAttrValue::Int32(NDColorMode::Mono as i32),
    ));

    let mut array = NDArray::with_data(
        vec![NDDimension::new(width), NDDimension::new(height)],
        NDDataBuffer::U32(pixels),
    );
    array.unique_id = counter;
    array.timestamp = EpicsTimestamp::now();
    array.time_stamp = array.timestamp.as_f64();
    array.attributes = attributes;

    ArrayPublisher::new(ctx.output.clone())
        .publish(Arc::new(array))
        .await;
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

    #[test]
    fn adjust_reports_the_shape() {
        assert_eq!(adjust_code(&json!({})), -1);
        assert_eq!(adjust_code(&json!({"Adjust": null})), -1);
        assert_eq!(adjust_code(&json!({"Adjust": [1, 2]})), -2);
        assert_eq!(adjust_code(&json!({"Adjust": 7})), -3);
    }
}
