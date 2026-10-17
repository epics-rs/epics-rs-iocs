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
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool, PublishedArray};
use epics_rs::ad_core::params::ad_driver::ADDriverParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interrupt::InterruptValue;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{DriverCall, ParamSetValue, RequestOp};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use serde_json::Value;

use crate::accum::{RangeState, Update, clamp_average_to_u32};
use crate::adlog;
use crate::coordinator::Phase;
use crate::driver::TimePix3Driver;
use crate::family::DetectorFamily;
use crate::http::{DEFAULT_TIMEOUT, HttpError, ServalHttp, TIMEOUT_5000_MS};
use crate::params::TimePixParams;
use crate::serval;
use crate::state::{Command, Mirrored, Shared};
use crate::stream::{
    Channel, EndOfStream, Frame, FrameDecoder, FrameError, ImageLimits, parse_tcp_path,
};
use crate::window::{Configuration, RollingSumStatus};
use crate::worker::{Generation, WorkerState};

/// C's `connectionPollPeriodSec_` (ADTimePix.cpp:1471).
const CONNECTION_POLL: Duration = Duration::from_secs(5);
/// C's `timePixCallback` poll period (acquire.cpp:430).
const ACQUISITION_POLL: Duration = Duration::from_millis(10);
/// C's `reconnectDelay` (serval_stream.cpp:497, histogram_io.cpp:962).
const RECONNECT_WAIT: Duration = Duration::from_millis(1000);
/// C `kReceivePollTimeoutMs`, the stream socket's `SO_RCVTIMEO`
/// (network_client.h:25, network_client.cpp:96-101).
const READ_TIMEOUT: Duration = Duration::from_millis(250);
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
/// C `NDARRAY_ADDR_IMG_THRESHOLD1` (ADTimePix.h:921).
const IMG_THRESHOLD1_ADDR: usize = 13;

/// C `previewNdarrayAddressForThreshold` (serval_stream.cpp:49-51).
fn threshold_addr(threshold_id: i32, th0: usize, th1: usize) -> usize {
    if threshold_id == 1 { th1 } else { th0 }
}

/// The constant half of C `PreviewJsonimageStream` (serval_stream.cpp:59-85,
/// :1557-1604); its state is [`crate::state::PreviewState`] at `index`.
#[derive(Clone, Copy)]
struct Preview {
    index: usize,
    th0: usize,
    th1: usize,
    diff: usize,
    /// C passes `paramFrameNumber == -1` for PrvImg1, which posts no
    /// geometry, frame parameters or rate.
    metadata: bool,
    tag: &'static str,
}

const PRV_IMG: Preview = Preview {
    index: 0,
    th0: PRV_IMG_ADDR,
    th1: 8,
    diff: 9,
    metadata: true,
    tag: "PrvImg",
};

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

/// Whether C runs a caller under the port lock: the stream workers never take
/// it (serval_stream.cpp, histogram_io.cpp), `pushProcessed*ToPlugins` runs
/// in writeInt32 under it (ADTimePix.cpp:588-600).
#[derive(Clone, Copy)]
enum Locking {
    Unlocked,
    Locked,
}

/// The actor's `CallParamCallbacks`, from a worker job: every update stored,
/// a failed one reported, then the callbacks for `addr`.
pub(crate) fn apply_updates(base: &mut PortDriverBase, addr: i32, updates: &[ParamSetValue]) {
    for u in updates {
        let outcome = match u {
            ParamSetValue::Value {
                reason,
                addr,
                value,
            } => base.set_param_value(*reason, *addr, value.clone()),
            ParamSetValue::UInt32Digital {
                reason,
                addr,
                value,
                mask,
                interrupt_mask,
            } => base.set_uint32_param(*reason, *addr, *value, *mask, *interrupt_mask),
            ParamSetValue::Status {
                reason,
                addr,
                status,
                alarm_status,
                alarm_severity,
            } => base.set_param_status(*reason, *addr, *status, *alarm_status, *alarm_severity),
        };
        if let Err(e) = outcome {
            base.trace_print(
                TraceMask::ERROR,
                format_args!("callParamCallbacks: param set failed: {e}"),
            );
        }
    }
    let _ = base.call_param_callbacks(addr);
}

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
    async fn post_range(
        &self,
        locking: Locking,
        saturated: usize,
        status: usize,
        range: Option<RangeState>,
    ) {
        let Some(range) = range else { return };
        self.post_as(
            locking,
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

    /// C's unlocked `getIntegerParam` from a stream worker.
    fn mirrored(&self, which: Mirrored) -> i32 {
        self.shared.mirror.get(which)
    }

    /// C's unlocked `set*Param` + `callParamCallbacks` from a stream worker,
    /// which a stop holding the port does not hold up.
    fn post(&self, addr: i32, updates: Vec<ParamSetValue>) {
        self.shared
            .port_jobs
            .push(move |base| apply_updates(base, addr, &updates));
        self.flush_jobs();
    }

    fn post_int64(&self, reason: usize, value: i64) {
        self.post(
            ADDR,
            vec![ParamSetValue::new(reason, ADDR, ParamValue::Int64(value))],
        );
    }

    /// Have the actor run the workers' jobs; a stop that holds the port runs
    /// them itself, and this flush then finds none.
    fn flush_jobs(&self) {
        let shared = self.shared.clone();
        let call = DriverCall::new(move |d| shared.port_jobs.run(d.base_mut()));
        let _ = self
            .handle
            .try_submit(RequestOp::WithDriver(call), AsynUser::default());
    }

    /// C's unlocked `getAttributes` from a stream worker, evaluated where its
    /// PARAM values live.
    async fn worker_attributes(&self, array: &mut DriverArray) {
        let owner = self.pool.owner().clone();
        if owner.attributes().lock().list.is_empty() {
            return;
        }
        let (tx, rx) = tokio::sync::oneshot::channel();
        self.shared.port_jobs.push(move |base| {
            let _ = tx.send(owner.attributes().lock().update(base));
        });
        self.flush_jobs();
        if let Ok(fresh) = rx.await {
            array.attributes.copy_from(&fresh);
        }
    }

    async fn post_as(&self, locking: Locking, addr: i32, updates: Vec<ParamSetValue>) {
        match locking {
            Locking::Unlocked => self.post(addr, updates),
            Locking::Locked => self.set(addr, updates).await,
        }
    }

    async fn attributes_as(&self, locking: Locking, array: &mut DriverArray) {
        match locking {
            Locking::Unlocked => self.worker_attributes(array).await,
            // C ignores the getAttributes status.
            Locking::Locked => {
                let _ = self.pool.owner().get_attributes(array).await;
            }
        }
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
/// `connected` is what [`initial_server_check_connection`] found.
pub fn start(
    ctx: Arc<Ctx>,
    cmd_rx: rt::CommandReceiver<Command>,
    connected: bool,
) -> Vec<std::thread::JoinHandle<()>> {
    let (acq_tx, acq_rx) = rt::command_channel::<()>(1);

    // The initial check already read the dashboard when a URL is set
    // (ADTimePix.cpp:2065-2068).
    let skip_once = !ctx.http.base_url().is_empty();
    let c = ctx.clone();
    let connection = rt::run_thread_named("tpx3ConnectionPoll", move || async move {
        connection_poll(c, cmd_rx, acq_tx, connected, skip_once).await;
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
    mut was_up: bool,
    mut skip_once: bool,
) {
    // C waits on an event only its destructor signals, so the poll runs a
    // period after the last one whatever the commands do
    // (serval_http.cpp:400-402).
    let mut next_poll = tokio::time::Instant::now() + CONNECTION_POLL;
    loop {
        let command = match tokio::time::timeout_at(next_poll, cmd_rx.recv()).await {
            Ok(Some(command)) => command,
            Ok(None) => return,
            Err(_) => {
                if !std::mem::take(&mut skip_once) {
                    was_up = poll_connection(&ctx, was_up).await;
                }
                next_poll = tokio::time::Instant::now() + CONNECTION_POLL;
                continue;
            }
        };

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
            // C runs checkConnection() from writeInt32, outside the poll's
            // edge tracking (ADTimePix.cpp:563-568).
            Command::RefreshConnection => {
                check_connection(&ctx).await;
            }
        }
    }
}

/// One pass of C `connectionPollThread` (serval_http.cpp:408-420): the
/// dashboard read publishes no HttpCode, a 0 -> 1 edge reconciles the
/// read-only state, and a partial reconciliation keeps the edge open for the
/// next poll. Returns the new "was up".
async fn poll_connection(ctx: &Arc<Ctx>, was_up: bool) -> bool {
    let reply = read_dashboard(ctx, TIMEOUT_5000_MS);
    let mut updates = Vec::new();
    let (serval_ok, det_ok) = publish_dashboard(ctx, &reply, &mut updates);
    let up = serval_ok && det_ok;
    let complete = if up && !was_up {
        ctx.set(0, updates).await;
        refresh_on_reconnect(ctx).await
    } else {
        connection_status(ctx, serval_ok, det_ok, &mut updates);
        ctx.set(0, updates).await;
        true
    };
    up && complete
}

/// C `refreshOnReconnect` (serval_http.cpp:287-312): every readback runs,
/// and the status reports whether all of them succeeded.
async fn refresh_on_reconnect(ctx: &Arc<Ctx>) -> bool {
    let destination = ctx
        .handle
        .with_driver::<TimePix3Driver, _>(|d| d.get_server().is_ok())
        .await
        .unwrap_or(false);
    let detector = refresh_detector(ctx).await;
    let measurement_config = refresh_measurement_config(ctx).await;
    let complete = destination && detector && measurement_config;

    let mut updates = Vec::new();
    if complete {
        connection_status(ctx, true, true, &mut updates);
    } else {
        ctx.trace(
            TraceMask::ERROR,
            &adlog::err(
                adlog::REFRESH_ON_RECONNECT,
                &format!(
                    "Reconnect state refresh incomplete: destination={} detector={} \
                     measurementConfig={}",
                    i32::from(destination),
                    i32::from(detector),
                    i32::from(measurement_config),
                ),
            ),
        );
        updates.push(ParamSetValue::new(
            ctx.ad.status_message,
            0,
            ParamValue::Octet(
                b"Connected; reconnect state refresh incomplete"
                    .to_vec()
                    .into(),
            ),
        ));
        updates.push(ParamSetValue::new(
            ctx.ad.status,
            0,
            ParamValue::Int32(ADStatus::Error as i32),
        ));
    }
    ctx.set(0, updates).await;
    complete
}

/// C `getMeasurementConfig`, which the Health scan and the reconnect refresh
/// run (ADTimePix.cpp:560, serval_http.cpp:297-298).
async fn refresh_measurement_config(ctx: &Arc<Ctx>) -> bool {
    ctx.handle
        .with_driver::<TimePix3Driver, _>(|d| d.get_measurement_config().is_ok())
        .await
        .unwrap_or(false)
}

/// C `checkConnection()` with its defaults, publishing the HTTP status and
/// the connection status (serval_http.cpp:363-392).
async fn check_connection(ctx: &Arc<Ctx>) -> bool {
    let reply = read_dashboard(ctx, TIMEOUT_5000_MS);
    let mut updates = vec![ParamSetValue::new(
        ctx.p.http_code,
        0,
        ParamValue::Int32(reply.code),
    )];
    log_dashboard_failure(
        ctx,
        adlog::CHECK_CONNECTION,
        "checkConnection",
        "checkConnection",
        &reply,
    );
    let (serval_ok, det_ok) = publish_dashboard(ctx, &reply, &mut updates);
    connection_status(ctx, serval_ok, det_ok, &mut updates);
    ctx.set(0, updates).await;
    serval_ok && det_ok
}

/// C `initialServerCheckConnection` (serval_http.cpp:315-354), which the
/// constructor runs before the connection poll starts
/// (ADTimePix.cpp:2050-2058); returns whether Serval and the detector are
/// connected.
pub fn initial_server_check_connection(ctx: &Arc<Ctx>) -> bool {
    let url = ctx.http.base_url();
    if url.is_empty() {
        ctx.trace(
            TraceMask::ERROR,
            &adlog::err(adlog::CONSTRUCTOR, "Connection failed, abort"),
        );
        return false;
    }
    let reply = match ctx.http.get("", DEFAULT_TIMEOUT) {
        Ok(_) => {
            let reply = read_dashboard(ctx, DEFAULT_TIMEOUT);
            log_dashboard_failure(
                ctx,
                adlog::INITIAL_SERVER_CHECK_CONNECTION,
                "initialServerCheckConnection",
                "initialServerCheckConnection dashboard",
                &reply,
            );
            reply
        }
        Err(e) => {
            ctx.trace(
                TraceMask::ERROR,
                &adlog::http_failure_reply(
                    "initialServerCheckConnection",
                    "GET",
                    url,
                    e.code(),
                    e.body(),
                ),
            );
            // publishDashboardFailure(false)
            DashboardReply {
                code: e.code(),
                body: e.body().to_string(),
                parsed: Err(serval::DashboardError::HttpFailure),
            }
        }
    };
    let mut updates = vec![ParamSetValue::new(
        ctx.p.http_code,
        0,
        ParamValue::Int32(reply.code),
    )];
    let (serval_ok, det_ok) = publish_dashboard(ctx, &reply, &mut updates);
    connection_status(ctx, serval_ok, det_ok, &mut updates);
    let _ = ctx.handle.set_params_and_notify_blocking(0, updates);
    if !det_ok {
        ctx.trace(
            TraceMask::ERROR,
            &adlog::err(
                adlog::INITIAL_SERVER_CHECK_CONNECTION,
                &format!("ERROR: Failed to connect to server {url}"),
            ),
        );
    }
    serval_ok && det_ok
}

/// One `GET /dashboard`: cpr's status code and C `parseResponse`'s verdict.
struct DashboardReply {
    code: i32,
    body: String,
    parsed: Result<serval::Dashboard, serval::DashboardError>,
}

fn read_dashboard(ctx: &Ctx, timeout: Duration) -> DashboardReply {
    let (code, body) = match ctx.http.get(serval::DASHBOARD, timeout) {
        Ok(body) => (200, body),
        Err(e) => (e.code(), e.body().to_string()),
    };
    let parsed = serval::parse_dashboard_response(code, &body);
    DashboardReply { code, body, parsed }
}

/// C `logHttpFailure` plus `ERR_ARGS("<context>: %s", responseErrorMessage)`
/// for a rejected dashboard reply (serval_http.cpp:382-384, :442-443).
fn log_dashboard_failure(
    ctx: &Ctx,
    func: &str,
    context: &str,
    http_context: &str,
    reply: &DashboardReply,
) {
    let Err(error) = &reply.parsed else { return };
    let url = format!("{}{}", ctx.http.base_url(), serval::DASHBOARD);
    ctx.trace(
        TraceMask::ERROR,
        &adlog::http_failure_reply(http_context, "GET", &url, reply.code, &reply.body),
    );
    ctx.trace(
        TraceMask::ERROR,
        &adlog::err(func, &format!("{context}: {}", error.message())),
    );
}

/// C `publishDashboardSnapshot` and `publishDashboardFailure`
/// (serval_http.cpp:219-249), which every dashboard read runs; returns
/// whether Serval answered 200 and whether the detector is connected
/// (serval_http.cpp:371-373).
fn publish_dashboard(
    ctx: &Arc<Ctx>,
    reply: &DashboardReply,
    updates: &mut Vec<ParamSetValue>,
) -> (bool, bool) {
    let p = ctx.p;
    let serval_ok = reply.code == 200;
    let det_type = reply
        .parsed
        .as_ref()
        .ok()
        .and_then(|d| d.detector_type.as_deref());
    let det_ok = det_type.is_some();
    updates.push(ParamSetValue::new(
        p.serval_connected,
        0,
        ParamValue::Int32(i32::from(serval_ok)),
    ));
    updates.push(ParamSetValue::new(
        p.det_connected,
        0,
        ParamValue::Int32(i32::from(det_ok)),
    ));
    let shown = det_type.unwrap_or("null");
    updates.push(ParamSetValue::new(
        p.det_type,
        0,
        ParamValue::Octet(shown.as_bytes().to_vec().into()),
    ));
    if let Some(det_type) = det_type {
        updates.push(ParamSetValue::new(
            ctx.ad.base.model,
            0,
            ParamValue::Octet(det_type.as_bytes().to_vec().into()),
        ));
    }
    match &reply.parsed {
        Ok(dashboard) => {
            serval_version(ctx, &dashboard.response, updates);
            disk_space(ctx, &dashboard.response, updates);
            if let Some(status) = dashboard_measurement_status(&dashboard.response) {
                measurement_status(p, status, updates);
            }
        }
        Err(_) => measurement_status(p, &Value::Null, updates),
    }
    (serval_ok, det_ok)
}

/// C `updateMeasurementFromDashboard` (acquire.cpp:100-111): the dashboard
/// clears `Measurement` when idle, and then the last status stays.
fn dashboard_measurement_status(dashboard: &Value) -> Option<&Value> {
    let meas = dashboard.get("Measurement").filter(|m| !m.is_null())?;
    meas.get("Info")
        .filter(|i| i.is_object())
        .and_then(|i| i.get("Status"))
        .or_else(|| meas.get("Status"))
}

/// C `updateMeasurementStatusFromJson` (acquire.cpp:84-98).
fn measurement_status(p: TimePixParams, status: &Value, updates: &mut Vec<ParamSetValue>) {
    let (text, index) = match status {
        Value::Null => (String::new(), -1),
        Value::String(s) => (s.clone(), pipeline_state_index(s)),
        other => (serval::dump(other, None, false), -1),
    };
    updates.push(ParamSetValue::new(
        p.status,
        0,
        ParamValue::Octet(text.into_bytes().into()),
    ));
    updates.push(ParamSetValue::new(
        p.pipeline_state,
        0,
        ParamValue::Int32(index),
    ));
}

/// C `pipelineStateIndex` (acquire.cpp:64-70).
fn pipeline_state_index(status: &str) -> i32 {
    match status {
        "DA_STARTING" => 0,
        "DA_RECORDING" => 1,
        "DA_STOPPING" => 2,
        "DA_IDLE" | "DA_STOPPED" => 3,
        _ => -1,
    }
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
/// is configured (C `publishDashboardSnapshot`, serval_http.cpp:228-239).
fn disk_space(ctx: &Arc<Ctx>, dashboard: &Value, updates: &mut Vec<ParamSetValue>) {
    let p = ctx.p;
    let Some(disk) = dashboard.pointer("/Server/DiskSpace/0") else {
        return;
    };
    if let Some(v) = disk.get("FreeSpace").and_then(Value::as_i64) {
        updates.push(ParamSetValue::new(p.free_space, 0, ParamValue::Int64(v)));
    }
    if let Some(v) = disk.get("WriteSpeed").and_then(Value::as_f64) {
        updates.push(ParamSetValue::new(p.write_speed, 0, ParamValue::Float64(v)));
    }
    if let Some(v) = disk.get("LowerLimit").and_then(Value::as_i64) {
        updates.push(ParamSetValue::new(p.lower_limit, 0, ParamValue::Int64(v)));
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

/// C `getDashboard` (serval_http.cpp:431-454).
async fn refresh_dashboard(ctx: &Arc<Ctx>) {
    let reply = read_dashboard(ctx, TIMEOUT_5000_MS);
    log_dashboard_failure(
        ctx,
        adlog::GET_DASHBOARD,
        "getDashboard",
        "getDashboard",
        &reply,
    );
    let mut updates = Vec::new();
    let (serval_ok, det_ok) = publish_dashboard(ctx, &reply, &mut updates);
    connection_status(ctx, serval_ok, det_ok, &mut updates);
    ctx.set(0, updates).await;
}

/// C `getDetector` (serval_http.cpp:1293) from the task; false when
/// `/detector` failed.
async fn refresh_detector(ctx: &Arc<Ctx>) -> bool {
    let read = read_detector(&ctx.http, &ctx.shared, ctx.p, ctx.ad, false);
    for step in read.steps {
        match step {
            DetectorStep::Trace(mask, line) => ctx.trace(mask, &line),
            DetectorStep::Post(addr, updates) => ctx.set(addr, updates).await,
        }
    }
    read.ok
}

/// One step of C `getDetector`, in C order: a trace line, or the param
/// writes made since the last `callParamCallbacks(addr)` and that call. A
/// write may name another address; C stores it there and it goes out with
/// that address's next callbacks.
pub(crate) enum DetectorStep {
    Trace(TraceMask, String),
    Post(i32, Vec<ParamSetValue>),
}

/// What C `getDetector` leaves, for the task or the port (initAcquisition)
/// to apply in order.
pub(crate) struct DetectorRead {
    pub ok: bool,
    pub steps: Vec<DetectorStep>,
}

/// C's `set*Param` calls and trace lines; `callbacks` is
/// `callParamCallbacks(addr)` over the writes made since the last one.
#[derive(Default)]
struct DetectorWrites {
    pending: Vec<ParamSetValue>,
    steps: Vec<DetectorStep>,
}

impl DetectorWrites {
    /// C stores nothing for an address past the port's maxAddr.
    fn set(&mut self, reason: usize, addr: i32, value: ParamValue) {
        if usize::try_from(addr).is_ok_and(|a| a < crate::driver::MAX_ADDR) {
            self.pending.push(ParamSetValue::new(reason, addr, value));
        }
    }

    fn int(&mut self, reason: usize, addr: i32, v: i32) {
        self.set(reason, addr, ParamValue::Int32(v));
    }

    fn float(&mut self, reason: usize, addr: i32, v: f64) {
        self.set(reason, addr, ParamValue::Float64(v));
    }

    fn text(&mut self, reason: usize, addr: i32, v: &str) {
        self.set(reason, addr, ParamValue::Octet(v.as_bytes().into()));
    }

    fn trace(&mut self, mask: TraceMask, line: String) {
        self.steps.push(DetectorStep::Trace(mask, line));
    }

    fn callbacks(&mut self, addr: i32) {
        if usize::try_from(addr).is_ok_and(|a| a < crate::driver::MAX_ADDR) {
            let writes = std::mem::take(&mut self.pending);
            self.steps.push(DetectorStep::Post(addr, writes));
        }
    }
}

static JSON_NULL: Value = Value::Null;

/// A nlohmann `type_error`, as its `what()` reads.
struct JsonTypeError(String);

impl JsonTypeError {
    fn new(id: u32, msg: &str, v: &Value) -> Self {
        Self(format!(
            "[json.exception.type_error.{id}] {msg}{}",
            json_type_name(v)
        ))
    }
}

/// nlohmann `type_name()`.
fn json_type_name(v: &Value) -> &'static str {
    match v {
        Value::Null => "null",
        Value::Bool(_) => "boolean",
        Value::Number(_) => "number",
        Value::String(_) => "string",
        Value::Array(_) => "array",
        Value::Object(_) => "object",
    }
}

/// nlohmann `get<int>()` of a number: a `static_cast<int>`.
fn json_number_as_int(n: &serde_json::Number) -> i32 {
    match (n.as_i64(), n.as_u64()) {
        (Some(i), _) => i as i32,
        (None, Some(u)) => u as i32,
        _ => n.as_f64().unwrap_or(0.0) as i32,
    }
}

/// nlohmann `get<int>()`, which also takes a boolean; an absent key reads
/// as the null nlohmann's mutable `operator[]` inserts.
fn json_get_int(v: Option<&Value>) -> Result<i32, JsonTypeError> {
    match v.unwrap_or(&JSON_NULL) {
        Value::Number(n) => Ok(json_number_as_int(n)),
        Value::Bool(b) => Ok(i32::from(*b)),
        other => Err(JsonTypeError::new(
            302,
            "type must be number, but is ",
            other,
        )),
    }
}

/// nlohmann `value(key, json())`, which only an object has.
fn json_value<'a>(obj: &'a Value, key: &str) -> Result<&'a Value, JsonTypeError> {
    match obj {
        Value::Object(o) => Ok(o.get(key).unwrap_or(&JSON_NULL)),
        other => Err(JsonTypeError::new(306, "cannot use value() with ", other)),
    }
}

/// C `jsonIntOr(j)` (serval_http.cpp:70-75).
fn json_int_or(v: Option<&Value>) -> i32 {
    match v {
        Some(Value::Number(n)) => json_number_as_int(n),
        Some(Value::Bool(b)) => i32::from(*b),
        _ => 0,
    }
}

/// C `jsonDoubleOr(j)` (serval_http.cpp:116-119).
fn json_double_or(v: Option<&Value>) -> f64 {
    v.and_then(Value::as_f64).unwrap_or(0.0)
}

/// C `jsonBoolOr(j)` (serval_http.cpp:121-125): a number is `get<int>() != 0`.
fn json_bool_or(v: Option<&Value>) -> bool {
    match v {
        Some(Value::Bool(b)) => *b,
        Some(Value::Number(n)) => json_number_as_int(n) != 0,
        _ => false,
    }
}

/// C `chipBoardEntryLabel` (serval_http.cpp:50-68).
fn chip_board_entry_label(chip: &Value) -> String {
    if let Value::Object(o) = chip {
        if let Some(Value::String(name)) = o.get("Name") {
            return name.clone();
        }
        match o.get("Id") {
            Some(Value::Number(n)) if n.is_i64() || n.is_u64() => {
                return json_number_as_int(n).to_string();
            }
            Some(Value::String(id)) => return id.clone(),
            _ => {}
        }
    }
    serval::json_to_string(chip)
}

/// C `getDetector` (serval_http.cpp:1293-1576); `publish_http_status` is its
/// argument: false from the task, true from initAcquisition.
pub(crate) fn read_detector(
    http: &ServalHttp,
    shared: &Shared,
    p: TimePixParams,
    ad: ADDriverParams,
    publish_http_status: bool,
) -> DetectorRead {
    let mut w = DetectorWrites::default();
    w.trace(
        TraceMask::FLOW,
        adlog::log(
            adlog::GET_DETECTOR,
            "Reading Detector Health, info, config, layout, chips",
        ),
    );
    let failure = |w: &mut DetectorWrites, write_msg: &str, message: &str, status: ADStatus| {
        w.int(p.det_connected, 0, 0);
        w.text(p.write_msg, 0, write_msg);
        w.text(ad.status_message, 0, message);
        w.int(ad.status, 0, status as i32);
        w.callbacks(0);
    };
    let reply = http.get(serval::DETECTOR, TIMEOUT_5000_MS);
    if publish_http_status {
        w.int(
            p.http_code,
            0,
            reply.as_ref().map_or_else(HttpError::code, |_| 200),
        );
    }
    let body = match reply {
        Ok(body) => body,
        Err(e) => {
            let url = format!("{}{}", http.base_url(), serval::DETECTOR);
            if let Some(line) = adlog::http_failure("getDetector GET /detector", "GET", &url, &e) {
                w.trace(TraceMask::ERROR, line);
            }
            failure(
                &mut w,
                e.body(),
                "Failed to read detector information",
                ADStatus::Disconnected,
            );
            return DetectorRead {
                ok: false,
                steps: w.steps,
            };
        }
    };
    let detector = match serval::parse_detector_response(&body) {
        Ok(d) => d,
        Err(e) => {
            let message = format!("Invalid detector response: {}", e.message());
            w.trace(
                TraceMask::ERROR,
                adlog::err(adlog::GET_DETECTOR, &format!("getDetector: {message}")),
            );
            failure(&mut w, &message, &message, ADStatus::Error);
            return DetectorRead {
                ok: false,
                steps: w.steps,
            };
        }
    };
    if let Err(e) = publish_detector(&mut w, shared, &p, &ad, &detector) {
        w.trace(
            TraceMask::ERROR,
            adlog::err(
                adlog::GET_DETECTOR,
                &format!("getDetector JSON error: {}", e.0),
            ),
        );
        let message = "Invalid detector response structure";
        failure(&mut w, message, message, ADStatus::Error);
        return DetectorRead {
            ok: false,
            steps: w.steps,
        };
    }
    DetectorRead {
        ok: true,
        steps: w.steps,
    }
}

/// The `try` block of C `getDetector` (serval_http.cpp:1327-1557); an error
/// is the `json::exception` its handler reports.
fn publish_detector(
    w: &mut DetectorWrites,
    shared: &Shared,
    p: &TimePixParams,
    ad: &ADDriverParams,
    detector: &Value,
) -> Result<(), JsonTypeError> {
    let (info, cfg) = (&detector["Info"], &detector["Config"]);
    update_detector_health(w, p, detector);

    let info_text = |key: &str| serval::json_to_string(info.get(key).unwrap_or(&JSON_NULL));
    w.text(p.iface_name, 0, &info_text("IfaceName"));
    w.text(p.sw_version, 0, &info_text("SW_version"));
    w.text(p.fw_version, 0, &info_text("FW_version"));
    // UPSTREAM DEFECT (serval_http.cpp:1341-1342): C sets ADSerialNumber from
    // `Info.SW_version` — the Serval software version, not a serial number —
    // and the line that would have used the chipboard ID sits commented out
    // right above it. The chipboard ID is the detector's identity.
    if let Some(id) = detector.pointer("/Info/Boards/0/ChipboardId") {
        w.text(ad.base.serial_number, 0, &serval::json_to_string(id));
    }
    w.text(ad.base.firmware_version, 0, &info_text("FW_version"));

    // parse_detector_response checked these are ints and the rows positive.
    let int = |key: &str| info.get(key).and_then(Value::as_i64).unwrap_or(0) as i32;
    let (pix_count, rows, chips) = (int("PixCount"), int("NumberOfRows"), int("NumberOfChips"));
    w.int(p.pix_count, 0, pix_count);
    w.int(p.row_len, 0, int("RowLen"));
    w.int(p.number_of_chips, 0, chips);
    w.int(p.number_of_rows, 0, rows);
    w.int(ad.max_size_y, 0, rows);
    w.int(ad.max_size_x, 0, pix_count.checked_div(rows).unwrap_or(0));
    w.int(p.mpx_type, 0, int("MpxType"));
    update_detector_family(w, shared, p, detector);

    let board0 = [p.boards_ch1, p.boards_ch2, p.boards_ch3, p.boards_ch4];
    let board1 = [p.boards_ch5, p.boards_ch6, p.boards_ch7, p.boards_ch8];
    let labels = |w: &mut DetectorWrites, reasons: [usize; 4], chips: Option<&Vec<Value>>| {
        for (i, reason) in reasons.into_iter().enumerate() {
            let label = chips.and_then(|c| c.get(i)).map(chip_board_entry_label);
            w.text(reason, 0, &label.unwrap_or_default());
        }
    };
    match info
        .get("Boards")
        .and_then(Value::as_array)
        .filter(|b| !b.is_empty())
    {
        Some(boards) => {
            let b0 = &boards[0];
            w.text(
                p.boards_id,
                0,
                &serval::json_to_string(json_value(b0, "ChipboardId")?),
            );
            w.text(
                p.boards_ip,
                0,
                &serval::json_to_string(json_value(b0, "IpAddress")?),
            );
            let chips = b0.get("Chips").and_then(Value::as_array);
            labels(w, board0, chips);
            match boards.get(1) {
                Some(b1) => {
                    w.text(
                        p.boards2_id,
                        0,
                        &serval::json_to_string(json_value(b1, "ChipboardId")?),
                    );
                    w.text(
                        p.boards2_ip,
                        0,
                        &serval::json_to_string(json_value(b1, "IpAddress")?),
                    );
                    let chips = b1
                        .get("Chips")
                        .and_then(Value::as_array)
                        .filter(|c| c.len() >= 4);
                    labels(w, board1, chips);
                }
                None => {
                    w.text(p.boards2_id, 0, "");
                    w.text(p.boards2_ip, 0, "");
                    labels(w, board1, None);
                }
            }
        }
        None => {
            w.text(p.boards_id, 0, "");
            w.text(p.boards_ip, 0, "");
            labels(w, board0, None);
            w.text(p.boards2_id, 0, "");
            w.text(p.boards2_ip, 0, "");
            labels(w, board1, None);
        }
    }

    w.int(p.supp_acq_modes, 0, json_int_or(info.get("SuppAcqModes")));
    w.float(p.clock_readout, 0, json_double_or(info.get("ClockReadout")));
    w.int(p.max_pulse_count, 0, json_int_or(info.get("MaxPulseCount")));
    for (key, reason) in [
        ("MaxPulseHeight", p.max_pulse_height),
        ("MaxPulsePeriod", p.max_pulse_period),
        ("TimerMaxVal", p.timer_max_val),
        ("TimerMinVal", p.timer_min_val),
        ("TimerStep", p.timer_step),
        ("ClockTimepix", p.clock_timepix),
    ] {
        w.float(reason, 0, json_double_or(info.get(key)));
    }

    // The Config readback (serval_http.cpp:1431-1508).
    let c = |key: &str| cfg.get(key);
    w.int(p.fan1_pw_m, 0, json_int_or(c("Fan1PWM")));
    w.int(p.fan2_pw_m, 0, json_int_or(c("Fan2PWM")));
    w.int(p.bias_volt, 0, json_int_or(c("BiasVoltage")));
    w.int(p.bias_enable, 0, json_bool_or(c("BiasEnabled")).into());
    w.int(p.trigger_in, 0, json_int_or(c("TriggerIn")));
    w.int(p.trigger_out, 0, json_int_or(c("TriggerOut")));
    if let Some(v) = c("Polarity") {
        w.int(
            p.polarity,
            0,
            (serval::json_to_string(v) == "Negative").into(),
        );
    }
    if let Some(v) = c("ChainMode") {
        let chain = match serval::json_to_string(v).as_str() {
            "LEADER" => 1,
            "FOLLOWER" => 2,
            _ => 0,
        };
        w.int(p.chain_mode, 0, chain);
    }
    if let Some(v) = c("TriggerMode") {
        let mode = serval::json_to_string(v);
        w.text(p.trigger_mode, 0, &mode);
        w.int(
            ad.trigger_mode,
            0,
            serval::trigger_mode_index_from_serval(&mode),
        );
    }
    let exposure = json_double_or(c("ExposureTime"));
    w.float(p.exposure_time, 0, exposure);
    w.float(ad.acquire_time, 0, exposure);
    let period = json_double_or(c("TriggerPeriod"));
    w.float(p.trigger_period, 0, period);
    w.float(ad.acquire_period, 0, period);
    let triggers = json_int_or(c("nTriggers"));
    w.int(p.n_triggers, 0, triggers);
    w.int(ad.num_images, 0, triggers);
    w.int(p.periph_clk80, 0, json_bool_or(c("PeriphClk80")).into());
    w.float(p.trigger_delay, 0, json_double_or(c("TriggerDelay")));
    let tdc = c("Tdc").unwrap_or(&JSON_NULL);
    w.text(p.tdc, 0, &serval::format_tdc(tdc));
    if let Some((tdc0, tdc1)) = serval::parse_tdc(tdc) {
        w.int(p.tdc0, 0, tdc0);
        w.int(p.tdc1, 0, tdc1);
    }
    w.float(
        p.global_timestamp_interval,
        0,
        json_double_or(c("GlobalTimestampInterval")),
    );
    w.int(
        p.external_reference_clock,
        0,
        json_bool_or(c("ExternalReferenceClock")).into(),
    );
    w.int(p.log_level, 0, json_int_or(c("LogLevel")));
    if let Some(v) = c("BothCounters") {
        w.int(p.both_counters, 0, json_bool_or(Some(v)).into());
    }
    if let Some(v) = c("GainMode") {
        w.int(
            p.gain_mode,
            0,
            serval::gain_mode_from_string(&serval::json_to_string(v)),
        );
    }
    if let Some(v) = c("ChargeSumming") {
        w.int(p.charge_summing, 0, json_bool_or(Some(v)).into());
    }
    if let Some(v) = c("Colour") {
        w.int(p.colour, 0, json_bool_or(Some(v)).into());
    }
    if let Some(v) = c("PixelDepth") {
        w.int(p.pixel_depth, 0, serval::pixel_depth_from_json(v));
    }
    if let Some(v) = c("CounterSelectIn") {
        w.int(p.counter_select_in, 0, json_int_or(Some(v)));
    }
    if let Some(v) = c("CounterSelectOut") {
        w.int(p.counter_select_out, 0, json_int_or(Some(v)));
    }
    if let Some(Value::Array(delays)) = c("IDelayConfig") {
        for (reason, v) in [p.i_delay0, p.i_delay1, p.i_delay2, p.i_delay3]
            .into_iter()
            .zip(delays)
        {
            w.int(reason, 0, json_int_or(Some(v)));
        }
    }

    let family = shared.family();
    fetch_dacs(w, p, detector, 0, family)?;
    // The detector orientation drives the mask geometry, so it also goes into
    // the shared state the mask code reads.
    if let Some(name) = detector
        .get("Layout")
        .filter(|l| l.is_object())
        .and_then(|l| l.get("DetectorOrientation"))
    {
        let o = serval::orientation_index(&serval::json_to_string(name));
        shared.set_orientation(o);
        w.int(p.detector_orientation, 0, o);
    }
    let layout = layout_chips(detector, family);
    let publish_layout = |w: &mut DetectorWrites, chip: i32| {
        if let Some(entry) = layout.and_then(|l| chip_layout_entry(l, chip)) {
            w.text(p.layout, chip, &serval::dump(entry, None, false));
        }
    };
    publish_layout(w, 0);
    w.callbacks(0);
    for chip in 1..chips {
        fetch_dacs(w, p, detector, chip, family)?;
        publish_layout(w, chip);
        w.callbacks(chip);
    }
    w.int(p.det_connected, 0, 1);
    w.callbacks(0);
    Ok(())
}

/// The chip layouts C getDetector publishes (serval_http.cpp:1525-1542):
/// MPX3 assembles its images in the `Rotated` layout, so it prefers it;
/// otherwise `Original`, falling back to `Rotated`.
fn layout_chips(detector: &Value, family: DetectorFamily) -> Option<&Vec<Value>> {
    let chips = |name: &str| {
        detector
            .pointer(&format!("/Layout/{name}/Chips"))
            .and_then(Value::as_array)
    };
    (family == DetectorFamily::Mpx3)
        .then(|| chips("Rotated"))
        .flatten()
        .or_else(|| chips("Original"))
        .or_else(|| chips("Rotated"))
}

/// C `publishChipLayout`: the entry whose integer `Chip` is `chip`, not the
/// entry at that position.
fn chip_layout_entry(chips: &[Value], chip: i32) -> Option<&Value> {
    chips.iter().find(|entry| {
        entry
            .get("Chip")
            .filter(|c| c.is_i64() || c.is_u64())
            .and_then(Value::as_i64)
            == Some(i64::from(chip))
    })
}

/// C `updateDetectorFamily` (ADTimePix.cpp:2107-2127), from getDetector's
/// `Info` (serval_http.cpp:1353-1363).
fn update_detector_family(
    w: &mut DetectorWrites,
    shared: &Shared,
    p: &TimePixParams,
    detector: &Value,
) {
    let info = &detector["Info"];
    let mpx_type = info.get("MpxType").and_then(Value::as_i64).unwrap_or(0) as i32;
    let chip_type = info
        .get("ChipType")
        .map(serval::json_to_string)
        .unwrap_or_default();
    let chipboard_id = info
        .get("Boards")
        .and_then(Value::as_array)
        .and_then(|b| b.first())
        .and_then(|b0| b0.get("ChipboardId"))
        .map(serval::json_to_string)
        .unwrap_or_default();
    let family = DetectorFamily::detect(mpx_type, &chip_type, &chipboard_id);
    let caps = family.capabilities();
    w.text(p.chip_type, 0, &chip_type);
    w.int(p.detector_family, 0, family as i32);
    w.int(p.cap_tdc, 0, caps.supports_tdc.into());
    w.int(p.cap_tof_hist, 0, caps.supports_tof_histogram.into());
    w.int(p.cap_dual_preview, 0, caps.supports_dual_preview.into());
    w.int(
        p.cap_img_thresholds,
        0,
        caps.supports_image_thresholds.into(),
    );

    if shared.note_family(family) {
        apply_family_defaults(p, family, &mut w.pending);
        w.trace(
            TraceMask::FLOW,
            adlog::log(
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

/// C `updateDetectorHealthFromJson` (serval_http.cpp:710-815): Serval 3
/// sends `Health` as one object, Serval 4 as one object per SPIDR board.
fn update_detector_health(w: &mut DetectorWrites, p: &TimePixParams, detector: &Value) {
    const F: &str = adlog::UPDATE_DETECTOR_HEALTH_FROM_JSON;
    let Some(root) = detector.get("Health") else {
        return;
    };
    let dump = |v: &Value| serval::dump(v, None, false);
    let scalars = |w: &mut DetectorWrites, h: &Value| {
        for (key, reason) in [
            ("LocalTemperature", p.local_temp),
            ("FPGATemperature", p.fp_ga_temp),
            ("Fan1Speed", p.fan1_speed),
            ("Fan2Speed", p.fan2_speed),
            ("BiasVoltage", p.bias_voltage),
        ] {
            w.float(reason, 0, json_double_or(h.get(key)));
        }
        // C's local jsonInt reads only an integer.
        let humidity = match h.get("Humidity") {
            Some(Value::Number(n)) if n.is_i64() || n.is_u64() => json_number_as_int(n),
            _ => 0,
        };
        w.int(p.humidity, 0, humidity);
    };
    // Three rails per board: board 0 at addresses 0-2, board 1 at 3-5.
    let rails = |w: &mut DetectorWrites, h: &Value, first: i32| {
        for (i, addr) in (first..first + 3).enumerate() {
            for (key, reason) in [("VDD", p.chip_nv_dd), ("AVDD", p.chip_na_vd_d)] {
                let rail = h.get(key).and_then(Value::as_array).and_then(|a| a.get(i));
                if let Some(v) = rail.and_then(Value::as_f64) {
                    w.float(reason, addr, v);
                }
            }
        }
    };
    match root {
        Value::Array(blocks) => {
            let Some(h0) = blocks.first() else {
                w.trace(
                    TraceMask::IO_DRIVER,
                    adlog::log(F, "getDetector: Health array is empty"),
                );
                return;
            };
            if !h0.is_object() {
                w.trace(
                    TraceMask::IO_DRIVER,
                    adlog::log(F, "getDetector: Health[0] is not an object"),
                );
                return;
            }
            scalars(w, h0);
            let temps: Vec<Value> = blocks
                .iter()
                .filter_map(|b| b.get("ChipTemperatures")?.as_array())
                .flatten()
                .cloned()
                .collect();
            if !temps.is_empty() {
                w.text(p.chip_temperature, 0, &dump(&Value::Array(temps)));
            } else if let Some(t) = h0.get("ChipTemperatures") {
                w.text(p.chip_temperature, 0, &dump(t));
            }
            for (key, reason) in [("VDD", p.vd_d), ("AVDD", p.av_dd)] {
                if blocks.len() > 1 {
                    let merged = blocks
                        .iter()
                        .filter_map(|b| b.get(key).filter(|v| v.is_array()))
                        .cloned()
                        .collect();
                    w.text(reason, 0, &dump(&Value::Array(merged)));
                } else if let Some(v) = h0.get(key) {
                    w.text(reason, 0, &dump(v));
                }
            }
            rails(w, h0, 0);
            match blocks.get(1) {
                Some(h1) => {
                    if h1.is_object() {
                        rails(w, h1, 3);
                    }
                }
                None => {
                    for addr in 3..6 {
                        w.float(p.chip_nv_dd, addr, 0.0);
                        w.float(p.chip_na_vd_d, addr, 0.0);
                    }
                }
            }
        }
        Value::Object(_) => {
            scalars(w, root);
            for (key, reason) in [
                ("ChipTemperatures", p.chip_temperature),
                ("VDD", p.vd_d),
                ("AVDD", p.av_dd),
            ] {
                if let Some(v) = root.get(key) {
                    w.text(reason, 0, &dump(v));
                }
            }
            rails(w, root, 0);
        }
        other => w.trace(
            TraceMask::ERROR,
            adlog::err(
                F,
                &format!(
                    "getDetector: Health has unexpected JSON type ({})",
                    json_type_name(other)
                ),
            ),
        ),
    }
}

/// C `chipTemperatureFromHealthV4` / `V3` (serval_http.cpp:671-703): on
/// Serval 4 each board lists only its own chips, so board 1's first entry
/// is global chip 4. A non-integer is truncated.
fn chip_temperature(detector: &Value, chip: usize) -> i32 {
    let entry = match detector.get("Health") {
        Some(Value::Array(blocks)) => blocks
            .iter()
            .filter_map(|b| b.get("ChipTemperatures")?.as_array())
            .flatten()
            .nth(chip),
        Some(h @ Value::Object(_)) => h
            .get("ChipTemperatures")
            .and_then(Value::as_array)
            .and_then(|t| t.get(chip)),
        _ => None,
    };
    match entry {
        Some(Value::Number(n)) => json_number_as_int(n),
        _ => 0,
    }
}

/// C `fetchDacs`' DAC order (serval_http.cpp:854-871), which decides the
/// DACs already stored when one is not a number.
const FETCH_DACS_ORDER: [&str; 18] = [
    "Ibias_CP_PLL",
    "Ibias_DiscS1_OFF",
    "Ibias_DiscS1_ON",
    "Ibias_DiscS2_OFF",
    "Ibias_DiscS2_ON",
    "Ibias_Ikrum",
    "Ibias_PixelDAC",
    "Ibias_Preamp_OFF",
    "Ibias_Preamp_ON",
    "Ibias_TPbufferIn",
    "Ibias_TPbufferOut",
    "PLL_Vcntrl",
    "VPreamp_NCAS",
    "VTP_coarse",
    "VTP_fine",
    "Vfbk",
    "Vthreshold_coarse",
    "Vthreshold_fine",
];

/// C `fetchDacs` (serval_http.cpp:835-884): chip `chip`'s DACs, temperature
/// and Adjust shape at address `chip`. Medipix3, or a chip without the
/// Timepix3 DACs, gets only the temperature.
fn fetch_dacs(
    w: &mut DetectorWrites,
    p: &TimePixParams,
    detector: &Value,
    chip: i32,
    family: DetectorFamily,
) -> Result<(), JsonTypeError> {
    let index = usize::try_from(chip).unwrap_or(usize::MAX);
    let Some(entry) = detector
        .get("Chips")
        .and_then(Value::as_array)
        .and_then(|c| c.get(index))
    else {
        return Ok(());
    };
    // C reads `Chips[chip]["DACs"]` through the mutable operator[], which
    // turns a null into an object and throws on any other non-object.
    let dacs = match entry {
        Value::Object(o) => o.get("DACs").and_then(Value::as_object),
        Value::Null => None,
        other => {
            return Err(JsonTypeError::new(
                305,
                "cannot use operator[] with a string argument with ",
                other,
            ));
        }
    };
    let temperature = chip_temperature(detector, index);
    let dacs = match dacs.filter(|d| d.contains_key("Ibias_Preamp_ON")) {
        Some(dacs) if family != DetectorFamily::Mpx3 => dacs,
        _ => {
            w.int(p.chip_n_temperature, chip, temperature);
            w.int(p.adjust, chip, -1);
            return Ok(());
        }
    };
    let params = crate::driver::dac_params(p);
    for name in FETCH_DACS_ORDER {
        let value = json_get_int(dacs.get(name))?;
        if let Some(&(_, reason)) = params.iter().find(|(n, _)| *n == name) {
            w.int(reason, chip, value);
        }
    }
    w.int(p.chip_n_temperature, chip, temperature);
    let adjust = match entry.get("Adjust") {
        None | Some(Value::Null) => -1,
        Some(Value::Array(_)) => -2,
        Some(_) => -3,
    };
    w.int(p.adjust, chip, adjust);
    Ok(())
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
        updates.push((p.status, ParamValue::Octet(status.as_bytes().into())));
        updates.push((
            p.pipeline_state,
            ParamValue::Int32(pipeline_state_index(status)),
        ));
    }
    updates
}

/// One `GET /measurement`, printing C `timePixCallback`'s line for a failure;
/// the error is the reason the acquisition rolls back with.
fn poll_measurement(ctx: &Ctx, first: bool) -> Result<serval::StatusSnapshot, &'static str> {
    match ctx.http.get(serval::MEASUREMENT, DEFAULT_TIMEOUT) {
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
    /// lines carry: connect, the shared read loop, header parse. C's PrvHst
    /// code prints with `printf`/`fprintf` instead, so it has none.
    fn c_functions(self) -> Option<[&'static str; 3]> {
        match self {
            Self::PrvImg => Some([
                "void ADTimePix::prvImgConnect()",
                RUN_JSON_IMAGE_TCP_WORKER,
                "bool ADTimePix::processPrvImgDataLine(char*, char*, size_t)",
            ]),
            Self::Img => Some([
                "void ADTimePix::imgConnect()",
                RUN_JSON_IMAGE_TCP_WORKER,
                "bool ADTimePix::processImgDataLine(char*, char*, size_t)",
            ]),
            Self::PrvHst => None,
        }
    }

    fn worker(self, s: &Shared) -> &WorkerState {
        match self {
            Self::PrvImg => &s.prv_img_worker,
            Self::Img => &s.img_worker,
            Self::PrvHst => &s.prv_hst_worker,
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

/// g++'s `__PRETTY_FUNCTION__` for C `runJsonImageTcpWorker`, the image
/// channels' read loop (serval_stream.cpp:484-495).
const RUN_JSON_IMAGE_TCP_WORKER: &str = "void ADTimePix::runJsonImageTcpWorker(epicsMutexId, \
ADTimePix3StreamWorker::State&, std::string&, int&, std::shared_ptr<NetworkClient>&, \
std::vector<char>&, size_t&, void (ADTimePix::*)(), void (ADTimePix::*)(), \
bool (ADTimePix::*)(char*, char*, size_t), const char*)";

/// One worker. C `runJsonImageTcpWorker` / `prvHstWorkerThread`
/// (serval_stream.cpp:483-645, histogram_io.cpp:957-1124) run once per
/// acquisition start: they retry the connect every second until it succeeds,
/// and a stream that ends or fails ends the run for the rest of that
/// acquisition.
///
/// C starts these threads per acquisition (`startStreamWorker`,
/// acquire.cpp:132-157) and joins them in `stopAndJoinStreamWorkers`
/// (:159-199); here each worker is created once and parks between runs.
async fn stream_worker(ctx: Arc<Ctx>, kind: StreamKind) {
    let worker = kind.worker(&ctx.shared);
    let mut served = None;
    loop {
        let run = worker.wait_for_start(served);
        served = Some(run);
        stream_run(&ctx, kind, run).await;
        match kind.c_functions() {
            Some([_, thread, _]) => ctx.trace(
                TraceMask::IO_DRIVER,
                &adlog::log(thread, &format!("{} worker thread exiting", kind.name())),
            ),
            None => println!("PrvHst worker thread exiting"),
        }
    }
}

async fn stream_run(ctx: &Arc<Ctx>, kind: StreamKind, run: Generation) {
    let worker = kind.worker(&ctx.shared);
    let name = kind.name();
    while worker.running(run) {
        // C parses the path at its check and skips the connect while the
        // host is empty (serval_stream.cpp:548-550, histogram_io.cpp:1004-1008).
        let Some((host, port)) = kind
            .path(&ctx.shared)
            .and_then(|p| parse_tcp_path(&p).ok())
            .filter(|(host, _)| !host.is_empty())
        else {
            worker.wait_for_stop_for(run, RECONNECT_WAIT);
            continue;
        };
        if kind.c_functions().is_none() {
            println!("PrvHst worker thread: Attempting to connect to {host}:{port}");
        }
        let stream = match TcpStream::connect((host.as_str(), port)) {
            Ok(stream) if worker.mark_connected(run, &stream) => stream,
            Ok(_) => break,
            Err(_) => {
                if worker.running(run) {
                    let msg = format!("{name} TCP failed to connect to {host}:{port}");
                    match kind.c_functions() {
                        Some([connect, ..]) => {
                            ctx.trace(TraceMask::ERROR, &adlog::err(connect, &msg))
                        }
                        None => eprintln!("ERROR | ADTimePix::prvHstConnect: {msg}"),
                    }
                }
                disconnected(ctx, kind);
                worker.wait_for_stop_for(run, RECONNECT_WAIT);
                continue;
            }
        };
        let _ = stream.set_read_timeout(Some(READ_TIMEOUT));
        let msg = format!("{name} TCP connected to {host}:{port}");
        match kind.c_functions() {
            Some([connect, ..]) => ctx.trace(TraceMask::IO_DRIVER, &adlog::log(connect, &msg)),
            None => println!("{msg}"),
        }
        read_stream(ctx, kind, run, stream).await;
        worker.fail(run);
        worker.mark_disconnected(run);
        disconnected(ctx, kind);
    }
}

/// C `prvImgDisconnect` / `imgDisconnect` / `prvHstDisconnect` closing a
/// client (serval_stream.cpp:1648-1706, histogram_io.cpp:944-953).
fn disconnected(ctx: &Ctx, kind: StreamKind) {
    match kind {
        StreamKind::PrvImg => ctx.trace(
            TraceMask::IO_DRIVER,
            &adlog::log(
                "void ADTimePix::prvImgDisconnect()",
                "PrvImg TCP disconnected",
            ),
        ),
        StreamKind::Img => {}
        StreamKind::PrvHst => println!("PrvHst TCP disconnected"),
    }
}

async fn read_stream(ctx: &Arc<Ctx>, kind: StreamKind, run: Generation, mut stream: TcpStream) {
    let mut decoder = FrameDecoder::new(kind.channel());
    let mut buf = vec![0u8; crate::stream::MAX_BUFFER_SIZE];

    let worker = kind.worker(&ctx.shared);
    while worker.running(run) {
        // C leaves the histogram socket unread while accumulation is off
        // (histogram_io.cpp:1018-1023).
        if kind == StreamKind::PrvHst && !prv_hst_accumulating(ctx) {
            worker.wait_for_stop_for(run, PRV_HST_PAUSED_WAIT);
            continue;
        }
        match stream.read(&mut buf) {
            Ok(0) => {
                let end = decoder.end_of_stream();
                if end != EndOfStream::Clean && worker.running(run) {
                    let msg = format!("{} TCP stream ended with {}", kind.name(), end.message());
                    match kind.c_functions() {
                        Some([_, worker, _]) => {
                            ctx.trace(TraceMask::ERROR, &adlog::err(worker, &msg))
                        }
                        None => eprintln!("ERROR | ADTimePix::prvHstWorkerThread: {msg}"),
                    }
                }
                println!("{} TCP connection closed by peer", kind.name());
                return;
            }
            Ok(n) => {
                if let Err(e) = decoder.push(&buf[..n]) {
                    framing_failed(ctx, kind, &e);
                    return;
                }
            }
            // C `isReceiveTimeout` (network_client.cpp:236-238).
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock && worker.running(run) => continue,
            Err(e) => {
                if worker.running(run) {
                    let msg = format!("{} TCP socket error: {}", kind.name(), strerror(&e));
                    match kind.c_functions() {
                        Some([_, worker, _]) => {
                            ctx.trace(TraceMask::IO_DRIVER, &adlog::log(worker, &msg))
                        }
                        None => eprintln!("{msg}"),
                    }
                }
                return;
            }
        }

        while worker.running(run) {
            if kind.channel() == Channel::Image {
                decoder.set_image_limits(image_limits(ctx));
            }
            match decoder.next_frame() {
                Ok(Some(frame)) => {
                    if !handle_frame(ctx, kind, frame).await {
                        return;
                    }
                }
                Ok(None) => break,
                Err(e) => {
                    framing_failed(ctx, kind, &e);
                    return;
                }
            }
        }
    }
}

/// C `ADTimePixPrvHstAccumulationEnable`, which gates the PrvHst worker and
/// both of its frame handlers.
fn prv_hst_accumulating(ctx: &Ctx) -> bool {
    ctx.mirrored(Mirrored::PrvHstAccumulationEnable) != 0
}

/// The window half of C `processImgFrame` / `processPrvHstFrame`
/// (serval_stream.cpp:1005-1043, :1132-1164; histogram_io.cpp:356-395,
/// :607-632): a reconfiguration posts the effective frames and the retention
/// status, a refused frame its failure, and every frame FramesSummed.
async fn post_window(ctx: &Ctx, which: Accum, update: &Update) {
    let p = &ctx.p;
    let (effective, summed, status, tag, process) = match which {
        Accum::Img => (
            p.img_effective_frames,
            p.img_frames_summed,
            p.img_retention_status,
            "Img",
            adlog::PROCESS_IMG_FRAME,
        ),
        Accum::PrvHst => (
            p.prv_hst_effective_frames,
            p.prv_hst_frames_summed,
            p.prv_hst_retention_status,
            "PrvHst",
            adlog::PROCESS_PRV_HST_FRAME,
        ),
    };
    let int = |reason, v: usize| {
        ParamSetValue::new(
            reason,
            ADDR,
            ParamValue::Int32(i32::try_from(v).unwrap_or(i32::MAX)),
        )
    };
    let text = |reason, t: String| {
        ParamSetValue::new(reason, ADDR, ParamValue::Octet(t.into_bytes().into()))
    };
    let mut updates = Vec::new();
    if let Some(c) = update.window_config {
        let report = which.window_report(&c);
        if let Some(warning) = &report.warning {
            ctx.trace(TraceMask::WARNING, warning);
        }
        updates.extend([
            int(effective, report.effective),
            text(status, report.status),
        ]);
    }
    if let Some(push) = update.window_push.filter(|&s| s != RollingSumStatus::Ok) {
        if push == RollingSumStatus::NumericOverflow {
            let range = which
                .get(&ctx.shared)
                .lock()
                .report_range(RangeState::AccumulatorSaturated);
            let (saturated, range_status) = which.range_params(p);
            ctx.post_range(Locking::Unlocked, saturated, range_status, range)
                .await;
        }
        let message = format!("Update failed: {}", push.name());
        ctx.trace(
            TraceMask::ERROR,
            &adlog::err(
                process,
                &format!("{tag} rolling window update failed: {message}"),
            ),
        );
        updates.push(text(status, message));
    }
    updates.push(int(summed, update.frames_summed));
    ctx.post(ADDR, updates);
}

/// C `IMG_MEMORY_UPDATE_INTERVAL_SEC` / `PRVHST_MEMORY_UPDATE_INTERVAL_SEC`
/// (ADTimePix.h:986, :1038).
const MEMORY_UPDATE_INTERVAL_SEC: f64 = 5.0;

/// C `MAX_BUFFER_SIZE` (ADTimePix.h:415), the PrvHst worker's line buffer.
pub(crate) const PRV_HST_LINE_BUFFER: usize = 32768;

/// The processing-time and memory tail of C `processImgFrame` /
/// `updateImgPerformanceMetrics` (serval_stream.cpp:1222-1255, :1346-1375)
/// and `processPrvHstFrame` (histogram_io.cpp:848-881).
async fn post_metrics(ctx: &Ctx, which: Accum, started: f64) {
    let now = EpicsTimestamp::now().as_f64();
    let rate_samples = ctx.shared.rates.lock().prv_hst.sample_count();
    let (time, memory) = {
        let mut acc = which.get(&ctx.shared).lock();
        let time = acc.record_processing_time((now - started) * 1e3, now);
        let memory = acc
            .memory_due(now, MEMORY_UPDATE_INTERVAL_SEC)
            .then(|| match which {
                Accum::Img => img_memory_mb(&acc),
                Accum::PrvHst => prv_hst_memory_mb(&acc, rate_samples, PRV_HST_LINE_BUFFER),
            });
        (time, memory)
    };
    let p = &ctx.p;
    let (time_param, memory_param) = match which {
        Accum::Img => (p.img_processing_time, p.img_memory_usage),
        Accum::PrvHst => (p.prv_hst_processing_time, p.prv_hst_memory_usage),
    };
    let updates: Vec<_> = time
        .map(|t| (time_param, t))
        .into_iter()
        .chain(memory.map(|m| (memory_param, m)))
        .map(|(reason, v)| ParamSetValue::new(reason, ADDR, ParamValue::Float64(v)))
        .collect();
    if !updates.is_empty() {
        ctx.post(ADDR, updates);
    }
}

/// C `calculateImgMemoryUsageMB` (serval_stream.cpp:1377-1415). Before the
/// first frame C's current frame is the constructor's 512x512 UINT16
/// (ADTimePix.cpp:1441).
pub(crate) fn img_memory_mb(img: &crate::accum::Accumulator) -> f64 {
    const MIB: f64 = 1024.0 * 1024.0;
    let (current, value_bytes) = img
        .current_frame()
        .map_or((512 * 512, 2), |c| (c.len(), img.value_bytes()));
    let running = img.elements();
    let max_pixels = if running > 0 { running } else { current };
    (running * 8 + current * value_bytes + img.window().memory_bytes()) as f64 / MIB
        + (max_pixels * (8 + 8 + 4)) as f64 / MIB
        + 0.1
}

/// C `calculatePrvHstMemoryUsageMB` (histogram_io.cpp:397-418); the line
/// buffer is `PRV_HST_LINE_BUFFER` once the worker has run, else empty.
pub(crate) fn prv_hst_memory_mb(
    hst: &crate::accum::Accumulator,
    rate_samples: usize,
    line_buffer: usize,
) -> f64 {
    const MIB: f64 = 1024.0 * 1024.0;
    let bins = hst.elements();
    let running = if bins > 0 {
        bins * 8 + (bins + 1) * 8
    } else {
        0
    };
    (running
        + hst.time_axis_len() * 8
        + hst.window().memory_bytes()
        + (rate_samples + hst.processing_samples()) * 8
        + line_buffer) as f64
        / MIB
        + 0.1
}

/// C `detectorImageFrameLimits` from the detector params
/// (serval_stream.cpp:520-527).
fn image_limits(ctx: &Ctx) -> ImageLimits {
    ImageLimits::detector(
        ctx.mirrored(Mirrored::MaxSizeX),
        ctx.mirrored(Mirrored::MaxSizeY),
        ctx.mirrored(Mirrored::PixCount),
    )
}

/// C's `rejected TCP stream` line for a framing failure
/// (serval_stream.cpp:589-611, histogram_io.cpp:1060-1083).
fn framing_failed(ctx: &Ctx, kind: StreamKind, e: &FrameError) {
    let msg = format!("{} rejected TCP stream: {e}", kind.name());
    match kind.c_functions() {
        Some([_, worker, _]) => ctx.trace(TraceMask::ERROR, &adlog::err(worker, &msg)),
        None => eprintln!("ERROR | ADTimePix::prvHstWorkerThread: {msg}"),
    }
}

/// C `processPreviewJsonimageLine`'s header log (serval_stream.cpp:142-151):
/// the first PrvImgLogHeaders headers after a start, cut at 1024 bytes.
fn log_json_header(ctx: &Ctx, s: Preview, header: &Value) {
    {
        let mut state = ctx.shared.preview[s.index].lock();
        if state.json_headers <= 0 {
            return;
        }
        state.json_headers -= 1;
    }
    const MAX_HEADER_LOG: usize = 1024;
    let mut text = serval::dump(header, None, false);
    if text.len() > MAX_HEADER_LOG {
        // C cuts bytes; a cut inside a UTF-8 character backs off to its start.
        let mut cut = MAX_HEADER_LOG;
        while !text.is_char_boundary(cut) {
            cut -= 1;
        }
        text.truncate(cut);
        text.push_str("...");
    }
    ctx.trace(
        TraceMask::IO_DRIVER,
        &adlog::log(
            adlog::PROCESS_PREVIEW_JSONIMAGE_LINE,
            &format!("{} jsonimage header: {text}", s.tag),
        ),
    );
}

/// C `strerror(errno)`: the OS text without Rust's " (os error N)" suffix.
fn strerror(e: &std::io::Error) -> String {
    let text = e.to_string();
    match text.find(" (os error ") {
        Some(end) => text[..end].to_string(),
        None => text,
    }
}

/// C's `processLineFn`: `false` (a refused NDArray allocation) fails the
/// worker as a framing failure does (serval_stream.cpp:618-626, :639-644).
async fn handle_frame(ctx: &Arc<Ctx>, kind: StreamKind, frame: Frame) -> bool {
    match (kind, frame) {
        (StreamKind::PrvImg, Frame::Image(f)) => return preview_frame(ctx, PRV_IMG, f).await,
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
                return false;
            };
            let p = ctx.p;
            // processImgDataLine counts every Img frame, NDArrayCallbacks or
            // not, before getAttributes (serval_stream.cpp:880-883).
            let counter = ctx.mirrored(Mirrored::ArrayCounter) + 1;
            let rate = frame_rate(
                ctx,
                f.frame_number,
                |r| &mut r.img,
                adlog::PROCESS_IMG_DATA_LINE,
                "Img ",
            );
            ctx.post(
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
            );
            attach_attributes(ctx, &mut array, &f).await;

            // processImgFrame runs after getAttributes, on threshold 0 only and
            // only while ImgAccumulationEnable is set (serval_stream.cpp:948-982).
            let enabled = ctx.mirrored(Mirrored::ImgAccumulationEnable) != 0;
            if enabled && f.threshold_id != 1 {
                let started = EpicsTimestamp::now().as_f64();
                // C's ImageData holds a uint8 frame as UINT16.
                let value_bytes = f.format.bytes().max(2);
                let update =
                    ctx.shared
                        .img
                        .lock()
                        .add_shaped(&f.pixels, &[f.width, f.height], value_bytes);
                post_window(ctx, Accum::Img, &update).await;
                let (counts, frames) = {
                    let img = ctx.shared.img.lock();
                    (img.total_counts(), img.frame_count())
                };
                post_metrics(ctx, Accum::Img, started).await;
                ctx.post(
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
                );
                ctx.post_int64(
                    p.img_total_counts,
                    i64::try_from(counts).unwrap_or(i64::MAX),
                );

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
            let addr = threshold_addr(f.threshold_id, IMG_ADDR, IMG_THRESHOLD1_ADDR);
            publish_image(ctx, addr, array).await;
            ctx.trace(
                TraceMask::IO_DRIVER,
                &adlog::log(
                    adlog::PROCESS_IMG_DATA_LINE,
                    &format!(
                        "Processed Img frame: width={}, height={}, format={}, frame={}, thresholdID={}, addr={addr}, counter={counter}",
                        f.width,
                        f.height,
                        f.format.name(),
                        f.frame_number,
                        f.threshold_id
                    ),
                ),
            );
        }
        (StreamKind::PrvHst, Frame::Histogram(h)) => {
            // histogram_io.cpp:165-170, :425-433: a frame read before
            // accumulation was turned off is dropped whole.
            if !prv_hst_accumulating(ctx) {
                return true;
            }
            let started = EpicsTimestamp::now().as_f64();
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
                let mut hst = ctx.shared.hst.lock();
                hst.grow_time_axis(h.counts.len() + 1);
                (hst.frame_count(), hst.total_counts())
            };

            ctx.post(
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
            );
            ctx.post(
                ADDR,
                vec![ParamSetValue::new(
                    p.prv_hst_histogram_time_ms,
                    ADDR,
                    ParamValue::Float64Array(h.time_axis_ms().into()),
                )],
            );
            ctx.post_int64(
                p.prv_hst_total_counts,
                i64::try_from(counts).unwrap_or(i64::MAX),
            );
            post_window(ctx, Accum::PrvHst, &update).await;

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
            if ctx.mirrored(Mirrored::ArrayCallbacks) != 0 {
                let time_ms = h.time_axis_ms();
                let arrays = HstArrays {
                    window: update.sum_n.map(|w| (w, update.frames_summed as u64)),
                    running: update.running,
                    frame_count: frames,
                    current: Some(h.counts),
                    time_ms,
                };
                publish_hst_arrays(ctx, Locking::Unlocked, arrays, SumOutput::Sum).await;
            }
            post_metrics(ctx, Accum::PrvHst, started).await;
        }
        // The decoder is created for the channel, so this cannot happen.
        _ => {}
    }
    true
}

/// C `processPreviewJsonimageLine` (serval_stream.cpp:88-361) after the
/// framing: the frame goes to its threshold's address, and a T1/T0 pair
/// makes the band-pass. `false` is a refused allocation.
async fn preview_frame(ctx: &Arc<Ctx>, s: Preview, mut f: crate::stream::ImageFrame) -> bool {
    log_json_header(ctx, s, &f.header);
    let slot = usize::from(f.threshold_id == 1);
    let addr = threshold_addr(f.threshold_id, s.th0, s.th1);
    // C releases pArrays[addr] before allocating its replacement (:165-171).
    ctx.shared.preview[s.index].lock().last[slot] = None;
    let pixels = std::mem::take(&mut f.pixels);
    let Some(mut array) = image_array(
        ctx,
        &f,
        pixels,
        &adlog::err(
            adlog::PROCESS_PREVIEW_JSONIMAGE_LINE,
            &format!("{} failed to allocate NDArray", s.tag),
        ),
    ) else {
        return false;
    };
    if s.metadata {
        let p = ctx.p;
        let rate = frame_rate(
            ctx,
            f.frame_number,
            |r| &mut r.prv_img,
            adlog::PROCESS_PREVIEW_JSONIMAGE_LINE,
            "PrvImg ",
        );
        ctx.post(
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
                .chain(
                    rate.map(|r| {
                        ParamSetValue::new(p.prv_img_acq_rate, ADDR, ParamValue::Float64(r))
                    }),
                )
                .collect(),
        );
    }
    attach_attributes(ctx, &mut array, &f).await;
    let array = array.share();
    ctx.shared.preview[s.index].lock().last[slot] = Some(array.clone());
    publish_image(ctx, addr, array).await;

    let pair_done = {
        let mut state = ctx.shared.preview[s.index].lock();
        if state.last_seen_frame >= 0 && f.frame_number.wrapping_add(10) < state.last_seen_frame {
            state.t1_ready = false;
            state.t0_orphan = false;
            state.last_diff_t0 = -1;
        }
        state.last_seen_frame = f.frame_number;
        match f.threshold_id {
            1 if state.t0_orphan => {
                state.t0_orphan = false;
                true
            }
            1 => {
                state.t1_ready = true;
                false
            }
            0 if state.t1_ready => {
                state.t1_ready = false;
                true
            }
            0 => {
                state.t0_orphan = true;
                false
            }
            _ => false,
        }
    };
    if pair_done {
        emit_preview_threshold_diff(ctx, s, f.frame_number).await;
    }
    ctx.trace(
        TraceMask::IO_DRIVER,
        &adlog::log(
            adlog::PROCESS_PREVIEW_JSONIMAGE_LINE,
            &format!(
                "Processed {} frame: width={}, height={}, format={}, frame={}, thresholdID={}",
                s.tag,
                f.width,
                f.height,
                f.format.name(),
                f.frame_number,
                f.threshold_id
            ),
        ),
    );
    true
}

/// C `emitPreviewThresholdDiff` (serval_stream.cpp:366-468): T0 - T1 of the
/// stream's last two frames as Int32, clipped at 0 when PrvImgThreshDiffClip.
async fn emit_preview_threshold_diff(ctx: &Ctx, s: Preview, frame_number: i32) {
    use epics_rs::ad_core::ndarray::NDDataType;
    let func = adlog::EMIT_PREVIEW_THRESHOLD_DIFF;
    let p = ctx.p;
    if ctx.handle.read_int32(p.both_counters, 0).await.unwrap_or(0) == 0 {
        return;
    }
    let clip = ctx
        .handle
        .read_int32(p.prv_img_thresh_diff_clip, 0)
        .await
        .unwrap_or(0)
        != 0;
    let (t0, t1, last_diff_t0) = {
        let state = ctx.shared.preview[s.index].lock();
        (
            state.last[0].clone(),
            state.last[1].clone(),
            state.last_diff_t0,
        )
    };
    let (Some(t0), Some(t1)) = (t0, t1) else {
        return;
    };
    let (t0_frame, t1_frame) = (t0.unique_id, t1.unique_id);
    if t0_frame == last_diff_t0 {
        return;
    }
    // C `previewThresholdFramesAligned` (serval_stream.cpp:54-56).
    if (i64::from(t0_frame) - i64::from(t1_frame)).abs() > 1 {
        ctx.trace(
            TraceMask::IO_DRIVER,
            &adlog::log(
                func,
                &format!(
                    "{} threshold diff: frame ids T0={t0_frame} T1={t1_frame} (header {frame_number}), pairing by order",
                    s.tag
                ),
            ),
        );
    }
    if t0.dims.len() != 2
        || t1.dims.len() != 2
        || t0.dims[0].size != t1.dims[0].size
        || t0.dims[1].size != t1.dims[1].size
    {
        ctx.trace(
            TraceMask::ERROR,
            &adlog::err(
                func,
                &format!("{} threshold diff skipped: dimension mismatch", s.tag),
            ),
        );
        return;
    }
    ctx.shared.preview[s.index].lock().diff = None;
    let dims = vec![
        NDDimension::new(t0.dims[0].size),
        NDDimension::new(t0.dims[1].size),
    ];
    let Ok(mut diff) = ctx.pool.alloc(dims, NDDataType::Int32) else {
        ctx.trace(
            TraceMask::ERROR,
            &adlog::err(
                func,
                &format!("{} failed to allocate threshold-diff NDArray", s.tag),
            ),
        );
        return;
    };
    fn band<T: Copy + Into<i64>>(out: &mut [i32], t0: &[T], t1: &[T], clip: bool) {
        for (o, (a, b)) in out.iter_mut().zip(t0.iter().zip(t1)) {
            // C subtracts the sources cast to int32.
            let d = ((*a).into() as i32).wrapping_sub((*b).into() as i32);
            *o = if clip { d.max(0) } else { d };
        }
    }
    let NDDataBuffer::I32(out) = &mut diff.data else {
        unreachable!("an Int32 allocation")
    };
    match (&t0.data, &t1.data) {
        (NDDataBuffer::U32(a), NDDataBuffer::U32(b)) => band(out, a, b, clip),
        (NDDataBuffer::U16(a), NDDataBuffer::U16(b)) => band(out, a, b, clip),
        (NDDataBuffer::U8(a), NDDataBuffer::U8(b)) => band(out, a, b, clip),
        (a, b) => {
            ctx.trace(
                TraceMask::ERROR,
                &adlog::err(
                    func,
                    &format!(
                        "{} threshold diff skipped: unsupported source types {} / {}",
                        s.tag,
                        a.data_type() as i32,
                        b.data_type() as i32
                    ),
                ),
            );
            return;
        }
    }
    diff.unique_id = t0_frame;
    diff.time_stamp = t0.time_stamp;
    diff.timestamp = t0.timestamp;
    diff.attributes.copy_from(&t0.attributes);
    diff.attributes.add(NDAttribute::new_static(
        "ThresholdBandPass",
        "Preview threshold band-pass",
        NDAttrSource::Driver,
        NDAttrValue::String(if clip { "max(0,T0-T1)" } else { "T0-T1" }.into()),
    ));
    let diff = diff.share();
    {
        let mut state = ctx.shared.preview[s.index].lock();
        state.diff = Some(diff.clone());
        state.last_diff_t0 = t0_frame;
    }
    publish_image(ctx, s.diff, diff).await;
    ctx.trace(
        TraceMask::IO_DRIVER,
        &adlog::log(
            func,
            &format!(
                "Processed {} threshold diff: T0={t0_frame} T1={t1_frame}, addr={} ({} - {}{})",
                s.tag,
                s.diff,
                s.th0,
                s.th1,
                if clip { ", clipped" } else { "" }
            ),
        ),
    );
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
    ctx.post_int64_array(sum_param, sum_addr, int64_array(&update.running));
    if let Some(sum_n) = &update.sum_n {
        ctx.post_int64_array(sum_n_param, sum_n_addr, int64_array(sum_n));
    }
}

/// An accumulation as the asynInt64Array interface carries it: C's
/// `static_cast<epicsInt64>` (serval_stream.cpp:1012, :1099,
/// histogram_io.cpp:537), so a sum past `i64::MAX` wraps.
pub(crate) fn int64_array(values: &[u64]) -> Vec<i64> {
    values.iter().map(|&v| v as i64).collect()
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
pub(crate) enum Accum {
    Img,
    PrvHst,
}

/// What C `configureImgRollingWindow` / `configurePrvHstRollingWindow` set
/// for a configuration, and the WARN line they print with it
/// (serval_stream.cpp:1005-1044, histogram_io.cpp:356-395).
pub(crate) struct WindowReport {
    pub effective: usize,
    pub status: String,
    pub warning: Option<String>,
}

impl Accum {
    pub(crate) fn window_report(self, c: &Configuration) -> WindowReport {
        let (tag, configure) = match self {
            Self::Img => ("Img", adlog::CONFIGURE_IMG_ROLLING_WINDOW),
            Self::PrvHst => ("PrvHst", adlog::CONFIGURE_PRV_HST_ROLLING_WINDOW),
        };
        let mb = c.budget_bytes / (1024 * 1024);
        if c.status != RollingSumStatus::Ok {
            let status = format!("Unavailable: {}", c.status.name());
            let warning = adlog::warn(
                configure,
                &format!(
                    "{tag} rolling window unavailable: {status}; geometry={}, budget={mb} MiB",
                    c.element_count
                ),
            );
            return WindowReport {
                effective: 0,
                status,
                warning: Some(warning),
            };
        }
        let (status, warning) = if c.limited {
            let status = format!(
                "Capped: {}/{} frames, {mb} MiB",
                c.effective_frames, c.requested_frames
            );
            let warning = adlog::warn(configure, &format!("{tag} rolling window {status}"));
            (status, Some(warning))
        } else {
            (format!("OK: {} frames, {mb} MiB", c.effective_frames), None)
        };
        WindowReport {
            effective: c.effective_frames,
            status,
            warning,
        }
    }

    pub(crate) fn get(self, shared: &Shared) -> &parking_lot::Mutex<crate::accum::Accumulator> {
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
    locking: Locking,
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
                ctx.post_range(locking, saturated, status, posted).await;
            }
            NDDataBuffer::U32(avg)
        }
    }
}

/// The PrvHst NDArrays in C's order: running sum on 5, bin times on 7, the
/// current frame on 6 and the window sum on 4 (ADTimePix.cpp:1209-1305,
/// histogram_io.cpp:765-830). Every array carries one `time_stamp`, taken
/// before the first is built.
async fn publish_hst_arrays(ctx: &Ctx, locking: Locking, a: HstArrays, output: SumOutput) {
    let bins = a.running.len();
    if bins == 0 {
        return;
    }
    let time_stamp = EpicsTimestamp::now().as_f64();
    let dims = || vec![NDDimension::new(bins)];
    let (range, units) = output.range_units();

    let data = sum_buffer(
        ctx,
        locking,
        Accum::PrvHst,
        &a.running,
        a.frame_count.max(1),
        output,
    )
    .await;
    if let Ok(mut array) = ctx.pool.alloc_with(dims(), data) {
        ctx.attributes_as(locking, &mut array).await;
        add_time_attributes(&mut array, &a.time_ms);
        let saturated = Accum::PrvHst.saturated(&ctx.shared);
        add_range_attributes(&mut array, range, units, saturated);
        ctx.publish_at(5, array, time_stamp).await;
    }

    if let Ok(mut array) = ctx
        .pool
        .alloc_with(dims(), NDDataBuffer::F64(a.time_ms.clone()))
    {
        ctx.attributes_as(locking, &mut array).await;
        ctx.publish_at(7, array, time_stamp).await;
    }

    if let Some(current) = a.current.filter(|c| c.len() == bins)
        && let Ok(mut array) = ctx.pool.alloc_with(dims(), NDDataBuffer::U32(current))
    {
        ctx.attributes_as(locking, &mut array).await;
        add_range_attributes(&mut array, "uint32", "counts", false);
        ctx.publish_at(6, array, time_stamp).await;
    }

    if let Some((window, frames)) = a.window.filter(|(w, _)| w.len() >= bins) {
        let data = sum_buffer(
            ctx,
            locking,
            Accum::PrvHst,
            &window[..bins],
            frames.max(1),
            output,
        )
        .await;
        if let Ok(mut array) = ctx.pool.alloc_with(dims(), data) {
            ctx.attributes_as(locking, &mut array).await;
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
    publish_hst_arrays(ctx, Locking::Locked, arrays, output).await;
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
    let Ok(mut array) = ctx.pool.alloc_with(
        vec![NDDimension::new(width), NDDimension::new(height)],
        NDDataBuffer::U64(sum),
    ) else {
        return;
    };
    array.unique_id = unique_id;
    ctx.worker_attributes(&mut array).await;
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

    let data = sum_buffer(
        ctx,
        Locking::Locked,
        Accum::Img,
        &running,
        frames.max(1),
        output,
    )
    .await;
    if let Ok(mut array) = ctx.pool.alloc_with(dims(), data) {
        let _ = ctx.pool.owner().get_attributes(&mut array).await;
        let saturated = Accum::Img.saturated(&ctx.shared);
        add_range_attributes(&mut array, range, units, saturated);
        ctx.publish_at(2, array, EpicsTimestamp::now().as_f64())
            .await;
    }

    if let Some((window, frames)) = window.filter(|(w, _)| w.len() >= running.len()) {
        let data = sum_buffer(
            ctx,
            Locking::Locked,
            Accum::Img,
            &window[..running.len()],
            frames,
            output,
        )
        .await;
        if let Ok(mut array) = ctx.pool.alloc_with(dims(), data) {
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
    let array = ctx.pool.alloc_with(
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
    ctx.worker_attributes(array).await;
    array.attributes.add(NDAttribute::new_static(
        "ThresholdID",
        "Serval jsonimage thresholdID",
        NDAttrSource::Driver,
        NDAttrValue::Int32(f.threshold_id),
    ));
}

async fn publish_image(ctx: &Ctx, addr: usize, array: impl Into<PublishedArray>) {
    if ctx.mirrored(Mirrored::ArrayCallbacks) != 0 {
        ArrayPublisher::new(ctx.outputs[addr].clone())
            .publish(array)
            .await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use serde_json::json;

    /// A u64 sum reaches the Int64Array as C's cast leaves it.
    #[test]
    fn int64_array_casts_like_c() {
        assert_eq!(
            int64_array(&[0, i64::MAX as u64, i64::MAX as u64 + 1, u64::MAX]),
            vec![0, i64::MAX, i64::MIN, -1]
        );
    }

    /// A set the actor has stored once this returns, so a worker reading
    /// the mirror next sees it.
    async fn set_synced(ctx: &Ctx, reason: usize, addr: i32, value: i32) {
        ctx.set_int(reason, addr, value).await;
        let _ = ctx.handle.read_int32(reason, addr).await;
    }

    fn idle_ctx(name: &str) -> Ctx {
        idle_ctx_at(name, "http://127.0.0.1:1", None)
    }

    /// An idle port talking to `serval`, its pool the driver's with
    /// `attributes` loaded when given.
    fn idle_ctx_at(name: &str, serval: &str, attributes: Option<&[u8]>) -> Ctx {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let http = Arc::new(ServalHttp::new(serval));
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut driver =
            crate::driver::TimePix3Driver::new(name, http.clone(), shared.clone(), 0, cmd_tx)
                .unwrap();
        let (p, ad) = (driver.p, driver.ad.params);
        let pool = match attributes {
            Some(xml) => {
                driver
                    .ad
                    .write_octet(&AsynUser::new(ad.base.attributes_file), xml)
                    .unwrap();
                driver.ad.pool.clone()
            }
            None => DriverPool::standalone(0),
        };
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
            pool,
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

    /// Every write `read` posts with `addr`'s callbacks, in order.
    fn posted(read: &DetectorRead, addr: i32) -> Vec<ParamSetValue> {
        read.steps
            .iter()
            .filter_map(|s| match s {
                DetectorStep::Post(a, u) if *a == addr => Some(u.iter().cloned()),
                _ => None,
            })
            .flatten()
            .collect()
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
        let family = |d: &Value| {
            let mut w = DetectorWrites::default();
            update_detector_family(&mut w, &ctx.shared, &p, d);
            w.pending
        };
        let u = family(&mpx3);
        assert_eq!(int_set(&u, p.detector_family), Some(2));
        assert_eq!(int_set(&u, p.cap_img_thresholds), Some(1));
        assert_eq!(int_set(&u, p.cap_tdc), Some(0));
        assert_eq!(int_set(&u, p.pixel_depth), Some(12));
        assert_eq!(int_set(&u, p.i_delay3), Some(10));

        let u = family(&mpx3);
        assert_eq!(int_set(&u, p.detector_family), Some(2));
        assert_eq!(int_set(&u, p.pixel_depth), None);

        let tpx3 = serde_json::json!({"Info": {"ChipType": "tpx3"}});
        let u = family(&tpx3);
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
        set_synced(&ctx, ad.base.array_callbacks, 0, 1).await;

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
                header: serde_json::Value::Null,
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
        set_synced(&ctx, ad.base.array_callbacks, 0, 1).await;

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
                    header: serde_json::Value::Null,
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

    /// C `processImgFrame` posts the rolling window's configuration,
    /// FramesSummed and the memory estimate (serval_stream.cpp:1005-1043,
    /// :1132-1164, :1346-1375).
    #[tokio::test]
    async fn an_img_frame_posts_the_window_and_memory_usage() {
        use crate::stream::PixelFormat;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let http = Arc::new(ServalHttp::new("http://127.0.0.1:1"));
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let driver = crate::driver::TimePix3Driver::new(
            "TPXIMGWIN",
            http.clone(),
            shared.clone(),
            0,
            cmd_tx,
        )
        .unwrap();
        let (p, ad) = (driver.p, driver.ad.params);
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port("TPXIMGWIN").unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctx = Arc::new(Ctx {
            handle: runtime.port_handle().clone(),
            http,
            shared,
            p,
            ad,
            outputs: new_outputs(),
            pool: DriverPool::standalone(0),
        });
        set_synced(&ctx, p.img_accumulation_enable, 0, 1).await;
        let frame = Frame::Image(crate::stream::ImageFrame {
            width: 2,
            height: 1,
            format: PixelFormat::UInt16,
            frame_number: 1,
            time_at_frame: 0.0,
            threshold_id: 0,
            integration_size: 0,
            pixels: vec![1, 200],
            header: serde_json::Value::Null,
        });
        handle_frame(&ctx, StreamKind::Img, frame).await;

        let h = &ctx.handle;
        assert_eq!(h.read_int32(p.img_effective_frames, 0).await.unwrap(), 10);
        assert_eq!(h.read_int32(p.img_frames_summed, 0).await.unwrap(), 1);
        assert_eq!(
            h.read_octet(p.img_retention_status, 0, 64).await.unwrap(),
            b"OK: 10 frames, 512 MiB\0"
        );
        // 2 u64 running + 2 UINT16 current + the window's 2 u64 and one
        // 2-pixel u32 frame + 2 pixels of waveforms (8 + 8 + 4).
        let mib = 1024.0 * 1024.0;
        let expected = f64::from(16 + 4 + 24 + 40) / mib + 0.1;
        assert_eq!(
            h.read_float64(p.img_memory_usage, 0).await.unwrap(),
            expected
        );
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
        let (tx, mut rx_t1) = ndarray_channel("TPXATTR_T1_SINK", 4);
        outputs[PRV_IMG.th1].lock().add(tx);
        let (tx, mut rx_img_t1) = ndarray_channel("TPXATTR_IMG_T1_SINK", 4);
        outputs[IMG_THRESHOLD1_ADDR].lock().add(tx);
        let ctx = Arc::new(Ctx {
            handle: runtime.port_handle().clone(),
            http,
            shared,
            p,
            ad,
            outputs,
            pool,
        });
        set_synced(&ctx, ad.base.array_callbacks, 0, 1).await;
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
                header: serde_json::Value::Null,
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

        // The header's threshold reaches the attribute, the stream's params
        // and the threshold's address (serval_stream.cpp:245-246, :894).
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
                header: serde_json::Value::Null,
            });
            handle_frame(&ctx, kind, frame).await;
            // A threshold-1 frame goes to its own address
            // (serval_stream.cpp:139-140, :780-781).
            let rx = match kind {
                StreamKind::Img => &mut rx_img_t1,
                _ => &mut rx_t1,
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
                header: serde_json::Value::Null,
            })
        };
        let attr = |a: &epics_rs::ad_core::ndarray::NDArray, name| {
            a.attributes.get(name).map(|a| a.value.clone())
        };
        let int = |reason| ctx.handle.read_int32(reason, 0);

        // Counted with NDArrayCallbacks off.
        set_synced(&ctx, ad.base.array_callbacks, 0, 0).await;
        handle_frame(
            &ctx,
            StreamKind::Img,
            frame(crate::stream::PixelFormat::UInt16),
        )
        .await;
        assert_eq!(int(ad.base.array_counter).await.unwrap(), 1);

        set_synced(&ctx, ad.base.array_callbacks, 0, 1).await;
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
        set_synced(&ctx, ad.base.array_callbacks, 0, 1).await;
        set_synced(&ctx, p.img_accumulation_enable, 0, 1).await;
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
                header: serde_json::Value::Null,
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
                header: serde_json::Value::Null,
            })
        };
        let total = || ctx.shared.img.lock().total_counts();
        handle_frame(&ctx, StreamKind::Img, frame(1)).await;
        assert_eq!(total(), 0);
        handle_frame(&ctx, StreamKind::Img, frame(0)).await;
        assert_eq!(total(), 3);
        set_synced(&ctx, ctx.p.img_accumulation_enable, 0, 0).await;
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
        set_synced(&ctx, ctx.ad.base.array_callbacks, 0, 1).await;
        set_synced(&ctx, ctx.p.prv_hst_frame_bin_offset, 0, 0).await;
        set_synced(&ctx, ctx.p.prv_hst_frame_bin_width, 0, 2).await;
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

        set_synced(&ctx, ctx.p.processed_hst_output_type, 0, 1).await;
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

    /// C `processPrvHstFrame` posts the rolling window's configuration,
    /// FramesSummed and `calculatePrvHstMemoryUsageMB`
    /// (histogram_io.cpp:356-418, :577-610, :848-881).
    #[tokio::test]
    async fn a_prv_hst_frame_posts_the_window_and_memory_usage() {
        let ctx = Arc::new(idle_ctx("TPX_HST_WIN"));
        let p = ctx.p;
        let frame = Frame::Histogram(crate::stream::HistogramFrame {
            bin_width: 2,
            bin_offset: 1,
            frame_number: 1,
            time_at_frame: 0.0,
            counts: vec![1, 2],
        });
        handle_frame(&ctx, StreamKind::PrvHst, frame).await;

        let h = &ctx.handle;
        assert_eq!(
            h.read_int32(p.prv_hst_effective_frames, 0).await.unwrap(),
            10
        );
        assert_eq!(h.read_int32(p.prv_hst_frames_summed, 0).await.unwrap(), 1);
        assert_eq!(
            h.read_octet(p.prv_hst_retention_status, 0, 64)
                .await
                .unwrap(),
            b"OK: 10 frames, 512 MiB\0"
        );
        // 2 u64 running + 3 f64 edges, a 3-value time axis, the window's
        // 2 u64 and one 2-bin u32 frame, one processing sample (the first
        // frame takes no rate sample) and the line buffer.
        let bytes = 16 + 24 + 24 + 24 + 8 + PRV_HST_LINE_BUFFER;
        let expected = bytes as f64 / (1024.0 * 1024.0) + 0.1;
        assert_eq!(
            h.read_float64(p.prv_hst_memory_usage, 0).await.unwrap(),
            expected
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
                    header: serde_json::Value::Null,
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

    /// A stream that ends ends the worker's run for the acquisition: C fails
    /// the state instead of reconnecting (serval_stream.cpp:563-585).
    #[test]
    fn a_stream_that_ends_is_not_reconnected() {
        let ctx = Arc::new(idle_ctx("TPX_ONE_SHOT"));
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let path = format!("tcp://{}", listener.local_addr().unwrap());
        ctx.shared.set_stream_paths(Some(path), None, None);
        assert!(ctx.shared.acquisition.begin_start());
        assert!(ctx.shared.prv_img_worker.start());
        let c = ctx.clone();
        let _worker = rt::run_thread_named("TpxOneShot", move || async move {
            stream_worker(c, StreamKind::PrvImg).await;
        });

        drop(listener.accept().unwrap());
        listener.set_nonblocking(true).unwrap();
        let deadline = std::time::Instant::now() + RECONNECT_WAIT + Duration::from_millis(500);
        while std::time::Instant::now() < deadline {
            assert!(listener.accept().is_err(), "the worker reconnected");
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(
            !ctx.shared
                .prv_img_worker
                .wait_for_connected_for(Duration::ZERO)
        );
    }

    /// C's stop holds the port while Serval sends the last frames and closes
    /// the streams, and its workers, which never take the port lock, post
    /// them meanwhile (acquisition_runtime.cpp:207-211): a frame Serval sends
    /// after /measurement/stop is posted before the stop returns.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_stop_drains_the_frames_serval_sends_after_the_stop_request() {
        use crate::coordinator::Resource;
        use std::io::{BufRead, BufReader, Write};

        let stream = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let peer: Arc<parking_lot::Mutex<Option<TcpStream>>> = Arc::default();
        let http = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", http.local_addr().unwrap());
        let tail = peer.clone();
        std::thread::spawn(move || {
            for conn in http.incoming().flatten() {
                let mut reader = BufReader::new(conn);
                let mut request = String::new();
                let _ = reader.read_line(&mut request);
                request = crate::http::without_serval_params(&request);
                loop {
                    let mut line = String::new();
                    if reader.read_line(&mut line).unwrap_or(0) == 0 || line == "\r\n" {
                        break;
                    }
                }
                if request.contains("/measurement/stop") {
                    let tail = tail.clone();
                    std::thread::spawn(move || {
                        std::thread::sleep(Duration::from_millis(300));
                        let mut peer = tail.lock().take().unwrap();
                        let mut bytes =
                            br#"{"width":1,"height":1,"pixelFormat":"uint16","frameNumber":7}"#
                                .to_vec();
                        bytes.extend_from_slice(b"\n\x00\x01\n");
                        peer.write_all(&bytes).unwrap();
                    });
                }
                let _ = reader.get_mut().write_all(
                    b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}",
                );
            }
        });

        let ctx = Arc::new(idle_ctx_at(
            "TPX_STOP_DRAIN",
            &url,
            Some(br#"<Attributes><Attribute name="Exposure" type="PARAM" source="ACQ_TIME" datatype="DOUBLE"/></Attributes>"#),
        ));
        let (tx, mut rx) =
            epics_rs::ad_core::plugin::channel::ndarray_channel("TPX_STOP_DRAIN_SINK", 4);
        ctx.outputs[PRV_IMG_ADDR].lock().add(tx);
        set_synced(&ctx, ctx.ad.base.array_callbacks, 0, 1).await;
        let path = format!("tcp://{}", stream.local_addr().unwrap());
        ctx.shared.set_stream_paths(Some(path), None, None);
        let acquisition = &ctx.shared.acquisition;
        assert!(acquisition.begin_start());
        for resource in [Resource::RemoteMeasurement, Resource::MonitorThread] {
            assert!(acquisition.note_resource(resource));
        }
        assert!(acquisition.commit_start());
        assert!(ctx.shared.prv_img_worker.start());
        let c = ctx.clone();
        let _worker = rt::run_thread_named("TpxStopDrain", move || async move {
            stream_worker(c, StreamKind::PrvImg).await;
        });
        *peer.lock() = Some(stream.accept().unwrap().0);
        assert!(
            ctx.shared
                .prv_img_worker
                .wait_for_connected_for(Duration::from_secs(5))
        );

        ctx.handle
            .with_driver(|d: &mut TimePix3Driver| d.stop_acquisition(false, ""))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(
            ctx.handle
                .read_int32(ctx.p.prv_img_frame_number, 0)
                .await
                .unwrap_or(0),
            7
        );
        // The frame's getAttributes was served while the stop held the port.
        let array = tokio::time::timeout(Duration::from_millis(10), rx.recv())
            .await
            .expect("the frame was not published before the stop returned")
            .unwrap();
        assert!(array.attributes.get("Exposure").is_some());
    }

    /// C neither reads the histogram socket nor handles a frame while
    /// PrvHstAccumulationEnable is 0 (histogram_io.cpp:165-170, :1018-1023):
    /// a frame sent meanwhile waits in the socket and is taken on re-enable.
    #[tokio::test(flavor = "multi_thread")]
    async fn prv_hst_accumulation_off_leaves_the_socket_unread() {
        use std::io::Write;

        let ctx = Arc::new(idle_ctx("TPX_HST_GATE"));
        let enable = ctx.p.prv_hst_accumulation_enable;
        set_synced(&ctx, enable, 0, 0).await;
        assert!(ctx.shared.prv_hst_worker.start());
        let run = ctx.shared.prv_hst_worker.wait_for_start(None);
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let _ = client.set_read_timeout(Some(Duration::from_millis(50)));
        let (mut peer, _) = listener.accept().unwrap();
        let reader = tokio::spawn({
            let ctx = ctx.clone();
            async move { read_stream(&ctx, StreamKind::PrvHst, run, client).await }
        });

        let mut bytes = br#"{"binSize":2,"binWidth":1,"binOffset":0,"frameNumber":1}"#.to_vec();
        bytes.push(b'\n');
        bytes.extend_from_slice(&[0, 0, 0, 1, 0, 0, 0, 2, b'\n']);
        peer.write_all(&bytes).unwrap();
        tokio::time::sleep(Duration::from_millis(400)).await;
        assert_eq!(ctx.shared.hst.lock().frame_count(), 0);

        set_synced(&ctx, enable, 0, 1).await;
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
        set_synced(&ctx, ctx.ad.base.array_callbacks, 0, 1).await;
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
                header: serde_json::Value::Null,
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
        set_synced(&ctx, ctx.ad.base.array_callbacks, 0, 1).await;
        push_processed_img(&ctx).await;
        let (sum, sum_n) = (recv(0).await, recv(1).await);
        assert!(matches!(&sum.data, NDDataBuffer::U64(v) if *v == vec![4, 6]));
        assert!(matches!(&sum_n.data, NDDataBuffer::U64(v) if *v == vec![4, 6]));
        assert_eq!(sum.dims.len(), 2);

        set_synced(&ctx, ctx.p.processed_img_output_type, 0, 1).await;
        push_processed_img(&ctx).await;
        let (avg, avg_n) = (recv(0).await, recv(1).await);
        assert!(matches!(&avg.data, NDDataBuffer::U32(v) if *v == vec![2, 3]));
        assert!(matches!(&avg_n.data, NDDataBuffer::U32(v) if *v == vec![2, 3]));
        assert_eq!(
            avg.attributes.get("ADTimePixCountUnits").map(|a| &a.value),
            Some(&NDAttrValue::String("counts/frame".into()))
        );
    }

    /// Serval answering every request with 200 and `body_for(path)`, except
    /// the paths in `fail` (500); it logs each request line.
    fn logging_serval(
        fail: &'static [&'static str],
        body_for: fn(&str) -> &'static str,
    ) -> (String, Arc<parking_lot::Mutex<Vec<String>>>) {
        use std::io::{BufRead, BufReader, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let log: Arc<parking_lot::Mutex<Vec<String>>> = Arc::default();
        let seen = log.clone();
        std::thread::spawn(move || {
            for conn in listener.incoming().flatten() {
                let mut reader = BufReader::new(conn);
                let mut request = String::new();
                let _ = reader.read_line(&mut request);
                request = crate::http::without_serval_params(&request);
                loop {
                    let mut line = String::new();
                    if reader.read_line(&mut line).unwrap_or(0) == 0 || line == "\r\n" {
                        break;
                    }
                }
                let path = request.split(' ').nth(1).unwrap_or("").to_string();
                seen.lock().push(request.trim_end().to_string());
                let (status, body) = if fail.contains(&path.as_str()) {
                    ("500 Internal Server Error", "")
                } else {
                    ("200 OK", body_for(&path))
                };
                let _ = reader.get_mut().write_all(
                    format!(
                        "HTTP/1.1 {status}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
                        body.len()
                    )
                    .as_bytes(),
                );
            }
        });
        (url, log)
    }

    /// A dashboard reply C `parseResponse` rejects is a dashboard failure as
    /// a whole; a 200 still means Serval is reachable
    /// (serval_http.cpp:363-392, serval_dashboard.cpp:49-98).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_rejected_dashboard_reply_is_a_failure_with_serval_up() {
        type Body = fn(&str) -> &'static str;
        let cases: [(&str, Body); 3] = [
            ("TPX_DASH_MALFORMED", |_| "{"),
            (
                "TPX_DASH_TYPE",
                |_| r#"{"Server":{"SoftwareVersion":"3.3"},"Detector":{"DetectorType":5}}"#,
            ),
            (
                "TPX_DASH_NODET",
                |_| r#"{"Server":{"SoftwareVersion":"3.3"}}"#,
            ),
        ];
        for (name, body_for) in cases {
            let (url, _log) = logging_serval(&[], body_for);
            let ctx = Arc::new(idle_ctx_at(name, &url, None));
            let (h, p) = (&ctx.handle, ctx.p);
            ctx.set(
                0,
                vec![
                    ParamSetValue::new(
                        ctx.ad.base.sdk_version,
                        0,
                        ParamValue::Octet(b"old".to_vec().into()),
                    ),
                    ParamSetValue::new(p.status, 0, ParamValue::Octet(b"DA_IDLE".to_vec().into())),
                ],
            )
            .await;
            assert!(!check_connection(&ctx).await, "{name}");
            assert_eq!(h.read_int32(p.http_code, 0).await.unwrap(), 200, "{name}");
            assert_eq!(
                h.read_int32(p.serval_connected, 0).await.unwrap(),
                1,
                "{name}"
            );
            assert_eq!(h.read_int32(p.det_connected, 0).await.unwrap(), 0, "{name}");
            assert_eq!(
                h.read_octet(p.det_type, 0, 64).await.unwrap(),
                b"null\0",
                "{name}"
            );
            assert_eq!(
                h.read_octet(ctx.ad.base.sdk_version, 0, 64).await.unwrap(),
                b"old\0",
                "{name}"
            );
            assert_eq!(
                h.read_octet(p.status, 0, 64).await.unwrap(),
                b"\0",
                "{name}"
            );
            assert_eq!(
                h.read_octet(ctx.ad.status_message, 0, 64).await.unwrap(),
                b"Detector disconnected\0",
                "{name}"
            );
            assert_eq!(
                h.read_int32(ctx.ad.status, 0).await.unwrap(),
                ADStatus::Disconnected as i32,
                "{name}"
            );
        }
    }

    /// C `runJsonImageTcpWorker`'s framing lines (serval_stream.cpp:567-611):
    /// a stream cut mid-frame and a header the resolver refuses.
    #[tokio::test(flavor = "multi_thread")]
    async fn the_image_read_loop_reports_framing_as_c() {
        use std::io::Write;

        for (name, sent, expected) in [
            (
                "TPX_END_MID",
                &br#"{"width":1,"height":1,"pixelFormat":"uint16"}"#[..],
                "PrvImg TCP stream ended with truncated frame payload",
            ),
            (
                "TPX_BAD_HDR",
                b"[1]\n",
                "PrvImg rejected TCP stream: invalid JSON frame header: \
                 malformed or non-object jsonimage header",
            ),
        ] {
            let ctx = Arc::new(idle_ctx(name));
            let path = capture_errors(&ctx, name);
            assert!(ctx.shared.prv_img_worker.start());
            let run = ctx.shared.prv_img_worker.wait_for_start(None);
            let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
            let _ = client.set_read_timeout(Some(READ_TIMEOUT));
            let (mut peer, _) = listener.accept().unwrap();
            let mut bytes = sent.to_vec();
            if name == "TPX_END_MID" {
                bytes.extend_from_slice(b"\n\x00");
            }
            peer.write_all(&bytes).unwrap();
            drop(peer);
            read_stream(&ctx, StreamKind::PrvImg, run, client).await;
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            assert!(
                text.contains(&adlog::err(RUN_JSON_IMAGE_TCP_WORKER, expected)),
                "{text}"
            );
        }
    }

    /// A refused frame allocation fails the worker as a framing failure
    /// does, with the peer still connected (serval_stream.cpp:177-183,
    /// :808-815, :618-626).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_refused_frame_allocation_fails_the_worker() {
        use std::io::Write;

        for (name, kind, expected) in [
            (
                "TPX_ALLOC_PRV",
                StreamKind::PrvImg,
                adlog::err(
                    adlog::PROCESS_PREVIEW_JSONIMAGE_LINE,
                    "PrvImg failed to allocate NDArray",
                ),
            ),
            (
                "TPX_ALLOC_IMG",
                StreamKind::Img,
                adlog::err(
                    adlog::PROCESS_IMG_DATA_LINE,
                    "Failed to allocate NDArray or NDArray has no data pointer",
                ),
            ),
        ] {
            let mut ctx = idle_ctx(name);
            ctx.pool = DriverPool::standalone(8);
            let ctx = Arc::new(ctx);
            let path = capture_errors(&ctx, name);
            assert!(kind.worker(&ctx.shared).start());
            let run = kind.worker(&ctx.shared).wait_for_start(None);
            let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
            let _ = client.set_read_timeout(Some(READ_TIMEOUT));
            let (mut peer, _) = listener.accept().unwrap();
            let mut frame = br#"{"width":2,"height":2,"pixelFormat":"uint32"}"#.to_vec();
            frame.push(b'\n');
            frame.resize(frame.len() + 16, 0);
            frame.push(b'\n');
            peer.write_all(&frame).unwrap();
            let reader = tokio::spawn({
                let ctx = ctx.clone();
                async move { read_stream(&ctx, kind, run, client).await }
            });
            let deadline = std::time::Instant::now() + Duration::from_secs(3);
            while !reader.is_finished() && std::time::Instant::now() < deadline {
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
            let returned = reader.is_finished();
            kind.worker(&ctx.shared).request_stop();
            reader.await.unwrap();
            drop(peer);
            assert!(returned, "{name}: the worker kept reading");
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            assert!(text.contains(&expected), "{name}: {text}");
        }
    }

    /// A largest-image frame followed at once by a second frame stays within
    /// the FrameBuffer limit, because each read is C's MAX_BUFFER_SIZE
    /// (serval_stream.cpp:505-510).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_largest_frame_with_its_successor_is_read_in_c_chunks() {
        use std::io::Write;

        let name = "TPX_READ_CHUNK";
        let ctx = Arc::new(idle_ctx(name));
        let path = capture_errors(&ctx, name);
        assert!(ctx.shared.prv_img_worker.start());
        let run = ctx.shared.prv_img_worker.wait_for_start(None);
        let frame = |w: usize, h: usize, fmt: &str, bpp: usize| {
            let mut f =
                format!(r#"{{"width":{w},"height":{h},"pixelFormat":"{fmt}"}}"#).into_bytes();
            f.push(b'\n');
            f.resize(f.len() + w * h * bpp, 0);
            f.push(b'\n');
            f
        };
        let (first, second) = (frame(2048, 256, "uint32", 4), frame(512, 256, "uint16", 2));
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let _ = client.set_read_timeout(Some(READ_TIMEOUT));
        let (mut peer, _) = listener.accept().unwrap();
        let writer = std::thread::spawn(move || {
            let (head, tail) = first.split_at(first.len() - 10);
            peer.write_all(head).unwrap();
            // The reader drains the head and then finds the tail and the
            // whole second frame queued for one read.
            std::thread::sleep(Duration::from_millis(300));
            peer.write_all(&[tail, &second].concat()).unwrap();
        });
        read_stream(&ctx, StreamKind::PrvImg, run, client).await;
        writer.join().unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(!text.contains("rejected TCP stream"), "{text}");
    }

    /// C sends a threshold-1 frame to its own address, and a T1 then T0
    /// pair makes the T0 - T1 band-pass (serval_stream.cpp:139-140, :322-351,
    /// :366-468, :780-781).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_threshold_pair_publishes_the_band_pass() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        let ctx = Arc::new(idle_ctx("TPX_BAND"));
        let mut rx = std::collections::BTreeMap::new();
        for addr in [PRV_IMG_ADDR, 8, 9, IMG_ADDR, IMG_THRESHOLD1_ADDR] {
            let (tx, r) = ndarray_channel(&format!("TPX_BAND_{addr}"), 4);
            ctx.outputs[addr].lock().add(tx);
            rx.insert(addr, r);
        }
        let (p, ad) = (ctx.p, ctx.ad);
        set_synced(&ctx, ad.base.array_callbacks, 0, 1).await;
        set_synced(&ctx, p.both_counters, 0, 1).await;
        let frame = |threshold_id, frame_number, pixels| {
            Frame::Image(crate::stream::ImageFrame {
                width: 2,
                height: 1,
                format: crate::stream::PixelFormat::UInt16,
                frame_number,
                time_at_frame: 0.0,
                threshold_id,
                integration_size: 0,
                pixels,
                header: Value::Null,
            })
        };
        async fn next(
            rx: &mut std::collections::BTreeMap<
                usize,
                epics_rs::ad_core::plugin::channel::NDArrayReceiver,
            >,
            addr: usize,
        ) -> Arc<epics_rs::ad_core::ndarray::NDArray> {
            tokio::time::timeout(Duration::from_secs(5), rx.get_mut(&addr).unwrap().recv())
                .await
                .unwrap_or_else(|_| panic!("no array at {addr}"))
                .unwrap()
        }

        for (clip, n, band, label) in [(0, 5, [-1, 3], "T0-T1"), (1, 6, [0, 3], "max(0,T0-T1)")] {
            set_synced(&ctx, p.prv_img_thresh_diff_clip, 0, clip).await;
            assert!(handle_frame(&ctx, StreamKind::PrvImg, frame(1, n, vec![3, 1])).await);
            assert_eq!(next(&mut rx, 8).await.unique_id, n);
            assert!(handle_frame(&ctx, StreamKind::PrvImg, frame(0, n, vec![2, 4])).await);
            assert_eq!(next(&mut rx, PRV_IMG_ADDR).await.unique_id, n);
            let diff = next(&mut rx, 9).await;
            assert_eq!(diff.unique_id, n);
            match &diff.data {
                NDDataBuffer::I32(v) => assert_eq!(v, &band),
                other => panic!("{:?}", other.data_type()),
            }
            assert_eq!(
                diff.attributes.get("ThresholdBandPass").map(|a| &a.value),
                Some(&NDAttrValue::String(label.into()))
            );
            assert_eq!(
                diff.attributes.get("ThresholdID").map(|a| &a.value),
                Some(&NDAttrValue::Int32(0))
            );
        }
        // A repeated T0 is an orphan waiting for its T1, not a new pair.
        assert!(handle_frame(&ctx, StreamKind::PrvImg, frame(0, 7, vec![2, 4])).await);
        next(&mut rx, PRV_IMG_ADDR).await;
        assert_eq!(rx[&9].pending(), 0);

        assert!(handle_frame(&ctx, StreamKind::Img, frame(1, 9, vec![1, 1])).await);
        assert_eq!(next(&mut rx, IMG_THRESHOLD1_ADDR).await.unique_id, 9);
        assert_eq!(rx[&IMG_ADDR].pending(), 0);
    }

    /// The first PrvImgLogHeaders PrvImg headers are logged as nlohmann
    /// `dump()`, cut at 1024 bytes (serval_stream.cpp:142-151).
    #[tokio::test(flavor = "multi_thread")]
    async fn the_first_prv_img_headers_are_logged() {
        let name = "TPX_HDR_LOG";
        let ctx = idle_ctx(name);
        let path = capture_errors(&ctx, name);
        ctx.handle
            .trace()
            .set_trace_mask(Some(name), TraceMask::IO_DRIVER);
        ctx.shared.preview[0].lock().json_headers = 2;
        let long = "x".repeat(2000);
        log_json_header(
            &ctx,
            PRV_IMG,
            &json!({"width": 2, "frameNumber": 1, "timeAtFrame": 1e-5}),
        );
        log_json_header(&ctx, PRV_IMG, &json!({"a": long}));
        log_json_header(&ctx, PRV_IMG, &json!({"never": true}));
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let line = |h: &str| {
            adlog::log(
                adlog::PROCESS_PREVIEW_JSONIMAGE_LINE,
                &format!("PrvImg jsonimage header: {h}"),
            )
        };
        assert!(
            text.contains(&line(r#"{"frameNumber":1,"timeAtFrame":1e-05,"width":2}"#)),
            "{text}"
        );
        let cut = format!("{{\"a\":\"{}...", "x".repeat(1024 - 6));
        assert!(text.contains(&line(&cut)), "{text}");
        assert!(!text.contains("never"), "{text}");
    }

    /// ERROR-mask trace output of the port `name` goes to the returned file.
    fn capture_errors(ctx: &Ctx, name: &str) -> std::path::PathBuf {
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        ctx.handle.trace().set_trace_file(
            Some(name),
            epics_rs::asyn::trace::TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        ctx.handle
            .trace()
            .set_trace_mask(Some(name), TraceMask::ERROR);
        path
    }

    /// A rejected dashboard read logs the HTTP reply and the parse verdict,
    /// except on the background poll (serval_http.cpp:380-385, :441-444).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_rejected_dashboard_read_is_logged_except_by_the_poll() {
        let (url, _log) = logging_serval(&[], |_| "{");
        let ctx = Arc::new(idle_ctx_at("TPX_DASH_LOG", &url, None));
        let path = capture_errors(&ctx, "TPX_DASH_LOG");
        poll_connection(&ctx, false).await;
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "");
        check_connection(&ctx).await;
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let dashboard = format!("{url}/dashboard");
        for line in [
            adlog::http_failure_reply("checkConnection", "GET", &dashboard, 200, "{"),
            adlog::err(
                "asynStatus ADTimePix::checkConnection(bool, bool)",
                "checkConnection: malformed dashboard JSON",
            ),
        ] {
            assert!(text.contains(&line), "{text}");
        }

        let (url, _log) = logging_serval(&["/dashboard"], |_| "{}");
        let ctx = Arc::new(idle_ctx_at("TPX_DASH_LOG500", &url, None));
        let path = capture_errors(&ctx, "TPX_DASH_LOG500");
        refresh_dashboard(&ctx).await;
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let dashboard = format!("{url}/dashboard");
        for line in [
            adlog::http_failure_reply("getDashboard", "GET", &dashboard, 500, ""),
            adlog::err(
                "asynStatus ADTimePix::getDashboard()",
                "getDashboard: dashboard GET failed",
            ),
        ] {
            assert!(text.contains(&line), "{text}");
        }
    }

    /// The startup check publishes the connection state and logs a failure
    /// as C `initialServerCheckConnection` (serval_http.cpp:315-354).
    #[tokio::test(flavor = "multi_thread")]
    async fn the_startup_check_publishes_the_connection_state() {
        type Body = fn(&str) -> &'static str;
        let ok: Body = |path| match path {
            "/dashboard" => r#"{"Server":{},"Detector":{"DetectorType":"TPX3"}}"#,
            _ => "{}",
        };
        let (url, _log) = logging_serval(&[], ok);
        let ctx = Arc::new(idle_ctx_at("TPX_INIT_OK", &url, None));
        let path = capture_errors(&ctx, "TPX_INIT_OK");
        assert!(initial_server_check_connection(&ctx));
        let (h, p) = (&ctx.handle, ctx.p);
        assert_eq!(h.read_int32(p.http_code, 0).await.unwrap(), 200);
        assert_eq!(h.read_int32(p.serval_connected, 0).await.unwrap(), 1);
        assert_eq!(h.read_int32(p.det_connected, 0).await.unwrap(), 1);
        assert_eq!(h.read_octet(p.det_type, 0, 64).await.unwrap(), b"TPX3\0");
        assert_eq!(
            h.read_int32(ctx.ad.status, 0).await.unwrap(),
            ADStatus::Idle as i32
        );
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "");
        let _ = std::fs::remove_file(&path);

        type Fail = &'static [&'static str];
        let cases: [(&str, Fail, Body, i32, i32, &str); 2] = [
            (
                "TPX_INIT_ROOT",
                &["/"],
                ok,
                500,
                0,
                "SERVAL and detector disconnected",
            ),
            (
                "TPX_INIT_DASH",
                &[],
                |_| "{",
                200,
                1,
                "Detector disconnected",
            ),
        ];
        for (name, fail, body_for, code, serval, message) in cases {
            let (url, _log) = logging_serval(fail, body_for);
            let ctx = Arc::new(idle_ctx_at(name, &url, None));
            let path = capture_errors(&ctx, name);
            assert!(!initial_server_check_connection(&ctx), "{name}");
            let (h, p) = (&ctx.handle, ctx.p);
            assert_eq!(h.read_int32(p.http_code, 0).await.unwrap(), code, "{name}");
            assert_eq!(
                h.read_int32(p.serval_connected, 0).await.unwrap(),
                serval,
                "{name}"
            );
            assert_eq!(h.read_int32(p.det_connected, 0).await.unwrap(), 0, "{name}");
            assert_eq!(
                h.read_octet(p.det_type, 0, 64).await.unwrap(),
                b"null\0",
                "{name}"
            );
            assert_eq!(
                h.read_octet(ctx.ad.status_message, 0, 64).await.unwrap(),
                format!("{message}\0").as_bytes(),
                "{name}"
            );
            assert_eq!(
                h.read_int32(ctx.ad.status, 0).await.unwrap(),
                ADStatus::Disconnected as i32,
                "{name}"
            );
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            let func = "asynStatus ADTimePix::initialServerCheckConnection()";
            let mut lines = vec![adlog::err(
                func,
                &format!("ERROR: Failed to connect to server {url}"),
            )];
            if code == 200 {
                lines.push(adlog::http_failure_reply(
                    "initialServerCheckConnection dashboard",
                    "GET",
                    &format!("{url}/dashboard"),
                    200,
                    "{",
                ));
                lines.push(adlog::err(
                    func,
                    "initialServerCheckConnection: malformed dashboard JSON",
                ));
            } else {
                lines.push(adlog::http_failure_reply(
                    "initialServerCheckConnection",
                    "GET",
                    &url,
                    500,
                    "",
                ));
            }
            for line in lines {
                assert!(text.contains(&line), "{name}: {text}");
            }
        }
    }

    /// The poll starts from the startup check: its first period is skipped
    /// and a detector found connected there is no reconnect edge
    /// (ADTimePix.cpp:2065-2068, serval_http.cpp:404-416).
    #[tokio::test(flavor = "multi_thread")]
    async fn the_poll_starts_from_the_startup_check() {
        let (url, log) = logging_serval(&[], |path| match path {
            "/dashboard" => r#"{"Server":{},"Detector":{"DetectorType":"TPX3"}}"#,
            _ => "{}",
        });
        let ctx = Arc::new(idle_ctx_at("TPX_INIT_POLL", &url, None));
        let (_cmd_tx, cmd_rx) = rt::command_channel::<Command>(8);
        let (acq_tx, _acq_rx) = rt::command_channel::<()>(1);
        tokio::spawn(connection_poll(ctx.clone(), cmd_rx, acq_tx, true, true));
        let reads = |path: &str| {
            log.lock()
                .iter()
                .filter(|r| r.starts_with(&format!("GET {path} ")))
                .count()
        };
        tokio::time::sleep(CONNECTION_POLL + Duration::from_millis(500)).await;
        assert_eq!(reads("/dashboard"), 0);
        tokio::time::sleep(CONNECTION_POLL + Duration::from_millis(500)).await;
        assert_eq!(reads("/dashboard"), 1);
        assert_eq!(reads("/server/destination"), 0);
    }

    /// A `/detector` reply that passes C `parseResponse`.
    const DETECTOR: &str = r#"{"Info":{"PixCount":65536,"RowLen":1,"NumberOfChips":1,
        "NumberOfRows":256,"MpxType":3},"Config":{}}"#;

    /// An orientation name C does not know reads as UP
    /// (detOrientationIndexFromName's default, serval_http.cpp:1514-1519).
    #[tokio::test(flavor = "multi_thread")]
    async fn an_unknown_orientation_reads_as_up() {
        let (url, _log) = logging_serval(&[], |path| match path {
            "/detector" => {
                r#"{"Info":{"PixCount":65536,"RowLen":1,"NumberOfChips":1,
                "NumberOfRows":256,"MpxType":3},"Config":{},
                "Layout":{"DetectorOrientation":"SIDEWAYS"}}"#
            }
            _ => "{}",
        });
        let ctx = idle_ctx_at("TPX_DET_ORIENT", &url, None);
        ctx.shared.set_orientation(3);
        let read = read_detector(&ctx.http, &ctx.shared, ctx.p, ctx.ad, false);
        assert!(read.ok);
        assert_eq!(ctx.shared.orientation(), 0);
        assert_eq!(
            int_set(&posted(&read, 0), ctx.p.detector_orientation),
            Some(0)
        );
    }

    /// The chip layout is the entry whose `Chip` is the chip number, from
    /// `Rotated` on MPX3 and `Original` otherwise (serval_http.cpp:1525-1553).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_chip_layout_is_matched_by_chip_from_the_family_layout() {
        fn read(name: &str, body_for: fn(&str) -> &'static str) -> Vec<String> {
            let (url, _log) = logging_serval(&[], body_for);
            let ctx = idle_ctx_at(name, &url, None);
            let read = read_detector(&ctx.http, &ctx.shared, ctx.p, ctx.ad, false);
            assert!(read.ok);
            (0..2)
                .map(|chip| octet_set(&posted(&read, chip), ctx.p.layout).unwrap_or_default())
                .collect()
        }
        let mpx3 = |path: &str| match path {
            "/detector" => {
                r#"{"Info":{"PixCount":131072,"RowLen":2,"NumberOfChips":2,"NumberOfRows":256,"MpxType":0,"ChipType":"MPX3"},"Config":{},"Layout":{
                "Original":{"Chips":[{"Chip":0,"X":0},{"Chip":1,"X":1}]},
                "Rotated":{"Chips":[{"Chip":1,"X":9},{"Chip":0,"X":8}]}}}"#
            }
            _ => "{}",
        };
        let tpx3 = |path: &str| match path {
            "/detector" => {
                r#"{"Info":{"PixCount":131072,"RowLen":2,"NumberOfChips":2,"NumberOfRows":256,"MpxType":0,"ChipType":"TPX3"},"Config":{},"Layout":{
                "Original":{"Chips":[{"Chip":0,"X":0},{"Chip":1,"X":1}]},
                "Rotated":{"Chips":[{"Chip":1,"X":9},{"Chip":0,"X":8}]}}}"#
            }
            _ => "{}",
        };
        let rotated_only = |path: &str| match path {
            "/detector" => {
                r#"{"Info":{"PixCount":131072,"RowLen":2,"NumberOfChips":2,"NumberOfRows":256,"MpxType":0,"ChipType":"TPX3"},"Config":{},"Layout":{
                "Rotated":{"Chips":[{"Chip":1,"X":9},{"Chip":"0","X":8}]}}}"#
            }
            _ => "{}",
        };
        assert_eq!(
            read("TPX_LAYOUT_MPX3", mpx3),
            [r#"{"Chip":0,"X":8}"#, r#"{"Chip":1,"X":9}"#]
        );
        assert_eq!(
            read("TPX_LAYOUT_TPX3", tpx3),
            [r#"{"Chip":0,"X":0}"#, r#"{"Chip":1,"X":1}"#]
        );
        assert_eq!(
            read("TPX_LAYOUT_ROT", rotated_only),
            ["", r#"{"Chip":1,"X":9}"#]
        );
    }

    /// The steps of a detector read, with each post's writes as
    /// `(reason, addr)` for the order checks.
    fn step_shape(read: &DetectorRead) -> Vec<String> {
        read.steps
            .iter()
            .map(|s| match s {
                DetectorStep::Trace(mask, line) => format!("trace {mask:?} {line}"),
                DetectorStep::Post(addr, _) => format!("post {addr}"),
            })
            .collect()
    }

    fn float_set(updates: &[ParamSetValue], reason: usize, addr: i32) -> Option<f64> {
        updates.iter().rev().find_map(|u| match u {
            ParamSetValue::Value {
                reason: r,
                addr: a,
                value: ParamValue::Float64(v),
            } if *r == reason && *a == addr => Some(*v),
            _ => None,
        })
    }

    /// C getDetector's order and Config readback (serval_http.cpp:1293-1557):
    /// the FLOW line, Health first, absent keys read as C's defaults, the
    /// Medipix3 defaults before the readback, one post per chip up to
    /// NumberOfChips with the board-1 rails stored for theirs.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_detector_read_publishes_the_config_readback_in_c_order() {
        let (url, _log) = logging_serval(&[], |path| match path {
            "/detector" => {
                r#"{"Info":{"PixCount":131072,"RowLen":2,"NumberOfChips":2,
                "NumberOfRows":256,"MpxType":0,"ChipType":"MPX3",
                "Boards":[{"ChipboardId":"B","IpAddress":"ip","Chips":[{"Name":"W1"},{"Id":7}]}]},
                "Health":[{"VDD":[1.5,0.1,0.6],"ChipTemperatures":[30.9,41]},
                          {"VDD":[2.5,0.2,0.7]}],
                "Config":{"Fan1PWM":12.7,"Polarity":"Negative","ChainMode":"LEADER",
                "TriggerMode":"SwReSt_SwReSp","nTriggers":5,"Tdc":["P0","PN0123"],
                "BothCounters":true,"GainMode":"LGM","ChargeSumming":1,"Colour":false,
                "PixelDepth":"24","CounterSelectIn":1,"IDelayConfig":[1,2,3,4]},
                "Chips":[{},{}]}"#
            }
            _ => "{}",
        });
        let ctx = idle_ctx_at("TPX_DET_READBACK", &url, None);
        let (p, ad) = (ctx.p, ctx.ad);
        let read = read_detector(&ctx.http, &ctx.shared, p, ad, true);
        assert!(read.ok);
        let shape = step_shape(&read);
        assert_eq!(
            shape,
            [
                format!(
                    "trace {:?} {}",
                    TraceMask::FLOW,
                    adlog::log(
                        adlog::GET_DETECTOR,
                        "Reading Detector Health, info, config, layout, chips"
                    )
                ),
                format!(
                    "trace {:?} {}",
                    TraceMask::FLOW,
                    adlog::log(
                        adlog::UPDATE_DETECTOR_FAMILY,
                        "Detector family MPX3 (MpxType=0 ChipType=MPX3 ChipboardId=B)"
                    )
                ),
                "post 0".into(),
                "post 1".into(),
                "post 0".into(),
            ]
        );
        let u = posted(&read, 0);
        assert_eq!(int_set(&u, p.http_code), Some(200));
        assert_eq!(int_set(&u, p.fan1_pw_m), Some(12));
        assert_eq!(int_set(&u, p.bias_enable), Some(0));
        assert_eq!(int_set(&u, p.polarity), Some(1));
        assert_eq!(int_set(&u, p.chain_mode), Some(1));
        assert_eq!(
            octet_set(&u, p.trigger_mode).as_deref(),
            Some("SwReSt_SwReSp")
        );
        assert_eq!(int_set(&u, ad.trigger_mode), Some(7));
        assert_eq!(int_set(&u, ad.num_images), Some(5));
        assert_eq!(octet_set(&u, p.tdc).as_deref(), Some(r#"["P0","PN0123"]"#));
        assert_eq!(
            (int_set(&u, p.tdc0), int_set(&u, p.tdc1)),
            (Some(3), Some(2))
        );
        assert_eq!(int_set(&u, p.both_counters), Some(1));
        assert_eq!(int_set(&u, p.gain_mode), Some(2));
        assert_eq!(int_set(&u, p.charge_summing), Some(1));
        assert_eq!(int_set(&u, p.colour), Some(0));
        assert_eq!(int_set(&u, p.pixel_depth), Some(24));
        assert_eq!(int_set(&u, p.counter_select_in), Some(1));
        assert_eq!(int_set(&u, p.i_delay3), Some(4));
        assert_eq!(octet_set(&u, p.boards_ch1).as_deref(), Some("W1"));
        assert_eq!(octet_set(&u, p.boards_ch2).as_deref(), Some("7"));
        assert_eq!(octet_set(&u, p.boards_ch3).as_deref(), Some(""));
        assert_eq!(octet_set(&u, p.boards2_id).as_deref(), Some(""));
        assert_eq!(octet_set(&u, p.iface_name).as_deref(), Some(""));
        assert_eq!(
            octet_set(&u, p.chip_temperature).as_deref(),
            Some("[30.9,41]")
        );
        assert_eq!(
            octet_set(&u, p.vd_d).as_deref(),
            Some("[[1.5,0.1,0.6],[2.5,0.2,0.7]]")
        );
        // Medipix3 stores only the temperature, truncated, and Adjust -1.
        assert_eq!(int_set(&u, p.chip_n_temperature), Some(30));
        assert_eq!(int_set(&u, p.adjust), Some(-1));
        // Board 1's rails are stored with chip 0's post and go out with
        // their own address's callbacks.
        assert_eq!(float_set(&u, p.chip_nv_dd, 3), Some(2.5));
        assert_eq!(float_set(&u, p.chip_nv_dd, 1), Some(0.1));
        assert_eq!(int_set(&posted(&read, 1), p.chip_n_temperature), Some(41));
    }

    /// A DAC that is not a number is C's json::exception: the chips before
    /// it are posted, the failure goes out with address 0's callbacks and
    /// the failing chip's partial writes are only stored
    /// (serval_http.cpp:854, :1558-1566).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_dac_that_is_not_a_number_fails_the_read_as_c() {
        let (url, _log) = logging_serval(&[], |path| match path {
            "/detector" => {
                r#"{"Info":{"PixCount":131072,"RowLen":2,"NumberOfChips":2,
                "NumberOfRows":256,"MpxType":6},"Config":{},
                "Chips":[{},{"DACs":{"Ibias_Preamp_ON":1,"Ibias_CP_PLL":"x"}}]}"#
            }
            _ => "{}",
        });
        let ctx = idle_ctx_at("TPX_DET_DAC_TYPE", &url, None);
        let (p, ad) = (ctx.p, ctx.ad);
        let read = read_detector(&ctx.http, &ctx.shared, p, ad, false);
        assert!(!read.ok);
        let shape = step_shape(&read);
        assert_eq!(
            &shape[shape.len() - 3..],
            [
                "post 0".to_string(),
                format!(
                    "trace {:?} {}",
                    TraceMask::ERROR,
                    adlog::err(
                        adlog::GET_DETECTOR,
                        "getDetector JSON error: [json.exception.type_error.302] \
                     type must be number, but is string"
                    )
                ),
                "post 0".to_string(),
            ]
        );
        let DetectorStep::Post(_, last) = read.steps.last().unwrap() else {
            unreachable!()
        };
        assert_eq!(int_set(last, p.det_connected), Some(0));
        assert_eq!(
            octet_set(last, ad.status_message).as_deref(),
            Some("Invalid detector response structure")
        );
        assert_eq!(int_set(last, ad.status), Some(ADStatus::Error as i32));
        assert!(posted(&read, 1).is_empty());
    }

    /// C updateDetectorHealthFromJson's lines come before any post
    /// (serval_http.cpp:723-733, :803-805).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_malformed_health_is_traced_as_c() {
        async fn first_trace(name: &str, body_for: fn(&str) -> &'static str) -> String {
            let (url, _log) = logging_serval(&[], body_for);
            let ctx = idle_ctx_at(name, &url, None);
            ctx.shared.note_family(DetectorFamily::Tpx3);
            let read = read_detector(&ctx.http, &ctx.shared, ctx.p, ctx.ad, false);
            assert!(read.ok);
            step_shape(&read).swap_remove(1)
        }
        let line = first_trace("TPX_HEALTH_EMPTY", |path| match path {
            "/detector" => {
                r#"{"Info":{"PixCount":65536,"RowLen":1,"NumberOfChips":1,
                "NumberOfRows":256,"MpxType":6},"Config":{},"Health":[]}"#
            }
            _ => "{}",
        })
        .await;
        assert_eq!(
            line,
            format!(
                "trace {:?} {}",
                TraceMask::IO_DRIVER,
                adlog::log(
                    adlog::UPDATE_DETECTOR_HEALTH_FROM_JSON,
                    "getDetector: Health array is empty"
                )
            )
        );
        let line = first_trace("TPX_HEALTH_TYPE", |path| match path {
            "/detector" => {
                r#"{"Info":{"PixCount":65536,"RowLen":1,"NumberOfChips":1,
                "NumberOfRows":256,"MpxType":6},"Config":{},"Health":"x"}"#
            }
            _ => "{}",
        })
        .await;
        assert_eq!(
            line,
            format!(
                "trace {:?} {}",
                TraceMask::ERROR,
                adlog::err(
                    adlog::UPDATE_DETECTOR_HEALTH_FROM_JSON,
                    "getDetector: Health has unexpected JSON type (string)"
                )
            )
        );
    }

    /// A good detector read reconnects the detector (serval_http.cpp:1556).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_good_detector_read_sets_det_connected() {
        let (url, _log) = logging_serval(&[], |path| match path {
            "/detector" => DETECTOR,
            _ => "{}",
        });
        let ctx = Arc::new(idle_ctx_at("TPX_DET_OK", &url, None));
        ctx.set(
            0,
            vec![ParamSetValue::new(
                ctx.p.det_connected,
                0,
                ParamValue::Int32(0),
            )],
        )
        .await;
        assert!(refresh_detector(&ctx).await);
        assert_eq!(
            ctx.handle.read_int32(ctx.p.det_connected, 0).await.unwrap(),
            1
        );
    }

    /// A failed or invalid detector read is published as C getDetector
    /// does: DetConnected 0, WriteMsg, StatusMessage, ADStatus and an ERR
    /// line (serval_http.cpp:1300-1323).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_failed_detector_read_is_published_as_c() {
        async fn read(
            name: &str,
            fail: &'static [&'static str],
            body_for: fn(&str) -> &'static str,
        ) -> (Arc<Ctx>, String, String) {
            let (url, _log) = logging_serval(fail, body_for);
            let ctx = Arc::new(idle_ctx_at(name, &url, None));
            let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
            let file = std::fs::File::create(&path).unwrap();
            ctx.handle.trace().set_trace_file(
                Some(name),
                epics_rs::asyn::trace::TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
            );
            ctx.handle
                .trace()
                .set_trace_mask(Some(name), TraceMask::ERROR);
            ctx.set(
                0,
                vec![
                    ParamSetValue::new(ctx.p.det_connected, 0, ParamValue::Int32(1)),
                    ParamSetValue::new(
                        ctx.p.write_msg,
                        0,
                        ParamValue::Octet(b"old".to_vec().into()),
                    ),
                ],
            )
            .await;
            assert!(!refresh_detector(&ctx).await);
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            (ctx, url, text)
        }
        async fn octet(ctx: &Ctx, reason: usize) -> String {
            let v = ctx.handle.read_octet(reason, 0, 128).await.unwrap();
            String::from_utf8(v)
                .unwrap()
                .trim_end_matches('\0')
                .to_string()
        }

        let (ctx, url, text) = read("TPX_DET_500", &["/detector"], |_| "{}").await;
        assert_eq!(
            ctx.handle.read_int32(ctx.p.det_connected, 0).await.unwrap(),
            0
        );
        assert_eq!(octet(&ctx, ctx.p.write_msg).await, "");
        assert_eq!(
            octet(&ctx, ctx.ad.status_message).await,
            "Failed to read detector information"
        );
        assert_eq!(
            ctx.handle.read_int32(ctx.ad.status, 0).await.unwrap(),
            ADStatus::Disconnected as i32
        );
        let line = format!("getDetector GET /detector: GET {url}/detector HTTP 500: ");
        assert!(
            text.contains("ERROR | ADTimePix::") && text.contains(&line),
            "{text}"
        );

        for (name, body_for, reason) in [
            (
                "TPX_DET_INFO",
                (|_: &str| "{}") as fn(&str) -> &'static str,
                "detector response is missing the Info object",
            ),
            (
                "TPX_DET_ROWS",
                |_: &str| r#"{"Info":{"PixCount":65536,"RowLen":1,"NumberOfChips":1,"NumberOfRows":0,"MpxType":3},"Config":{}}"#,
                "detector response contains invalid geometry",
            ),
        ] {
            let (ctx, _url, text) = read(name, &[], body_for).await;
            let message = format!("Invalid detector response: {reason}");
            assert_eq!(
                ctx.handle.read_int32(ctx.p.det_connected, 0).await.unwrap(),
                0
            );
            assert_eq!(octet(&ctx, ctx.p.write_msg).await, message);
            assert_eq!(octet(&ctx, ctx.ad.status_message).await, message);
            assert_eq!(
                ctx.handle.read_int32(ctx.ad.status, 0).await.unwrap(),
                ADStatus::Error as i32
            );
            assert!(
                text.contains(&adlog::err(
                    "asynStatus ADTimePix::getDetector(bool)",
                    &format!("getDetector: {message}")
                )),
                "{text}"
            );
        }
    }

    /// The reconnect refresh reads the destination, the detector and the
    /// measurement config; a failed readback reports the refresh incomplete
    /// and is retried on the next poll (serval_http.cpp:287-312, :408-420).
    #[tokio::test(flavor = "multi_thread")]
    async fn an_incomplete_reconnect_refresh_is_reported_and_retried() {
        let (url, log) = logging_serval(&["/server/destination"], |path| match path {
            "/dashboard" => r#"{"Server":{},"Detector":{"DetectorType":"TPX3"}}"#,
            "/detector" => DETECTOR,
            _ => "{}",
        });
        let ctx = Arc::new(idle_ctx_at("TPX_RECONNECT", &url, None));
        let path = std::env::temp_dir().join(format!("TPX_RECONNECT-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        ctx.handle.trace().set_trace_file(
            Some("TPX_RECONNECT"),
            epics_rs::asyn::trace::TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        ctx.handle
            .trace()
            .set_trace_mask(Some("TPX_RECONNECT"), TraceMask::ERROR);
        let destination_reads = || {
            log.lock()
                .iter()
                .filter(|r| r.starts_with("GET /server/destination "))
                .count()
        };
        let mut was_up = false;
        for expected in 1..=2 {
            was_up = poll_connection(&ctx, was_up).await;
            assert!(!was_up);
            assert_eq!(destination_reads(), expected);
            assert_eq!(
                ctx.handle
                    .read_octet(ctx.ad.status_message, 0, 64)
                    .await
                    .unwrap(),
                b"Connected; reconnect state refresh incomplete\0"
            );
            assert_eq!(
                ctx.handle.read_int32(ctx.ad.status, 0).await.unwrap(),
                ADStatus::Error as i32
            );
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let line = adlog::err(
            "asynStatus ADTimePix::refreshOnReconnect()",
            "Reconnect state refresh incomplete: destination=0 detector=1 measurementConfig=1",
        );
        assert_eq!(text.matches(&line).count(), 2, "{text}");
    }

    /// An idle dashboard has no `Measurement`, and the last status stays;
    /// a non-string status is shown as its JSON (acquire.cpp:84-111).
    #[test]
    fn the_dashboard_measurement_status_follows_c() {
        let status = |dashboard: &str| {
            dashboard_measurement_status(&serde_json::from_str(dashboard).unwrap()).cloned()
        };
        assert_eq!(status(r#"{"Measurement":null}"#), None);
        assert_eq!(status(r#"{}"#), None);
        assert_eq!(
            status(r#"{"Measurement":{"Status":"DA_IDLE"}}"#),
            Some("DA_IDLE".into())
        );
        assert_eq!(
            status(r#"{"Measurement":{"Info":{},"Status":"DA_IDLE"}}"#),
            Some("DA_IDLE".into())
        );
        let p = idle_ctx("TPX_DASH_JSON").p;
        let mut updates = Vec::new();
        measurement_status(p, &serde_json::json!({"a": 1}), &mut updates);
        assert_eq!(octet_set(&updates, p.status).as_deref(), Some(r#"{"a":1}"#));
        assert_eq!(int_set(&updates, p.pipeline_state), Some(-1));
    }

    /// Every dashboard read publishes the free space, the lower limit and
    /// the measurement status, and a failed read clears the status
    /// (serval_http.cpp:219-249, acquire.cpp:84-111).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_dashboard_read_publishes_the_measurement_status() {
        let (url, _log) = logging_serval(&[], |path| match path {
            "/dashboard" => {
                r#"{"Server":{"DiskSpace":[{"FreeSpace":7000000000,"LowerLimit":42}]},
                    "Detector":{"DetectorType":"TPX3"},
                    "Measurement":{"Status":"DA_IDLE","Info":{"Status":"DA_RECORDING"}}}"#
            }
            _ => "{}",
        });
        let ctx = Arc::new(idle_ctx_at("TPX_DASH_MEAS", &url, None));
        let (h, p) = (&ctx.handle, ctx.p);
        assert!(check_connection(&ctx).await);
        assert_eq!(h.read_int64(p.free_space, 0).await.unwrap(), 7_000_000_000);
        assert_eq!(h.read_int64(p.lower_limit, 0).await.unwrap(), 42);
        assert_eq!(
            h.read_octet(p.status, 0, 64).await.unwrap(),
            b"DA_RECORDING\0"
        );
        assert_eq!(h.read_int32(p.pipeline_state, 0).await.unwrap(), 1);

        let ctx = Arc::new(idle_ctx_at("TPX_DASH_GONE", "http://127.0.0.1:1", None));
        let (h, p) = (&ctx.handle, ctx.p);
        ctx.set(
            0,
            vec![
                ParamSetValue::new(p.status, 0, ParamValue::Octet(b"DA_IDLE".to_vec().into())),
                ParamSetValue::new(p.pipeline_state, 0, ParamValue::Int32(3)),
            ],
        )
        .await;
        assert!(!check_connection(&ctx).await);
        assert_eq!(h.read_octet(p.status, 0, 64).await.unwrap(), b"\0");
        assert_eq!(h.read_int32(p.pipeline_state, 0).await.unwrap(), -1);
    }

    /// RefreshConnection is C checkConnection(): it publishes the HTTP status
    /// and runs no reconnect refresh, while the periodic poll leaves the
    /// last operator HTTP status alone (serval_http.cpp:363-420,
    /// ADTimePix.cpp:563-568).
    #[tokio::test(flavor = "multi_thread")]
    async fn only_refresh_connection_publishes_the_http_status() {
        let (url, log) = logging_serval(&[], |path| match path {
            "/dashboard" => r#"{"Server":{},"Detector":{"DetectorType":"TPX3"}}"#,
            _ => "{}",
        });
        let ctx = Arc::new(idle_ctx_at("TPX_REFRESH_CONN", &url, None));
        let (h, p) = (&ctx.handle, ctx.p);
        let (cmd_tx, cmd_rx) = rt::command_channel::<Command>(1);
        let (acq_tx, _acq_rx) = rt::command_channel::<()>(1);
        tokio::spawn(connection_poll(ctx.clone(), cmd_rx, acq_tx, false, false));

        ctx.set_int(p.http_code, 0, 409).await;
        cmd_tx.send(Command::RefreshConnection).await.unwrap();
        for _ in 0..200 {
            if h.read_int32(p.http_code, 0).await.unwrap() == 200 {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(h.read_int32(p.http_code, 0).await.unwrap(), 200);
        assert_eq!(
            log.lock()
                .iter()
                .filter(|r| !r.starts_with("GET /dashboard "))
                .count(),
            0
        );
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_poll_leaves_the_http_status_alone() {
        let (url, _log) = logging_serval(&[], |path| match path {
            "/dashboard" => r#"{"Server":{},"Detector":{"DetectorType":"TPX3"}}"#,
            _ => "{}",
        });
        let ctx = Arc::new(idle_ctx_at("TPX_POLL_HTTP", &url, None));
        ctx.set_int(ctx.p.http_code, 0, 409).await;
        assert!(poll_connection(&ctx, true).await);
        assert_eq!(
            ctx.handle.read_int32(ctx.p.http_code, 0).await.unwrap(),
            409
        );
        assert_eq!(
            ctx.handle
                .read_int32(ctx.p.serval_connected, 0)
                .await
                .unwrap(),
            1
        );
    }

    /// A 1 Hz Health scan does not hold the connection poll off
    /// (serval_http.cpp:400-402).
    #[tokio::test(flavor = "multi_thread")]
    async fn commands_do_not_delay_the_connection_poll() {
        let (url, log) = logging_serval(&[], |path| match path {
            "/dashboard" => r#"{"Server":{},"Detector":{"DetectorType":"TPX3"}}"#,
            _ => "{}",
        });
        let ctx = Arc::new(idle_ctx_at("TPX_POLL_TIMER", &url, None));
        let (cmd_tx, cmd_rx) = rt::command_channel::<Command>(1);
        let (acq_tx, _acq_rx) = rt::command_channel::<()>(1);
        tokio::spawn(connection_poll(ctx.clone(), cmd_rx, acq_tx, false, false));
        // Only the poll's reconnect refresh reads the destination.
        let polled = || {
            log.lock()
                .iter()
                .any(|r| r.starts_with("GET /server/destination "))
        };
        let start = std::time::Instant::now();
        while !polled() && start.elapsed() < CONNECTION_POLL + Duration::from_secs(2) {
            cmd_tx.send(Command::RefreshStatus).await.unwrap();
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
        assert!(polled());
    }

    /// serval_stream.cpp:177-183 allocates the frame with a NULL pData, so a
    /// frame over the pool's maxMemory is refused.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_frame_over_the_pool_limit_is_refused() {
        let mut ctx = idle_ctx("TPX3_POOL_LIMIT");
        ctx.pool = DriverPool::standalone(8);
        let frame = crate::stream::ImageFrame {
            width: 2,
            height: 2,
            format: crate::stream::PixelFormat::UInt32,
            frame_number: 1,
            time_at_frame: 0.0,
            threshold_id: 0,
            integration_size: 0,
            pixels: Vec::new(),
            header: serde_json::Value::Null,
        };
        assert!(image_array(&ctx, &frame, vec![1; 4], "refused\n").is_none());
    }
}
