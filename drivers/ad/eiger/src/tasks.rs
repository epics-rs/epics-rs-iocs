//! The background tasks (port of the ten `epicsThreadCreate` threads in
//! `eigerDetector`, eigerDetector.cpp:436-474).
//!
//! Each task is an OS thread with its own single-threaded async runtime, so a
//! blocking HTTP call in one never stalls another — the same isolation C gets
//! from ten `epicsThread`s. The C `epicsMessageQueue`s become
//! [`rt::command_channel`]s and the `epicsEvent`s become either a channel
//! message or an atomic flag, whichever matches how the event is consumed.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicUsize, Ordering};
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::driver::ndarray_driver::check_path_str;
use epics_rs::ad_core::driver::{ADDriver, ADStatus};
use epics_rs::ad_core::error::ADResult;
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType, NDDataView, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool, PublishedArray};
use epics_rs::ad_core::params::ad_driver::ADDriverParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

use crate::h5;
use crate::param::{ParamOps, ParamUpdate};
use crate::params::{
    EigerParams, MAX_THRESHOLDS, Model, SOURCE_FILEWRITER, SOURCE_STREAM, STREAM_VERSION_STREAM,
    TRIGGER_MODE_CONTINUOUS, TRIGGER_MODE_EXTE, TRIGGER_MODE_EXTS, TRIGGER_MODE_INTE,
    TRIGGER_MODE_INTS,
};
use crate::rest::{self, ApiVersion, RestApi};
use crate::stream::{StreamApi, StreamMessage};
use crate::stream2::{Frame, Stream2Api};
use crate::tiff;

/// The only thing the control task waits for on a channel.
///
/// Stop and Trigger are C `epicsEvent`s consumed from *inside* the acquisition
/// sequence, not at its top, so they are flags ([`Flags`]) rather than channel
/// messages — a message would sit unread until the sequence finished, which is
/// exactly when it is no longer needed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CtlCommand {
    Start,
}

/// The C `epicsEvent`s the acquisition sequence polls (`mStopEvent`,
/// `mTriggerEvent`).
#[derive(Debug, Default)]
pub struct Flags {
    stop: AtomicBool,
    trigger: AtomicBool,
    /// Which of C's `mStreamAPI` / `mStream2API` exist ([`STREAM_API`],
    /// [`STREAM2_API`]); the stream task holds a socket only for these.
    stream_apis: AtomicU8,
}

const STREAM_API: u8 = 1;
const STREAM2_API: u8 = 2;

/// The driver's end of every task signal.
#[derive(Clone)]
pub struct Signals {
    ctl_tx: rt::CommandSender<CtlCommand>,
    init_tx: rt::CommandSender<()>,
    restart_tx: rt::CommandSender<()>,
    stream_apis_tx: rt::CommandSender<()>,
    flags: Arc<Flags>,
}

/// The task ends of [`Signals`].
pub struct Receivers {
    ctl: rt::CommandReceiver<CtlCommand>,
    init: rt::CommandReceiver<()>,
    restart: rt::CommandReceiver<()>,
    stream_apis: rt::CommandReceiver<()>,
}

impl Signals {
    pub fn start(&self) {
        let _ = self.ctl_tx.try_send(CtlCommand::Start);
    }

    pub fn stop(&self) {
        self.flags.stop.store(true, Ordering::Release);
    }

    pub fn trigger(&self) {
        self.flags.trigger.store(true, Ordering::Release);
    }

    pub fn initialize(&self) {
        let _ = self.init_tx.try_send(());
    }

    pub fn restart(&self) {
        let _ = self.restart_tx.try_send(());
    }

    /// `Some(version)` creates that version's stream API, `None` deletes both
    /// (C `writeInt32` on DataSource/StreamVersion, eigerDetector.cpp:553-584),
    /// so a DataSource other than stream leaves no PULL socket taking the
    /// detector's frames from other clients.
    pub fn stream_apis(&self, version: Option<i32>) {
        match version {
            Some(STREAM_VERSION_STREAM) => self
                .flags
                .stream_apis
                .fetch_or(STREAM_API, Ordering::AcqRel),
            Some(_) => self
                .flags
                .stream_apis
                .fetch_or(STREAM2_API, Ordering::AcqRel),
            None => self.flags.stream_apis.swap(0, Ordering::AcqRel),
        };
        let _ = self.stream_apis_tx.try_send(());
    }
}

const QUEUE_CAPACITY: usize = 16;
/// The monitor task's poll rate cap (C `epicsThreadSleep(0.1)`).
const MONITOR_PERIOD: Duration = Duration::from_millis(100);

/// The NDArray streams the driver publishes on.
///
/// C fans images out on asyn *addresses* of one port: 0 for every frame,
/// `threshold+1` for the per-threshold streams and 10 (`MONITOR_ASYN_ADDRESS`)
/// for the monitor. `ad-core-rs` 0.23 routes NDArrays by *port name* — a plugin
/// binds to `NDARRAY_PORT`, and `NDArrayAddr` is accepted but never consulted —
/// so each of C's addresses becomes its own named output here. The st.cmd sets
/// `NDARRAY_PORT=$(PORT)_TH1` where the C one set `NDArrayAddr=1`.
#[derive(Clone)]
pub struct Outputs {
    pub main: Arc<parking_lot::Mutex<NDArrayOutput>>,
    pub thresholds: Vec<Arc<parking_lot::Mutex<NDArrayOutput>>>,
    pub monitor: Arc<parking_lot::Mutex<NDArrayOutput>>,
}

impl Outputs {
    pub fn new() -> Self {
        Self {
            main: Arc::new(parking_lot::Mutex::new(NDArrayOutput::new())),
            thresholds: (0..MAX_THRESHOLDS)
                .map(|_| Arc::new(parking_lot::Mutex::new(NDArrayOutput::new())))
                .collect(),
            monitor: Arc::new(parking_lot::Mutex::new(NDArrayOutput::new())),
        }
    }
}

impl Default for Outputs {
    fn default() -> Self {
        Self::new()
    }
}

/// Everything the tasks share.
pub struct Shared {
    pub ops: Arc<ParamOps>,
    pub handle: PortHandle,
    pub p: EigerParams,
    pub ad: ADDriverParams,
    pub model: Model,
    pub api: ApiVersion,
    pub hostname: String,
    pub outputs: Outputs,
    pub pool: DriverPool,

    /// Files downloaded but not yet reaped (mirrored into `PENDING_FILES`).
    pending_files: AtomicI32,
    /// The control task asks the poll task to give up on missing files.
    poll_stop: AtomicBool,
    poll_complete: AtomicBool,
    stream_complete: AtomicBool,
    /// C `mStopEvent` / `mTriggerEvent`, shared with the driver.
    flags: Arc<Flags>,
}

impl Shared {
    fn rest(&self) -> &RestApi {
        &self.ops.rest
    }

    fn trace(&self) -> &crate::trace::Trace {
        &self.ops.trace
    }

    async fn apply(&self, updates: Vec<ParamUpdate>) {
        if updates.is_empty() {
            return;
        }
        let values = updates
            .into_iter()
            .map(|u| match u {
                ParamUpdate::Int32(reason, value) => {
                    ParamSetValue::new(reason, 0, ParamValue::Int32(value))
                }
                ParamUpdate::Float64(reason, value) => {
                    ParamSetValue::new(reason, 0, ParamValue::Float64(value))
                }
                ParamUpdate::Octet(reason, value) => {
                    ParamSetValue::new(reason, 0, ParamValue::Octet(value.into_bytes().into()))
                }
            })
            .collect();
        let _ = self.handle.set_params_and_notify(0, values).await;
    }

    async fn set_int(&self, reason: usize, value: i32) {
        self.apply(vec![ParamUpdate::Int32(reason, value)]).await;
    }

    async fn set_str(&self, reason: usize, value: &str) {
        self.apply(vec![ParamUpdate::Octet(reason, value.to_string())])
            .await;
    }

    async fn get_int(&self, reason: usize) -> i32 {
        self.handle.read_int32(reason, 0).await.unwrap_or(0)
    }

    async fn get_f64(&self, reason: usize) -> f64 {
        self.handle.read_float64(reason, 0).await.unwrap_or(0.0)
    }

    async fn get_str(&self, reason: usize) -> String {
        match self.handle.get_string_param(reason, 0, 256).await {
            Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
            Err(_) => String::new(),
        }
    }

    /// Fetch a remote parameter and push the result into the param library.
    async fn fetch(&self, reason: usize) {
        if let Ok(updates) = self.ops.fetch(reason) {
            self.apply(updates).await;
        }
    }

    /// Fetch a remote parameter and return its string value.
    async fn fetch_str(&self, reason: usize) -> String {
        self.fetch(reason).await;
        self.get_str(reason).await
    }

    async fn put_int(&self, reason: usize, value: i32) {
        if let Ok(updates) = self.ops.put_int(reason, value) {
            self.apply(updates).await;
        }
    }

    async fn add_pending(&self, delta: i32) {
        let n = self.pending_files.fetch_add(delta, Ordering::AcqRel) + delta;
        self.set_int(self.p.pending_files, n).await;
    }
}

/// One file the FileWriter produced (C `file_t`).
struct FileJob {
    name: String,
    save: bool,
    parse: bool,
    /// The reap task deletes the file from the detector when the last reference
    /// goes; a failed local save clears this so the only copy is not destroyed.
    remove: AtomicBool,
    perms: u32,
    refs: AtomicUsize,
    data: parking_lot::Mutex<Option<Vec<u8>>>,
}

/// A whole acquisition's worth of files (C `acquisition_t`).
struct Acquisition {
    pattern: String,
    sequence_id: i32,
    n_data_files: usize,
    save_files: bool,
    parse_files: bool,
    remove_files: bool,
    perms: u32,
}

/// The channel ends the control task keeps.
pub struct TaskChannels {
    ctl_rx: rt::CommandReceiver<CtlCommand>,
    poll_tx: rt::CommandSender<Acquisition>,
    poll_done_rx: rt::CommandReceiver<bool>,
    stream_start_tx: rt::CommandSender<()>,
    stream_done_rx: rt::CommandReceiver<bool>,
}

/// Start every background task. Returns the thread handles.
pub fn start(shared: Arc<Shared>, rx: Receivers) -> Vec<std::thread::JoinHandle<()>> {
    let Receivers {
        ctl: ctl_rx,
        init: init_rx,
        restart: restart_rx,
        stream_apis: stream_apis_rx,
    } = rx;
    let (poll_tx, poll_rx) = rt::command_channel::<Acquisition>(1);
    let (poll_done_tx, poll_done_rx) = rt::command_channel::<bool>(1);
    let (download_tx, download_rx) = rt::command_channel::<Arc<FileJob>>(QUEUE_CAPACITY);
    let (parse_tx, parse_rx) = rt::command_channel::<Arc<FileJob>>(QUEUE_CAPACITY);
    let (save_tx, save_rx) = rt::command_channel::<Arc<FileJob>>(QUEUE_CAPACITY);
    let (reap_tx, reap_rx) = rt::command_channel::<Arc<FileJob>>(QUEUE_CAPACITY * 2);
    let (stream_start_tx, stream_start_rx) = rt::command_channel::<()>(1);
    let (stream_done_tx, stream_done_rx) = rt::command_channel::<bool>(1);

    let mut threads = Vec::new();

    {
        let s = shared.clone();
        let ch = TaskChannels {
            ctl_rx,
            poll_tx,
            poll_done_rx,
            stream_start_tx,
            stream_done_rx,
        };
        threads.push(rt::run_thread_named(
            "eigerControlTask",
            move || async move {
                control_task(s, ch).await;
            },
        ));
    }
    {
        let s = shared.clone();
        threads.push(rt::run_thread_named("eigerPollTask", move || async move {
            poll_task(s, poll_rx, download_tx, poll_done_tx).await;
        }));
    }
    {
        let s = shared.clone();
        let reap = reap_tx.clone();
        threads.push(rt::run_thread_named(
            "eigerDownloadTask",
            move || async move {
                download_task(s, download_rx, parse_tx, save_tx, reap).await;
            },
        ));
    }
    {
        let s = shared.clone();
        let reap = reap_tx.clone();
        threads.push(rt::run_thread_named("eigerParseTask", move || async move {
            parse_task(s, parse_rx, reap).await;
        }));
    }
    {
        let s = shared.clone();
        let reap = reap_tx.clone();
        threads.push(rt::run_thread_named("eigerSaveTask", move || async move {
            save_task(s, save_rx, reap).await;
        }));
    }
    {
        let s = shared.clone();
        threads.push(rt::run_thread_named("eigerReapTask", move || async move {
            reap_task(s, reap_rx).await;
        }));
    }
    {
        let s = shared.clone();
        threads.push(rt::run_thread_named(
            "eigerMonitorTask",
            move || async move {
                monitor_task(s).await;
            },
        ));
    }
    {
        let s = shared.clone();
        threads.push(rt::run_thread_named(
            "eigerStreamTask",
            move || async move {
                stream_task(s, stream_start_rx, stream_apis_rx, stream_done_tx).await;
            },
        ));
    }
    {
        let s = shared.clone();
        threads.push(rt::run_thread_named(
            "eigerInitializeTask",
            move || async move {
                command_task(s, init_rx, "initialize").await;
            },
        ));
    }
    {
        let s = shared;
        threads.push(rt::run_thread_named(
            "eigerRestartTask",
            move || async move {
                command_task(s, restart_rx, "restart").await;
            },
        ));
    }

    threads
}

/// The `initialize` and `restart` tasks (C `initializeTask` / `restartTask`).
///
/// Both are long REST commands that must not block the control task, and both
/// clear their own push-button parameter when they are done.
async fn command_task(s: Arc<Shared>, mut rx: rt::CommandReceiver<()>, which: &str) {
    let (task, failed) = match which {
        "initialize" => ("initializeTask", "Failed to initialize"),
        _ => ("restartTask", "Failed to restart"),
    };
    while rx.recv().await.is_some() {
        s.trace().warning(task, &format!("Sending {which} command"));
        let (result, param) = match which {
            "initialize" => (s.rest().initialize(), s.p.initialize),
            _ => (s.rest().restart(), s.p.restart),
        };
        s.set_int(param, 0).await;
        if result.is_err() {
            s.trace().err(task, failed);
        }
    }
}

// ---------------------------------------------------------------------------
// Control
// ---------------------------------------------------------------------------

async fn control_task(s: Arc<Shared>, mut ch: TaskChannels) {
    loop {
        if s.get_int(s.ad.status).await == ADStatus::Idle as i32 {
            s.set_str(s.ad.status_message, "Ready").await;
        }

        // Wait for a start.
        if ch.ctl_rx.recv().await.is_none() {
            return;
        }

        s.flags.stop.store(false, Ordering::Release);
        s.flags.trigger.store(false, Ordering::Release);
        // A previous abort can leave a completion in the done channels.
        let _ = ch.poll_done_rx.try_recv();
        let _ = ch.stream_done_rx.try_recv();

        // Continuous mode re-arms until an explicit stop; C signals mStartEvent
        // back to itself, which is this loop.
        while acquire(&s, &mut ch).await {}
    }
}

/// One acquisition (C `controlTask`'s loop body, eigerDetector.cpp:820-1099).
///
/// Returns `true` when the detector should be re-armed immediately (continuous
/// trigger mode with no stop in between).
async fn acquire(s: &Arc<Shared>, ch: &mut TaskChannels) -> bool {
    let p = &s.p;

    // Latch the parameters this acquisition runs with.
    let data_source = s.get_int(p.data_source).await;
    let fw_enable = s.get_int(p.fw_enable).await != 0;
    let stream_enable = s.get_int(p.stream_enable).await != 0;
    let save_files = s.get_int(p.save_files).await != 0;
    let n_images_per_file = s.get_int(p.fw_nimgs_per_file).await.max(1);
    let acquire_period = s.get_f64(p.acquire_period).await;
    let mut acquire_time = s.get_f64(p.acquire_time).await;
    let num_images = s.get_int(p.num_images).await;
    let num_triggers = s.get_int(p.n_triggers).await;
    let trigger_mode = s.get_int(p.trigger_mode).await;
    let manual_trigger = s.get_int(p.manual_trigger).await != 0;
    let remove_files = s.get_int(p.fw_auto_remove).await != 0;
    let file_perms = s.get_int(p.file_perms).await;
    let pattern = s.get_str(p.fw_name_pattern).await;

    let mut err = None;
    if data_source == SOURCE_FILEWRITER && !fw_enable {
        err = Some("FileWriter API is disabled");
    } else if data_source == SOURCE_STREAM && !stream_enable {
        err = Some("Stream API is disabled");
    }
    if fw_enable && save_files {
        // C `checkPath(); getIntegerParam(NDFilePathExists, ...)`
        // (eigerDetector.cpp:870-871). An empty FilePath leaves both params as
        // they were; on a never-defined NDFilePathExists C then reads an
        // uninitialized int (a C defect), which rs takes as missing.
        let base = &s.ad.base;
        let mut path = s.get_str(base.file_path).await;
        if !path.is_empty() {
            let exists = check_path_str(&mut path);
            s.apply(vec![
                ParamUpdate::Octet(base.file_path, path),
                ParamUpdate::Int32(base.file_path_exists, i32::from(exists)),
            ])
            .await;
        }
        let exists = s.handle.read_int32(base.file_path_exists, 0).await;
        if !matches!(exists, Ok(v) if v != 0) {
            err = Some("Invalid file path");
        }
    }
    if let Some(err) = err {
        // C prints only the file-path refusal.
        if err == "Invalid file path" {
            s.trace().err("controlTask", err);
        }
        s.set_int(s.ad.acquire, 0).await;
        s.set_int(s.ad.status, ADStatus::Error as i32).await;
        s.set_str(s.ad.status_message, err).await;
        return false;
    }

    // INTE and EXTE take one image per trigger; restore the user's value after.
    let saved_num_images = num_images;
    let mut num_images = num_images;
    if trigger_mode == TRIGGER_MODE_INTE || trigger_mode == TRIGGER_MODE_EXTE {
        num_images = 1;
        s.put_int(p.num_images, num_images).await;
    }

    s.set_str(s.ad.status_message, "Arming").await;
    let arm_start = std::time::Instant::now();
    let armed = s.rest().arm();
    s.trace().flow(
        "controlTask",
        format_args!(
            "arming time {:.6}",
            CFloat(arm_start.elapsed().as_secs_f64())
        ),
    );
    let sequence_id = match armed {
        Ok(id) => id,
        Err(_) => {
            s.trace().err("controlTask", "Failed to arm the detector");
            s.set_int(s.ad.acquire, 0).await;
            s.set_int(s.ad.status, ADStatus::Error as i32).await;
            s.set_str(s.ad.status_message, "Failed to arm the detector")
                .await;
            if saved_num_images != num_images {
                s.put_int(p.num_images, saved_num_images).await;
            }
            return false;
        }
    };

    s.set_int(s.ad.num_images_counter, 0).await;
    s.set_str(s.ad.status_message, "Armed").await;
    s.put_int(p.sequence_id, sequence_id).await;
    s.put_int(p.armed, 1).await;

    let mut wait_poll = false;
    let mut wait_stream = false;

    if data_source == SOURCE_FILEWRITER || (fw_enable && save_files) {
        let n_images_total = i64::from(num_images) * i64::from(num_triggers.max(1));
        let per_file = i64::from(n_images_per_file).max(1);
        let n_data_files = ((n_images_total + per_file - 1) / per_file).max(0) as usize;
        s.poll_complete.store(false, Ordering::Release);
        s.poll_stop.store(false, Ordering::Release);
        let acq = Acquisition {
            pattern: pattern.clone(),
            sequence_id,
            n_data_files,
            save_files,
            parse_files: data_source == SOURCE_FILEWRITER,
            remove_files,
            perms: (file_perms & 0o777) as u32,
        };
        wait_poll = ch.poll_tx.try_send(acq).is_ok();
    }

    if data_source == SOURCE_STREAM {
        s.stream_complete.store(false, Ordering::Release);
        wait_stream = ch.stream_start_tx.try_send(()).is_ok();
    }

    let msg = if trigger_mode == TRIGGER_MODE_EXTS || trigger_mode == TRIGGER_MODE_EXTE {
        "Waiting for external triggers (press Stop when done)"
    } else if manual_trigger {
        "Waiting for manual triggers"
    } else {
        "Triggering"
    };
    s.set_str(s.ad.status_message, msg).await;

    // Internal triggering: the driver issues each trigger itself.
    if matches!(
        trigger_mode,
        TRIGGER_MODE_INTS | TRIGGER_MODE_INTE | TRIGGER_MODE_CONTINUOUS
    ) {
        let mut trigger_timeout = 0.0;
        if trigger_mode == TRIGGER_MODE_INTS || trigger_mode == TRIGGER_MODE_CONTINUOUS {
            trigger_timeout = acquire_period * f64::from(num_images) + 10.0;
            if s.model.has_thresholds_1_2()
                && let Some(delay) = p.trigger_start_delay
            {
                trigger_timeout += s.get_f64(delay).await;
            }
        }

        let mut triggers = 0;
        while s.get_int(s.ad.status).await == ADStatus::Acquire as i32 && triggers < num_triggers {
            let mut do_trigger = true;
            if manual_trigger {
                do_trigger = wait_flag(&s.flags.trigger, Duration::from_millis(100)).await;
            }

            // The exposure time can change between manual triggers.
            if trigger_mode == TRIGGER_MODE_INTE {
                acquire_time = s.get_f64(p.acquire_time).await;
                trigger_timeout = acquire_time + 1.0;
            }

            if do_trigger {
                s.trace().flow(
                    "controlTask",
                    format_args!(
                        "sending trigger {}/{num_triggers}. timeout={trigger_timeout:.6}, exposure={acquire_time:.6}",
                        triggers + 1, trigger_timeout = CFloat(trigger_timeout), acquire_time = CFloat(acquire_time)
                    ),
                );
                set_shutter(s, true).await;
                let exposure = (trigger_mode == TRIGGER_MODE_INTE).then_some(acquire_time);
                let timeout = Duration::from_secs_f64(trigger_timeout.max(0.0));
                let _ = s.rest().trigger(timeout, exposure);
                set_shutter(s, false).await;
                triggers += 1;
            }
        }
    }

    // Wait for the detector to leave the acquisition states, for the expected
    // image count to arrive, or for a stop.
    s.trace().flow(
        "controlTask",
        format_args!("waiting for detector state or numImages = expectedImages"),
    );
    let expected_images = num_images.saturating_mul(num_triggers.max(1));
    loop {
        let state = s.fetch_str(p.state).await;
        let counter = s.get_int(s.ad.num_images_counter).await;
        if counter >= expected_images {
            break;
        }
        if !matches!(state.as_str(), "configure" | "ready" | "acquire") {
            break;
        }
        if wait_flag(&s.flags.stop, Duration::from_millis(100)).await {
            break;
        }
    }

    let _ = s.rest().disarm();

    s.put_int(p.armed, 0).await;
    s.set_str(s.ad.status_message, "Processing files").await;

    if wait_poll {
        // Let the FileWriter finish writing before telling the poll task to stop.
        s.trace()
            .flow("controlTask", format_args!("waiting for FileWriter"));
        loop {
            if s.fetch_str(p.fw_state).await != "acquire" {
                break;
            }
            rt::sleep(Duration::from_millis(100)).await;
        }
        rt::sleep(Duration::from_millis(500)).await;

        s.poll_stop.store(true, Ordering::Release);
        s.trace()
            .flow("controlTask", format_args!("waiting for pollTask"));
        let _ = ch.poll_done_rx.recv().await;
        let complete = s.poll_complete.load(Ordering::Acquire);
        s.trace().flow(
            "controlTask",
            format_args!("pollTask complete = {}", i32::from(complete)),
        );
    }

    if wait_stream {
        s.trace()
            .flow("controlTask", format_args!("waiting for streamTask"));
        let _ = ch.stream_done_rx.recv().await;
        let complete = s.stream_complete.load(Ordering::Acquire);
        s.trace().flow(
            "controlTask",
            format_args!("streamTask complete = {}", i32::from(complete)),
        );
    }

    if saved_num_images != num_images {
        s.put_int(p.num_images, saved_num_images).await;
    }

    let ad_status = s.get_int(s.ad.status).await;
    if ad_status == ADStatus::Acquire as i32 {
        if trigger_mode == TRIGGER_MODE_CONTINUOUS {
            return true;
        }
        s.set_int(s.ad.status, ADStatus::Idle as i32).await;
        s.set_int(s.ad.acquire, 0).await;
    } else if ad_status == ADStatus::Aborted as i32 {
        s.set_str(s.ad.status_message, "Acquisition aborted").await;
        s.set_int(s.ad.acquire, 0).await;
    }
    false
}

/// ADDriver::setShutter, run on the port: C calls it under the lock, so the
/// EPICS-mode delay holds the port too.
async fn set_shutter(s: &Shared, open: bool) {
    let _ = s
        .handle
        .with_driver(move |d: &mut crate::driver::EigerDriver| d.set_shutter(open))
        .await;
}

/// Wait up to `timeout` for a flag to be set, clearing it if it was.
async fn wait_flag(flag: &AtomicBool, timeout: Duration) -> bool {
    let deadline = Duration::from_millis(10);
    let mut waited = Duration::ZERO;
    loop {
        if flag.swap(false, Ordering::AcqRel) {
            return true;
        }
        if waited >= timeout {
            return false;
        }
        rt::sleep(deadline).await;
        waited += deadline;
    }
}

// ---------------------------------------------------------------------------
// FileWriter pipeline: poll → download → {parse, save} → reap
// ---------------------------------------------------------------------------

/// The most times the poll task re-checks for a missing file once the control
/// task has asked it to stop (C `MAX_RETRIES`).
const MAX_RETRIES: usize = 2;

async fn poll_task(
    s: Arc<Shared>,
    mut rx: rt::CommandReceiver<Acquisition>,
    download_tx: rt::CommandSender<Arc<FileJob>>,
    done_tx: rt::CommandSender<bool>,
) {
    while let Some(acq) = rx.recv().await {
        let total_files = acq.n_data_files + 1;
        let mut files = Vec::with_capacity(total_files);
        for i in 0..total_files {
            let is_master = i == 0;
            let name = if is_master {
                rest::build_master_name(&acq.pattern, acq.sequence_id)
            } else {
                rest::build_data_name(
                    i - 1 + crate::params::DEFAULT_NR_START as usize,
                    &acq.pattern,
                    acq.sequence_id,
                )
            };
            // The master file carries no images, so it is never parsed.
            let parse = !is_master && acq.parse_files;
            let refs = usize::from(acq.save_files) + usize::from(parse);
            files.push(Arc::new(FileJob {
                name,
                save: acq.save_files,
                parse,
                remove: AtomicBool::new(acq.remove_files),
                perms: acq.perms,
                refs: AtomicUsize::new(refs),
                data: parking_lot::Mutex::new(None),
            }));
        }

        s.pending_files.store(0, Ordering::Release);
        s.set_int(s.p.pending_files, 0).await;

        let mut i = 0;
        let mut retries = 0;
        while i < total_files && retries <= MAX_RETRIES {
            let file = &files[i];
            s.trace()
                .flow("pollTask", format_args!("file={}", file.name));
            match s.rest().wait_file(&file.name, Duration::from_secs(1)) {
                Ok(true) => {
                    s.trace()
                        .flow("pollTask", format_args!("file={} exists", file.name));
                    if file.save || file.parse {
                        s.add_pending(1).await;
                        if download_tx.send(file.clone()).await.is_err() {
                            s.add_pending(-1).await;
                        }
                    } else if file.remove.load(Ordering::Acquire) {
                        let _ = s.rest().delete_file(&file.name);
                    }
                    i += 1;
                }
                // The file is not there yet. While the acquisition is running
                // that is normal; once the control task has asked us to stop it
                // means the acquisition was aborted and the file will never come.
                Ok(false) => {
                    if s.poll_stop.load(Ordering::Acquire) {
                        not_found_on_stop(&s, &file.name);
                        retries += 1;
                    }
                }
                Err(_) => {
                    if s.poll_stop.load(Ordering::Acquire) {
                        not_found_on_stop(&s, &file.name);
                        retries += 1;
                    } else {
                        rt::sleep(Duration::from_millis(100)).await;
                    }
                }
            }
        }

        // Every file we did claim must be reaped before the acquisition is over.
        s.trace()
            .flow("pollTask", format_args!("waiting for pending files"));
        while s.pending_files.load(Ordering::Acquire) > 0 {
            rt::sleep(Duration::from_millis(100)).await;
        }
        s.trace()
            .flow("pollTask", format_args!("done waiting for pending files"));

        s.poll_complete.store(i == total_files, Ordering::Release);
        let _ = done_tx.send(i == total_files).await;
    }
}

/// C's `waitFile` fails alike for a missing file and a failed request.
fn not_found_on_stop(s: &Shared, name: &str) {
    s.trace().flow(
        "pollTask",
        format_args!("file={name} not found and pollTask asked to stop"),
    );
}

async fn download_task(
    s: Arc<Shared>,
    mut rx: rt::CommandReceiver<Arc<FileJob>>,
    parse_tx: rt::CommandSender<Arc<FileJob>>,
    save_tx: rt::CommandSender<Arc<FileJob>>,
    reap_tx: rt::CommandSender<Arc<FileJob>>,
) {
    while let Some(file) = rx.recv().await {
        s.trace()
            .flow("downloadTask", format_args!("file={}", file.name));
        match s.rest().get_file(&file.name) {
            Err(_) => {
                s.trace().err(
                    "downloadTask",
                    &format!("underlying getFile({}) failed", file.name),
                );
                // The file was never fetched, so no consumer will ever see it:
                // hand it straight to the reaper, which owns the pending count.
                // Its refcount is still `save + parse`, so drop it to one first.
                file.refs.store(1, Ordering::Release);
                let _ = reap_tx.send(file).await;
            }
            Ok(data) => {
                *file.data.lock() = Some(data);
                if file.parse {
                    let _ = parse_tx.send(file.clone()).await;
                }
                if file.save {
                    let _ = save_tx.send(file.clone()).await;
                }
            }
        }
    }
}

async fn parse_task(
    s: Arc<Shared>,
    mut rx: rt::CommandReceiver<Arc<FileJob>>,
    reap_tx: rt::CommandSender<Arc<FileJob>>,
) {
    while let Some(file) = rx.recv().await {
        s.trace()
            .flow("parseTask", format_args!("file={}", file.name));
        let data = file.data.lock().clone();
        if let Some(data) = data
            && parse_h5(&s, &data).await.is_err()
        {
            s.trace().err(
                "parseTask",
                &format!("underlying parseH5File({}) failed", file.name),
            );
        }
        let _ = reap_tx.send(file).await;
    }
}

/// Publish every image in one FileWriter data file (C `parseH5File`).
async fn parse_h5(s: &Arc<Shared>, data: &[u8]) -> Result<(), h5::Reported> {
    let signed_data = s.get_int(s.p.signed_data).await != 0;
    let trace = s.trace();
    let frames = h5::parse(data, signed_data, &mut |msg| trace.err("parseH5File", msg))?;

    // The thresholds that were enabled for this acquisition, in order — the
    // Nth frame of a 4-D dataset belongs to the Nth *active* threshold.
    let active = active_thresholds(s).await;
    // Once per file, counted on locally (eigerDetector.cpp:1841-1842, :1903-1904).
    let mut counters = Counters::read(s).await;

    // C `break`s out of the threshold loop on a refused alloc
    // (eigerDetector.cpp:1849), dropping the rest of that image's thresholds.
    let mut skip_image = false;
    for frame in frames {
        if frame.threshold == 0 {
            skip_image = false;
        }
        if skip_image {
            continue;
        }
        let (number, energy) = active
            .get(frame.threshold)
            .copied()
            .unwrap_or((frame.threshold as i32 + 1, 0.0));
        let mut attrs = NDAttributeList::new();
        attrs.add(threshold_number_attr(number));
        attrs.add(threshold_energy_attr(energy));

        // Per frame, after its updateTimeStamps (eigerDetector.cpp:1880, :1892).
        let timestamp = port_stamp(&s.handle).await;
        let array_callbacks = s.get_int(s.ad.base.array_callbacks).await != 0;
        let image = Image {
            dims: frame.dims,
            data: frame.data,
            codec: None,
            attributes: attrs,
            threshold_index: frame.threshold,
            timestamp: Some(timestamp),
        };
        let Ok(array) = prepare(s, image, &counters).await else {
            trace.err("parseH5File", "couldn't allocate NDArray");
            skip_image = true;
            continue;
        };
        if array_callbacks {
            trace.flow_single_colon("parseH5File", "calling NDArray callback");
        }
        deliver(s, array, frame.threshold, array_callbacks, &mut counters).await;
    }
    Ok(())
}

/// `(threshold number, energy)` of every enabled threshold, in threshold order.
async fn active_thresholds(s: &Arc<Shared>) -> Vec<(i32, f64)> {
    let mut out = Vec::new();
    for t in crate::params::thresholds(&s.p) {
        let enabled = match t.enable {
            // Eiger1 has a single, always-active threshold.
            None => true,
            Some(e) => s.get_int(e).await != 0,
        };
        if enabled {
            out.push((t.number, s.get_f64(t.energy).await));
        }
    }
    out
}

fn threshold_number_attr(number: i32) -> NDAttribute {
    NDAttribute::new_static(
        "ThresholdNumber",
        "Threshold number",
        NDAttrSource::Driver,
        NDAttrValue::Int32(number),
    )
}

fn threshold_energy_attr(energy: f64) -> NDAttribute {
    NDAttribute::new_static(
        "ThresholdEnergy",
        "Threshold energy (eV)",
        NDAttrSource::Driver,
        NDAttrValue::Float64(energy),
    )
}

async fn save_task(
    s: Arc<Shared>,
    mut rx: rt::CommandReceiver<Arc<FileJob>>,
    reap_tx: rt::CommandSender<Arc<FileJob>>,
) {
    while let Some(file) = rx.recv().await {
        save_flow(&s, &file.name);
        let data = file.data.lock().clone();
        let Some(data) = data else {
            let _ = reap_tx.send(file).await;
            continue;
        };

        // C sets NDFileName/NDFileTemplate and calls createFileName, whose
        // template is fixed to "%s%s" — i.e. FilePath followed by the detector's
        // own file name.
        let path = s.get_str(s.ad.base.file_path).await;
        let full = format!("{path}{}", file.name);
        s.apply(vec![
            ParamUpdate::Octet(s.ad.base.file_name, file.name.clone()),
            ParamUpdate::Octet(s.ad.base.file_template, "%s%s".into()),
            ParamUpdate::Octet(s.ad.base.full_file_name, full.clone()),
        ])
        .await;

        if !write_file(&s, &file.name, &full, &data, file.perms) {
            // The local copy failed, so the detector's copy is the only one left:
            // do not let the reaper delete it.
            file.remove.store(false, Ordering::Release);
        }
        let _ = reap_tx.send(file).await;
    }
}

/// saveTask's first line. Without `FILE_OWNER` (see [`write_file`]) the owner
/// is always the IOC's own uid and gid, C's `mFsUid`/`mFsGid` defaults.
fn save_flow(s: &Shared, name: &str) {
    #[cfg(unix)]
    {
        // SAFETY: getuid and getgid cannot fail and touch no memory.
        let (uid, gid) = unsafe { (libc::getuid(), libc::getgid()) };
        s.trace()
            .flow("saveTask", format_args!("file={name} uid={uid} gid={gid}"));
    }
    #[cfg(not(unix))]
    s.trace().flow("saveTask", format_args!("file={name}"));
}

/// Write one downloaded file to `FilePath` with `FILE_PERMISSIONS`.
///
/// SCOPED GAP: C additionally honours `FILE_OWNER` / `FILE_OWNER_GROUP` by
/// resolving them with `getpwnam`/`getgrnam` and wrapping the write in
/// `setfsuid`/`setfsgid` (eigerDetector.cpp:719, 1266-1267). That is a Linux-only,
/// libc-dependent path with no portable Rust equivalent; it is not ported. The
/// two PVs still exist and hold their strings, but they do not change the owner
/// of the written file, and C's "setting FS UID/GID" FLOW lines never print.
///
/// Returns whether the whole file was written. Failures print C's
/// `saveTask` lines and the `perror` that follows each; a failed `fchmod`
/// is reported and the write goes on, as in C.
fn write_file(s: &Shared, name: &str, path: &str, data: &[u8], perms: u32) -> bool {
    use std::io::Write;

    let perror =
        |call: &str, e: &std::io::Error| epics_rs::base::stderr_println!("{call}: {}", strerror(e));
    let mut opts = std::fs::OpenOptions::new();
    // C DEFECT, not copied: C opens without O_TRUNC (eigerDetector.cpp:1293),
    // so a shorter file written over an existing one keeps the old tail.
    opts.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        opts.mode(perms);
    }
    let mut f = match opts.open(path) {
        Ok(f) => f,
        Err(e) => {
            s.trace().err(
                "saveTask",
                &format!("[file={name}] unable to open file to be written\n[{path}]"),
            );
            perror("open", &e);
            return false;
        }
    };
    // An existing file keeps its old mode through `open`, so set it explicitly
    // (C calls fchmod for the same reason). POSIX mode bits have no portable
    // Windows equivalent, so on non-unix the file is left at the OS default.
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        if let Err(e) = f.set_permissions(std::fs::Permissions::from_mode(perms)) {
            s.trace().err(
                "saveTask",
                &format!("[file={name}] failed to set permissions {perms:o}"),
            );
            perror("fchmod", &e);
        }
    }
    #[cfg(not(unix))]
    {
        let _ = perms;
    }
    let mut written = 0;
    while written < data.len() {
        match f.write(&data[written..]) {
            Ok(n) if n > 0 => written += n,
            result => {
                s.trace().err(
                    "saveTask",
                    &format!("[file={name}] failed to write to local file ({written} written)"),
                );
                let e = result
                    .err()
                    .unwrap_or_else(|| std::io::ErrorKind::WriteZero.into());
                perror("write", &e);
                return false;
            }
        }
    }
    true
}

/// `perror`'s text: the OS message without Rust's ` (os error N)` suffix.
fn strerror(e: &std::io::Error) -> String {
    let text = e.to_string();
    match text.find(" (os error ") {
        Some(i) => text[..i].to_string(),
        None => text,
    }
}

/// Drop one reference to a file; the last one deletes it from the detector
/// (C `reapTask`, eigerDetector.cpp:1331).
async fn reap(s: &Arc<Shared>, file: Arc<FileJob>) {
    let refs = file.refs.fetch_sub(1, Ordering::AcqRel);
    s.trace().flow(
        "reapTask",
        format_args!("file={} refCount={refs}", file.name),
    );
    if refs != 1 {
        return;
    }
    if file.remove.load(Ordering::Acquire) {
        let _ = s.rest().delete_file(&file.name);
    }
    s.fetch(s.p.fw_free).await;
    if file.data.lock().take().is_some() {
        s.trace()
            .flow("reapTask", format_args!("file={} reaped", file.name));
    }
    s.add_pending(-1).await;
}

async fn reap_task(s: Arc<Shared>, mut rx: rt::CommandReceiver<Arc<FileJob>>) {
    while let Some(file) = rx.recv().await {
        reap(&s, file).await;
    }
}

// ---------------------------------------------------------------------------
// Monitor
// ---------------------------------------------------------------------------

async fn monitor_task(s: Arc<Shared>) {
    let mut unique_id = 1;
    loop {
        let enabled = s.get_int(s.p.monitor_enable).await != 0;
        if enabled {
            let timeout = s.get_int(s.p.monitor_timeout).await.max(0) as u32;
            // No new image within the long-poll window is the normal case
            // (C ignores the failure, eigerDetector.cpp:1384).
            if let Ok(buf) = s.rest().get_monitor_image(timeout) {
                match tiff::decode(&buf) {
                    Ok(image) => {
                        let Ok(array) = build_array(
                            &s.pool,
                            image.dims,
                            image.data,
                            None,
                            NDAttributeList::new(),
                            unique_id,
                            port_stamp(&s.handle).await,
                        ) else {
                            s.trace().err("parseTiffFile", "couldn't allocate NDArray");
                            s.trace().err("monitorTask", "couldn't parse file");
                            rt::sleep(MONITOR_PERIOD).await;
                            continue;
                        };
                        unique_id += 1;
                        ArrayPublisher::new(s.outputs.monitor.clone())
                            .publish(array)
                            .await;
                    }
                    Err(e) => {
                        s.trace().err("parseTiffFile", &e.0);
                        s.trace().err("monitorTask", "couldn't parse file");
                    }
                }
            }
        }
        rt::sleep(MONITOR_PERIOD).await;
    }
}

// ---------------------------------------------------------------------------
// Stream
// ---------------------------------------------------------------------------

async fn stream_task(
    s: Arc<Shared>,
    mut start_rx: rt::CommandReceiver<()>,
    mut apis_rx: rt::CommandReceiver<()>,
    done_tx: rt::CommandSender<bool>,
) {
    let mut api: Option<StreamApi> = None;
    let mut api2: Option<Stream2Api> = None;

    loop {
        let start = tokio::select! {
            r = start_rx.recv() => r.is_some(),
            r = apis_rx.recv() => {
                if r.is_none() {
                    return;
                }
                false
            }
        };

        // Hold a socket for each API C has. zmq.rs connects eagerly where
        // libzmq reconnects in the background, so a failed connect is retried
        // here on the next signal.
        let apis = s.flags.stream_apis.load(Ordering::Acquire);
        if apis & STREAM_API == 0 {
            api = None;
        } else if api.is_none() {
            api = StreamApi::connect(&s.hostname).await.ok();
        }
        if apis & STREAM2_API == 0 {
            api2 = None;
        } else if api2.is_none() {
            api2 = Stream2Api::connect(&s.hostname).await.ok();
        }
        if !start {
            continue;
        }

        let version = s.get_int(s.p.stream_version).await;
        let complete = match (
            version == STREAM_VERSION_STREAM,
            api.as_mut(),
            api2.as_mut(),
        ) {
            (true, Some(stream), _) => stream_series(&s, stream).await,
            (false, _, Some(stream)) => stream2_series(&s, stream).await,
            _ => {
                s.trace()
                    .err("streamTask", "mStreamAPI is null, Stream API not enabled?");
                let _ = done_tx.send(false).await;
                continue;
            }
        };
        s.stream_complete.store(complete, Ordering::Release);

        s.fetch(s.p.stream_dropped).await;
        let _ = done_tx.send(complete).await;
    }
}

fn stream_timeout(s: &Shared) {
    s.trace()
        .flow("streamTask", format_args!("got stream timeout"));
}

fn end_frame(s: &Shared) {
    s.trace().flow("streamTask", format_args!("got end frame"));
}

/// Receive one whole series. Returns whether it ended with a `dseries_end`
/// (C `mStreamComplete`).
async fn stream_series(s: &Arc<Shared>, stream: &mut StreamApi) -> bool {
    let timeout = Duration::from_secs(1);

    // Wait out the series header. Only the header or an error ends the wait;
    // C leaves no Stop exit, which would strand this series in the socket
    // for the next acquisition (eigerDetector.cpp:1441-1455).
    loop {
        match stream
            .recv(timeout)
            .await
            .and_then(|m| m.map(|m| m.header()).transpose())
        {
            Ok(Some(_)) => break,
            Ok(None) => stream_timeout(s),
            Err(crate::stream::StreamError::WrongHtype(h)) => {
                epics_rs::base::stderr_println!(
                    "StreamApi::getHeader: wrong header type, htype={h}"
                );
                s.trace().err("streamTask", "got stray packet, ignoring");
            }
            Err(_) => {
                s.trace().err("streamTask", "failed to get header packet");
                return false;
            }
        }
    }

    loop {
        let received = match stream.recv(timeout).await {
            Ok(Some(m)) => m,
            Ok(None) => {
                stream_timeout(s);
                continue;
            }
            Err(_) => {
                s.trace().err("streamTask", "failed to get frame packet");
                return false;
            }
        };
        // C reads these per frame, once waitFrame has it (eigerDetector.cpp:1497-1517).
        let decompress = s.get_int(s.p.stream_decompress).await != 0;
        let signed_data = s.get_int(s.p.signed_data).await != 0;
        let array_callbacks = s.get_int(s.ad.base.array_callbacks).await != 0;
        let mut counters = Counters::read(s).await;
        let msg = match received.decode(decompress) {
            Ok(m) => m,
            Err(crate::stream::StreamError::WrongHtype(_)) => continue,
            // UPSTREAM DEFECT (streamApi.cpp:1503-1507 in the caller): C assigns
            // `err = mStreamAPI->getFrame(&pArray, ...)` and then dereferences
            // `pArray` without ever testing `err`, so a failed decode reads an
            // uninitialised pointer. Here a failed frame ends the series.
            Err(_) => {
                s.trace().err("streamTask", "failed to get frame packet");
                return false;
            }
        };

        match msg {
            StreamMessage::End => {
                end_frame(s);
                return true;
            }
            StreamMessage::Header(_) => {}
            StreamMessage::Image(frame) => {
                // Stream v1 carries one image per message and no threshold axis
                // (C `numThresholds` stays 1 on this path, eigerDetector.cpp:1422),
                // so every frame goes to the main output and to threshold 1 —
                // C's addresses 0 and 1. It also carries no ThresholdNumber /
                // ThresholdEnergy attributes; only the HDF5 and stream2 paths add
                // those.
                let number = frame.frame;
                let image = Image {
                    dims: frame.dims,
                    data: frame.data,
                    codec: frame.codec,
                    attributes: NDAttributeList::new(),
                    threshold_index: 0,
                    timestamp: None,
                };
                // A refused alloc is getFrame's STREAM_ERROR (streamApi.cpp:370),
                // which ends the series like any other failed frame above.
                if publish(
                    s,
                    image.signed_if(signed_data),
                    array_callbacks,
                    &mut counters,
                )
                .await
                .is_err()
                {
                    epics_rs::base::stderr_println!(
                        "StreamApi::getFrame: failed to allocate NDArray for frame {number}"
                    );
                    s.trace().err("streamTask", "failed to get frame packet");
                    return false;
                }
            }
        }
    }
}

/// [`stream_series`] for StreamVersion=Stream2 (the `mStream2API` branches of
/// C `streamTask`).
async fn stream2_series(s: &Arc<Shared>, stream: &mut Stream2Api) -> bool {
    let timeout = Duration::from_secs(1);
    let ts_source = s.get_int(s.p.stream_as_ts_source).await != 0;

    // C reads the status as an int in streamApi.h's code space
    // (`Stream2Error::code`): 1 timeout, 2 stray packet, 3 error. No Stop
    // exit, as in stream_series.
    loop {
        match stream.get_header(timeout).await {
            Ok(()) => break,
            Err(e) => match e.code() {
                1 => stream_timeout(s),
                2 => s.trace().err("streamTask", "got stray packet, ignoring"),
                3 => {
                    s.trace().err("streamTask", "failed to get header packet");
                    return false;
                }
                _ => {
                    s.trace()
                        .err("streamTask", "unknown err from mStreamAPI->getHeader()");
                    return false;
                }
            },
        }
    }

    loop {
        let num_thresholds = match stream.wait_frame(timeout).await {
            Ok(Frame::End) => {
                end_frame(s);
                return true;
            }
            Ok(Frame::Image(n)) => n,
            Err(e) => match e.code() {
                1 => {
                    stream_timeout(s);
                    continue;
                }
                3 => {
                    s.trace().err("streamTask", "failed to get frame packet");
                    return false;
                }
                _ => {
                    s.trace()
                        .err("streamTask", "unknown err from mStreamAPI->waitFrame()");
                    return false;
                }
            },
        };

        for thresh in 0..num_thresholds {
            let decompress = s.get_int(s.p.stream_decompress).await != 0;
            // UPSTREAM DEFECT (eigerDetector.cpp:1505): C does not test what
            // getFrame returns and publishes `pArray` regardless, unset when the
            // data type was unknown. Here that frame is skipped.
            let Ok(frame) = stream.get_frame(thresh, decompress, ts_source) else {
                continue;
            };
            let signed_data = s.get_int(s.p.signed_data).await != 0;
            let array_callbacks = s.get_int(s.ad.base.array_callbacks).await != 0;
            let mut counters = Counters::read(s).await;
            let mut attributes = NDAttributeList::new();
            if let Some((number, energy)) = frame.threshold {
                attributes.add(threshold_number_attr(number));
                attributes.add(threshold_energy_attr(energy));
            }
            let image = Image {
                dims: frame.dims,
                data: frame.data,
                codec: frame.codec,
                attributes,
                threshold_index: thresh,
                timestamp: frame.timestamp,
            };
            // A refused alloc is getFrame's STREAM_ERROR (stream2Api.cpp:290),
            // skipped like the other failed frames above.
            if publish(
                s,
                image.signed_if(signed_data),
                array_callbacks,
                &mut counters,
            )
            .await
            .is_err()
            {
                epics_rs::base::stderr_println!(
                    "Stream2Api::getFrame: failed to allocate NDArray for frame"
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

/// C `updateTimeStamps` (asynNDArrayDriver.cpp:832-836): the port's stamp,
/// moved now, which the frame's callbacks carry too.
async fn port_stamp(handle: &PortHandle) -> EpicsTimestamp {
    handle
        .update_time_stamp()
        .await
        .unwrap_or_else(|_| std::time::SystemTime::now())
        .into()
}

fn build_array(
    pool: &DriverPool,
    dims: [usize; 2],
    data: NDDataBuffer,
    codec: Option<epics_rs::ad_core::codec::Codec>,
    attributes: NDAttributeList,
    unique_id: i32,
    timestamp: EpicsTimestamp,
) -> ADResult<DriverArray> {
    // A compressed frame holds its payload bytes, which its dims do not size.
    let mut array = pool.alloc_sized(
        vec![NDDimension::new(dims[0]), NDDimension::new(dims[1])],
        data.data_type(),
        data.total_bytes(),
    )?;
    let mut frame = array.fill();
    frame
        .data_mut()
        .as_u8_slice_mut()
        .copy_from_slice(data.as_u8_slice());
    frame.set_unique_id(unique_id);
    // C sets `timeStamp = ts.secPastEpoch + ts.nsec/1.e9` (stream2Api.cpp:337),
    // as updateTimeStamps does.
    frame.set_timestamp(timestamp);
    frame.set_time_stamp(f64::from(timestamp.sec) + f64::from(timestamp.nsec) / 1.0e9);
    *frame.attributes_mut() = attributes;
    frame.set_codec(codec);
    Ok(array)
}

/// One image on its way out: what [`publish`] needs to build the NDArray and to
/// pick the threshold output it also goes to.
struct Image {
    dims: [usize; 2],
    data: NDDataBuffer,
    codec: Option<epics_rs::ad_core::codec::Codec>,
    attributes: NDAttributeList,
    /// 0-based threshold index; picks `outputs.thresholds[i]`, which is C's
    /// asyn address `thresh + 1`.
    threshold_index: usize,
    /// A stamp the caller already took: the stream2 frame time when
    /// STREAM_AS_TIMESTAMP_SOURCE is set, or parseH5File's own. `None` stamps
    /// the array now.
    timestamp: Option<EpicsTimestamp>,
}

impl Image {
    /// Relabel unsigned pixels as signed (C's `SignedData`,
    /// eigerDetector.cpp:1519) when `signed` is set.
    ///
    /// The detector's counts are unsigned, but bad pixels and gaps are very
    /// large positive numbers that wreck autoscaling; re-tagging the same bits
    /// as signed turns them into small negatives at the cost of half the count
    /// range. C changes `pArray->dataType`, which on a still-compressed frame
    /// is the type the codec plugin decompresses to, so a codec's
    /// `original_data_type` is relabelled instead of its bytes.
    fn signed_if(mut self, signed: bool) -> Self {
        if !signed {
            return self;
        }
        if let Some(codec) = &mut self.codec {
            codec.original_data_type = match codec.original_data_type {
                NDDataType::UInt8 => NDDataType::Int8,
                NDDataType::UInt16 => NDDataType::Int16,
                NDDataType::UInt32 => NDDataType::Int32,
                other => other,
            };
            return self;
        }
        self.data = match self.data.view() {
            NDDataView::U8(v) => NDDataBuffer::from_vec(v.iter().map(|&x| x as i8).collect()),
            NDDataView::U16(v) => NDDataBuffer::from_vec(v.iter().map(|&x| x as i16).collect()),
            NDDataView::U32(v) => NDDataBuffer::from_vec(v.iter().map(|&x| x as i32).collect()),
            _ => return self,
        };
        self
    }
}

/// C's `imageCounter` / `numImagesCounter` locals: each path reads them where
/// C does and [`publish`] counts them on.
struct Counters {
    image: i32,
    num_images: i32,
}

impl Counters {
    async fn read(s: &Shared) -> Self {
        Self {
            image: s.get_int(s.ad.base.array_counter).await,
            num_images: s.get_int(s.ad.num_images_counter).await,
        }
    }
}

/// `Err` when the pool refuses the frame (C's `alloc` returning NULL); the
/// caller prints C's line for its path, and the counters stay as they were.
async fn publish(
    s: &Arc<Shared>,
    image: Image,
    array_callbacks: bool,
    counters: &mut Counters,
) -> ADResult<()> {
    let threshold_index = image.threshold_index;
    let array = prepare(s, image, counters).await?;
    deliver(s, array, threshold_index, array_callbacks, counters).await;
    Ok(())
}

/// Stamp and allocate one image; [`publish`]'s first half.
async fn prepare(s: &Arc<Shared>, image: Image, counters: &Counters) -> ADResult<PublishedArray> {
    let Image {
        dims,
        data,
        codec,
        attributes,
        threshold_index: _,
        timestamp,
    } = image;
    // C stamps the counter before its `++imageCounter` (eigerDetector.cpp:1537,
    // :1878), so the first array of an IOC's life is uniqueId 0.
    let unique_id = counters.image;
    // C calls updateTimeStamps only when stream2 left the ts unset
    // (eigerDetector.cpp:1539-1541).
    let timestamp = match timestamp {
        Some(ts) => ts,
        None => port_stamp(&s.handle).await,
    };
    Ok(build_array(&s.pool, dims, data, codec, attributes, unique_id, timestamp)?.share())
}

/// Publish one image on the main output and on its threshold's output
/// (C `doCallbacksGenericPointer(pArray, NDArrayData, 0)` plus the
/// `thresh + 1` address), then count it.
async fn deliver(
    s: &Arc<Shared>,
    array: PublishedArray,
    threshold_index: usize,
    array_callbacks: bool,
    counters: &mut Counters,
) {
    if array_callbacks {
        ArrayPublisher::new(s.outputs.main.clone())
            .publish(array.clone())
            .await;
        if let Some(out) = s.outputs.thresholds.get(threshold_index) {
            ArrayPublisher::new(out.clone())
                .publish(array.clone())
                .await;
        }
    }

    counters.image += 1;
    counters.num_images += 1;
    // C stores no array size, type, codec or time-stamp param per frame
    // (eigerDetector.cpp:1553-1554, :1902-1903).
    s.apply(vec![
        ParamUpdate::Int32(s.ad.base.array_counter, counters.image),
        ParamUpdate::Int32(s.ad.num_images_counter, counters.num_images),
    ])
    .await;
}

/// Build the shared state the tasks run on, plus the driver's signal handles.
#[allow(clippy::too_many_arguments)]
pub fn shared(
    ops: Arc<ParamOps>,
    handle: PortHandle,
    p: EigerParams,
    ad: ADDriverParams,
    model: Model,
    api: ApiVersion,
    hostname: String,
    outputs: Outputs,
    pool: DriverPool,
    signals: &Signals,
) -> Arc<Shared> {
    Arc::new(Shared {
        ops,
        handle,
        p,
        ad,
        model,
        api,
        hostname,
        outputs,
        pool,
        pending_files: AtomicI32::new(0),
        poll_stop: AtomicBool::new(false),
        poll_complete: AtomicBool::new(false),
        stream_complete: AtomicBool::new(false),
        flags: signals.flags.clone(),
    })
}

/// Create the driver's signal handles and the matching receiver ends.
pub fn signals() -> (Signals, Receivers) {
    let (ctl_tx, ctl) = rt::command_channel::<CtlCommand>(4);
    let (init_tx, init) = rt::command_channel::<()>(1);
    let (restart_tx, restart) = rt::command_channel::<()>(1);
    let (stream_apis_tx, stream_apis) = rt::command_channel::<()>(1);
    (
        Signals {
            ctl_tx,
            init_tx,
            restart_tx,
            stream_apis_tx,
            flags: Arc::new(Flags::default()),
        },
        Receivers {
            ctl,
            init,
            restart,
            stream_apis,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stream_apis_follow_c_create_and_delete() {
        let (signals, mut rx) = signals();
        let apis = || signals.flags.stream_apis.load(Ordering::Acquire);

        signals.stream_apis(Some(STREAM_VERSION_STREAM));
        assert_eq!(apis(), STREAM_API);
        // Switching version under DataSource=stream keeps the other object.
        signals.stream_apis(Some(STREAM_VERSION_STREAM + 1));
        assert_eq!(apis(), STREAM_API | STREAM2_API);
        // Any other DataSource deletes both.
        signals.stream_apis(None);
        assert_eq!(apis(), 0);

        // Signals coalesce: the task rereads the flags on one wake.
        assert!(rx.stream_apis.try_recv().is_ok());
        assert!(rx.stream_apis.try_recv().is_err());
    }

    fn image(data: NDDataBuffer, codec: Option<epics_rs::ad_core::codec::Codec>) -> Image {
        Image {
            dims: [2, 1],
            data,
            codec,
            attributes: NDAttributeList::new(),
            threshold_index: 0,
            timestamp: None,
        }
    }

    #[test]
    fn signed_data_relabels_pixels_or_the_codec_type() {
        let im = image(NDDataBuffer::from_vec::<u16>(vec![1, 0xffff]), None).signed_if(true);
        let NDDataView::I16(v) = im.data.view() else {
            panic!("expected I16");
        };
        assert_eq!(v, [1, -1]);

        let codec = epics_rs::ad_core::codec::Codec {
            name: epics_rs::ad_core::codec::CodecName::BSLZ4,
            compressed_size: 3,
            level: 0,
            shuffle: 0,
            compressor: 0,
            original_data_type: NDDataType::UInt32,
        };
        let im = image(NDDataBuffer::from_vec::<u8>(vec![1, 2, 3]), Some(codec)).signed_if(true);
        assert_eq!(im.codec.unwrap().original_data_type, NDDataType::Int32);
        let NDDataView::U8(v) = im.data.view() else {
            panic!("compressed bytes stay bytes");
        };
        assert_eq!(v, [1, 2, 3]);

        let im = image(NDDataBuffer::from_vec::<u16>(vec![1]), None).signed_if(false);
        assert!(matches!(im.data.view(), NDDataView::U16(_)));
    }

    /// A detector that answers every GET as a two-valued enum and every PUT
    /// with an empty change list.
    fn agreeable_detector() -> u16 {
        use std::io::{BufRead, BufReader, Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("listener");
        let port = listener.local_addr().unwrap().port();
        std::thread::spawn(move || {
            for stream in listener.incoming() {
                let Ok(mut stream) = stream else { break };
                let mut reader = BufReader::new(stream.try_clone().expect("clone"));
                let mut request = String::new();
                if reader.read_line(&mut request).unwrap_or(0) == 0 {
                    continue;
                }
                let mut length = 0usize;
                loop {
                    let mut header = String::new();
                    if reader.read_line(&mut header).unwrap_or(0) <= 2 {
                        break;
                    }
                    if let Some(v) = header.to_ascii_lowercase().strip_prefix("content-length:") {
                        length = v.trim().parse().unwrap_or(0);
                    }
                }
                let _ = reader.read_exact(&mut vec![0u8; length]);
                let body = if request.contains("/command/arm ") {
                    r#"{"sequence id":1}"#
                } else if request.starts_with("PUT") {
                    "[]"
                } else {
                    r#"{"value":"disabled","value_type":"string","access_mode":"rw",
                       "allowed_values":["disabled","enabled"]}"#
                };
                let _ = stream.write_all(
                    format!(
                        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\
                         Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                        body.len()
                    )
                    .as_bytes(),
                );
            }
        });
        port
    }

    /// eigerDetector.cpp:587-588 hands every int32 below mFirstParam to
    /// ADDriver::writeInt32.
    #[test]
    fn a_base_class_write_reaches_addriver() {
        use epics_rs::ad_core::driver::ShutterMode;
        use epics_rs::asyn::port::PortDriver;
        use epics_rs::asyn::user::AsynUser;

        let (signals, _receivers) = signals();
        let cfg = crate::driver::EigerConfig {
            port_name: "EIGER_BASE_WRITE".to_string(),
            api: ApiVersion::V1_6_0,
            model: Model::Eiger1,
            max_memory: 0,
        };
        let rest = RestApi::new("127.0.0.1", agreeable_detector());
        let mut det = crate::driver::EigerDriver::new(cfg, rest, signals).expect("driver");
        let ad = det.ad.params;
        det.ad
            .port_base
            .set_int32_param(ad.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        det.write_int32(&mut AsynUser::new(ad.shutter_control), 1)
            .unwrap();
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(ad.shutter_control_epics, 0)
                .unwrap(),
            1,
            "ADShutterControl did not reach ADDriver::setShutter"
        );
    }

    /// initParams sets NDDataType to NDUInt32 with NDArraySize and
    /// ADImageMode (eigerDetector.cpp:1655-1657).
    #[test]
    fn the_constructor_sets_data_type_uint32() {
        let (signals, _receivers) = signals();
        let cfg = crate::driver::EigerConfig {
            port_name: "EIGER_DATA_TYPE".to_string(),
            api: ApiVersion::V1_6_0,
            model: Model::Eiger1,
            max_memory: 0,
        };
        let rest = RestApi::new("127.0.0.1", agreeable_detector());
        let det = crate::driver::EigerDriver::new(cfg, rest, signals).expect("driver");
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(det.ad.params.base.data_type, 0)
                .unwrap(),
            NDDataType::UInt32 as u8 as i32
        );
    }

    /// eigerDetector.cpp:667-668 hands every float64 below mFirstParam to
    /// ADDriver::writeFloat64, which is asynPortDriver's: its IO_DRIVER line
    /// precedes the driver's.
    #[test]
    fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
        use epics_rs::asyn::port::PortDriver;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        use epics_rs::asyn::user::{AsynUser, UserTrace};

        let port = "EIGER_BASE_F64";
        let (signals, _receivers) = signals();
        let cfg = crate::driver::EigerConfig {
            port_name: port.to_string(),
            api: ApiVersion::V1_6_0,
            model: Model::Eiger1,
            max_memory: 0,
        };
        let rest = RestApi::new("127.0.0.1", agreeable_detector());
        let mut det = crate::driver::EigerDriver::new(cfg, rest, signals).expect("driver");
        let manager = epics_rs::asyn::services::PortServices::global()
            .trace()
            .clone();
        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        let delay = det.ad.params.shutter_open_delay;
        let mut user = AsynUser::new(delay).with_addr(0);
        user.trace = Some(UserTrace {
            manager,
            port: port.into(),
        });
        det.write_float64(&mut user, 2.0).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let base = format!(
            "asynPortDriver:writeFloat64: function={delay}, name=SHUTTER_OPEN_DELAY, value=2.000000"
        );
        let own = format!("eigerDetector:writeFloat64: function={delay}, value=2.000000");
        let at = |line: &str| {
            text.find(line)
                .unwrap_or_else(|| panic!("{line}: {text:?}"))
        };
        assert!(at(&base) < at(&own));
    }

    #[tokio::test]
    async fn the_unique_id_is_the_array_counter_before_its_increment() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let name = "EIGER_UNIQUE_ID";
        let (signals, _receivers) = signals();
        let cfg = crate::driver::EigerConfig {
            port_name: name.to_string(),
            api: ApiVersion::V1_6_0,
            model: Model::Eiger1,
            max_memory: 0,
        };
        let rest = RestApi::new("127.0.0.1", agreeable_detector());
        let det = crate::driver::EigerDriver::new(cfg, rest, signals.clone()).expect("driver");
        let (p, ad, ops) = (det.p, det.ad.params, det.ops.clone());
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), det, RuntimeConfig::default())
                .expect("runtime");
        let outputs = Outputs::new();
        let (tx, mut rx) = ndarray_channel("UNIQUE_ID_SINK", 4);
        outputs.main.lock().add(tx);
        let s = shared(
            ops,
            runtime.port_handle().clone(),
            p,
            ad,
            Model::Eiger1,
            ApiVersion::V1_6_0,
            "127.0.0.1".to_string(),
            outputs,
            DriverPool::standalone(0),
            &signals,
        );

        for _ in 0..2 {
            let mut counters = Counters::read(&s).await;
            publish(
                &s,
                image(NDDataBuffer::from_vec::<u16>(vec![1, 2]), None),
                true,
                &mut counters,
            )
            .await
            .unwrap();
        }
        let mut ids = Vec::new();
        for _ in 0..2 {
            ids.push(rx.recv().await.expect("array").unique_id);
        }
        assert_eq!(ids, [0, 1]);
        assert_eq!(s.get_int(s.ad.base.array_counter).await, 2);
        std::mem::forget(runtime);
    }

    /// C streamTask and the file reader store only NDArrayCounter and
    /// ADNumImagesCounter per frame (eigerDetector.cpp:1536-1556, :1877-1904).
    #[tokio::test]
    async fn a_frame_leaves_the_array_params_alone() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let name = "EIGER_FRAME_PARAMS";
        let (signals, _receivers) = signals();
        let cfg = crate::driver::EigerConfig {
            port_name: name.to_string(),
            api: ApiVersion::V1_6_0,
            model: Model::Eiger1,
            max_memory: 0,
        };
        let rest = RestApi::new("127.0.0.1", agreeable_detector());
        let det = crate::driver::EigerDriver::new(cfg, rest, signals.clone()).expect("driver");
        let (p, ad, ops) = (det.p, det.ad.params, det.ops.clone());
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), det, RuntimeConfig::default())
                .expect("runtime");
        let outputs = Outputs::new();
        let (tx, mut rx) = ndarray_channel("FRAME_PARAMS_SINK", 4);
        outputs.main.lock().add(tx);
        let s = shared(
            ops,
            runtime.port_handle().clone(),
            p,
            ad,
            Model::Eiger1,
            ApiVersion::V1_6_0,
            "127.0.0.1".to_string(),
            outputs,
            DriverPool::standalone(0),
            &signals,
        );

        let mut counters = Counters::read(&s).await;
        publish(
            &s,
            image(NDDataBuffer::from_vec::<u16>(vec![1, 2]), None),
            true,
            &mut counters,
        )
        .await
        .unwrap();
        rx.recv().await.expect("array");
        assert_eq!(s.get_int(s.ad.base.array_counter).await, 1);
        assert_eq!(s.get_int(s.ad.base.array_size).await, 0);
        assert_eq!(s.get_int(s.ad.base.n_dimensions).await, 0);
        assert_eq!(
            s.get_int(s.ad.base.data_type).await,
            NDDataType::UInt32 as u8 as i32
        );
        assert_eq!(s.get_int(s.ad.base.epics_ts_sec).await, 0);
        assert_eq!(s.get_f64(s.ad.base.timestamp_rbv).await, 0.0);
        std::mem::forget(runtime);
    }

    /// C streamTask reads StreamDecompress, SignedData and ArrayCallbacks for
    /// every frame, once waitFrame has it (eigerDetector.cpp:1497-1517).
    #[tokio::test]
    async fn stream_v1_reads_its_frame_params_per_frame() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use zeromq::{Socket, SocketSend};

        let name = "EIGER_STREAM_PER_FRAME";
        let (signals, _receivers) = signals();
        let cfg = crate::driver::EigerConfig {
            port_name: name.to_string(),
            api: ApiVersion::V1_6_0,
            model: Model::Eiger1,
            max_memory: 0,
        };
        let rest = RestApi::new("127.0.0.1", agreeable_detector());
        let det = crate::driver::EigerDriver::new(cfg, rest, signals.clone()).expect("driver");
        let (p, ad, ops) = (det.p, det.ad.params, det.ops.clone());
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), det, RuntimeConfig::default())
                .expect("runtime");
        let outputs = Outputs::new();
        let (tx, mut rx) = ndarray_channel("STREAM_PER_FRAME_SINK", 8);
        // Frame 3 passes through compressed.
        rx.admission().set_compression_aware(true);
        outputs.main.lock().add(tx);
        let s = shared(
            ops,
            runtime.port_handle().clone(),
            p,
            ad,
            Model::Eiger1,
            ApiVersion::V1_6_0,
            "127.0.0.1".to_string(),
            outputs,
            DriverPool::standalone(0),
            &signals,
        );

        let mut push = zeromq::PushSocket::new();
        let endpoint = push.bind("tcp://127.0.0.1:0").await.expect("bind");
        let mut stream = StreamApi::connect_to(&endpoint.to_string())
            .await
            .expect("connect");
        let task = {
            let s = Arc::clone(&s);
            tokio::spawn(async move { stream_series(&s, &mut stream).await })
        };
        let message = |parts: Vec<Vec<u8>>| {
            zeromq::ZmqMessage::try_from(
                parts
                    .into_iter()
                    .map(bytes::Bytes::from)
                    .collect::<Vec<_>>(),
            )
            .unwrap()
        };
        let raw: Vec<u8> = [1u16, 0xffff]
            .iter()
            .flat_map(|p| p.to_le_bytes())
            .collect();
        let image = |frame: u64, compressed: bool| {
            let (enc, blob) = if compressed {
                ("lz4<", lz4_flex::block::compress(&raw))
            } else {
                ("<", raw.clone())
            };
            message(vec![
                format!(r#"{{"htype":"dimage-1.0","series":1,"frame":{frame},"hash":"x"}}"#)
                    .into_bytes(),
                format!(
                    r#"{{"htype":"dimage_d-1.0","shape":[2,1],"type":"uint16","encoding":"{enc}","size":{}}}"#,
                    blob.len()
                )
                .into_bytes(),
                blob,
            ])
        };
        let frame_done = |n: i32| {
            let s = Arc::clone(&s);
            async move {
                for _ in 0..500 {
                    if s.get_int(s.ad.base.array_counter).await == n {
                        return;
                    }
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
                panic!("frame {n} never published");
            }
        };
        let set = |updates: Vec<ParamUpdate>| {
            let s = Arc::clone(&s);
            async move { s.apply(updates).await }
        };

        set(vec![
            ParamUpdate::Int32(s.ad.base.array_callbacks, 1),
            ParamUpdate::Int32(s.p.signed_data, 0),
            ParamUpdate::Int32(s.p.stream_decompress, 1),
        ])
        .await;
        // A PUSH socket refuses to send until the PULL side has connected.
        let header = br#"{"htype":"dheader-1.0","series":1,"header_detail":"none"}"#;
        while push.send(message(vec![header.to_vec()])).await.is_err() {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        push.send(image(0, false)).await.unwrap();
        frame_done(1).await;
        let a = rx.recv().await.expect("frame 0");
        assert!(matches!(a.data().view(), NDDataView::U16(_)));

        set(vec![ParamUpdate::Int32(s.ad.base.array_callbacks, 0)]).await;
        push.send(image(1, false)).await.unwrap();
        frame_done(2).await;
        assert_eq!(
            rx.pending(),
            0,
            "ArrayCallbacks=0 on frame 1 still published it"
        );

        set(vec![
            ParamUpdate::Int32(s.ad.base.array_callbacks, 1),
            ParamUpdate::Int32(s.p.signed_data, 1),
        ])
        .await;
        push.send(image(2, false)).await.unwrap();
        frame_done(3).await;
        let a = rx.recv().await.expect("frame 2");
        assert!(
            matches!(a.data().view(), NDDataView::I16(_)),
            "SignedData=1 on frame 2"
        );

        set(vec![
            ParamUpdate::Int32(s.p.signed_data, 0),
            ParamUpdate::Int32(s.p.stream_decompress, 0),
        ])
        .await;
        push.send(image(3, true)).await.unwrap();
        frame_done(4).await;
        let a = rx.recv().await.expect("frame 3");
        assert!(a.codec.is_some(), "StreamDecompress=0 on frame 3");

        push.send(message(vec![
            br#"{"htype":"dseries_end-1.0","series":1}"#.to_vec(),
        ]))
        .await
        .unwrap();
        assert!(task.await.unwrap(), "dseries_end completes the series");
        std::mem::forget(runtime);
    }

    /// A FileWriter-shaped file: `/entry/data/data` of u16 images, `[n, h, w]`.
    fn h5_file(shape: [usize; 3]) -> Vec<u8> {
        let path = std::env::temp_dir().join(format!(
            "eiger-h5-{}-{:?}.h5",
            std::process::id(),
            std::thread::current().id()
        ));
        let file = rust_hdf5::H5File::create(&path).unwrap();
        let data = file
            .create_group("entry")
            .unwrap()
            .create_group("data")
            .unwrap();
        let n: usize = shape.iter().product();
        data.new_dataset::<u16>()
            .shape(shape)
            .create("data")
            .unwrap()
            .write_raw(&(0..n as u16).collect::<Vec<_>>())
            .unwrap();
        file.close().unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        bytes
    }

    /// An Eiger1 port with its interrupts open and its main output wired to
    /// the returned sink.
    fn h5_rig(
        name: &str,
    ) -> (
        Arc<Shared>,
        epics_rs::ad_core::plugin::channel::NDArrayReceiver,
    ) {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::base::runtime::interrupt_accept::InterruptGate;

        let (signals, _receivers) = signals();
        let cfg = crate::driver::EigerConfig {
            port_name: name.to_string(),
            api: ApiVersion::V1_6_0,
            model: Model::Eiger1,
            max_memory: 0,
        };
        let rest = RestApi::new("127.0.0.1", agreeable_detector());
        let mut det = crate::driver::EigerDriver::new(cfg, rest, signals.clone()).expect("driver");
        det.ad.port_base.own_interrupts(InterruptGate::opened());
        let (p, ad, ops) = (det.p, det.ad.params, det.ops.clone());
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), det, RuntimeConfig::default())
                .expect("runtime");
        let outputs = Outputs::new();
        let (tx, rx) = ndarray_channel(&format!("{name}_SINK"), 4);
        outputs.main.lock().add(tx);
        let s = shared(
            ops,
            runtime.port_handle().clone(),
            p,
            ad,
            Model::Eiger1,
            ApiVersion::V1_6_0,
            "127.0.0.1".to_string(),
            outputs,
            DriverPool::standalone(0),
            &signals,
        );
        std::mem::forget(runtime);
        (s, rx)
    }

    /// A client's store of `updates`, sent from the port thread as frame 0
    /// posts ArrayCounter: it reaches the port ahead of everything frame 1
    /// sends. A direct op, so the asyn queue cannot reorder it.
    fn store_after_frame_0(
        s: &Shared,
        updates: Vec<ParamSetValue>,
    ) -> epics_rs::asyn::interrupt::SyncCallbackSubscription {
        use epics_rs::asyn::interrupt::InterruptFilter;
        use epics_rs::asyn::request::RequestOp;
        use epics_rs::asyn::user::AsynUser;

        let pending = parking_lot::Mutex::new(Some(updates));
        let client = s.handle.clone();
        s.handle
            .interrupts()
            .register_sync_callback(
                InterruptFilter {
                    reason: Some(s.ad.base.array_counter),
                    addr: Some(0),
                    ..Default::default()
                },
                move |iv| {
                    if matches!(iv.value, ParamValue::Int32(1))
                        && let Some(updates) = pending.lock().take()
                    {
                        client
                            .try_submit(
                                RequestOp::CallParamCallbacks {
                                    addr: 0,
                                    updates,
                                    time_stamp: None,
                                },
                                AsynUser::new(0),
                            )
                            .expect("client store");
                    }
                },
            )
            .unwrap()
    }

    /// C parseH5File reads NDArrayCounter and ADNumImagesCounter once per file
    /// and counts on from there (eigerDetector.cpp:1841-1842, :1903-1904): a
    /// client store between two frames is overwritten, not counted on from.
    #[tokio::test]
    async fn parse_h5_counts_on_from_the_counters_read_at_file_start() {
        let (s, mut rx) = h5_rig("EIGER_H5_COUNTERS");
        let (counter, images) = (s.ad.base.array_counter, s.ad.num_images_counter);
        let _sub = store_after_frame_0(
            &s,
            [counter, images]
                .map(|r| ParamSetValue::new(r, 0, ParamValue::Int32(100)))
                .to_vec(),
        );

        s.set_int(s.ad.base.array_callbacks, 1).await;
        parse_h5(&s, &h5_file([2, 1, 2])).await.expect("parse");
        let mut ids = Vec::new();
        for _ in 0..2 {
            ids.push(rx.recv().await.expect("array").unique_id);
        }
        assert_eq!(ids, [0, 1]);
        assert_eq!(s.get_int(counter).await, 2);
        assert_eq!(s.get_int(images).await, 2);
    }

    /// C parseH5File reads NDArrayCallbacks for every frame, after its
    /// updateTimeStamps (eigerDetector.cpp:1880, :1892).
    #[tokio::test]
    async fn parse_h5_reads_array_callbacks_per_frame() {
        let (s, mut rx) = h5_rig("EIGER_H5_CALLBACKS");
        let callbacks = s.ad.base.array_callbacks;
        let _sub = store_after_frame_0(
            &s,
            vec![ParamSetValue::new(callbacks, 0, ParamValue::Int32(0))],
        );

        s.set_int(callbacks, 1).await;
        parse_h5(&s, &h5_file([2, 1, 2])).await.expect("parse");
        assert_eq!(rx.recv().await.expect("frame 0").unique_id, 0);
        assert_eq!(s.get_int(s.ad.base.array_counter).await, 2);
        assert_eq!(rx.pending(), 0, "ArrayCallbacks=0 before frame 1");
    }

    /// One single-part stream v1 message; a PUSH socket refuses to send until
    /// the PULL side has connected.
    async fn push_v1(push: &mut zeromq::PushSocket, part: &[u8]) {
        use zeromq::SocketSend;
        let message = || zeromq::ZmqMessage::from(part.to_vec());
        while push.send(message()).await.is_err() {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    /// C's header wait has no Stop exit (eigerDetector.cpp:1424-1462): a Stop
    /// while the header is pending leaves both stream versions waiting.
    #[tokio::test]
    async fn a_stop_does_not_end_the_stream_header_wait() {
        use zeromq::Socket;

        let (s, _rx) = h5_rig("EIGER_HEADER_STOP");
        s.flags.stop.store(true, Ordering::Release);
        let mut push = zeromq::PushSocket::new();
        let v1 = push.bind("tcp://127.0.0.1:0").await.expect("bind");
        let mut push2 = zeromq::PushSocket::new();
        let v2 = push2.bind("tcp://127.0.0.1:0").await.expect("bind");
        let mut stream = StreamApi::connect_to(&v1.to_string()).await.unwrap();
        let mut stream2 = Stream2Api::connect_to(&v2.to_string()).await.unwrap();
        let series = [
            tokio::spawn({
                let s = Arc::clone(&s);
                async move { stream_series(&s, &mut stream).await }
            }),
            tokio::spawn({
                let s = Arc::clone(&s);
                async move { stream2_series(&s, &mut stream2).await }
            }),
        ];
        tokio::time::sleep(Duration::from_millis(1500)).await;
        let finished = series.each_ref().map(|t| t.is_finished());
        series.iter().for_each(|t| t.abort());
        assert_eq!(finished, [false, false], "[v1, v2] ended under Stop");
    }

    /// C getHeader takes a series end as a wrong header type
    /// (streamApi.cpp:226-231), so the header wait goes on to the real header.
    #[tokio::test]
    async fn a_series_end_before_the_header_is_a_stray_packet() {
        use zeromq::Socket;

        let (s, rx) = h5_rig("EIGER_HEADER_END");
        s.set_int(s.ad.base.array_callbacks, 1).await;
        let mut push = zeromq::PushSocket::new();
        let endpoint = push.bind("tcp://127.0.0.1:0").await.expect("bind");
        let mut stream = StreamApi::connect_to(&endpoint.to_string()).await.unwrap();
        let series = tokio::spawn({
            let s = Arc::clone(&s);
            async move { stream_series(&s, &mut stream).await }
        });
        push_v1(&mut push, br#"{"htype":"dseries_end-1.0","series":0}"#).await;
        push_v1(&mut push, br#"{"htype":"dheader-1.0","series":1}"#).await;
        let pixels: Vec<u8> = [1u16, 2].iter().flat_map(|p| p.to_le_bytes()).collect();
        let image = zeromq::ZmqMessage::try_from(vec![
            bytes::Bytes::from_static(br#"{"htype":"dimage-1.0","series":1,"frame":0}"#),
            bytes::Bytes::from_static(
                br#"{"htype":"dimage_d-1.0","shape":[2,1],"type":"uint16","encoding":"<"}"#,
            ),
            bytes::Bytes::from(pixels),
        ])
        .unwrap();
        {
            use zeromq::SocketSend;
            push.send(image).await.unwrap();
        }
        push_v1(&mut push, br#"{"htype":"dseries_end-1.0","series":1}"#).await;
        assert!(series.await.unwrap());
        assert_eq!(rx.pending(), 1, "the series after the stray end");
    }

    /// C opens the shutter before each internal trigger and closes it after
    /// (eigerDetector.cpp:1003, :1007).
    #[tokio::test]
    async fn each_internal_trigger_is_wrapped_in_set_shutter() {
        use epics_rs::ad_core::driver::ShutterMode;
        use epics_rs::asyn::interrupt::InterruptFilter;

        let (s, _rx) = h5_rig("EIGER_SHUTTER");
        let seen = Arc::new(parking_lot::Mutex::new(Vec::new()));
        let _sub = {
            let seen = seen.clone();
            s.handle
                .interrupts()
                .register_sync_callback(
                    InterruptFilter {
                        reason: Some(s.ad.shutter_control_epics),
                        addr: Some(0),
                        ..Default::default()
                    },
                    move |iv| {
                        if let ParamValue::Int32(v) = iv.value {
                            seen.lock().push(v);
                        }
                    },
                )
                .unwrap()
        };
        s.apply(vec![
            ParamUpdate::Int32(s.ad.shutter_mode, ShutterMode::EpicsOnly as i32),
            ParamUpdate::Int32(s.p.data_source, crate::params::SOURCE_NONE),
            ParamUpdate::Int32(s.p.trigger_mode, TRIGGER_MODE_INTS),
            ParamUpdate::Int32(s.p.n_triggers, 2),
            ParamUpdate::Int32(s.p.num_images, 1),
            ParamUpdate::Int32(s.ad.status, ADStatus::Acquire as i32),
        ])
        .await;
        let (_ctl_tx, ctl_rx) = rt::command_channel::<CtlCommand>(1);
        let (poll_tx, _poll_rx) = rt::command_channel::<Acquisition>(1);
        let (_poll_done_tx, poll_done_rx) = rt::command_channel::<bool>(1);
        let (stream_start_tx, _stream_start_rx) = rt::command_channel::<()>(1);
        let (_stream_done_tx, stream_done_rx) = rt::command_channel::<bool>(1);
        let mut ch = TaskChannels {
            ctl_rx,
            poll_tx,
            poll_done_rx,
            stream_start_tx,
            stream_done_rx,
        };

        assert!(!acquire(&s, &mut ch).await);
        assert_eq!(*seen.lock(), [1, 0, 1, 0]);
    }

    /// eigerDetector.cpp's and eigerParam.cpp's ASYN_TRACE_FLOW lines, in the
    /// order C prints them.
    #[tokio::test]
    async fn flow_lines_print_under_the_ports_flow_mask() {
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        use zeromq::Socket;

        let port = "EIGER_FLOW";
        let (s, mut rx) = h5_rig(port);
        let manager = epics_rs::asyn::services::PortServices::global()
            .trace()
            .clone();
        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::FLOW);

        s.ops.fetch(s.p.fw_enable).unwrap();
        s.ops.fetch(s.p.file_owner).unwrap();
        s.ops.put_int(s.p.fw_enable, 1).unwrap();
        s.set_int(s.ad.base.array_callbacks, 1).await;
        parse_h5(&s, &h5_file([1, 1, 2])).await.expect("parse");
        rx.recv().await.expect("array");
        let job = Arc::new(FileJob {
            name: "f.h5".into(),
            save: false,
            parse: true,
            remove: AtomicBool::new(false),
            perms: 0o644,
            refs: AtomicUsize::new(1),
            data: parking_lot::Mutex::new(Some(Vec::new())),
        });
        reap(&s, job).await;
        // A stream silent past one timeout, then a header and the series end.
        let mut push = zeromq::PushSocket::new();
        let endpoint = push.bind("tcp://127.0.0.1:0").await.expect("bind");
        let mut stream = StreamApi::connect_to(&endpoint.to_string())
            .await
            .expect("connect");
        let series = tokio::spawn({
            let s = Arc::clone(&s);
            async move { stream_series(&s, &mut stream).await }
        });
        tokio::time::sleep(Duration::from_millis(1200)).await;
        push_v1(&mut push, br#"{"htype":"dheader-1.0","series":1}"#).await;
        push_v1(&mut push, br#"{"htype":"dseries_end-1.0","series":1}"#).await;
        assert!(series.await.unwrap());

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let mut from = 0;
        for line in [
            "Param[FW_ENABLE]::baseFetch: disabled\n",
            "Param[FW_ENABLE]::fetch<int>: 0\n",
            "Param[FILE_OWNER]::fetch<string>: \n",
            "Param[FW_ENABLE]::put<int>: 1\n",
            "Param[FW_ENABLE]::basePut: '",
            "eigerDetector:parseH5File: calling NDArray callback\n",
            "eigerDetector::reapTask: file=f.h5 refCount=1\n",
            "eigerDetector::reapTask: file=f.h5 reaped\n",
            "eigerDetector::streamTask: got stream timeout\n",
        ] {
            let at = text[from..]
                .find(line)
                .unwrap_or_else(|| panic!("{line:?} after byte {from}: {text:?}"));
            from += at + line.len();
        }
    }

    /// Arming with SaveFiles runs `checkPath()` and then reads NDFilePathExists
    /// (eigerDetector.cpp:870-871): an empty FilePath touches neither param, any
    /// other is written back with its separator.
    #[tokio::test]
    async fn arming_runs_ndarray_driver_check_path_on_the_file_path() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let name = "EIGER_CHECK_PATH";
        let (signals, _receivers) = signals();
        let cfg = crate::driver::EigerConfig {
            port_name: name.to_string(),
            api: ApiVersion::V1_6_0,
            model: Model::Eiger1,
            max_memory: 0,
        };
        let rest = RestApi::new("127.0.0.1", agreeable_detector());
        let det = crate::driver::EigerDriver::new(cfg, rest, signals.clone()).expect("driver");
        let (p, ad, ops) = (det.p, det.ad.params, det.ops.clone());
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), det, RuntimeConfig::default())
                .expect("runtime");
        let s = shared(
            ops,
            runtime.port_handle().clone(),
            p,
            ad,
            Model::Eiger1,
            ApiVersion::V1_6_0,
            "127.0.0.1".to_string(),
            Outputs::new(),
            DriverPool::standalone(0),
            &signals,
        );
        let (_ctl_tx, ctl_rx) = rt::command_channel::<CtlCommand>(1);
        let (poll_tx, _poll_rx) = rt::command_channel::<Acquisition>(1);
        let (_poll_done_tx, poll_done_rx) = rt::command_channel::<bool>(1);
        let (stream_start_tx, _stream_start_rx) = rt::command_channel::<()>(1);
        let (_stream_done_tx, stream_done_rx) = rt::command_channel::<bool>(1);
        let mut ch = TaskChannels {
            ctl_rx,
            poll_tx,
            poll_done_rx,
            stream_start_tx,
            stream_done_rx,
        };
        s.apply(vec![
            ParamUpdate::Int32(s.p.data_source, SOURCE_FILEWRITER),
            ParamUpdate::Int32(s.p.fw_enable, 1),
            ParamUpdate::Int32(s.p.save_files, 1),
            ParamUpdate::Octet(s.ad.base.file_path, String::new()),
        ])
        .await;
        let handle = s.handle.clone();
        let exists = s.ad.base.file_path_exists;

        assert!(!acquire(&s, &mut ch).await);
        assert!(
            handle.read_int32(exists, 0).await.is_err(),
            "an empty FilePath leaves NDFilePathExists undefined"
        );
        assert_eq!(s.get_str(s.ad.status_message).await, "Invalid file path");

        let missing = std::env::temp_dir().join(format!("eiger-no-dir-{}", std::process::id()));
        let missing = missing.to_str().unwrap().to_string();
        s.set_str(s.ad.base.file_path, &missing).await;
        assert!(!acquire(&s, &mut ch).await);
        assert_eq!(
            s.get_str(s.ad.base.file_path).await,
            format!("{missing}/"),
            "checkPath writes the path back with its separator"
        );
        assert_eq!(handle.read_int32(exists, 0).await.unwrap(), 0);
        std::mem::forget(runtime);
    }

    /// eigerDetector's writeInt32/writeFloat64/writeOctet call
    /// callParamCallbacks() (eigerDetector.cpp:596, :682, :772): a write to
    /// another address still posts what is pending at address 0.
    #[test]
    fn a_write_to_another_address_posts_address_zero() {
        use epics_rs::asyn::interrupt::{InterruptFilter, InterruptValue};
        use epics_rs::asyn::port::PortDriver;
        use epics_rs::asyn::user::AsynUser;
        use epics_rs::base::runtime::interrupt_accept::InterruptGate;

        let (signals, _receivers) = signals();
        let cfg = crate::driver::EigerConfig {
            port_name: "EIGER_ADDR0_POST".to_string(),
            api: ApiVersion::V1_6_0,
            model: Model::Eiger1,
            max_memory: 0,
        };
        let rest = RestApi::new("127.0.0.1", agreeable_detector());
        let mut det = crate::driver::EigerDriver::new(cfg, rest, signals).expect("driver");
        det.ad.port_base.own_interrupts(InterruptGate::opened());
        let seen = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let seen_cb = std::sync::Arc::clone(&seen);
        let _sub = det
            .ad
            .port_base
            .interrupts
            .register_sync_callback(
                InterruptFilter {
                    addr: Some(0),
                    ..Default::default()
                },
                move |iv: &InterruptValue| seen_cb.lock().unwrap().push(iv.reason),
            )
            .unwrap();
        let pending = det.ad.params.shutter_close_delay;
        // A store marks the parameter changed only when the value differs.
        let pend = |det: &mut crate::driver::EigerDriver, value: f64| {
            det.ad
                .port_base
                .set_float64_param(pending, 0, value)
                .unwrap();
            seen.lock().unwrap().clear();
        };
        pend(&mut det, 1.5);
        let reason = det.ad.params.num_images;
        let _ = det.write_int32(&mut AsynUser::new(reason).with_addr(1), 3);
        assert!(seen.lock().unwrap().contains(&pending), "writeInt32");

        pend(&mut det, 2.5);
        let reason = det.ad.params.shutter_open_delay;
        let _ = det.write_float64(&mut AsynUser::new(reason).with_addr(1), 2.0);
        assert!(seen.lock().unwrap().contains(&pending), "writeFloat64");

        pend(&mut det, 3.5);
        let reason = det.ad.params.base.file_name;
        let _ = det.write_octet(&mut AsynUser::new(reason).with_addr(1), b"f");
        assert!(seen.lock().unwrap().contains(&pending), "writeOctet");
    }
}
