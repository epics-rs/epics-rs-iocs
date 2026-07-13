//! The background tasks (port of the ten `epicsThreadCreate` threads in
//! `eigerDetector`, eigerDetector.cpp:436-474).
//!
//! Each task is an OS thread with its own single-threaded async runtime, so a
//! blocking HTTP call in one never stalls another — the same isolation C gets
//! from ten `epicsThread`s. The C `epicsMessageQueue`s become
//! [`rt::command_channel`]s and the `epicsEvent`s become either a channel
//! message or an atomic flag, whichever matches how the event is consumed.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicUsize, Ordering};
use std::time::Duration;

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
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
}

/// The driver's end of every task signal.
#[derive(Clone)]
pub struct Signals {
    ctl_tx: rt::CommandSender<CtlCommand>,
    init_tx: rt::CommandSender<()>,
    restart_tx: rt::CommandSender<()>,
    flags: Arc<Flags>,
}

impl Signals {
    pub fn start(&self) {
        if self.ctl_tx.try_send(CtlCommand::Start).is_err() {
            log::error!("eiger: control task is not running");
        }
    }

    pub fn stop(&self) {
        self.flags.stop.store(true, Ordering::Release);
    }

    pub fn trigger(&self) {
        self.flags.trigger.store(true, Ordering::Release);
    }

    pub fn initialize(&self) {
        if self.init_tx.try_send(()).is_err() {
            log::error!("eiger: initialize task is not running");
        }
    }

    pub fn restart(&self) {
        if self.restart_tx.try_send(()).is_err() {
            log::error!("eiger: restart task is not running");
        }
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
                    ParamSetValue::new(reason, 0, ParamValue::Octet(value))
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
        match self.handle.read_octet(reason, 0, 256).await {
            Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
            Err(_) => String::new(),
        }
    }

    /// Fetch a remote parameter and push the result into the param library.
    async fn fetch(&self, reason: usize) {
        match self.ops.fetch(reason) {
            Ok(updates) => self.apply(updates).await,
            Err(e) => log::warn!("eiger: fetch failed: {e}"),
        }
    }

    /// Fetch a remote parameter and return its string value.
    async fn fetch_str(&self, reason: usize) -> String {
        self.fetch(reason).await;
        self.get_str(reason).await
    }

    async fn put_int(&self, reason: usize, value: i32) {
        match self.ops.put_int(reason, value) {
            Ok(updates) => self.apply(updates).await,
            Err(e) => log::error!("eiger: write failed: {e}"),
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
pub fn start(
    shared: Arc<Shared>,
    ctl_rx: rt::CommandReceiver<CtlCommand>,
    init_rx: rt::CommandReceiver<()>,
    restart_rx: rt::CommandReceiver<()>,
) -> Vec<std::thread::JoinHandle<()>> {
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
                stream_task(s, stream_start_rx, stream_done_tx).await;
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
    while rx.recv().await.is_some() {
        log::warn!("eiger: sending the {which} command");
        let (result, param) = match which {
            "initialize" => (s.rest().initialize(), s.p.initialize),
            _ => (s.rest().restart(), s.p.restart),
        };
        s.set_int(param, 0).await;
        if let Err(e) = result {
            log::error!("eiger: {which} failed: {e}");
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
        let path = s.get_str(s.ad.base.file_path).await;
        let exists = !path.is_empty() && Path::new(&path).is_dir();
        s.set_int(s.ad.base.file_path_exists, i32::from(exists))
            .await;
        if !exists {
            err = Some("Invalid file path");
        }
    }
    if let Some(err) = err {
        log::error!("eiger: {err}");
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
    let sequence_id = match s.rest().arm() {
        Ok(id) => id,
        Err(e) => {
            log::error!("eiger: failed to arm the detector: {e}");
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
        if ch.poll_tx.try_send(acq).is_ok() {
            wait_poll = true;
        } else {
            log::error!("eiger: poll task is not running");
        }
    }

    if data_source == SOURCE_STREAM {
        s.stream_complete.store(false, Ordering::Release);
        if ch.stream_start_tx.try_send(()).is_ok() {
            wait_stream = true;
        } else {
            log::error!("eiger: stream task is not running");
        }
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
                let exposure = (trigger_mode == TRIGGER_MODE_INTE).then_some(acquire_time);
                let timeout = Duration::from_secs_f64(trigger_timeout.max(0.0));
                if let Err(e) = s.rest().trigger(timeout, exposure) {
                    log::error!("eiger: trigger failed: {e}");
                }
                triggers += 1;
            }
        }
    }

    // Wait for the detector to leave the acquisition states, for the expected
    // image count to arrive, or for a stop.
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

    if let Err(e) = s.rest().disarm() {
        log::error!("eiger: disarm failed: {e}");
    }

    s.put_int(p.armed, 0).await;
    s.set_str(s.ad.status_message, "Processing files").await;

    let mut success = true;
    if wait_poll {
        // Let the FileWriter finish writing before telling the poll task to stop.
        loop {
            if s.fetch_str(p.fw_state).await != "acquire" {
                break;
            }
            rt::sleep(Duration::from_millis(100)).await;
        }
        rt::sleep(Duration::from_millis(500)).await;

        s.poll_stop.store(true, Ordering::Release);
        let complete = ch.poll_done_rx.recv().await.unwrap_or(false);
        success = success && complete;
    }

    if wait_stream {
        let complete = ch.stream_done_rx.recv().await.unwrap_or(false);
        success = success && complete;
    }
    if !success {
        log::warn!("eiger: acquisition did not complete cleanly");
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
            match s.rest().wait_file(&file.name, Duration::from_secs(1)) {
                Ok(true) => {
                    if file.save || file.parse {
                        s.add_pending(1).await;
                        if download_tx.send(file.clone()).await.is_err() {
                            log::error!("eiger: download task is not running");
                            s.add_pending(-1).await;
                        }
                    } else if file.remove.load(Ordering::Acquire)
                        && let Err(e) = s.rest().delete_file(&file.name)
                    {
                        log::error!("eiger: delete of {} failed: {e}", file.name);
                    }
                    i += 1;
                }
                // The file is not there yet. While the acquisition is running
                // that is normal; once the control task has asked us to stop it
                // means the acquisition was aborted and the file will never come.
                Ok(false) => {
                    if s.poll_stop.load(Ordering::Acquire) {
                        retries += 1;
                    }
                }
                Err(e) => {
                    log::error!("eiger: waiting for {} failed: {e}", file.name);
                    if s.poll_stop.load(Ordering::Acquire) {
                        retries += 1;
                    } else {
                        rt::sleep(Duration::from_millis(100)).await;
                    }
                }
            }
        }

        // Every file we did claim must be reaped before the acquisition is over.
        while s.pending_files.load(Ordering::Acquire) > 0 {
            rt::sleep(Duration::from_millis(100)).await;
        }

        s.poll_complete.store(i == total_files, Ordering::Release);
        let _ = done_tx.send(i == total_files).await;
    }
}

async fn download_task(
    s: Arc<Shared>,
    mut rx: rt::CommandReceiver<Arc<FileJob>>,
    parse_tx: rt::CommandSender<Arc<FileJob>>,
    save_tx: rt::CommandSender<Arc<FileJob>>,
    reap_tx: rt::CommandSender<Arc<FileJob>>,
) {
    while let Some(file) = rx.recv().await {
        match s.rest().get_file(&file.name) {
            Err(e) => {
                log::error!("eiger: download of {} failed: {e}", file.name);
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
        let data = file.data.lock().clone();
        match data {
            Some(data) => {
                if let Err(e) = parse_h5(&s, &data).await {
                    log::error!("eiger: parsing {} failed: {e}", file.name);
                }
            }
            None => log::error!("eiger: {} arrived at the parse task empty", file.name),
        }
        let _ = reap_tx.send(file).await;
    }
}

/// Publish every image in one FileWriter data file (C `parseH5File`).
async fn parse_h5(s: &Arc<Shared>, data: &[u8]) -> Result<(), String> {
    let signed_data = s.get_int(s.p.signed_data).await != 0;
    let frames = h5::parse(data, signed_data)?;

    // The thresholds that were enabled for this acquisition, in order — the
    // Nth frame of a 4-D dataset belongs to the Nth *active* threshold.
    let active = active_thresholds(s).await;
    let array_callbacks = s.get_int(s.ad.base.array_callbacks).await != 0;

    for frame in frames {
        let (number, energy) = active
            .get(frame.threshold)
            .copied()
            .unwrap_or((frame.threshold as i32 + 1, 0.0));
        let mut attrs = NDAttributeList::new();
        attrs.add(threshold_number_attr(number));
        attrs.add(threshold_energy_attr(energy));

        publish(
            s,
            Image {
                dims: frame.dims,
                data: frame.data,
                codec: None,
                attributes: attrs,
                threshold_index: frame.threshold,
            },
            array_callbacks,
        )
        .await;
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
        let data = file.data.lock().clone();
        let Some(data) = data else {
            log::error!("eiger: {} arrived at the save task empty", file.name);
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

        if let Err(e) = write_file(&full, &data, file.perms) {
            log::error!("eiger: writing {full} failed: {e}");
            // The local copy failed, so the detector's copy is the only one left:
            // do not let the reaper delete it.
            file.remove.store(false, Ordering::Release);
        }
        let _ = reap_tx.send(file).await;
    }
}

/// Write one downloaded file to `FilePath` with `FILE_PERMISSIONS`.
///
/// SCOPED GAP: C additionally honours `FILE_OWNER` / `FILE_OWNER_GROUP` by
/// resolving them with `getpwnam`/`getgrnam` and wrapping the write in
/// `setfsuid`/`setfsgid` (eigerDetector.cpp:719, 1266-1267). That is a Linux-only,
/// libc-dependent path with no portable Rust equivalent; it is not ported. The
/// two PVs still exist and hold their strings, but they do not change the owner
/// of the written file.
fn write_file(path: &str, data: &[u8], perms: u32) -> std::io::Result<()> {
    use std::io::Write;
    use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};

    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .mode(perms)
        .open(path)?;
    f.write_all(data)?;
    // An existing file keeps its old mode through `open`, so set it explicitly
    // (C calls fchmod for the same reason).
    f.set_permissions(std::fs::Permissions::from_mode(perms))?;
    Ok(())
}

/// Drop one reference to a file; the last one deletes it from the detector
/// (C `reapTask`, eigerDetector.cpp:1331).
async fn reap(s: &Arc<Shared>, file: Arc<FileJob>) {
    if file.refs.fetch_sub(1, Ordering::AcqRel) != 1 {
        return;
    }
    if file.remove.load(Ordering::Acquire)
        && let Err(e) = s.rest().delete_file(&file.name)
    {
        log::error!("eiger: delete of {} failed: {e}", file.name);
    }
    s.fetch(s.p.fw_free).await;
    *file.data.lock() = None;
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
            match s.rest().get_monitor_image(timeout) {
                Ok(buf) => match tiff::decode(&buf) {
                    Ok(image) => {
                        let array = build_array(
                            image.dims,
                            image.data,
                            None,
                            NDAttributeList::new(),
                            unique_id,
                        );
                        unique_id += 1;
                        ArrayPublisher::new(s.outputs.monitor.clone())
                            .publish(Arc::new(array))
                            .await;
                    }
                    Err(e) => log::error!("eiger: couldn't parse the monitor image: {e}"),
                },
                // No new image within the long-poll window is the normal case
                // (C ignores the failure, eigerDetector.cpp:1384).
                Err(e) => log::debug!("eiger: no monitor image: {e}"),
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
    done_tx: rt::CommandSender<bool>,
) {
    let mut api: Option<StreamApi> = None;

    while start_rx.recv().await.is_some() {
        let version = s.get_int(s.p.stream_version).await;
        if version != STREAM_VERSION_STREAM {
            log::error!(
                "eiger: stream format {version} (stream2/CBOR) is not implemented; \
                 set STREAM_VERSION to 0 (stream)"
            );
            let _ = done_tx.send(false).await;
            continue;
        }

        if api.is_none() {
            match StreamApi::connect(&s.hostname).await {
                Ok(a) => api = Some(a),
                Err(e) => {
                    log::error!("eiger: cannot connect to the zmq stream: {e}");
                    let _ = done_tx.send(false).await;
                    continue;
                }
            }
        }
        let stream = api.as_mut().expect("just connected");
        let complete = stream_series(&s, stream).await;
        s.stream_complete.store(complete, Ordering::Release);

        s.fetch(s.p.stream_dropped).await;
        let _ = done_tx.send(complete).await;
    }
}

/// Receive one whole series. Returns whether it ended with a `dseries_end`
/// (C `mStreamComplete`).
async fn stream_series(s: &Arc<Shared>, stream: &mut StreamApi) -> bool {
    let decompress = s.get_int(s.p.stream_decompress).await != 0;
    let timeout = Duration::from_secs(1);

    // Wait out the series header.
    loop {
        match stream.recv(timeout, decompress).await {
            Ok(Some(StreamMessage::Header(_))) => break,
            Ok(Some(StreamMessage::End)) => return true,
            Ok(Some(StreamMessage::Image(_))) => {
                log::error!("eiger: got an image before the series header, ignoring");
            }
            Ok(None) => {}
            Err(crate::stream::StreamError::WrongHtype(h)) => {
                log::error!("eiger: got stray packet ({h}), ignoring");
            }
            Err(e) => {
                log::error!("eiger: failed to get the header packet: {e}");
                return false;
            }
        }
        if s.flags.stop.load(Ordering::Acquire) {
            return false;
        }
    }

    let signed_data = s.get_int(s.p.signed_data).await != 0;
    let array_callbacks = s.get_int(s.ad.base.array_callbacks).await != 0;

    loop {
        let msg = match stream.recv(timeout, decompress).await {
            Ok(Some(m)) => m,
            Ok(None) => continue,
            Err(crate::stream::StreamError::WrongHtype(h)) => {
                log::error!("eiger: got stray packet ({h}), ignoring");
                continue;
            }
            // UPSTREAM DEFECT (streamApi.cpp:1503-1507 in the caller): C assigns
            // `err = mStreamAPI->getFrame(&pArray, ...)` and then dereferences
            // `pArray` without ever testing `err`, so a failed decode reads an
            // uninitialised pointer. Here a failed frame ends the series.
            Err(e) => {
                log::error!("eiger: failed to get a frame packet: {e}");
                return false;
            }
        };

        match msg {
            StreamMessage::End => return true,
            StreamMessage::Header(_) => {
                log::error!("eiger: got a second series header, ignoring");
            }
            StreamMessage::Image(frame) => {
                // Stream v1 carries one image per message and no threshold axis
                // (C `numThresholds` stays 1 on this path, eigerDetector.cpp:1422),
                // so every frame goes to the main output and to threshold 1 —
                // C's addresses 0 and 1. It also carries no ThresholdNumber /
                // ThresholdEnergy attributes; only the HDF5 and stream2 paths add
                // those.
                let data = if signed_data && frame.codec.is_none() {
                    reinterpret_signed(frame.data)
                } else {
                    frame.data
                };
                publish(
                    s,
                    Image {
                        dims: frame.dims,
                        data,
                        codec: frame.codec,
                        attributes: NDAttributeList::new(),
                        threshold_index: 0,
                    },
                    array_callbacks,
                )
                .await;
            }
        }
    }
}

/// Relabel unsigned pixels as signed (C's `SignedData`, eigerDetector.cpp:1519).
///
/// The detector's counts are unsigned, but bad pixels and gaps are very large
/// positive numbers that wreck autoscaling; re-tagging the same bits as signed
/// turns them into small negatives at the cost of half the count range.
fn reinterpret_signed(data: NDDataBuffer) -> NDDataBuffer {
    match data {
        NDDataBuffer::U8(v) => NDDataBuffer::I8(v.into_iter().map(|x| x as i8).collect()),
        NDDataBuffer::U16(v) => NDDataBuffer::I16(v.into_iter().map(|x| x as i16).collect()),
        NDDataBuffer::U32(v) => NDDataBuffer::I32(v.into_iter().map(|x| x as i32).collect()),
        other => other,
    }
}

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

fn build_array(
    dims: [usize; 2],
    data: NDDataBuffer,
    codec: Option<epics_rs::ad_core::codec::Codec>,
    attributes: NDAttributeList,
    unique_id: i32,
) -> NDArray {
    let mut array = NDArray::with_data(
        vec![NDDimension::new(dims[0]), NDDimension::new(dims[1])],
        data,
    );
    array.unique_id = unique_id;
    array.timestamp = EpicsTimestamp::now();
    array.time_stamp = array.timestamp.as_f64();
    array.attributes = attributes;
    array.codec = codec;
    array
}

/// Publish one image on the main output and on its threshold's output
/// (C `doCallbacksGenericPointer(pArray, NDArrayData, 0)` plus the
/// `thresh + 1` address).
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
}

async fn publish(s: &Arc<Shared>, image: Image, array_callbacks: bool) {
    let Image {
        dims,
        data,
        codec,
        attributes,
        threshold_index,
    } = image;
    let counter = s.get_int(s.ad.base.array_counter).await + 1;
    let array = Arc::new(build_array(dims, data, codec, attributes, counter));

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

    let n_elements: i64 = array.dims.iter().map(|d| d.size as i64).product();
    let element_size = array.data.data_type().element_size() as i64;
    let array_size = array
        .codec
        .as_ref()
        .map(|c| c.compressed_size as i64)
        .unwrap_or(n_elements * element_size)
        .min(i64::from(i32::MAX)) as i32;
    let codec_name = array
        .codec
        .as_ref()
        .map(|c| c.name.as_str())
        .unwrap_or("")
        .to_string();

    let num_images_counter = s.get_int(s.ad.num_images_counter).await + 1;
    s.apply(vec![
        ParamUpdate::Int32(s.ad.base.array_counter, counter),
        ParamUpdate::Int32(s.ad.num_images_counter, num_images_counter),
        ParamUpdate::Float64(s.ad.base.timestamp_rbv, array.time_stamp),
        ParamUpdate::Int32(s.ad.base.epics_ts_sec, array.timestamp.sec as i32),
        ParamUpdate::Int32(s.ad.base.epics_ts_nsec, array.timestamp.nsec as i32),
        ParamUpdate::Int32(s.ad.base.array_size, array_size),
        ParamUpdate::Int32(s.ad.base.n_dimensions, array.dims.len() as i32),
        ParamUpdate::Int32(s.ad.base.data_type, array.data.data_type() as u8 as i32),
        ParamUpdate::Octet(s.ad.base.codec, codec_name),
        ParamUpdate::Int32(
            s.ad.base.compressed_size,
            array
                .codec
                .as_ref()
                .map(|c| c.compressed_size.min(i32::MAX as usize) as i32)
                .unwrap_or(0),
        ),
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
        pending_files: AtomicI32::new(0),
        poll_stop: AtomicBool::new(false),
        poll_complete: AtomicBool::new(false),
        stream_complete: AtomicBool::new(false),
        flags: signals.flags.clone(),
    })
}

/// Create the driver's signal handles and the matching receiver ends.
pub fn signals() -> (
    Signals,
    rt::CommandReceiver<CtlCommand>,
    rt::CommandReceiver<()>,
    rt::CommandReceiver<()>,
) {
    let (ctl_tx, ctl_rx) = rt::command_channel::<CtlCommand>(4);
    let (init_tx, init_rx) = rt::command_channel::<()>(1);
    let (restart_tx, restart_rx) = rt::command_channel::<()>(1);
    (
        Signals {
            ctl_tx,
            init_tx,
            restart_tx,
            flags: Arc::new(Flags::default()),
        },
        ctl_rx,
        init_rx,
        restart_rx,
    )
}
