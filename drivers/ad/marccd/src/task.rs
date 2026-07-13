//! The three marCCD worker threads and the acquisition state machine.
//!
//! * `MarccdCmdTask` executes the server work C does inline in `writeInt32` /
//!   `writeFloat64` (`set_bin`, `set_gating`, `set_readout_mode`,
//!   `set_frameshift`, `set_stability`, `get_state` for `ADReadStatus`,
//!   `saveFile` for `NDWriteFile`) plus the `ADAcquire == 1` `getState`
//!   precondition and start-event signalling.
//! * `MarccdTask` is C's `marCCDTask` acquisition state machine
//!   (`collectNormal` / `collectSeries`).
//! * `MarccdImageTask` is C's `getImageDataTask` — the overlapped
//!   correction/readback path.
//!
//! All three reach the marServer socket through the shared [`Server`] behind a
//! `tokio::sync::Mutex`, exactly as C's three contexts share the driver lock.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use tokio::sync::Mutex as AsyncMutex;

use crate::file_name::{format_file_template, full_file_template};
use crate::image::decode_tiff;
use crate::protocol::{cmd_dezinger, cmd_readout, cmd_set_bin, cmd_writefile};
use crate::protocol::{
    cmd_set_frameshift, cmd_set_gating, cmd_set_readout_mode, cmd_set_stability,
    cmd_start_series_timed, cmd_start_series_triggered_itemp, cmd_start_series_triggered_timed,
};
use crate::server::Server;
use crate::types::{
    CamStatus, Event, FILE_READ_DELAY, FrameType, ImageMode, MARCCD_POLL_DELAY, MAX_FILENAME_LEN,
    TASK_ACQUIRE, TASK_CORRECT, TASK_DEZINGER, TASK_READ, TASK_STATE_BUSY, TASK_STATE_ERROR,
    TASK_STATUS_EXECUTING, TASK_STATUS_QUEUED, TASK_WRITE, TriggerMode, secs, task_state,
    test_task_status,
};

/// Work C performs synchronously inside an asyn `write*` callback. A Rust
/// `PortDriver` method must not block on another port, so the actor enqueues
/// these instead.
#[derive(Debug)]
pub enum Cmd {
    /// The server round-trips C runs at the end of the constructor:
    /// `getServerMode`, `getConfig`, `getState`.
    Init,
    /// C's `ADAcquire == 1` branch: `getState`, and if the acquire task is not
    /// already queued/executing, clear a stale stop and signal the start event.
    StartAcquire,
    /// C's `ADBinX` / `ADBinY` branch: `set_bin,<x>,<y>`.
    SetBin,
    /// C's `marCCDGateMode` branch (server mode 2 only): `set_gating,<v>` then
    /// `getConfig`.
    SetGating(i32),
    /// C's `marCCDReadoutMode` branch (server mode 2 only):
    /// `set_readout_mode,<v>` then `getConfig`.
    SetReadoutMode(i32),
    /// C's `marCCDFrameShift` branch: `set_frameshift,<v>` then `getConfig`.
    SetFrameShift(i32),
    /// C's `ADReadStatus` branch: `getState`.
    ReadStatus,
    /// C's `NDWriteFile` branch: `saveFile(correctedFlag, 1)`.
    WriteFile,
    /// C's `marCCDStability` branch: `set_stability,<v>` then `getConfig`.
    SetStability(f64),
}

/// Everything a worker thread needs to reach the driver.
pub(crate) struct Worker {
    pub server: Arc<AsyncMutex<Server>>,
    pub start: Arc<Event>,
    pub stop: Arc<Event>,
    pub image_event: Arc<Event>,
    /// C `acqStartTime` as an f64 epoch second count (bits).
    pub acq_start: Arc<AtomicU64>,
    pub output: ArrayPublisher,
}

/// Current time as an f64 epoch second count (C `secPastEpoch + nsec/1e9`).
fn now_epoch_f64() -> f64 {
    match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(d) => d.as_secs_f64(),
        Err(e) => -(e.duration().as_secs_f64()),
    }
}

/// Whole seconds since the epoch, matching C's `time_t` truncation used for the
/// TIFF modification-time comparison.
fn epoch_secs(t: SystemTime) -> i64 {
    match t.duration_since(UNIX_EPOCH) {
        Ok(d) => d.as_secs() as i64,
        Err(e) => -(e.duration().as_secs() as i64),
    }
}

impl Worker {
    fn store_acq_start(&self, v: f64) {
        self.acq_start.store(v.to_bits(), Ordering::Relaxed);
    }

    fn load_acq_start(&self) -> f64 {
        f64::from_bits(self.acq_start.load(Ordering::Relaxed))
    }

    /// C `getState`, locking the shared server for the round-trip.
    async fn get_state(&self) -> i32 {
        self.server.lock().await.get_state().await
    }

    // -----------------------------------------------------------------------
    // acquireFrame / readoutFrame / saveFile
    // -----------------------------------------------------------------------

    /// C `acquireFrame`.
    async fn acquire_frame(&self, exposure_time: f64, use_shutter: bool) {
        let trigger_mode = {
            let mut guard = self.server.lock().await;
            let srv = &mut *guard;
            srv.get_i32(srv.ad.trigger_mode).await
        };

        // Wait for the acquire task to be done with the previous acquisition.
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_ACQUIRE, TASK_STATUS_EXECUTING) != 0
            || task_state(s) >= TASK_STATE_BUSY
        {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
        }

        {
            let mut guard = self.server.lock().await;
            let srv = &mut *guard;
            srv.set_str(srv.ad.status_message, "Starting exposure");
            srv.write_server("start").await;
            srv.callbacks().await;
        }

        // Wait for acquisition to actually start.
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_ACQUIRE, TASK_STATUS_EXECUTING) == 0
            || task_state(s) >= TASK_STATE_BUSY
        {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
        }

        let start = Instant::now();
        if use_shutter {
            self.server.lock().await.set_shutter(true).await;
        }

        // Wait for the exposure time, aborting on the stop event. In internal
        // trigger mode C arms an epicsTimer that signals the stop event when the
        // exposure time expires; this port folds that deadline into the wait so
        // no separate timer thread is needed. In external trigger modes there is
        // no timer and the loop ends only on an abort (external hardware
        // starts/stops the acquisition).
        let internal = trigger_mode == TriggerMode::Internal as i32;
        let deadline = internal.then(|| start + secs(exposure_time));
        loop {
            let wait = match deadline {
                Some(dl) => {
                    let now = Instant::now();
                    if now >= dl {
                        break;
                    }
                    (dl - now).min(secs(MARCCD_POLL_DELAY))
                }
                None => secs(MARCCD_POLL_DELAY),
            };
            if self.stop.wait_timeout(wait) {
                break;
            }
            let mut time_remaining = exposure_time - start.elapsed().as_secs_f64();
            if time_remaining < 0.0 {
                time_remaining = 0.0;
            }
            let mut guard = self.server.lock().await;
            let srv = &mut *guard;
            srv.set_f64(srv.ad.time_remaining, time_remaining);
            srv.callbacks().await;
        }

        {
            let mut guard = self.server.lock().await;
            let srv = &mut *guard;
            srv.set_f64(srv.ad.time_remaining, 0.0);
            srv.callbacks().await;
        }
        if use_shutter {
            self.server.lock().await.set_shutter(false).await;
        }
    }

    /// C `readoutFrame`.
    async fn readout_frame(
        &self,
        buffer_number: i32,
        file_name: Option<&str>,
        wait: bool,
    ) -> CamStatus {
        let has_file = matches!(file_name, Some(n) if !n.is_empty());

        // Wait for the readout task to be done with the previous frame.
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_READ, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
            || task_state(s) >= TASK_STATE_BUSY
        {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
            if task_state(s) == TASK_STATE_ERROR {
                return CamStatus::Error;
            }
        }

        {
            let mut guard = self.server.lock().await;
            let srv = &mut *guard;
            if has_file {
                let name = file_name.unwrap().to_string();
                srv.set_str(srv.ad.base.full_file_name, name);
                srv.callbacks().await;
            }
            srv.write_server(&cmd_readout(buffer_number, file_name))
                .await;
        }

        // Wait for the readout to start.
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_READ, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) == 0 {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
            if task_state(s) == TASK_STATE_ERROR {
                return CamStatus::Error;
            }
        }

        // Wait for the readout to complete.
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_READ, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0 {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
            if task_state(s) == TASK_STATE_ERROR {
                return CamStatus::Error;
            }
        }

        if !wait {
            return CamStatus::Success;
        }

        // Wait for the correction to complete.
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_CORRECT, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0 {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
            if task_state(s) == TASK_STATE_ERROR {
                return CamStatus::Error;
            }
        }

        // If a filename was specified, wait for the write to complete.
        if !has_file {
            return CamStatus::Success;
        }
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_WRITE, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
            || task_state(s) >= TASK_STATE_BUSY
        {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
            if task_state(s) == TASK_STATE_ERROR {
                return CamStatus::Error;
            }
        }
        CamStatus::Success
    }

    /// C `saveFile`.
    async fn save_file(&self, corrected_flag: i32, wait: bool) {
        // Wait for any previous write to complete.
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_WRITE, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
            || task_state(s) >= TASK_STATE_BUSY
        {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
        }

        self.server.lock().await.write_header().await;
        let full_file_name = self.create_file_name().await;
        {
            let mut guard = self.server.lock().await;
            let srv = &mut *guard;
            srv.write_server(&cmd_writefile(&full_file_name, corrected_flag))
                .await;
            srv.set_str(srv.ad.base.full_file_name, full_file_name.clone());
            srv.callbacks().await;
        }
        if !wait {
            return;
        }
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_WRITE, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
            || task_state(s) >= TASK_STATE_BUSY
        {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
        }
    }

    /// C `asynNDArrayDriver::createFileName`.
    async fn create_file_name(&self) -> String {
        let mut guard = self.server.lock().await;
        let srv = &mut *guard;
        let base = srv.ad.base;
        let file_path = srv.get_str(base.file_path, MAX_FILENAME_LEN).await;
        let file_name = srv.get_str(base.file_name, MAX_FILENAME_LEN).await;
        let template = srv.get_str(base.file_template, MAX_FILENAME_LEN).await;
        let file_number = srv.get_i32(base.file_number).await;
        let auto_increment = srv.get_i32(base.auto_increment).await;

        let full = format_file_template(&template, &file_path, &file_name, file_number)
            .unwrap_or_else(|| {
                log::error!("marccd: cannot expand NDFileTemplate {template:?}");
                String::new()
            });
        if auto_increment != 0 {
            srv.set_i32(base.file_number, file_number + 1);
        }
        full
    }

    // -----------------------------------------------------------------------
    // Image readback
    // -----------------------------------------------------------------------

    /// C `getImageData`.
    async fn get_image_data(&self) -> CamStatus {
        // Inquire about the image dimensions.
        self.server.lock().await.get_config().await;

        let (full_file_name, nx, ny, image_counter, array_callbacks) = {
            let mut guard = self.server.lock().await;
            let srv = &mut *guard;
            let base = srv.ad.base;
            let full = srv.get_str(base.full_file_name, MAX_FILENAME_LEN).await;
            let nx = srv.get_i32(base.array_size_x).await;
            let ny = srv.get_i32(base.array_size_y).await;
            let ic = srv.get_i32(base.array_counter).await;
            let ac = srv.get_i32(base.array_callbacks).await;
            (full, nx.max(0) as usize, ny.max(0) as usize, ic, ac)
        };

        {
            let mut guard = self.server.lock().await;
            let srv = &mut *guard;
            srv.set_str(
                srv.ad.status_message,
                format!("Reading TIFF file {full_file_name}"),
            );
            srv.callbacks().await;
        }

        let data = match self.read_tiff(&full_file_name, nx, ny).await {
            Ok(d) => d,
            // Fixed (upstream-c-defects.md #14): on a read error C still runs
            // doCallbacksGenericPointer with the unfilled buffer, publishing
            // garbage as a frame. Propagate the error and do not publish.
            Err(s) => return s,
        };

        if array_callbacks != 0 {
            let mut array = NDArray::with_data(
                vec![NDDimension::new(nx), NDDimension::new(ny)],
                NDDataBuffer::U16(data),
            );
            array.unique_id = image_counter;
            array.timestamp = EpicsTimestamp::now();
            // C uses `acqStartTime` for the double timestamp, and
            // `updateTimeStamp` (current time) for the epicsTS.
            array.time_stamp = self.load_acq_start();
            self.output.publish(Arc::new(array)).await;
        }
        CamStatus::Success
    }

    /// C `readTiff` — wait for a new file, then retry the decode until the
    /// writer has finished.
    ///
    /// Fixed (upstream-c-defects.md #13): when the retry loop exhausts `timeout`
    /// without a successful decode, C falls through and returns `asynSuccess`
    /// with a buffer it never filled — the caller then publishes stale pixels.
    /// This returns [`CamStatus::Error`] instead so no image is published.
    async fn read_tiff(&self, file: &str, nx: usize, ny: usize) -> Result<Vec<u16>, CamStatus> {
        let timeout = {
            let mut guard = self.server.lock().await;
            let srv = &mut *guard;
            srv.get_f64(srv.p.tiff_timeout).await
        };
        let t_start = Instant::now();
        let start_time = epoch_secs(SystemTime::now());

        // Wait for the file to exist and be new (skip the age check when
        // timeout == 0, which C uses for flat-field files).
        let mut file_exists = false;
        let mut opened = false;
        let mut delta_time = 0.0f64;
        let path = Path::new(file);
        while delta_time <= timeout {
            if let Ok(f) = std::fs::File::open(path) {
                if timeout != 0.0 {
                    file_exists = true;
                    let modified = match f.metadata().and_then(|m| m.modified()) {
                        Ok(m) => epoch_secs(m),
                        Err(e) => {
                            log::error!("marccd: error calling fstat on {file}: {e}");
                            return Err(CamStatus::Error);
                        }
                    };
                    // Allow up to 10 seconds of clock skew.
                    if (modified - start_time) as f64 > -10.0 {
                        opened = true;
                        break;
                    }
                } else {
                    opened = true;
                }
            }
            if self.stop.wait_timeout(secs(FILE_READ_DELAY)) {
                return Err(CamStatus::Error);
            }
            delta_time = t_start.elapsed().as_secs_f64();
        }
        if !opened {
            if file_exists {
                log::error!(
                    "marccd: timeout waiting for {file}; file exists but is more than \
                     10 seconds old, possible clock synchronization problem"
                );
            } else {
                log::error!("marccd: timeout waiting for file to be created {file}");
            }
            return Err(CamStatus::Error);
        }

        // The file exists but may not be completely written; retry the decode.
        let mut decoded = None;
        delta_time = 0.0;
        while delta_time <= timeout {
            match decode_tiff(path) {
                Ok(img) if img.width as usize == nx && img.height as usize == ny => {
                    decoded = Some(img.data);
                    break;
                }
                Ok(img) => log::error!(
                    "marccd: image size incorrect = {}x{}, should be {nx}x{ny}",
                    img.width,
                    img.height
                ),
                Err(e) => log::debug!("marccd: error reading TIFF file {file}: {e}"),
            }
            if self.stop.wait_timeout(secs(FILE_READ_DELAY)) {
                return Err(CamStatus::Error);
            }
            delta_time = t_start.elapsed().as_secs_f64();
        }

        let Some(data) = decoded else {
            // Fixed (upstream-c-defects.md #13): the retry loop timed out
            // without a valid frame. Do not fabricate a buffer or return it as
            // success.
            log::error!("marccd: timeout reading TIFF file {file}; image not published");
            return Err(CamStatus::Error);
        };
        Ok(data)
    }
}

// ---------------------------------------------------------------------------
// Command thread
// ---------------------------------------------------------------------------

pub(crate) fn start_cmd_task(
    w: Worker,
    rx: rt::CommandReceiver<Cmd>,
) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("MarccdCmdTask", move || cmd_loop(w, rx))
}

async fn cmd_loop(w: Worker, mut rx: rt::CommandReceiver<Cmd>) {
    while let Some(cmd) = rx.recv().await {
        match cmd {
            Cmd::Init => {
                let mut guard = w.server.lock().await;
                let srv = &mut *guard;
                // C constructor: getServerMode -> getConfig -> getState.
                srv.get_server_mode().await;
                srv.get_config().await;
                srv.get_state().await;
            }
            Cmd::StartAcquire => {
                let s = w.get_state().await;
                if test_task_status(s, TASK_ACQUIRE, TASK_STATUS_QUEUED | TASK_STATUS_EXECUTING)
                    == 0
                {
                    // Kill any stale stop event, then wake the marCCD task.
                    w.stop.try_wait();
                    w.start.signal();
                }
            }
            Cmd::SetBin => {
                let mut guard = w.server.lock().await;
                let srv = &mut *guard;
                let bin_x = srv.get_i32(srv.ad.bin_x).await;
                let bin_y = srv.get_i32(srv.ad.bin_y).await;
                srv.write_server(&cmd_set_bin(bin_x, bin_y)).await;
            }
            Cmd::SetGating(value) => {
                let mut guard = w.server.lock().await;
                let srv = &mut *guard;
                if srv.server_mode == 2 {
                    srv.write_server(&cmd_set_gating(value)).await;
                    srv.get_config().await;
                }
            }
            Cmd::SetReadoutMode(value) => {
                let mut guard = w.server.lock().await;
                let srv = &mut *guard;
                if srv.server_mode == 2 {
                    srv.write_server(&cmd_set_readout_mode(value)).await;
                    srv.get_config().await;
                }
            }
            Cmd::SetFrameShift(value) => {
                let mut guard = w.server.lock().await;
                let srv = &mut *guard;
                srv.write_server(&cmd_set_frameshift(value)).await;
                srv.get_config().await;
            }
            Cmd::ReadStatus => {
                w.get_state().await;
            }
            Cmd::WriteFile => {
                let frame_type = {
                    let mut guard = w.server.lock().await;
                    let srv = &mut *guard;
                    srv.get_i32(srv.ad.frame_type).await
                };
                // Raw frames are saved uncorrected.
                let corrected_flag = if frame_type == FrameType::Raw as i32 {
                    0
                } else {
                    1
                };
                w.save_file(corrected_flag, true).await;
            }
            Cmd::SetStability(value) => {
                let mut guard = w.server.lock().await;
                let srv = &mut *guard;
                srv.write_server(&cmd_set_stability(value)).await;
                srv.get_config().await;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Acquisition thread
// ---------------------------------------------------------------------------

pub(crate) fn start_det_task(w: Worker) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("MarccdTask", move || det_loop(w))
}

/// C `marCCDTask`.
async fn det_loop(w: Worker) {
    loop {
        let acquire = {
            let mut guard = w.server.lock().await;
            let srv = &mut *guard;
            srv.get_i32(srv.ad.acquire).await
        };
        if acquire == 0 {
            {
                let mut guard = w.server.lock().await;
                let srv = &mut *guard;
                srv.set_str(srv.ad.status_message, "Waiting for acquire command");
                srv.callbacks().await;
            }
            w.start.wait();
            let mut guard = w.server.lock().await;
            let srv = &mut *guard;
            srv.set_i32(srv.ad.num_images_counter, 0);
            srv.callbacks().await;
        }

        let image_mode = {
            let mut guard = w.server.lock().await;
            let srv = &mut *guard;
            srv.get_i32(srv.ad.image_mode).await
        };
        match ImageMode::from_i32(image_mode) {
            Some(ImageMode::Single | ImageMode::Multiple | ImageMode::Continuous) => {
                collect_normal(&w).await;
            }
            Some(ImageMode::SeriesTriggered | ImageMode::SeriesTimed) => {
                collect_series(&w).await;
            }
            // C's switch has no default: an out-of-range mode does nothing and
            // the loop re-checks ADAcquire.
            None => {}
        }
    }
}

/// Set `ADAcquire` (and `ADAcquireBusy`) to 0 at acquisition completion. C's
/// literal `setIntegerParam(ADAcquire, 0)` runs through
/// `asynNDArrayDriver::setIntegerParam`, which also clears `ADAcquireBusy` when
/// no plugins are being waited on; this port clears both directly (the
/// `ADWaitForPlugins` interaction is not modelled from the worker side).
async fn set_acquire_off(w: &Worker) {
    let mut guard = w.server.lock().await;
    let srv = &mut *guard;
    srv.set_i32(srv.ad.acquire, 0);
    srv.set_i32(srv.ad.acquire_busy, 0);
}

/// C `collectNormal`.
async fn collect_normal(w: &Worker) {
    let (image_mode, frame_type, acquire_time, auto_save, overlap, shutter_mode, array_callbacks) = {
        let mut guard = w.server.lock().await;
        let srv = &mut *guard;
        let image_mode = srv.get_i32(srv.ad.image_mode).await;
        let frame_type = srv.get_i32(srv.ad.frame_type).await;
        let acquire_time = srv.get_f64(srv.ad.acquire_time).await;
        let auto_save = srv.get_i32(srv.ad.base.auto_save).await;
        let overlap = srv.get_i32(srv.p.overlap).await;
        let shutter_mode = srv.get_i32(srv.ad.shutter_mode).await;
        let array_callbacks = srv.get_i32(srv.ad.base.array_callbacks).await;
        (
            image_mode,
            frame_type,
            acquire_time,
            auto_save,
            overlap,
            shutter_mode,
            array_callbacks,
        )
    };
    let wait = overlap == 0;
    // ADShutterModeNone == 0.
    let use_shutter = shutter_mode != 0;
    if auto_save != 0 {
        w.server.lock().await.write_header().await;
    }

    w.store_acq_start(now_epoch_f64());

    // C's `goto cleanup` on a readout error skips the counter increments and the
    // image-readback dispatch; `completed` tracks whether that jump fired.
    let mut completed = true;
    match FrameType::from_i32(frame_type) {
        Some(FrameType::Normal) | Some(FrameType::Raw) => {
            let mut full_file_name = String::new();
            if auto_save != 0 {
                full_file_name = w.create_file_name().await;
            }
            w.acquire_frame(acquire_time, use_shutter).await;
            let buffer_number = if frame_type == FrameType::Normal as i32 {
                0
            } else {
                3
            };
            let file = if full_file_name.is_empty() {
                None
            } else {
                Some(full_file_name.as_str())
            };
            let status = w.readout_frame(buffer_number, file, wait).await;
            completed = !status.is_err();
        }
        Some(FrameType::Background) => {
            w.acquire_frame(0.001, false).await;
            let status = w.readout_frame(1, None, true).await;
            if status.is_err() {
                completed = false;
            } else {
                w.acquire_frame(0.001, false).await;
                let status = w.readout_frame(2, None, true).await;
                if status.is_err() {
                    completed = false;
                } else {
                    w.server.lock().await.write_server(&cmd_dezinger(1)).await;
                    let mut s = w.get_state().await;
                    while test_task_status(
                        s,
                        TASK_DEZINGER,
                        TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED,
                    ) != 0
                        || task_state(s) >= TASK_STATE_BUSY
                    {
                        rt::sleep(secs(MARCCD_POLL_DELAY)).await;
                        s = w.get_state().await;
                    }
                }
            }
        }
        Some(FrameType::DoubleCorrelation) => {
            w.acquire_frame(acquire_time / 2.0, use_shutter).await;
            let status = w.readout_frame(2, None, true).await;
            if status.is_err() {
                completed = false;
            } else {
                // If the user aborted then ADAcquire will be 0.
                let acquire = {
                    let mut guard = w.server.lock().await;
                    let srv = &mut *guard;
                    srv.get_i32(srv.ad.acquire).await
                };
                if acquire == 0 {
                    completed = false;
                } else {
                    w.acquire_frame(acquire_time / 2.0, use_shutter).await;
                    let status = w.readout_frame(0, None, true).await;
                    if status.is_err() {
                        completed = false;
                    } else {
                        w.server.lock().await.write_server(&cmd_dezinger(0)).await;
                        let mut s = w.get_state().await;
                        while test_task_status(
                            s,
                            TASK_DEZINGER,
                            TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED,
                        ) != 0
                            || task_state(s) >= TASK_STATE_BUSY
                        {
                            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
                            s = w.get_state().await;
                        }
                        w.server.lock().await.write_server("correct").await;
                        let mut s = w.get_state().await;
                        while test_task_status(
                            s,
                            TASK_CORRECT,
                            TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED,
                        ) != 0
                            || task_state(s) >= TASK_STATE_BUSY
                        {
                            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
                            s = w.get_state().await;
                        }
                        if auto_save != 0 {
                            w.save_file(1, true).await;
                        }
                    }
                }
            }
        }
        None => {}
    }

    // C runs the counter increments and the overlap/readback dispatch only when
    // no `goto cleanup` fired.
    if completed {
        {
            let mut guard = w.server.lock().await;
            let srv = &mut *guard;
            let image_counter = srv.get_i32(srv.ad.base.array_counter).await + 1;
            srv.set_i32(srv.ad.base.array_counter, image_counter);
            let num_images_counter = srv.get_i32(srv.ad.num_images_counter).await + 1;
            srv.set_i32(srv.ad.num_images_counter, num_images_counter);
            srv.callbacks().await;
        }

        // If a file was saved and array callbacks are on, read it back.
        if auto_save != 0 && array_callbacks != 0 && frame_type != FrameType::Background as i32 {
            if overlap != 0 {
                w.image_event.signal();
            } else {
                w.get_image_data().await;
            }
        }
    }

    // cleanup
    if image_mode == ImageMode::Multiple as i32 {
        let mut guard = w.server.lock().await;
        let srv = &mut *guard;
        let num_images = srv.get_i32(srv.ad.num_images).await;
        let num_images_counter = srv.get_i32(srv.ad.num_images_counter).await;
        if num_images_counter >= num_images {
            srv.set_i32(srv.ad.acquire, 0);
            srv.set_i32(srv.ad.acquire_busy, 0);
        }
    }
    if image_mode == ImageMode::Single as i32 {
        set_acquire_off(w).await;
    }

    let acquire = {
        let mut guard = w.server.lock().await;
        let srv = &mut *guard;
        srv.get_i32(srv.ad.acquire).await
    };
    if acquire != 0 {
        // Continuous or multiple mode: sleep until the acquire period expires or
        // acquire is set to stop.
        let elapsed = now_epoch_f64() - w.load_acq_start();
        let acquire_period = {
            let mut guard = w.server.lock().await;
            let srv = &mut *guard;
            srv.get_f64(srv.ad.acquire_period).await
        };
        let delay_time = acquire_period - elapsed;
        if delay_time > 0.0 {
            {
                let mut guard = w.server.lock().await;
                let srv = &mut *guard;
                srv.set_i32(srv.ad.status, ADStatus::Waiting as i32);
                srv.callbacks().await;
            }
            w.stop.wait_timeout(secs(delay_time));
        }
    }

    w.server.lock().await.callbacks().await;
}

/// C `collectSeries`.
async fn collect_series(w: &Worker) {
    let params = {
        let mut guard = w.server.lock().await;
        let srv = &mut *guard;
        let frame_type = srv.get_i32(srv.ad.frame_type).await;
        let image_mode = srv.get_i32(srv.ad.image_mode).await;
        let auto_increment = srv.get_i32(srv.ad.base.auto_increment).await;
        let num_images = srv.get_i32(srv.ad.num_images).await;
        let acquire_time = srv.get_f64(srv.ad.acquire_time).await;
        let acquire_period = srv.get_f64(srv.ad.acquire_period).await;
        let tiff_timeout = srv.get_f64(srv.p.tiff_timeout).await;
        let shutter_mode = srv.get_i32(srv.ad.shutter_mode).await;
        let trigger_mode = srv.get_i32(srv.ad.trigger_mode).await;
        let file_path = srv.get_str(srv.ad.base.file_path, MAX_FILENAME_LEN).await;
        let file_name = srv.get_str(srv.ad.base.file_name, MAX_FILENAME_LEN).await;
        let file_number = srv.get_i32(srv.ad.base.file_number).await;
        let series_file_template = srv
            .get_str(srv.p.series_file_template, MAX_FILENAME_LEN)
            .await;
        let series_file_digits = srv.get_i32(srv.p.series_file_digits).await;
        let series_file_first = srv.get_i32(srv.p.series_file_first).await;
        let overlap = srv.get_i32(srv.p.overlap).await;
        (
            frame_type,
            image_mode,
            auto_increment,
            num_images,
            acquire_time,
            acquire_period,
            tiff_timeout,
            shutter_mode,
            trigger_mode,
            file_path,
            file_name,
            file_number,
            series_file_template,
            series_file_digits,
            series_file_first,
            overlap,
        )
    };
    let (
        frame_type,
        image_mode,
        auto_increment,
        num_images,
        acquire_time,
        acquire_period,
        tiff_timeout,
        shutter_mode,
        trigger_mode,
        file_path,
        file_name,
        file_number,
        series_file_template,
        series_file_digits,
        series_file_first,
        _overlap,
    ) = params;

    let use_shutter = shutter_mode != 0;

    if frame_type != FrameType::Normal as i32 {
        log::error!("marccd: collectSeries error, frame type must be Normal");
        finish_series(w, tiff_timeout, false, auto_increment, file_number).await;
        return;
    }

    // Build the base file name from the user template.
    let base_file_name =
        match format_file_template(&series_file_template, &file_path, &file_name, file_number) {
            Some(s) => s,
            None => {
                log::error!("marccd: collectSeries error creating base file name");
                // C returns here without restoring state, which spins the task;
                // this port cleans up and stops instead.
                finish_series(w, tiff_timeout, use_shutter, auto_increment, file_number).await;
                return;
            }
        };
    let full_file_template = full_file_template(series_file_digits);

    w.server.lock().await.write_header().await;
    w.store_acq_start(now_epoch_f64());

    if use_shutter {
        w.server.lock().await.set_shutter(true).await;
    }

    let file_suffix = ".tif";
    match ImageMode::from_i32(image_mode) {
        Some(ImageMode::SeriesTriggered) => {
            let cmd = match TriggerMode::from_i32(trigger_mode) {
                Some(TriggerMode::Timed) => cmd_start_series_triggered_timed(
                    acquire_time,
                    num_images,
                    series_file_first,
                    &base_file_name,
                    file_suffix,
                    series_file_digits,
                ),
                mode => {
                    // itemp: 0 for internal/frame, 1 for bulb, 0 otherwise.
                    let itemp = if mode == Some(TriggerMode::Bulb) {
                        1
                    } else {
                        0
                    };
                    cmd_start_series_triggered_itemp(
                        itemp,
                        num_images,
                        series_file_first,
                        &base_file_name,
                        file_suffix,
                        series_file_digits,
                    )
                }
            };
            w.server.lock().await.write_server(&cmd).await;
        }
        Some(ImageMode::SeriesTimed) => {
            let cmd = cmd_start_series_timed(
                num_images,
                series_file_first,
                acquire_time,
                acquire_period,
                &base_file_name,
                file_suffix,
                series_file_digits,
            );
            w.server.lock().await.write_server(&cmd).await;
        }
        _ => {}
    }

    // Bump the TIFF timeout while waiting for the series files to appear.
    {
        let mut guard = w.server.lock().await;
        let srv = &mut *guard;
        srv.set_f64(srv.p.tiff_timeout, tiff_timeout + acquire_period);
        srv.callbacks().await;
    }

    for i in 0..num_images {
        let full_file_name = format_file_template(
            &full_file_template,
            &base_file_name,
            "",
            i + series_file_first,
        )
        .unwrap_or_default();
        {
            let mut guard = w.server.lock().await;
            let srv = &mut *guard;
            srv.set_str(srv.ad.base.full_file_name, full_file_name);
            srv.callbacks().await;
        }
        let status = w.get_image_data().await;
        if status.is_err() {
            w.server.lock().await.write_server("abort").await;
            break;
        }
        let mut guard = w.server.lock().await;
        let srv = &mut *guard;
        let image_counter = srv.get_i32(srv.ad.base.array_counter).await + 1;
        srv.set_i32(srv.ad.base.array_counter, image_counter);
        let num_images_counter = srv.get_i32(srv.ad.num_images_counter).await + 1;
        srv.set_i32(srv.ad.num_images_counter, num_images_counter);
        srv.callbacks().await;
    }

    finish_series(w, tiff_timeout, use_shutter, auto_increment, file_number).await;
}

/// C `collectSeries` `done:` block.
async fn finish_series(
    w: &Worker,
    tiff_timeout: f64,
    use_shutter: bool,
    auto_increment: i32,
    file_number: i32,
) {
    {
        let mut guard = w.server.lock().await;
        let srv = &mut *guard;
        srv.set_f64(srv.p.tiff_timeout, tiff_timeout);
        srv.callbacks().await;
    }
    if use_shutter {
        w.server.lock().await.set_shutter(false).await;
    }
    if auto_increment != 0 {
        let mut guard = w.server.lock().await;
        let srv = &mut *guard;
        srv.set_i32(srv.ad.base.file_number, file_number + 1);
    }
    {
        let mut guard = w.server.lock().await;
        let srv = &mut *guard;
        srv.set_i32(srv.ad.acquire, 0);
        srv.set_i32(srv.ad.acquire_busy, 0);
        srv.callbacks().await;
    }
}

// ---------------------------------------------------------------------------
// Image thread
// ---------------------------------------------------------------------------

pub(crate) fn start_image_task(w: Worker) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("MarccdImageTask", move || image_loop(w))
}

/// C `getImageDataTask`.
async fn image_loop(w: Worker) {
    loop {
        w.image_event.wait();

        // Wait for the correction to complete.
        let mut s = w.get_state().await;
        while test_task_status(s, TASK_CORRECT, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0 {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = w.get_state().await;
        }

        // Wait for the write to complete.
        let mut s = w.get_state().await;
        while test_task_status(s, TASK_WRITE, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
            || task_state(s) >= TASK_STATE_BUSY
        {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = w.get_state().await;
        }

        w.get_image_data().await;
    }
}
