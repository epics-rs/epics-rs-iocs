//! The two marCCD worker threads and the acquisition state machine.
//!
//! * `MarccdTask` is C's `marCCDTask` acquisition state machine
//!   (`collectNormal` / `collectSeries`).
//! * `MarccdImageTask` is C's `getImageDataTask` — the overlapped
//!   correction/readback path.
//!
//! Where C's tasks hold the driver lock, these run their step on the driver
//! through [`Worker::locked`], so marServer I/O always happens on the port
//! actor, exactly once per C call.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use epics_rs::ad_core::driver::ndarray_driver::ArrayOwner;
use epics_rs::ad_core::driver::{ADDriver, ADStatus};
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDimension};
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;

use crate::driver::MarccdDriver;
use crate::file_name::{format_file_template, full_file_template};
use crate::image::decode_tiff;
use crate::protocol::{
    cmd_dezinger, cmd_readout, cmd_start_series_timed, cmd_start_series_triggered_itemp,
    cmd_start_series_triggered_timed,
};
use crate::types::{
    CamStatus, Event, FILE_READ_DELAY, FrameType, ImageMode, MARCCD_POLL_DELAY, MAX_FILENAME_LEN,
    TASK_ACQUIRE, TASK_CORRECT, TASK_DEZINGER, TASK_READ, TASK_STATE_BUSY, TASK_STATE_ERROR,
    TASK_STATUS_EXECUTING, TASK_STATUS_QUEUED, TASK_WRITE, TriggerMode, secs, task_state,
    test_task_status,
};

/// Everything a worker thread needs to reach the driver.
pub(crate) struct Worker {
    /// The marCCD port, whose actor is the driver lock.
    pub port: PortHandle,
    pub start: Arc<Event>,
    pub stop: Arc<Event>,
    pub image_event: Arc<Event>,
    /// C `acqStartTime` as an f64 epoch second count (bits).
    pub acq_start: Arc<AtomicU64>,
    pub output: ArrayPublisher,
    pub pool: DriverPool,
    /// The driver's attribute set, for C `getAttributes`.
    pub owner: Arc<ArrayOwner>,
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
    /// C `lock()` … `unlock()` around one step: run `f` on the driver, on its
    /// port actor. A stopped port yields `R::default()`.
    pub(crate) async fn locked<R: Default + Send + 'static>(
        &self,
        f: impl FnOnce(&mut MarccdDriver) -> R + Send + 'static,
    ) -> R {
        self.port
            .with_driver::<MarccdDriver, R>(f)
            .await
            .unwrap_or_default()
    }

    /// C `asynPrint(pasynUserSelf, mask, ...)`.
    fn trace(&self, mask: TraceMask, message: &str) {
        self.port
            .trace()
            .output(self.port.port_name(), mask, message);
    }

    fn store_acq_start(&self, v: f64) {
        self.acq_start.store(v.to_bits(), Ordering::Relaxed);
    }

    fn load_acq_start(&self) -> f64 {
        f64::from_bits(self.acq_start.load(Ordering::Relaxed))
    }

    /// C `getState`.
    async fn get_state(&self) -> i32 {
        self.locked(|d| d.get_state()).await
    }

    // -----------------------------------------------------------------------
    // acquireFrame / readoutFrame
    // -----------------------------------------------------------------------

    /// C `acquireFrame`.
    async fn acquire_frame(&self, exposure_time: f64, use_shutter: bool) {
        let trigger_mode = self.locked(|d| d.get_i32(d.ad.params.trigger_mode)).await;

        // Wait for the acquire task to be done with the previous acquisition.
        let mut s = self.get_state().await;
        while test_task_status(s, TASK_ACQUIRE, TASK_STATUS_EXECUTING) != 0
            || task_state(s) >= TASK_STATE_BUSY
        {
            rt::sleep(secs(MARCCD_POLL_DELAY)).await;
            s = self.get_state().await;
        }

        self.locked(|d| {
            d.set_str(d.ad.params.status_message, "Starting exposure");
            d.write_server("start");
            d.callbacks();
        })
        .await;

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
            self.set_shutter(true).await;
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
            self.locked(move |d| {
                d.set_f64(d.ad.params.time_remaining, time_remaining);
                d.callbacks();
            })
            .await;
        }

        self.locked(|d| {
            d.set_f64(d.ad.params.time_remaining, 0.0);
            d.callbacks();
        })
        .await;
        if use_shutter {
            self.set_shutter(false).await;
        }
    }

    /// C `setShutter(open)` from the acquisition task.
    async fn set_shutter(&self, open: bool) {
        self.locked(move |d| {
            let _ = ADDriver::set_shutter(d, open);
        })
        .await;
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

        let name = file_name.unwrap_or_default().to_string();
        let command = cmd_readout(buffer_number, file_name);
        self.locked(move |d| {
            if has_file {
                d.set_str(d.ad.params.base.full_file_name, &name);
                d.callbacks();
            }
            d.write_server(&command);
        })
        .await;

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
        self.locked(move |d| d.save_file(corrected_flag, wait))
            .await;
    }

    /// C `asynNDArrayDriver::createFileName`.
    async fn create_file_name(&self) -> String {
        self.locked(|d| d.create_file_name()).await
    }

    // -----------------------------------------------------------------------
    // Image readback
    // -----------------------------------------------------------------------

    /// C `getImageData`.
    async fn get_image_data(&self) -> CamStatus {
        // Inquire about the image dimensions.
        let (full_file_name, nx, ny, image_counter, array_callbacks) = self
            .locked(|d| {
                d.get_config();
                let base = d.ad.params.base;
                let full = d.get_str(base.full_file_name, MAX_FILENAME_LEN);
                let nx = d.get_i32(base.array_size_x);
                let ny = d.get_i32(base.array_size_y);
                let ic = d.get_i32(base.array_counter);
                let ac = d.get_i32(base.array_callbacks);
                let message = format!("Reading TIFF file {full}");
                d.set_str(d.ad.params.status_message, &message);
                d.callbacks();
                (full, nx.max(0) as usize, ny.max(0) as usize, ic, ac)
            })
            .await;

        let data = match self.read_tiff(&full_file_name, nx, ny).await {
            Ok(d) => d,
            // Fixed (upstream-c-defects.md #14): on a read error C still runs
            // doCallbacksGenericPointer with the unfilled buffer, publishing
            // garbage as a frame. Propagate the error and do not publish.
            Err(s) => return s,
        };

        // C dereferences a refused alloc (marCCD.cpp:314); the frame is
        // dropped after the pool's reached-limit line.
        if array_callbacks != 0
            && let Ok(mut array) = self.pool.wrap(
                vec![NDDimension::new(nx), NDDimension::new(ny)],
                NDDataBuffer::U16(data),
            )
        {
            array.unique_id = image_counter;
            array.timestamp = EpicsTimestamp::now();
            // C uses `acqStartTime` for the double timestamp, and
            // `updateTimeStamp` (current time) for the epicsTS.
            array.time_stamp = self.load_acq_start();
            let _ = self.owner.get_attributes(&mut array).await;
            self.output.publish(array).await;
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
        let timeout = self.locked(|d| d.get_f64(d.p.tiff_timeout)).await;
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
                            self.trace(
                                TraceMask::ERROR,
                                &format!(
                                    "marCCD::readTiff error calling fstat, errno={} {file}",
                                    e.raw_os_error().unwrap_or(0)
                                ),
                            );
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
            self.trace(
                TraceMask::ERROR,
                &format!("marCCD::readTiff timeout waiting for file to be created {file}"),
            );
            if file_exists {
                self.trace(
                    TraceMask::ERROR,
                    "  file exists but is more than 10 seconds old, possible clock \
                     synchronization problem",
                );
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
                Ok(img) if img.width as usize != nx => {
                    self.trace(
                        TraceMask::ERROR,
                        &format!(
                            "marCCD::readTiff, image width incorrect ={}, should be {nx}",
                            img.width
                        ),
                    );
                }
                Ok(img) => {
                    self.trace(
                        TraceMask::ERROR,
                        &format!(
                            "marCCD::readTiff, image length incorrect ={}, should be {ny}",
                            img.height
                        ),
                    );
                }
                Err(_) => {
                    self.trace(
                        TraceMask::FLOW,
                        &format!("marCCD::readTiff, error reading TIFF file {file}"),
                    );
                }
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
            return Err(CamStatus::Error);
        };
        Ok(data)
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
        let acquire = w.locked(|d| d.get_i32(d.ad.params.acquire)).await;
        if acquire == 0 {
            w.locked(|d| {
                d.set_str(d.ad.params.status_message, "Waiting for acquire command");
                d.callbacks();
            })
            .await;
            w.start.wait();
            w.locked(|d| {
                d.set_i32(d.ad.params.num_images_counter, 0);
                d.callbacks();
            })
            .await;
        }

        let image_mode = w.locked(|d| d.get_i32(d.ad.params.image_mode)).await;
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

/// C `collectNormal`.
async fn collect_normal(w: &Worker) {
    let (image_mode, frame_type, acquire_time, auto_save, overlap, shutter_mode, array_callbacks) =
        w.locked(|d| {
            let ad = d.ad.params;
            (
                d.get_i32(ad.image_mode),
                d.get_i32(ad.frame_type),
                d.get_f64(ad.acquire_time),
                d.get_i32(ad.base.auto_save),
                d.get_i32(d.p.overlap),
                d.get_i32(ad.shutter_mode),
                d.get_i32(ad.base.array_callbacks),
            )
        })
        .await;
    let wait = overlap == 0;
    // ADShutterModeNone == 0.
    let use_shutter = shutter_mode != 0;
    if auto_save != 0 {
        w.locked(|d| {
            d.write_header();
        })
        .await;
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
                    write_server(w, cmd_dezinger(1)).await;
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
                let acquire = w.locked(|d| d.get_i32(d.ad.params.acquire)).await;
                if acquire == 0 {
                    completed = false;
                } else {
                    w.acquire_frame(acquire_time / 2.0, use_shutter).await;
                    let status = w.readout_frame(0, None, true).await;
                    if status.is_err() {
                        completed = false;
                    } else {
                        write_server(w, cmd_dezinger(0)).await;
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
                        write_server(w, "correct".to_string()).await;
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
        w.locked(|d| {
            let ad = d.ad.params;
            let image_counter = d.get_i32(ad.base.array_counter) + 1;
            d.set_i32(ad.base.array_counter, image_counter);
            let num_images_counter = d.get_i32(ad.num_images_counter) + 1;
            d.set_i32(ad.num_images_counter, num_images_counter);
            d.callbacks();
        })
        .await;

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
    let (acquire, acquire_period) = w
        .locked(move |d| {
            let ad = d.ad.params;
            if image_mode == ImageMode::Multiple as i32 {
                let num_images = d.get_i32(ad.num_images);
                let num_images_counter = d.get_i32(ad.num_images_counter);
                if num_images_counter >= num_images {
                    d.set_i32(ad.acquire, 0);
                }
            }
            // C `setIntegerParam(ADAcquire, 0)` at acquisition completion;
            // ADAcquireBusy follows that store, WaitForPlugins included
            // (asynNDArrayDriver.cpp:636-663).
            if image_mode == ImageMode::Single as i32 {
                d.set_i32(ad.acquire, 0);
            }
            (d.get_i32(ad.acquire), d.get_f64(ad.acquire_period))
        })
        .await;
    if acquire != 0 {
        // Continuous or multiple mode: sleep until the acquire period expires or
        // acquire is set to stop.
        let elapsed = now_epoch_f64() - w.load_acq_start();
        let delay_time = acquire_period - elapsed;
        if delay_time > 0.0 {
            w.locked(|d| {
                d.set_i32(d.ad.params.status, ADStatus::Waiting as i32);
                d.callbacks();
            })
            .await;
            w.stop.wait_timeout(secs(delay_time));
        }
    }

    w.locked(|d| d.callbacks()).await;
}

/// C `writeServer` from the acquisition task.
async fn write_server(w: &Worker, command: String) {
    w.locked(move |d| {
        d.write_server(&command);
    })
    .await;
}

/// The parameters C's `collectSeries` reads at its top.
#[derive(Default)]
struct SeriesParams {
    frame_type: i32,
    image_mode: i32,
    auto_increment: i32,
    num_images: i32,
    acquire_time: f64,
    acquire_period: f64,
    tiff_timeout: f64,
    shutter_mode: i32,
    trigger_mode: i32,
    file_path: String,
    file_name: String,
    file_number: i32,
    series_file_template: String,
    series_file_digits: i32,
    series_file_first: i32,
}

/// C `collectSeries`.
async fn collect_series(w: &Worker) {
    let SeriesParams {
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
    } = w
        .locked(|d| {
            let ad = d.ad.params;
            let p = d.p;
            SeriesParams {
                frame_type: d.get_i32(ad.frame_type),
                image_mode: d.get_i32(ad.image_mode),
                auto_increment: d.get_i32(ad.base.auto_increment),
                num_images: d.get_i32(ad.num_images),
                acquire_time: d.get_f64(ad.acquire_time),
                acquire_period: d.get_f64(ad.acquire_period),
                tiff_timeout: d.get_f64(p.tiff_timeout),
                shutter_mode: d.get_i32(ad.shutter_mode),
                trigger_mode: d.get_i32(ad.trigger_mode),
                file_path: d.get_str(ad.base.file_path, MAX_FILENAME_LEN),
                file_name: d.get_str(ad.base.file_name, MAX_FILENAME_LEN),
                file_number: d.get_i32(ad.base.file_number),
                series_file_template: d.get_str(p.series_file_template, MAX_FILENAME_LEN),
                series_file_digits: d.get_i32(p.series_file_digits),
                series_file_first: d.get_i32(p.series_file_first),
            }
        })
        .await;

    let use_shutter = shutter_mode != 0;

    if frame_type != FrameType::Normal as i32 {
        w.trace(
            TraceMask::ERROR,
            "marCCD:collectSeries: error, frame type must be Normal",
        );
        finish_series(w, tiff_timeout, false, auto_increment, file_number).await;
        return;
    }

    // Build the base file name from the user template.
    let base_file_name =
        match format_file_template(&series_file_template, &file_path, &file_name, file_number) {
            Some(s) => s,
            None => {
                w.trace(
                    TraceMask::ERROR,
                    "marCCD:collectSeries: error creating base file name",
                );
                // C returns here without restoring state, which spins the task;
                // this port cleans up and stops instead.
                finish_series(w, tiff_timeout, use_shutter, auto_increment, file_number).await;
                return;
            }
        };
    let full_file_template = full_file_template(series_file_digits);

    w.locked(|d| {
        d.write_header();
    })
    .await;
    w.store_acq_start(now_epoch_f64());

    if use_shutter {
        w.set_shutter(true).await;
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
            write_server(w, cmd).await;
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
            write_server(w, cmd).await;
        }
        _ => {}
    }

    // Bump the TIFF timeout while waiting for the series files to appear.
    w.locked(move |d| {
        d.set_f64(d.p.tiff_timeout, tiff_timeout + acquire_period);
        d.callbacks();
    })
    .await;

    for i in 0..num_images {
        let full_file_name = format_file_template(
            &full_file_template,
            &base_file_name,
            "",
            i + series_file_first,
        )
        .unwrap_or_default();
        w.locked(move |d| {
            d.set_str(d.ad.params.base.full_file_name, &full_file_name);
            d.callbacks();
        })
        .await;
        let status = w.get_image_data().await;
        if status.is_err() {
            write_server(w, "abort".to_string()).await;
            break;
        }
        w.locked(|d| {
            let ad = d.ad.params;
            let image_counter = d.get_i32(ad.base.array_counter) + 1;
            d.set_i32(ad.base.array_counter, image_counter);
            let num_images_counter = d.get_i32(ad.num_images_counter) + 1;
            d.set_i32(ad.num_images_counter, num_images_counter);
            d.callbacks();
        })
        .await;
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
    w.locked(move |d| {
        d.set_f64(d.p.tiff_timeout, tiff_timeout);
        d.callbacks();
    })
    .await;
    if use_shutter {
        w.set_shutter(false).await;
    }
    w.locked(move |d| {
        let ad = d.ad.params;
        if auto_increment != 0 {
            d.set_i32(ad.base.file_number, file_number + 1);
        }
        d.set_i32(ad.acquire, 0);
        d.callbacks();
    })
    .await;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::{create_marccd_detector, tests::fake_server};
    use epics_rs::ad_core::attributes::NDAttrValue;
    use epics_rs::ad_core::plugin::channel::ndarray_channel;
    use epics_rs::asyn::registry::claim_port;

    /// marCCD.cpp:327-328 reads the driver's attributes into every frame.
    #[test]
    fn each_frame_carries_the_driver_attributes() {
        use tiff::encoder::{TiffEncoder, colortype};

        let name = "MARCCD_ATTRS";
        fake_server(name, |c| {
            if c.starts_with("get_size") {
                "4,4"
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
        let file = std::env::temp_dir().join(format!("{name}_{}.tif", std::process::id()));
        TiffEncoder::new(std::fs::File::create(&file).unwrap())
            .unwrap()
            .write_image::<colortype::Gray16>(4, 4, &[7u16; 16])
            .unwrap();
        let path = file.to_string_lossy().into_owned();
        h.write_int32_blocking(ad.base.array_callbacks, 0, 1)
            .unwrap();
        h.write_float64_blocking(ad.acquire_period, 0, 2.5).unwrap();

        let (sender, mut receiver) = ndarray_channel(&format!("{name}_SINK"), 4);
        det.connect_downstream(sender);
        let worker = Worker {
            port: h.clone(),
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            image_event: Arc::new(Event::new()),
            acq_start: Arc::new(AtomicU64::new(0)),
            output: ArrayPublisher::new(det.array_output().clone()),
            pool: det.pool().clone(),
            owner: h
                .with_driver_blocking::<MarccdDriver, _>(|d| d.ad.owner.clone())
                .unwrap(),
        };
        let xml = r#"<Attributes>
            <Attribute name="Period" type="PARAM" source="ACQ_PERIOD" datatype="DOUBLE"/>
        </Attributes>"#;
        rt::run_thread(move || async move {
            let p = &worker.port;
            p.write_octet(ad.base.attributes_file, 0, xml.as_bytes().to_vec())
                .await
                .unwrap();
            p.write_octet(ad.base.full_file_name, 0, path.into_bytes())
                .await
                .unwrap();
            assert_eq!(worker.get_image_data().await, CamStatus::Success);
        })
        .join()
        .unwrap();
        let _ = std::fs::remove_file(&file);

        let frame = receiver.blocking_recv().unwrap();
        assert_eq!(
            frame.attributes.get("Period").unwrap().value,
            NDAttrValue::Float64(2.5)
        );
        std::mem::forget(det);
    }
}
