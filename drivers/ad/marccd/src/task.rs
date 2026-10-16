//! The two marCCD worker threads and the acquisition state machine.
//!
//! * `MarccdTask` is C's `marCCDTask` acquisition state machine
//!   (`collectNormal` / `collectSeries`).
//! * `MarccdImageTask` is C's `getImageDataTask` — the overlapped
//!   correction/readback path.
//!
//! Each thread holds the driver lock ([`Held`]) over exactly C's
//! spans, releasing it only where C calls `unlock()`: the start and image
//! events, the exposure and acquire-period waits, `readTiff`'s waits and
//! the image task's poll sleeps.

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
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::port_handle::{DriverHold, PortHandle};
use epics_rs::asyn::trace::TraceMask;

use crate::driver::MarccdDriver;
use crate::file_name::{format_file_template, full_file_template};
use crate::image::decode_tiff;
use crate::protocol::{
    cmd_start_series_timed, cmd_start_series_triggered_itemp, cmd_start_series_triggered_timed,
};
use crate::types::{
    CamStatus, Event, FILE_READ_DELAY, FrameType, ImageMode, MARCCD_POLL_DELAY, MAX_FILENAME_LEN,
    TASK_ACQUIRE, TASK_CORRECT, TASK_DEZINGER, TASK_STATE_BUSY, TASK_STATUS_EXECUTING,
    TASK_STATUS_QUEUED, TASK_WRITE, TriggerMode, secs, task_state, test_task_status,
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

/// The driver lock a worker thread holds — C `lock()` … `unlock()`.
pub(crate) type Held = DriverHold<MarccdDriver>;

/// [`DriverHold::run`] for a thread that carries on past a stopped port:
/// the step yields `R::default()`.
pub(crate) trait Step {
    fn step<R: Default + Send + 'static>(
        &mut self,
        f: impl FnOnce(&mut MarccdDriver) -> R + Send + 'static,
    ) -> R;
}

impl Step for Held {
    fn step<R: Default + Send + 'static>(
        &mut self,
        f: impl FnOnce(&mut MarccdDriver) -> R + Send + 'static,
    ) -> R {
        self.run(f).unwrap_or_default()
    }
}

impl Worker {
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

    /// C `acquireFrame`: the lock is released only while waiting out the
    /// exposure.
    fn acquire_frame(&self, l: &mut Held, exposure_time: f64, use_shutter: bool) {
        let trigger_mode = l.step(|d| {
            let trigger_mode = d.get_i32(d.ad.params.trigger_mode);
            // Wait for the acquire task to be done with the previous acquisition.
            d.wait_state(|s| {
                test_task_status(s, TASK_ACQUIRE, TASK_STATUS_EXECUTING) != 0
                    || task_state(s) >= TASK_STATE_BUSY
            });
            d.set_str(d.ad.params.status_message, "Starting exposure");
            d.write_server("start");
            d.callbacks();
            // Wait for acquisition to actually start.
            d.wait_state(|s| {
                test_task_status(s, TASK_ACQUIRE, TASK_STATUS_EXECUTING) == 0
                    || task_state(s) >= TASK_STATE_BUSY
            });
            trigger_mode
        });

        let start = Instant::now();
        if use_shutter {
            l.step(|d| {
                ADDriver::set_shutter(d, true);
            });
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
            if l.unlocked(|| self.stop.wait_timeout(wait)) {
                break;
            }
            let time_remaining = (exposure_time - start.elapsed().as_secs_f64()).max(0.0);
            l.step(move |d| {
                d.set_f64(d.ad.params.time_remaining, time_remaining);
                d.callbacks();
            });
        }

        l.step(move |d| {
            d.set_f64(d.ad.params.time_remaining, 0.0);
            d.callbacks();
            if use_shutter {
                ADDriver::set_shutter(d, false);
            }
        });
    }

    /// C `getImageData`, entered and left with the lock held.
    async fn get_image_data(&self, l: &mut Held) -> CamStatus {
        // Inquire about the image dimensions.
        let (full_file_name, nx, ny, image_counter, array_callbacks) = l.step(|d| {
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
        });

        let data = match self.read_tiff(l, &full_file_name, nx, ny) {
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
            // C `getAttributes`, evaluated where its PARAM values live.
            let owner = self.owner.clone();
            let fresh = l.step(move |d| {
                let mut attributes = owner.attributes().lock();
                (!attributes.list.is_empty()).then(|| attributes.update(d.base()))
            });
            if let Some(fresh) = fresh {
                array.attributes.copy_from(&fresh);
            }
            self.output.publish(array).await;
        }
        CamStatus::Success
    }

    /// C `readTiff` — wait for a new file, then retry the decode until the
    /// writer has finished; the lock is released only for the waits.
    ///
    /// Fixed (upstream-c-defects.md #13): when the retry loop exhausts `timeout`
    /// without a successful decode, C falls through and returns `asynSuccess`
    /// with a buffer it never filled — the caller then publishes stale pixels.
    /// This returns [`CamStatus::Error`] instead so no image is published.
    fn read_tiff(
        &self,
        l: &mut Held,
        file: &str,
        nx: usize,
        ny: usize,
    ) -> Result<Vec<u16>, CamStatus> {
        let timeout = l.step(|d| d.get_f64(d.p.tiff_timeout));
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
            if l.unlocked(|| self.stop.wait_timeout(secs(FILE_READ_DELAY))) {
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
            if l.unlocked(|| self.stop.wait_timeout(secs(FILE_READ_DELAY))) {
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

/// C `marCCDTask`: it holds the lock except while waiting for a start.
async fn det_loop(w: Worker) {
    let mut l = match w.port.hold_driver_blocking::<MarccdDriver>() {
        Ok(l) => l,
        Err(e) => {
            w.trace(
                TraceMask::ERROR,
                &format!("marCCD:marCCDTask: no driver lock: {e}"),
            );
            return;
        }
    };
    loop {
        let acquire = l.step(|d| d.get_i32(d.ad.params.acquire));
        if acquire == 0 {
            l.step(|d| {
                d.set_str(d.ad.params.status_message, "Waiting for acquire command");
                d.callbacks();
            });
            l.unlocked(|| w.start.wait());
            l.step(|d| {
                d.set_i32(d.ad.params.num_images_counter, 0);
                d.callbacks();
            });
        }

        let image_mode = l.step(|d| d.get_i32(d.ad.params.image_mode));
        match ImageMode::from_i32(image_mode) {
            Some(ImageMode::Single | ImageMode::Multiple | ImageMode::Continuous) => {
                collect_normal(&w, &mut l).await;
            }
            Some(ImageMode::SeriesTriggered | ImageMode::SeriesTimed) => {
                collect_series(&w, &mut l).await;
            }
            // C's switch has no default: an out-of-range mode does nothing and
            // the loop re-checks ADAcquire.
            None => {}
        }
    }
}

/// `writeServer(command)` then poll until `task` is neither queued nor
/// executing and marServer is not busy.
fn run_server_task(l: &mut Held, command: &'static str, task: i32) {
    l.step(move |d| {
        d.write_server(command);
        d.wait_state(|s| {
            test_task_status(s, task, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
                || task_state(s) >= TASK_STATE_BUSY
        });
    });
}

/// C `collectNormal`.
async fn collect_normal(w: &Worker, l: &mut Held) {
    let (image_mode, frame_type, acquire_time, auto_save, overlap, shutter_mode, array_callbacks) =
        l.step(|d| {
            let ad = d.ad.params;
            let values = (
                d.get_i32(ad.image_mode),
                d.get_i32(ad.frame_type),
                d.get_f64(ad.acquire_time),
                d.get_i32(ad.base.auto_save),
                d.get_i32(d.p.overlap),
                d.get_i32(ad.shutter_mode),
                d.get_i32(ad.base.array_callbacks),
            );
            if values.3 != 0 {
                d.write_header();
            }
            values
        });
    let wait = overlap == 0;
    // ADShutterModeNone == 0.
    let use_shutter = shutter_mode != 0;

    w.store_acq_start(now_epoch_f64());

    // C's `goto cleanup` on a readout error skips the counter increments and the
    // image-readback dispatch; `completed` tracks whether that jump fired.
    let mut completed = true;
    match FrameType::from_i32(frame_type) {
        Some(FrameType::Normal) | Some(FrameType::Raw) => {
            let full_file_name = if auto_save != 0 {
                l.step(|d| d.create_file_name())
            } else {
                String::new()
            };
            w.acquire_frame(l, acquire_time, use_shutter);
            let buffer_number = if frame_type == FrameType::Normal as i32 {
                0
            } else {
                3
            };
            completed = l.step(move |d| {
                !d.readout_frame(buffer_number, Some(&full_file_name), wait)
                    .is_err()
            });
        }
        Some(FrameType::Background) => {
            w.acquire_frame(l, 0.001, false);
            completed = l.step(|d| !d.readout_frame(1, None, true).is_err());
            if completed {
                w.acquire_frame(l, 0.001, false);
                completed = l.step(|d| !d.readout_frame(2, None, true).is_err());
            }
            if completed {
                run_server_task(l, "dezinger,1", TASK_DEZINGER);
            }
        }
        Some(FrameType::DoubleCorrelation) => {
            w.acquire_frame(l, acquire_time / 2.0, use_shutter);
            completed = l.step(|d| !d.readout_frame(2, None, true).is_err());
            // If the user aborted then ADAcquire will be 0.
            if completed && l.step(|d| d.get_i32(d.ad.params.acquire)) == 0 {
                completed = false;
            }
            if completed {
                w.acquire_frame(l, acquire_time / 2.0, use_shutter);
                completed = l.step(|d| !d.readout_frame(0, None, true).is_err());
            }
            if completed {
                run_server_task(l, "dezinger,0", TASK_DEZINGER);
                run_server_task(l, "correct", TASK_CORRECT);
                if auto_save != 0 {
                    l.step(|d| d.save_file(1, true));
                }
            }
        }
        None => {}
    }

    // C runs the counter increments and the overlap/readback dispatch only when
    // no `goto cleanup` fired.
    if completed {
        l.step(|d| {
            let ad = d.ad.params;
            let image_counter = d.get_i32(ad.base.array_counter) + 1;
            d.set_i32(ad.base.array_counter, image_counter);
            let num_images_counter = d.get_i32(ad.num_images_counter) + 1;
            d.set_i32(ad.num_images_counter, num_images_counter);
            d.callbacks();
        });

        // If a file was saved and array callbacks are on, read it back.
        if auto_save != 0 && array_callbacks != 0 && frame_type != FrameType::Background as i32 {
            if overlap != 0 {
                w.image_event.signal();
            } else {
                w.get_image_data(l).await;
            }
        }
    }

    // cleanup
    let (acquire, acquire_period) = l.step(move |d| {
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
    });
    if acquire != 0 {
        // Continuous or multiple mode: sleep until the acquire period expires or
        // acquire is set to stop.
        let elapsed = now_epoch_f64() - w.load_acq_start();
        let delay_time = acquire_period - elapsed;
        if delay_time > 0.0 {
            l.step(|d| {
                d.set_i32(d.ad.params.status, ADStatus::Waiting as i32);
                d.callbacks();
            });
            l.unlocked(|| w.stop.wait_timeout(secs(delay_time)));
        }
    }

    l.step(|d| d.callbacks());
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
async fn collect_series(w: &Worker, l: &mut Held) {
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
    } = l.step(|d| {
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
    });

    let use_shutter = shutter_mode != 0;

    if frame_type != FrameType::Normal as i32 {
        w.trace(
            TraceMask::ERROR,
            "marCCD:collectSeries: error, frame type must be Normal",
        );
        finish_series(l, tiff_timeout, use_shutter, auto_increment, file_number);
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
                finish_series(l, tiff_timeout, use_shutter, auto_increment, file_number);
                return;
            }
        };
    let full_file_template = full_file_template(series_file_digits);

    l.step(|d| {
        d.write_header();
    });
    w.store_acq_start(now_epoch_f64());

    if use_shutter {
        l.step(|d| {
            ADDriver::set_shutter(d, true);
        });
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
            l.step(move |d| {
                d.write_server(&cmd);
            });
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
            l.step(move |d| {
                d.write_server(&cmd);
            });
        }
        _ => {}
    }

    // Bump the TIFF timeout while waiting for the series files to appear.
    l.step(move |d| {
        d.set_f64(d.p.tiff_timeout, tiff_timeout + acquire_period);
        d.callbacks();
    });

    for i in 0..num_images {
        let full_file_name = format_file_template(
            &full_file_template,
            &base_file_name,
            "",
            i + series_file_first,
        )
        .unwrap_or_default();
        l.step(move |d| {
            d.set_str(d.ad.params.base.full_file_name, &full_file_name);
            d.callbacks();
        });
        let status = w.get_image_data(l).await;
        if status.is_err() {
            l.step(|d| {
                d.write_server("abort");
            });
            break;
        }
        l.step(|d| {
            let ad = d.ad.params;
            let image_counter = d.get_i32(ad.base.array_counter) + 1;
            d.set_i32(ad.base.array_counter, image_counter);
            let num_images_counter = d.get_i32(ad.num_images_counter) + 1;
            d.set_i32(ad.num_images_counter, num_images_counter);
            d.callbacks();
        });
    }

    finish_series(l, tiff_timeout, use_shutter, auto_increment, file_number);
}

/// C `collectSeries` `done:` block.
fn finish_series(
    l: &mut Held,
    tiff_timeout: f64,
    use_shutter: bool,
    auto_increment: i32,
    file_number: i32,
) {
    l.step(move |d| {
        d.set_f64(d.p.tiff_timeout, tiff_timeout);
        d.callbacks();
        if use_shutter {
            ADDriver::set_shutter(d, false);
        }
        let ad = d.ad.params;
        if auto_increment != 0 {
            d.set_i32(ad.base.file_number, file_number + 1);
        }
        d.set_i32(ad.acquire, 0);
        d.callbacks();
    });
}

// ---------------------------------------------------------------------------
// Image thread
// ---------------------------------------------------------------------------

pub(crate) fn start_image_task(w: Worker) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("MarccdImageTask", move || image_loop(w))
}

/// C `getImageDataTask`: it releases the lock for the image event and each
/// poll sleep.
async fn image_loop(w: Worker) {
    let mut l = match w.port.hold_driver_blocking::<MarccdDriver>() {
        Ok(l) => l,
        Err(e) => {
            w.trace(
                TraceMask::ERROR,
                &format!("marCCD:getImageDataTask: no driver lock: {e}"),
            );
            return;
        }
    };
    loop {
        l.unlocked(|| w.image_event.wait());

        // Wait for the correction to complete.
        let mut s = l.step(|d| d.get_state());
        while test_task_status(s, TASK_CORRECT, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0 {
            l.unlocked(|| std::thread::sleep(secs(MARCCD_POLL_DELAY)));
            s = l.step(|d| d.get_state());
        }

        // Wait for the write to complete.
        let mut s = l.step(|d| d.get_state());
        while test_task_status(s, TASK_WRITE, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) != 0
            || task_state(s) >= TASK_STATE_BUSY
        {
            l.unlocked(|| std::thread::sleep(secs(MARCCD_POLL_DELAY)));
            s = l.step(|d| d.get_state());
        }

        w.get_image_data(&mut l).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::{create_marccd_detector, tests::fake_server};
    use epics_rs::ad_core::attributes::NDAttrValue;
    use epics_rs::ad_core::plugin::channel::ndarray_channel;
    use epics_rs::asyn::registry::claim_port;

    /// A worker whose hold fails says so on the port trace instead of
    /// exiting silently.
    #[test]
    fn a_task_without_the_driver_lock_traces_and_exits() {
        use epics_rs::asyn::trace::TraceFile;

        let name = "MARCCD_NOLOCK";
        fake_server(name, |c| if c.starts_with("get_") { "0" } else { "" });
        let det =
            create_marccd_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        let h = det.port_handle().clone();
        // The marServer port: its driver is no MarccdDriver, so no hold.
        let ip = epics_rs::asyn::asyn_record::get_port(&format!("{name}_IP"))
            .unwrap()
            .handle
            .clone();
        let path = std::env::temp_dir().join(format!("{name}_{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        ip.trace().set_trace_file(
            Some(ip.port_name()),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        let worker = || Worker {
            port: ip.clone(),
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
        start_det_task(worker()).join().unwrap();
        start_image_task(worker()).join().unwrap();
        ip.trace()
            .set_trace_file(Some(ip.port_name()), TraceFile::default());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            text.contains("marCCD:marCCDTask: no driver lock"),
            "{text:?}"
        );
        assert!(
            text.contains("marCCD:getImageDataTask: no driver lock"),
            "{text:?}"
        );
        std::mem::forget(det);
    }

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
            let mut l = worker.port.hold_driver_blocking().unwrap();
            assert_eq!(worker.get_image_data(&mut l).await, CamStatus::Success);
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

    /// marCCD.cpp:1131-1140 computes useShutter before the frame-type check,
    /// and its `goto done` closes the shutter (:1235).
    #[test]
    fn a_series_refused_for_its_frame_type_closes_the_shutter() {
        use crate::driver::tests::sent_count;
        use epics_rs::ad_core::driver::ShutterMode;
        use std::time::Duration;

        let name = "MARCCD_SERIES_FRAME";
        let log = fake_server(name, |c| {
            if c.starts_with("get_mode") {
                "1"
            } else if c.starts_with("get_size") {
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
        // An acquire the task reads before it first waits leaves the start
        // event set behind it, and a second series runs.
        let deadline = Instant::now() + Duration::from_secs(10);
        while h
            .get_string_param_blocking(ad.status_message, 0, 256)
            .unwrap()
            != b"Waiting for acquire command"
        {
            assert!(Instant::now() < deadline, "the task never waited");
            std::thread::sleep(Duration::from_millis(10));
        }
        h.write_int32_blocking(ad.image_mode, 0, ImageMode::SeriesTimed as i32)
            .unwrap();
        h.write_int32_blocking(ad.frame_type, 0, FrameType::Raw as i32)
            .unwrap();
        h.write_int32_blocking(ad.shutter_mode, 0, ShutterMode::DetectorOnly as i32)
            .unwrap();
        h.write_int32_blocking(ad.acquire, 0, 1).unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        while h.read_int32_blocking(ad.acquire, 0).unwrap() != 0 {
            assert!(Instant::now() < deadline, "the series never finished");
            std::thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(sent_count(&log, "shutter,0"), 1);
        std::mem::forget(det);
    }
}
