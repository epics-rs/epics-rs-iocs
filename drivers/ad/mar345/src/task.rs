//! The single `mar345Task` worker thread and the acquisition state machine
//! (`erase` / `changeMode` / `acquireFrame` / `getImageData`).
//!
//! `mar345.cpp` runs all of this on one `mar345Task` thread under the driver
//! lock, woken by the start event and told what to do by the shared `mode`. This
//! port keeps that structure: [`Worker`] owns the [`Server`] and does every
//! socket round-trip and file read; the [`crate::driver::Mar345Driver`] running
//! in the port actor only sets `mode` and signals the events.

use std::sync::Arc;
use std::sync::atomic::{AtomicI32, AtomicU64, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use epics_rs::ad_core::driver::ImageMode;
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::file_name::format_file_template;
use crate::params::Mar345Params;
use crate::pck::get_pck;
use crate::protocol::{
    CMD_ERASE, DONE_MODE_CHANGE, DONE_SCAN_DATA, cmd_change, cmd_scan, full_file_name,
};
use crate::server::Server;
use crate::types::{
    CamStatus, EraseMode, Event, IMAGE_SIZES, MAR345_COMMAND_TIMEOUT, MAR345_POLL_DELAY,
    MAX_FILENAME_LEN, Mode, Status, secs,
};

/// C `imageSizes[res][size]` with C's out-of-bounds index made safe: an
/// out-of-range `res`/`size` (impossible via the template's mbbo records) yields
/// 0 rather than reading past the table.
fn image_size(res: i32, size: i32) -> i32 {
    IMAGE_SIZES
        .get(res.max(0) as usize)
        .and_then(|row| row.get(size.max(0) as usize))
        .copied()
        .unwrap_or(0)
}

/// Everything the `mar345Task` worker needs.
pub struct Worker {
    pub server: Server,
    pub p: Mar345Params,
    pub start: Arc<Event>,
    pub stop: Arc<Event>,
    pub abort: Arc<Event>,
    /// C `mar345Mode_t mode`, shared with the driver (set in `writeInt32`, read
    /// and reset here).
    pub mode: Arc<AtomicI32>,
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

impl Worker {
    fn store_acq_start(&self, v: f64) {
        self.acq_start.store(v.to_bits(), Ordering::Relaxed);
    }

    fn load_acq_start(&self) -> f64 {
        f64::from_bits(self.acq_start.load(Ordering::Relaxed))
    }

    // -----------------------------------------------------------------------
    // Slow tasks
    // -----------------------------------------------------------------------

    /// C `erase`.
    async fn erase(&mut self) -> CamStatus {
        let ad = self.server.ad;
        let p = self.p;

        let mut num_erase = self.server.get_i32(p.num_erase).await;
        if num_erase < 1 {
            num_erase = 1;
        }
        self.server.set_i32(ad.status, Status::Erase as i32);
        self.server.set_i32(p.num_erased, 0);
        self.server.callbacks().await;

        let mut status = CamStatus::Success;
        for i in 0..num_erase {
            if self.abort.try_wait() {
                status = CamStatus::Error;
                break;
            }
            self.server.write_server(CMD_ERASE).await;
            status = self
                .server
                .wait_for_completion(DONE_SCAN_DATA, MAR345_COMMAND_TIMEOUT)
                .await;
            if status.is_err() {
                break;
            }
            self.server.set_i32(p.num_erased, i + 1);
            self.server.callbacks().await;
        }

        self.server.set_i32(ad.status, Status::Idle as i32);
        self.server.set_i32(p.erase, 0);
        self.server.callbacks().await;
        status
    }

    /// C `changeMode`.
    async fn change_mode(&mut self) -> CamStatus {
        let ad = self.server.ad;
        let p = self.p;

        self.server.set_i32(ad.status, Status::ChangeMode as i32);
        self.server.callbacks().await;
        let size = self.server.get_i32(p.size).await;
        let res = self.server.get_i32(p.res).await;
        let size_x = image_size(res, size);
        self.server.set_i32(ad.base.array_size_x, size_x);
        self.server.set_i32(ad.base.array_size_y, size_x);
        self.server.set_i32(
            ad.base.array_size,
            size_x.wrapping_mul(size_x).wrapping_mul(2),
        );

        self.server.write_server(&cmd_change(size_x)).await;
        let status = self
            .server
            .wait_for_completion(DONE_MODE_CHANGE, MAR345_COMMAND_TIMEOUT)
            .await;

        self.server.set_i32(ad.status, Status::Idle as i32);
        self.server.set_i32(p.change_mode, 0);
        self.server.callbacks().await;
        status
    }

    /// C `asynNDArrayDriver::createFileName`.
    async fn create_file_name(&mut self) -> String {
        let base = self.server.ad.base;
        let file_path = self.server.get_str(base.file_path, MAX_FILENAME_LEN).await;
        let file_name = self.server.get_str(base.file_name, MAX_FILENAME_LEN).await;
        let template = self
            .server
            .get_str(base.file_template, MAX_FILENAME_LEN)
            .await;
        let file_number = self.server.get_i32(base.file_number).await;
        let auto_increment = self.server.get_i32(base.auto_increment).await;

        let full = format_file_template(&template, &file_path, &file_name, file_number)
            .unwrap_or_else(|| {
                log::error!("mar345: cannot expand NDFileTemplate {template:?}");
                String::new()
            });
        if auto_increment != 0 {
            self.server.set_i32(base.file_number, file_number + 1);
        }
        full
    }

    /// C `acquireFrame`. Returns [`CamStatus::Error`] on an abort or a failed
    /// erase / scan (C's `return asynError` paths).
    async fn acquire_frame(&mut self) -> CamStatus {
        let ad = self.server.ad;
        let p = self.p;

        let acquire_time = self.server.get_f64(ad.acquire_time).await;
        let shutter_mode = self.server.get_i32(ad.shutter_mode).await;
        let size = self.server.get_i32(p.size).await;
        let res = self.server.get_i32(p.res).await;
        let array_callbacks = self.server.get_i32(ad.base.array_callbacks).await;
        let erase_mode = self.server.get_i32(p.erase_mode).await;
        // ADShutterModeNone == 0.
        let use_shutter = shutter_mode != 0;

        // C stores acqStartTime here (used later for the NDArray timestamp).
        self.store_acq_start(now_epoch_f64());

        let img_size = image_size(res, size);
        let base_name = self.create_file_name().await;
        let full = full_file_name(&base_name, img_size);

        // Erase before exposure if set.
        if erase_mode == EraseMode::Before as i32 {
            let status = self.erase().await;
            if status.is_err() {
                return status;
            }
        }

        // Start the exposure. C arms an epicsTimer that signals the stop event
        // when the exposure time expires; this port folds that deadline into the
        // wait loop below. `start` is the timeRemaining reference (C `startTime`,
        // captured before the shutter opens).
        let start = Instant::now();
        if use_shutter {
            self.server.set_shutter(true).await;
        }
        let deadline = Instant::now() + secs(acquire_time);
        self.server.set_i32(ad.status, Status::Expose as i32);
        self.server.callbacks().await;

        let mut status = CamStatus::Success;
        loop {
            if self.abort.try_wait() {
                status = CamStatus::Error;
                break;
            }
            let now = Instant::now();
            if now >= deadline {
                // The exposure timer would have fired the stop event.
                break;
            }
            let wait = (deadline - now).min(secs(MAR345_POLL_DELAY));
            if self.stop.wait_timeout(wait) {
                // Acquisition stopped before the time was complete (client set
                // ADAcquire=0). This ends the exposure but is not an abort.
                break;
            }
            let mut time_remaining = acquire_time - start.elapsed().as_secs_f64();
            if time_remaining < 0.0 {
                time_remaining = 0.0;
            }
            self.server.set_f64(ad.time_remaining, time_remaining);
            self.server.callbacks().await;
        }

        self.server.set_f64(ad.time_remaining, 0.0);
        if use_shutter {
            self.server.set_shutter(false).await;
        }
        self.server.set_i32(ad.status, Status::Idle as i32);
        self.server.callbacks().await;

        // If the exposure was aborted return error.
        if status.is_err() {
            return CamStatus::Error;
        }

        self.server.set_i32(ad.status, Status::Scan as i32);
        self.server.callbacks().await;
        self.server.set_str(ad.base.full_file_name, full.clone());
        self.server.callbacks().await;
        self.server.write_server(&cmd_scan(&full)).await;
        let status = self
            .server
            .wait_for_completion(DONE_SCAN_DATA, MAR345_COMMAND_TIMEOUT)
            .await;
        if status.is_err() {
            return CamStatus::Error;
        }

        let image_counter = self.server.get_i32(ad.base.array_counter).await + 1;
        self.server.set_i32(ad.base.array_counter, image_counter);
        self.server.callbacks().await;

        // If arrayCallbacks is set then read the file back in.
        if array_callbacks != 0 {
            self.get_image_data().await;
        }

        // Erase after scanning if set.
        if erase_mode == EraseMode::After as i32 {
            return self.erase().await;
        }
        CamStatus::Success
    }

    /// C `getImageData` — read the `.mar` file, decode the pck stream, publish.
    async fn get_image_data(&mut self) {
        let ad = self.server.ad;
        let base = ad.base;
        let full = self
            .server
            .get_str(base.full_file_name, MAX_FILENAME_LEN)
            .await;
        let nx = self.server.get_i32(base.array_size_x).await.max(0) as usize;
        let ny = self.server.get_i32(base.array_size_y).await.max(0) as usize;
        let image_counter = self.server.get_i32(base.array_counter).await;

        self.server
            .set_str(ad.status_message, format!("Reading mar345 file {full}"));
        self.server.callbacks().await;

        let data = match std::fs::read(&full) {
            Ok(d) => d,
            Err(e) => {
                // C logs the errno and returns without publishing.
                log::error!("mar345: unable to open input file {full}, error={e}");
                return;
            }
        };

        let pixels = get_pck(&data, nx, ny);

        let mut array = NDArray::with_data(
            vec![NDDimension::new(nx), NDDimension::new(ny)],
            NDDataBuffer::U16(pixels.iter().map(|&v| v as u16).collect()),
        );
        array.unique_id = image_counter;
        // C sets the double timeStamp from acqStartTime and the epicsTS from the
        // current time (updateTimeStamp).
        array.time_stamp = self.load_acq_start();
        array.timestamp = EpicsTimestamp::now();
        self.output.publish(Arc::new(array)).await;
    }

    // -----------------------------------------------------------------------
    // Acquisition loop
    // -----------------------------------------------------------------------

    /// C `mar345Task`'s `mar345ModeAcquire` case.
    async fn run_acquire(&mut self) {
        let ad = self.server.ad;

        let image_mode = self.server.get_i32(ad.image_mode).await;
        let mut num_images = self.server.get_i32(ad.num_images).await;
        if num_images < 1 {
            num_images = 1;
        }
        if image_mode == ImageMode::Single as i32 {
            num_images = 1;
        }
        let continuous = image_mode == ImageMode::Continuous as i32;

        let mut counter = 0;
        while counter < num_images || continuous {
            if self.abort.try_wait() {
                break;
            }
            self.server.set_i32(ad.num_images_counter, counter);
            self.server.callbacks().await;
            let status = self.acquire_frame().await;
            if status.is_err() {
                break;
            }
            // We get out of the loop in single-shot mode or if acquire was set
            // to 0 by the client.
            if image_mode == ImageMode::Single as i32 {
                self.server.set_i32(ad.acquire, 0);
            }
            let acquire = self.server.get_i32(ad.acquire).await;
            if acquire == 0 {
                break;
            }
            // Continuous or multiple mode: sleep until the acquire period expires
            // or acquire is set to stop.
            let elapsed = now_epoch_f64() - self.load_acq_start();
            let acquire_period = self.server.get_f64(ad.acquire_period).await;
            let delay_time = acquire_period - elapsed;
            if delay_time > 0.0 {
                self.server.set_i32(ad.status, Status::Waiting as i32);
                self.server.callbacks().await;
                if self.abort.wait_timeout(secs(delay_time)) {
                    break;
                }
            }
            counter += 1;
        }

        self.mode.store(Mode::Idle as i32, Ordering::SeqCst);
        self.server.set_i32(ad.acquire, 0);
        self.server.set_i32(ad.status, Status::Idle as i32);
    }

    /// C `mar345Task` main loop.
    async fn run(mut self) {
        let ad = self.server.ad;
        loop {
            self.server.set_str(ad.status_message, "Waiting for event");
            self.server.callbacks().await;
            // Release the (nonexistent) lock and block until a start event.
            self.start.wait();

            match Mode::from_i32(self.mode.load(Ordering::SeqCst)) {
                Some(Mode::Erase) => {
                    self.erase().await;
                    self.mode.store(Mode::Idle as i32, Ordering::SeqCst);
                }
                Some(Mode::Acquire) => {
                    self.run_acquire().await;
                }
                Some(Mode::Change) => {
                    self.change_mode().await;
                    self.mode.store(Mode::Idle as i32, Ordering::SeqCst);
                }
                // Idle or unknown: nothing to do.
                _ => {}
            }

            self.server.callbacks().await;
        }
    }
}

/// Spawn the `mar345Task` worker thread.
pub(crate) fn start_task(w: Worker) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("mar345Task", move || w.run())
}
