//! The two Pilatus worker threads.
//!
//! * `PilatusDetTask` is C's `pilatusTask` acquisition state machine; it talks
//!   to camserver directly, as `pilatusTask` does while the driver lock is
//!   released around each socket poll.
//! * `PilatusFlatField` publishes the flat field the port thread reads.

use std::path::Path;
use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::error::ADResult;
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataView, NDDataViewMut, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool, PersistentArray, PublishedArray};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::trace::TraceMask;
use parking_lot::Mutex;

use crate::camserver::{CamParams, Ctx};
use crate::file_name::{MultipleFileFormat, format_file_template, make_multiple_file_format};
use crate::image::{apply_flat_field, correct_bad_pixels, decode_tiff};
use crate::protocol::{cmd_exposure, cmd_header_string, cmd_mx_f64};
use crate::types::{
    BadPixel, CAMSERVER_ACQUIRE_TIMEOUT, CAMSERVER_DEFAULT_TIMEOUT, Event, FILE_READ_DELAY,
    MAX_FILENAME_LEN, MAX_HEADER_STRING_LEN, TriggerMode, secs,
};

/// State shared between the port actor and the two worker threads.
#[derive(Default)]
pub struct Shared {
    /// C `demandedThreshold` (keV).
    pub demanded_threshold: f64,
    /// C `demandedEnergy` (keV).
    pub demanded_energy: f64,
    /// C `badPixelMap` truncated to `numBadPixels`.
    pub bad_pixels: Vec<BadPixel>,
    /// C `pFlatField`, `None` when the pool refused it at construction.
    pub flat_field: Option<PersistentArray>,
    /// C `averageFlatField`.
    pub average_flat_field: f64,
}

/// Everything a worker thread needs beyond its [`Ctx`].
pub(crate) struct Worker {
    pub ctx: Ctx,
    pub shared: Arc<Mutex<Shared>>,
    pub output: ArrayPublisher,
    pub pool: DriverPool,
}

// ---------------------------------------------------------------------------
// Flat-field publisher
// ---------------------------------------------------------------------------

/// A flat field and the port thread waiting for its publish.
pub(crate) type FlatFieldJob = (PublishedArray, std::sync::mpsc::SyncSender<()>);

/// Fans out the flat field `readFlatFieldFile` reads on the port thread. C
/// calls `doCallbacksGenericPointer` there (pilatusDetector.cpp:316); the
/// port actor's current-thread runtime cannot await [`ArrayPublisher::publish`],
/// so the port thread waits for the ack instead.
pub(crate) fn start_flat_field_task(
    output: ArrayPublisher,
    mut rx: rt::CommandReceiver<FlatFieldJob>,
) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("PilatusFlatField", move || async move {
        while let Some((array, done)) = rx.recv().await {
            output.publish(array).await;
            let _ = done.send(());
        }
    })
}

/// C reads into `pFlatField` and publishes it (pilatusDetector.cpp:290,
/// :316). `None` with no flat-field array, which leaves FlatFieldValid 0.
pub(crate) fn refill_flat_field(
    s: &mut Shared,
    data: &[i32],
    description: Option<String>,
    average: f64,
) -> Option<PublishedArray> {
    let flat = s.flat_field.as_mut()?;
    let array = flat.refill();
    let NDDataViewMut::I32(pixels) = array.data.view_mut() else {
        unreachable!("allocated as NDInt32")
    };
    pixels.copy_from_slice(data);
    if let Some(d) = description {
        array.attributes.add(tiff_description(d));
    }
    s.average_flat_field = average;
    Some(flat.published())
}

// ---------------------------------------------------------------------------
// Image files
// ---------------------------------------------------------------------------

fn ends_with_ignore_case(name: &str, suffix: &str) -> bool {
    let n = name.as_bytes();
    let s = suffix.as_bytes();
    n.len() >= s.len()
        && n[n.len() - s.len()..]
            .iter()
            .zip(s)
            .all(|(a, b)| a.eq_ignore_ascii_case(b))
}

/// Whole seconds since the epoch, matching C's `time_t` truncation in
/// `epicsTimeToTime_t` and `struct stat::st_mtime`.
fn epoch_secs(t: SystemTime) -> i64 {
    match t.duration_since(UNIX_EPOCH) {
        Ok(d) => d.as_secs() as i64,
        Err(e) => -(e.duration().as_secs() as i64),
    }
}

/// The image-file readers C runs on both the port thread and `pilatusTask`:
/// where they post status, and the stop event their retry sleeps watch.
pub(crate) struct ImageReader<'a, P: CamParams> {
    pub cx: &'a mut P,
    pub ad: ADBaseParams,
    pub stop: &'a Event,
}

impl<P: CamParams> ImageReader<'_, P> {
    /// C `waitForFileToExist()`.
    fn wait_for_file_to_exist(
        &mut self,
        file: &Path,
        start_time: Option<SystemTime>,
        timeout: f64,
    ) -> AsynStatus {
        let acq_start = start_time.map(epoch_secs).unwrap_or(0);
        let t_start = Instant::now();
        let mut delta_time = 0.0f64;
        let mut file_exists = false;
        let mut opened = false;

        while delta_time <= timeout {
            if let Ok(f) = std::fs::File::open(file) {
                if timeout != 0.0 {
                    file_exists = true;
                    // The file exists. Make sure it is a new file, not an old one.
                    // This check is skipped when timeout == 0 (flat field files).
                    let modified = match f.metadata().and_then(|m| m.modified()) {
                        Ok(m) => m,
                        Err(e) => {
                            self.cx.trace(
                            TraceMask::ERROR,
                            &format!(
                                "pilatusDetector::waitForFileToExist error calling fstat, errno={} {}\n",
                                e.raw_os_error().unwrap_or(0),
                                file.display()
                            ),
                        );
                            return AsynStatus::Error;
                        }
                    };
                    // Allow up to 10 seconds of clock skew between this IOC and the
                    // machine whose file system reports the modification time.
                    if (epoch_secs(modified) - acq_start) as f64 > -10.0 {
                        opened = true;
                        break;
                    }
                } else {
                    opened = true;
                }
            }
            if self.stop.wait_timeout(secs(FILE_READ_DELAY)) {
                self.cx
                    .set_str(self.ad.status_message, "Acquisition aborted".into());
                self.cx.set_i32(self.ad.status, ADStatus::Aborted as i32);
                return AsynStatus::Error;
            }
            delta_time = t_start.elapsed().as_secs_f64();
        }

        if !opened {
            self.cx.trace(
            TraceMask::ERROR,
            &format!(
                "pilatusDetector::waitForFileToExist timeout waiting for file to be created {}\n",
                file.display()
            ),
        );
            if file_exists {
                self.cx.trace(
                TraceMask::ERROR,
                "  file exists but is more than 10 seconds old, possible clock synchronization problem\n",
            );
                self.cx.set_str(
                    self.ad.status_message,
                    "Image file is more than 10 seconds old".into(),
                );
            } else {
                self.cx.set_str(
                    self.ad.status_message,
                    "Timeout waiting for file to be created".into(),
                );
            }
            return AsynStatus::Error;
        }
        AsynStatus::Success
    }

    /// C `readTiff()` — wait for the file, then retry the decode until the writer
    /// has finished, and finally apply the bad-pixel map.
    ///
    /// Fixed (upstream-c-defects.md #8): when the retry loop exhausts `timeout`
    /// without a successful decode, C falls through and returns `asynSuccess` with
    /// a buffer it never filled — the caller then publishes stale/garbage pixels.
    /// That is a failed read, so this returns [`AsynStatus::Error`] and the caller
    /// does not publish.
    fn read_tiff(
        &mut self,
        file: &Path,
        start_time: Option<SystemTime>,
        timeout: f64,
        nx: usize,
        ny: usize,
        bad_pixels: &[BadPixel],
    ) -> Result<(Vec<i32>, Option<String>), AsynStatus> {
        let t_start = Instant::now();
        let mut delta_time = 0.0f64;

        let status = self.wait_for_file_to_exist(file, start_time, timeout);
        if status != AsynStatus::Success {
            return Err(status);
        }

        let mut decoded = None;
        while delta_time <= timeout {
            // The file exists but may not be completely written yet.
            match decode_tiff(file) {
                Ok(img) if img.width as usize == nx && img.height as usize == ny => {
                    decoded = Some(img);
                    break;
                }
                Ok(img) if img.width as usize != nx => self.cx.trace(
                    TraceMask::ERROR,
                    &format!(
                        "pilatusDetector::readTiff, image width incorrect ={}, should be {nx}\n",
                        img.width
                    ),
                ),
                Ok(img) => self.cx.trace(
                    TraceMask::ERROR,
                    &format!(
                        "pilatusDetector::readTiff, image length incorrect ={}, should be {ny}\n",
                        img.height
                    ),
                ),
                Err(_) => self.cx.trace(
                    TraceMask::FLOW,
                    &format!(
                        "pilatusDetector::readTiff, error reading TIFF file {}\n",
                        file.display()
                    ),
                ),
            }
            // Sleep, but check for the stop event so a long acquisition can abort.
            if self.stop.wait_timeout(secs(FILE_READ_DELAY)) {
                self.cx.set_i32(self.ad.status, ADStatus::Aborted as i32);
                return Err(AsynStatus::Error);
            }
            delta_time = t_start.elapsed().as_secs_f64();
        }

        let Some(img) = decoded else {
            // Fixed (upstream-c-defects.md #8): the retry loop timed out without a
            // valid frame. Do not fabricate a buffer or publish it.
            return Err(AsynStatus::Error);
        };
        let (mut data, description) = (img.data, img.description);
        correct_bad_pixels(&mut data, bad_pixels);
        Ok((data, description))
    }

    /// C `readImageFile()`.
    ///
    /// Deviation: `.cbf` is not supported. C links CBFlib and calls `readCbf`;
    /// no equivalent Rust crate is vendored here, so `.cbf` is rejected.
    pub(crate) fn read_image_file(
        &mut self,
        file: &str,
        start_time: Option<SystemTime>,
        timeout: f64,
        nx: usize,
        ny: usize,
        bad_pixels: &[BadPixel],
    ) -> Result<(Vec<i32>, Option<String>), AsynStatus> {
        if ends_with_ignore_case(file, ".tif") || ends_with_ignore_case(file, ".tiff") {
            self.read_tiff(Path::new(file), start_time, timeout, nx, ny, bad_pixels)
        } else if ends_with_ignore_case(file, ".cbf") {
            self.cx
                .set_str(self.ad.status_message, "CBF files are not supported".into());
            Err(AsynStatus::Error)
        } else {
            self.cx.trace(
                TraceMask::ERROR,
                &format!(
                    "pilatusDetector::readImageFile, unsupported image file name extension, \
                 expected .tif or .cbf, fileName={file}\n"
                ),
            );
            self.cx.set_str(
                self.ad.status_message,
                "Unsupported file extension, expected .tif or .cbf".into(),
            );
            Err(AsynStatus::Error)
        }
    }
}

fn build_array(
    pool: &DriverPool,
    unique_id: i32,
    ts: EpicsTimestamp,
    nx: usize,
    ny: usize,
    data: Vec<i32>,
    description: Option<String>,
) -> ADResult<DriverArray> {
    let buffer = NDDataBuffer::from_vec(data);
    let mut attributes = NDAttributeList::new();
    if let Some(d) = description {
        attributes.add(tiff_description(d));
    }
    let mut array = pool.alloc_with(vec![NDDimension::new(nx), NDDimension::new(ny)], buffer)?;
    array.unique_id = unique_id;
    array.timestamp = ts;
    array.time_stamp = ts.as_f64();
    array.attributes = attributes;
    Ok(array)
}

/// C `pImage->pAttributeList->add("TIFFImageDescription", ...)`
/// (pilatusDetector.cpp:722).
fn tiff_description(description: String) -> NDAttribute {
    NDAttribute::new_static(
        "TIFFImageDescription",
        "TIFFImageDescription",
        NDAttrSource::Driver,
        NDAttrValue::String(description),
    )
}

async fn max_size(ctx: &mut Ctx) -> (usize, usize) {
    let nx = ctx.get_i32(ctx.ad.max_size_x).await.unwrap_or(0).max(0) as usize;
    let ny = ctx.get_i32(ctx.ad.max_size_y).await.unwrap_or(0).max(0) as usize;
    (nx, ny)
}

// ---------------------------------------------------------------------------
// Acquisition thread
// ---------------------------------------------------------------------------

pub(crate) fn start_acq_task(w: Worker, start: Arc<Event>) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("PilatusDetTask", move || acq_loop(w, start))
}

/// C `asynNDArrayDriver::createFileName`.
async fn create_file_name(ctx: &mut Ctx) -> String {
    let base = ctx.ad.base;
    let file_path = ctx.get_str(base.file_path, MAX_FILENAME_LEN).await;
    let file_name = ctx.get_str(base.file_name, MAX_FILENAME_LEN).await;
    let template = ctx.get_str(base.file_template, MAX_FILENAME_LEN).await;
    let file_number = ctx.get_i32(base.file_number).await.unwrap_or(0);
    let auto_increment = ctx.get_i32(base.auto_increment).await.unwrap_or(0);

    let full =
        format_file_template(&template, &file_path, &file_name, file_number).unwrap_or_default();
    if auto_increment != 0 {
        ctx.set_i32(base.file_number, file_number + 1);
    }
    full
}

async fn acq_loop(mut w: Worker, start: Arc<Event>) {
    let mut status = AsynStatus::Success;
    let mut aborted = false;
    let mut multiple_file_number = 0i32;
    let mut multiple_file_next_image;
    let mut mff = MultipleFileFormat {
        prefix: String::new(),
        digits: 5,
        extension: String::new(),
        start_number: 0,
    };

    loop {
        let mut acquire = w.ctx.get_i32(w.ctx.ad.acquire).await.unwrap_or(0) != 0;

        if aborted || !acquire {
            // Only set the status message if we didn't encounter any errors
            // last time, so we don't overwrite the error message.
            if status == AsynStatus::Success {
                let reason = w.ctx.ad.status_message;
                w.ctx.set_str(reason, "Waiting for acquire command");
            }
            w.ctx.callbacks().await;
            start.wait();
            aborted = false;
            acquire = true;
        }

        let start_time = SystemTime::now();

        let acquire_time = w.ctx.get_f64(w.ctx.ad.acquire_time).await.unwrap_or(0.0);
        let acquire_period = w.ctx.get_f64(w.ctx.ad.acquire_period).await.unwrap_or(0.0);
        let read_image_file_timeout = w.ctx.get_f64(w.ctx.p.image_file_tmot).await.unwrap_or(0.0);
        let trigger_mode = w.ctx.get_i32(w.ctx.ad.trigger_mode).await.unwrap_or(0);
        let num_images = w.ctx.get_i32(w.ctx.ad.num_images).await.unwrap_or(0);
        let num_exposures = w.ctx.get_i32(w.ctx.ad.num_exposures).await.unwrap_or(0);

        w.ctx.set_i32(w.ctx.ad.status, ADStatus::Acquire as i32);

        // Reset the MX settings start angle.
        let start_angle = w.ctx.get_f64(w.ctx.p.start_angle).await.unwrap_or(0.0);
        w.ctx.cam_write_read(
            &cmd_mx_f64("Start_angle", start_angle),
            CAMSERVER_DEFAULT_TIMEOUT,
        );

        // Send the header string. This needs to be sent for each exposure.
        let header = w
            .ctx
            .get_str(w.ctx.p.header_string, MAX_HEADER_STRING_LEN)
            .await;
        w.ctx
            .cam_write_read(&cmd_header_string(&header), CAMSERVER_DEFAULT_TIMEOUT);

        let mut full_file_name = create_file_name(&mut w.ctx).await;
        let mode = TriggerMode::from_i32(trigger_mode);
        let exposure_cmd = match mode {
            Some(TriggerMode::Alignment) => {
                let file_path = w
                    .ctx
                    .get_str(w.ctx.ad.base.file_path, MAX_FILENAME_LEN)
                    .await;
                full_file_name = format!("{file_path}alignment.tif");
                cmd_exposure(TriggerMode::Alignment, &full_file_name)
            }
            Some(m) => cmd_exposure(m, &full_file_name),
            // C's switch has no default: `toCamserver` keeps whatever the
            // previous command left in it, which is the `HeaderString` write.
            None => w.ctx.to_camserver().to_string(),
        };

        let reason = w.ctx.ad.status_message;
        w.ctx.set_str(reason, "Starting exposure");
        // Send the acquire command and wait for the 15 OK response. C discards
        // this status.
        w.ctx.cam_write_read(&exposure_cmd, 2.0);
        // Do another read in case there is an ERR string at the end of the
        // input buffer.
        status = w.ctx.cam_read(0.0);

        let mut array_callbacks = 0;
        multiple_file_next_image = 0;

        if status as i32 > 1 {
            acquire = false;
        } else {
            // The timeout was expected.
            status = AsynStatus::Success;
            w.ctx.set_shutter(true).await;
            w.ctx.set_i32(w.ctx.p.armed, 1);
            mff = make_multiple_file_format(&full_file_name, num_images);
            multiple_file_number = mff.start_number;
            let reason = w.ctx.ad.base.full_file_name;
            w.ctx.set_str(reason, full_file_name.clone());
            w.ctx.callbacks().await;
        }

        while acquire {
            if num_images == 1 {
                // Single-frame and alignment modes must wait for the 7 OK
                // response before reading the file, else we get a recent but
                // stale one.
                let reason = w.ctx.ad.status_message;
                w.ctx.set_str(reason, "Waiting for 7OK response");
                w.ctx.callbacks().await;
                let timeout = ((num_exposures - 1) as f64 * acquire_period) + acquire_time;
                status = w.ctx.cam_read(timeout + CAMSERVER_ACQUIRE_TIMEOUT);
                if status != AsynStatus::Success {
                    acquire = false;
                    aborted = true;
                    if status == AsynStatus::Timeout {
                        let reason = w.ctx.ad.status_message;
                        w.ctx
                            .set_str(reason, "Timeout waiting for camserver response");
                        w.ctx.cam_write_read("camcmd k", CAMSERVER_DEFAULT_TIMEOUT);
                        w.ctx.cam_write("K", CAMSERVER_DEFAULT_TIMEOUT);
                    }
                    continue;
                }
            } else {
                full_file_name = mff.file_name(multiple_file_number);
                let reason = w.ctx.ad.base.full_file_name;
                w.ctx.set_str(reason, full_file_name.clone());
            }

            array_callbacks = w
                .ctx
                .get_i32(w.ctx.ad.base.array_callbacks)
                .await
                .unwrap_or(0);

            if array_callbacks != 0 {
                let (nx, ny) = max_size(&mut w.ctx).await;
                let reason = w.ctx.ad.status_message;
                w.ctx
                    .set_str(reason, format!("Reading image file {full_file_name}"));
                w.ctx.callbacks().await;

                let bad_pixels = w.shared.lock().bad_pixels.clone();
                let read = w.ctx.image_reader().read_image_file(
                    &full_file_name,
                    Some(start_time),
                    (num_exposures as f64 * acquire_time) + read_image_file_timeout,
                    nx,
                    ny,
                    &bad_pixels,
                );
                let (mut data, description) = match read {
                    Ok(v) => v,
                    Err(s) => {
                        status = s;
                        acquire = false;
                        aborted = true;
                        continue;
                    }
                };

                // We successfully read an image — increment the array counter.
                let image_counter = w
                    .ctx
                    .get_i32(w.ctx.ad.base.array_counter)
                    .await
                    .unwrap_or(0)
                    + 1;
                w.ctx.set_i32(w.ctx.ad.base.array_counter, image_counter);
                w.ctx.callbacks().await;

                let flat_field_valid = w.ctx.get_i32(w.ctx.p.flat_field_valid).await.unwrap_or(0);
                if flat_field_valid != 0 {
                    let s = w.shared.lock();
                    if let Some(NDDataView::I32(flat)) =
                        s.flat_field.as_ref().map(|f| f.data.view())
                    {
                        apply_flat_field(&mut data, flat, s.average_flat_field);
                    }
                }

                // C dereferences a refused alloc (pilatusDetector.cpp:1190);
                // the frame is dropped after the pool's reached-limit line.
                // C updateTimeStamps (pilatusDetector.cpp:1227).
                let ts = w.ctx.update_time_stamp().await;
                if let Ok(array) =
                    build_array(&w.pool, image_counter, ts, nx, ny, data, description)
                {
                    w.output.publish(array).await;
                }
            }

            if num_images == 1 {
                if mode == Some(TriggerMode::Alignment) {
                    w.ctx.cam_write_read(
                        &cmd_exposure(TriggerMode::Alignment, &full_file_name),
                        2.0,
                    );
                } else {
                    acquire = false;
                }
            } else if num_images > 1 {
                multiple_file_next_image += 1;
                multiple_file_number += 1;
                if multiple_file_next_image == num_images {
                    acquire = false;
                }
            }
        }

        // Wait for the 7 OK response from camserver for multiple images.
        if num_images > 1 && status == AsynStatus::Success {
            // With arrayCallbacks off we never waited for the individual image
            // files, so the response can take a long time.
            let timeout = if array_callbacks != 0 {
                read_image_file_timeout
            } else {
                (num_images as f64 * num_exposures as f64 * acquire_period)
                    + CAMSERVER_ACQUIRE_TIMEOUT
            };
            let reason = w.ctx.ad.status_message;
            w.ctx.set_str(reason, "Waiting for 7OK response");
            w.ctx.callbacks().await;
            status = w.ctx.cam_read(timeout);
            // On a timeout camserver could still be acquiring, so send a stop.
            if status == AsynStatus::Timeout {
                let reason = w.ctx.ad.status_message;
                w.ctx
                    .set_str(reason, "Timeout waiting for camserver response");
                w.ctx.cam_write_read("camcmd k", CAMSERVER_DEFAULT_TIMEOUT);
                w.ctx.cam_write("K", CAMSERVER_DEFAULT_TIMEOUT);
                aborted = true;
            }
        }

        let status_param = w.ctx.get_i32(w.ctx.ad.status).await.unwrap_or(0);
        if status == AsynStatus::Success {
            w.ctx.set_i32(w.ctx.ad.status, ADStatus::Idle as i32);
        } else if status_param != ADStatus::Aborted as i32 {
            w.ctx.set_i32(w.ctx.ad.status, ADStatus::Error as i32);
        }
        w.ctx.callbacks().await;

        w.ctx.set_shutter(false).await;
        w.ctx.set_i32(w.ctx.ad.acquire, 0);
        w.ctx.set_i32(w.ctx.p.armed, 0);
        w.ctx.callbacks().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extension_match_is_case_insensitive() {
        assert!(ends_with_ignore_case("a.TIF", ".tif"));
        assert!(ends_with_ignore_case("a.tiff", ".tiff"));
        assert!(!ends_with_ignore_case("a.tif", ".tiff"));
        assert!(ends_with_ignore_case("a.CBF", ".cbf"));
        assert!(!ends_with_ignore_case("tif", ".tif"));
    }

    #[test]
    fn epoch_secs_truncates_like_time_t() {
        let t = UNIX_EPOCH + std::time::Duration::from_millis(1_500);
        assert_eq!(epoch_secs(t), 1);
    }

    fn flat_field_pool(max_memory: usize) -> (DriverPool, Shared) {
        let pool = DriverPool::standalone(max_memory);
        let flat = pool
            .alloc(
                vec![NDDimension::new(2), NDDimension::new(1)],
                epics_rs::ad_core::ndarray::NDDataType::Int32,
            )
            .unwrap();
        let s = Shared {
            flat_field: Some(PersistentArray::new(flat)),
            ..Shared::default()
        };
        (pool, s)
    }

    /// C publishes the one `pFlatField` it allocated (pilatusDetector.cpp:316,
    /// :1728), so a second read charges no second buffer.
    #[test]
    fn a_flat_field_read_publishes_the_array_allocated_at_construction() {
        let (pool, mut s) = flat_field_pool(8);
        let first = refill_flat_field(&mut s, &[5, 6], Some("d".into()), 5.5).unwrap();
        assert!(matches!(first.data.view(), NDDataView::I32(v) if v == &[5, 6]));
        drop(first);
        let second = refill_flat_field(&mut s, &[7, 8], None, 7.5).unwrap();
        assert!(matches!(second.data.view(), NDDataView::I32(v) if v == &[7, 8]));
        assert!(second.attributes.get("TIFFImageDescription").is_some());
        assert_eq!(s.average_flat_field, 7.5);
        assert_eq!(
            (
                pool.num_alloc_buffers(),
                pool.num_free_buffers(),
                pool.allocated_bytes()
            ),
            (1, 0, 8)
        );
    }

    /// C refills `pFlatField` in place while a plugin still holds it, so a
    /// held publish neither refuses the read nor charges a second buffer.
    #[test]
    fn a_held_flat_field_is_still_refilled() {
        let (pool, mut s) = flat_field_pool(8);
        let held = refill_flat_field(&mut s, &[5, 6], None, 5.5).unwrap();
        let next = refill_flat_field(&mut s, &[7, 8], None, 7.5).unwrap();
        assert!(matches!(next.data.view(), NDDataView::I32(v) if v == &[7, 8]));
        assert!(matches!(held.data.view(), NDDataView::I32(v) if v == &[5, 6]));
        assert_eq!(s.average_flat_field, 7.5);
        assert_eq!((pool.num_alloc_buffers(), pool.allocated_bytes()), (1, 8));
        assert!(refill_flat_field(&mut Shared::default(), &[1, 2], None, 1.0).is_none());
    }

    /// readTiff rejects a file whose size is not pFlatField's
    /// (pilatusDetector.cpp:678, :685), whatever MaxSizeX reads now.
    #[test]
    fn a_flat_field_file_sized_to_max_size_not_pflatfield_is_rejected() {
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let mut det = flat_field_driver("PILATUS_FLAT_DIMS", flat_field_tx);
        let (ad, p) = (det.ad.params, det.p);
        det.ad.port_base.set_i32(ad.max_size_x, 3);
        let path = flat_field_tiff("dims", 3, &[7, 8, 9]);
        det.read_flat_field_file(&path.to_string_lossy());
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(p.flat_field_valid, 0)
                .unwrap(),
            0
        );
    }

    /// FlatFieldValid follows the publish (pilatusDetector.cpp:315-316): a
    /// flat field the publisher never took leaves it 0.
    #[test]
    fn a_flat_field_the_publisher_never_took_is_not_valid() {
        let (flat_field_tx, flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        drop(flat_field_rx);
        let mut det = flat_field_driver("PILATUS_FLAT_UNPUBLISHED", flat_field_tx);
        let p = det.p;
        let path = flat_field_tiff("unpublished", 2, &[7, 9]);
        det.read_flat_field_file(&path.to_string_lossy());
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(p.flat_field_valid, 0)
                .unwrap(),
            0
        );
    }

    /// A 2x1 detector with no camserver and a 2x1 `pFlatField`.
    fn flat_field_driver(
        port: &str,
        flat_field_tx: rt::CommandSender<FlatFieldJob>,
    ) -> crate::driver::PilatusDriver {
        use crate::driver::{DriverLinks, PilatusDriver};

        let shared = Arc::new(Mutex::new(Shared::default()));
        let det = PilatusDriver::new(
            port,
            2,
            1,
            0,
            DriverLinks {
                flat_field_tx,
                start: Arc::new(Event::new()),
                stop: Arc::new(Event::new()),
                shared: shared.clone(),
                cam: Arc::new(
                    epics_rs::asyn::sync_io::SyncIOHandle::connect_port(
                        "PILATUS_NO_CAM",
                        0,
                        std::time::Duration::ZERO,
                        epics_rs::asyn::interfaces::InterfaceType::Octet,
                    )
                    .0,
                ),
            },
        )
        .unwrap();
        shared.lock().flat_field = Some(PersistentArray::new(
            det.ad
                .pool
                .alloc(
                    vec![NDDimension::new(2), NDDimension::new(1)],
                    epics_rs::ad_core::ndarray::NDDataType::Int32,
                )
                .unwrap(),
        ));
        det
    }

    /// A one-row Gray32 TIFF of `width` pixels.
    fn flat_field_tiff(tag: &str, width: u32, pixels: &[u32]) -> std::path::PathBuf {
        let path = std::env::temp_dir().join(format!("pil-flat-{tag}-{}.tif", std::process::id()));
        let mut enc =
            tiff::encoder::TiffEncoder::new(std::fs::File::create(&path).unwrap()).unwrap();
        enc.write_image::<tiff::encoder::colortype::Gray32>(width, 1, pixels)
            .unwrap();
        path
    }

    #[test]
    fn build_array_carries_tiff_description() {
        let a = build_array(
            &DriverPool::standalone(0),
            7,
            EpicsTimestamp::default(),
            2,
            2,
            vec![1, 2, 3, 4],
            Some("desc".into()),
        )
        .unwrap();
        assert_eq!(a.unique_id, 7);
        assert_eq!(a.dims.len(), 2);
        assert_eq!(a.data_size, 16);
        assert_eq!(
            a.attributes
                .get("TIFFImageDescription")
                .map(|x| x.value.as_string()),
            Some("desc".to_string())
        );
    }
}
