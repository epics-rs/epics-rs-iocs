//! The two Pilatus worker threads.
//!
//! * `PilatusCmdTask` executes everything C does inline in `writeInt32` /
//!   `writeFloat64` / `writeOctet` (`setAcquireParams`, `setThreshold`,
//!   `resetModulePower`, `pilatusStatus`, the `mxsettings` writes, `imgpath`,
//!   the bad-pixel / flat-field file reads and the abort sequence).
//! * `PilatusDetTask` is C's `pilatusTask` acquisition state machine.
//!
//! Both talk to camserver directly, exactly as C's port thread and
//! `pilatusTask` do while the driver lock is released around each socket poll.

use std::path::Path;
use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use parking_lot::Mutex;

use crate::camserver::Ctx;
use crate::file_name::{
    MultipleFileFormat, check_path, format_file_template, make_multiple_file_format,
};
use crate::image::{
    apply_flat_field, apply_flat_field_floor, correct_bad_pixels, decode_tiff, flat_field_average,
    parse_bad_pixel_file,
};
use crate::protocol::{
    cmd_delay, cmd_expperiod, cmd_exptime, cmd_gapfill, cmd_nexpframe, cmd_nimages,
    parse_tau_cutoff,
};
use crate::protocol::{
    cmd_exposure, cmd_header_string, cmd_imgpath, cmd_mx_f64, cmd_reset_module_power,
    cmd_set_threshold, gain_index, parse_energy_setting, parse_thread_channel, parse_threshold_ev,
    parse_version,
};
use crate::types::{
    BadPixel, CAMSERVER_ACQUIRE_TIMEOUT, CAMSERVER_DEFAULT_TIMEOUT, CAMSERVER_RESET_POWER_TIMEOUT,
    CamStatus, Event, FILE_READ_DELAY, MAX_FILENAME_LEN, MAX_HEADER_STRING_LEN, TriggerMode, secs,
};

/// Work that C performs synchronously inside an asyn `write*` callback. A Rust
/// `PortDriver` method runs inside the port actor and must not block on another
/// port, so the actor enqueues these instead.
#[derive(Debug)]
pub enum Cmd {
    /// C `setAcquireParams()`.
    SetAcquireParams,
    /// C `setThreshold()`.
    SetThreshold,
    /// C `resetModulePower()`.
    ResetModulePower,
    /// C `pilatusStatus()`.
    ReadStatus,
    /// C's `ADAcquire == 0` branch: `camcmd k`, `K`, sleep 2 s.
    AbortAcquire,
    /// C's `NDFilePath` branch: `imgpath <path>` then `checkPath()`.
    ImgPath(String),
    /// C `readBadPixelFile()`.
    ReadBadPixelFile(String),
    /// C `readFlatFieldFile()`.
    ReadFlatFieldFile(String),
    /// A camserver command already formatted by the actor (`mxsettings ...`,
    /// `mxsettings N_oscillations ...`), sent with `CAMSERVER_DEFAULT_TIMEOUT`.
    Raw(String),
}

/// State shared between the port actor and the two worker threads.
#[derive(Default)]
pub struct Shared {
    /// C `demandedThreshold` (keV).
    pub demanded_threshold: f64,
    /// C `demandedEnergy` (keV).
    pub demanded_energy: f64,
    /// C `badPixelMap` truncated to `numBadPixels`.
    pub bad_pixels: Vec<BadPixel>,
    /// C `pFlatField->pData`.
    pub flat_field: Vec<i32>,
    /// C `averageFlatField`.
    pub average_flat_field: f64,
}

/// Everything a worker thread needs beyond its [`Ctx`].
pub(crate) struct Worker {
    pub ctx: Ctx,
    pub shared: Arc<Mutex<Shared>>,
    pub output: ArrayPublisher,
}

// ---------------------------------------------------------------------------
// Command thread
// ---------------------------------------------------------------------------

pub(crate) fn start_cmd_task(
    w: Worker,
    rx: rt::CommandReceiver<Cmd>,
) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("PilatusCmdTask", move || cmd_loop(w, rx))
}

async fn cmd_loop(mut w: Worker, mut rx: rt::CommandReceiver<Cmd>) {
    // C keeps these as driver members but only `pilatusStatus` /
    // `resetModulePower` — both on this thread — ever touch them.
    let mut first_status_call = true;
    let mut version = (0i32, 0i32, 0i32);

    while let Some(cmd) = rx.recv().await {
        match cmd {
            Cmd::SetAcquireParams => {
                set_acquire_params(&mut w.ctx).await;
                w.ctx.callbacks().await;
            }
            Cmd::SetThreshold => set_threshold(&mut w).await,
            Cmd::ResetModulePower => reset_module_power(&mut w, version).await,
            Cmd::ReadStatus => {
                pilatus_status(&mut w.ctx, &mut first_status_call, &mut version).await
            }
            Cmd::AbortAcquire => {
                w.ctx
                    .cam_write_read("camcmd k", CAMSERVER_DEFAULT_TIMEOUT)
                    .await;
                w.ctx.cam_write("K", CAMSERVER_DEFAULT_TIMEOUT).await;
                // C: epicsThreadSleep(2) to let camserver stop acquiring.
                rt::sleep(secs(2.0)).await;
                let reason = w.ctx.ad.status_message;
                w.ctx.set_str(reason, "Acquisition aborted");
                w.ctx.callbacks().await;
            }
            Cmd::ImgPath(path) => {
                w.ctx
                    .cam_write_read(&cmd_imgpath(&path), CAMSERVER_DEFAULT_TIMEOUT)
                    .await;
                let exists = check_path(&path);
                let reason = w.ctx.ad.base.file_path_exists;
                w.ctx.set_i32(reason, i32::from(exists));
                w.ctx.callbacks().await;
            }
            Cmd::ReadBadPixelFile(file) => read_bad_pixel_file(&mut w, &file).await,
            Cmd::ReadFlatFieldFile(file) => read_flat_field_file(&mut w, &file).await,
            Cmd::Raw(cmd) => {
                w.ctx.cam_write_read(&cmd, CAMSERVER_DEFAULT_TIMEOUT).await;
                w.ctx.callbacks().await;
            }
        }
    }
}

/// C `setAcquireParams()`.
async fn set_acquire_params(ctx: &mut Ctx) {
    let trigger_mode = ctx
        .get_i32(ctx.ad.trigger_mode)
        .await
        .unwrap_or(TriggerMode::Internal as i32);
    // When we change modes download all exposure parameters, since some modes
    // replace values with new parameters.
    if trigger_mode == TriggerMode::Alignment as i32 {
        ctx.set_i32(ctx.ad.num_images, 1);
    }

    let mut ival = ctx.get_i32(ctx.ad.num_images).await.unwrap_or(0);
    if ival < 1 {
        ival = 1;
        ctx.set_i32(ctx.ad.num_images, ival);
    }
    ctx.cam_write_read(&cmd_nimages(ival), CAMSERVER_DEFAULT_TIMEOUT)
        .await;

    let mut ival = ctx.get_i32(ctx.ad.num_exposures).await.unwrap_or(0);
    if ival < 1 {
        ival = 1;
        ctx.set_i32(ctx.ad.num_exposures, ival);
    }
    ctx.cam_write_read(&cmd_nexpframe(ival), CAMSERVER_DEFAULT_TIMEOUT)
        .await;

    let mut dval = ctx.get_f64(ctx.ad.acquire_time).await.unwrap_or(-1.0);
    if dval < 0.0 {
        dval = 1.0;
        ctx.set_f64(ctx.ad.acquire_time, dval);
    }
    ctx.cam_write_read(&cmd_exptime(dval), CAMSERVER_DEFAULT_TIMEOUT)
        .await;

    let mut dval = ctx.get_f64(ctx.ad.acquire_period).await.unwrap_or(-1.0);
    if dval < 0.0 {
        dval = 2.0;
        ctx.set_f64(ctx.ad.acquire_period, dval);
    }
    ctx.cam_write_read(&cmd_expperiod(dval), CAMSERVER_DEFAULT_TIMEOUT)
        .await;

    let mut dval = ctx.get_f64(ctx.p.delay_time).await.unwrap_or(-1.0);
    if dval < 0.0 {
        dval = 0.0;
        ctx.set_f64(ctx.p.delay_time, dval);
    }
    ctx.cam_write_read(&cmd_delay(dval), CAMSERVER_DEFAULT_TIMEOUT)
        .await;

    let mut ival = ctx.get_i32(ctx.p.gap_fill).await.unwrap_or(-2);
    if !(-2..=0).contains(&ival) {
        ival = -2;
        ctx.set_i32(ctx.p.gap_fill, ival);
    }
    // -2 means GapFill is not supported (single-element detector).
    if ival != -2 {
        ctx.cam_write_read(&cmd_gapfill(ival), CAMSERVER_DEFAULT_TIMEOUT)
            .await;
    }

    // Read back the pixel count-rate cut-off.
    let status = ctx.cam_write_read("Tau", 5.0).await;
    if !status.is_err()
        && let Some(cutoff) = parse_tau_cutoff(ctx.from())
    {
        ctx.set_i32(ctx.p.pixel_cutoff, cutoff);
    }
}

/// C `setThreshold()`.
async fn set_threshold(w: &mut Worker) {
    let ctx = &mut w.ctx;
    let dgain = ctx.get_f64(ctx.ad.gain).await.unwrap_or(0.0);
    let igain = gain_index(dgain);
    let (threshold, mut energy) = {
        let s = w.shared.lock();
        (s.demanded_threshold, s.demanded_energy)
    };
    if energy == 0.0 {
        energy = threshold * 2.0;
    }
    let cmd = cmd_set_threshold(energy, igain, threshold);

    // Set the status to waiting so we can be notified when it has finished.
    ctx.set_i32(ctx.ad.status, ADStatus::Waiting as i32);
    ctx.set_str(ctx.ad.status_message, "Setting threshold");
    ctx.callbacks().await;

    // This command can take 96 seconds on a 6M.
    let status = ctx.cam_write_read(&cmd, 110.0).await;
    let new_status = if status.is_err() {
        ADStatus::Error
    } else {
        ADStatus::Idle
    };
    ctx.set_i32(ctx.ad.status, new_status as i32);
    ctx.set_i32(ctx.p.threshold_apply, 0);

    // Read back the actual threshold setting, in case we are out of bounds.
    let status = ctx.cam_write_read("SetThreshold", 5.0).await;
    if !status.is_err()
        && let Some(ev) = parse_threshold_ev(ctx.from())
    {
        ctx.set_f64(ctx.p.threshold, ev as f64 / 1000.0);
    }

    // Read back the actual energy setting.
    let status = ctx.cam_write_read("SetEnergy", 5.0).await;
    if !status.is_err() {
        let ev = parse_energy_setting(ctx.from());
        ctx.set_f64(ctx.p.energy, ev as f64 / 1000.0);
    }

    // SetThreshold resets nimages and gapfill, so re-send the acquisition
    // parameters.
    set_acquire_params(ctx).await;
    ctx.callbacks().await;
}

/// C `resetModulePower()`.
async fn reset_module_power(w: &mut Worker, version: (i32, i32, i32)) {
    let (major, minor, patch) = version;
    // This command only exists on camserver 7.9.0 and higher.
    if major < 7 || (major == 7 && minor < 9) {
        log::error!(
            "pilatus: ResetModulePower not supported on version {major}.{minor}.{patch} of camserver"
        );
        return;
    }
    let reason = w.ctx.ad.status_message;
    w.ctx.set_str(reason, "Resetting module power");
    w.ctx.callbacks().await;
    let reset_time = w.ctx.get_i32(w.ctx.p.reset_power_time).await.unwrap_or(0);
    w.ctx
        .cam_write_read(
            &cmd_reset_module_power(reset_time),
            CAMSERVER_RESET_POWER_TIMEOUT + reset_time as f64,
        )
        .await;
    // The threshold must be set after resetting module power.
    set_threshold(w).await;
}

/// C `pilatusStatus()`.
async fn pilatus_status(
    ctx: &mut Ctx,
    first_status_call: &mut bool,
    version: &mut (i32, i32, i32),
) {
    if *first_status_call {
        let status = ctx.cam_write_read("version", 1.0).await;
        if !status.is_err() {
            match parse_version(ctx.from()) {
                Some(v) => {
                    ctx.set_str(ctx.p.tvx_version, v.version_string.clone());
                    ctx.set_str(ctx.ad.base.sdk_version, v.version_string);
                    if let Some(x) = v.major {
                        version.0 = x;
                    }
                    if let Some(x) = v.minor {
                        version.1 = x;
                    }
                    if let Some(x) = v.patch {
                        version.2 = x;
                    }
                }
                None => log::error!("pilatus: cannot parse camserver version reply"),
            }
            ctx.set_i32(ctx.ad.status, ADStatus::Idle as i32);
        } else {
            ctx.set_i32(ctx.ad.status, ADStatus::Error as i32);
        }
        *first_status_call = false;
    }

    let status = ctx.cam_write_read("thread", 1.0).await;
    if !status.is_err() {
        // C reuses the same `temp` / `humid` locals for every channel, so a
        // partial `sscanf` leaves the previous channel's value in place.
        let mut temp = 0.0f32;
        let mut humid = 0.0f32;
        let reply = ctx.from().to_string();

        if let Some((t, h)) = parse_thread_channel(&reply, 0) {
            if let Some(v) = t {
                temp = v;
            }
            if let Some(v) = h {
                humid = v;
            }
            ctx.set_f64(ctx.p.th_temp_0, temp as f64);
            ctx.set_f64(ctx.p.th_humid_0, humid as f64);
            ctx.set_f64(ctx.ad.temperature, temp as f64);
        }
        if let Some((t, h)) = parse_thread_channel(&reply, 1) {
            if let Some(v) = t {
                temp = v;
            }
            if let Some(v) = h {
                humid = v;
            }
            ctx.set_f64(ctx.p.th_temp_1, temp as f64);
            ctx.set_f64(ctx.p.th_humid_1, humid as f64);
        }
        if let Some((t, h)) = parse_thread_channel(&reply, 2) {
            if let Some(v) = t {
                temp = v;
            }
            if let Some(v) = h {
                humid = v;
            }
            ctx.set_f64(ctx.p.th_temp_2, temp as f64);
            ctx.set_f64(ctx.p.th_humid_2, humid as f64);
        }
        // Fixed (upstream-c-defects.md #10): C has a fourth block that parses a
        // "Channel 3" line and writes its temperature/humidity into ThTemp0 /
        // ThHumid0 — a copy-paste of the channel-0 targets that silently
        // corrupts channel 0's readings. The camserver `thread` reply defines
        // only channels 0-2 and there is no channel-3 parameter, so the block
        // is removed rather than remapped (a channel-3 parameter would be
        // fabricated hardware).
    } else {
        ctx.set_i32(ctx.ad.status, ADStatus::Error as i32);
    }
    ctx.callbacks().await;
}

/// C `readBadPixelFile()`.
async fn read_bad_pixel_file(w: &mut Worker, file: &str) {
    let nx = w.ctx.get_i32(w.ctx.ad.base.array_size_x).await.unwrap_or(0);
    let reason = w.ctx.p.num_bad_pixels;
    w.ctx.set_i32(reason, 0);
    w.shared.lock().bad_pixels.clear();

    if file.is_empty() {
        w.ctx.callbacks().await;
        return;
    }
    let text = match std::fs::read_to_string(file) {
        Ok(t) => t,
        Err(e) => {
            log::error!("pilatus: cannot open bad pixel file {file}: {e}");
            w.ctx.callbacks().await;
            return;
        }
    };
    match parse_bad_pixel_file(&text, nx) {
        Ok(map) => {
            let n = map.len() as i32;
            w.shared.lock().bad_pixels = map;
            w.ctx.set_i32(reason, n);
        }
        // C returns without updating NUM_BAD_PIXELS, which it zeroed above.
        Err(e) => log::error!("pilatus: bad pixel file {file}: {e}"),
    }
    w.ctx.callbacks().await;
}

/// C `readFlatFieldFile()`.
async fn read_flat_field_file(w: &mut Worker, file: &str) {
    let valid = w.ctx.p.flat_field_valid;
    w.ctx.set_i32(valid, 0);
    let min_flat_field = w.ctx.get_i32(w.ctx.p.min_flat_field).await.unwrap_or(0);
    if file.is_empty() {
        w.ctx.callbacks().await;
        return;
    }

    let (nx, ny) = max_size(&mut w.ctx).await;
    let bad_pixels = w.shared.lock().bad_pixels.clone();
    let read = read_image_file(&mut w.ctx, file, None, 0.0, nx, ny, &bad_pixels).await;
    let (mut data, _desc) = match read {
        Ok(v) => v,
        Err(_) => {
            log::error!("pilatus: error reading flat field file {file}");
            w.ctx.callbacks().await;
            return;
        }
    };

    let Some(average) = flat_field_average(&data, min_flat_field) else {
        // Fixed (upstream-c-defects.md #11): no pixel reached MinFlatField, so
        // the average would be 0/0 = NaN and would poison every corrected
        // pixel. Skip normalization: leave FlatFieldValid = 0 (set above) and
        // do not publish a NaN-based flat field.
        log::error!(
            "pilatus: flat field file {file} has no pixel >= MinFlatField \
             {min_flat_field}; flat field not applied"
        );
        w.ctx.callbacks().await;
        return;
    };
    apply_flat_field_floor(&mut data, min_flat_field, average);
    {
        let mut s = w.shared.lock();
        s.flat_field = data.clone();
        s.average_flat_field = average;
    }

    // C `doCallbacksGenericPointer(pFlatField, NDArrayData, 0)`.
    let array = build_array(0, nx, ny, data, None);
    w.output.publish(Arc::new(array)).await;

    w.ctx.set_i32(valid, 1);
    w.ctx.callbacks().await;
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

/// C `waitForFileToExist()`.
async fn wait_for_file_to_exist(
    ctx: &mut Ctx,
    file: &Path,
    start_time: Option<SystemTime>,
    timeout: f64,
) -> CamStatus {
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
                        log::error!("pilatus: error calling fstat on {}: {e}", file.display());
                        return CamStatus::Error;
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
        if ctx.stop.wait_timeout(secs(FILE_READ_DELAY)) {
            ctx.set_str(ctx.ad.status_message, "Acquisition aborted");
            ctx.set_i32(ctx.ad.status, ADStatus::Aborted as i32);
            return CamStatus::Error;
        }
        delta_time = t_start.elapsed().as_secs_f64();
    }

    if !opened {
        log::error!(
            "pilatus: timeout waiting for file to be created {}",
            file.display()
        );
        if file_exists {
            ctx.set_str(
                ctx.ad.status_message,
                "Image file is more than 10 seconds old",
            );
        } else {
            ctx.set_str(
                ctx.ad.status_message,
                "Timeout waiting for file to be created",
            );
        }
        return CamStatus::Error;
    }
    CamStatus::Success
}

/// C `readTiff()` — wait for the file, then retry the decode until the writer
/// has finished, and finally apply the bad-pixel map.
///
/// Fixed (upstream-c-defects.md #8): when the retry loop exhausts `timeout`
/// without a successful decode, C falls through and returns `asynSuccess` with
/// a buffer it never filled — the caller then publishes stale/garbage pixels.
/// That is a failed read, so this returns [`CamStatus::Error`] and the caller
/// does not publish.
async fn read_tiff(
    ctx: &mut Ctx,
    file: &Path,
    start_time: Option<SystemTime>,
    timeout: f64,
    nx: usize,
    ny: usize,
    bad_pixels: &[BadPixel],
) -> Result<(Vec<i32>, Option<String>), CamStatus> {
    let t_start = Instant::now();
    let mut delta_time = 0.0f64;

    let status = wait_for_file_to_exist(ctx, file, start_time, timeout).await;
    if status.is_err() {
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
            Ok(img) => log::error!(
                "pilatus: image size incorrect = {}x{}, should be {nx}x{ny}",
                img.width,
                img.height
            ),
            Err(e) => log::debug!("pilatus: error reading TIFF file {}: {e}", file.display()),
        }
        // Sleep, but check for the stop event so a long acquisition can abort.
        if ctx.stop.wait_timeout(secs(FILE_READ_DELAY)) {
            ctx.set_i32(ctx.ad.status, ADStatus::Aborted as i32);
            return Err(CamStatus::Error);
        }
        delta_time = t_start.elapsed().as_secs_f64();
    }

    let Some(img) = decoded else {
        // Fixed (upstream-c-defects.md #8): the retry loop timed out without a
        // valid frame. Do not fabricate a buffer or publish it.
        log::error!(
            "pilatus: timeout reading TIFF file {}; image not published",
            file.display()
        );
        return Err(CamStatus::Error);
    };
    let (mut data, description) = (img.data, img.description);
    correct_bad_pixels(&mut data, bad_pixels);
    Ok((data, description))
}

/// C `readImageFile()`.
///
/// Deviation: `.cbf` is not supported. C links CBFlib and calls `readCbf`;
/// no equivalent Rust crate is vendored here, so `.cbf` is rejected.
async fn read_image_file(
    ctx: &mut Ctx,
    file: &str,
    start_time: Option<SystemTime>,
    timeout: f64,
    nx: usize,
    ny: usize,
    bad_pixels: &[BadPixel],
) -> Result<(Vec<i32>, Option<String>), CamStatus> {
    if ends_with_ignore_case(file, ".tif") || ends_with_ignore_case(file, ".tiff") {
        read_tiff(
            ctx,
            Path::new(file),
            start_time,
            timeout,
            nx,
            ny,
            bad_pixels,
        )
        .await
    } else if ends_with_ignore_case(file, ".cbf") {
        log::error!("pilatus: CBF image files are not supported by this port, fileName={file}");
        ctx.set_str(ctx.ad.status_message, "CBF files are not supported");
        Err(CamStatus::Error)
    } else {
        log::error!(
            "pilatus: unsupported image file name extension, expected .tif or .cbf, fileName={file}"
        );
        ctx.set_str(
            ctx.ad.status_message,
            "Unsupported file extension, expected .tif or .cbf",
        );
        Err(CamStatus::Error)
    }
}

fn build_array(
    unique_id: i32,
    nx: usize,
    ny: usize,
    data: Vec<i32>,
    description: Option<String>,
) -> NDArray {
    let ts = EpicsTimestamp::now();
    let buffer = NDDataBuffer::I32(data);
    let mut attributes = NDAttributeList::new();
    if let Some(d) = description {
        attributes.add(NDAttribute::new_static(
            "TIFFImageDescription",
            "TIFFImageDescription",
            NDAttrSource::Driver,
            NDAttrValue::String(d),
        ));
    }
    NDArray {
        unique_id,
        timestamp: ts,
        time_stamp: ts.as_f64(),
        dims: vec![NDDimension::new(nx), NDDimension::new(ny)],
        data_size: buffer.total_bytes(),
        pool_id: 0,
        data: buffer,
        attributes,
        codec: None,
    }
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

    let full = match format_file_template(&template, &file_path, &file_name, file_number) {
        Some(s) => s,
        None => {
            log::error!("pilatus: cannot expand NDFileTemplate {template:?}");
            String::new()
        }
    };
    if auto_increment != 0 {
        ctx.set_i32(base.file_number, file_number + 1);
    }
    full
}

async fn acq_loop(mut w: Worker, start: Arc<Event>) {
    let mut status = CamStatus::Success;
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
            if !status.is_err() {
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
        w.ctx
            .cam_write_read(
                &cmd_mx_f64("Start_angle", start_angle),
                CAMSERVER_DEFAULT_TIMEOUT,
            )
            .await;

        // Send the header string. This needs to be sent for each exposure.
        let header = w
            .ctx
            .get_str(w.ctx.p.header_string, MAX_HEADER_STRING_LEN)
            .await;
        w.ctx
            .cam_write_read(&cmd_header_string(&header), CAMSERVER_DEFAULT_TIMEOUT)
            .await;

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
        w.ctx.cam_write_read(&exposure_cmd, 2.0).await;
        // Do another read in case there is an ERR string at the end of the
        // input buffer.
        status = w.ctx.cam_read(0.0).await;

        let mut array_callbacks = 0;
        multiple_file_next_image = 0;

        if status.code() > 1 {
            acquire = false;
        } else {
            // The timeout was expected.
            status = CamStatus::Success;
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
                status = w.ctx.cam_read(timeout + CAMSERVER_ACQUIRE_TIMEOUT).await;
                if status.is_err() {
                    acquire = false;
                    aborted = true;
                    if status == CamStatus::Timeout {
                        let reason = w.ctx.ad.status_message;
                        w.ctx
                            .set_str(reason, "Timeout waiting for camserver response");
                        w.ctx
                            .cam_write_read("camcmd k", CAMSERVER_DEFAULT_TIMEOUT)
                            .await;
                        w.ctx.cam_write("K", CAMSERVER_DEFAULT_TIMEOUT).await;
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
                let read = read_image_file(
                    &mut w.ctx,
                    &full_file_name,
                    Some(start_time),
                    (num_exposures as f64 * acquire_time) + read_image_file_timeout,
                    nx,
                    ny,
                    &bad_pixels,
                )
                .await;
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
                    apply_flat_field(&mut data, &s.flat_field, s.average_flat_field);
                }

                let array = build_array(image_counter, nx, ny, data, description);
                w.output.publish(Arc::new(array)).await;
            }

            if num_images == 1 {
                if mode == Some(TriggerMode::Alignment) {
                    w.ctx
                        .cam_write_read(&cmd_exposure(TriggerMode::Alignment, &full_file_name), 2.0)
                        .await;
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
        if num_images > 1 && status == CamStatus::Success {
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
            status = w.ctx.cam_read(timeout).await;
            // On a timeout camserver could still be acquiring, so send a stop.
            if status == CamStatus::Timeout {
                let reason = w.ctx.ad.status_message;
                w.ctx
                    .set_str(reason, "Timeout waiting for camserver response");
                w.ctx
                    .cam_write_read("camcmd k", CAMSERVER_DEFAULT_TIMEOUT)
                    .await;
                w.ctx.cam_write("K", CAMSERVER_DEFAULT_TIMEOUT).await;
                aborted = true;
            }
        }

        let status_param = w.ctx.get_i32(w.ctx.ad.status).await.unwrap_or(0);
        if !status.is_err() {
            w.ctx.set_i32(w.ctx.ad.status, ADStatus::Idle as i32);
        } else if status_param != ADStatus::Aborted as i32 {
            w.ctx.set_i32(w.ctx.ad.status, ADStatus::Error as i32);
        }
        w.ctx.callbacks().await;

        w.ctx.set_shutter(false).await;
        w.ctx.set_i32(w.ctx.ad.acquire, 0);
        w.ctx.set_i32(w.ctx.ad.acquire_busy, 0);
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

    #[test]
    fn build_array_carries_tiff_description() {
        let a = build_array(7, 2, 2, vec![1, 2, 3, 4], Some("desc".into()));
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
