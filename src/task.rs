use std::ffi::CString;
use std::sync::Arc;
use std::time::Duration;

use asyn_rs::port_handle::PortHandle;

use ad_core::color::NDColorMode;
use ad_core::driver::{ADStatus, ImageMode};
use ad_core::attributes::NDAttributeList;
use ad_core::ndarray::{NDArray, NDDataBuffer, NDDataType, NDDimension};
use ad_core::params::ADBaseParams;
use ad_core::plugin::channel::{NDArrayOutput, QueuedArrayCounter};

use realsense_rust::config::Config;
use realsense_rust::context::Context;
use realsense_rust::frame::{ColorFrame, DepthFrame, AccelFrame, GyroFrame, CompositeFrame};
use realsense_rust::kind::{Rs2CameraInfo, Rs2Format, Rs2Option, Rs2StreamKind};
use realsense_rust::pipeline::InactivePipeline;

use asyn_rs::request::RequestOp;
use asyn_rs::user::AsynUser;

use crate::params::{D435iConfigSnapshot, D435iParams};
use crate::types::{AcqCommand, DirtyFlags};

/// Helper: write a string param via the no-wait path.
fn write_string_no_wait(handle: &PortHandle, reason: usize, addr: i32, value: String) {
    let user = AsynUser::new(reason).with_addr(addr);
    handle.submit_no_wait(RequestOp::OctetWrite { data: value.into_bytes() }, user);
}

/// Bundled state for the acquisition task thread.
pub(crate) struct AcquisitionContext {
    pub acq_rx: std::sync::mpsc::Receiver<AcqCommand>,
    // Color port
    pub color_handle: PortHandle,
    pub color_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    pub color_queued: Arc<QueuedArrayCounter>,
    // Depth port
    pub depth_handle: PortHandle,
    pub depth_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    pub depth_queued: Arc<QueuedArrayCounter>,
    // Shared
    pub dirty: Arc<parking_lot::Mutex<DirtyFlags>>,
    pub color_ad: ADBaseParams,
    pub depth_ad: ADBaseParams,
    pub rs_params: D435iParams,
    /// Device serial number (set at create time, not read from params).
    pub serial: String,
}

impl AcquisitionContext {
    fn end_acquisition(&self, wait_for_plugins: bool) {
        if wait_for_plugins {
            self.color_queued.wait_until_zero(Duration::from_secs(5));
            self.depth_queued.wait_until_zero(Duration::from_secs(5));
        }
        let _ = self.color_handle.write_int32_blocking(self.color_ad.acquire_busy, 0, 0);
        let _ = self.color_handle.write_int32_blocking(self.color_ad.status, 0, ADStatus::Idle as i32);
        let _ = self.color_handle.write_int32_blocking(self.color_ad.acquire, 0, 0);
        let _ = self.color_handle.call_param_callbacks_blocking(0);

        let _ = self.color_handle.write_int32_blocking(self.rs_params.rs_connected, 0, 0);
        let _ = self.color_handle.call_param_callbacks_blocking(0);
    }
}

/// Start the acquisition task thread.
pub(crate) fn start_acquisition_task(ctx: AcquisitionContext) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("D435iTask".into())
        .spawn(move || acquisition_loop(ctx))
        .expect("failed to spawn D435iTask thread")
}

fn build_config(config: &D435iConfigSnapshot) -> anyhow::Result<Config> {
    let mut cfg = Config::new();
    let w = config.res_x as usize;
    let h = config.res_y as usize;
    let fps = config.frame_rate as usize;

    cfg.enable_stream(Rs2StreamKind::Color, None, w, h, Rs2Format::Rgb8, fps)?;
    cfg.enable_stream(Rs2StreamKind::Depth, None, w, h, Rs2Format::Z16, fps)?;
    cfg.enable_stream(Rs2StreamKind::Accel, None, 0, 0, Rs2Format::MotionXyz32F, 0)?;
    cfg.enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::MotionXyz32F, 0)?;

    if !config.serial.is_empty() {
        let serial_cstr = CString::new(config.serial.as_str())
            .map_err(|e| anyhow::anyhow!("invalid serial string: {e}"))?;
        cfg.enable_device_from_serial(&serial_cstr)?;
    }

    Ok(cfg)
}

fn apply_sensor_options(composite: &CompositeFrame, config: &D435iConfigSnapshot) {
    use realsense_rust::frame::FrameEx;

    // Color sensor options (exposure, gain, auto-exposure)
    let color_frames: Vec<ColorFrame> = composite.frames_of_type();
    if let Some(color_frame) = color_frames.first() {
        if let Ok(mut sensor) = FrameEx::sensor(color_frame) {
            if config.auto_exposure {
                let _ = sensor.set_option(Rs2Option::EnableAutoExposure, 1.0);
            } else {
                let _ = sensor.set_option(Rs2Option::EnableAutoExposure, 0.0);
                let _ = sensor.set_option(Rs2Option::Exposure, config.exposure as f32);
                let _ = sensor.set_option(Rs2Option::Gain, config.gain as f32);
            }
        }
    }

    // Depth sensor options (emitter, laser power)
    let depth_frames: Vec<DepthFrame> = composite.frames_of_type();
    if let Some(depth_frame) = depth_frames.first() {
        if let Ok(mut sensor) = FrameEx::sensor(depth_frame) {
            let _ = sensor.set_option(
                Rs2Option::EmitterEnabled,
                if config.emitter_enabled { 1.0 } else { 0.0 },
            );
            if config.emitter_enabled {
                let _ = sensor.set_option(Rs2Option::LaserPower, config.laser_power as f32);
            }
        }
    }
}

fn update_device_info(ctx: &AcquisitionContext, composite: &CompositeFrame) {
    use realsense_rust::frame::FrameEx;

    let color_frames: Vec<ColorFrame> = composite.frames_of_type();
    if let Some(color_frame) = color_frames.first() {
        if let Ok(sensor) = FrameEx::sensor(color_frame) {
            if let Ok(device) = sensor.device() {
                if let Some(serial) = device.info(Rs2CameraInfo::SerialNumber) {
                    let s = serial.to_string_lossy().into_owned();
                    write_string_no_wait(&ctx.color_handle, ctx.color_ad.base.serial_number, 0, s.clone());
                    write_string_no_wait(&ctx.color_handle, ctx.rs_params.rs_serial, 0, s);
                }
                if let Some(fw) = device.info(Rs2CameraInfo::FirmwareVersion) {
                    write_string_no_wait(&ctx.color_handle, ctx.color_ad.base.firmware_version, 0, fw.to_string_lossy().into_owned());
                }
                if let Some(name) = device.info(Rs2CameraInfo::Name) {
                    write_string_no_wait(&ctx.color_handle, ctx.color_ad.base.model, 0, name.to_string_lossy().into_owned());
                }
            }
        }
    }
}

/// Copy frame data, stripping any row stride padding.
/// Returns a tightly-packed buffer of `row_bytes * h` bytes.
fn copy_frame_data(frame_ptr: *const u8, stride: usize, row_bytes: usize, h: usize) -> Vec<u8> {
    if stride == row_bytes {
        // No padding — fast path
        let total = row_bytes * h;
        unsafe { std::slice::from_raw_parts(frame_ptr, total) }.to_vec()
    } else {
        // Strip per-row padding
        let mut buf = Vec::with_capacity(row_bytes * h);
        for row in 0..h {
            let offset = row * stride;
            let row_slice = unsafe { std::slice::from_raw_parts(frame_ptr.add(offset), row_bytes) };
            buf.extend_from_slice(row_slice);
        }
        buf
    }
}

fn process_color_frame(
    composite: &CompositeFrame,
    ctx: &AcquisitionContext,
    array_counter: i32,
) {
    let color_frames: Vec<ColorFrame> = composite.frames_of_type();
    if let Some(frame) = color_frames.first() {
        let w = frame.width();
        let h = frame.height();
        let stride = frame.stride();
        let row_bytes = w * 3; // RGB8: 3 bytes per pixel

        let ptr = unsafe { frame.get_data() as *const std::os::raw::c_void as *const u8 };
        let bytes = copy_frame_data(ptr, stride, row_bytes, h);

        let data = NDDataBuffer::U8(bytes);
        // RGB1 layout: [3, width, height]
        let dims = vec![
            NDDimension::new(3),
            NDDimension::new(w),
            NDDimension::new(h),
        ];

        let ts = ad_core::timestamp::EpicsTimestamp::now();
        let array = NDArray {
            unique_id: array_counter,
            timestamp: ts,
            dims,
            data,
            attributes: NDAttributeList::new(),
            codec: None,
        };

        // Update counters
        ctx.color_handle.write_int32_no_wait(ctx.color_ad.base.array_counter, 0, array_counter);
        ctx.color_handle.write_float64_no_wait(ctx.color_ad.base.timestamp_rbv, 0, ts.as_f64());
        ctx.color_handle.write_int32_no_wait(ctx.color_ad.base.epics_ts_sec, 0, ts.sec as i32);
        ctx.color_handle.write_int32_no_wait(ctx.color_ad.base.epics_ts_nsec, 0, ts.nsec as i32);

        // Update array size and type info
        ctx.color_handle.write_int32_no_wait(ctx.color_ad.base.array_size_x, 0, w as i32);
        ctx.color_handle.write_int32_no_wait(ctx.color_ad.base.array_size_y, 0, h as i32);
        ctx.color_handle.write_int32_no_wait(ctx.color_ad.base.array_size_z, 0, 3);
        ctx.color_handle.write_int32_no_wait(ctx.color_ad.base.n_dimensions, 0, 3);
        ctx.color_handle.write_int32_no_wait(ctx.color_ad.base.color_mode, 0, NDColorMode::RGB1 as i32);
        ctx.color_handle.write_int32_no_wait(ctx.color_ad.base.data_type, 0, NDDataType::UInt8 as u8 as i32);

        let _ = ctx.color_handle.call_param_callbacks_blocking(0);

        ctx.color_output.lock().publish(Arc::new(array));
    }
}

fn process_depth_frame(
    composite: &CompositeFrame,
    ctx: &AcquisitionContext,
    array_counter: i32,
) {
    let depth_frames: Vec<DepthFrame> = composite.frames_of_type();
    if let Some(frame) = depth_frames.first() {
        let w = frame.width();
        let h = frame.height();
        let stride = frame.stride();
        let row_bytes = w * 2; // Z16: 2 bytes per pixel

        let ptr = unsafe { frame.get_data() as *const std::os::raw::c_void as *const u8 };
        let bytes = copy_frame_data(ptr, stride, row_bytes, h);

        // Reinterpret as u16
        let pixels: Vec<u16> = bytes
            .chunks_exact(2)
            .map(|c| u16::from_ne_bytes([c[0], c[1]]))
            .collect();

        let data = NDDataBuffer::U16(pixels);
        // Mono layout: [width, height]
        let dims = vec![
            NDDimension::new(w),
            NDDimension::new(h),
        ];

        let ts = ad_core::timestamp::EpicsTimestamp::now();
        let array = NDArray {
            unique_id: array_counter,
            timestamp: ts,
            dims,
            data,
            attributes: NDAttributeList::new(),
            codec: None,
        };

        // Update counters on depth port
        ctx.depth_handle.write_int32_no_wait(ctx.depth_ad.base.array_counter, 0, array_counter);
        ctx.depth_handle.write_float64_no_wait(ctx.depth_ad.base.timestamp_rbv, 0, ts.as_f64());
        ctx.depth_handle.write_int32_no_wait(ctx.depth_ad.base.epics_ts_sec, 0, ts.sec as i32);
        ctx.depth_handle.write_int32_no_wait(ctx.depth_ad.base.epics_ts_nsec, 0, ts.nsec as i32);

        // Update array size and type info
        ctx.depth_handle.write_int32_no_wait(ctx.depth_ad.base.array_size_x, 0, w as i32);
        ctx.depth_handle.write_int32_no_wait(ctx.depth_ad.base.array_size_y, 0, h as i32);
        ctx.depth_handle.write_int32_no_wait(ctx.depth_ad.base.array_size_z, 0, 0);
        ctx.depth_handle.write_int32_no_wait(ctx.depth_ad.base.n_dimensions, 0, 2);
        ctx.depth_handle.write_int32_no_wait(ctx.depth_ad.base.color_mode, 0, NDColorMode::Mono as i32);
        ctx.depth_handle.write_int32_no_wait(ctx.depth_ad.base.data_type, 0, NDDataType::UInt16 as u8 as i32);

        let _ = ctx.depth_handle.call_param_callbacks_blocking(0);

        ctx.depth_output.lock().publish(Arc::new(array));

        // Read depth units and publish to color port (since RS params are on color port)
        if let Ok(units) = frame.depth_units() {
            ctx.color_handle.write_float64_no_wait(ctx.rs_params.rs_depth_units, 0, units as f64);
        }
    }
}

fn process_imu(composite: &CompositeFrame, ctx: &AcquisitionContext) {
    let accel_frames: Vec<AccelFrame> = composite.frames_of_type();
    if let Some(accel) = accel_frames.first() {
        let a = accel.acceleration();
        ctx.color_handle.write_float64_no_wait(ctx.rs_params.rs_accel_x, 0, a[0] as f64);
        ctx.color_handle.write_float64_no_wait(ctx.rs_params.rs_accel_y, 0, a[1] as f64);
        ctx.color_handle.write_float64_no_wait(ctx.rs_params.rs_accel_z, 0, a[2] as f64);
    }

    let gyro_frames: Vec<GyroFrame> = composite.frames_of_type();
    if let Some(gyro) = gyro_frames.first() {
        let g = gyro.rotational_velocity();
        ctx.color_handle.write_float64_no_wait(ctx.rs_params.rs_gyro_x, 0, g[0] as f64);
        ctx.color_handle.write_float64_no_wait(ctx.rs_params.rs_gyro_y, 0, g[1] as f64);
        ctx.color_handle.write_float64_no_wait(ctx.rs_params.rs_gyro_z, 0, g[2] as f64);
    }
}

fn acquisition_loop(ctx: AcquisitionContext) {
    loop {
        // Wait for Start command
        match ctx.acq_rx.recv() {
            Ok(AcqCommand::Start) => {}
            Ok(AcqCommand::Stop) => continue,
            Err(_) => break,
        }

        // Initialize counters
        let _ = ctx.color_handle.write_int32_blocking(ctx.color_ad.num_images_counter, 0, 0);
        let _ = ctx.color_handle.write_int32_blocking(ctx.color_ad.status, 0, ADStatus::Acquire as i32);
        let _ = ctx.color_handle.write_int32_blocking(ctx.color_ad.acquire_busy, 0, 1);

        let mut num_counter = 0i32;
        let mut color_array_counter = ctx.color_handle
            .read_int32_blocking(ctx.color_ad.base.array_counter, 0)
            .unwrap_or(0);
        let mut depth_array_counter = ctx.depth_handle
            .read_int32_blocking(ctx.depth_ad.base.array_counter, 0)
            .unwrap_or(0);

        // Read initial config
        let mut config = match D435iConfigSnapshot::read_via_handle(
            &ctx.color_handle,
            &ctx.color_ad,
            &ctx.rs_params,
            &ctx.serial,
        ) {
            Ok(cfg) => cfg,
            Err(e) => {
                eprintln!("D435i: failed to read config: {e}");
                ctx.end_acquisition(false);
                continue;
            }
        };

        // Create RealSense context and pipeline
        let rs_ctx = match Context::new() {
            Ok(c) => c,
            Err(e) => {
                eprintln!("D435i: failed to create RealSense context: {e}");
                ctx.end_acquisition(false);
                continue;
            }
        };

        let rs_pipeline = match InactivePipeline::try_from(&rs_ctx) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("D435i: failed to create pipeline: {e}");
                ctx.end_acquisition(false);
                continue;
            }
        };

        let rs_config = match build_config(&config) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("D435i: failed to build config: {e}");
                ctx.end_acquisition(false);
                continue;
            }
        };

        let mut pipeline = match rs_pipeline.start(Some(rs_config)) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("D435i: failed to start pipeline: {e}");
                ctx.end_acquisition(false);
                continue;
            }
        };

        // Mark connected
        let _ = ctx.color_handle.write_int32_blocking(ctx.rs_params.rs_connected, 0, 1);
        let _ = ctx.color_handle.call_param_callbacks_blocking(0);

        let mut first_frame = true;
        let mut sensor_options_applied = false;
        let mut consecutive_errors: u32 = 0;
        const MAX_CONSECUTIVE_ERRORS: u32 = 50;

        // Inner acquisition loop
        loop {
            // Check dirty flags
            let dirty_flags = ctx.dirty.lock().take();

            if dirty_flags.reconfigure_pipeline {
                // Need to restart pipeline with new config
                config = match D435iConfigSnapshot::read_via_handle(
                    &ctx.color_handle,
                    &ctx.color_ad,
                    &ctx.rs_params,
                    &ctx.serial,
                ) {
                    Ok(cfg) => cfg,
                    Err(_) => break,
                };

                let inactive = pipeline.stop();
                let new_config = match build_config(&config) {
                    Ok(c) => c,
                    Err(e) => {
                        eprintln!("D435i: failed to rebuild config: {e}");
                        break;
                    }
                };
                pipeline = match inactive.start(Some(new_config)) {
                    Ok(p) => p,
                    Err(e) => {
                        eprintln!("D435i: failed to restart pipeline: {e}");
                        break;
                    }
                };
                sensor_options_applied = false;
                first_frame = true;
            } else if dirty_flags.update_sensor_options {
                config = match D435iConfigSnapshot::read_via_handle(
                    &ctx.color_handle,
                    &ctx.color_ad,
                    &ctx.rs_params,
                    &ctx.serial,
                ) {
                    Ok(cfg) => cfg,
                    Err(_) => break,
                };
                sensor_options_applied = false;
            }

            // Wait for frames
            let composite = match pipeline.wait(Some(Duration::from_secs(5))) {
                Ok(f) => {
                    consecutive_errors = 0;
                    f
                }
                Err(e) => {
                    consecutive_errors += 1;
                    // Log first error, then every 10th to avoid spam
                    if consecutive_errors == 1 || consecutive_errors % 10 == 0 {
                        eprintln!(
                            "D435i: frame wait error ({consecutive_errors}x): {e}"
                        );
                    }
                    if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                        eprintln!(
                            "D435i: {} consecutive frame errors, stopping acquisition",
                            consecutive_errors
                        );
                        break;
                    }
                    // Check for stop
                    if let Ok(AcqCommand::Stop) = ctx.acq_rx.try_recv() {
                        break;
                    }
                    // Backoff: sleep up to 2 seconds based on error count
                    let backoff = Duration::from_millis(
                        100 * (consecutive_errors as u64).min(20)
                    );
                    std::thread::sleep(backoff);
                    continue;
                }
            };

            // On first frame, update device info
            if first_frame {
                update_device_info(&ctx, &composite);
                first_frame = false;
            }

            // Apply sensor options on first frame after config change
            if !sensor_options_applied {
                apply_sensor_options(&composite, &config);
                sensor_options_applied = true;
            }

            // Process frames
            num_counter += 1;
            color_array_counter += 1;
            depth_array_counter += 1;

            ctx.color_handle.write_int32_no_wait(ctx.color_ad.num_images_counter, 0, num_counter);

            if config.array_callbacks {
                process_color_frame(&composite, &ctx, color_array_counter);
                process_depth_frame(&composite, &ctx, depth_array_counter);
            }

            process_imu(&composite, &ctx);
            let _ = ctx.color_handle.call_param_callbacks_blocking(0);

            // Check stop conditions
            if config.image_mode == ImageMode::Single
                || (config.image_mode == ImageMode::Multiple && num_counter >= config.num_images)
            {
                break;
            }

            // Check for stop command (non-blocking)
            match ctx.acq_rx.try_recv() {
                Ok(AcqCommand::Stop) => break,
                Ok(AcqCommand::Start) => {} // stale
                Err(_) => {}
            }
        }

        // Pipeline is dropped here (ActivePipeline::drop calls rs2_delete_pipeline)
        ctx.end_acquisition(config.wait_for_plugins);
    }
}
