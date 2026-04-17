use std::ffi::CString;
use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::port_handle::PortHandle;

use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::attributes::NDAttributeList;
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{NDArrayOutput, QueuedArrayCounter};
use epics_rs::ad_core::runtime as rt;

use realsense_rust::config::Config;
use realsense_rust::context::Context;
use realsense_rust::frame::{ColorFrame, DepthFrame, AccelFrame, GyroFrame, CompositeFrame};
use realsense_rust::kind::{Rs2CameraInfo, Rs2Format, Rs2Option, Rs2StreamKind};
use realsense_rust::pipeline::InactivePipeline;
use realsense_rust::processing_blocks::align::Align;
use realsense_rust::processing_blocks::decimation::Decimation;
use realsense_rust::processing_blocks::spatial_filter::SpatialFilter;
use realsense_rust::processing_blocks::temporal_filter::TemporalFilter;
use realsense_rust::processing_blocks::hole_filling::HoleFillingFilter;
use realsense_rust::processing_blocks::pointcloud::PointCloud;
use realsense_rust::processing_blocks::options::{
    DecimationOptions, SpatialFilterOptions, TemporalFilterOptions, HoleFillingOptions,
};

use epics_rs::asyn::request::ParamSetValue;

use crate::params::{D435iConfigSnapshot, D435iParams};
use crate::types::{AcqCommand, DirtyFlags};

const MAX_CONSECUTIVE_ERRORS: u32 = 50;
const MAX_CONNECT_RETRIES: u32 = 10;

/// Helper: update a single string param via the notify path.
async fn write_string(handle: &PortHandle, reason: usize, addr: i32, value: String) {
    let _ = handle.set_params_and_notify(
        addr,
        vec![ParamSetValue::Octet { reason, addr, value }],
    ).await;
}

/// Bundled state for the acquisition task thread.
pub(crate) struct AcquisitionContext {
    pub acq_rx: rt::CommandReceiver<AcqCommand>,
    // Color port
    pub color_handle: PortHandle,
    pub color_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    pub color_queued: Arc<QueuedArrayCounter>,
    // Depth port
    pub depth_handle: PortHandle,
    pub depth_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    pub depth_queued: Arc<QueuedArrayCounter>,
    // Pointcloud output
    pub pc_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    #[allow(dead_code)] // kept alive for QueuedArrayCounter
    pub pc_queued: Arc<QueuedArrayCounter>,
    // Shared
    pub dirty: Arc<parking_lot::Mutex<DirtyFlags>>,
    pub color_ad: ADBaseParams,
    pub depth_ad: ADBaseParams,
    pub rs_params: D435iParams,
    /// Device serial number (set at create time, not read from params).
    pub serial: String,
}

impl AcquisitionContext {
    async fn end_acquisition(&self, wait_for_plugins: bool) {
        if wait_for_plugins {
            self.color_queued.wait_until_zero(Duration::from_secs(5));
            self.depth_queued.wait_until_zero(Duration::from_secs(5));
        }
        let _ = self.color_handle.set_params_and_notify(0, vec![
            ParamSetValue::Int32 { reason: self.color_ad.acquire_busy, addr: 0, value: 0 },
            ParamSetValue::Int32 { reason: self.color_ad.status,       addr: 0, value: ADStatus::Idle as i32 },
            ParamSetValue::Int32 { reason: self.color_ad.acquire,      addr: 0, value: 0 },
            ParamSetValue::Int32 { reason: self.rs_params.rs_connected, addr: 0, value: 0 },
        ]).await;
    }
}

/// Start the acquisition task thread via the `rt` facade.
pub(crate) fn start_acquisition_task(ctx: AcquisitionContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("D435iTask", move || acquisition_loop_async(ctx))
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

async fn update_device_info(ctx: &AcquisitionContext, composite: &CompositeFrame) {
    use realsense_rust::frame::FrameEx;

    let color_frames: Vec<ColorFrame> = composite.frames_of_type();
    if let Some(color_frame) = color_frames.first() {
        if let Ok(sensor) = FrameEx::sensor(color_frame) {
            if let Ok(device) = sensor.device() {
                if let Some(serial) = device.info(Rs2CameraInfo::SerialNumber) {
                    let s = serial.to_string_lossy().into_owned();
                    write_string(&ctx.color_handle, ctx.color_ad.base.serial_number, 0, s.clone()).await;
                    write_string(&ctx.color_handle, ctx.rs_params.rs_serial, 0, s).await;
                }
                if let Some(fw) = device.info(Rs2CameraInfo::FirmwareVersion) {
                    write_string(&ctx.color_handle, ctx.color_ad.base.firmware_version, 0, fw.to_string_lossy().into_owned()).await;
                }
                if let Some(name) = device.info(Rs2CameraInfo::Name) {
                    write_string(&ctx.color_handle, ctx.color_ad.base.model, 0, name.to_string_lossy().into_owned()).await;
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

/// Publish an NDArray through a port handle, updating counters and metadata.
async fn publish_array(
    handle: &PortHandle,
    output: &parking_lot::Mutex<NDArrayOutput>,
    base_params: &epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams,
    array: NDArray,
    color_mode: NDColorMode,
) {
    let ts = array.timestamp;
    let n_dims = array.dims.len();
    let (size_x, size_y, size_z) = match n_dims {
        2 => (array.dims[0].size as i32, array.dims[1].size as i32, 0),
        3 => (array.dims[1].size as i32, array.dims[2].size as i32, array.dims[0].size as i32),
        _ => (0, 0, 0),
    };
    let data_type = array.data.data_type();
    // ArraySize_RBV is total bytes (C ADCore NDPluginBase convention):
    //   size = prod(dims) * bytes_per_element
    let num_elements: i64 = array.dims.iter().map(|d| d.size as i64).product();
    let array_size: i32 = num_elements
        .saturating_mul(data_type.element_size() as i64)
        .min(i32::MAX as i64) as i32;

    let _ = handle.set_params_and_notify(0, vec![
        ParamSetValue::Int32   { reason: base_params.array_counter, addr: 0, value: array.unique_id },
        ParamSetValue::Float64 { reason: base_params.timestamp_rbv, addr: 0, value: ts.as_f64() },
        ParamSetValue::Int32   { reason: base_params.epics_ts_sec,  addr: 0, value: ts.sec as i32 },
        ParamSetValue::Int32   { reason: base_params.epics_ts_nsec, addr: 0, value: ts.nsec as i32 },
        ParamSetValue::Int32   { reason: base_params.array_size_x,  addr: 0, value: size_x },
        ParamSetValue::Int32   { reason: base_params.array_size_y,  addr: 0, value: size_y },
        ParamSetValue::Int32   { reason: base_params.array_size_z,  addr: 0, value: size_z },
        ParamSetValue::Int32   { reason: base_params.array_size,    addr: 0, value: array_size },
        ParamSetValue::Int32   { reason: base_params.n_dimensions,  addr: 0, value: n_dims as i32 },
        ParamSetValue::Int32   { reason: base_params.color_mode,    addr: 0, value: color_mode as i32 },
        ParamSetValue::Int32   { reason: base_params.data_type,     addr: 0, value: data_type as u8 as i32 },
    ]).await;

    // Hold the parking_lot guard across .await: safe on a current-thread runtime
    // with no concurrent tasks — no other task will try to re-acquire the lock.
    output.lock().publish(Arc::new(array)).await;
}

async fn process_color_frame(
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

        let ts = epics_rs::ad_core::timestamp::EpicsTimestamp::now();
        let array = NDArray {
            unique_id: array_counter,
            timestamp: ts,
            time_stamp: ts.as_f64(),
            dims,
            data,
            attributes: NDAttributeList::new(),
            codec: None,
        };

        publish_array(
            &ctx.color_handle,
            &ctx.color_output,
            &ctx.color_ad.base,
            array,
            NDColorMode::RGB1,
        ).await;
    }
}

async fn process_depth_frame(
    depth_frame: DepthFrame,
    ctx: &AcquisitionContext,
    array_counter: i32,
    filters: &mut DepthFilterChain,
    config: &D435iConfigSnapshot,
) {
    // Apply post-processing filters (consumes the frame, returns filtered)
    let frame = match filters.apply(depth_frame, config) {
        Some(f) => f,
        None => return, // filter error consumed the frame
    };

    let w = frame.width();
    let h = frame.height();
    let stride = frame.stride();
    let row_bytes = w * 2; // Z16: 2 bytes per pixel

    let ptr = unsafe { frame.get_data() as *const std::os::raw::c_void as *const u8 };
    let bytes = copy_frame_data(ptr, stride, row_bytes, h);

    // Reinterpret as u16 (RealSense USB protocol is little-endian)
    let pixels: Vec<u16> = bytes
        .chunks_exact(2)
        .map(|c| u16::from_le_bytes([c[0], c[1]]))
        .collect();

    let data = NDDataBuffer::U16(pixels);
    // Mono layout: [width, height]
    let dims = vec![
        NDDimension::new(w),
        NDDimension::new(h),
    ];

    let ts = epics_rs::ad_core::timestamp::EpicsTimestamp::now();
    let array = NDArray {
        unique_id: array_counter,
        timestamp: ts,
        time_stamp: ts.as_f64(),
        dims,
        data,
        attributes: NDAttributeList::new(),
        codec: None,
    };

    publish_array(
        &ctx.depth_handle,
        &ctx.depth_output,
        &ctx.depth_ad.base,
        array,
        NDColorMode::Mono,
    ).await;
}

async fn process_pointcloud(
    depth_frame: DepthFrame,
    ctx: &AcquisitionContext,
    pc_block: &mut PointCloud,
    array_counter: i32,
) {
    let w = depth_frame.width();
    let h = depth_frame.height();
    if pc_block.queue(depth_frame).is_err() {
        return;
    }
    if let Ok(points_frame) = pc_block.wait(Duration::from_millis(200)) {
        let vertices = points_frame.vertices();
        let count = points_frame.points_count();
        if count == 0 {
            return;
        }

        // Flatten vertices [N][3] → Vec<f32>
        let data: Vec<f32> = vertices.iter()
            .take(count)
            .flat_map(|v| v.xyz.iter().copied())
            .collect();

        let ts = epics_rs::ad_core::timestamp::EpicsTimestamp::now();
        let array = NDArray {
            unique_id: array_counter,
            timestamp: ts,
            time_stamp: ts.as_f64(),
            dims: vec![
                NDDimension::new(3),
                NDDimension::new(w),
                NDDimension::new(h),
            ],
            data: NDDataBuffer::F32(data),
            attributes: NDAttributeList::new(),
            codec: None,
        };

        publish_array(
            &ctx.color_handle,
            &ctx.pc_output,
            &ctx.color_ad.base,
            array,
            NDColorMode::Mono,
        ).await;
    }
}

async fn process_imu(composite: &CompositeFrame, ctx: &AcquisitionContext) {
    let mut updates: Vec<ParamSetValue> = Vec::new();

    let accel_frames: Vec<AccelFrame> = composite.frames_of_type();
    if let Some(accel) = accel_frames.first() {
        let a = accel.acceleration();
        updates.push(ParamSetValue::Float64 { reason: ctx.rs_params.rs_accel_x, addr: 0, value: a[0] as f64 });
        updates.push(ParamSetValue::Float64 { reason: ctx.rs_params.rs_accel_y, addr: 0, value: a[1] as f64 });
        updates.push(ParamSetValue::Float64 { reason: ctx.rs_params.rs_accel_z, addr: 0, value: a[2] as f64 });
    }

    let gyro_frames: Vec<GyroFrame> = composite.frames_of_type();
    if let Some(gyro) = gyro_frames.first() {
        let g = gyro.rotational_velocity();
        updates.push(ParamSetValue::Float64 { reason: ctx.rs_params.rs_gyro_x, addr: 0, value: g[0] as f64 });
        updates.push(ParamSetValue::Float64 { reason: ctx.rs_params.rs_gyro_y, addr: 0, value: g[1] as f64 });
        updates.push(ParamSetValue::Float64 { reason: ctx.rs_params.rs_gyro_z, addr: 0, value: g[2] as f64 });
    }

    if !updates.is_empty() {
        let _ = ctx.color_handle.set_params_and_notify(0, updates).await;
    }
}

/// Depth post-processing filter chain, created once per acquisition session.
struct DepthFilterChain {
    decimation: Decimation,
    spatial: SpatialFilter,
    temporal: TemporalFilter,
    hole_fill: HoleFillingFilter,
}

impl DepthFilterChain {
    fn new() -> anyhow::Result<Self> {
        Ok(Self {
            decimation: Decimation::new(1)?,
            spatial: SpatialFilter::new(1)?,
            temporal: TemporalFilter::new(1)?,
            hole_fill: HoleFillingFilter::new(1)?,
        })
    }

    /// Update filter options from config snapshot.
    fn apply_options(&mut self, config: &D435iConfigSnapshot) {
        let _ = self.decimation.apply_options(&DecimationOptions {
            filter_magnitude: Some(config.decimation_magnitude as f32),
        });
        let _ = self.spatial.apply_options(&SpatialFilterOptions {
            smooth_alpha: Some(config.spatial_alpha as f32),
            smooth_delta: Some(config.spatial_delta as f32),
            magnitude: Some(config.spatial_magnitude as f32),
            holes_fill: None,
        });
        let _ = self.temporal.apply_options(&TemporalFilterOptions {
            smooth_alpha: Some(config.temporal_alpha as f32),
            smooth_delta: Some(config.temporal_delta as f32),
            persistence_control: None,
        });
        let _ = self.hole_fill.apply_options(&HoleFillingOptions {
            holes_fill: Some(config.hole_fill_mode as f32),
        });
    }

    /// Apply enabled filters sequentially. Each filter consumes the input frame.
    /// Returns None if a filter error caused the frame to be lost.
    fn apply(&mut self, frame: DepthFrame, config: &D435iConfigSnapshot) -> Option<DepthFrame> {
        let timeout = Duration::from_millis(100);
        let mut f = frame;

        macro_rules! apply_filter {
            ($filter:expr, $name:expr, $enable:expr) => {
                if $enable {
                    $filter.queue(f).ok()?;
                    f = $filter.wait(timeout).map_err(|e| {
                        log::warn!("D435i: {} wait failed: {e}", $name);
                    }).ok()?;
                }
            };
        }

        apply_filter!(self.decimation, "decimation", config.decimation_enable);
        apply_filter!(self.spatial, "spatial", config.spatial_enable);
        apply_filter!(self.temporal, "temporal", config.temporal_enable);
        apply_filter!(self.hole_fill, "hole_fill", config.hole_fill_enable);
        Some(f)
    }
}

/// Try to connect to the camera with retries.
/// Returns Some(pipeline) on success, None if all retries exhausted or Stop received.
async fn try_connect_pipeline(
    ctx: &mut AcquisitionContext,
    config: &D435iConfigSnapshot,
) -> Option<realsense_rust::pipeline::ActivePipeline> {
    let _ = ctx.color_handle.set_params_and_notify(0, vec![
        ParamSetValue::Octet { reason: ctx.color_ad.status_message, addr: 0, value: "Connecting to camera...".into() },
        ParamSetValue::Int32 { reason: ctx.rs_params.rs_connected, addr: 0, value: 0 },
    ]).await;

    let mut retry_count = 0u32;
    loop {
        // Check for Stop command
        if matches!(ctx.acq_rx.try_recv(), Ok(AcqCommand::Stop)) {
            return None;
        }

        let result = (|| -> anyhow::Result<_> {
            let rs_ctx = Context::new()?;
            let rs_pipeline = InactivePipeline::try_from(&rs_ctx)?;
            let rs_config = build_config(config)?;
            rs_pipeline.start(Some(rs_config))
        })();

        match result {
            Ok(p) => return Some(p),
            Err(e) => {
                retry_count += 1;
                if retry_count == 1 || retry_count.is_multiple_of(5) {
                    log::warn!("D435i: connection attempt {retry_count} failed: {e}");
                }
                if retry_count >= MAX_CONNECT_RETRIES {
                    log::error!("D435i: giving up after {retry_count} connection attempts");
                    write_string(&ctx.color_handle, ctx.color_ad.status_message, 0,
                        format!("Connection failed: {e}")).await;
                    return None;
                }
                // Exponential backoff: 1s, 2s, 4s, ... max 10s, interruptible by Stop.
                let backoff = Duration::from_secs((1u64 << retry_count.min(3)).min(10));
                if matches!(
                    rt::timeout(backoff, ctx.acq_rx.recv()).await,
                    Ok(Some(AcqCommand::Stop)) | Ok(None)
                ) {
                    return None;
                }
            }
        }
    }
}

async fn acquisition_loop_async(mut ctx: AcquisitionContext) {
    loop {
        // Wait for Start command
        match ctx.acq_rx.recv().await {
            Some(AcqCommand::Start) => {}
            Some(AcqCommand::Stop) => continue,
            None => break,
        }

        // Initialize counters
        let _ = ctx.color_handle.set_params_and_notify(0, vec![
            ParamSetValue::Int32 { reason: ctx.color_ad.num_images_counter, addr: 0, value: 0 },
            ParamSetValue::Int32 { reason: ctx.color_ad.status,             addr: 0, value: ADStatus::Acquire as i32 },
            ParamSetValue::Int32 { reason: ctx.color_ad.acquire_busy,       addr: 0, value: 1 },
        ]).await;

        let mut num_counter = 0i32;
        let mut color_array_counter = ctx.color_handle
            .read_int32(ctx.color_ad.base.array_counter, 0)
            .await
            .unwrap_or(0);
        let mut depth_array_counter = ctx.depth_handle
            .read_int32(ctx.depth_ad.base.array_counter, 0)
            .await
            .unwrap_or(0);

        // Diagnostic counters
        let mut total_errors: i32 = 0;
        let mut frames_dropped: i32 = 0;

        // Read initial config
        let mut config = match D435iConfigSnapshot::read_via_handle(
            &ctx.color_handle,
            &ctx.color_ad,
            &ctx.rs_params,
            &ctx.serial,
        ).await {
            Ok(cfg) => cfg,
            Err(e) => {
                log::error!("D435i: failed to read config: {e}");
                ctx.end_acquisition(false).await;
                continue;
            }
        };

        // Connect to camera with retries
        let mut pipeline = match try_connect_pipeline(&mut ctx, &config).await {
            Some(p) => p,
            None => {
                ctx.end_acquisition(false).await;
                continue;
            }
        };

        // Mark connected
        let _ = ctx.color_handle.set_params_and_notify(0, vec![
            ParamSetValue::Int32 { reason: ctx.rs_params.rs_connected, addr: 0, value: 1 },
            ParamSetValue::Octet { reason: ctx.color_ad.status_message, addr: 0, value: "Acquiring data".into() },
        ]).await;

        // Create processing blocks
        let mut depth_filters = match DepthFilterChain::new() {
            Ok(f) => f,
            Err(e) => {
                log::error!("D435i: failed to create depth filters: {e}");
                ctx.end_acquisition(false).await;
                continue;
            }
        };
        depth_filters.apply_options(&config);

        let mut align = match Align::new(Rs2StreamKind::Color, 1) {
            Ok(a) => a,
            Err(e) => {
                log::error!("D435i: failed to create align block: {e}");
                ctx.end_acquisition(false).await;
                continue;
            }
        };

        let mut pc_block = match PointCloud::new(1) {
            Ok(p) => p,
            Err(e) => {
                log::error!("D435i: failed to create pointcloud block: {e}");
                ctx.end_acquisition(false).await;
                continue;
            }
        };

        let mut first_frame = true;
        let mut sensor_options_applied = false;
        let mut consecutive_errors: u32 = 0;

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
                ).await {
                    Ok(cfg) => cfg,
                    Err(_) => break,
                };

                let inactive = pipeline.stop();
                let new_config = match build_config(&config) {
                    Ok(c) => c,
                    Err(e) => {
                        log::error!("D435i: failed to rebuild config: {e}");
                        break;
                    }
                };
                pipeline = match inactive.start(Some(new_config)) {
                    Ok(p) => p,
                    Err(e) => {
                        log::error!("D435i: failed to restart pipeline: {e}");
                        break;
                    }
                };
                sensor_options_applied = false;
                first_frame = true;
                depth_filters.apply_options(&config);
            } else if dirty_flags.update_sensor_options {
                config = match D435iConfigSnapshot::read_via_handle(
                    &ctx.color_handle,
                    &ctx.color_ad,
                    &ctx.rs_params,
                    &ctx.serial,
                ).await {
                    Ok(cfg) => cfg,
                    Err(_) => break,
                };
                sensor_options_applied = false;
                depth_filters.apply_options(&config);
            }

            // Wait for frames (dynamic timeout based on fps)
            let frame_timeout = config.frame_timeout();
            let composite = match pipeline.wait(Some(frame_timeout)) {
                Ok(f) => {
                    consecutive_errors = 0;
                    f
                }
                Err(e) => {
                    consecutive_errors += 1;
                    frames_dropped += 1;
                    total_errors += 1;
                    let _ = ctx.color_handle.set_params_and_notify(0, vec![
                        ParamSetValue::Int32 { reason: ctx.rs_params.rs_frames_dropped, addr: 0, value: frames_dropped },
                        ParamSetValue::Int32 { reason: ctx.rs_params.rs_error_count,    addr: 0, value: total_errors },
                    ]).await;
                    write_string(&ctx.color_handle, ctx.rs_params.rs_last_error, 0,
                        format!("Frame wait: {e}")).await;

                    // Log first error, then every 10th to avoid spam
                    if consecutive_errors == 1 || consecutive_errors.is_multiple_of(10) {
                        log::warn!(
                            "D435i: frame wait error ({consecutive_errors}x): {e}"
                        );
                    }
                    if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                        log::error!(
                            "D435i: {} consecutive frame errors",
                            consecutive_errors
                        );

                        // Continuous mode: attempt reconnection
                        if config.image_mode == ImageMode::Continuous {
                            log::info!("D435i: attempting reconnection...");
                            write_string(&ctx.color_handle, ctx.color_ad.status_message, 0,
                                "Reconnecting...".into()).await;
                            drop(pipeline);

                            match try_connect_pipeline(&mut ctx, &config).await {
                                Some(p) => {
                                    pipeline = p;
                                    consecutive_errors = 0;
                                    first_frame = true;
                                    sensor_options_applied = false;
                                    let _ = ctx.color_handle.set_params_and_notify(0, vec![
                                        ParamSetValue::Int32 { reason: ctx.rs_params.rs_connected, addr: 0, value: 1 },
                                    ]).await;
                                    continue;
                                }
                                None => {
                                    total_errors += 1;
                                    let _ = ctx.color_handle.set_params_and_notify(0, vec![
                                        ParamSetValue::Int32 { reason: ctx.rs_params.rs_error_count, addr: 0, value: total_errors },
                                    ]).await;
                                    write_string(&ctx.color_handle, ctx.rs_params.rs_last_error, 0,
                                        "Reconnection failed".into()).await;
                                    break;
                                }
                            }
                        } else {
                            break;
                        }
                    }
                    // Check for stop
                    if matches!(ctx.acq_rx.try_recv(), Ok(AcqCommand::Stop)) {
                        break;
                    }
                    // Backoff: sleep up to 2 seconds based on error count
                    let backoff = Duration::from_millis(
                        100 * (consecutive_errors as u64).min(20)
                    );
                    rt::sleep(backoff).await;
                    continue;
                }
            };

            // Apply alignment if enabled (CompositeFrame → CompositeFrame)
            let composite = if config.align_enable {
                match align.queue(composite) {
                    Ok(()) => match align.wait(Duration::from_millis(100)) {
                        Ok(aligned) => aligned,
                        Err(_) => continue, // skip this frame
                    },
                    Err(_) => continue,
                }
            } else {
                composite
            };

            // On first frame, update device info
            if first_frame {
                update_device_info(&ctx, &composite).await;
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

            let _ = ctx.color_handle.set_params_and_notify(0, vec![
                ParamSetValue::Int32 { reason: ctx.color_ad.num_images_counter, addr: 0, value: num_counter },
            ]).await;

            if config.array_callbacks {
                process_color_frame(&composite, &ctx, color_array_counter).await;

                // Extract depth frames — each frames_of_type() call gives new owned frames
                let depth_frames: Vec<DepthFrame> = composite.frames_of_type();
                if let Some(depth_frame) = depth_frames.into_iter().next() {
                    // Read depth units from the original frame before filtering
                    if let Ok(units) = depth_frame.depth_units() {
                        let _ = ctx.color_handle.set_params_and_notify(0, vec![
                            ParamSetValue::Float64 { reason: ctx.rs_params.rs_depth_units, addr: 0, value: units as f64 },
                        ]).await;
                    }
                    process_depth_frame(depth_frame, &ctx, depth_array_counter, &mut depth_filters, &config).await;
                }

                if config.pointcloud_enable {
                    // Get another owned depth frame for pointcloud processing
                    let depth_frames: Vec<DepthFrame> = composite.frames_of_type();
                    if let Some(depth_frame) = depth_frames.into_iter().next() {
                        process_pointcloud(depth_frame, &ctx, &mut pc_block, depth_array_counter).await;
                    }
                }
            }

            process_imu(&composite, &ctx).await;
            let _ = ctx.color_handle.call_param_callbacks(0).await;
            let _ = ctx.depth_handle.call_param_callbacks(0).await;

            // Check stop conditions
            if config.image_mode == ImageMode::Single
                || (config.image_mode == ImageMode::Multiple && num_counter >= config.num_images)
            {
                break;
            }

            // Check for stop command (non-blocking)
            if matches!(ctx.acq_rx.try_recv(), Ok(AcqCommand::Stop)) {
                break;
            }
        }

        // Pipeline is dropped here (ActivePipeline::drop calls rs2_delete_pipeline)
        ctx.end_acquisition(config.wait_for_plugins).await;
    }
}
