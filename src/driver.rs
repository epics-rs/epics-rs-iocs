use std::sync::Arc;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ImageMode};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{NDArrayOutput, NDArraySender, QueuedArrayCounter};

use crate::params::D435iParams;
use crate::task::{AcquisitionContext, start_acquisition_task};
use crate::types::{AcqCommand, DirtyFlags, STREAM_MODES, DEFAULT_STREAM_MODE};

// ============================================================================
// Color Driver (main)
// ============================================================================

pub struct D435iColorDriver {
    pub ad: ADDriverBase,
    pub rs_params: D435iParams,
    pub dirty: Arc<parking_lot::Mutex<DirtyFlags>>,
    acq_tx: std::sync::mpsc::Sender<AcqCommand>,
}

impl D435iColorDriver {
    pub fn new(
        port_name: &str,
        max_size_x: i32,
        max_size_y: i32,
        max_memory: usize,
        acq_tx: std::sync::mpsc::Sender<AcqCommand>,
        dirty: Arc<parking_lot::Mutex<DirtyFlags>>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(port_name, max_size_x, max_size_y, max_memory)?;
        let rs_params = D435iParams::create(&mut ad.port_base)?;

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.manufacturer, 0, "Intel".into())?;
        base.set_string_param(ad.params.base.model, 0, "RealSense D435i".into())?;
        base.set_string_param(ad.params.base.serial_number, 0, "Not connected".into())?;
        base.set_string_param(ad.params.base.firmware_version, 0, "Unknown".into())?;
        base.set_string_param(ad.params.base.sdk_version, 0, env!("CARGO_PKG_VERSION").into())?;

        // Default stream config
        let default_mode = &STREAM_MODES[DEFAULT_STREAM_MODE as usize];
        base.set_int32_param(rs_params.rs_stream_mode, 0, DEFAULT_STREAM_MODE)?;
        base.set_int32_param(rs_params.rs_res_x, 0, default_mode.width)?;
        base.set_int32_param(rs_params.rs_res_y, 0, default_mode.height)?;
        base.set_int32_param(rs_params.rs_frame_rate, 0, default_mode.fps)?;

        // Image size and ROI
        base.set_int32_param(ad.params.size_x, 0, default_mode.width)?;
        base.set_int32_param(ad.params.size_y, 0, default_mode.height)?;
        base.set_int32_param(ad.params.min_x, 0, 0)?;
        base.set_int32_param(ad.params.min_y, 0, 0)?;
        base.set_int32_param(ad.params.bin_x, 0, 1)?;
        base.set_int32_param(ad.params.bin_y, 0, 1)?;
        base.set_int32_param(ad.params.reverse_x, 0, 0)?;
        base.set_int32_param(ad.params.reverse_y, 0, 0)?;

        // Acquire timing and control
        base.set_float64_param(ad.params.acquire_time, 0, 1.0 / default_mode.fps as f64)?;
        base.set_float64_param(ad.params.acquire_period, 0, 1.0 / default_mode.fps as f64)?;
        base.set_int32_param(ad.params.image_mode, 0, ImageMode::Continuous as i32)?;
        base.set_int32_param(ad.params.num_images, 0, 100)?;
        base.set_int32_param(ad.params.num_exposures, 0, 1)?;
        base.set_int32_param(ad.params.trigger_mode, 0, 0)?;

        // Default sensor options
        const DEFAULT_EXPOSURE_US: f64 = 8500.0;
        const DEFAULT_GAIN: f64 = 16.0;
        const DEFAULT_LASER_POWER_MW: f64 = 150.0;
        base.set_float64_param(rs_params.rs_exposure, 0, DEFAULT_EXPOSURE_US)?;
        base.set_float64_param(rs_params.rs_gain, 0, DEFAULT_GAIN)?;
        base.set_int32_param(rs_params.rs_auto_exposure, 0, 1)?;
        base.set_float64_param(rs_params.rs_laser_power, 0, DEFAULT_LASER_POWER_MW)?;
        base.set_int32_param(rs_params.rs_emitter_enabled, 0, 1)?;

        // Read-only defaults
        base.set_float64_param(rs_params.rs_depth_units, 0, 0.001)?;
        base.set_int32_param(rs_params.rs_connected, 0, 0)?;

        // Diagnostics
        base.set_int32_param(rs_params.rs_frames_dropped, 0, 0)?;
        base.set_int32_param(rs_params.rs_error_count, 0, 0)?;

        // Post-processing filter defaults (all off)
        base.set_int32_param(rs_params.rs_decimation_enable, 0, 0)?;
        base.set_int32_param(rs_params.rs_decimation_magnitude, 0, 2)?;
        base.set_int32_param(rs_params.rs_spatial_enable, 0, 0)?;
        base.set_float64_param(rs_params.rs_spatial_alpha, 0, 0.5)?;
        base.set_int32_param(rs_params.rs_spatial_delta, 0, 20)?;
        base.set_int32_param(rs_params.rs_spatial_magnitude, 0, 2)?;
        base.set_int32_param(rs_params.rs_temporal_enable, 0, 0)?;
        base.set_float64_param(rs_params.rs_temporal_alpha, 0, 0.4)?;
        base.set_int32_param(rs_params.rs_temporal_delta, 0, 20)?;
        base.set_int32_param(rs_params.rs_hole_fill_enable, 0, 0)?;
        base.set_int32_param(rs_params.rs_hole_fill_mode, 0, 1)?;

        // Alignment & pointcloud (off)
        base.set_int32_param(rs_params.rs_align_enable, 0, 0)?;
        base.set_int32_param(rs_params.rs_pointcloud_enable, 0, 0)?;

        Ok(Self {
            ad,
            rs_params,
            dirty,
            acq_tx,
        })
    }
}

impl PortDriver for D435iColorDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let acquire_idx = self.ad.params.acquire;

        if reason == acquire_idx {
            let acquiring = self.ad.port_base.get_int32_param(acquire_idx, 0).unwrap_or(0);
            if value != 0 && acquiring == 0 {
                self.ad.port_base.set_string_param(
                    self.ad.params.status_message,
                    0,
                    "Acquiring data".into(),
                )?;
                self.ad.port_base.set_int32_param(acquire_idx, 0, value)?;
                if self.acq_tx.send(AcqCommand::Start).is_err() {
                    log::error!("D435i: acquisition task is not running");
                    self.ad.port_base.set_string_param(self.ad.params.status_message, 0, "Acquisition task crashed".into())?;
                    self.ad.port_base.set_int32_param(acquire_idx, 0, 0)?;
                }
            } else if value == 0 && acquiring != 0 {
                self.ad.port_base.set_string_param(
                    self.ad.params.status_message,
                    0,
                    "Acquisition stopped".into(),
                )?;
                self.ad.port_base.set_int32_param(acquire_idx, 0, value)?;
                if self.acq_tx.send(AcqCommand::Stop).is_err() {
                    log::error!("D435i: acquisition task is not running");
                }
            } else {
                self.ad.port_base.set_int32_param(acquire_idx, 0, value)?;
            }
        } else if reason == self.rs_params.rs_stream_mode {
            // Validate mode index and apply
            if let Some(mode) = STREAM_MODES.get(value as usize) {
                self.ad.port_base.params.set_int32(reason, user.addr, value)?;
                self.ad.port_base.params.set_int32(self.rs_params.rs_res_x, 0, mode.width)?;
                self.ad.port_base.params.set_int32(self.rs_params.rs_res_y, 0, mode.height)?;
                self.ad.port_base.params.set_int32(self.rs_params.rs_frame_rate, 0, mode.fps)?;
                // Update AD params to match
                self.ad.port_base.params.set_int32(self.ad.params.size_x, 0, mode.width)?;
                self.ad.port_base.params.set_int32(self.ad.params.size_y, 0, mode.height)?;
                self.ad.port_base.params.set_float64(self.ad.params.acquire_time, 0, 1.0 / mode.fps as f64)?;
                self.dirty.lock().reconfigure_pipeline = true;
            } else {
                log::warn!("D435i: invalid stream mode index {value}, max is {}", STREAM_MODES.len() - 1);
            }
        } else {
            self.ad.port_base.params.set_int32(reason, user.addr, value)?;

            // Dirty flag routing
            if reason == self.rs_params.rs_auto_exposure
                || reason == self.rs_params.rs_emitter_enabled
                || reason == self.ad.params.base.array_callbacks
                || reason == self.rs_params.rs_decimation_enable
                || reason == self.rs_params.rs_decimation_magnitude
                || reason == self.rs_params.rs_spatial_enable
                || reason == self.rs_params.rs_spatial_delta
                || reason == self.rs_params.rs_spatial_magnitude
                || reason == self.rs_params.rs_temporal_enable
                || reason == self.rs_params.rs_temporal_delta
                || reason == self.rs_params.rs_hole_fill_enable
                || reason == self.rs_params.rs_hole_fill_mode
                || reason == self.rs_params.rs_align_enable
                || reason == self.rs_params.rs_pointcloud_enable
            {
                self.dirty.lock().update_sensor_options = true;
            }
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        self.ad.port_base.params.set_float64(reason, user.addr, value)?;

        let mut dirty = self.dirty.lock();
        if reason == self.rs_params.rs_exposure
            || reason == self.rs_params.rs_gain
            || reason == self.rs_params.rs_laser_power
            || reason == self.rs_params.rs_spatial_alpha
            || reason == self.rs_params.rs_temporal_alpha
        {
            dirty.update_sensor_options = true;
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }
}

impl ADDriver for D435iColorDriver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

// ============================================================================
// Depth Driver (secondary)
// ============================================================================

pub struct D435iDepthDriver {
    pub ad: ADDriverBase,
}

impl D435iDepthDriver {
    pub fn new(
        port_name: &str,
        max_size_x: i32,
        max_size_y: i32,
        max_memory: usize,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(port_name, max_size_x, max_size_y, max_memory)?;

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.manufacturer, 0, "Intel".into())?;
        base.set_string_param(ad.params.base.model, 0, "RealSense D435i (Depth)".into())?;
        base.set_string_param(ad.params.base.serial_number, 0, "Not connected".into())?;
        base.set_string_param(ad.params.base.firmware_version, 0, "Unknown".into())?;
        base.set_string_param(ad.params.base.sdk_version, 0, env!("CARGO_PKG_VERSION").into())?;

        // Image size and ROI (use default stream mode)
        let default_mode = &STREAM_MODES[DEFAULT_STREAM_MODE as usize];
        base.set_int32_param(ad.params.size_x, 0, default_mode.width)?;
        base.set_int32_param(ad.params.size_y, 0, default_mode.height)?;
        base.set_int32_param(ad.params.min_x, 0, 0)?;
        base.set_int32_param(ad.params.min_y, 0, 0)?;
        base.set_int32_param(ad.params.bin_x, 0, 1)?;
        base.set_int32_param(ad.params.bin_y, 0, 1)?;
        base.set_int32_param(ad.params.reverse_x, 0, 0)?;
        base.set_int32_param(ad.params.reverse_y, 0, 0)?;

        // Acquire timing and control
        base.set_float64_param(ad.params.acquire_time, 0, 1.0 / default_mode.fps as f64)?;
        base.set_float64_param(ad.params.acquire_period, 0, 1.0 / default_mode.fps as f64)?;
        base.set_int32_param(ad.params.image_mode, 0, ImageMode::Continuous as i32)?;
        base.set_int32_param(ad.params.num_images, 0, 100)?;
        base.set_int32_param(ad.params.num_exposures, 0, 1)?;
        base.set_int32_param(ad.params.trigger_mode, 0, 0)?;

        Ok(Self { ad })
    }
}

impl PortDriver for D435iDepthDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }
}

impl ADDriver for D435iDepthDriver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

// ============================================================================
// Runtime handles
// ============================================================================

pub struct D435iColorRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub rs_params: D435iParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    pc_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    pc_queued: Arc<QueuedArrayCounter>,
    #[allow(dead_code)]
    task_handle: Option<std::thread::JoinHandle<()>>,
}

impl D435iColorRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, mut sender: NDArraySender) {
        sender.set_queued_counter(self.queued_counter.clone());
        self.array_output.lock().add(sender);
    }

    pub fn pc_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.pc_output
    }

    pub fn connect_pc_downstream(&self, mut sender: NDArraySender) {
        sender.set_queued_counter(self.pc_queued.clone());
        self.pc_output.lock().add(sender);
    }
}

pub struct D435iDepthRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
}

impl D435iDepthRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, mut sender: NDArraySender) {
        sender.set_queued_counter(self.queued_counter.clone());
        self.array_output.lock().add(sender);
    }
}

/// Create both Color and Depth drivers, start the shared acquisition task.
pub fn create_d435i_detector(
    port_name: &str,
    serial: &str,
    max_size_x: i32,
    max_size_y: i32,
    max_memory: usize,
) -> AsynResult<(D435iColorRuntime, D435iDepthRuntime)> {
    let depth_port_name = format!("{port_name}_DEPTH");

    // Shared state
    let (acq_tx, acq_rx) = std::sync::mpsc::channel();
    let dirty = Arc::new(parking_lot::Mutex::new(DirtyFlags::default()));
    dirty.lock().set_all();

    // --- Color port ---
    let color_det = D435iColorDriver::new(
        port_name, max_size_x, max_size_y, max_memory, acq_tx, dirty.clone(),
    )?;
    let color_ad_params = color_det.ad.params;
    let color_rs_params = color_det.rs_params;
    let color_pool = color_det.ad.pool.clone();
    let (color_runtime_handle, _) = create_port_runtime(color_det, RuntimeConfig::default());
    let color_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let color_queued = Arc::new(QueuedArrayCounter::new());
    let pc_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let pc_queued = Arc::new(QueuedArrayCounter::new());

    // --- Depth port ---
    let depth_det = D435iDepthDriver::new(&depth_port_name, max_size_x, max_size_y, max_memory)?;
    let depth_ad_params = depth_det.ad.params;
    let depth_pool = depth_det.ad.pool.clone();
    let (depth_runtime_handle, _) = create_port_runtime(depth_det, RuntimeConfig::default());
    let depth_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let depth_queued = Arc::new(QueuedArrayCounter::new());

    // --- Acquisition task (shared) ---
    let task_handle = start_acquisition_task(AcquisitionContext {
        acq_rx,
        color_handle: color_runtime_handle.port_handle().clone(),
        color_output: color_output.clone(),
        color_queued: color_queued.clone(),
        depth_handle: depth_runtime_handle.port_handle().clone(),
        depth_output: depth_output.clone(),
        depth_queued: depth_queued.clone(),
        pc_output: pc_output.clone(),
        pc_queued: pc_queued.clone(),
        dirty,
        color_ad: color_ad_params,
        depth_ad: depth_ad_params,
        rs_params: color_rs_params,
        serial: serial.to_string(),
    });

    let color_runtime = D435iColorRuntime {
        runtime_handle: color_runtime_handle,
        ad_params: color_ad_params,
        rs_params: color_rs_params,
        pool: color_pool,
        array_output: color_output,
        queued_counter: color_queued,
        pc_output,
        pc_queued,
        task_handle: Some(task_handle),
    };

    let depth_runtime = D435iDepthRuntime {
        runtime_handle: depth_runtime_handle,
        ad_params: depth_ad_params,
        pool: depth_pool,
        array_output: depth_output,
        queued_counter: depth_queued,
    };

    Ok((color_runtime, depth_runtime))
}
