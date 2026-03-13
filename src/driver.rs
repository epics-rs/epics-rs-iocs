use std::sync::Arc;

use asyn_rs::error::AsynResult;
use asyn_rs::port::{PortDriver, PortDriverBase};
use asyn_rs::port_handle::PortHandle;
use asyn_rs::runtime::config::RuntimeConfig;
use asyn_rs::runtime::port::{PortRuntimeHandle, create_port_runtime};
use asyn_rs::user::AsynUser;

use ad_core::driver::{ADDriver, ADDriverBase, ImageMode};
use ad_core::ndarray_pool::NDArrayPool;
use ad_core::params::ADBaseParams;
use ad_core::plugin::channel::{NDArrayOutput, NDArraySender, QueuedArrayCounter};

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
        base.set_string_param(ad.params.base.sdk_version, 0, env!("CARGO_PKG_VERSION").into())?;

        // Default stream config
        let default_mode = &STREAM_MODES[DEFAULT_STREAM_MODE as usize];
        base.set_int32_param(rs_params.rs_stream_mode, 0, DEFAULT_STREAM_MODE)?;
        base.set_int32_param(rs_params.rs_res_x, 0, default_mode.width)?;
        base.set_int32_param(rs_params.rs_res_y, 0, default_mode.height)?;
        base.set_int32_param(rs_params.rs_frame_rate, 0, default_mode.fps)?;

        // Default sensor options
        base.set_float64_param(rs_params.rs_exposure, 0, 8500.0)?;
        base.set_float64_param(rs_params.rs_gain, 0, 16.0)?;
        base.set_int32_param(rs_params.rs_auto_exposure, 0, 1)?;
        base.set_float64_param(rs_params.rs_laser_power, 0, 150.0)?;
        base.set_int32_param(rs_params.rs_emitter_enabled, 0, 1)?;

        // Read-only defaults
        base.set_float64_param(rs_params.rs_depth_units, 0, 0.001)?;
        base.set_int32_param(rs_params.rs_connected, 0, 0)?;

        // Acquire defaults
        base.set_int32_param(ad.params.image_mode, 0, ImageMode::Continuous as i32)?;

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
                let _ = self.acq_tx.send(AcqCommand::Start);
            } else if value == 0 && acquiring != 0 {
                self.ad.port_base.set_string_param(
                    self.ad.params.status_message,
                    0,
                    "Acquisition stopped".into(),
                )?;
                self.ad.port_base.set_int32_param(acquire_idx, 0, value)?;
                let _ = self.acq_tx.send(AcqCommand::Stop);
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
                self.dirty.lock().reconfigure_pipeline = true;
            }
            // Invalid index: silently ignore
        } else {
            self.ad.port_base.params.set_int32(reason, user.addr, value)?;

            // Dirty flag routing
            if reason == self.rs_params.rs_auto_exposure
                || reason == self.rs_params.rs_emitter_enabled
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
        base.set_string_param(ad.params.base.sdk_version, 0, env!("CARGO_PKG_VERSION").into())?;

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
