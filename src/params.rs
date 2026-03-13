use ad_core::params::ADBaseParams;
use asyn_rs::error::AsynResult;
use asyn_rs::param::ParamType;
use asyn_rs::port::PortDriverBase;
use asyn_rs::port_handle::PortHandle;

/// D435i-specific parameter indices, registered on the Color port.
#[derive(Clone, Copy)]
pub struct D435iParams {
    // Stream configuration
    pub rs_stream_mode: usize,
    pub rs_res_x: usize,
    pub rs_res_y: usize,
    pub rs_frame_rate: usize,

    // Sensor options
    pub rs_exposure: usize,
    pub rs_gain: usize,
    pub rs_auto_exposure: usize,
    pub rs_laser_power: usize,
    pub rs_emitter_enabled: usize,

    // Depth info (read-only)
    pub rs_depth_units: usize,

    // IMU
    pub rs_accel_x: usize,
    pub rs_accel_y: usize,
    pub rs_accel_z: usize,
    pub rs_gyro_x: usize,
    pub rs_gyro_y: usize,
    pub rs_gyro_z: usize,

    // Device info
    pub rs_serial: usize,
    pub rs_connected: usize,
}

impl D435iParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            rs_stream_mode: base.create_param("RS_STREAM_MODE", ParamType::Int32)?,
            rs_res_x: base.create_param("RS_RES_X", ParamType::Int32)?,
            rs_res_y: base.create_param("RS_RES_Y", ParamType::Int32)?,
            rs_frame_rate: base.create_param("RS_FRAME_RATE", ParamType::Int32)?,
            rs_exposure: base.create_param("RS_EXPOSURE", ParamType::Float64)?,
            rs_gain: base.create_param("RS_GAIN", ParamType::Float64)?,
            rs_auto_exposure: base.create_param("RS_AUTO_EXPOSURE", ParamType::Int32)?,
            rs_laser_power: base.create_param("RS_LASER_POWER", ParamType::Float64)?,
            rs_emitter_enabled: base.create_param("RS_EMITTER_ENABLED", ParamType::Int32)?,
            rs_depth_units: base.create_param("RS_DEPTH_UNITS", ParamType::Float64)?,
            rs_accel_x: base.create_param("RS_ACCEL_X", ParamType::Float64)?,
            rs_accel_y: base.create_param("RS_ACCEL_Y", ParamType::Float64)?,
            rs_accel_z: base.create_param("RS_ACCEL_Z", ParamType::Float64)?,
            rs_gyro_x: base.create_param("RS_GYRO_X", ParamType::Float64)?,
            rs_gyro_y: base.create_param("RS_GYRO_Y", ParamType::Float64)?,
            rs_gyro_z: base.create_param("RS_GYRO_Z", ParamType::Float64)?,
            rs_serial: base.create_param("RS_SERIAL", ParamType::Octet)?,
            rs_connected: base.create_param("RS_CONNECTED", ParamType::Int32)?,
        })
    }
}

/// Configuration snapshot read from the Color port for the acquisition thread.
pub struct D435iConfigSnapshot {
    pub res_x: i32,
    pub res_y: i32,
    pub frame_rate: i32,
    pub exposure: f64,
    pub gain: f64,
    pub auto_exposure: bool,
    pub laser_power: f64,
    pub emitter_enabled: bool,
    pub serial: String,
    pub image_mode: ad_core::driver::ImageMode,
    pub num_images: i32,
    pub array_callbacks: bool,
    pub wait_for_plugins: bool,
}

impl D435iConfigSnapshot {
    /// Read config via PortHandle (blocking). For use from the acquisition task thread.
    ///
    /// `serial` is passed in separately since PortHandle has no blocking string read.
    pub fn read_via_handle(
        handle: &PortHandle,
        ad: &ADBaseParams,
        rs: &D435iParams,
        serial: &str,
    ) -> AsynResult<Self> {
        Ok(Self {
            res_x: handle.read_int32_blocking(rs.rs_res_x, 0)?,
            res_y: handle.read_int32_blocking(rs.rs_res_y, 0)?,
            frame_rate: handle.read_int32_blocking(rs.rs_frame_rate, 0)?,
            exposure: handle.read_float64_blocking(rs.rs_exposure, 0)?,
            gain: handle.read_float64_blocking(rs.rs_gain, 0)?,
            auto_exposure: handle.read_int32_blocking(rs.rs_auto_exposure, 0)? != 0,
            laser_power: handle.read_float64_blocking(rs.rs_laser_power, 0)?,
            emitter_enabled: handle.read_int32_blocking(rs.rs_emitter_enabled, 0)? != 0,
            serial: serial.to_string(),
            image_mode: ad_core::driver::ImageMode::from_i32(
                handle.read_int32_blocking(ad.image_mode, 0)?,
            ),
            num_images: handle.read_int32_blocking(ad.num_images, 0)?,
            array_callbacks: handle.read_int32_blocking(ad.base.array_callbacks, 0)? != 0,
            wait_for_plugins: handle
                .read_int32_blocking(ad.base.wait_for_plugins, 0)
                .unwrap_or(0)
                != 0,
        })
    }
}
