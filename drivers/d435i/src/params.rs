use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;

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
    pub rs_has_imu: usize,
    pub rs_has_emitter: usize,

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

    // Diagnostics
    pub rs_frames_dropped: usize,
    pub rs_error_count: usize,
    pub rs_last_error: usize,

    // Post-processing filters
    pub rs_decimation_enable: usize,
    pub rs_decimation_magnitude: usize,
    pub rs_spatial_enable: usize,
    pub rs_spatial_alpha: usize,
    pub rs_spatial_delta: usize,
    pub rs_spatial_magnitude: usize,
    pub rs_temporal_enable: usize,
    pub rs_temporal_alpha: usize,
    pub rs_temporal_delta: usize,
    pub rs_hole_fill_enable: usize,
    pub rs_hole_fill_mode: usize,

    // Alignment
    pub rs_align_enable: usize,

    // Pointcloud
    pub rs_pointcloud_enable: usize,

    // Intrinsics, read from the stream profiles once the pipeline is up.
    //
    // Both sets live on the Color port because that is where every other RS_*
    // parameter lives -- `D435iDepthDriver` carries only an `ADDriverBase` and
    // has no parameter set of its own. Depth values are therefore prefixed
    // rather than sitting on the depth port.
    //
    // A pinhole model in pixels: fx/fy focal length, ppx/ppy principal point.
    // These belong to the *stream profile in use*, so they follow RSStreamMode
    // -- a 1280x720 profile does not share a 640x480 profile's numbers.
    pub rs_fx: usize,
    pub rs_fy: usize,
    pub rs_ppx: usize,
    pub rs_ppy: usize,
    /// Distortion model name, e.g. "Inverse Brown Conrady" (librealsense's own
    /// spelling), published as a string so a client need not track the enum.
    pub rs_dist_model: usize,
    /// The five distortion coefficients, in librealsense's `coeffs[]` order.
    pub rs_dist_coeff1: usize,
    pub rs_dist_coeff2: usize,
    pub rs_dist_coeff3: usize,
    pub rs_dist_coeff4: usize,
    pub rs_dist_coeff5: usize,

    pub rs_depth_fx: usize,
    pub rs_depth_fy: usize,
    pub rs_depth_ppx: usize,
    pub rs_depth_ppy: usize,
    pub rs_depth_dist_model: usize,
    pub rs_depth_dist_coeff1: usize,
    pub rs_depth_dist_coeff2: usize,
    pub rs_depth_dist_coeff3: usize,
    pub rs_depth_dist_coeff4: usize,
    pub rs_depth_dist_coeff5: usize,
}

impl D435iParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            rs_stream_mode: base.create_param("RS_STREAM_MODE", ParamType::Enum)?,
            rs_res_x: base.create_param("RS_RES_X", ParamType::Int32)?,
            rs_res_y: base.create_param("RS_RES_Y", ParamType::Int32)?,
            rs_frame_rate: base.create_param("RS_FRAME_RATE", ParamType::Int32)?,
            rs_exposure: base.create_param("RS_EXPOSURE", ParamType::Float64)?,
            rs_gain: base.create_param("RS_GAIN", ParamType::Float64)?,
            rs_auto_exposure: base.create_param("RS_AUTO_EXPOSURE", ParamType::Int32)?,
            rs_laser_power: base.create_param("RS_LASER_POWER", ParamType::Float64)?,
            rs_emitter_enabled: base.create_param("RS_EMITTER_ENABLED", ParamType::Int32)?,
            rs_depth_units: base.create_param("RS_DEPTH_UNITS", ParamType::Float64)?,
            rs_has_imu: base.create_param("RS_HAS_IMU", ParamType::Int32)?,
            rs_has_emitter: base.create_param("RS_HAS_EMITTER", ParamType::Int32)?,
            rs_accel_x: base.create_param("RS_ACCEL_X", ParamType::Float64)?,
            rs_accel_y: base.create_param("RS_ACCEL_Y", ParamType::Float64)?,
            rs_accel_z: base.create_param("RS_ACCEL_Z", ParamType::Float64)?,
            rs_gyro_x: base.create_param("RS_GYRO_X", ParamType::Float64)?,
            rs_gyro_y: base.create_param("RS_GYRO_Y", ParamType::Float64)?,
            rs_gyro_z: base.create_param("RS_GYRO_Z", ParamType::Float64)?,
            rs_serial: base.create_param("RS_SERIAL", ParamType::Octet)?,
            rs_connected: base.create_param("RS_CONNECTED", ParamType::Int32)?,

            rs_frames_dropped: base.create_param("RS_FRAMES_DROPPED", ParamType::Int32)?,
            rs_error_count: base.create_param("RS_ERROR_COUNT", ParamType::Int32)?,
            rs_last_error: base.create_param("RS_LAST_ERROR", ParamType::Octet)?,

            rs_decimation_enable: base.create_param("RS_DECIMATION_ENABLE", ParamType::Int32)?,
            rs_decimation_magnitude: base
                .create_param("RS_DECIMATION_MAGNITUDE", ParamType::Int32)?,
            rs_spatial_enable: base.create_param("RS_SPATIAL_ENABLE", ParamType::Int32)?,
            rs_spatial_alpha: base.create_param("RS_SPATIAL_ALPHA", ParamType::Float64)?,
            rs_spatial_delta: base.create_param("RS_SPATIAL_DELTA", ParamType::Int32)?,
            rs_spatial_magnitude: base.create_param("RS_SPATIAL_MAGNITUDE", ParamType::Int32)?,
            rs_temporal_enable: base.create_param("RS_TEMPORAL_ENABLE", ParamType::Int32)?,
            rs_temporal_alpha: base.create_param("RS_TEMPORAL_ALPHA", ParamType::Float64)?,
            rs_temporal_delta: base.create_param("RS_TEMPORAL_DELTA", ParamType::Int32)?,
            rs_hole_fill_enable: base.create_param("RS_HOLE_FILL_ENABLE", ParamType::Int32)?,
            rs_hole_fill_mode: base.create_param("RS_HOLE_FILL_MODE", ParamType::Int32)?,

            rs_align_enable: base.create_param("RS_ALIGN_ENABLE", ParamType::Int32)?,
            rs_pointcloud_enable: base.create_param("RS_POINTCLOUD_ENABLE", ParamType::Int32)?,

            rs_fx: base.create_param("RS_FX", ParamType::Float64)?,
            rs_fy: base.create_param("RS_FY", ParamType::Float64)?,
            rs_ppx: base.create_param("RS_PPX", ParamType::Float64)?,
            rs_ppy: base.create_param("RS_PPY", ParamType::Float64)?,
            rs_dist_model: base.create_param("RS_DIST_MODEL", ParamType::Octet)?,
            rs_dist_coeff1: base.create_param("RS_DIST_COEFF1", ParamType::Float64)?,
            rs_dist_coeff2: base.create_param("RS_DIST_COEFF2", ParamType::Float64)?,
            rs_dist_coeff3: base.create_param("RS_DIST_COEFF3", ParamType::Float64)?,
            rs_dist_coeff4: base.create_param("RS_DIST_COEFF4", ParamType::Float64)?,
            rs_dist_coeff5: base.create_param("RS_DIST_COEFF5", ParamType::Float64)?,

            rs_depth_fx: base.create_param("RS_DEPTH_FX", ParamType::Float64)?,
            rs_depth_fy: base.create_param("RS_DEPTH_FY", ParamType::Float64)?,
            rs_depth_ppx: base.create_param("RS_DEPTH_PPX", ParamType::Float64)?,
            rs_depth_ppy: base.create_param("RS_DEPTH_PPY", ParamType::Float64)?,
            rs_depth_dist_model: base.create_param("RS_DEPTH_DIST_MODEL", ParamType::Octet)?,
            rs_depth_dist_coeff1: base.create_param("RS_DEPTH_DIST_COEFF1", ParamType::Float64)?,
            rs_depth_dist_coeff2: base.create_param("RS_DEPTH_DIST_COEFF2", ParamType::Float64)?,
            rs_depth_dist_coeff3: base.create_param("RS_DEPTH_DIST_COEFF3", ParamType::Float64)?,
            rs_depth_dist_coeff4: base.create_param("RS_DEPTH_DIST_COEFF4", ParamType::Float64)?,
            rs_depth_dist_coeff5: base.create_param("RS_DEPTH_DIST_COEFF5", ParamType::Float64)?,
        })
    }
}

/// Configuration snapshot read from the Color port for the acquisition thread.
#[derive(Clone)]
pub struct D435iConfigSnapshot {
    /// Index into `STREAM_MODES`, kept so a refused mode can be put back.
    pub stream_mode: i32,
    pub res_x: i32,
    pub res_y: i32,
    pub frame_rate: i32,
    pub exposure: f64,
    pub gain: f64,
    pub auto_exposure: bool,
    pub laser_power: f64,
    pub emitter_enabled: bool,
    pub serial: String,
    pub image_mode: epics_rs::ad_core::driver::ImageMode,
    pub num_images: i32,
    pub array_callbacks: bool,
    pub wait_for_plugins: bool,

    // Post-processing filters
    pub decimation_enable: bool,
    pub decimation_magnitude: i32,
    pub spatial_enable: bool,
    pub spatial_alpha: f64,
    pub spatial_delta: i32,
    pub spatial_magnitude: i32,
    pub temporal_enable: bool,
    pub temporal_alpha: f64,
    pub temporal_delta: i32,
    pub hole_fill_enable: bool,
    pub hole_fill_mode: i32,

    // Alignment & pointcloud
    pub align_enable: bool,
    pub pointcloud_enable: bool,
}

impl D435iConfigSnapshot {
    /// Frame timeout: 3 frames worth of time + 2 seconds margin.
    pub fn frame_timeout(&self) -> std::time::Duration {
        let timeout_ms = (3000 / self.frame_rate.max(1) as u64) + 2000;
        std::time::Duration::from_millis(timeout_ms)
    }

    /// Read config via PortHandle (async). For use from the acquisition task.
    ///
    /// `serial` is passed in separately since PortHandle has no blocking string read.
    pub async fn read_via_handle(
        handle: &PortHandle,
        ad: &ADBaseParams,
        rs: &D435iParams,
        serial: &str,
    ) -> AsynResult<Self> {
        Ok(Self {
            stream_mode: handle.read_int32(rs.rs_stream_mode, 0).await?,
            res_x: handle.read_int32(rs.rs_res_x, 0).await?,
            res_y: handle.read_int32(rs.rs_res_y, 0).await?,
            frame_rate: handle.read_int32(rs.rs_frame_rate, 0).await?,
            exposure: handle.read_float64(rs.rs_exposure, 0).await?,
            gain: handle.read_float64(rs.rs_gain, 0).await?,
            auto_exposure: handle.read_int32(rs.rs_auto_exposure, 0).await? != 0,
            laser_power: handle.read_float64(rs.rs_laser_power, 0).await?,
            emitter_enabled: handle.read_int32(rs.rs_emitter_enabled, 0).await? != 0,
            serial: serial.to_string(),
            image_mode: epics_rs::ad_core::driver::ImageMode::from_i32(
                handle.read_int32(ad.image_mode, 0).await?,
            ),
            num_images: handle.read_int32(ad.num_images, 0).await?,
            array_callbacks: handle.read_int32(ad.base.array_callbacks, 0).await? != 0,
            wait_for_plugins: handle
                .read_int32(ad.base.wait_for_plugins, 0)
                .await
                .unwrap_or(0)
                != 0,

            decimation_enable: handle
                .read_int32(rs.rs_decimation_enable, 0)
                .await
                .unwrap_or(0)
                != 0,
            decimation_magnitude: handle
                .read_int32(rs.rs_decimation_magnitude, 0)
                .await
                .unwrap_or(2),
            spatial_enable: handle
                .read_int32(rs.rs_spatial_enable, 0)
                .await
                .unwrap_or(0)
                != 0,
            spatial_alpha: handle
                .read_float64(rs.rs_spatial_alpha, 0)
                .await
                .unwrap_or(0.5),
            spatial_delta: handle
                .read_int32(rs.rs_spatial_delta, 0)
                .await
                .unwrap_or(20),
            spatial_magnitude: handle
                .read_int32(rs.rs_spatial_magnitude, 0)
                .await
                .unwrap_or(2),
            temporal_enable: handle
                .read_int32(rs.rs_temporal_enable, 0)
                .await
                .unwrap_or(0)
                != 0,
            temporal_alpha: handle
                .read_float64(rs.rs_temporal_alpha, 0)
                .await
                .unwrap_or(0.4),
            temporal_delta: handle
                .read_int32(rs.rs_temporal_delta, 0)
                .await
                .unwrap_or(20),
            hole_fill_enable: handle
                .read_int32(rs.rs_hole_fill_enable, 0)
                .await
                .unwrap_or(0)
                != 0,
            hole_fill_mode: handle
                .read_int32(rs.rs_hole_fill_mode, 0)
                .await
                .unwrap_or(1),

            align_enable: handle.read_int32(rs.rs_align_enable, 0).await.unwrap_or(0) != 0,
            pointcloud_enable: handle
                .read_int32(rs.rs_pointcloud_enable, 0)
                .await
                .unwrap_or(0)
                != 0,
        })
    }
}
