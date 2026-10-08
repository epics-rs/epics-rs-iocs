//! Port of ADPylon `ADPylon.cpp`: the pylon backend of the ADGenICam driver.

use std::sync::Arc;
use std::sync::atomic::{AtomicI32, AtomicI64, AtomicU64, Ordering};

use asyn_rs::error::{AsynError, AsynResult, AsynStatus};
use asyn_rs::param::ParamType;
use asyn_rs::port_handle::PortHandle;
use asyn_rs::runtime::config::RuntimeConfig;
use asyn_rs::runtime::port::{PortRuntimeHandle, create_port_runtime};
use asyn_rs::trace::TraceMask;
use parking_lot::{Mutex, RwLock};

use ad_core_rs::driver::{ADDriverBase, ADStatus, ImageMode};
use ad_core_rs::ndarray_pool::NDArrayPool;
use ad_core_rs::params::ADBaseParams;
use ad_core_rs::plugin::channel::{ArrayPublisher, NDArrayOutput};

use ad_genicam::{ADGenICam, DriverCtx, FeatureIndex, GCFeatureType, GenICamBackend, GenICamNode};

use crate::camera::{Camera, NodeKind, NodeMap, PixelFormat};
use crate::node::PylonNode;
use crate::task::{self, GrabHandler, GrabShared, ImageTask, Msg};
use crate::trace::Trace;

const DRIVER_NAME: &str = "ADPylon";
const DRIVER_VERSION: &str = "1.0.0";

/// `PYLON_CONVERT_PIXEL_FORMAT` choices, in record order; 0 is "None".
const CONVERT_PIXEL_FORMATS: [Option<PixelFormat>; 5] = [
    None,
    Some(PixelFormat::Mono8),
    Some(PixelFormat::Mono16),
    Some(PixelFormat::Rgb8),
    Some(PixelFormat::Rgb16),
];

#[derive(Clone, Copy, Debug)]
pub struct PylonParams {
    pub convert_pixel_format: usize,
    pub convert_bit_align: usize,
    pub convert_shift_bits: usize,
    pub time_stamp_mode: usize,
    pub unique_id_mode: usize,
}

impl PylonParams {
    fn create(ad: &mut ADDriverBase) -> AsynResult<Self> {
        let base = &mut ad.port_base;
        Ok(Self {
            convert_pixel_format: base
                .create_param("PYLON_CONVERT_PIXEL_FORMAT", ParamType::Int32)?,
            convert_bit_align: base.create_param("PYLON_CONVERT_BIT_ALIGN", ParamType::Int32)?,
            convert_shift_bits: base.create_param("PYLON_CONVERT_SHIFT_BITS", ParamType::Int32)?,
            time_stamp_mode: base.create_param("PYLON_TIME_STAMP_MODE", ParamType::Int32)?,
            unique_id_mode: base.create_param("PYLON_UNIQUE_ID_MODE", ParamType::Int32)?,
        })
    }
}

pub struct PylonBackend {
    camera: Arc<Camera>,
    camera_id: String,
    shared: Arc<GrabShared>,
    params: PylonParams,
    device_is_reachable: bool,
    acquiring: bool,
}

fn error(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

impl PylonBackend {
    fn new(
        ad: &mut ADDriverBase,
        index: FeatureIndex,
        camera_id: &str,
        tx: ad_core_rs::runtime::CommandSender<Msg>,
    ) -> AsynResult<Self> {
        let shared = Arc::new(GrabShared {
            ad: ad.params,
            pool: ad.pool.clone(),
            index,
            events: RwLock::new(Vec::new()),
            time_stamp_mode: AtomicI32::new(task::TIME_STAMP_CAMERA),
            unique_id_mode: AtomicI32::new(task::UNIQUE_ID_CAMERA),
            unique_id: AtomicI32::new(0),
            grab: AtomicU64::new(0),
            results_left: AtomicI64::new(-1),
            tx,
            trace: Trace::new(&ad.port_base.port_name),
        });
        let camera = Arc::new(Camera::new(Box::new(GrabHandler(shared.clone()))));

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.driver_version, 0, DRIVER_VERSION)?;
        base.set_string_param(ad.params.base.sdk_version, 0, crate::camera::sdk_version())?;

        let params = PylonParams::create(ad)?;
        let mut backend = Self {
            camera,
            camera_id: camera_id.to_string(),
            shared,
            params,
            device_is_reachable: false,
            acquiring: false,
        };
        if backend.connect_camera(ad).is_err() {
            // The port starts out disconnected, and its auto-connect keeps
            // trying. List the cameras that could have been meant.
            ad.port_base.init_connected(false);
            let mut cameras = String::new();
            backend.report(&mut cameras, 1);
            print!("{cameras}");
        }
        Ok(backend)
    }

    /// The event data features of event source `source`, named per the
    /// camera's SFNC version.
    fn event_feature(&self, source: &str, suffix: &str) -> String {
        if self.camera.sfnc_major() >= 2 {
            format!("Event{source}{suffix}")
        } else {
            format!("{source}Event{suffix}")
        }
    }

    fn connect_camera(&mut self, ad: &mut ADDriverBase) -> AsynResult<()> {
        if let Err(e) = self.open_camera() {
            ad.port_base.trace_print(
                TraceMask::ERROR,
                &format!(
                    "{DRIVER_NAME}::connectCamera error opening camera {}: {e}\n",
                    self.camera_id
                ),
            );
            self.camera_disconnected(ad);
            return Err(error(e));
        }
        self.device_is_reachable = true;
        ad.port_base
            .set_int32_param(ad.params.status, 0, ADStatus::Idle as i32)?;
        ad.port_base
            .set_string_param(ad.params.status_message, 0, "")?;
        Ok(())
    }

    fn open_camera(&mut self) -> Result<(), String> {
        self.camera.open(&self.camera_id)?;

        // Register for every event source, so that the event's Timestamp and
        // FrameID features follow it.
        // See https://docs.baslerweb.com/event-notification
        let mut events = Vec::new();
        if self.camera.kind(NodeMap::Camera, "EventSelector") == Some(NodeKind::Enum) {
            let sources = self
                .camera
                .enum_entries(NodeMap::Camera, "EventSelector", false)?;
            for (i, (source, _)) in sources.iter().enumerate() {
                let data: Vec<String> = ["Timestamp", "FrameID"]
                    .iter()
                    .map(|suffix| self.event_feature(source, suffix))
                    .filter(|name| self.camera.kind(NodeMap::Camera, name).is_some())
                    .collect();
                self.camera.register_event(
                    &self.event_feature(source, "Data"),
                    i as i32,
                    i != 0,
                )?;
                events.push(data);
            }
        }
        *self.shared.events.write() = events;
        Ok(())
    }

    /// The one way out of a camera session short of dropping the driver:
    /// whatever ends it — removal, a failed open — the grab is over and the
    /// device closed. At IOC exit the port actor drops the driver, and
    /// `Camera`'s `Drop` destroys the device.
    fn camera_disconnected(&mut self, ad: &mut ADDriverBase) {
        self.device_is_reachable = false;
        // Waits for the grab-loop thread, which never waits on this one.
        self.camera.close();
        self.acquiring = false;
        let _ = ad.set_acquire(0);
        let base = &mut ad.port_base;
        let _ = base.set_int32_param(ad.params.status, 0, ADStatus::Disconnected as i32);
        let _ = base.set_string_param(ad.params.status_message, 0, "Camera disconnected");
    }

    fn push_convert(&self, ad: &ADDriverBase) {
        let get = |index| ad.port_base.get_int32_param(index, 0).unwrap_or(0);
        let format = get(self.params.convert_pixel_format);
        let pixel = match CONVERT_PIXEL_FORMATS.get(format as usize) {
            Some(pixel) => *pixel,
            None => {
                ad.port_base.trace_print(
                    TraceMask::ERROR,
                    &format!(
                        "{DRIVER_NAME}::processFrame Error: Unknown pixel conversion format {format}\n"
                    ),
                );
                Some(PixelFormat::Mono8)
            }
        };
        self.camera.set_convert(
            pixel,
            get(self.params.convert_bit_align),
            get(self.params.convert_shift_bits),
        );
    }
}

impl GenICamBackend for PylonBackend {
    fn create_node(
        &mut self,
        feature_name: &str,
        feature_type: GCFeatureType,
    ) -> Box<dyn GenICamNode> {
        Box::new(PylonNode::new(
            self.camera.clone(),
            feature_name,
            feature_type,
            self.shared.trace.clone(),
        ))
    }

    fn start_capture(&mut self, ctx: &mut DriverCtx<'_>) -> AsynResult<()> {
        // If we are already acquiring return immediately
        if self.acquiring {
            return Ok(());
        }
        let ad = &mut *ctx.ad;
        let p = ad.params;
        let image_mode = ImageMode::from_i32(ad.port_base.get_int32_param(p.image_mode, 0)?);
        let num_images = ad.port_base.get_int32_param(p.num_images, 0)?;

        ad.port_base.set_int32_param(p.num_images_counter, 0, 0)?;
        ad.set_shutter(true)?;
        let count = match image_mode {
            ImageMode::Single => Some(1),
            ImageMode::Multiple => Some(num_images.max(0) as u64),
            ImageMode::Continuous => None,
        };
        self.shared.begin_grab(count);
        if let Err(e) = self.camera.start(count) {
            ad.set_acquire(0)?;
            ad.port_base.trace_print(
                TraceMask::ERROR,
                &format!("{DRIVER_NAME}:startCapture: failed to start grabbing: {e}\n"),
            );
            return Err(error(e));
        }
        self.acquiring = true;
        // We are now waiting for an image
        ad.port_base
            .set_int32_param(p.status, 0, ADStatus::Waiting as i32)?;
        Ok(())
    }

    fn stop_capture(&mut self, ctx: &mut DriverCtx<'_>) -> AsynResult<()> {
        let ad = &mut *ctx.ad;
        ad.set_acquire(0)?;
        ad.set_shutter(false)?;
        // Waits for the grab-loop thread, which never waits on this one.
        let stopped = self.camera.stop();
        self.acquiring = false;
        ad.port_base
            .set_int32_param(ad.params.status, 0, ADStatus::Idle as i32)?;
        stopped.map_err(error)
    }

    fn connect(&mut self, ctx: &mut DriverCtx<'_>) -> AsynResult<bool> {
        // Try to connect the camera if it is previously disconnected.
        if self.device_is_reachable {
            return Ok(false);
        }
        if let Err(e) = self.connect_camera(ctx.ad) {
            ctx.ad.port_base.trace_print(
                TraceMask::ERROR,
                &format!("{DRIVER_NAME}:connect:  camera connection failed ({e})\n"),
            );
            ctx.ad.port_base.call_param_callbacks(0)?;
            return Err(e);
        }
        self.push_convert(ctx.ad);
        Ok(true)
    }

    fn disconnect(&mut self, ctx: &mut DriverCtx<'_>) {
        self.camera_disconnected(ctx.ad);
    }

    fn int32_written(&mut self, ctx: &mut DriverCtx<'_>, reason: usize, value: i32) {
        let p = self.params;
        if reason == p.time_stamp_mode {
            self.shared.time_stamp_mode.store(value, Ordering::Relaxed);
        } else if reason == p.unique_id_mode {
            self.shared.unique_id_mode.store(value, Ordering::Relaxed);
        } else if [
            p.convert_pixel_format,
            p.convert_bit_align,
            p.convert_shift_bits,
        ]
        .contains(&reason)
        {
            self.push_convert(ctx.ad);
        }
    }

    /// Some GenICam enum features can become unavailable depending on other
    /// settings. If they happen to be unavailable on startup, the records
    /// would stay with 0 choices even when the feature becomes available
    /// later, so they keep the static choices of the database, which come from
    /// the camera's GenICam XML.
    fn keep_db_enum(&self, feature_name: &str) -> bool {
        feature_name == "LineSource"
    }

    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        match Camera::enumerate() {
            Ok(devices) => {
                let _ = writeln!(out, "\nNumber of cameras detected: {}", devices.len());
                if level >= 1 {
                    for (i, d) in devices.iter().enumerate() {
                        let _ = writeln!(out, "Camera {i}");
                        let _ = writeln!(out, "            Name: {}", d.friendly_name);
                        let _ = writeln!(out, "           Model: {}", d.model);
                        let _ = writeln!(out, "        Serial #: {}", d.serial);
                        let _ = writeln!(out, "    Interface ID: {}", d.interface_id);
                    }
                }
            }
            Err(e) => {
                let _ = writeln!(out, "{DRIVER_NAME}::report exception {e}");
            }
        }
    }
}

pub type ADPylon = ADGenICam<PylonBackend>;

/// A running ADPylon port with its image task.
pub struct PylonRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub pylon_params: PylonParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<Mutex<NDArrayOutput>>,
    _task: std::thread::JoinHandle<()>,
}

impl PylonRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<Mutex<NDArrayOutput>> {
        &self.array_output
    }
}

/// C `ADPylonConfig`. `camera_id` of fewer than 4 digits is an index into the
/// cameras found, anything else a serial number; `max_memory` of 0 is
/// unlimited.
pub fn create_pylon(
    port_name: &str,
    camera_id: &str,
    max_memory: usize,
    array_output: NDArrayOutput,
) -> AsynResult<PylonRuntime> {
    let (tx, rx) = task::channel();
    let driver = ADPylon::new(port_name, max_memory, |ad, index| {
        PylonBackend::new(ad, index, camera_id, tx)
    })?;
    let ad_params = driver.ad.params;
    let pylon_params = driver.backend.params;
    let pool = driver.ad.pool.clone();
    let shared = driver.backend.shared.clone();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default())?;
    let array_output = Arc::new(Mutex::new(array_output));
    let task = ImageTask {
        rx,
        shared,
        port: runtime_handle.port_handle().clone(),
        publisher: ArrayPublisher::new(array_output.clone()),
        ad: ad_params,
        trace: Trace::new(port_name),
    }
    .start();

    Ok(PylonRuntime {
        runtime_handle,
        ad_params,
        pylon_params,
        pool,
        array_output,
        _task: task,
    })
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use ad_core_rs::color::{NDBayerPattern, NDColorMode};
    use ad_core_rs::ndarray::{NDDataType, NDDimension};
    use ad_core_rs::plugin::channel::NDArrayOutput;
    use asyn_rs::param::ParamValue;
    use asyn_rs::port::DrvUserRequest;
    use asyn_rs::request::ParamSetValue;

    use super::*;
    use crate::task::{GrabTag, Outcome};

    /// A result of the grab before the running one, still queued when the
    /// next grab started, as if Acquire had been toggled off and on quickly.
    #[test]
    fn a_result_of_an_earlier_grab_leaves_the_acquisition_alone() {
        // SAFETY: set before this test starts pylon; no other test here
        // reads the environment.
        unsafe { std::env::set_var("PYLON_CAMEMU", "1") };
        let rt = create_pylon("EMU_STALE", "0815-0000", 0, NDArrayOutput::new()).unwrap();
        let h = rt.port_handle();
        let bind = |drv_info: &str| {
            h.drv_user_create_blocking(&DrvUserRequest::new(drv_info, 0))
                .unwrap()
                .reason
        };
        bind("IMAGE_MODE");
        // Mono12 has no NDArray layout: every real result fails, and a long
        // Multiple acquisition keeps running.
        h.write_int32_blocking(bind("GC_E_PixelFormat"), 0, 0x0110_0005)
            .unwrap();
        let p = rt.ad_params;
        h.write_int32_blocking(p.image_mode, 0, ImageMode::Multiple as i32)
            .unwrap();
        h.write_int32_blocking(p.num_images, 0, 1000).unwrap();
        h.write_int32_blocking(p.acquire, 0, 1).unwrap();

        let shared = h
            .with_driver_blocking(|d: &mut ADPylon| d.backend.shared.clone())
            .unwrap();
        let stale = GrabTag {
            grab: shared.current_grab() - 1,
            last: true,
        };
        let array = rt
            .pool()
            .alloc(vec![NDDimension::new(4)], NDDataType::UInt8)
            .unwrap();
        let counter = h.read_int32_blocking(p.base.array_counter, 0).unwrap();
        for outcome in [
            Outcome::Frame {
                array,
                layout: (NDColorMode::Mono, NDBayerPattern::RGGB),
                updates: Vec::new(),
            },
            Outcome::Failed {
                updates: Vec::new(),
            },
            Outcome::NoBuffer {
                updates: Vec::new(),
            },
        ] {
            let msg = Msg::Result {
                tag: stale,
                outcome,
            };
            assert!(shared.tx.try_send(msg).is_ok());
        }
        // The image task takes messages in order: once this lands, the
        // stale ones have been handled.
        let marker = rt.pylon_params.convert_shift_bits;
        assert!(
            shared
                .tx
                .try_send(Msg::Params(vec![ParamSetValue::new(
                    marker,
                    0,
                    ParamValue::Int32(7)
                )]))
                .is_ok()
        );
        let deadline = Instant::now() + Duration::from_secs(10);
        while h.read_int32_blocking(marker, 0).ok() != Some(7) {
            assert!(
                Instant::now() < deadline,
                "timed out waiting for the marker"
            );
            std::thread::sleep(Duration::from_millis(20));
        }

        assert_eq!(h.read_int32_blocking(p.acquire, 0).unwrap(), 1);
        assert_eq!(
            h.read_int32_blocking(p.status, 0).unwrap(),
            ADStatus::Waiting as i32
        );
        assert_eq!(h.read_int32_blocking(p.num_images_counter, 0).unwrap(), 0);
        // The frame itself was still delivered, as C delivered it.
        assert_eq!(
            h.read_int32_blocking(p.base.array_counter, 0).unwrap(),
            counter + 1
        );
        h.write_int32_blocking(p.acquire, 0, 0).unwrap();
    }
}
