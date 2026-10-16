use std::sync::Arc;

use epics_rs::ad_core::plugin::runtime::ASYN_NONE;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus};
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::runtime as rt;

use crate::params::URLParams;
use crate::task::{AcquisitionContext, start_acquisition_task};
use crate::types::AcqCommand;

/// C++ `#define DRIVER_VERSION 2` / `DRIVER_REVISION 3` / `DRIVER_MODIFICATION 0`,
/// formatted the same way as `epicsSnprintf(versionString, ..., "%d.%d.%d", ...)`.
const DRIVER_VERSION_STRING: &str = "2.3.0";

pub struct URLDriver {
    pub ad: ADDriverBase,
    pub url_params: URLParams,
    acq_tx: rt::CommandSender<AcqCommand>,
}

impl URLDriver {
    /// `max_memory`: NDArrayPool byte budget (C++ `maxMemory`).
    ///
    /// C++ `URLDriver::URLDriver` never sets `ADMaxSizeX`/`ADMaxSizeY` — the
    /// image size is unknown until the first successful fetch, so this
    /// driver constructs `ADDriverBase` with `max_size_x = max_size_y = 0`
    /// (matching upstream) rather than requiring a fixed size up front.
    pub fn new(
        port_name: &str,
        max_memory: usize,
        acq_tx: rt::CommandSender<AcqCommand>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(port_name, 1, 0, 0, max_memory, ASYN_NONE)?;
        let url_params = URLParams::create(&mut ad.port_base)?;

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.manufacturer, 0, "URL Driver")?;
        // Deviation from C++ (was "GraphicsMagick"): this port decodes with the
        // `image` crate and fetches with `ureq` instead of GraphicsMagick, so the
        // literal upstream string would misidentify the backend.
        base.set_string_param(ad.params.base.model, 0, "image-rs")?;
        base.set_string_param(ad.params.base.serial_number, 0, "No serial number")?;
        base.set_string_param(ad.params.base.firmware_version, 0, "No firmware")?;
        // Deviation from C++ (was GraphicsMagick's `MagickLibVersionText`): no
        // equivalent version macro exists for the substituted Rust crates.
        base.set_string_param(ad.params.base.sdk_version, 0, "image 0.25 + ureq 2")?;
        // C++ explicitly overrides NDDriverVersion with its own DRIVER_VERSION.
        // DRIVER_REVISION.DRIVER_MODIFICATION, distinct from the ad-core-rs/
        // Cargo package version ADDriverBase::new() defaults it to.
        base.set_string_param(ad.params.base.driver_version, 0, DRIVER_VERSION_STRING)?;

        Ok(Self {
            ad,
            url_params,
            acq_tx,
        })
    }
}

impl PortDriver for URLDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    /// C inherits `ADDriver::writeOctet`.
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        self.ad.write_octet(user, data)
    }

    /// C++ `URLDriver::writeInt32`: only `ADAcquire` is special-cased
    /// (start/stop-signals the acquisition task, gated on `ADStatus` being
    /// idle or not); every other Int32 param goes to `ADDriver::writeInt32`.
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        // C keeps setIntegerParam's status unless ADDriver::writeInt32
        // replaces it, and acts on the write either way.
        let mut status = self.ad.port_base.set_int32_param(reason, user.addr, value);

        if reason == self.ad.params.acquire {
            let adstatus = self
                .ad
                .port_base
                .get_int32_param(self.ad.params.status, 0)
                .unwrap_or(ADStatus::Idle as i32);
            if value != 0 && adstatus == ADStatus::Idle as i32 {
                let _ = self.acq_tx.try_send(AcqCommand::Start);
            } else if value == 0 && adstatus != ADStatus::Idle as i32 {
                let _ = self.acq_tx.try_send(AcqCommand::Stop);
            }
        } else if reason < self.url_params.url_name {
            status = ADDriver::ad_write_int32(self, user, value);
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "URLDriver:writeInt32 error, status={} function={reason}, value={value}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("URLDriver:writeInt32: function={reason}, value={value}"),
            ),
        }
        status
    }
}

impl ADDriver for URLDriver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

pub struct URLRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub url_params: URLParams,
    pool: DriverPool,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    #[allow(dead_code)] // kept alive for the acquisition task's lifetime
    task_handle: Option<std::thread::JoinHandle<()>>,
}

impl URLRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &DriverPool {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, sender: NDArraySender) {
        self.array_output.lock().add(sender);
    }
}

/// Create the URL detector port and start its acquisition task.
///
/// `max_memory` is C++ `URLDriverConfig`'s `maxMemory` argument. C++'s
/// `maxBuffers`/`priority`/`stackSize` arguments have no equivalent in this
/// framework (`ADDriverBase::new` takes only a byte budget, and the
/// acquisition task always runs on its own `rt::run_thread_named` OS thread
/// at default priority/stack size) — accepted but unused, same framework
/// limitation as the rest of this workspace's AD ports (e.g. d435i).
pub fn create_url_detector(claim: &PortClaim, max_memory: usize) -> AsynResult<URLRuntime> {
    let port_name = claim.name();
    let (acq_tx, acq_rx) = rt::command_channel::<AcqCommand>(16);

    let det = URLDriver::new(port_name, max_memory, acq_tx)?;
    let ad_params = det.ad.params;
    let url_params = det.url_params;
    let pool = det.ad.pool.clone();
    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let task_handle = start_acquisition_task(AcquisitionContext {
        acq_rx,
        handle: runtime_handle.port_handle().clone(),
        publisher: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
        ad: ad_params,
        url: url_params,
    });

    Ok(URLRuntime {
        runtime_handle,
        ad_params,
        url_params,
        pool,
        array_output,
        task_handle: Some(task_handle),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::driver::ShutterMode;
    use epics_rs::asyn::error::AsynError;

    #[test]
    fn a_base_class_write_reaches_addriver() {
        let (acq_tx, _acq_rx) = rt::command_channel(1);
        let mut det = URLDriver::new("URL_BASE_WRITE", 1 << 20, acq_tx).unwrap();
        let p = det.ad.params;
        det.ad
            .port_base
            .set_int32_param(p.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        let mut user = AsynUser::new(p.shutter_control);
        det.write_int32(&mut user, 1).unwrap();
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(p.shutter_control_epics, 0)
                .unwrap(),
            1,
            "ADShutterControl did not reach ADDriver::setShutter"
        );
    }

    /// URLDriver.cpp:305-319: setIntegerParam's refusal is the write's
    /// status, and the acquire event is still signalled.
    #[test]
    fn a_refused_acquire_set_still_starts_and_fails_the_write() {
        let (acq_tx, mut acq_rx) = rt::command_channel(1);
        let mut det = URLDriver::new("URL_REFUSED_SET", 1 << 20, acq_tx).unwrap();
        let mut user = AsynUser::new(det.ad.params.acquire).with_addr(1);
        assert!(matches!(
            det.write_int32(&mut user, 1),
            Err(AsynError::ParamInvalidList(1))
        ));
        assert!(matches!(acq_rx.try_recv(), Ok(AcqCommand::Start)));
    }

    #[test]
    fn a_counter_written_during_acquisition_is_where_the_next_frame_counts_from() {
        use epics_rs::ad_core::driver::ImageMode;
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::param::ParamValue;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::request::ParamSetValue;

        let dir = std::env::temp_dir().join(format!("url-counter-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let png = dir.join("frame.png");
        image::DynamicImage::ImageLuma8(image::GrayImage::new(2, 2))
            .save(&png)
            .unwrap();

        let rt = create_url_detector(&claim_port("URL_COUNTER").unwrap(), 1 << 20).unwrap();
        let (tx, mut rx) = ndarray_channel("URL_COUNTER_SINK", 8);
        rt.connect_downstream(tx);
        let (h, ad) = (rt.port_handle(), rt.ad_params);
        h.set_params_and_notify_blocking(
            0,
            vec![
                ParamSetValue::new(
                    rt.url_params.url_name,
                    0,
                    ParamValue::Octet(format!("file://{}", png.display()).into_bytes().into()),
                ),
                ParamSetValue::new(ad.base.array_callbacks, 0, ParamValue::Int32(1)),
                ParamSetValue::new(
                    ad.image_mode,
                    0,
                    ParamValue::Int32(ImageMode::Continuous as i32),
                ),
                ParamSetValue::new(ad.acquire_period, 0, ParamValue::Float64(0.2)),
            ],
        )
        .unwrap();

        h.write_int32_blocking(ad.acquire, 0, 1).unwrap();
        let first = rx.blocking_recv().expect("first frame").unique_id;
        h.write_int32_blocking(ad.base.array_counter, 0, 100)
            .unwrap();
        let mut next = rx.blocking_recv().expect("next frame").unique_id;
        // A frame already past its counter read when the write landed.
        if next == first + 1 {
            next = rx.blocking_recv().expect("next frame").unique_id;
        }
        h.write_int32_blocking(ad.acquire, 0, 0).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(next, 101);
        std::mem::forget(rt);
    }

    #[test]
    fn wait_for_plugins_holds_acquire_busy_until_the_frame_is_released() {
        use epics_rs::ad_core::driver::ImageMode;
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::param::ParamValue;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::request::ParamSetValue;
        use std::time::{Duration, Instant};

        let dir = std::env::temp_dir().join(format!("url-busy-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let png = dir.join("frame.png");
        image::DynamicImage::ImageLuma8(image::GrayImage::new(2, 2))
            .save(&png)
            .unwrap();

        let rt = create_url_detector(&claim_port("URL_BUSY").unwrap(), 1 << 20).unwrap();
        let (tx, mut rx) = ndarray_channel("URL_BUSY_SINK", 8);
        rt.connect_downstream(tx);
        let (h, ad) = (rt.port_handle(), rt.ad_params);
        h.set_params_and_notify_blocking(
            0,
            vec![
                ParamSetValue::new(
                    rt.url_params.url_name,
                    0,
                    ParamValue::Octet(format!("file://{}", png.display()).into_bytes().into()),
                ),
                ParamSetValue::new(ad.base.array_callbacks, 0, ParamValue::Int32(1)),
                ParamSetValue::new(ad.base.wait_for_plugins, 0, ParamValue::Int32(1)),
                ParamSetValue::new(
                    ad.image_mode,
                    0,
                    ParamValue::Int32(ImageMode::Single as i32),
                ),
            ],
        )
        .unwrap();

        let read = |reason| h.read_int32_blocking(reason, 0).unwrap();
        let wait_for = |reason, value| {
            let deadline = Instant::now() + Duration::from_secs(10);
            while read(reason) != value {
                assert!(
                    Instant::now() < deadline,
                    "param {reason} never became {value}"
                );
                std::thread::sleep(Duration::from_millis(10));
            }
        };
        h.write_int32_blocking(ad.acquire, 0, 1).unwrap();
        wait_for(ad.acquire, 0);
        // Idle is the task's last store of the acquisition.
        wait_for(ad.status, ADStatus::Idle as i32);
        assert_eq!(read(ad.acquire_busy), 1, "busy cleared with a frame queued");
        drop(rx.blocking_recv().expect("frame"));
        wait_for(ad.acquire_busy, 0);
        let _ = std::fs::remove_dir_all(&dir);
        std::mem::forget(rt);
    }

    /// C keeps the last image in pArrays[0] until the next alloc
    /// (URLDriver.cpp:132-134), published or not.
    #[test]
    fn the_last_image_stays_charged_after_the_acquisition() {
        use epics_rs::ad_core::driver::ImageMode;
        use epics_rs::asyn::param::ParamValue;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::request::ParamSetValue;
        use std::time::{Duration, Instant};

        let dir = std::env::temp_dir().join(format!("url-last-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let png = dir.join("frame.png");
        image::DynamicImage::ImageLuma8(image::GrayImage::new(2, 2))
            .save(&png)
            .unwrap();

        let rt = create_url_detector(&claim_port("URL_LAST").unwrap(), 1 << 20).unwrap();
        let (h, ad) = (rt.port_handle(), rt.ad_params);
        h.set_params_and_notify_blocking(
            0,
            vec![
                ParamSetValue::new(
                    rt.url_params.url_name,
                    0,
                    ParamValue::Octet(format!("file://{}", png.display()).into_bytes().into()),
                ),
                ParamSetValue::new(ad.base.array_callbacks, 0, ParamValue::Int32(0)),
                ParamSetValue::new(
                    ad.image_mode,
                    0,
                    ParamValue::Int32(ImageMode::Single as i32),
                ),
            ],
        )
        .unwrap();

        h.write_int32_blocking(ad.acquire, 0, 1).unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        while h.read_int32_blocking(ad.base.array_counter, 0).unwrap() != 1
            || h.read_int32_blocking(ad.status, 0).unwrap() != ADStatus::Idle as i32
        {
            assert!(Instant::now() < deadline, "no image");
            std::thread::sleep(Duration::from_millis(10));
        }
        let _ = std::fs::remove_dir_all(&dir);
        let pool = rt.pool();
        assert_eq!(pool.num_alloc_buffers() - pool.num_free_buffers(), 1);

        // pArrays[0] is preAllocateBuffers' template (asynNDArrayDriver.cpp:717-726).
        let p = &ad.base;
        h.write_int32_blocking(p.pool_num_pre_alloc_buffers, 0, 2)
            .unwrap();
        let free = pool.num_free_buffers();
        h.write_int32_blocking(p.pool_pre_alloc, 0, 1).unwrap();
        assert_eq!(pool.num_free_buffers(), free + 2);
        std::mem::forget(rt);
    }
}
