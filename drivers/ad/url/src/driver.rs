use std::sync::Arc;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
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
        let mut ad = ADDriverBase::new(port_name, 0, 0, max_memory)?;
        let url_params = URLParams::create(&mut ad.port_base)?;

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.manufacturer, 0, "URL Driver".into())?;
        // Deviation from C++ (was "GraphicsMagick"): this port decodes with the
        // `image` crate and fetches with `ureq` instead of GraphicsMagick, so the
        // literal upstream string would misidentify the backend.
        base.set_string_param(ad.params.base.model, 0, "image-rs".into())?;
        base.set_string_param(ad.params.base.serial_number, 0, "No serial number".into())?;
        base.set_string_param(ad.params.base.firmware_version, 0, "No firmware".into())?;
        // Deviation from C++ (was GraphicsMagick's `MagickLibVersionText`): no
        // equivalent version macro exists for the substituted Rust crates.
        base.set_string_param(ad.params.base.sdk_version, 0, "image 0.25 + ureq 2".into())?;
        // C++ explicitly overrides NDDriverVersion with its own DRIVER_VERSION.
        // DRIVER_REVISION.DRIVER_MODIFICATION, distinct from the ad-core-rs/
        // Cargo package version ADDriverBase::new() defaults it to.
        base.set_string_param(
            ad.params.base.driver_version,
            0,
            DRIVER_VERSION_STRING.into(),
        )?;

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

    /// Mirrors C++ `URLDriver::writeInt32`: only `ADAcquire` is special-cased
    /// (start/stop-signals the acquisition task, gated on `ADStatus` being
    /// idle or not); every other Int32 param falls through to the same
    /// "set param, call callbacks" behavior as the base `ADDriver::writeInt32`
    /// (this crate's `PortDriver::write_int32` default already does that).
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        self.ad
            .port_base
            .params
            .set_int32(reason, user.addr, value)?;

        if reason == self.ad.params.acquire {
            let status = self
                .ad
                .port_base
                .get_int32_param(self.ad.params.status, 0)
                .unwrap_or(ADStatus::Idle as i32);
            if value != 0 && status == ADStatus::Idle as i32 {
                if self.acq_tx.try_send(AcqCommand::Start).is_err() {
                    log::error!("ad-url: acquisition task is not running");
                }
            } else if value == 0
                && status != ADStatus::Idle as i32
                && self.acq_tx.try_send(AcqCommand::Stop).is_err()
            {
                log::error!("ad-url: acquisition task is not running");
            }
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
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
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    #[allow(dead_code)] // kept alive for the acquisition task's lifetime
    task_handle: Option<std::thread::JoinHandle<()>>,
}

impl URLRuntime {
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

/// Create the URL detector port and start its acquisition task.
///
/// `max_memory` is C++ `URLDriverConfig`'s `maxMemory` argument. C++'s
/// `maxBuffers`/`priority`/`stackSize` arguments have no equivalent in this
/// framework (`ADDriverBase::new` takes only a byte budget, and the
/// acquisition task always runs on its own `rt::run_thread_named` OS thread
/// at default priority/stack size) — accepted but unused, same framework
/// limitation as the rest of this workspace's AD ports (e.g. d435i).
pub fn create_url_detector(port_name: &str, max_memory: usize) -> AsynResult<URLRuntime> {
    let (acq_tx, acq_rx) = rt::command_channel::<AcqCommand>(16);

    let det = URLDriver::new(port_name, max_memory, acq_tx)?;
    let ad_params = det.ad.params;
    let url_params = det.url_params;
    let pool = det.ad.pool.clone();
    let (runtime_handle, _) = create_port_runtime(det, RuntimeConfig::default());
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let queued_counter = Arc::new(QueuedArrayCounter::new());

    let task_handle = start_acquisition_task(AcquisitionContext {
        acq_rx,
        handle: runtime_handle.port_handle().clone(),
        publisher: ArrayPublisher::new(array_output.clone()),
        queued: queued_counter.clone(),
        ad: ad_params,
        url: url_params,
    });

    Ok(URLRuntime {
        runtime_handle,
        ad_params,
        url_params,
        pool,
        array_output,
        queued_counter,
        task_handle: Some(task_handle),
    })
}
