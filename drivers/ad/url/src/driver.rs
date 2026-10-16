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
        let mut ad = ADDriverBase::new(port_name, 0, 0, max_memory, ASYN_NONE)?;
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
        self.ad
            .port_base
            .set_int32_param(reason, user.addr, value)?;

        let status = if reason == self.ad.params.acquire {
            let status = self
                .ad
                .port_base
                .get_int32_param(self.ad.params.status, 0)
                .unwrap_or(ADStatus::Idle as i32);
            if value != 0 && status == ADStatus::Idle as i32 {
                let _ = self.acq_tx.try_send(AcqCommand::Start);
            } else if value == 0 && status != ADStatus::Idle as i32 {
                let _ = self.acq_tx.try_send(AcqCommand::Stop);
            }
            Ok(())
        } else if reason < self.url_params.url_name {
            // `ADDriver::writeInt32` -> `asynNDArrayDriver::writeInt32`.
            if reason == self.ad.params.shutter_control {
                self.ad.set_shutter(value != 0)
            } else {
                self.ad.write_int32_pool(reason, value).map(|_| ())
            }
        } else {
            Ok(())
        };

        self.ad.port_base.call_param_callbacks(0)?;
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
pub fn create_url_detector(claim: &PortClaim, max_memory: usize) -> AsynResult<URLRuntime> {
    let port_name = claim.name();
    let (acq_tx, acq_rx) = rt::command_channel::<AcqCommand>(16);

    let det = URLDriver::new(port_name, max_memory, acq_tx)?;
    let ad_params = det.ad.params;
    let url_params = det.url_params;
    let pool = det.ad.pool.clone();
    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::driver::ShutterMode;

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
}
