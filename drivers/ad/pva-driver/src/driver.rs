use std::sync::Arc;

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
use epics_rs::ad_core::runtime as rt;

use crate::params::PvaParams;
use crate::task::{MonitorContext, start_pva_task};
use crate::types::PvaCommand;

/// C++ `#define DRIVER_VERSION 1` / `DRIVER_REVISION 6` / `DRIVER_MODIFICATION 0`,
/// formatted the same way as `epicsSnprintf(versionString, ..., "%d.%d.%d", ...)`.
const DRIVER_VERSION_STRING: &str = "1.6.0";

pub struct PvaDriver {
    pub ad: ADDriverBase,
    pub pva_params: PvaParams,
    cmd_tx: rt::CommandSender<PvaCommand>,
}

impl PvaDriver {
    /// `max_memory`: NDArrayPool byte budget (C++ `maxMemory`).
    ///
    /// C++ `pvaDriver::pvaDriver` never sets `ADMaxSizeX`/`ADMaxSizeY` up
    /// front (they track the most recently received frame's shape instead,
    /// starting at 0) — matches `ADDriverBase::new`'s own `max_size_x =
    /// max_size_y = 0` default.
    pub fn new(
        port_name: &str,
        pv_name: &str,
        max_memory: usize,
        cmd_tx: rt::CommandSender<PvaCommand>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(port_name, 1, 0, 0, max_memory, ASYN_CANBLOCK)?;
        let pva_params = PvaParams::create(&mut ad.port_base)?;

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.manufacturer, 0, "PVAccess driver")?;
        base.set_string_param(ad.params.base.model, 0, "Basic PVAccess driver")?;
        // C++ explicitly overrides NDDriverVersion with its own DRIVER_VERSION.
        // DRIVER_REVISION.DRIVER_MODIFICATION, distinct from the ad-core-rs/
        // Cargo package version ADDriverBase::new() defaults it to.
        base.set_string_param(ad.params.base.driver_version, 0, DRIVER_VERSION_STRING)?;
        // C++ uses the PvAccess protocol version (EPICS_PVA_MAJOR_VERSION.
        // EPICS_PVA_MINOR_VERSION.EPICS_PVA_MAINTENANCE_VERSION) as the SDK
        // version. `epics_pva_rs::VERSION` (this crate's own package version)
        // is the equivalent constant in this port — its doc comment states it
        // mirrors pvxs's own `version_int()`.
        base.set_string_param(ad.params.base.sdk_version, 0, epics_rs::pva::VERSION)?;
        base.set_string_param(ad.params.base.serial_number, 0, "No serial number")?;
        base.set_string_param(ad.params.base.firmware_version, 0, "No firmware")?;
        base.set_string_param(pva_params.pv_name, 0, pv_name)?;

        Ok(Self {
            ad,
            pva_params,
            cmd_tx,
        })
    }
}

impl PortDriver for PvaDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    /// Mirrors C++ `pvaDriver::writeInt32`: only `ADAcquire` is special-cased
    /// (`value != 0` resets `ADNumImagesCounter` and starts the monitor task;
    /// otherwise stops it); every other Int32 param goes to
    /// `ADDriver::writeInt32`.
    /// Unlike `URLDriver`, there is no `ADStatus`/`ADAcquireBusy` gate here —
    /// `pvaDriver.cpp` never references either param.
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        self.ad
            .port_base
            .set_int32_param(reason, user.addr, value)?;

        if reason == self.ad.params.acquire {
            if value != 0 {
                self.ad.port_base.set_int32_param(
                    self.ad.params.num_images_counter,
                    user.addr,
                    0,
                )?;
                let _ = self.cmd_tx.try_send(PvaCommand::Start);
            } else {
                let _ = self.cmd_tx.try_send(PvaCommand::Stop);
            }
        } else if reason < self.pva_params.overrun_counter {
            // `ADDriver::writeInt32` -> `asynNDArrayDriver::writeInt32`. C
            // overwrites its status with `callParamCallbacks()`'s.
            let _ = if reason == self.ad.params.shutter_control {
                self.ad.set_shutter(value != 0)
            } else {
                self.ad.write_int32_pool(reason, value).map(|_| ())
            };
        }

        self.ad.port_base.call_param_callbacks(0)?;
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("pvaDriver:writeInt32: function={reason}, value={value}"),
        );
        Ok(())
    }

    /// Mirrors C++ `pvaDriver::writeOctet`: sets the string param
    /// unconditionally first; a `PVAPvName` write additionally triggers
    /// `connectPv()` (here, `PvaCommand::Reconnect` sent to the monitor
    /// task), reverting the param back to the old value if that connect
    /// attempt cannot even be dispatched (mirrors `connectPv()`'s exception
    /// path, which leaves `m_pvName`/`m_channel` untouched on failure).
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let new_value = String::from_utf8_lossy(data).into_owned();

        if reason == self.pva_params.pv_name {
            let old_value = self
                .ad
                .port_base
                .get_string_param(reason, user.addr)?
                .to_vec();
            self.ad
                .port_base
                .set_string_param(reason, user.addr, new_value.clone())?;
            if self
                .cmd_tx
                .try_send(PvaCommand::Reconnect(new_value))
                .is_err()
            {
                self.ad
                    .port_base
                    .set_string_param(reason, user.addr, old_value)?;
            }
        } else {
            let n = self.ad.write_octet(user, data)?;
            trace_octet(user, data);
            return Ok(n);
        }

        trace_octet(user, data);
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(data.len())
    }
}

/// The `ASYN_TRACEIO_DRIVER` line `pvaDriver::writeOctet` ends with; a
/// failure only fills `errorMessage`, so it prints nothing.
fn trace_octet(user: &AsynUser, value: &[u8]) {
    user.print(
        TraceMask::IO_DRIVER,
        file!(),
        line!(),
        format_args!(
            "pvaDriver:writeOctet: function={}, value={}",
            user.reason,
            String::from_utf8_lossy(value)
        ),
    );
}

impl ADDriver for PvaDriver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

pub struct PvaRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub pva_params: PvaParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    #[allow(dead_code)] // kept alive for the monitor task's lifetime
    task_handle: Option<std::thread::JoinHandle<()>>,
}

impl PvaRuntime {
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

/// Create the PVAccess detector port and start its monitor task.
///
/// `max_memory` is C++ `pvaDriverConfig`'s `maxMemory` argument. C++'s
/// `maxBuffers`/`priority`/`stackSize` arguments have no equivalent in this
/// framework (`ADDriverBase::new` takes only a byte budget, and the monitor
/// task always runs on its own `rt::run_thread_named` OS thread at default
/// priority/stack size) — same framework limitation as the rest of this
/// workspace's AD ports (e.g. ad-url).
pub fn create_pva_detector(
    claim: &PortClaim,
    pv_name: &str,
    max_memory: usize,
) -> AsynResult<PvaRuntime> {
    let port_name = claim.name();
    let (cmd_tx, cmd_rx) = rt::command_channel::<PvaCommand>(16);

    let det = PvaDriver::new(port_name, pv_name, max_memory, cmd_tx)?;
    let ad_params = det.ad.params;
    let pva_params = det.pva_params;
    let pool = det.ad.pool.clone();
    let queued_counter = det.ad.queued_counter.clone();
    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let task_handle = start_pva_task(MonitorContext {
        cmd_rx,
        handle: runtime_handle.port_handle().clone(),
        publisher: ArrayPublisher::new(array_output.clone()),
        ad: ad_params,
        pva: pva_params,
        initial_pv_name: pv_name.to_string(),
    });

    Ok(PvaRuntime {
        runtime_handle,
        ad_params,
        pva_params,
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
        let (cmd_tx, _cmd_rx) = rt::command_channel(1);
        let mut det = PvaDriver::new("PVA_BASE_WRITE", "", 1 << 20, cmd_tx).unwrap();
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
