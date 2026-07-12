//! The PhotonII areaDetector driver (C `PhotonII`).
//!
//! Ownership: the p2util socket is reached only through its own asyn port
//! handle, whose actor serialises each operation, so a `writeRead` from the
//! parameter path can never interleave with a poll read from the acquisition
//! task. The acquisition task never touches the parameter library directly —
//! it goes through this port's actor, which is the only writer.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
use epics_rs::ad_core::runtime as rt;

use crate::connection::P2Util;
use crate::params::PhotonIIParams;
use crate::protocol;
use crate::raw;
use crate::task::{AcquisitionContext, start_acquisition_task};
use crate::types::*;

/// The acquisition task's command channel: the only thing the actor sends it.
pub(crate) enum TaskCommand {
    Start,
}

/// State shared between the port actor and the acquisition task.
pub struct SharedState {
    /// Set when Acquire goes to 0 during an acquisition; the task clears it
    /// when it starts the next one. Only the actor sets it, only the task
    /// clears it (C used an `epicsEvent` that both threads could consume).
    pub stop_requested: AtomicBool,
}

pub struct PhotonIIDetector {
    pub ad: ADDriverBase,
    pub params: PhotonIIParams,
    p2: P2Util,
    shared: Arc<SharedState>,
    commands: rt::CommandSender<TaskCommand>,
}

impl PhotonIIDetector {
    fn new(
        port_name: &str,
        p2: P2Util,
        max_memory: usize,
        shared: Arc<SharedState>,
        commands: rt::CommandSender<TaskCommand>,
    ) -> AsynResult<Self> {
        let mut ad =
            ADDriverBase::new(port_name, PII_SIZE_X as i32, PII_SIZE_Y as i32, max_memory)?;
        let params = PhotonIIParams::create(&mut ad.port_base)?;

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.manufacturer, 0, "Bruker".into())?;
        base.set_string_param(ad.params.base.model, 0, "PhotonII".into())?;
        base.set_string_param(
            ad.params.base.driver_version,
            0,
            env!("CARGO_PKG_VERSION").into(),
        )?;

        base.set_int32_param(ad.params.max_size_x, 0, PII_SIZE_X as i32)?;
        base.set_int32_param(ad.params.max_size_y, 0, PII_SIZE_Y as i32)?;
        base.set_int32_param(ad.params.size_x, 0, PII_SIZE_X as i32)?;
        base.set_int32_param(ad.params.size_y, 0, PII_SIZE_Y as i32)?;
        base.set_int32_param(ad.params.base.array_size_x, 0, PII_SIZE_X as i32)?;
        base.set_int32_param(ad.params.base.array_size_y, 0, PII_SIZE_Y as i32)?;
        base.set_int32_param(ad.params.base.array_size, 0, raw::frame_bytes() as i32)?;
        // C advertised NDUInt32 here while allocating every frame as NDInt32,
        // so the DataType readback disagreed with the arrays the plugins got.
        // The frames are signed 32-bit; say so.
        base.set_int32_param(ad.params.base.data_type, 0, NDDataType::Int32 as u8 as i32)?;
        base.set_int32_param(ad.params.image_mode, 0, 2)?; // ADImageContinuous
        base.set_int32_param(ad.params.status, 0, ADStatus::Idle as i32)?;

        Ok(Self {
            ad,
            params,
            p2,
            shared,
            commands,
        })
    }

    /// Send one command line to p2util and publish both sides of the exchange
    /// to the StringToServer / StringFromServer records (C `writePhotonII`).
    fn command(&mut self, command: &str) -> AsynResult<()> {
        let reply = match self.p2.write_read(command, COMMAND_TIMEOUT) {
            Ok(reply) => reply,
            Err(e) => {
                log::error!("photonii: '{command}' failed: {e}");
                String::new()
            }
        };
        let base = &mut self.ad.port_base;
        base.set_string_param(self.ad.params.string_to_server, 0, command.into())?;
        base.set_string_param(self.ad.params.string_from_server, 0, reply)?;
        Ok(())
    }

    fn get_i32(&self, reason: usize) -> i32 {
        self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0)
    }

    fn set_acquire(&mut self, value: i32, was_acquiring: bool) -> AsynResult<()> {
        if value != 0 && !was_acquiring {
            if self.commands.try_send(TaskCommand::Start).is_err() {
                log::error!("photonii: the acquisition task is not accepting commands");
            }
        } else if value == 0 && was_acquiring {
            self.shared.stop_requested.store(true, Ordering::Release);
            self.command(&protocol::abort())?;
        }
        Ok(())
    }
}

impl PortDriver for PhotonIIDetector {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        // Read Acquire before the write, as C does: the transition matters.
        let was_acquiring = self.get_i32(self.ad.params.acquire) != 0;
        self.ad
            .port_base
            .params
            .set_int32(reason, user.addr, value)?;

        if reason == self.ad.params.acquire {
            self.set_acquire(value, was_acquiring)?;
        } else if reason == self.params.shutter {
            self.ad.set_shutter(value != 0)?;
        } else if reason == self.params.dr_sum_enable {
            self.command(&protocol::set_dr_summation(value))?;
        } else if reason == self.ad.params.trigger_mode {
            match TriggerSource::from_i32(value) {
                Some(source) => self.command(&protocol::set_trigger_source(source))?,
                None => log::error!("photonii: unknown TriggerMode {value}"),
            }
        } else if reason == self.params.trigger_type {
            match TriggerType::from_i32(value) {
                Some(t) => self.command(&protocol::set_trigger_type(t))?,
                None => log::error!("photonii: unknown TriggerType {value}"),
            }
        } else if reason == self.params.trigger_edge {
            match TriggerEdge::from_i32(value) {
                Some(e) => self.command(&protocol::set_trigger_edge(e))?,
                None => log::error!("photonii: unknown TriggerEdge {value}"),
            }
        } else if reason == self.params.num_subframes {
            self.command(&protocol::set_num_subframes(value))?;
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        self.ad
            .port_base
            .params
            .set_float64(reason, user.addr, value)?;

        if reason == self.ad.params.acquire_time {
            self.command(&protocol::set_exposure_time(value))?;
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn write_octet(&mut self, user: &mut AsynUser, value: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        if reason == self.params.util {
            // The `p2util` iocsh command. C copied the line into a fixed 512-byte
            // buffer with strncpy and no terminator, then called strlen on it.
            let command = String::from_utf8_lossy(value).trim_end().to_string();
            self.command(&command)?;
            self.ad.port_base.call_param_callbacks(0)?;
            return Ok(value.len());
        }
        self.ad.port_base.params.set_string(
            reason,
            user.addr,
            String::from_utf8_lossy(value).into_owned(),
        )?;
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(value.len())
    }
}

impl ADDriver for PhotonIIDetector {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

/// What the IOC layer holds on to.
pub struct PhotonIIRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub params: PhotonIIParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    #[allow(dead_code)]
    task: std::thread::JoinHandle<()>,
}

impl PhotonIIRuntime {
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

/// Create the detector port and start its acquisition task.
///
/// `command_handle` is the `drvAsynIPPort` that reaches p2util (C
/// `PhotonIIConfig`'s `commandPort`).
pub fn create_photonii_detector(
    port_name: &str,
    command_handle: PortHandle,
    max_memory: usize,
) -> AsynResult<PhotonIIRuntime> {
    let shared = Arc::new(SharedState {
        stop_requested: AtomicBool::new(false),
    });
    let p2 = P2Util::new(command_handle);
    let (tx, rx) = rt::command_channel::<TaskCommand>(4);

    let det = PhotonIIDetector::new(port_name, p2.clone(), max_memory, shared.clone(), tx)?;
    let ad_params = det.ad.params;
    let params = det.params;
    let pool = det.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(det, RuntimeConfig::default());
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let queued_counter = Arc::new(QueuedArrayCounter::new());

    let task = start_acquisition_task(AcquisitionContext {
        p2,
        handle: runtime_handle.port_handle().clone(),
        output: ArrayPublisher::new(array_output.clone()),
        queued: queued_counter.clone(),
        ad_params,
        params,
        shared,
        commands: rx,
    });

    Ok(PhotonIIRuntime {
        runtime_handle,
        ad_params,
        params,
        pool,
        array_output,
        queued_counter,
        task,
    })
}
