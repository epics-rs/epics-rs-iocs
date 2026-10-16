//! The PhotonII areaDetector driver (C `PhotonII`).
//!
//! Ownership: the p2util socket is reached only through its own asyn port
//! handle, whose actor serialises each operation, so a `writeRead` from the
//! parameter path can never interleave with a poll read from the acquisition
//! task. The acquisition task never touches the parameter library directly —
//! it goes through this port's actor, which is the only writer.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynResult, AsynStatus};
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::runtime as rt;

use crate::connection::{ChannelError, P2Util};
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
        let mut ad = ADDriverBase::new(
            port_name,
            1,
            PII_SIZE_X as i32,
            PII_SIZE_Y as i32,
            max_memory,
            ASYN_CANBLOCK,
        )?;
        let params = PhotonIIParams::create(&mut ad.port_base)?;

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.manufacturer, 0, "Bruker")?;
        base.set_string_param(ad.params.base.model, 0, "PhotonII")?;
        base.set_string_param(ad.params.base.driver_version, 0, env!("CARGO_PKG_VERSION"))?;

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
                let status = match &e {
                    ChannelError::Timeout => AsynStatus::Timeout,
                    ChannelError::Transport(e) => e.status(),
                };
                self.ad.port_base.trace_print(
                    TraceMask::ERROR,
                    format_args!(
                        "PhotonII::writePhotonII, status={}, sent\n{command}",
                        status as i32
                    ),
                );
                String::new()
            }
        };
        let base = &mut self.ad.port_base;
        base.set_string_param(self.ad.params.string_to_server, 0, command)?;
        base.set_string_param(self.ad.params.string_from_server, 0, reply)?;
        Ok(())
    }

    fn get_i32(&self, reason: usize) -> i32 {
        self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0)
    }

    fn set_acquire(&mut self, value: i32, was_acquiring: bool) -> AsynResult<()> {
        if value != 0 && !was_acquiring {
            let _ = self.commands.try_send(TaskCommand::Start);
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
            .set_int32_param(reason, user.addr, value)?;

        let mut status = Ok(());
        if reason == self.ad.params.acquire {
            self.set_acquire(value, was_acquiring)?;
        } else if reason == self.params.shutter {
            status = self.ad.set_shutter(value != 0);
        } else if reason == self.params.dr_sum_enable {
            self.command(&protocol::set_dr_summation(value))?;
        } else if reason == self.ad.params.trigger_mode {
            if let Some(source) = TriggerSource::from_i32(value) {
                self.command(&protocol::set_trigger_source(source))?;
            }
        } else if reason == self.params.trigger_type {
            if let Some(t) = TriggerType::from_i32(value) {
                self.command(&protocol::set_trigger_type(t))?;
            }
        } else if reason == self.params.trigger_edge {
            if let Some(e) = TriggerEdge::from_i32(value) {
                self.command(&protocol::set_trigger_edge(e))?;
            }
        } else if reason == self.params.num_subframes {
            self.command(&protocol::set_num_subframes(value))?;
        } else if reason < self.params.dr_sum_enable {
            // FIRST_PII_PARAM is PII_DRSumEnable (PhotonII.h:47).
            status = ADDriver::ad_write_int32(self, user, value);
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "PhotonII::writeInt32: error, status={} function={reason}, value={value}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("PhotonII::writeInt32: function={reason}, value={value}"),
            ),
        }
        status
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let mut status = self
            .ad
            .port_base
            .set_float64_param(reason, user.addr, value);

        if reason == self.ad.params.acquire_time {
            let _ = self.command(&protocol::set_exposure_time(value));
        } else if reason < self.params.dr_sum_enable {
            // ADDriver::writeFloat64 is asynPortDriver's (PhotonII.cpp:549).
            status = self.ad.port_base.write_float64_param(user, value);
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "PhotonII::writeFloat64: error, status={} function={reason}, value={value:.6}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("PhotonII::writeFloat64: function={reason}, value={value:.6}"),
            ),
        }
        status
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
        // C has no writeOctet and inherits `ADDriver::writeOctet`.
        self.ad.write_octet(user, value)
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
    pool: DriverPool,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    #[allow(dead_code)]
    task: std::thread::JoinHandle<()>,
}

impl PhotonIIRuntime {
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

/// Create the detector port and start its acquisition task.
///
/// `command_handle` is the `drvAsynIPPort` that reaches p2util (C
/// `PhotonIIConfig`'s `commandPort`).
pub fn create_photonii_detector(
    claim: &PortClaim,
    command_handle: PortHandle,
    max_memory: usize,
) -> AsynResult<PhotonIIRuntime> {
    let port_name = claim.name();
    let shared = Arc::new(SharedState {
        stop_requested: AtomicBool::new(false),
    });
    let p2 = P2Util::new(command_handle);
    let (tx, rx) = rt::command_channel::<TaskCommand>(4);

    let det = PhotonIIDetector::new(port_name, p2.clone(), max_memory, shared.clone(), tx)?;
    let ad_params = det.ad.params;
    let params = det.params;
    let pool = det.ad.pool.clone();
    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let task = start_acquisition_task(AcquisitionContext {
        p2,
        handle: runtime_handle.port_handle().clone(),
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
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
        task,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::driver::ShutterMode;
    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::registry::claim_port;

    fn detector(name: &str) -> PhotonIIDetector {
        let ip = format!("{name}_IP");
        let (runtime, _) = create_port_runtime(
            &claim_port(&ip).unwrap(),
            DrvAsynIPPort::new(&ip, "127.0.0.1:1").unwrap(),
            RuntimeConfig::default(),
        )
        .unwrap();
        let p2 = P2Util::new(runtime.port_handle().clone());
        std::mem::forget(runtime);
        let shared = Arc::new(SharedState {
            stop_requested: AtomicBool::new(false),
        });
        let (tx, _rx) = rt::command_channel::<TaskCommand>(4);
        PhotonIIDetector::new(name, p2, 0, shared, tx).unwrap()
    }

    #[test]
    fn a_base_class_write_reaches_addriver() {
        let mut det = detector("PII_BASE_WRITE");
        let ad = det.ad.params;
        det.ad
            .port_base
            .set_int32_param(ad.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        det.write_int32(&mut AsynUser::new(ad.shutter_control), 1)
            .unwrap();
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(ad.shutter_control_epics, 0)
                .unwrap(),
            1,
            "ADShutterControl did not reach ADDriver::setShutter"
        );
    }

    /// ADDriver::writeFloat64 is asynPortDriver's (PhotonII.cpp:549), whose
    /// IO_DRIVER line precedes PhotonII's.
    #[test]
    fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let port = "PII_BASE_F64";
        let mut det = detector(port);
        let manager = epics_rs::asyn::services::PortServices::global()
            .trace()
            .clone();
        let path = std::env::temp_dir().join(format!("pii-trace-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        let gain = det.ad.params.gain;
        let mut user = AsynUser::new(gain).with_addr(0);
        user.trace = Some(UserTrace {
            manager,
            port: port.into(),
        });
        det.write_float64(&mut user, 2.0).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let base =
            format!("asynPortDriver:writeFloat64: function={gain}, name=GAIN, value=2.000000");
        let own = format!("PhotonII::writeFloat64: function={gain}, value=2.000000");
        let at = |line: &str| {
            text.find(line)
                .unwrap_or_else(|| panic!("{line}: {text:?}"))
        };
        assert!(at(&base) < at(&own));
    }
}
