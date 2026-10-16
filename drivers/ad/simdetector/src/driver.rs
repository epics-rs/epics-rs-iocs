//! Port of `simDetector` (the `asynPortDriver` half of `simDetector.cpp`).

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

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::runtime as rt;

use crate::params::SimParams;
use crate::shutter::{ShutterOp, shutter_op};
use crate::task::{SimTaskContext, start_sim_task};
use crate::types::Signal;

/// `DRIVER_VERSION.DRIVER_REVISION.DRIVER_MODIFICATION` (simDetector.cpp:26-28).
pub const DRIVER_VERSION: &str = "2.11.0";

pub struct SimDetector {
    pub ad: ADDriverBase,
    pub sim: SimParams,
    /// C `startEventId_` / `stopEventId_`. A full capacity-1 channel is the
    /// binary-semaphore "already signalled" state, so a failed `try_send` is
    /// exactly C's second `epicsEventSignal` being a no-op.
    start_tx: rt::CommandSender<Signal>,
    stop_tx: rt::CommandSender<Signal>,
}

impl SimDetector {
    pub fn new(
        port_name: &str,
        max_size_x: i32,
        max_size_y: i32,
        data_type: NDDataType,
        max_memory: usize,
        start_tx: rt::CommandSender<Signal>,
        stop_tx: rt::CommandSender<Signal>,
    ) -> AsynResult<Self> {
        let mut ad =
            ADDriverBase::new(port_name, 1, max_size_x, max_size_y, max_memory, ASYN_NONE)?;
        let sim = SimParams::create(&mut ad.port_base)?;

        let p = ad.params;
        let base = &mut ad.port_base;

        // simDetector.cpp:1093-1100
        base.set_string_param(p.base.manufacturer, 0, "Simulated detector")?;
        base.set_string_param(p.base.model, 0, "Basic simulator")?;
        base.set_string_param(p.base.driver_version, 0, DRIVER_VERSION)?;
        base.set_string_param(p.base.sdk_version, 0, DRIVER_VERSION)?;
        base.set_string_param(p.base.serial_number, 0, "No serial number")?;
        base.set_string_param(p.base.firmware_version, 0, "No firmware")?;

        // simDetector.cpp:1102-1134
        base.set_int32_param(p.max_size_x, 0, max_size_x)?;
        base.set_int32_param(p.max_size_y, 0, max_size_y)?;
        base.set_int32_param(p.min_x, 0, 0)?;
        base.set_int32_param(p.min_y, 0, 0)?;
        base.set_int32_param(p.bin_x, 0, 1)?;
        base.set_int32_param(p.bin_y, 0, 1)?;
        base.set_int32_param(p.reverse_x, 0, 0)?;
        base.set_int32_param(p.reverse_y, 0, 0)?;
        base.set_int32_param(p.size_x, 0, max_size_x)?;
        base.set_int32_param(p.size_y, 0, max_size_y)?;
        base.set_int32_param(p.base.array_size_x, 0, max_size_x)?;
        base.set_int32_param(p.base.array_size_y, 0, max_size_y)?;
        base.set_int32_param(p.base.array_size, 0, 0)?;
        base.set_int32_param(p.base.data_type, 0, data_type as u8 as i32)?;
        base.set_int32_param(p.image_mode, 0, ImageMode::Continuous as i32)?;
        base.set_float64_param(p.acquire_time, 0, 0.001)?;
        base.set_float64_param(p.acquire_period, 0, 0.005)?;
        base.set_int32_param(p.num_images, 0, 100)?;
        base.set_int32_param(sim.reset_image, 0, 1)?;
        base.set_float64_param(sim.gain_x, 0, 1.0)?;
        base.set_float64_param(sim.gain_y, 0, 1.0)?;
        base.set_float64_param(sim.gain_red, 0, 1.0)?;
        base.set_float64_param(sim.gain_green, 0, 1.0)?;
        base.set_float64_param(sim.gain_blue, 0, 1.0)?;
        base.set_int32_param(sim.mode, 0, 0)?;
        base.set_int32_param(sim.peak_start_x, 0, 1)?;
        base.set_int32_param(sim.peak_start_y, 0, 1)?;
        base.set_int32_param(sim.peak_width_x, 0, 10)?;
        base.set_int32_param(sim.peak_width_y, 0, 20)?;
        base.set_int32_param(sim.peak_num_x, 0, 1)?;
        base.set_int32_param(sim.peak_num_y, 0, 1)?;
        base.set_int32_param(sim.peak_step_x, 0, 1)?;
        base.set_int32_param(sim.peak_step_y, 0, 1)?;

        Ok(Self {
            ad,
            sim,
            start_tx,
            stop_tx,
        })
    }
}

impl PortDriver for SimDetector {
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

    /// `simDetector::writeInt32` (simDetector.cpp:891-957).
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let p = self.ad.params;
        let sim = self.sim;

        // C ignores these statuses and gets 0 (simDetector.cpp:900-902).
        let acquiring = self.ad.port_base.get_int32_param(p.acquire, 0).unwrap_or(0) != 0;
        let image_mode = ImageMode::from_i32(
            self.ad
                .port_base
                .get_int32_param(p.image_mode, 0)
                .unwrap_or(0),
        );

        // "Ensure that ADStatus is set correctly before we set ADAcquire."
        if reason == p.acquire {
            if value != 0 && !acquiring {
                self.ad
                    .port_base
                    .set_string_param(p.status_message, 0, "Acquiring data")?;
            }
            if value == 0 && acquiring {
                self.ad
                    .port_base
                    .set_string_param(p.status_message, 0, "Acquisition stopped")?;
                // FIXED (doc/upstream-c-defects.md #1): upstream computes
                // Idle/Aborted and then unconditionally overwrites it with
                // ADStatusAcquire on the next line (simDetector.cpp:913-918),
                // discarding the real stopped status. Keep the computed status.
                let stopped = if image_mode == ImageMode::Continuous {
                    ADStatus::Idle
                } else {
                    ADStatus::Aborted
                };
                self.ad
                    .port_base
                    .set_int32_param(p.status, 0, stopped as i32)?;
            }
        }
        self.ad.port_base.call_param_callbacks(0)?;

        // C `setIntegerParam(function, value)`, which for ADAcquire is
        // intercepted by asynNDArrayDriver::setIntegerParam to drive
        // ADAcquireBusy.
        if reason == p.acquire {
            self.ad.set_acquire(value)?;
        } else {
            self.ad
                .port_base
                .set_int32_param(reason, user.addr, value)?;
        }

        let mut status = Ok(());
        if reason == p.acquire {
            if value != 0 && !acquiring {
                let _ = self.start_tx.try_send(Signal);
            }
            if value == 0 && acquiring {
                let _ = self.stop_tx.try_send(Signal);
            }
        } else if sim.int32_write_dirties_image(reason, &p) {
            status = self.ad.port_base.set_int32_param(sim.reset_image, 0, 1);
        } else if !sim.owns(reason) {
            status = ADDriver::ad_write_int32(self, user, value);
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "simDetector:writeInt32 error, status={} function={reason}, value={value}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("simDetector:writeInt32: function={reason}, value={value}"),
            ),
        }
        status
    }

    /// `simDetector::writeFloat64` (simDetector.cpp:965-993).
    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        // Both branches replace the store's status (simDetector.cpp:973-980).
        let _ = self
            .ad
            .port_base
            .set_float64_param(reason, user.addr, value);
        let status = if self
            .sim
            .float64_write_dirties_image(reason, self.ad.params.gain)
        {
            self.ad
                .port_base
                .set_int32_param(self.sim.reset_image, 0, 1)
        } else {
            // ADDriver::writeFloat64 is asynPortDriver's (simDetector.cpp:979).
            self.ad.port_base.write_float64_param(user, value)
        };

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "simDetector:writeFloat64 error, status={} function={reason}, value={value:.6}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("simDetector:writeFloat64: function={reason}, value={value:.6}"),
            ),
        }
        status
    }
}

impl ADDriver for SimDetector {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }

    /// `simDetector::setShutter` (simDetector.cpp:490-502). C's is void and
    /// ignores every param status, so an unset `ADShutterMode` reads as 0.
    fn set_shutter(&mut self, open: bool) {
        let mode = self
            .ad
            .port_base
            .get_int32_param(self.ad.params.shutter_mode, 0)
            .unwrap_or(0);
        match shutter_op(mode, open) {
            ShutterOp::DetectorStatus(v) => {
                let _ = self
                    .ad
                    .port_base
                    .set_int32_param(self.ad.params.shutter_status, 0, v);
            }
            ShutterOp::Nothing | ShutterOp::EpicsControl(_) => self.ad.set_shutter(open),
        }
    }
}

/// Handles kept by the IOC after the driver has been moved into its runtime.
pub struct SimDetectorRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub sim_params: SimParams,
    pool: DriverPool,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    /// Owns the `SimDetTask` thread for the lifetime of the IOC.
    pub task_handle: std::thread::JoinHandle<()>,
}

impl SimDetectorRuntime {
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

/// `simDetectorConfig` (simDetector.cpp:1160-1170).
///
/// DEVIATION: C's `maxBuffers` caps the number of NDArrays the pool may
/// allocate. `ad-core-rs` 0.22.1's `NDArrayPool` is bounded by `maxMemory`
/// only, so `max_buffers` is accepted for iocsh signature parity and ignored.
pub fn create_sim_detector(
    claim: &PortClaim,
    max_size_x: i32,
    max_size_y: i32,
    data_type: NDDataType,
    _max_buffers: i32,
    max_memory: usize,
) -> AsynResult<SimDetectorRuntime> {
    let port_name = claim.name();
    let (start_tx, start_rx) = rt::command_channel::<Signal>(1);
    let (stop_tx, stop_rx) = rt::command_channel::<Signal>(1);

    let det = SimDetector::new(
        port_name, max_size_x, max_size_y, data_type, max_memory, start_tx, stop_tx,
    )?;
    let ad_params = det.ad.params;
    let sim_params = det.sim;
    let pool = det.ad.pool.clone();
    // The `NDArrayOutput` must live outside the driver, because
    // `create_port_runtime` takes ownership of it and the IOC still needs to
    // attach plugin senders.

    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let task_handle = start_sim_task(SimTaskContext {
        start_rx,
        stop_rx,
        handle: runtime_handle.port_handle().clone(),
        publisher: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
        ad: ad_params,
        sim: sim_params,
    });

    Ok(SimDetectorRuntime {
        runtime_handle,
        ad_params,
        sim_params,
        pool,
        array_output,
        task_handle,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::driver::ShutterMode;

    struct Fixture {
        det: SimDetector,
        start_rx: rt::CommandReceiver<Signal>,
        stop_rx: rt::CommandReceiver<Signal>,
    }

    impl Fixture {
        fn new() -> Self {
            let (start_tx, start_rx) = rt::command_channel::<Signal>(1);
            let (stop_tx, stop_rx) = rt::command_channel::<Signal>(1);
            let det = SimDetector::new("SIMTEST", 64, 48, NDDataType::UInt8, 0, start_tx, stop_tx)
                .unwrap();
            Self {
                det,
                start_rx,
                stop_rx,
            }
        }

        fn write_i32(&mut self, reason: usize, value: i32) {
            let mut user = AsynUser::new(reason);
            self.det.write_int32(&mut user, value).unwrap();
        }

        fn write_f64(&mut self, reason: usize, value: f64) {
            let mut user = AsynUser::new(reason);
            self.det.write_float64(&mut user, value).unwrap();
        }

        fn get_i32(&self, reason: usize) -> i32 {
            self.det.ad.port_base.get_int32_param(reason, 0).unwrap()
        }

        fn clear_reset(&mut self) {
            self.det
                .ad
                .port_base
                .set_int32_param(self.det.sim.reset_image, 0, 0)
                .unwrap();
        }
    }

    /// simDetector inherits `asynNDArrayDriver::writeOctet`, so an
    /// `NDAttributesFile` write loads the set and a `FilePath` write runs
    /// `checkPath`.
    /// ADDriver::writeInt32 and asynNDArrayDriver::writeInt32 each close with
    /// an ASYN_TRACEIO_DRIVER line (ADDriver.cpp:111-113,
    /// asynNDArrayDriver.cpp:703-705).
    #[test]
    fn a_base_class_write_runs_both_base_class_write_int32s() {
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "SIM_BASE_TRACE";
        let det = {
            let (start_tx, _) = rt::command_channel::<Signal>(1);
            let (stop_tx, _) = rt::command_channel::<Signal>(1);
            SimDetector::new(name, 64, 48, NDDataType::UInt8, 0, start_tx, stop_tx).unwrap()
        };
        let p = det.ad.params.base;
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            det,
            RuntimeConfig::default(),
        )
        .unwrap();
        let h = runtime.port_handle().clone();
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        h.trace().set_trace_file(
            Some(name),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace()
            .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
        h.write_int32_blocking(p.pool_poll_stats, 0, 1).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        for driver in ["asynNDArrayDriver", "ADDriver"] {
            assert!(
                text.contains(&format!(
                    "{driver}:writeInt32: function={}",
                    p.pool_poll_stats
                )),
                "{driver}: {text:?}"
            );
        }
        std::mem::forget(runtime);
    }

    #[test]
    fn octet_writes_reach_the_asyn_nd_array_driver_branches() {
        let mut f = Fixture::new();
        let p = f.det.ad.params.base;
        let xml = br#"<Attributes><Attribute name="G" type="PARAM" source="GAIN" datatype="DOUBLE"/></Attributes>"#;
        f.det
            .write_octet(&mut AsynUser::new(p.attributes_file), xml)
            .unwrap();
        assert_eq!(f.get_i32(p.attributes_status), 0);

        let dir = std::env::temp_dir();
        f.det
            .write_octet(
                &mut AsynUser::new(p.file_path),
                dir.to_string_lossy().as_bytes(),
            )
            .unwrap();
        assert_eq!(f.get_i32(p.file_path_exists), 1);
    }

    #[test]
    fn constructor_defaults_match_the_c_constructor() {
        let f = Fixture::new();
        let p = f.det.ad.params;
        let s = f.det.sim;
        let get_f64 = |r| f.det.ad.port_base.get_float64_param(r, 0).unwrap();

        assert_eq!(f.get_i32(p.max_size_x), 64);
        assert_eq!(f.get_i32(p.max_size_y), 48);
        assert_eq!(f.get_i32(p.size_x), 64);
        assert_eq!(f.get_i32(p.size_y), 48);
        assert_eq!(f.get_i32(p.min_x), 0);
        assert_eq!(f.get_i32(p.min_y), 0);
        assert_eq!(f.get_i32(p.bin_x), 1);
        assert_eq!(f.get_i32(p.bin_y), 1);
        assert_eq!(f.get_i32(p.reverse_x), 0);
        assert_eq!(f.get_i32(p.reverse_y), 0);
        assert_eq!(f.get_i32(p.base.array_size_x), 64);
        assert_eq!(f.get_i32(p.base.array_size_y), 48);
        assert_eq!(f.get_i32(p.base.array_size), 0);
        assert_eq!(f.get_i32(p.base.data_type), NDDataType::UInt8 as u8 as i32);
        assert_eq!(f.get_i32(p.image_mode), ImageMode::Continuous as i32);
        assert_eq!(get_f64(p.acquire_time), 0.001);
        assert_eq!(get_f64(p.acquire_period), 0.005);
        assert_eq!(f.get_i32(p.num_images), 100);
        assert_eq!(f.get_i32(s.reset_image), 1);
        assert_eq!(get_f64(s.gain_x), 1.0);
        assert_eq!(get_f64(s.gain_y), 1.0);
        assert_eq!(get_f64(s.gain_red), 1.0);
        assert_eq!(get_f64(s.gain_green), 1.0);
        assert_eq!(get_f64(s.gain_blue), 1.0);
        assert_eq!(f.get_i32(s.mode), 0);
        assert_eq!(f.get_i32(s.peak_start_x), 1);
        assert_eq!(f.get_i32(s.peak_start_y), 1);
        assert_eq!(f.get_i32(s.peak_width_x), 10);
        assert_eq!(f.get_i32(s.peak_width_y), 20);
        assert_eq!(f.get_i32(s.peak_num_x), 1);
        assert_eq!(f.get_i32(s.peak_num_y), 1);
        assert_eq!(f.get_i32(s.peak_step_x), 1);
        assert_eq!(f.get_i32(s.peak_step_y), 1);
    }

    #[test]
    fn starting_acquisition_signals_the_task_and_raises_acquire_busy() {
        let mut f = Fixture::new();
        let acquire = f.det.ad.params.acquire;
        f.write_i32(acquire, 1);

        assert_eq!(f.get_i32(acquire), 1);
        assert_eq!(f.get_i32(f.det.ad.params.acquire_busy), 1);
        assert!(f.start_rx.try_recv().is_ok());
        assert!(f.stop_rx.try_recv().is_err());
    }

    #[test]
    fn a_second_start_while_acquiring_does_not_resignal() {
        let mut f = Fixture::new();
        let acquire = f.det.ad.params.acquire;
        f.write_i32(acquire, 1);
        assert!(f.start_rx.try_recv().is_ok());
        f.write_i32(acquire, 1);
        assert!(f.start_rx.try_recv().is_err());
    }

    #[test]
    fn stopping_a_non_continuous_acquisition_leaves_status_aborted() {
        let mut f = Fixture::new();
        let acquire = f.det.ad.params.acquire;
        // The driver defaults to Continuous; pick a non-continuous mode.
        f.write_i32(f.det.ad.params.image_mode, ImageMode::Single as i32);
        f.write_i32(acquire, 1);
        let _ = f.start_rx.try_recv();

        f.write_i32(acquire, 0);
        assert_eq!(f.get_i32(acquire), 0);
        assert_eq!(f.get_i32(f.det.ad.params.acquire_busy), 0);
        // FIXED (doc/upstream-c-defects.md #1): the computed stopped status is
        // kept, no longer overwritten by ADStatusAcquire.
        assert_eq!(
            f.get_i32(f.det.ad.params.status),
            ADStatus::Aborted as i32,
            "non-continuous stop reports Aborted"
        );
        assert!(f.stop_rx.try_recv().is_ok());
    }

    #[test]
    fn stopping_a_continuous_acquisition_leaves_status_idle() {
        let mut f = Fixture::new();
        let acquire = f.det.ad.params.acquire;
        f.write_i32(f.det.ad.params.image_mode, ImageMode::Continuous as i32);
        f.write_i32(acquire, 1);
        let _ = f.start_rx.try_recv();

        f.write_i32(acquire, 0);
        assert_eq!(f.get_i32(acquire), 0);
        // FIXED (doc/upstream-c-defects.md #1): continuous mode reports Idle.
        assert_eq!(
            f.get_i32(f.det.ad.params.status),
            ADStatus::Idle as i32,
            "continuous stop reports Idle"
        );
        assert!(f.stop_rx.try_recv().is_ok());
    }

    #[test]
    fn a_stop_while_idle_does_not_signal_the_task() {
        let mut f = Fixture::new();
        f.write_i32(f.det.ad.params.acquire, 0);
        assert!(f.stop_rx.try_recv().is_err());
        assert_eq!(f.get_i32(f.det.ad.params.status), ADStatus::Idle as i32);
    }

    #[test]
    fn image_dirtying_int32_writes_set_reset_image() {
        let mut f = Fixture::new();
        let p = f.det.ad.params;
        let s = f.det.sim;
        for (reason, value) in [
            (p.base.data_type, 3),
            (p.base.color_mode, 2),
            (s.mode, 2),
            (s.peak_start_x, 5),
            (s.peak_step_y, 7),
        ] {
            f.clear_reset();
            f.write_i32(reason, value);
            assert_eq!(f.get_i32(s.reset_image), 1, "reason {reason}");
            assert_eq!(f.get_i32(reason), value);
        }
    }

    #[test]
    fn non_dirtying_int32_writes_leave_reset_image_alone() {
        let mut f = Fixture::new();
        let p = f.det.ad.params;
        for (reason, value) in [(p.size_x, 32), (p.bin_x, 2), (p.num_images, 5)] {
            f.clear_reset();
            f.write_i32(reason, value);
            assert_eq!(f.get_i32(f.det.sim.reset_image), 0, "reason {reason}");
            assert_eq!(f.get_i32(reason), value);
        }
    }

    #[test]
    fn gain_and_every_simulation_double_set_reset_image() {
        let mut f = Fixture::new();
        let p = f.det.ad.params;
        let s = f.det.sim;
        for reason in [
            p.gain,
            s.gain_x,
            s.gain_blue,
            s.offset,
            s.noise,
            s.peak_height_variation,
            s.y_sine2_phase,
        ] {
            f.clear_reset();
            f.write_f64(reason, 3.5);
            assert_eq!(f.get_i32(s.reset_image), 1, "reason {reason}");
        }
    }

    #[test]
    fn base_class_doubles_do_not_set_reset_image() {
        let mut f = Fixture::new();
        for reason in [
            f.det.ad.params.acquire_time,
            f.det.ad.params.acquire_period,
            f.det.ad.params.temperature,
        ] {
            f.clear_reset();
            f.write_f64(reason, 0.25);
            assert_eq!(f.get_i32(f.det.sim.reset_image), 0, "reason {reason}");
        }
    }

    #[test]
    fn shutter_control_in_detector_mode_drives_shutter_status() {
        let mut f = Fixture::new();
        let p = f.det.ad.params;
        f.det
            .ad
            .port_base
            .set_int32_param(p.shutter_mode, 0, ShutterMode::DetectorOnly as i32)
            .unwrap();

        f.write_i32(p.shutter_control, 1);
        assert_eq!(f.get_i32(p.shutter_status), 1);
        f.write_i32(p.shutter_control, 0);
        assert_eq!(f.get_i32(p.shutter_status), 0);
    }

    #[test]
    fn shutter_control_in_none_mode_leaves_shutter_status_untouched() {
        let mut f = Fixture::new();
        let p = f.det.ad.params;
        f.write_i32(p.shutter_control, 1);
        assert_eq!(f.get_i32(p.shutter_status), 0);
    }

    /// An 8 x 8 detector set up for one 1 ms Single image, with the
    /// parameters the IOC's PINI records seed.
    fn single_image_detector(name: &str, array_callbacks: i32) -> SimDetectorRuntime {
        single_image_detector_in(name, array_callbacks, 0)
    }

    fn single_image_detector_in(
        name: &str,
        array_callbacks: i32,
        max_memory: usize,
    ) -> SimDetectorRuntime {
        use epics_rs::ad_core::driver::ImageMode;
        use epics_rs::asyn::param::ParamValue;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::request::ParamSetValue;

        let rt = create_sim_detector(
            &claim_port(name).unwrap(),
            8,
            8,
            NDDataType::UInt8,
            0,
            max_memory,
        )
        .unwrap();
        let (h, ad) = (rt.runtime_handle.port_handle(), rt.ad_params);
        h.set_params_and_notify_blocking(
            0,
            vec![
                ParamSetValue::new(
                    ad.base.array_callbacks,
                    0,
                    ParamValue::Int32(array_callbacks),
                ),
                ParamSetValue::new(ad.base.wait_for_plugins, 0, ParamValue::Int32(1)),
                ParamSetValue::new(
                    ad.image_mode,
                    0,
                    ParamValue::Int32(ImageMode::Single as i32),
                ),
                ParamSetValue::new(ad.acquire_time, 0, ParamValue::Float64(0.001)),
            ],
        )
        .unwrap();
        // The IOC's PINI records seed these; read_config refuses them unset.
        let sp = rt.sim_params;
        let floats = [
            ad.gain,
            sp.gain_x,
            sp.gain_y,
            sp.gain_red,
            sp.gain_green,
            sp.gain_blue,
            sp.offset,
            sp.noise,
            sp.peak_height_variation,
            sp.x_sine1_amplitude,
            sp.x_sine1_frequency,
            sp.x_sine1_phase,
            sp.x_sine2_amplitude,
            sp.x_sine2_frequency,
            sp.x_sine2_phase,
            sp.y_sine1_amplitude,
            sp.y_sine1_frequency,
            sp.y_sine1_phase,
            sp.y_sine2_amplitude,
            sp.y_sine2_frequency,
            sp.y_sine2_phase,
        ];
        let ints = [
            sp.mode,
            sp.peak_num_x,
            sp.peak_num_y,
            sp.peak_start_x,
            sp.peak_start_y,
            sp.peak_step_x,
            sp.peak_step_y,
            sp.peak_width_x,
            sp.peak_width_y,
            sp.x_sine_operation,
            sp.y_sine_operation,
        ];
        let mut seeds = Vec::new();
        for r in floats {
            if h.read_float64_blocking(r, 0).is_err() {
                seeds.push(ParamSetValue::new(r, 0, ParamValue::Float64(0.0)));
            }
        }
        for r in ints {
            if h.read_int32_blocking(r, 0).is_err() {
                seeds.push(ParamSetValue::new(r, 0, ParamValue::Int32(0)));
            }
        }
        h.set_params_and_notify_blocking(0, seeds).unwrap();
        rt
    }

    #[test]
    fn wait_for_plugins_holds_acquire_busy_until_the_frame_is_released() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use std::time::{Duration, Instant};

        let rt = single_image_detector("SIM_BUSY", 1);
        let (tx, mut rx) = ndarray_channel("SIM_BUSY_SINK", 8);
        rt.connect_downstream(tx);
        let (h, ad) = (rt.runtime_handle.port_handle(), rt.ad_params);
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
        assert_eq!(read(ad.acquire_busy), 1, "busy cleared with a frame queued");
        drop(rx.blocking_recv().expect("frame"));
        wait_for(ad.acquire_busy, 0);
        std::mem::forget(rt);
    }

    /// C keeps the last image in pArrays[0] until just before the next
    /// convert (simDetector.cpp:696-700), published or not.
    #[test]
    fn the_last_image_stays_charged_after_the_acquisition() {
        use std::time::{Duration, Instant};

        let rt = single_image_detector("SIM_LAST", 0);
        let (h, ad) = (rt.runtime_handle.port_handle(), rt.ad_params);
        h.write_int32_blocking(ad.acquire, 0, 1).unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        while h.read_int32_blocking(ad.base.array_counter, 0).unwrap() != 1
            || h.read_int32_blocking(ad.acquire_busy, 0).unwrap() != 0
        {
            assert!(Instant::now() < deadline, "no image");
            std::thread::sleep(Duration::from_millis(10));
        }
        let pool = rt.pool();
        // pRaw_, pBackground_, pRamp_, pPeak_ and pArrays[0].
        assert_eq!(pool.num_alloc_buffers() - pool.num_free_buffers(), 5);

        // pArrays[0] is preAllocateBuffers' template (asynNDArrayDriver.cpp:717-726).
        let p = &ad.base;
        h.write_int32_blocking(p.pool_num_pre_alloc_buffers, 0, 2)
            .unwrap();
        let free = pool.num_free_buffers();
        h.write_int32_blocking(p.pool_pre_alloc, 0, 1).unwrap();
        assert_eq!(pool.num_free_buffers(), free + 2);
        std::mem::forget(rt);
    }

    /// C releases pArrays[0] after the SimBuffers reset (simDetector.cpp:625-646,
    /// :696), so a reset the pool refuses leaves the last image charged.
    #[test]
    fn a_refused_reset_keeps_the_last_image_in_parrays0() {
        use std::time::{Duration, Instant};

        let rt = single_image_detector_in("SIM_RESET_HOLD", 0, 5 * 64);
        let (h, ad) = (rt.runtime_handle.port_handle(), rt.ad_params);
        let wait_idle = |count: i32| {
            let deadline = Instant::now() + Duration::from_secs(10);
            while h.read_int32_blocking(ad.base.array_counter, 0).unwrap() != count
                && h.read_int32_blocking(ad.status, 0).unwrap() != ADStatus::Error as i32
                || h.read_int32_blocking(ad.acquire_busy, 0).unwrap() != 0
            {
                assert!(Instant::now() < deadline, "acquisition did not end");
                std::thread::sleep(Duration::from_millis(10));
            }
        };
        h.write_int32_blocking(ad.acquire, 0, 1).unwrap();
        wait_idle(1);
        let pool = rt.pool();
        assert!(pool.owner().array(0).is_some());

        h.write_int32_blocking(ad.base.data_type, 0, NDDataType::UInt16 as i32)
            .unwrap();
        h.write_int32_blocking(ad.acquire, 0, 1).unwrap();
        wait_idle(1);
        assert_eq!(
            h.read_int32_blocking(ad.status, 0).unwrap(),
            ADStatus::Error as i32
        );
        assert!(pool.owner().array(0).is_some());
        std::mem::forget(rt);
    }

    /// ADDriver::writeFloat64 is asynPortDriver's (simDetector.cpp:979), whose
    /// IO_DRIVER line precedes the driver's.
    #[test]
    fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let port = "SIM_F64_BASE";
        let mut det = {
            let (start_tx, _) = rt::command_channel::<Signal>(1);
            let (stop_tx, _) = rt::command_channel::<Signal>(1);
            SimDetector::new(port, 64, 48, NDDataType::UInt8, 0, start_tx, stop_tx).unwrap()
        };
        let manager = epics_rs::asyn::services::PortServices::global()
            .trace()
            .clone();
        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        let delay = det.ad.params.shutter_open_delay;
        let mut user = AsynUser::new(delay).with_addr(0);
        user.trace = Some(UserTrace {
            manager,
            port: port.into(),
        });
        let _ = det.write_float64(&mut user, 2.0);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let base = format!(
            "asynPortDriver:writeFloat64: function={delay}, name=SHUTTER_OPEN_DELAY, value=2.000000"
        );
        let own = format!("simDetector:writeFloat64: function={delay}, value=2.000000");
        let at = |line: &str| {
            text.find(line)
                .unwrap_or_else(|| panic!("{line}: {text:?}"))
        };
        assert!(at(&base) < at(&own));
    }
}
