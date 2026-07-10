//! Port of `simDetector` (the `asynPortDriver` half of `simDetector.cpp`).

use std::sync::Arc;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
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
        let mut ad = ADDriverBase::new(port_name, max_size_x, max_size_y, max_memory)?;
        let sim = SimParams::create(&mut ad.port_base)?;

        let p = ad.params;
        let base = &mut ad.port_base;

        // simDetector.cpp:1093-1100
        base.set_string_param(p.base.manufacturer, 0, "Simulated detector".into())?;
        base.set_string_param(p.base.model, 0, "Basic simulator".into())?;
        base.set_string_param(p.base.driver_version, 0, DRIVER_VERSION.into())?;
        base.set_string_param(p.base.sdk_version, 0, DRIVER_VERSION.into())?;
        base.set_string_param(p.base.serial_number, 0, "No serial number".into())?;
        base.set_string_param(p.base.firmware_version, 0, "No firmware".into())?;

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

    /// `simDetector::setShutter` (simDetector.cpp:490-502).
    fn set_shutter(&mut self, open: bool) -> AsynResult<()> {
        let mode = self
            .ad
            .port_base
            .get_int32_param(self.ad.params.shutter_mode, 0)?;
        match shutter_op(mode, open) {
            ShutterOp::DetectorStatus(v) => {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.shutter_status, 0, v)?;
            }
            ShutterOp::Nothing | ShutterOp::EpicsControl(_) => self.ad.set_shutter(open)?,
        }
        Ok(())
    }
}

impl PortDriver for SimDetector {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    /// `simDetector::writeInt32` (simDetector.cpp:891-957).
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let p = self.ad.params;
        let sim = self.sim;

        let acquiring = self.ad.port_base.get_int32_param(p.acquire, 0)? != 0;
        let image_mode = ImageMode::from_i32(self.ad.port_base.get_int32_param(p.image_mode, 0)?);

        // "Ensure that ADStatus is set correctly before we set ADAcquire."
        if reason == p.acquire {
            if value != 0 && !acquiring {
                self.ad
                    .port_base
                    .set_string_param(p.status_message, 0, "Acquiring data".into())?;
            }
            if value == 0 && acquiring {
                self.ad.port_base.set_string_param(
                    p.status_message,
                    0,
                    "Acquisition stopped".into(),
                )?;
                // Upstream computes Idle/Aborted and then unconditionally
                // overwrites it with ADStatusAcquire on the next line
                // (simDetector.cpp:913-918). Ported as written.
                let interim = if image_mode == ImageMode::Continuous {
                    ADStatus::Idle
                } else {
                    ADStatus::Aborted
                };
                self.ad
                    .port_base
                    .set_int32_param(p.status, 0, interim as i32)?;
                self.ad
                    .port_base
                    .set_int32_param(p.status, 0, ADStatus::Acquire as i32)?;
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
                .params
                .set_int32(reason, user.addr, value)?;
        }

        if reason == p.acquire {
            if value != 0 && !acquiring {
                let _ = self.start_tx.try_send(Signal);
            }
            if value == 0 && acquiring {
                let _ = self.stop_tx.try_send(Signal);
            }
        } else if sim.int32_write_dirties_image(reason, &p) {
            self.ad.port_base.set_int32_param(sim.reset_image, 0, 1)?;
        } else if !sim.owns(reason) {
            // `ADDriver::writeInt32` -> `asynNDArrayDriver::writeInt32`.
            if reason == p.shutter_control {
                self.set_shutter(value != 0)?;
            } else {
                self.ad.write_int32_pool(reason, value)?;
            }
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    /// `simDetector::writeFloat64` (simDetector.cpp:965-993).
    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        self.ad
            .port_base
            .params
            .set_float64(reason, user.addr, value)?;

        if self
            .sim
            .float64_write_dirties_image(reason, self.ad.params.gain)
        {
            self.ad
                .port_base
                .set_int32_param(self.sim.reset_image, 0, 1)?;
        }
        // Otherwise C calls `ADDriver::writeFloat64`, which neither ADDriver nor
        // asynNDArrayDriver override: asynPortDriver::writeFloat64 only sets the
        // parameter (done above) and fires callbacks (done below).

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }
}

impl ADDriver for SimDetector {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

/// Handles kept by the IOC after the driver has been moved into its runtime.
pub struct SimDetectorRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub sim_params: SimParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    /// Owns the `SimDetTask` thread for the lifetime of the IOC.
    pub task_handle: std::thread::JoinHandle<()>,
}

impl SimDetectorRuntime {
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

/// `simDetectorConfig` (simDetector.cpp:1160-1170).
///
/// DEVIATION: C's `maxBuffers` caps the number of NDArrays the pool may
/// allocate. `ad-core-rs` 0.22.1's `NDArrayPool` is bounded by `maxMemory`
/// only, so `max_buffers` is accepted for iocsh signature parity and ignored.
pub fn create_sim_detector(
    port_name: &str,
    max_size_x: i32,
    max_size_y: i32,
    data_type: NDDataType,
    _max_buffers: i32,
    max_memory: usize,
) -> AsynResult<SimDetectorRuntime> {
    let (start_tx, start_rx) = rt::command_channel::<Signal>(1);
    let (stop_tx, stop_rx) = rt::command_channel::<Signal>(1);

    let det = SimDetector::new(
        port_name, max_size_x, max_size_y, data_type, max_memory, start_tx, stop_tx,
    )?;
    let ad_params = det.ad.params;
    let sim_params = det.sim;
    let pool = det.ad.pool.clone();
    // Share the driver's own counter so `ADDriverBase::set_acquire` sees the
    // live queued-array count when ADWaitForPlugins is set. The `NDArrayOutput`
    // must live outside the driver instead, because `create_port_runtime` takes
    // ownership of it and the IOC still needs to attach plugin senders.
    let queued_counter = det.ad.queued_counter.clone();

    let (runtime_handle, _) = create_port_runtime(det, RuntimeConfig::default());
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let task_handle = start_sim_task(SimTaskContext {
        start_rx,
        stop_rx,
        handle: runtime_handle.port_handle().clone(),
        publisher: ArrayPublisher::new(array_output.clone()),
        queued: queued_counter.clone(),
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
        queued_counter,
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
    fn stopping_acquisition_signals_the_task_and_leaves_status_acquire() {
        let mut f = Fixture::new();
        let acquire = f.det.ad.params.acquire;
        f.write_i32(acquire, 1);
        let _ = f.start_rx.try_recv();

        f.write_i32(acquire, 0);
        assert_eq!(f.get_i32(acquire), 0);
        assert_eq!(f.get_i32(f.det.ad.params.acquire_busy), 0);
        // Upstream overwrites the Idle/Aborted it just wrote with Acquire.
        assert_eq!(
            f.get_i32(f.det.ad.params.status),
            ADStatus::Acquire as i32,
            "simDetector.cpp:918 unconditionally sets ADStatusAcquire"
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
}
