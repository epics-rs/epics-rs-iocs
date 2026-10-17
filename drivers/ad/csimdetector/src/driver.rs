//! Port of `ADCSimDetector` (the `asynNDArrayDriver` half of `ADCSimDetector.cpp`).
//!
//! `ADCSimDetector` derives from `asynNDArrayDriver` directly, with
//! `maxAddr = MAX_SIGNALS + 1` and `ASYN_MULTIDEVICE`.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::ndarray_driver::{
    ArrayOwner, NDArrayDriverBase, handle_write_int32,
};
use epics_rs::ad_core::driver::write_tail::{OwnList, WriteTail};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::runtime as rt;

use crate::params::CSimParams;
use crate::task::{CSimTaskContext, start_sim_task};
use crate::types::{MAX_SIGNALS, Signal};

const INT32_TAIL: WriteTail = WriteTail {
    error: "ADCSimDetector:writeInt32 error, ",
    done: "ADCSimDetector:writeInt32: ",
    list: OwnList::Addressed,
};

/// `DRIVER_VERSION.DRIVER_REVISION.DRIVER_MODIFICATION` (ADCSimDetector.h:22-24).
pub const DRIVER_VERSION: &str = "2.5.0";

/// `maxAddr` — addr 0 carries the 2-D array, `1..=MAX_SIGNALS` the 1-D signals.
pub const MAX_ADDR: usize = MAX_SIGNALS + 1;

pub struct CSimDetector {
    pub port_base: PortDriverBase,
    pub nd: NDArrayDriverParams,
    pub sim: CSimParams,
    pub pool: DriverPool,
    pub owner: Arc<ArrayOwner>,
    /// C `acquiring_`, written by the simulation task and read by `setAcquire`.
    acquiring: Arc<AtomicBool>,
    /// C `startEventId_` / `stopEventId_`. A full capacity-1 channel is the
    /// binary-semaphore "already signalled" state, so a failed `try_send` is
    /// exactly C's second `epicsEventSignal` being a no-op.
    start_tx: rt::CommandSender<Signal>,
    stop_tx: rt::CommandSender<Signal>,
}

/// The state `ADCSimDetector` shares with its `simTask` thread: C's
/// `acquiring_`, `startEventId_` and `stopEventId_`.
pub(crate) struct SharedTaskState {
    pub acquiring: Arc<AtomicBool>,
    pub start_tx: rt::CommandSender<Signal>,
    pub stop_tx: rt::CommandSender<Signal>,
}

impl CSimDetector {
    fn new(
        port_name: &str,
        num_time_points: i32,
        data_type: NDDataType,
        max_memory: usize,
        shared: SharedTaskState,
    ) -> AsynResult<Self> {
        // asynNDArrayDriver(portName, MAX_SIGNALS+1, ..., ASYN_CANBLOCK |
        // ASYN_MULTIDEVICE, ...), ADCSimDetector.cpp:56-59.
        let NDArrayDriverBase {
            mut port_base,
            params: nd,
            pool,
            owner,
            ..
        } = NDArrayDriverBase::new(
            port_name,
            MAX_ADDR,
            max_memory,
            PortFlags {
                can_block: true,
                multi_device: true,
                ..Default::default()
            },
            InterfaceMask::UNMIGRATED_SUBCLASS,
            InterfaceMask::UNMIGRATED_SUBCLASS,
        )?;
        let sim = CSimParams::create(&mut port_base)?;

        // ADCSimDetector.cpp:93-104.
        port_base.set_string_param(nd.driver_version, 0, DRIVER_VERSION)?;
        port_base.set_int32_param(sim.num_time_points, 0, num_time_points)?;
        port_base.set_int32_param(nd.data_type, 0, data_type as u8 as i32)?;
        port_base.set_float64_param(sim.time_step, 0, 0.001)?;
        // The two-argument `setDoubleParam` writes list 0 only, so signals 1..7
        // start with amplitude/period 0 until their `PINI YES` records post.
        port_base.set_float64_param(sim.amplitude, 0, 1.0)?;
        port_base.set_float64_param(sim.offset, 0, 0.0)?;
        port_base.set_float64_param(sim.period, 0, 1.0)?;
        port_base.set_float64_param(sim.phase, 0, 0.0)?;
        port_base.set_float64_param(sim.noise, 0, 0.0)?;

        Ok(Self {
            port_base,
            nd,
            sim,
            pool,
            owner,
            acquiring: shared.acquiring,
            start_tx: shared.start_tx,
            stop_tx: shared.stop_tx,
        })
    }

    /// `ADCSimDetector::setAcquire` (ADCSimDetector.cpp:255-266).
    fn set_acquire(&mut self, value: i32) {
        let acquiring = self.acquiring.load(Ordering::Acquire);
        if value != 0 && !acquiring {
            let _ = self.start_tx.try_send(Signal);
        }
        if value == 0 && acquiring {
            let _ = self.stop_tx.try_send(Signal);
        }
    }
}

impl PortDriver for CSimDetector {
    fn base(&self) -> &PortDriverBase {
        &self.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.port_base
    }

    /// `ADCSimDetector::writeInt32` (ADCSimDetector.cpp:354-385).
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        INT32_TAIL.write_int32(self, user, value, |det, user, status| {
            if reason == det.nd.acquire {
                det.set_acquire(value);
                status
            } else if det.sim.belongs_to_base(reason) {
                let template = det.owner.array(0);
                handle_write_int32(
                    &mut det.port_base,
                    &det.nd,
                    &det.pool,
                    template.as_deref(),
                    user,
                    value,
                )
            } else {
                status
            }
        })
    }

    // `writeFloat64` is not overridden by `ADCSimDetector`, and neither
    // `asynNDArrayDriver` nor `asynPortDriver` do anything beyond setting the
    // parameter at `addr` and firing that list's callbacks — which is exactly
    // the `PortDriver` default implementation.
}

/// Handles kept by the IOC after the driver has been moved into its runtime.
pub struct CSimDetectorRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub nd_params: NDArrayDriverParams,
    pub sim_params: CSimParams,
    pool: DriverPool,
    /// Index 0 is the 2-D array output; `1..=MAX_SIGNALS` the per-signal ones.
    outputs: Vec<Arc<parking_lot::Mutex<NDArrayOutput>>>,
    /// Owns the `SimDetTask` thread for the lifetime of the IOC.
    pub task_handle: std::thread::JoinHandle<()>,
}

impl CSimDetectorRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &DriverPool {
        &self.pool
    }

    /// The `NDArrayOutput` for asyn address `addr` (`0..=MAX_SIGNALS`).
    pub fn array_output(&self, addr: usize) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.outputs[addr]
    }

    pub fn connect_downstream(&self, addr: usize, sender: NDArraySender) {
        self.outputs[addr].lock().add(sender);
    }
}

/// `ADCSimDetectorConfig` (ADCSimDetector.cpp:412-420).
///
/// DEVIATION: C's `maxBuffers` caps the number of NDArrays the pool may
/// allocate. `ad-core-rs` 0.22.1's `NDArrayPool` is bounded by `maxMemory`
/// only, so `max_buffers` is accepted for iocsh signature parity and ignored.
pub fn create_c_sim_detector(
    claim: &PortClaim,
    num_time_points: i32,
    data_type: NDDataType,
    _max_buffers: i32,
    max_memory: usize,
) -> AsynResult<CSimDetectorRuntime> {
    let port_name = claim.name();
    let (start_tx, start_rx) = rt::command_channel::<Signal>(1);
    let (stop_tx, stop_rx) = rt::command_channel::<Signal>(1);

    let acquiring = Arc::new(AtomicBool::new(false));

    let det = CSimDetector::new(
        port_name,
        num_time_points,
        data_type,
        max_memory,
        SharedTaskState {
            acquiring: acquiring.clone(),
            start_tx,
            stop_tx: stop_tx.clone(),
        },
    )?;
    let nd_params = det.nd;
    let sim_params = det.sim;
    let pool = det.pool.clone();

    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;

    let outputs: Vec<Arc<parking_lot::Mutex<NDArrayOutput>>> = (0..MAX_ADDR)
        .map(|_| Arc::new(parking_lot::Mutex::new(NDArrayOutput::new())))
        .collect();
    let publishers: Vec<ArrayPublisher> =
        outputs.iter().cloned().map(ArrayPublisher::new).collect();

    let task_handle = start_sim_task(CSimTaskContext {
        start_rx,
        stop_rx,
        stop_tx,
        acquiring,
        handle: runtime_handle.port_handle().clone(),
        publishers,
        pool: pool.clone(),
        nd: nd_params,
        sim: sim_params,
    });

    Ok(CSimDetectorRuntime {
        runtime_handle,
        nd_params,
        sim_params,
        pool,
        outputs,
        task_handle,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::NDDimension;
    use epics_rs::asyn::error::AsynError;
    use epics_rs::asyn::trace::TraceMask;

    struct Fixture {
        det: CSimDetector,
        acquiring: Arc<AtomicBool>,
        start_rx: rt::CommandReceiver<Signal>,
        stop_rx: rt::CommandReceiver<Signal>,
    }

    impl Fixture {
        fn new() -> Self {
            let (start_tx, start_rx) = rt::command_channel::<Signal>(1);
            let (stop_tx, stop_rx) = rt::command_channel::<Signal>(1);
            let acquiring = Arc::new(AtomicBool::new(false));
            let det = CSimDetector::new(
                "CSIMTEST",
                2000,
                NDDataType::Float64,
                0,
                SharedTaskState {
                    acquiring: acquiring.clone(),
                    start_tx,
                    stop_tx,
                },
            )
            .unwrap();
            Self {
                det,
                acquiring,
                start_rx,
                stop_rx,
            }
        }

        fn write_i32(&mut self, reason: usize, addr: i32, value: i32) {
            let mut user = AsynUser::new(reason).with_addr(addr);
            self.det.write_int32(&mut user, value).unwrap();
        }

        fn write_f64(&mut self, reason: usize, addr: i32, value: f64) {
            let mut user = AsynUser::new(reason).with_addr(addr);
            self.det.write_float64(&mut user, value).unwrap();
        }

        fn get_i32(&self, reason: usize, addr: i32) -> i32 {
            self.det.port_base.get_int32_param(reason, addr).unwrap()
        }

        fn get_f64(&self, reason: usize, addr: i32) -> f64 {
            self.det.port_base.get_float64_param(reason, addr).unwrap()
        }
    }

    #[test]
    fn the_port_serves_one_address_per_signal_plus_the_two_d_array() {
        let f = Fixture::new();
        assert_eq!(MAX_ADDR, 9);
        assert_eq!(f.det.port_base.max_addr, MAX_ADDR);
    }

    #[test]
    fn constructor_defaults_match_the_c_constructor() {
        let f = Fixture::new();
        let s = f.det.sim;
        assert_eq!(f.get_i32(s.num_time_points, 0), 2000);
        assert_eq!(
            f.get_i32(f.det.nd.data_type, 0),
            NDDataType::Float64 as u8 as i32
        );
        assert_eq!(f.get_f64(s.time_step, 0), 0.001);
        assert_eq!(f.get_f64(s.amplitude, 0), 1.0);
        assert_eq!(f.get_f64(s.offset, 0), 0.0);
        assert_eq!(f.get_f64(s.period, 0), 1.0);
        assert_eq!(f.get_f64(s.phase, 0), 0.0);
        assert_eq!(f.get_f64(s.noise, 0), 0.0);
        // ADCSimDetector.cpp:97-104 and asynNDArrayDriver leave
        // NDArrayCallbacks to the ArrayCallbacks record's PINI.
        let callbacks = f.det.nd.array_callbacks;
        assert!(matches!(
            f.det.port_base.get_int32_param(callbacks, 0),
            Err(AsynError::ParamUndefined(r)) if r == callbacks
        ));
        assert_eq!(
            f.det
                .port_base
                .get_string_param(f.det.nd.driver_version, 0)
                .unwrap(),
            b"2.5.0"
        );
    }

    #[test]
    fn amplitude_and_period_defaults_apply_to_address_zero_only() {
        // C's two-argument `setDoubleParam` writes parameter list 0
        // (ADCSimDetector.cpp:100-102). Signals 1..7 stay undefined until
        // their records post a value.
        let f = Fixture::new();
        let base = &f.det.port_base;
        for addr in 1..MAX_SIGNALS as i32 {
            for reason in [f.det.sim.amplitude, f.det.sim.period] {
                assert!(
                    matches!(
                        base.get_float64_param(reason, addr),
                        Err(AsynError::ParamUndefined(r)) if r == reason
                    ),
                    "addr {addr}"
                );
            }
        }
    }

    #[test]
    fn starting_acquisition_signals_the_task_and_raises_acquire_busy() {
        let mut f = Fixture::new();
        f.write_i32(f.det.nd.acquire, 0, 1);
        assert_eq!(f.get_i32(f.det.nd.acquire, 0), 1);
        assert_eq!(f.get_i32(f.det.nd.acquire_busy, 0), 1);
        assert!(f.start_rx.try_recv().is_ok());
        assert!(f.stop_rx.try_recv().is_err());
    }

    /// ADCSimDetector.cpp:364-367: setIntegerParam's asynParamInvalidList is
    /// the write's status, and setAcquire still runs.
    #[test]
    fn a_refused_acquire_set_still_starts_and_fails_the_write() {
        let mut f = Fixture::new();
        let mut user = AsynUser::new(f.det.nd.acquire).with_addr(MAX_ADDR as i32);
        assert!(matches!(
            f.det.write_int32(&mut user, 1),
            Err(AsynError::ParamInvalidList(_))
        ));
        assert!(f.start_rx.try_recv().is_ok());
    }

    /// ADCSimDetector.cpp:375-378 prints setIntegerParam's
    /// asynParamInvalidList.
    #[test]
    fn a_refused_write_prints_the_param_librarys_status() {
        use epics_rs::asyn::services::PortServices;
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let mut f = Fixture::new();
        let port = "CSIMTEST";
        let manager = PortServices::global().trace().clone();
        let log = std::env::temp_dir().join(format!("csim-status-{}", std::process::id()));
        let file = std::fs::File::create(&log).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        let reason = f.det.nd.acquire;
        let mut user = AsynUser::new(reason).with_addr(MAX_ADDR as i32);
        user.trace = Some(UserTrace {
            manager,
            port: port.into(),
        });
        assert!(f.det.write_int32(&mut user, 1).is_err());
        let text = std::fs::read_to_string(&log).unwrap();
        let _ = std::fs::remove_file(&log);
        assert!(
            text.contains(&format!(
                "ADCSimDetector:writeInt32 error, status=11 function={reason}, value=1"
            )),
            "{text}"
        );
    }

    #[test]
    fn a_second_start_while_the_task_is_acquiring_does_not_resignal() {
        let mut f = Fixture::new();
        f.write_i32(f.det.nd.acquire, 0, 1);
        assert!(f.start_rx.try_recv().is_ok());
        // The task has picked the start event up.
        f.acquiring.store(true, Ordering::Release);
        f.write_i32(f.det.nd.acquire, 0, 1);
        assert!(f.start_rx.try_recv().is_err());
    }

    #[test]
    fn stopping_while_acquiring_signals_the_stop_event_and_clears_acquire_busy() {
        let mut f = Fixture::new();
        f.write_i32(f.det.nd.acquire, 0, 1);
        let _ = f.start_rx.try_recv();
        f.acquiring.store(true, Ordering::Release);

        f.write_i32(f.det.nd.acquire, 0, 0);
        assert_eq!(f.get_i32(f.det.nd.acquire, 0), 0);
        assert_eq!(f.get_i32(f.det.nd.acquire_busy, 0), 0);
        assert!(f.stop_rx.try_recv().is_ok());
    }

    #[test]
    fn a_stop_while_idle_does_not_signal_the_task() {
        let mut f = Fixture::new();
        f.write_i32(f.det.nd.acquire, 0, 0);
        assert!(f.stop_rx.try_recv().is_err());
    }

    #[test]
    fn acquire_busy_stays_raised_while_plugins_are_still_queued() {
        let mut f = Fixture::new();
        f.det
            .port_base
            .set_int32_param(f.det.nd.wait_for_plugins, 0, 1)
            .unwrap();
        f.write_i32(f.det.nd.acquire, 0, 1);
        f.acquiring.store(true, Ordering::Release);
        f.det.owner.queued().increment();

        f.write_i32(f.det.nd.acquire, 0, 0);
        assert_eq!(f.get_i32(f.det.nd.acquire, 0), 0);
        assert_eq!(f.get_i32(f.det.nd.acquire_busy, 0), 1);
    }

    #[test]
    fn the_queue_draining_to_zero_clears_acquire_busy_once_acquire_is_off() {
        let mut f = Fixture::new();
        f.det
            .port_base
            .set_int32_param(f.det.nd.acquire_busy, 0, 1)
            .unwrap();
        f.write_i32(f.det.nd.num_queued_arrays, 0, 0);
        assert_eq!(f.get_i32(f.det.nd.acquire_busy, 0), 0);
    }

    #[test]
    fn the_queue_draining_to_zero_while_acquiring_leaves_acquire_busy_raised() {
        let mut f = Fixture::new();
        f.write_i32(f.det.nd.acquire, 0, 1);
        f.write_i32(f.det.nd.num_queued_arrays, 0, 0);
        assert_eq!(f.get_i32(f.det.nd.acquire_busy, 0), 1);
    }

    #[test]
    fn integer_writes_land_on_the_addressed_parameter_list() {
        let mut f = Fixture::new();
        f.write_i32(f.det.sim.num_time_points, 3, 512);
        assert_eq!(f.get_i32(f.det.sim.num_time_points, 3), 512);
        assert_eq!(f.get_i32(f.det.sim.num_time_points, 0), 2000);
    }

    #[test]
    fn double_writes_land_on_the_addressed_parameter_list() {
        let mut f = Fixture::new();
        for addr in 0..MAX_SIGNALS as i32 {
            f.write_f64(f.det.sim.amplitude, addr, addr as f64 + 0.5);
        }
        for addr in 0..MAX_SIGNALS as i32 {
            assert_eq!(f.get_f64(f.det.sim.amplitude, addr), addr as f64 + 0.5);
        }
    }

    #[test]
    fn polling_the_pool_statistics_refreshes_them() {
        let mut f = Fixture::new();
        f.write_i32(f.det.nd.pool_poll_stats, 0, 1);
        assert_eq!(f.get_i32(f.det.nd.pool_alloc_buffers, 0), 0);
        assert_eq!(f.get_i32(f.det.nd.pool_free_buffers, 0), 0);
        assert_eq!(f.get_f64(f.det.nd.pool_used_memory, 0), 0.0);
    }

    #[test]
    fn pre_allocating_buffers_without_a_template_array_resets_the_request() {
        let mut f = Fixture::new();
        f.det
            .port_base
            .set_int32_param(f.det.nd.pool_num_pre_alloc_buffers, 0, 4)
            .unwrap();
        f.write_i32(f.det.nd.pool_pre_alloc, 0, 1);
        assert_eq!(f.get_i32(f.det.nd.pool_pre_alloc, 0), 0);
        assert_eq!(f.get_i32(f.det.nd.pool_alloc_buffers, 0), 0);
    }

    /// pArrays[0] is preAllocateBuffers' template (asynNDArrayDriver.cpp:717-726).
    #[test]
    fn the_last_array_is_the_pre_alloc_template() {
        let mut f = Fixture::new();
        let image = f
            .det
            .pool
            .alloc(
                vec![NDDimension::new(MAX_SIGNALS), NDDimension::new(4)],
                NDDataType::Float64,
            )
            .unwrap();
        f.det
            .owner
            .store_array(0, Some(image.share().array().clone()));
        f.det
            .port_base
            .set_int32_param(f.det.nd.pool_num_pre_alloc_buffers, 0, 2)
            .unwrap();
        f.write_i32(f.det.nd.pool_pre_alloc, 0, 1);
        assert_eq!(f.get_i32(f.det.nd.pool_free_buffers, 0), 2);
    }

    /// ADCSimDetector.cpp:370 runs asynNDArrayDriver::writeInt32, whose
    /// closing ASYN_TRACEIO_DRIVER line is asynNDArrayDriver.cpp:703-705.
    #[test]
    fn a_base_class_write_runs_asyn_nd_array_drivers_write_int32() {
        use epics_rs::asyn::trace::TraceFile;
        let name = "CSIMTEST";
        let f = Fixture::new();
        let poll = f.det.nd.pool_poll_stats;
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            f.det,
            RuntimeConfig::default(),
        )
        .unwrap();
        let h = runtime.port_handle().clone();
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        h.trace().set_trace_file(
            Some(name),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace()
            .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
        h.write_int32_blocking(poll, 0, 1).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            text.contains(&format!("asynNDArrayDriver:writeInt32: function={poll}")),
            "{text:?}"
        );
        std::mem::forget(runtime);
    }

    #[test]
    fn emptying_the_free_list_refreshes_the_pool_statistics() {
        let mut f = Fixture::new();
        f.write_i32(f.det.nd.pool_empty_free_list, 0, 1);
        assert_eq!(f.get_i32(f.det.nd.pool_free_buffers, 0), 0);
    }
}
