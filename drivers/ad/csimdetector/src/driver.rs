//! Port of `ADCSimDetector` (the `asynNDArrayDriver` half of `ADCSimDetector.cpp`).
//!
//! `ADCSimDetector` derives from `asynNDArrayDriver` directly, with
//! `maxAddr = MAX_SIGNALS + 1` and `ASYN_MULTIDEVICE`. `ad-core-rs` 0.22.1's
//! `NDArrayDriverBase::new` hardcodes `max_addr = 1` and a single-device
//! `PortFlags`, so this driver assembles the same three pieces
//! (`PortDriverBase` + `NDArrayDriverParams` + `NDArrayPool`) by hand.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::ndarray::{NDArray, NDDataType};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
use epics_rs::ad_core::runtime as rt;

use crate::params::CSimParams;
use crate::task::{CSimTaskContext, start_sim_task};
use crate::types::{MAX_SIGNALS, Signal};

/// `DRIVER_VERSION.DRIVER_REVISION.DRIVER_MODIFICATION` (ADCSimDetector.h:22-24).
pub const DRIVER_VERSION: &str = "2.5.0";

/// `maxAddr` — addr 0 carries the 2-D array, `1..=MAX_SIGNALS` the 1-D signals.
pub const MAX_ADDR: usize = MAX_SIGNALS + 1;

const MEGABYTE: f64 = 1_048_576.0;

/// The most recent 2-D array (C `pArrays[0]`). Shared with the simulation task,
/// which replaces it each frame; `NDPoolPreAllocBuffers` uses it as its template.
pub(crate) type LastArray = Arc<parking_lot::Mutex<Option<Arc<NDArray>>>>;

pub struct CSimDetector {
    pub port_base: PortDriverBase,
    pub nd: NDArrayDriverParams,
    pub sim: CSimParams,
    pub pool: Arc<NDArrayPool>,
    pub queued_counter: Arc<QueuedArrayCounter>,
    last_array: LastArray,
    /// C `acquiring_`, written by the simulation task and read by `setAcquire`.
    acquiring: Arc<AtomicBool>,
    /// C `startEventId_` / `stopEventId_`. A full capacity-1 channel is the
    /// binary-semaphore "already signalled" state, so a failed `try_send` is
    /// exactly C's second `epicsEventSignal` being a no-op.
    start_tx: rt::CommandSender<Signal>,
    stop_tx: rt::CommandSender<Signal>,
}

/// The state `ADCSimDetector` shares with its `simTask` thread: C's
/// `pArrays[0]`, `acquiring_`, `startEventId_` and `stopEventId_`.
pub(crate) struct SharedTaskState {
    pub last_array: LastArray,
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
        let mut port_base = PortDriverBase::new(
            port_name,
            MAX_ADDR,
            PortFlags {
                can_block: true,
                multi_device: true,
                ..Default::default()
            },
        );

        let nd = NDArrayDriverParams::create(&mut port_base)?;
        let sim = CSimParams::create(&mut port_base)?;

        // `asynNDArrayDriver`'s own constructor defaults, as applied by
        // `NDArrayDriverBase::new`.
        port_base.set_int32_param(nd.array_callbacks, 0, 1)?;
        port_base.set_float64_param(nd.pool_max_memory, 0, max_memory as f64 / MEGABYTE)?;

        // ADCSimDetector.cpp:93-104.
        port_base.set_string_param(nd.driver_version, 0, DRIVER_VERSION.into())?;
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
            pool: Arc::new(NDArrayPool::new(max_memory)),
            queued_counter: Arc::new(QueuedArrayCounter::new()),
            last_array: shared.last_array,
            acquiring: shared.acquiring,
            start_tx: shared.start_tx,
            stop_tx: shared.stop_tx,
        })
    }

    /// `asynNDArrayDriver::setIntegerParam(list, index, value)`
    /// (asynNDArrayDriver.cpp:636-663): writes to `ADAcquire` and
    /// `NDNumQueuedArrays` are intercepted to drive `ADAcquireBusy`.
    fn set_integer_param(&mut self, addr: i32, reason: usize, value: i32) -> AsynResult<()> {
        if reason == self.nd.acquire {
            if value == 0 {
                let wait_for_plugins = self
                    .port_base
                    .get_int32_param(self.nd.wait_for_plugins, addr)
                    .unwrap_or(0)
                    != 0;
                if !wait_for_plugins || self.queued_counter.get() == 0 {
                    self.port_base
                        .set_int32_param(self.nd.acquire_busy, addr, 0)?;
                }
            } else {
                self.port_base
                    .set_int32_param(self.nd.acquire_busy, addr, 1)?;
            }
        } else if reason == self.nd.num_queued_arrays
            && value == 0
            && self
                .port_base
                .get_int32_param(self.nd.acquire, addr)
                .unwrap_or(0)
                == 0
        {
            self.port_base
                .set_int32_param(self.nd.acquire_busy, addr, 0)?;
        }
        self.port_base.params.set_int32(reason, addr, value)
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

    /// The pool branch of `asynNDArrayDriver::writeInt32`
    /// (asynNDArrayDriver.cpp:684-694). `ad-core-rs` keeps its own
    /// implementation `pub(crate)` behind `NDArrayDriverBase`, so it is
    /// reproduced here over the public `NDArrayPool` API.
    fn write_int32_pool(&mut self, reason: usize) -> AsynResult<bool> {
        if reason == self.nd.pool_empty_free_list {
            self.pool.empty_free_list();
            self.refresh_pool_stats()?;
            Ok(true)
        } else if reason == self.nd.pool_poll_stats {
            self.refresh_pool_stats()?;
            Ok(true)
        } else if reason == self.nd.pool_pre_alloc {
            let template = self.last_array.lock().clone();
            if let Some(template) = template {
                let count = self
                    .port_base
                    .get_int32_param(self.nd.pool_num_pre_alloc_buffers, 0)
                    .unwrap_or(0)
                    .max(0) as usize;
                self.pool
                    .pre_allocate_buffers(&template, count)
                    .map_err(|e| AsynError::Status {
                        status: AsynStatus::Error,
                        message: e.to_string(),
                    })?;
                self.refresh_pool_stats()?;
            }
            self.port_base
                .set_int32_param(self.nd.pool_pre_alloc, 0, 0)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn refresh_pool_stats(&mut self) -> AsynResult<()> {
        let max_memory = self.pool.max_memory() as f64 / MEGABYTE;
        let used = self.pool.allocated_bytes() as f64 / MEGABYTE;
        let alloc = self.pool.num_alloc_buffers() as i32;
        let free = self.pool.num_free_buffers() as i32;
        self.port_base
            .set_float64_param(self.nd.pool_max_memory, 0, max_memory)?;
        self.port_base
            .set_float64_param(self.nd.pool_used_memory, 0, used)?;
        self.port_base
            .set_int32_param(self.nd.pool_alloc_buffers, 0, alloc)?;
        self.port_base
            .set_int32_param(self.nd.pool_free_buffers, 0, free)?;
        Ok(())
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
        let addr = user.addr;

        self.set_integer_param(addr, reason, value)?;

        if reason == self.nd.acquire {
            self.set_acquire(value);
        } else if self.sim.belongs_to_base(reason) {
            // `asynNDArrayDriver::writeInt32`: the pool parameters act, the rest
            // is just the parameter write already done above.
            self.write_int32_pool(reason)?;
        }

        self.port_base.call_param_callbacks(addr)?;
        Ok(())
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
    pool: Arc<NDArrayPool>,
    /// Index 0 is the 2-D array output; `1..=MAX_SIGNALS` the per-signal ones.
    outputs: Vec<Arc<parking_lot::Mutex<NDArrayOutput>>>,
    queued_counter: Arc<QueuedArrayCounter>,
    /// Owns the `SimDetTask` thread for the lifetime of the IOC.
    pub task_handle: std::thread::JoinHandle<()>,
}

impl CSimDetectorRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    /// The `NDArrayOutput` for asyn address `addr` (`0..=MAX_SIGNALS`).
    pub fn array_output(&self, addr: usize) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.outputs[addr]
    }

    pub fn connect_downstream(&self, addr: usize, mut sender: NDArraySender) {
        sender.set_queued_counter(self.queued_counter.clone());
        self.outputs[addr].lock().add(sender);
    }
}

/// `ADCSimDetectorConfig` (ADCSimDetector.cpp:412-420).
///
/// DEVIATION: C's `maxBuffers` caps the number of NDArrays the pool may
/// allocate. `ad-core-rs` 0.22.1's `NDArrayPool` is bounded by `maxMemory`
/// only, so `max_buffers` is accepted for iocsh signature parity and ignored.
pub fn create_c_sim_detector(
    port_name: &str,
    num_time_points: i32,
    data_type: NDDataType,
    _max_buffers: i32,
    max_memory: usize,
) -> AsynResult<CSimDetectorRuntime> {
    let (start_tx, start_rx) = rt::command_channel::<Signal>(1);
    let (stop_tx, stop_rx) = rt::command_channel::<Signal>(1);

    let last_array: LastArray = Arc::new(parking_lot::Mutex::new(None));
    let acquiring = Arc::new(AtomicBool::new(false));

    let det = CSimDetector::new(
        port_name,
        num_time_points,
        data_type,
        max_memory,
        SharedTaskState {
            last_array: last_array.clone(),
            acquiring: acquiring.clone(),
            start_tx,
            stop_tx: stop_tx.clone(),
        },
    )?;
    let nd_params = det.nd;
    let sim_params = det.sim;
    let pool = det.pool.clone();
    let queued_counter = det.queued_counter.clone();

    let (runtime_handle, _) = create_port_runtime(det, RuntimeConfig::default());

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
        queued: queued_counter.clone(),
        pool: pool.clone(),
        last_array,
        nd: nd_params,
        sim: sim_params,
    });

    Ok(CSimDetectorRuntime {
        runtime_handle,
        nd_params,
        sim_params,
        pool,
        outputs,
        queued_counter,
        task_handle,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

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
                    last_array: Arc::new(parking_lot::Mutex::new(None)),
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
        assert_eq!(f.get_i32(f.det.nd.array_callbacks, 0), 1);
        assert_eq!(
            f.det
                .port_base
                .get_string_param(f.det.nd.driver_version, 0)
                .unwrap(),
            "2.5.0"
        );
    }

    #[test]
    fn amplitude_and_period_defaults_apply_to_address_zero_only() {
        // C's two-argument `setDoubleParam` writes parameter list 0. Signals
        // 1..7 therefore start at 0 amplitude and 0 period.
        let f = Fixture::new();
        for addr in 1..MAX_SIGNALS as i32 {
            assert_eq!(f.get_f64(f.det.sim.amplitude, addr), 0.0, "addr {addr}");
            assert_eq!(f.get_f64(f.det.sim.period, addr), 0.0, "addr {addr}");
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
        f.det.queued_counter.increment();

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

    #[test]
    fn emptying_the_free_list_refreshes_the_pool_statistics() {
        let mut f = Fixture::new();
        f.write_i32(f.det.nd.pool_empty_free_list, 0, 1);
        assert_eq!(f.get_i32(f.det.nd.pool_free_buffers, 0), 0);
    }
}
