//! `drvFastSweep` — a software MCA that sweeps an upstream asyn
//! `asynInt32Array` source into channels, ported from `drvFastSweep.cpp`/
//! `drvFastSweep.h` (`mcaApp/mcaSrc`). The first driver built against
//! [`crate::interface`]/[`crate::dev_mca_asyn`]; round-2 vendor crates
//! follow the same shape (a [`PortDriver`] whose mca-command params are
//! [`McaReason::create_params`]).
//!
//! # Architecture
//! `maxSignals` channels share one "sweep clock" -- one upstream
//! `asynInt32Array` sample (C `dataCallback(newData, nelem)`, `nelem ==
//! maxSignals`) supplies one new data point per signal, per callback;
//! `nextPoint` (`drvFastSweep.cpp:249-274`) writes signal `i`'s new sample
//! into channel `numAcquired_` of that signal's own spectrum row, then
//! advances `numAcquired_` for every signal at once. A record's own asyn
//! `addr` (`getAddress`, `readInt32Array`, `drvFastSweep.cpp:351`) selects
//! which signal's spectrum it reads -- there is one FastSweep instance
//! (one asyn port) per `maxSignals` signals, not one per record.
//!
//! An optional upstream `asynFloat64` interval source (C `intervalCallback`,
//! `drvFastSweep.cpp:213-219`) retunes how many upstream samples are
//! averaged into one channel (`computeNumAverage`,
//! `drvFastSweep.cpp:276-284`) so the recorded dwell time tracks a whole
//! multiple of the upstream sample period.
//!
//! # Restructuring vs. C
//! C's status/current-channel publication from the data/interval callback
//! threads goes through `setIntegerParam`/`setDoubleParam` +
//! `callParamCallbacks` directly, because `asynPortDriver`'s param list is
//! already thread-safe under its own lock. asyn-rs's [`PortDriverBase`] is
//! owned by the port's single actor task instead, so a callback firing on a
//! background OS thread (this crate's data/interval subscriber threads, not
//! the actor) cannot reach it directly; it instead builds a
//! `Vec<ParamSetValue>` and hands it to
//! [`PortHandle::set_params_and_notify_blocking`], which the actor applies
//! as one atomic `set_value` + `call_param_callbacks` step (see
//! `drivers/ur-robot/src/drivers/control.rs`'s poll thread for the same
//! pattern). [`SweepState`] plays the role of C's directly-shared `pData_`/
//! `numAcquired_`/... fields, guarded by one `Mutex` instead of asyn's own
//! port lock.
//!
//! # Preserved upstream quirks (not "fixed")
//! - **Every scalar mca param collapses to addr 0**, matching C's 2-arg
//!   `setIntegerParam(command, value)`/`setDoubleParam` overloads
//!   (`drvFastSweep.cpp:298,333`), which ignore `pasynUser->addr` entirely.
//!   A record bound to a nonzero addr's status params reads stale/zero
//!   status forever -- an intentional characteristic of one shared sweep
//!   clock across `maxSignals` channels, not something [`crate::dev_mca_asyn`]
//!   (itself correctly generic over its own fixed addr) needs to special-case.
//! - **`NumChannels` caches an out-of-range value before rejecting it.** C's
//!   `writeInt32` calls `setIntegerParam(command, value)` unconditionally
//!   before the `1 <= value <= maxPoints_` bounds check
//!   (`drvFastSweep.cpp:317-322`); an invalid write leaves the cached
//!   `MCA_NUM_CHANNELS` param out of sync with the real internal channel
//!   count while still returning an error. Reproduced as-is -- it does not
//!   corrupt anything, only the cached readback of a rejected write.
//! - **`Sequence`/`Prescale`/`PresetSweeps`/`PresetLowChannel`/
//!   `PresetHighChannel`/`AcquireMode`/`PresetLiveTime`/`PresetCounts`/
//!   `ChannelAdvanceSource` are accepted and cached but never acted on.**
//!   FastSweep is a minimal proving driver (PHA/MCS preset/ROI logic is
//!   `McaRecord`'s own job), matching C exactly (`drvFastSweep.cpp` has no
//!   case for them beyond the generic post-switch cache).
//!
//! # Fixed (not reproduced) upstream defects
//! - **`readInt32Array` buffer overflow.** C's `readInt32Array`
//!   (`drvFastSweep.cpp:345-356`) ignores the caller's `maxChans` capacity
//!   entirely and always `memcpy`s `numPoints_` ints into the caller buffer
//!   -- an overflow if `numPoints_ > maxChans`. [`FastSweepDriver::read_int32_array`]
//!   clamps to `buf.len()` (the caller's real capacity, carried directly in
//!   the trait signature here, unlike C's separate out-parameter).
//! - **`computeNumAverage` division by zero / UB.** C computes
//!   `numAverage_ = (int)(dwellTime_/callbackInterval_ + 0.5)` with no guard
//!   against `callbackInterval_ == 0.0` (`drvFastSweep.cpp:278`) -- a
//!   possible state before any interval sample has arrived, or if the
//!   upstream interval source is absent. `inf`/`nan` cast to `int` is
//!   undefined behaviour in C; Rust's saturating float-to-int cast would
//!   instead silently freeze acquisition forever (`numAverage_` pinned at
//!   `i32::MAX`, never reaching the accumulation threshold).
//!   [`SweepState::compute_num_average`] guards `callback_interval <= 0.0`
//!   and falls back to `num_average = 1` directly, extending the file's own
//!   existing "unsupported interval source" fallback to also cover this case.

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Instant;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interrupt::{InterruptFilter, InterruptReceiver, InterruptSubscription};
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::{DrvUserRequest, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;

use crate::interface::McaReason;

/// C `#define fastSweepMaxChannelsString "FAST_SWEEP_MAX_CHANNELS"`
/// (`drvFastSweep.h:20`).
pub const FAST_SWEEP_MAX_CHANNELS: &str = "FAST_SWEEP_MAX_CHANNELS";
/// C `#define fastSweepCurrentChannelString "FAST_SWEEP_CURRENT_CHANNEL"`
/// (`drvFastSweep.h:21`).
pub const FAST_SWEEP_CURRENT_CHANNEL: &str = "FAST_SWEEP_CURRENT_CHANNEL";

fn asyn_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// C's directly-shared `pData_`/`numAcquired_`/`numPoints_`/`acquiring_`/...
/// fields (`drvFastSweep.h:60-100`), guarded by one `Mutex` in place of
/// asyn's own port lock (see module doc).
struct SweepState {
    max_signals: usize,
    max_points: usize,
    /// C `pData_[maxSignals_*maxPoints_]`, column-major: signal `i`'s
    /// channel `j` is `p_data[j + i*max_points]` (`nextPoint`'s `offset =
    /// numAcquired_ + i*maxPoints_`, `drvFastSweep.cpp:257-260`).
    p_data: Vec<i32>,
    num_acquired: i32,
    num_points: i32,
    acquiring: bool,
    real_time: f64,
    elapsed_time: f64,
    start_time: Instant,
    dwell_time: f64,
    callback_interval: f64,
    num_average: i32,
    accumulated: i32,
    /// C `pAverageStore_[maxSignals_]` (`drvFastSweep.cpp:34`).
    average_store: Vec<f64>,
}

impl SweepState {
    fn new(max_signals: usize, max_points: usize) -> Self {
        SweepState {
            max_signals,
            max_points,
            p_data: vec![0; max_signals * max_points],
            num_acquired: 0,
            num_points: 0,
            acquiring: false,
            real_time: 0.0,
            elapsed_time: 0.0,
            start_time: Instant::now(),
            dwell_time: 0.0,
            callback_interval: 0.0,
            num_average: 1,
            accumulated: 0,
            average_store: vec![0.0; max_signals],
        }
    }

    /// C `stopAcquire()` (`drvFastSweep.cpp:286-290`), minus the
    /// `setIntegerParam`/`callParamCallbacks` publish, which the caller
    /// does (it alone can reach `PortDriverBase`/`PortHandle`).
    fn stop_acquire(&mut self) {
        self.acquiring = false;
    }

    /// C `computeNumAverage()` (`drvFastSweep.cpp:276-284`), with the
    /// division-by-zero/UB guard described in the module doc.
    fn compute_num_average(&mut self) {
        self.num_average = if self.callback_interval <= 0.0 {
            1
        } else {
            (self.dwell_time / self.callback_interval).round() as i32
        }
        .max(1);
        self.accumulated = 0;
        self.dwell_time = self.callback_interval * f64::from(self.num_average);
    }

    /// C `nextPoint(newData)` (`drvFastSweep.cpp:249-274`). Returns the
    /// param updates the caller must publish: current channel and elapsed
    /// real time are published on every call, acquiring only if this
    /// sample just finished the sweep.
    fn next_point(
        &mut self,
        new_data: &[i32],
        reasons: &[usize; McaReason::COUNT],
        current_channel_reason: usize,
    ) -> Vec<ParamSetValue> {
        if !self.acquiring {
            return Vec::new();
        }
        for (i, &value) in new_data.iter().take(self.max_signals).enumerate() {
            let offset = i * self.max_points + self.num_acquired as usize;
            if offset < self.p_data.len() {
                self.p_data[offset] = value;
            }
        }
        self.num_acquired += 1;
        if self.num_acquired >= self.num_points {
            self.stop_acquire();
        }
        self.elapsed_time = self.start_time.elapsed().as_secs_f64();
        if self.real_time > 0.0 && self.elapsed_time >= self.real_time {
            self.stop_acquire();
        }

        let mut updates = vec![
            ParamSetValue::new(
                current_channel_reason,
                0,
                ParamValue::Int32(self.num_acquired),
            ),
            ParamSetValue::new(
                reasons[McaReason::ElapsedRealTime as usize],
                0,
                ParamValue::Float64(self.elapsed_time),
            ),
        ];
        if !self.acquiring {
            updates.push(ParamSetValue::new(
                reasons[McaReason::Acquiring as usize],
                0,
                ParamValue::Int32(0),
            ));
        }
        updates
    }

    /// C `readInt32Array`'s copy (`drvFastSweep.cpp:351-354`), with the
    /// buffer overflow fixed at source: C ignores the caller's `maxChans`
    /// capacity entirely and always `memcpy`s `numPoints_` ints, an
    /// overflow if `numPoints_ > maxChans`. `buf.len()` -- the caller's
    /// real capacity, carried directly in the trait signature here unlike
    /// C's separate out-parameter -- clamps the copy. Returns `nactual`.
    fn read_signal(&self, signal: usize, buf: &mut [i32]) -> usize {
        let copy_len = (self.num_points.max(0) as usize).min(buf.len());
        let row = signal * self.max_points;
        buf[..copy_len].copy_from_slice(&self.p_data[row..row + copy_len]);
        (self.num_acquired.max(0) as usize).min(copy_len)
    }
}

/// State the data/interval subscriber threads share with
/// [`FastSweepDriver`]'s write handlers: the [`SweepState`] lock plus the
/// reason table both sides need to build/apply [`ParamSetValue`]s.
struct Shared {
    state: Mutex<SweepState>,
    reasons: [usize; McaReason::COUNT],
    current_channel_reason: usize,
}

/// C's `drvFastSweep` (`drvFastSweep.h`).
pub struct FastSweepDriver {
    base: PortDriverBase,
    shared: Arc<Shared>,
}

impl FastSweepDriver {
    /// C `initFastSweep(portName, inputName, maxSignals, maxPoints,
    /// dataString, intervalString)` (`drvFastSweep.cpp:57-211`).
    ///
    /// The upstream `asynInt32Array` data connection is mandatory (matches
    /// C's hard-fail `goto error` if either the interface lookup, the
    /// `drvUserCreate`, or the `registerInterruptUser` fails). The
    /// `asynFloat64` interval connection is best-effort (matches C: a
    /// failed `drvUserCreate` here leaves `numAverage_` at its default `1`
    /// rather than failing the whole driver).
    pub fn connect(
        port_name: &str,
        input_port_name: &str,
        max_signals: usize,
        max_points: usize,
        data_drv_info: &str,
        interval_drv_info: &str,
    ) -> AsynResult<Self> {
        // C `if (dataString[0]==0) dataString_ = "DATA";` /
        // `if (intervalString[0]==0) intervalString_ = "SCAN_PERIOD";`
        // (`drvFastSweep.cpp:116-125`).
        let data_drv_info = if data_drv_info.is_empty() {
            "DATA"
        } else {
            data_drv_info
        };
        let interval_drv_info = if interval_drv_info.is_empty() {
            "SCAN_PERIOD"
        } else {
            interval_drv_info
        };

        let input = get_port(input_port_name)
            .ok_or_else(|| asyn_error(format!("no asyn port named '{input_port_name}'")))?;

        let mut base = PortDriverBase::new(
            port_name,
            max_signals,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let reasons = McaReason::create_params(&mut base)?;
        let max_channels_idx = base.create_param(FAST_SWEEP_MAX_CHANNELS, ParamType::Int32)?;
        let current_channel_idx =
            base.create_param(FAST_SWEEP_CURRENT_CHANNEL, ParamType::Int32)?;
        // C `setIntegerParam(fastSweepMaxChannels_, maxPoints_);
        // setIntegerParam(fastSweepCurrentChannel_, 0);` (`drvFastSweep.cpp:113-114`).
        base.params
            .set_int32(max_channels_idx, 0, max_points as i32)?;
        base.params.set_int32(current_channel_idx, 0, 0)?;

        let shared = Arc::new(Shared {
            state: Mutex::new(SweepState::new(max_signals, max_points)),
            reasons,
            current_channel_reason: current_channel_idx,
        });

        let data_req = DrvUserRequest::new(data_drv_info, 0).with_iface(InterfaceType::Int32Array);
        let data_info = input.handle.drv_user_create_blocking(&data_req)?;
        let (data_sub, data_rx) =
            input
                .handle
                .interrupts()
                .register_interrupt_user(InterruptFilter {
                    reason: Some(data_info.reason),
                    ..Default::default()
                });
        spawn_data_thread(data_sub, data_rx, input.handle.clone(), Arc::clone(&shared));

        // Best-effort: C falls back to `numAverage_ = 1` rather than
        // failing the driver when no interval source is configured
        // (`drvFastSweep.cpp:200-208`).
        let interval_req =
            DrvUserRequest::new(interval_drv_info, 0).with_iface(InterfaceType::Float64);
        if let Ok(interval_info) = input.handle.drv_user_create_blocking(&interval_req) {
            // C's one-time `pasynFloat64SyncIO->read` seed before any
            // callback arrives (`drvFastSweep.cpp:207`).
            if let Ok(seed) = input.handle.read_float64_blocking(interval_info.reason, 0) {
                shared.state.lock().unwrap().callback_interval = seed;
            }
            let (interval_sub, interval_rx) =
                input
                    .handle
                    .interrupts()
                    .register_interrupt_user(InterruptFilter {
                        reason: Some(interval_info.reason),
                        ..Default::default()
                    });
            spawn_interval_thread(
                interval_sub,
                interval_rx,
                input.handle.clone(),
                Arc::clone(&shared),
            );
        }

        Ok(FastSweepDriver { base, shared })
    }
}

impl PortDriver for FastSweepDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// C `writeInt32` (`drvFastSweep.cpp:292-325`).
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        // C `setIntegerParam(command, value)` -- the 2-arg overload,
        // addr-0 always (see module doc's "every scalar param collapses to
        // addr 0" quirk).
        self.base.params.set_int32(reason, 0, value)?;

        let r = self.shared.reasons;
        let mut result = Ok(());

        if reason == r[McaReason::StartAcquire as usize] {
            let just_started = {
                let mut state = self.shared.state.lock().unwrap();
                if state.acquiring {
                    false
                } else {
                    state.acquiring = true;
                    state.start_time = Instant::now();
                    true
                }
            };
            if just_started {
                self.base
                    .params
                    .set_int32(r[McaReason::Acquiring as usize], 0, 1)?;
            }
        } else if reason == r[McaReason::StopAcquire as usize] {
            self.shared.state.lock().unwrap().stop_acquire();
            self.base
                .params
                .set_int32(r[McaReason::Acquiring as usize], 0, 0)?;
        } else if reason == r[McaReason::Erase as usize] {
            {
                let mut state = self.shared.state.lock().unwrap();
                state.p_data.iter_mut().for_each(|v| *v = 0);
                state.num_acquired = 0;
                state.elapsed_time = 0.0;
                state.start_time = Instant::now();
            }
            self.base
                .params
                .set_float64(r[McaReason::ElapsedRealTime as usize], 0, 0.0)?;
        } else if reason == r[McaReason::NumChannels as usize] {
            let mut state = self.shared.state.lock().unwrap();
            // C caches the value (above) before this bounds check, and the
            // error return does not undo that cache -- preserved (module
            // doc).
            if value < 1 || value as usize > state.max_points {
                result = Err(asyn_error(format!(
                    "NumChannels {value} out of range 1..={}",
                    state.max_points
                )));
            } else {
                state.num_points = value;
            }
        }

        self.base.call_param_callbacks(0)?;
        result
    }

    /// C `writeFloat64` (`drvFastSweep.cpp:327-343`).
    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        self.base.params.set_float64(reason, 0, value)?;

        let r = self.shared.reasons;

        if reason == r[McaReason::DwellTime as usize] {
            let dwell = {
                let mut state = self.shared.state.lock().unwrap();
                state.dwell_time = value;
                state.compute_num_average();
                state.dwell_time
            };
            self.base
                .params
                .set_float64(r[McaReason::DwellTime as usize], 0, dwell)?;
        } else if reason == r[McaReason::PresetRealTime as usize] {
            self.shared.state.lock().unwrap().real_time = value;
        }

        self.base.call_param_callbacks(0)?;
        Ok(())
    }

    /// C `readInt32Array` (`drvFastSweep.cpp:345-356`), with the buffer
    /// overflow fixed at source (module doc).
    fn read_int32_array(&mut self, user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        let state = self.shared.state.lock().unwrap();
        let signal = usize::try_from(user.addr)
            .ok()
            .filter(|s| *s < state.max_signals)
            .ok_or_else(|| asyn_error(format!("signal address {} out of range", user.addr)))?;

        Ok(state.read_signal(signal, buf))
    }
}

/// C `dataCallback` (`drvFastSweep.cpp:221-246`): accumulate/average, then
/// `nextPoint`.
fn spawn_data_thread(
    sub: InterruptSubscription,
    mut rx: InterruptReceiver,
    handle: PortHandle,
    shared: Arc<Shared>,
) {
    thread::Builder::new()
        .name("fastsweep-data".into())
        .spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .expect("failed to build the fastsweep data thread's runtime");
            rt.block_on(async move {
                // Keeps the subscription (and therefore delivery) alive for
                // as long as this thread runs.
                let _sub = sub;
                while let Some(iv) = rx.recv().await {
                    let ParamValue::Int32Array(new_data) = iv.value else {
                        continue;
                    };
                    let reasons = shared.reasons;
                    let current_channel_reason = shared.current_channel_reason;
                    let updates = {
                        let mut state = shared.state.lock().unwrap();
                        if state.num_average == 1 {
                            state.next_point(&new_data, &reasons, current_channel_reason)
                        } else {
                            for (i, &v) in new_data.iter().take(state.max_signals).enumerate() {
                                state.average_store[i] += f64::from(v);
                            }
                            state.accumulated += 1;
                            if state.accumulated >= state.num_average {
                                let averaged: Vec<i32> = state
                                    .average_store
                                    .iter()
                                    .map(|&sum| (sum / f64::from(state.accumulated)).round() as i32)
                                    .collect();
                                state.average_store.iter_mut().for_each(|v| *v = 0.0);
                                state.accumulated = 0;
                                state.next_point(&averaged, &reasons, current_channel_reason)
                            } else {
                                Vec::new()
                            }
                        }
                    };
                    if !updates.is_empty() {
                        let _ = handle.set_params_and_notify_blocking(0, updates);
                    }
                }
            });
        })
        .expect("failed to spawn the fastsweep data thread");
}

/// C `intervalCallback` (`drvFastSweep.cpp:213-219`).
fn spawn_interval_thread(
    sub: InterruptSubscription,
    mut rx: InterruptReceiver,
    handle: PortHandle,
    shared: Arc<Shared>,
) {
    thread::Builder::new()
        .name("fastsweep-interval".into())
        .spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .expect("failed to build the fastsweep interval thread's runtime");
            rt.block_on(async move {
                let _sub = sub;
                while let Some(iv) = rx.recv().await {
                    let ParamValue::Float64(seconds) = iv.value else {
                        continue;
                    };
                    let reason = shared.reasons[McaReason::DwellTime as usize];
                    let dwell = {
                        let mut state = shared.state.lock().unwrap();
                        state.callback_interval = seconds;
                        state.compute_num_average();
                        state.dwell_time
                    };
                    let updates = vec![ParamSetValue::new(reason, 0, ParamValue::Float64(dwell))];
                    let _ = handle.set_params_and_notify_blocking(0, updates);
                }
            });
        })
        .expect("failed to spawn the fastsweep interval thread");
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `McaReason`-indexed table of dummy, distinct indices -- enough for
    /// [`SweepState::next_point`]'s tests to tell which reasons it published.
    fn dummy_reasons() -> [usize; McaReason::COUNT] {
        let mut r = [0usize; McaReason::COUNT];
        for (i, slot) in r.iter_mut().enumerate() {
            *slot = i + 1;
        }
        r
    }

    fn find_int32(updates: &[ParamSetValue], reason: usize) -> Option<i32> {
        updates.iter().find_map(|u| match u {
            ParamSetValue::Value {
                reason: r,
                value: ParamValue::Int32(v),
                ..
            } if *r == reason => Some(*v),
            _ => None,
        })
    }

    fn find_float64(updates: &[ParamSetValue], reason: usize) -> Option<f64> {
        updates.iter().find_map(|u| match u {
            ParamSetValue::Value {
                reason: r,
                value: ParamValue::Float64(v),
                ..
            } if *r == reason => Some(*v),
            _ => None,
        })
    }

    /// C `computeNumAverage`'s `(int)(dwellTime_/callbackInterval_ + 0.5)`
    /// (`drvFastSweep.cpp:278`) for a normal, nonzero interval.
    #[test]
    fn compute_num_average_rounds_to_a_whole_multiple_of_the_interval() {
        let mut state = SweepState::new(1, 10);
        state.dwell_time = 1.0;
        state.callback_interval = 0.3;
        state.compute_num_average();
        assert_eq!(state.num_average, 3);
        assert!((state.dwell_time - 0.9).abs() < 1e-9);
        assert_eq!(state.accumulated, 0);
    }

    /// The division-by-zero/UB fix: C's `(int)(dwellTime_/0.0 + 0.5)` is
    /// undefined behaviour and would saturate to `i32::MAX` under Rust's
    /// defined `as i32` cast, freezing acquisition forever. This must
    /// resolve to `num_average = 1` instead (module doc).
    #[test]
    fn compute_num_average_guards_against_a_zero_callback_interval() {
        let mut state = SweepState::new(1, 10);
        state.dwell_time = 1.0;
        state.callback_interval = 0.0;
        state.compute_num_average();
        assert_eq!(state.num_average, 1);
    }

    /// C `nextPoint`'s early return: `if (!acquiring_) return;`
    /// (`drvFastSweep.cpp:255`).
    #[test]
    fn next_point_is_a_no_op_when_not_acquiring() {
        let mut state = SweepState::new(2, 4);
        let reasons = dummy_reasons();
        let updates = state.next_point(&[1, 2], &reasons, 999);
        assert!(updates.is_empty());
        assert_eq!(state.num_acquired, 0);
    }

    /// C `nextPoint`'s per-signal write + always-publish current channel /
    /// elapsed real time (`drvFastSweep.cpp:257-273`), across the fixed
    /// buffer-overflow read path.
    #[test]
    fn next_point_writes_every_signal_and_publishes_current_channel() {
        let mut state = SweepState::new(2, 4);
        state.acquiring = true;
        state.num_points = 4;

        let reasons = dummy_reasons();
        let updates = state.next_point(&[10, 20], &reasons, 999);

        assert_eq!(state.num_acquired, 1);
        assert_eq!(state.p_data[0], 10); // signal 0, channel 0
        assert_eq!(state.p_data[4], 20); // signal 1, channel 0
        assert_eq!(find_int32(&updates, 999), Some(1));
        assert!(find_float64(&updates, reasons[McaReason::ElapsedRealTime as usize]).is_some());
        assert!(state.acquiring);
        assert_eq!(
            find_int32(&updates, reasons[McaReason::Acquiring as usize]),
            None
        );
    }

    /// C `nextPoint`'s `if (numAcquired_ >= numPoints_) stopAcquire();`
    /// (`drvFastSweep.cpp:263-265`) -- the sweep-complete boundary.
    #[test]
    fn next_point_stops_acquiring_and_publishes_it_when_the_sweep_completes() {
        let mut state = SweepState::new(1, 1);
        state.acquiring = true;
        state.num_points = 1;

        let reasons = dummy_reasons();
        let updates = state.next_point(&[7], &reasons, 999);

        assert!(!state.acquiring);
        assert_eq!(
            find_int32(&updates, reasons[McaReason::Acquiring as usize]),
            Some(0)
        );
    }

    /// `readInt32Array`'s buffer-overflow fix: a caller buffer shorter than
    /// `numPoints_` must clamp the copy, not overrun it (module doc).
    #[test]
    fn read_int32_array_clamps_to_the_caller_buffer_capacity() {
        let mut state = SweepState::new(1, 8);
        state.num_points = 8;
        state.num_acquired = 8;
        for (i, v) in state.p_data.iter_mut().enumerate() {
            *v = i as i32;
        }

        let mut buf = [0i32; 3];
        let nactual = state.read_signal(0, &mut buf);

        assert_eq!(buf, [0, 1, 2]);
        assert_eq!(nactual, 3);
    }
}
