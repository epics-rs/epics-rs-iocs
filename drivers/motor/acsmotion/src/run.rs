//! The SPiiPlus profile thread: C `SPiiPlusController::profileThread`,
//! `runProfile`, `waitMotors`, `stopDataCollection` and `stopPEG`.
//!
//! The thread holds the controller port only where C's `runProfile` takes
//! `lock()`; its controller I/O goes through the comm, as C's does.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::runtime::task::{StackSizeClass, ThreadPriority, spawn_dedicated_thread};
use epics_rs::motor::controller_port::{
    ControllerCtx, ControllerPort, PROFILE_EXECUTE_DONE, PROFILE_EXECUTE_EXECUTING,
    PROFILE_EXECUTE_FLYBACK, PROFILE_MOVE_MODE_ABSOLUTE, PROFILE_STATUS_ABORT,
    PROFILE_STATUS_FAILURE, PROFILE_STATUS_SUCCESS, index,
};

use crate::profile::axes_to_string;
use crate::spiiplus::{AcsMotionParams, SpiiPlusController, fmt};

/// C `SPIIPLUS_MAX_DC_AXES`.
const MAX_DC_AXES: usize = 8;
/// The controller's segment buffer: C's 50 `POINT`s.
const POINT_BUFFER: usize = 50;
/// C's `epicsThreadSleep(0.1)` between checks.
const CHECK_PERIOD: Duration = Duration::from_millis(100);

/// The port side of the profile thread: C's `profileExecuteEvent_` and
/// `halted_`.
pub(crate) struct RunControl {
    execute: SyncSender<()>,
    pub(crate) halted: Arc<AtomicBool>,
}

impl RunControl {
    /// C `executeProfile`'s `epicsEventSignal`: a signal while one is
    /// pending is the same signal.
    pub(crate) fn signal(&self) {
        // Full: a signal is pending. Disconnected: no thread, as when C's
        // thread create failed.
        let _ = self.execute.try_send(());
    }
}

/// The thread side, until [`ProfileThread::spawn`] starts it on the
/// controller port.
pub struct ProfileThread {
    execute: Receiver<()>,
    halted: Arc<AtomicBool>,
    controller: Arc<Mutex<SpiiPlusController>>,
}

pub(crate) fn profile_thread(
    controller: Arc<Mutex<SpiiPlusController>>,
) -> (RunControl, ProfileThread) {
    let (execute, signals) = sync_channel(1);
    let halted = Arc::new(AtomicBool::new(false));
    let control = RunControl {
        execute,
        halted: halted.clone(),
    };
    let thread = ProfileThread {
        execute: signals,
        halted,
        controller,
    };
    (control, thread)
}

impl ProfileThread {
    /// C's constructor's `epicsThreadCreate("SPiiPlusProfile", ...)`: run
    /// a profile on controller port `port` each time it is executed.
    pub fn spawn(self, port: PortHandle) -> std::io::Result<()> {
        let runner = Runner {
            port,
            halted: self.halted,
            controller: self.controller,
        };
        let execute = self.execute;
        spawn_dedicated_thread(
            "SPiiPlusProfile".into(),
            ThreadPriority::Low,
            StackSizeClass::Medium,
            move || {
                while execute.recv().is_ok() {
                    runner.run_profile();
                }
            },
        )
        .map(drop)
    }
}

/// What `runProfile` reads when it starts: the last build and the
/// parameters C reads under its first `lock()`.
pub(crate) struct RunPlan {
    pub(crate) axes: Vec<usize>,
    pub(crate) start: Vec<f64>,
    pub(crate) flyback: Vec<f64>,
    /// Each profile axis's full profile.
    pub(crate) full_positions: Vec<Vec<f64>>,
    pub(crate) full_times: Vec<f64>,
    pub(crate) num_accel: usize,
    pub(crate) max_points: usize,
    pub(crate) data_collection_interval: f64,
    pub(crate) pulse_start_pos: f64,
    pub(crate) pulse_end_pos: f64,
    pub(crate) pulse_spacing: f64,
    /// The position data collection records for each recorded axis.
    pub(crate) recorded: Vec<&'static str>,
    pub(crate) start_pulses: i32,
    pub(crate) end_pulses: i32,
    pub(crate) num_points: i32,
    pub(crate) num_pulses: i32,
    pub(crate) pulse_mode: i32,
    pub(crate) pulse_axis: i32,
    pub(crate) peg_eng_enc_code: String,
    pub(crate) peg_out_assign_code: String,
    pub(crate) pouts_output_index: i32,
    pub(crate) pouts_bit_code: String,
    pub(crate) pulse_width: f64,
    pub(crate) move_mode: i32,
}

/// How a run ended short of success.
enum Stop {
    Failed,
    Aborted,
}

/// C `calculateCurrentPulse`: the pulse output has reached by
/// `current_point`, spread evenly from `start_pulse` to `end_pulse`.
pub(crate) fn current_pulse(
    current_point: i64,
    start_pulse: i32,
    end_pulse: i32,
    num_pulses: i32,
    pulse_mode: i32,
) -> i64 {
    if pulse_mode == 3 || current_point < i64::from(start_pulse) {
        0
    } else if current_point > i64::from(end_pulse) {
        i64::from(num_pulses)
    } else {
        let raw = (current_point - i64::from(start_pulse)) as f64
            * ((f64::from(num_pulses) - 1.0) / f64::from(end_pulse - start_pulse))
            + 1.0;
        // C lrint, under the default round-to-nearest-even.
        raw.round_ties_even() as i64
    }
}

struct Runner {
    port: PortHandle,
    halted: Arc<AtomicBool>,
    controller: Arc<Mutex<SpiiPlusController>>,
}

impl Runner {
    fn lock_controller(&self) -> MutexGuard<'_, SpiiPlusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C's `lock()` ... `unlock()` around a step.
    fn locked<R: Send + 'static>(
        &self,
        f: impl FnOnce(&mut AcsMotionParams, &mut ControllerCtx<'_>) -> R + Send + 'static,
    ) -> Option<R> {
        self.port
            .with_driver_blocking::<ControllerPort<AcsMotionParams>, _>(|d| d.with_params(f))
            .ok()
    }

    fn ack(&self, cmd: &str) -> AsynResult<()> {
        self.lock_controller().comm().write_read_ack(cmd)
    }

    fn halted(&self) -> bool {
        self.halted.load(Ordering::SeqCst)
    }

    /// `asynPrint(pasynUserSelf, mask, "SPiiPlusController:func: ...")`.
    fn print(&self, mask: TraceMask, func: &str, args: std::fmt::Arguments<'_>) {
        let port = self.lock_controller().port().to_string();
        PortManager::global().trace_manager().print(
            Some(&port),
            Some(0),
            0,
            mask,
            format_args!("SPiiPlusController:{func}: {args}\n"),
        );
    }

    /// Send `cmd` with C's `ASYN_TRACE_ERROR` echo of it.
    fn echo_ack(&self, cmd: &str) -> AsynResult<()> {
        self.print(TraceMask::ERROR, "runProfile", format_args!("{cmd}"));
        self.ack(cmd)
    }

    fn set_point(&self, current_point: i64, actual_pulses: i64) {
        self.locked(move |_, ctx| {
            let _ = ctx
                .base
                .set_int32_param(index::PROFILE_CURRENT_POINT, 0, current_point as i32);
            let _ = ctx
                .base
                .set_int32_param(index::PROFILE_ACTUAL_PULSES, 0, actual_pulses as i32);
            let _ = ctx.base.call_param_callbacks(0);
        });
    }

    fn set_execute_state(&self, state: i32) {
        self.locked(move |_, ctx| {
            let _ = ctx
                .base
                .set_int32_param(index::PROFILE_EXECUTE_STATE, 0, state);
            let _ = ctx.base.call_param_callbacks(0);
        });
    }

    /// C `wakeupPoller` then `waitMotors`: until no profile axis's poll
    /// says it moves.
    fn wait_motors(&self, axes: &[usize]) {
        self.locked(|_, ctx| ctx.wakeup_poller());
        loop {
            std::thread::sleep(CHECK_PERIOD);
            let ctrl = self.lock_controller();
            if !axes.iter().any(|&a| ctrl.is_moving(a)) {
                break;
            }
        }
        self.print(
            TraceMask::FLOW,
            "waitMotors",
            format_args!("motors are done moving"),
        );
    }

    /// C `stopDataCollection`.
    fn stop_data_collection(&self, axes: &[usize]) {
        for axis in axes.iter().take(MAX_DC_AXES) {
            let _ = self.ack(&format!("STOPDC/s {axis}"));
        }
    }

    /// C `stopPEG`.
    fn stop_peg(&self, pulse_axis: i32) {
        let _ = self.ack(&format!("STOPPEG {pulse_axis}"));
    }

    /// C `positionsToString`: each profile axis's position at `point`.
    fn positions(plan: &RunPlan, point: usize) -> String {
        plan.full_positions
            .iter()
            .map(|full| fmt(full[point]))
            .collect::<Vec<_>>()
            .join(",")
    }

    fn send_point(&self, plan: &RunPlan, axes: &str, point: usize) {
        let ms = (plan.full_times[point] * 1000.0).round() as i64;
        let _ = self.ack(&format!(
            "POINT {axes}, {}, {ms}",
            Self::positions(plan, point)
        ));
    }

    /// `?GSFREE(lead axis)`: the free segments in the controller's buffer.
    /// C keeps the last count when the query fails.
    fn free_points(&self, lead: usize, last: i64) -> i64 {
        self.lock_controller()
            .comm()
            .write_read_int(&format!("?GSFREE({lead})"))
            .map_or(last, i64::from)
    }

    /// C `runProfile`.
    fn run_profile(&self) {
        self.print(TraceMask::FLOW, "runProfile", format_args!("start"));
        let started = self.locked(|params, ctx| params.start_run(ctx));
        let (outcome, message) = match started {
            Some(Ok((plan, mut message))) => {
                let outcome = self.run_path(&plan, &mut message);
                (outcome, message)
            }
            Some(Err(message)) => (Err(Stop::Failed), message),
            None => (Err(Stop::Failed), String::new()),
        };
        let halted = self.halted.clone();
        let print_message = message.clone();
        let failed = outcome.is_err();
        self.locked(move |_, ctx| {
            let status = match outcome {
                Ok(()) => PROFILE_STATUS_SUCCESS,
                Err(Stop::Aborted) => PROFILE_STATUS_ABORT,
                Err(Stop::Failed) => PROFILE_STATUS_FAILURE,
            };
            let _ = ctx
                .base
                .set_int32_param(index::PROFILE_EXECUTE_STATUS, 0, status);
            let _ = ctx
                .base
                .set_string_param(index::PROFILE_EXECUTE_MESSAGE, 0, &message);
            // PROFILE_EXECUTE is a busy record: cleared only now.
            let _ = ctx.base.set_int32_param(index::PROFILE_EXECUTE, 0, 0);
            let _ = ctx
                .base
                .set_int32_param(index::PROFILE_EXECUTE_STATE, 0, PROFILE_EXECUTE_DONE);
            let _ = ctx.base.call_param_callbacks(0);
            halted.store(false, Ordering::SeqCst);
        });
        if failed {
            self.print(
                TraceMask::ERROR,
                "runProfile",
                format_args!("{print_message}"),
            );
        }
    }

    /// The body of C `runProfile` up to its `done:` label; `message` is
    /// left as C leaves its `message`.
    fn run_path(&self, plan: &RunPlan, message: &mut String) -> Result<(), Stop> {
        let axes = axes_to_string(&plan.axes);
        let absolute = plan.move_mode == PROFILE_MOVE_MODE_ABSOLUTE;
        let join = |values: &[f64]| values.iter().map(|&v| fmt(v)).collect::<Vec<_>>().join(",");
        let ptp = if absolute { "PTP/m" } else { "PTP/mr" };

        // To the start, accelerating distance before the profile.
        if self
            .ack(&format!("{ptp} {axes}, {}", join(&plan.start)))
            .is_err()
        {
            return Err(Stop::Failed);
        }
        self.wait_motors(&plan.axes);
        if self.halted() {
            *message = "Aborted during move to start".into();
            return Err(Stop::Aborted);
        }
        self.set_execute_state(PROFILE_EXECUTE_EXECUTING);

        // Data collection, started by the GO below: position, position
        // error and time of each axis into DC_DATA_n.
        let interval_ms = (plan.data_collection_interval * 1000.0).round() as i64;
        for (i, (&axis, pos)) in plan.axes.iter().zip(&plan.recorded).enumerate() {
            let n = i + 1;
            let _ = self.echo_ack(&format!("FILL(0,DC_DATA_{n})"));
            let _ = self.echo_ack(&format!(
                "DC/sw {axis},DC_DATA_{n},{},{interval_ms},{pos}({axis}),PE({axis}),TIME",
                plan.max_points
            ));
        }
        // The firmware does not start synchronized collection on the
        // PATH's GO, so collection starts here.
        let _ = self.ack(&format!("GO {axes}"));

        // Pulses count from 1; numElements+1 is the first deceleration
        // element.
        let num_elements = plan.num_points - 1;
        if plan.start_pulses < 1
            || plan.start_pulses > num_elements + 1
            || plan.end_pulses < plan.start_pulses
            || plan.end_pulses > num_elements + 1
        {
            *message = "Error: start or end pulses outside valid range".into();
            return Err(Stop::Failed);
        }

        let pulse_axis = plan.pulse_axis;
        // PEG_R/d needs ASSIGNPEG's /f.
        let assign = if plan.pulse_mode == 1 {
            "ASSIGNPEG/f"
        } else {
            "ASSIGNPEG"
        };
        let _ = self.echo_ack(&format!(
            "{assign} {pulse_axis}, {}, {}",
            plan.peg_eng_enc_code, plan.peg_out_assign_code
        ));
        let _ = self.echo_ack(&format!(
            "ASSIGNPOUTS {pulse_axis}, {}, {}",
            plan.pouts_output_index, plan.pouts_bit_code
        ));
        let width = fmt(plan.pulse_width);
        match plan.pulse_mode {
            0 | 2 => {
                self.print(
                    TraceMask::ERROR,
                    "runProfile",
                    format_args!(
                        "startPulsePos={:.6}, endPulsePos={:.6}, numElements={}, pulseInterval={:.6}",
                        plan.pulse_start_pos, plan.pulse_end_pos, plan.num_pulses, plan.pulse_spacing
                    ),
                );
                let cmd = format!(
                    "PEG_I {pulse_axis}, {width}, {}, {}, {}",
                    fmt(plan.pulse_start_pos),
                    fmt(plan.pulse_spacing),
                    fmt(plan.pulse_end_pos)
                );
                if self.echo_ack(&cmd).is_err() {
                    *message = "Aborting due to PEG_I error".into();
                    return Err(Stop::Failed);
                }
            }
            1 => {
                let cmd = format!(
                    "PEG_R/d {pulse_axis}, {width}, 0x4444, 0, {}, pulsePos",
                    plan.num_pulses - 1
                );
                if self.echo_ack(&cmd).is_err() {
                    *message = "Aborting due to PEG_R error".into();
                    return Err(Stop::Failed);
                }
            }
            _ => {}
        }
        if plan.pulse_mode != 3 {
            loop {
                // C indexes its axes with the pulse axis unchecked.
                let ready = usize::try_from(pulse_axis)
                    .ok()
                    .and_then(|a| self.lock_controller().peg_ready(a));
                let Some(ready) = ready else {
                    *message = "Error: pulse axis outside valid range".into();
                    return Err(Stop::Failed);
                };
                if ready {
                    break;
                }
                self.print(
                    TraceMask::ERROR,
                    "runProfile",
                    format_args!("axisStatus({pulse_axis}).#PEGREAD = 0"),
                );
                // The poller rereads the axis status meanwhile.
                std::thread::sleep(CHECK_PERIOD);
                if self.halted() {
                    self.stop_data_collection(&plan.axes);
                    self.stop_peg(pulse_axis);
                    *message = "Aborted during wait for PEGREADY".into();
                    return Err(Stop::Aborted);
                }
            }
        }

        let pulse = |point: i64| {
            current_pulse(
                point,
                plan.start_pulses,
                plan.end_pulses,
                plan.num_pulses,
                plan.pulse_mode,
            )
        };
        let full_size = plan.full_times.len() as i64;
        let num_accel = plan.num_accel as i64;
        let (mut loaded, mut executed, mut free) = (0i64, 0i64, 0i64);
        self.set_point(executed, pulse(executed));

        // The segments wait for the GO.
        let path = if absolute { "PATH/tw" } else { "PATH/twr" };
        let _ = self.ack(&format!("{path} {axes}"));
        self.print(
            TraceMask::FLOW,
            "runProfile",
            format_args!("point buffer fill start"),
        );
        for point in 0..plan.full_times.len().min(POINT_BUFFER) {
            self.send_point(plan, &axes, point);
            loaded += 1;
        }
        self.print(
            TraceMask::FLOW,
            "runProfile",
            format_args!("point buffer fill end"),
        );
        if full_size > POINT_BUFFER as i64 {
            let _ = self.ack(&format!("GO {axes}"));
            while loaded < full_size {
                if self.halted() {
                    self.stop_data_collection(&plan.axes);
                    self.stop_peg(pulse_axis);
                    *message = "Aborted during profile move".into();
                    return Err(Stop::Aborted);
                }
                std::thread::sleep(CHECK_PERIOD);
                free = self.free_points(plan.axes[0], free);
                executed += free;
                // C also sends the points past the profile's end the free
                // segments leave room for.
                for point in loaded..(loaded + free).min(full_size) {
                    self.print(
                        TraceMask::FLOW,
                        "runProfile",
                        format_args!("sending another point"),
                    );
                    self.send_point(plan, &axes, point as usize);
                }
                loaded += free;
                // Only the user's points count.
                if executed > num_accel {
                    self.set_point(executed - num_accel, pulse(executed - num_accel));
                } else {
                    self.set_point(0, 0);
                }
            }
            let _ = self.ack(&format!("ENDS {axes}"));
        } else {
            let _ = self.ack(&format!("ENDS {axes}"));
            let _ = self.ack(&format!("GO {axes}"));
        }

        let num_points = i64::from(plan.num_points);
        while executed < full_size {
            if self.halted() {
                self.stop_data_collection(&plan.axes);
                self.stop_peg(pulse_axis);
                *message = "Aborted during profile move".into();
                return Err(Stop::Aborted);
            }
            std::thread::sleep(CHECK_PERIOD);
            free = self.free_points(plan.axes[0], free);
            executed = full_size - POINT_BUFFER as i64 + free;
            if executed < num_accel {
                self.set_point(0, 0);
            } else if executed < num_accel + num_points {
                self.set_point(executed - num_accel, pulse(executed - num_accel));
            } else {
                self.set_point(num_points, pulse(num_points));
            }
        }
        self.print(
            TraceMask::FLOW,
            "runProfile",
            format_args!("profile move is done"),
        );
        // No triggers on the way to the flyback position.
        if plan.pulse_mode != 3 {
            self.stop_peg(pulse_axis);
        }

        self.set_execute_state(PROFILE_EXECUTE_FLYBACK);
        let _ = self.ack(&format!("{ptp} {axes}, {}", join(&plan.flyback)));
        self.wait_motors(&plan.axes);
        if self.halted() {
            *message = "Aborted during flyback".into();
            return Err(Stop::Aborted);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use epics_rs::motor::controller_port::PROFILE_BUILD_DONE;

    use epics_rs::asyn::user::AsynUser;

    use super::*;
    use crate::profile::tests::{Rig, SERVOS, rig};
    use crate::spiiplus::SpiiPlusAxis;
    use motor_common::testing::Polled;

    fn until(what: &str, done: impl Fn() -> bool) {
        let end = Instant::now() + Duration::from_secs(10);
        while !done() {
            assert!(Instant::now() < end, "timed out waiting for {what}");
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// The commands sent, without the pollers' queries.
    fn commands(rig: &Rig) -> Vec<String> {
        rig.sent()
            .into_iter()
            .filter(|c| !c.starts_with('?') || c.starts_with("?GSFREE"))
            // The controller poll's reads.
            .filter(|c| !c.starts_with("BIN ??") || c.starts_with("BIN ??DC_DATA"))
            .collect()
    }

    /// A built absolute profile of `points` on axis 0, pulses over every
    /// point, the controller's segment buffer always free.
    fn built(rig: Rig, points: &[f64], pulse_mode: i32) -> Rig {
        rig.int(rig.param("SPIIPLUS_PULSE_MODE"), 0, pulse_mode);
        rig.int(index::PROFILE_NUM_PULSES, 0, points.len() as i32);
        rig.int(index::PROFILE_START_PULSES, 0, 1);
        rig.int(index::PROFILE_END_PULSES, 0, points.len() as i32);
        rig.define(&[0], points, PROFILE_MOVE_MODE_ABSOLUTE);
        rig.int(index::PROFILE_BUILD, 0, 1);
        let state = rig
            .port
            .read_int32_blocking(index::PROFILE_BUILD_STATE, 0)
            .unwrap();
        let status = rig
            .port
            .read_int32_blocking(index::PROFILE_BUILD_STATUS, 0)
            .unwrap();
        assert_eq!(
            (state, status),
            (PROFILE_BUILD_DONE, PROFILE_STATUS_SUCCESS)
        );
        rig.line.lock().unwrap().set("GSFREE", &[50.0]);
        rig.sent();
        rig
    }

    /// A poller pass over axis 0, which the profile thread reads; the rig
    /// has no record to start the poller.
    fn poll(rig: &Rig, var: &str, value: i32) {
        rig.line.lock().unwrap().set(var, &[f64::from(value)]);
        rig.ctrl.lock().unwrap().poll().unwrap();
        SpiiPlusAxis::new(rig.ctrl.clone(), 0)
            .polled(&AsynUser::default())
            .unwrap();
    }

    fn was_sent(rig: &Rig, cmd: &str) -> bool {
        rig.line.lock().unwrap().sent.iter().any(|c| c == cmd)
    }

    fn int(rig: &Rig, index: usize) -> i32 {
        rig.port.read_int32_blocking(index, 0).unwrap()
    }

    /// Execute, and the run's status and message once it is done.
    fn execute(rig: &Rig) -> (i32, String) {
        rig.int(index::PROFILE_EXECUTE, 0, 1);
        until("the run", || int(rig, index::PROFILE_EXECUTE) == 0);
        assert_eq!(int(rig, index::PROFILE_EXECUTE_STATE), PROFILE_EXECUTE_DONE);
        (
            int(rig, index::PROFILE_EXECUTE_STATUS),
            rig.message(index::PROFILE_EXECUTE_MESSAGE),
        )
    }

    fn points(sent: &[String]) -> usize {
        sent.iter().filter(|c| c.starts_with("POINT ")).count()
    }

    #[test]
    fn a_short_profile_runs_from_start_to_flyback() {
        let rig = built(rig("spiiRShort", 1, SERVOS), &[0.0, 1.0, 2.0, 3.0], 3);
        assert_eq!(
            execute(&rig),
            (PROFILE_STATUS_SUCCESS, "Selected axes: 1".into())
        );
        let sent = commands(&rig);
        let others: Vec<&str> = sent
            .iter()
            .filter(|c| !c.starts_with("POINT ") && !c.starts_with("DC/sw "))
            .map(String::as_str)
            .collect();
        assert_eq!(
            others,
            [
                "PTP/m 0, -0.5",
                "FILL(0,DC_DATA_1)",
                "GO 0",
                "ASSIGNPEG 0, , ",
                "ASSIGNPOUTS 0, 0, ",
                "PATH/tw 0",
                "ENDS 0",
                "GO 0",
                "?GSFREE(0)",
                "PTP/m 0, 15",
            ]
        );
        assert!(sent[2].starts_with("DC/sw 0,DC_DATA_1,8,"));
        assert!(sent[2].ends_with(",FPOS(0),PE(0),TIME"));
        // Ten segments each way around the profile's three.
        assert_eq!(points(&sent), 23);
        // The profile's second point follows the acceleration.
        assert_eq!(sent[17], "POINT 0, 5, 500");
        assert_eq!(int(&rig, index::PROFILE_CURRENT_POINT), 4);
    }

    #[test]
    fn a_long_profile_loads_only_its_own_points_into_the_freed_buffer() {
        let rig = rig("spiiRLong", 1, SERVOS);
        rig.with(|params, ctx| params.initialize_profile_pulses(ctx, 60, 3))
            .unwrap();
        let positions: Vec<f64> = (0..60).map(f64::from).collect();
        let rig = built(rig, &positions, 3);
        assert_eq!(execute(&rig).0, PROFILE_STATUS_SUCCESS);
        let sent = commands(&rig);
        // 79 segments; C sends the 21 after them the 50 free ones allow.
        assert_eq!(points(&sent), 79);
        let go = sent.iter().position(|c| c == "PATH/tw 0").unwrap() + 51;
        assert_eq!(sent[go], "GO 0");
        assert_eq!(sent[go + 1], "?GSFREE(0)");
        assert_eq!(sent[go + 31], "ENDS 0");
        assert_eq!(int(&rig, index::PROFILE_CURRENT_POINT), 60);
    }

    #[test]
    fn fixed_pulses_wait_for_peg_ready_and_stop_before_the_flyback() {
        let rig = built(rig("spiiRPeg", 1, SERVOS), &[0.0, 1.0, 2.0, 3.0], 0);
        rig.int(index::PROFILE_EXECUTE, 0, 1);
        until("the PEG", || was_sent(&rig, "PEG_I 0, 0, 0, 5, 15"));
        std::thread::sleep(Duration::from_millis(300));
        assert!(!was_sent(&rig, "PATH/tw 0"));
        poll(&rig, "AST", 1 << 4);
        until("the run", || int(&rig, index::PROFILE_EXECUTE) == 0);
        assert_eq!(
            int(&rig, index::PROFILE_EXECUTE_STATUS),
            PROFILE_STATUS_SUCCESS
        );
        let sent = commands(&rig);
        let flyback = sent.iter().position(|c| c == "PTP/m 0, 15").unwrap();
        assert_eq!(sent[flyback - 1], "STOPPEG 0");
        assert_eq!(int(&rig, index::PROFILE_ACTUAL_PULSES), 4);
    }

    #[test]
    fn pulses_outside_the_profile_fail_the_run_before_the_peg() {
        let rig = built(rig("spiiRRange", 1, SERVOS), &[0.0, 1.0, 2.0, 3.0], 3);
        rig.int(index::PROFILE_START_PULSES, 0, 0);
        assert_eq!(
            execute(&rig),
            (
                PROFILE_STATUS_FAILURE,
                "Error: start or end pulses outside valid range".into()
            )
        );
        assert_eq!(commands(&rig).last().unwrap(), "GO 0");
    }

    #[test]
    fn an_abort_halts_the_axes_and_ends_the_run_at_its_next_check() {
        let rig = built(rig("spiiRAbort", 1, SERVOS), &[0.0, 1.0, 2.0, 3.0], 3);
        // Axis 0 moves until the test lets it stop.
        poll(&rig, "MST", 1 << 5);
        rig.int(index::PROFILE_EXECUTE, 0, 1);
        until("the move to the start", || was_sent(&rig, "PTP/m 0, -0.5"));
        rig.int(index::PROFILE_ABORT, 0, 1);
        poll(&rig, "MST", 0);
        until("the run", || int(&rig, index::PROFILE_EXECUTE) == 0);
        assert_eq!(
            int(&rig, index::PROFILE_EXECUTE_STATUS),
            PROFILE_STATUS_ABORT
        );
        assert_eq!(
            rig.message(index::PROFILE_EXECUTE_MESSAGE),
            "Aborted during move to start"
        );
        assert_eq!(commands(&rig), ["PTP/m 0, -0.5", "HALT 0"]);
    }

    #[test]
    fn an_abort_with_no_run_sends_nothing() {
        let rig = built(rig("spiiRIdle", 1, SERVOS), &[0.0, 1.0, 2.0, 3.0], 3);
        rig.int(index::PROFILE_ABORT, 0, 1);
        assert!(commands(&rig).is_empty());
    }

    #[test]
    fn current_pulse_spreads_the_pulses_from_start_to_end() {
        assert_eq!(current_pulse(5, 1, 9, 5, 3), 0);
        assert_eq!(current_pulse(0, 1, 9, 5, 0), 0);
        assert_eq!(current_pulse(10, 1, 9, 5, 0), 5);
        assert_eq!(current_pulse(1, 1, 9, 5, 0), 1);
        assert_eq!(current_pulse(9, 1, 9, 5, 0), 5);
        // 2.5 and 3.5 round to even, as lrint does.
        assert_eq!(current_pulse(4, 1, 5, 3, 0), 2);
        assert_eq!(current_pulse(4, 1, 7, 6, 0), 4);
    }
}
