//! Aerotech A3200 motor controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorAerotech/aerotechApp/src/drvA3200Asyn.cc` (the
//! `motorAxisDrvSET_t` "motorAxis" API over `pasynOctetSyncIO`). The A3200 is the
//! Ensemble's larger sibling: it speaks the same ASCII reply protocol (each reply
//! begins with a status char — `%` ACK, `#` fault) but differs in several ways
//! this port preserves:
//!
//! - Axes are addressed by **name string** (`X`, `Y`, …) discovered at config via
//!   `GETPARMSTRING`, not by an `@n` index.
//! - Parameters use `Param.axisName = value` / `Param.axisName` syntax.
//! - Homing uses the controller-native `HOME` command (no vendor `.bcx` program).
//! - Motion runs on a **task** (`taskNumber`); `stop` brackets `ABORT` with
//!   `~TASK n+1` / `~TASK n`, and enable/disable consult the task state.
//! - Moves may be **linear** (`ABSOLUTE`/`INCREMENTAL` + `LINEAR`) or single-axis
//!   (`MOVEABS`/`MOVEIINC`), selected by the controller `linear` flag.
//! - Poll is a single combined `~STATUS (...)` query returning six values.
//!
//! ## Units
//!
//! As C, `stepSize = 1 / CountsPerUnit` scales every outgoing record step
//! value into the A3200's user units and divides every readback, and values
//! are formatted to `maxDigits` places derived from it.
//!
//! ## Fault handling (owner model)
//!
//! A controller fault must be acknowledged before the axis re-enables. The C
//! tracks this with `pAxis->lastFault`, and so does this port, with one clear
//! owner per transition:
//!
//! - **`move` owns raising it:** a `#`-fault reply latches `problem = true` and,
//!   if `~STATUS TaskErrorCode` is non-zero, stores it in `last_fault`.
//! - **`poll` observes** a live `AxisFault` and latches the newest non-zero code
//!   into `last_fault` (never clearing it).
//! - **`set_closed_loop` (enable) owns clearing it:** it acknowledges the fault
//!   (`ACKNOWLEDGEALL` for codes 52/78 that need a task reset, else `FAULTACK`),
//!   then clears `last_fault` and `problem`.
//!
//! `problem` is therefore a latched state cleared only by an enable that
//! got past its TaskState query, exactly as in C (the C poller never sets
//! `motorAxisProblem` from the live fault either).
//!
//! ## Deviations from C (documented)
//!
//! - The C config builds the `ReverseMotionDirection` query string but never
//!   sends it — it reads `reverseDirec` from the previous `RAMP MODE RATE` reply
//!   (a `sprintf`-without-send bug). This port sends the query properly.
//! - Profile / trajectory moves are absent: drvMotorAsyn never calls
//!   `profileMove` / `triggerProfile`.
//! - The C maps `AXISSTATUS_Homed` to `motorAxisHomeSignal` (the home-switch
//!   field) and never sets `motorAxisHomed`; this port reproduces that mapping
//!   (`home` = Homed bit, `homed` left unset).
//! - The poller does not refresh direction; it is set only by move/home/velocity
//!   commands. This port stores the last commanded direction and reports it.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_libcom_rs::cfmt::CFloat;
use motor_common::model2::AxisLog;
use motor_common::util::{atof, atoi, atol};

use crate::axis_report::AxisMembers;
use crate::exchange::Exchange;

/// Reply read buffer size (C `BUFFER_SIZE`).
const READ_BUF: usize = 256;

/// Port EOS set at connect (C `ASCII_EOS_STR`, drvA3200Asyn.cc:890-891).
pub const INPUT_EOS: &[u8] = b"\n";
pub const OUTPUT_EOS: &[u8] = b"\n";

/// Reply status characters (C `ASCII_ACK_CHAR` / `ASCII_FAULT_CHAR`).
const ACK: u8 = b'%';
const FAULT: u8 = b'#';

/// Axis status word bits (C `AXISSTATUS_*`).
const AXISSTATUS_HOMED: u32 = 1 << 0;
const AXISSTATUS_NOT_VIRTUAL: u32 = 1 << 13;
const AXISSTATUS_MOVE_DONE: u32 = 1 << 22;

/// Drive status word bits (C `DRIVESTATUS_*`).
const DRIVESTATUS_ENABLED: u32 = 1 << 0;
const DRIVESTATUS_CW_EOT: u32 = 1 << 1;
const DRIVESTATUS_CCW_EOT: u32 = 1 << 2;

/// End-of-travel switch-level bits (C `Switch_Level.Bits`, same layout as the
/// Ensemble: bit 0 home, bit 1 CCW, bit 2 CW).
const CCW_EOT_SW_STATE: u32 = 1 << 1;
const CW_EOT_SW_STATE: u32 = 1 << 2;

/// Task state that means the motion task is idle (C `TASKSTATE_Idle`).
const TASKSTATE_IDLE: i32 = 2;

/// Fault codes that require a full `ACKNOWLEDGEALL` + task reset rather than a
/// plain `FAULTACK` (C `lastFault == 52 || lastFault == 78`).
const FAULT_ACK_ALL_CODES: [i32; 2] = [52, 78];

/// Max axes on a controller (C `A3200_MAX_AXES`).
pub const A3200_MAX_AXES: i32 = 32;

fn a3200_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// C `%.*f` with the axis `maxDigits`.
fn fmt(v: f64, digits: usize) -> String {
    format!("{:.digits$}", CFloat(v))
}

/// C's `stepSize` from a `CountsPerUnit` reply, 1 when it was refused.
fn step_size(counts_per_unit: Option<String>) -> f64 {
    counts_per_unit.map_or(1.0, |s| 1.0 / atof(&s))
}

/// C `maxDigits`: two places past the step's, at least one.
fn max_digits(step_size: f64) -> usize {
    ((-step_size.abs().log10()) as i32 + 2).max(1) as usize
}

/// An A3200 controller endpoint owning the asyn octet handle plus the motion
/// task number and linear/single-axis move mode. Shared by its axes behind a
/// mutex so command/reply pairs stay atomic (C `sendReceiveMutex`).
pub struct A3200Controller {
    io: Exchange,
    task_number: u32,
    linear: bool,
    /// C `inputBuff`: the last reply. A failed exchange leaves it, and C's
    /// callers that ignore the status read it anyway.
    input: String,
}

impl A3200Controller {
    /// Wrap a connected octet handle with the motion task number and move mode.
    pub fn new(
        handle: SyncIOHandle,
        serial_port: &str,
        motor_port: &str,
        task_number: u32,
        linear: bool,
    ) -> Self {
        Self {
            io: Exchange::new(
                handle,
                "drvA3200Asyn",
                READ_BUF,
                None,
                serial_port,
                motor_port,
            ),
            task_number,
            linear,
            input: String::new(),
        }
    }

    /// C `sendAndReceive`: fails only on I/O; a NAK or fault reply is the
    /// caller's to check. Its ACK-then-EOS re-read (drvA3200Asyn.cc:1049)
    /// never matches a reply the input EOS already stripped.
    pub fn send(&mut self, cmd: &str) -> AsynResult<()> {
        self.input = self.io.write_read(cmd)?;
        Ok(())
    }

    /// Whether the last reply is an ACK.
    pub fn ack(&self) -> bool {
        self.input.as_bytes().first() == Some(&ACK)
    }

    /// C `&inputBuff[1]`, whatever the status char.
    fn rest(&self) -> &str {
        self.input.get(1..).unwrap_or("")
    }

    /// `Param.axisName`'s value when the reply is an ACK (C
    /// `GET_PARAM_FORMAT_STRING`).
    pub fn get_param(&mut self, param: &str, axis_name: &str) -> Option<String> {
        let _ = self.send(&format!("{param}.{axis_name}"));
        self.ack().then(|| self.rest().to_owned())
    }

    /// Select the motion task and reset it (C config `~TASK` retry +
    /// `~STOPTASK`); the `~STOPTASK` reply is left for [`Self::ack`].
    pub fn init_task(&mut self) -> AsynResult<()> {
        let mut last = Ok(());
        for _ in 0..3 {
            last = self.send(&format!("~TASK {}", self.task_number));
            if last.is_ok() {
                break;
            }
        }
        last?;
        let _ = self.send("~STOPTASK");
        Ok(())
    }

    /// An axis's name string via `GETPARMSTRING` + `~GETVARIABLE`: C's
    /// `strncpy(axisName, &inputBuff[1])` whatever either returned.
    pub fn discover_axis_name(&mut self, axis: i32) -> String {
        let _ = self.send(&format!(
            "$strtask0 = GETPARMSTRING {axis}, PARAMETERID_AxisName"
        ));
        let _ = self.send("~GETVARIABLE $strtask0");
        self.rest().to_owned()
    }

    /// Final config housekeeping: init the queue and set `WAIT MODE AUTO` on both
    /// the motion task and its `+1` companion (C config tail, every status
    /// ignored).
    pub fn finalize(&mut self) {
        let _ = self.send("~INITQUEUE");
        let _ = self.send("WAIT MODE AUTO");
        let _ = self.send(&format!("~TASK {}", self.task_number + 1));
        let _ = self.send("WAIT MODE AUTO");
        let _ = self.send(&format!("~TASK {}", self.task_number));
    }

    /// A poll query, failing on I/O or a reply other than an ACK.
    fn query(&mut self, cmd: &str) -> AsynResult<String> {
        self.send(cmd)?;
        if !self.ack() {
            return Err(a3200_err(format!(
                "a3200: query '{cmd}' rejected: '{}'",
                self.input
            )));
        }
        Ok(self.rest().to_owned())
    }
}

/// One A3200 axis sharing a controller. Implements [`AsynMotor`].
pub struct A3200Axis {
    controller: Arc<Mutex<A3200Controller>>,
    /// Controller axis name string (the `X`/`Y`/… address).
    axis_name: String,
    has_encoder: bool,
    /// `HomeSetup` word; bit 0 is the home direction, updated by [`Self::home`].
    home_direction: u32,
    /// `ReverseMotionDirection` parameter.
    reverse_direc: bool,
    /// `EndOfTravelLimitSetup` word (limit-switch active levels).
    swconfig: u32,
    /// C `fabs(stepSize)`: user units per record step.
    step: f64,
    /// C `maxDigits`.
    digits: usize,
    /// Latched fault code awaiting acknowledgement (owner: move sets, poll
    /// refreshes, enable clears).
    last_fault: i32,
    /// Latched controller-error state (owner: move sets, enable clears).
    problem: bool,
    /// Last commanded position (for absolute-move direction; refreshed by poll).
    current_cmd_pos: f64,
    /// Last commanded direction; the poller does not refresh it.
    last_direction: bool,
    report: Arc<AxisMembers>,
    log: Arc<AxisLog>,
}

impl A3200Axis {
    /// Construct the named axis and probe its feedback type, home/limit setup and
    /// reverse-direction parameters, then set `RAMP MODE RATE`.
    pub fn new(controller: Arc<Mutex<A3200Controller>>, axis_name: String) -> Self {
        let (has_encoder, step_size, home_preset, home_direction, reverse_direc, swconfig) = {
            let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let has_encoder = ctrl
                .get_param("PositionFeedbackType", &axis_name)
                .is_some_and(|s| atoi(&s) > 0);
            let step_size = step_size(ctrl.get_param("CountsPerUnit", &axis_name));
            let home_preset = ctrl
                .get_param("HomeOffset", &axis_name)
                .map_or(0.0, |s| atof(&s));
            let home_direction = ctrl
                .get_param("HomeSetup", &axis_name)
                .map_or(0, |s| atol(&s) as u32 & 0x1);
            let swconfig = ctrl
                .get_param("EndOfTravelLimitSetup", &axis_name)
                .map_or(0, |s| atol(&s) as u32);
            let _ = ctrl.send(&format!("RAMP MODE RATE {axis_name}"));
            // C builds this query and never sends it, reading the RAMP MODE
            // RATE reply instead (drvA3200Asyn.cc:971-974); not copied, so
            // the config's closing ACK check (:978) reads this reply.
            let reverse_direc = ctrl
                .get_param("ReverseMotionDirection", &axis_name)
                .is_some_and(|s| atoi(&s) != 0);
            (
                has_encoder,
                step_size,
                home_preset,
                home_direction,
                reverse_direc,
                swconfig,
            )
        };

        Self {
            controller,
            report: Arc::new(AxisMembers::new(
                axis_name.clone(),
                step_size,
                max_digits(step_size),
                home_preset,
            )),
            axis_name,
            has_encoder,
            home_direction,
            reverse_direc,
            swconfig,
            step: step_size.abs(),
            digits: max_digits(step_size),
            last_fault: 0,
            problem: false,
            current_cmd_pos: 0.0,
            last_direction: false,
            log: Arc::default(),
        }
    }

    /// What C's `motorAxisReport` prints of this axis.
    pub fn report(&self) -> Arc<AxisMembers> {
        self.report.clone()
    }

    /// The user C's `PRINT(pAxis->logParam, ...)` lines go through.
    pub fn log(&self) -> Arc<AxisLog> {
        self.log.clone()
    }

    fn lock(&self) -> MutexGuard<'_, A3200Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A `motorAxisSetDouble` case C answers with only a TERROR line
    /// (drvA3200Asyn.cc:348-382).
    fn refuse(&self, what: &str) {
        self.log.print(
            TraceMask::ERROR,
            format_args!("motorAxisSetDouble: {what}\n"),
        );
    }

    /// A record step value in user units, formatted as C's `%.*f`.
    fn units(&self, v: f64) -> String {
        fmt(v * self.step, self.digits)
    }

    /// Issue the move command sequence. Returns `Ok(None)` on ACK, `Ok(Some(code))`
    /// when the move command's reply is a `#` fault (the `TaskErrorCode`
    /// C then reads from `inputBuff`), or `Err` when an exchange C checks
    /// failed or the move was not ACKed (drvA3200Asyn.cc:497-543).
    fn move_io(
        &self,
        position: f64,
        relative: bool,
        max_v: f64,
        accel: f64,
    ) -> AsynResult<Option<i32>> {
        let name = &self.axis_name;
        let mut ctrl = self.lock();
        let linear = ctrl.linear;
        let task = ctrl.task_number;
        let mode = match (relative, linear) {
            (true, true) => "INCREMENTAL",
            (false, true) => "ABSOLUTE",
            (true, false) => "MOVEIINC",
            (false, false) => "MOVEABS",
        };
        if linear {
            ctrl.send(mode)?;
        }
        if accel > 0.0 {
            let cmd = if linear {
                format!("RAMP RATE {}", self.units(accel))
            } else {
                format!("RAMP RATE {name} {}", self.units(accel))
            };
            let _ = ctrl.send(&cmd);
        }
        let cmd = if linear {
            format!(
                "LINEAR {name} {} F{}",
                self.units(position),
                self.units(max_v)
            )
        } else {
            format!(
                "{mode} {name} {} {}",
                self.units(position),
                self.units(max_v)
            )
        };
        let sent = ctrl.send(&cmd);
        if sent.is_ok() && ctrl.ack() {
            return Ok(None);
        }
        if ctrl.input.as_bytes().first() == Some(&FAULT) {
            let _ = ctrl.send(&format!("~STATUS({task}, TaskErrorCode)"));
            return Ok(Some(atoi(ctrl.rest())));
        }
        sent.and(Err(a3200_err(format!(
            "a3200: move '{cmd}' rejected: '{}'",
            ctrl.input
        ))))
    }

    fn do_move(&mut self, position: f64, relative: bool, max_v: f64, accel: f64) -> AsynResult<()> {
        let posdir = if relative {
            position >= 0.0
        } else {
            position >= self.current_cmd_pos
        };
        match self.move_io(position, relative, max_v, accel)? {
            None => {
                self.last_direction = posdir;
                Ok(())
            }
            Some(taskerr) => {
                self.problem = true;
                if taskerr != 0 {
                    self.last_fault = taskerr;
                }
                Err(a3200_err("a3200: move rejected (controller fault)"))
            }
        }
    }
}

impl AsynMotor for A3200Axis {
    /// C `A3200Poller` sets `motorAxisCommError` and `continue`s past
    /// `callCallback`, posting nothing (drvA3200Asyn.cc).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.last_direction = velocity > 0.0;
        let name = self.axis_name.clone();
        let mut ctrl = self.lock();
        let _ = ctrl.send(&format!(
            "AbortDecelRate.{name} = {}",
            self.units(acceleration)
        ));
        let _ = ctrl.send(&format!("RAMP RATE {name} {}", self.units(acceleration)));
        ctrl.send(&format!("FREERUN {name} {}", self.units(velocity)))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // Adjust home direction for the reverse-direction parameter (C posdir).
        let posdir = forward == self.reverse_direc;
        let hparam = if posdir {
            self.home_direction | 1
        } else {
            self.home_direction & !1
        };
        self.home_direction = hparam;
        // C sets motorAxisDirection to `forwards` (not posdir) for a home.
        self.last_direction = forward;

        let name = self.axis_name.clone();
        let mut ctrl = self.lock();
        if velocity > 0.0 {
            let _ = ctrl.send(&format!("HomeSpeed.{name} = {}", self.units(velocity)));
        }
        if acceleration > 0.0 {
            let _ = ctrl.send(&format!(
                "HomeRampRate.{name} = {}",
                self.units(acceleration)
            ));
        }
        let _ = ctrl.send(&format!("HomeSetup.{name} = {hparam}"));
        ctrl.send(&format!("HOME {name}"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let name = self.axis_name.clone();
        let mut ctrl = self.lock();
        let task = ctrl.task_number;
        // C returns only the last `~TASK`'s status (drvA3200Asyn.cc:680-685).
        let _ = ctrl.send(&format!("~TASK {}", task + 1));
        let _ = ctrl.send(&format!("ABORT {name}"));
        ctrl.send(&format!("~TASK {task}"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let name = self.axis_name.clone();
        let mut ctrl = self.lock();
        let sent = ctrl.send(&format!("POSOFFSET SET {name} {}", self.units(position)));
        if !ctrl.ack() {
            return Err(a3200_err(format!(
                "a3200: POSOFFSET refused: '{}'",
                ctrl.input
            )));
        }
        sent
    }

    fn set_encoder_ratio(&mut self, _user: &AsynUser, _ratio: f64) -> AsynResult<()> {
        self.refuse("A3200 does not support setting encoder ratio");
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        self.refuse("Driver does not set A3200's Low Limit");
        Ok(())
    }

    fn set_high_limit(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        self.refuse("Driver does not set A3200's High Limit");
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        self.refuse(match kind {
            PidGainKind::Proportional => "A3200 does not support setting proportional gain",
            PidGainKind::Integral => "A3200 does not support setting integral gain",
            PidGainKind::Derivative => "A3200 does not support setting derivative gain",
        });
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, closed_loop: bool) -> AsynResult<()> {
        let name = self.axis_name.clone();
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let task = ctrl.task_number;
        // C's status is TaskState's when it bails, a NAK being success
        // (drvA3200Asyn.cc:414-420).
        let sent = ctrl.send(&format!(
            "~STATUS ({task}, TaskState) ({}, TaskState)",
            task + 1
        ));
        if sent.is_err() || !ctrl.ack() {
            return sent;
        }
        // If the motion task is idle, re-init its queue before enabling.
        let taskn_state = ctrl.rest().split_whitespace().next().map_or(0, atoi);
        if taskn_state == TASKSTATE_IDLE {
            let _ = ctrl.send(&format!("~INITQUEUE {task}"));
        }

        if !closed_loop {
            let _ = ctrl.send(&format!("DISABLE {name}"));
        } else {
            if self.last_fault != 0 {
                if FAULT_ACK_ALL_CODES.contains(&self.last_fault) {
                    // These faults need a task reset before acknowledging.
                    let _ = ctrl.send(&format!("~TASK {}", task + 1));
                    let _ = ctrl.send("WAIT MODE AUTO");
                    let _ = ctrl.send("ACKNOWLEDGEALL");
                    let _ = ctrl.send(&format!("~TASK {task}"));
                    let _ = ctrl.send("~INITQUEUE");
                } else {
                    let _ = ctrl.send(&format!("FAULTACK {name}"));
                }
                // Enable is the owner that clears the latched fault/problem
                // state, whatever the acknowledgement returned.
                self.last_fault = 0;
                self.problem = false;
            }
            let _ = ctrl.send(&format!("ENABLE {name}"));
        }
        ctrl.send("WAIT MODE AUTO")
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let name = self.axis_name.clone();
        let payload = {
            let mut ctrl = self.lock();
            ctrl.query(&format!(
                "~STATUS ({name}, AxisStatus) ({name}, DriveStatus) ({name}, AxisFault) \
                 ({name}, ProgramPositionFeedback) ({name}, ProgramPositionCommand) \
                 ({name}, ProgramVelocityFeedback)"
            ))?
        };

        let mut it = payload.split_whitespace();
        let axis_status = it.next().map(atol).unwrap_or(0) as u32;
        self.report.set_axis_status(axis_status);
        let drive_status = it.next().map(atol).unwrap_or(0) as u32;
        let axis_fault = it.next().map(atoi).unwrap_or(0);
        let enc_pos = it.next().map(atof).unwrap_or(0.0);
        let cmd_pos = it.next().map(atof).unwrap_or(0.0);
        let act_vel = it.next().map(atof).unwrap_or(0.0);

        self.current_cmd_pos = cmd_pos / self.step;
        // Latch the newest non-zero fault; never clear it here (owner: enable).
        if axis_fault != 0 && axis_fault != self.last_fault {
            self.last_fault = axis_fault;
        }

        let move_active = (axis_status & AXISSTATUS_MOVE_DONE) == 0;
        let powered = (drive_status & DRIVESTATUS_ENABLED) != 0;
        let at_home = (axis_status & AXISSTATUS_HOMED) != 0;

        // Limit switches are only meaningful on a physical (non-virtual) axis.
        let (high_limit, low_limit) = if (axis_status & AXISSTATUS_NOT_VIRTUAL) != 0 {
            let cw_active = !(((drive_status & DRIVESTATUS_CW_EOT) != 0)
                ^ ((self.swconfig & CW_EOT_SW_STATE) != 0));
            let ccw_active = !(((drive_status & DRIVESTATUS_CCW_EOT) != 0)
                ^ ((self.swconfig & CCW_EOT_SW_STATE) != 0));
            if self.reverse_direc {
                (ccw_active, cw_active)
            } else {
                (cw_active, ccw_active)
            }
        } else {
            (false, false)
        };

        Ok(MotorStatus {
            position: cmd_pos / self.step,
            encoder_position: enc_pos / self.step,
            velocity: act_vel / self.step,
            done: !move_active,
            moving: move_active,
            high_limit,
            low_limit,
            // C maps AXISSTATUS_Homed to the home-switch signal, not "homed".
            home: at_home,
            homed: false,
            direction: self.last_direction,
            problem: self.problem,
            powered,
            has_encoder: self.has_encoder,
            gain_support: true,
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn max_digits_follows_the_step() {
        assert_eq!(max_digits(0.0004), 5);
        assert_eq!(fmt(1.5, 5), "1.50000");
        // C `%.*f` of a non-finite value.
        assert_eq!(fmt(f64::NAN, 5), "nan");
        assert_eq!(fmt(f64::INFINITY, 5), "inf");
        assert_eq!(step_size(None), 1.0);
    }

    /// drvA3200Asyn.cc:935-974: the axis probe in C's order, with the
    /// ReverseMotionDirection query C builds after RAMP MODE RATE sent.
    #[test]
    fn the_axis_probe_reads_reverse_direction_after_ramp_mode() {
        use motor_common::testing::LineRig;
        let (line, io) = LineRig::new("a32ProbeLine", |_| "%0".into());
        let controller = A3200Controller::new(io, "a32ProbeLine", "0", 1, true);
        line.sent();
        A3200Axis::new(Arc::new(Mutex::new(controller)), "X".into());
        assert_eq!(
            line.sent(),
            [
                "PositionFeedbackType.X",
                "CountsPerUnit.X",
                "HomeOffset.X",
                "HomeSetup.X",
                "EndOfTravelLimitSetup.X",
                "RAMP MODE RATE X",
                "ReverseMotionDirection.X",
            ]
        );
    }

    /// drvA3200Asyn.cc:348-382: the setters the A3200 refuses print their
    /// TERROR line on the axis's drvAsynMotorConfigure user.
    #[epics_rs::base::epics_test]
    async fn refused_setters_print_on_the_axis_s_log_user() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::base::server::iocsh::registry::ArgValue;
        use motor_common::model2::{Model2, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        static DRIVER: Model2 = Model2::new("a32LogTest", 0);
        let (_line, io) = LineRig::new("a32LogLine", |_| "%0".into());
        let controller = A3200Controller::new(io, "a32LogLine", "0", 1, true);
        let axis = A3200Axis::new(Arc::new(Mutex::new(controller)), "X".into());
        let axis = Arc::new(Mutex::new(axis));
        let motor: Arc<Mutex<dyn AsynMotor>> = axis.clone();
        DRIVER.set_card(0, vec![motor], 100, 1000);
        DRIVER.set_axis_logs(0, vec![axis.lock().unwrap().log()]);
        let args = vec![
            ArgValue::String("a32Log".into()),
            ArgValue::String("a32LogTest".into()),
            ArgValue::Int(0),
            ArgValue::Int(1),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&DRIVER]), args).await;

        let path = std::env::temp_dir().join(format!("a32-log-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some("a32Log"), TraceMask::ERROR);
        trace.set_trace_file(Some("a32Log"), TraceFile::File(file.clone()));
        trace.set_device_trace_file("a32Log", 0, TraceFile::File(file.clone()));
        let user = AsynUser::new(0);
        {
            let mut axis = axis.lock().unwrap();
            axis.set_encoder_ratio(&user, 1.0).unwrap();
            axis.set_low_limit(&user, -1.0).unwrap();
            axis.set_high_limit(&user, 1.0).unwrap();
            axis.set_pid_gain(&user, PidGainKind::Proportional, 1.0)
                .unwrap();
            axis.set_pid_gain(&user, PidGainKind::Integral, 1.0)
                .unwrap();
            axis.set_pid_gain(&user, PidGainKind::Derivative, 1.0)
                .unwrap();
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        // Each line follows asynPrint's timestamp.
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(
            lines,
            [
                "motorAxisSetDouble: A3200 does not support setting encoder ratio",
                "motorAxisSetDouble: Driver does not set A3200's Low Limit",
                "motorAxisSetDouble: Driver does not set A3200's High Limit",
                "motorAxisSetDouble: A3200 does not support setting proportional gain",
                "motorAxisSetDouble: A3200 does not support setting integral gain",
                "motorAxisSetDouble: A3200 does not support setting derivative gain",
            ],
            "{text}"
        );
    }

    /// Axis X of a task-1 controller, `linear` moves, on a line answering
    /// `reply`, its writes `fails` picks refused.
    fn rig_axis(
        name: &str,
        linear: bool,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
        fails: impl Fn(&str) -> bool + Send + Sync + 'static,
    ) -> (motor_common::testing::LineRig, A3200Axis) {
        let (line, io) = motor_common::testing::LineRig::failing(name, reply, fails);
        let controller = A3200Controller::new(io, name, "0", 1, linear);
        let axis = A3200Axis::new(Arc::new(Mutex::new(controller)), "X".into());
        line.sent();
        (line, axis)
    }

    /// drvA3200Asyn.cc:1001-1059: `sendAndReceive` fails only on I/O; a NAK
    /// fails only where the caller checks the ACK, the move command
    /// (:530-531) and POSOFFSET (:343-345). A NAK'd TaskState sets the comm
    /// error and returns its asyn status, success (:415-420).
    #[test]
    fn a_nak_fails_only_where_c_checks_the_ack() {
        let user = AsynUser::new(0);
        let nak = |cmd: &str| {
            if cmd.starts_with("LINEAR") || cmd.starts_with("POSOFFSET") || cmd.ends_with(".X") {
                "%0".into()
            } else {
                "!".into()
            }
        };
        let (_line, mut axis) = rig_axis("a32NakLine", true, nak, |_| false);
        axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        axis.move_velocity(&user, 0.0, 1.0, 1.0).unwrap();
        axis.home(&user, 0.0, 1.0, 1.0, true).unwrap();
        axis.stop(&user, 1.0).unwrap();
        axis.set_closed_loop(&user, true).unwrap();
        axis.set_closed_loop(&user, false).unwrap();
        let (_line, mut axis) = rig_axis("a32NakMove", true, |_| "!".into(), |_| false);
        assert!(axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).is_err());
        assert!(axis.set_position(&user, 1.0).is_err());
    }

    /// drvA3200Asyn.cc:497-531, :596-606, :629-638, :680-685: a failed
    /// exchange fails a move at its mode or move command, a home at HOME, a
    /// velocity move at FREERUN, a stop at its last `~TASK`, an enable at
    /// TaskState or WAIT MODE AUTO.
    #[test]
    fn an_io_failure_fails_only_the_checked_exchange() {
        let user = AsynUser::new(0);
        let unchecked = |cmd: &str| {
            cmd.starts_with("RAMP")
                || cmd.starts_with("Home")
                || cmd.starts_with("AbortDecel")
                || cmd.starts_with("ABORT")
                || cmd.starts_with("ENABLE")
                || cmd.starts_with("DISABLE")
                || cmd.starts_with("~INITQUEUE")
        };
        let (_line, mut axis) = rig_axis("a32IoLine", true, |_| "%2 0".into(), unchecked);
        axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        axis.move_velocity(&user, 0.0, 1.0, 1.0).unwrap();
        axis.home(&user, 0.0, 1.0, 1.0, true).unwrap();
        axis.stop(&user, 1.0).unwrap();
        axis.set_closed_loop(&user, true).unwrap();
        axis.set_closed_loop(&user, false).unwrap();

        for (name, fails) in [
            ("a32IoAbs", "ABSOLUTE"),
            ("a32IoLin", "LINEAR"),
            ("a32IoHome", "HOME"),
            ("a32IoFree", "FREERUN"),
            ("a32IoTask", "~TASK 1"),
            ("a32IoState", "~STATUS"),
            ("a32IoWait", "WAIT"),
        ] {
            let (_line, mut axis) =
                rig_axis(name, true, |_| "%0".into(), move |c| c.starts_with(fails));
            let result = match fails {
                "ABSOLUTE" | "LINEAR" => axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0),
                "HOME" => axis.home(&user, 0.0, 1.0, 1.0, true),
                "FREERUN" => axis.move_velocity(&user, 0.0, 1.0, 1.0),
                "~TASK 1" => axis.stop(&user, 1.0),
                _ => axis.set_closed_loop(&user, true),
            };
            assert!(result.is_err(), "{fails}");
        }
    }

    /// drvA3200Asyn.cc:531-543: a `#` reply to the move latches problem and
    /// the TaskErrorCode, read from `inputBuff` whatever its query's status;
    /// the next enable acknowledges it and clears both whatever FAULTACK
    /// returned (:428-451).
    #[test]
    fn a_faulted_move_latches_until_the_next_enable() {
        let user = AsynUser::new(0);
        let reply = |cmd: &str| match cmd {
            c if c.starts_with("LINEAR") => "#".into(),
            c if c.starts_with("~STATUS(1, TaskErrorCode)") => "%17".into(),
            _ => "%0 0".into(),
        };
        let fails = |cmd: &str| cmd.starts_with("FAULTACK");
        let (line, mut axis) = rig_axis("a32FaultLine", true, reply, fails);
        assert!(axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).is_err());
        assert_eq!(axis.last_fault, 17);
        assert!(axis.problem);
        line.sent();
        axis.set_closed_loop(&user, true).unwrap();
        assert!(line.sent().iter().any(|c| c == "FAULTACK X"));
        assert_eq!(axis.last_fault, 0);
        assert!(!axis.problem);

        // The TaskErrorCode query failing leaves the move's own `#` reply.
        let fails = |cmd: &str| cmd.starts_with("~STATUS(1");
        let reply = |cmd: &str| {
            if cmd.starts_with("LINEAR") {
                "#23".into()
            } else {
                "%0".into()
            }
        };
        let (_line, mut axis) = rig_axis("a32FaultStale", true, reply, fails);
        assert!(axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).is_err());
        assert_eq!(axis.last_fault, 23);
    }

    /// drvA3200Asyn.cc:1049: a bare ACK is the reply, read once.
    #[test]
    fn a_bare_ack_is_a_zero_value() {
        use motor_common::testing::LineRig;
        use std::sync::atomic::{AtomicUsize, Ordering};
        let reads = Arc::new(AtomicUsize::new(0));
        let reply = {
            let reads = reads.clone();
            move |_: &str| {
                reads.fetch_add(1, Ordering::SeqCst);
                "%".into()
            }
        };
        let (_line, io) = LineRig::new("a32BareLine", reply);
        let mut ctrl = A3200Controller::new(io, "a32BareLine", "0", 1, true);
        assert_eq!(ctrl.get_param("HomeSetup", "X").as_deref(), Some(""));
        assert_eq!(reads.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn status_bit_positions() {
        assert_eq!(AXISSTATUS_HOMED, 0x0000_0001);
        assert_eq!(AXISSTATUS_NOT_VIRTUAL, 0x0000_2000);
        assert_eq!(AXISSTATUS_MOVE_DONE, 0x0040_0000);
        assert_eq!(DRIVESTATUS_ENABLED, 0x0000_0001);
        assert_eq!(DRIVESTATUS_CW_EOT, 0x0000_0002);
        assert_eq!(DRIVESTATUS_CCW_EOT, 0x0000_0004);
    }

    #[test]
    fn limit_switch_xor_mapping() {
        // CW EOT input high, active-high configured (state bit 0) -> !(1 ^ 0) = 0.
        let drive_status = DRIVESTATUS_CW_EOT;
        let swconfig = 0;
        let cw_active =
            !(((drive_status & DRIVESTATUS_CW_EOT) != 0) ^ ((swconfig & CW_EOT_SW_STATE) != 0));
        assert!(!cw_active);
        // CW EOT input high, active-low configured (state bit set) -> !(1 ^ 1) = 1.
        let swconfig = CW_EOT_SW_STATE;
        let cw_active =
            !(((drive_status & DRIVESTATUS_CW_EOT) != 0) ^ ((swconfig & CW_EOT_SW_STATE) != 0));
        assert!(cw_active);
    }

    #[test]
    fn move_done_clear_means_moving() {
        // MoveDone set -> not moving; clear -> moving.
        let moving = |status: u32| (status & AXISSTATUS_MOVE_DONE) == 0;
        assert!(!moving(AXISSTATUS_MOVE_DONE | AXISSTATUS_HOMED));
        assert!(moving(AXISSTATUS_HOMED));
    }

    #[test]
    fn fault_ack_all_codes_membership() {
        assert!(FAULT_ACK_ALL_CODES.contains(&52));
        assert!(FAULT_ACK_ALL_CODES.contains(&78));
        assert!(!FAULT_ACK_ALL_CODES.contains(&1));
    }
}
