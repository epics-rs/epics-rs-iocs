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
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, StatusWrite,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_libcom_rs::cfmt::CFloat;
use motor_common::drv_motor_asyn::{DrvSet, function};
use motor_common::model2::AxisLog;
use motor_common::param_lib::Params;
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

/// C's delay after a move sets Done 0 "so controller has time to set
/// MoveDone false" before the next poll (drvA3200Asyn.cc:553, :645).
const MOVE_DONE_SETTLE: Duration = Duration::from_millis(10);

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
    /// C `inputBuff`: the last reply, or what a failed read delivered. A
    /// failed write leaves it, and C's callers that ignore the status read
    /// it anyway.
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
        let (reply, status) = self.io.exchange(cmd);
        if let Some(reply) = reply {
            self.input = reply;
        }
        status
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
}

/// One A3200 axis sharing a controller. Implements [`AsynMotor`].
pub struct A3200Axis {
    controller: Arc<Mutex<A3200Controller>>,
    /// C `pAxis->card`, the `A3200AsynConfig` card.
    card: i32,
    /// C `pAxis->axis`, the axis's place on the card.
    axis: i32,
    /// Controller axis name string (the `X`/`Y`/… address).
    axis_name: String,
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
    /// Last commanded position (for absolute-move direction; refreshed by poll).
    current_cmd_pos: f64,
    /// `pAxis->params`: the poller's sets, kept when a poll bails;
    /// `motorAxisProblem` latched by a faulted move and cleared by enable,
    /// `motorAxisDirection` set by the commands, never by the poller.
    params: Arc<Params>,
    /// The sets a command made, for its callback.
    status_writes: Vec<StatusWrite>,
    report: Arc<AxisMembers>,
    log: Arc<AxisLog>,
}

impl A3200Axis {
    /// Construct the named axis and probe its feedback type, home/limit setup and
    /// reverse-direction parameters, then set `RAMP MODE RATE`.
    pub fn new(
        controller: Arc<Mutex<A3200Controller>>,
        card: i32,
        axis: i32,
        axis_name: String,
    ) -> Self {
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

        let params = Arc::new(Params::default());
        // motorAxisInit's GAIN_SUPPORT (drvA3200Asyn.cc:245) and the
        // config's encoder (:940).
        let _ = params.set_integer(function::HAS_CLOSED_LOOP, 1);
        if has_encoder {
            let _ = params.set_integer(function::HAS_ENCODER, 1);
        }
        Self {
            controller,
            params,
            card,
            axis,
            report: Arc::new(AxisMembers::new(
                axis_name.clone(),
                step_size,
                max_digits(step_size),
                home_preset,
            )),
            axis_name,
            home_direction,
            reverse_direc,
            swconfig,
            step: step_size.abs(),
            digits: max_digits(step_size),
            last_fault: 0,
            current_cmd_pos: 0.0,
            status_writes: Vec::new(),
            log: Arc::default(),
        }
    }

    /// What C's `motorAxisReport` prints of this axis.
    pub fn report(&self) -> Arc<AxisMembers> {
        self.report.clone()
    }

    /// The axis' parameters, which drvAsynMotorConfigure reads and calls
    /// back through.
    pub fn params(&self) -> Arc<Params> {
        self.params.clone()
    }

    /// A command's set, which the poller also posts to the record.
    fn post(&mut self, write: StatusWrite) {
        self.params.apply(write);
        self.status_writes.push(write);
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

    /// `motorAxisDirection` and `motorAxisDone` 0, which a move, a home
    /// and a velocity move set together before their callback, so the
    /// record's next status update sees the motion started.
    fn set_started(&mut self, direction: bool) {
        self.post(StatusWrite::Direction(direction));
        self.post(StatusWrite::Done(false));
        self.params.call_callback();
    }

    fn do_move(
        &mut self,
        position: f64,
        relative: bool,
        min_v: f64,
        max_v: f64,
        accel: f64,
    ) -> AsynResult<()> {
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "Set card {}, axis {} move to {:.6}, min vel={:.6}, max_vel={:.6}, accel={:.6}\n",
                self.card,
                self.axis_name,
                CFloat(position),
                CFloat(min_v),
                CFloat(max_v),
                CFloat(accel)
            ),
        );
        let posdir = if relative {
            position >= 0.0
        } else {
            position >= self.current_cmd_pos
        };
        match self.move_io(position, relative, max_v, accel)? {
            None => {
                self.set_started(posdir);
                std::thread::sleep(MOVE_DONE_SETTLE);
                Ok(())
            }
            Some(taskerr) => {
                let _ = self.params.set_integer(function::PROBLEM, 1);
                if taskerr != 0 {
                    self.last_fault = taskerr;
                }
                Err(a3200_err("a3200: move rejected (controller fault)"))
            }
        }
    }
}

impl A3200Axis {
    /// One axis of C `A3200Poller` (drvA3200Asyn.cc:738-797), setting
    /// [`Self::params`] in its order; an `Err` is its `continue` before
    /// `callCallback`.
    fn poll_status(&mut self) -> AsynResult<()> {
        let name = &self.axis_name;
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let sent = ctrl.send(&format!(
            "~STATUS ({name}, AxisStatus) ({name}, DriveStatus) ({name}, AxisFault) \
             ({name}, ProgramPositionFeedback) ({name}, ProgramPositionCommand) \
             ({name}, ProgramVelocityFeedback)"
        ));
        if sent.is_err() || !ctrl.ack() {
            let _ = self.params.set_integer(function::COMM_ERROR, 1);
            return sent.and(Err(a3200_err(format!(
                "a3200: ~STATUS refused: '{}'",
                ctrl.input
            ))));
        }
        let payload = ctrl.rest().to_owned();
        drop(ctrl);

        // C's sscanf leaves a field it could not parse at the value the
        // previous axis left in the poller's local (:753); not copied, it
        // is 0 here.
        let mut it = payload.split_whitespace();
        let axis_status = it.next().map(atol).unwrap_or(0) as u32;
        let drive_status = it.next().map(atol).unwrap_or(0) as u32;
        let axis_fault = it.next().map(atoi).unwrap_or(0);
        let enc_pos = it.next().map(atof).unwrap_or(0.0);
        let cmd_pos = it.next().map(atof).unwrap_or(0.0);
        let act_vel = it.next().map(atof).unwrap_or(0.0);

        let params = self.params.clone();
        let bit = |f, v: bool| {
            let _ = params.set_integer(f, i32::from(v));
        };
        bit(function::COMM_ERROR, false);
        let move_active = (axis_status & AXISSTATUS_MOVE_DONE) == 0;
        bit(function::DONE, !move_active);
        bit(
            function::POWER_ON,
            (drive_status & DRIVESTATUS_ENABLED) != 0,
        );
        // C maps AXISSTATUS_Homed to the home-switch signal, not "homed".
        bit(function::HOME_SIGNAL, (axis_status & AXISSTATUS_HOMED) != 0);
        // Limit switches are only set on a physical (non-virtual) axis.
        if (axis_status & AXISSTATUS_NOT_VIRTUAL) != 0 {
            let cw_active = !(((drive_status & DRIVESTATUS_CW_EOT) != 0)
                ^ ((self.swconfig & CW_EOT_SW_STATE) != 0));
            let ccw_active = !(((drive_status & DRIVESTATUS_CCW_EOT) != 0)
                ^ ((self.swconfig & CCW_EOT_SW_STATE) != 0));
            let (high, low) = if self.reverse_direc {
                (ccw_active, cw_active)
            } else {
                (cw_active, ccw_active)
            };
            bit(function::HIGH_HARD_LIMIT, high);
            bit(function::LOW_HARD_LIMIT, low);
        }
        self.report.set_axis_status(axis_status);
        let _ = params.set_double(function::ENCODER_POSN, enc_pos / self.step);
        self.current_cmd_pos = cmd_pos / self.step;
        let _ = params.set_double(function::POSITION, self.current_cmd_pos);
        self.log.print(
            TraceMask::IO_DRIVER,
            format_args!(
                "A3200Poller: axis {name} axisStatus={axis_status:x}, position={:.6}\n",
                CFloat(self.current_cmd_pos)
            ),
        );
        // Latch the newest non-zero fault; never clear it here (owner: enable).
        if axis_fault != 0 && axis_fault != self.last_fault {
            self.log.print(
                TraceMask::ERROR,
                format_args!(
                    "A3200Poller: controller fault on axis={name} fault=0x{axis_fault:X}\n"
                ),
            );
            self.last_fault = axis_fault;
        }
        let _ = params.set_double(function::ACTUAL_VEL, act_vel / self.step);
        params.call_callback();
        Ok(())
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
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false, min_velocity, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true, min_velocity, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let name = self.axis_name.clone();
        let sent = {
            let mut ctrl = self.lock();
            let _ = ctrl.send(&format!(
                "AbortDecelRate.{name} = {}",
                self.units(acceleration)
            ));
            let _ = ctrl.send(&format!("RAMP RATE {name} {}", self.units(acceleration)));
            ctrl.send(&format!("FREERUN {name} {}", self.units(velocity)))
        };
        // C sets them whatever FREERUN returned (drvA3200Asyn.cc:639-645).
        self.set_started(velocity > 0.0);
        std::thread::sleep(MOVE_DONE_SETTLE);
        sent
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "motorAxisHome: set card {}, axis {} to home, forwards = {}\n",
                self.card,
                self.axis,
                i32::from(forward)
            ),
        );
        // Adjust home direction for the reverse-direction parameter (C posdir).
        let posdir = forward == self.reverse_direc;
        let hparam = if posdir {
            self.home_direction | 1
        } else {
            self.home_direction & !1
        };
        self.home_direction = hparam;
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
        ctrl.send(&format!("HOME {name}"))?;
        drop(ctrl);
        // `forwards`, not posdir, once HOME was sent (drvA3200Asyn.cc:605-610).
        self.set_started(forward);
        Ok(())
    }

    /// `motorAxisforceCallback` (drvA3200Asyn.cc:688-700); the status update it asks for
    /// is the poll that follows it.
    fn force_callback(&mut self, _user: &AsynUser) {
        self.params.force_callback();
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "motorAxisforceCallback: request card {}, axis {} status update\n",
                self.card, self.axis
            ),
        );
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.log.print(
            TraceMask::FLOW,
            format_args!("Abort on card {}, axis {}\n", self.card, self.axis),
        );
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
            self.post(StatusWrite::CommsError(true));
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
                self.post(StatusWrite::Problem(false));
            }
            let _ = ctrl.send(&format!("ENABLE {name}"));
        }
        // PowerOn follows the request even when ENABLE fails, and C calls
        // back whatever WAIT MODE AUTO returns (:454-466).
        self.post(StatusWrite::PowerOn(closed_loop));
        ctrl.send("WAIT MODE AUTO")
    }

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        let polled = self.poll_status();
        *status = self.params.motor_status();
        polled
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }
}

/// C `motorAxisSetDouble`/`motorAxisSetInteger` (drvA3200Asyn.cc:327-472):
/// a double it does not set prints a TERROR line and still succeeds, an
/// integer other than the closed loop fails; a success goes into the
/// parameters and is called back.
impl DrvSet for A3200Axis {
    fn set_double(&mut self, user: &AsynUser, function: i32, value: f64) -> AsynResult<()> {
        match function {
            function::POSITION => self.set_position(user, value)?,
            function::ENCODER_RATIO => self.set_encoder_ratio(user, value)?,
            function::RESOLUTION => self.refuse("A3200 does not support setting motor resolution"),
            function::LOW_LIMIT => self.set_low_limit(user, value)?,
            function::HIGH_LIMIT => self.set_high_limit(user, value)?,
            function::P_GAIN => self.set_pid_gain(user, PidGainKind::Proportional, value)?,
            function::I_GAIN => self.set_pid_gain(user, PidGainKind::Integral, value)?,
            function::D_GAIN => self.set_pid_gain(user, PidGainKind::Derivative, value)?,
            _ => self.refuse(&format!("unknown function {function}")),
        }
        let _ = self.params.set_double(function, value);
        self.params.call_callback();
        Ok(())
    }

    fn set_integer(&mut self, user: &AsynUser, function: i32, value: i32) -> AsynResult<()> {
        if function != function::CLOSED_LOOP {
            self.log.print(
                TraceMask::ERROR,
                format_args!("motorAxisSetInteger: unknown function {function}\n"),
            );
            return Err(a3200_err(format!(
                "motorAxisSetInteger: unknown function {function}"
            )));
        }
        self.set_closed_loop(user, value != 0)?;
        let _ = self.params.set_integer(function, value);
        self.params.call_callback();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::Polled;

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
        A3200Axis::new(Arc::new(Mutex::new(controller)), 0, 0, "X".into());
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
        use motor_common::model2::{Model2, Model2Axis, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        static DRIVER: Model2 = Model2::new("a32LogTest", 0);
        let (_line, io) = LineRig::new("a32LogLine", |_| "%0".into());
        let controller = A3200Controller::new(io, "a32LogLine", "0", 1, true);
        let axis = A3200Axis::new(Arc::new(Mutex::new(controller)), 0, 0, "X".into());
        let (getter, log) = (axis.params(), axis.log());
        let axis = Arc::new(Mutex::new(axis));
        DRIVER
            .start_card(
                0,
                vec![Model2Axis::new(axis.clone(), getter, log)],
                100,
                1000,
            )
            .unwrap();
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
        let axis = A3200Axis::new(Arc::new(Mutex::new(controller)), 0, 0, "X".into());
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

    /// drvA3200Asyn.cc:539-540 reads TaskErrorCode from `inputBuff`
    /// whatever the `~STATUS` status, so a reply cut by a timeout still
    /// latches its code.
    #[test]
    fn a_task_error_code_cut_by_a_timeout_is_still_latched() {
        let user = AsynUser::new(0);
        let reply = |cmd: &str| match cmd {
            c if c.starts_with("LINEAR") => "#".into(),
            c if c.starts_with("~STATUS(1, TaskErrorCode)") => "%17".into(),
            _ => "%0 0".into(),
        };
        let cut = |cmd: &str| cmd.starts_with("~STATUS(1, TaskErrorCode)");
        let (line, io) = motor_common::testing::LineRig::cut("a32CutLine", reply, cut);
        let controller = A3200Controller::new(io, "a32CutLine", "0", 1, true);
        let mut axis = A3200Axis::new(Arc::new(Mutex::new(controller)), 0, 0, "X".into());
        line.sent();
        assert!(axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).is_err());
        assert_eq!(axis.last_fault, 17);
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
        assert_eq!(axis.params.get_integer(function::PROBLEM), Ok(Some(1)));
        line.sent();
        axis.set_closed_loop(&user, true).unwrap();
        assert!(line.sent().iter().any(|c| c == "FAULTACK X"));
        assert_eq!(axis.last_fault, 0);
        assert_eq!(axis.params.get_integer(function::PROBLEM), Ok(Some(0)));

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

    /// drvA3200Asyn.cc:747-797: a failed or NAK'd `~STATUS` sets
    /// motorAxisCommError and leaves the rest for the next post; a good one
    /// clears it, and a virtual axis's limits keep their last value.
    #[test]
    fn a_failed_poll_leaves_comm_error_and_the_last_status() {
        let user = AsynUser::new(0);
        let reply = Arc::new(Mutex::new(String::new()));
        let fail = Arc::new(Mutex::new(false));
        let (_line, mut axis) = {
            let (reply, fail) = (reply.clone(), fail.clone());
            rig_axis(
                "a32PollLine",
                true,
                move |cmd: &str| {
                    if cmd.starts_with("~STATUS (X") {
                        reply.lock().unwrap().clone()
                    } else if cmd.starts_with("CountsPerUnit") {
                        "%1".into()
                    } else {
                        "%0".into()
                    }
                },
                move |cmd: &str| cmd.starts_with("~STATUS (X") && *fail.lock().unwrap(),
            )
        };
        let set = |r: &str| *reply.lock().unwrap() = r.into();
        // A real axis with its CW input active-high and the drive enabled.
        set(&format!("%{} 3 0 5 6 0", AXISSTATUS_NOT_VIRTUAL));
        let good = axis.polled(&user).unwrap();
        assert!(good.powered && good.low_limit && !good.comms_error);
        assert_eq!((good.position, good.encoder_position), (6.0, 5.0));

        *fail.lock().unwrap() = true;
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
        assert_eq!(
            left,
            MotorStatus {
                comms_error: true,
                ..good.clone()
            }
        );
        *fail.lock().unwrap() = false;
        set("!");
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
        assert!(left.comms_error);

        // A virtual axis: the limits stay where the real reading left them.
        set("%0 0 0 7 8 0");
        let virt = axis.polled(&user).unwrap();
        assert!(!virt.comms_error && !virt.powered);
        assert_eq!(
            (virt.high_limit, virt.low_limit),
            (good.high_limit, good.low_limit)
        );
        assert_eq!(virt.position, 8.0);
    }

    /// drvA3200Asyn.cc:414-466: enable and disable call back whatever
    /// their status, with the comm error a refused TaskState set, the
    /// cleared problem of an acknowledged fault, and the requested PowerOn.
    #[test]
    fn closed_loop_posts_what_it_set() {
        use epics_rs::asyn::interfaces::motor::StatusWrite;
        let user = AsynUser::new(0);
        let state = Arc::new(Mutex::new("%0 0".to_string()));
        let (_line, mut axis) = {
            let state = state.clone();
            rig_axis(
                "a32PostLine",
                true,
                move |cmd: &str| {
                    if cmd.starts_with("~STATUS (1, TaskState)") {
                        state.lock().unwrap().clone()
                    } else {
                        "%0".into()
                    }
                },
                |cmd: &str| cmd.starts_with("WAIT"),
            )
        };
        axis.last_fault = 3;
        assert!(axis.set_closed_loop(&user, true).is_err());
        assert_eq!(
            axis.take_status_writes(),
            [StatusWrite::Problem(false), StatusWrite::PowerOn(true)]
        );
        assert!(axis.set_closed_loop(&user, false).is_err());
        assert_eq!(axis.take_status_writes(), [StatusWrite::PowerOn(false)]);
        *state.lock().unwrap() = "!".into();
        axis.set_closed_loop(&user, true).unwrap();
        assert_eq!(axis.take_status_writes(), [StatusWrite::CommsError(true)]);
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

    /// drvA3200Asyn.cc:327-472: through the port, a double the A3200 does
    /// not set succeeds into its parameters, an unknown integer fails, and a
    /// move posts not-done.
    #[epics_rs::base::epics_test]
    async fn the_port_writes_go_through_the_c_set_switch() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::base::server::iocsh::registry::ArgValue;
        use motor_common::model2::{Model2, Model2Axis, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        static DRIVER: Model2 = Model2::new("a32SetTest", 0);
        let (_line, io) = LineRig::new("a32SetLine", |_| "%0".into());
        let controller = A3200Controller::new(io, "a32SetLine", "0", 1, false);
        let axis = A3200Axis::new(Arc::new(Mutex::new(controller)), 0, 0, "X".into());
        let params = axis.params();
        let axis = Arc::new(Mutex::new(axis));
        let log = axis.lock().unwrap().log();
        let axes = vec![Model2Axis::new(axis.clone(), params.clone(), log)];
        DRIVER.start_card(0, axes, 3_600_000, 3_600_000).unwrap();
        let args = vec![
            ArgValue::String("a32Set".into()),
            ArgValue::String("a32SetTest".into()),
            ArgValue::Int(0),
            ArgValue::Int(1),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&DRIVER]), args).await;
        let port = PortManager::global().find_port_handle("a32Set").unwrap();
        let reads = std::thread::spawn(move || {
            let gain = port.write_float64_blocking(function::P_GAIN as usize, 0, 0.5);
            let unknown = port.write_float64_blocking(function::SLIP as usize, 0, 2.0);
            let int = port.write_int32_blocking(function::SLIP as usize, 0, 1);
            let read = port.read_float64_blocking(function::P_GAIN as usize, 0);
            (gain.is_ok(), unknown.is_ok(), int.is_err(), read.ok())
        })
        .join()
        .unwrap();
        assert_eq!(reads, (true, true, true, Some(0.5)));
        assert_eq!(params.get_double(function::SLIP), Ok(Some(2.0)));
        assert_eq!(params.get_integer(function::HAS_CLOSED_LOOP), Ok(Some(1)));
        let user = AsynUser::new(0);
        axis.lock()
            .unwrap()
            .move_velocity(&user, 0.0, 1.0, 1.0)
            .unwrap();
        assert_eq!(params.get_integer(function::DONE), Ok(Some(0)));
        assert_eq!(params.get_integer(function::DIRECTION), Ok(Some(1)));
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

    /// drvA3200Asyn.cc:553, :645: a move and a velocity move wait for the
    /// controller to clear MoveDone, so the poll after them is not done.
    #[test]
    fn the_poll_after_a_move_is_not_done_early() {
        use std::time::Instant;
        let started: Arc<Mutex<Option<Instant>>> = Arc::default();
        let reply = {
            let started = started.clone();
            move |cmd: &str| {
                let mut started = started.lock().unwrap();
                if cmd.starts_with("LINEAR") || cmd.starts_with("FREERUN") {
                    *started = Some(Instant::now());
                }
                // MoveDone stays set until 10 ms after the move command.
                let settled = started.is_some_and(|t| t.elapsed() >= MOVE_DONE_SETTLE);
                if cmd.starts_with("~STATUS") && !settled {
                    format!("%{AXISSTATUS_MOVE_DONE} 0 0 0 0 0")
                } else if cmd.starts_with("~STATUS") {
                    "%0 0 0 0 0 0".into()
                } else {
                    "%0".into()
                }
            }
        };
        let (_line, mut axis) = rig_axis("a32SettleLine", true, reply, |_| false);
        let user = AsynUser::new(0);
        axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        assert!(!axis.polled(&user).unwrap().done);
        *started.lock().unwrap() = None;
        axis.move_velocity(&user, 0.0, 1.0, 1.0).unwrap();
        assert!(!axis.polled(&user).unwrap().done);
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

    /// drvA3200Asyn.cc:488-489, :575-576, :675, :693: move, home, abort and
    /// forceCallback print
    /// their FLOW line with the card the config gave the axis; the move
    /// names the axis, home and abort number it.
    #[epics_rs::base::epics_test]
    async fn commands_print_their_flow_line_with_the_card() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;
        use motor_common::testing::LineRig;

        let (_line, io) = LineRig::new("a32FlowLine", |_| "%0".into());
        let controller = A3200Controller::new(io, "a32FlowLine", "3", 1, true);
        let mut axis = A3200Axis::new(Arc::new(Mutex::new(controller)), 3, 2, "Y".into());
        let path = std::env::temp_dir().join(format!("a32-flow-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(None, TraceMask::FLOW);
        trace.set_trace_file(None, TraceFile::File(file));
        let user = AsynUser::new(0);
        axis.move_absolute(&user, 1.5, 0.5, 2.0, 3.0).unwrap();
        axis.home(&user, 0.0, 1.0, 1.0, false).unwrap();
        axis.stop(&user, 1.0).unwrap();
        axis.force_callback(&user);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(
            lines,
            [
                "Set card 3, axis Y move to 1.500000, min vel=0.500000, max_vel=2.000000, accel=3.000000",
                "motorAxisHome: set card 3, axis 2 to home, forwards = 0",
                "Abort on card 3, axis 2",
                "motorAxisforceCallback: request card 3, axis 2 status update",
            ],
            "{text}"
        );
    }

    /// drvA3200Asyn.cc:789-790: a poll prints its IODRIVER line with the
    /// status word and the commanded position.
    #[epics_rs::base::epics_test]
    async fn a_poll_prints_its_iodriver_line() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;

        let reply = |cmd: &str| {
            if cmd.starts_with("~STATUS (X") {
                "%513 3 0 5 6.5 0".into()
            } else if cmd.starts_with("CountsPerUnit") {
                "%1".into()
            } else {
                "%0".into()
            }
        };
        let (_line, mut axis) = rig_axis("a32IoDrvLine", true, reply, |_| false);
        let path = std::env::temp_dir().join(format!("a32-iodrv-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(None, TraceMask::IO_DRIVER);
        trace.set_trace_file(None, TraceFile::File(file));
        axis.polled(&AsynUser::new(0)).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(
            lines,
            ["A3200Poller: axis X axisStatus=201, position=6.500000"],
            "{text}"
        );
    }

    /// drvA3200Asyn.cc:792-796: a new non-zero AxisFault prints its TERROR
    /// line once, until the fault changes.
    #[epics_rs::base::epics_test]
    async fn a_new_fault_prints_once() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;

        let fault = Arc::new(Mutex::new(0));
        let reply = {
            let fault = fault.clone();
            move |cmd: &str| {
                if cmd.starts_with("~STATUS (X") {
                    format!("%0 0 {} 0 0 0", fault.lock().unwrap())
                } else {
                    "%0".into()
                }
            }
        };
        let (_line, mut axis) = rig_axis("a32FaultLine", true, reply, |_| false);
        let path = std::env::temp_dir().join(format!("a32-fault-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(None, TraceMask::ERROR);
        trace.set_trace_file(None, TraceFile::File(file));
        let user = AsynUser::new(0);
        for f in [0, 26, 26, 0, 26, 4] {
            *fault.lock().unwrap() = f;
            axis.polled(&user).unwrap();
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(
            lines,
            [
                "A3200Poller: controller fault on axis=X fault=0x1A",
                "A3200Poller: controller fault on axis=X fault=0x4",
            ],
            "{text}"
        );
    }

    /// drvA3200Asyn.cc:528-551, :601-612, :639-645: a move and a home set motorAxisDirection and
    /// motorAxisDone 0 once their checked exchange succeeded, a velocity move
    /// whatever FREERUN returned.
    #[test]
    fn commands_post_the_direction_they_set() {
        use epics_rs::asyn::interfaces::motor::StatusWrite::{Direction, Done};
        let failing = Arc::new(Mutex::new(String::new()));
        let fails = {
            let failing = failing.clone();
            move |cmd: &str| {
                let f = failing.lock().unwrap();
                !f.is_empty() && cmd.starts_with(f.as_str())
            }
        };
        let fail = |prefix: &str| *failing.lock().unwrap() = prefix.into();
        let (_line, mut axis) = rig_axis("a32DirLine", true, |_| "%0".into(), fails);
        let user = AsynUser::new(0);
        axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        assert_eq!(axis.take_status_writes(), [Direction(true), Done(false)]);
        axis.move_relative(&user, -1.0, 0.0, 1.0, 1.0).unwrap();
        assert_eq!(axis.take_status_writes(), [Direction(false), Done(false)]);
        axis.move_velocity(&user, 0.0, 1.0, 1.0).unwrap();
        assert_eq!(axis.take_status_writes(), [Direction(true), Done(false)]);
        axis.home(&user, 0.0, 1.0, 1.0, false).unwrap();
        assert_eq!(axis.take_status_writes(), [Direction(false), Done(false)]);

        fail("LINEAR");
        assert!(axis.move_absolute(&user, -1.0, 0.0, 1.0, 1.0).is_err());
        assert_eq!(axis.take_status_writes(), []);
        fail("FREERUN");
        assert!(axis.move_velocity(&user, 0.0, -1.0, 1.0).is_err());
        assert_eq!(axis.take_status_writes(), [Direction(false), Done(false)]);
        fail("HOME");
        assert!(axis.home(&user, 0.0, 1.0, 1.0, true).is_err());
        assert_eq!(axis.take_status_writes(), []);
    }
}
