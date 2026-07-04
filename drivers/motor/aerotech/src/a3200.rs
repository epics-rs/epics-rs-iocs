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
//! As in the Ensemble port, the C `stepSize = 1 / CountsPerUnit` scaling cancels
//! at the asyn-rs [`AsynMotor`] (dial-frame EGU) boundary and is dropped: the
//! driver works in the controller's user units with `MRES` = 1. Command values
//! are formatted at a fixed [`PRECISION`] (the C derived a per-axis digit count
//! from `stepSize`, which this port does not carry).
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
//! `problem` is therefore a latched state cleared only by a successful enable,
//! exactly as in C (the C poller never sets `motorAxisProblem` from the live
//! fault either).
//!
//! ## Deviations from C (documented)
//!
//! - The C config builds the `ReverseMotionDirection` query string but never
//!   sends it — it reads `reverseDirec` from the previous `RAMP MODE RATE` reply
//!   (a `sprintf`-without-send bug). This port sends the query properly.
//! - Profile / trajectory moves (`motorAxisProfileMove` / `TriggerProfile`)
//!   return `MOTOR_AXIS_ERROR` in C — unimplemented there, so not modeled here.
//! - The C maps `AXISSTATUS_Homed` to `motorAxisHomeSignal` (the home-switch
//!   field) and never sets `motorAxisHomed`; this port reproduces that mapping
//!   (`home` = Homed bit, `homed` left unset).
//! - The poller does not refresh direction; it is set only by move/home/velocity
//!   commands. This port stores the last commanded direction and reports it.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi, atol};

/// Reply read buffer size (C `BUFFER_SIZE`).
const READ_BUF: usize = 256;

/// Command terminator appended by the driver (C `ASCII_EOS_STR`).
const TERMINATOR: &[u8] = b"\n";

/// Decimal places used when formatting outgoing numeric values.
const PRECISION: usize = 6;

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

/// Format a numeric command value at the fixed precision.
fn fmt(v: f64) -> String {
    format!("{v:.PRECISION$}")
}

/// An A3200 controller endpoint owning the asyn octet handle plus the motion
/// task number and linear/single-axis move mode. Shared by its axes behind a
/// mutex so command/reply pairs stay atomic (C `sendReceiveMutex`).
pub struct A3200Controller {
    handle: SyncIOHandle,
    task_number: u32,
    linear: bool,
}

impl A3200Controller {
    /// Wrap a connected octet handle with the motion task number and move mode.
    pub fn new(handle: SyncIOHandle, task_number: u32, linear: bool) -> Self {
        Self {
            handle,
            task_number,
            linear,
        }
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    fn read_line(&self) -> AsynResult<String> {
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let s = String::from_utf8_lossy(&raw);
        Ok(s.trim_end_matches(['\r', '\n', '\0']).to_string())
    }

    /// Write a command and read one reply line.
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        self.read_line()
    }

    /// Send a command that returns only an ACK. Errors on fault/empty.
    pub fn command(&self, cmd: &str) -> AsynResult<()> {
        let reply = self.write_read(cmd)?;
        match reply.as_bytes().first() {
            Some(&ACK) => Ok(()),
            _ => Err(a3200_err(format!(
                "a3200: command '{cmd}' rejected: '{reply}'"
            ))),
        }
    }

    /// Send a value query and return the payload after the ACK char. A bare ACK
    /// line (value on the next line) triggers one more read.
    pub fn query(&self, cmd: &str) -> AsynResult<String> {
        let mut reply = self.write_read(cmd)?;
        let mut tries = 0;
        while reply == "%" && tries < 3 {
            reply = self.read_line()?;
            tries += 1;
        }
        match reply.as_bytes().first() {
            Some(&ACK) => Ok(reply[1..].to_string()),
            _ => Err(a3200_err(format!(
                "a3200: query '{cmd}' rejected: '{reply}'"
            ))),
        }
    }

    /// `Param.axisName` parameter value (C `GET_PARAM_FORMAT_STRING`).
    pub fn get_param(&self, param: &str, axis_name: &str) -> AsynResult<String> {
        self.query(&format!("{param}.{axis_name}"))
    }

    /// Select the motion task and reset it (C config `~TASK` retry + `~STOPTASK`).
    pub fn init_task(&self) -> AsynResult<()> {
        let mut last = Err(a3200_err("a3200: no response to ~TASK"));
        for _ in 0..3 {
            last = self.command(&format!("~TASK {}", self.task_number));
            if last.is_ok() {
                break;
            }
        }
        last?;
        // C ignores the ~STOPTASK reply.
        let _ = self.command("~STOPTASK");
        Ok(())
    }

    /// Discover an axis's name string via `GETPARMSTRING` + `~GETVARIABLE`.
    pub fn discover_axis_name(&self, axis: i32) -> AsynResult<String> {
        self.command(&format!(
            "$strtask0 = GETPARMSTRING {axis}, PARAMETERID_AxisName"
        ))?;
        let name = self.query("~GETVARIABLE $strtask0")?;
        Ok(name.trim().to_string())
    }

    /// Final config housekeeping: init the queue and set `WAIT MODE AUTO` on both
    /// the motion task and its `+1` companion (C config tail).
    pub fn finalize(&self) -> AsynResult<()> {
        self.command("~INITQUEUE")?;
        self.command("WAIT MODE AUTO")?;
        self.command(&format!("~TASK {}", self.task_number + 1))?;
        self.command("WAIT MODE AUTO")?;
        self.command(&format!("~TASK {}", self.task_number))?;
        Ok(())
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
    /// Latched fault code awaiting acknowledgement (owner: move sets, poll
    /// refreshes, enable clears).
    last_fault: i32,
    /// Latched controller-error state (owner: move sets, enable clears).
    problem: bool,
    /// Last commanded position (for absolute-move direction; refreshed by poll).
    current_cmd_pos: f64,
    /// Last commanded direction; the poller does not refresh it.
    last_direction: bool,
}

impl A3200Axis {
    /// Construct the named axis and probe its feedback type, home/limit setup and
    /// reverse-direction parameters, then set `RAMP MODE RATE`.
    pub fn new(controller: Arc<Mutex<A3200Controller>>, axis_name: String) -> AsynResult<Self> {
        let (has_encoder, home_direction, reverse_direc, swconfig) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let has_encoder = ctrl
                .get_param("PositionFeedbackType", &axis_name)
                .map(|s| atoi(&s) > 0)
                .unwrap_or(false);
            let home_direction = ctrl
                .get_param("HomeSetup", &axis_name)
                .map(|s| atol(&s) as u32 & 0x1)
                .unwrap_or(0);
            let swconfig = ctrl
                .get_param("EndOfTravelLimitSetup", &axis_name)
                .map(|s| atol(&s) as u32)
                .unwrap_or(0);
            let reverse_direc = ctrl
                .get_param("ReverseMotionDirection", &axis_name)
                .map(|s| atoi(&s) != 0)
                .unwrap_or(false);
            ctrl.command(&format!("RAMP MODE RATE {axis_name}"))?;
            (has_encoder, home_direction, reverse_direc, swconfig)
        };

        Ok(Self {
            controller,
            axis_name,
            has_encoder,
            home_direction,
            reverse_direc,
            swconfig,
            last_fault: 0,
            problem: false,
            current_cmd_pos: 0.0,
            last_direction: false,
        })
    }

    fn lock(&self) -> MutexGuard<'_, A3200Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Issue the move command sequence. Returns `Ok(None)` on ACK, `Ok(Some(code))`
    /// when the controller replies with a `#` fault (payload is the queried
    /// `TaskErrorCode`), or `Err` on a comms/other-reply failure.
    fn move_io(
        &self,
        position: f64,
        relative: bool,
        max_v: f64,
        accel: f64,
    ) -> AsynResult<Option<i32>> {
        let name = &self.axis_name;
        let ctrl = self.lock();
        let linear = ctrl.linear;
        let task = ctrl.task_number;
        let mode = match (relative, linear) {
            (true, true) => "INCREMENTAL",
            (false, true) => "ABSOLUTE",
            (true, false) => "MOVEIINC",
            (false, false) => "MOVEABS",
        };
        if linear {
            ctrl.command(mode)?;
        }
        if accel > 0.0 {
            let cmd = if linear {
                format!("RAMP RATE {}", fmt(accel))
            } else {
                format!("RAMP RATE {name} {}", fmt(accel))
            };
            ctrl.command(&cmd)?;
        }
        let cmd = if linear {
            format!("LINEAR {name} {} F{}", fmt(position), fmt(max_v))
        } else {
            format!("{mode} {name} {} {}", fmt(position), fmt(max_v))
        };
        let reply = ctrl.write_read(&cmd)?;
        match reply.as_bytes().first() {
            Some(&ACK) => Ok(None),
            Some(&FAULT) => {
                let taskerr = ctrl
                    .query(&format!("~STATUS({task}, TaskErrorCode)"))
                    .map(|s| atoi(&s))
                    .unwrap_or(0);
                Ok(Some(taskerr))
            }
            _ => Err(a3200_err(format!(
                "a3200: move '{cmd}' rejected: '{reply}'"
            ))),
        }
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
        let ctrl = self.lock();
        ctrl.command(&format!("AbortDecelRate.{name} = {}", fmt(acceleration)))?;
        ctrl.command(&format!("RAMP RATE {name} {}", fmt(acceleration)))?;
        ctrl.command(&format!("FREERUN {name} {}", fmt(velocity)))?;
        Ok(())
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
        let ctrl = self.lock();
        if velocity > 0.0 {
            ctrl.command(&format!("HomeSpeed.{name} = {}", fmt(velocity)))?;
        }
        if acceleration > 0.0 {
            ctrl.command(&format!("HomeRampRate.{name} = {}", fmt(acceleration)))?;
        }
        ctrl.command(&format!("HomeSetup.{name} = {hparam}"))?;
        ctrl.command(&format!("HOME {name}"))?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let name = self.axis_name.clone();
        let ctrl = self.lock();
        let task = ctrl.task_number;
        ctrl.command(&format!("~TASK {}", task + 1))?;
        ctrl.command(&format!("ABORT {name}"))?;
        ctrl.command(&format!("~TASK {task}"))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let name = self.axis_name.clone();
        self.lock()
            .command(&format!("POSOFFSET SET {name} {}", fmt(position)))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, closed_loop: bool) -> AsynResult<()> {
        let name = self.axis_name.clone();
        let last_fault = self.last_fault;
        {
            let ctrl = self.lock();
            let task = ctrl.task_number;
            // If the motion task is idle, re-init its queue before enabling.
            let states = ctrl.query(&format!(
                "~STATUS ({task}, TaskState) ({}, TaskState)",
                task + 1
            ))?;
            let taskn_state = states.split_whitespace().next().map(atoi).unwrap_or(0);
            if taskn_state == TASKSTATE_IDLE {
                ctrl.command(&format!("~INITQUEUE {task}"))?;
            }

            if !closed_loop {
                ctrl.command(&format!("DISABLE {name}"))?;
            } else {
                if last_fault != 0 {
                    if FAULT_ACK_ALL_CODES.contains(&last_fault) {
                        // These faults need a task reset before acknowledging.
                        ctrl.command(&format!("~TASK {}", task + 1))?;
                        ctrl.command("WAIT MODE AUTO")?;
                        ctrl.command("ACKNOWLEDGEALL")?;
                        ctrl.command(&format!("~TASK {task}"))?;
                        ctrl.command("~INITQUEUE")?;
                    } else {
                        ctrl.command(&format!("FAULTACK {name}"))?;
                    }
                }
                ctrl.command(&format!("ENABLE {name}"))?;
            }
            ctrl.command("WAIT MODE AUTO")?;
        }
        // Enable is the owner that clears the latched fault/problem state.
        if closed_loop && last_fault != 0 {
            self.last_fault = 0;
            self.problem = false;
        }
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let name = self.axis_name.clone();
        let payload = {
            let ctrl = self.lock();
            ctrl.query(&format!(
                "~STATUS ({name}, AxisStatus) ({name}, DriveStatus) ({name}, AxisFault) \
                 ({name}, ProgramPositionFeedback) ({name}, ProgramPositionCommand) \
                 ({name}, ProgramVelocityFeedback)"
            ))?
        };

        let mut it = payload.split_whitespace();
        let axis_status = it.next().map(atol).unwrap_or(0) as u32;
        let drive_status = it.next().map(atol).unwrap_or(0) as u32;
        let axis_fault = it.next().map(atoi).unwrap_or(0);
        let enc_pos = it.next().map(atof).unwrap_or(0.0);
        let cmd_pos = it.next().map(atof).unwrap_or(0.0);
        let act_vel = it.next().map(atof).unwrap_or(0.0);

        self.current_cmd_pos = cmd_pos;
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
            position: cmd_pos,
            encoder_position: enc_pos,
            velocity: act_vel,
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
    fn framed_appends_terminator() {
        assert_eq!(A3200Controller::framed("ENABLE X"), b"ENABLE X\n");
    }

    #[test]
    fn fmt_uses_fixed_precision() {
        assert_eq!(fmt(1.5), "1.500000");
        assert_eq!(fmt(-100.0), "-100.000000");
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
