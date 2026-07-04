//! Aerotech Ensemble motor controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorAerotech/aerotechApp/src/drvEnsembleAsyn.cc` (the
//! `motorAxisDrvSET_t` "motorAxis" API over `pasynOctetSyncIO`). The Ensemble
//! speaks an ASCII command language over serial or TCP; each reply begins with a
//! status char — `%` ACK, `!` NAK, `#` fault, `$` timeout — followed by the
//! payload. This port drives the per-axis [`AsynMotor`] boundary directly,
//! replacing the C controller-central poller thread with a per-axis poll.
//!
//! ## Units
//!
//! The C driver carries a `stepSize = 1 / CountsPerUnit` and multiplies every
//! outgoing position/velocity/acceleration by it (and divides every readback),
//! bridging the motor record's raw-step frame to the Ensemble's user-unit
//! frame. At the asyn-rs `AsynMotor` boundary (dial-frame EGU) that scaling
//! cancels, so it is dropped: the driver works in the controller's user units
//! natively with `MRES` = 1. Command values are formatted to [`PRECISION`]
//! decimal places (the C derived a per-axis digit count from `stepSize`; with
//! `stepSize` dropped a fixed precision is used).
//!
//! ## Not modeled (documented)
//!
//! - Profile / trajectory moves (`motorAxisProfileMove` / `TriggerProfile`
//!   return `MOTOR_AXIS_ERROR` in C too — unimplemented there.)
//! - Encoder-ratio / resolution / limit / PID setters (the C `motorAxisSetDouble`
//!   cases all just log "Ensemble does not support ..." — no-ops here.)
//! - The `CountsPerUnit` re-read on every enable (C did it only to refresh
//!   `stepSize`, which this port does not use).
//! - The async notification / forced-fast-poll machinery: replaced by periodic
//!   polling.
//!
//! ## Deviations from C (documented)
//!
//! - `motorAxisHome` in C builds the `HomeRampRate` SETPARM string but never
//!   sends it (the next `sprintf` overwrites the buffer with no intervening
//!   write) — the home acceleration is silently dropped. This port sends it,
//!   honouring the evident intent.
//! - `home` requires the vendor `HomeAsync.bcx` program to be loaded on the
//!   controller (task 5), exactly as the C driver does.
//! - The C raises `motorAxisProblem` for any non-zero fault — including a
//!   travel-limit fault — then clears it inside `DISABLE` so the user can jog
//!   off the limit switch. This port instead excludes the travel-limit fault
//!   bits from `problem` (the limit itself is reported through high/low limit),
//!   which removes the `set_closed_loop`/`poll` coupling and matches the motor
//!   record's limit semantics (a limit is not a fault that must be acknowledged).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi, atol};

/// Reply read buffer size (C `BUFFER_SIZE`).
const READ_BUF: usize = 128;

/// Command terminator appended by the driver (C `ASCII_EOS_STR`).
const TERMINATOR: &[u8] = b"\n";

/// Decimal places used when formatting outgoing numeric values.
const PRECISION: usize = 6;

/// Reply status characters (C `ASCII_*_CHAR`).
const ACK: u8 = b'%';

/// Axis status word bits (C `Axis_Status.Bits`, little-endian layout).
const AXIS_ENABLED: u32 = 1 << 0;
const HOME_CYCLE_COMPLETE: u32 = 1 << 1;
const MOVE_ACTIVE: u32 = 1 << 3;
const MOTION_CCW: u32 = 1 << 9;
const CW_LIMIT: u32 = 1 << 22;
const CCW_LIMIT: u32 = 1 << 23;
const HOME_LIMIT: u32 = 1 << 24;

/// End-of-travel switch-level bits (C `Switch_Level.Bits`).
const CCW_EOT_SW_STATE: u32 = 1 << 1;
const CW_EOT_SW_STATE: u32 = 1 << 2;

/// Ensemble parameter IDs (C `EnsembleParameterId.h`; the `(0 << 24)` high byte
/// is always zero for these).
const PARAM_REVERSE_MOTION_DIRECTION: i32 = 1;
const PARAM_POSITION_FEEDBACK_TYPE: i32 = 47;
const PARAM_END_OF_TRAVEL_LIMIT_SETUP: i32 = 61;
const PARAM_ABORT_DECEL_RATE: i32 = 73;
const PARAM_HOME_SETUP: i32 = 75;
const PARAM_HOME_SPEED: i32 = 76;
const PARAM_HOME_RAMP_RATE: i32 = 78;
const PARAM_AXIS_NAME: i32 = 140;

/// Travel-limit fault mask (C `AXISFAULTBITS_Cw/CcwEndOfTravelLimitFaultBit`).
const TRAVEL_LIMIT_FAULT_MASK: i32 = (1 << 2) | (1 << 3);

/// Max axes probed on a controller (C `ENSEMBLE_MAX_AXES`).
pub const ENSEMBLE_MAX_AXES: i32 = 10;

fn ens_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Format a numeric command value at the fixed precision.
fn fmt(v: f64) -> String {
    format!("{v:.PRECISION$}")
}

/// An Ensemble controller endpoint owning the asyn octet handle, shared by its
/// axes behind a mutex so command/reply pairs stay atomic (C
/// `sendReceiveMutex`).
pub struct EnsembleController {
    handle: SyncIOHandle,
}

impl EnsembleController {
    /// Wrap a connected octet handle.
    pub fn new(handle: SyncIOHandle) -> Self {
        Self { handle }
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

    /// Send a command that returns only an ACK. Errors on NAK/fault/timeout/empty.
    pub fn command(&self, cmd: &str) -> AsynResult<()> {
        let reply = self.write_read(cmd)?;
        match reply.as_bytes().first() {
            Some(&ACK) => Ok(()),
            _ => Err(ens_err(format!(
                "ensemble: command '{cmd}' rejected: '{reply}'"
            ))),
        }
    }

    /// Send a value query and return the payload after the ACK char. A bare ACK
    /// line (older firmware puts the value on the next line) triggers one more
    /// read (C `sendAndReceive` re-read loop).
    pub fn query(&self, cmd: &str) -> AsynResult<String> {
        let mut reply = self.write_read(cmd)?;
        let mut tries = 0;
        while reply == "%" && tries < 3 {
            reply = self.read_line()?;
            tries += 1;
        }
        match reply.as_bytes().first() {
            Some(&ACK) => Ok(reply[1..].to_string()),
            _ => Err(ens_err(format!(
                "ensemble: query '{cmd}' rejected: '{reply}'"
            ))),
        }
    }

    /// `GETPARM(@axis, param)` value.
    pub fn get_param(&self, axis: i32, param: i32) -> AsynResult<String> {
        self.query(&format!("GETPARM(@{axis}, {param})"))
    }

    /// Whether an axis exists (its `AxisName` parameter ACKs).
    pub fn axis_exists(&self, axis: i32) -> bool {
        self.get_param(axis, PARAM_AXIS_NAME).is_ok()
    }

    /// Comms check: send an (invalid) `NONE` command and accept any reply
    /// (C config `do { NONE } while` loop).
    pub fn ping(&self) -> AsynResult<()> {
        let reply = self.write_read("NONE")?;
        if reply.is_empty() {
            Err(ens_err("ensemble: no response to ping"))
        } else {
            Ok(())
        }
    }

    /// Prevent the ASCII interpreter from blocking during moves (C
    /// `WAIT MODE NOWAIT`, sent at config and on every enable).
    pub fn wait_mode_nowait(&self) -> AsynResult<()> {
        self.command("WAIT MODE NOWAIT")
    }
}

/// One Ensemble axis sharing a controller. Implements [`AsynMotor`].
pub struct EnsembleAxis {
    controller: Arc<Mutex<EnsembleController>>,
    /// Controller axis number (the `@n` address).
    axis: i32,
    has_encoder: bool,
    /// `HomeSetup` word; bit 0 is the home direction, updated by [`Self::home`].
    home_direction: u32,
    /// `ReverseMotionDirection` parameter.
    reverse_direc: bool,
    /// `EndOfTravelLimitSetup` word (limit-switch active levels).
    swconfig: u32,
}

impl EnsembleAxis {
    /// Construct axis `axis` and probe its feedback type, home/limit setup and
    /// reverse-direction parameters, then set `RAMP MODE RATE`.
    pub fn new(controller: Arc<Mutex<EnsembleController>>, axis: i32) -> AsynResult<Self> {
        let (has_encoder, home_direction, reverse_direc, swconfig) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let has_encoder = ctrl
                .get_param(axis, PARAM_POSITION_FEEDBACK_TYPE)
                .map(|s| atoi(&s) > 0)
                .unwrap_or(false);
            let home_direction = ctrl
                .get_param(axis, PARAM_HOME_SETUP)
                .map(|s| atol(&s) as u32)
                .unwrap_or(0);
            let swconfig = ctrl
                .get_param(axis, PARAM_END_OF_TRAVEL_LIMIT_SETUP)
                .map(|s| atol(&s) as u32)
                .unwrap_or(0);
            let reverse_direc = ctrl
                .get_param(axis, PARAM_REVERSE_MOTION_DIRECTION)
                .map(|s| atoi(&s) != 0)
                .unwrap_or(false);
            ctrl.command(&format!("RAMP MODE @{axis} RATE"))?;
            (has_encoder, home_direction, reverse_direc, swconfig)
        };

        Ok(Self {
            controller,
            axis,
            has_encoder,
            home_direction,
            reverse_direc,
            swconfig,
        })
    }

    fn lock(&self) -> MutexGuard<'_, EnsembleController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn do_move(&mut self, position: f64, relative: bool, max_v: f64, accel: f64) -> AsynResult<()> {
        let axis = self.axis;
        let mode = if relative { "INC" } else { "ABS" };
        {
            let ctrl = self.lock();
            ctrl.command(mode)?;
            if accel > 0.0 {
                ctrl.command(&format!("RAMP RATE {}", fmt(accel)))?;
            }
            ctrl.command(&format!("LINEAR @{axis} {} F{}", fmt(position), fmt(max_v)))?;
        }
        Ok(())
    }
}

impl AsynMotor for EnsembleAxis {
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
        let axis = self.axis;
        let ctrl = self.lock();
        ctrl.command(&format!(
            "SETPARM @{axis}, {PARAM_ABORT_DECEL_RATE}, {}",
            fmt(acceleration)
        ))?;
        ctrl.command(&format!("RAMP RATE @{axis} {}", fmt(acceleration)))?;
        ctrl.command(&format!("FREERUN @{axis} {}", fmt(velocity)))?;
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
        let axis = self.axis;
        // Adjust home direction for the reverse-direction parameter (C posdir).
        let posdir = forward == self.reverse_direc;
        let hparam = if posdir {
            self.home_direction | 1
        } else {
            self.home_direction & !1
        };
        self.home_direction = hparam;

        let ctrl = self.lock();
        if velocity > 0.0 {
            ctrl.command(&format!(
                "SETPARM @{axis}, {PARAM_HOME_SPEED}, {}",
                fmt(velocity)
            ))?;
        }
        if acceleration > 0.0 {
            // C builds this SETPARM but never sends it (buffer overwritten); we
            // send it to honour the intent.
            ctrl.command(&format!(
                "SETPARM @{axis}, {PARAM_HOME_RAMP_RATE}, {}",
                fmt(acceleration)
            ))?;
        }
        ctrl.command(&format!("SETPARM @{axis}, {PARAM_HOME_SETUP}, {hparam}"))?;
        // HomeAsync.bcx protocol: IGLOBAL(32)=1, IGLOBAL(33)=axis, run task 5.
        ctrl.command("IGLOBAL(32) = 1")?;
        ctrl.command(&format!("IGLOBAL(33) = {axis}"))?;
        ctrl.command("PROGRAM RUN 5, \"HomeAsync.bcx\"")?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let axis = self.axis;
        self.lock().command(&format!("ABORT @{axis}"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let axis = self.axis;
        self.lock()
            .command(&format!("POSOFFSET SET @{axis}, {}", fmt(position)))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, closed_loop: bool) -> AsynResult<()> {
        let axis = self.axis;
        let ctrl = self.lock();
        if closed_loop {
            // Acknowledge any pending fault, then enable.
            let fault = ctrl
                .query(&format!("AXISFAULT @{axis}"))
                .map(|s| atoi(&s))
                .unwrap_or(0);
            if fault != 0 {
                ctrl.command(&format!("FAULTACK @{axis}"))?;
            }
            ctrl.command(&format!("ENABLE @{axis}"))?;
        } else {
            ctrl.command(&format!("DISABLE @{axis}"))?;
        }
        ctrl.wait_mode_nowait()?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let axis = self.axis;
        let (axis_status, plane_active, enc_pos, cmd_pos, fault, act_vel) = {
            let ctrl = self.lock();
            let axis_status = atol(&ctrl.query(&format!("AXISSTATUS(@{axis})"))?) as u32;
            let plane_active = (atoi(&ctrl.query("PLANESTATUS(0)")?) & 0x01) != 0;
            let enc_pos = atof(&ctrl.query(&format!("PFBKPROG(@{axis})"))?);
            let cmd_pos = atof(&ctrl.query(&format!("PCMDPROG(@{axis})"))?);
            let fault = atoi(&ctrl.query(&format!("AXISFAULT(@{axis})"))?);
            let act_vel = atof(&ctrl.query(&format!("VFBK(@{axis})"))?);
            (axis_status, plane_active, enc_pos, cmd_pos, fault, act_vel)
        };

        let move_active = (axis_status & MOVE_ACTIVE) != 0 || plane_active;
        let done = !move_active;
        let powered = (axis_status & AXIS_ENABLED) != 0;
        let homed = (axis_status & HOME_CYCLE_COMPLETE) != 0;
        let at_home = (axis_status & HOME_LIMIT) != 0;
        let motion_ccw = (axis_status & MOTION_CCW) != 0;
        let direction = if self.reverse_direc {
            motion_ccw
        } else {
            !motion_ccw
        };

        // Limit switches: XOR the raw limit bit against its configured active
        // level, then map CW/CCW to high/low honouring the reverse-direction flag.
        let cw_sw_active =
            !(((axis_status & CW_LIMIT) != 0) ^ ((self.swconfig & CW_EOT_SW_STATE) != 0));
        let ccw_sw_active =
            !(((axis_status & CCW_LIMIT) != 0) ^ ((self.swconfig & CCW_EOT_SW_STATE) != 0));
        let (high_limit, low_limit) = if self.reverse_direc {
            (ccw_sw_active, cw_sw_active)
        } else {
            (cw_sw_active, ccw_sw_active)
        };

        Ok(MotorStatus {
            position: cmd_pos,
            encoder_position: enc_pos,
            velocity: act_vel,
            done,
            moving: move_active,
            high_limit,
            low_limit,
            home: at_home,
            homed,
            direction,
            // Travel-limit faults are reported via high/low limit, not problem.
            problem: (fault & !TRAVEL_LIMIT_FAULT_MASK) != 0,
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
        assert_eq!(EnsembleController::framed("ENABLE @0"), b"ENABLE @0\n");
    }

    #[test]
    fn fmt_uses_fixed_precision() {
        assert_eq!(fmt(1.5), "1.500000");
        assert_eq!(fmt(-100.0), "-100.000000");
    }

    #[test]
    fn axis_status_bit_positions() {
        assert_eq!(AXIS_ENABLED, 0x0000_0001);
        assert_eq!(HOME_CYCLE_COMPLETE, 0x0000_0002);
        assert_eq!(MOVE_ACTIVE, 0x0000_0008);
        assert_eq!(MOTION_CCW, 0x0000_0200);
        assert_eq!(CW_LIMIT, 0x0040_0000);
        assert_eq!(CCW_LIMIT, 0x0080_0000);
        assert_eq!(HOME_LIMIT, 0x0100_0000);
    }

    #[test]
    fn limit_switch_xor_mapping() {
        // Raw CW limit set, active-high configured (state bit 0) -> !(1 ^ 0) = 0.
        let status = CW_LIMIT;
        let swconfig = 0;
        let cw_active = !(((status & CW_LIMIT) != 0) ^ ((swconfig & CW_EOT_SW_STATE) != 0));
        assert!(!cw_active);
        // Raw CW limit set, active-low configured (state bit set) -> !(1 ^ 1) = 1.
        let swconfig = CW_EOT_SW_STATE;
        let cw_active = !(((status & CW_LIMIT) != 0) ^ ((swconfig & CW_EOT_SW_STATE) != 0));
        assert!(cw_active);
    }

    #[test]
    fn travel_limit_fault_mask_value() {
        assert_eq!(TRAVEL_LIMIT_FAULT_MASK, 0x0C);
    }
}
