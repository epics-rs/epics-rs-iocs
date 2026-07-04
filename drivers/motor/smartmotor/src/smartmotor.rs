//! Animatics SmartMotor integrated servo controller driver (serial ASCII).
//!
//! Ported from `motorSmartMotor/smartMotorApp/src/drvSmartMotor.cc` +
//! `devSmartMotor.cc`. A SmartMotor is an integrated servo + drive; commands
//! use the Animatics language (`P=`, `D=`, `V=`, `A=`, `G`, `S`, `RP`, `RBt`,
//! …) and are terminated by a newline (the driver owns output framing; the
//! startup script sets only the input EOS `\r`). Set commands produce no reply;
//! status is read back with `R*` query commands.
//!
//! ## Single-motor mode only
//!
//! The C driver supports two wire modes, auto-detected at init by probing
//! `RBe`:
//!
//! - **Single motor (no echo):** commands are sent verbatim and status queries
//!   reply terminated by `\r`. This is the mode implemented here.
//! - **Daisy chain (echo on):** each command is prefixed with a binary address
//!   byte (`128 + axis`) and the motor echoes the command terminated by `\n`
//!   *before* the `\r`-terminated response. The C driver reads these two frames
//!   by switching the port input EOS between `\n` (echo) and `\r` (response) on
//!   every transaction.
//!
//! The asyn-rs [`SyncIOHandle`] reads to the port's fixed input EOS and exposes
//! no per-read EOS override, so the daisy-chain framing cannot be reproduced
//! faithfully. [`SmartMotorController::new`] therefore detects echo mode and
//! returns an error rather than driving it incorrectly. Supporting daisy chain
//! needs an input-EOS control API on `SyncIOHandle` (upstream `epics-rs`).
//!
//! ## Units
//!
//! The SmartMotor works natively in encoder counts (`RP` returns counts, `P=`
//! takes counts) with no resolution scaling, so the asyn-rs motor boundary is
//! counts: positions pass through with `NINT` rounding, the record's `MRES` is
//! 1, and its `EGU` is counts.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `R*` reads run inside
//!   [`poll`](AsynMotor::poll).
//! - `HOME_FOR`/`HOME_REV` and the PID-gain commands are unsupported in C
//!   (`build_trans` sends nothing and returns `ERROR`); [`home`](AsynMotor::home)
//!   returns an error and [`set_pid_gain`](AsynMotor::set_pid_gain) is a no-op.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 20;

/// Command terminator; the driver owns output framing (C output EOS `\n`).
const TERMINATOR: &[u8] = b"\n";

/// Minimum acceleration the controller accepts (C forces `A >= 2`).
const MIN_ACCEL: i32 = 2;

/// Round an acceleration to the controller value, clamped to the minimum the
/// SmartMotor accepts (C overrides `A <= 1` to `2`).
fn clamped_accel(acceleration: f64) -> i32 {
    nint(acceleration).max(MIN_ACCEL)
}

fn smartmotor_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle for a single SmartMotor.
pub struct SmartMotorController {
    handle: SyncIOHandle,
}

impl SmartMotorController {
    /// Connect and probe a single SmartMotor (C `motor_init`, single-motor path):
    /// send `RBe` and require a bare `0`/`1` reply. If the command is echoed back
    /// the controller is in daisy-chain echo mode, which this port cannot frame
    /// (see the module docs) and is rejected. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let ctrl = Self { handle };
        let reply = ctrl.query("RBe")?;
        if reply.starts_with("RBe") || reply.contains('\n') {
            return Err(smartmotor_err(
                "SmartMotor: controller is in daisy-chain echo mode, which is \
                 unsupported (needs per-read input-EOS control on SyncIOHandle)",
            ));
        }
        if reply != "0" && reply != "1" {
            return Err(smartmotor_err(format!(
                "SmartMotor: no valid response to RBe probe (got '{reply}')"
            )));
        }
        Ok(ctrl)
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Send a command with no reply (moves and set commands; the SmartMotor does
    /// not echo or answer these in single-motor mode).
    fn send_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Send a query command and return its `\r`-terminated reply (trimmed).
    fn query(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_matches(['\r', '\n', '\0', ' ']).to_string())
    }
}

/// A single SmartMotor axis. Implements [`AsynMotor`].
pub struct SmartMotorAxis {
    controller: Arc<Mutex<SmartMotorController>>,
    prev_position: i32,
    last_status: MotorStatus,
}

impl SmartMotorAxis {
    /// Construct the (single) axis for a SmartMotor controller.
    pub fn new(controller: Arc<Mutex<SmartMotorController>>) -> Self {
        Self {
            controller,
            prev_position: 0,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
        }
    }

    fn lock(&self) -> MutexGuard<'_, SmartMotorController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Program speed/acceleration before a move (C sends `V=`/`A=` as separate
    /// transactions ahead of the move command). Non-positive values are skipped;
    /// acceleration is clamped to the controller minimum.
    fn program_speeds(
        ctrl: &SmartMotorController,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if velocity > 0.0 {
            ctrl.send_only(&format!("V={}", nint(velocity)))?;
        }
        if acceleration > 0.0 {
            ctrl.send_only(&format!("A={}", clamped_accel(acceleration)))?;
        }
        Ok(())
    }
}

impl AsynMotor for SmartMotorAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        SmartMotorAxis::program_speeds(&ctrl, velocity, acceleration)?;
        ctrl.send_only(&format!("P={}", nint(position)))?;
        ctrl.send_only("G")?;
        Ok(())
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        SmartMotorAxis::program_speeds(&ctrl, velocity, acceleration)?;
        ctrl.send_only(&format!("D={}", nint(distance)))?;
        ctrl.send_only("G")?;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C JOG: "MV\rV=<v>\rG" — velocity mode, set signed velocity, go.
        let ctrl = self.lock();
        ctrl.send_only(&format!("MV\rV={}\rG", nint(velocity)))?;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C: Animatics SmartMotors do not use home positions (build_trans ERROR).
        Err(smartmotor_err("SmartMotor: homing is not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.send_only("S")?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.send_only(&format!("O={}", nint(position)))?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        if enable {
            // C ENABLE_TORQUE: position mode, hold at the current actual position.
            ctrl.send_only("MP\ra=@P\rP=a\rG")?;
        } else {
            // C DISABL_TORQUE: de-energize.
            ctrl.send_only("OFF")?;
        }
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // C: SET_PGAIN/IGAIN/DGAIN send nothing and return ERROR — no-op here.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();

        // Trajectory-busy flag: RBt == 0 means the move is done.
        let done = atoi(&ctrl.query("RBt")?) == 0;

        let position = nint(atof(&ctrl.query("RP")?));

        let plus_ls = atoi(&ctrl.query("RBp")?) != 0;
        let minus_ls = atoi(&ctrl.query("RBm")?) != 0;

        // Latched right/left travel-limit flags: clear them if set (C Zr/Zl).
        if atoi(&ctrl.query("RBr")?) != 0 {
            ctrl.send_only("Zr")?;
        }
        if atoi(&ctrl.query("RBl")?) != 0 {
            ctrl.send_only("Zl")?;
        }

        // RBo != 0 means "not on commanded position" (C clears EA_POSITION).
        let on_position = atoi(&ctrl.query("RBo")?) == 0;

        let velocity_raw = atoi(&ctrl.query("RV")?);
        drop(ctrl);

        let direction = position >= self.prev_position;
        self.prev_position = position;

        let velocity = if direction {
            velocity_raw as f64
        } else {
            -(velocity_raw as f64)
        };

        let status = MotorStatus {
            position: position as f64,
            encoder_position: position as f64,
            velocity,
            done,
            moving: !done,
            high_limit: plus_ls,
            low_limit: minus_ls,
            direction,
            powered: on_position,
            has_encoder: true,
            gain_support: true,
            ..MotorStatus::default()
        };
        self.last_status = status.clone();
        Ok(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framed_appends_newline() {
        assert_eq!(SmartMotorController::framed("P=100"), b"P=100\n");
        assert_eq!(SmartMotorController::framed("G"), b"G\n");
    }

    #[test]
    fn accel_clamps_to_minimum() {
        assert_eq!(clamped_accel(1.0), 2); // C overrides A <= 1 to 2
        assert_eq!(clamped_accel(0.0), 2);
        assert_eq!(clamped_accel(-5.0), 2);
        assert_eq!(clamped_accel(2.4), 2); // NINT rounds down
        assert_eq!(clamped_accel(2.5), 3); // NINT rounds up
        assert_eq!(clamped_accel(100.0), 100);
    }
}
