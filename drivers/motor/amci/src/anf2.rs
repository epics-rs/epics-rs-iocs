//! AMCI ANF2 stepper motor controller driver — a model-3 asyn driver that
//! talks Modbus/TCP registers (no serial ASCII protocol). Ported from
//! `motorAMCI/amciApp/src/ANF2Driver.cpp`.
//!
//! ## Register map
//!
//! Two Modbus asyn ports carry all I/O: `AXIS_REG_OFFSET` (10) 16-bit input
//! registers per axis (`STATUS_1`/`STATUS_2`/`POS_RD_*`/`EN_POS_*`/...), read
//! with Modbus function 4 (Read Input Registers) as `INT16`; and 5 32-bit
//! output "registers" per axis (`COMMAND`/`POSITION`/`SPEED`/`ACCEL_DECEL`/..),
//! written atomically with Modbus function 16 (Write Multiple Registers) as
//! `INT32_LE_BS` — true bit-shift-16 hi/lo word packing. Contrast
//! [`crate::ang1`], which packs 32-bit values as decimal base-1000 words over
//! a plain `INT16` port with scalar writes only. The output block doubles as
//! a configuration block: the same 5 registers, sent once at axis creation,
//! carry `CONFIGURATION`/`BASE_SPEED`/`HOME_TIMEOUT` instead of
//! `COMMAND`/`POSITION`/`SPEED`.
//!
//! ## Not modeled (documented)
//!
//! `ANF2_RESET_ERRORS`/`ANF2_GET_INFO` are C custom asyn `Int32` params meant
//! for `ao`/`longout` records outside the motor record (a controller
//! `writeInt32` override) — there is no analogous generic-asyn-param-to-record
//! binding at this port's [`AsynMotor`] boundary (mirrors the "not modeled"
//! auxiliary parameters documented in `smaract::mcs2`). `resetErrors` is
//! still wired internally: `poll` calls it automatically on a command-error
//! bit, exactly as C does; only the *external*, user-triggered path is
//! unmodeled. C's per-axis diagnostic decode fields (`CaptInput_`,
//! `HomeInput_`, ... used only by C's `report()`) are also not modeled —
//! `has_encoder` is the only bit of `config` with a functional effect on
//! `AsynMotor`, so it alone is decoded.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::user::AsynUser;

use motor_common::util::nint;

use crate::regs::ModbusRegs;

/// C `AXIS_REG_OFFSET` — asyn-address stride per axis on both ports.
pub const AXIS_REG_OFFSET: i32 = 10;
/// C `MAX_AXES`.
pub const MAX_AXES: i32 = 12;

/// C `DEFAULT_CONTROLLER_TIMEOUT` (`asynMotorController.h`).
const TIMEOUT: Duration = Duration::from_millis(2000);

// Input registers (16-bit), offset from the axis base.
const STATUS_1: i32 = 0;
const STATUS_2: i32 = 1;
const POS_RD_UPR: i32 = 2;
const EN_POS_UPR: i32 = 4;

// Output "registers" (32-bit elements of the 5-element command/config array).
const COMMAND: usize = 0;
const POSITION: usize = 1;
const SPEED: usize = 2;
const ACCEL_DECEL: usize = 3;
const CONFIGURATION: usize = 0;
const BASE_SPEED: usize = 1;
const HOME_TIMEOUT: usize = 2;

const ZERO5: [i32; 5] = [0; 5];

/// Shared controller state: the in/out Modbus register handles and the
/// axis-created gate (C `axesCreated_`, guards `poll` until every axis
/// configured on the controller has been constructed).
pub struct Anf2Controller {
    in_regs: ModbusRegs,
    out_regs: ModbusRegs,
    num_axes: usize,
    axes_created: usize,
}

impl Anf2Controller {
    /// Connect the In/Out Modbus ports previously created by
    /// `drvModbusAsynConfigure` (C constructor's `pasynInt32SyncIO`/
    /// `pasynInt32ArraySyncIO` connect loop).
    pub fn new(in_port: &str, out_port: &str, num_axes: usize) -> Result<Self, String> {
        Ok(Self {
            in_regs: ModbusRegs::connect(in_port, TIMEOUT)?,
            out_regs: ModbusRegs::connect(out_port, TIMEOUT)?,
            num_axes,
            axes_created: 0,
        })
    }

    fn read16(&self, axis_no: i32, reg: i32) -> AsynResult<i32> {
        self.in_regs.read16(axis_no * AXIS_REG_OFFSET + reg)
    }

    /// C `readReg32`: combine two `INT16` registers with a true bit-shift-16
    /// pack (`(upper << 16) | lower`). Both words are already exact integers,
    /// so C's `NINT(...)` around the combine is a no-op and is not repeated
    /// here.
    fn read32(&self, axis_no: i32, reg: i32) -> AsynResult<i32> {
        let upper = self.read16(axis_no, reg)?;
        let lower = self.read16(axis_no, reg + 1)?;
        Ok((upper << 16) | lower)
    }

    fn write_array(&self, axis_no: i32, data: &[i32]) -> AsynResult<()> {
        self.out_regs
            .write_array(axis_no * AXIS_REG_OFFSET, data.to_vec())
    }

    fn force_read(&self) -> AsynResult<()> {
        self.in_regs.force_read()
    }
}

/// One ANF2 axis. Implements [`AsynMotor`].
pub struct Anf2Axis {
    controller: Arc<Mutex<Anf2Controller>>,
    axis_no: i32,
    /// C `baseSpeed_` — used by [`Self::correct_accel`].
    base_speed: i32,
    /// C `jogging_` — `stop` skips the hold-move write while a jog is active.
    jogging: bool,
    /// Decoded once at construction from `config` bits 24/25 (quadrature
    /// encoder / diagnostic feedback) — the only config bits with a
    /// functional effect at the `AsynMotor` boundary.
    has_encoder: bool,
    /// C `motorStatusDirection_`, persisted across polls (only updated while
    /// moving) and used to gate the high/low limit bits.
    direction: bool,
    /// Last published status. C's poll reuses a single stale `read_val` on a
    /// failed read and always publishes; this carries forward the last value
    /// field-by-field for any read that fails in a cycle (see [`Self::poll`]).
    last: MotorStatus,
}

impl Anf2Axis {
    /// C `ANF2Axis::ANF2Axis`: send the configuration array, decode the
    /// has-encoder bits, and zero the position. Performs blocking I/O.
    pub fn new(
        controller: Arc<Mutex<Anf2Controller>>,
        axis_no: i32,
        config: i32,
        base_speed: i32,
        homing_timeout: i32,
    ) -> AsynResult<Self> {
        let mut ax = Self {
            controller,
            axis_no,
            base_speed,
            jogging: false,
            has_encoder: false,
            direction: true,
            last: MotorStatus {
                gain_support: true,
                ..MotorStatus::default()
            },
        };

        sleep(Duration::from_millis(100));

        let mut conf_reg = ZERO5;
        conf_reg[CONFIGURATION] = config;
        conf_reg[BASE_SPEED] = base_speed;
        conf_reg[HOME_TIMEOUT] = homing_timeout << 16;
        ax.lock().write_array(ax.axis_no, &conf_reg)?;

        sleep(Duration::from_millis(50));

        // Only allow UEIP if the axis has a quadrature encoder or diagnostic
        // feedback configured (config bits 24/25).
        ax.has_encoder = (config & 0x0100_0000) != 0 || (config & 0x0200_0000) != 0;

        ax.set_position(&AsynUser::new(0), 0.0)?;

        ax.lock().axes_created += 1;

        Ok(ax)
    }

    fn lock(&self) -> MutexGuard<'_, Anf2Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `sendAccelAndVelocity`: clamp velocity/acceleration to the ANF2's
    /// wire ranges and fill the SPEED/ACCEL_DECEL slots of `reg`.
    fn send_accel_and_velocity(reg: &mut [i32; 5], acceleration: f64, velocity: f64) {
        let velocity = velocity.clamp(1.0, 1_000_000.0);
        reg[SPEED] = nint(velocity);

        let acceleration = acceleration.clamp(1000.0, 2_000_000.0);
        let steps_per_ms = nint(acceleration / 1000.0);
        reg[ACCEL_DECEL] = (steps_per_ms << 16) | steps_per_ms;
    }

    /// C `correctAccel`: recompute the acceleration that gives the record's
    /// requested accel *time* using the base speed fixed at axis creation,
    /// since the controller has no way to re-read VBAS after startup. No
    /// protection against a zero `acceleration`/`accelTime` — matches C,
    /// which has none either.
    fn correct_accel(
        base_speed: f64,
        min_velocity: f64,
        max_velocity: f64,
        acceleration: f64,
    ) -> f64 {
        let accel_time = (max_velocity - min_velocity) / acceleration;
        (max_velocity - base_speed) / accel_time
    }

    /// Shared `move`/`moveVelocity`/`home` prologue: clear the
    /// command/configuration register and give the controller 50ms.
    fn clear_command_reg(&self) -> AsynResult<()> {
        self.lock().write_array(self.axis_no, &ZERO5)?;
        sleep(Duration::from_millis(50));
        Ok(())
    }

    fn do_move(
        &mut self,
        position: f64,
        relative: bool,
        min_velocity: f64,
        max_velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.clear_command_reg()?;

        let acceleration = Self::correct_accel(
            self.base_speed as f64,
            min_velocity,
            max_velocity,
            acceleration,
        );
        let mut reg = ZERO5;
        Self::send_accel_and_velocity(&mut reg, acceleration, max_velocity);

        reg[COMMAND] = if relative { 0x2 << 16 } else { 0x1 << 16 };
        reg[POSITION] = nint(position);

        self.lock().write_array(self.axis_no, &reg)?;
        sleep(Duration::from_millis(50));
        Ok(())
    }

    /// C `resetErrors`, called automatically from `poll` on a command-error
    /// bit. Takes the already-locked controller to avoid re-locking from
    /// within `poll`.
    fn reset_errors(&self, ctrl: &Anf2Controller) -> AsynResult<()> {
        let mut reg = ZERO5;
        reg[COMMAND] = 0x800 << 16;
        ctrl.write_array(self.axis_no, &reg)
    }
}

impl AsynMotor for Anf2Axis {
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
        mut velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // The jog command requires a different stop than a move command.
        self.jogging = true;
        self.clear_command_reg()?;

        let mut reg = ZERO5;
        if velocity > 0.0 {
            reg[COMMAND] = 0x80 << 16; // positive jog
        } else {
            reg[COMMAND] = 0x100 << 16; // negative jog
            velocity = velocity.abs(); // ANF2 only accepts speeds > 0
        }
        Self::send_accel_and_velocity(&mut reg, acceleration, velocity);

        self.lock().write_array(self.axis_no, &reg)?;
        sleep(Duration::from_millis(50));
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        self.clear_command_reg()?;

        let acceleration =
            Self::correct_accel(self.base_speed as f64, min_velocity, velocity, acceleration);
        let mut reg = ZERO5;
        Self::send_accel_and_velocity(&mut reg, acceleration, velocity);
        // If the home input is active when the home command is sent, the
        // axis will appear to move in the wrong direction (C parity note).
        reg[COMMAND] = if forward { 0x20 << 16 } else { 0x40 << 16 };

        self.lock().write_array(self.axis_no, &reg)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        // The stop commands ignore all 32-bit registers beyond the first.
        ctrl.write_array(self.axis_no, &ZERO5[..1])?;
        if self.jogging {
            drop(ctrl);
            self.jogging = false;
            Ok(())
        } else {
            // Hold move: works well with normal moves (an immediate stop cuts
            // the pulses without deceleration and invalidates the position).
            ctrl.write_array(self.axis_no, &[0x4 << 16])
        }
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.lock().write_array(self.axis_no, &ZERO5)?;
        sleep(Duration::from_millis(100));

        let mut reg = ZERO5;
        reg[COMMAND] = 0x200 << 16;
        reg[POSITION] = nint(position);

        self.lock().write_array(self.axis_no, &reg)?;
        sleep(Duration::from_millis(200));
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, _enable: bool) -> AsynResult<()> {
        // The ANF2 has no closed-loop enable/disable command (C no-op): its
        // configuration can disable an axis or its encoder inputs, but not
        // on the fly, and doing so wouldn't disable torque anyway.
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // Not implemented in the C ANF2 driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();

        // Don't poll until every axis on this controller has been created —
        // avoids interpreting still-being-written config values as command
        // values (C `axesCreated_ != numAxes_`).
        if ctrl.axes_created != ctrl.num_axes {
            return Ok(MotorStatus {
                moving: false,
                ..MotorStatus::default()
            });
        }

        // C presses on through every read and always publishes: each read's
        // status is discarded (the base poller ignores poll()'s return) and
        // callParamCallbacks fires unconditionally. Mirror that — attempt each
        // read, carry forward the last published value for any field whose read
        // fails this cycle (defined, safe equivalent of C reusing a stale
        // `read_val`), and always return Ok so the framework publishes.
        let last = self.last.clone();

        let _ = ctrl.force_read();

        let position = ctrl
            .read32(self.axis_no, POS_RD_UPR)
            .map_or(last.position, |v| v as f64);
        let encoder_position = ctrl
            .read32(self.axis_no, EN_POS_UPR)
            .map_or(last.encoder_position, |v| v as f64);

        // Direction is only updated while moving; otherwise keep the last
        // polled value (it gates the limit bits below either way).
        let mut direction = self.direction;
        // STATUS_1: done / direction / command-error / powered. On a read
        // failure carry forward last-known done/powered and treat command-error
        // as clear (avoid a spurious reset from a stale read_val).
        let (done, cmd_error, powered) = match ctrl.read16(self.axis_no, STATUS_1) {
            Ok(status1) => {
                // Status word 1 bit 3 set to 1 when the motor is not in motion.
                let done = (status1 & 0x8) != 0;
                if !done {
                    if status1 & 0x1 != 0 {
                        direction = true;
                    }
                    if status1 & 0x2 != 0 {
                        direction = false;
                    }
                }
                // Enable/disable (not actually the torque status) — determined
                // by the configuration; it isn't obvious why one would disable
                // an axis.
                (done, status1 & 0x1000 != 0, status1 & 0x4000 != 0)
            }
            Err(_) => (last.done, false, last.powered),
        };

        // High limit reported only while moving positively, low limit only
        // while moving negatively.
        let (high_limit, low_limit) = match ctrl.read16(self.axis_no, STATUS_2) {
            Ok(status2) => (
                (status2 & 0x8 != 0) && direction,
                (status2 & 0x10 != 0) && !direction,
            ),
            Err(_) => (last.high_limit, last.low_limit),
        };

        // Clear command errors so we can attempt to move again (error
        // discarded, matching C).
        if cmd_error {
            let _ = self.reset_errors(&ctrl);
        }

        drop(ctrl);
        self.direction = direction;

        let status = MotorStatus {
            position,
            encoder_position,
            done,
            moving: !done,
            direction,
            high_limit,
            low_limit,
            powered,
            has_encoder: self.has_encoder,
            gain_support: true,
            ..MotorStatus::default()
        };
        self.last = status.clone();
        Ok(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn send_accel_and_velocity_clamps_and_packs() {
        let mut reg = ZERO5;
        Anf2Axis::send_accel_and_velocity(&mut reg, 500_000.0, 2_000_000.0);
        // Velocity clamped to 1_000_000 max.
        assert_eq!(reg[SPEED], 1_000_000);
        // Acceleration converted to steps/ms and packed into both halves.
        let steps_per_ms = 500;
        assert_eq!(reg[ACCEL_DECEL], (steps_per_ms << 16) | steps_per_ms);
    }

    #[test]
    fn send_accel_and_velocity_clamps_low_end() {
        let mut reg = ZERO5;
        Anf2Axis::send_accel_and_velocity(&mut reg, 1.0, 0.0);
        // Velocity clamped up to 1, acceleration clamped up to 1000 (1 step/ms).
        assert_eq!(reg[SPEED], 1);
        assert_eq!(reg[ACCEL_DECEL], (1 << 16) | 1);
    }

    #[test]
    fn correct_accel_matches_c_formula() {
        // accelTime = (max - min) / accel; corrected = (max - baseSpeed) / accelTime.
        let corrected = Anf2Axis::correct_accel(500.0, 0.0, 2500.0, 1000.0);
        // accelTime = 2500 / 1000 = 2.5; corrected = (2500 - 500) / 2.5 = 800.
        assert!((corrected - 800.0).abs() < 1e-9);
    }
}
