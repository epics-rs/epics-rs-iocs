//! AMCI ANG1 stepper motor controller driver — a model-3 asyn driver talking
//! Modbus/TCP registers. Ported from `motorAMCI/amciApp/src/ANG1Driver.cpp`.
//!
//! Single fixed 10-register input/output block — no per-axis register
//! offset. The C driver never uses `axisNo_` to offset a register address, so
//! a controller created with `numAxes > 1` would have every axis read/write
//! the *same* registers; this is preserved faithfully (the C example
//! `st.cmd.ANG1` in practice creates one controller instance per physical
//! module with `numAxes=1`, sidestepping the issue rather than fixing it).
//!
//! 32-bit values (position, speed) are packed as **decimal base-1000** words
//! (`upper = value/1000, lower = value%1000`, computed via `f32` division
//! matching C's `float`-typed intermediate) over a plain `INT16` Modbus port —
//! contrast [`crate::anf2`], which bit-shift-packs 32-bit values natively via
//! `INT32_LE_BS` and array writes. ANG1 never uses array writes; every
//! register write is a single scalar `INT16` (Modbus function 6, Write Single
//! Register).
//!
//! ## Not modeled (documented)
//!
//! `ANG1_JERK` is a C custom asyn `Int32` param (writes the `JERK` register,
//! reg 9) meant for a record outside the motor record boundary — no
//! analogous binding exists at this port's [`AsynMotor`] boundary (see
//! `crate::anf2`'s equivalent note for `ANF2_RESET_ERRORS`/`ANF2_GET_INFO`).

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::user::AsynUser;

use motor_common::util::nint;

use crate::regs::ModbusRegs;

/// C `DEFAULT_CONTROLLER_TIMEOUT`.
const TIMEOUT: Duration = Duration::from_millis(2000);

// Input registers.
const STATUS_1: i32 = 0;
const STATUS_2: i32 = 1;
const POS_RD_UPR: i32 = 2;

// Output registers.
const CMD_MSW: i32 = 0;
const CMD_LSW: i32 = 1;
const POS_WR_UPR: i32 = 2;
const SPD_UPR: i32 = 4;
const ACCEL: i32 = 6;
const DECEL: i32 = 7;

/// C `writeReg32`'s decimal base-1000 split, computed in `f32` precision
/// (matching C's `float fnum`) rather than `f64` — the split is bit-exact to
/// C only if the intermediate stays single-precision.
fn pack1000(value: i32) -> (i32, i32) {
    let fnum = value as f32 / 1000.0;
    let upper = fnum as i32;
    let frac = fnum - upper as f32;
    let lower = nint((frac * 1000.0) as f64);
    (upper, lower)
}

/// C `readReg32`'s inverse combine: two `epicsInt16`-truncated halves back
/// into a decimal base-1000 value.
fn unpack1000(upper: i16, lower: i16) -> i32 {
    nint((upper as i32 * 1000 + lower as i32) as f64)
}

/// Shared controller state: the in/out Modbus register handles.
pub struct Ang1Controller {
    in_regs: ModbusRegs,
    out_regs: ModbusRegs,
}

impl Ang1Controller {
    /// Connect the In/Out Modbus ports previously created by
    /// `drvModbusAsynConfigure`.
    pub fn new(in_port: &str, out_port: &str) -> Result<Self, String> {
        Ok(Self {
            in_regs: ModbusRegs::connect(in_port, TIMEOUT)?,
            out_regs: ModbusRegs::connect(out_port, TIMEOUT)?,
        })
    }

    fn read16(&self, reg: i32) -> AsynResult<i32> {
        self.in_regs.read16(reg)
    }

    /// C `readReg32`: read two `INT16` registers and combine as a decimal
    /// base-1000 pair.
    fn read32(&self, reg: i32) -> AsynResult<i32> {
        let upper = self.read16(reg)? as i16;
        let lower = self.read16(reg + 1)? as i16;
        Ok(unpack1000(upper, lower))
    }

    /// C `writeReg16`: write one register, then sleep 10ms — baked into
    /// every ANG1 register write (unlike ANF2, which has no per-write
    /// delay).
    fn write16(&self, reg: i32, value: i32) -> AsynResult<()> {
        self.out_regs.write16(reg, value)?;
        sleep(Duration::from_millis(10));
        Ok(())
    }

    /// C `writeReg32`: decimal base-1000 split (written as two `writeReg16`
    /// calls, so this issues two 10ms delays).
    fn write32(&self, reg: i32, value: i32) -> AsynResult<()> {
        let (upper, lower) = pack1000(value);
        self.write16(reg, upper)?;
        self.write16(reg + 1, lower)
    }

    fn force_read(&self) -> AsynResult<()> {
        self.in_regs.force_read()
    }
}

/// One ANG1 axis. Implements [`AsynMotor`].
pub struct Ang1Axis {
    controller: Arc<Mutex<Ang1Controller>>,
}

impl Ang1Axis {
    /// C `ANG1Axis::ANG1Axis`: zero the position. `axis_no` is accepted for
    /// parity with the multi-axis C constructor signature but is otherwise
    /// unused — see the module doc on the shared, non-offset register block.
    pub fn new(controller: Arc<Mutex<Ang1Controller>>, _axis_no: i32) -> AsynResult<Self> {
        let mut ax = Self { controller };
        ax.set_position(&AsynUser::new(0), 0.0)?;
        Ok(ax)
    }

    fn lock(&self) -> MutexGuard<'_, Ang1Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `sendAccelAndVelocity`: velocity is sent unclamped; acceleration is
    /// clamped to the ANG1's wire range and converted to steps/ms/s. Takes the
    /// already-locked controller so a move's accel/velocity writes and its
    /// position/command writes stay in one lock section (see [`Self::do_move`]).
    fn send_accel_and_velocity(
        &self,
        ctrl: &Ang1Controller,
        acceleration: f64,
        velocity: f64,
    ) -> AsynResult<()> {
        ctrl.write32(SPD_UPR, nint(velocity))?;

        let accel_steps_per_ms = nint(acceleration.clamp(1000.0, 5_000_000.0) / 1000.0);
        ctrl.write16(ACCEL, accel_steps_per_ms)?;
        ctrl.write16(DECEL, accel_steps_per_ms)
    }

    fn do_move(
        &mut self,
        position: f64,
        move_bit: i32,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C runs sendAccelAndVelocity + the position/command writes under one
        // continuous asyn-port lock; hold the controller lock across the whole
        // sequence so the poller (which takes the same lock for a full cycle,
        // incl. limit-recovery CMD_MSW writes) cannot interleave mid-move.
        let ctrl = self.lock();
        self.send_accel_and_velocity(&ctrl, acceleration, velocity)?;

        let distance = nint(position);
        ctrl.write32(POS_WR_UPR, distance)?;
        ctrl.write16(CMD_MSW, 0x0)?;
        ctrl.write16(CMD_MSW, move_bit)?;
        drop(ctrl);
        sleep(Duration::from_millis(50));
        Ok(())
    }

    /// C `setClosedLoop`. Takes the already-locked controller so `poll`'s
    /// limit-reset path (which shares one lock across the whole poll cycle,
    /// matching C) doesn't re-lock.
    fn do_set_closed_loop(&self, ctrl: &Ang1Controller, enable: bool) -> AsynResult<()> {
        if enable {
            ctrl.write16(CMD_MSW, 0x0)?;
            ctrl.write16(CMD_MSW, 0x400)?;
            ctrl.write16(CMD_MSW, 0x0)?;
            ctrl.write16(CMD_LSW, 0x8000)
        } else {
            ctrl.write16(CMD_LSW, 0x0)
        }
    }

    /// C `setPosition`. Takes the already-locked controller — see
    /// [`Self::do_set_closed_loop`].
    fn do_set_position(&self, ctrl: &Ang1Controller, position: f64) -> AsynResult<()> {
        ctrl.write32(POS_WR_UPR, nint(position))?;
        ctrl.write16(CMD_MSW, 0x200)?;
        ctrl.write16(CMD_MSW, 0x0)
    }
}

impl AsynMotor for Ang1Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, 0x1, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, 0x2, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // Single lock section across accel/velocity + move writes (see do_move).
        let ctrl = self.lock();
        self.send_accel_and_velocity(&ctrl, acceleration, velocity.abs())?;

        // ANG1 has no jog command: simulate one with a million-step move.
        let distance = if velocity > 0.0 {
            1_000_000
        } else {
            -1_000_000
        };
        ctrl.write32(POS_WR_UPR, distance)?;
        ctrl.write16(CMD_MSW, 0x0)?;
        ctrl.write16(CMD_MSW, 0x2)?;
        drop(ctrl);
        sleep(Duration::from_millis(50));
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // C doesn't call sendAccelAndVelocity here (commented out).
        let home_bit = if forward { 0x20 } else { 0x40 };
        self.lock().write16(CMD_MSW, home_bit)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write16(CMD_MSW, 0x0)?;
        ctrl.write16(CMD_MSW, 0x4) // Hold move
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        self.do_set_position(&ctrl, position)
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        self.do_set_closed_loop(&ctrl, enable)
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // Not implemented in the C ANG1 driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        ctrl.force_read()?;

        let position = ctrl.read32(POS_RD_UPR)? as f64;

        let status1 = ctrl.read16(STATUS_1)?;
        // Status word 1 bit 3 set to 1 when the motor is not in motion.
        let done = (status1 & 0x8) != 0;

        let status2 = ctrl.read16(STATUS_2)?;

        // A CW limit reached: reset the error and set position so the axis
        // can move off the limit.
        let high_limit = status2 & 0x1 != 0;
        if high_limit {
            self.do_set_closed_loop(&ctrl, true)?;
            self.do_set_position(&ctrl, position)?;
        }

        // A CCW limit reached: same reset.
        let low_limit = status2 & 0x2 != 0;
        if low_limit {
            self.do_set_closed_loop(&ctrl, true)?;
            self.do_set_position(&ctrl, position)?;
        }

        let powered = status2 & 0x8000 != 0;

        Ok(MotorStatus {
            position,
            done,
            moving: !done,
            high_limit,
            low_limit,
            powered,
            gain_support: true,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack1000_exact_multiple() {
        assert_eq!(pack1000(5000), (5, 0));
        assert_eq!(pack1000(-5000), (-5, 0));
    }

    #[test]
    fn pack1000_roundtrips_through_unpack1000() {
        for value in [
            0, 1, 999, 1000, 1001, 123_456, -123_456, 2_000_000, -2_000_000,
        ] {
            let (upper, lower) = pack1000(value);
            assert_eq!(
                unpack1000(upper as i16, lower as i16),
                value,
                "value={value}"
            );
        }
    }

    #[test]
    fn unpack1000_combines_decimal_pair() {
        assert_eq!(unpack1000(5, 0), 5000);
        assert_eq!(unpack1000(5, 250), 5250);
        assert_eq!(unpack1000(-5, -250), -5250);
    }
}
