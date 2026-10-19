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
//! ## Custom params
//!
//! `ANG1_JERK` (the `JERK` register, reg 9) is ported in [`Ang1Params`].

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{AxisUser, ParamWrite, has_motor_axis};
use epics_rs::motor::trace::MotorTrace;

use motor_common::util::nint;
use motor_common::{AxisReport, ControllerParams, ControllerReport};

use crate::regs::{ForceRead, ModbusRegs};

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
    /// The controller port, whose pasynUserSelf C traces on.
    port: String,
    /// C `inputDriver_`, which each axis connects its force-read user to.
    in_port: String,
    in_regs: ModbusRegs,
    out_regs: ModbusRegs,
}

impl Ang1Controller {
    /// Connect the In/Out Modbus ports previously created by
    /// `drvModbusAsynConfigure`. C keeps the controller either way; the
    /// output connect's status, the only one C checks, is returned.
    pub fn new(port: &str, in_port: &str, out_port: &str) -> (Self, AsynResult<()>) {
        let (in_regs, _) = ModbusRegs::connect(in_port, TIMEOUT);
        let (out_regs, out_status) = ModbusRegs::connect(out_port, TIMEOUT);
        let ctrl = Self {
            port: port.to_string(),
            in_port: in_port.to_string(),
            in_regs,
            out_regs,
        };
        (ctrl, out_status)
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
        MotorTrace::new(self.port.clone(), 0).print_self(
            TraceMask::IO_DRIVER,
            format_args!("writeReg16: writing {value} to register {reg}\n"),
        );
        let status = self.out_regs.write16(reg, value);
        sleep(Duration::from_millis(10));
        status
    }

    /// C `writeReg32`: decimal base-1000 split (written as two `writeReg16`
    /// calls, so this issues two 10ms delays).
    fn write32(&self, reg: i32, value: i32) -> AsynResult<()> {
        let (upper, lower) = pack1000(value);
        // C's writeReg32 issues both halves unconditionally and returns the
        // second's status; a failed MSW write does not suppress the LSW write.
        let _ = self.write16(reg, upper);
        self.write16(reg + 1, lower)
    }
}

/// C `JERK`, the output register `ANG1_JERK` writes.
pub(crate) const JERK: i32 = 9;

/// The ANG1 controller's `ANG1_JERK` and its poll periods, in seconds, for
/// its report.
pub struct Ang1Params {
    controller: Arc<Mutex<Ang1Controller>>,
    moving_poll: f64,
    idle_poll: f64,
    /// C `ANG1Jerk_`, once created.
    jerk: Option<usize>,
}

impl Ang1Params {
    pub fn new(controller: Arc<Mutex<Ang1Controller>>, moving_poll: f64, idle_poll: f64) -> Self {
        Self {
            controller,
            moving_poll,
            idle_poll,
            jerk: None,
        }
    }
}

impl ControllerParams for Ang1Params {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        self.jerk = Some(base.create_param("ANG1_JERK", ParamType::Int32)?);
        Ok(())
    }

    /// C `ANG1Controller::writeInt32` (ANG1Driver.cpp:161-185): the value is
    /// stored, then written to the `JERK` register whatever it is.
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        value: i32,
    ) -> ParamWrite {
        // C dereferences the null axis there (ANG1Driver.cpp:165-169); refused as the base does.
        if !has_motor_axis(base, user) {
            return ParamWrite::PassToBase(Ok(()));
        }
        if Some(user.reason) != self.jerk {
            return ParamWrite::PassToBase(Ok(()));
        }
        let addr = user.addr;
        let _ = base.set_int32_param(user.reason, addr, value);
        println!("Jerk = {value}");
        let status = self
            .controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .write16(JERK, value);
        let _ = base.call_param_callbacks(addr);
        if let Err(e) = &status {
            MotorTrace::new(base.port_name.to_string(), user.asyn().addr).print(
                TraceMask::ERROR,
                user.reason as i32,
                format_args!(
                    "ANG1MotorDriver:writeInt32: error, status={} function={}, value={value}\n",
                    e.status() as i32,
                    user.reason,
                ),
            );
        }
        ParamWrite::Handled(status)
    }

    /// C `ANG1Controller::report` (ANG1Driver.cpp:127-134).
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "ANG1 motor driver {}, numAxes={}, moving poll period={:.6}, idle poll period={:.6}",
            ctx.port_name(),
            ctx.num_axes(),
            self.moving_poll,
            self.idle_poll,
        );
        ctx.report(out, level);
    }
}

/// C `ANG1Axis::report` (ANG1Driver.cpp:313-322) for axis `.0`.
pub struct Ang1AxisReport(pub i32);

impl AxisReport for Ang1AxisReport {
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level > 0 {
            let _ = writeln!(out, "  axis {}", self.0);
        }
    }
}

/// One ANG1 axis. Implements [`AsynMotor`].
pub struct Ang1Axis {
    controller: Arc<Mutex<Ang1Controller>>,
    force_read: ForceRead,
    /// Last published status. C's poll reuses a single stale `read_val` on a
    /// failed read and always publishes; this carries forward the last value
    /// field-by-field for any read that fails in a cycle (see [`Self::poll`]).
    last: MotorStatus,
}

impl Ang1Axis {
    /// C `ANG1Axis::ANG1Axis`: zero the position. `axis_no` is accepted for
    /// parity with the multi-axis C constructor signature but is otherwise
    /// unused — see the module doc on the shared, non-offset register block.
    ///
    /// Infallible, matching C: the constructor calls `setPosition(0)` and
    /// ignores its status, so a transient link error at IOC boot cannot abort
    /// `ANG1CreateController` (which C always returns `asynSuccess` from).
    pub fn new(controller: Arc<Mutex<Ang1Controller>>, _axis_no: i32) -> Self {
        let in_port = controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .in_port
            .clone();
        let force_read = ForceRead::connect(&in_port, 0, TIMEOUT);
        if !force_read.connected() {
            println!(
                "{in_port}: Error, unable to connect pasynUserForceRead_ to Modbus input driver"
            );
        }
        println!(
            "ANG1Axis::ANG1Axis : pasynUserForceRead_->reason={}",
            force_read.reason()
        );
        let mut ax = Self {
            controller,
            force_read,
            last: MotorStatus {
                gain_support: true,
                ..MotorStatus::default()
            },
        };
        let _ = ax.set_position(&AsynUser::new(0), 0.0);
        ax
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
        // C attempts each write and returns the last status (the intermediate
        // ones are never checked).
        let _ = ctrl.write32(SPD_UPR, nint(velocity));

        let accel_steps_per_ms = nint(acceleration.clamp(1000.0, 5_000_000.0) / 1000.0);
        let _ = ctrl.write16(ACCEL, accel_steps_per_ms);
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
        // C attempts every write and returns the last (CMD_MSW=move_bit)
        // status, never short-circuiting on an intermediate failure.
        let _ = self.send_accel_and_velocity(&ctrl, acceleration, velocity);

        let distance = nint(position);
        let _ = ctrl.write32(POS_WR_UPR, distance);
        let _ = ctrl.write16(CMD_MSW, 0x0);
        let result = ctrl.write16(CMD_MSW, move_bit);
        drop(ctrl);
        sleep(Duration::from_millis(50));
        result
    }

    /// C `setClosedLoop`. Takes the already-locked controller so `poll`'s
    /// limit-reset path (which shares one lock across the whole poll cycle,
    /// matching C) doesn't re-lock.
    fn do_set_closed_loop(&self, ctrl: &Ang1Controller, enable: bool) -> AsynResult<()> {
        if enable {
            // C attempts all four writes, returning only the last status.
            let _ = ctrl.write16(CMD_MSW, 0x0);
            let _ = ctrl.write16(CMD_MSW, 0x400);
            let _ = ctrl.write16(CMD_MSW, 0x0);
            ctrl.write16(CMD_LSW, 0x8000)
        } else {
            ctrl.write16(CMD_LSW, 0x0)
        }
    }

    /// C `setPosition`. Takes the already-locked controller — see
    /// [`Self::do_set_closed_loop`].
    fn do_set_position(&self, ctrl: &Ang1Controller, position: f64) -> AsynResult<()> {
        // C attempts all writes and returns the last status.
        let _ = ctrl.write32(POS_WR_UPR, nint(position));
        let _ = ctrl.write16(CMD_MSW, 0x200);
        ctrl.write16(CMD_MSW, 0x0)
    }
}

impl AsynMotor for Ang1Axis {
    /// C `ANG1Axis::poll` ignores each read's status and sets no status bit.
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
        let _ = self.send_accel_and_velocity(&ctrl, acceleration, velocity.abs());

        // ANG1 has no jog command: simulate one with a million-step move.
        let distance = if velocity > 0.0 {
            1_000_000
        } else {
            -1_000_000
        };
        // C attempts every write and returns the last status.
        let _ = ctrl.write32(POS_WR_UPR, distance);
        let _ = ctrl.write16(CMD_MSW, 0x0);
        let result = ctrl.write16(CMD_MSW, 0x2);
        drop(ctrl);
        sleep(Duration::from_millis(50));
        result
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
        // C attempts both writes and returns the last status.
        let _ = ctrl.write16(CMD_MSW, 0x0);
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
        // C presses on through every read and always publishes: each read's
        // status is discarded (the base poller ignores poll()'s return) and
        // callParamCallbacks fires unconditionally. Mirror that — attempt each
        // read, carry forward the last published value for any field whose read
        // fails this cycle, and always return Ok so the framework publishes.
        // (C reuses a single stale `read_val` on failure; last-known is the
        // defined, safe equivalent.)
        let last = self.last.clone();

        let ctrl = self.lock();
        let _ = self.force_read.write();

        let position = ctrl.read32(POS_RD_UPR).map_or(last.position, |v| v as f64);

        // Status word 1 bit 3 set to 1 when the motor is not in motion.
        let done = ctrl.read16(STATUS_1).map_or(last.done, |v| v & 0x8 != 0);

        let (high_limit, low_limit, powered) = match ctrl.read16(STATUS_2) {
            Ok(status2) => (
                status2 & 0x1 != 0,
                status2 & 0x2 != 0,
                status2 & 0x8000 != 0,
            ),
            Err(_) => (last.high_limit, last.low_limit, last.powered),
        };

        // A CW limit reached: reset the error and set position so the axis
        // can move off the limit (errors discarded, matching C).
        if high_limit {
            let _ = self.do_set_closed_loop(&ctrl, true);
            let _ = self.do_set_position(&ctrl, position);
        }

        // A CCW limit reached: same reset.
        if low_limit {
            let _ = self.do_set_closed_loop(&ctrl, true);
            let _ = self.do_set_position(&ctrl, position);
        }
        drop(ctrl);

        let status = MotorStatus {
            position,
            done,
            moving: !done,
            high_limit,
            low_limit,
            powered,
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
    fn a_controller_of_missing_ports_keeps_its_axes() {
        let (ctrl, out_status) =
            Ang1Controller::new("ang1Missing", "ang1MissingIn", "ang1MissingOut");
        assert!(out_status.is_err());
        let ax = Ang1Axis::new(Arc::new(Mutex::new(ctrl)), 0);
        assert!(ax.lock().read16(STATUS_1).is_err());
    }

    /// C `writeReg16` prints each register write on pasynUserSelf before it
    /// is tried (ANG1Driver.cpp:203).
    #[test]
    fn a_register_write_prints_its_traceio_driver_line() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;
        let port = "ang1RegTrace";
        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(port), TraceMask::IO_DRIVER);
        trace.set_trace_file(Some(port), TraceFile::File(file.clone()));
        trace.set_device_trace_file(port, 0, TraceFile::File(file));
        let (ctrl, _) = Ang1Controller::new(port, "ang1RegTraceIn", "ang1RegTraceOut");
        assert!(ctrl.write32(SPD_UPR, 12_345).is_err());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            text.matches("writeReg16: writing 12 to register 4\n")
                .count(),
            1,
            "{text}"
        );
        assert_eq!(
            text.matches("writeReg16: writing 345 to register 5\n")
                .count(),
            1,
            "{text}"
        );
    }

    /// C `writeReg16` sleeps after the write whatever it returned
    /// (ANG1Driver.cpp:204-205).
    #[test]
    fn a_failed_register_write_still_waits_10_ms() {
        let (ctrl, _) = Ang1Controller::new("ang1FailWait", "ang1FailWaitIn", "ang1FailWaitOut");
        let started = std::time::Instant::now();
        assert!(ctrl.write16(CMD_MSW, 0).is_err());
        assert!(started.elapsed() >= Duration::from_millis(10));
    }

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
