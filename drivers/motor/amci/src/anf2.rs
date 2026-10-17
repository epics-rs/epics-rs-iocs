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
//! ## Custom params
//!
//! `ANF2_RESET_ERRORS` and `ANF2_GET_INFO` are ported in [`Anf2Params`].
//! `resetErrors` also runs from `poll` on a command-error bit, exactly as C
//! does. `has_encoder` is the only bit of `config` with a functional effect
//! on `AsynMotor`; the rest is decoded for the report alone.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{AxisUser, ParamWrite, has_motor_axis};

use motor_common::util::nint;
use motor_common::{AxisReport, ControllerParams, ControllerReport};

use crate::regs::{ForceRead, ModbusRegs};

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

/// C `MAX_INPUT_REGS`.
const MAX_INPUT_REGS: usize = 10;

const RULE: &str = "====================================";

/// Shared controller state: the in/out Modbus register handles and the
/// axis-created gate (C `axesCreated_`, guards `poll` until every axis
/// configured on the controller has been constructed).
pub struct Anf2Controller {
    /// C `inputDriver_`, which each axis connects its force-read user to.
    in_port: String,
    in_regs: ModbusRegs,
    out_regs: ModbusRegs,
    num_axes: usize,
    axes_created: usize,
    /// The axes `ANF2CreateAxis` made, by axis number.
    axes: BTreeMap<i32, AxisSetup>,
    /// C `movingPollPeriod_` / `idlePollPeriod_`, in seconds, once
    /// `ANF2StartPoller` ran.
    poll_periods: (f64, f64),
}

/// What C's `ANF2Axis` keeps of its creation, and its `inputReg_`.
struct AxisSetup {
    /// C `pasynUserForceRead_`.
    force_read: ForceRead,
    config: i32,
    base_speed: i32,
    homing_timeout: i32,
    /// C `inputReg_`, as the last `getInfo` read it.
    input_regs: [i32; MAX_INPUT_REGS],
}

impl Anf2Controller {
    /// Connect the In/Out Modbus ports previously created by
    /// `drvModbusAsynConfigure` (C constructor's `pasynInt32SyncIO`/
    /// `pasynInt32ArraySyncIO` connect loop). C keeps the controller either
    /// way; the status of the last output connect, the only one C checks, is
    /// returned, and is success when the loop has no axis to run.
    pub fn new(in_port: &str, out_port: &str, num_axes: usize) -> (Self, AsynResult<()>) {
        let (in_regs, _) = ModbusRegs::connect(in_port, TIMEOUT);
        let (out_regs, out_status) = ModbusRegs::connect(out_port, TIMEOUT);
        let ctrl = Self {
            in_port: in_port.to_string(),
            in_regs,
            out_regs,
            num_axes,
            axes_created: 0,
            axes: BTreeMap::new(),
            poll_periods: (0.0, 0.0),
        };
        (ctrl, if num_axes == 0 { Ok(()) } else { out_status })
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

    /// C `ANF2Axis::resetErrors` (ANF2Driver.cpp:480-496).
    fn reset_errors(&self, axis_no: i32) -> AsynResult<()> {
        let mut reg = ZERO5;
        reg[COMMAND] = 0x800 << 16;
        self.write_array(axis_no, &reg)
    }

    /// C `startPoller`'s periods, in seconds.
    pub fn set_poll_periods(&mut self, moving: f64, idle: f64) {
        self.poll_periods = (moving, idle);
    }

    /// C `ANF2Axis::getInfo` (ANF2Driver.cpp:498-513): a register whose
    /// read fails keeps what it held.
    fn get_info(&mut self, axis_no: i32) {
        let Some(axis) = self.axes.get_mut(&axis_no) else {
            return;
        };
        let _ = axis.force_read.write();
        for (reg, value) in (0..).zip(axis.input_regs.iter_mut()) {
            if let Ok(v) = self.in_regs.read16(axis_no * AXIS_REG_OFFSET + reg) {
                *value = v;
            }
        }
    }

    /// C `ANF2Controller::report` ahead of the base's (ANF2Driver.cpp:
    /// 140-183). C reads `getAxis(j)` of every address and crashes on one
    /// `ANF2CreateAxis` never made; it has no column here.
    fn report(&mut self, port: &str, out: &mut dyn std::fmt::Write) {
        let (moving, idle) = self.poll_periods;
        let _ = write!(
            out,
            "{RULE}\nANF2 motor driver:\n    asyn port: {port}\n    num axes: {}\n    \
             axes created: {}\n    moving poll period: {moving:.6}\n    \
             idle poll period: {idle:.6}\n\nInput registers:\n\n",
            self.num_axes, self.axes_created,
        );
        let created: Vec<i32> = (0..self.num_axes as i32)
            .filter(|j| self.axes.contains_key(j))
            .collect();
        for &j in &created {
            self.get_info(j);
        }
        let _ = write!(out, " Reg\t");
        for j in &created {
            let _ = write!(out, "Axis {j}\t");
        }
        let _ = writeln!(out);
        for i in 0..MAX_INPUT_REGS {
            let _ = write!(out, "  {i}\t");
            for j in &created {
                let _ = write!(out, "0x{:04x}\t", self.axes[j].input_regs[i]);
            }
            let _ = writeln!(out);
        }
        let _ = writeln!(out);
    }

    /// C `ANF2Axis::report` (ANF2Driver.cpp:558-588), its fields decoded
    /// from `config` as the constructor does (:363-380).
    fn axis_report(&self, axis_no: i32, out: &mut dyn std::fmt::Write) {
        let Some(axis) = self.axes.get(&axis_no) else {
            return;
        };
        let config = axis.config;
        let field = |mask: i32, shift: u32| ((config as u32 & mask as u32) >> shift) as i32;
        let _ = write!(
            out,
            "Configuration for axis {axis_no} [0x{config:x}]:\n  Base Speed: {}\n  \
             Homing Timeout: {}\n  Capture Input: {} (Active State: {})\n  \
             External Input: {} (Active State: {})\n  Home Input: {} (Active State: {})\n  \
             CW Input: {} (Active State: {})\n  CCW Input: {} (Active State: {})\n  \
             Backplane Home Proximity Operation: {}\n  Quadrature Encoder: {}\n  \
             Diagnostic Feedback: {}\n  Output Pulse Type: {}\n  Home Operation: {}\n  \
             Card Axis: {}\n  Operation Mode for Axis: {}\n\n",
            axis.base_speed,
            axis.homing_timeout,
            field(0x1 << 16, 16),
            field(0x1, 0),
            field(0x2 << 16, 17),
            field(0x2, 1),
            field(0x4 << 16, 18),
            field(0x4, 2),
            field(0x18 << 16, 19),
            field(0x8, 3),
            field(0x60 << 16, 21),
            field(0x10, 4),
            field(0x80 << 16, 23),
            field(0x100 << 16, 24),
            field(0x200 << 16, 25),
            field(0x400 << 16, 26),
            field(0x800 << 16, 27),
            field(0x4000 << 16, 30),
            field(0x8000 << 16, 31),
        );
    }
}

/// The ANF2 controller's own parameters and report on its asyn port.
pub struct Anf2Params {
    controller: Arc<Mutex<Anf2Controller>>,
    /// C `ANF2ResetErrors_`, once created.
    reset_errors: Option<usize>,
    /// C `ANF2GetInfo_`, once created.
    get_info: Option<usize>,
}

impl Anf2Params {
    pub fn new(controller: Arc<Mutex<Anf2Controller>>) -> Self {
        Self {
            controller,
            reset_errors: None,
            get_info: None,
        }
    }
}

impl ControllerParams for Anf2Params {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        self.reset_errors = Some(base.create_param("ANF2_RESET_ERRORS", ParamType::Int32)?);
        self.get_info = Some(base.create_param("ANF2_GET_INFO", ParamType::Int32)?);
        Ok(())
    }

    /// C `ANF2Controller::writeInt32` (ANF2Driver.cpp:211-246): the value is
    /// stored, and a 1 runs the axis's command. C dereferences a null axis
    /// at an address `ANF2CreateAxis` never made; nothing runs there.
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        value: i32,
    ) -> ParamWrite {
        // C dereferences the null axis there (ANF2Driver.cpp:215-219); refused as the base does.
        if !has_motor_axis(base, user) {
            return ParamWrite::PassToBase(Ok(()));
        }
        let addr = user.addr;
        let reset = Some(user.reason) == self.reset_errors;
        if !reset && Some(user.reason) != self.get_info {
            return ParamWrite::PassToBase(Ok(()));
        }
        let _ = base.set_int32_param(user.reason, addr, value);
        let mut ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        if value == 1 && ctrl.axes.contains_key(&addr) {
            if reset {
                println!("ANF2Controller:writeInt32: Resetting errors for axis = {addr}");
                let _ = ctrl.reset_errors(addr);
            } else {
                println!("ANF2Controller:writeInt32: Getting info for axis = {addr}");
                ctrl.get_info(addr);
            }
        }
        drop(ctrl);
        let _ = base.call_param_callbacks(addr);
        ParamWrite::Handled(Ok(()))
    }

    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        // Released before the base's report runs each axis's, which takes it.
        self.controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .report(ctx.port_name(), out);
        ctx.report(out, level);
        let _ = writeln!(out, "{RULE}");
    }
}

/// C `ANF2Axis::report` for axis `axis`.
pub struct Anf2AxisReport {
    pub controller: Arc<Mutex<Anf2Controller>>,
    pub axis: i32,
}

impl AxisReport for Anf2AxisReport {
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level > 0 {
            let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
            ctrl.axis_report(self.axis, out);
        }
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
    /// has-encoder bits, and zero the position. Performs blocking I/O. C
    /// ignores every status here, so the axis is always built and counted.
    pub fn new(
        controller: Arc<Mutex<Anf2Controller>>,
        axis_no: i32,
        config: i32,
        base_speed: i32,
        homing_timeout: i32,
    ) -> Self {
        let in_port = controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .in_port
            .clone();
        let force_read = ForceRead::connect(&in_port, axis_no * AXIS_REG_OFFSET, TIMEOUT);
        if !force_read.connected() {
            println!(
                "{in_port}: Error, unable to connect pasynUserForceRead_ to Modbus input driver"
            );
        }
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
        let _ = ax.lock().write_array(ax.axis_no, &conf_reg);

        sleep(Duration::from_millis(50));

        // Only allow UEIP if the axis has a quadrature encoder or diagnostic
        // feedback configured (config bits 24/25).
        ax.has_encoder = (config & 0x0100_0000) != 0 || (config & 0x0200_0000) != 0;

        let _ = ax.set_position(&AsynUser::new(0), 0.0);

        let mut ctrl = ax.lock();
        ctrl.axes.insert(
            axis_no,
            AxisSetup {
                force_read,
                config,
                base_speed,
                homing_timeout,
                input_regs: [0; MAX_INPUT_REGS],
            },
        );
        ctrl.axes_created += 1;
        drop(ctrl);

        ax
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
        ctrl.reset_errors(self.axis_no)
    }
}

impl AsynMotor for Anf2Axis {
    /// C `ANF2Axis::poll` ignores each read's status and sets no status bit.
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

        if let Some(axis) = ctrl.axes.get(&self.axis_no) {
            let _ = axis.force_read.write();
        }

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
    fn axes_of_missing_ports_are_built_and_counted() {
        let (ctrl, out_status) = Anf2Controller::new("anf2MissingIn", "anf2MissingOut", 1);
        assert!(out_status.is_err());
        let ctrl = Arc::new(Mutex::new(ctrl));
        let axis = Anf2Axis::new(ctrl.clone(), 0, 0x0100_0000, 1, 0);
        assert!(axis.has_encoder);
        assert_eq!(ctrl.lock().unwrap().axes_created, 1);
    }

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

    #[test]
    fn an_axis_forces_its_read_at_its_register_block() {
        let (reason, taken) = crate::regs::fake::publish("anf2ForceReadIn");
        crate::regs::fake::publish("anf2ForceReadOut");
        let ctrl = Anf2Controller::new("anf2ForceReadIn", "anf2ForceReadOut", 1).0;
        let mut ax = Anf2Axis::new(Arc::new(Mutex::new(ctrl)), 2, 0, 1, 0);
        ax.poll(&AsynUser::new(0)).unwrap();
        assert_eq!(*taken.lock().unwrap(), [(reason, 2 * AXIS_REG_OFFSET)]);
    }
}
