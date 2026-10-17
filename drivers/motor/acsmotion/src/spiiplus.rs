//! ACS SPiiPlus motion controller driver (ACSPL+ over [`SpiiPlusComm`]).
//!
//! Ported from `motorAcsMotion/acsMotionApp/src/SPiiPlusDriver.cpp` (the
//! model-3 `asynMotorController`/`asynMotorAxis` driver). Every exchange goes
//! through the controller's comm, as C's `pComm_` does; axes are addressed by
//! 0-based index on the wire.
//!
//! ## Poll
//!
//! As C, [`SpiiPlusController::poll`] reads every axis at once with the
//! binary protocol before the axes' polls, which only parse what it read.
//!
//! ## Units
//!
//! The motor record talks in steps; the controller in its own units. As C
//! does, each axis scales by `resolution_`, the stepper factor (`STEPF`) for
//! a microstepper or a stepper without encoder feedback and the encoder
//! factor (`EFAC`) otherwise: commands send `steps * resolution_`, the poll
//! reports `APOS / resolution_`, and the encoder position is `FPOS / EFAC`
//! (`VPOS / EFAC` for a virtual axis when the firmware has `VPOS`).
//!
//! ## Homing
//!
//! C `home()` reads the axis's `SPIIPLUS_HOMING_*` parameters, which
//! `SPiiPlusHoming.db` writes through the controller port;
//! [`AcsMotionParams`] takes those writes into the controller.
//!
//! ## Not modeled (documented)
//!
//! The aux asyn parameters (MFLAGS/MFLAGSX bit fields, safe-torque-off, encoder offsets
//! and factors, reference position, PEG, disable-set-position guard, global
//! variable read/write, ACSPL program start/stop).

use std::collections::HashSet;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::calc::engine::cvt::{fmt_e, fmt_f, fmt_g};
use epics_rs::motor::controller_port::{
    ControllerCtx, ControllerParams, ControllerReport, PROFILE_EXECUTE_DONE, ParamWrite, index,
};
use epics_rs::motor::device_support::AxisReport;

use crate::bincomm::Span;
use crate::comm::SpiiPlusComm;
use crate::profile::{PulseParams, SpiiProfile, axes_to_string};
use crate::run::{ProfileThread, RunControl, profile_thread};

/// Port EOS set at connect: C sets it in ACS_Motion.iocsh:28-29 and
/// SPiiPlusCommDriver.cpp:425-426 restores it after every binary poll.
pub const INPUT_EOS: &[u8] = b"\r";
pub const OUTPUT_EOS: &[u8] = b"\r";

/// Motor-status (`MST`) bit masks (C `SPiiPlusDriver::poll`).
const MST_ENABLED: i32 = 1 << 0;
const MST_MOTION: i32 = 1 << 5;

/// Axis-status (`AST`) motion bit, used for dummy/virtual axes (C `poll`).
const AST_MOTION: i32 = 1 << 5;
/// C `SPIIPLUS_AXIS_STATUS_PEGREADY`.
const AST_PEG_READY: i32 = 1 << 4;

/// Fault-word (`FAULT`) bit masks (C `SPIIPLUS_FAULT_*`). NOTE: bit 0 is the
/// RIGHT (high) limit and bit 1 is the LEFT (low) limit — the opposite ordering
/// to the older motorAcsTech80 module; follow this module's own definitions.
const FAULT_HARD_RIGHT_LIMIT: i32 = 1 << 0;
const FAULT_HARD_LEFT_LIMIT: i32 = 1 << 1;

/// Motor-flags (`MFLAGS`) bit masks (C `SPIIPLUS_MFLAGS_*`).
const MFLAGS_DUMMY: i32 = 1 << 0;
const MFLAGS_OPEN: i32 = 1 << 1;
const MFLAGS_MICRO: i32 = 1 << 2;
const MFLAGS_HOME: i32 = 1 << 3;
const MFLAGS_STEPPER: i32 = 1 << 4;
const MFLAGS_ENCLOOP: i32 = 1 << 5;
const MFLAGS_STEPENC: i32 = 1 << 6;
const MFLAGS_BRUSHL: i32 = 1 << 8;
const MFLAGS_BRUSHOK: i32 = 1 << 9;
const MFLAGS_PHASE2: i32 = 1 << 10;
const MFLAGS_DEFCON: i32 = 1 << 17;
const MFLAGS_LINEAR: i32 = 1 << 21;
const MFLAGS_ABSCOMM: i32 = 1 << 22;
const MFLAGS_HALL: i32 = 1 << 27;

/// mbbo homing-method selectors (C `MBBO_HOME_*`), `SPiiPlusHoming.db`'s
/// `homingMethod` values.
pub const HOME_NONE: i32 = 0;
pub const HOME_LIMIT_INDEX: i32 = 1;
pub const HOME_LIMIT: i32 = 2;
pub const HOME_INDEX: i32 = 3;
pub const HOME_CURRENT_POS: i32 = 4;
pub const HOME_HARDSTOP_INDEX: i32 = 5;
pub const HOME_HARDSTOP: i32 = 6;

/// SPiiPlus controller homing-method codes (C `SPIIPLUS_HOME_*`).
const SP_HOME_NEG_LIMIT_INDEX: i32 = 1;
const SP_HOME_POS_LIMIT_INDEX: i32 = 2;
const SP_HOME_NEG_LIMIT: i32 = 17;
const SP_HOME_POS_LIMIT: i32 = 18;
const SP_HOME_NEG_INDEX: i32 = 33;
const SP_HOME_POS_INDEX: i32 = 34;
const SP_HOME_CURRENT_POS: i32 = 37;
const SP_HOME_NEG_HARDSTOP_INDEX: i32 = 50;
const SP_HOME_POS_HARDSTOP_INDEX: i32 = 51;
const SP_HOME_NEG_HARDSTOP: i32 = 52;
const SP_HOME_POS_HARDSTOP: i32 = 53;

/// Resolve an mbbo homing-method selector and the record's home direction to the
/// SPiiPlus `HOME` method code (C `home()` switch). Returns `None` for methods
/// that produce `SPIIPLUS_HOME_NONE` (none/custom/unknown), for which the C
/// driver refuses to home.
fn home_code(method: i32, forward: bool) -> Option<i32> {
    let code = match method {
        HOME_LIMIT_INDEX if forward => SP_HOME_POS_LIMIT_INDEX,
        HOME_LIMIT_INDEX => SP_HOME_NEG_LIMIT_INDEX,
        HOME_LIMIT if forward => SP_HOME_POS_LIMIT,
        HOME_LIMIT => SP_HOME_NEG_LIMIT,
        HOME_INDEX if forward => SP_HOME_POS_INDEX,
        HOME_INDEX => SP_HOME_NEG_INDEX,
        HOME_CURRENT_POS => SP_HOME_CURRENT_POS,
        HOME_HARDSTOP_INDEX if forward => SP_HOME_POS_HARDSTOP_INDEX,
        HOME_HARDSTOP_INDEX => SP_HOME_NEG_HARDSTOP_INDEX,
        HOME_HARDSTOP if forward => SP_HOME_POS_HARDSTOP,
        HOME_HARDSTOP => SP_HOME_NEG_HARDSTOP,
        _ => SP_HOME_NONE,
    };
    (code != SP_HOME_NONE).then_some(code)
}

const SP_HOME_NONE: i32 = 0;

pub(crate) fn spiiplus_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// A double in a command as C's `std::stringstream << double` writes it:
/// `%g`, six significant digits.
pub(crate) fn fmt(v: f64) -> String {
    fmt_g(v, 6, false, false)
}

/// One axis's `SPIIPLUS_HOMING_*` parameters (C `home()` reads them at each
/// home).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Homing {
    pub method: i32,
    pub max_dist: f64,
    pub offset_pos: f64,
    pub offset_neg: f64,
    pub curr_limit: f64,
}

impl Homing {
    /// C `home()`'s `HOME axis,method,vel,[maxdist],offset[,currlimit]`: a
    /// zero max distance leaves its field empty and a zero current limit is
    /// omitted, since the controller takes either literally.
    fn command(&self, axis: usize, code: i32, velocity: f64, forward: bool) -> String {
        let max_dist = match self.max_dist {
            0.0 => String::new(),
            d => fmt(d),
        };
        let offset = if forward {
            self.offset_pos
        } else {
            self.offset_neg
        };
        let mut cmd = format!(
            "HOME {axis},{code},{},{max_dist},{}",
            fmt(velocity),
            fmt(offset)
        );
        if self.curr_limit != 0.0 {
            cmd += &format!(",{}", fmt(self.curr_limit));
        }
        cmd
    }
}

/// What C's constructor sets on each `SPiiPlusAxis`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct AxisSetup {
    /// C `virtual_`, from the config `virtualAxisList`.
    is_virtual: bool,
    /// C `dummy_`, the `MFLAGS` `#DUMMY` bit.
    dummy: bool,
    /// C `resolution_`: controller units per step.
    resolution: f64,
    /// The constructor's `MFLAGS`, which C splits into `dummy_` .. `hall_`.
    flags: i32,
}

/// What an axis's poll last saw, for the profile thread.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct AxisMotion {
    /// C `moving_`.
    moving: bool,
    /// C `pegReady_`, the `AST` `#PEGREADY` bit.
    peg_ready: bool,
}

/// Shared controller endpoint: owns the comm, the version string, the
/// axis count, the setup flags, and each axis's homing parameters.
pub struct SpiiPlusController {
    comm: SpiiPlusComm,
    /// C `pasynUserSelf`'s port, the `AcsMotionConfig` port name.
    port: String,
    ident: String,
    num_axes: usize,
    /// C `encoderFactor_`.
    encoder_factor: Vec<f64>,
    /// C `encoder2Factor_`.
    encoder2_factor: Vec<f64>,
    /// C `encoderType_`.
    encoder_type: Vec<i32>,
    /// C `encoder2Type_`.
    encoder2_type: Vec<i32>,
    /// C `movingPollPeriod_` and `idlePollPeriod_`, seconds.
    poll_periods: (f64, f64),
    /// C `virtualFeedbackPositionSupported_`: the firmware has `VPOS`.
    vpos_supported: bool,
    setup: Vec<AxisSetup>,
    motion: Vec<AxisMotion>,
    homing: Vec<Homing>,
    polled: Polled,
}

/// The arrays C's controller `poll()` reads that the axis polls parse,
/// each kept from the last poll that read it.
#[derive(Debug, Clone, Default)]
struct Polled {
    /// `APOS`, C `axisPosition_`.
    axis_position: Vec<f64>,
    /// `RPOS`, C `referencePosition_`.
    reference_position: Vec<f64>,
    /// `EPOS`, C `encoderPosition_`.
    encoder_position: Vec<f64>,
    /// `F2POS`, C `feedback2Position_`.
    feedback2_position: Vec<f64>,
    /// `ROFFS`, C `referenceOffset_`.
    reference_offset: Vec<f64>,
    /// `EOFFS`, C `encoderOffset_`.
    encoder_offset: Vec<f64>,
    /// `E2OFFS`, C `encoder2Offset_`.
    encoder2_offset: Vec<f64>,
    /// `XVEL`, C `maxVelocity_`.
    max_velocity: Vec<f64>,
    /// `XACC`, C `maxAcceleration_`.
    max_acceleration: Vec<f64>,
    /// `FPOS`, C `feedbackPosition_`.
    feedback_position: Vec<f64>,
    /// `VPOS`, C `virtualFeedbackPosition_`.
    virtual_feedback_position: Vec<f64>,
    /// `FVEL`, C `feedbackVelocity_`.
    feedback_velocity: Vec<f64>,
    /// `AST`, C `axisStatus_`.
    axis_status: Vec<i32>,
    /// `MST`, C `motorStatus_`.
    motor_status: Vec<i32>,
    /// `FAULT`, C `faultStatus_`.
    fault_status: Vec<i32>,
    /// `MFLAGS`, C `motorFlags_`.
    motor_flags: Vec<i32>,
}

impl SpiiPlusController {
    /// C `SPiiPlusController` constructor's queries: the firmware version
    /// (`?VR`), whether `VPOS` exists, and the setup arrays read over the
    /// binary protocol, which give each axis its `resolution_`;
    /// `virtual_axes` is the config's `virtualAxisList`. C ignores every
    /// status here, so a controller that does not answer still gets its
    /// axes, with zeroed setup. Performs blocking I/O.
    pub fn new(
        comm: SpiiPlusComm,
        port: &str,
        num_axes: usize,
        virtual_axes: &HashSet<usize>,
    ) -> Self {
        let ident = comm.write_read_str("?VR").unwrap_or_default();
        let vpos_supported = comm.is_variable_defined("VPOS").unwrap_or(false);
        let span = Span::one(0, num_axes as i32 - 1);
        let ints = |var| fill(comm.get_integer_array(var, span), num_axes);
        let doubles = |var| fill(comm.get_double_array(var, span), num_axes);
        let motor_flags = ints("MFLAGS");
        let stepper_factor = doubles("STEPF");
        let encoder_factor = doubles("EFAC");
        let encoder2_factor = doubles("E2FAC");
        let encoder_type = ints("E_TYPE");
        let encoder2_type = ints("E2_TYPE");
        for (index, kind) in encoder_type.iter().enumerate() {
            // An absolute encoder's position is valid once its error is
            // cleared.
            if *kind > 4 {
                let _ = comm.write_read_ack(&format!("FCLEAR {index}"));
            }
        }
        let setup = (0..num_axes)
            .map(|index| {
                let flags = motor_flags[index];
                let is_virtual = virtual_axes.contains(&index);
                let stepper = flags & MFLAGS_MICRO != 0
                    || (flags & MFLAGS_STEPPER != 0 && flags & MFLAGS_STEPENC == 0);
                let resolution = if !is_virtual && stepper {
                    stepper_factor[index]
                } else {
                    encoder_factor[index]
                };
                AxisSetup {
                    is_virtual,
                    dummy: flags & MFLAGS_DUMMY != 0,
                    resolution,
                    flags,
                }
            })
            .collect();
        Self {
            comm,
            port: port.to_string(),
            ident,
            num_axes,
            encoder_factor,
            encoder2_factor,
            encoder_type,
            encoder2_type,
            poll_periods: (0.0, 0.0),
            vpos_supported,
            setup,
            motion: vec![AxisMotion::default(); num_axes],
            homing: vec![Homing::default(); num_axes],
            polled: Polled {
                axis_position: vec![0.0; num_axes],
                reference_position: vec![0.0; num_axes],
                encoder_position: vec![0.0; num_axes],
                feedback2_position: vec![0.0; num_axes],
                reference_offset: vec![0.0; num_axes],
                encoder_offset: vec![0.0; num_axes],
                encoder2_offset: vec![0.0; num_axes],
                max_velocity: vec![0.0; num_axes],
                max_acceleration: vec![0.0; num_axes],
                feedback_position: vec![0.0; num_axes],
                virtual_feedback_position: vec![0.0; num_axes],
                feedback_velocity: vec![0.0; num_axes],
                axis_status: vec![0; num_axes],
                motor_status: vec![0; num_axes],
                fault_status: vec![0; num_axes],
                // C's poll reads into the array the constructor filled.
                motor_flags,
            },
        }
    }

    /// C `SPiiPlusController::poll` (SPiiPlusDriver.cpp:547-635): every
    /// axis's position, status and limits in one binary read each, for the
    /// axes' polls to parse. The first failed read ends the poll, leaving
    /// it and the arrays after it as the last poll read them. The arrays
    /// only C's parameters use are read and dropped.
    pub fn poll(&mut self) -> AsynResult<()> {
        self.trace_flow(format_args!("POLL_START"));
        let span = Span::one(0, self.num_axes as i32 - 1);
        let comm = &self.comm;
        let polled = &mut self.polled;
        polled.axis_position = comm.get_double_array("APOS", span)?;
        polled.reference_position = comm.get_double_array("RPOS", span)?;
        polled.encoder_position = comm.get_double_array("EPOS", span)?;
        polled.feedback_position = comm.get_double_array("FPOS", span)?;
        polled.feedback2_position = comm.get_double_array("F2POS", span)?;
        if self.vpos_supported && self.setup.iter().any(|s| s.is_virtual) {
            polled.virtual_feedback_position = comm.get_double_array("VPOS", span)?;
        }
        polled.feedback_velocity = comm.get_double_array("FVEL", span)?;
        polled.reference_offset = comm.get_double_array("ROFFS", span)?;
        polled.encoder_offset = comm.get_double_array("EOFFS", span)?;
        polled.encoder2_offset = comm.get_double_array("E2OFFS", span)?;
        comm.get_double_array("E_AOFFS", span)?;
        polled.axis_status = comm.get_integer_array("AST", span)?;
        polled.motor_status = comm.get_integer_array("MST", span)?;
        polled.fault_status = comm.get_integer_array("FAULT", span)?;
        polled.motor_flags = comm.get_integer_array("MFLAGS", span)?;
        comm.get_integer_array("MFLAGSX", span)?;
        polled.max_velocity = comm.get_double_array("XVEL", span)?;
        polled.max_acceleration = comm.get_double_array("XACC", span)?;
        self.trace_flow(format_args!("POLL_END"));
        Ok(())
    }

    fn trace_flow(&self, args: std::fmt::Arguments<'_>) {
        PortManager::global().trace_manager().print(
            Some(&self.port),
            Some(0),
            0,
            TraceMask::FLOW,
            format_args!("SPiiPlusController:poll: {args}\n"),
        );
    }

    /// The firmware version string (`?VR`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes (from the config command).
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    /// The homing parameters of axis `axis`.
    pub fn homing(&self, axis: usize) -> Homing {
        self.homing[axis]
    }

    /// C `pComm_`.
    pub(crate) fn comm(&self) -> &SpiiPlusComm {
        &self.comm
    }

    /// Axis `axis`'s C `resolution_`.
    pub(crate) fn resolution(&self, axis: usize) -> f64 {
        self.setup[axis].resolution
    }

    /// The `AcsMotionConfig` port name.
    pub(crate) fn port(&self) -> &str {
        &self.port
    }

    /// Axis `axis`'s C `moving_`.
    pub(crate) fn is_moving(&self, axis: usize) -> bool {
        self.motion[axis].moving
    }

    /// Axis `axis`'s C `pegReady_`; `None` past the last axis.
    pub(crate) fn peg_ready(&self, axis: usize) -> Option<bool> {
        self.motion.get(axis).map(|m| m.peg_ready)
    }

    /// C `runProfile`'s `posData`: the position data collection records
    /// for axis `axis`. FPOS and PE stay zero on a dummy axis, so it
    /// records the desired position, as does a virtual axis without VPOS.
    pub(crate) fn recorded_position(&self, axis: usize) -> &'static str {
        let setup = self.setup[axis];
        if setup.dummy || (setup.is_virtual && !self.vpos_supported) {
            "APOS"
        } else if setup.is_virtual {
            "VPOS"
        } else {
            "FPOS"
        }
    }

    /// C `startPoller`'s periods, seconds.
    pub(crate) fn set_poll_periods(&mut self, moving: f64, idle: f64) {
        self.poll_periods = (moving, idle);
    }

    /// C `SPiiPlusController::report`'s own lines (SPiiPlusDriver.cpp:3089-3097).
    fn report(&self, out: &mut dyn std::fmt::Write) {
        let yes_no = |b| if b { "Yes" } else { "No" };
        let _ = write!(
            out,
            "====================================\n\
             SPiiPlus motor driver:\n    \
             asyn port: {}\n    \
             num axes: {}\n    \
             moving poll period: {}\n    \
             idle poll period: {}\n    \
             firmware version: {}\n    \
             virtual feedback position support: {}\n\n",
            self.port,
            self.num_axes,
            fmt_f(self.poll_periods.0, 6),
            fmt_f(self.poll_periods.1, 6),
            self.ident,
            yes_no(self.vpos_supported),
        );
    }

    /// C `SPiiPlusAxis::report` (SPiiPlusDriver.cpp:3029-3083). C prints
    /// `moving_`, which its poll sets from the previous axis's result; this
    /// prints the axis's own.
    fn axis_report(&self, a: usize, out: &mut dyn std::fmt::Write) {
        let p = &self.polled;
        let setup = self.setup[a];
        let f = |values: &[f64]| fmt_f(values.get(a).copied().unwrap_or_default(), 6);
        let i = |values: &[i32]| values.get(a).copied().unwrap_or_default();
        let _ = writeln!(out, "Configuration for axis {a}:");
        let _ = writeln!(out, "  mflags: {}", i(&p.motor_flags));
        for (name, mask) in [
            ("dummy:  ", MFLAGS_DUMMY),
            ("open:  ", MFLAGS_OPEN),
            ("micro:  ", MFLAGS_MICRO),
            ("homed:  ", MFLAGS_HOME),
            ("stepper:  ", MFLAGS_STEPPER),
            ("encloop:  ", MFLAGS_ENCLOOP),
            ("stepenc:  ", MFLAGS_STEPENC),
            ("brushl:  ", MFLAGS_BRUSHL),
            ("brushok  ", MFLAGS_BRUSHOK),
            ("phase2:  ", MFLAGS_PHASE2),
            ("defcon:  ", MFLAGS_DEFCON),
            ("linear:  ", MFLAGS_LINEAR),
            ("abscomm:  ", MFLAGS_ABSCOMM),
            ("hall:  ", MFLAGS_HALL),
        ] {
            // C stores the masked bit, not 0/1.
            let _ = writeln!(out, "    {name}{}", setup.flags & mask);
        }
        let _ = writeln!(out, "  resolution: {}", fmt_e(setup.resolution, 6, false));
        let _ = writeln!(out, "  reference offset: {}", f(&p.reference_offset));
        let _ = writeln!(out, "  homing method: {}", self.homing[a].method);
        let _ = writeln!(out, "  max velocity: {}", f(&p.max_velocity));
        let _ = writeln!(out, "  max acceleration: {}", f(&p.max_acceleration));
        let virtual_ = if setup.is_virtual { "Yes" } else { "No" };
        let _ = writeln!(out, "  virtual: {virtual_}");
        let _ = writeln!(out, "Encoder info for axis {a}:");
        let _ = writeln!(out, "  encoder type: {}", i(&self.encoder_type));
        let e = |values: &[f64]| fmt_e(values.get(a).copied().unwrap_or_default(), 6, false);
        let _ = writeln!(out, "  encoder resolution: {}", e(&self.encoder_factor));
        let _ = writeln!(out, "  encoder offset: {}", f(&p.encoder_offset));
        let _ = writeln!(out, "  encoder 2 type: {}", i(&self.encoder2_type));
        let _ = writeln!(out, "  encoder 2 resolution: {}", e(&self.encoder2_factor));
        let _ = writeln!(out, "  encoder 2 offset: {}", f(&p.encoder2_offset));
        let _ = writeln!(out, "Position info for axis {a}:");
        let _ = writeln!(out, "  axis position: {}", f(&p.axis_position));
        let _ = writeln!(out, "  reference position: {}", f(&p.reference_position));
        let _ = writeln!(out, "  encoder position: {}", f(&p.encoder_position));
        let _ = writeln!(out, "  feedback position: {}", f(&p.feedback_position));
        let _ = writeln!(out, "  feedback 2 position: {}", f(&p.feedback2_position));
        let _ = writeln!(
            out,
            "  virtual feedback position: {}",
            f(&p.virtual_feedback_position)
        );
        let _ = writeln!(out, "Status for axis {a}:");
        let _ = writeln!(out, "  moving: {}", i32::from(self.motion[a].moving));
        let _ = writeln!(out, "  axis status: {}", i(&p.axis_status));
        let _ = writeln!(out, "  motor status: {}", i(&p.motor_status));
        let _ = writeln!(out, "  fault status: {}", i(&p.fault_status));
        let _ = writeln!(out);
    }

    /// Send a set/motion command (C `writeReadAck`).
    fn command(&self, cmd: &str) -> AsynResult<()> {
        self.comm.write_read_ack(cmd)
    }
}

/// A setup array of `n` axes; a failed or short read leaves zeros.
fn fill<T: Default + Clone>(read: AsynResult<Vec<T>>, n: usize) -> Vec<T> {
    let mut values = read.unwrap_or_default();
    values.resize(n, T::default());
    values
}

/// One SPiiPlus axis sharing a controller. Implements [`AsynMotor`].
pub struct SpiiPlusAxis {
    controller: Arc<Mutex<SpiiPlusController>>,
    /// 0-based axis index (used directly on the wire).
    axis: usize,
    /// True for a virtual axis (from the config `virtualAxisList`) or a dummy
    /// axis (MFLAGS `#DUMMY` bit, read once at construction). Reduced axes have
    /// no limits/faults and take motion from `AST`.
    reduced: bool,
    /// True for a dummy axis (MFLAGS `#DUMMY`).
    dummy: bool,
    /// C `resolution_`: controller units per step.
    resolution: f64,
    /// True only for a config-listed virtual axis (home / set-position error, as
    /// in C, independent of the dummy path).
    is_virtual: bool,
    /// Last non-zero feedback-velocity direction (C updates direction only while
    /// the axis is moving); 0 until then, as asynMotorAxis's status starts.
    last_direction: bool,
}

impl SpiiPlusAxis {
    /// Construct axis `index` (0-based) with the setup the controller's
    /// constructor gave it.
    pub fn new(controller: Arc<Mutex<SpiiPlusController>>, index: usize) -> Self {
        let setup = controller.lock().unwrap_or_else(|e| e.into_inner()).setup[index];
        Self {
            controller,
            axis: index,
            reduced: setup.dummy || setup.is_virtual,
            dummy: setup.dummy,
            resolution: setup.resolution,
            is_virtual: setup.is_virtual,
            last_direction: false,
        }
    }

    fn lock(&self) -> MutexGuard<'_, SpiiPlusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Steps (or steps/s, steps/s²) in controller units.
    fn units(&self, steps: f64) -> f64 {
        steps * self.resolution
    }

    /// Send the ACC/DEC/VEL preamble of a point-to-point move (C `move`).
    fn set_move_params(
        &self,
        ctrl: &SpiiPlusController,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let a = self.axis;
        ctrl.command(&format!("ACC({a})={}", fmt(self.units(acceleration))))?;
        ctrl.command(&format!("DEC({a})={}", fmt(self.units(acceleration))))?;
        ctrl.command(&format!("VEL({a})={}", fmt(self.units(velocity))))?;
        Ok(())
    }
}

impl AsynMotor for SpiiPlusAxis {
    /// C `SPiiPlusAxis::poll` copies the controller's last read and sets no
    /// status bit when `SPiiPlusController::poll` fails.
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
        let ctrl = self.lock();
        let a = self.axis;
        self.set_move_params(&ctrl, velocity, acceleration)?;
        ctrl.command(&format!("PTP {a}, {}", fmt(self.units(position))))?;
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
        let a = self.axis;
        self.set_move_params(&ctrl, velocity, acceleration)?;
        ctrl.command(&format!("PTP/r {a}, {}", fmt(self.units(distance))))?;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        let a = self.axis;
        // C moveVelocity sets ACC/DEC then JOG/v with the |velocity| magnitude
        // and a separate direction sign (it does not change the normal VEL).
        ctrl.command(&format!("ACC({a})={}", fmt(self.units(acceleration))))?;
        ctrl.command(&format!("DEC({a})={}", fmt(self.units(acceleration))))?;
        let dir = if velocity > 0.0 { '+' } else { '-' };
        let speed = self.units(velocity.abs());
        ctrl.command(&format!("JOG/v {a}, {}, {dir}", fmt(speed)))?;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        if self.is_virtual {
            return Err(spiiplus_err(format!(
                "SPiiPlus: axis {} home not supported because axis is virtual",
                self.axis
            )));
        }
        let ctrl = self.lock();
        let a = self.axis;
        let homing = ctrl.homing(a);
        let Some(code) = home_code(homing.method, forward) else {
            // C prints and returns asynSuccess.
            PortManager::global().trace_manager().print(
                Some(&ctrl.port),
                Some(0),
                0,
                TraceMask::ERROR,
                format_args!("SPiiPlusController:home: No homing method selected for axis {a}\n"),
            );
            return Ok(());
        };
        ctrl.command(&homing.command(a, code, self.units(velocity), forward))?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.command(&format!("HALT {}", self.axis))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        if self.is_virtual {
            return Err(spiiplus_err(format!(
                "SPiiPlus: axis {} position change not supported because axis is virtual",
                self.axis
            )));
        }
        let ctrl = self.lock();
        // C setPosition writes RPOS; the controller auto-updates APOS/FPOS.
        let rpos = self.units(position);
        ctrl.command(&format!("SET RPOS({})={}", self.axis, fmt(rpos)))?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C skips enable/disable entirely for dummy and virtual axes.
        if self.reduced {
            return Ok(());
        }
        let ctrl = self.lock();
        let a = self.axis;
        if enable {
            ctrl.command(&format!("ENABLE {a}"))?;
        } else {
            ctrl.command(&format!("DISABLE {a}"))?;
        }
        Ok(())
    }

    /// C `SPiiPlusAxis::poll` (SPiiPlusDriver.cpp:751-866): what the
    /// controller's [`SpiiPlusController::poll`] read, with no I/O of its own.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let mut ctrl = self.lock();
        let a = self.axis;
        let polled = &ctrl.polled;
        let at = |values: &[f64]| values.get(a).copied().unwrap_or_default();
        let bits = |values: &[i32]| values.get(a).copied().unwrap_or_default();

        let apos = at(&polled.axis_position) / self.resolution;
        let velocity = at(&polled.feedback_velocity) / self.resolution;
        let ast = bits(&polled.axis_status);

        let (moving, powered, high_limit, low_limit, homed, encoder) = if self.reduced {
            // Dummy/virtual: motion from AST, no faults; the encoder is the
            // commanded position, or VPOS for a virtual axis that has it.
            let encoder = if !self.dummy && ctrl.vpos_supported {
                at(&polled.virtual_feedback_position) / ctrl.encoder_factor[a]
            } else {
                apos
            };
            ((ast & AST_MOTION) != 0, false, false, false, false, encoder)
        } else {
            let mst = bits(&polled.motor_status);
            let fault = bits(&polled.fault_status);
            let mflags = bits(&polled.motor_flags);
            (
                (mst & MST_MOTION) != 0,
                (mst & MST_ENABLED) != 0,
                (fault & FAULT_HARD_RIGHT_LIMIT) != 0,
                (fault & FAULT_HARD_LEFT_LIMIT) != 0,
                (mflags & MFLAGS_HOME) != 0,
                at(&polled.feedback_position) / ctrl.encoder_factor[a],
            )
        };
        // C stores the poller's `moving` from before this axis's poll, the
        // previous axis's; the profile thread's wait needs this axis's.
        ctrl.motion[a] = AxisMotion {
            moving,
            peg_ready: ast & AST_PEG_READY != 0,
        };
        drop(ctrl);

        // C updates the direction bit only while moving and only on a non-zero
        // feedback velocity; otherwise it keeps the previous direction.
        if moving {
            if velocity > 0.0 {
                self.last_direction = true;
            } else if velocity < 0.0 {
                self.last_direction = false;
            }
        }

        Ok(MotorStatus {
            position: apos,
            encoder_position: encoder,
            velocity,
            done: !moving,
            moving,
            msta_moving: moving,
            high_limit,
            low_limit,
            direction: self.last_direction,
            powered,
            homed,
            has_encoder: true,
            gain_support: true,
            ..MotorStatus::default()
        })
    }
}

/// C `SPiiPlusController`'s homing parameters (SPiiPlusDriver.cpp:73,
/// :113-116), per axis.
#[derive(Clone, Copy, Debug)]
struct HomingParams {
    method: usize,
    max_dist: usize,
    offset_pos: usize,
    offset_neg: usize,
    curr_limit: usize,
}

/// C `SPiiPlusController`'s parameters and profile on the controller
/// port: the homing parameters, stored per axis for `home()` to read, and
/// the profile move.
pub struct AcsMotionParams {
    controller: Arc<Mutex<SpiiPlusController>>,
    params: Option<HomingParams>,
    pub(crate) pulse: Option<PulseParams>,
    pub(crate) profile: SpiiProfile,
    run: RunControl,
}

impl AcsMotionParams {
    /// The parameters, and the profile thread that runs their profile
    /// once [`ProfileThread::spawn`] starts it on their port.
    pub fn new(controller: Arc<Mutex<SpiiPlusController>>) -> (Self, ProfileThread) {
        let (run, thread) = profile_thread(controller.clone());
        let params = Self {
            controller,
            params: None,
            pulse: None,
            profile: SpiiProfile::default(),
            run,
        };
        (params, thread)
    }

    pub(crate) fn lock(&self) -> MutexGuard<'_, SpiiPlusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn set(&self, addr: i32, set: impl FnOnce(&mut Homing)) {
        set(&mut self.lock().homing[addr as usize]);
    }
}

impl ControllerParams for AcsMotionParams {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        self.params = Some(HomingParams {
            method: base.create_param("SPIIPLUS_HOMING_METHOD", ParamType::Int32)?,
            max_dist: base.create_param("SPIIPLUS_HOMING_MAX_DIST", ParamType::Float64)?,
            offset_pos: base.create_param("SPIIPLUS_HOMING_OFFSET_POS", ParamType::Float64)?,
            offset_neg: base.create_param("SPIIPLUS_HOMING_OFFSET_NEG", ParamType::Float64)?,
            curr_limit: base.create_param("SPIIPLUS_HOMING_CURR_LIMIT", ParamType::Float64)?,
        });
        let mode = base.create_param("SPIIPLUS_PULSE_MODE", ParamType::Int32)?;
        let pos = base.create_param("SPIIPLUS_PULSE_POS", ParamType::Float64Array)?;
        // C reads NUM_PULSES from the base's PROFILE_NUM_PULSES, never this.
        base.create_param("SPIIPLUS_NUM_PULSES", ParamType::Int32)?;
        let axis = base.create_param("SPIIPLUS_PULSE_AXIS", ParamType::Int32)?;
        self.pulse = Some(PulseParams {
            mode,
            pos,
            axis,
            peg_eng_enc_code: base.create_param("SPIIPLUS_PEG_ENG_ENC_CODE", ParamType::Octet)?,
            peg_out_assign_code: base
                .create_param("SPIIPLUS_PEG_OUT_ASSIGN_CODE", ParamType::Octet)?,
            pouts_output_index: base
                .create_param("SPIIPLUS_POUTS_OUTPUT_INDEX", ParamType::Int32)?,
            pouts_bit_code: base.create_param("SPIIPLUS_POUTS_BIT_CODE", ParamType::Octet)?,
            pulse_width: base.create_param("SPIIPLUS_PULSE_WIDTH", ParamType::Float64)?,
        });
        Ok(())
    }

    fn write_float64_array(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        user: &AsynUser,
        value: &[f64],
    ) -> AsynResult<()> {
        if self.is_pulse_positions(user.reason) {
            return self.write_pulse_positions(ctx, user, value);
        }
        ctx.write_float64_array(self, user, value)
    }

    fn define_profile(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        addr: i32,
        positions: &[f64],
    ) -> AsynResult<()> {
        self.define_axis_profile(ctx, addr, positions)
    }

    fn build_profile(&mut self, ctx: &mut ControllerCtx<'_>) -> AsynResult<()> {
        self.build(ctx)
    }

    fn readback_profile(&mut self, ctx: &mut ControllerCtx<'_>) -> AsynResult<()> {
        self.readback(ctx)
    }

    /// C `executeProfile`: the profile thread runs it; the base's axis
    /// execute is never called.
    fn execute_profile(&mut self, _: &mut ControllerCtx<'_>) -> AsynResult<()> {
        self.run.signal();
        Ok(())
    }

    /// C `abortProfile`: halt a running profile's axes and have the
    /// profile thread stop at its next check. C does not call the base.
    fn abort_profile(&mut self, ctx: &mut ControllerCtx<'_>) -> AsynResult<()> {
        let state = ctx
            .base
            .get_int32_param(index::PROFILE_EXECUTE_STATE, 0)
            .unwrap_or_default();
        if state == PROFILE_EXECUTE_DONE {
            return Ok(());
        }
        let halt = format!("HALT {}", axes_to_string(&self.profile.profile_axes));
        let status = self.lock().comm().write_read_ack(&halt);
        self.run.halted.store(true, Ordering::SeqCst);
        status
    }

    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: i32,
    ) -> ParamWrite {
        self.write_int32_own(base, user, value).into()
    }

    fn write_float64(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: f64,
    ) -> ParamWrite {
        self.write_float64_own(base, user, value).into()
    }

    /// C `SPiiPlusController::report`: its own lines, then at level > 0 the
    /// base's at level - 1 (SPiiPlusDriver.cpp:3087-3104).
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        self.lock().report(out);
        if level > 0 {
            ctx.report(out, level - 1);
        }
        let _ = writeln!(out, "====================================");
    }
}

/// C `SPiiPlusAxis::report` for axis `axis`, which the controller's report
/// runs.
pub struct SpiiPlusAxisReport {
    pub controller: Arc<Mutex<SpiiPlusController>>,
    pub axis: usize,
}

impl AxisReport for SpiiPlusAxisReport {
    fn report(&self, out: &mut dyn std::fmt::Write, _level: i32) {
        let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        ctrl.axis_report(self.axis, out);
    }
}

/// The driver's own write code, which then passes to the base.
impl AcsMotionParams {
    fn write_int32_own(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: i32,
    ) -> AsynResult<()> {
        if let Some(p) = self.params
            && user.reason == p.method
        {
            self.set(user.addr, |h| h.method = value);
            // C prints on pasynUserSelf, which is connected to address 0.
            PortManager::global().trace_manager().print(
                Some(&base.port_name),
                Some(0),
                0,
                TraceMask::IO_DRIVER,
                format_args!("SPiiPlusController:writeInt32: homingMethod = {value}\n"),
            );
        }
        Ok(())
    }
    fn write_float64_own(
        &mut self,
        _: &mut PortDriverBase,
        user: &AsynUser,
        value: f64,
    ) -> AsynResult<()> {
        let Some(p) = self.params else {
            return Ok(());
        };
        self.set(user.addr, |h| match user.reason {
            r if r == p.max_dist => h.max_dist = value,
            r if r == p.offset_pos => h.offset_pos = value,
            r if r == p.offset_neg => h.offset_neg = value,
            r if r == p.curr_limit => h.curr_limit = value,
            _ => {}
        });
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::trace::TraceFile;

    use super::*;
    use crate::profile::tests::{SERVOS, rig};

    #[test]
    fn a_homing_method_write_is_traced_as_c_traces_it() {
        let rig = rig("spiiHomingTrace", 2, SERVOS);
        let method = rig.param("SPIIPLUS_HOMING_METHOD");
        let path = std::env::temp_dir().join(format!("spii-trace-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some("spiiHomingTraceCtrl"), TraceMask::IO_DRIVER);
        trace.set_trace_file(Some("spiiHomingTraceCtrl"), TraceFile::File(file.clone()));
        // An address's own slot keeps the file it had.
        for addr in 0..2 {
            trace.set_device_trace_file("spiiHomingTraceCtrl", addr, TraceFile::File(file.clone()));
        }
        rig.port.write_int32_blocking(method, 1, 3).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let mine: Vec<_> = text
            .lines()
            .filter(|l| l.contains("SPiiPlusController"))
            .collect();
        assert_eq!(mine.len(), 1, "{text}");
        assert!(
            text.contains("SPiiPlusController:writeInt32: homingMethod = 3\n"),
            "{text}"
        );
        assert_eq!(rig.ctrl.lock().unwrap().homing(1).method, 3);
    }

    /// One poller pass over `axis`: the controller poll, then the axis's.
    fn pass(ctrl: &Arc<Mutex<SpiiPlusController>>, axis: &mut SpiiPlusAxis) -> MotorStatus {
        ctrl.lock().unwrap().poll().unwrap();
        axis.poll(&AsynUser::default()).unwrap()
    }

    /// C's controller poll reads every axis in binary, in its order, and
    /// the axis polls only parse what it read (SPiiPlusDriver.cpp:547-635,
    /// :751-866).
    #[test]
    fn the_controller_poll_reads_every_axis_and_the_axis_poll_sends_nothing() {
        let (ctrl, line) = fake("spiiBulk", 2, &[("EFAC", &[1.0, 1.0])]);
        line.lock().unwrap().set("APOS", &[3.0, 4.0]);
        let mut axes = [0, 1].map(|i| SpiiPlusAxis::new(ctrl.clone(), i));
        line.lock().unwrap().sent.clear();
        ctrl.lock().unwrap().poll().unwrap();
        let reads: Vec<String> = [
            "APOS", "RPOS", "EPOS", "FPOS", "F2POS", "FVEL", "ROFFS", "EOFFS", "E2OFFS", "E_AOFFS",
            "AST", "MST", "FAULT", "MFLAGS", "MFLAGSX", "XVEL", "XACC",
        ]
        .iter()
        .map(|var| format!("BIN ??{var}(0,1)(0,0)"))
        .collect();
        assert_eq!(line.lock().unwrap().sent, reads);
        line.lock().unwrap().sent.clear();
        let positions = axes
            .each_mut()
            .map(|axis| axis.poll(&AsynUser::default()).unwrap().position);
        assert_eq!(positions, [3.0, 4.0]);
        assert!(line.lock().unwrap().sent.is_empty());
    }

    /// A controller of `axes` axes on a fake line, its setup arrays given.
    fn fake(
        name: &str,
        axes: usize,
        setup: &[(&str, &[f64])],
    ) -> (
        Arc<Mutex<SpiiPlusController>>,
        Arc<Mutex<crate::fake::Controller>>,
    ) {
        fake_virtual(name, axes, setup, &[])
    }

    /// [`fake`] with the config's virtual axes.
    fn fake_virtual(
        name: &str,
        axes: usize,
        setup: &[(&str, &[f64])],
        virtual_axes: &[usize],
    ) -> (
        Arc<Mutex<SpiiPlusController>>,
        Arc<Mutex<crate::fake::Controller>>,
    ) {
        let line = crate::fake::line(name);
        for (var, values) in setup {
            line.lock().unwrap().set(var, values);
        }
        let comm = SpiiPlusComm::connect(&format!("{name}Comm"), name).unwrap();
        let virtual_axes = virtual_axes.iter().copied().collect();
        let ctrl = SpiiPlusController::new(comm, name, axes, &virtual_axes);
        (Arc::new(Mutex::new(ctrl)), line)
    }

    #[test]
    fn the_constructor_reads_setup_in_binary_and_clears_absolute_encoders() {
        let (_, line) = fake("spiiCtor", 2, &[("E_TYPE", &[0.0, 5.0])]);
        assert_eq!(
            line.lock().unwrap().sent,
            [
                "?VR",
                "?VPOS",
                "BIN ??MFLAGS(0,1)(0,0)",
                "BIN ??STEPF(0,1)(0,0)",
                "BIN ??EFAC(0,1)(0,0)",
                "BIN ??E2FAC(0,1)(0,0)",
                "BIN ??E_TYPE(0,1)(0,0)",
                "BIN ??E2_TYPE(0,1)(0,0)",
                "FCLEAR 1",
            ]
        );
    }

    #[test]
    fn a_microstepper_moves_and_polls_in_stepper_factor_units() {
        let (ctrl, line) = fake(
            "spiiStep",
            2,
            &[
                ("MFLAGS", &[f64::from(MFLAGS_MICRO), 0.0]),
                ("STEPF", &[0.5, 0.25]),
                ("EFAC", &[2.0, 4.0]),
                ("APOS", &[5.0]),
                ("FPOS", &[6.0]),
            ],
        );
        let mut axis = SpiiPlusAxis::new(ctrl.clone(), 0);
        line.lock().unwrap().sent.clear();
        let user = AsynUser::default();
        axis.move_absolute(&user, 10.0, 0.0, 4.0, 8.0).unwrap();
        assert_eq!(
            line.lock().unwrap().sent,
            ["ACC(0)=4", "DEC(0)=4", "VEL(0)=2", "PTP 0, 5"]
        );
        let status = pass(&ctrl, &mut axis);
        // APOS / STEPF, and FPOS / EFAC.
        assert_eq!((status.position, status.encoder_position), (10.0, 3.0));
    }

    #[test]
    fn a_servo_axis_uses_the_encoder_factor() {
        let (ctrl, line) = fake("spiiServo", 1, &[("STEPF", &[0.5]), ("EFAC", &[2.0])]);
        let mut axis = SpiiPlusAxis::new(ctrl, 0);
        line.lock().unwrap().sent.clear();
        axis.set_position(&AsynUser::default(), 3.0).unwrap();
        assert_eq!(line.lock().unwrap().sent, ["SET RPOS(0)=6"]);
    }

    #[test]
    fn a_virtual_axis_reads_vpos_when_the_firmware_has_it() {
        let (ctrl, _) = fake_virtual(
            "spiiVpos",
            1,
            &[
                ("VPOS", &[8.0]),
                ("EFAC", &[2.0]),
                ("STEPF", &[0.5]),
                ("MFLAGS", &[f64::from(MFLAGS_MICRO)]),
                ("APOS", &[1.0]),
            ],
            &[0],
        );
        // A virtual microstepper still takes the encoder factor.
        let mut axis = SpiiPlusAxis::new(ctrl.clone(), 0);
        let status = pass(&ctrl, &mut axis);
        assert_eq!((status.position, status.encoder_position), (0.5, 4.0));
    }

    #[test]
    fn direction_is_zero_until_a_move_runs_forward() {
        let (ctrl, line) = fake("spiiDir", 1, &[("EFAC", &[1.0]), ("FVEL", &[2.0])]);
        let mut axis = SpiiPlusAxis::new(ctrl.clone(), 0);
        // Stationary: C leaves motorStatusDirection_ unset.
        assert!(!pass(&ctrl, &mut axis).direction);
        line.lock().unwrap().set("MST", &[f64::from(MST_MOTION)]);
        assert!(pass(&ctrl, &mut axis).direction);
        line.lock().unwrap().set("MST", &[0.0]);
        line.lock().unwrap().set("FVEL", &[-2.0]);
        assert!(pass(&ctrl, &mut axis).direction);
    }

    #[test]
    fn motion_sets_the_msta_moving_bit() {
        let (ctrl, line) = fake("spiiMsta", 1, &[("EFAC", &[1.0])]);
        let mut axis = SpiiPlusAxis::new(ctrl.clone(), 0);
        assert!(!pass(&ctrl, &mut axis).msta_moving);
        line.lock().unwrap().set("MST", &[f64::from(MST_MOTION)]);
        assert!(pass(&ctrl, &mut axis).msta_moving);
    }

    #[test]
    fn each_poll_keeps_its_own_axis_motion_and_peg_ready_for_the_profile() {
        let (ctrl, line) = fake("spiiMotion", 2, &[("EFAC", &[1.0, 1.0])]);
        // Axis 0 moving with PEG ready, axis 1 still.
        line.lock()
            .unwrap()
            .set("MST", &[f64::from(MST_MOTION), 0.0]);
        line.lock()
            .unwrap()
            .set("AST", &[f64::from(AST_PEG_READY), 0.0]);
        let user = AsynUser::default();
        let mut axes = [0, 1].map(|i| SpiiPlusAxis::new(ctrl.clone(), i));
        ctrl.lock().unwrap().poll().unwrap();
        for axis in &mut axes {
            axis.poll(&user).unwrap();
        }
        assert_eq!(
            ctrl.lock().unwrap().motion,
            [
                AxisMotion {
                    moving: true,
                    peg_ready: true
                },
                AxisMotion::default()
            ]
        );
    }

    #[test]
    fn a_failed_or_short_setup_read_leaves_zeros() {
        assert_eq!(fill(Ok(vec![5, 6]), 3), vec![5, 6, 0]);
        assert_eq!(fill::<f64>(Err(spiiplus_err("down")), 2), vec![0.0, 0.0]);
    }

    #[test]
    fn home_code_resolves_method_and_direction() {
        // Limit+index maps to the positive/negative code by direction.
        assert_eq!(
            home_code(HOME_LIMIT_INDEX, true),
            Some(SP_HOME_POS_LIMIT_INDEX)
        );
        assert_eq!(
            home_code(HOME_LIMIT_INDEX, false),
            Some(SP_HOME_NEG_LIMIT_INDEX)
        );
        // Hardstop and plain-limit likewise.
        assert_eq!(home_code(HOME_HARDSTOP, true), Some(SP_HOME_POS_HARDSTOP));
        assert_eq!(home_code(HOME_LIMIT, false), Some(SP_HOME_NEG_LIMIT));
        // Current-position is direction independent.
        assert_eq!(home_code(HOME_CURRENT_POS, true), Some(SP_HOME_CURRENT_POS));
        assert_eq!(
            home_code(HOME_CURRENT_POS, false),
            Some(SP_HOME_CURRENT_POS)
        );
    }

    #[test]
    fn home_code_none_and_unknown_refuse() {
        assert_eq!(home_code(HOME_NONE, true), None);
        assert_eq!(home_code(7, true), None); // MBBO_HOME_CUSTOM -> SPIIPLUS_HOME_NONE
        assert_eq!(home_code(999, false), None);
    }

    #[test]
    fn home_command_leaves_zero_max_dist_empty_and_omits_zero_curr_limit() {
        let mut h = Homing {
            method: HOME_LIMIT_INDEX,
            offset_pos: 1.5,
            offset_neg: -2.0,
            ..Homing::default()
        };
        assert_eq!(h.command(3, 2, 10.0, true), "HOME 3,2,10,,1.5");
        assert_eq!(h.command(3, 1, 10.0, false), "HOME 3,1,10,,-2");
        h.max_dist = 50.0;
        h.curr_limit = 0.5;
        assert_eq!(h.command(0, 2, 10.0, true), "HOME 0,2,10,50,1.5,0.5");
    }

    #[test]
    fn fmt_writes_six_significant_digits_as_c_streams_a_double() {
        assert_eq!(fmt(1000.0), "1000");
        assert_eq!(fmt(12.5), "12.5");
        assert_eq!(fmt(-0.25), "-0.25");
        assert_eq!(fmt(1.0 / 3.0), "0.333333");
        assert_eq!(fmt(1234567.0), "1.23457e+06");
        assert_eq!(fmt(0.0000001), "1e-07");
    }
}
