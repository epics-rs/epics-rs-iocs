//! Phytron phyMOTION (MCM) and MCC-1/MCC-2 motor controller driver (ASCII with
//! STX/ETX framing, over an asyn octet port).
//!
//! Ported from `motorPhytron/phytronApp/src/phytronAxisMotor.cpp` (a model-3
//! `asynMotorController`/`asynMotorAxis` pair). Connects over a `drvAsynIPPort`
//! or `drvAsynSerialPort`.
//!
//! ## Wire protocol
//!
//! Every command is wrapped `STX <addr> <command> [":" <CRC>] ETX`:
//! - `<addr>` is one hex digit (the controller address 0..15, `0` for a
//!   phyMOTION).
//! - phyMOTION frames carry a trailing `:` plus a two-hex-digit XOR checksum
//!   over `<addr><command>:`; the MCC omits the CRC.
//! - The reply is `STX <ACK|NAK> <data> [":" <CRC>] ETX`. `ACK` (0x06) means
//!   success, `NAK` (0x15) failure. The driver owns the framing both ways,
//!   so the port sets no EOS: a reply without its ETX is invalid, as C's
//!   `find_first_of('\x03')` makes it, and an input EOS of ETX would strip it.
//!
//! ## Controller / axis addressing
//!
//! A phyMOTION axis is addressed `M<module>.<index>` (e.g. `M1.2`); an MCC axis
//! is a single digit `1`..`8`. Parameter *writes* use `=` on a phyMOTION and `S`
//! on an MCC (e.g. `M1.2P14=<v>` vs `1P14S<v>`); *reads* use `R` on both
//! (`M1.2P20R`). Axes are created one at a time (module, index) via
//! `phytronCreateAxis`, matching the C two-step configuration.
//!
//! ## Units
//!
//! Velocities are steps/s and accelerations steps/s² at the controller. The
//! asyn-rs `AsynMotor` boundary is dial-frame EGU; with the motor record's
//! `MRES` = 1 a step is one EGU, so the record's EGU/s velocity and EGU/s²
//! acceleration pass straight through (clamped to the controller's ranges:
//! velocity 1..500000, acceleration 4000..500000). Motor and encoder positions
//! are reported in controller-native steps. There is no resolution factor to
//! cancel: the driver boundary is controller-native steps with `MRES` = 1.
//!
//! ## Not modeled (documented)
//!
//! These belong to auxiliary records or iocsh commands outside the motor-record
//! boundary and are intentionally not ported; the driver runs with their C
//! defaults:
//!
//! - The fake-HOMED-bit feature (Phytron registers 1001..1020, enabled by the
//!   `fakeHomedEnable` option): disabled by default, so HOMED is reported from
//!   status bit 0x08.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PollFailure, PortSets, StatusWrite,
};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{AxisUser, ParamWrite, has_motor_axis, index};
use epics_rs::motor::trace::MotorTrace;

use epics_libcom_rs::cfmt::CFloat;
use epics_libcom_rs::runtime::epics_string::print_escaped;
use motor_common::util::{atof, atoi, nint};
use motor_common::{AxisReport, ControllerParams, ControllerReport};

use crate::io::IoChannel;

/// Reply read buffer size (C expects up to ~2010 bytes: 2000 data + framing).
const READ_BUF: usize = 2048;

const STX: u8 = 0x02;
const ETX: u8 = 0x03;
const ACK: u8 = 0x06;
const NAK: u8 = 0x15;

/// Controller velocity clamp (steps/s), phytronAxisMotor.h.
const MIN_VELOCITY: f64 = 1.0;
const MAX_VELOCITY: f64 = 500000.0;
/// Controller acceleration clamp (steps/s²), phytronAxisMotor.h.
const MIN_ACCELERATION: f64 = 4000.0;
const MAX_ACCELERATION: f64 = 500000.0;

/// C `MAXIMUM_BRAKE_TIME`, seconds (phytronAxisMotor.cpp:76).
const MAX_BRAKE_TIME: f64 = 10.0;

/// Controller family — selects framing, addressing and the parameter set-char.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CtrlType {
    /// phyMOTION MCM (CRC-framed, `M<mod>.<idx>` axes, `=` set-char).
    Phymotion,
    /// MCC-1 / MCC-2 (no CRC, single-digit axes, `S` set-char).
    Mcc,
}

fn phytron_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// C `phyToAsyn`: the two phytron codes past the asyn ones
/// (`phytronInvalidReturn` 6, `phytronInvalidCommand` 7) are an asyn error.
fn phy_to_asyn(code: i32) -> AsynError {
    let status = match code {
        1 => AsynStatus::Timeout,
        2 => AsynStatus::Overflow,
        4 => AsynStatus::Disconnected,
        5 => AsynStatus::Disabled,
        _ => AsynStatus::Error,
    };
    AsynError::Status {
        status,
        message: format!("phytron: error code {code}"),
    }
}

/// The axis parameters on a P register, in C's createParam order
/// (phytronAxisMotor.cpp:144-165); the map of readInt32 (:331-350) and
/// writeInt32 (:397-450).
const AXIS_P_PARAMS: [(&str, u32); 20] = [
    ("AXIS_MODE", 1),
    ("MOP_POS", 11),
    ("MOP_NEG", 12),
    ("STEP_RES", 45),
    ("STOP_CURRENT", 40),
    ("RUN_CURRENT", 41),
    ("BOOST_CURRENT", 42),
    ("ENCODER_TYP", 34),
    ("INIT_TIME", 13),
    ("POSITION_TIME", 16),
    ("BOOST", 17),
    ("ENC_RATE", 26),
    ("SWITCH_TYP", 27),
    ("PWR_STAGE_MODE", 28),
    ("ENC_RESOLUTION", 35),
    ("ENC_FUNCTION", 36),
    ("ENC_SFI_WIDTH", 37),
    ("ENC_DIRECTION", 38),
    ("PS_MONITOR", 53),
    ("CURRENT_DELAY_TIME", 43),
];

/// The temperatures readFloat64 asks in 0.1 °C (:487-490).
const AXIS_TEMP_PARAMS: [(&str, u32); 2] = [("PS_TEMPERATURE", 49), ("MOTOR_TEMP", 54)];

/// The current registers count 10 mA; their records' EGU is mA.
fn is_current(p: u32) -> bool {
    matches!(p, 40..=42)
}

/// Uppercase hex digit for a nibble/address value in 0..=15.
fn hex_upper(v: u8) -> u8 {
    if v > 9 { b'A' + (v - 10) } else { b'0' + v }
}

/// Format a value the way C `std::to_string(double)` does (six decimals), used
/// for the `=`/`S` parameter writes.
fn fmt6(v: f64) -> String {
    format!("{:.6}", CFloat(v))
}

/// Clamp a (base, max) velocity pair into the controller range, taking the
/// magnitude (C `setVelocity` clamps `fabs` of each independently).
fn clamp_velocity(min_v: f64, max_v: f64) -> (f64, f64) {
    let c = |v: f64| v.abs().clamp(MIN_VELOCITY, MAX_VELOCITY);
    (c(min_v), c(max_v))
}

/// Clamp an acceleration into the controller range (C `setAcceleration`).
fn clamp_acceleration(a: f64) -> f64 {
    a.clamp(MIN_ACCELERATION, MAX_ACCELERATION)
}

/// Wrap a command as `STX <addr> <cmd> [":" <CRC>] ETX` (CRC only on phyMOTION).
fn build_frame(ctrl_type: CtrlType, address: u8, cmd: &[u8]) -> Vec<u8> {
    let mut inner: Vec<u8> = Vec::with_capacity(cmd.len() + 8);
    inner.push(hex_upper(address));
    inner.extend_from_slice(cmd);
    if ctrl_type == CtrlType::Phymotion {
        inner.push(b':');
        let mut crc: u8 = 0;
        for &b in &inner {
            crc ^= b;
        }
        inner.push(hex_upper(crc >> 4));
        inner.push(hex_upper(crc & 0x0F));
    }
    let mut frame: Vec<u8> = Vec::with_capacity(inner.len() + 2);
    frame.push(STX);
    frame.extend_from_slice(&inner);
    frame.push(ETX);
    frame
}

/// The body of `STX <ACK|NAK> <data> [":" <CRC>] [ETX]`, its ACK or NAK
/// kept, as C `sendPhytronCommand` checks it before the ACK test.
fn parse_frame(raw: &[u8]) -> AsynResult<Vec<u8>> {
    let start = raw
        .iter()
        .position(|&b| b == STX)
        .ok_or_else(|| phytron_err("phytron: reply without STX"))?;
    let after = &raw[start + 1..];
    let body = match after.iter().position(|&b| b == ETX) {
        Some(i) => &after[..i],
        None => return Err(phytron_err("phytron: reply without ETX")),
    };
    let mut resp = body.to_vec();

    // Optional trailing ":" + two-char CRC.
    let n = resp.len();
    if n > 3 && resp[n - 3] == b':' {
        let c_hi = resp[n - 2];
        let c_lo = resp[n - 1];
        if !c_hi.is_ascii_alphanumeric() || !c_lo.is_ascii_alphanumeric() {
            return Err(phytron_err("phytron: malformed reply CRC"));
        }
        // "XX" disables the CRC check; otherwise verify it.
        if !c_hi.eq_ignore_ascii_case(&b'X') || !c_lo.eq_ignore_ascii_case(&b'X') {
            let mut crc: u8 = 0;
            for &b in &resp[..n - 2] {
                crc ^= b;
            }
            if c_hi.to_ascii_uppercase() != hex_upper(crc >> 4)
                || c_lo.to_ascii_uppercase() != hex_upper(crc & 0x0F)
            {
                return Err(phytron_err("phytron: reply CRC mismatch"));
            }
        }
        resp.truncate(n - 3);
    }
    Ok(resp)
}

/// C `sendPhytronCommand`'s reply check (phytronAxisMotor.cpp:1081-1126):
/// with `ack_only` an ACK reply, stripped; without, an ACK or NAK reply,
/// kept. Anything else is `phytronInvalidReturn` (6).
fn reply_bytes(raw: &[u8], ack_only: bool) -> Result<Vec<u8>, i32> {
    let mut resp = parse_frame(raw).map_err(|_| 6)?;
    match (resp.first(), ack_only) {
        (Some(&ACK), true) => {
            resp.remove(0);
            Ok(resp)
        }
        (Some(&ACK | &NAK), false) => Ok(resp),
        _ => Err(6),
    }
}

/// C `isspace`, which `epicsParseInt64` and writeOption's trim skip.
pub(crate) fn c_isspace(c: char) -> bool {
    c == ' ' || ('\t'..='\r').contains(&c)
}

/// C `epicsParseInt64(value, &v, 0, NULL)`: a base-0 integer with only
/// white space around it.
pub(crate) fn parse_int64(value: &str) -> Option<i64> {
    let t = value.trim_matches(c_isspace);
    let (neg, body) = match t.as_bytes().first() {
        Some(b'-') => (true, &t[1..]),
        Some(b'+') => (false, &t[1..]),
        _ => (false, t),
    };
    let (radix, digits) = if body.len() > 2 && (body.starts_with("0x") || body.starts_with("0X")) {
        (16, &body[2..])
    } else if body.len() > 1 && body.starts_with('0') {
        (8, &body[1..])
    } else {
        (10, body)
    };
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_alphanumeric()) {
        return None;
    }
    let v = i64::from_str_radix(digits, radix).ok()?;
    Some(if neg { -v } else { v })
}

/// C writeOption's boolean words (phytronAxisMotor.cpp:745-754).
fn c_bool(value: &str) -> Option<bool> {
    let is = |words: &[&str]| words.iter().any(|w| value.eq_ignore_ascii_case(w));
    if is(&["t", "true", "y", "yes", "on"]) {
        Some(true)
    } else if is(&["f", "false", "n", "no", "off"]) {
        Some(false)
    } else {
        None
    }
}

/// C writeOption "clearAxisStatus" (phytronAxisMotor.cpp:743-756): seconds
/// 0..10, a boolean (on is 1 us, no wait), or an axis's "default".
fn parse_clear_axis_status(value: &str, axis: bool) -> Option<f32> {
    // C `epicsParseFloat`.
    let seconds = epics_libcom_rs::runtime::stdlib::epics_parse_double(value)
        .ok()
        .map(|v| v as f32)
        .filter(|t| (0.0..=10.0).contains(t));
    if seconds.is_some() {
        return seconds;
    }
    if axis && value.eq_ignore_ascii_case("default") {
        return Some(f32::INFINITY);
    }
    c_bool(value).map(|on| if on { 0.000001 } else { 0.0 })
}

/// C readOption "clearAxisStatus" (phytronAxisMotor.cpp:609-622).
fn show_clear_axis_status(time: f32) -> String {
    let time = time.abs();
    if !time.is_finite() {
        "default".to_owned()
    } else if time >= 0.001 {
        CFloat(f64::from(time)).g().to_string()
    } else if time > 0.0 {
        "on".to_owned()
    } else {
        "off".to_owned()
    }
}

/// C `pollMethod` (phytronAxisMotor.h:96-101). An axis's `Default` takes
/// the controller's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PollMethod {
    Default,
    /// One command and reply at a time: `P20R`, `P22R`, `==H`, `SE`.
    Serial,
    /// The axis's `SE`, `P20R`, `P22R` on one line.
    AxisParallel,
    /// Every such axis's requests on one line, sent by the controller poll.
    ControllerParallel,
}

impl PollMethod {
    fn code(self) -> i64 {
        match self {
            Self::Default => -1,
            Self::Serial => 0,
            Self::AxisParallel => 1,
            Self::ControllerParallel => 2,
        }
    }

    /// C readOption's spelling (phytronAxisMotor.cpp:597-604).
    fn name(self) -> &'static str {
        match self {
            Self::Default => "default",
            Self::Serial => "serial",
            Self::AxisParallel => "axis-parallel",
            Self::ControllerParallel => "ctrl-parallel",
        }
    }

    /// C writeOption's value (phytronAxisMotor.cpp:676-708): a code or a
    /// name; only an axis may take `default`.
    fn parse(value: &str, axis: bool) -> Option<Self> {
        let all = [
            Self::Default,
            Self::Serial,
            Self::AxisParallel,
            Self::ControllerParallel,
        ];
        let allowed = |m: &Self| axis || *m != Self::Default;
        if let Some(code) = parse_int64(value) {
            return all.into_iter().filter(allowed).find(|m| m.code() == code);
        }
        let v = value.trim_matches(c_isspace);
        let names: [(Self, &[&str]); 4] = [
            (Self::Default, &["default", "standard", "controller"]),
            (
                Self::Serial,
                &["serial", "no-parallel", "not-parallel", "old"],
            ),
            (Self::AxisParallel, &["axis", "axis-parallel"]),
            (
                Self::ControllerParallel,
                &["ctrl", "parallel", "ctrl-parallel", "controller-parallel"],
            ),
        ];
        names
            .into_iter()
            .filter(|(m, _)| allowed(m))
            .find(|(_, words)| words.iter().any(|w| v.eq_ignore_ascii_case(w)))
            .map(|(m, _)| m)
    }
}

/// The controller's poll periods, in seconds, for its report, and its
/// axis parameters.
pub struct PhytronParams {
    moving_poll: f64,
    idle_poll: f64,
    /// Set once the controller connects; C's axis lookup fails before.
    controller: Arc<OnceLock<Arc<Mutex<PhytronController>>>>,
    reasons: Reasons,
    /// Reason -> P register.
    p_regs: HashMap<usize, u32>,
    temps: HashMap<usize, u32>,
}

/// The reasons C answers outside the P registers.
#[derive(Default)]
struct Reasons {
    controller_status: usize,
    controller_status_reset: usize,
    controller_reset: usize,
    axis_status_reset: usize,
    axis_reset: usize,
    homing_procedure: usize,
    brake_output: usize,
    disable_motor: usize,
    brake_engage_time: usize,
    brake_release_time: usize,
    direct_command: usize,
    direct_reply: usize,
    direct_status: usize,
}

impl PhytronParams {
    pub fn new(moving_poll: f64, idle_poll: f64) -> Self {
        Self {
            moving_poll,
            idle_poll,
            controller: Arc::new(OnceLock::new()),
            reasons: Reasons::default(),
            p_regs: HashMap::new(),
            temps: HashMap::new(),
        }
    }

    /// The slot the create command fills with the connected controller.
    pub fn controller_slot(&self) -> Arc<OnceLock<Arc<Mutex<PhytronController>>>> {
        self.controller.clone()
    }

    /// C `getAxis` (the base's axis table, which phytronCreateAxis fills
    /// before `axes_`): the axis's address and command prefix, or the "Axis
    /// not found" line `func` prints on pasynUserSelf.
    fn axis_controller(
        &self,
        base: &PortDriverBase,
        user: AxisUser<'_>,
        func: &str,
    ) -> AsynResult<(MutexGuard<'_, PhytronController>, i32, String)> {
        let port = &*base.port_name;
        if let Some(ctrl) = self.controller.get().filter(|_| has_motor_axis(base, user)) {
            let ctrl = ctrl.lock().unwrap_or_else(|e| e.into_inner());
            let addr = user.addr;
            let m = axis_module_no(ctrl.ctrl_type, addr)?;
            return Ok((ctrl, addr, m));
        }
        MotorTrace::new(port, 0).print_self(
            TraceMask::ERROR,
            format_args!("{func}: Axis not found on the controller {port}\n"),
        );
        Err(phytron_err(format!("{func}: Axis not found")))
    }

    /// The connected controller; C has none to command before it connects.
    fn controller(&self) -> AsynResult<MutexGuard<'_, PhytronController>> {
        let ctrl = self
            .controller
            .get()
            .ok_or_else(|| phytron_err("phytron: controller not connected"))?;
        Ok(ctrl.lock().unwrap_or_else(|e| e.into_inner()))
    }

    /// CONTROLLER_RESET (`CR`, then every axis's encoder ratio back to 1,
    /// C `resetAxisEncoderRatio`) or CONTROLLER_STATUS_RESET (`STC`).
    fn controller_write(&self, base: &mut PortDriverBase, user: AxisUser<'_>) -> AsynResult<()> {
        let port = base.port_name.clone();
        let mut ctrl = self.controller()?;
        if user.reason == self.reasons.controller_status_reset {
            let sent =
                ctrl.ctrl_command(&port, "writeInt32", "Reseting status", user.reason, "STC");
            return sent.map(|_| ());
        }
        let sent = ctrl.reset(&port, user.reason);
        for a in &ctrl.axes {
            let _ = base.set_float64_param(index::ENCODER_RATIO, a.axis_no, 1.0);
        }
        // C's callParamCallbacks() is address 0's.
        let _ = base.call_param_callbacks(0);
        sent
    }

    /// C writeOption "clearAxisStatus" (phytronAxisMotor.cpp:726-759): an
    /// address above 0 is an axis's. A disarmed time stays disarmed.
    fn write_clear_axis_status(
        &self,
        user: AxisUser<'_>,
        key: &str,
        value: &str,
    ) -> AsynResult<()> {
        let refuse = |why: &str| {
            phytron_err(format!(
                "phytronController:writeOption({key}, {value}) {why}"
            ))
        };
        let mut ctrl = self.controller().map_err(|_| refuse("wrong axis"))?;
        let axis = user.addr > 0;
        if axis && ctrl.axis(user.addr).is_none() {
            return Err(refuse("wrong axis"));
        }
        let Some(time) = parse_clear_axis_status(value, axis) else {
            let default = if axis { " or \"default\"" } else { "" };
            return Err(refuse(&format!(
                "wrong value - allowed is seconds (0..10) or boolean{default}"
            )));
        };
        let slot = match ctrl.axis_mut(user.addr).filter(|_| axis) {
            Some(a) => &mut a.status_reset_time,
            None => &mut ctrl.status_reset_time,
        };
        *slot = if *slot <= 0.0 { -time } else { time };
        Ok(())
    }

    /// C's missing-axis refusal for a reason that is not a P register,
    /// which never takes the controller lock.
    fn refuses(base: &PortDriverBase, user: AxisUser<'_>, func: &str) -> Option<ParamWrite> {
        if has_motor_axis(base, user) {
            return None;
        }
        let port = &*base.port_name;
        MotorTrace::new(port, 0).print_self(
            TraceMask::ERROR,
            format_args!("{func}: Axis not found on the controller {port}\n"),
        );
        Some(ParamWrite::PassToBase(Err(phytron_err(format!(
            "{func}: Axis not found"
        )))))
    }
}

impl ControllerParams for PhytronParams {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        let r = &mut self.reasons;
        r.controller_status = base.create_param("CONTROLLER_STATUS", ParamType::Int32)?;
        r.controller_status_reset =
            base.create_param("CONTROLLER_STATUS_RESET", ParamType::Int32)?;
        r.controller_reset = base.create_param("CONTROLLER_RESET", ParamType::Int32)?;
        r.axis_status_reset = base.create_param("AXIS_STATUS_RESET", ParamType::Int32)?;
        r.axis_reset = base.create_param("AXIS_RESET", ParamType::Int32)?;
        base.create_param("AXIS_STATUS", ParamType::Int32)?;
        r.homing_procedure = base.create_param("HOMING_PROCEDURE", ParamType::Int32)?;
        for (name, p) in AXIS_P_PARAMS {
            self.p_regs
                .insert(base.create_param(name, ParamType::Int32)?, p);
        }
        for (name, p) in AXIS_TEMP_PARAMS {
            self.temps
                .insert(base.create_param(name, ParamType::Float64)?, p);
        }
        let r = &mut self.reasons;
        r.brake_output = base.create_param("AXIS_BRAKE_OUTPUT", ParamType::Float64)?;
        r.disable_motor = base.create_param("AXIS_DISABLE_MOTOR", ParamType::Int32)?;
        r.brake_engage_time = base.create_param("AXIS_BRAKE_ENGAGE_TIME", ParamType::Float64)?;
        r.brake_release_time = base.create_param("AXIS_BRAKE_RELEASE_TIME", ParamType::Float64)?;
        r.direct_command = base.create_param("DIRECT_COMMAND", ParamType::Octet)?;
        r.direct_reply = base.create_param("DIRECT_REPLY", ParamType::Octet)?;
        r.direct_status = base.create_param("DIRECT_STATUS", ParamType::Int32)?;
        base.set_string_param(r.direct_reply, 0, "")?;
        base.set_int32_param(r.direct_status, 0, 0)?;
        Ok(())
    }

    /// C `phytronController::writeOctet` (phytronAxisMotor.cpp:550-578):
    /// DIRECT_COMMAND's bytes go to the controller as they are; the reply
    /// lands in DIRECT_REPLY less its ACK or NAK, which DIRECT_STATUS names
    /// (1, 2; 3 for neither, kept). C writes the base only on success.
    fn write_octet(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        data: &[u8],
    ) -> ParamWrite {
        let r = &self.reasons;
        if user.reason == r.direct_reply || user.reason == r.direct_status {
            return ParamWrite::Handled(Err(phytron_err(
                "phytronController::writeOctet: read-only parameter",
            )));
        }
        if user.reason != r.direct_command {
            return ParamWrite::PassToBase(Ok(()));
        }
        let sent = self
            .controller()
            .and_then(|ctrl| ctrl.send_bytes(data, false).map_err(phy_to_asyn));
        let reply = match sent {
            Ok(reply) => reply,
            Err(e) => return ParamWrite::Handled(Err(e)),
        };
        let (status, text) = match reply.first() {
            Some(&ACK) => (1, &reply[1..]),
            Some(&NAK) => (2, &reply[1..]),
            _ => (3, &reply[..]),
        };
        let _ = base.set_string_param(r.direct_reply, 0, text);
        let _ = base.set_int32_param(r.direct_status, 0, status);
        ParamWrite::PassToBase(Ok(()))
    }

    /// C `phytronController::writeInt32` (phytronAxisMotor.cpp:365-466),
    /// which calls the base before it sends the register; here the register
    /// is sent first and the base stores after. `ENC_FUNCTION` sets `P36` to
    /// 1 for any positive value.
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        value: i32,
    ) -> ParamWrite {
        let r = &self.reasons;
        if user.reason == r.controller_reset || user.reason == r.controller_status_reset {
            let status = self.controller_write(base, user);
            // C's base stores nothing without an axis, and C returns the
            // command's status rather than its error (:375-386).
            return if has_motor_axis(base, user) {
                ParamWrite::PassToBase(status)
            } else {
                ParamWrite::Handled(status)
            };
        }
        if user.reason == r.homing_procedure {
            let found = self.axis_controller(base, user, "phytronAxis::writeInt32");
            return ParamWrite::PassToBase(found.map(|(mut ctrl, addr, _)| {
                if let Some(a) = ctrl.axis_mut(addr) {
                    a.homing_procedure = Some(value);
                }
            }));
        }
        if user.reason == r.disable_motor {
            let found = self.axis_controller(base, user, "phytronAxis::writeInt32");
            return ParamWrite::PassToBase(found.map(|(mut ctrl, addr, _)| {
                ctrl.reconfigure_brake(addr, |b| b.disable_idle = value != 0);
            }));
        }
        let p = self.p_regs.get(&user.reason).copied();
        if p.is_none() && user.reason != r.axis_reset && user.reason != r.axis_status_reset {
            return Self::refuses(base, user, "phytronAxis::writeInt32")
                .unwrap_or(ParamWrite::PassToBase(Ok(())));
        }
        let port = base.port_name.clone();
        let (mut ctrl, addr, m) = match self.axis_controller(base, user, "phytronAxis::writeInt32")
        {
            Ok(found) => found,
            Err(e) => return ParamWrite::PassToBase(Err(e)),
        };
        let cmd = match p {
            Some(p) => {
                let value = match p {
                    36 => i32::from(value > 0),
                    p if is_current(p) => value / 10,
                    _ => value,
                };
                format!("{m}P{p:02}{}{value}", ctrl.set_char())
            }
            None if user.reason == self.reasons.axis_reset => format!("{m}C"),
            None if ctrl.ctrl_type == CtrlType::Phymotion => format!("SEC{}", &m[1..]),
            None => {
                return ParamWrite::PassToBase(Err(phytron_err(
                    "phytronAxis::writeInt32: an MCC has no axis status reset",
                )));
            }
        };
        let sent = ctrl.axis_command(&port, "writeInt32", "Writing parameter", addr, &cmd);
        ParamWrite::PassToBase(sent.map(|_| ()))
    }

    /// C `phytronController::writeFloat64` (phytronAxisMotor.cpp:519-541)
    /// looks the axis up first, then sets the brake's members.
    fn write_float64(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        value: f64,
    ) -> ParamWrite {
        let r = &self.reasons;
        let engage = user.reason == r.brake_engage_time;
        if user.reason != r.brake_output && !engage && user.reason != r.brake_release_time {
            return Self::refuses(base, user, "phytronAxis::writeFloat64")
                .unwrap_or(ParamWrite::PassToBase(Ok(())));
        }
        let (mut ctrl, addr, _) =
            match self.axis_controller(base, user, "phytronAxis::writeFloat64") {
                Ok(found) => found,
                Err(e) => return ParamWrite::PassToBase(Err(e)),
            };
        let Some(a) = ctrl.axis_mut(addr) else {
            return ParamWrite::PassToBase(Ok(()));
        };
        if user.reason == r.brake_output {
            a.brake.output = Brake::output_of(value);
            return ParamWrite::PassToBase(Ok(()));
        }
        // C hands the base the time clamped to MAXIMUM_BRAKE_TIME, which
        // the records read back; the base here stores what was written, so
        // its store, callbacks and trace line are done here.
        let value = if value > MAX_BRAKE_TIME {
            MAX_BRAKE_TIME
        } else {
            value
        };
        let time = if value > 0.0 { value as f32 } else { 0.0 };
        if engage {
            a.brake.engage_time = time;
        } else {
            a.brake.release_time = time;
        }
        drop(ctrl);
        let _ = base.set_float64_param(user.reason, addr, value);
        let _ = base.call_param_callbacks(addr);
        PortManager::global().trace_manager().print(
            Some(&base.port_name),
            Some(addr),
            user.reason as i32,
            TraceMask::IO_DRIVER,
            format_args!(
                "asynMotorController:writeFloat64:: axis={addr}, function={}, value={value:.6}\n",
                user.reason,
                value = CFloat(value)
            ),
        );
        ParamWrite::Handled(Ok(()))
    }

    /// C `phytronController::readInt32` (phytronAxisMotor.cpp:285-357): the
    /// base value, unless the reason is a P register, which is asked.
    fn read_int32(&mut self, base: &mut PortDriverBase, user: AxisUser<'_>) -> AsynResult<i32> {
        let value = base.read_int32_param(user.asyn()).unwrap_or(0);
        let port = base.port_name.clone();
        let r = &self.reasons;
        if user.reason == r.controller_reset || user.reason == r.controller_status_reset {
            return Ok(value);
        }
        if user.reason == r.controller_status {
            let mut ctrl = self.controller()?;
            let reply = ctrl.ctrl_command(
                &port,
                "readInt32",
                "Reading controller status",
                user.reason,
                "ST",
            )?;
            return Ok(atoi(&reply));
        }
        let (mut ctrl, addr, m) =
            self.axis_controller(base, user, "phytronController::readInt32")?;
        if user.reason == r.disable_motor {
            return Ok(ctrl
                .axis(addr)
                .map_or(0, |a| i32::from(a.brake.disable_idle)));
        }
        let Some(&p) = self.p_regs.get(&user.reason) else {
            return Ok(value);
        };
        let reply = ctrl.axis_command(
            &port,
            "readInt32",
            "Reading parameter",
            addr,
            &format!("{m}P{p}R"),
        )?;
        let value = atoi(&reply);
        Ok(if is_current(p) { value * 10 } else { value })
    }

    /// C `phytronController::readFloat64` (phytronAxisMotor.cpp:472-512),
    /// which looks the axis up before the base.
    fn read_float64(&mut self, base: &mut PortDriverBase, user: AxisUser<'_>) -> AsynResult<f64> {
        let port = base.port_name.clone();
        let (mut ctrl, addr, m) =
            self.axis_controller(base, user, "phytronController::readFloat64")?;
        let value = base.read_float64_param(user.asyn()).unwrap_or(0.0);
        let r = &self.reasons;
        if let Some(b) = ctrl.axis(addr).map(|a| a.brake) {
            for (reason, v) in [
                (r.brake_output, b.output),
                (r.brake_engage_time, b.engage_time),
                (r.brake_release_time, b.release_time),
            ] {
                if user.reason == reason {
                    return Ok(f64::from(v));
                }
            }
        }
        let Some(&p) = self.temps.get(&user.reason) else {
            return Ok(value);
        };
        let reply = ctrl.axis_command(
            &port,
            "readFloat64",
            "Reading parameter",
            addr,
            &format!("{m}P{p}R"),
        )?;
        Ok(atof(&reply) / 10.)
    }

    /// C `phytronController::readOption` (phytronAxisMotor.cpp:589-655):
    /// the addressed axis's poll method, else the controller's. Unlike
    /// writeOption it takes any address `getAxis` resolves, 0 included.
    fn read_option(
        &self,
        _base: &PortDriverBase,
        user: AxisUser<'_>,
        key: &str,
    ) -> Option<AsynResult<String>> {
        if key.eq_ignore_ascii_case("clearAxisStatus") {
            return Some(self.controller().map(|ctrl| {
                let time = ctrl
                    .axis(user.addr)
                    .map_or(ctrl.status_reset_time, |a| a.status_reset_time);
                show_clear_axis_status(time)
            }));
        }
        if !key.eq_ignore_ascii_case("pollMethod") {
            return None;
        }
        Some(self.controller().map(|ctrl| {
            let m = ctrl
                .axis(user.addr)
                .map_or(ctrl.default_poll_method, |a| a.poll_method);
            format!("{}/{}", m.code(), m.name())
        }))
    }

    /// C `phytronController::writeOption` (phytronAxisMotor.cpp:661-818);
    /// an address above 0 is an axis's.
    fn write_option(
        &mut self,
        _base: &mut PortDriverBase,
        user: AxisUser<'_>,
        key: &str,
        value: &str,
    ) -> Option<AsynResult<()>> {
        if key.eq_ignore_ascii_case("clearAxisStatus") {
            return Some(self.write_clear_axis_status(user, key, value));
        }
        if !key.eq_ignore_ascii_case("pollMethod") {
            return None;
        }
        let refuse = |why: &str| -> AsynResult<()> {
            Err(phytron_err(format!(
                "phytronController:writeOption({key}, {value}) {why}"
            )))
        };
        let Ok(mut ctrl) = self.controller() else {
            return Some(refuse("wrong axis"));
        };
        let axis = user.addr > 0;
        if axis && ctrl.axis(user.addr).is_none() {
            return Some(refuse("wrong axis"));
        }
        let Some(method) = PollMethod::parse(value, axis) else {
            let (default, controller) = if axis {
                ("default,", "controller-")
            } else {
                ("", "")
            };
            return Some(refuse(&format!(
                "wrong value - allowed are {default}serial,axis-parallel,{controller}parallel"
            )));
        };
        if ctrl.ctrl_type != CtrlType::Phymotion
            && !matches!(method, PollMethod::Default | PollMethod::Serial)
        {
            return Some(refuse("pollMethod not allowed for this controller"));
        }
        match ctrl.axis_mut(user.addr).filter(|_| axis) {
            Some(a) => a.poll_method = method,
            None => ctrl.default_poll_method = method,
        }
        Some(Ok(()))
    }

    /// C `phytronController::report` (phytronAxisMotor.cpp:836-843), which
    /// says phyMOTION for an MCC too.
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "Phytron phyMOTION motor driver {}, numAxes={}, moving poll period={:.6}, \
             idle poll period={:.6}",
            ctx.port_name(),
            ctx.num_axes(),
            CFloat(self.moving_poll),
            CFloat(self.idle_poll),
        );
        ctx.report(out, level);
    }
}

/// C `axisModuleNo_`, axis `axis_no`'s command prefix: `M<mod>.<idx>` or
/// the MCC digit.
fn axis_module_no(ctrl_type: CtrlType, axis_no: i32) -> AsynResult<String> {
    Ok(match ctrl_type {
        // C: epicsSnprintf(..., "M%.1f", iAxisNo / 10.)
        CtrlType::Phymotion => format!("M{:.1}", CFloat(axis_no as f64 / 10.0)),
        CtrlType::Mcc => {
            let n = axis_no % 10;
            if !(1..=8).contains(&n) {
                return Err(phytron_err("phytron MCC: axis index must be 1..=8"));
            }
            n.to_string()
        }
    })
}

/// C `phytronAxis::report` (phytronAxisMotor.cpp:1377-1384): the axis
/// number, `module*10 + index`.
pub struct PhytronAxisReport(pub i32);

impl AxisReport for PhytronAxisReport {
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level > 0 {
            let _ = writeln!(out, "  axis {}", self.0);
        }
    }
}

/// The controller's view of one created axis, which its writes, options
/// and controller poll reach (C reaches the `phytronAxis` itself).
struct AxisState {
    axis_no: i32,
    /// C `axisModuleNo_`.
    module_no: String,
    /// C `iPollMethod_`.
    poll_method: PollMethod,
    /// C `phytronAxis::statusResetTime_` (phytronAxisMotor.h:148): 0 off,
    /// below 0 armed by the next limit-switch error, above 0 sends `SEC`
    /// before the next motion; infinite takes the controller's.
    status_reset_time: f32,
    /// C `phytronAxis::lastStatus_`, which the axis's `CHECK_AXIS` compares.
    last_status: i32,
    /// The controller-parallel poll's answers for this axis this pass, which
    /// its own poll parses (C parses them in the controller poll).
    answers: Option<Vec<String>>,
    /// The axis has been polled, for [`PhytronController::did_initial_readout`].
    read_once: bool,
    /// HOMING_PROCEDURE as written, which C's `home` reads back; unset
    /// until a write, as C's param is.
    homing_procedure: Option<i32>,
    brake: Brake,
}

/// C `phytronAxis`'s brake members (phytronAxisMotor.h:126-130), in C's
/// `float`.
#[derive(Clone, Copy, Default)]
struct Brake {
    /// C `brakeOutput_`: the digital output `<module>.<index>`, an MCC's
    /// in its tenths, 0 for none, negative inverted.
    output: f32,
    /// C `disableMotor_` bit 0: the idle motor's output is disabled.
    disable_idle: bool,
    /// C `disableMotor_` bit 1: the motor output is enabled.
    enabled: bool,
    /// C `brakeEngageTime_` / `brakeReleaseTime_`, seconds.
    engage_time: f32,
    release_time: f32,
    /// C `brakeReleased_`.
    released: bool,
}

impl Brake {
    /// C's output, tenths within ±100 (phytronAxisMotor.cpp:529, 1512).
    fn output_of(v: f64) -> f32 {
        let v = if v > -100.0 && v < 100.0 { v } else { 0.0 };
        ((10.0 * v + 0.5).floor() / 10.0) as f32
    }

    /// The command that releases (`release`) or engages the brake
    /// (phytronAxisMotor.cpp:1544-1568). C's MCC digit is
    /// `fabs(10*out+0.5)`, which names output 2 for an inverted 3 (-0.3);
    /// the digit here is that of `|out|`.
    fn command(&self, phymotion: bool, release: bool) -> String {
        let c = if (self.output > 0.0) == release {
            'S'
        } else {
            'R'
        };
        let out = f64::from(self.output.abs());
        if phymotion {
            format!("A{:.1}{c}", CFloat(out))
        } else {
            format!("A{}{c}", (10.0 * out + 0.5) as i32 % 10)
        }
    }
}

/// A Phytron controller endpoint owning the asyn octet handle. Shared by all of
/// its axes behind a mutex so command/reply pairs stay atomic.
pub struct PhytronController {
    handle: SyncIOHandle,
    /// C `controllerName_`, the controller port.
    name: String,
    ctrl_type: CtrlType,
    /// Controller address 0..=15 (one hex digit in the frame).
    address: u8,
    /// The MCC `SUI` reply of this poll pass, ACK stripped (C `sLastSUI_`).
    last_sui: String,
    /// C `axes_`, in creation order; `getAxis` looks in the base's table
    /// instead.
    axes: Vec<AxisState>,
    /// C `lastStatus_`, the controller-wide status `CHECK_CTRL` and the
    /// controller's `CHECK_AXIS` compare.
    last_status: i32,
    /// C `iDefaultPollMethod_`.
    default_poll_method: PollMethod,
    /// C `phytronController::statusResetTime_`, the time of an axis left
    /// at "default"; only its magnitude is read.
    status_reset_time: f32,
    /// C's `motorEncoderRatio_` by axis prefix, which a controller reset
    /// sets back to 1 (phytronAxisMotor.cpp:821-827); an absent axis's is 1.
    encoder_ratios: HashMap<String, f64>,
    /// The AXIS_STATUS reason, which the poll sets.
    axis_status: Option<usize>,
    /// C `do_initial_readout_`, cleared by the first controller poll.
    do_initial_readout: bool,
    /// C `sCtrlType_`, escaped.
    type_name: String,
    /// C `IOs_`, the phytronIO channels the poll reads.
    ios: Vec<IoChannel>,
}

impl PhytronController {
    /// Wrap a connected octet handle for controller port `name`. `address`
    /// is the MCC hardware selector 0..=15 (0 for a phyMOTION).
    pub fn new(handle: SyncIOHandle, name: &str, ctrl_type: CtrlType, address: u8) -> Self {
        Self {
            handle,
            name: name.to_owned(),
            ctrl_type,
            address: address.min(15),
            last_sui: String::new(),
            axes: Vec::new(),
            last_status: 0,
            default_poll_method: PollMethod::Serial,
            status_reset_time: 0.0,
            encoder_ratios: HashMap::new(),
            axis_status: None,
            do_initial_readout: true,
            type_name: String::new(),
            ios: Vec::new(),
        }
    }

    pub(crate) fn ctrl_type(&self) -> CtrlType {
        self.ctrl_type
    }

    /// C `phytronIO`'s constructor's `IOs_.push_back` (phytronIOctrl.cpp:95-97).
    pub(crate) fn add_io(&mut self, io: IoChannel) {
        self.ios.push(io);
    }

    /// C `sCtrlType_`, which phytronIO reads.
    pub fn type_name(&self) -> &str {
        &self.type_name
    }

    /// The constructor's type queries (phytronAxisMotor.cpp:214-238). C
    /// keeps a failed query's reply buffer in the type; nothing is kept
    /// here. `print_escaped` is C `escapeC` but for a leading NUL.
    pub fn read_type(&mut self) {
        let reply = |cmd: &str| self.send(cmd, true).ok();
        let name = match self.ctrl_type {
            CtrlType::Phymotion => reply("IM0").unwrap_or_default(),
            CtrlType::Mcc => {
                let iar = reply("IAR").unwrap_or_default();
                mcc_type(&iar, reply("1P48R").as_deref())
            }
        };
        self.type_name = print_escaped(name.as_bytes());
    }

    /// C `phytronBrakeOutput`'s axis lookup and `configureBrake`
    /// (phytronAxisMotor.cpp:1287-1330, 1510-1524): axis `axis` is
    /// `module.index`, the times are seconds. A missing axis is `Err` with
    /// the name C looked for. C formats a phyMOTION's into 5 bytes, which
    /// cuts `M10.1`; the whole name is looked for here.
    pub fn configure_brake(
        &mut self,
        axis: f32,
        output: f32,
        disable_idle: bool,
        engage: f64,
        release: f64,
    ) -> Result<(), String> {
        let m = match self.ctrl_type {
            CtrlType::Phymotion => format!("M{:.1}", CFloat(f64::from(axis))),
            CtrlType::Mcc => {
                let i = (10.0 * f64::from(axis) + 0.5).floor() as i32;
                // C prints its unset name then.
                if !(1..=8).contains(&i) {
                    return Err(String::new());
                }
                i.to_string()
            }
        };
        let Some(axis_no) = self
            .axes
            .iter()
            .find(|a| a.module_no == m)
            .map(|a| a.axis_no)
        else {
            return Err(m);
        };
        let time = |t: f64| {
            let t = if t > MAX_BRAKE_TIME {
                MAX_BRAKE_TIME
            } else {
                t
            };
            if t > 0.0 { t as f32 } else { 0.0 }
        };
        self.reconfigure_brake(axis_no, |b| {
            b.output = Brake::output_of(f64::from(output));
            b.disable_idle = disable_idle;
            b.engage_time = time(engage);
            b.release_time = time(release);
        });
        Ok(())
    }

    /// `change` on axis `axis_no`'s brake, then C `setBrakeOutput(NULL,
    /// brakeReleased_)`, as configureBrake and the AXIS_DISABLE_MOTOR write
    /// do.
    fn reconfigure_brake(&mut self, axis_no: i32, change: impl FnOnce(&mut Brake)) {
        let Some(a) = self.axis_mut(axis_no) else {
            return;
        };
        change(&mut a.brake);
        let released = a.brake.released;
        self.set_brake_output(axis_no, None, released);
    }

    /// C `setBrakeOutput` (phytronAxisMotor.cpp:1531-1585) on axis
    /// `axis_no`: onto `list`, or sent at once with none. A wait sends the
    /// list first. The sends' statuses are dropped, as C's are.
    fn set_brake_output(&mut self, axis_no: i32, list: Option<&mut Vec<String>>, want_move: bool) {
        let phymotion = self.ctrl_type == CtrlType::Phymotion;
        let Some(a) = self.axis_mut(axis_no) else {
            return;
        };
        let b = a.brake;
        a.brake.released = want_move;
        if want_move {
            a.brake.enabled = true;
        } else if b.disable_idle {
            a.brake.enabled = false;
        }
        let m = a.module_no.clone();
        let mut own = Vec::new();
        let direct = list.is_none();
        let list = list.unwrap_or(&mut own);
        if want_move {
            if !b.enabled {
                list.push(format!("{m}MA"));
            }
            if b.output != 0.0 {
                list.push(b.command(phymotion, true));
            }
            self.flush_brake(list, direct || b.release_time > 0.0, b.release_time);
            return;
        }
        if b.output != 0.0 {
            list.push(b.command(phymotion, false));
            self.flush_brake(list, direct || b.engage_time > 0.0, b.engage_time);
        }
        if b.disable_idle {
            list.push(format!("{m}MD"));
            self.flush_brake(list, direct, 0.0);
        }
    }

    /// `list` on one line when `send`, then the brake's `wait`.
    fn flush_brake(&self, list: &mut Vec<String>, send: bool, wait: f32) {
        if send {
            let _ = self.multi_command(std::mem::take(list), false, false);
        }
        if wait > 0.0 {
            std::thread::sleep(Duration::from_secs_f32(wait));
        }
    }

    /// What C `epicsInithookFunction` (phytronAxisMotor.cpp:1003-1024)
    /// means to wait for: the controller and each of its axes read once.
    /// C waits on `do_initial_readout_` alone, which the startup pass clears
    /// before any axis exists, so whether the axis read lands before
    /// init_record is a race; that defeats the hook's purpose (:183-185),
    /// so the axes are waited for here.
    pub fn did_initial_readout(&self) -> bool {
        !self.do_initial_readout && self.axes.iter().all(|a| a.read_once)
    }

    /// Let each axis poll set AXIS_STATUS, reason `reason`.
    pub fn set_axis_status_reason(&mut self, reason: usize) {
        self.axis_status = Some(reason);
    }

    fn axis(&self, axis_no: i32) -> Option<&AxisState> {
        self.axes.iter().find(|a| a.axis_no == axis_no)
    }

    fn axis_mut(&mut self, axis_no: i32) -> Option<&mut AxisState> {
        self.axes.iter_mut().find(|a| a.axis_no == axis_no)
    }

    /// Axis `axis_no`'s poll method, its `Default` the controller's.
    fn poll_method(&self, axis_no: i32) -> PollMethod {
        match self.axis(axis_no).map(|a| a.poll_method) {
            Some(PollMethod::Default) | None => self.default_poll_method,
            Some(m) => m,
        }
    }

    /// C `phytronController::poll` (phytronAxisMotor.cpp:886-1001): an MCC
    /// reads the `SUI` limit-switch word once for every axis's poll to
    /// parse; a phyMOTION reads every controller-parallel axis on one line.
    /// Every phytronIO channel is read on the same line.
    pub fn poll(&mut self) {
        self.do_initial_readout = false;
        let mut commands = Vec::new();
        let mut todo = Vec::new();
        for a in &mut self.axes {
            a.answers = None;
        }
        match self.ctrl_type {
            CtrlType::Phymotion => {
                for a in &self.axes {
                    if self.poll_method(a.axis_no) == PollMethod::ControllerParallel {
                        todo.push(a.axis_no);
                        request(self.ctrl_type, PollMethod::ControllerParallel, &a.module_no)
                            .into_iter()
                            .for_each(|c| commands.push(c));
                    }
                }
            }
            CtrlType::Mcc => {
                self.last_sui.clear();
                commands.push("SUI".to_owned());
            }
        }
        let mcc = self.ctrl_type == CtrlType::Mcc;
        let io_counts: Vec<usize> = self
            .ios
            .iter()
            .map(|io| {
                let requests = io.requests(mcc);
                let n = requests.len();
                commands.extend(requests);
                n
            })
            .collect();
        if commands.is_empty() {
            return;
        }
        let status = match self.multi_command(commands, true, self.ctrl_type != CtrlType::Phymotion)
        {
            Ok(mut answers) => {
                let mut ok = true;
                if mcc {
                    // multi_command answers every command, SUI first.
                    match answers.remove(0).strip_prefix('\x06') {
                        Some(sui) => self.last_sui = sui.to_owned(),
                        None => ok = false,
                    }
                }
                for axis_no in todo {
                    // C erases the axis's three whatever they hold.
                    let n = answers.len().min(3);
                    let mine: Vec<String> = answers.drain(..n).collect();
                    if let Some(a) = self.axis_mut(axis_no) {
                        a.answers = Some(mine);
                    }
                }
                // A channel whose replies fail to parse leaves them to the
                // next in C (phytronIOctrl.cpp:408-483 return ahead of the
                // erase at :484), which posts its neighbour's values; each
                // channel takes its own here.
                for (io, n) in self.ios.iter().zip(io_counts) {
                    let mine: Vec<String> = answers.drain(..n.min(answers.len())).collect();
                    ok &= io.parse(mcc, &mine);
                }
                if ok && answers.is_empty() {
                    0
                } else {
                    AsynStatus::Error as i32
                }
            }
            // Each controller-parallel axis then finds no answers and raises
            // its problem bit. C also raises it on every serial axis, whose
            // own poll overwrites it in the same pass (phytronAxisMotor.cpp:
            // 982-986, a loop over axes_ under the apTodoAxis branch); that
            // C defect is not copied.
            Err(code) => phy_to_asyn(code).status() as i32,
        };
        if status != 0 {
            let name = &self.name;
            MotorTrace::new(name, 0).print_self(
                TraceMask::ERROR,
                format_args!("phytronController::poll({name}) error {status}\n"),
            );
        }
    }

    /// Parameter set-char: `=` on phyMOTION, `S` on MCC.
    fn set_char(&self) -> char {
        match self.ctrl_type {
            CtrlType::Phymotion => '=',
            CtrlType::Mcc => 'S',
        }
    }

    /// Send one command line and return its reply payload (ACK/CRC/framing
    /// removed). A NAK or malformed reply is an error (C `bACKonly = true`).
    pub(crate) fn command(&self, cmd: &str) -> AsynResult<String> {
        self.send(cmd, true).map_err(phy_to_asyn)
    }

    /// C `sendPhytronCommand` with its status code: the asyn status of a
    /// failed transfer, `phytronInvalidReturn` (6) for a reply that is not
    /// an ACK (or a NAK when `ack_only` is false, when the reply keeps it),
    /// `phytronInvalidCommand` (7) past 994 bytes.
    pub(crate) fn send(&self, cmd: &str, ack_only: bool) -> Result<String, i32> {
        let reply = self.send_bytes(cmd.as_bytes(), ack_only)?;
        Ok(String::from_utf8_lossy(&reply).into_owned())
    }

    /// [`Self::send`] of the bytes a DIRECT_COMMAND write holds.
    fn send_bytes(&self, cmd: &[u8], ack_only: bool) -> Result<Vec<u8>, i32> {
        if cmd.len() > 994 {
            return Err(7);
        }
        let frame = build_frame(self.ctrl_type, self.address, cmd);
        let (raw, status) = self.handle.write_read(0, &frame, READ_BUF).into_parts();
        status.map_err(|e| e.status() as i32)?;
        reply_bytes(&raw, ack_only)
    }

    /// C `sendPhytronMultiCommand` (phytronAxisMotor.cpp:1151-1216):
    /// `commands` joined by spaces, up to 42 (one when `single`) and 994
    /// bytes a line, and the replies split at each ACK or NAK. Without
    /// `allow_nak` every reply must be an ACK, which is stripped.
    pub(crate) fn multi_command(
        &self,
        mut commands: Vec<String>,
        allow_nak: bool,
        single: bool,
    ) -> Result<Vec<String>, i32> {
        let total = commands.len();
        let mut replies = Vec::new();
        loop {
            let mut len = 0;
            let mut count = 0;
            for c in commands.iter().take(if single { 1 } else { 42 }) {
                len += c.len() + 1;
                if c.is_empty() {
                    return Err(7);
                }
                if len > 994 {
                    break;
                }
                count += 1;
            }
            // C sends an empty line forever when the first command alone
            // passes 994 bytes; that command is refused here.
            if count == 0 {
                return Err(7);
            }
            let line = commands.drain(..count).collect::<Vec<_>>().join(" ");
            let owned = self.send(&line, false)?;
            let mut reply = owned.as_str();
            while !reply.is_empty() {
                let end = reply
                    .char_indices()
                    .skip(1)
                    .find(|&(_, c)| c == '\x06' || c == '\x15')
                    .map_or(reply.len(), |(i, _)| i);
                let (one, rest) = reply.split_at(end);
                if allow_nak {
                    replies.push(one.to_owned());
                } else {
                    replies.push(one.strip_prefix('\x06').ok_or(6)?.to_owned());
                }
                reply = rest;
            }
            if commands.is_empty() {
                break;
            }
        }
        if replies.len() == total {
            Ok(replies)
        } else {
            Err(6)
        }
    }

    /// C `CHECK_AXIS` in an axis method: a failure printed on the axis
    /// only when it differs from that axis's last status.
    fn check_axis<T>(
        &mut self,
        axis_no: i32,
        func: &str,
        txt: &str,
        result: Result<T, i32>,
    ) -> AsynResult<T> {
        let code = result.as_ref().err().copied().unwrap_or(0);
        let name = self.name.clone();
        if let Some(a) = self.axis_mut(axis_no) {
            if code != 0 && code != a.last_status {
                MotorTrace::new(&name, axis_no).print(
                    TraceMask::ERROR,
                    0,
                    format_args!(
                        "phytronAxis::{func}: ({name}) {txt} for axis {axis_no} failed with error code: {code}\n"
                    ),
                );
            }
            a.last_status = code;
        }
        result.map_err(phy_to_asyn)
    }

    /// `cmd` for axis `axis_no` under C `CHECK_AXIS` in a controller method,
    /// which prints a failure on the axis only when it differs from the
    /// controller's last status.
    fn axis_command(
        &mut self,
        port: &str,
        func: &str,
        txt: &str,
        axis_no: i32,
        cmd: &str,
    ) -> AsynResult<String> {
        let result = self.send(cmd, true);
        let code = result.as_ref().err().copied().unwrap_or(0);
        if code != 0 && code != self.last_status {
            MotorTrace::new(port, axis_no).print(
                TraceMask::ERROR,
                0,
                format_args!(
                    "phytronAxis::{func}: ({port}) {txt} for axis {axis_no} failed with error code: {code}\n"
                ),
            );
        }
        self.last_status = code;
        result.map_err(phy_to_asyn)
    }

    /// CONTROLLER_RESET's `CR`, after which every axis's encoder ratio is 1
    /// whether or not it was taken (phytronAxisMotor.cpp:377-381).
    fn reset(&mut self, port: &str, reason: usize) -> AsynResult<()> {
        let sent = self.ctrl_command(port, "writeInt32", "Reseting controller", reason, "CR");
        self.encoder_ratios.clear();
        sent.map(|_| ())
    }

    /// `cmd` under C `CHECK_CTRL`, which prints a failure on pasynUserSelf
    /// only when it differs from the last status.
    fn ctrl_command(
        &mut self,
        port: &str,
        func: &str,
        txt: &str,
        reason: usize,
        cmd: &str,
    ) -> AsynResult<String> {
        let result = self.send(cmd, true);
        let code = result.as_ref().err().copied().unwrap_or(0);
        if code != 0 && code != self.last_status {
            MotorTrace::new(port, 0).print_self(
                TraceMask::ERROR,
                format_args!(
                    "phytronController::{func}: ({port}) {txt} failed with error code: {code}, reason: {reason}\n"
                ),
            );
        }
        self.last_status = code;
        result.map_err(phy_to_asyn)
    }
    /// Reset the controller at boot (C constructor, unless `noResetAtBoot`):
    /// send `CR`, wait, then poll `S` until it answers (capped at 120 s). Never
    /// fails construction — matches the C behaviour of logging and continuing
    /// on `pasynUserSelf` of controller port `name`.
    pub fn reset_at_boot(&self, name: &str) {
        let trace = MotorTrace::new(name, 0);
        if self.command("CR").is_err() {
            trace.print_self(
                TraceMask::WARNING,
                format_args!(
                    "phytronController::phytronController: Could not reset controller {name}\n"
                ),
            );
        }
        std::thread::sleep(Duration::from_secs(5));
        let started = Instant::now();
        while self.command("S").is_err() {
            if started.elapsed() >= Duration::from_secs(120) {
                trace.print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "phytronController::phytronController: no valid answer after controller reset {name}\n"
                    ),
                );
                break;
            }
            std::thread::sleep(Duration::from_millis(500));
        }
    }
}

/// C's MCC type (phytronAxisMotor.cpp:216-233): `MCC-` and the `IAR`
/// reply, then the stage `1P48R` names when it answers.
fn mcc_type(iar: &str, p48: Option<&str>) -> String {
    let mut name = format!("MCC-{iar}");
    match p48.map(|p| (p, parse_int64(p))) {
        None | Some((_, Some(0))) => {}
        Some((_, Some(1))) => name.push_str(" LIN"),
        Some((p, _)) => {
            name.push_str(" type");
            name.push_str(p);
        }
    }
    name
}

/// C `appendRequestString` (phytronAxisMotor.cpp:1879-1907): the poll's
/// requests for axis `m` under `method`.
fn request(ctrl_type: CtrlType, method: PollMethod, m: &str) -> Vec<String> {
    match ctrl_type {
        CtrlType::Phymotion if method == PollMethod::Serial => vec![
            format!("{m}P20R"),
            format!("{m}P22R"),
            format!("{m}==H"),
            format!("SE{}", &m[1..]),
        ],
        // The status word's bit 0 stands in for `==H`, read in one go.
        CtrlType::Phymotion => vec![
            format!("SE{}", &m[1..]),
            format!("{m}P20R"),
            format!("{m}P22R"),
        ],
        CtrlType::Mcc => vec![
            format!("{m}=H"),
            format!("{m}=E"),
            format!("{m}P20R"),
            format!("{m}P22R"),
        ],
    }
}

/// One Phytron axis sharing a controller. Implements [`AsynMotor`].
pub struct PhytronAxis {
    controller: Arc<Mutex<PhytronController>>,
    ctrl_type: CtrlType,
    /// C `axisNo_`, `module*10 + index`.
    axis_no: i32,
    /// Command prefix: `M<mod>.<idx>` (phyMOTION) or a single digit (MCC).
    axis_module_no: String,
    /// Limit-home state machine (C `homeState_`): 0 = idle, else bit0 = homing
    /// direction (1 = forward/high), bits>>1 = phase.
    home_state: i32,
    /// AXIS_STATUS as the last poll set it, for the poller to post.
    sets: PortSets,
    /// What the constructor set on the axis's status.
    status_writes: Vec<StatusWrite>,
}

impl PhytronAxis {
    /// Construct axis `(module, index)` on `controller` and add it to the
    /// controller's axes, as C's constructor and `axes_.push_back` do under
    /// one lock. `index` must be >= 1 (C rejects `iAxis <= 0`); for an MCC
    /// it must be 1..=8.
    pub fn new(
        controller: Arc<Mutex<PhytronController>>,
        module: i32,
        index: i32,
    ) -> AsynResult<Self> {
        if index <= 0 {
            return Err(phytron_err("phytron: axis index must be >= 1"));
        }
        let axis_no = module * 10 + index;
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let ctrl_type = ctrl.ctrl_type;
        let axis_module_no = axis_module_no(ctrl_type, axis_no)?;
        ctrl.axes.push(AxisState {
            axis_no,
            module_no: axis_module_no.clone(),
            poll_method: PollMethod::Default,
            status_reset_time: f32::INFINITY,
            last_status: 0,
            answers: None,
            read_once: false,
            homing_procedure: None,
            brake: Brake::default(),
        });
        drop(ctrl);
        Ok(Self {
            controller,
            ctrl_type,
            axis_no,
            axis_module_no,
            home_state: 0,
            sets: PortSets::default(),
            // phytronAxisMotor.cpp:1352: the controller always has an encoder.
            status_writes: vec![StatusWrite::HasEncoder(true)],
        })
    }

    /// The command prefix, `M<mod>.<idx>` or the MCC digit.
    pub fn module_no(&self) -> &str {
        &self.axis_module_no
    }

    fn lock(&self) -> MutexGuard<'_, PhytronController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// SE-command suffix (`M1.2` -> `1.2`), phyMOTION status reads only.
    fn se_suffix(&self) -> &str {
        &self.axis_module_no[1..]
    }

    /// C `setVelocity` and `setAcceleration` onto `commands`
    /// (phytronAxisMotor.cpp:1425-1508): P14/P04/P15 for a move, P08/P10/P09
    /// for a home.
    fn profile(&self, commands: &mut Vec<String>, set: char, v: (f64, f64), a: f64, home: bool) {
        let m = &self.axis_module_no;
        let (mn, mx) = clamp_velocity(v.0, v.1);
        let (max, min, acc) = if home {
            ("P08", "P10", "P09")
        } else {
            ("P14", "P04", "P15")
        };
        commands.push(format!("{m}{max}{set}{}", fmt6(mx)));
        commands.push(format!("{m}{min}{set}{}", fmt6(mn)));
        commands.push(format!("{m}{acc}{set}{}", fmt6(clamp_acceleration(a))));
    }

    /// C `setBrakeOutput(list, 1)` of a motion method.
    fn release(&self, commands: &mut Vec<String>) {
        self.lock()
            .set_brake_output(self.axis_no, Some(commands), true);
    }

    /// C `clearAxisError` (phytronAxisMotor.cpp:1391-1415) under the
    /// motion methods' `CHECK_AXIS("move", "Clearing motor errors")`: an
    /// armed phyMOTION axis disarms and sends `SEC`, at once with its wait
    /// under the lock, or first on `commands` when there is no wait.
    fn clear_axis_error(&self, commands: &mut Vec<String>) -> AsynResult<()> {
        let mut ctrl = self.lock();
        let shared = ctrl.status_reset_time.abs();
        let phymotion = ctrl.ctrl_type == CtrlType::Phymotion;
        let Some(a) = ctrl.axis_mut(self.axis_no).filter(|_| phymotion) else {
            return Ok(());
        };
        let mut time = a.status_reset_time;
        if !time.is_finite() && time > 0.0 {
            time = shared;
        }
        if time <= 0.0 {
            return Ok(());
        }
        a.status_reset_time = -a.status_reset_time;
        let cmd = format!("SEC{}", self.se_suffix());
        let sent = if time >= 0.001 {
            let sent = ctrl.send(&cmd, true).map(|_| ());
            std::thread::sleep(Duration::from_secs_f32(time.min(10.0)));
            sent
        } else {
            commands.push(cmd);
            Ok(())
        };
        ctrl.check_axis(self.axis_no, "move", "Clearing motor errors", sent)
    }

    /// `commands` as one exchange under the axis's poll method, C's last
    /// `sendPhytronMultiCommand` of a motion method and its `CHECK_AXIS`.
    fn run(&self, commands: Vec<String>, func: &str, txt: &str) -> AsynResult<()> {
        let mut ctrl = self.lock();
        let single = ctrl.poll_method(self.axis_no) == PollMethod::Serial;
        let sent = ctrl.multi_command(commands, false, single).map(|_| ());
        ctrl.check_axis(self.axis_no, func, txt, sent)
    }

    /// C `move` (phytronAxisMotor.cpp:1597-1628).
    fn do_move(
        &mut self,
        position: f64,
        relative: bool,
        min_v: f64,
        max_v: f64,
        accel: f64,
    ) -> AsynResult<()> {
        let set = self.lock().set_char();
        let mut commands = Vec::new();
        self.clear_axis_error(&mut commands)?;
        self.profile(&mut commands, set, (min_v, max_v), accel, false);
        self.release(&mut commands);
        let m = &self.axis_module_no;
        commands.push(if relative {
            let sign = if position > 0.0 { '+' } else { '-' };
            format!("{m}{sign}{}", nint(position).unsigned_abs())
        } else {
            format!("{m}A{}", nint(position))
        });
        self.run(commands, "move", "Start move")
    }

    /// C `parseAnswer` (phytronAxisMotor.cpp:1916-2091) over this axis's
    /// `answers`, each ACK already stripped: every field it sets is set in
    /// `status`, and an answer it cannot parse sets none.
    fn parse_answer(
        &mut self,
        answers: &[String],
        method: PollMethod,
        status: &mut MotorStatus,
    ) -> AsynResult<()> {
        let bad = || phytron_err("phytron: invalid poll answer");
        let (pos, enc, moving, word) = match self.ctrl_type {
            CtrlType::Phymotion if method == PollMethod::Serial => {
                let [pos, enc, idle, se] = answers else {
                    return Err(bad());
                };
                let moving = !idle.starts_with('E');
                (atof(pos), atof(enc), moving, atoi(se))
            }
            CtrlType::Phymotion => {
                let [se, pos, enc] = answers else {
                    return Err(bad());
                };
                let word = atoi(se);
                (atof(pos), atof(enc), word & 1 != 0, word)
            }
            CtrlType::Mcc => {
                let [idle, perr, pos, enc] = answers else {
                    return Err(bad());
                };
                let moving = !idle.starts_with('E');
                let word = self.mcc_status_word(perr, &self.lock().last_sui, moving)?;
                (atof(pos), atof(enc), moving, word)
            }
        };
        // The poll reports `P22R` times the ratio (:2005-2006).
        let (ratio, axis_status) = {
            let ctrl = self.lock();
            let ratio = ctrl
                .encoder_ratios
                .get(&self.axis_module_no)
                .copied()
                .unwrap_or(1.0);
            (ratio, ctrl.axis_status)
        };
        status.position = pos;
        status.encoder_position = enc * ratio;
        // C engages the brake once the axis stops (:2013-2024).
        let old_done = status.done;
        status.done = !moving;
        if !old_done && !moving {
            self.lock().set_brake_output(self.axis_no, None, false);
        }

        // Limit-home state machine. `problem` is the axis
        // internal/power-stage/SFI/ENDAT error mask, cleared through the state
        // machine while homing onto a limit switch.
        let step = step_home_state(self.home_state, word, moving, self.ctrl_type);
        self.home_state = step.new_home_state;
        let problem = if step.clear_limit_error {
            // End of reference: clear the limit-switch error the phyMOTION sets.
            let cmd = format!("SEC{}", self.se_suffix());
            self.lock().command(&cmd).is_err()
        } else {
            step.problem
        };

        // A limit-switch error arms the next motion's `SEC` (:2070-2071).
        if word & 0x1000 != 0 {
            let mut ctrl = self.lock();
            if let Some(a) = ctrl.axis_mut(self.axis_no)
                && a.status_reset_time < 0.0
            {
                a.status_reset_time = -a.status_reset_time;
            }
        }

        // phytronAxisMotor.cpp:2084
        status.high_limit = step.high;
        status.low_limit = step.low;
        status.home = (word & 0x40) != 0;
        status.homed = (word & 0x08) != 0;
        status.encoder_home = (word & 0x08) != 0;
        status.slip = (word & 0x4000) != 0;
        if let Some(reason) = axis_status {
            self.sets.axis(reason, ParamValue::Int32(word));
        }
        status.problem = problem;
        Ok(())
    }
}

impl AsynMotor for PhytronAxis {
    fn take_port_sets(&mut self) -> PortSets {
        std::mem::take(&mut self.sets)
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }

    /// C sets motorStatusProblem_ and returns asynSuccess on every failure:
    /// a failed exchange (:1860-1863), a parse that fails (:2087) or a
    /// controller-parallel exchange that fails (:982-986).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM.succeeding()
    }

    /// C `phytronAxis::setEncoderRatio` (phytronAxisMotor.cpp:1759-1779):
    /// read the encoder (`P22R`), then set the conversion factor `P39` to
    /// `1/ratio`.
    fn set_encoder_ratio(&mut self, _user: &AsynUser, ratio: f64) -> AsynResult<()> {
        // asynMotorController::writeFloat64 stores the ratio before
        // dispatching, so a refused command still scales the poll (:289).
        let mut ctrl = self.lock();
        ctrl.encoder_ratios
            .insert(self.axis_module_no.clone(), ratio);
        let set = ctrl.set_char();
        let m = self.axis_module_no.as_str();
        // An unparsable reply is phytronInvalidReturn (:1767-1769).
        let read = ctrl
            .send(&format!("{m}P22R"), true)
            .and_then(|reply| reply.trim().parse::<f64>().map_err(|_| 6));
        let func = "setEncoderRatio";
        let encoder = ctrl.check_axis(self.axis_no, func, "Reading encoder position", read)?;
        let sent = ctrl.send(&format!("{m}P39{set}{}", fmt6(1.0 / ratio)), true);
        ctrl.check_axis(self.axis_no, func, "Setting ratio", sent)?;
        drop(ctrl);
        self.status_writes
            .push(StatusWrite::EncoderPosition(encoder * ratio));
        Ok(())
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

    /// C `moveVelocity` (phytronAxisMotor.cpp:1712-1736): the direction is
    /// the velocity's sign.
    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.jog(min_velocity, velocity, acceleration)
    }

    /// C `home` (phytronAxisMotor.cpp:1637-1705).
    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        self.start_home(min_velocity, velocity, acceleration, forward)
    }

    /// C `stop` (phytronAxisMotor.cpp:1741-1756): the deceleration on P15.
    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let set = self.lock().set_char();
        let m = &self.axis_module_no;
        let commands = vec![
            format!("{m}P15{set}{}", fmt6(clamp_acceleration(acceleration))),
            format!("{m}S"),
        ];
        self.run(commands, "stop", "Stop move")
    }

    /// C `setPosition` (phytronAxisMotor.cpp:1790-1818), which sends under
    /// the axis's own poll method unresolved; the resolved one is used here,
    /// as every other motion method does.
    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let set = self.lock().set_char();
        let commands = vec![format!("{}P20{set}{}", self.axis_module_no, fmt6(position))];
        self.run(commands, "setPosition", "Setting motor position")
    }

    /// C `phytronAxis::poll` (phytronAxisMotor.cpp:1842-1872): a serial or
    /// axis-parallel axis reads itself; a controller-parallel one parses
    /// what the controller poll read for it.
    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        let (method, answers) = {
            let mut ctrl = self.lock();
            let method = ctrl.poll_method(self.axis_no);
            let answers = if method == PollMethod::ControllerParallel {
                let answers = ctrl.axis_mut(self.axis_no).and_then(|a| a.answers.take());
                // Each one's ACK is still on it (:1963-1969).
                answers
                    .and_then(|a| {
                        a.iter()
                            .map(|s| s.strip_prefix('\x06').map(str::to_owned))
                            .collect::<Option<Vec<_>>>()
                    })
                    .ok_or_else(|| phytron_err("phytron: no controller poll answer"))
            } else {
                let commands = request(self.ctrl_type, method, &self.axis_module_no);
                let read = ctrl.multi_command(commands, false, method == PollMethod::Serial);
                ctrl.check_axis(self.axis_no, "poll", "Reading axis", read)
            };
            if let Some(a) = ctrl.axis_mut(self.axis_no) {
                a.read_once = true;
            }
            (method, answers)
        };
        let parsed = answers.and_then(|answers| self.parse_answer(&answers, method, status));
        // `!motorStatusDone_` whatever the poll read (:1868-1869), so an
        // unreachable controller keeps the poller at the moving period.
        status.moving = !status.done;
        parsed
    }
}

impl PhytronAxis {
    /// [`AsynMotor::move_velocity`]'s commands.
    fn jog(&mut self, min_velocity: f64, velocity: f64, acceleration: f64) -> AsynResult<()> {
        let set = self.lock().set_char();
        let mut commands = Vec::new();
        self.clear_axis_error(&mut commands)?;
        self.profile(
            &mut commands,
            set,
            (min_velocity, velocity),
            acceleration,
            false,
        );
        self.release(&mut commands);
        let dir = if velocity < 0.0 { "L-" } else { "L+" };
        commands.push(format!("{}{dir}", self.axis_module_no));
        self.run(commands, "moveVelocity", "Start move")
    }

    /// [`AsynMotor::home`]'s commands.
    fn start_home(
        &mut self,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let (set, procedure) = {
            let ctrl = self.lock();
            let procedure = ctrl.axis(self.axis_no).and_then(|a| a.homing_procedure);
            (ctrl.set_char(), procedure)
        };
        let mut commands = Vec::new();
        self.clear_axis_error(&mut commands)?;
        self.profile(
            &mut commands,
            set,
            (min_velocity, velocity),
            acceleration,
            true,
        );
        self.release(&mut commands);
        // C homingType; only `limit` arms the limit-home state machine.
        let d = if forward { '+' } else { '-' };
        let run = match procedure {
            Some(0) => {
                self.home_state = if forward { 3 } else { 2 };
                format!("R{d}")
            }
            Some(1) => format!("R{d}C"),
            Some(2) => format!("R{d}I"),
            Some(3) => format!("R{d}^I"),
            Some(4) => format!("R{d}C^I"),
            Some(5) => format!("RC{d}"),
            Some(6) => format!("RC{d}^I"),
            _ => return Err(phytron_err("phytronAxis::home: no homing procedure")),
        };
        commands.push(format!("{}{run}", self.axis_module_no));
        self.run(commands, "home", "Start homing")
    }

    /// Build the MCC status word from the `=E` power-stage reply, the controller
    /// `SUI` limit-switch line, and the moving flag (C parseAnswer MCC branch).
    fn mcc_status_word(&self, perr: &str, sui: &str, moving: bool) -> AsynResult<i32> {
        let axis_digit = self
            .axis_module_no
            .as_bytes()
            .first()
            .copied()
            .ok_or_else(|| phytron_err("phytron MCC: bad axis id"))?;
        mcc_status_word(axis_digit, perr, sui, moving)
    }
}

/// Outcome of one step of the limit-home state machine.
struct HomeStep {
    new_home_state: i32,
    high: bool,
    low: bool,
    /// Axis error present (before any end-of-reference SEC clear).
    problem: bool,
    /// The finalize reached the end-of-reference SEC clear (phyMOTION only); the
    /// caller sends `SEC` and takes `problem = !ok`.
    clear_limit_error: bool,
}

/// Advance the C `parseAnswer` limit-home state machine one poll. Pure: the
/// caller applies `new_home_state` and performs the `SEC` side effect when
/// `clear_limit_error` is set.
fn step_home_state(home_state: i32, status: i32, moving: bool, ctrl_type: CtrlType) -> HomeStep {
    let mut high = (status & 0x10) != 0;
    let mut low = (status & 0x20) != 0;
    if (home_state >> 1) == 0 {
        return HomeStep {
            new_home_state: home_state,
            high,
            low,
            problem: (status & 0xE800) != 0,
            clear_limit_error: false,
        };
    }

    let mut hs = home_state;
    let problem = (status & 0xE800) != 0;
    let mut clear = false;
    // Suppress the limit we are homing toward during the move.
    if (hs & 1) != 0 {
        high = false;
    } else {
        low = false;
    }
    match hs >> 1 {
        1 => {
            // Homing started, wait for moving.
            if moving {
                hs += 2;
            }
        }
        2..=4 => {
            if !moving {
                hs += 2;
                if (hs & 1) != 0 {
                    high = true;
                } else {
                    low = true;
                }
            }
            if (hs >> 1) >= 4 {
                let mask = if (hs & 1) != 0 { 0xFA29 } else { 0xFA19 };
                if ctrl_type == CtrlType::Phymotion && (status & mask) == 0x1208 {
                    clear = true;
                }
                hs = 0;
            }
        }
        _ => hs = 0,
    }
    HomeStep {
        new_home_state: hs,
        high,
        low,
        problem,
        clear_limit_error: clear,
    }
}

/// Build the MCC status word from the axis id digit, `=E` power-stage reply,
/// `SUI` limit-switch line and the moving flag (C parseAnswer MCC branch).
fn mcc_status_word(axis_digit: u8, perr: &str, sui: &str, moving: bool) -> AsynResult<i32> {
    let iaxis = match axis_digit {
        c @ b'1'..=b'8' => (c - b'1') as usize,
        _ => return Err(phytron_err("phytron MCC: bad axis id")),
    };
    let s = sui.as_bytes();
    if s.len() <= iaxis + 2 || s[0] != b'I' || s[1] != b'=' {
        return Err(phytron_err("phytron MCC: bad SUI response"));
    }
    let mut status = match s[2 + iaxis] {
        b'0' => 0x00, // no limit switch
        b'2' => 0x30, // both limit switches
        b'+' => 0x10, // positive limit switch
        b'-' => 0x20, // negative limit switch
        _ => return Err(phytron_err("phytron MCC: bad SUI limit char")),
    };
    if moving {
        status |= 0x01;
    }
    if perr.as_bytes().first() == Some(&b'E') {
        status |= 0x2000; // power-stage error
    }
    Ok(status)
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::motor::PortSet;
    use motor_common::testing::Polled;

    /// An MCC that ACKs every command, answering `P22R` with 1000 and the
    /// rest of a poll with an idle, fault-free axis; records each command
    /// payload. As a phyMOTION (`crc`) its `SE` status word is `0x4000`.
    struct Mcc {
        base: epics_rs::asyn::port::PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
        reply: Vec<u8>,
        crc: bool,
        /// The `P22R` encoder reply.
        encoder: &'static [u8],
    }

    impl epics_rs::asyn::port::PortDriver for Mcc {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            // STX, address, then `:` and the CRC on a phyMOTION, ETX.
            let tail = if self.crc { 4 } else { 1 };
            let cmd = String::from_utf8_lossy(&data[2..data.len() - tail]).into_owned();
            let payload: &[u8] = match cmd.as_str() {
                c if self.crc && c.starts_with("SE") => b"16384",
                c if c.ends_with("P22R") => self.encoder,
                c if c.ends_with("P20R") || c.ends_with("=E") => b"0",
                c if c.ends_with("=H") => b"E",
                "SUI" => b"I=00000000",
                _ => b"",
            };
            self.reply = [&[STX, ACK][..], payload, &[ETX]].concat();
            self.written.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            let reply = std::mem::take(&mut self.reply);
            buf[..reply.len()].copy_from_slice(&reply);
            epics_rs::asyn::interpose::OctetReadResult::ok(
                reply.len(),
                epics_rs::asyn::interpose::EomReason::empty(),
            )
        }
    }

    /// MCC axis 1 on a fresh [`Mcc`] port `name`, and its command log.
    fn mcc_axis(name: &str) -> (PhytronAxis, Arc<Mutex<Vec<String>>>) {
        mcc_axis_reading(name, b"1000")
    }

    /// [`mcc_axis`] whose controller answers `P22R` with `encoder`.
    fn mcc_axis_reading(
        name: &str,
        encoder: &'static [u8],
    ) -> (PhytronAxis, Arc<Mutex<Vec<String>>>) {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = Mcc {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written: written.clone(),
            reply: Vec::new(),
            crc: false,
            encoder,
        };
        let (rt, _actor) =
            create_port_runtime(&claim_port(name).unwrap(), port, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let controller = Arc::new(Mutex::new(PhytronController::new(
            io,
            name,
            CtrlType::Mcc,
            0,
        )));
        (PhytronAxis::new(controller, 0, 1).unwrap(), written)
    }

    /// MSTA EA_PRESENT before any poll and under a failed one's PROBLEM.
    #[test]
    fn the_constructor_sets_has_encoder() {
        let (mut axis, _) = mcc_axis("phytronHasEncoder");
        assert_eq!(axis.take_status_writes(), [StatusWrite::HasEncoder(true)]);
    }

    #[test]
    fn set_encoder_ratio_reads_the_encoder_then_sets_p39_to_its_inverse() {
        let (mut axis, written) = mcc_axis("phytronEncoderRatio");
        axis.set_encoder_ratio(&AsynUser::new(0), 0.5).unwrap();
        assert_eq!(*written.lock().unwrap(), ["1P22R", "1P39S2.000000"]);
    }

    /// C sets the encoder position the read gave times the new ratio
    /// (phytronAxisMotor.cpp:1776-1777).
    #[test]
    fn set_encoder_ratio_posts_the_scaled_encoder_position() {
        let (mut axis, _) = mcc_axis("phytronEncoderRatioPost");
        let _ = axis.take_status_writes();
        axis.set_encoder_ratio(&AsynUser::new(0), 0.5).unwrap();
        assert_eq!(
            axis.take_status_writes(),
            [StatusWrite::EncoderPosition(500.0)]
        );
    }

    /// C `CHECK_AXIS("setEncoderRatio", "Reading encoder position")`
    /// (phytronAxisMotor.cpp:1770): an unparsable reply is
    /// phytronInvalidReturn, printed once and kept as the axis's last status,
    /// and P39 is never sent.
    #[test]
    fn set_encoder_ratio_checks_the_encoder_read_on_the_axis() {
        let (mut axis, written) = mcc_axis_reading("phytronEncoderRatioBad", b"x");
        assert!(axis.set_encoder_ratio(&AsynUser::new(0), 0.5).is_err());
        assert_eq!(*written.lock().unwrap(), ["1P22R"]);
        let mut ctrl = axis.lock();
        assert_eq!(ctrl.axis_mut(axis.axis_no).unwrap().last_status, 6);
    }

    /// C `std::to_string(double)` is `%f`, which spells a NaN `nan`.
    #[test]
    fn a_non_finite_parameter_is_written_as_c_prints_it() {
        let (mut axis, written) = mcc_axis("phytronNonFinite");
        axis.set_encoder_ratio(&AsynUser::new(0), f64::NAN).unwrap();
        assert_eq!(*written.lock().unwrap(), ["1P22R", "1P39Snan"]);
        assert_eq!(fmt6(f64::NAN), "nan");
    }

    // C's poll reports !motorStatusDone_ past a failure: 0 until a poll
    // parses, 0 again after a move start (phytronAxisMotor.cpp:1868-1869,
    // :2022, asynMotorController.cpp:301).
    #[test]
    fn a_failed_poll_reports_moving_until_a_poll_reads_the_axis_idle() {
        let (mut axis, _) = mcc_axis("phytronFailedMoving");
        let user = AsynUser::new(0);
        let mut status = axis.constructed();
        // No SUI read yet, so the parse fails.
        assert!(axis.poll(&user, &mut status).is_err());
        assert!(status.moving);
        axis.controller.lock().unwrap().poll();
        axis.poll(&user, &mut status).unwrap();
        assert!(status.done && !status.moving);
        status.done = false;
        axis.controller.lock().unwrap().last_sui.clear();
        assert!(axis.poll(&user, &mut status).is_err());
        assert!(status.moving);
    }

    #[test]
    fn the_poll_reports_the_encoder_times_the_encoder_ratio() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = Mcc {
            base: PortDriverBase::new(
                "phytronEncoderPoll",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written: Arc::new(Mutex::new(Vec::new())),
            reply: Vec::new(),
            crc: false,
            encoder: b"1000",
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("phytronEncoderPoll").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let controller = Arc::new(Mutex::new(PhytronController::new(
            io,
            "phytronEncoderPoll",
            CtrlType::Mcc,
            0,
        )));
        controller.lock().unwrap().poll();
        controller.lock().unwrap().set_axis_status_reason(99);
        let mut axis = PhytronAxis::new(controller.clone(), 0, 1).unwrap();
        let user = AsynUser::new(0);
        assert_eq!(axis.polled(&user).unwrap().encoder_position, 1000.0);
        // AXIS_STATUS on the axis's own address, the poll's only step on
        // the port (:2084).
        let sets: Vec<_> = axis
            .take_port_sets()
            .into_iter()
            .map(|set| match set {
                PortSet::Axis {
                    reason,
                    value: ParamValue::Int32(_),
                } => reason,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(sets, [99]);
        axis.set_encoder_ratio(&user, 0.5).unwrap();
        assert_eq!(axis.polled(&user).unwrap().encoder_position, 500.0);
        controller
            .lock()
            .unwrap()
            .reset("phytronEncoderPoll", 0)
            .unwrap();
        assert_eq!(axis.polled(&user).unwrap().encoder_position, 1000.0);
    }

    /// phytronAxisMotor.cpp:2083: status bit 0x4000 is motorStatusSlip_,
    /// MSTA bit 4, not the following error.
    #[test]
    fn status_bit_0x4000_is_slip() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = Mcc {
            base: PortDriverBase::new(
                "phytronSlip",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written: Arc::new(Mutex::new(Vec::new())),
            reply: Vec::new(),
            crc: true,
            encoder: b"1000",
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("phytronSlip").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let controller = Arc::new(Mutex::new(PhytronController::new(
            io,
            "phytronSlip",
            CtrlType::Phymotion,
            0,
        )));
        let mut axis = PhytronAxis::new(controller, 1, 1).unwrap();
        let status = axis.polled(&AsynUser::new(0)).unwrap();
        assert!(status.slip);
        assert!(!status.slip_stall);
    }

    /// A phyMOTION that answers each space-separated command of a line,
    /// ACK and `answer(command)`, or NAK where that is `None`; records each
    /// line.
    struct Lines {
        base: epics_rs::asyn::port::PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
        answer: fn(&str) -> Option<String>,
        reply: Vec<u8>,
    }

    impl epics_rs::asyn::port::PortDriver for Lines {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            // STX, address, then `:` and the CRC, ETX.
            let line = String::from_utf8_lossy(&data[2..data.len() - 4]).into_owned();
            let mut body = Vec::new();
            for cmd in line.split(' ') {
                match (self.answer)(cmd) {
                    Some(a) => body.extend([&[ACK][..], a.as_bytes()].concat()),
                    None => body.push(NAK),
                }
            }
            self.reply = [&[STX][..], &body, b":XX", &[ETX]].concat();
            self.written.lock().unwrap().push(line);
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            let reply = std::mem::take(&mut self.reply);
            buf[..reply.len()].copy_from_slice(&reply);
            epics_rs::asyn::interpose::OctetReadResult::ok(
                reply.len(),
                epics_rs::asyn::interpose::EomReason::empty(),
            )
        }
    }

    /// A moving axis at 7 with its encoder at 9 and status word 0x11.
    fn moving_axis(cmd: &str) -> Option<String> {
        Some(
            match cmd {
                c if c.starts_with("SE") => "17",
                c if c.ends_with("P20R") => "7",
                c if c.ends_with("P22R") => "9",
                c if c.ends_with("==H") => "N",
                c if c.starts_with("BAD") => return None,
                _ => "",
            }
            .to_owned(),
        )
    }

    type Rig = (
        Arc<Mutex<PhytronController>>,
        [PhytronAxis; 2],
        Arc<Mutex<Vec<String>>>,
    );

    /// A phyMOTION on a fresh [`Lines`] port `name`, axes 1.1 and 1.2, and
    /// its line log.
    fn lines(name: &str) -> Rig {
        lines_with(name, moving_axis)
    }

    /// [`lines`] answering `answer`.
    fn lines_with(name: &str, answer: fn(&str) -> Option<String>) -> Rig {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = Lines {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written: written.clone(),
            answer,
            reply: Vec::new(),
        };
        let (rt, _actor) =
            create_port_runtime(&claim_port(name).unwrap(), port, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let ctrl = PhytronController::new(io, name, CtrlType::Phymotion, 0);
        let ctrl = Arc::new(Mutex::new(ctrl));
        let axes = [1, 2].map(|i| PhytronAxis::new(ctrl.clone(), 1, i).unwrap());
        (ctrl, axes, written)
    }

    /// C's axis poll raises the problem bit on a failed exchange and still
    /// returns asynSuccess (phytronAxisMotor.cpp:1842-1872), so a GET_INFO
    /// write prints no error.
    #[test]
    fn a_get_info_on_a_failed_exchange_succeeds() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::motor::flags::MotorCommand;
        use epics_rs::motor::poll_loop::execute_commands;

        let name = "phyQuietPoll";
        let (_ctrl, [mut axis, _], _) = lines_with(name, |_| None);
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        PortManager::global()
            .trace_manager()
            .set_trace_file(Some(name), TraceFile::File(file));
        let trace = MotorTrace::new(name, 0);
        execute_commands(&mut axis, &trace, &[MotorCommand::Poll], &mut |_| {});
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(!text.contains("asynMotorController"), "{text}");
    }

    /// C `sendPhytronMultiCommand` (phytronAxisMotor.cpp:1151-1216): up
    /// to 42 commands and 994 bytes a line, one reply per command.
    #[test]
    fn a_multi_command_line_holds_42_commands_or_994_bytes() {
        let (ctrl, _, written) = lines("phyMulti");
        let ctrl = ctrl.lock().unwrap();
        let replies = ctrl.multi_command(vec!["M1.1P20R".to_owned(); 43], false, false);
        assert_eq!(replies.unwrap(), vec!["7"; 43]);
        let sent = std::mem::take(&mut *written.lock().unwrap());
        assert_eq!(
            sent.iter()
                .map(|l| l.split(' ').count())
                .collect::<Vec<_>>(),
            [42, 1]
        );
        let long = "M1.1P20R".to_owned() + &"0".repeat(490);
        ctrl.multi_command(vec![long; 2], false, false).unwrap();
        assert_eq!(
            written.lock().unwrap().len(),
            2,
            "two 499-byte commands pass 994"
        );
        let single = ctrl.multi_command(vec!["M1.1P20R".to_owned(); 2], false, true);
        assert_eq!(single.unwrap().len(), 2);
        assert_eq!(written.lock().unwrap().len(), 4);
        // A NAK is refused unless allowed, when it is kept with each ACK.
        let commands = vec!["BAD".to_owned(), "M1.1P20R".to_owned()];
        assert_eq!(ctrl.multi_command(commands.clone(), false, false), Err(6));
        assert_eq!(
            ctrl.multi_command(commands, true, false).unwrap(),
            ["\x15", "\x067"]
        );
        // C loops forever on a command over 994 bytes.
        assert_eq!(
            ctrl.multi_command(vec!["0".repeat(995)], false, false),
            Err(7)
        );
    }

    /// C `appendRequestString`/`parseAnswer` (phytronAxisMotor.cpp:
    /// 1879-1907, 1916-2003): serial asks four times, axis-parallel once
    /// with the status word's bit 0 for moving.
    #[test]
    fn an_axis_parallel_poll_reads_its_status_and_positions_on_one_line() {
        let (ctrl, [mut axis, _], written) = lines("phyAxisParallel");
        let user = AsynUser::new(0);
        let serial = axis.polled(&user).unwrap();
        assert_eq!(
            *written.lock().unwrap(),
            ["M1.1P20R", "M1.1P22R", "M1.1==H", "SE1.1"]
        );
        assert!(serial.moving);
        written.lock().unwrap().clear();
        ctrl.lock().unwrap().axis_mut(11).unwrap().poll_method = PollMethod::AxisParallel;
        let parallel = axis.polled(&user).unwrap();
        assert_eq!(*written.lock().unwrap(), ["SE1.1 M1.1P20R M1.1P22R"]);
        assert!(parallel.moving && parallel.high_limit);
        assert_eq!((parallel.position, parallel.encoder_position), (7.0, 9.0));
    }

    /// C's controller poll reads every controller-parallel axis on one line
    /// and their own polls send nothing (phytronAxisMotor.cpp:899-985).
    #[test]
    fn a_controller_parallel_poll_reads_every_axis_on_one_line() {
        let (ctrl, mut axes, written) = lines("phyCtrlParallel");
        ctrl.lock().unwrap().default_poll_method = PollMethod::ControllerParallel;
        let user = AsynUser::new(0);
        assert!(axes[0].polled(&user).is_err(), "no controller poll yet");
        ctrl.lock().unwrap().poll();
        for axis in &mut axes {
            let status = axis.polled(&user).unwrap();
            assert_eq!(status.position, 7.0);
        }
        assert_eq!(
            *written.lock().unwrap(),
            ["SE1.1 M1.1P20R M1.1P22R SE1.2 M1.2P20R M1.2P22R"]
        );
        assert!(axes[1].polled(&user).is_err(), "each pass's answers once");
        // A pass that reads no axis in parallel leaves none of the last's.
        ctrl.lock().unwrap().poll();
        ctrl.lock().unwrap().default_poll_method = PollMethod::Serial;
        ctrl.lock().unwrap().poll();
        ctrl.lock().unwrap().default_poll_method = PollMethod::ControllerParallel;
        assert!(axes[0].polled(&user).is_err());
    }

    /// A motion method is one line unless the axis polls serially.
    #[test]
    fn a_move_is_one_line_unless_the_axis_polls_serially() {
        let (ctrl, [mut axis, _], written) = lines("phyMoveLine");
        let user = AsynUser::new(0);
        axis.move_absolute(&user, 12.4, 10.0, 100.0, 5000.0)
            .unwrap();
        ctrl.lock().unwrap().axis_mut(11).unwrap().poll_method = PollMethod::AxisParallel;
        axis.move_relative(&user, -3.0, 10.0, 100.0, 5000.0)
            .unwrap();
        let sent = written.lock().unwrap().clone();
        assert_eq!(
            sent,
            [
                "M1.1P14=100.000000",
                "M1.1P04=10.000000",
                "M1.1P15=5000.000000",
                "M1.1MA",
                "M1.1A12",
                "M1.1P14=100.000000 M1.1P04=10.000000 M1.1P15=5000.000000 M1.1-3",
            ]
        );
    }

    /// C writeOption/readOption "clearAxisStatus" values
    /// (phytronAxisMotor.cpp:609-622, 743-756).
    #[test]
    fn clear_axis_status_values_parse_and_show_as_c() {
        let cases = [
            ("0.5", false, Some(0.5)),
            ("10", false, Some(10.0)),
            ("11", false, None),
            ("-1", false, None),
            ("ON", false, Some(0.000001)),
            ("no", false, Some(0.0)),
            ("default", false, None),
            ("Default", true, Some(f32::INFINITY)),
            ("maybe", true, None),
        ];
        for (value, axis, want) in cases {
            assert_eq!(
                parse_clear_axis_status(value, axis),
                want,
                "{value:?} axis {axis}"
            );
        }
        let shown = [f32::NEG_INFINITY, -0.5, 2.0, 0.000001, 0.0].map(show_clear_axis_status);
        assert_eq!(shown, ["default", "0.5", "2", "on", "off"]);
    }

    /// An axis whose status word carries the limit-switch error 0x1000.
    fn limit_error_axis(cmd: &str) -> Option<String> {
        match cmd {
            c if c.starts_with("SE") => Some("4096".to_owned()),
            c => moving_axis(c),
        }
    }

    /// C `clearAxisError` and the poll that arms it (phytronAxisMotor.cpp:
    /// 1391-1415, 2070-2071): one `SEC` before the next motion, then none
    /// until a limit-switch error.
    #[test]
    fn an_armed_axis_clears_its_status_once_before_a_motion() {
        let (ctrl, [mut axis, _], written) = lines_with("phyClearStatus", limit_error_axis);
        let user = AsynUser::new(0);
        let secs = |written: &Arc<Mutex<Vec<String>>>| {
            let sent = std::mem::take(&mut *written.lock().unwrap());
            sent.iter().filter(|l| l.starts_with("SEC")).count()
        };
        axis.move_absolute(&user, 1.0, 10.0, 100.0, 5000.0).unwrap();
        assert_eq!(secs(&written), 0, "off by default");
        ctrl.lock().unwrap().axis_mut(11).unwrap().status_reset_time = 0.000001;
        axis.move_absolute(&user, 1.0, 10.0, 100.0, 5000.0).unwrap();
        assert_eq!(written.lock().unwrap()[0], "SEC1.1");
        assert_eq!(secs(&written), 1);
        axis.move_velocity(&user, 10.0, 100.0, 5000.0).unwrap();
        assert_eq!(secs(&written), 0, "disarmed");
        axis.polled(&user).unwrap();
        written.lock().unwrap().clear();
        ctrl.lock().unwrap().axis_mut(11).unwrap().homing_procedure = Some(0);
        axis.home(&user, 10.0, 100.0, 5000.0, true).unwrap();
        assert_eq!(secs(&written), 1, "a limit-switch error re-arms it");

        // A wait sends it alone, ahead of the motion's line.
        ctrl.lock().unwrap().axis_mut(11).unwrap().status_reset_time = 0.002;
        ctrl.lock().unwrap().axis_mut(11).unwrap().poll_method = PollMethod::AxisParallel;
        axis.move_absolute(&user, 12.4, 10.0, 100.0, 5000.0)
            .unwrap();
        let sent = std::mem::take(&mut *written.lock().unwrap());
        assert_eq!(sent.len(), 2, "{sent:?}");
        assert_eq!(sent[0], "SEC1.1");

        // "default" takes the controller's time, whatever its sign.
        let mut c = ctrl.lock().unwrap();
        c.axis_mut(11).unwrap().status_reset_time = f32::INFINITY;
        c.status_reset_time = -0.000001;
        drop(c);
        axis.move_absolute(&user, 1.0, 10.0, 100.0, 5000.0).unwrap();
        assert_eq!(secs(&written), 1);
        axis.move_absolute(&user, 1.0, 10.0, 100.0, 5000.0).unwrap();
        assert_eq!(secs(&written), 0);
    }

    /// C `home`'s seven homing procedures (phytronAxisMotor.cpp:1679-1699),
    /// of which only `limit` arms the limit-home state machine; an unset or
    /// unknown one sends nothing and fails.
    #[test]
    fn a_home_sends_the_axis_s_homing_procedure() {
        let (ctrl, [mut axis, _], written) = lines_with("phyHomingProc", |_| Some(String::new()));
        let user = AsynUser::new(0);
        let mut home = |procedure, forward| {
            ctrl.lock().unwrap().axis_mut(11).unwrap().homing_procedure = procedure;
            let homed = axis.home(&user, 10.0, 100.0, 5000.0, forward).is_ok();
            let sent = std::mem::take(&mut *written.lock().unwrap());
            (homed, sent.last().cloned(), axis.home_state)
        };
        let some = |c: &str| (true, Some(format!("M1.1{c}")), 0);
        assert_eq!(home(None, true), (false, None, 0));
        assert_eq!(home(Some(7), true), (false, None, 0));
        assert_eq!(home(Some(1), true), some("R+C"));
        assert_eq!(home(Some(1), false), some("R-C"));
        assert_eq!(home(Some(2), true), some("R+I"));
        assert_eq!(home(Some(3), false), some("R-^I"));
        assert_eq!(home(Some(4), true), some("R+C^I"));
        assert_eq!(home(Some(5), false), some("RC-"));
        assert_eq!(home(Some(6), true), some("RC+^I"));
        assert_eq!(home(Some(0), false), (true, Some("M1.1R-".into()), 2));
        assert_eq!(home(Some(0), true), (true, Some("M1.1R+".into()), 3));
    }

    /// C `configureBrake`'s output and `setBrakeOutput`'s commands
    /// (phytronAxisMotor.cpp:1512, 1544-1568).
    #[test]
    fn a_brake_command_names_its_output_as_c() {
        assert_eq!(Brake::output_of(1.25), 1.3);
        assert_eq!(Brake::output_of(-0.34), -0.3);
        assert_eq!(Brake::output_of(150.0), 0.0);
        assert_eq!(Brake::output_of(f64::NAN), 0.0);
        let brake = |output| Brake {
            output,
            ..Brake::default()
        };
        assert_eq!(brake(1.2).command(true, true), "A1.2S");
        assert_eq!(brake(1.2).command(true, false), "A1.2R");
        assert_eq!(brake(-1.2).command(true, true), "A1.2R");
        assert_eq!(brake(0.3).command(false, true), "A3S");
        assert_eq!(brake(0.3).command(false, false), "A3R");
        assert_eq!(brake(-0.3).command(false, true), "A3R");
    }

    /// A release wait sends the motion's list up to the release first; the
    /// poll that reads the axis stopped engages the brake, then disables
    /// the motor, each its own line (phytronAxisMotor.cpp:1531-1585, 2022).
    #[test]
    fn a_brake_is_released_ahead_of_a_move_and_engaged_once_it_stops() {
        let idle = |c: &str| Some(if c.ends_with("==H") { "E" } else { "0" }.to_owned());
        let (ctrl, [mut axis, _], written) = lines_with("phyBrakeLines", idle);
        let take = || std::mem::take(&mut *written.lock().unwrap());
        ctrl.lock()
            .unwrap()
            .configure_brake(1.1, 1.2, true, 0.0, 0.001)
            .unwrap();
        assert_eq!(take(), ["A1.2R", "M1.1MD"]);
        let user = AsynUser::new(0);
        axis.move_absolute(&user, 1.0, 10.0, 100.0, 5000.0).unwrap();
        let sent = take();
        let [.., release, mv] = &sent[..] else {
            panic!("{sent:?}");
        };
        assert!(release.ends_with(" M1.1MA A1.2S"), "{sent:?}");
        assert_eq!(mv, "M1.1A1");
        // Done is 0 from construction and from the move start.
        let mut status = axis.constructed();
        axis.poll(&user, &mut status).unwrap();
        let sent = take();
        assert_eq!(sent[sent.len() - 2..], ["A1.2R", "M1.1MD"], "{sent:?}");
        axis.poll(&user, &mut status).unwrap();
        assert!(!take().iter().any(|l| l.contains("A1.2")));
    }

    /// An MCC has no `SEC` (phytronAxisMotor.cpp:1394-1395).
    #[test]
    fn an_mcc_never_clears_its_status() {
        let (mut axis, written) = mcc_axis("phyMccClearStatus");
        axis.lock().axis_mut(1).unwrap().status_reset_time = 0.000001;
        axis.move_absolute(&AsynUser::new(0), 1.0, 10.0, 100.0, 5000.0)
            .unwrap();
        assert!(!written.lock().unwrap().iter().any(|c| c.starts_with("SEC")));
    }

    /// C writeOption's poll method values (phytronAxisMotor.cpp:676-708).
    #[test]
    fn poll_method_values_parse_as_c_parses_them() {
        use PollMethod::*;
        let cases = [
            ("2", false, Some(ControllerParallel)),
            ("0x1", false, Some(AxisParallel)),
            ("-1", false, None),
            ("-1", true, Some(Default)),
            ("3", true, None),
            (" Parallel\t", false, Some(ControllerParallel)),
            ("default", false, None),
            ("controller", true, Some(Default)),
            ("old", false, Some(Serial)),
            ("axis-parallel", false, Some(AxisParallel)),
            ("ax", false, None),
        ];
        for (value, axis, want) in cases {
            assert_eq!(
                PollMethod::parse(value, axis),
                want,
                "{value:?} axis {axis}"
            );
        }
    }

    #[test]
    fn phymotion_frame_has_stx_etx_and_crc() {
        // "0" + "M1.2P20R" + ":" then XOR-CRC over those bytes, wrapped STX..ETX.
        let frame = build_frame(CtrlType::Phymotion, 0, b"M1.2P20R");
        assert_eq!(frame[0], STX);
        assert_eq!(*frame.last().unwrap(), ETX);
        let inner = &frame[1..frame.len() - 1];
        // Address + command + ':' + 2 CRC chars.
        assert_eq!(&inner[..inner.len() - 2], b"0M1.2P20R:");
        let mut crc: u8 = 0;
        for &b in &inner[..inner.len() - 2] {
            crc ^= b;
        }
        assert_eq!(inner[inner.len() - 2], hex_upper(crc >> 4));
        assert_eq!(inner[inner.len() - 1], hex_upper(crc & 0x0F));
    }

    #[test]
    fn mcc_frame_has_address_no_crc() {
        // Address F (15), no CRC on the MCC.
        let frame = build_frame(CtrlType::Mcc, 15, b"1P20R");
        assert_eq!(frame, b"\x02F1P20R\x03");
    }

    #[test]
    fn reply_bytes_roundtrips_a_phymotion_frame() {
        // A controller reply: STX ACK "12345" ':' CRC ETX. Reuse build_frame's
        // CRC by framing "\x0612345" as the inner payload.
        let mut payload = vec![ACK];
        payload.extend_from_slice(b"12345");
        // Build a phyMOTION-style CRC over "0<payload>:" is not what the reply
        // uses; the reply CRC covers the response bytes only. Construct manually.
        let mut resp: Vec<u8> = Vec::new();
        resp.push(STX);
        resp.extend_from_slice(&payload);
        resp.push(b':');
        let mut crc: u8 = 0;
        for &b in &payload {
            crc ^= b;
        }
        // CRC also covers the ':' separator (matches the C send/verify).
        crc ^= b':';
        resp.push(hex_upper(crc >> 4));
        resp.push(hex_upper(crc & 0x0F));
        resp.push(ETX);
        assert_eq!(reply_bytes(&resp, true).unwrap(), b"12345");
    }

    #[test]
    fn reply_bytes_rejects_a_reply_without_etx() {
        // A read that ended before ETX (an input EOS that consumed it, or a
        // short raw read): C returns phytronInvalidReturn, not "678".
        let mut resp = vec![STX, ACK];
        resp.extend_from_slice(b"678");
        assert_eq!(reply_bytes(&resp, true), Err(6));
    }

    #[test]
    fn reply_bytes_rejects_nak() {
        let resp = vec![STX, NAK, ETX];
        assert!(reply_bytes(&resp, true).is_err());
        assert_eq!(reply_bytes(&resp, false).unwrap(), [NAK]);
    }

    #[test]
    fn reply_bytes_rejects_crc_mismatch() {
        // STX ACK "5" ':' "00" ETX — CRC "00" is wrong for the payload.
        let resp = vec![STX, ACK, b'5', b':', b'0', b'0', ETX];
        assert!(reply_bytes(&resp, true).is_err());
    }

    #[test]
    fn reply_bytes_xx_disables_crc_check() {
        // ":XX" marks the CRC as disabled; payload is accepted verbatim.
        let resp = vec![STX, ACK, b'9', b':', b'X', b'X', ETX];
        assert_eq!(reply_bytes(&resp, true).unwrap(), b"9");
    }

    #[test]
    fn mcc_status_maps_sui_limit_chars() {
        // Axis 3 (digit '3', index 2) -> SUI char at offset 4.
        assert_eq!(
            mcc_status_word(b'3', "", "I=00+00000", false).unwrap(),
            0x10
        );
        assert_eq!(
            mcc_status_word(b'3', "", "I=00-00000", false).unwrap(),
            0x20
        );
        assert_eq!(
            mcc_status_word(b'1', "", "I=200000000", false).unwrap(),
            0x30
        );
        assert_eq!(
            mcc_status_word(b'1', "", "I=000000000", true).unwrap(),
            0x01
        );
        // Power-stage error bit from "=E".
        assert_eq!(
            mcc_status_word(b'1', "E", "I=000000000", false).unwrap(),
            0x2000
        );
    }

    #[test]
    fn mcc_status_rejects_bad_sui() {
        assert!(mcc_status_word(b'1', "", "junk", false).is_err());
        assert!(mcc_status_word(b'9', "", "I=00000000", false).is_err());
    }

    // Home-state boundary cases: forward home (home_state starts 3) advances
    // 1 -> moving -> wait-stop -> reports high limit -> finalize -> 0.
    #[test]
    fn home_state_forward_progression() {
        let ct = CtrlType::Phymotion;
        // Phase 1 (hs=3): not moving yet -> stays.
        let s = step_home_state(3, 0, false, ct);
        assert_eq!(s.new_home_state, 3);
        // Phase 1: moving -> advance to phase 2 (hs=5).
        let s = step_home_state(3, 0, true, ct);
        assert_eq!(s.new_home_state, 5);
        // Phase 2 (hs=5): still moving -> stays.
        let s = step_home_state(5, 0, true, ct);
        assert_eq!(s.new_home_state, 5);
        assert!(!s.high); // homing-toward limit suppressed while moving
        // Phase 2: stopped -> advance to phase 3 (hs=7), report high limit.
        let s = step_home_state(5, 0, false, ct);
        assert_eq!(s.new_home_state, 7);
        assert!(s.high);
        // Phase 3 (hs=7): stopped -> advance to phase 4 (hs=9 >> 1 == 4),
        // finalize back to 0.
        let s = step_home_state(7, 0, false, ct);
        assert_eq!(s.new_home_state, 0);
        assert!(s.high);
        assert!(!s.clear_limit_error); // status not the 0x1208 error pattern
    }

    #[test]
    fn home_state_reverse_reports_low_limit() {
        // Reverse home starts hs=2. Suppress low while moving, report it on stop.
        let ct = CtrlType::Phymotion;
        let s = step_home_state(2, 0, true, ct);
        assert_eq!(s.new_home_state, 4); // phase 1 -> 2
        let s = step_home_state(4, 0, false, ct);
        assert_eq!(s.new_home_state, 6); // phase 2 -> 3
        assert!(s.low);
        assert!(!s.high);
    }

    #[test]
    fn home_state_finalize_requests_sec_on_limit_error() {
        // Forward, at finalize (hs=7 -> 0) with the 0x1208 error pattern under
        // the forward mask 0xFA29 -> request SEC clear.
        let s = step_home_state(7, 0x1208, false, CtrlType::Phymotion);
        assert_eq!(s.new_home_state, 0);
        assert!(s.clear_limit_error);
        // MCC never issues the SEC clear.
        let s = step_home_state(7, 0x1208, false, CtrlType::Mcc);
        assert!(!s.clear_limit_error);
    }

    #[test]
    fn home_state_idle_passes_status_through() {
        // Not homing (hs=0): limits come straight from the status word.
        let s = step_home_state(0, 0x10, false, CtrlType::Phymotion);
        assert!(s.high);
        assert!(!s.low);
        assert!(!s.problem);
        let s = step_home_state(0, 0x2000, false, CtrlType::Phymotion);
        assert!(s.problem); // power-stage error in the 0xE800 mask
    }

    /// phytronAxisMotor.cpp:216-238.
    #[test]
    fn the_type_is_what_c_composes_from_its_queries() {
        assert_eq!(mcc_type("2", Some("0")), "MCC-2");
        assert_eq!(mcc_type("1", Some(" 1 ")), "MCC-1 LIN");
        assert_eq!(mcc_type("1", Some("0x7")), "MCC-1 type0x7");
        assert_eq!(mcc_type("1", Some("x")), "MCC-1 typex");
        assert_eq!(mcc_type("", None), "MCC-");
        let (ctrl, _, written) = lines_with("phyType", |_| Some("phy\x07MOTION".into()));
        ctrl.lock().unwrap().read_type();
        assert_eq!(ctrl.lock().unwrap().type_name(), "phy\\aMOTION");
        assert_eq!(written.lock().unwrap()[..], ["IM0"]);
    }

    #[test]
    fn clamps_apply_controller_ranges() {
        assert_eq!(clamp_velocity(0.0, 1e9), (MIN_VELOCITY, MAX_VELOCITY));
        assert_eq!(clamp_velocity(-50.0, -50.0), (50.0, 50.0));
        assert_eq!(clamp_acceleration(10.0), MIN_ACCELERATION);
        assert_eq!(clamp_acceleration(1e9), MAX_ACCELERATION);
    }
}
