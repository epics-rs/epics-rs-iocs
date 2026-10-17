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
//!   success, `NAK` (0x15) failure. The startup script sets the input EOS to
//!   `"\x03"` (ETX); this driver owns the output framing (it appends STX..ETX),
//!   so the script must NOT set an output EOS.
//!
//! The reply parser accepts the payload whether or not the transport left the
//! trailing ETX in the buffer (an EOS-configured port strips it; a raw read
//! keeps it), matching the C driver's `find('\x03')` which tolerates both.
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
//! - The auxiliary controller/axis parameters other than the P registers
//!   (the `AXIS_STATUS`/`CONTROLLER_STATUS` records, the resets, the
//!   direct-command passthrough, ...).
//! - The `phytronIO` digital/analog I/O subsystem (`phytronIOctrl.cpp`).
//! - The brake output + idle-motor-disable feature (`phytronBrakeOutput`): with
//!   the default (no brake, keep motor enabled) a move only needs to enable the
//!   module output once, which this driver sends as `MA` before the first move.
//! - The `HOMING_PROCEDURE` aux record selects one of seven homing procedures;
//!   its record default is `limit` (home onto a limit switch), which is the only
//!   procedure this driver issues (`R+`/`R-`), together with the C limit-home
//!   state machine that reports the reached limit as home-done. The other six
//!   procedures require the unported aux record.
//! - The fake-HOMED-bit feature (Phytron registers 1001..1020, enabled by the
//!   `fakeHomedEnable` option): disabled by default, so HOMED is reported from
//!   status bit 0x08.
//! - The `pollMethod` batching options (axis-parallel / controller-parallel):
//!   this driver always polls serially (one command + reply each), the C
//!   default `pollMethodSerial`.
//! - The auto status-reset-on-error option (`SEC` via the `statusResetTime`
//!   option): dormant by default, so `clearAxisError` is a no-op here.

use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{ParamWrite, has_motor_axis, index};
use epics_rs::motor::trace::MotorTrace;

use epics_libcom_rs::cfmt::CFloat;
use motor_common::util::{atof, atoi, nint};
use motor_common::{AxisReport, ControllerParams, ControllerReport};

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
fn build_frame(ctrl_type: CtrlType, address: u8, cmd: &str) -> Vec<u8> {
    let mut inner: Vec<u8> = Vec::with_capacity(cmd.len() + 8);
    inner.push(hex_upper(address));
    inner.extend_from_slice(cmd.as_bytes());
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

/// Parse `STX <ACK|NAK> <data> [":" <CRC>] [ETX]` into the data payload. The
/// trailing ETX may already be stripped by the port's input EOS, so both an
/// EOS-stripped and a raw buffer are accepted.
fn parse_reply(raw: &[u8]) -> AsynResult<String> {
    let start = raw
        .iter()
        .position(|&b| b == STX)
        .ok_or_else(|| phytron_err("phytron: reply without STX"))?;
    let after = &raw[start + 1..];
    let body = match after.iter().position(|&b| b == ETX) {
        Some(i) => &after[..i],
        None => after,
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

    match resp.first() {
        Some(&ACK) => Ok(String::from_utf8_lossy(&resp[1..]).into_owned()),
        Some(&NAK) => Err(phytron_err("phytron: controller returned NAK")),
        _ => Err(phytron_err("phytron: reply without ACK/NAK")),
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

    /// C `getAxis` plus the "Axis not found" line `func` prints on
    /// pasynUserSelf.
    fn axis_controller(
        &self,
        port: &str,
        func: &str,
        addr: i32,
    ) -> AsynResult<(MutexGuard<'_, PhytronController>, String)> {
        if let Some(ctrl) = self.controller.get() {
            let ctrl = ctrl.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(m) = ctrl.axes.get(&addr).cloned() {
                return Ok((ctrl, m));
            }
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
    fn controller_write(&self, base: &mut PortDriverBase, user: &AsynUser) -> AsynResult<()> {
        let port = base.port_name.clone();
        let mut ctrl = self.controller()?;
        if user.reason == self.reasons.controller_status_reset {
            let sent =
                ctrl.ctrl_command(&port, "writeInt32", "Reseting status", user.reason, "STC");
            return sent.map(|_| ());
        }
        let sent = ctrl.reset(&port, user.reason);
        for &axis_no in ctrl.axes.keys() {
            let _ = base.set_float64_param(index::ENCODER_RATIO, axis_no, 1.0);
        }
        // C's callParamCallbacks() is address 0's.
        let _ = base.call_param_callbacks(0);
        sent
    }

    /// C's missing-axis refusal for a reason that is not a P register,
    /// which never takes the controller lock.
    fn refuses(base: &PortDriverBase, user: &AsynUser, func: &str) -> Option<ParamWrite> {
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
        for (name, p) in AXIS_P_PARAMS {
            self.p_regs
                .insert(base.create_param(name, ParamType::Int32)?, p);
        }
        for (name, p) in AXIS_TEMP_PARAMS {
            self.temps
                .insert(base.create_param(name, ParamType::Float64)?, p);
        }
        Ok(())
    }

    /// C `phytronController::writeInt32` (phytronAxisMotor.cpp:365-466):
    /// the register is set after the base stores the value; `ENC_FUNCTION`
    /// sets `P36` to 1 for any positive value.
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
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
        let Some(&p) = self.p_regs.get(&user.reason) else {
            return Self::refuses(base, user, "phytronAxis::writeInt32")
                .unwrap_or(ParamWrite::PassToBase(Ok(())));
        };
        let port = base.port_name.clone();
        let (mut ctrl, m) = match self.axis_controller(&port, "phytronAxis::writeInt32", user.addr)
        {
            Ok(found) => found,
            Err(e) => return ParamWrite::PassToBase(Err(e)),
        };
        let value = match p {
            36 => i32::from(value > 0),
            p if is_current(p) => value / 10,
            _ => value,
        };
        let cmd = format!("{m}P{p:02}{}{value}", ctrl.set_char());
        let sent = ctrl.axis_command(&port, "writeInt32", "Writing parameter", user.addr, &cmd);
        ParamWrite::PassToBase(sent.map(|_| ()))
    }

    /// C `phytronController::writeFloat64` (phytronAxisMotor.cpp:519-541)
    /// looks the axis up first.
    fn write_float64(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        _value: f64,
    ) -> ParamWrite {
        Self::refuses(base, user, "phytronAxis::writeFloat64")
            .unwrap_or(ParamWrite::PassToBase(Ok(())))
    }

    /// C `phytronController::readInt32` (phytronAxisMotor.cpp:285-357): the
    /// base value, unless the reason is a P register, which is asked.
    fn read_int32(&mut self, base: &mut PortDriverBase, user: &AsynUser) -> AsynResult<i32> {
        let value = base.read_int32_param(user).unwrap_or(0);
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
        let (mut ctrl, m) =
            self.axis_controller(&port, "phytronController::readInt32", user.addr)?;
        let Some(&p) = self.p_regs.get(&user.reason) else {
            return Ok(value);
        };
        let reply = ctrl.axis_command(
            &port,
            "readInt32",
            "Reading parameter",
            user.addr,
            &format!("{m}P{p}R"),
        )?;
        let value = atoi(&reply);
        Ok(if is_current(p) { value * 10 } else { value })
    }

    /// C `phytronController::readFloat64` (phytronAxisMotor.cpp:472-512),
    /// which looks the axis up before the base.
    fn read_float64(&mut self, base: &mut PortDriverBase, user: &AsynUser) -> AsynResult<f64> {
        let port = base.port_name.clone();
        let (mut ctrl, m) =
            self.axis_controller(&port, "phytronController::readFloat64", user.addr)?;
        let value = base.read_float64_param(user).unwrap_or(0.0);
        let Some(&p) = self.temps.get(&user.reason) else {
            return Ok(value);
        };
        let reply = ctrl.axis_command(
            &port,
            "readFloat64",
            "Reading parameter",
            user.addr,
            &format!("{m}P{p}R"),
        )?;
        Ok(atof(&reply) / 10.)
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

/// A Phytron controller endpoint owning the asyn octet handle. Shared by all of
/// its axes behind a mutex so command/reply pairs stay atomic.
pub struct PhytronController {
    handle: SyncIOHandle,
    ctrl_type: CtrlType,
    /// Controller address 0..=15 (one hex digit in the frame).
    address: u8,
    /// The MCC `SUI` reply of this poll pass, ACK stripped (C `sLastSUI_`).
    last_sui: String,
    /// Each created axis's command prefix by axis number (C `getAxis`).
    axes: BTreeMap<i32, String>,
    /// C `lastStatus_`, the controller-wide status `CHECK_AXIS` compares.
    last_status: i32,
    /// C's `motorEncoderRatio_` by axis prefix, which a controller reset
    /// sets back to 1 (phytronAxisMotor.cpp:821-827); an absent axis's is 1.
    encoder_ratios: HashMap<String, f64>,
}

impl PhytronController {
    /// Wrap a connected octet handle. `address` is the MCC hardware selector
    /// 0..=15 (0 for a phyMOTION).
    pub fn new(handle: SyncIOHandle, ctrl_type: CtrlType, address: u8) -> Self {
        Self {
            handle,
            ctrl_type,
            address: address.min(15),
            last_sui: String::new(),
            axes: BTreeMap::new(),
            last_status: 0,
            encoder_ratios: HashMap::new(),
        }
    }

    /// Make axis `axis_no` (`PhytronAxis::module_no`) one the parameter
    /// reads and writes find.
    pub fn add_axis(&mut self, axis_no: i32, module_no: &str) {
        self.axes.insert(axis_no, module_no.to_owned());
    }

    /// C `phytronController::poll` with the serial poll method: an MCC reads
    /// the `SUI` limit-switch word once for every axis's poll to parse; a
    /// phyMOTION sends nothing.
    pub fn poll(&mut self, name: &str) {
        if self.ctrl_type != CtrlType::Mcc {
            return;
        }
        self.last_sui.clear();
        match self.command("SUI") {
            Ok(sui) => self.last_sui = sui,
            Err(e) => MotorTrace::new(name, 0).print_self(
                TraceMask::ERROR,
                format_args!(
                    "phytronController::poll({name}) error {}\n",
                    e.status() as i32
                ),
            ),
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
    fn command(&self, cmd: &str) -> AsynResult<String> {
        let frame = build_frame(self.ctrl_type, self.address, cmd);
        let raw = self.handle.write_read(0, &frame, READ_BUF)?;
        parse_reply(&raw)
    }

    /// C `sendPhytronCommand` with its status code: the asyn status of a
    /// failed transfer, `phytronInvalidReturn` (6) for a reply that is not
    /// an ACK, `phytronInvalidCommand` (7) past 994 bytes.
    fn phy_command(&self, cmd: &str) -> Result<String, i32> {
        if cmd.len() > 994 {
            return Err(7);
        }
        let frame = build_frame(self.ctrl_type, self.address, cmd);
        let raw = self
            .handle
            .write_read(0, &frame, READ_BUF)
            .map_err(|e| e.status() as i32)?;
        parse_reply(&raw).map_err(|_| 6)
    }

    /// `cmd` for axis `axis_no` under C `CHECK_AXIS`, which prints a failure
    /// on the axis only when it differs from the last status.
    fn axis_command(
        &mut self,
        port: &str,
        func: &str,
        txt: &str,
        axis_no: i32,
        cmd: &str,
    ) -> AsynResult<String> {
        let result = self.phy_command(cmd);
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
        let result = self.phy_command(cmd);
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

/// One Phytron axis sharing a controller. Implements [`AsynMotor`].
pub struct PhytronAxis {
    controller: Arc<Mutex<PhytronController>>,
    ctrl_type: CtrlType,
    /// Command prefix: `M<mod>.<idx>` (phyMOTION) or a single digit (MCC).
    axis_module_no: String,
    /// True once the module output has been enabled (`MA`) for a move; the C
    /// default (no brake, motor kept enabled) enables it once and never
    /// disables it.
    motor_enabled: bool,
    /// Limit-home state machine (C `homeState_`): 0 = idle, else bit0 = homing
    /// direction (1 = forward/high), bits>>1 = phase.
    home_state: i32,
}

impl PhytronAxis {
    /// Construct axis `(module, index)` on `controller`. `index` must be >= 1
    /// (C rejects `iAxis <= 0`); for an MCC it must be 1..=8.
    pub fn new(
        controller: Arc<Mutex<PhytronController>>,
        module: i32,
        index: i32,
    ) -> AsynResult<Self> {
        if index <= 0 {
            return Err(phytron_err("phytron: axis index must be >= 1"));
        }
        let ctrl_type = controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .ctrl_type;
        let axis_no = module * 10 + index;
        let axis_module_no = match ctrl_type {
            // C: epicsSnprintf(..., "M%.1f", iAxisNo / 10.)
            CtrlType::Phymotion => format!("M{:.1}", CFloat(axis_no as f64 / 10.0)),
            CtrlType::Mcc => {
                let n = axis_no % 10;
                if !(1..=8).contains(&n) {
                    return Err(phytron_err("phytron MCC: axis index must be 1..=8"));
                }
                n.to_string()
            }
        };
        Ok(Self {
            controller,
            ctrl_type,
            axis_module_no,
            motor_enabled: false,
            home_state: 0,
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

    /// Shared move body for absolute and relative moves.
    fn do_move(
        &mut self,
        position: f64,
        relative: bool,
        min_v: f64,
        max_v: f64,
        accel: f64,
    ) -> AsynResult<()> {
        let need_enable = !self.motor_enabled;
        {
            let ctrl = self.lock();
            let set = ctrl.set_char();
            let m = self.axis_module_no.as_str();
            let (mn, mx) = clamp_velocity(min_v, max_v);
            // stdMove: max velocity P14, base velocity P04, acceleration P15.
            ctrl.command(&format!("{m}P14{set}{}", fmt6(mx)))?;
            ctrl.command(&format!("{m}P04{set}{}", fmt6(mn)))?;
            ctrl.command(&format!("{m}P15{set}{}", fmt6(clamp_acceleration(accel))))?;
            if need_enable {
                ctrl.command(&format!("{m}MA"))?;
            }
            if relative {
                let mag = nint(position).unsigned_abs();
                let sign = if position > 0.0 { '+' } else { '-' };
                ctrl.command(&format!("{m}{sign}{mag}"))?;
            } else {
                ctrl.command(&format!("{m}A{}", nint(position)))?;
            }
        }
        if need_enable {
            self.motor_enabled = true;
        }
        Ok(())
    }
}

impl AsynMotor for PhytronAxis {
    /// C `phytronAxis::poll` sets only `motorStatusProblem_` on a failed
    /// exchange.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
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
        ctrl.command(&format!("{m}P22R"))?
            .trim()
            .parse::<f64>()
            .map_err(|_| phytron_err("phytron: setEncoderRatio: invalid encoder position"))?;
        ctrl.command(&format!("{m}P39{set}{}", fmt6(1.0 / ratio)))?;
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

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let need_enable = !self.motor_enabled;
        {
            let ctrl = self.lock();
            let set = ctrl.set_char();
            let m = self.axis_module_no.as_str();
            let (mn, mx) = clamp_velocity(min_velocity, velocity);
            ctrl.command(&format!("{m}P14{set}{}", fmt6(mx)))?;
            ctrl.command(&format!("{m}P04{set}{}", fmt6(mn)))?;
            ctrl.command(&format!(
                "{m}P15{set}{}",
                fmt6(clamp_acceleration(acceleration))
            ))?;
            if need_enable {
                ctrl.command(&format!("{m}MA"))?;
            }
            // Jog direction from the sign of the requested velocity.
            let dir = if velocity < 0.0 { "L-" } else { "L+" };
            ctrl.command(&format!("{m}{dir}"))?;
        }
        if need_enable {
            self.motor_enabled = true;
        }
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
        let need_enable = !self.motor_enabled;
        {
            let ctrl = self.lock();
            let set = ctrl.set_char();
            let m = self.axis_module_no.as_str();
            let (mn, mx) = clamp_velocity(min_velocity, velocity);
            // homeMove: max velocity P08, base velocity P10, acceleration P09.
            ctrl.command(&format!("{m}P08{set}{}", fmt6(mx)))?;
            ctrl.command(&format!("{m}P10{set}{}", fmt6(mn)))?;
            ctrl.command(&format!(
                "{m}P09{set}{}",
                fmt6(clamp_acceleration(acceleration))
            ))?;
            if need_enable {
                ctrl.command(&format!("{m}MA"))?;
            }
            // Default homing procedure = limit: R+ / R-.
            let cmd = if forward {
                format!("{m}R+")
            } else {
                format!("{m}R-")
            };
            ctrl.command(&cmd)?;
        }
        if need_enable {
            self.motor_enabled = true;
        }
        // Arm the limit-home state machine (C: homeState_ = forwards ? 3 : 2).
        self.home_state = if forward { 3 } else { 2 };
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let set = ctrl.set_char();
        let m = self.axis_module_no.as_str();
        // stopMove acceleration -> P15, then the axis stop command S.
        ctrl.command(&format!(
            "{m}P15{set}{}",
            fmt6(clamp_acceleration(acceleration))
        ))?;
        ctrl.command(&format!("{m}S"))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let set = ctrl.set_char();
        let m = self.axis_module_no.as_str();
        ctrl.command(&format!("{m}P20{set}{}", fmt6(position)))?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Gather the raw readings while holding the controller lock.
        let (pos, enc, moving, status) = {
            let ctrl = self.lock();
            let m = self.axis_module_no.as_str();
            // The poll reports `P22R` times the ratio (:2005-2006).
            let ratio = ctrl.encoder_ratios.get(m).copied().unwrap_or(1.0);
            let (pos, enc, moving, status) = match self.ctrl_type {
                CtrlType::Phymotion => {
                    let pos = atof(&ctrl.command(&format!("{m}P20R"))?);
                    let enc = atof(&ctrl.command(&format!("{m}P22R"))?);
                    let idle = ctrl.command(&format!("{m}==H"))?;
                    let status = atoi(&ctrl.command(&format!("SE{}", self.se_suffix()))?);
                    let moving = idle.as_bytes().first() != Some(&b'E');
                    (pos, enc, moving, status)
                }
                CtrlType::Mcc => {
                    let idle = ctrl.command(&format!("{m}=H"))?;
                    let perr = ctrl.command(&format!("{m}=E"))?;
                    let pos = atof(&ctrl.command(&format!("{m}P20R"))?);
                    let enc = atof(&ctrl.command(&format!("{m}P22R"))?);
                    let moving = idle.as_bytes().first() != Some(&b'E');
                    let status = self.mcc_status_word(&perr, &ctrl.last_sui, moving)?;
                    (pos, enc, moving, status)
                }
            };
            (pos, enc * ratio, moving, status)
        };

        // Limit-home state machine (C parseAnswer). `problem` is the axis
        // internal/power-stage/SFI/ENDAT error mask, cleared through the state
        // machine while homing onto a limit switch.
        let step = step_home_state(self.home_state, status, moving, self.ctrl_type);
        self.home_state = step.new_home_state;
        let high = step.high;
        let low = step.low;
        let problem = if step.clear_limit_error {
            // End of reference: clear the limit-switch error the phyMOTION sets.
            let cmd = format!("SEC{}", self.se_suffix());
            self.lock().command(&cmd).is_err()
        } else {
            step.problem
        };

        Ok(MotorStatus {
            position: pos,
            encoder_position: enc,
            done: !moving,
            moving,
            high_limit: high,
            low_limit: low,
            home: (status & 0x40) != 0,
            encoder_home: (status & 0x08) != 0,
            homed: (status & 0x08) != 0,
            slip_stall: (status & 0x4000) != 0,
            problem,
            has_encoder: true,
            ..Default::default()
        })
    }
}

impl PhytronAxis {
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

    /// An MCC that ACKs every command, answering `P22R` with 1000 and the
    /// rest of a poll with an idle, fault-free axis; records each command
    /// payload.
    struct Mcc {
        base: epics_rs::asyn::port::PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
        reply: Vec<u8>,
    }

    impl epics_rs::asyn::port::PortDriver for Mcc {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(&data[2..data.len() - 1]).into_owned();
            let payload: &[u8] = match cmd.as_str() {
                c if c.ends_with("P22R") => b"1000",
                c if c.ends_with("P20R") || c.ends_with("=E") => b"0",
                c if c.ends_with("=H") => b"E",
                "SUI" => b"I=00000000",
                _ => b"",
            };
            self.reply = [&[STX, ACK][..], payload, &[ETX]].concat();
            self.written.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    /// MCC axis 1 on a fresh [`Mcc`] port `name`, and its command log.
    fn mcc_axis(name: &str) -> (PhytronAxis, Arc<Mutex<Vec<String>>>) {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = Mcc {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            written: written.clone(),
            reply: Vec::new(),
        };
        let (rt, _actor) =
            create_port_runtime(&claim_port(name).unwrap(), port, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let controller = Arc::new(Mutex::new(PhytronController::new(io, CtrlType::Mcc, 0)));
        (PhytronAxis::new(controller, 0, 1).unwrap(), written)
    }

    #[test]
    fn set_encoder_ratio_reads_the_encoder_then_sets_p39_to_its_inverse() {
        let (mut axis, written) = mcc_axis("phytronEncoderRatio");
        axis.set_encoder_ratio(&AsynUser::new(0), 0.5).unwrap();
        assert_eq!(*written.lock().unwrap(), ["1P22R", "1P39S2.000000"]);
    }

    /// C `std::to_string(double)` is `%f`, which spells a NaN `nan`.
    #[test]
    fn a_non_finite_parameter_is_written_as_c_prints_it() {
        let (mut axis, written) = mcc_axis("phytronNonFinite");
        axis.set_encoder_ratio(&AsynUser::new(0), f64::NAN).unwrap();
        assert_eq!(*written.lock().unwrap(), ["1P22R", "1P39Snan"]);
        assert_eq!(fmt6(f64::NAN), "nan");
    }

    #[test]
    fn the_poll_reports_the_encoder_times_the_encoder_ratio() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = Mcc {
            base: PortDriverBase::new("phytronEncoderPoll", 1, PortFlags::default()),
            written: Arc::new(Mutex::new(Vec::new())),
            reply: Vec::new(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("phytronEncoderPoll").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let controller = Arc::new(Mutex::new(PhytronController::new(io, CtrlType::Mcc, 0)));
        controller.lock().unwrap().poll("phytronEncoderPoll");
        let mut axis = PhytronAxis::new(controller.clone(), 0, 1).unwrap();
        let user = AsynUser::new(0);
        assert_eq!(axis.poll(&user).unwrap().encoder_position, 1000.0);
        axis.set_encoder_ratio(&user, 0.5).unwrap();
        assert_eq!(axis.poll(&user).unwrap().encoder_position, 500.0);
        controller
            .lock()
            .unwrap()
            .reset("phytronEncoderPoll", 0)
            .unwrap();
        assert_eq!(axis.poll(&user).unwrap().encoder_position, 1000.0);
    }

    #[test]
    fn phymotion_frame_has_stx_etx_and_crc() {
        // "0" + "M1.2P20R" + ":" then XOR-CRC over those bytes, wrapped STX..ETX.
        let frame = build_frame(CtrlType::Phymotion, 0, "M1.2P20R");
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
        let frame = build_frame(CtrlType::Mcc, 15, "1P20R");
        assert_eq!(frame, b"\x02F1P20R\x03");
    }

    #[test]
    fn parse_reply_roundtrips_a_phymotion_frame() {
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
        assert_eq!(parse_reply(&resp).unwrap(), "12345");
    }

    #[test]
    fn parse_reply_accepts_eos_stripped_etx() {
        // No trailing ETX (input EOS consumed it), no CRC: STX ACK data.
        let mut resp = vec![STX, ACK];
        resp.extend_from_slice(b"678");
        assert_eq!(parse_reply(&resp).unwrap(), "678");
    }

    #[test]
    fn parse_reply_rejects_nak() {
        let resp = vec![STX, NAK, ETX];
        assert!(parse_reply(&resp).is_err());
    }

    #[test]
    fn parse_reply_rejects_crc_mismatch() {
        // STX ACK "5" ':' "00" ETX — CRC "00" is wrong for the payload.
        let resp = vec![STX, ACK, b'5', b':', b'0', b'0', ETX];
        assert!(parse_reply(&resp).is_err());
    }

    #[test]
    fn parse_reply_xx_disables_crc_check() {
        // ":XX" marks the CRC as disabled; payload is accepted verbatim.
        let resp = vec![STX, ACK, b'9', b':', b'X', b'X', ETX];
        assert_eq!(parse_reply(&resp).unwrap(), "9");
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

    #[test]
    fn clamps_apply_controller_ranges() {
        assert_eq!(clamp_velocity(0.0, 1e9), (MIN_VELOCITY, MAX_VELOCITY));
        assert_eq!(clamp_velocity(-50.0, -50.0), (50.0, 50.0));
        assert_eq!(clamp_acceleration(10.0), MIN_ACCELERATION);
        assert_eq!(clamp_acceleration(1e9), MAX_ACCELERATION);
    }
}
