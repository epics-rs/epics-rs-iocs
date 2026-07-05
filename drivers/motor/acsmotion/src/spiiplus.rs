//! ACS SPiiPlus motion controller driver (serial/TCP ASCII, ACSPL+).
//!
//! Ported from `motorAcsMotion/acsMotionApp/src/SPiiPlusDriver.cpp` +
//! `SPiiPlusCommDriver.cpp` (the model-3 `asynMotorController`/`asynMotorAxis`
//! driver). Commands are ACSPL+ text terminated by CR (`\r`); the driver owns
//! output framing and the startup script sets only the input EOS (`\r`), as in
//! the C module's `ACS_Motion.iocsh` (`asynOctetSetInputEos ... "\r"`). A reply
//! whose first character is `?` is a controller error (`?<errno>`), matching the
//! C `writeReadAck`/`writeReadDouble` check `inString[0] == '?'`. Axes are
//! addressed by 0-based index on the wire.
//!
//! ## Poll transport (deviation, documented)
//!
//! The C controller `poll()` reads all axes at once with the binary
//! SPiiPlusBinComm protocol (`getDoubleArray`/`getIntegerArray`). This port
//! replaces that bulk transport with per-axis ASCII single-element queries
//! (`?APOS(n)`, `?FPOS(n)`, `?MST(n)`, `?FAULT(n)`, `?MFLAGS(n)`, `?FVEL(n)`,
//! `?AST(n)`) issued from each axis's [`poll`](AsynMotor::poll). The values are
//! identical — the binary protocol is purely a bulk-transfer optimization with
//! no extra semantics — so only the number of round-trips differs. The binary
//! comm layer (SPiiPlusBinComm), profile/PVT moves, the profile thread, and the
//! SPiiPlusAux driver are therefore not modeled.
//!
//! ## Units
//!
//! The C driver scales every record value by `resolution_` (the axis
//! stepper/encoder factor) to bridge the motor record's raw-step frame to the
//! controller's native units, and divides back in `poll` (`APOS/resolution_`).
//! At the asyn-rs motor boundary (EGU frame) that scale cancels, so this port
//! drops it and works in the controller's native user units: positions pass
//! through unscaled, the record's `MRES` is 1, and its `EGU` is the controller
//! unit (typically mm). Consequently the encoder readback is `?FPOS` directly
//! (already in user units), not `FPOS/EFAC` as in the raw-step C frame.
//!
//! ## Homing (deviation, documented)
//!
//! C `home()` reads a per-axis mbbo homing method PV (`SPiiPlusHomingMethod_`,
//! default `NONE` → home errors) plus offset/max-distance/current-limit PVs, and
//! emits `HOME axis,method,vel,[maxdist],offset[,currlimit]`. Those aux PVs are
//! not present at the plain motor boundary, so the homing *method* is supplied
//! once at controller-configuration time (default [`HOME_LIMIT_INDEX`], resolved
//! to the negative/positive limit+index code by the record's home direction).
//! The offset/max-distance/current-limit are fixed at 0 (max-distance emitted as
//! the empty field the controller treats as "unbounded", current-limit omitted),
//! matching the C behavior when those PVs are left at their defaults.
//!
//! ## Not modeled (documented)
//!
//! SPiiPlusBinComm binary array protocol (replaced by ASCII queries), profile /
//! PVT moves and the profile thread, the SPiiPlusAux driver, and the ~30 aux
//! asyn parameters (MFLAGS/MFLAGSX bit fields, safe-torque-off, encoder offsets
//! and factors, reference position, PEG, disable-set-position guard, global
//! variable read/write, ACSPL program start/stop).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi};

/// Response buffer size (C `MAX_CONTROLLER_STRING_SIZE` is 256).
const READ_BUF: usize = 256;

/// Command terminator (C port EOS `\r`); the driver owns output framing.
const TERMINATOR: &[u8] = b"\r";

/// Motor-status (`MST`) bit masks (C `SPiiPlusDriver::poll`).
const MST_ENABLED: i32 = 1 << 0;
const MST_MOTION: i32 = 1 << 5;

/// Axis-status (`AST`) motion bit, used for dummy/virtual axes (C `poll`).
const AST_MOTION: i32 = 1 << 5;

/// Fault-word (`FAULT`) bit masks (C `SPIIPLUS_FAULT_*`). NOTE: bit 0 is the
/// RIGHT (high) limit and bit 1 is the LEFT (low) limit — the opposite ordering
/// to the older motorAcsTech80 module; follow this module's own definitions.
const FAULT_HARD_RIGHT_LIMIT: i32 = 1 << 0;
const FAULT_HARD_LEFT_LIMIT: i32 = 1 << 1;

/// Motor-flags (`MFLAGS`) bit masks (C `SPIIPLUS_MFLAGS_*`).
const MFLAGS_DUMMY: i32 = 1 << 0;
const MFLAGS_HOME: i32 = 1 << 3;

/// mbbo homing-method selectors (C `MBBO_HOME_*`), supplied at config time.
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

fn spiiplus_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Format a floating-point command argument as a plain decimal (never
/// scientific), matching the controller's ASCII numeric parsing.
fn fmt(v: f64) -> String {
    // Rust's f64 `Display` emits a shortest round-trip decimal with no exponent,
    // which the SPiiPlus command interpreter accepts directly.
    format!("{v}")
}

/// Shared controller endpoint: owns the octet handle, the version string, the
/// axis count, and the configuration-time homing method.
pub struct SpiiPlusController {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
    homing_method: i32,
}

impl SpiiPlusController {
    /// Connect and identify a SPiiPlus controller (C `SPiiPlusController`
    /// constructor): probe `?VR` (retry up to 3 times) and read the firmware
    /// version. `num_axes` and `homing_method` come from the config command.
    /// Performs blocking octet I/O.
    pub fn new(handle: SyncIOHandle, num_axes: usize, homing_method: i32) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes,
            homing_method,
        };

        let mut probed = false;
        for _ in 0..3 {
            if let Ok(v) = ctrl.command("?VR")
                && !v.is_empty()
            {
                ctrl.ident = v;
                probed = true;
                break;
            }
        }
        if !probed {
            return Err(spiiplus_err("SPiiPlus: no valid response to ?VR probe"));
        }
        Ok(ctrl)
    }

    /// The firmware version string (`?VR`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes (from the config command).
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Send a command and return its reply (trimmed of CR/LF/NUL/`:` prompt and
    /// spaces). A reply whose first character is `?` is a controller error
    /// (`?<errno>`), returned as an [`AsynError`] (C `writeReadAck`). A
    /// successful set/motion command replies with only the prompt (empty after
    /// trimming); a query replies with the value.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        let text = text.trim_matches(['\r', '\n', '\0', ':', ' ']).to_string();
        if text.starts_with('?') {
            return Err(spiiplus_err(format!(
                "SPiiPlus command failed: {cmd} -> {text}"
            )));
        }
        Ok(text)
    }

    /// Read a single integer array element `?VAR(index)` (0 on empty reply).
    fn query_int(&self, var: &str, index: usize) -> AsynResult<i32> {
        Ok(atoi(&self.command(&format!("?{var}({index})"))?))
    }

    /// Read a single double array element `?VAR(index)` (0.0 on empty reply).
    fn query_double(&self, var: &str, index: usize) -> AsynResult<f64> {
        Ok(atof(&self.command(&format!("?{var}({index})"))?))
    }
}

/// One SPiiPlus axis sharing a controller. Implements [`AsynMotor`].
pub struct SpiiPlusAxis {
    controller: Arc<Mutex<SpiiPlusController>>,
    /// 0-based axis index (used directly on the wire).
    axis: usize,
    /// True for a virtual axis (from the config `virtualAxisList`) or a dummy
    /// axis (MFLAGS `#DUMMY` bit, read once at construction). Reduced axes have
    /// no limits/faults, take motion from `AST`, and report the commanded
    /// position as their encoder position.
    reduced: bool,
    /// True only for a config-listed virtual axis (home / set-position error, as
    /// in C, independent of the dummy path).
    is_virtual: bool,
    /// Last non-zero feedback-velocity direction (C updates direction only while
    /// the axis is moving).
    last_direction: bool,
}

impl SpiiPlusAxis {
    /// Construct axis `index` (0-based). `is_virtual` comes from the config
    /// `virtualAxisList`; the dummy flag is read from `?MFLAGS(index)` here (C
    /// reads MFLAGS at controller init). A probe failure leaves the axis
    /// non-dummy, matching a zero MFLAGS read.
    pub fn new(controller: Arc<Mutex<SpiiPlusController>>, index: usize, is_virtual: bool) -> Self {
        let dummy = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            ctrl.query_int("MFLAGS", index)
                .map(|f| (f & MFLAGS_DUMMY) != 0)
                .unwrap_or(false)
        };
        Self {
            controller,
            axis: index,
            reduced: dummy || is_virtual,
            is_virtual,
            last_direction: true,
        }
    }

    fn lock(&self) -> MutexGuard<'_, SpiiPlusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Send the ACC/DEC/VEL preamble shared by point-to-point and jog moves
    /// (C `move`/`moveVelocity`): `ACC(n)=`, `DEC(n)=`, `VEL(n)=`.
    fn set_move_params(
        ctrl: &SpiiPlusController,
        axis: usize,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        ctrl.command(&format!("ACC({axis})={}", fmt(acceleration)))?;
        ctrl.command(&format!("DEC({axis})={}", fmt(acceleration)))?;
        ctrl.command(&format!("VEL({axis})={}", fmt(velocity)))?;
        Ok(())
    }
}

impl AsynMotor for SpiiPlusAxis {
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
        Self::set_move_params(&ctrl, a, velocity, acceleration)?;
        ctrl.command(&format!("PTP {a}, {}", fmt(position)))?;
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
        Self::set_move_params(&ctrl, a, velocity, acceleration)?;
        ctrl.command(&format!("PTP/r {a}, {}", fmt(distance)))?;
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
        ctrl.command(&format!("ACC({a})={}", fmt(acceleration)))?;
        ctrl.command(&format!("DEC({a})={}", fmt(acceleration)))?;
        let dir = if velocity > 0.0 { '+' } else { '-' };
        ctrl.command(&format!("JOG/v {a}, {}, {dir}", fmt(velocity.abs())))?;
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
        let code = home_code(ctrl.homing_method, forward).ok_or_else(|| {
            spiiplus_err(format!(
                "SPiiPlus: no homing method configured for axis {a} (homingMethod=none)"
            ))
        })?;
        // HOME axis,method,vel,[maxdist],offset — maxdist empty (unbounded),
        // offset 0, current-limit omitted (C defaults when the aux PVs are 0).
        ctrl.command(&format!("HOME {a},{code},{},,0", fmt(velocity)))?;
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
        ctrl.command(&format!("SET RPOS({})={}", self.axis, fmt(position)))?;
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

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let a = self.axis;

        let apos = ctrl.query_double("APOS", a)?;
        let velocity = ctrl.query_double("FVEL", a)?;

        let (moving, powered, high_limit, low_limit, homed, encoder) = if self.reduced {
            // Dummy/virtual: motion from AST, no faults, encoder = commanded pos.
            let ast = ctrl.query_int("AST", a)?;
            ((ast & AST_MOTION) != 0, false, false, false, false, apos)
        } else {
            let mst = ctrl.query_int("MST", a)?;
            let fault = ctrl.query_int("FAULT", a)?;
            let mflags = ctrl.query_int("MFLAGS", a)?;
            let fpos = ctrl.query_double("FPOS", a)?;
            (
                (mst & MST_MOTION) != 0,
                (mst & MST_ENABLED) != 0,
                (fault & FAULT_HARD_RIGHT_LIMIT) != 0,
                (fault & FAULT_HARD_LEFT_LIMIT) != 0,
                (mflags & MFLAGS_HOME) != 0,
                fpos,
            )
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

#[cfg(test)]
mod tests {
    use super::*;

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
    fn fmt_is_plain_decimal_not_scientific() {
        assert_eq!(fmt(1000.0), "1000");
        assert_eq!(fmt(12.5), "12.5");
        assert_eq!(fmt(-0.25), "-0.25");
        assert_eq!(fmt(0.0000001), "0.0000001");
    }
}
