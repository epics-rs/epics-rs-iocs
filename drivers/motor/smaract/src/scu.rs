//! SmarAct SCU controller driver (ASCII, over an asyn octet serial port).
//!
//! Ported from `motorSmarAct/smarActApp/src/smarActSCUMotorDriver.cpp` (a
//! model-1-style `asynMotorController`/`asynMotorAxis` pair, configured with a
//! separate controller + per-axis create command). Communication is over a
//! `drvAsynSerialPort` (FTDI USB serial); commands and replies are `\n`
//! terminated (the startup script sets the input EOS; the driver owns output
//! framing).
//!
//! ## Protocol
//!
//! Every command is colon-prefixed and channel-addressed and returns a reply
//! that is read back (`writeReadController`). Query replies have the shape
//! `:<CMD><axis><PARAM><value>` for integers/doubles (e.g. `GCLF`, `GST`, `GP`,
//! `GPPK`), `:<CMD><axis><char>` for the single-character move status (`M`), and
//! `:A<axis>A<angle>R<rev>` for the rotary angle (`GA`). A reply whose command
//! letters begin with `E` signals a controller error. Move/home/stop commands
//! append a trailing `:GP<ch>` position query, so their reply is a position
//! readback that the driver discards.
//!
//! ## Linear vs rotary stages
//!
//! The positioner type (`GST`) selects the frame: rotary types use `GA`
//! (angle + revolution) and `MAA`/`MAR` moves; linear types use `GP` and
//! `MPA`/`MPR`. `poll` folds `rev * UDEG_PER_REV + angle` into a single position.
//!
//! ## Units
//!
//! Per the SCU README, the controller works in microns (linear) / millidegrees
//! (rotary) and the driver deliberately reports motor-record *steps* scaled by
//! `STEPS_PER_EGU` (1000): 1000 steps is one micron or one millidegree. This is
//! an intentional unit choice (not a resolution bridge), so it is preserved —
//! positions cross the driver boundary in steps, `MRES` = 1, and setting
//! `MRES` = 0.001 makes the record read in microns/millidegrees.
//!
//! ## Hold time
//!
//! The SCU can actively hold the target after a move. The driver mirrors C:
//! `holdTime` is `HOLD_FOREVER` when closed-loop (record `CNEN`) is enabled and
//! `0` otherwise, cached at each move. In `poll`, the `Holding` state counts as
//! "not moving" only under infinite hold; a finite hold keeps the move
//! incomplete until it expires.
//!
//! ## Software position offset
//!
//! The SCU has no set-position command; `set_position` records a software
//! offset (`position / STEPS_PER_EGU`) that is added to readback and subtracted
//! from move targets, matching C `setPosition`/`positionOffset_`.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi};

/// Response buffer size.
const READ_BUF: usize = 64;

/// Command terminator; the driver owns output framing.
const TERMINATOR: &[u8] = b"\n";

/// Hold-target-forever time (ms) — used in closed-loop mode.
const HOLD_FOREVER: u32 = 60000;

/// "Move a long way" target for velocity moves (steps, C `FAR_AWAY` in nm).
const FAR_AWAY: f64 = 1_000_000_000.0;

/// Micro-degrees per revolution (C `UDEG_PER_REV`).
const UDEG_PER_REV: i64 = 360_000_000;

/// Motor-record steps per controller EGU (micron / millidegree).
const STEPS_PER_EGU: f64 = 1000.0;

/// SCU move-status characters (C `parseMovingStatus`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum MovingStatus {
    Stopped,
    AmplSetting,
    Moving,
    Targeting,
    Holding,
    Calibrating,
    Referencing,
    Unknown,
}

fn parse_moving_status(c: char) -> MovingStatus {
    match c {
        'S' => MovingStatus::Stopped,
        'A' => MovingStatus::AmplSetting,
        'M' => MovingStatus::Moving,
        'T' => MovingStatus::Targeting,
        'H' => MovingStatus::Holding,
        'C' => MovingStatus::Calibrating,
        'R' => MovingStatus::Referencing,
        _ => MovingStatus::Unknown,
    }
}

/// Is this positioner type a rotation stage (C `SmarActSCUAxis` constructor)?
fn is_rotary_type(t: i32) -> bool {
    matches!(t, 2 | 8 | 14 | 20 | 22 | 23) || (25..=29).contains(&t)
}

fn scu_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Split a `:<CMD><axis><PARAM><value>` reply into (command letters, value
/// string). Returns `None` if the shape does not match.
fn parse_kv(reply: &str) -> Option<(String, String)> {
    let s = reply.trim().strip_prefix(':')?;
    let b = s.as_bytes();
    let mut i = 0;
    let cmd_start = i;
    while i < b.len() && b[i].is_ascii_uppercase() {
        i += 1;
    }
    if i == cmd_start {
        return None;
    }
    let cmd = s[cmd_start..i].to_string();
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let dig_start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    if i == dig_start {
        return None;
    }
    // Parameter letters (present for integer/double replies).
    while i < b.len() && b[i].is_ascii_uppercase() {
        i += 1;
    }
    Some((cmd, s[i..].trim().to_string()))
}

/// Split a `:<CMD><axis><char>` reply (the move-status form, no parameter
/// letters) into (command letters, the single status character).
fn parse_char_reply(reply: &str) -> Option<(String, char)> {
    let s = reply.trim().strip_prefix(':')?;
    let b = s.as_bytes();
    let mut i = 0;
    let cmd_start = i;
    while i < b.len() && b[i].is_ascii_uppercase() {
        i += 1;
    }
    if i == cmd_start {
        return None;
    }
    let cmd = s[cmd_start..i].to_string();
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let dig_start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    if i == dig_start {
        return None;
    }
    let value = s[i..].trim().chars().next()?;
    Some((cmd, value))
}

/// Parse a `:A<axis>A<angle>R<rev>` rotary reply into (angle, revolutions).
fn parse_angle(reply: &str) -> Option<(f64, i64)> {
    let s = reply.trim().strip_prefix(":A")?;
    let a = s.find('A')?;
    let rest = &s[a + 1..];
    let r = rest.find('R')?;
    let angle = atof(rest[..r].trim());
    let rev = atoi(rest[r + 1..].trim()) as i64;
    Some((angle, rev))
}

/// Shared SCU controller endpoint owning the asyn octet handle.
pub struct ScuController {
    handle: SyncIOHandle,
}

impl ScuController {
    /// Wrap a connected octet handle. The C controller constructor only
    /// connects and starts the poller — all probing happens per axis.
    pub fn new(handle: SyncIOHandle) -> Self {
        Self { handle }
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Send a command and read back its reply (every SCU command replies).
    fn transact(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string())
    }

    /// Read an integer parameter (`:<parm><ch>` → `:<CMD><axis><PARAM><int>`).
    fn query_int(&self, parm: &str, channel: i32) -> AsynResult<i32> {
        let reply = self.transact(&format!(":{parm}{channel}"))?;
        let (cmd, value) =
            parse_kv(&reply).ok_or_else(|| scu_err(format!("SCU: bad reply {reply:?}")))?;
        if cmd.starts_with('E') {
            return Err(scu_err(format!("SCU: controller error {reply:?}")));
        }
        Ok(atoi(&value))
    }

    /// Read a double parameter (`:<parm><ch>` → `:<CMD><axis><PARAM><float>`).
    fn query_double(&self, parm: &str, channel: i32) -> AsynResult<f64> {
        let reply = self.transact(&format!(":{parm}{channel}"))?;
        let (cmd, value) =
            parse_kv(&reply).ok_or_else(|| scu_err(format!("SCU: bad reply {reply:?}")))?;
        if cmd.starts_with('E') {
            return Err(scu_err(format!("SCU: controller error {reply:?}")));
        }
        Ok(atof(&value))
    }

    /// Read the single-character move status (`:M<ch>`).
    fn query_char(&self, parm: &str, channel: i32) -> AsynResult<char> {
        let reply = self.transact(&format!(":{parm}{channel}"))?;
        let (cmd, value) =
            parse_char_reply(&reply).ok_or_else(|| scu_err(format!("SCU: bad reply {reply:?}")))?;
        if cmd.starts_with('E') {
            return Err(scu_err(format!("SCU: controller error {reply:?}")));
        }
        Ok(value)
    }

    /// Read the rotary angle (`:GA<ch>` → angle, revolutions).
    fn query_angle(&self, channel: i32) -> AsynResult<(f64, i64)> {
        let reply = self.transact(&format!(":GA{channel}"))?;
        parse_angle(&reply).ok_or_else(|| scu_err(format!("SCU: bad angle reply {reply:?}")))
    }
}

/// One SCU channel sharing a controller. Implements [`AsynMotor`].
pub struct ScuAxis {
    controller: Arc<Mutex<ScuController>>,
    /// Wire channel (from the create-axis command; distinct from the axis no.).
    channel: i32,
    /// Rotary vs linear stage (from the positioner type at construction).
    is_rot: bool,
    /// Cached hold time applied at each move (closed-loop → `HOLD_FOREVER`).
    hold_time: u32,
    /// Software position offset (steps / `STEPS_PER_EGU`), C `positionOffset_`.
    position_offset: f64,
    /// Closed-loop (record `CNEN`) state, updated by `set_closed_loop`.
    closed_loop: bool,
    /// Sensor-present flag probed at construction.
    has_encoder: bool,
}

impl ScuAxis {
    /// Construct channel `channel`, probing max frequency, initial move status
    /// (for inherited infinite hold), positioner type and position, matching the
    /// C `SmarActSCUAxis` constructor. Performs blocking I/O.
    pub fn new(controller: Arc<Mutex<ScuController>>, channel: i32) -> AsynResult<Self> {
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());

        // Probe max closed-loop frequency (also the first comms check).
        ctrl.query_int("GCLF", channel)?;

        // Inherit infinite hold if the controller is still holding from a prior
        // life; otherwise start from the (initially disabled) closed-loop state.
        let move_status = ctrl.query_char("M", channel)?;
        let hold_time = if move_status == 'H' { HOLD_FOREVER } else { 0 };

        let positioner_type = ctrl.query_int("GST", channel)?;
        let is_rot = is_rotary_type(positioner_type);

        // A successful position read means the sensor knows the position.
        let has_encoder = if is_rot {
            ctrl.query_angle(channel).is_ok()
        } else {
            ctrl.query_double("GP", channel).is_ok()
        };

        drop(ctrl);
        Ok(Self {
            controller,
            channel,
            is_rot,
            hold_time,
            position_offset: 0.0,
            closed_loop: false,
            has_encoder,
        })
    }

    fn lock(&self) -> MutexGuard<'_, ScuController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Shared move: cache the hold time, convert to controller units minus the
    /// software offset, and issue the linear or rotary move (with a trailing
    /// position query the controller answers and we discard).
    fn do_move(&mut self, position: f64, relative: bool) -> AsynResult<()> {
        self.hold_time = if self.closed_loop { HOLD_FOREVER } else { 0 };
        let hold = self.hold_time;
        let ch = self.channel;
        let rpos = (position / STEPS_PER_EGU) - self.position_offset;

        let cmd = if self.is_rot {
            let mut angle = (rpos as i64 % UDEG_PER_REV) as f64;
            let mut rev = (rpos / UDEG_PER_REV as f64) as i32;
            if angle < 0.0 {
                angle += UDEG_PER_REV as f64;
                rev -= 1;
            }
            let mnem = if relative { "MAR" } else { "MAA" };
            format!(":{mnem}{ch}A{angle:.3}R{rev}H{hold}:GP{ch}")
        } else {
            let mnem = if relative { "MPR" } else { "MPA" };
            format!(":{mnem}{ch}P{rpos:.3}H{hold}:GP{ch}")
        };

        let ctrl = self.lock();
        ctrl.transact(&cmd)?;
        Ok(())
    }
}

impl AsynMotor for ScuAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true)
    }

    fn move_velocity(
        &mut self,
        user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // No native jog: a relative move to a very-far target (C moveVelocity).
        if velocity == 0.0 {
            return self.stop(user, acceleration);
        }
        let target = if velocity < 0.0 { -FAR_AWAY } else { FAR_AWAY };
        self.do_move(target, true)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // The SCU homes to its reference mark (Z0); direction is not selectable.
        self.hold_time = if self.closed_loop { HOLD_FOREVER } else { 0 };
        let hold = self.hold_time;
        let ch = self.channel;
        let ctrl = self.lock();
        ctrl.transact(&format!(":MTR{ch}H{hold}Z0:GP{ch}"))?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ch = self.channel;
        let ctrl = self.lock();
        ctrl.transact(&format!(":S{ch}:GP{ch}"))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // SCU has no set-position command; record a software offset.
        self.position_offset = position / STEPS_PER_EGU;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // Only affects the hold time applied at the next move.
        self.closed_loop = enable;
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // Not implemented in the C SCU driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ch = self.channel;
        let ctrl = self.lock();

        let raw = if self.is_rot {
            let (angle, rev) = ctrl.query_angle(ch)?;
            rev as f64 * UDEG_PER_REV as f64 + angle
        } else {
            ctrl.query_double("GP", ch)?
        };
        let position = (raw + self.position_offset) * STEPS_PER_EGU;

        let status = parse_moving_status(ctrl.query_char("M", ch)?);
        let moving = match status {
            MovingStatus::Holding => self.hold_time != HOLD_FOREVER,
            MovingStatus::Targeting
            | MovingStatus::Moving
            | MovingStatus::Calibrating
            | MovingStatus::Referencing => true,
            MovingStatus::Stopped | MovingStatus::AmplSetting | MovingStatus::Unknown => false,
        };

        let homed = ctrl.query_int("GPPK", ch)? != 0;
        drop(ctrl);

        Ok(MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0,
            done: !moving,
            moving,
            direction: true,
            has_encoder: self.has_encoder,
            gain_support: self.has_encoder,
            homed,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_integer_and_double_replies() {
        // :<CMD><axis><PARAM><value>
        let (cmd, v) = parse_kv(":CLF0F1000").unwrap();
        assert_eq!(cmd, "CLF");
        assert_eq!(atoi(&v), 1000);
        let (cmd, v) = parse_kv(":P0P-12.500").unwrap();
        assert_eq!(cmd, "P");
        assert_eq!(atof(&v), -12.5);
        // Error replies begin with 'E'.
        let (cmd, _) = parse_kv(":E0X0").unwrap();
        assert!(cmd.starts_with('E'));
    }

    #[test]
    fn parses_char_and_angle_replies() {
        let (cmd, c) = parse_char_reply(":M0H").unwrap();
        assert_eq!(cmd, "M");
        assert_eq!(c, 'H');
        assert_eq!(parse_moving_status(c), MovingStatus::Holding);
        let (angle, rev) = parse_angle(":A0A123.456R2").unwrap();
        assert_eq!(angle, 123.456);
        assert_eq!(rev, 2);
    }

    #[test]
    fn rotary_type_classification() {
        for t in [2, 8, 14, 20, 22, 23, 25, 27, 29] {
            assert!(is_rotary_type(t), "type {t} should be rotary");
        }
        for t in [1, 3, 24, 30] {
            assert!(!is_rotary_type(t), "type {t} should be linear");
        }
    }
}
