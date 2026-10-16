//! SmarAct SCU controller driver (ASCII, over an asyn octet serial port).
//!
//! Ported from `motorSmarAct/smarActApp/src/smarActSCUMotorDriver.cpp` (a
//! model-1-style `asynMotorController`/`asynMotorAxis` pair, configured with a
//! separate controller + per-axis create command). Communication is over a
//! `drvAsynSerialPort` (FTDI USB serial); commands and replies are `\n`
//! terminated by the port EOS, which the startup script sets both ways as C's
//! `smaractscu.iocsh` does.
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
//! The controller works in microns (linear) / millidegrees (rotary); the
//! record hands the driver raw steps and C's `STEPS_PER_EGU` (1000) makes one
//! micron or millidegree 1000 steps, so `MRES` = 0.001 reads in those units.
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
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi};

/// Response buffer size.
const READ_BUF: usize = 64;

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
// C getAngle tests `4 != sscanf` on a 3-conversion format, so C fails every
// rotary poll; rs parses the reply.
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

    /// Send a command and read back its reply (C `sendCmd`; every SCU
    /// command replies).
    fn transact(&self, io: &mut AxisIo, cmd: &str) -> AsynResult<String> {
        match self.handle.write_read(0, cmd.as_bytes(), READ_BUF) {
            Ok(raw) => {
                io.from_controller = String::from_utf8_lossy(&raw)
                    .trim_matches(['\r', '\n', '\0', ' '])
                    .to_string();
                Ok(io.from_controller.clone())
            }
            Err(e) => {
                // A failed writeRead leaves fromController_ holding the last reply.
                io.print(format_args!(
                    "ERROR: sendCmd: status={}, sent: {cmd}, received: {}\n",
                    e.status() as i32,
                    io.from_controller
                ));
                Err(e)
            }
        }
    }

    /// Read an integer parameter (`:<parm><ch>` → `:<CMD><axis><PARAM><int>`).
    fn query_int(&self, io: &mut AxisIo, parm: &str, channel: i32) -> AsynResult<i32> {
        let reply = self.transact(io, &format!(":{parm}{channel}"))?;
        let (cmd, value) =
            parse_kv(&reply).ok_or_else(|| io.parse_error("getIntegerVal", &reply))?;
        if cmd.starts_with('E') {
            return Err(scu_err(format!("SCU: controller error {reply:?}")));
        }
        Ok(atoi(&value))
    }

    /// Read a double parameter (`:<parm><ch>` → `:<CMD><axis><PARAM><float>`).
    fn query_double(&self, io: &mut AxisIo, parm: &str, channel: i32) -> AsynResult<f64> {
        let reply = self.transact(io, &format!(":{parm}{channel}"))?;
        let (cmd, value) =
            parse_kv(&reply).ok_or_else(|| io.parse_error("getDoubleVal", &reply))?;
        if cmd.starts_with('E') {
            return Err(scu_err(format!("SCU: controller error {reply:?}")));
        }
        Ok(atof(&value))
    }

    /// Read the single-character move status (`:M<ch>`).
    fn query_char(&self, io: &mut AxisIo, parm: &str, channel: i32) -> AsynResult<char> {
        let reply = self.transact(io, &format!(":{parm}{channel}"))?;
        let (cmd, value) =
            parse_char_reply(&reply).ok_or_else(|| io.parse_error("getCharVal", &reply))?;
        if cmd.starts_with('E') {
            return Err(scu_err(format!("SCU: controller error {reply:?}")));
        }
        Ok(value)
    }

    /// Read the rotary angle (`:GA<ch>` → angle, revolutions).
    fn query_angle(&self, io: &mut AxisIo, channel: i32) -> AsynResult<(f64, i64)> {
        let reply = self.transact(io, &format!(":GA{channel}"))?;
        parse_angle(&reply).ok_or_else(|| io.parse_error("getAngle", &reply))
    }
}

/// An axis's side of C `sendCmd`: the axis `pasynUser_` it prints on and
/// its own `fromController_`.
struct AxisIo {
    port: String,
    addr: i32,
    from_controller: String,
}

impl AxisIo {
    fn print(&self, args: std::fmt::Arguments<'_>) {
        PortManager::global().trace_manager().print(
            Some(&self.port),
            Some(self.addr),
            0,
            TraceMask::ERROR,
            args,
        );
    }

    /// C `get*Val`'s unparsable reply: printed, then asynError.
    fn parse_error(&self, getter: &str, reply: &str) -> AsynError {
        self.print(format_args!("{getter}:ERROR parsing response {reply}\n"));
        scu_err(format!("SCU: bad reply {reply:?}"))
    }
}

fn lock(controller: &Mutex<ScuController>) -> MutexGuard<'_, ScuController> {
    controller.lock().unwrap_or_else(|e| e.into_inner())
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
    io: AxisIo,
}

impl ScuAxis {
    /// Construct channel `channel`, probing max frequency, initial move status
    /// (for inherited infinite hold), positioner type and position, matching the
    /// C `SmarActSCUAxis` constructor. Performs blocking I/O.
    /// `port` and `axis` name the motor port's asyn user the axis prints on.
    ///
    /// Like C's `goto bail`, a failed probe keeps the axis with the values
    /// probed so far; the failure is returned alongside it.
    pub fn new(
        controller: Arc<Mutex<ScuController>>,
        channel: i32,
        port: &str,
        axis: i32,
    ) -> (Self, AsynResult<()>) {
        let mut ax = Self {
            controller,
            channel,
            is_rot: false,
            hold_time: 0,
            position_offset: 0.0,
            closed_loop: false,
            has_encoder: false,
            io: AxisIo {
                port: port.to_owned(),
                addr: axis,
                from_controller: String::new(),
            },
        };
        let probed = ax.probe();
        (ax, probed)
    }

    fn probe(&mut self) -> AsynResult<()> {
        let channel = self.channel;
        let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        let io = &mut self.io;

        // Probe max closed-loop frequency (also the first comms check).
        ctrl.query_int(io, "GCLF", channel)?;

        // Inherit infinite hold if the controller is still holding from a prior
        // life; otherwise start from the (initially disabled) closed-loop state.
        let move_status = ctrl.query_char(io, "M", channel)?;
        self.hold_time = if move_status == 'H' { HOLD_FOREVER } else { 0 };

        let positioner_type = ctrl.query_int(io, "GST", channel)?;
        self.is_rot = is_rotary_type(positioner_type);

        // A successful position read means the sensor knows the position.
        self.has_encoder = if self.is_rot {
            ctrl.query_angle(io, channel).is_ok()
        } else {
            ctrl.query_double(io, "GP", channel).is_ok()
        };
        Ok(())
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

        lock(&self.controller).transact(&mut self.io, &cmd)?;
        Ok(())
    }
}

impl AsynMotor for ScuAxis {
    /// C `SCUAxis::poll`'s `bail` label sets `motorStatusProblem_` and
    /// `motorStatusCommsError_`.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM_AND_COMMS_ERROR
    }

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
        lock(&self.controller).transact(&mut self.io, &format!(":MTR{ch}H{hold}Z0:GP{ch}"))?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ch = self.channel;
        lock(&self.controller).transact(&mut self.io, &format!(":S{ch}:GP{ch}"))?;
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
        let ctrl = lock(&self.controller);
        let io = &mut self.io;

        let raw = if self.is_rot {
            let (angle, rev) = ctrl.query_angle(io, ch)?;
            rev as f64 * UDEG_PER_REV as f64 + angle
        } else {
            ctrl.query_double(io, "GP", ch)?
        };
        let position = (raw + self.position_offset) * STEPS_PER_EGU;

        let status = parse_moving_status(ctrl.query_char(io, "M", ch)?);
        let moving = match status {
            MovingStatus::Holding => self.hold_time != HOLD_FOREVER,
            MovingStatus::Targeting
            | MovingStatus::Moving
            | MovingStatus::Calibrating
            | MovingStatus::Referencing => true,
            MovingStatus::Stopped | MovingStatus::AmplSetting | MovingStatus::Unknown => false,
        };

        let homed = ctrl.query_int(io, "GPPK", ch)? != 0;
        drop(ctrl);

        Ok(MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0,
            done: !moving,
            moving,
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

    /// smarActSCUMotorDriver.cpp:185-193: a failed probe bails and the axis
    /// stays, its poll then failing on the same user.
    #[test]
    fn an_axis_of_an_unconnected_controller_is_kept() {
        let (handle, status) =
            SyncIOHandle::connect_port("scu-missing-port", 0, std::time::Duration::from_millis(10));
        assert!(status.is_err());
        let controller = Arc::new(Mutex::new(ScuController::new(handle)));
        let (mut axis, probed) = ScuAxis::new(controller, 3, "scu-motor", 1);
        assert!(probed.is_err());
        assert_eq!(axis.channel, 3);
        assert!(!axis.is_rot && !axis.has_encoder);
        assert_eq!(axis.hold_time, 0);
        assert!(axis.poll(&AsynUser::new(0)).is_err());
    }

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
