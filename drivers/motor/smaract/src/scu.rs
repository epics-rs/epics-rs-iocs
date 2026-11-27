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

use std::cell::{Cell, RefCell};
use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, StatusWrite,
};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_libcom_rs::cfmt::CFloat;
use epics_libcom_rs::runtime::scanf::{ScanTarget, sscanf};

/// Response buffer size.
const READ_BUF: usize = 64;

/// C `REP_LEN`, the size of the `cmd` and `param` buffers replies scan into.
const REP_LEN: usize = 50;

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

/// C `sscanf(reply, fmt, cmd, &axis, ..)` over `cmd[REP_LEN]`, the reply's
/// own targets `rest` after the axis: whether `cmd` starts with `E`, or
/// `None` unless every conversion succeeds. The axis is read at 64 bits, so
/// that one C wraps into its `int` is `None`.
fn scan_reply(reply: &str, fmt: &str, rest: &[&dyn ScanTarget]) -> Option<bool> {
    let cmd = RefCell::new([0u8; REP_LEN]);
    let axis = Cell::new(0i64);
    let mut out: Vec<&dyn ScanTarget> = vec![&cmd, &axis];
    out.extend_from_slice(rest);
    (sscanf(reply, fmt, &out) == out.len() as i32 && i32::try_from(axis.get()).is_ok())
        .then(|| cmd.borrow()[0] == b'E')
}

/// A `:<CMD><axis><PARAM><int>` reply as C `getIntegerVal` scans it, into
/// (is-error-command, value), the value read at 64 bits as the axis is.
fn parse_int_reply(reply: &str) -> Option<(bool, i32)> {
    let (param, val) = (RefCell::new([0u8; REP_LEN]), Cell::new(0i64));
    let is_err = scan_reply(reply, ":%10[A-Z]%lli%10[A-Z]%lli", &[&param, &val])?;
    Some((is_err, val.get().try_into().ok()?))
}

/// A `:<CMD><axis><PARAM><float>` reply as C `getDoubleVal` scans it.
fn parse_double_reply(reply: &str) -> Option<(bool, f64)> {
    let (param, val) = (RefCell::new([0u8; REP_LEN]), Cell::new(0f64));
    let is_err = scan_reply(reply, ":%10[A-Z]%lli%10[A-Z]%lf", &[&param, &val])?;
    Some((is_err, val.get()))
}

/// A `:<CMD><axis><char>` move-status reply as C `getCharVal` scans it: the
/// character right after the axis.
fn parse_char_reply(reply: &str) -> Option<(bool, char)> {
    let val = Cell::new(0u8);
    let is_err = scan_reply(reply, ":%10[A-Z]%lli%c", &[&val])?;
    Some((is_err, val.get().into()))
}

/// A `:A<axis>A<angle>R<rev>` rotary reply as C `getAngle` scans it, into
/// (angle, revolutions), the numbers read at 64 bits.
// C getAngle tests `4 != sscanf` on a 3-conversion format, so C fails every
// rotary poll; rs parses the reply.
fn parse_angle(reply: &str) -> Option<(f64, i64)> {
    let (axis, angle, rev) = (Cell::new(0i64), Cell::new(0f64), Cell::new(0i64));
    if sscanf(reply, ":A%lliA%lfR%lli", &[&axis, &angle, &rev]) != 3 {
        return None;
    }
    i32::try_from(axis.get()).ok()?;
    Some((angle.get(), i32::try_from(rev.get()).ok()?.into()))
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
        match self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts()
        {
            (raw, Ok(())) => {
                io.from_controller = String::from_utf8_lossy(&raw)
                    .trim_matches(['\r', '\n', '\0', ' '])
                    .to_string();
                Ok(io.from_controller.clone())
            }
            (_, Err(e)) => {
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
        let (is_err, value) =
            parse_int_reply(&reply).ok_or_else(|| io.parse_error("getIntegerVal", &reply))?;
        if is_err {
            return Err(scu_err(format!("SCU: controller error {reply:?}")));
        }
        Ok(value)
    }

    /// Read a double parameter (`:<parm><ch>` → `:<CMD><axis><PARAM><float>`).
    fn query_double(&self, io: &mut AxisIo, parm: &str, channel: i32) -> AsynResult<f64> {
        let reply = self.transact(io, &format!(":{parm}{channel}"))?;
        let (is_err, value) =
            parse_double_reply(&reply).ok_or_else(|| io.parse_error("getDoubleVal", &reply))?;
        if is_err {
            return Err(scu_err(format!("SCU: controller error {reply:?}")));
        }
        Ok(value)
    }

    /// Read the single-character move status (`:M<ch>`).
    fn query_char(&self, io: &mut AxisIo, parm: &str, channel: i32) -> AsynResult<char> {
        let reply = self.transact(io, &format!(":{parm}{channel}"))?;
        let (is_err, value) =
            parse_char_reply(&reply).ok_or_else(|| io.parse_error("getCharVal", &reply))?;
        if is_err {
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
    io: AxisIo,
    /// The status bits the constructor or the last failed command set.
    status_writes: Vec<StatusWrite>,
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
            io: AxisIo {
                port: port.to_owned(),
                addr: axis,
                from_controller: String::new(),
            },
            status_writes: Vec::new(),
        };
        let probed = ax.probe();
        let probed = ax.bail(probed);
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
        let has_encoder = if self.is_rot {
            ctrl.query_angle(io, channel).is_ok()
        } else {
            ctrl.query_double(io, "GP", channel).is_ok()
        };
        if has_encoder {
            self.status_writes.extend([
                StatusWrite::HasEncoder(true),
                StatusWrite::GainSupport(true),
            ]);
        }
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
            format!(":{mnem}{ch}A{:.3}R{rev}H{hold}:GP{ch}", CFloat(angle))
        } else {
            let mnem = if relative { "MPR" } else { "MPA" };
            format!(":{mnem}{ch}P{:.3}H{hold}:GP{ch}", CFloat(rpos))
        };

        lock(&self.controller)
            .transact(&mut self.io, &cmd)
            .map(drop)
    }

    /// C's check after a command: a failure raises `motorStatusProblem_`
    /// and `motorStatusCommsError_` (smarActSCUMotorDriver.cpp:184-186,
    /// 446-450, 467-471, 485-489).
    fn bail(&mut self, result: AsynResult<()>) -> AsynResult<()> {
        if result.is_err() {
            self.status_writes
                .extend([StatusWrite::Problem(true), StatusWrite::CommsError(true)]);
        }
        result
    }
}

impl AsynMotor for ScuAxis {
    /// The C constructor calls `callParamCallbacks` past `bail:`, before
    /// any throw (smarActSCUMotorDriver.cpp:188).
    fn constructor_calls_param_callbacks(&self) -> bool {
        true
    }

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
        let moved = self.do_move(position, false);
        self.bail(moved)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let moved = self.do_move(distance, true);
        self.bail(moved)
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
        let moved = self.do_move(target, true);
        self.bail(moved)
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
        let homed = lock(&self.controller)
            .transact(&mut self.io, &format!(":MTR{ch}H{hold}Z0:GP{ch}"))
            .map(drop);
        self.bail(homed)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ch = self.channel;
        let stopped = lock(&self.controller)
            .transact(&mut self.io, &format!(":S{ch}:GP{ch}"))
            .map(drop);
        self.bail(stopped)
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

    fn poll(&mut self, _user: &AsynUser, out: &mut MotorStatus) -> AsynResult<()> {
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
        out.set_encoder_position(position);
        out.set_position(position);

        let status = parse_moving_status(ctrl.query_char(io, "M", ch)?);
        let moving = match status {
            MovingStatus::Holding => self.hold_time != HOLD_FOREVER,
            MovingStatus::Targeting
            | MovingStatus::Moving
            | MovingStatus::Calibrating
            | MovingStatus::Referencing => true,
            MovingStatus::Stopped | MovingStatus::AmplSetting | MovingStatus::Unknown => false,
        };
        out.moving = moving;
        out.done = !moving;

        out.homed = ctrl.query_int(io, "GPPK", ch)? != 0;
        drop(ctrl);

        out.problem = false;
        out.comms_error = false;
        Ok(())
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;
    use motor_common::testing::Polled;

    /// smarActSCUMotorDriver.cpp:184-188: past `bail:`, so a failed probe
    /// calls back too.
    #[test]
    fn the_constructor_calls_param_callbacks() {
        let (handle, _) = SyncIOHandle::connect_port(
            "scu-ctor-callback",
            0,
            std::time::Duration::from_millis(10),
            InterfaceType::Octet,
        );
        let controller = Arc::new(Mutex::new(ScuController::new(handle)));
        let (axis, probed) = ScuAxis::new(controller, 0, "scu-ctor-callback", 0);
        assert!(probed.is_err());
        assert!(axis.constructor_calls_param_callbacks());
    }

    /// smarActSCUMotorDriver.cpp:185-193: a failed probe bails and the axis
    /// stays, its poll then failing on the same user.
    #[test]
    fn an_axis_of_an_unconnected_controller_is_kept() {
        let (handle, status) = SyncIOHandle::connect_port(
            "scu-missing-port",
            0,
            std::time::Duration::from_millis(10),
            InterfaceType::Octet,
        );
        assert!(status.is_err());
        let controller = Arc::new(Mutex::new(ScuController::new(handle)));
        let (mut axis, probed) = ScuAxis::new(controller, 3, "scu-motor", 1);
        assert!(probed.is_err());
        assert_eq!(axis.channel, 3);
        assert!(!axis.is_rot);
        assert_eq!(axis.hold_time, 0);
        assert_eq!(
            axis.take_status_writes(),
            [StatusWrite::Problem(true), StatusWrite::CommsError(true)]
        );
        assert!(axis.polled(&AsynUser::new(0)).is_err());
    }

    fn scu_replies(cmd: &str) -> String {
        match cmd.trim_end() {
            ":GCLF0" => ":CLF0F1000",
            ":M0" => ":M0S",
            ":GST0" => ":ST0T1",
            ":GP0" => ":P0P12.5",
            ":GPPK0" => ":PPK0K1",
            _ => ":E0E0",
        }
        .into()
    }

    /// smarActSCUMotorDriver.cpp:344-380 sets the positions and DONE before
    /// GPPK can `goto bail`, so a refused GPPK keeps them.
    #[test]
    fn a_failed_gppk_keeps_the_position_and_done_read_before_it() {
        use motor_common::testing::LineRig;
        let (_rig, io) = LineRig::failing("scuFailedGppk", scu_replies, |cmd| {
            cmd.trim_end() == ":GPPK0"
        });
        let controller = Arc::new(Mutex::new(ScuController::new(io)));
        let (mut axis, probed) = ScuAxis::new(controller, 0, "scuFailedGppk", 0);
        probed.unwrap();
        let mut status = MotorStatus {
            done: false,
            ..MotorStatus::default()
        };
        assert!(axis.poll(&AsynUser::new(0), &mut status).is_err());
        let position = 12.5 * STEPS_PER_EGU;
        assert_eq!(
            (status.position(), status.encoder_position()),
            (position, position)
        );
        assert!(status.done && !status.moving);
    }

    /// smarActSCUMotorDriver.cpp:171-181: only the constructor sets
    /// HAS_ENCODER and GAIN_SUPPORT, from its position probe.
    #[test]
    fn the_constructor_sets_has_encoder_and_gain_support() {
        use motor_common::testing::LineRig;
        let (_rig, io) = LineRig::new("scuCtorEncoder", scu_replies);
        let controller = Arc::new(Mutex::new(ScuController::new(io)));
        let (mut axis, probed) = ScuAxis::new(controller, 0, "scuCtorEncoder", 0);
        probed.unwrap();
        assert_eq!(
            axis.take_status_writes(),
            [
                StatusWrite::HasEncoder(true),
                StatusWrite::GainSupport(true)
            ]
        );
    }

    /// A failed move, home or stop raises Problem and CommsError
    /// (smarActSCUMotorDriver.cpp:446-450, 467-471, 485-489).
    #[test]
    fn a_failed_command_raises_comms_error() {
        let (handle, _) = SyncIOHandle::connect_port(
            "scu-refusing-port",
            0,
            std::time::Duration::from_millis(10),
            InterfaceType::Octet,
        );
        let controller = Arc::new(Mutex::new(ScuController::new(handle)));
        let (mut axis, _) = ScuAxis::new(controller, 0, "scu-refusing", 0);
        let user = AsynUser::new(0);
        let raised = vec![StatusWrite::Problem(true), StatusWrite::CommsError(true)];
        assert_eq!(axis.take_status_writes(), raised, "the constructor's bail");
        assert!(axis.move_absolute(&user, 1.0, 0.0, 1.0, 0.0).is_err());
        assert_eq!(axis.take_status_writes(), raised);
        assert!(axis.home(&user, 0.0, 1.0, 0.0, true).is_err());
        assert_eq!(axis.take_status_writes(), raised);
        assert!(axis.stop(&user, 0.0).is_err());
        assert_eq!(axis.take_status_writes(), raised);
    }

    /// C `epicsSnprintf(":MPA%uP%.3fH%d:GP%u")` spells a NaN `nan`.
    #[test]
    fn a_non_finite_move_is_written_as_c_prints_it() {
        use motor_common::testing::LineRig;
        let (rig, io) = LineRig::new("scuNonFinite", |_| String::new());
        let controller = Arc::new(Mutex::new(ScuController::new(io)));
        let (mut axis, _) = ScuAxis::new(controller, 0, "scuNonFinite", 0);
        rig.sent();
        let _ = axis.do_move(f64::NAN, false);
        let sent = rig.sent();
        assert!(
            sent.iter().any(|c| c.trim_end() == ":MPA0PnanH0:GP0"),
            "{sent:?}"
        );
    }

    #[test]
    fn parses_integer_and_double_replies() {
        // :<CMD><axis><PARAM><value>
        assert_eq!(parse_int_reply(":CLF0F1000"), Some((false, 1000)));
        assert_eq!(parse_double_reply(":P0P-12.500"), Some((false, -12.5)));
        // Error replies begin with 'E'.
        assert_eq!(parse_int_reply(":E0F5"), Some((true, 5)));
        // `%i` reads `0X0` as a hex axis, leaving no parameter letters.
        assert_eq!(parse_int_reply(":E0X0"), None);
    }

    #[test]
    fn parses_char_and_angle_replies() {
        let (is_err, c) = parse_char_reply(":M0H").unwrap();
        assert!(!is_err);
        assert_eq!(c, 'H');
        assert_eq!(parse_moving_status(c), MovingStatus::Holding);
        let (angle, rev) = parse_angle(":A0A123.456R2").unwrap();
        assert_eq!(angle, 123.456);
        assert_eq!(rev, 2);
    }

    /// smarActSCUMotorDriver.cpp:232, :261, :288 and :315 scan replies with
    /// `":%10[A-Z]%i%10[A-Z]%i"`, `":%10[A-Z]%i%10[A-Z]%lf"`,
    /// `":%10[A-Z]%i%c"` and `":A%iA%lfR%i"`.
    #[test]
    fn replies_are_scanned_as_c_sscanf() {
        // C wraps the last of these into its `int`.
        const INTS: &[(&str, Option<i32>)] = &[
            (":CLF0F0x10", Some(16)),
            (":CLF0F010", Some(8)),
            (":CLF0F", None),
            (":CLF0-5", None),
            (":CLF0ABCDEFGHIJK5", None),
            (":CLF0F3000000000", None),
        ];
        const DOUBLES: &[(&str, Option<f64>)] = &[
            (":P0P-12.500", Some(-12.5)),
            (":P0Pjunk", None),
            (":P0P", None),
        ];
        const CHARS: &[(&str, Option<char>)] = &[(":M0H", Some('H')), (":M0 H", Some(' '))];
        const ANGLES: &[(&str, Option<(f64, i64)>)] = &[
            (":A0A123.456R2", Some((123.456, 2))),
            (":A0A1R0x10", Some((1.0, 16))),
            (":AxA1R2", None),
            (":A0AR2", None),
            (":A0A1R", None),
            (":A0A1R3000000000", None),
        ];
        let (_rig, handle) = motor_common::testing::LineRig::new("scuSscanf", |cmd| {
            let cmd = cmd.trim_end();
            let (kind, n) = match cmd.strip_prefix(":GA") {
                Some(n) => ("GA", n),
                None => cmd.split_at(2),
            };
            let n: usize = n.parse().unwrap();
            match kind {
                ":I" => INTS[n].0,
                ":D" => DOUBLES[n].0,
                ":C" => CHARS[n].0,
                _ => ANGLES[n].0,
            }
            .to_string()
        });
        let ctrl = ScuController::new(handle);
        let mut io = AxisIo {
            port: "scuSscanf".into(),
            addr: 0,
            from_controller: String::new(),
        };
        let mut wrong = Vec::new();
        for (n, &(reply, want)) in INTS.iter().enumerate() {
            let got = ctrl.query_int(&mut io, "I", n as i32).ok();
            if got != want {
                wrong.push(format!("{reply:?} -> {got:?}"));
            }
        }
        for (n, &(reply, want)) in DOUBLES.iter().enumerate() {
            let got = ctrl.query_double(&mut io, "D", n as i32).ok();
            if got != want {
                wrong.push(format!("{reply:?} -> {got:?}"));
            }
        }
        for (n, &(reply, want)) in CHARS.iter().enumerate() {
            let got = ctrl.query_char(&mut io, "C", n as i32).ok();
            if got != want {
                wrong.push(format!("{reply:?} -> {got:?}"));
            }
        }
        for (n, &(reply, want)) in ANGLES.iter().enumerate() {
            let got = ctrl.query_angle(&mut io, n as i32).ok();
            if got != want {
                wrong.push(format!("{reply:?} -> {got:?}"));
            }
        }
        assert!(wrong.is_empty(), "{wrong:#?}");
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
