//! SmarAct MCS (RS-232) controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorSmarAct/smarActApp/src/smarActMCSMotorDriver.cpp` (a
//! model-1-style `asynMotorController`/`asynMotorAxis` pair, configured with a
//! separate controller + per-axis create command). Communication is `\n`
//! terminated by the port EOS, set both ways at connect as C does.
//!
//! ## Protocol
//!
//! Commands are colon-prefixed, channel-addressed, comma-separated, and every
//! command reads back a reply. Query replies are `:<CMD><axis>,<value>`
//! (e.g. `GP`, `GS`, `GST`, `GPPK`, `GCLS`) or `:<CMD><axis>,<angle>,<rev>` for
//! the rotary angle (`GA`). A reply whose command letters begin with `E` is a
//! status code: `E…,0` is a synchronous acknowledgement (success), `E…,<n>`
//! with `n != 0` is an error. Move/home/stop/set-position/set-speed commands
//! reply with such an acknowledgement.
//!
//! ## Linear vs rotary stages
//!
//! Rotation is detected by querying linear position (`GP`): if it errors the
//! stage is rotary. Rotary stages use `GA`/`MAA`/`MAR` (angle + revolution);
//! linear stages use `GP`/`MPA`/`MPR`. `poll` folds `rev * UDEG_PER_REV + angle`
//! into a single position.
//!
//! ## Units
//!
//! Unlike the SCU, the MCS driver reports the controller's raw integer position
//! directly (nanometres linear / micro-degrees rotary) with no scaling, so the
//! driver boundary is the controller-native unit and `MRES` = 1 (set
//! `MRES` = 1e-3 to read microns / millidegrees). Move targets are rounded to
//! whole controller units.
//!
//! ## Channel parameters
//!
//! `PTYP`, `MCLF` and `CAL` send the positioner type (`SST`), max clock
//! frequency (`SCLF`) and calibration (`CS`); `HOLD` and `AUTO_ZERO` are the
//! hold time and auto-zero flag moves and homes send, and `poll` reads the
//! sensor type into `PTYP_RB`. `set_closed_loop`/`set_pid_gain` are no-ops
//! (as in C).

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PortSets, StatusWrite,
};
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::{DrvUserRequest, InterfaceMask, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::motor::controller_port::{
    AxisUser, FinishedWrite, ParamWrite, WriteHook, WriteOutcome, WriteValue, has_motor_axis,
};
use epics_rs::motor::trace::MotorTrace;

use motor_common::connect::{connect_controller, set_eos_ignored};
use motor_common::util::atoi;
use motor_common::{ControllerParams, controller_status};

/// Response buffer size.
const READ_BUF: usize = 64;

/// Port EOS set at connect (smarActMCSMotorDriver.cpp:202-203).
pub const INPUT_EOS: &[u8] = b"\n";
pub const OUTPUT_EOS: &[u8] = b"\n";

/// C's constructor link setup: connect, printing a missing `io_port` on
/// `port_name`'s own user and epicsPrintf (`THROW_` without
/// ASYN_CANDO_EXCEPTIONS), then slurp up to 2.0 s of telnet negotiation
/// before the EOS is set, warning if any arrived, then set both EOS. C
/// carries on with the user either way.
pub fn connect_link(port_name: &str, io_port: &str, timeout: Duration) -> SyncIOHandle {
    let (link, connected) = connect_controller(
        io_port,
        timeout,
        port_name,
        format_args!("SmarActMCSController:SmarActMCSController: cannot connect to MCS controller"),
    );
    if connected.is_err() {
        errlog_vprintf(format_args!(
            "SmarActMCSController: unable to connect serial channel\n"
        ));
    }
    let (junk, _) =
        SyncIOHandle::connect_port(io_port, 0, Duration::from_secs(2), InterfaceType::Octet);
    // C counts what the read delivered whatever its status.
    let got_junk = junk.read_octet(0, 100).nbytes_transferred();
    if got_junk > 0 {
        errlog_vprintf(format_args!(
            "SmarActMCSController({port_name}): WARNING - detected unexpected characters on \
             link ({io_port}); make sure you have a RAW (not TELNET) connection\n"
        ));
    }
    set_eos_ignored(&link, INPUT_EOS, OUTPUT_EOS);
    link
}

/// Micro-degrees per revolution (C `UDEG_PER_REV`).
const UDEG_PER_REV: i64 = 360_000_000;

/// Very-far linear velocity-move target (nm, C `FAR_AWAY_LIN`).
const FAR_AWAY_LIN: i64 = 1_000_000_000;

/// Very-far rotary velocity-move target (revolutions, C `FAR_AWAY_ROT`).
const FAR_AWAY_ROT: i64 = 32767;

/// The parameters C creates, in its order (smarActMCSMotorDriver.cpp:180-185).
const CHANNEL_PARAMS: [&str; 6] = ["PTYP", "PTYP_RB", "AUTO_ZERO", "HOLD", "MCLF", "CAL"];

/// The reasons of [`CHANNEL_PARAMS`].
#[derive(Clone, Copy, Default)]
pub struct McsReasons {
    ptyp: usize,
    ptyp_rb: usize,
    auto_zero: usize,
    hold: usize,
    sclf: usize,
    cal: usize,
}

impl McsReasons {
    fn from_slice(r: [usize; 6]) -> Self {
        let [ptyp, ptyp_rb, auto_zero, hold, sclf, cal] = r;
        Self {
            ptyp,
            ptyp_rb,
            auto_zero,
            hold,
            sclf,
            cal,
        }
    }

    /// The reasons controller port `port` created.
    pub fn lookup(port: &PortHandle) -> AsynResult<Self> {
        let mut r = [0; 6];
        for (slot, name) in r.iter_mut().zip(CHANNEL_PARAMS) {
            *slot = port
                .drv_user_create_blocking(&DrvUserRequest::new(name, 0))?
                .reason;
        }
        Ok(Self::from_slice(r))
    }
}

// SmarActMCSStatus codes returned by `GS` that mean the stage is in motion
// (Stepping, Scanning, Targeting, MoveDelay, Calibrating, FindRefMark). Stopped
// (0), Holding (3) and Locked (9) mean not moving.
fn status_is_moving(code: i32) -> bool {
    matches!(code, 1 | 2 | 4 | 5 | 6 | 7)
}

fn mcs_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Parse a `:<CMD><axis>,<value>` reply into (is-error-command, last value).
/// The MCS error convention is applied by callers: an `E` command with a
/// non-zero value is an error, `E…,0` is an acknowledgement.
fn parse_reply(reply: &str) -> Option<(bool, i32)> {
    let s = reply.trim().strip_prefix(':')?;
    let cmd_end = s.find(|c: char| !c.is_ascii_uppercase())?;
    if cmd_end == 0 {
        return None;
    }
    let is_err = s.as_bytes()[0] == b'E';
    let rest = &s[cmd_end..]; // "<axis>,<value>[,...]"
    let comma = rest.find(',')?;
    let tail = &rest[comma + 1..];
    let val_end = tail.find(',').unwrap_or(tail.len());
    Some((is_err, atoi(tail[..val_end].trim())))
}

/// Parse a `:<CMD><axis>,<angle>,<rev>` rotary reply into (is-error-command,
/// angle, revolutions). Returns `None` unless all three numbers are present
/// (an error reply carries fewer fields and fails to parse, matching C).
fn parse_angle(reply: &str) -> Option<(bool, i32, i32)> {
    let s = reply.trim().strip_prefix(':')?;
    let cmd_end = s.find(|c: char| !c.is_ascii_uppercase())?;
    if cmd_end == 0 {
        return None;
    }
    let is_err = s.as_bytes()[0] == b'E';
    let rest = &s[cmd_end..]; // "<axis>,<angle>,<rev>"
    let parts: Vec<&str> = rest.splitn(3, ',').collect();
    if parts.len() != 3 {
        return None;
    }
    Some((is_err, atoi(parts[1].trim()), atoi(parts[2].trim())))
}

/// What C keeps per axis that the controller's writes reach as well.
#[derive(Clone, Copy)]
struct McsChannel {
    /// Wire channel (from the create-axis command; distinct from the axis no.).
    channel: i32,
    /// C `isRot_`, which `checkType` sets.
    is_rot: bool,
    /// The axis's `HOLD` and `AUTO_ZERO`, which C's move and home read
    /// back from the parameter library.
    hold_time: i32,
    auto_zero: i32,
}

/// Shared MCS controller endpoint owning the asyn octet handle.
pub struct McsController {
    handle: SyncIOHandle,
    /// When set, speed-set commands (`SCLS`) are suppressed (C `disableSpeed`).
    disable_speed: bool,
    /// C `pAxes_`, by axis number.
    axes: BTreeMap<i32, McsChannel>,
    /// The parameters, once the port created them.
    reasons: Option<McsReasons>,
}

impl McsController {
    /// Wrap a connected octet handle. The C controller constructor connects,
    /// slurps stray telnet negotiation bytes and starts the poller; the telnet
    /// slurp is a workaround for TELNET-mode terminal servers and is omitted
    /// here — use a RAW (or serial) connection.
    pub fn new(handle: SyncIOHandle, disable_speed: bool) -> Self {
        Self {
            handle,
            disable_speed,
            axes: BTreeMap::new(),
            reasons: None,
        }
    }

    /// Let the axes set the parameters, `reasons`.
    pub fn set_reasons(&mut self, reasons: McsReasons) {
        self.reasons = Some(reasons);
    }

    /// Axis `axis`'s channel state.
    fn axis(&self, axis: i32) -> &McsChannel {
        &self.axes[&axis]
    }

    /// C `SmarActMCSAxis::checkType`: a stage whose linear position does
    /// not read back is rotary.
    fn check_type(&mut self, axis: i32) {
        let is_rot = self.get_val("GP", self.axis(axis).channel).is_err();
        if let Some(c) = self.axes.get_mut(&axis) {
            c.is_rot = is_rot;
        }
    }

    /// Send a command and read back its reply (every MCS command replies).
    fn transact(&self, cmd: &str) -> AsynResult<String> {
        let (raw, status) = self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts();
        status.map_err(controller_status)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string())
    }

    /// Read an integer parameter (`:<parm><ch>`).
    fn get_val(&self, parm: &str, channel: i32) -> AsynResult<i32> {
        let reply = self.transact(&format!(":{parm}{channel}"))?;
        let (is_err, val) =
            parse_reply(&reply).ok_or_else(|| mcs_err(format!("MCS: bad reply {reply:?}")))?;
        if is_err && val != 0 {
            return Err(mcs_err(format!("MCS: controller error {val} ({reply:?})")));
        }
        Ok(val)
    }

    /// Read the rotary angle (`:GA<ch>` → angle, revolutions).
    fn get_angle(&self, channel: i32) -> AsynResult<(i32, i32)> {
        let reply = self.transact(&format!(":GA{channel}"))?;
        let (is_err, angle, rev) =
            parse_angle(&reply).ok_or_else(|| mcs_err(format!("MCS: bad angle {reply:?}")))?;
        if is_err && angle != 0 {
            return Err(mcs_err(format!(
                "MCS: controller error {angle} ({reply:?})"
            )));
        }
        Ok((angle, rev))
    }

    /// Issue a command whose reply is only an acknowledgement (move/home/stop/
    /// set-position/set-speed); succeed on a non-error reply or `E…,0`. A
    /// refused reply is C `parseReply`'s bare asynError
    /// (smarActMCSMotorDriver.cpp:293-310).
    fn command(&self, cmd: &str) -> AsynResult<()> {
        let reply = self.transact(cmd)?;
        match parse_reply(&reply) {
            Some((is_err, val)) if !is_err || val == 0 => Ok(()),
            _ => Err(mcs_err("")),
        }
    }
}

/// The MCS controller port's parameters and its `writeInt32`.
#[derive(Default)]
pub struct McsParams {
    /// Set once the controller is created, after the port as in C.
    controller: Arc<OnceLock<Arc<Mutex<McsController>>>>,
    reasons: McsReasons,
}

impl McsParams {
    /// The slot the create command fills with the controller.
    pub fn controller_slot(&self) -> Arc<OnceLock<Arc<Mutex<McsController>>>> {
        self.controller.clone()
    }
}

/// C `writeInt32` traces each write at its end, the base's included; a
/// failed `SST`, `CS` or `SCLF` returns before it
/// (smarActMCSMotorDriver.cpp:293-327).
struct McsWriteHook {
    sent: [usize; 3],
}

impl WriteHook for McsWriteHook {
    fn after_write(&self, write: &FinishedWrite<'_>) -> WriteOutcome {
        let WriteValue::Int32(value) = write.value else {
            return WriteOutcome::default();
        };
        let function = write.function;
        match write.status {
            Err(_) if self.sent.contains(&function) => {}
            Err(e) => write.print(
                TraceMask::ERROR,
                format_args!(
                    "SmarActMCSController:writeInt32: error, status={} function={function}, value={value}\n",
                    e.status() as i32
                ),
            ),
            Ok(()) => write.print(
                TraceMask::IO_DRIVER,
                format_args!("SmarActMCSController:writeInt32: function={function}, value={value}\n"),
            ),
        }
        WriteOutcome::default()
    }
}

impl ControllerParams for McsParams {
    const INTERFACE_MASK: InterfaceMask = InterfaceMask::NONE;
    const INTERRUPT_MASK: InterfaceMask = InterfaceMask::NONE;

    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        let mut r = [0; 6];
        for (slot, name) in r.iter_mut().zip(CHANNEL_PARAMS) {
            *slot = base.create_param(name, ParamType::Int32)?;
        }
        self.reasons = McsReasons::from_slice(r);
        Ok(())
    }

    fn write_hook(&self) -> Option<Arc<dyn WriteHook>> {
        let r = self.reasons;
        Some(Arc::new(McsWriteHook {
            sent: [r.ptyp, r.cal, r.sclf],
        }))
    }

    /// C `SmarActMCSController::writeInt32` (smarActMCSMotorDriver.cpp:
    /// 274-329). C posts at the axis's channel what it set at its axis
    /// number, a bug where the two differ; both are the axis number here.
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        value: i32,
    ) -> ParamWrite {
        let function = user.reason;
        // C's getAxis, which prints an out-of-range address.
        if !has_motor_axis(base, user) {
            MotorTrace::new(&*base.port_name, user.asyn().addr).print(
                TraceMask::ERROR,
                function as i32,
                format_args!(
                    "SmarActMCSController:writeInt32: error, function: {function}. Invalid axis number.\n"
                ),
            );
            return ParamWrite::Handled(Err(mcs_err("MCS: invalid axis number")));
        }
        let axis = user.addr;
        let Some(ctrl) = self.controller.get() else {
            return ParamWrite::PassToBase(Ok(()));
        };
        let mut ctrl = ctrl.lock().unwrap_or_else(|e| e.into_inner());
        let r = self.reasons;
        let ch = ctrl.axis(axis).channel;
        let cmd = match function {
            f if f == r.ptyp => format!(":SST{ch},{value}"),
            f if f == r.cal => format!(":CS{ch}"),
            f if f == r.sclf => format!(":SCLF{ch},{value}"),
            f => {
                if let Some(c) = ctrl.axes.get_mut(&axis) {
                    if f == r.hold {
                        c.hold_time = value;
                    } else if f == r.auto_zero {
                        c.auto_zero = value;
                    }
                }
                return ParamWrite::PassToBase(Ok(()));
            }
        };
        let _ = base.set_int32_param(function, axis, value);
        let sent = ctrl.command(&cmd);
        if sent.is_err() {
            return ParamWrite::Handled(sent);
        }
        if function == r.ptyp {
            ctrl.check_type(axis);
        }
        drop(ctrl);
        let _ = base.call_param_callbacks(axis);
        ParamWrite::Handled(Ok(()))
    }
}

/// One MCS channel sharing a controller. Implements [`AsynMotor`].
pub struct McsAxis {
    controller: Arc<Mutex<McsController>>,
    /// The axis number, the key of its channel state on the controller.
    axis_no: i32,
    /// Whether speed-set commands are suppressed (copied from the controller).
    disable_speed: bool,
    /// Cached closed-loop speed (`GCLS`), to skip redundant `SCLS` writes.
    vel_cached: i32,
    /// What the constructor or the last poll set, for the poller's
    /// callParamCallbacks.
    sets: PortSets,
    /// The status bits the constructor or the last failed command set.
    status_writes: Vec<StatusWrite>,
}

impl McsAxis {
    /// Construct axis `axis_no` on channel `channel`, probing speed,
    /// status, positioner type and rotation, matching the C
    /// `SmarActMCSAxis` constructor. Blocking I/O.
    ///
    /// Like C's `goto bail`, a failed probe keeps the axis with the values
    /// probed so far; the failure is returned alongside it.
    pub fn new(
        controller: Arc<Mutex<McsController>>,
        axis_no: i32,
        channel: i32,
    ) -> (Self, AsynResult<()>) {
        let disable_speed = {
            let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // C leaves both unset, and its move reads garbage, when the
            // constructor bails before setting them.
            ctrl.axes.insert(
                axis_no,
                McsChannel {
                    channel,
                    is_rot: false,
                    hold_time: 0,
                    auto_zero: 1,
                },
            );
            ctrl.disable_speed
        };
        let mut axis = Self {
            controller,
            axis_no,
            disable_speed,
            vel_cached: 0,
            sets: PortSets::default(),
            status_writes: Vec::new(),
        };
        let probed = axis.probe();
        let probed = axis.bail(probed);
        (axis, probed)
    }

    fn probe(&mut self) -> AsynResult<()> {
        let axis_no = self.axis_no;
        let mut ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        let channel = ctrl.axis(axis_no).channel;

        if !self.disable_speed {
            self.vel_cached = ctrl.get_val("GCLS", channel)?;
        }

        // Comms check.
        ctrl.get_val("GS", channel)?;

        // smarActMCSMotorDriver.cpp:367-368
        if let Some(r) = ctrl.reasons {
            self.sets.axis(r.auto_zero, ParamValue::Int32(1));
            self.sets.axis(r.hold, ParamValue::Int32(0));
        }

        ctrl.check_type(axis_no);

        // Sensor type query (also a comms check).
        ctrl.get_val("GST", channel)?;

        // C reads the position again rather than reusing checkType's `GP`.
        let has_encoder = if ctrl.axis(axis_no).is_rot {
            ctrl.get_angle(channel).is_ok()
        } else {
            ctrl.get_val("GP", channel).is_ok()
        };
        if has_encoder {
            self.status_writes.extend([
                StatusWrite::HasEncoder(true),
                StatusWrite::GainSupport(true),
            ]);
        }
        Ok(())
    }

    /// C's `bail` label after a command: a failure raises
    /// `motorStatusProblem_` and `motorStatusCommsError_`
    /// (smarActMCSMotorDriver.cpp:385-387, 599-604, 624-628, 638-642,
    /// 665-669, 713-717).
    fn bail(&mut self, result: AsynResult<()>) -> AsynResult<()> {
        if result.is_err() {
            self.status_writes
                .extend([StatusWrite::Problem(true), StatusWrite::CommsError(true)]);
        }
        result
    }

    /// C `moveVelocity`'s relative move to a very-far target.
    fn jog(&mut self, velocity: f64) -> AsynResult<()> {
        self.set_speed(velocity)?;
        let dir: i64 = if velocity < 0.0 { -1 } else { 1 };
        let ctrl = self.lock();
        let McsChannel {
            channel: ch,
            is_rot,
            ..
        } = *ctrl.axis(self.axis_no);
        let cmd = if is_rot {
            format!(":MAR{ch},0,{},0", FAR_AWAY_ROT * dir)
        } else {
            format!(":MPR{ch},{},0", FAR_AWAY_LIN * dir)
        };
        ctrl.command(&cmd)
    }

    fn lock(&self) -> MutexGuard<'_, McsController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Change the closed-loop speed if it differs from the cached value
    /// (C `setSpeed`); suppressed when `disable_speed` is set.
    fn set_speed(&mut self, velocity: f64) -> AsynResult<()> {
        if self.disable_speed {
            return Ok(());
        }
        let vel = velocity.abs().round() as i32;
        if vel != self.vel_cached {
            let ctrl = self.lock();
            let ch = ctrl.axis(self.axis_no).channel;
            ctrl.command(&format!(":SCLS{ch},{vel}"))?;
            drop(ctrl);
            self.vel_cached = vel;
        }
        Ok(())
    }

    /// Shared move: set speed, round to controller units, and issue the linear
    /// or rotary absolute/relative move with the axis's hold time.
    fn do_move(&mut self, position: f64, relative: bool, velocity: f64) -> AsynResult<()> {
        self.set_speed(velocity)?;
        let ctrl = self.lock();
        let McsChannel {
            channel: ch,
            is_rot,
            hold_time,
            ..
        } = *ctrl.axis(self.axis_no);
        let rpos = position.round() as i64;

        let cmd = if is_rot {
            let mut angle = (rpos as i32) % (UDEG_PER_REV as i32);
            let mut rev = (rpos / UDEG_PER_REV) as i32;
            if angle < 0 {
                angle += UDEG_PER_REV as i32;
                rev -= 1;
            }
            let mnem = if relative { "MAR" } else { "MAA" };
            format!(":{mnem}{ch},{angle},{rev},{hold_time}")
        } else {
            let mnem = if relative { "MPR" } else { "MPA" };
            format!(":{mnem}{ch},{rpos},{hold_time}")
        };
        ctrl.command(&cmd)
    }
}

impl AsynMotor for McsAxis {
    /// The C constructor calls `callParamCallbacks` past `bail:`, before
    /// any throw (smarActMCSMotorDriver.cpp:389).
    fn constructor_calls_param_callbacks(&self) -> bool {
        true
    }

    /// C `MCSAxis::poll`'s `bail` label sets `motorStatusProblem_` and
    /// `motorStatusCommsError_`.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM_AND_COMMS_ERROR
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let moved = self.do_move(position, false, velocity);
        self.bail(moved)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let moved = self.do_move(distance, true, velocity);
        self.bail(moved)
    }

    fn move_velocity(
        &mut self,
        user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // Speed 0 would mean "unlimited" on the MCS. C only sets the
        // motorStop_ parameter there, which stops nothing
        // (smarActMCSMotorDriver.cpp:684-694); this stops.
        let speed = velocity.abs().round() as i32;
        if speed == 0 {
            return self.stop(user, acceleration);
        }
        let moved = self.jog(velocity);
        self.bail(moved)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let homed = self.set_speed(velocity).and_then(|()| {
            let ctrl = self.lock();
            let c = ctrl.axis(self.axis_no);
            let dir = if forward { 0 } else { 1 };
            ctrl.command(&format!(
                ":FRM{},{dir},{},{}",
                c.channel, c.hold_time, c.auto_zero
            ))
        });
        self.bail(homed)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let stopped = ctrl.command(&format!(":S{}", ctrl.axis(self.axis_no).channel));
        drop(ctrl);
        self.bail(stopped)
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let McsChannel {
            channel: ch,
            is_rot,
            ..
        } = *ctrl.axis(self.axis_no);
        let rpos = position.round() as i32;
        // Rotary set-position is only valid within one revolution (C).
        let set = if is_rot && !(0..UDEG_PER_REV as i32).contains(&rpos) {
            Err(mcs_err(format!(
                "MCS: rotary set-position {rpos} out of [0, {UDEG_PER_REV})"
            )))
        } else {
            ctrl.command(&format!(":SP{ch},{rpos}"))
        };
        drop(ctrl);
        self.bail(set)
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, _enable: bool) -> AsynResult<()> {
        // Not implemented in the C MCS driver.
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // Not implemented in the C MCS driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        let McsChannel {
            channel: ch,
            is_rot,
            ..
        } = *ctrl.axis(self.axis_no);

        let pos = if is_rot {
            let (angle, rev) = ctrl.get_angle(ch)?;
            rev as i64 * UDEG_PER_REV + angle as i64
        } else {
            ctrl.get_val("GP", ch)? as i64
        };
        status.set_encoder_position(pos as f64);
        status.set_position(pos as f64);

        let moving = status_is_moving(ctrl.get_val("GS", ch)?);
        status.moving = moving;
        status.done = !moving;
        status.homed = ctrl.get_val("GPPK", ch)? != 0;
        // C reads the sensor type, `GST`, into PTYP_RB, then reads it again
        // (smarActMCSMotorDriver.cpp:495-502).
        let sensor = ctrl.get_val("GST", ch)?;
        if let Some(r) = ctrl.reasons {
            self.sets.axis(r.ptyp_rb, ParamValue::Int32(sensor));
        }
        ctrl.get_val("GST", ch)?;
        drop(ctrl);

        status.problem = false;
        status.comms_error = false;
        Ok(())
    }

    fn take_port_sets(&mut self) -> PortSets {
        std::mem::take(&mut self.sets)
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// smarActMCSMotorDriver.cpp:385-389: past `bail:`, so a failed probe
    /// calls back too.
    #[test]
    fn the_constructor_calls_param_callbacks() {
        let (_rig, io) =
            motor_common::testing::LineRig::failing("mcsCtorCallback", |_| String::new(), |_| true);
        let controller = Arc::new(Mutex::new(McsController::new(io, false)));
        let (axis, probed) = McsAxis::new(controller, 0, 0);
        assert!(probed.is_err());
        assert!(axis.constructor_calls_param_callbacks());
    }

    #[test]
    fn parses_value_replies_and_errors() {
        // Normal value reply.
        let (is_err, val) = parse_reply(":P0,12345").unwrap();
        assert!(!is_err);
        assert_eq!(val, 12345);
        // Negative position.
        let (_, val) = parse_reply(":P1,-500").unwrap();
        assert_eq!(val, -500);
        // Acknowledgement E…,0 is not an error.
        let (is_err, val) = parse_reply(":E0,0").unwrap();
        assert!(is_err);
        assert_eq!(val, 0);
        // E…,<n!=0> is an error code.
        let (is_err, val) = parse_reply(":E0,7").unwrap();
        assert!(is_err);
        assert_eq!(val, 7);
    }

    #[test]
    fn parses_angle_and_rejects_short_replies() {
        let (is_err, angle, rev) = parse_angle(":A0,123,2").unwrap();
        assert!(!is_err);
        assert_eq!(angle, 123);
        assert_eq!(rev, 2);
        // An error reply has too few fields to be an angle.
        assert!(parse_angle(":E0,5").is_none());
    }

    #[test]
    fn moving_status_codes() {
        for code in [1, 2, 4, 5, 6, 7] {
            assert!(status_is_moving(code), "code {code} should be moving");
        }
        for code in [0, 3, 9] {
            assert!(!status_is_moving(code), "code {code} should be idle");
        }
    }

    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use motor_common::testing::Polled;

    /// A link that holds `pending` bytes and logs each read and EOS set.
    struct Link {
        base: PortDriverBase,
        pending: Vec<u8>,
        log: Arc<Mutex<Vec<&'static str>>>,
    }

    impl PortDriver for Link {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            self.log.lock().unwrap().push("read");
            if self.pending.is_empty() {
                return epics_rs::asyn::interpose::OctetReadResult::failed(AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "timeout".into(),
                });
            }
            let n = self.pending.len().min(buf.len());
            buf[..n].copy_from_slice(&self.pending[..n]);
            self.pending.drain(..n);
            epics_rs::asyn::interpose::OctetReadResult::ok(
                n,
                epics_rs::asyn::interpose::EomReason::empty(),
            )
        }
        fn set_input_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            self.log.lock().unwrap().push("input eos");
            Ok(())
        }
        fn set_output_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            self.log.lock().unwrap().push("output eos");
            Ok(())
        }
    }

    fn publish(name: &str, pending: &[u8]) -> Arc<Mutex<Vec<&'static str>>> {
        let log = Arc::new(Mutex::new(Vec::new()));
        let driver = Link {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
            pending: pending.to_vec(),
            log: log.clone(),
        };
        let claim = claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        std::mem::forget(rt);
        log
    }

    /// One read, then both EOS sets (their order is not C's concern here).
    fn assert_drained_first(log: &[&str]) {
        assert_eq!(log.len(), 3, "{log:?}");
        assert_eq!(log[0], "read", "{log:?}");
    }

    #[test]
    fn the_link_is_drained_before_the_eos_is_set() {
        let log = publish("mcsTelnetLink", b"\xff\xfb\x01");
        connect_link("MCS1", "mcsTelnetLink", Duration::from_secs(1));
        assert_drained_first(&log.lock().unwrap());
    }

    #[test]
    fn a_quiet_link_still_connects() {
        let log = publish("mcsRawLink", b"");
        connect_link("MCS1", "mcsRawLink", Duration::from_secs(1));
        assert_drained_first(&log.lock().unwrap());
    }

    /// smarActMCSMotorDriver.cpp:367-368 and :495-498: the constructor's
    /// AUTO_ZERO and HOLD, then the poll's PTYP_RB, on the axis's own
    /// address in the order C sets them.
    #[test]
    fn the_axis_s_params_go_out_on_the_axis_in_c_s_order() {
        use epics_rs::asyn::interfaces::motor::PortSet;
        let (_rig, io) = motor_common::testing::LineRig::new("mcsAxisSets", |cmd| {
            match cmd.trim_end() {
                ":GCLS0" => ":CLS0,0\n",
                ":GS0" => ":S0,0\n",
                ":GP0" => ":P0,0\n",
                ":GST0" => ":ST0,300\n",
                ":GPPK0" => ":PPK0,1\n",
                _ => ":E0,0\n",
            }
            .into()
        });
        let controller = Arc::new(Mutex::new(McsController::new(io, false)));
        controller
            .lock()
            .unwrap()
            .set_reasons(McsReasons::from_slice([10, 11, 12, 13, 14, 15]));
        let (mut axis, probed) = McsAxis::new(controller, 1, 0);
        probed.unwrap();
        let sets = |axis: &mut McsAxis| -> Vec<(usize, i32)> {
            axis.take_port_sets()
                .into_iter()
                .map(|set| match set {
                    PortSet::Axis {
                        reason,
                        value: ParamValue::Int32(v),
                    } => (reason, v),
                    other => panic!("{other:?}"),
                })
                .collect()
        };
        assert_eq!(sets(&mut axis), [(12, 1), (13, 0)]);
        axis.polled(&AsynUser::new(0)).unwrap();
        assert_eq!(sets(&mut axis), [(11, 300)]);
    }

    fn mcs_replies(cmd: &str) -> String {
        match cmd.trim_end() {
            ":GCLS0" => ":CLS0,0\n",
            ":GS0" => ":S0,0\n",
            ":GP0" => ":P0,1234\n",
            ":GST0" => ":ST0,300\n",
            ":GPPK0" => ":PPK0,1\n",
            _ => ":E0,0\n",
        }
        .into()
    }

    /// smarActMCSMotorDriver.cpp:461-485 sets the positions and DONE before
    /// GPPK can `goto bail`, so a refused GPPK keeps them.
    #[test]
    fn a_failed_gppk_keeps_the_position_and_done_read_before_it() {
        let (_rig, io) =
            motor_common::testing::LineRig::failing("mcsFailedGppk", mcs_replies, |cmd| {
                cmd.trim_end() == ":GPPK0"
            });
        let controller = Arc::new(Mutex::new(McsController::new(io, false)));
        let (mut axis, probed) = McsAxis::new(controller, 1, 0);
        probed.unwrap();
        let mut status = MotorStatus {
            done: false,
            ..MotorStatus::default()
        };
        assert!(axis.poll(&AsynUser::new(0), &mut status).is_err());
        assert_eq!(
            (status.position(), status.encoder_position()),
            (1234.0, 1234.0)
        );
        assert!(status.done && !status.moving);
    }

    /// smarActMCSMotorDriver.cpp:376-383: only the constructor sets
    /// HAS_ENCODER and GAIN_SUPPORT, from its position probe.
    #[test]
    fn the_constructor_sets_has_encoder_and_gain_support() {
        let (_rig, io) = motor_common::testing::LineRig::new("mcsCtorEncoder", mcs_replies);
        let controller = Arc::new(Mutex::new(McsController::new(io, false)));
        let (mut axis, probed) = McsAxis::new(controller, 1, 0);
        probed.unwrap();
        assert_eq!(
            axis.take_status_writes(),
            [
                StatusWrite::HasEncoder(true),
                StatusWrite::GainSupport(true)
            ]
        );
    }

    /// smarActMCSMotorDriver.cpp:187-215 and :385-393: a missing link keeps
    /// the controller and a user that refuses every transfer; the axis bails
    /// and stays, its poll failing on the same user.
    #[test]
    fn a_missing_link_keeps_the_controller_and_its_axes() {
        let link = connect_link("MCS1", "mcsMissingLink", Duration::from_millis(10));
        let e = link.write_read(0, b":GS0", 16).into_parts().1.unwrap_err();
        assert_eq!(e.message(), "asynManager::queueLockPort not connected");
        let controller = Arc::new(Mutex::new(McsController::new(link, false)));
        let (mut axis, probed) = McsAxis::new(controller.clone(), 1, 2);
        assert!(probed.is_err());
        let ctrl = controller.lock().unwrap();
        assert_eq!(ctrl.axis(1).channel, 2);
        assert!(!ctrl.axis(1).is_rot);
        drop(ctrl);
        assert_eq!(axis.vel_cached, 0);
        assert_eq!(
            axis.take_status_writes(),
            [StatusWrite::Problem(true), StatusWrite::CommsError(true)]
        );
        assert!(axis.polled(&AsynUser::new(0)).is_err());
    }
}
