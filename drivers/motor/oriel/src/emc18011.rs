//! Oriel Encoder Mike 18011 (EMC18011) controller driver (serial ASCII).
//!
//! Ported from `motorOriel/orielApp/src/drvEMC18011.cc` + `devEMC18011.cc` (the
//! model-1 dev/drv pair). One serial line drives up to three encoder-mike axes,
//! but the controller can address only **one motor at a time**: a motor must be
//! selected with `M<n>` before it will move or report status. Commands are
//! terminated by the port output EOS CR and replies by input EOS LF, both set
//! by `EMC18011Config` as C `motor_init` does. Every command produces a
//! reply, so each is written and its reply consumed to keep the stream
//! synchronized.
//!
//! ## Single-motor multiplex
//!
//! The controller selects one motor and holds it until motion completes: while
//! a motor is moving, motor selection cannot change, and only the selected motor
//! reports status/position. C's `motorSelect` and `motorLock` are one
//! [`Selection`]: a record's JOG, LOAD_POS, SET_VELOCITY or GET_INFO claims
//! a free controller in `build_trans`, every command of a motor that does
//! not hold it is deferred to the record's retries ([`Emc18011Build`]), and
//! [`poll`](AsynMotor::poll) releases it when the motor stops (or hits a
//! limit) and a valid position has been read. The `M<n>` C sends at the
//! claim goes out ahead of the poller's first command for that motor.
//!
//! ## Position feedback only when stopped
//!
//! The controller does not report reliable position during motion (noted as a
//! possible controller bug in C), so position is read (`A`) only once the
//! selected motor has stopped or reached a limit. A non-selected motor reports
//! its last known position and `done`.
//!
//! ## Units
//!
//! The controller works in millimetres (`A` returns mm, `G`/`T` take mm).
//! As C, record steps and steps/s go out times [`DRIVE_RESOLUTION`] and `A`
//! comes back as `NINT(mm / DRIVE_RESOLUTION)`, so the record's `MRES` must
//! be that value for `EGU` mm.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `Z`/`A` reads run inside
//!   [`poll`](AsynMotor::poll).
//! - Homing, PID gains, torque enable/disable and travel-limit setting are
//!   unsupported in C (`build_trans` returns `ERROR`); [`home`](AsynMotor::home)
//!   returns an error, [`Emc18011Build`] refuses the limits, and the others
//!   are no-ops.
//! - `set_position` only supports zeroing (`CA`), matching C `LOAD_POS`.

use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::device_support::{BuildTrans, LimitBuild};
use epics_rs::motor::flags::MotorCommand;

use epics_libcom_rs::cfmt::CFloat;

use motor_common::model1::MoveStrings;
use motor_common::util::{atof, nint};

/// Millimetres per record step (C `EMC18011_RESOLUTION`, drvEMC18011.h).
pub const DRIVE_RESOLUTION: f64 = 0.01;

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 120;

/// C `MOTION_DELAY` (drvEMC18011.cc:72), slept on entering RETRY.
const MOTION_DELAY: Duration = Duration::from_millis(100);

/// C's sleep in an unselected motor's `set_status` (drvEMC18011.cc:291).
const UNSELECTED_DELAY: Duration = Duration::from_millis(100);

/// Fixed number of axes (C `EMC18011_MAX_MOTORS`).
pub const MAX_MOTORS: usize = 3;

/// Reply substring confirming the controller went remote (C `RTN_REMOTE`).
const RTN_REMOTE: &str = "ON LINE";

/// `Z` motion-status reply characters (C `Z_*`).
const Z_STOPPED: char = 'a';
const Z_RUNDOWN: char = 'b'; // positive motion
const Z_RUNUP: char = 'c'; // negative motion
const Z_LSDOWN: char = 'd'; // negative limit (hard stop)
const Z_LSUP: char = 'e'; // positive limit (hard stop)

fn oriel_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Build the absolute (`G`) / relative (`T`) move command for a millimetre
/// value, matching the C 7-character field limit: one decimal place, dropping to
/// none if too long, then clamping to the controller's max travel string.
fn move_cmd(prefix: char, mm: f64) -> String {
    let mut field = format!("{:.1}", CFloat(mm));
    if field.len() > 7 {
        field = format!("{:.0}", CFloat(mm));
    }
    if field.len() > 7 {
        field = if mm < 0.0 {
            "-999999".to_string()
        } else {
            "9999999".to_string()
        };
    }
    format!("{prefix}{field}")
}

/// Build the velocity command for a millimetre/second value, matching the C
/// magnitude-dependent precision (finer at low speeds, capped at `V200`).
fn vel_cmd(mms: f64) -> String {
    if mms < 5.0 {
        format!("V{:.2}", CFloat(mms))
    } else if mms < 50.0 {
        format!("V{:.1}", CFloat(mms))
    } else if mms < 200.0 {
        format!("V{:.0}", CFloat(mms))
    } else {
        "V200".to_string()
    }
}

/// Decoded `Z` motion status.
struct ZStatus {
    done: bool,
    /// Still moving (`b`/`c`): position must not be read yet.
    moving: bool,
    plus_ls: bool,
    minus_ls: bool,
    direction: bool,
}

/// Decode a `Z` status character. Unrecognized characters are treated as
/// negative motion (C falls back to `Z_RUNUP`), i.e. "still moving".
fn decode_z(c: char) -> ZStatus {
    let known = (Z_STOPPED..=Z_LSUP).contains(&c);
    let c = if known { c } else { Z_RUNUP };
    let moving = c == Z_RUNDOWN || c == Z_RUNUP;
    // C plusdir: stopped, negative-motion, or positive-limit.
    let direction = c == Z_STOPPED || c == Z_RUNUP || c == Z_LSUP;
    ZStatus {
        done: c == Z_STOPPED,
        moving,
        plus_ls: c == Z_LSUP,
        minus_ls: c == Z_LSDOWN,
        direction,
    }
}

/// C `cntrl->motorSelect` and `motorLock`: the 0-based motor holding the
/// controller, or none. A record's `build_trans` claims it without the
/// controller lock, as C's does.
#[derive(Debug)]
pub struct Selection(AtomicI32);

impl Selection {
    const NONE: i32 = -1;

    fn new() -> Self {
        Self(AtomicI32::new(Self::NONE))
    }

    /// C `motorLock->tryWait()` then `motorSelect = signal`: whether
    /// `index` holds the controller now.
    fn claim(&self, index: usize) -> bool {
        let index = index as i32;
        match self
            .0
            .compare_exchange(Self::NONE, index, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => true,
            Err(holder) => holder == index,
        }
    }

    fn holder(&self) -> Option<usize> {
        usize::try_from(self.0.load(Ordering::Acquire)).ok()
    }

    fn holds(&self, index: usize) -> bool {
        self.holder() == Some(index)
    }
}

/// Shared controller endpoint: owns the serial handle and the single-motor
/// selection.
pub struct Emc18011Controller {
    handle: SyncIOHandle,
    ident: String,
    selection: Arc<Selection>,
    /// The holder the last `M<n>` selected; `None` until the holder's first
    /// command, and again from its release, so a new claim sends `M<n>`.
    wired: Option<usize>,
    comm_state: CommState,
}

/// Controller communication state (C `cntrl->status`): one invalid `Z` reply
/// is retried silently, a second in a row is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

impl Emc18011Controller {
    /// Connect and identify an EMC18011 (C `motor_init`): toggle local/remote
    /// and confirm the `ON LINE` reply, then stop all motion. The axis count is
    /// fixed at [`MAX_MOTORS`]. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let ctrl = Self {
            handle,
            ident: "Oriel Encoder Mike 18011".to_string(),
            selection: Arc::new(Selection::new()),
            wired: None,
            comm_state: CommState::Normal,
        };

        let mut online = false;
        for _ in 0..3 {
            ctrl.send(b"L"); // local
            let reply = ctrl.command("R").unwrap_or_default(); // remote
            if reply.contains(RTN_REMOTE) {
                online = true;
                break;
            }
        }
        if !online {
            return Err(oriel_err(
                "EMC18011: no 'ON LINE' response to remote (R) command",
            ));
        }

        ctrl.send(b"S"); // ensure all motion stopped
        Ok(ctrl)
    }

    /// The controller identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Fixed number of axes.
    pub fn num_axes(&self) -> usize {
        MAX_MOTORS
    }

    /// Send a query and return its reply (trimmed).
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.command_bytes(cmd.as_bytes())
    }

    /// A set command: written and its reply read, both statuses ignored as
    /// C ignores send_recv_mess's and motordrvCom's getmsg.
    fn send(&self, cmd: &[u8]) {
        let _ = self.command_bytes(cmd);
    }

    /// [`command`](Self::command) for a record string's raw bytes.
    fn command_bytes(&self, cmd: &[u8]) -> AsynResult<String> {
        let raw = self.handle.write_read(0, cmd, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_matches(['\r', '\n', '\0', ' ']).to_string())
    }

    /// Hold the controller for motor `index` (0-based), claiming it if
    /// free, and send the claim's `M<n>` (1-based) if not yet sent. A
    /// "busy" error if another motor holds it.
    fn select(&mut self, index: usize) -> AsynResult<()> {
        if !self.selection.claim(index) {
            let holder = self.selection.holder().map_or(0, |h| h + 1);
            return Err(oriel_err(format!(
                "EMC18011: controller busy (motor {holder} in motion)"
            )));
        }
        if self.wired != Some(index) {
            self.send(format!("M{}", index + 1).as_bytes());
            self.wired = Some(index);
        }
        Ok(())
    }

    /// C `motorSelect = -1; motorLock->signal()` (drvEMC18011.cc:274-275).
    fn release(&mut self) {
        self.selection.0.store(Selection::NONE, Ordering::Release);
        self.wired = None;
    }
}

/// devEMC18011's `build_trans` for one motor: what it defers, and its
/// refused soft limits.
pub struct Emc18011Build {
    selection: Arc<Selection>,
    index: usize,
}

impl BuildTrans for Emc18011Build {
    /// devEMC18011.cc:206-231: JOG, LOAD_POS, SET_VELOCITY (which leads
    /// every move and home, motorRecord.cc:2052, 2527) and GET_INFO claim a
    /// free controller; any command of a motor not holding it is deferred.
    fn defers(&self, command: &MotorCommand) -> bool {
        match command {
            // What the record leads a transaction with, not build_trans calls.
            MotorCommand::OutParm { .. }
            | MotorCommand::MoveStrings { .. }
            | MotorCommand::MoveDirection { .. }
            | MotorCommand::RecordFields(_) => return false,
            MotorCommand::MoveVelocity { .. }
            | MotorCommand::SetPosition { .. }
            | MotorCommand::InitPosition { .. }
            | MotorCommand::SetVelocity { .. }
            | MotorCommand::Poll => {
                self.selection.claim(self.index);
            }
            _ => {}
        }
        !self.selection.holds(self.index)
    }

    /// devEMC18011.cc:390-394: SET_HIGH_LIMIT and SET_LOW_LIMIT are ERROR
    /// once past the deferral, which the record asks first.
    fn limit(&self, _high: bool, _raw: f64, _mres: f64) -> LimitBuild {
        LimitBuild::Refused(None)
    }
}

/// One EMC18011 axis sharing a controller. Implements [`AsynMotor`].
pub struct Emc18011Axis {
    controller: Arc<Mutex<Emc18011Controller>>,
    /// 0-based motor index (wire selection is `index + 1`).
    index: usize,
    prev_position: f64,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// missed `Z` (RETRY, drvEMC18011.cc:303), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :306). Per axis, as the
    /// controller's state can move on under another axis's poll.
    failure: PollFailure,
    strings: MoveStrings,
}

impl Emc18011Axis {
    /// Construct axis `index` (0-based; wire selection = `index + 1`) and
    /// read its status as C `motor_init` does (drvEMC18011.cc:725-730):
    /// select it, then `Z` and, once stopped, `A`, which releases it.
    /// Performs blocking serial I/O; failures are ignored as in C.
    pub fn new(controller: Arc<Mutex<Emc18011Controller>>, index: usize) -> Self {
        let mut axis = Self {
            controller,
            index,
            prev_position: 0.0,
            failure: PollFailure::NONE,
            strings: MoveStrings::default(),
        };
        {
            // C sets motorSelect outright (drvEMC18011.cc:728).
            let mut ctrl = axis.lock();
            ctrl.selection.0.store(index as i32, Ordering::Release);
            ctrl.wired = None;
            let _ = ctrl.select(index);
        }
        let _ = axis.poll(&AsynUser::default());
        axis
    }

    /// The record side of this motor's devEMC18011 `build_trans`.
    pub fn build_trans(&self) -> Arc<Emc18011Build> {
        Arc::new(Emc18011Build {
            selection: self.lock().selection.clone(),
            index: self.index,
        })
    }

    fn status(&self, z: &ZStatus) -> MotorStatus {
        MotorStatus {
            position: self.prev_position,
            encoder_position: 0.0,
            velocity: 0.0,
            done: z.done,
            high_limit: z.plus_ls,
            low_limit: z.minus_ls,
            direction: z.direction,
            ..MotorStatus::default()
        }
    }

    fn lock(&self) -> MutexGuard<'_, Emc18011Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Program the velocity, then send PREM and `move_str`, then `A` (which
    /// both starts motion and returns an acknowledgment), after selecting
    /// this motor.
    fn issue_move(&mut self, velocity: f64, move_str: &str) -> AsynResult<()> {
        let prem = self.strings.arm();
        let mut ctrl = self.lock();
        ctrl.select(self.index)?;
        ctrl.send(vel_cmd(velocity * DRIVE_RESOLUTION).as_bytes());
        send_prem(&ctrl, &prem);
        ctrl.send(move_str.as_bytes());
        ctrl.send(b"A");
        Ok(())
    }
}

/// PREM as its own message ahead of the move's (devEMC18011.cc:251-264).
fn send_prem(ctrl: &Emc18011Controller, prem: &[u8]) {
    if !prem.is_empty() {
        ctrl.send(prem);
    }
}

impl AsynMotor for Emc18011Axis {
    // devEMC18011.cc:129 forces NTM to NO.
    fn disables_ntm(&self) -> bool {
        true
    }

    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    // drvEMC18011.cc:356-358: the position is not updated in motion.
    fn no_motion_test(&self) -> bool {
        false
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.issue_move(velocity, &move_cmd('G', position * DRIVE_RESOLUTION))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.issue_move(velocity, &move_cmd('T', distance * DRIVE_RESOLUTION))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C hangs POST off the `V` message, and the direction message's
        // VELOCITY node replaces it, so C never sends it; not copied.
        let prem = self.strings.arm();
        let mut ctrl = self.lock();
        ctrl.select(self.index)?;
        send_prem(&ctrl, &prem);
        // C prints the signed velocity, a `V-` the direction character
        // already carries; not copied.
        ctrl.send(vel_cmd((velocity * DRIVE_RESOLUTION).abs()).as_bytes());
        // Direction character: '>' unless C's NINT(steps/s) is negative.
        let dir = if nint(velocity) >= 0 { ">" } else { "<" };
        ctrl.send(dir.as_bytes());
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C HOME_FOR/HOME_REV ERROR (devEMC18011.cc:298-301) leaves the
        // record DMOV=0 forever; not copied. Its SET_VELOCITY claimed the
        // controller, which C's GO poll releases; released here.
        let mut ctrl = self.lock();
        if ctrl.selection.holds(self.index) {
            let _ = ctrl.select(self.index);
            ctrl.release();
        }
        Err(oriel_err("EMC18011: homing is not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let mut ctrl = self.lock();
        // Only the selected (moving) motor can be stopped; otherwise no-op.
        if ctrl.selection.holds(self.index) {
            ctrl.select(self.index)?;
            ctrl.send(b"S");
        }
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C LOAD_POS only supports zeroing the (selected) motor with "CA".
        if nint(position) != 0 {
            return Ok(());
        }
        let mut ctrl = self.lock();
        // If another motor is active, defer (no-op), matching C's deferral.
        if matches!(ctrl.selection.holder(), Some(a) if a != self.index) {
            return Ok(());
        }
        ctrl.select(self.index)?;
        ctrl.send(b"CA");
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, _enable: bool) -> AsynResult<()> {
        // C ENABLE_TORQUE/DISABL_TORQUE return ERROR — no-op here.
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // C SET_PGAIN/IGAIN/DGAIN return ERROR — no-op here.
        Ok(())
    }

    /// `PRIMITIVE`: INIT as its own message, to this motor only; the
    /// record's build_trans deferred it unless this motor holds the
    /// controller (devEMC18011.cc:234-249).
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let mut ctrl = self.lock();
        if matches!(ctrl.selection.holder(), Some(a) if a != self.index) {
            return Ok(());
        }
        ctrl.select(self.index)?;
        ctrl.send(init);
        Ok(())
    }

    fn set_move_strings(
        &mut self,
        _user: &AsynUser,
        prem: &[u8],
        post: Option<PostMessage>,
    ) -> AsynResult<()> {
        self.strings.set(prem, post);
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());

        // Only the selected motor yields fresh status; C reports the others
        // as Z_STOPPED at their last position after a 0.1 s "in-case of
        // record retry loop", and the controller back in NORMAL
        // (drvEMC18011.cc:286-297).
        if !ctrl.selection.holds(self.index) {
            ctrl.comm_state = CommState::Normal;
            drop(ctrl);
            sleep(UNSELECTED_DELAY);
            return Ok(self.status(&decode_z(Z_STOPPED)));
        }
        let _ = ctrl.select(self.index);

        // C drvEMC18011.cc:249-321: a missing or unknown `Z` reply fails the
        // poll raising nothing once (RETRY); a second in a row raises COMM_ERR.
        let z_char = ctrl
            .command("Z")
            .ok()
            .and_then(|r| r.chars().next())
            .filter(|c| (Z_STOPPED..=Z_LSUP).contains(c));
        let Some(z_char) = z_char else {
            self.failure = if ctrl.comm_state == CommState::Normal {
                ctrl.comm_state = CommState::Retry;
                drop(ctrl);
                sleep(MOTION_DELAY);
                PollFailure::NONE
            } else {
                ctrl.comm_state = CommState::CommErr;
                PollFailure::PROBLEM_AND_COMMS_ERROR
            };
            return Err(oriel_err(format!(
                "EMC18011 motor {}: no Z reply",
                self.index + 1
            )));
        };
        ctrl.comm_state = CommState::Normal;
        let z = decode_z(z_char);

        let mut position = self.prev_position;
        if !z.moving {
            // Stopped or at a limit: a position reply releases the slot; with
            // none C keeps the motor selected and the old position.
            // C tests the strtod end against an uninitialised `brkptr`
            // (drvEMC18011.cc:225, :270), which passes any reply.
            if let Some(reply) = ctrl.command("A").ok().filter(|r| !r.is_empty()) {
                position = f64::from(nint(atof(&reply) / DRIVE_RESOLUTION));
                ctrl.release();
            }
        }
        // drvEMC18011.cc:347, 379-386, reading the reply C leaves for the
        // next command's flush.
        if (z.done || z.plus_ls || z.minus_ls)
            && let Some(post) = self.strings.post_due()
        {
            ctrl.send(&post);
        }
        drop(ctrl);

        self.prev_position = position;
        Ok(self.status(&z))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use std::sync::Mutex as StdMutex;

    /// Motor 1 of a controller on a [`LineRig`] whose `Z` reply is `z`.
    fn strings_rig(name: &str) -> (LineRig, Emc18011Axis, Arc<StdMutex<char>>) {
        let z = Arc::new(StdMutex::new(Z_STOPPED));
        let zr = z.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "R" => RTN_REMOTE.into(),
            "Z" => zr.lock().unwrap().to_string(),
            "A" => "1.0".into(),
            _ => "OK".into(),
        });
        let ctrl = Emc18011Controller::new(io).unwrap();
        let axis = Emc18011Axis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        (line, axis, z)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut Emc18011Axis) -> Vec<String> {
        axis.poll(&AsynUser::default()).unwrap();
        line.sent()
            .into_iter()
            .filter(|l| l != "Z" && l != "A")
            .collect()
    }

    /// drvEMC18011.cc:725-730: each motor is selected and read at init, so
    /// a motor away from zero reports its position before any move.
    #[test]
    fn init_reads_each_motors_position() {
        let (line, io) = LineRig::new("emcInitPos", |last| match last {
            "R" => RTN_REMOTE.into(),
            "Z" => Z_STOPPED.to_string(),
            "A" => "1.23".into(),
            _ => "OK".into(),
        });
        let ctrl = Arc::new(Mutex::new(Emc18011Controller::new(io).unwrap()));
        line.sent();
        let mut axis = Emc18011Axis::new(ctrl.clone(), 1);
        assert_eq!(line.sent(), ["M2", "Z", "A"]);
        assert_eq!(ctrl.lock().unwrap().selection.holder(), None);
        assert_eq!(axis.poll(&AsynUser::default()).unwrap().position, 123.0);
    }

    /// devEMC18011.cc:243-249: INIT as its own message.
    #[test]
    fn init_goes_out_as_its_own_message() {
        let (line, mut axis, _) = strings_rig("emcInit");
        axis.primitive(&AsynUser::default(), b"CA").unwrap();
        assert_eq!(line.sent(), ["M1", "CA"]);
    }

    /// devEMC18011.cc:251-264: PREM as its own message ahead of the move's,
    /// for that move only.
    #[test]
    fn prem_goes_ahead_of_the_move() {
        let (line, mut axis, _) = strings_rig("emcPrem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"CA", None).unwrap();
        axis.move_absolute(&user, 150.0, 0.0, 200.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["M1", "V2.00", "CA", "G1.5", "A"]);
        axis.move_relative(&user, 150.0, 0.0, 200.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["V2.00", "T1.5", "A"]);
        axis.set_move_strings(&user, b"CA", None).unwrap();
        axis.move_velocity(&user, 0.0, -200.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["CA", "V2.00", "<"]);
    }

    /// drvEMC18011.cc:379-386: POST once the move is done, not while it
    /// runs; a jog's too.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, z) = strings_rig("emcPost");
        let user = AsynUser::default();
        for jog in [false, true] {
            axis.set_move_strings(&user, b"", Some(post(b"S"))).unwrap();
            if jog {
                axis.move_velocity(&user, 0.0, 2.0, 0.0).unwrap();
            } else {
                axis.move_relative(&user, 1.5, 0.0, 2.0, 0.0).unwrap();
            }
            line.sent();
            *z.lock().unwrap() = Z_RUNDOWN;
            assert!(poll_extra(&line, &mut axis).is_empty());
            *z.lock().unwrap() = Z_STOPPED;
            assert_eq!(poll_extra(&line, &mut axis), ["S"]);
        }
    }

    /// drvEMC18011.cc:347: either limit ends the move too.
    #[test]
    fn post_follows_a_move_a_limit_stops() {
        let (line, mut axis, z) = strings_rig("emcPostLs");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"S"))).unwrap();
        axis.move_relative(&user, -1.5, 0.0, 2.0, 0.0).unwrap();
        line.sent();
        *z.lock().unwrap() = Z_LSDOWN;
        assert_eq!(poll_extra(&line, &mut axis), ["S"]);
    }

    /// devEMC18011.cc:189, 371-374 and drvEMC18011.cc:272: steps go out
    /// times drive_resolution, `A` comes back divided by it, and a jog
    /// under half a step/s backwards is still `>`.
    #[test]
    fn record_steps_scale_by_drive_resolution() {
        let (line, mut axis, _) = strings_rig("emcSteps");
        let user = AsynUser::default();
        axis.move_absolute(&user, 1234.0, 0.0, 50.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["M1", "V0.50", "G12.3", "A"]);
        assert_eq!(axis.poll(&user).unwrap().position, 100.0);
        axis.move_velocity(&user, 0.0, -0.3, 0.0).unwrap();
        assert_eq!(line.sent().last().unwrap(), ">");
    }

    /// devEMC18011.cc:317-327: the record's SET_VELOCITY goes out with every
    /// move, a zero one too.
    #[test]
    fn a_zero_velocity_move_still_sends_v() {
        let (line, mut axis, _) = strings_rig("emcV0");
        let user = AsynUser::default();
        axis.move_absolute(&user, 100.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["M1", "V0.00", "G1.0", "A"]);
    }

    #[test]
    fn move_cmd_precision_and_clamp() {
        assert_eq!(move_cmd('G', 12.34), "G12.3");
        assert_eq!(move_cmd('T', -5.0), "T-5.0");
        // > 7 chars at .1f -> drop to .0f.
        assert_eq!(move_cmd('G', 123456.0), "G123456");
        // Still too long -> clamp.
        assert_eq!(move_cmd('G', 12345678.0), "G9999999");
        assert_eq!(move_cmd('G', -12345678.0), "G-999999");
    }

    /// C's `%.1f` spells a NaN target `nan`, keeping its sign.
    #[test]
    fn a_nan_target_goes_out_as_c_printf_spells_it() {
        assert_eq!(move_cmd('G', -f64::NAN), "G-nan");
        assert_eq!(move_cmd('T', f64::NAN), "Tnan");
    }

    #[test]
    fn vel_cmd_magnitude_precision() {
        assert_eq!(vel_cmd(1.234), "V1.23");
        assert_eq!(vel_cmd(12.34), "V12.3");
        assert_eq!(vel_cmd(123.4), "V123");
        assert_eq!(vel_cmd(500.0), "V200");
    }

    #[test]
    fn decode_z_states() {
        let s = decode_z(Z_STOPPED);
        assert!(s.done && !s.moving && !s.plus_ls && !s.minus_ls);

        let s = decode_z(Z_RUNDOWN);
        assert!(!s.done && s.moving);

        let s = decode_z(Z_LSUP);
        assert!(!s.done && !s.moving && s.plus_ls && s.direction);

        let s = decode_z(Z_LSDOWN);
        assert!(s.minus_ls && !s.direction);

        // Unknown char falls back to negative motion (still moving).
        let s = decode_z('z');
        assert!(s.moving);
    }

    /// drvEMC18011.cc:249-321: a first bad `Z` is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and a good one clears it.
    #[test]
    fn a_bad_z_reply_is_retry_then_comm_error() {
        let (line, mut axis, z) = strings_rig("emcRetry");
        let user = AsynUser::default();
        axis.move_relative(&user, 1.5, 0.0, 2.0, 0.0).unwrap();
        line.sent();
        *z.lock().unwrap() = 'x';
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        *z.lock().unwrap() = Z_RUNDOWN;
        let s = axis.poll(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        *z.lock().unwrap() = 'x';
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }

    /// Motors 1 and 2 of one controller on a [`LineRig`] whose `Z` reply
    /// is `z`, both released after init.
    fn pair_rig(name: &str) -> (LineRig, [Emc18011Axis; 2], Arc<StdMutex<char>>) {
        let z = Arc::new(StdMutex::new(Z_STOPPED));
        let zr = z.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "R" => RTN_REMOTE.into(),
            "Z" => zr.lock().unwrap().to_string(),
            "A" => "1.0".into(),
            _ => "OK".into(),
        });
        let ctrl = Arc::new(Mutex::new(Emc18011Controller::new(io).unwrap()));
        let axes = [
            Emc18011Axis::new(ctrl.clone(), 0),
            Emc18011Axis::new(ctrl, 1),
        ];
        line.sent();
        (line, axes, z)
    }

    /// drvEMC18011.cc:286-297: an unselected motor reads as stopped off
    /// its limits at its last position, and resets the controller to
    /// NORMAL, so the selected motor's next bad `Z` is RETRY again.
    #[test]
    fn an_unselected_motor_reads_stopped_and_resets_the_comm_state() {
        let (_line, [mut a, mut b], z) = pair_rig("emcUnsel");
        let user = AsynUser::default();
        assert!(!b.build_trans().defers(&MotorCommand::Poll));
        *z.lock().unwrap() = Z_LSUP;
        assert!(b.poll(&user).unwrap().high_limit);

        assert!(!a.build_trans().defers(&MotorCommand::Poll));
        *z.lock().unwrap() = 'x';
        assert!(a.poll(&user).is_err());
        assert_eq!(a.poll_failure(), PollFailure::NONE);
        let s = b.poll(&user).unwrap();
        assert!(s.done && s.direction && !s.high_limit && !s.low_limit);
        assert_eq!(s.position, 100.0);
        assert!(a.poll(&user).is_err());
        assert_eq!(a.poll_failure(), PollFailure::NONE);
    }

    /// devEMC18011.cc:206-231: SET_VELOCITY claims a free controller; the
    /// other motor's commands defer until a stopped poll releases it.
    #[test]
    fn an_unselected_motors_commands_defer() {
        let (_line, [a, b], z) = pair_rig("emcDefer");
        let (ba, bb) = (a.build_trans(), b.build_trans());
        let velocity = MotorCommand::SetVelocity { velocity: 1.0 };
        let stop = MotorCommand::Stop { acceleration: 1.0 };
        let closed = MotorCommand::SetClosedLoop { enable: true };

        // A command that claims nothing defers on a free controller.
        assert!(ba.defers(&closed));
        assert!(!ba.defers(&velocity));
        assert!(!ba.defers(&closed));
        assert!(bb.defers(&velocity));
        assert!(bb.defers(&stop));
        assert!(bb.defers(&MotorCommand::Poll));
        assert!(!bb.defers(&MotorCommand::OutParm {
            parm: Default::default()
        }));

        let mut a = a;
        *z.lock().unwrap() = Z_RUNUP;
        a.poll(&AsynUser::default()).unwrap();
        assert!(bb.defers(&MotorCommand::Poll));
        *z.lock().unwrap() = Z_STOPPED;
        a.poll(&AsynUser::default()).unwrap();
        assert!(!bb.defers(&MotorCommand::Poll));
        assert!(ba.defers(&velocity));
    }

    /// devEMC18011.cc:217-222: the claim's `M<n>` goes out ahead of the
    /// claiming motor's first command, and its poll then reads it.
    #[test]
    fn a_claim_selects_the_motor_on_the_wire() {
        let (line, [mut a, mut b], _) = pair_rig("emcClaim");
        let user = AsynUser::default();
        assert!(!b.build_trans().defers(&MotorCommand::Poll));
        a.poll(&user).unwrap();
        assert_eq!(line.sent(), Vec::<String>::new());
        b.poll(&user).unwrap();
        assert_eq!(line.sent(), ["M2", "Z", "A"]);

        assert!(
            !a.build_trans()
                .defers(&MotorCommand::SetVelocity { velocity: 2.0 })
        );
        a.move_absolute(&user, 150.0, 0.0, 200.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["M1", "V2.00", "G1.5", "A"]);
    }

    /// devEMC18011.cc:390-394 with motorRecord.cc:4112-4121: the selected
    /// motor's refused HLM/LLM go back to the dial limit; an unselected
    /// motor's limit is deferred to a bare GET_INFO and kept.
    #[test]
    fn limits_refuse_when_selected_and_defer_otherwise() {
        use epics_rs::base::server::record::Record;
        use epics_rs::base::types::EpicsValue;
        use epics_rs::motor::MotorRecord;
        use epics_rs::motor::device_support::CommandPort;
        use epics_rs::motor::flags::MstaFlags;
        use epics_rs::motor::poll_loop::ToAxis;

        let (_line, [a, b], _) = pair_rig("emcLimit");
        let record = |axis: &Emc18011Axis| {
            let mut rec = MotorRecord::new();
            let (port, rx) = CommandPort::capture();
            rec.set_command_port(port.with_build_trans(axis.build_trans()));
            rec.conv.mres = 1.0;
            rec.stat.msta = MstaFlags::DONE;
            rec.limits.dhlm = 100.0;
            rec.limits.hlm = 100.0;
            rec.limits.dllm = -100.0;
            rec.limits.llm = -100.0;
            rec.limits.rhlm = 100.0;
            rec.limits.rllm = -100.0;
            rec.internal.init_invariants_synced = true;
            (rec, rx)
        };
        macro_rules! sent {
            ($rx:expr) => {{
                let mut commands = Vec::new();
                while let Ok(to_axis) = $rx.try_recv() {
                    if let ToAxis::Batch(batch) = to_axis {
                        commands.extend(batch.commands);
                    }
                }
                commands
            }};
        }
        let (mut ra, mut rxa) = record(&a);
        let (mut rb, mut rxb) = record(&b);
        assert!(!a.build_trans().defers(&MotorCommand::Poll));

        ra.put_field("HLM", EpicsValue::Double(50.0)).unwrap();
        ra.put_field("LLM", EpicsValue::Double(-50.0)).unwrap();
        assert_eq!(sent!(rxa), []);
        assert_eq!(
            (ra.limits.hlm, ra.limits.dhlm, ra.limits.rhlm),
            (100.0, 100.0, 100.0)
        );
        assert_eq!(
            (ra.limits.llm, ra.limits.dllm, ra.limits.rllm),
            (-100.0, -100.0, -100.0)
        );

        rb.put_field("HLM", EpicsValue::Double(50.0)).unwrap();
        assert_eq!(sent!(rxb), [MotorCommand::Poll]);
        assert_eq!((rb.limits.hlm, rb.limits.dhlm), (50.0, 50.0));
    }

    /// A selection released between two of a move's commands, as the
    /// holder's own stopped poll on the poller thread can: the commands
    /// built before the deferred MOVE still go out, and the deferred last
    /// one leaves a status query (motordrvCom.cc:441).
    #[test]
    fn commands_built_before_a_deferred_move_still_go_out() {
        use epics_rs::base::server::record::Record;
        use epics_rs::base::types::EpicsValue;
        use epics_rs::motor::MotorRecord;
        use epics_rs::motor::device_state::{StampedStatus, new_shared_state};
        use epics_rs::motor::device_support::CommandPort;
        use std::sync::atomic::AtomicUsize;

        /// Oriel's build_trans with the controller released after the
        /// first command it sees.
        struct ReleasedMidBatch {
            inner: Arc<Emc18011Build>,
            seen: AtomicUsize,
        }
        impl BuildTrans for ReleasedMidBatch {
            fn defers(&self, command: &MotorCommand) -> bool {
                let deferred = self.inner.defers(command);
                if self.seen.fetch_add(1, Ordering::Relaxed) == 0 {
                    self.inner
                        .selection
                        .0
                        .store(Selection::NONE, Ordering::Release);
                }
                deferred
            }
            fn limit(&self, high: bool, raw: f64, mres: f64) -> LimitBuild {
                self.inner.limit(high, raw, mres)
            }
        }

        let (_line, [a, _b], _) = pair_rig("emcMidBatch");
        let device = Arc::new(ReleasedMidBatch {
            inner: a.build_trans(),
            seen: AtomicUsize::new(usize::MAX / 2),
        });
        let status = |seq| StampedStatus {
            seq,
            status: MotorStatus {
                done: true,
                ..MotorStatus::default()
            },
        };
        let state = new_shared_state();
        state.lock().unwrap().latest_status = Some(status(1));
        let mut rec = MotorRecord::new();
        rec.conv.mres = 1.0;
        rec.limits.dhlm = 100.0;
        rec.limits.hlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.limits.llm = -100.0;
        rec.vel.velo = 10.0;
        rec.vel.accl = 0.5;
        rec.set_device_state(state.clone());
        let (port, _rx) = CommandPort::capture();
        rec.set_command_port(port.with_build_trans(device.clone()));
        rec.process().unwrap();
        state.lock().unwrap().pending_actions.take();

        device.seen.store(0, Ordering::Relaxed);
        rec.put_field("VAL", EpicsValue::Double(10.0)).unwrap();
        rec.process().unwrap();
        let commands = state
            .lock()
            .unwrap()
            .pending_actions
            .take()
            .map(|a| a.commands)
            .unwrap_or_default();
        assert!(
            matches!(
                commands[..],
                [MotorCommand::SetVelocity { .. }, MotorCommand::Poll]
            ),
            "{commands:?}"
        );
    }
}
