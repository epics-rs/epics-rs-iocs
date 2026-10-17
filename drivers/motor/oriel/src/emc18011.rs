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
//! reports status/position. This port models that with an `active` slot on the
//! controller. A move on an idle controller selects its motor (`M<n>`) and
//! claims the slot; [`poll`](AsynMotor::poll) releases the slot when the
//! selected motor stops (or hits a limit) and a valid position has been read.
//!
//! **Deviation from C:** the C driver defers a move requested while another
//! motor is active by returning without sending and leaning on the motor
//! record's retry loop to complete it later. The asyn-rs [`AsynMotor`] boundary
//! has no equivalent deferred-retry queue, so this port instead rejects a move
//! aimed at a non-active motor while another is in motion, with a "controller
//! busy" error. Concurrent moves on this single-channel controller are a
//! hardware impossibility either way; only the surfacing differs.
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
//! The controller works in millimetres (`A` returns mm, `G`/`T` take mm). The C
//! driver multiplies the record's raw steps by a fixed `drive_resolution`
//! (0.01) to reach mm and divides on readback; at the asyn-rs motor boundary,
//! which is dial-frame EGU, that scaling cancels, so this port works directly in
//! mm (`EGU` is mm). The record's `MRES` must still be `drive_resolution`:
//! raw counts are integers, so it is the step.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `Z`/`A` reads run inside
//!   [`poll`](AsynMotor::poll).
//! - Homing, PID gains, torque enable/disable and travel-limit setting are
//!   unsupported in C (`build_trans` returns `ERROR`); [`home`](AsynMotor::home)
//!   returns an error and the others are no-ops.
//! - `set_position` only supports zeroing (`CA`), matching C `LOAD_POS`.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::MoveStrings;
use motor_common::util::{atof, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 120;

/// C `MOTION_DELAY` (drvEMC18011.cc:72), slept on entering RETRY.
const MOTION_DELAY: Duration = Duration::from_millis(100);

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
    let mut field = format!("{mm:.1}");
    if field.len() > 7 {
        field = format!("{mm:.0}");
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
        format!("V{mms:.2}")
    } else if mms < 50.0 {
        format!("V{mms:.1}")
    } else if mms < 200.0 {
        format!("V{mms:.0}")
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

/// Shared controller endpoint: owns the serial handle and the single-motor
/// selection slot.
pub struct Emc18011Controller {
    handle: SyncIOHandle,
    ident: String,
    /// The 0-based index of the motor currently selected and in motion, if any.
    active: Option<usize>,
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
            active: None,
            comm_state: CommState::Normal,
        };

        let mut online = false;
        for _ in 0..3 {
            ctrl.command("L")?; // local
            let reply = ctrl.command("R")?; // remote
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

        ctrl.command("S")?; // ensure all motion stopped
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

    /// Send a command and return its reply (trimmed). Every command produces a
    /// reply, so this is used for both queries and set commands (whose reply is
    /// discarded) to keep the stream synchronized.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.command_bytes(cmd.as_bytes())
    }

    /// [`command`](Self::command) for a record string's raw bytes.
    fn command_bytes(&self, cmd: &[u8]) -> AsynResult<String> {
        let raw = self.handle.write_read(0, cmd, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_matches(['\r', '\n', '\0', ' ']).to_string())
    }

    /// Acquire the selection slot for motor `index` (0-based), sending `M<n>`
    /// (1-based) if it is not already selected. Returns a "busy" error if another
    /// motor is currently in motion.
    fn select(&mut self, index: usize) -> AsynResult<()> {
        match self.active {
            Some(a) if a == index => Ok(()),
            Some(a) => Err(oriel_err(format!(
                "EMC18011: controller busy (motor {} in motion)",
                a + 1
            ))),
            None => {
                self.command(&format!("M{}", index + 1))?;
                self.active = Some(index);
                Ok(())
            }
        }
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
    last_status: MotorStatus,
    strings: MoveStrings,
}

impl Emc18011Axis {
    /// Construct axis `index` (0-based; wire selection = `index + 1`).
    pub fn new(controller: Arc<Mutex<Emc18011Controller>>, index: usize) -> Self {
        Self {
            controller,
            index,
            prev_position: 0.0,
            failure: PollFailure::NONE,
            last_status: MotorStatus {
                done: true,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
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
        if velocity > 0.0 {
            ctrl.command(&vel_cmd(velocity))?;
        }
        send_prem(&ctrl, &prem)?;
        ctrl.command(move_str)?;
        ctrl.command("A")?;
        Ok(())
    }
}

/// PREM as its own message ahead of the move's (devEMC18011.cc:251-264).
fn send_prem(ctrl: &Emc18011Controller, prem: &[u8]) -> AsynResult<()> {
    if !prem.is_empty() {
        ctrl.command_bytes(prem)?;
    }
    Ok(())
}

impl AsynMotor for Emc18011Axis {
    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.issue_move(velocity, &move_cmd('G', position))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.issue_move(velocity, &move_cmd('T', distance))
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
        send_prem(&ctrl, &prem)?;
        ctrl.command(&vel_cmd(velocity.abs()))?;
        // Direction character: '>' forward, '<' reverse (C JOG).
        let dir = if velocity >= 0.0 { ">" } else { "<" };
        ctrl.command(dir)?;
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
        // C: HOME_FOR/HOME_REV return ERROR (unsupported).
        Err(oriel_err("EMC18011: homing is not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        // Only the selected (moving) motor can be stopped; otherwise no-op.
        if ctrl.active == Some(self.index) {
            ctrl.command("S")?;
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
        if matches!(ctrl.active, Some(a) if a != self.index) {
            return Ok(());
        }
        ctrl.select(self.index)?;
        ctrl.command("CA")?;
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

    /// `PRIMITIVE`: INIT as its own message, to this motor only. C sends
    /// it only when this motor is already the selected one; here it is
    /// selected, as for `set_position`.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let mut ctrl = self.lock();
        if matches!(ctrl.active, Some(a) if a != self.index) {
            return Ok(());
        }
        ctrl.select(self.index)?;
        ctrl.command_bytes(init)?;
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

        // Only the selected motor yields fresh status; others report last state.
        if ctrl.active != Some(self.index) {
            return Ok(self.last_status.clone());
        }

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
            if let Some(reply) = ctrl.command("A").ok().filter(|r| !r.is_empty()) {
                position = atof(&reply);
                ctrl.active = None;
            }
        }
        // drvEMC18011.cc:347, 379-386, reading the reply C leaves for the
        // next command's flush.
        if (z.done || z.plus_ls || z.minus_ls)
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.command_bytes(&post);
        }
        drop(ctrl);

        self.prev_position = position;

        let status = MotorStatus {
            position,
            encoder_position: 0.0,
            velocity: 0.0,
            done: z.done,
            high_limit: z.plus_ls,
            low_limit: z.minus_ls,
            direction: z.direction,
            ..MotorStatus::default()
        };
        self.last_status = status.clone();
        Ok(status)
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
        axis.move_absolute(&user, 1.5, 0.0, 2.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["M1", "V2.00", "CA", "G1.5", "A"]);
        axis.move_relative(&user, 1.5, 0.0, 2.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["V2.00", "T1.5", "A"]);
        axis.set_move_strings(&user, b"CA", None).unwrap();
        axis.move_velocity(&user, 0.0, -2.0, 0.0).unwrap();
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
}
