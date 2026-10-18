//! Kohzu SC-200/400/800 stepper motor controller driver (serial ASCII).
//!
//! Ported from `motorKohzu/kohzuApp/src/drvSC800.cc` + `devSC800.cc`. The SC-800
//! (and its 4-axis SC-400 / 2-axis SC-200 siblings) is a stepper controller.
//! Every command is prefixed with STX (0x02) and terminated by CR+LF (the
//! port EOS, set both ways at connect as C does).
//! The controller answers **every** command with a tab-separated `C\t…` reply,
//! so each command is written and its reply consumed to keep the stream
//! synchronized. Queries used here:
//!
//! - `STR1/<axis>`  → `C\tSTR<a>\t1\t<move>\t<norg>\t<orgg>\t<cw>\t<ccw>\t<swng>\t<err>`
//! - `RDP<axis>/0`  → `C\tRDP<a>\t<position>`
//! - `RSY<axis>/21` → `C\tRSY<a>\t21\t<torque>` (0 = torque enabled)
//!
//! Motion commands: `ASI` (set base/slew/accel), `APS`/`RPS` (absolute /
//! relative move), `ORG` (home), `FRP` (jog), `WRP` (set position), `STP`
//! (stop), `COF` (torque on/off). The wire axis number is 1-based.
//!
//! ## Units
//!
//! The controller works natively in motor steps (`RDP` returns steps, `APS`
//! takes steps) with no resolution scaling, so the asyn-rs motor boundary is
//! steps: positions pass through with `NINT` rounding, the record's `MRES` is
//! 1, and its `EGU` is steps. Base/slew speeds cross the boundary in steps/s.
//! The controller's acceleration parameter is the record's acceleration *time*
//! scaled by 100 (C `NINT(mr->accl * 100)`); at the boundary the acceleration
//! *rate* (steps/s²) is supplied instead, so the time is reconstructed by
//! inverting motorRecord's accEGUfromVelo, `(velocity - vbase) / acceleration`.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `STR`/`RDP`/`RSY` reads run
//!   inside [`poll`](AsynMotor::poll).
//! - The C move path chooses backlash vs slew acceleration from record fields
//!   (`bvel`/`bacc`/`mres`) not visible at the asyn-rs boundary; this port uses
//!   the acceleration passed with the move.
//! - `HOME_FOR`/`HOME_REV` both emit the same `ORG…/3/1` sequence as in C, so
//!   the requested home direction does not change the command.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, PostMessage};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::MoveStrings;
use motor_common::util::nint;

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 160;

/// Start-of-text prefix on every command (C `STX`).
const STX: u8 = 0x02;

/// Port EOS set at connect (C `SC800_IN_EOS`/`SC800_OUT_EOS`,
/// drvSC800.cc:551-554).
pub const INPUT_EOS: &[u8] = b"\r\n";
pub const OUTPUT_EOS: &[u8] = b"\r\n";

/// Base/slew speed clamp (C `devSC800.cc` SET_VEL_BASE/SET_VELOCITY/JOG validity check).
const SPEED_MIN: i32 = 1;
const SPEED_MAX: i32 = 4095500;

fn kohzu_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle and STX/CRLF framing.
pub struct KohzuController {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
    comm_state: CommState,
}

/// Controller communication state (C `cntrl->status`, shared by the axes):
/// one bad `STR` reply is RETRY, a second in a row COMM_ERR.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// The model and version an IDN reply `C\tIDN0\t<model>\t<version>` names.
fn identify(reply: &str) -> AsynResult<(i32, String)> {
    let fields: Vec<&str> = reply.split('\t').collect();
    match fields.get(2).and_then(|s| s.trim().parse::<i32>().ok()) {
        Some(model @ (800 | 400 | 200)) => {
            Ok((model, fields.get(3).map_or("", |s| s.trim()).to_owned()))
        }
        _ => Err(kohzu_err(format!(
            "SC800: unrecognized IDN reply '{reply}'"
        ))),
    }
}

impl KohzuController {
    /// Connect and identify a Kohzu controller (C `motor_init`): `IDN` reports
    /// the model (SC-800/400/200), which fixes the axis count (8/4/2). Performs
    /// blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes: 0,
            comm_state: CommState::Normal,
        };
        // C sends IDN up to 3 times, until a reply names a model
        // (drvSC800.cc:557-578).
        let mut tries = 0;
        let (model, version) = loop {
            tries += 1;
            match ctrl.command("IDN").and_then(|reply| identify(&reply)) {
                Ok(id) => break id,
                Err(e) if tries == 3 => return Err(e),
                Err(_) => {}
            }
        };
        ctrl.ident = format!("SC-{model} Ver{version}");
        ctrl.num_axes = match model {
            800 => 8,
            400 => 4,
            _ => 2,
        };
        Ok(ctrl)
    }

    /// The identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes fixed by the controller model.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(cmd: impl AsRef<[u8]>) -> Vec<u8> {
        let mut out = vec![STX];
        out.extend_from_slice(cmd.as_ref());
        out
    }

    /// Send a command and return its reply (trimmed of framing). Every SC-800
    /// command produces a `C\t…` reply, so this is used for both queries and
    /// set commands (whose reply is discarded) to keep the stream synchronized.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.command_bytes(cmd.as_bytes())
    }

    /// [`command`](Self::command) for a record string's raw bytes.
    fn command_bytes(&self, cmd: &[u8]) -> AsynResult<String> {
        // C send_mess flushes the input before every write.
        let raw = self.handle.write_read(0, &Self::framed(cmd), READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text
            .trim_matches(|c: char| c == '\r' || c == '\n' || c == '\0' || c == '\u{2}')
            .to_string())
    }
}

/// C's `sscanf(buff, "C\tSTR%d\t1\t%d\t%d\t%d\t%d\t%d\t%d\t%d", ..) == 8`:
/// the move, norg, orgg, cw, ccw, swng and err fields, each a whitespace
/// directive apart as scanf reads `\t`.
fn parse_str(reply: &str) -> Option<[i32; 7]> {
    let rest = reply.strip_prefix('C')?.trim_start().strip_prefix("STR")?;
    let mut words = rest.split_whitespace();
    words.next()?.parse::<i32>().ok()?;
    if words.next()? != "1" {
        return None;
    }
    let mut fields = [0; 7];
    for f in &mut fields {
        *f = words.next()?.parse().ok()?;
    }
    Some(fields)
}

/// The three programmable move speeds (C `base_speed`/`slew_speed`/`accl_rate`).
#[derive(Clone, Copy)]
struct Speeds {
    base: i32,
    slew: i32,
    accel: i32,
}

/// Derive the programmable speeds from move parameters. Base/slew are the
/// min/target velocities (steps/s), clamped as SET_VEL_BASE/SET_VELOCITY do
/// (devSC800.cc). The controller's acceleration parameter is C
/// `NINT(mr->accl * 100)`, the ACCL that motorRecord's accEGUfromVelo turned
/// into `acceleration`; when it cannot be derived (non-positive acceleration)
/// `prev_accel` is kept.
fn compute_speeds(min_velocity: f64, velocity: f64, acceleration: f64, prev_accel: i32) -> Speeds {
    let accel = if acceleration > 0.0 {
        let rise = if velocity > min_velocity {
            velocity - min_velocity
        } else {
            velocity
        };
        nint(rise / acceleration * 100.0)
    } else {
        prev_accel
    };
    Speeds {
        base: nint(min_velocity).clamp(SPEED_MIN, SPEED_MAX),
        slew: nint(velocity).clamp(SPEED_MIN, SPEED_MAX),
        accel,
    }
}

/// One SC-800 axis sharing a controller. Implements [`AsynMotor`].
pub struct KohzuAxis {
    controller: Arc<Mutex<KohzuController>>,
    /// 1-based wire axis number.
    axis: u32,
    speeds: Speeds,
    prev_position: i32,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// bad `STR` (RETRY), `CNTRL_COMM_ERR` and `RA_PROBLEM` on the next
    /// (COMM_ERR). Per axis, as the controller's state moves under another
    /// axis's poll.
    failure: PollFailure,
    last_status: MotorStatus,
    strings: MoveStrings,
}

impl KohzuAxis {
    /// Construct axis `index` (0-based; wire axis = `index + 1`).
    pub fn new(controller: Arc<Mutex<KohzuController>>, index: usize) -> Self {
        Self {
            controller,
            axis: index as u32 + 1,
            speeds: Speeds {
                base: 0,
                slew: 0,
                accel: 100,
            },
            prev_position: 0,
            failure: PollFailure::NONE,
            last_status: MotorStatus {
                gain_support: true,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, KohzuController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Update the stored speeds from move parameters (keeping the previous
    /// acceleration parameter when it cannot be derived).
    fn set_speeds(&mut self, min_velocity: f64, velocity: f64, acceleration: f64) {
        self.speeds = compute_speeds(min_velocity, velocity, acceleration, self.speeds.accel);
    }

    /// A move: PREM as its own command, the axis speeds, then `command`. C puts
    /// PREM ahead of `ASI` in one message that `write_parms` strcpy's `ASI`
    /// over, and the move's message replaces the motion node holding POST,
    /// so C sends neither (devSC800.cc:198-217, write_parms); not copied.
    fn write_move(&mut self, command: &str) -> AsynResult<()> {
        let prem = self.strings.arm();
        let ctrl = self.lock();
        if !prem.is_empty() {
            ctrl.command_bytes(&prem)?;
        }
        self.program_speeds(&ctrl)?;
        ctrl.command(command)?;
        Ok(())
    }

    /// Program the axis speeds (C `write_parms` → `ASI`).
    fn program_speeds(&self, ctrl: &KohzuController) -> AsynResult<()> {
        let s = self.speeds;
        ctrl.command(&format!(
            "ASI{}/{}/{}/{}/{}/0/0/0/0/0/0/0/0/0",
            self.axis, s.base, s.slew, s.accel, s.accel
        ))?;
        Ok(())
    }
}

impl AsynMotor for KohzuAxis {
    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.set_speeds(min_velocity, velocity, acceleration);
        self.write_move(&format!("APS{}/2/0/0/{}/0/0/1", self.axis, nint(position)))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.set_speeds(min_velocity, velocity, acceleration);
        self.write_move(&format!("RPS{}/2/1/0/{}/0/0/1", self.axis, nint(distance)))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C JOG: cap base speed to the slew speed, program, then FRP
        // with the direction polarity.
        // C takes the polarity from NINT(v), so |v| < 0.5 jogs reverse.
        let polarity = if nint(velocity) > 0 { '1' } else { '0' };
        self.set_speeds(min_velocity, velocity.abs(), acceleration);
        if self.speeds.base > self.speeds.slew {
            self.speeds.base = self.speeds.slew;
        }
        // C `NINT((jvel - vbas) / jar * 100)`, with no VBAS guard.
        if acceleration > 0.0 {
            self.speeds.accel = nint((velocity.abs() - min_velocity) / acceleration * 100.0);
        }
        self.write_move(&format!("FRP{}/2/0/0/{}/1", self.axis, polarity))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C HOME_FOR/HOME_REV both emit the same ORG sequence.
        // C sends SET_ACCEL's steps/s^2 as ASI's time (devSC800.cc:274); not copied.
        self.set_speeds(min_velocity, velocity, acceleration);
        self.write_move(&format!("ORG{}/2/0/0/3/1", self.axis))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.command(&format!("STP{}/0", self.axis))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.command(&format!("WRP{}/{}", self.axis, nint(position)))?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // COF/0 enables torque, COF/1 disables it.
        let ctrl = self.lock();
        ctrl.command(&format!("COF{}/{}", self.axis, i32::from(!enable)))?;
        Ok(())
    }

    /// `PRIMITIVE`: INIT as its own command. C's PRIMITIVE falls to the
    /// `send = false` default, so C never sends it (devSC800.cc:193-197,
    /// 328-330); not copied.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        self.lock().command_bytes(init)?;
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
        self.poll_status(None)
    }

    /// drvSC800.cc:278-282: a moving axis reports the record's CDIR.
    fn poll_in_motion(&mut self, _user: &AsynUser, cdir: &mut bool) -> AsynResult<MotorStatus> {
        self.poll_status(Some(*cdir))
    }
}

impl KohzuAxis {
    /// C `set_status`; `cdir` is the record's while its move is held.
    fn poll_status(&mut self, cdir: Option<bool>) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        // Status: "C\tSTR<a>\t1\t<move>\t<norg>\t<orgg>\t<cw>\t<ccw>\t<swng>\t<err>"
        let status_reply = ctrl.command(&format!("STR1/{}", self.axis));
        // drvSC800.cc:229-249: a bad reply fails the poll raising nothing
        // once (RETRY); a second in a row raises COMM_ERR. Nothing else is
        // sent on that poll.
        let Some(sf) = status_reply.ok().as_deref().and_then(parse_str) else {
            (ctrl.comm_state, self.failure) = match ctrl.comm_state {
                CommState::Normal => (CommState::Retry, PollFailure::NONE),
                _ => (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR),
            };
            return Err(kohzu_err(format!("SC800 axis {}: no STR reply", self.axis)));
        };
        ctrl.comm_state = CommState::Normal;
        let pos_reply = ctrl.command(&format!("RDP{}/0", self.axis));
        let torque_reply = ctrl.command(&format!("RSY{}/21", self.axis));
        let str_move = sf[0];
        let plus_ls = sf[3] == 1; // cw limit
        let minus_ls = sf[4] == 1; // ccw limit
        let done = str_move == 0;

        // Position: "C\tRDP<a>\t<position>"
        let position = pos_reply
            .ok()
            .and_then(|r| {
                let f: Vec<&str> = r.split('\t').collect();
                f.get(2).and_then(|s| s.trim().parse::<i32>().ok())
            })
            .unwrap_or(self.prev_position);
        let direction = cdir.unwrap_or(if position != self.prev_position {
            position >= self.prev_position
        } else {
            self.last_status.direction
        });
        self.prev_position = position;

        // Torque: "C\tRSY<a>\t21\t<torque>" (0 = enabled).
        let powered = torque_reply
            .ok()
            .and_then(|r| {
                let f: Vec<&str> = r.split('\t').collect();
                f.get(3).and_then(|s| s.trim().parse::<i32>().ok())
            })
            .map(|v| v == 0)
            .unwrap_or(false);

        // drvSC800.cc:325-332, reading the reply C leaves for the next
        // command's flush.
        let ls_active = (plus_ls && direction) || (minus_ls && !direction);
        if (done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.command_bytes(&post);
        }
        drop(ctrl);

        self.last_status = MotorStatus {
            position: position as f64,
            encoder_position: position as f64,
            velocity: 0.0,
            direction,
            done,
            high_limit: plus_ls,
            low_limit: minus_ls,
            home: false,
            powered,
            comms_error: false,
            problem: false,
            // C motor_init sets GAIN_SUPPORT, so CNEN drives COF.
            gain_support: true,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use std::sync::atomic::{AtomicU8, Ordering};

    /// `STR` fields of the rig: moving, cw limit, ccw limit.
    const MOVING: u8 = 1;
    const CW: u8 = 2;
    const CCW: u8 = 4;
    /// Every reply empty.
    const QUIET: u8 = 8;

    /// An SC-200's axis 1 on a [`LineRig`] whose `STR` reply follows
    /// `state`.
    fn strings_rig(name: &str) -> (LineRig, KohzuAxis, Arc<AtomicU8>) {
        let state = Arc::new(AtomicU8::new(0));
        let st = state.clone();
        let (line, io) = LineRig::new(name, move |last| {
            let bit = |b: u8| u8::from(st.load(Ordering::SeqCst) & b != 0);
            if bit(QUIET) == 1 {
                return String::new();
            }
            match last.trim_start_matches('\u{2}') {
                "IDN" => "C\tIDN0\t200\t1.0".into(),
                "STR1/1" => format!(
                    "C\tSTR1\t1\t{}\t0\t0\t{}\t{}\t0\t0",
                    bit(MOVING),
                    bit(CW),
                    bit(CCW)
                ),
                "RDP1/0" => "C\tRDP1\t0".into(),
                "RSY1/21" => "C\tRSY1\t21\t0".into(),
                c => format!("C\t{c}"),
            }
        });
        let ctrl = KohzuController::new(io).unwrap();
        let axis = KohzuAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        (line, axis, state)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// The commands a rig wrote, without their STX.
    fn sent(line: &LineRig) -> Vec<String> {
        line.sent()
            .into_iter()
            .map(|l| l.trim_start_matches('\u{2}').to_string())
            .collect()
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut KohzuAxis) -> Vec<String> {
        axis.poll(&AsynUser::default()).unwrap();
        sent(line)
            .into_iter()
            .filter(|l| !["STR1/1", "RDP1/0", "RSY1/21"].contains(&l.as_str()))
            .collect()
    }

    /// devSC800.cc:193-197: INIT as its own command.
    #[test]
    fn init_goes_out_as_its_own_command() {
        let (line, mut axis, _) = strings_rig("sc800Init");
        axis.primitive(&AsynUser::default(), b"COF1/0").unwrap();
        assert_eq!(sent(&line), ["COF1/0"]);
    }

    /// devSC800.cc:198-217: PREM as its own command ahead of each move,
    /// for that move only.
    #[test]
    fn prem_goes_ahead_of_the_move() {
        let (line, mut axis, _) = strings_rig("sc800Prem");
        let user = AsynUser::default();
        let asi = "ASI1/10/100/9/9/0/0/0/0/0/0/0/0/0";
        axis.set_move_strings(&user, b"COF1/0", None).unwrap();
        axis.move_absolute(&user, 5.0, 10.0, 100.0, 1000.0).unwrap();
        assert_eq!(sent(&line), ["COF1/0", asi, "APS1/2/0/0/5/0/0/1"]);
        axis.move_relative(&user, 5.0, 10.0, 100.0, 1000.0).unwrap();
        assert_eq!(sent(&line), [asi, "RPS1/2/1/0/5/0/0/1"]);
        axis.set_move_strings(&user, b"COF1/0", None).unwrap();
        axis.home(&user, 10.0, 100.0, 1000.0, true).unwrap();
        assert_eq!(sent(&line), ["COF1/0", asi, "ORG1/2/0/0/3/1"]);
        axis.set_move_strings(&user, b"COF1/0", None).unwrap();
        axis.move_velocity(&user, 10.0, -100.0, 1000.0).unwrap();
        assert_eq!(sent(&line), ["COF1/0", asi, "FRP1/2/0/0/0/1"]);
        axis.move_velocity(&user, 0.0, 0.4, 1000.0).unwrap();
        assert!(
            sent(&line).last().unwrap().ends_with("/0/1"),
            "NINT(0.4) = 0"
        );
    }

    /// drvSC800.cc:325-332: POST once the move is done, not while it runs.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, state) = strings_rig("sc800Post");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"COF1/1")))
            .unwrap();
        axis.move_absolute(&user, 5.0, 10.0, 100.0, 1000.0).unwrap();
        line.sent();
        state.store(MOVING, Ordering::SeqCst);
        assert!(poll_extra(&line, &mut axis).is_empty());
        state.store(0, Ordering::SeqCst);
        assert_eq!(poll_extra(&line, &mut axis), ["COF1/1"]);
        assert!(poll_extra(&line, &mut axis).is_empty(), "sent once");
    }

    /// drvSC800.cc:293-308: a limit in the move's direction ends it too.
    #[test]
    fn post_follows_a_move_a_limit_stops() {
        let (line, mut axis, state) = strings_rig("sc800PostLs");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"COF1/1")))
            .unwrap();
        axis.move_absolute(&user, -5.0, 10.0, 100.0, 1000.0)
            .unwrap();
        line.sent();
        state.store(MOVING | CCW, Ordering::SeqCst);
        assert_eq!(poll_extra(&line, &mut axis), ["COF1/1"]);
    }

    /// drvSC800.cc:278-282: a held move reports its record's CDIR, and the
    /// direction stays there at rest until the position moves.
    #[test]
    fn a_held_move_reports_the_record_cdir() {
        let (_line, mut axis, state) = strings_rig("sc800Cdir");
        let user = AsynUser::default();
        state.store(MOVING | CCW, Ordering::SeqCst);
        let mut cdir = false;
        let s = axis.poll_in_motion(&user, &mut cdir).unwrap();
        assert!(!s.direction && !cdir);
        let mut cdir = true;
        assert!(axis.poll_in_motion(&user, &mut cdir).unwrap().direction);
        state.store(0, Ordering::SeqCst);
        assert!(axis.poll(&user).unwrap().direction, "kept at rest");
    }

    #[test]
    fn framed_prefixes_stx() {
        let f = KohzuController::framed("APS1/2/0/0/500/0/0/1");
        assert_eq!(f[0], STX);
        assert_eq!(&f[1..], b"APS1/2/0/0/500/0/0/1");
    }

    #[test]
    fn speeds_reconstruct_accel_time() {
        // accEGUfromVelo inverted: (1000 - 100) / 4500 = ACCL 0.2 s → 20.
        let s = compute_speeds(100.0, 1000.0, 4500.0, 100);
        assert_eq!(s.slew, 1000);
        assert_eq!(s.base, 100);
        assert_eq!(s.accel, 20);
        // VELO <= VBAS: the record's rate is VELO / ACCL.
        assert_eq!(compute_speeds(100.0, 100.0, 500.0, 0).accel, 20);
        // devSC800.cc SET_VEL_BASE/SET_VELOCITY clamp to [1, 4095500].
        let s = compute_speeds(0.0, 5e6, 5000.0, 100);
        assert_eq!((s.base, s.slew), (1, 4_095_500));
        // Non-positive acceleration keeps the previous accel parameter.
        let s = compute_speeds(100.0, 1000.0, 0.0, 42);
        assert_eq!(s.accel, 42);
    }

    /// drvSC800.cc:229-249: a first bad `STR` is RETRY (nothing raised,
    /// nothing posted, nothing more sent), the next COMM_ERR, and a reply
    /// clears it.
    #[test]
    fn a_missed_str_is_retry_then_comm_error() {
        let (line, mut axis, state) = strings_rig("sc800Retry");
        let user = AsynUser::default();
        state.store(QUIET, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(sent(&line), ["STR1/1"]);
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        state.store(0, Ordering::SeqCst);
        let s = axis.poll(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        state.store(QUIET, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }

    #[test]
    fn str_parses_as_c_sscanf() {
        assert_eq!(
            parse_str("C\tSTR1\t1\t1\t0\t0\t1\t0\t0\t2"),
            Some([1, 0, 0, 1, 0, 0, 2])
        );
        assert_eq!(parse_str("C\tSTR1\t2\t1\t0\t0\t1\t0\t0\t2"), None);
        assert_eq!(parse_str("C\tSTR1\t1\t1\t0\t0\t1\t0\t0"), None);
        assert_eq!(parse_str("C\tSTR1\t1\tx\t0\t0\t1\t0\t0\t2"), None);
    }
}
