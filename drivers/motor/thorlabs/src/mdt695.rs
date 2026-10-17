//! ThorLabs MDT693/694/695 piezo controller driver (serial ASCII).
//!
//! Ported from `motorThorLabs/thorlabsApp/src/drvMDT695.cc` + `devMDT695.cc`
//! (the model-1 dev/drv pair). The MDT695/693 drives three piezo channels
//! (`X`/`Y`/`Z`) and the MDT694 a single channel, all over one serial line.
//! Each command is prefixed with the axis letter; commands and replies are
//! CR-terminated (`\r`, the port EOS set both ways at connect as C does).
//!
//! ## Open-loop voltage control
//!
//! This is an **open-loop** piezo controller: a "move" sets the channel output
//! voltage (`<axis>V<volts>`) and readback reports the commanded voltage
//! (`<axis>R?`). There is no motion feedback, so the controller is always
//! considered done, with no limit switches, no home, and no encoder — matching
//! the C `set_status`, which reports `RA_DONE` unconditionally. Relative moves,
//! homing, jog, stop, velocity/acceleration and PID are all no-ops (the C
//! `build_trans` emits no message for them).
//!
//! ## Units
//!
//! The controller works in volts (`<axis>R?` returns volts, `<axis>V` takes
//! volts). As C, the record's steps are scaled by the fixed `drive_resolution`
//! (0.1 V) and the readback is `NINT(volts / drive_resolution)` steps.
//!
//! ## Echo
//!
//! The controller can echo commands, which corrupts reply parsing. Startup
//! sends `E` (echo toggle) and retries the device probe until a clean `MDT`
//! identification is seen, matching the C retry loop.

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

/// Port EOS set at connect (C `MDT695_IN_EOS`/`MDT695_OUT_EOS`, drvMDT695.cc:555-558).
pub const INPUT_EOS: &[u8] = b"\r";
pub const OUTPUT_EOS: &[u8] = b"\r";

/// Axis letters by 0-based signal (C `MDT694_axis`).
const AXIS_LETTERS: [char; 3] = ['X', 'Y', 'Z'];

/// C `MOTION_DELAY` (drvMDT695.cc:67), slept on entering RETRY.
const MOTION_DELAY: Duration = Duration::from_millis(100);

/// Map a 0-based axis index to its wire letter, clamped to the last channel.
/// Volts per record step (C `MDT695_RESOLUTION`).
const DRIVE_RESOLUTION: f64 = 0.1;

fn axis_letter(index: usize) -> char {
    AXIS_LETTERS[index.min(AXIS_LETTERS.len() - 1)]
}

fn thorlabs_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

fn trim_line(raw: &[u8]) -> String {
    String::from_utf8_lossy(raw)
        .trim_matches(['\r', '\n', '\0'])
        .to_string()
}

/// Shared controller endpoint owning the serial handle.
pub struct Mdt695Controller {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
    comm_state: CommState,
}

/// Controller communication state (C `cntrl->status`, shared by the axes):
/// one failed readback is retried silently, a second in a row is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

impl Mdt695Controller {
    /// Connect and identify an MDT693/694/695 (C `motor_init`): probe with `D`
    /// (expecting an `MDT` reply, toggling echo off with `E` on failure), pick
    /// the axis count from the model string (`694` → 1, otherwise 3), then read
    /// the `I` identification. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes: 3,
            comm_state: CommState::Normal,
        };

        let mut model = String::new();
        let mut found = false;
        for _ in 0..3 {
            let reply = ctrl.query("D")?;
            if reply.contains("MDT") {
                model = reply;
                found = true;
                break;
            }
            // Reply corrupted (likely echo on): toggle echo and retry.
            let _ = ctrl.query("E");
        }
        if !found {
            return Err(thorlabs_err(
                "MDT695: no 'MDT' identification response to device (D) command",
            ));
        }

        ctrl.num_axes = if model.contains("694") { 1 } else { 3 };

        // Multi-line identification (Model / Version lines).
        let mut ident_parts = Vec::new();
        let mut next = ctrl.query("I");
        while let Ok(line) = next {
            if line.is_empty() {
                break;
            }
            if line.contains("Model") || line.contains("Version") {
                ident_parts.push(line);
            }
            next = ctrl.read_line();
        }
        ctrl.ident = if ident_parts.is_empty() {
            model
        } else {
            ident_parts.join(", ")
        };

        Ok(ctrl)
    }

    /// The controller identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes (1 for MDT694, otherwise 3).
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    /// Write a command with no reply expected (C `cmnd_response == false`).
    fn write_only(&self, cmd: &[u8]) -> AsynResult<()> {
        self.handle.write_octet(0, cmd)?;
        Ok(())
    }

    /// Read one CR-terminated reply line (trimmed).
    fn read_line(&self) -> AsynResult<String> {
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(trim_line(&raw))
    }

    /// Write a query and return its reply line. C send_mess flushes the input
    /// before every write (drvMDT695.cc:393), as `write_read` does.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?;
        Ok(trim_line(&raw))
    }
}

/// One MDT695 piezo channel sharing a controller. Implements [`AsynMotor`].
pub struct Mdt695Axis {
    controller: Arc<Mutex<Mdt695Controller>>,
    letter: char,
    prev_position: f64,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// missed reply (RETRY, drvMDT695.cc:262), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :265). Per axis, as the
    /// controller's state can move on under another axis's poll.
    failure: PollFailure,
    strings: MoveStrings,
}

impl Mdt695Axis {
    /// Construct axis `index` (0-based; wire letter = `X`/`Y`/`Z`).
    pub fn new(controller: Arc<Mutex<Mdt695Controller>>, index: usize) -> Self {
        Self {
            controller,
            letter: axis_letter(index),
            prev_position: 0.0,
            failure: PollFailure::NONE,
            strings: MoveStrings::default(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, Mdt695Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `send_mess` with the axis name: its first '#' becomes the axis letter.
    fn addressed(&self, msg: &[u8]) -> Vec<u8> {
        let mut out = msg.to_vec();
        if let Some(b) = out.iter_mut().find(|b| **b == b'#') {
            *b = self.letter as u8;
        }
        out
    }

    /// devMDT695.cc:203-210: PREM goes ahead of every MOVE_ABS, MOVE_REL,
    /// HOME and JOG, and POST is armed. C sends PREM and the command as one
    /// message whose only '#' substitution lands in PREM when PREM has one,
    /// leaving the voltage command unaddressed; here each is addressed.
    fn send_prem(&mut self) -> AsynResult<()> {
        let prem = self.strings.arm();
        if prem.is_empty() {
            return Ok(());
        }
        let line = self.addressed(&prem);
        self.lock().write_only(&line)
    }
}

impl AsynMotor for Mdt695Axis {
    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.send_prem()?;
        // Set the channel output voltage: "<axis>V<volts>" (C "#V%.1f").
        let ctrl = self.lock();
        ctrl.write_only(format!("{}V{:.1}", self.letter, position * DRIVE_RESOLUTION).as_bytes())
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        _distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C: MOVE_REL emits only PREM (no relative move on this controller).
        self.send_prem()
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C: JOG emits only PREM.
        self.send_prem()
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C: HOME_FOR/HOME_REV emit only PREM.
        self.send_prem()
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C: STOP_AXIS emits no message.
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C: LOAD_POS emits no message.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, _enable: bool) -> AsynResult<()> {
        // Piezo channel has no torque control (C ENABLE/DISABL_TORQUE ERROR).
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // No PID (C SET_PGAIN/IGAIN/DGAIN ERROR).
        Ok(())
    }

    /// `PRIMITIVE`: INIT, addressed (devMDT695.cc:190-194). C ends it with
    /// `MDT695_OUT_EOS` ahead of the port's own, so an empty line follows
    /// INIT on the C wire; that separator has nothing to separate here.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let line = self.addressed(init);
        self.lock().write_only(&line)
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
        let reading = ctrl
            .query(&format!("{}R?", self.letter))
            .ok()
            .and_then(|reply| parse_reading(&reply));
        let Some(volts) = reading else {
            // C drvMDT695.cc:254-257: flush, toggle echo with `E`, read its
            // reply.
            let _ = ctrl.query("E");
            self.failure = if ctrl.comm_state == CommState::Normal {
                ctrl.comm_state = CommState::Retry;
                drop(ctrl);
                sleep(MOTION_DELAY);
                PollFailure::NONE
            } else {
                ctrl.comm_state = CommState::CommErr;
                PollFailure::PROBLEM_AND_COMMS_ERROR
            };
            return Err(thorlabs_err(format!(
                "MDT695 {}R?: no reading",
                self.letter
            )));
        };
        ctrl.comm_state = CommState::Normal;
        // drvMDT695.cc:333-339: RA_DONE is always set, so POST goes at the
        // first good readback after the move, unaddressed.
        if let Some(post) = self.strings.post_due() {
            let _ = ctrl.write_only(&post);
        }
        drop(ctrl);
        self.prev_position = f64::from(nint(volts / DRIVE_RESOLUTION));
        let position = self.prev_position;

        Ok(MotorStatus {
            position,
            encoder_position: 0.0,
            velocity: 0.0,
            // Open-loop: always done, always "positive" direction, no limits.
            done: true,
            moving: false,
            direction: true,
            ..MotorStatus::default()
        })
    }
}

/// C parses the voltage from reply offset 2 (skips the 2-char prefix).
fn parse_reading(reply: &str) -> Option<f64> {
    let trimmed = reply.get(2..)?.trim_start();
    trimmed
        .chars()
        .next()
        .is_some_and(|c| c.is_ascii_digit() || c == '+' || c == '-' || c == '.')
        .then(|| atof(trimmed))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_letters_map_by_index() {
        assert_eq!(axis_letter(0), 'X');
        assert_eq!(axis_letter(1), 'Y');
        assert_eq!(axis_letter(2), 'Z');
        // Out-of-range clamps to the last channel.
        assert_eq!(axis_letter(5), 'Z');
    }

    #[test]
    fn reading_parses_after_the_two_char_prefix() {
        assert_eq!(parse_reading("X 12.5"), Some(12.5));
        assert_eq!(parse_reading("XR?"), None);
        assert_eq!(parse_reading("X"), None);
    }

    use motor_common::testing::LineRig;

    fn rig(name: &str) -> (LineRig, Mdt695Axis) {
        let (line, io) = LineRig::new(name, |last| match last {
            "D" => "MDT695".into(),
            "YR?" => "Y 1.5".into(),
            _ => String::new(),
        });
        let ctrl = Mdt695Controller::new(io).unwrap();
        let axis = Mdt695Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        line.sent();
        (line, axis)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// drvMDT695.cc:250-283: a first missed `R?` is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and a reading clears it.
    #[test]
    fn a_missed_reading_is_retry_then_comm_error() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (line, io) = LineRig::new("mdt695Retry", move |last| match last {
            "D" => "MDT695".into(),
            "YR?" if !q.load(Ordering::SeqCst) => "Y 1.5".into(),
            _ => String::new(),
        });
        let ctrl = Mdt695Controller::new(io).unwrap();
        let mut axis = Mdt695Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        line.sent();
        let user = AsynUser::default();
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert_eq!(line.sent(), ["YR?", "E"]);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        quiet.store(false, Ordering::SeqCst);
        assert_eq!(axis.poll(&user).unwrap().position, 15.0);
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }

    /// devMDT695.cc:190-194: INIT's first '#' carries the axis letter.
    #[test]
    fn init_goes_out_addressed() {
        let (line, mut axis) = rig("mdt695Init");
        axis.primitive(&AsynUser::default(), b"#V0.0").unwrap();
        assert_eq!(line.sent(), ["YV0.0"]);
    }

    /// devMDT695.cc:203-210 with drvMDT695.cc:333-339: PREM ahead of the
    /// move, both addressed, and POST once at the next poll, unaddressed.
    #[test]
    fn prem_leads_the_move_and_post_follows_it() {
        let (line, mut axis) = rig("mdt695Prem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"#L", Some(post(b"#H")))
            .unwrap();
        axis.move_absolute(&user, 10.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["YL", "YV1.0"]);
        axis.poll(&user).unwrap();
        assert_eq!(line.sent(), ["YR?", "#H"]);
        axis.poll(&user).unwrap();
        assert_eq!(line.sent(), ["YR?"]);
    }

    /// MOVE_REL builds no command but still carries PREM.
    #[test]
    fn a_relative_move_sends_only_prem() {
        let (line, mut axis) = rig("mdt695Rel");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"P", None).unwrap();
        axis.move_relative(&user, 1.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["P"]);
        axis.move_relative(&user, 1.0, 0.0, 0.0, 0.0).unwrap();
        assert!(line.sent().is_empty());
    }
}
