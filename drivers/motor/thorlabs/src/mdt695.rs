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
    AsynMotor, Model1Motor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::device_support::{BuildTrans, LimitBuild};

use epics_libcom_rs::cfmt::CFloat;
use epics_libcom_rs::runtime::stdlib::strtod;

use motor_common::model1::{MoveStrings, init_poll, init_read};
use motor_common::util::nint;

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
            // drvMDT695.cc:566-569: a failed read is no match, retried.
            let reply = ctrl.query("D").unwrap_or_default();
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

        // drvMDT695.cc:590-606: read until a read fails; a Model line
        // replaces the ident, a Version line appends to it.
        let mut next = ctrl.query("I");
        while let Ok(line) = next {
            if line.contains("Model") {
                ctrl.ident = line;
            } else if line.contains("Version") {
                ctrl.ident.push_str(", ");
                ctrl.ident.push_str(&line);
            }
            next = ctrl.read_line();
        }

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
        let (raw, status) = self.handle.read_octet(0, READ_BUF).into_parts();
        status?;
        Ok(trim_line(&raw))
    }

    /// Write a query and return its reply line. C send_mess flushes the input
    /// before every write (drvMDT695.cc:393), as `write_read` does.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        let (raw, status) = self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts();
        status?;
        Ok(trim_line(&raw))
    }
}

/// devMDT695's `build_trans` as the record sees it.
pub struct Mdt695Build;

impl BuildTrans for Mdt695Build {
    /// devMDT695.cc:273-276: SET_HIGH_LIMIT and SET_LOW_LIMIT are ERROR.
    fn limit(&self, _high: bool, _raw: f64, _mres: f64) -> LimitBuild {
        LimitBuild::Refused(None)
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
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl Mdt695Axis {
    /// Construct axis `index` (0-based; wire letter = `X`/`Y`/`Z`) and read
    /// its status, as `motor_init`'s `set_status` does (drvMDT695.cc:631).
    pub fn new(controller: Arc<Mutex<Mdt695Controller>>, index: usize) -> Self {
        let mut me = Self {
            controller,
            letter: axis_letter(index),
            prev_position: 0.0,
            failure: PollFailure::NONE,
            strings: MoveStrings::default(),
            init_status: None,
        };
        me.init_status = Some(init_poll(&mut me, &AsynUser::new(0)));
        me
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

    /// devMDT695.cc:190-210: INIT, or the PREM of a MOVE_ABS, MOVE_REL,
    /// HOME or JOG, closed by `MDT695_OUT_EOS` ahead of `command` in one
    /// message, so INIT and a PREM with no command leave an empty line on
    /// the wire. C's send_mess substitutes only the message's first '#',
    /// which leaves the voltage command after an addressed PREM unaddressed;
    /// here each part is addressed.
    fn send_message(&self, lead: &[u8], command: &[u8]) -> AsynResult<()> {
        let mut message = Vec::new();
        if !lead.is_empty() {
            message = self.addressed(lead);
            message.extend_from_slice(OUTPUT_EOS);
        }
        message.extend_from_slice(command);
        if message.is_empty() {
            return Ok(());
        }
        self.lock().write_only(&message)
    }
}

impl Model1Motor for Mdt695Axis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, MotorStatus::default(), self.poll_failure())
    }
}

impl AsynMotor for Mdt695Axis {
    // devMDT695.cc:129 forces NTM to NO.
    fn disables_ntm(&self) -> bool {
        true
    }

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
        let prem = self.strings.arm();
        // Set the channel output voltage: "<axis>V<volts>" (C "#V%.1f").
        let command = format!("{}V{:.1}", self.letter, CFloat(position * DRIVE_RESOLUTION));
        self.send_message(&prem, command.as_bytes())
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
        let prem = self.strings.arm();
        self.send_message(&prem, b"")
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C: JOG emits only PREM.
        let prem = self.strings.arm();
        self.send_message(&prem, b"")
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
        let prem = self.strings.arm();
        self.send_message(&prem, b"")
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

    /// `PRIMITIVE`: INIT (devMDT695.cc:190-194).
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        self.send_message(init, b"")
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

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
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

        *status = MotorStatus {
            position,
            encoder_position: 0.0,
            velocity: 0.0,
            // Open-loop: always done, always "positive" direction, no limits.
            done: true,
            moving: false,
            direction: true,
            ..MotorStatus::default()
        };
        Ok(())
    }
}

/// The voltage `strtod` reads from reply offset 2; a reply it converts
/// nothing of is a failed read (drvMDT695.cc:240-241).
fn parse_reading(reply: &str) -> Option<f64> {
    let parsed = strtod(reply.as_bytes().get(2..)?);
    (parsed.len != 0).then_some(parsed.value)
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

    /// drvMDT695.cc:240-241: `strtod` converts no bare sign or point.
    #[test]
    fn a_reading_strtod_converts_nothing_of_is_none() {
        for reply in ["X +", "X -", "X .", "X -.e1"] {
            assert_eq!(parse_reading(reply), None, "{reply:?}");
        }
        assert_eq!(parse_reading("X  -.5e1"), Some(-5.0));
    }

    use motor_common::testing::LineRig;
    use motor_common::testing::Polled;

    fn rig(name: &str) -> (LineRig, Mdt695Axis) {
        let (line, io) = LineRig::queued(name, |last| match last {
            "D" => vec!["MDT695".into()],
            "YR?" => vec!["Y 1.5".into()],
            _ => Vec::new(),
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

    /// drvMDT695.cc:563-578: a D that gets no reply is retried after `E`,
    /// not the end of the probe.
    #[test]
    fn a_silent_device_probe_is_retried() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let answered = AtomicBool::new(false);
        let (line, io) = LineRig::queued("mdt695Probe", move |last| match last {
            "D" if answered.swap(true, Ordering::SeqCst) => vec!["MDT694".into()],
            _ => Vec::new(),
        });
        let ctrl = Mdt695Controller::new(io).unwrap();
        assert_eq!(ctrl.num_axes(), 1);
        assert_eq!(line.sent(), ["D", "E", "D", "I"]);
    }

    /// drvMDT695.cc:590-606: an empty line does not end the I reply, and
    /// the ident is the Model line with each Version line after it.
    #[test]
    fn the_ident_is_read_to_the_end_of_the_i_reply() {
        let (_line, io) = LineRig::queued("mdt695Ident", |last| match last {
            "D" => vec!["MDT694".into()],
            "I" => ["", "Model MDT694B", "Firmware", "Version 1.08"]
                .map(String::from)
                .to_vec(),
            _ => Vec::new(),
        });
        let ctrl = Mdt695Controller::new(io).unwrap();
        assert_eq!(ctrl.ident(), "Model MDT694B, Version 1.08");
    }

    /// drvMDT695.cc:250-283: a first missed `R?` is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and a reading clears it.
    #[test]
    fn a_missed_reading_is_retry_then_comm_error() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (line, io) = LineRig::queued("mdt695Retry", move |last| match last {
            "D" => vec!["MDT695".into()],
            "YR?" if !q.load(Ordering::SeqCst) => vec!["Y 1.5".into()],
            _ => Vec::new(),
        });
        let ctrl = Mdt695Controller::new(io).unwrap();
        let mut axis = Mdt695Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        line.sent();
        let user = AsynUser::default();
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert_eq!(line.sent(), ["YR?", "E"]);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        quiet.store(false, Ordering::SeqCst);
        assert_eq!(axis.polled(&user).unwrap().position, 15.0);
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }

    /// devMDT695.cc:224: C's `%.*f` spells a NaN voltage `nan`.
    #[test]
    fn a_nan_voltage_goes_out_as_c_printf_spells_it() {
        let (line, mut axis) = rig("mdt695Nan");
        let user = AsynUser::default();
        axis.move_absolute(&user, f64::NAN, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent().last().map(String::as_str), Some("YVnan"));
    }

    /// devMDT695.cc:190-194: INIT's first '#' carries the axis letter and
    /// `MDT695_OUT_EOS` closes it ahead of the port's own.
    #[test]
    fn init_goes_out_addressed() {
        let (line, mut axis) = rig("mdt695Init");
        axis.primitive(&AsynUser::default(), b"#V0.0").unwrap();
        assert_eq!(line.sent(), ["YV0.0\r"]);
    }

    /// devMDT695.cc:203-210 with drvMDT695.cc:333-339: PREM and the move in
    /// one message, both addressed, and POST once at the next poll,
    /// unaddressed.
    #[test]
    fn prem_leads_the_move_and_post_follows_it() {
        let (line, mut axis) = rig("mdt695Prem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"#L", Some(post(b"#H")))
            .unwrap();
        axis.move_absolute(&user, 10.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["YL\rYV1.0"]);
        axis.polled(&user).unwrap();
        assert_eq!(line.sent(), ["YR?", "#H"]);
        axis.polled(&user).unwrap();
        assert_eq!(line.sent(), ["YR?"]);
    }

    /// devMDT695.cc:273-276 with motorRecord.cc:4112-4121: a refused HLM or
    /// LLM sends nothing and goes back to the dial limit.
    #[test]
    fn limits_are_refused_and_restored() {
        use epics_rs::base::server::record::Record;
        use epics_rs::base::types::EpicsValue;
        use epics_rs::motor::MotorRecord;
        use epics_rs::motor::device_support::CommandPort;
        use epics_rs::motor::flags::MstaFlags;

        let mut rec = MotorRecord::new();
        let (port, mut rx) = CommandPort::capture();
        rec.set_command_port(port.with_build_trans(Arc::new(Mdt695Build)));
        rec.conv.mres = 1.0;
        rec.stat.msta = MstaFlags::DONE;
        rec.limits.dhlm = 100.0;
        rec.limits.hlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.limits.llm = -100.0;
        rec.limits.rhlm = 100.0;
        rec.limits.rllm = -100.0;
        rec.internal.init_invariants_synced = true;
        rec.put_field("HLM", EpicsValue::Double(50.0)).unwrap();
        rec.put_field("LLM", EpicsValue::Double(-50.0)).unwrap();
        assert!(rx.try_recv().is_err());
        assert_eq!(
            (
                rec.limits.hlm,
                rec.limits.dhlm,
                rec.limits.llm,
                rec.limits.dllm
            ),
            (100.0, 100.0, -100.0, -100.0)
        );
    }

    /// MOVE_REL, HOME and JOG build no command but still carry PREM and its
    /// `MDT695_OUT_EOS` (devMDT695.cc:203-210).
    #[test]
    fn a_relative_move_sends_only_prem() {
        let (line, mut axis) = rig("mdt695Rel");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"P", None).unwrap();
        axis.move_relative(&user, 1.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["P\r"]);
        axis.set_move_strings(&user, b"Q", None).unwrap();
        axis.home(&user, 0.0, 0.0, 0.0, true).unwrap();
        axis.set_move_strings(&user, b"R", None).unwrap();
        axis.move_velocity(&user, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["Q\r", "R\r"]);
        axis.move_relative(&user, 1.0, 0.0, 0.0, 0.0).unwrap();
        assert!(line.sent().is_empty());
    }
}
