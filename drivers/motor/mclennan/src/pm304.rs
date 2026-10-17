//! Mclennan PM304 / PM600 stepper motor controller driver (serial ASCII).
//!
//! Ported from `motorMclennan/mclennanApp/src/drvPM304.cc` + `devPM304.cc` (the
//! model-1 dev/drv pair). One serial line drives a multi-axis controller; each
//! command is prefixed with the 1-based axis number and terminated by CR (the
//! driver owns output framing; the startup script sets only the input EOS). The
//! controller answers **every** command, so each command is written and its
//! reply consumed to keep the stream synchronized.
//!
//! ## Two models
//!
//! `1ID` identifies the controller at startup. A reply containing `PM304`
//! selects [`Model::Pm304`]; anything else is [`Model::Pm600`]. The two differ
//! in wire formatting:
//!
//! - **Status** (`<axis>OS`): PM304 replies with an 8-character `0`/`1` string;
//!   PM600 prefixes it with `01:` and reorders the bits.
//! - **Position** (`<axis>OA` / `<axis>OC`): PM304 replies `AP=<n>`, PM600
//!   replies `01:<n>`; both are parsed by skipping the first three characters.
//! - **Echo**: the PM600 echoes the command followed by a lone CR before its
//!   response, so on that model everything up to and including the first CR is
//!   stripped from each reply.
//! - **Home** (`IX`/`IX-1` on PM304, `HD`/`HD-1` on PM600) and **jog** (`SV`+
//!   `CV1`/`CV-1` on PM304, signed `CV` on PM600) use different commands.
//!
//! ## Units
//!
//! The controller works natively in motor steps / encoder counts (`OA`/`OC`
//! return counts, `MA` takes counts) with no resolution scaling, so the asyn-rs
//! motor boundary is steps: positions pass through with `NINT` rounding, the
//! record's `MRES` is 1, and its `EGU` is steps. Velocity (`SV`, steps/s) and
//! acceleration (`SA`/`SD`) cross the boundary directly as controller values.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `OS`/`OA`/`OC` reads run
//!   inside [`poll`](AsynMotor::poll).
//! - `use_encoder` follows C: always true on the PM304; on the PM600 it is
//!   false only when the per-axis `ID` reply reports `Open loop stepper mode`.
//! - The C `set_status` velocity readback is marked `NEEDS WORK` and always
//!   reports 0; this port reports 0 likewise.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, PostMessage};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::MoveStrings;
use motor_common::util::{atoi, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 200;

/// Command terminator; the driver owns output framing (PM304/PM600 accept CR).
const TERMINATOR: &[u8] = b"\r";

/// Controller family, selected from the `1ID` reply.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// PM304: bare 8-character status string, `AP=<n>` position, no echo.
    Pm304,
    /// PM600: `01:`-prefixed status/position and a CR-terminated command echo.
    Pm600,
}

fn pm304_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle, the model, and per-axis
/// encoder flags.
pub struct Pm304Controller {
    handle: SyncIOHandle,
    ident: String,
    model: Model,
    n_axes: usize,
    use_encoder: Vec<bool>,
}

impl Pm304Controller {
    /// Connect and identify a Mclennan controller (C `motor_init`): find it
    /// by a `1OA` reply, stop axis 1, read `1ID` to select the model, then read each axis `ID` to decide
    /// whether it reports position from the encoder (`OA`) or the command
    /// counter (`OC`). Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle, n_axes: usize) -> AsynResult<Self> {
        let n_axes = n_axes.max(1);
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            model: Model::Pm304,
            n_axes,
            use_encoder: Vec::with_capacity(n_axes),
        };

        // drvPM304.cc:668-675: the card is there once `1OA` gets a reply,
        // tried three times; every later reply's status is ignored.
        if !(0..3).any(|_| ctrl.write_read_raw("1OA").is_ok_and(|r| !r.is_empty())) {
            return Err(pm304_err("PM304: no response to 1OA position query"));
        }

        // Stop axis 1 for safety (C sends "1ST;" before identifying).
        let _ = ctrl.write_read_raw("1ST");

        // Model detection is robust to the PM600 echo: the reply is
        // "1ID\r<ident>" on the PM600 and "<ident>" on the PM304, and both
        // contain "PM304" only for the PM304.
        let ident = ctrl.write_read_raw("1ID").unwrap_or_default();
        ctrl.model = if ident.contains("PM304") {
            Model::Pm304
        } else {
            Model::Pm600
        };
        ctrl.ident = ctrl.strip_echo(&ident).trim().to_string();

        for axis in 1..=n_axes {
            let id = ctrl
                .write_read_raw(&format!("{axis}ID"))
                .unwrap_or_default();
            // C: PM304 always uses the encoder; PM600 uses the command counter
            // only when the axis reports "Open loop stepper mode".
            let use_encoder = match ctrl.model {
                Model::Pm304 => true,
                Model::Pm600 => !id.contains("Open loop stepper mode"),
            };
            ctrl.use_encoder.push(use_encoder);
        }

        Ok(ctrl)
    }

    /// The identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// The detected controller model.
    pub fn model(&self) -> Model {
        self.model
    }

    /// Number of axes (as configured; C `n_axes`).
    pub fn num_axes(&self) -> usize {
        self.n_axes
    }

    /// Whether axis `index` (0-based) reports position from its encoder.
    fn use_encoder(&self, index: usize) -> bool {
        self.use_encoder.get(index).copied().unwrap_or(true)
    }

    fn framed(cmd: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd);
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// On the PM600, drop the echoed command (everything up to and including the
    /// first CR); on the PM304 the reply has no echo and is returned unchanged.
    fn strip_echo<'a>(&self, reply: &'a str) -> &'a str {
        if self.model == Model::Pm600 {
            match reply.find('\r') {
                Some(i) => &reply[i + 1..],
                None => reply,
            }
        } else {
            reply
        }
    }

    /// Write a command and return its raw reply (input EOS already stripped by
    /// the port), without removing the PM600 echo — used during identification
    /// before the model is known.
    fn write_read_raw(&self, cmd: &str) -> AsynResult<String> {
        // C send_mess/send_recv_mess: pasynOctetSyncIO->writeRead, which
        // flushes first so a stray line cannot shift every later reply.
        let raw = self
            .handle
            .write_read(0, &Self::framed(cmd.as_bytes()), READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_end_matches(['\r', '\n', '\0']).to_string())
    }

    /// Write a command and return its reply with the PM600 echo removed. Every
    /// command produces a reply, so this is used for both queries and set
    /// commands (whose reply is discarded) to keep the stream synchronized.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.write_read_raw(cmd)?;
        Ok(self.strip_echo(&raw).trim().to_string())
    }

    /// C `send_mess`: each `;`-separated piece of `msg` is its own command,
    /// its reply read and dropped, and its status too, so every piece goes
    /// out (drvPM304.cc:367-376).
    fn send_mess(&self, msg: &[u8]) {
        for piece in msg.split(|&b| b == b';').filter(|p| !p.is_empty()) {
            let _ = self.handle.write_read(0, &Self::framed(piece), READ_BUF);
        }
    }
}

/// Status bits decoded from an `OS` reply.
struct StatusBits {
    done: bool,
    plus_ls: bool,
    minus_ls: bool,
    problem: bool,
}

/// Decode an `OS` status reply for the given model.
///
/// - PM304: an 8-character `0`/`1` string. Index 3 is the busy flag (`0` =
///   moving); index 0/1 are the minus/plus limit switches; any `1` in indices
///   4..8 signals a problem.
/// - PM600: the same fields prefixed with `01:`, reordered to
///   done/problem/plus-LS/minus-LS at indices 3/4/5/6 of the raw reply.
fn parse_status(model: Model, reply: &str) -> StatusBits {
    let b = reply.as_bytes();
    let at = |i: usize| b.get(i).copied().unwrap_or(b'0');
    match model {
        Model::Pm304 => StatusBits {
            done: at(3) != b'0',
            minus_ls: at(0) == b'1',
            plus_ls: at(1) == b'1',
            problem: at(4) == b'1' || at(5) == b'1' || at(6) == b'1' || at(7) == b'1',
        },
        // C strips the leading "01:" (3 chars), then reads [0..4] of the rest;
        // here that is indices 3..7 of the raw reply.
        Model::Pm600 => StatusBits {
            done: at(3) != b'0',
            problem: at(4) == b'1',
            plus_ls: at(5) == b'1',
            minus_ls: at(6) == b'1',
        },
    }
}

/// Parse a position reply (`AP=<n>` on PM304, `01:<n>` on PM600) by skipping the
/// three-character prefix, matching C `atoi(&response[3])`.
fn parse_position(reply: &str) -> i32 {
    match reply.get(3..) {
        Some(rest) => atoi(rest),
        None => atoi(reply),
    }
}

/// One PM304/PM600 axis sharing a controller. Implements [`AsynMotor`].
pub struct Pm304Axis {
    controller: Arc<Mutex<Pm304Controller>>,
    /// 1-based wire axis number.
    axis: u32,
    /// 0-based index into the controller's `use_encoder` table.
    index: usize,
    prev_position: i32,
    /// C's RA_DIRECTION, carried from poll to poll in `motor_info->status`.
    direction: bool,
    strings: MoveStrings,
}

impl Pm304Axis {
    /// Construct axis `index` (0-based; wire axis = `index + 1`).
    pub fn new(controller: Arc<Mutex<Pm304Controller>>, index: usize) -> Self {
        Self {
            controller,
            axis: index as u32 + 1,
            index,
            prev_position: 0,
            direction: false,
            strings: MoveStrings::default(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, Pm304Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// PREM, which devPM304.cc puts just ahead of the motion command; it
    /// goes out piece by piece like the rest of the message.
    fn send_prem(&mut self, ctrl: &Pm304Controller) {
        let prem = self.strings.arm();
        ctrl.send_mess(&prem);
    }

    /// Program the speeds before a move or home (C sends `SV`/`SA`/`SD` as
    /// separate transactions ahead of the motion command). SV goes out
    /// whatever its value, as SET_VELOCITY always does; the record sends
    /// SET_ACCEL only for a positive rate (motorRecord.cc:874, 940).
    fn program_speeds(ctrl: &Pm304Controller, axis: u32, velocity: f64, acceleration: f64) {
        ctrl.send_mess(format!("{axis}SV{}", nint(velocity)).as_bytes());
        if acceleration > 0.0 {
            let accel = nint(acceleration);
            ctrl.send_mess(format!("{axis}SA{accel}").as_bytes());
            ctrl.send_mess(format!("{axis}SD{accel}").as_bytes());
        }
    }
}

impl AsynMotor for Pm304Axis {
    /// C `set_status` ignores `send_recv_mess`'s result (drvPM304.cc).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        Pm304Axis::program_speeds(&ctrl, self.axis, velocity, acceleration);
        self.send_prem(&ctrl);
        ctrl.send_mess(format!("{}MA{}", self.axis, nint(position)).as_bytes());
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
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        Pm304Axis::program_speeds(&ctrl, self.axis, velocity, acceleration);
        self.send_prem(&ctrl);
        ctrl.send_mess(format!("{}MR{}", self.axis, nint(distance)).as_bytes());
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        // The record's jog sends SET_ACCEL whatever the rate
        // (motorRecord.cc:2141), so SA/SD always lead.
        let accel = nint(acceleration);
        ctrl.send_mess(format!("{}SA{accel}", self.axis).as_bytes());
        ctrl.send_mess(format!("{}SD{accel}", self.axis).as_bytes());
        self.send_prem(&ctrl);
        match ctrl.model {
            Model::Pm304 => {
                // PM304: the signed speed as SV, then CV1 only for a
                // positive one (devPM304.cc JOG case).
                let speed = nint(velocity);
                ctrl.send_mess(format!("{}SV{speed}", self.axis).as_bytes());
                let dir = if speed > 0 { 1 } else { -1 };
                ctrl.send_mess(format!("{}CV{dir}", self.axis).as_bytes());
            }
            Model::Pm600 => {
                // PM600: signed continuous velocity in one command.
                ctrl.send_mess(format!("{}CV{}", self.axis, nint(velocity)).as_bytes());
            }
        }
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        Pm304Axis::program_speeds(&ctrl, self.axis, velocity, acceleration);
        self.send_prem(&ctrl);
        let cmd = match (ctrl.model, forward) {
            (Model::Pm304, true) => format!("{}IX", self.axis),
            (Model::Pm304, false) => format!("{}IX-1", self.axis),
            (Model::Pm600, true) => format!("{}HD", self.axis),
            (Model::Pm600, false) => format!("{}HD-1", self.axis),
        };
        ctrl.send_mess(cmd.as_bytes());
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.send_mess(format!("{}ST", self.axis).as_bytes());
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let cmd = if ctrl.use_encoder(self.index) {
            format!("{}AP{}", self.axis, nint(position))
        } else {
            format!("{}CP{}", self.axis, nint(position))
        };
        ctrl.send_mess(cmd.as_bytes());
        Ok(())
    }

    /// `PRIMITIVE`: INIT and a CR (devPM304.cc:194-198), split on `;`.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let mut msg = init.to_vec();
        msg.push(b'\r');
        self.lock().send_mess(&msg);
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
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let use_encoder = ctrl.use_encoder(self.index);
        let model = ctrl.model;

        let status_reply = ctrl.command(&format!("{}OS", self.axis))?;
        let bits = parse_status(model, &status_reply);

        let pos_cmd = if use_encoder {
            format!("{}OA", self.axis)
        } else {
            format!("{}OC", self.axis)
        };
        let pos_reply = ctrl.command(&pos_cmd)?;

        // drvPM304.cc:330-338: POST and a CR once done or on a limit; only
        // the PM304 branch sets ls_active.
        let ls_active = model == Model::Pm304 && (bits.plus_ls || bits.minus_ls);
        if (bits.done || ls_active)
            && let Some(mut post) = self.strings.post_due()
        {
            post.push(b'\r');
            ctrl.send_mess(&post);
        }
        drop(ctrl);

        let position = parse_position(&pos_reply);

        // drvPM304.cc:252-261, 302-311: a PM304 limit sets the direction,
        // a position change sets it after that, and otherwise it keeps its
        // last value.
        if model == Model::Pm304 {
            if bits.plus_ls {
                self.direction = true;
            }
            if bits.minus_ls {
                self.direction = false;
            }
        }
        if position != self.prev_position {
            self.direction = position > self.prev_position;
            self.prev_position = position;
        }

        let status = MotorStatus {
            position: position as f64,
            encoder_position: position as f64,
            velocity: 0.0,
            done: bits.done,
            high_limit: bits.plus_ls,
            low_limit: bits.minus_ls,
            problem: bits.problem,
            direction: self.direction,
            // No GAIN_SUPPORT or EA_PRESENT: C set_status keeps the status
            // word zeroed at init, so the record never sends torque (RS/AB)
            // or gain commands and never offers UEIP.
            ..MotorStatus::default()
        };
        Ok(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;

    #[test]
    fn pm304_status_moving_and_limits() {
        // 8-char string: minus-LS=0, plus-LS=0, [2]=0, busy=0 (moving), 0000.
        let s = parse_status(Model::Pm304, "00000000");
        assert!(!s.done);
        assert!(!s.plus_ls && !s.minus_ls && !s.problem);

        // busy flag (index 3) = 1 -> done; plus-LS (index 1) = 1; a problem bit.
        let s = parse_status(Model::Pm304, "01011000");
        assert!(s.done);
        assert!(s.plus_ls);
        assert!(!s.minus_ls);
        assert!(s.problem); // index 4 == '1'
    }

    #[test]
    fn pm600_status_strips_prefix_and_reorders() {
        // "01:" + done=1, problem=0, plus-LS=1, minus-LS=0, then pad.
        let s = parse_status(Model::Pm600, "01:10100000");
        assert!(s.done);
        assert!(!s.problem);
        assert!(s.plus_ls);
        assert!(!s.minus_ls);
    }

    fn pm304_line(name: &str) -> (LineRig, Pm304Axis) {
        let (line, io) = LineRig::new(name, |last| match last {
            "1ID\r" => "Mclennan PM304".into(),
            _ => "OK".into(),
        });
        let ctrl = Pm304Controller::new(io, 1).unwrap();
        line.sent();
        (line, Pm304Axis::new(Arc::new(Mutex::new(ctrl)), 0))
    }

    /// devPM304.cc builds the record's SET_VELOCITY and SET_ACCEL parts of
    /// a home, and SET_ACCEL of a jog, ahead of the motion command.
    #[test]
    fn home_and_jog_program_the_record_speeds() {
        let (line, mut axis) = pm304_line("pm304_speeds");
        let user = AsynUser::default();
        axis.home(&user, 0.0, 400.0, 800.0, true).unwrap();
        assert_eq!(line.sent(), ["1SV400\r", "1SA800\r", "1SD800\r", "1IX\r"]);
        axis.move_velocity(&user, 0.0, -300.0, 600.0).unwrap();
        assert_eq!(
            line.sent(),
            ["1SA600\r", "1SD600\r", "1SV-300\r", "1CV-1\r"]
        );
    }

    /// devPM304.cc sends SV%ld whatever its value, signed on a jog, and
    /// CV-1 unless the rounded jog speed is positive.
    #[test]
    fn sv_goes_out_signed_and_a_zero_jog_runs_negative() {
        let (line, mut axis) = pm304_line("pm304_sv");
        let user = AsynUser::default();
        axis.move_absolute(&user, 10.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["1SV0\r", "1MA10\r"]);
        axis.move_velocity(&user, 0.0, -300.0, 600.0).unwrap();
        assert_eq!(
            line.sent(),
            ["1SA600\r", "1SD600\r", "1SV-300\r", "1CV-1\r"]
        );
        axis.move_velocity(&user, 0.0, 0.4, 600.0).unwrap();
        assert_eq!(line.sent(), ["1SA600\r", "1SD600\r", "1SV0\r", "1CV-1\r"]);
    }

    /// devPM304.cc: INIT and a CR, PREM just ahead of the motion command,
    /// POST and a CR once done, each `;` piece its own command.
    #[test]
    fn init_prem_and_post_go_out_piece_by_piece() {
        let (line, io) = LineRig::new("pm304_strings", |last| match last {
            "1ID\r" => "Mclennan PM304".into(),
            "1OS\r" => "00010000".into(),
            "1OA\r" => "AP=10".into(),
            _ => "OK".into(),
        });
        let ctrl = Pm304Controller::new(io, 1).unwrap();
        let mut axis = Pm304Axis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        let user = AsynUser::default();
        axis.primitive(&user, b"1RS;1KP5").unwrap();
        assert_eq!(line.sent(), ["1RS\r", "1KP5\r\r"]);
        let post = PostMessage::default();
        post.set(b"1AB");
        axis.set_move_strings(&user, b"1RS;1SC", Some(post))
            .unwrap();
        axis.move_absolute(&user, 50.0, 0.0, 400.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["1SV400\r", "1RS\r", "1SC\r", "1MA50\r"]);
        axis.poll(&user).unwrap();
        axis.poll(&user).unwrap();
        assert_eq!(line.sent(), ["1OS\r", "1OA\r", "1AB\r\r", "1OS\r", "1OA\r"]);
    }

    /// drvPM304.cc:668-690: `1OA` finds the card, retried while silent;
    /// an empty `1ID` does not lose it.
    #[test]
    fn the_card_is_found_by_its_position_reply() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let probes = AtomicUsize::new(0);
        let (line, io) = LineRig::new("pm304Probe", move |last| match last {
            "1OA\r" if probes.fetch_add(1, Ordering::SeqCst) < 2 => String::new(),
            "1OA\r" => "AP=0".into(),
            _ => String::new(),
        });
        let ctrl = Pm304Controller::new(io, 1).unwrap();
        assert_eq!(ctrl.ident(), "");
        assert_eq!(
            line.sent(),
            ["1OA\r", "1OA\r", "1OA\r", "1ST\r", "1ID\r", "1ID\r"]
        );

        let (_line, io) = LineRig::new("pm304Absent", |_| String::new());
        assert!(Pm304Controller::new(io, 1).is_err());
    }

    /// drvPM304.cc:367-376: send_mess ignores each piece's status, so a
    /// refused SV does not keep the move from going out.
    #[test]
    fn a_refused_piece_does_not_stop_the_move() {
        let (line, io) = LineRig::failing(
            "pm304Refused",
            |last| match last {
                "1ID\r" => "Mclennan PM304".into(),
                _ => "OK".into(),
            },
            |sent| sent == "1SV400\r",
        );
        let ctrl = Pm304Controller::new(io, 1).unwrap();
        let mut axis = Pm304Axis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        axis.move_absolute(&AsynUser::default(), 50.0, 0.0, 400.0, 0.0)
            .unwrap();
        assert_eq!(line.sent(), ["1SV400\r", "1MA50\r"]);
    }

    /// drvPM304.cc:252-261, 302-311: the direction follows a position
    /// change, holds while the position does, and only a PM304 limit sets it.
    #[test]
    fn direction_holds_until_the_position_moves() {
        use std::sync::Mutex as StdMutex;
        let replies = Arc::new(StdMutex::new(("00010000", "AP=10")));
        let r = replies.clone();
        let (_line, io) = LineRig::new("pm304Dir", move |last| match last {
            "1ID\r" => "Mclennan PM304".into(),
            "1OS\r" => r.lock().unwrap().0.into(),
            "1OA\r" => r.lock().unwrap().1.into(),
            _ => "OK".into(),
        });
        let ctrl = Pm304Controller::new(io, 1).unwrap();
        let mut axis = Pm304Axis::new(Arc::new(Mutex::new(ctrl)), 0);
        let user = AsynUser::default();
        assert!(axis.poll(&user).unwrap().direction);
        *replies.lock().unwrap() = ("00010000", "AP=5");
        assert!(!axis.poll(&user).unwrap().direction);
        assert!(!axis.poll(&user).unwrap().direction);
        *replies.lock().unwrap() = ("01010000", "AP=5");
        assert!(axis.poll(&user).unwrap().direction);

        let (_line, io) = LineRig::new("pm600Dir", |last| match last {
            "1ID\r" => "1ID\rMclennan PM600".into(),
            "1OS\r" => "1OS\r01:10100000".into(),
            _ => "1OA\r01:0".into(),
        });
        let ctrl = Pm304Controller::new(io, 1).unwrap();
        let mut axis = Pm304Axis::new(Arc::new(Mutex::new(ctrl)), 0);
        let s = axis.poll(&user).unwrap();
        assert!(s.high_limit && !s.direction);
    }

    #[test]
    fn position_skips_three_char_prefix() {
        assert_eq!(parse_position("AP=10234"), 10234);
        assert_eq!(parse_position("01:-512"), -512);
    }
}
