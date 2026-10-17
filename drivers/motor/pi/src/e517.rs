//! PI (Physik Instrumente) E-517 digital piezo/nano-positioning controller
//! (`motorPI`).
//!
//! Ported from `drvPIE517.cc` + `devPIE517.cc`, which were themselves "copied
//! from devPIE516.cc" and edited for the E-517. It shares the E-516's closed-
//! loop piezo model (on-target = done, servo-overflow = `+` limit, no home, no
//! set-position, `NTM = No`) with three deliberate E-517 differences derived
//! below; it is **not** a byte-for-byte clone, so it is its own module.
//!
//! ## Differences from the E-516
//! 1. **Axis addressing is by digit, not letter.** C `PIE517_axis[] = {"1 ",
//!    "2 ", "3 "}` — only the *first* character (`'1'`/`'2'`/`'3'`) is used
//!    (`send_mess` does `*pbuff = *name`), so the trailing space is inert. This
//!    port uses the bare digits.
//! 2. **Command operand is space-separated.** The move/velocity templates are
//!    `"MOV # %.3f"` (a space between the `#`/axis and the value) — e.g.
//!    `MOV 1 1.234` — whereas the E-516 has `"MOV #%.3f"` (`MOV A1.234`). The
//!    `SVO`/`STP` templates keep the E-516 shape (`SVO #1` → `SVO 11`,
//!    `STP #` → `STP 1`).
//! 3. **Replies are `=`-delimited.** C `recv_mess` runs `pos = strchr(com,
//!    '='); if (pos) strcpy(com, &pos[1])` on every read — the controller
//!    echoes the query and returns `…=value`, so everything up to and including
//!    the first `'='` is stripped before parsing. `strip_eq` reproduces this.
//!
//! ## Connect handshake (C `motor_init`, mostly commented out)
//! The E-517 `motor_init` has the ONLINE loop, `GET_IDENT`, and `VCO`
//! **commented out**: `online` is hardcoded `true`, and `brdptr->ident` is
//! `strcpy`'d from an *uninitialised* `buff` (a latent C bug — this port stores
//! an empty ident rather than reproduce the read of uninitialised stack). So
//! `new` does only: probe axes 0..3 via `POS? {digit}` (stop at first silence),
//! no online check, no identify, no velocity-control-mode command.
//!
//! ## Status poll (C `set_status`, ONLINE block commented out)
//! Per axis, each gated on `recv_mess && sscanf("%d")` (the E-516's leading
//! `ONL?` step is commented out here): `ONT? {digit}`, `OVF? {digit}`,
//! `SVO? {digit}`, then `POS? {digit}` (non-empty required). Every reply is
//! `=`-stripped first. Decode is identical to the E-516:
//! - `RA_DONE` = `ontarget != 0`; `RA_HOME = RA_DONE`.
//! - `powered` (`EA_POSITION`) = `servo != 0`.
//! - `high_limit` = `RA_DONE && overflow != 0`; `low_limit` always 0.
//! - `RA_DIRECTION` from the position delta, persisting across no-motion polls.
//! - `motorData = NINT(atof(reply) / POS_RES)`, `POS_RES = 0.001` → `MRES = 1`.
//!
//! Same per-controller `NORMAL`/`RETRY`/`COMM_ERR` two-strike debounce.
//!
//! ## Motion (C `devPIE517_build_trans`)
//! - `MOVE_ABS`/`MOVE_REL` → `VEL {axis} {v·res}` then `MOV {axis} {p·res}` /
//!   `MVR {axis} {d·res}`, `%.3f`. `SET_VELOCITY` scales by `res`; `JOG` does
//!   not (same C asymmetry as the E-516).
//! - `JOG` (`move_velocity`) → `VEL {axis} {velocity}` (raw), signed.
//! - `HOME_FOR`/`HOME_REV` and `LOAD_POS` → C `ERROR`; `home()`/`set_position()`
//!   error.
//! - `ENABLE_TORQUE`/`DISABL_TORQUE` → `SVO {axis}1` / `SVO {axis}0`.
//! - `STOP_AXIS` → `STP {axis}`.
//! - gains, limits, `SET_ENC_RATIO`, `GO`, `SET_ACCEL`, `SET_VEL_BASE` → no
//!   wire command.
//!
//! ## Framing & not-modeled deviations
//! Same as the E-516 port: port-owned EOS (`"\n"` both ways, set at Config),
//! the redundant `EOL`+output-EOS trailing empty command on moves collapsed to
//! a single terminator, and `recv_mess(FLUSH)` / `no_motion_count` / `report()`
//! not modeled. See [`crate::e516`] for the full rationale.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, PostMessage};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::MoveStrings;
use motor_common::util::{atof, nint};

use crate::scan_int;

/// Response buffer size for a single reply (C `BUFF_SIZE`).
const READ_BUF: usize = 100;
/// Position resolution, micrometres per step (C `drvPIE517.h` `POS_RES`).
const POS_RES: f64 = 0.001;
/// Maximum axes per controller (C `MAX_AXES`).
const MAX_AXES: usize = 3;
/// Per-axis command digits (C `PIE517_axis[]`, first char only).
const AXIS_LABELS: [&str; MAX_AXES] = ["1", "2", "3"];
/// Command decimal places (C `devPIE517_build_trans` `maxdigits`).
const MAX_DIGITS: usize = 3;

fn pie517_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Controller communication state (C `cntrl->status`), shared by all axes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Strip a reply up to and including the first `'='` (C `recv_mess`: `pos =
/// strchr(com, '='); if (pos) strcpy(com, &pos[1])`). A reply with no `'='` is
/// returned unchanged.
fn strip_eq(reply: &str) -> &str {
    match reply.find('=') {
        Some(i) => &reply[i + 1..],
        None => reply,
    }
}

/// Raw per-axis readings from one successful `ONT?`/`OVF?`/`SVO?`/`POS?`
/// exchange (same shape as the E-516).
struct AxisReading {
    ontarget: bool,
    overflow: bool,
    servo: bool,
    position: i32,
}

/// `VEL {label} {value}` fragment (space-separated operand). The caller
/// supplies the already-scaled value (`SET_VELOCITY` scales by `res`, `JOG`
/// does not).
fn vel_cmd(label: &str, value: f64) -> String {
    format!("VEL {label} {value:.*}", MAX_DIGITS)
}

/// `MOV {label} {p·res}` absolute-move fragment.
fn mov_cmd(label: &str, position: f64) -> String {
    format!("MOV {label} {:.*}", MAX_DIGITS, position * POS_RES)
}

/// `MVR {label} {d·res}` relative-move fragment.
fn mvr_cmd(label: &str, distance: f64) -> String {
    format!("MVR {label} {:.*}", MAX_DIGITS, distance * POS_RES)
}

/// Fold one raw reading into a [`MotorStatus`] (C `set_status` tail); identical
/// decode to the E-516. Updates `prev_position`; `last_direction` persists when
/// the position did not change.
fn fold_reading(
    prev_position: &mut i32,
    last_direction: bool,
    reading: &AxisReading,
) -> MotorStatus {
    let done = reading.ontarget;
    let mut direction = last_direction;
    if reading.position != *prev_position {
        direction = reading.position >= *prev_position;
        *prev_position = reading.position;
    }
    let position = reading.position as f64;
    MotorStatus {
        position,
        encoder_position: position,
        velocity: 0.0,
        done,
        moving: !done,
        high_limit: done && reading.overflow,
        low_limit: false,
        home: done,
        encoder_home: false,
        powered: reading.servo,
        problem: false,
        direction,
        slip_stall: false,
        comms_error: false,
        homed: false,
        gain_support: true,
        has_encoder: true,
        vbas_supported: false,
    }
}

/// Shared controller endpoint: owns the octet handle, the per-controller comm
/// debounce state, and the probed axis count.
pub struct PIE517Controller {
    handle: SyncIOHandle,
    comm_state: CommState,
    num_axes: usize,
}

impl PIE517Controller {
    /// Connect and probe axes (C `motor_init` with the ONLINE/identify/VCO
    /// blocks commented out). Performs blocking octet I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            comm_state: CommState::Normal,
            num_axes: 0,
        };

        let mut num_axes = 0;
        for label in AXIS_LABELS {
            if ctrl.exchange(&format!("POS? {label}")).is_empty() {
                break;
            }
            num_axes += 1;
        }
        ctrl.num_axes = num_axes;

        Ok(ctrl)
    }

    /// Number of axes that responded to the connect-time `POS?` probe.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    /// Write a command, no terminator (the port's output EOS adds `"\n"`).
    fn send(&self, cmd: &str) -> AsynResult<()> {
        self.send_bytes(cmd.as_bytes())
    }

    /// [`send`](Self::send) for raw bytes; an empty message is not sent.
    fn send_bytes(&self, cmd: &[u8]) -> AsynResult<()> {
        if cmd.is_empty() {
            return Ok(());
        }
        self.handle.write_octet(0, cmd)?;
        Ok(())
    }

    /// Write a query and read its reply as one exchange that first flushes
    /// stale input, so a stray line cannot shift every later reply by one
    /// (C recovers the same way through `recv_mess(FLUSH)`).
    fn exchange(&self, cmd: &str) -> String {
        Self::reply(self.handle.write_read(0, cmd.as_bytes(), READ_BUF))
    }

    /// Read one reply, `=`-stripped (C `recv_mess`'s `strchr('=')` fixup). The
    /// port's input EOS already removed the `"\n"`. Any transport failure or
    /// empty read folds into `""`.
    fn reply(read: AsynResult<Vec<u8>>) -> String {
        match read {
            Ok(raw) if !raw.is_empty() => strip_eq(&String::from_utf8_lossy(&raw)).to_owned(),
            _ => String::new(),
        }
    }

    /// C `set_status` I/O for one axis, wrapped in the comm debounce.
    fn poll_axis(&mut self, axis: usize) -> Option<AxisReading> {
        match self.read_axis(axis) {
            Some(reading) => {
                self.comm_state = CommState::Normal;
                Some(reading)
            }
            None => {
                self.comm_state = if self.comm_state == CommState::Normal {
                    CommState::Retry
                } else {
                    CommState::CommErr
                };
                None
            }
        }
    }

    /// The raw `set_status` read chain (no ONLINE step); `None` on the first
    /// failed read/parse.
    fn read_axis(&mut self, axis: usize) -> Option<AxisReading> {
        let label = AXIS_LABELS[axis];

        let ontarget = scan_int(&self.exchange(&format!("ONT? {label}")))?;

        let overflow = scan_int(&self.exchange(&format!("OVF? {label}")))?;

        let servo = scan_int(&self.exchange(&format!("SVO? {label}")))?;

        let pos_reply = self.exchange(&format!("POS? {label}"));
        if pos_reply.is_empty() {
            return None;
        }
        let position = nint(atof(&pos_reply) / POS_RES);

        Some(AxisReading {
            ontarget: ontarget != 0,
            overflow: overflow != 0,
            servo: servo != 0,
            position,
        })
    }
}

/// One axis of an E-517 controller. Implements [`AsynMotor`].
pub struct PIE517Axis {
    controller: Arc<Mutex<PIE517Controller>>,
    axis: usize,
    prev_position: i32,
    last_status: MotorStatus,
    strings: MoveStrings,
}

impl PIE517Axis {
    /// Construct one axis, seeding the initial status (C `motor_init`'s
    /// `set_status` call). `prev_position` starts at 0.
    pub fn new(controller: Arc<Mutex<PIE517Controller>>, axis: usize) -> AsynResult<Self> {
        let mut me = Self {
            controller: controller.clone(),
            axis,
            prev_position: 0,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                has_encoder: true,
                vbas_supported: false,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
        };
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(reading) = ctrl.poll_axis(axis) {
            drop(ctrl);
            me.last_status = fold_reading(&mut me.prev_position, false, &reading);
        }
        Ok(me)
    }

    fn label(&self) -> &'static str {
        AXIS_LABELS[self.axis]
    }

    fn lock(&self) -> MutexGuard<'_, PIE517Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A record string with its first `#` replaced by the axis label, as
    /// C's `send_mess` addresses a message (drvPIE517.cc:423).
    fn addressed(&self, text: &[u8]) -> Vec<u8> {
        let mut text = text.to_vec();
        if let Some(b) = text.iter_mut().find(|b| **b == b'#') {
            *b = self.label().as_bytes()[0];
        }
        text
    }

    /// A move's own message: PREM and an `EOL_E517` ahead of its command
    /// (devPIE517.cc:201-210), whose POST waits for the move to end. C
    /// addresses only the first `#` of the whole message, so a PREM holding
    /// one leaves the command's `#` on the wire; not copied.
    fn move_line(&mut self, command: &str) -> Vec<u8> {
        let prem = self.strings.arm();
        let mut line = self.addressed(&prem);
        if !line.is_empty() && !command.is_empty() {
            line.push(b'\n');
        }
        line.extend_from_slice(command.as_bytes());
        line
    }
}

impl AsynMotor for PIE517Axis {
    /// C `set_status`'s COMM_ERR branch raises `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` (drvPIE517.cc). Its first failure's silent RETRY is not modelled:
    /// a transport error raises both at once.
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
        let label = self.label();
        let line = self.move_line(&mov_cmd(label, position));
        let ctrl = self.lock();
        ctrl.send(&vel_cmd(label, velocity * POS_RES))?;
        ctrl.send_bytes(&line)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let label = self.label();
        let line = self.move_line(&mvr_cmd(label, distance));
        let ctrl = self.lock();
        ctrl.send(&vel_cmd(label, velocity * POS_RES))?;
        ctrl.send_bytes(&line)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // JOG: raw velocity, no `res` scaling, signed; no move command follows.
        let line = self.move_line(&vel_cmd(self.label(), velocity));
        self.lock().send_bytes(&line)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `devPIE517_build_trans` HOME_FOR/HOME_REV → ERROR (no home
        // command), after the message already took PREM and its EOL,
        // which C still sends; POST follows the next done poll.
        let line = self.move_line("");
        self.lock().send_bytes(&line)?;
        Err(pie517_err("PIE517: homing is not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let cmd = format!("STP {}", self.label());
        self.lock().send(&cmd)
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        Err(pie517_err(
            "PIE517: set-position (LOAD_POS) is not supported",
        ))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let cmd = format!("SVO {}{}", self.label(), if enable { 1 } else { 0 });
        self.lock().send(&cmd)
    }

    /// `PRIMITIVE`: INIT as its own message, its first `#` addressed
    /// (devPIE517.cc:193-194).
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let line = self.addressed(init);
        self.lock().send_bytes(&line)
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
        let reading = ctrl.poll_axis(self.axis);
        let comm_err = ctrl.comm_state == CommState::CommErr;

        let Some(reading) = reading else {
            if comm_err {
                self.last_status.comms_error = true;
                self.last_status.problem = true;
            }
            return Ok(self.last_status.clone());
        };
        let last_direction = self.last_status.direction;
        self.last_status = fold_reading(&mut self.prev_position, last_direction, &reading);
        // drvPIE517.cc:333-379: POST, unaddressed, once done or stopped by a
        // limit in the move's direction.
        let s = &self.last_status;
        let ls_active = (s.high_limit && s.direction) || (s.low_limit && !s.direction);
        if (s.done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.send_bytes(&post);
        }
        drop(ctrl);
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use std::sync::atomic::{AtomicU8, Ordering};

    const QUERIES: [&str; 4] = ["ONT? 1", "OVF? 1", "SVO? 1", "POS? 1"];

    /// Axis 1 of a one-axis E-517 on a [`LineRig`] whose `ONT? 1` reply
    /// follows `ont`.
    fn strings_rig(name: &str) -> (LineRig, PIE517Axis, Arc<AtomicU8>) {
        let ont = Arc::new(AtomicU8::new(1));
        let o = ont.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "SVO? 1" => "1=1".into(),
            "POS? 1" => "1=0.000".into(),
            "ONT? 1" => format!("1={}", o.load(Ordering::SeqCst)),
            "OVF? 1" => "1=0".into(),
            _ => String::new(),
        });
        let ctrl = PIE517Controller::new(io).unwrap();
        let axis = PIE517Axis::new(Arc::new(Mutex::new(ctrl)), 0).unwrap();
        line.sent();
        (line, axis, ont)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut PIE517Axis) -> Vec<String> {
        axis.poll(&AsynUser::default()).unwrap();
        line.sent()
            .into_iter()
            .filter(|l| !QUERIES.contains(&l.as_str()))
            .collect()
    }

    /// devPIE517.cc:193-194: INIT as its own message, its first `#`
    /// addressed.
    #[test]
    fn init_goes_out_as_its_own_message() {
        let (line, mut axis, _) = strings_rig("pie517Init");
        axis.primitive(&AsynUser::default(), b"SVO # 1").unwrap();
        assert_eq!(line.sent(), ["SVO 1 1"]);
    }

    /// devPIE517.cc:201-210: PREM and a LF ahead of the move's own command,
    /// for that move only; the velocity goes ahead as its own message.
    #[test]
    fn prem_goes_ahead_of_the_move() {
        let (line, mut axis, _) = strings_rig("pie517Prem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"SVO # 1", None).unwrap();
        axis.move_absolute(&user, 1000.0, 0.0, 1000.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["VEL 1 1.000", "SVO 1 1\nMOV 1 1.000"]);
        axis.move_relative(&user, 1000.0, 0.0, 1000.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["VEL 1 1.000", "MVR 1 1.000"]);
        axis.set_move_strings(&user, b"SVO # 1", None).unwrap();
        axis.move_velocity(&user, 0.0, -2.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["SVO 1 1\nVEL 1 -2.000"]);
        axis.set_move_strings(&user, b"SVO # 1", None).unwrap();
        assert!(axis.home(&user, 0.0, 1.0, 0.0, true).is_err());
        assert_eq!(line.sent(), ["SVO 1 1"]);
    }

    /// drvPIE517.cc:372-379: POST, unaddressed, once the move is done, not
    /// while it runs, and only once.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, ont) = strings_rig("pie517Post");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"SVO # 0")))
            .unwrap();
        axis.move_absolute(&user, 1000.0, 0.0, 1000.0, 0.0).unwrap();
        line.sent();
        ont.store(0, Ordering::SeqCst);
        assert!(poll_extra(&line, &mut axis).is_empty());
        ont.store(1, Ordering::SeqCst);
        assert_eq!(poll_extra(&line, &mut axis), ["SVO # 0"]);
        assert!(poll_extra(&line, &mut axis).is_empty(), "sent once");
    }

    #[test]
    fn move_commands_are_space_separated_and_res_scaled() {
        assert_eq!(mov_cmd("1", 1000.0), "MOV 1 1.000");
        assert_eq!(mov_cmd("2", -500.0), "MOV 2 -0.500");
        assert_eq!(mvr_cmd("3", 250.0), "MVR 3 0.250");
        assert_eq!(vel_cmd("1", 1000.0 * POS_RES), "VEL 1 1.000");
    }

    #[test]
    fn jog_velocity_is_raw_not_res_scaled() {
        assert_eq!(vel_cmd("1", 5.0), "VEL 1 5.000");
        assert_eq!(vel_cmd("2", -2.5), "VEL 2 -2.500");
    }

    #[test]
    fn strip_eq_keeps_text_after_first_equals() {
        assert_eq!(strip_eq("POS 1=1.234"), "1.234");
        assert_eq!(strip_eq("1=0=9"), "0=9"); // only the first '=' is consumed
        assert_eq!(strip_eq("no-equals"), "no-equals");
        assert_eq!(strip_eq("="), "");
    }

    #[test]
    fn position_reply_converts_um_to_steps_after_eq_strip() {
        // A `=`-delimited POS? reply is stripped, then atof/POS_RES converts.
        assert_eq!(nint(atof(strip_eq("POS 1=1.234")) / POS_RES), 1234);
        assert_eq!(nint(atof(strip_eq("POS 2=-0.500")) / POS_RES), -500);
    }

    /// Fold a raw reading with a known previous position and last direction.
    fn fold(prev: i32, last_dir: bool, reading: AxisReading) -> (MotorStatus, i32) {
        let mut prev_position = prev;
        let status = fold_reading(&mut prev_position, last_dir, &reading);
        (status, prev_position)
    }

    #[test]
    fn on_target_and_overflow_decode_like_e516() {
        let (status, _) = fold(
            0,
            false,
            AxisReading {
                ontarget: true,
                overflow: true,
                servo: false,
                position: 100,
            },
        );
        assert!(status.done);
        assert!(status.home);
        assert!(status.high_limit); // overflow while on-target
        assert!(!status.powered); // servo disabled
    }

    #[test]
    fn direction_persists_across_no_motion() {
        let (kept, prev) = fold(
            42,
            true,
            AxisReading {
                ontarget: true,
                overflow: false,
                servo: true,
                position: 42,
            },
        );
        assert!(kept.direction);
        assert_eq!(prev, 42);
    }

    #[test]
    fn scan_int_gates_reads() {
        assert_eq!(scan_int("1"), Some(1));
        assert_eq!(scan_int(""), None);
    }
}
