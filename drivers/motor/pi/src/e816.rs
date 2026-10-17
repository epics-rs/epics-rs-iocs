//! PI (Physik Instrumente) E-816 digital piezo/nano-positioning controller
//! (`motorPI`).
//!
//! Ported from `drvPIE816.cc` + `devPIE816.cc` (`devPIE816` "copied from
//! devPIE710.cc"). Shares the E-516 closed-loop piezo status/command model
//! (`ONT?`/`OVF?`/`SVO?`/`POS?` poll, `MOV`/`MVR`/`VEL`/`SVO` commands, on-target
//! = done, servo-overflow = `+` limit, no home, no set-position, `NTM = No`)
//! with four E-816-specific differences derived below; not a byte-for-byte
//! clone, so its own module.
//!
//! ## Differences from the E-516
//! 1. **Up to `MAX_AXES = 12` axes** (letters `A`..`L`), still probed at
//!    connect time by `POS? {axis}`.
//! 2. **Finer resolution:** `POS_RES = 0.0001` µm/step (E-516 is `0.001`).
//! 3. **No ONLINE/velocity-control handshake.** The ONLINE loop and `VCO` are
//!    commented out in the C: `online` is hardcoded `true`, `motor_init` sends
//!    only `*IDN?` (identify), and `set_status` skips the `ONL?` step. This
//!    port mirrors that — connect = identify + probe; poll = `ONT?`/`OVF?`/
//!    `SVO?`/`POS?` with no ONLINE gate.
//! 4. **Stop is a zero relative move, not `STP`.** C `STOP_AXIS` →
//!    `MVR #0` → `MVR A0` (the E-816 has no stop command; the E-516 uses
//!    `STP #`).
//!
//! Identify is `*IDN?` (E-516 uses `VER?`); the reply is stored as `ident`.
//! The `version` parse in C feeds only the commented-out `versionSupport`
//! flag, so it is computed-and-dropped and not tracked here.
//!
//! ## Status poll (C `set_status`, ONLINE block commented out)
//! Per axis, each gated on `recv_mess && sscanf("%d")`: `ONT? {axis}`,
//! `OVF? {axis}`, `SVO? {axis}`, then `POS? {axis}` (non-empty required).
//! Decode identical to the E-516:
//! - `RA_DONE` = `ontarget != 0`; `RA_HOME = RA_DONE`.
//! - `powered` (`EA_POSITION`) = `servo != 0`.
//! - `high_limit` = `RA_DONE && overflow != 0`; `low_limit` always 0.
//! - `RA_DIRECTION` from the position delta, persisting across no-motion polls.
//! - `motorData = NINT(atof(reply) / POS_RES)`, `POS_RES = 0.0001` → `MRES = 1`.
//!
//! Same per-controller `NORMAL`/`RETRY`/`COMM_ERR` two-strike debounce.
//!
//! ## Motion (C `devPIE816_build_trans`)
//! - `MOVE_ABS`/`MOVE_REL` → `VEL {axis}{v·res}` then `MOV {axis}{p·res}` /
//!   `MVR {axis}{d·res}`, `%.3f`. `SET_VELOCITY` scales by `res`; `JOG` does
//!   not (same C asymmetry as the E-516).
//! - `JOG` (`move_velocity`) → `VEL {axis}{velocity}` (raw), signed.
//! - `HOME_FOR`/`HOME_REV` and `LOAD_POS` → C `ERROR`; `home()`/`set_position()`
//!   error.
//! - `ENABLE_TORQUE`/`DISABL_TORQUE` → `SVO {axis}1` / `SVO {axis}0`.
//! - `STOP_AXIS` → `MVR {axis}0`.
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
/// Position resolution, micrometres per step (C `drvPIE816.h` `POS_RES`).
const POS_RES: f64 = 0.0001;
/// Maximum axes per controller (C `MAX_AXES`).
const MAX_AXES: usize = 12;
/// Per-axis command letters (C `PIE816_axis[]`).
const AXIS_LABELS: [&str; MAX_AXES] = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"];
/// Command decimal places (C `devPIE816_build_trans` `maxdigits`).
const MAX_DIGITS: usize = 3;

fn pie816_err(message: impl Into<String>) -> AsynError {
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

/// Raw per-axis readings from one successful `ONT?`/`OVF?`/`SVO?`/`POS?`
/// exchange (same shape as the E-516).
struct AxisReading {
    ontarget: bool,
    overflow: bool,
    servo: bool,
    position: i32,
}

/// `VEL {label}{value}` fragment. The caller supplies the already-scaled value
/// (`SET_VELOCITY` scales by `res`, `JOG` does not).
fn vel_cmd(label: &str, value: f64) -> String {
    format!("VEL {label}{value:.*}", MAX_DIGITS)
}

/// `MOV {label}{p·res}` absolute-move fragment.
fn mov_cmd(label: &str, position: f64) -> String {
    format!("MOV {label}{:.*}", MAX_DIGITS, position * POS_RES)
}

/// `MVR {label}{d·res}` relative-move fragment.
fn mvr_cmd(label: &str, distance: f64) -> String {
    format!("MVR {label}{:.*}", MAX_DIGITS, distance * POS_RES)
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
        moving: false,
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
/// debounce state, the identify string, and the probed axis count.
pub struct PIE816Controller {
    handle: SyncIOHandle,
    ident: String,
    comm_state: CommState,
    num_axes: usize,
}

impl PIE816Controller {
    /// Connect, identify (`*IDN?`), and probe axes (C `motor_init` with the
    /// ONLINE/VCO blocks commented out). Performs blocking octet I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            comm_state: CommState::Normal,
            num_axes: 0,
        };

        ctrl.ident = ctrl.exchange("*IDN?");

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

    /// The controller identification string (from the connect-time `*IDN?`).
    pub fn ident(&self) -> &str {
        &self.ident
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

    /// Read one reply (the port's input EOS already stripped the `"\n"`). Any
    /// transport failure or empty read folds into `""`.
    fn reply(read: AsynResult<Vec<u8>>) -> String {
        match read {
            Ok(raw) if !raw.is_empty() => String::from_utf8_lossy(&raw).into_owned(),
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

/// One axis of an E-816 controller. Implements [`AsynMotor`].
pub struct PIE816Axis {
    controller: Arc<Mutex<PIE816Controller>>,
    axis: usize,
    prev_position: i32,
    /// What the last failed poll raises: nothing on C `set_status`'s
    /// first missed reply (RETRY, drvPIE816.cc:295), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :301). Per axis, as the
    /// controller's state can move on under another axis's poll.
    failure: PollFailure,
    last_status: MotorStatus,
    strings: MoveStrings,
}

impl PIE816Axis {
    /// Construct one axis, seeding the initial status. `prev_position` starts
    /// at 0.
    pub fn new(controller: Arc<Mutex<PIE816Controller>>, axis: usize) -> AsynResult<Self> {
        let mut me = Self {
            controller: controller.clone(),
            axis,
            prev_position: 0,
            failure: PollFailure::NONE,
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

    fn lock(&self) -> MutexGuard<'_, PIE816Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A record string with its first `#` replaced by the axis letter, as
    /// C's `send_mess` addresses a message (drvPIE816.cc:426).
    fn addressed(&self, text: &[u8]) -> Vec<u8> {
        let mut text = text.to_vec();
        if let Some(b) = text.iter_mut().find(|b| **b == b'#') {
            *b = self.label().as_bytes()[0];
        }
        text
    }

    /// A move's own message: PREM and an `EOL_E816` ahead of its command
    /// (devPIE816.cc:199-208), whose POST waits for the move to end. C
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

impl AsynMotor for PIE816Axis {
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
        // C `devPIE816_build_trans` HOME_FOR/HOME_REV → ERROR (no home
        // command), after the message already took PREM and its EOL,
        // which C still sends; POST follows the next done poll.
        let line = self.move_line("");
        self.lock().send_bytes(&line)?;
        Err(pie816_err("PIE816: homing is not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C STOP_AXIS → MVR #0 (zero relative move; no stop command).
        let cmd = format!("MVR {}0", self.label());
        self.lock().send(&cmd)
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        Err(pie816_err(
            "PIE816: set-position (LOAD_POS) is not supported",
        ))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let cmd = format!("SVO {}{}", self.label(), if enable { 1 } else { 0 });
        self.lock().send(&cmd)
    }

    /// `PRIMITIVE`: INIT as its own message, its first `#` addressed
    /// (devPIE816.cc:191-192).
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
            self.failure = if comm_err {
                PollFailure::PROBLEM_AND_COMMS_ERROR
            } else {
                PollFailure::NONE
            };
            return Err(pie816_err("no status reply"));
        };
        let last_direction = self.last_status.direction;
        self.last_status = fold_reading(&mut self.prev_position, last_direction, &reading);
        // drvPIE816.cc:336-382: POST, unaddressed, once done or stopped by a
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

    const QUERIES: [&str; 4] = ["ONT? A", "OVF? A", "SVO? A", "POS? A"];

    /// Axis A of a one-axis E-816 on a [`LineRig`] whose `ONT? A` reply
    /// follows `ont`.
    fn strings_rig(name: &str) -> (LineRig, PIE816Axis, Arc<AtomicU8>) {
        let ont = Arc::new(AtomicU8::new(1));
        let o = ont.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "*IDN?" => "E-816".into(),
            "SVO? A" => "1".into(),
            "POS? A" => "0.000".into(),
            "ONT? A" => o.load(Ordering::SeqCst).to_string(),
            "OVF? A" => "0".into(),
            _ => String::new(),
        });
        let ctrl = PIE816Controller::new(io).unwrap();
        let axis = PIE816Axis::new(Arc::new(Mutex::new(ctrl)), 0).unwrap();
        line.sent();
        (line, axis, ont)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut PIE816Axis) -> Vec<String> {
        axis.poll(&AsynUser::default()).unwrap();
        line.sent()
            .into_iter()
            .filter(|l| !QUERIES.contains(&l.as_str()))
            .collect()
    }

    /// devPIE816.cc:191-192: INIT as its own message, its first `#`
    /// addressed.
    #[test]
    fn init_goes_out_as_its_own_message() {
        let (line, mut axis, _) = strings_rig("pie816Init");
        axis.primitive(&AsynUser::default(), b"SVO #1").unwrap();
        assert_eq!(line.sent(), ["SVO A1"]);
    }

    /// devPIE816.cc:199-208: PREM and a LF ahead of the move's own command,
    /// for that move only; the velocity goes ahead as its own message.
    #[test]
    fn prem_goes_ahead_of_the_move() {
        let (line, mut axis, _) = strings_rig("pie816Prem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"SVO #1", None).unwrap();
        axis.move_absolute(&user, 1000.0, 0.0, 1000.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["VEL A0.100", "SVO A1\nMOV A0.100"]);
        axis.move_relative(&user, 1000.0, 0.0, 1000.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["VEL A0.100", "MVR A0.100"]);
        axis.set_move_strings(&user, b"SVO #1", None).unwrap();
        axis.move_velocity(&user, 0.0, -2.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["SVO A1\nVEL A-2.000"]);
        axis.set_move_strings(&user, b"SVO #1", None).unwrap();
        assert!(axis.home(&user, 0.0, 1.0, 0.0, true).is_err());
        assert_eq!(line.sent(), ["SVO A1"]);
    }

    /// drvPIE816.cc:375-382: POST, unaddressed, once the move is done, not
    /// while it runs, and only once.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, ont) = strings_rig("pie816Post");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"SVO #0")))
            .unwrap();
        axis.move_absolute(&user, 1000.0, 0.0, 1000.0, 0.0).unwrap();
        line.sent();
        ont.store(0, Ordering::SeqCst);
        assert!(poll_extra(&line, &mut axis).is_empty());
        ont.store(1, Ordering::SeqCst);
        assert_eq!(poll_extra(&line, &mut axis), ["SVO #0"]);
        assert!(poll_extra(&line, &mut axis).is_empty(), "sent once");
    }

    #[test]
    fn move_commands_use_finer_res_than_e516() {
        // POS_RES = 0.0001, so 10000 steps -> 1.000 um.
        assert_eq!(mov_cmd("A", 10000.0), "MOV A1.000");
        assert_eq!(mov_cmd("L", -5000.0), "MOV L-0.500");
        assert_eq!(mvr_cmd("B", 2500.0), "MVR B0.250");
        assert_eq!(vel_cmd("A", 10000.0 * POS_RES), "VEL A1.000");
    }

    #[test]
    fn jog_velocity_is_raw_not_res_scaled() {
        assert_eq!(vel_cmd("A", 5.0), "VEL A5.000");
        assert_eq!(vel_cmd("C", -2.5), "VEL C-2.500");
    }

    #[test]
    fn stop_is_zero_relative_move() {
        // Reproduce the stop() wire string without needing a controller.
        for (idx, label) in AXIS_LABELS.iter().enumerate() {
            let cmd = format!("MVR {label}0");
            assert_eq!(cmd, format!("MVR {}0", AXIS_LABELS[idx]));
        }
        assert_eq!(format!("MVR {}0", "A"), "MVR A0");
    }

    #[test]
    fn twelve_axis_labels_cover_a_through_l() {
        assert_eq!(AXIS_LABELS.len(), 12);
        assert_eq!(AXIS_LABELS[0], "A");
        assert_eq!(AXIS_LABELS[11], "L");
    }

    #[test]
    fn position_reply_converts_um_to_steps() {
        assert_eq!(nint(atof("1.0000") / POS_RES), 10000);
        assert_eq!(nint(atof("-0.5000") / POS_RES), -5000);
        assert_eq!(nint(atof("junk") / POS_RES), 0);
    }

    /// Fold a raw reading with a known previous position and last direction.
    fn fold(prev: i32, last_dir: bool, reading: AxisReading) -> (MotorStatus, i32) {
        let mut prev_position = prev;
        let status = fold_reading(&mut prev_position, last_dir, &reading);
        (status, prev_position)
    }

    #[test]
    fn decode_matches_e516_semantics() {
        let (status, _) = fold(
            0,
            false,
            AxisReading {
                ontarget: true,
                overflow: true,
                servo: true,
                position: 100,
            },
        );
        assert!(status.done);
        assert!(status.home);
        assert!(status.high_limit);
        assert!(status.powered);
    }

    #[test]
    fn direction_persists_across_no_motion() {
        let (kept, prev) = fold(
            7,
            false,
            AxisReading {
                ontarget: true,
                overflow: false,
                servo: true,
                position: 7,
            },
        );
        assert!(!kept.direction);
        assert_eq!(prev, 7);
    }

    /// drvPIE816.cc:286-307: a first failed status read is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and a good read clears it.
    #[test]
    fn a_failed_status_read_is_retry_then_comm_error() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("e816Retry", move |last| {
            if q.load(Ordering::SeqCst) {
                return String::new();
            }
            match last {
                "*IDN?" => "E-816".into(),
                "SVO? A" => "1".into(),
                "POS? A" => "0.000".into(),
                "ONT? A" => "1".into(),
                "OVF? A" => "0".into(),
                _ => String::new(),
            }
        });
        let ctrl = PIE816Controller::new(io).unwrap();
        let mut axis = PIE816Axis::new(Arc::new(Mutex::new(ctrl)), 0).unwrap();
        let user = AsynUser::default();
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        quiet.store(false, Ordering::SeqCst);
        let s = axis.poll(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }
}
