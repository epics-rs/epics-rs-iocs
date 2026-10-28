//! Newport MM3000 motor controller driver (serial/GPIB ASCII).
//!
//! Ported from `motorNewport/newportApp/src/drvMM3000.cc` + `devMM3000.cc`
//! (a model-1 dev/drv pair). Commands are `{axis}CC{value}` with a 1-based
//! *unpadded* axis prefix and integer values; record transactions join
//! commands with `;` into one write (a move is `VA;[VB;]AC;PA`). Commands go out
//! bare and the port EOS `"\r"` both ways comes from the startup script, as in
//! C's MM3000.cmd:17-20 (the C driver sets no EOS).
//! Replies are not trimmed: the `MS` status reply is a raw binary byte.
//!
//! ## Units
//!
//! The MM3000 wire is step-native: commands and `TP` readbacks are integer
//! motor steps (encoder counts on DC axes), the record steps that cross the
//! motor boundary, so any MRES works as in C.
//!
//! ## Ported upstream quirks (bug-for-bug)
//!
//! - `SET_HIGH_LIMIT` and `SET_LOW_LIMIT` both emit the `SL` command
//!   (`devMM3000.cc` sends `%dSL%d` for both cases).
//! - Encoder detection always concludes "present": C `motor_init` indexes
//!   `cntrl->type[total_axis]` (one past the last axis) for its DC shortcut,
//!   and the `TPE` probe's `"E01"` reply is eaten by `recv_mess`'s
//!   error-retry (`com[0] == 'E'` → re-read, which times out and returns an
//!   empty string), so the `strcmp(buff, "E01")` check never matches. This
//!   port still sends `TPE` per axis (wire parity, including the C re-read
//!   timeout on encoderless axes) and sets `encoder_present = true`.
//! - `SET_ENC_RATIO` (`{axis}ER{p}:{q}`, stepper only) has no [`Model1Motor`]
//!   counterpart and is not ported.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    Model1Motor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_vprintf;

use crate::util::{atof, is_unsolicited_limit_error, nint};
use motor_common::model1::{MoveStrings, init_poll, init_read};

/// Response buffer size for a single controller reply (C `BUFF_SIZE` 100).
const READ_BUF: usize = 256;

/// C `MOTOR_STATUS` bit layout of the binary `MS` reply byte (LSB first:
/// in-motion, NOT-power (0 = ON), direction (1 = plus), plus travel limit,
/// minus travel limit, home limit switch).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MsBits {
    in_motion: bool,
    powered: bool,
    direction: bool,
    plus_tl: bool,
    minus_tl: bool,
    home_ls: bool,
}

impl MsBits {
    fn from_byte(mstat: u8) -> Self {
        Self {
            in_motion: mstat & 0x01 != 0,
            powered: mstat & 0x02 == 0,
            direction: mstat & 0x04 != 0,
            plus_tl: mstat & 0x08 != 0,
            minus_tl: mstat & 0x10 != 0,
            home_ls: mstat & 0x20 != 0,
        }
    }
}

fn mm_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

/// Axis type from the `RC` (read configuration) reply (C `MM_motor_type`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisType {
    /// `stepper1.5M`.
    Stepper,
    /// `dc`.
    Dc,
    /// `unused` — C still counts it in `total_axis`.
    Unused,
}

/// Parse the `RC` reply, e.g. `"1=stepper1.5M 2=dc 3=unused"` (C
/// `motor_init` strtok loop): each `=`-separated segment's first
/// space-delimited word is an axis type; `unused` ends the scan but is still
/// counted. An unrecognized word is errlogged as C does and counted as
/// [`AxisType::Unused`] (C leaves the slot uninitialized and keeps going —
/// the deterministic port choice is the inert type).
fn parse_rc_types(reply: &str) -> Vec<AxisType> {
    let mut types = Vec::new();
    for segment in reply.split('=').skip(1) {
        let word = segment.split_whitespace().next().unwrap_or("");
        if word.starts_with("unused") {
            types.push(AxisType::Unused);
            break;
        } else if word.starts_with("stepper1.5M") {
            types.push(AxisType::Stepper);
        } else if word.starts_with("dc") {
            types.push(AxisType::Dc);
        } else {
            errlog_vprintf(format_args!(
                "drvMM3000:motor_init() - invalid RC response = {word}\n"
            ));
            types.push(AxisType::Unused);
        }
    }
    types
}

/// Controller communication state (C `cntrl->status`): one failed status
/// read is retried silently; a second consecutive one is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Shared controller endpoint: owns the serial handle and the cross-axis
/// communication state. The caller holds the `Arc<Mutex<..>>` lock.
pub struct Mm3000Controller {
    handle: SyncIOHandle,
    ident: String,
    axis_types: Vec<AxisType>,
    comm_state: CommState,
}

impl Mm3000Controller {
    /// Connect and identify an MM3000 (C `motor_init` per-card block): probe
    /// with `VE` (existence), stop all motors (`ST`), read the ident (`VE`),
    /// then read the axis configuration (`RC`). `None` is C dropping a card
    /// that never answers `VE`, without a word; past that C ignores every
    /// status and a failed read is an empty reply. Performs blocking serial
    /// I/O.
    pub fn new(handle: SyncIOHandle) -> Option<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            axis_types: Vec::new(),
            comm_state: CommState::Normal,
        };
        if ctrl.write_read("VE").unwrap_or_default().is_empty() {
            return None;
        }
        let _ = ctrl.write("ST"); // stop all motors; the ST command has no reply
        ctrl.ident = ctrl.write_read("VE").unwrap_or_default();
        ctrl.axis_types = parse_rc_types(&ctrl.write_read("RC").unwrap_or_default());
        Some(ctrl)
    }

    /// Identity string from `VE`.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Axis types from `RC`; the length is the axis count (C `total_axis`,
    /// which includes a trailing `unused` slot).
    pub fn axis_types(&self) -> &[AxisType] {
        &self.axis_types
    }

    /// Write a command (C `send_mess`); the port output EOS terminates it.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        self.write_bytes(cmd.as_bytes())
    }

    /// [`write`](Self::write) for a record string's raw bytes.
    fn write_bytes(&self, cmd: &[u8]) -> AsynResult<()> {
        self.handle.write_octet(0, cmd)?;
        Ok(())
    }

    /// Write a command and read the reply (C `send_mess` + `recv_mess`).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        // write_read flushes first; C reads with no flush past motor_init
        // (drvMM3000.cc:492), so one late reply shifts every later answer: a
        // C defect, not copied.
        let (raw, status) = self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts();
        self.recv_mess(String::from_utf8_lossy(&raw).into_owned(), status)
    }

    /// C `recv_mess` from its first read. Each pass is one C call, which
    /// tests the bytes before the status (drvMM3000.cc:492-510): an
    /// unsolicited hard-travel-limit error (`E35`..`E42`, an unconfirmed
    /// MM3000 firmware bug), even one a timeout cut, is flushed with one
    /// extra read whose status is kept, and any other reply starting with
    /// `E` is reported through C's `errPrintf` and read again, where C
    /// recurses. The retry normally times out, which surfaces here as the
    /// read error.
    fn recv_mess(&self, mut reply: String, mut status: AsynResult<()>) -> AsynResult<String> {
        loop {
            if is_unsolicited_limit_error(&reply) {
                (reply, status) = self.read_once();
            }
            status?;
            if !reply.starts_with('E') {
                return Ok(reply);
            }
            // errPrintf(-1, ...): no errSym name, then a space for the nonzero
            // status. The file and line are ours.
            errlog_vprintf(format_args!(
                " filename=\"{}\" line number={} {reply}\n",
                file!(),
                line!()
            ));
            (reply, status) = self.read_once();
        }
    }

    /// One raw read. No trimming: the `MS` status reply is a binary byte and
    /// the port's input EOS (`\r`) already framed the message.
    fn read_once(&self) -> (String, AsynResult<()>) {
        let (raw, status) = self.handle.read_octet(0, READ_BUF).into_parts();
        (String::from_utf8_lossy(&raw).into_owned(), status)
    }
}

/// One MM3000 axis sharing a controller. Implements [`Model1Motor`].
pub struct Mm3000Axis {
    controller: Arc<Mutex<Mm3000Controller>>,
    /// 1-based controller axis number, sent unpadded (`%d`).
    axis: usize,
    axis_type: AxisType,
    /// Always `true` — see the module quirk note on C's encoder detection.
    encoder_present: bool,
    /// Soft limits in steps, cached from `set_high_limit`/`set_low_limit`
    /// (the record forwards DHLM/DLLM at init): the MM3000 has no jog
    /// command, so C's `JOG` moves to the record soft limit in the jog
    /// direction (`mr->dhlm / mr->mres`).
    high_limit: Option<f64>,
    low_limit: Option<f64>,
    /// Last polled status, reused on the comm-retry poll path where C leaves
    /// the record's other bits stale.
    last_status: MotorStatus,
    /// What the last failed poll raises: nothing on C's first missed reply
    /// (RETRY), `CNTRL_COMM_ERR` and `RA_PROBLEM` on the next (COMM_ERR).
    failure: PollFailure,
    /// The status a RETRY exit stored after a good `MS`, which the poll
    /// leaves.
    left: Option<MotorStatus>,
    strings: MoveStrings,
    /// The record's SET_* parts of the transaction under way, which the
    /// move's own command follows in one message.
    parts: Vec<String>,
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl Mm3000Axis {
    /// Construct axis `axis` (1-based). Sends the C `motor_init` `TPE`
    /// encoder probe for wire parity and ignores the result (see the module
    /// quirk note — C always ends up with `encoder_present = YES`), then
    /// `set_status`. Performs blocking serial I/O under the controller lock;
    /// on encoderless axes the probe costs one read timeout, as it does in C.
    pub fn new(controller: Arc<Mutex<Mm3000Controller>>, axis: usize) -> Self {
        let axis_type = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let _ = ctrl.write_read(&format!("{axis}TPE"));
            ctrl.axis_types()[axis - 1]
        };
        let mut ax = Self {
            controller,
            axis,
            axis_type,
            encoder_present: true,
            high_limit: None,
            low_limit: None,
            // C clears the status, then sets EA_PRESENT and GAIN_SUPPORT
            // for an encoder.
            failure: PollFailure::NONE,
            left: None,
            last_status: MotorStatus {
                gain_support: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
            parts: Vec::new(),
            init_status: None,
        };
        ax.init_status = Some(init_poll(&mut ax, &AsynUser::new(0)));
        ax
    }

    fn lock(&self) -> MutexGuard<'_, Mm3000Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// One move transaction: the record's earlier parts, then PREM and a
    /// `;` ahead of the move's own command (devMM3000.cc:211-222), whose
    /// POST waits for the move to end.
    fn write_move(&mut self, command: &str) -> AsynResult<()> {
        let mut line = String::new();
        for part in self.parts.drain(..) {
            line.push_str(&part);
            line.push(';');
        }
        let mut line = line.into_bytes();
        let prem = self.strings.arm();
        if !prem.is_empty() {
            line.extend_from_slice(&prem);
            line.push(b';');
        }
        line.extend_from_slice(command.as_bytes());
        self.lock().write_bytes(&line)
    }
}

impl Model1Motor for Mm3000Axis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        // What the exit stored, a RETRY's MS bits included, which the
        // first post now carries instead of `left`.
        self.left = None;
        init_read(read, self.last_status.clone(), self.poll_failure())
    }

    /// `update_delay` is 0 (`targs`, drvMM3000.cc:168).
    fn update_delay(&self) -> Duration {
        Duration::ZERO
    }

    /// SET_VELOCITY: `VA`, floored at 100 on a stepper (devMM3000.cc:256).
    fn set_velocity(&mut self, _user: &AsynUser, velocity: f64) -> AsynResult<()> {
        let mut va = nint(velocity);
        if self.axis_type == AxisType::Stepper {
            va = va.max(100);
        }
        self.parts.push(format!("{}VA{va}", self.axis));
        Ok(())
    }

    /// SET_VEL_BASE: `VB` on a stepper only, floored at 100.
    fn set_vel_base(&mut self, _user: &AsynUser, velocity: f64) -> AsynResult<()> {
        if self.axis_type == AxisType::Stepper {
            self.parts
                .push(format!("{}VB{}", self.axis, nint(velocity).max(100)));
        }
        Ok(())
    }

    /// SET_ACCEL: `AC`, floored at 15000 on a stepper and 250 on a DC axis.
    fn set_accel(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let mut ac = nint(acceleration);
        match self.axis_type {
            AxisType::Stepper => ac = ac.max(15000),
            AxisType::Dc => ac = ac.max(250),
            AxisType::Unused => {}
        }
        self.parts.push(format!("{}AC{ac}", self.axis));
        Ok(())
    }

    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // One transaction: VA;[VB;]AC;PA (the MM3000 starts moving on PA;
        // the record's GO is a no-op).
        self.write_move(&format!("{}PA{}", self.axis, nint(position)))
    }

    fn move_relative(&mut self, _user: &AsynUser, distance: f64) -> AsynResult<()> {
        self.write_move(&format!("{}PR{}", self.axis, nint(distance)))
    }

    fn move_velocity(&mut self, _user: &AsynUser, velocity: f64) -> AsynResult<()> {
        // C `JOG`: no jog command on the MM3000 — set the jog speed and move
        // to the record soft limit in the jog direction (record sends
        // SET_ACCEL ahead of JOG in the same transaction).
        let target = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        }
        .ok_or_else(|| {
            mm_err(format!(
                "MM3000 axis {}: jog needs the record soft limits (none set yet)",
                self.axis
            ))
        })?;
        let a = self.axis;
        self.write_move(
            // C truncates the limit: `(int) (mr->dhlm / mr->mres)`.
            &format!("{a}VA{};{a}PA{}", nint(velocity).abs(), target as i32),
        )
    }

    fn home(&mut self, _user: &AsynUser, _forward: bool) -> AsynResult<()> {
        // C sends the same `OR1` for HOME_FOR and HOME_REV.
        self.write_move(&format!("{}OR1", self.axis))
    }

    fn stop(&mut self, _user: &AsynUser) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&format!("{}ST", self.axis))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `LOAD_POS`: only zero is supported — `DH` defines the current
        // position as home (0); any other value is an error.
        if position != 0.0 {
            return Err(mm_err(format!(
                "MM3000 axis {}: only position 0 can be loaded (DH)",
                self.axis
            )));
        }
        let ctrl = self.lock();
        ctrl.write(&format!("{}DH", self.axis))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C ENABLE_TORQUE/DISABL_TORQUE: controller-wide MO/MF, no axis
        // prefix.
        let ctrl = self.lock();
        ctrl.write(if enable { "MO" } else { "MF" })
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // Upstream quirk kept bug-for-bug: devMM3000.cc sends `SL` for
        // SET_HIGH_LIMIT too. The value is still cached as the jog-forward
        // target (C jogs to `mr->dhlm`, a record field).
        self.high_limit = Some(position);
        let ctrl = self.lock();
        ctrl.write(&format!("{}SL{}", self.axis, nint(position)))
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.low_limit = Some(position);
        let ctrl = self.lock();
        ctrl.write(&format!("{}SL{}", self.axis, nint(position)))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        // C SET_[PID]GAIN: the 0..=1 record gain scaled onto the MM3000's
        // 0..=32767 range, then KP/KI/KD + UF (update filter).
        let a = self.axis;
        let cc = match kind {
            PidGainKind::Proportional => "KP",
            PidGainKind::Integral => "KI",
            PidGainKind::Derivative => "KD",
        };
        let v = nint(gain * 32767.0);
        let ctrl = self.lock();
        ctrl.write(&format!("{a}{cc}{v};{a}UF"))
    }

    /// `PRIMITIVE`: INIT and a CR, its own transaction.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let mut line = init.to_vec();
        line.push(b'\r');
        self.lock().write_bytes(&line)
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
        match self.set_status() {
            Ok(polled) => *status = polled,
            Err(e) => {
                if let Some(left) = self.left.take() {
                    *status = left;
                }
                return Err(e);
            }
        }
        Ok(())
    }
}

impl Mm3000Axis {
    /// Port of C `set_status`: `{axis}MS` (binary status byte) then
    /// `{axis}TP` (position), each guarded by the NORMAL/RETRY/COMM_ERR
    /// machine on an empty or failed read. RETRY goes to C's `exit` with
    /// `rtn_state` 0, which posts nothing: the poll fails.
    fn set_status(&mut self) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let a = self.axis;

        let exchange = |ctrl: &mut Mm3000Controller, cmd: &str| -> Option<String> {
            match ctrl.write_read(cmd) {
                Ok(reply) if !reply.is_empty() => {
                    ctrl.comm_state = CommState::Normal;
                    Some(reply)
                }
                _ => {
                    if ctrl.comm_state == CommState::Normal {
                        ctrl.comm_state = CommState::Retry;
                    } else {
                        ctrl.comm_state = CommState::CommErr;
                    }
                    None
                }
            }
        };

        let Some(ms) = exchange(&mut ctrl, &format!("{a}MS")) else {
            return Err(self.missed(ctrl.comm_state, false));
        };
        // C writes the MS bits into the status before asking TP, and its
        // exit stores them even when TP goes unanswered.
        let mstat = MsBits::from_byte(ms.as_bytes()[0]);
        let s = &mut self.last_status;
        s.comms_error = false;
        s.done = !mstat.in_motion;
        s.moving = mstat.in_motion;
        s.direction = mstat.direction;
        s.high_limit = mstat.plus_tl;
        s.low_limit = mstat.minus_tl;
        s.home = mstat.home_ls;
        s.powered = mstat.powered;

        let Some(tp) = exchange(&mut ctrl, &format!("{a}TP")) else {
            return Err(self.missed(ctrl.comm_state, true));
        };
        // drvMM3000.cc:371-378; C ignores the write's status too.
        let ls_active = (mstat.plus_tl && mstat.direction) || (mstat.minus_tl && !mstat.direction);
        if (s.done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.write_bytes(&post);
        }
        drop(ctrl);

        // C: first space-delimited token, `atof`, `(epicsInt32)` cast.
        let position = atof(tp.split_whitespace().next().unwrap_or("")).trunc();
        s.position = position;
        s.encoder_position = if self.encoder_present { position } else { 0.0 };
        s.velocity = 0.0; // C: "Parse motor velocity? NEEDS WORK"
        s.problem = false;
        Ok(s.clone())
    }

    /// C `set_status`'s `exit` after a missed reply, storing
    /// `motor_info->status`: COMM_ERR with both bits raised, RETRY with the
    /// MS bits when `ms_set`.
    fn missed(&mut self, comm_state: CommState, ms_set: bool) -> AsynError {
        self.failure = failure(comm_state);
        if comm_state == CommState::CommErr {
            self.last_status.comms_error = true;
            self.last_status.problem = true;
        } else if ms_set {
            self.left = Some(self.last_status.clone());
        }
        retry(self.axis)
    }
}

/// C `set_status`'s RETRY: the first missed reply.
/// What a failed exchange raises in `comm_state`: nothing on RETRY, both
/// bits on COMM_ERR.
fn failure(comm_state: CommState) -> PollFailure {
    match comm_state {
        CommState::CommErr => PollFailure::PROBLEM_AND_COMMS_ERROR,
        _ => PollFailure::NONE,
    }
}

fn retry(axis: impl std::fmt::Display) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: format!("MM3000 axis {axis}: no status reply"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;
    use motor_common::testing::LineRig;
    use motor_common::testing::Model1Polled;
    use motor_common::testing::RecordWrites;
    use std::sync::atomic::{AtomicU8, Ordering};

    /// Answers `VE` and the commands in `replies`, fails every other read,
    /// and records every command. A reply of `\n`-separated lines is read
    /// one line per read.
    struct IdentOnly {
        base: epics_rs::asyn::port::PortDriverBase,
        replies: &'static [(&'static str, &'static str)],
        /// Reply lines delivered by a read that then times out.
        cuts: &'static [&'static str],
        reply: std::collections::VecDeque<String>,
        sent: Arc<Mutex<Vec<String>>>,
    }

    impl epics_rs::asyn::port::PortDriver for IdentOnly {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            let reply = match cmd.as_str() {
                "VE" => "MM3000 Version 2.4",
                c => self
                    .replies
                    .iter()
                    .find(|(k, _)| *k == c)
                    .map_or("", |(_, v)| v),
            };
            self.reply = reply.split('\n').map(str::to_owned).collect();
            self.sent.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            let reply = self.reply.pop_front().unwrap_or_default();
            if reply.is_empty() {
                return epics_rs::asyn::interpose::OctetReadResult::failed(mm_err(
                    "timeout".into(),
                ));
            }
            buf[..reply.len()].copy_from_slice(reply.as_bytes());
            let read = epics_rs::asyn::interpose::OctetReadResult::ok(
                reply.len(),
                epics_rs::asyn::interpose::EomReason::empty(),
            );
            if self.cuts.contains(&reply.as_str()) {
                return epics_rs::asyn::interpose::OctetReadResult {
                    status: Err(mm_err("reply cut by a timeout".into())),
                    ..read
                };
            }
            read
        }
    }

    /// C keeps a card that answers `VE` whatever its `RC` read returns; a
    /// failed one is an empty buffer and so no axes.
    #[test]
    fn an_unanswered_rc_keeps_the_controller_with_no_axes() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = IdentOnly {
            base: PortDriverBase::new(
                "mm3000IdentOnly",
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
            replies: &[],
            cuts: &[],
            reply: Default::default(),
            sent: Arc::default(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("mm3000IdentOnly").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_millis(100),
            InterfaceType::Octet,
        )
        .unwrap();
        let ctrl = Mm3000Controller::new(io).unwrap();
        assert!(ctrl.axis_types().is_empty());
    }

    /// drvMM3000.cc:492-510: each `recv_mess` tests the bytes before the
    /// status, so a limit error cut by a timeout is read past silently, also
    /// on the read C's recursion makes after an `E` reply; any other cut
    /// reply is still a failure.
    #[test]
    fn a_limit_error_cut_by_a_timeout_is_read_past() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
        use epics_rs::base::runtime::log::{
            errlog_add_listener, errlog_flush, errlog_remove_listener,
        };

        let port = IdentOnly {
            base: PortDriverBase::new(
                "mm3000CutLimit",
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
            replies: &[
                ("1TP", "E38 LIMIT\n12.5"),
                ("2TP", "E12 SYNTAX\nE39 LIMIT\n7.25"),
                ("3TP", "E38 LIMIT"),
                ("4TP", "4.5"),
            ],
            cuts: &["E38 LIMIT", "E39 LIMIT", "4.5"],
            reply: Default::default(),
            sent: Arc::default(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("mm3000CutLimit").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_millis(100),
            InterfaceType::Octet,
        )
        .unwrap();
        let ctrl = Mm3000Controller::new(io).unwrap();
        let printed = Arc::new(Mutex::new(Vec::new()));
        let sink = printed.clone();
        let id = errlog_add_listener(move |m| sink.lock().unwrap().push(m.trim_end().to_owned()));
        assert_eq!(ctrl.write_read("1TP").unwrap(), "12.5");
        assert_eq!(ctrl.write_read("2TP").unwrap(), "7.25");
        // The re-read's status is kept: nothing follows the limit error.
        assert!(ctrl.write_read("3TP").is_err());
        assert!(ctrl.write_read("4TP").is_err());
        errlog_flush();
        errlog_remove_listener(id);
        let printed = printed.lock().unwrap();
        assert_eq!(printed.len(), 1, "{printed:?}");
        assert!(printed[0].ends_with(" E12 SYNTAX"), "{printed:?}");
    }

    /// An MM3000 line answering `replies`: its controller, and the wire log
    /// from after the controller was built.
    fn mm3000_line(
        name: &str,
        replies: &'static [(&'static str, &'static str)],
    ) -> (Mm3000Controller, Arc<Mutex<Vec<String>>>) {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let sent = Arc::new(Mutex::new(Vec::new()));
        let port = IdentOnly {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
            replies,
            cuts: &[],
            reply: Default::default(),
            sent: sent.clone(),
        };
        let (rt, _actor) =
            create_port_runtime(&claim_port(name).unwrap(), port, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_millis(100),
            InterfaceType::Octet,
        )
        .unwrap();
        let ctrl = Mm3000Controller::new(io).unwrap();
        assert_eq!(
            std::mem::take(&mut *sent.lock().unwrap()),
            ["VE", "ST", "VE", "RC"]
        );
        (ctrl, sent)
    }

    /// Axis 1, a DC axis, on a [`LineRig`] whose `MS` byte is `ms`, with the
    /// construction traffic taken.
    fn strings_rig(name: &str) -> (LineRig, Mm3000Axis, Arc<AtomicU8>) {
        let ms = Arc::new(AtomicU8::new(0x04));
        let m = ms.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "VE" => "MM3000".into(),
            "RC" => "1=dc".into(),
            "1MS" => char::from(m.load(Ordering::SeqCst)).to_string(),
            "1TP" => "0 ".into(),
            _ => String::new(),
        });
        let ctrl = Mm3000Controller::new(io).unwrap();
        let axis = Mm3000Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        line.sent();
        (line, axis, ms)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut Mm3000Axis) -> Vec<String> {
        axis.polled(&AsynUser::default()).unwrap();
        line.sent()
            .into_iter()
            .filter(|l| l != "1MS" && l != "1TP")
            .collect()
    }

    /// devMM3000.cc:192-196: INIT and a CR.
    #[test]
    fn init_goes_out_with_a_cr() {
        let (line, mut axis, _) = strings_rig("mm3000Init");
        axis.primitive(&AsynUser::default(), b"MO").unwrap();
        assert_eq!(line.sent(), ["MO\r"]);
    }

    /// devMM3000.cc:211-222: PREM and a `;` go ahead of each move's own
    /// command, for that move only.
    #[test]
    fn a_zero_rate_move_or_home_has_no_ac() {
        let (line, mut axis, _) = strings_rig("mm3000ZeroAc");
        let user = AsynUser::default();
        line.sent();
        axis.record_move_absolute(&user, 15.0, 0.0, 400.0, 0.0)
            .unwrap();
        axis.record_home(&user, 0.0, 400.0, 0.0, true).unwrap();
        assert_eq!(line.sent(), ["1VA400;1PA15", "1VA400;1OR1"]);
    }

    #[test]
    fn jog_truncates_the_soft_limit_as_c() {
        let (line, mut axis, _) = strings_rig("mm3000JogTrunc");
        let user = AsynUser::default();
        axis.set_high_limit(&user, 900.7).unwrap();
        axis.set_low_limit(&user, -900.7).unwrap();
        line.sent();
        axis.record_jog(&user, 0.0, 400.0, 300.0).unwrap();
        axis.record_jog(&user, 0.0, -400.0, 300.0).unwrap();
        assert_eq!(
            line.sent(),
            ["1AC300;1VA400;1PA900", "1AC300;1VA400;1PA-900"]
        );
    }

    #[test]
    fn prem_leads_the_move_command() {
        let (line, mut axis, _) = strings_rig("mm3000Prem");
        let user = AsynUser::default();
        axis.set_high_limit(&user, 900.0).unwrap();
        line.sent();
        axis.set_move_strings(&user, b"MO", None).unwrap();
        axis.record_move_absolute(&user, 15.0, 0.0, 400.0, 300.0)
            .unwrap();
        assert_eq!(line.sent(), ["1VA400;1AC300;MO;1PA15"]);
        axis.record_move_relative(&user, 15.0, 0.0, 400.0, 300.0)
            .unwrap();
        assert_eq!(line.sent(), ["1VA400;1AC300;1PR15"]);
        axis.set_move_strings(&user, b"MO", None).unwrap();
        axis.record_home(&user, 0.0, 400.0, 300.0, true).unwrap();
        assert_eq!(line.sent(), ["1VA400;1AC300;MO;1OR1"]);
        axis.set_move_strings(&user, b"MO", None).unwrap();
        axis.record_jog(&user, 0.0, 400.0, 300.0).unwrap();
        assert_eq!(line.sent(), ["1AC300;MO;1VA400;1PA900"]);
    }

    /// drvMM3000.cc:371-378: POST once the move is done, not while it runs.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, ms) = strings_rig("mm3000Post");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"MF")))
            .unwrap();
        axis.record_move_absolute(&user, 15.0, 0.0, 400.0, 300.0)
            .unwrap();
        line.sent();
        ms.store(0x05, Ordering::SeqCst);
        assert!(poll_extra(&line, &mut axis).is_empty());
        ms.store(0x04, Ordering::SeqCst);
        assert_eq!(poll_extra(&line, &mut axis), ["MF"]);
        assert!(poll_extra(&line, &mut axis).is_empty(), "sent once");
    }

    /// drvMM3000.cc:372: a limit in the move's direction ends it too.
    #[test]
    fn post_follows_a_move_a_limit_stops() {
        let (line, mut axis, ms) = strings_rig("mm3000PostLs");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"MF")))
            .unwrap();
        axis.record_move_absolute(&user, -15.0, 0.0, 400.0, 300.0)
            .unwrap();
        line.sent();
        ms.store(0x11, Ordering::SeqCst);
        assert_eq!(poll_extra(&line, &mut axis), ["MF"]);
    }

    /// drvMM3000.cc motor_init ends each axis with set_status.
    #[test]
    fn axis_init_runs_set_status_after_the_tpe_probe() {
        let (ctrl, sent) = mm3000_line(
            "mm3000InitStatus",
            &[("RC", "1=dc 2=unused"), ("1MS", "D"), ("1TP", "1234 ")],
        );
        let axis = Mm3000Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        assert_eq!(*sent.lock().unwrap(), ["1TPE", "1MS", "1TP"]);
        let s = &axis.last_status;
        assert_eq!(s.position, 1234.0);
        assert!(s.done && s.direction && s.powered && s.has_encoder);
    }

    /// An unanswered TP is a RETRY, but C's exit still stores the MS bits
    /// it decoded before asking.
    #[test]
    fn a_silent_tp_keeps_the_ms_bits() {
        let (ctrl, sent) = mm3000_line("mm3000SilentTp", &[("RC", "1=dc"), ("1MS", "%")]);
        let axis = Mm3000Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        assert_eq!(*sent.lock().unwrap(), ["1TPE", "1MS", "1TP"]);
        let s = &axis.last_status;
        assert!(s.moving && s.direction && s.home && !s.comms_error && !s.problem);
        assert_eq!(s.position, 0.0);
    }

    /// C stores the TP reading through an `(epicsInt32)` cast, which
    /// truncates toward zero.
    #[test]
    fn the_tp_position_truncates_as_c_casts() {
        let (ctrl, _) = mm3000_line(
            "mm3000Trunc",
            &[
                ("RC", "1=dc 2=dc"),
                ("1MS", "@"),
                ("1TP", "12.7"),
                ("2MS", "@"),
                ("2TP", "-12.7"),
            ],
        );
        let ctrl = Arc::new(Mutex::new(ctrl));
        assert_eq!(Mm3000Axis::new(ctrl.clone(), 1).last_status.position, 12.0);
        assert_eq!(Mm3000Axis::new(ctrl, 2).last_status.position, -12.0);
    }

    #[test]
    fn rc_reply_parses_types_and_counts_trailing_unused() {
        assert_eq!(
            parse_rc_types("1=stepper1.5M 2=dc 3=unused"),
            vec![AxisType::Stepper, AxisType::Dc, AxisType::Unused]
        );
        // `unused` ends the scan but is counted (C `total_axis` off-by-one
        // inclusive behavior).
        assert_eq!(parse_rc_types("1=unused 2=dc"), vec![AxisType::Unused]);
        assert_eq!(parse_rc_types("garbage"), vec![]);
    }

    #[test]
    fn preamble_applies_c_clamps_per_axis_type() {
        let user = AsynUser::default();
        let preamble = |name, axis_type, vbas, velo, accel| {
            let (line, mut axis, _) = strings_rig(name);
            axis.axis_type = axis_type;
            axis.record_speeds(&user, vbas, velo, accel).unwrap();
            axis.write_move("PA0").unwrap();
            line.sent().pop().unwrap()
        };
        // Stepper: VB floored at 100, VA at 100, AC at 15000; VB emitted.
        assert_eq!(
            preamble("mm3000Clampsa", AxisType::Stepper, 10.0, 50.0, 200.0),
            "1VA100;1VB100;1AC15000;PA0"
        );
        assert_eq!(
            preamble("mm3000Clampsb", AxisType::Stepper, 500.0, 2000.0, 20000.0),
            "1VA2000;1VB500;1AC20000;PA0"
        );
        // DC: no VB, VA unclamped, AC floored at 250.
        assert_eq!(
            preamble("mm3000Clampsc", AxisType::Dc, 10.0, 50.0, 200.0),
            "1VA50;1AC250;PA0"
        );
        // Unused: no clamps at all.
        assert_eq!(
            preamble("mm3000Clampsd", AxisType::Unused, 10.0, 50.0, 200.0),
            "1VA50;1AC200;PA0"
        );
    }

    #[test]
    fn ms_status_byte_decodes_per_bit() {
        // Idle, powered (NOT-power bit clear), everything else clear.
        assert_eq!(
            MsBits::from_byte(0x00),
            MsBits {
                in_motion: false,
                powered: true,
                direction: false,
                plus_tl: false,
                minus_tl: false,
                home_ls: false,
            }
        );
        // Moving plus-direction into the plus limit, power off.
        assert_eq!(
            MsBits::from_byte(0x01 | 0x02 | 0x04 | 0x08),
            MsBits {
                in_motion: true,
                powered: false,
                direction: true,
                plus_tl: true,
                minus_tl: false,
                home_ls: false,
            }
        );
        // Minus limit + home switch; N/A high bits are ignored.
        assert_eq!(
            MsBits::from_byte(0x10 | 0x20 | 0xC0),
            MsBits {
                in_motion: false,
                powered: true,
                direction: false,
                plus_tl: false,
                minus_tl: true,
                home_ls: true,
            }
        );
    }

    /// drvMM3000.cc:254-269: a first missed `MS` is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and a reply clears it.
    #[test]
    fn a_missed_status_is_retry_then_comm_error() {
        use std::sync::atomic::AtomicBool;
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("mm3000Retry", move |last| match last {
            "VE" => "MM3000".into(),
            "RC" => "1=dc".into(),
            _ if q.load(Ordering::SeqCst) => String::new(),
            "1MS" => "\u{4}".into(),
            "1TP" => "0 ".into(),
            _ => String::new(),
        });
        let ctrl = Mm3000Controller::new(io).unwrap();
        let mut axis = Mm3000Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        let user = AsynUser::default();
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        quiet.store(false, Ordering::SeqCst);
        let s = axis.polled(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }

    /// drvMM3000.cc:685-711: a failed `motor_init` read stores the
    /// EA_PRESENT and GAIN_SUPPORT `motor_init` set: a missed MS raises
    /// nothing (RETRY), a TP missed after a good MS keeps its bits, and the
    /// next miss raises CNTRL_COMM_ERR and RA_PROBLEM (:250-334).
    #[test]
    fn a_failed_init_read_keeps_what_motor_init_set() {
        let (_line, io) = LineRig::new("mm3000InitFail", |last| match last {
            "VE" => "MM3000".into(),
            "RC" => "1=dc 2=dc 3=dc".into(),
            "2MS" => "%".into(),
            _ => String::new(),
        });
        let ctrl = Arc::new(Mutex::new(Mm3000Controller::new(io).unwrap()));
        let init = |axis| Mm3000Axis::new(ctrl.clone(), axis).take_init_status();
        let prepared = MotorStatus {
            gain_support: true,
            has_encoder: true,
            ..MotorStatus::default()
        };
        assert_eq!(init(1), prepared);
        assert_eq!(
            init(2),
            MotorStatus {
                moving: true,
                direction: true,
                home: true,
                powered: true,
                ..prepared.clone()
            }
        );
        assert_eq!(
            init(3),
            MotorStatus {
                problem: true,
                comms_error: true,
                ..prepared
            }
        );
    }

    /// drvMM3000.cc:311-334,380: a TP missed after a good MS is RETRY, and
    /// `exit` stores the MS bits over the COMM_ERR status before it, for
    /// the next post to carry.
    #[test]
    fn a_tp_retry_leaves_the_ms_bits_for_the_next_post() {
        use std::sync::atomic::AtomicBool;
        let (quiet, quiet_tp) = (
            Arc::new(AtomicBool::new(true)),
            Arc::new(AtomicBool::new(false)),
        );
        let (q, qt) = (quiet.clone(), quiet_tp.clone());
        let (_line, io) = LineRig::new("mm3000TpRetry", move |last| match last {
            "VE" => "MM3000".into(),
            "RC" => "1=dc".into(),
            _ if q.load(Ordering::SeqCst) => String::new(),
            "1MS" => "%".into(),
            "1TP" if qt.load(Ordering::SeqCst) => String::new(),
            "1TP" => "0 ".into(),
            _ => String::new(),
        });
        let ctrl = Mm3000Controller::new(io).unwrap();
        let mut axis = Mm3000Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        let user = AsynUser::default();
        // An exit before any reply leaves the axis's status as it was.
        let before = MotorStatus {
            position: 99.0,
            ..MotorStatus::default()
        };
        let mut left = before.clone();
        assert!(axis.poll(&user, &mut left).is_err());
        assert_eq!(left, before);
        assert!(axis.poll(&user, &mut left).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        assert_eq!(left, before);
        quiet.store(false, Ordering::SeqCst);
        quiet_tp.store(true, Ordering::SeqCst);
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(left.moving && !left.done && left.direction && left.home);
        assert!(!left.comms_error && left.problem && !left.msta_moving);
    }
}
