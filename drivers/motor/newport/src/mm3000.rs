//! Newport MM3000 motor controller driver (serial/GPIB ASCII).
//!
//! Ported from `motorNewport/newportApp/src/drvMM3000.cc` + `devMM3000.cc`
//! (a model-1 dev/drv pair). Commands are `{axis}CC{value}` with a 1-based
//! *unpadded* axis prefix and integer values; record transactions join
//! commands with `;` into one write (a move is `[VB;]VA;AC;PA`). Commands go out
//! bare and the port EOS `"\r"` both ways comes from the startup script, as in
//! C's MM3000.cmd:17-20 (the C driver sets no EOS).
//! Replies are not trimmed: the `MS` status reply is a raw binary byte.
//!
//! ## Units
//!
//! The MM3000 wire is step-native: commands and `TP` readbacks are integer
//! motor steps (encoder counts on DC axes). The asyn-rs motor boundary is
//! dial-frame EGU, so pair this driver with `MRES = 1` records (EGU ≡
//! counts), exactly like the AG-UC driver — see [`crate::agilis`].
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
//! - `SET_ENC_RATIO` (`{axis}ER{p}:{q}`, stepper only) has no [`AsynMotor`]
//!   counterpart and is not ported.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_vprintf;

use crate::util::{atof, is_unsolicited_limit_error, nint};

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

/// Build the velocity/acceleration preamble of a motion transaction (record
/// `SET_VEL_BASE`/`SET_VELOCITY`/`SET_ACCEL` before the move), with the C
/// clamps: steppers floor `VB`/`VA` at 100 and `AC` at 15000, DC axes floor
/// `AC` at 250; `VB` is emitted for steppers only. Values are integer steps
/// (C `NINT`).
fn motion_preamble(
    axis: usize,
    axis_type: AxisType,
    vel_base: f64,
    velocity: f64,
    accel: f64,
) -> String {
    let mut vb = nint(vel_base);
    let mut va = nint(velocity);
    let mut ac = nint(accel);
    let mut out = String::new();
    if axis_type == AxisType::Stepper {
        vb = vb.max(100);
        va = va.max(100);
        ac = ac.max(15000);
        out.push_str(&format!("{axis}VB{vb};"));
    }
    if axis_type == AxisType::Dc {
        ac = ac.max(250);
    }
    out.push_str(&format!("{axis}VA{va};{axis}AC{ac}"));
    out
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
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Write a command and read the reply (C `send_mess` + `recv_mess`).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?;
        self.read_reply(String::from_utf8_lossy(&raw).into_owned())
    }

    /// Apply C `recv_mess`'s two retry rules to the first reply: an unsolicited
    /// hard-travel-limit error (`E35`..`E42`, an unconfirmed MM3000 firmware
    /// bug) is flushed with one extra read, and any other reply starting with
    /// `E` is reported through C's `errPrintf` and re-read (C recurses; the
    /// retry normally times out, which surfaces here as the read error).
    fn read_reply(&self, mut reply: String) -> AsynResult<String> {
        if is_unsolicited_limit_error(&reply) {
            reply = self.read_once()?;
        }
        while reply.starts_with('E') {
            // errPrintf(-1, ...): no errSym name, then a space for the nonzero
            // status. The file and line are ours.
            errlog_vprintf(format_args!(
                " filename=\"{}\" line number={} {reply}\n",
                file!(),
                line!()
            ));
            reply = self.read_once()?;
        }
        Ok(reply)
    }

    /// One raw read. No trimming: the `MS` status reply is a binary byte and
    /// the port's input EOS (`\r`) already framed the message.
    fn read_once(&self) -> AsynResult<String> {
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw).into_owned())
    }
}

/// One MM3000 axis sharing a controller. Implements [`AsynMotor`].
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
            last_status: MotorStatus {
                gain_support: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
        };
        ax.set_status();
        ax
    }

    fn lock(&self) -> MutexGuard<'_, Mm3000Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn motion_preamble(&self, vel_base: f64, velocity: f64, accel: f64) -> String {
        motion_preamble(self.axis, self.axis_type, vel_base, velocity, accel)
    }
}

impl AsynMotor for Mm3000Axis {
    /// C `set_status`'s COMM_ERR branch raises `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` (drvMM3000.cc). Its first failure's silent RETRY is not modelled:
    /// a transport error raises both at once.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM_AND_COMMS_ERROR
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // One transaction: [VB;]VA;AC;PA (the MM3000 starts moving on PA;
        // the record's GO is a no-op).
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{};{}PA{}",
            self.motion_preamble(min_velocity, velocity, acceleration),
            self.axis,
            nint(position)
        ))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{};{}PR{}",
            self.motion_preamble(min_velocity, velocity, acceleration),
            self.axis,
            nint(distance)
        ))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
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
        let mut ac = nint(acceleration);
        match self.axis_type {
            AxisType::Stepper => ac = ac.max(15000),
            AxisType::Dc => ac = ac.max(250),
            AxisType::Unused => {}
        }
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{a}AC{ac};{a}VA{};{a}PA{}",
            nint(velocity).abs(),
            nint(target)
        ))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C sends the same `OR1` for HOME_FOR and HOME_REV.
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{};{}OR1",
            self.motion_preamble(min_velocity, velocity, acceleration),
            self.axis
        ))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
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

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        Ok(self.set_status())
    }
}

impl Mm3000Axis {
    /// Port of C `set_status`: `{axis}MS` (binary status byte) then
    /// `{axis}TP` (position), each guarded by the NORMAL/RETRY/COMM_ERR
    /// machine on an empty or failed read.
    fn set_status(&mut self) -> MotorStatus {
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
            if ctrl.comm_state == CommState::CommErr {
                self.last_status.comms_error = true;
                self.last_status.problem = true;
            }
            return self.last_status.clone();
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
            if ctrl.comm_state == CommState::CommErr {
                s.comms_error = true;
                s.problem = true;
            }
            return s.clone();
        };
        drop(ctrl);

        // C: first space-delimited token, `atof`, cast to integer counts.
        let position = f64::from(nint(atof(tp.split_whitespace().next().unwrap_or(""))));
        s.position = position;
        s.encoder_position = if self.encoder_present { position } else { 0.0 };
        s.velocity = 0.0; // C: "Parse motor velocity? NEEDS WORK"
        s.problem = false;
        s.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Answers `VE` and the commands in `replies`, fails every other read,
    /// and records every command.
    struct IdentOnly {
        base: epics_rs::asyn::port::PortDriverBase,
        replies: &'static [(&'static str, &'static str)],
        reply: Vec<u8>,
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
            self.reply = match cmd.as_str() {
                "VE" => b"MM3000 Version 2.4".to_vec(),
                c => self
                    .replies
                    .iter()
                    .find(|(k, _)| *k == c)
                    .map_or_else(Vec::new, |(_, v)| v.as_bytes().to_vec()),
            };
            self.sent.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            if reply.is_empty() {
                return Err(mm_err("timeout".into()));
            }
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    /// C keeps a card that answers `VE` whatever its `RC` read returns; a
    /// failed one is an empty buffer and so no axes.
    #[test]
    fn an_unanswered_rc_keeps_the_controller_with_no_axes() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = IdentOnly {
            base: PortDriverBase::new("mm3000IdentOnly", 1, PortFlags::default()),
            replies: &[],
            reply: Vec::new(),
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
        );
        let ctrl = Mm3000Controller::new(io).unwrap();
        assert!(ctrl.axis_types().is_empty());
    }

    /// An MM3000 line answering `replies`: its controller, and the wire log
    /// from after the controller was built.
    fn mm3000_line(
        name: &str,
        replies: &'static [(&'static str, &'static str)],
    ) -> (Mm3000Controller, Arc<Mutex<Vec<String>>>) {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let sent = Arc::new(Mutex::new(Vec::new()));
        let port = IdentOnly {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            replies,
            reply: Vec::new(),
            sent: sent.clone(),
        };
        let (rt, _actor) =
            create_port_runtime(&claim_port(name).unwrap(), port, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_millis(100),
        );
        let ctrl = Mm3000Controller::new(io).unwrap();
        assert_eq!(
            std::mem::take(&mut *sent.lock().unwrap()),
            ["VE", "ST", "VE", "RC"]
        );
        (ctrl, sent)
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
        // Stepper: VB floored at 100, VA at 100, AC at 15000; VB emitted.
        assert_eq!(
            motion_preamble(1, AxisType::Stepper, 10.0, 50.0, 200.0),
            "1VB100;1VA100;1AC15000"
        );
        assert_eq!(
            motion_preamble(2, AxisType::Stepper, 500.0, 2000.0, 20000.0),
            "2VB500;2VA2000;2AC20000"
        );
        // DC: no VB, VA unclamped, AC floored at 250.
        assert_eq!(
            motion_preamble(3, AxisType::Dc, 10.0, 50.0, 200.0),
            "3VA50;3AC250"
        );
        // Unused: no clamps at all.
        assert_eq!(
            motion_preamble(4, AxisType::Unused, 10.0, 50.0, 200.0),
            "4VA50;4AC200"
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
}
