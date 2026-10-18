//! Newport ESP100/ESP300/ESP301 motor controller driver (serial/GPIB ASCII).
//!
//! Ported from `motorNewport/newportApp/src/drvESP300.cc` + `devESP300.cc`
//! (a model-1 dev/drv pair). Commands are `{axis:02}CC[value]` with a 1-based
//! zero-padded axis prefix; several record transactions join commands with
//! `;` into one write (a move is `VB;VA;AC;AG;PA`). Replies end in CR/LF
//! (the port EOS is set at connect as C does, input `\n` / output `\r`; the
//! leftover `\r` is trimmed).
//!
//! ## Shared-serial concurrency
//!
//! One controller drives up to six axes over a single line: every axis is an
//! independent [`AsynMotor`] sharing the [`Esp300Controller`] behind an
//! `Arc<Mutex<..>>`, each operation holding the lock for its whole
//! write→read exchange (see [`crate::agap`] for the pattern rationale).
//!
//! ## Units
//!
//! The record boundary is raw steps, as in devESP300.cc: every command value
//! is `steps * drive_resolution` (gains included) and `TP?` reads back as
//! `NINT(TP / drive_resolution)` steps. `drive_resolution` comes per axis
//! from `ZB?` → `FR?`/`QS?` or `SU?`. A resolution of 0 or a non-finite one
//! (a failed `FR?`/`SU?`, a `QS?` of 0) refuses commands and flags the
//! poll a problem, where C sends `0`/`inf` (devESP300.cc:187, a `QS?` of 0
//! divides at drvESP300.cc:789) and reads back `inf` (drvESP300.cc:277).
//! That C defect is not copied.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_vprintf;

use crate::util::{atof, atoi, is_unsolicited_limit_error, leading_hex};
use motor_common::model1::MoveStrings;

/// Response buffer size for a single controller reply (C `BUFF_SIZE` 100).
const READ_BUF: usize = 256;

/// Replies end in CR/LF (C strips the LF via input EOS, the CR itself).
fn trim_reply(raw: &[u8]) -> String {
    let s = String::from_utf8_lossy(raw);
    s.trim_end_matches(['\r', '\n']).to_string()
}

/// Port EOS set at connect (C `input_terminator`/`output_terminator`, drvESP300.cc:649-650, :679-684).
pub const INPUT_EOS: &[u8] = b"\n";
pub const OUTPUT_EOS: &[u8] = b"\r";

/// Maximum axes per controller (C `ESP300_MAX_AXIS`).
pub(crate) const ESP300_MAX_AXES: usize = 6;

/// Controller communication state (C `cntrl->status`): one garbled status
/// reply is retried silently; a second consecutive one is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Shared controller endpoint: owns the serial handle and the cross-axis
/// communication state. Methods take `&self`/`&mut self`; the caller holds
/// the `Arc<Mutex<..>>` lock.
pub struct Esp300Controller {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
    comm_state: CommState,
}

impl Esp300Controller {
    /// Connect and identify an ESP300: flush, read the identity (`VE?`, up to
    /// 3 tries), then discover the axis count by stopping each axis and
    /// checking `TB?` for error 9 ("axis number out of range") — C
    /// `motor_init`. `None` is C dropping a card that never answers `VE?`,
    /// which it does without a word. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> Option<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes: 0,
            comm_state: CommState::Normal,
        };
        for _ in 0..3 {
            if let Ok(ident) = ctrl.write_read("VE?")
                && !ident.is_empty()
            {
                // C copies from `&buff[1]` ("Skip \n"), which drops the first
                // character under its `\n` input EOS; not copied.
                ctrl.ident = ident;
                break;
            }
        }
        if ctrl.ident.is_empty() {
            return None;
        }

        // Stop each axis in turn; the first "axis number out of range" (9)
        // gives the axis count. Any other error aborts the scan there (a
        // missing stage is not an error). C ignores both statuses: a failed
        // read is an empty reply, which atoi reads as no error.
        let mut num_axes = 0;
        for axis in 1..=ESP300_MAX_AXES {
            let _ = ctrl.write(&format!("{axis:02}ST"));
            let reply = ctrl.write_read("TB?").unwrap_or_default();
            let code = atoi(&reply);
            if code == 9 {
                break;
            }
            if code != 0 {
                errlog_vprintf(format_args!("Error accessing motor {axis}: {reply}\n"));
                break;
            }
            num_axes = axis;
        }
        ctrl.num_axes = num_axes;
        Some(ctrl)
    }

    /// Identity string from `VE?`.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes discovered at construction.
    pub fn num_axes(&self) -> usize {
        self.num_axes
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
        // C writes then reads with no flush, so one stray line shifts every
        // later reply; write_read flushes first. The ESP300 firmware can
        // still put an unsolicited hard-limit error (E35..E42) ahead of the
        // real reply, which C recv_mess re-reads past.
        let reply = trim_reply(&self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?);
        if is_unsolicited_limit_error(&reply) {
            return self.read_once();
        }
        Ok(reply)
    }

    fn read_once(&self) -> AsynResult<String> {
        Ok(trim_reply(&self.handle.read_octet(0, READ_BUF)?))
    }
}

/// The velocity/acceleration preamble every motion transaction carries
/// in the motor record's order (`SET_VELOCITY`, `SET_VEL_BASE`,
/// `SET_ACCEL`): `VA;VB;AC;AG` — the ESP300 takes acceleration and
/// deceleration (`AG`) separately, C sends both. The record sends no
/// SET_ACCEL for a zero rate (motorRecord.cc:874, 940). Values are
/// controller units.
fn motion_preamble(axis: usize, vel_base: f64, velocity: f64, acceleration: f64) -> String {
    let mut msg = format!(
        "{axis:02}VA{velocity:.6};{axis:02}VB{vel_base:.6}",
        velocity = CFloat(velocity),
        vel_base = CFloat(vel_base)
    );
    if acceleration > 0.0 {
        msg += &format!(
            ";{axis:02}AC{acceleration:.6};{axis:02}AG{acceleration:.6}",
            acceleration = CFloat(acceleration)
        );
    }
    msg
}

/// One ESP300 axis sharing a controller. Implements [`AsynMotor`].
pub struct Esp300Axis {
    controller: Arc<Mutex<Esp300Controller>>,
    /// 1-based controller axis number, sent zero-padded (`%.2d`).
    axis: usize,
    /// Controller units per motor step (C `drive_resolution`), discovered at
    /// construction from the feedback configuration (module Units note).
    drive_resolution: f64,
    /// Stage has an encoder (`ZB?` feedback bits 8/9) — drives the
    /// gain-support/has-encoder status bits (C `EA_PRESENT`/`GAIN_SUPPORT`).
    encoder_present: bool,
    /// Last polled status, reused on the early-exit poll paths where C leaves
    /// the record's other bits stale (TB?/MD errors).
    last_status: MotorStatus,
    /// What the last failed poll raises: nothing on C's first missed reply
    /// (RETRY), `CNTRL_COMM_ERR` and `RA_PROBLEM` on the next (COMM_ERR).
    failure: PollFailure,
    strings: MoveStrings,
}

impl Esp300Axis {
    /// Construct axis `axis` (1-based), reading its drive resolution from the
    /// controller (C `motor_init` per-motor block): stepper closed-loop
    /// without encoder feedback (`ZB?` bits 9:8 = `10`) uses full-step (`FR?`)
    /// / microstep (`QS?`); open loop (`00`) uses `FR?` alone; anything else
    /// has an encoder and uses its resolution (`SU?`), then `set_status`.
    /// Performs blocking serial I/O under the controller lock.
    /// C ignores every reply status here: a failed read is an empty reply.
    pub fn new(controller: Arc<Mutex<Esp300Controller>>, axis: usize) -> Self {
        let (drive_resolution, encoder_present) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let read = |cmd: String| ctrl.write_read(&cmd).unwrap_or_default();
            // C reads the controller's EGU (SN?) only for its report(); issue
            // the query for startup wire fidelity and discard the reply.
            let _ = read(format!("{axis:02}SN?"));
            let feedback = leading_hex(&read(format!("{axis:02}ZB?"))).unwrap_or(0);
            let resolution = match feedback & 0x300 {
                0x200 => {
                    let full_step = atof(&read(format!("{axis:02}FR?")));
                    let micro_step = atoi(&read(format!("{axis:02}QS?")));
                    full_step / f64::from(micro_step)
                }
                0x0 => atof(&read(format!("{axis:02}FR?"))),
                _ => atof(&read(format!("{axis:02}SU?"))),
            };
            (resolution, feedback & 0x300 != 0)
        };
        let mut ax = Self {
            controller,
            axis,
            drive_resolution,
            encoder_present,
            // C clears the status, then sets EA_PRESENT and GAIN_SUPPORT
            // for an encoder.
            failure: PollFailure::NONE,
            last_status: MotorStatus {
                gain_support: encoder_present,
                has_encoder: encoder_present,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
        };
        // C motor_init ignores set_status's verdict.
        let _ = ax.set_status();
        ax
    }

    fn lock(&self) -> MutexGuard<'_, Esp300Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `drive_resolution`, refused at 0 or non-finite (module Units note).
    fn resolution(&self) -> AsynResult<f64> {
        let res = self.drive_resolution;
        if res == 0.0 || !res.is_finite() {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("ESP300 axis {}: drive resolution is {res}", self.axis),
            });
        }
        Ok(res)
    }

    /// [`motion_preamble`] for this axis, steps scaled to controller units.
    fn motion_preamble(
        &self,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<String> {
        let res = self.resolution()?;
        Ok(motion_preamble(
            self.axis,
            min_velocity * res,
            velocity * res,
            acceleration * res,
        ))
    }

    /// One move transaction: the record's earlier parts, then PREM and a
    /// `;` ahead of the move's own command (devESP300.cc:201-216), whose
    /// POST waits for the move to end.
    fn write_move(&mut self, parts: &str, command: &str) -> AsynResult<()> {
        let mut line = format!("{parts};").into_bytes();
        let prem = self.strings.arm();
        if !prem.is_empty() {
            line.extend_from_slice(&prem);
            line.push(b';');
        }
        line.extend_from_slice(command.as_bytes());
        self.lock().write_bytes(&line)
    }
}

impl AsynMotor for Esp300Axis {
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
        // One transaction: VA;VB;AC;AG;PA (the ESP300 starts moving on PA;
        // the record's GO is a no-op).
        let parts = self.motion_preamble(min_velocity, velocity, acceleration)?;
        let position = position * self.resolution()?;
        self.write_move(
            &parts,
            &format!(
                "{:02}PA{position:.6}",
                self.axis,
                position = CFloat(position)
            ),
        )
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let parts = self.motion_preamble(min_velocity, velocity, acceleration)?;
        let distance = distance * self.resolution()?;
        self.write_move(
            &parts,
            &format!(
                "{:02}PR{distance:.6}",
                self.axis,
                distance = CFloat(distance)
            ),
        )
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // Record JOG transaction: SET_VEL_BASE, SET_ACCEL, then VA with the
        // jog speed and a signed MV (C `JOG` case).
        let a = self.axis;
        let res = self.resolution()?;
        let (vel_base, velocity, acceleration) =
            (min_velocity * res, velocity * res, acceleration * res);
        let sign = if velocity > 0.0 { '+' } else { '-' };
        self.write_move(
            &format!(
                "{a:02}VB{vel_base:.6};{a:02}AC{acceleration:.6};{a:02}AG{acceleration:.6}",
                vel_base = CFloat(vel_base),
                acceleration = CFloat(acceleration)
            ),
            &format!("{a:02}VA{:.6};{a:02}MV{sign}", CFloat(velocity.abs())),
        )
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C sends the same `OR` for HOME_FOR and HOME_REV (direction ignored),
        // after the record's velocity/acceleration transaction parts.
        let parts = self.motion_preamble(min_velocity, velocity, acceleration)?;
        self.write_move(&parts, &format!("{:02}OR", self.axis))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&format!("{:02}ST", self.axis))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `LOAD_POS`: define home (DH) at the given position.
        let position = position * self.resolution()?;
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{:02}DH{position:.6}",
            self.axis,
            position = CFloat(position)
        ))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C ENABLE_TORQUE → motor on, DISABL_TORQUE → motor off.
        let ctrl = self.lock();
        if enable {
            ctrl.write(&format!("{:02}MO", self.axis))
        } else {
            ctrl.write(&format!("{:02}MF", self.axis))
        }
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let position = position * self.resolution()?;
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{:02}SR{position:.6}",
            self.axis,
            position = CFloat(position)
        ))
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let position = position * self.resolution()?;
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{:02}SL{position:.6}",
            self.axis,
            position = CFloat(position)
        ))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        // C SET_[PID]GAIN: KP/KI/KD then UF (update filter), with an
        // *unpadded* axis prefix (C uses %d here, unlike every other command)
        // and the gain scaled by drive resolution like any other value.
        let a = self.axis;
        let cc = match kind {
            PidGainKind::Proportional => "KP",
            PidGainKind::Integral => "KI",
            PidGainKind::Derivative => "KD",
        };
        let value = gain * self.resolution()?;
        let ctrl = self.lock();
        ctrl.write(&format!("{a}{cc}{value:.6};{a}UF", value = CFloat(value)))
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

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        self.set_status()
    }
}

impl Esp300Axis {
    /// Port of C `set_status`, one exchange sequence per axis under the
    /// controller lock: TB? (clear/check errors), MD (done), TP?
    /// (position), ZH?, MO? (power), TE? (error code). C ignores every read
    /// status: a failed read is an empty reply. RETRY goes to C's `exit`
    /// with `rtn_state` 0, which posts nothing: the poll fails.
    fn set_status(&mut self) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let read = |ctrl: &Esp300Controller, cmd: &str| ctrl.write_read(cmd).unwrap_or_default();
        let a = self.axis;

        let tb = read(&ctrl, "TB?");
        if !tb.starts_with('0') {
            println!("ESP300 status error: {tb}.");
            // C goes to `exit` with `rtn_state` uninitialised; this posts.
            self.last_status.problem = true;
            return Ok(self.last_status.clone());
        }
        self.last_status.problem = false;

        let md = read(&ctrl, &format!("{a:02}MD"));
        if md == "0" || md == "1" {
            ctrl.comm_state = CommState::Normal;
            self.last_status.comms_error = false;
        } else {
            (ctrl.comm_state, self.failure) = match ctrl.comm_state {
                CommState::Normal => (CommState::Retry, PollFailure::NONE),
                _ => (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR),
            };
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("ESP300 axis {a}: MD reply {md:?}"),
            });
        }
        let done = md == "1";

        // TP? reports controller units; C NINTs them to steps.
        let tp = atof(&read(&ctrl, &format!("{a:02}TP?")));
        let steps = self.resolution().map(|res| (tp / res).round());
        let position = steps.as_ref().copied().unwrap_or(self.last_status.position);
        // Direction from the position delta; only one position query exists,
        // so the encoder position is the same value (C set_status).
        if position != self.last_status.position {
            self.last_status.direction = position >= self.last_status.position;
        }

        // Hardware limit/home switches: C reads the ZH? configuration and
        // computes `use_limits = (limits&0x1 == 0) || (limits&0x5 == 0)` —
        // with C precedence that is `limits & (0x1==0)`, always false, so the
        // PH limit query is dead code and the switch bits are always cleared.
        // Kept bug-for-bug: issue ZH? (wire parity), report no switches.
        read(&ctrl, &format!("{a:02}ZH?"));

        let powered = atoi(&read(&ctrl, &format!("{a:02}MO?"))) != 0;

        let te = read(&ctrl, "TE?");
        let errcode = atoi(&te);
        let problem = errcode != 0 || steps.is_err();
        if errcode != 0 {
            println!("ESP300 controller error = {errcode}.");
        }
        // drvESP300.cc:391-398; C ignores the write's status too.
        if done && let Some(post) = self.strings.post_due() {
            let _ = ctrl.write_bytes(&post);
        }
        drop(ctrl);

        self.last_status = MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0, // C: "Parse motor velocity? NEEDS WORK"
            done,
            moving: !done,
            high_limit: false,
            low_limit: false,
            home: false,
            powered,
            problem,
            direction: self.last_status.direction,
            comms_error: false,
            gain_support: self.encoder_present,
            has_encoder: self.encoder_present,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use std::sync::atomic::{AtomicBool, Ordering};

    /// Answers `VE?` and the commands in `replies`; fails every other read.
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
            let cmd = String::from_utf8_lossy(data);
            self.sent.lock().unwrap().push(cmd.to_string());
            self.reply = match cmd.as_ref() {
                "VE?" => b"ESP300 Version 3.08 09/09/02".to_vec(),
                c => self
                    .replies
                    .iter()
                    .find(|(k, _)| *k == c)
                    .map_or_else(Vec::new, |(_, v)| v.as_bytes().to_vec()),
            };
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            if reply.is_empty() {
                return Err(epics_rs::asyn::error::AsynError::Status {
                    status: epics_rs::asyn::error::AsynStatus::Timeout,
                    message: "timeout".into(),
                });
            }
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    /// C reads a failed `TB?` as an empty buffer, which `atoi` takes as no
    /// error, so every axis slot is counted and built.
    #[test]
    fn unanswered_axis_queries_count_every_axis_as_c_does() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = IdentOnly {
            base: PortDriverBase::new(
                "esp300IdentOnly",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            replies: &[],
            reply: Vec::new(),
            sent: Arc::default(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("esp300IdentOnly").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_millis(100),
        );
        let ctrl = Esp300Controller::new(io).unwrap();
        assert_eq!(ctrl.num_axes(), ESP300_MAX_AXES);
        let axis = Esp300Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        assert!(!axis.encoder_present);
    }

    /// A QS? of 0 keeps the axis, as C does, with an infinite resolution
    /// that commands refuse rather than sending `inf`, and polls flag.
    #[test]
    fn a_non_finite_resolution_never_reaches_the_wire() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = IdentOnly {
            base: PortDriverBase::new(
                "esp300ZeroQs",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            replies: &[
                ("TB?", "9, 0, AXIS NUMBER OUT OF RANGE"),
                ("01ZB?", "200"),
                ("01FR?", "0.001"),
                ("01QS?", "0"),
            ],
            reply: Vec::new(),
            sent: Arc::default(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("esp300ZeroQs").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_millis(100),
        );
        let ctrl = Esp300Controller::new(io).unwrap();
        let mut axis = Esp300Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        assert!(axis.drive_resolution.is_infinite());
        let user = AsynUser::default();
        assert!(
            axis.set_pid_gain(&user, PidGainKind::Proportional, 0.5)
                .is_err()
        );
        assert!(axis.move_absolute(&user, 10.0, 0.0, 1.0, 1.0).is_err());
        assert!(axis.poll(&user).unwrap().problem);
    }

    /// An ESP300 line answering `replies`: the controller and the wire log
    /// from after its construction.
    fn esp300_line(
        name: &str,
        replies: &'static [(&'static str, &'static str)],
    ) -> (Esp300Controller, Arc<Mutex<Vec<String>>>) {
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
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
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
        let ctrl = Esp300Controller::new(io).unwrap();
        sent.lock().unwrap().clear();
        (ctrl, sent)
    }

    /// One axis on a [`LineRig`], reporting `MD` from `moving`, with the
    /// construction traffic taken.
    fn strings_rig(name: &str) -> (LineRig, Esp300Axis, Arc<AtomicBool>) {
        let moving = Arc::new(AtomicBool::new(false));
        let m = moving.clone();
        let (line, io) = LineRig::new(name, move |last| {
            match last {
                "VE?" => "ESP300 Version 3.08",
                "TB?" => "0, 0, NO ERROR",
                "01MD" if m.load(Ordering::SeqCst) => "0",
                "01MD" | "01MO?" => "1",
                "TE?" | "01TP?" | "01ZB?" => "0",
                // One controller unit per record step.
                "01FR?" => "1",
                _ => "",
            }
            .to_string()
        });
        let ctrl = Esp300Controller::new(io).unwrap();
        let axis = Esp300Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        line.sent();
        (line, axis, moving)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// devESP300.cc:195-199: INIT and a CR.
    #[test]
    fn init_goes_out_with_a_cr() {
        let (line, mut axis, _) = strings_rig("esp300Init");
        axis.primitive(&AsynUser::default(), b"01MO").unwrap();
        assert_eq!(line.sent(), ["01MO\r"]);
    }

    /// devESP300.cc:201-216: PREM and a `;` go ahead of each move's own
    /// command, for that move only.
    #[test]
    fn prem_leads_the_move_command() {
        let (line, mut axis, _) = strings_rig("esp300Prem");
        let user = AsynUser::default();
        let parts = "01VA2.000000;01VB0.000000;01AC3.000000;01AG3.000000";
        axis.set_move_strings(&user, b"01MO", None).unwrap();
        axis.move_absolute(&user, 1.5, 0.0, 2.0, 3.0).unwrap();
        assert_eq!(line.sent(), [format!("{parts};01MO;01PA1.500000")]);
        axis.move_relative(&user, 1.5, 0.0, 2.0, 3.0).unwrap();
        assert_eq!(line.sent(), [format!("{parts};01PR1.500000")]);
        axis.set_move_strings(&user, b"01MO", None).unwrap();
        axis.home(&user, 0.0, 2.0, 3.0, true).unwrap();
        assert_eq!(line.sent(), [format!("{parts};01MO;01OR")]);
        axis.set_move_strings(&user, b"01MO", None).unwrap();
        axis.move_velocity(&user, 0.0, -2.0, 3.0).unwrap();
        assert_eq!(
            line.sent(),
            ["01VB0.000000;01AC3.000000;01AG3.000000;01MO;01VA2.000000;01MV-"]
        );
    }

    /// drvESP300.cc:391-398: POST once the move is done, not while it runs.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, moving) = strings_rig("esp300Post");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"01MF")))
            .unwrap();
        axis.move_absolute(&user, 1.0, 0.0, 2.0, 3.0).unwrap();
        line.sent();
        let extra = |line: &LineRig| -> Vec<String> {
            line.sent()
                .into_iter()
                .filter(|l| !l.ends_with('?') && l != "01MD")
                .collect()
        };
        moving.store(true, Ordering::SeqCst);
        axis.poll(&user).unwrap();
        assert!(extra(&line).is_empty());
        moving.store(false, Ordering::SeqCst);
        axis.poll(&user).unwrap();
        assert_eq!(extra(&line), ["01MF"]);
        axis.poll(&user).unwrap();
        assert!(extra(&line).is_empty(), "sent once");
    }

    /// drvESP300.cc motor_init ends each axis with set_status.
    #[test]
    fn axis_init_runs_set_status_after_the_resolution_queries() {
        let (ctrl, sent) = esp300_line(
            "esp300InitStatus",
            &[
                ("TB?", "0, 0, NO ERROR DETECTED"),
                ("01MD", "1"),
                ("01FR?", "0.001"),
                ("01TP?", "1.5"),
                ("01ZH?", "24"),
                ("01MO?", "1"),
                ("TE?", "0"),
            ],
        );
        let axis = Esp300Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        assert_eq!(
            *sent.lock().unwrap(),
            [
                "01SN?", "01ZB?", "01FR?", "TB?", "01MD", "01TP?", "01ZH?", "01MO?", "TE?"
            ]
        );
        let s = &axis.last_status;
        assert_eq!(s.position, 1500.0);
        assert!(s.done && s.powered && !s.problem && !s.comms_error);
    }

    /// A silent `TB?` is C's empty buffer: "ESP300 status error: ." and
    /// RA_PROBLEM, not an asyn error out of the poll.
    #[test]
    fn a_silent_tb_is_a_status_error_not_a_failed_poll() {
        let (ctrl, sent) = esp300_line("esp300SilentTb", &[]);
        let mut axis = Esp300Axis::new(Arc::new(Mutex::new(ctrl)), 1);
        sent.lock().unwrap().clear();
        let status = axis.poll(&AsynUser::default()).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["TB?"]);
        assert!(status.problem);
    }

    #[test]
    fn preamble_matches_c_transaction() {
        // C build_trans: %.2d axis prefix, %f (6-decimal) values, AC and AG
        // both sent with the acceleration.
        assert_eq!(
            motion_preamble(1, 0.5, 2.0, 8.0),
            "01VA2.000000;01VB0.500000;01AC8.000000;01AG8.000000"
        );
        assert_eq!(
            motion_preamble(12, 0.0, 1.0, 4.0),
            "12VA1.000000;12VB0.000000;12AC4.000000;12AG4.000000"
        );
    }

    /// motorRecord.cc:874, 940: no SET_ACCEL for a zero move or home rate.
    #[test]
    fn a_zero_rate_preamble_has_no_ac_or_ag() {
        assert_eq!(
            motion_preamble(3, 0.5, 2.0, 0.0),
            "03VA2.000000;03VB0.500000"
        );
    }

    /// drvESP300.cc:241-266: a first bad `MD` is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and a good one clears it.
    #[test]
    fn a_bad_md_is_retry_then_comm_error() {
        let user = AsynUser::default();
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("esp300Retry", move |last| {
            match last {
                "VE?" => "ESP300 Version 3.08",
                "TB?" => "0, 0, NO ERROR",
                "01MD" if q.load(Ordering::SeqCst) => "",
                "01MD" | "01MO?" => "1",
                "TE?" | "01TP?" | "01ZB?" => "0",
                "01FR?" => "1",
                _ => "",
            }
            .to_string()
        });
        let ctrl = Esp300Controller::new(io).unwrap();
        let mut axis = Esp300Axis::new(Arc::new(Mutex::new(ctrl)), 1);
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
