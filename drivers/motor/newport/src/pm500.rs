//! Newport PM500 precision motor controller driver (serial/GPIB ASCII).
//!
//! Ported from `motorNewport/newportApp/src/drvPM500.cc` + `devPM500.cc`
//! (a model-1 dev/drv pair, itself derived from drvMM4000). Axes are named
//! by letter — `X Y Z A B C D E F G H I` — and commands are `{letter}CC` with
//! values at 4 decimals (C `res_decpts` from the fixed 0.01 drive
//! resolution). The controller is put in `SCUM 1` mode with `SENAINT $AF` at
//! startup: responses are CR-terminated with the axis letter prefixed and no
//! command acknowledgement suppression, and the C driver reads one reply
//! after *every* command message (`cmnd_response = true`) — this port does
//! the same via `Pm500Controller::command`.
//!
//! Commands go out bare (the C driver sets no EOS); the port EOS `"\r"` both
//! ways comes from the startup script, as in C's Newport_PM500.iocsh:22-23.
//!
//! ## Units
//!
//! The record boundary is raw steps, as in devPM500.cc: every value goes
//! out as `steps * drive_resolution` (0.01 for every firmware type C
//! recognizes), and the `R` readback comes back as `NINT(position /
//! drive_resolution)` steps. The wire speaks microns/arc-sec for positions
//! but mm/sec and karc-sec/sec for velocities (and /sec² for
//! accelerations), so C divides velocity and acceleration by 1000 after the
//! step conversion.
//!
//! ## Deviations from C (documented)
//!
//! - C leaves `drive_resolution` at 0.0 for an unrecognized `CONFIG?`
//!   firmware string and still creates the axis, with 1-digit command
//!   precision, then sends every value as 0 (devPM500.cc:181, so a move
//!   goes to 0) and divides its readback by 0 (drvPM500.cc:267). Not
//!   copied: this port keeps the axis but refuses its commands and flags its
//!   polls a problem.
//! - C's axis-discovery and status parsing index fixed reply offsets into a
//!   buffer that may hold stale bytes after a read failure; this port treats
//!   a too-short reply like an empty one (comm-retry machine).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_message;

use crate::util::{atof, max_digits, nint};
use motor_common::model1::MoveStrings;

/// Response buffer size for a single controller reply (C `BUFF_SIZE` 100).
const READ_BUF: usize = 256;

/// Axis letters, in controller channel order (C `PM500_axis_names`,
/// `PM500_NUM_CHANNELS` 12).
const PM500_AXIS_NAMES: [char; 12] = ['X', 'Y', 'Z', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'];

/// Controller communication state (C `cntrl->status`): one empty status
/// reply is retried silently; a second consecutive one is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// C `motor_init` firmware lookup: the drive resolution comes from the axis
/// firmware reported by `CONFIG?` — 302 (50 nm translator), 309 (25 nm
/// translator), 300 (unknown translator), and the XXX rotator placeholder
/// all map to 0.01. Unknown firmware keeps C's initial 0.0.
fn resolution_for_firmware(firmware: &str) -> f64 {
    match firmware {
        "302" | "309" | "300" | "XXX" => 0.01,
        _ => 0.0,
    }
}

/// Shared controller endpoint: owns the serial handle and the cross-axis
/// communication state. The caller holds the `Arc<Mutex<..>>` lock.
pub struct Pm500Controller {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
    comm_state: CommState,
}

impl Pm500Controller {
    /// Connect and identify a PM500 (C `motor_init`): put the controller in
    /// `SCUM 1` mode, configure `SENAINT $AF` framing, probe `SVN?` for the
    /// identity, then discover axes by querying `{letter}STAT?` for each of
    /// the 12 channel letters until one answers with an error (`E` at reply
    /// byte 1). `None` is C's silently disabled controller: no `SVN?`
    /// reply. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> Option<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes: 0,
            comm_state: CommState::Normal,
        };
        // C sends SCUM 1 and SENAINT $AF and reads (ignoring) one reply each.
        let _ = ctrl.command("SCUM 1");
        let _ = ctrl.command("SENAINT $AF");

        // Identity probe (C sends SVN? twice: existence check, then read).
        if ctrl.command("SVN?").unwrap_or_default().is_empty() {
            return None;
        }
        let ident = ctrl.command("SVN?").unwrap_or_default();
        // C strncpy's 50 bytes past the "XD" (drvPM500.cc:584).
        ctrl.ident = ident.get(2..).unwrap_or("").chars().take(50).collect();

        // Axis discovery: STAT? each channel letter in order; an 'E' at
        // reply byte 1 ends the scan (C checks buff[1]).
        let mut num_axes = 0;
        for name in PM500_AXIS_NAMES {
            let reply = ctrl.command(&format!("{name}STAT?")).unwrap_or_default();
            match reply.as_bytes().get(1) {
                Some(b'E') | None => break,
                Some(_) => num_axes += 1,
            }
        }
        ctrl.num_axes = num_axes;
        Some(ctrl)
    }

    /// Identity string from `SVN?` (the 2-byte echo prefix stripped).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes discovered at construction.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    /// Write one command message and read its one reply: C `send_mess`,
    /// which flushes stale input first, + `recv_mess`, which logs a "system
    /// error" on an `SE` reply but still returns it. C's motor task pairs
    /// every `send_mess` with a `recv_mess` (`cmnd_response = true`), so even
    /// pure motion messages consume a response.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.command_bytes(cmd.as_bytes())
    }

    fn command_bytes(&self, cmd: &[u8]) -> AsynResult<String> {
        let raw = self.handle.write_read(0, cmd, READ_BUF)?;
        let reply = String::from_utf8_lossy(&raw).into_owned();
        if reply.starts_with("SE") {
            errlog_message("recv_mess(): PM500 system error.\n");
        }
        Ok(reply)
    }
}

/// One PM500 axis sharing a controller. Implements [`AsynMotor`].
pub struct Pm500Axis {
    controller: Arc<Mutex<Pm500Controller>>,
    /// Controller channel letter (C `PM500_axis_names[signal]`).
    name: char,
    /// Command decimal precision (C `res_decpts`, from the firmware-table
    /// drive resolution — 4 for every recognized type).
    digits: usize,
    /// Wire units per record step (C `drive_resolution`).
    drive_resolution: f64,
    /// What the last failed poll raises, kept per axis: the controller's
    /// state can move on under another axis's poll before it is read.
    failure: PollFailure,
    /// Last good status: the base of the next one, whose position and
    /// encoder it keeps when the steps are unchanged.
    last_status: MotorStatus,
    strings: MoveStrings,
}

impl Pm500Axis {
    /// Construct the axis at 0-based channel `index`, running the C
    /// per-motor init: `{letter}CONFIG?` → firmware string at reply offset 8
    /// → drive resolution by lookup, then `set_status`. Performs
    /// blocking serial I/O under the controller lock.
    pub fn new(controller: Arc<Mutex<Pm500Controller>>, index: usize) -> Self {
        let name = PM500_AXIS_NAMES[index];
        let drive_resolution = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // A failed read leaves C an empty firmware string, so resolution 0.0.
            let config = ctrl.command(&format!("{name}CONFIG?")).unwrap_or_default();
            resolution_for_firmware(config.get(8..).unwrap_or(""))
        };
        let mut ax = Self {
            controller,
            name,
            digits: max_digits(drive_resolution),
            drive_resolution,
            failure: PollFailure::NONE,
            // C clears the status, then hard-wires EA_PRESENT and
            // GAIN_SUPPORT ("PM500 only supports DC motors").
            last_status: MotorStatus {
                gain_support: true,
                has_encoder: true,
                vbas_supported: false,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
        };
        // C motor_init ignores set_status's verdict.
        let _ = ax.set_status();
        ax
    }

    fn lock(&self) -> MutexGuard<'_, Pm500Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// One record transaction: its speed parts, then PREM and `;`, then the
    /// motion part, in one message (devPM500.cc:216-232).
    fn transaction(&mut self, speeds: String, motion: String) -> AsynResult<()> {
        let mut msg = speeds.into_bytes();
        let prem = self.strings.arm();
        if !prem.is_empty() {
            msg.extend_from_slice(&prem);
            msg.push(b';');
        }
        msg.extend_from_slice(motion.as_bytes());
        self.lock().command_bytes(&msg)?;
        Ok(())
    }

    /// `drive_resolution`, refused at the 0.0 of an unknown firmware
    /// (module Deviations).
    fn resolution(&self) -> AsynResult<f64> {
        match self.drive_resolution {
            0.0 => Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!(
                    "PM500 axis {}: unknown firmware, no drive resolution",
                    self.name
                ),
            }),
            res => Ok(res),
        }
    }

    /// A record step count as a wire position at command precision.
    fn wire(&self, steps: f64) -> AsynResult<String> {
        Ok(format!(
            "{:.1$}",
            CFloat(steps * self.resolution()?),
            self.digits
        ))
    }

    /// A record step rate as wire mm/sec(²), C's ÷1000.
    fn rate(&self, steps: f64) -> AsynResult<f64> {
        Ok(steps * self.resolution()? / 1000.0)
    }

    /// The SET_VELOCITY and SET_ACCEL parts of a move or home; the record
    /// sends SET_ACCEL only for a positive rate (motorRecord.cc:874, 940).
    fn speeds(&self, velocity: f64, acceleration: f64) -> AsynResult<String> {
        let (c, d) = (self.name, self.digits);
        let mut msg = format!("{c}V{:.d$};", CFloat(self.rate(velocity)?));
        if acceleration > 0.0 {
            msg += &format!("{c}ACCEL{:.d$};", CFloat(self.rate(acceleration)?));
        }
        Ok(msg)
    }
}

impl AsynMotor for Pm500Axis {
    /// C `set_status` (drvPM500.cc:244-260): the first unanswered status
    /// query is RETRY, which raises nothing and posts nothing; the next is
    /// COMM_ERR, raising `CNTRL_COMM_ERR` and `RA_PROBLEM`.
    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // Record move transaction: SET_VELOCITY;SET_ACCEL;MOVE_ABS (VEL_BASE
        // and GO build nothing). Wire velocity/acceleration are mm/sec(²) —
        // the ÷1000 from position units is C's (module Units note).
        let speeds = self.speeds(velocity, acceleration)?;
        let motion = format!("{}G{};", self.name, self.wire(position)?);
        self.transaction(speeds, motion)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let speeds = self.speeds(velocity, acceleration)?;
        let motion = format!("{}R{};", self.name, self.wire(distance)?);
        self.transaction(speeds, motion)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // Record JOG transaction: SET_ACCEL then the signed slew command
        // (C JOG case, `%f` — 6 decimals, unlike the other values).
        let (c, d) = (self.name, self.digits);
        let accel = format!("{c}ACCEL{:.d$};", CFloat(self.rate(acceleration)?));
        let motion = format!("{c}S{:.6};", CFloat(self.rate(velocity)?));
        self.transaction(accel, motion)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C builds the same `F0` for HOME_FOR and HOME_REV, after the
        // record's velocity/acceleration transaction parts.
        let speeds = self.speeds(velocity, acceleration)?;
        self.transaction(speeds, format!("{}F0;", self.name))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C STOP_AXIS: T (stop) then an R status query in the same message.
        let c = self.name;
        let ctrl = self.lock();
        ctrl.command(&format!("{c}T;{c}R"))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C LOAD_POS builds no command (the PM500 cannot redefine its
        // position) and reports success; kept as a quiet no-op.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C ENABLE_TORQUE → `T;M?`, DISABL_TORQUE → `M;M?` (the M? readback
        // is consumed as the message's reply).
        let c = self.name;
        let ctrl = self.lock();
        if enable {
            ctrl.command(&format!("{c}T;{c}M?"))?;
        } else {
            ctrl.command(&format!("{c}M;{c}M?"))?;
        }
        Ok(())
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let msg = format!("{}PSLIM{};", self.name, self.wire(position)?);
        self.lock().command(&msg)?;
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let msg = format!("{}NSLIM{};", self.name, self.wire(position)?);
        self.lock().command(&msg)?;
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // C SET_[PID]GAIN builds no command but reports success (the record
        // sees GAIN_SUPPORT set); kept as a quiet no-op.
        Ok(())
    }

    /// `PRIMITIVE`: INIT and a CR (devPM500.cc:202-206).
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let mut msg = init.to_vec();
        msg.push(b'\r');
        self.lock().command_bytes(&msg)?;
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
        self.set_status()
    }
}

impl Pm500Axis {
    /// Port of C `set_status`: one `{letter}R` status/position query, then
    /// `{letter}M?` for the servo state. Reply format
    /// `[letter][status][sign+digits...]`, e.g. `XB+5.012`.
    fn set_status(&mut self) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let c = self.name;

        // C turns any read failure into an empty reply and runs the retry
        // machine on it; a too-short reply is treated the same (module
        // Deviations).
        let reply = ctrl.command(&format!("{c}R")).unwrap_or_default();
        if reply.len() < 3 {
            (ctrl.comm_state, self.failure) = match ctrl.comm_state {
                CommState::Normal => (CommState::Retry, PollFailure::NONE),
                _ => (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR),
            };
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("PM500 axis {c}: no status reply"),
            });
        }
        ctrl.comm_state = CommState::Normal;

        let status_char = reply.as_bytes()[1];
        let dir_char = reply.as_bytes()[2];
        // Position parses from the direction sign onward, in steps.
        let steps = self.resolution().map(|res| atof(&reply[2..]) / res);

        let moving = status_char == b'B';
        let direction = dir_char == b'+';
        let on_limit = status_char == b'L';

        // Servo on/off (C maps the M? readback to EA_POSITION).
        let servo = ctrl.command(&format!("{c}M?")).unwrap_or_default();
        let powered = atof(servo.get(2..).unwrap_or("")) as i32 != 0;

        // drvPM500.cc:334-342: POST and a CR, written with no reply read,
        // once the move is done or on a limit ('L' is also done).
        if !moving && let Some(mut post) = self.strings.post_due() {
            post.push(b'\r');
            let _ = ctrl.handle.write_octet(0, &post);
        }
        drop(ctrl);

        // drvPM500.cc:306-320: NINT steps and the encoder truncated, both
        // kept when the steps equal the last position.
        let last = &self.last_status;
        let (position, encoder_position) = match steps {
            Ok(steps) if steps != last.position => (nint(steps) as f64, steps as i32 as f64),
            _ => (last.position, last.encoder_position),
        };
        self.last_status = MotorStatus {
            position,
            encoder_position,
            velocity: 0.0, // C: "Parse motor velocity? NEEDS WORK"
            done: !moving,
            direction,
            // C sets only the switch in the travel direction on an 'L' and
            // leaves the other as it was.
            high_limit: on_limit && (direction || self.last_status.high_limit),
            low_limit: on_limit && (!direction || self.last_status.low_limit),
            home: false,
            powered,
            // C sets RA_PROBLEM from an 'E' status but unconditionally
            // clears it a few lines later (upstream dead store) — net
            // effect: never a problem outside the comm-error path. Kept
            // bug-for-bug. An unknown firmware's 0.0 resolution is one.
            problem: steps.is_err(),
            comms_error: false,
            // C: "PM500 only supports DC motors" — EA_PRESENT and
            // GAIN_SUPPORT are hard-wired on.
            gain_support: true,
            has_encoder: true,
            // C SET_VEL_BASE: "PM500 does not use base velocity".
            vbas_supported: false,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A line that never answers.
    struct Silent(epics_rs::asyn::port::PortDriverBase);

    impl epics_rs::asyn::port::PortDriver for Silent {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.0
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, _buf: &mut [u8]) -> AsynResult<usize> {
            Err(epics_rs::asyn::error::AsynError::Status {
                status: epics_rs::asyn::error::AsynStatus::Timeout,
                message: "timeout".into(),
            })
        }
    }

    /// C `motor_init` drops a controller whose `SVN?` goes unanswered
    /// (drvPM500.cc:577, :671-672).
    #[test]
    fn a_silent_ident_creates_no_controller() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
        use std::time::Duration;

        let line = Silent(PortDriverBase::new(
            "pm500_silent",
            1,
            PortFlags::default(),
            InterfaceMask::UNMIGRATED_ALL,
            InterfaceMask::UNMIGRATED_INTERRUPTS,
        ));
        let (rt, _jh) = create_port_runtime(
            &claim_port("pm500_silent").unwrap(),
            line,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_millis(100));
        assert!(Pm500Controller::new(io).is_none());
    }

    /// A PM500 with one axis (`X`) that records every command.
    struct Scripted {
        base: epics_rs::asyn::port::PortDriverBase,
        reply: Vec<u8>,
        sent: Arc<Mutex<Vec<String>>>,
        xr: Arc<Mutex<&'static str>>,
    }

    impl epics_rs::asyn::port::PortDriver for Scripted {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            self.reply = match cmd.as_str() {
                "SVN?" => "XD PM500 1.0",
                "XSTAT?" => "X0",
                "YSTAT?" => "YE",
                "XCONFIG?" => "XCONFIG 302",
                "XR" => *self.xr.lock().unwrap(),
                "XM?" => "XM1",
                _ => "OK",
            }
            .as_bytes()
            .to_vec();
            self.sent.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    type Line = (Pm500Axis, Arc<Mutex<Vec<String>>>, Arc<Mutex<&'static str>>);

    /// drvPM500.cc motor_init for one controller and its `X` axis: the
    /// axis, the wire log, and the `XR` reply.
    fn pm500_axis(name: &str) -> Line {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
        use std::time::Duration;

        let sent = Arc::new(Mutex::new(Vec::new()));
        let xr = Arc::new(Mutex::new("XB+5.017"));
        let line = Scripted {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            reply: Vec::new(),
            sent: sent.clone(),
            xr: xr.clone(),
        };
        let (rt, _jh) =
            create_port_runtime(&claim_port(name).unwrap(), line, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_millis(100));
        let ctrl = Pm500Controller::new(io).unwrap();
        assert_eq!(ctrl.num_axes(), 1);
        (Pm500Axis::new(Arc::new(Mutex::new(ctrl)), 0), sent, xr)
    }

    /// drvPM500.cc motor_init ends each axis with set_status.
    #[test]
    fn axis_init_runs_set_status_after_config() {
        let (axis, sent, _) = pm500_axis("pm500_init");
        assert_eq!(
            *sent.lock().unwrap(),
            [
                "SCUM 1",
                "SENAINT $AF",
                "SVN?",
                "SVN?",
                "XSTAT?",
                "YSTAT?",
                "XCONFIG?",
                "XR",
                "XM?"
            ]
        );
        let s = &axis.last_status;
        // drvPM500.cc: NINT steps, the encoder truncated.
        assert_eq!((s.position, s.encoder_position), (502.0, 501.0));
        // drvPM500 reports a busy axis as RA_DONE 0 and never sets RA_MOVING.
        assert!(!s.done && !s.moving && s.direction && s.powered && s.has_encoder);
    }

    /// devPM500.cc sends `steps * drive_resolution`, speeds further ÷1000.
    #[test]
    fn a_move_sends_record_steps_times_drive_resolution() {
        let (mut axis, sent, _) = pm500_axis("pm500_move");
        let user = AsynUser::default();
        sent.lock().unwrap().clear();
        axis.move_absolute(&user, 1234.0, 0.0, 5000.0, 20000.0)
            .unwrap();
        axis.move_velocity(&user, 0.0, -5000.0, 20000.0).unwrap();
        axis.set_high_limit(&user, 1000.0).unwrap();
        assert_eq!(
            *sent.lock().unwrap(),
            [
                "XV0.0500;XACCEL0.2000;XG12.3400;",
                "XACCEL0.2000;XS-0.050000;",
                "XPSLIM10.0000;"
            ]
        );
    }

    /// devPM500.cc `%.*f` / `%f`: glibc's nan, sign bit included.
    #[test]
    fn a_nan_move_goes_out_as_c_prints_it() {
        let (mut axis, sent, _) = pm500_axis("pm500_nan");
        let user = AsynUser::default();
        sent.lock().unwrap().clear();
        let _ = axis.move_absolute(&user, f64::NAN, 0.0, -f64::NAN, 20000.0);
        let _ = axis.move_velocity(&user, 0.0, f64::NAN, -f64::NAN);
        assert_eq!(
            *sent.lock().unwrap(),
            ["XV-nan;XACCEL0.2000;XGnan;", "XACCEL-nan;XSnan;"]
        );
    }

    /// drvPM500.cc:306-320 keeps the encoder when the steps equal the last
    /// position, and refreshes both otherwise.
    #[test]
    fn an_unchanged_position_keeps_the_encoder() {
        let (mut axis, _, xr) = pm500_axis("pm500_enc");
        let user = AsynUser::default();
        let mut poll = |reply: &'static str| {
            *xr.lock().unwrap() = reply;
            let s = axis.poll(&user).unwrap();
            (s.position, s.encoder_position)
        };
        assert_eq!(poll("XD+4.997"), (500.0, 499.0));
        assert_eq!(poll("XD+5.000"), (500.0, 499.0));
        assert_eq!(poll("XD+5.034"), (503.0, 503.0));
    }

    /// drvPM500.cc set_status on an 'L' status sets only the switch in the
    /// travel direction; a clear status clears both.
    #[test]
    fn a_limit_status_keeps_the_other_switch() {
        let (mut axis, _, xr) = pm500_axis("pm500_limits");
        let user = AsynUser::default();
        let mut poll = |reply: &'static str| {
            *xr.lock().unwrap() = reply;
            let s = axis.poll(&user).unwrap();
            (s.high_limit, s.low_limit)
        };
        assert_eq!(poll("XL+5.012"), (true, false));
        assert_eq!(poll("XL-5.012"), (true, true));
        assert_eq!(poll("XD-5.012"), (false, false));
        assert_eq!(poll("XL-5.012"), (false, true));
    }

    /// motorRecord.cc builds no SET_ACCEL for a zero rate, so the move
    /// carries no `ACCEL` part.
    #[test]
    fn a_zero_acceleration_sends_no_accel() {
        let (mut axis, sent, _) = pm500_axis("pm500_acc0");
        let user = AsynUser::default();
        sent.lock().unwrap().clear();
        axis.move_absolute(&user, 150.0, 0.0, 200000.0, 0.0)
            .unwrap();
        axis.home(&user, 0.0, 200000.0, 300000.0, true).unwrap();
        assert_eq!(
            *sent.lock().unwrap(),
            ["XV2.0000;XG1.5000;", "XV2.0000;XACCEL3.0000;XF0;"]
        );
    }

    /// devPM500.cc: INIT and a CR alone, PREM between the speed parts and
    /// the motion part of one message, POST and a CR once the move is done.
    #[test]
    fn init_prem_and_post_go_out_as_c_builds_them() {
        let (mut axis, sent, xr) = pm500_axis("pm500_strings");
        let user = AsynUser::default();
        let post = PostMessage::default();
        post.set(b"XSON");
        sent.lock().unwrap().clear();
        axis.primitive(&user, b"SCUM 1").unwrap();
        axis.set_move_strings(&user, b"XSOFF", Some(post)).unwrap();
        axis.move_relative(&user, 200.0, 0.0, 200000.0, 100000.0)
            .unwrap();
        axis.poll(&user).unwrap();
        *xr.lock().unwrap() = "XD+7.012";
        axis.poll(&user).unwrap();
        axis.poll(&user).unwrap();
        assert_eq!(
            *sent.lock().unwrap(),
            [
                "SCUM 1\r",
                "XV2.0000;XACCEL1.0000;XSOFF;XR2.0000;",
                "XR",
                "XM?",
                "XR",
                "XM?",
                "XSON\r",
                "XR",
                "XM?"
            ]
        );
    }

    /// drvPM500.cc:244-260: a first unanswered `R` is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and an answer clears it.
    #[test]
    fn an_unanswered_status_is_retry_then_comm_error() {
        let (mut axis, _, xr) = pm500_axis("pm500_retry");
        let user = AsynUser::default();
        *xr.lock().unwrap() = "";
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        *xr.lock().unwrap() = "XD+5.000";
        let s = axis.poll(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        assert!(axis.poll(&user).is_ok());
        *xr.lock().unwrap() = "";
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }

    #[test]
    fn firmware_lookup_matches_c_table() {
        assert_eq!(resolution_for_firmware("302"), 0.01);
        assert_eq!(resolution_for_firmware("309"), 0.01);
        assert_eq!(resolution_for_firmware("300"), 0.01);
        assert_eq!(resolution_for_firmware("XXX"), 0.01);
        assert_eq!(resolution_for_firmware("310"), 0.0);
        assert_eq!(resolution_for_firmware(""), 0.0);
        // C motor_init: digits from 0.0 floor at 1.
        assert_eq!(max_digits(resolution_for_firmware("310")), 1);
    }

    #[test]
    fn axis_letters_match_c_channel_order() {
        assert_eq!(PM500_AXIS_NAMES[0], 'X');
        assert_eq!(PM500_AXIS_NAMES[3], 'A');
        assert_eq!(PM500_AXIS_NAMES[11], 'I');
    }
}
