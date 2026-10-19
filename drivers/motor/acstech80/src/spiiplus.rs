//! ACS Tech80 SPiiPlus motion controller driver (serial/TCP ASCII, ACSPL+).
//!
//! Ported from `motorAcsTech80/acsTech80App/src/drvSPiiPlus.cc` +
//! `devSPiiPlus.cc` (the model-1 dev/drv pair). Commands are ACSPL+ text
//! terminated by CR (`\r`, the port EOS set both ways at connect as C does). Query commands begin with `?` and return a value;
//! a reply containing `?` signals a controller error. Set/motion commands do
//! not return a value, but the framework reads an acknowledgment after every
//! command, so this port reads (and discards) a reply after each command to
//! keep the stream synchronized. Axes are addressed by 0-based index.
//!
//! ## Command interface modes
//!
//! The controller is driven in one of three modes (C `SPiiPlusConfig` `modeStr`,
//! default [`CommandMode::Buffer`]):
//!
//! - [`Buffer`](CommandMode::Buffer): motion is initiated by ACSPL program
//!   buffers resident on the controller. Moves set `Done`/`target_pos`/`opReq`
//!   variables and then `start` the per-axis buffer; done and the home request
//!   are read back from `Done`/`opReq`.
//! - [`Connect`](CommandMode::Connect): the ACSPL `CONNECT` kinematic mode
//!   (e.g. hexapods). Motion uses the direct `ptp`/`jog` commands; the feedback
//!   position is read with the kinematic `?FPOS<letter>` form (axis letter
//!   `X Y Z T A B C D`).
//! - [`Direct`](CommandMode::Direct): direct access to the physical motors via
//!   the command interpreter (`ptp`, `jog/v`, `halt`). Homing is not available
//!   in this mode.
//!
//! The `Buffer` and `Connect` modes reference controller-resident ACSPL
//! variables/programs (`Done`, `target_pos`, `opReq`, per-axis buffers) that the
//! site must have loaded, exactly as in the C driver — this port only sends the
//! same command strings.
//!
//! ## Units
//!
//! Positions are read with `?APOS`/`?FPOS` and commanded with `ptp`/`target_pos`
//! in controller counts, with no resolution scaling, so the asyn-rs motor
//! boundary is counts: a commanded position is rounded with `NINT`, a read one
//! truncated by C's `(long)` cast, the record's `MRES` is 1, and its `EGU` is
//! counts.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the per-mode query set runs
//!   inside [`poll`](AsynMotor::poll).
//! - The C `set_status` velocity readback is marked `NEEDS WORK` and reports the
//!   feedback velocity `?FVEL`; this port reports the same.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, Model1Motor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::{MoveStrings, init_read};
use motor_common::util::{atof, atoi, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 120;

/// Port EOS set at connect (C `ACS_EOS`, drvSPiiPlus.cc:690-691).
pub const INPUT_EOS: &[u8] = b"\r";
pub const OUTPUT_EOS: &[u8] = b"\r";

/// Motor-status (`MST`) bit masks (C `MOTOR_STATUS`).
const MST_ENABLED: i32 = 0x01;
const MST_INPOSITION: i32 = 0x10;
const MST_INMOTION: i32 = 0x20;

/// Fault-word (`FAULT`) bit masks (C `MOTOR_FAULTS`).
const FAULT_LL: i32 = 0x01; // left limit
const FAULT_RL: i32 = 0x02; // right limit
const FAULT_SRL: i32 = 0x20; // software right limit
const FAULT_SLL: i32 = 0x40; // software left limit

/// `opReq` operation codes (C `OP_*`).
const OP_HOME_F: i32 = 4;
const OP_HOME_R: i32 = 5;
const OP_ABS_MOVE: i32 = 1;
const OP_REL_MOVE: i32 = 2;
const OP_JOG_MOVE: i32 = 3;

/// ACSPL kinematic axis letters, indexed by 0-based axis (C `ACSPL_axis`).
const AXIS_LETTERS: [&str; 8] = ["X", "Y", "Z", "T", "A", "B", "C", "D"];

/// C `CommStatus`, kept per card.
#[derive(Clone, Copy, PartialEq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Controller command-interface mode (C `enum CMND_MODES`).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum CommandMode {
    /// Motion via ACSPL program buffers (`BUF`; the C default).
    Buffer,
    /// ACSPL `CONNECT` kinematic mode (`CON`).
    Connect,
    /// Direct command-interpreter access (`DIR`).
    Direct,
}

impl CommandMode {
    /// Parse the `SPiiPlusConfig` mode string (first three letters, case
    /// insensitive); anything unrecognized defaults to `Buffer`, as in C.
    pub fn parse(mode: &str) -> Self {
        let m: String = mode.chars().take(3).collect::<String>().to_uppercase();
        match m.as_str() {
            "DIR" => CommandMode::Direct,
            "CON" => CommandMode::Connect,
            _ => CommandMode::Buffer,
        }
    }
}

/// The SET_ACCEL the record sends ahead of a move or home: none unless the
/// rate is positive (motorRecord.cc:874); a jog's goes at any (:2141).
fn move_accel(acceleration: f64) -> Option<f64> {
    (acceleration > 0.0).then_some(acceleration)
}

fn spiiplus_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle, the command mode, and the
/// axis count.
pub struct SpiiPlusController {
    handle: SyncIOHandle,
    ident: String,
    mode: CommandMode,
    num_axes: usize,
    comm_state: CommState,
}

impl SpiiPlusController {
    /// Connect and identify a SPiiPlus controller (C `motor_init`): probe `?VR`,
    /// `halt all`, read the version, then auto-detect the axis count by querying
    /// `?APOS(n)` until the controller errors. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle, mode: CommandMode) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            mode,
            num_axes: 0,
            comm_state: CommState::Normal,
        };

        // C's send_recv_mess returns the bytes read, so any reply makes the
        // card present and only a silent one is probed again
        // (drvSPiiPlus.cc:692-701); halt and ident statuses are ignored.
        let answered = (0..3).any(|_| {
            ctrl.handle
                .write_read(0, b"?VR", READ_BUF)
                .nbytes_transferred()
                > 0
        });
        if !answered {
            return Err(spiiplus_err("SPiiPlus: no response to ?VR probe"));
        }
        let _ = ctrl.command("halt all");
        ctrl.ident = ctrl.command("?VR").unwrap_or_default();

        // Auto-detect the axis count: ?APOS(n) succeeds until n is out of
        // range. A failed one only ends the count (drvSPiiPlus.cc:711-717),
        // so a card that answered ?VR stays, with no axes at worst.
        // C counts on past its 8 ACSPL_axis[] letters and reads beyond them;
        // a C defect not copied: the count stops at the last letter.
        ctrl.num_axes = (0..AXIS_LETTERS.len())
            .take_while(|n| {
                let reply = ctrl.command(&format!("?APOS({n})")).unwrap_or_default();
                !reply.is_empty() && !reply.contains('?')
            })
            .count();
        Ok(ctrl)
    }

    /// The version/identification string (`?VR`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// The command-interface mode.
    pub fn mode(&self) -> CommandMode {
        self.mode
    }

    /// Number of auto-detected axes.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    /// Send a command and return its reply (trimmed). Every command produces an
    /// acknowledgment or value, so this is used for both queries and set
    /// commands (whose reply is discarded) to keep the stream synchronized.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.command_bytes(cmd.as_bytes())
    }

    /// [`command`](Self::command) for a record string's raw bytes.
    fn command_bytes(&self, cmd: &[u8]) -> AsynResult<String> {
        // C send_mess/send_recv_mess flush the input before every write;
        // write_read is that flush, write and read as one exchange.
        let (raw, status) = self.handle.write_read(0, cmd, READ_BUF).into_parts();
        status?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_matches(['\r', '\n', '\0', ' ']).to_string())
    }

    /// A `set_status` query: the reply, or `None` for C's command error
    /// (drvSPiiPlus.cc:266-267), a failed or empty read or a `?` in the
    /// reply's C string.
    fn query(&self, cmd: &str) -> Option<String> {
        let (raw, status) = self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts();
        let c_str = raw.split(|&b| b == 0).next().unwrap_or_default();
        if status.is_err() || raw.is_empty() || c_str.contains(&b'?') {
            return None;
        }
        let text = String::from_utf8_lossy(c_str);
        Some(text.trim_matches(['\r', '\n', ' ']).to_string())
    }
}

/// Decoded motor status for one poll (mode-dependent done/limit logic).
struct Decoded {
    done: bool,
    moving: bool,
    powered: bool,
    plus_ls: bool,
    minus_ls: bool,
    direction: bool,
}

/// Decode the per-poll status from the raw query values, matching C
/// `set_status`. `done_val`/`opreq_val` are only meaningful in
/// [`CommandMode::Buffer`] (they come from `?Done`/`?opReq`).
fn decode_status(
    mode: CommandMode,
    mst: i32,
    fault: i32,
    done_val: i32,
    opreq_val: i32,
    velocity: f64,
) -> Decoded {
    let direction = velocity >= 0.0;

    let done = match mode {
        CommandMode::Buffer => done_val != 0,
        _ => (mst & MST_INPOSITION) != 0,
    };
    let moving = (mst & MST_INMOTION) != 0;
    let powered = (mst & MST_ENABLED) != 0;

    let homing =
        matches!(mode, CommandMode::Buffer) && (opreq_val == OP_HOME_F || opreq_val == OP_HOME_R);

    // A limit is asserted only when its hard OR soft fault bit is set and the
    // axis is not homing (C masks limits away while homing).
    let plus_ls = !homing && (fault & (FAULT_RL | FAULT_SRL)) != 0;
    let minus_ls = !homing && (fault & (FAULT_LL | FAULT_SLL)) != 0;

    Decoded {
        done,
        moving,
        powered,
        plus_ls,
        minus_ls,
        direction,
    }
}

/// One SPiiPlus axis sharing a controller. Implements [`AsynMotor`].
pub struct SpiiPlusAxis {
    controller: Arc<Mutex<SpiiPlusController>>,
    /// 0-based axis index (used directly on the wire).
    axis: usize,
    prev_position: i32,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// failed pass (RETRY, drvSPiiPlus.cc:276), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :278). Per axis, as the card's
    /// state can move on under another axis's poll.
    failure: PollFailure,
    last_status: MotorStatus,
    strings: MoveStrings,
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl SpiiPlusAxis {
    /// Construct axis `index` (0-based; used directly as the wire axis
    /// number) and read its status, as `motor_init` does for each axis in
    /// turn (drvSPiiPlus.cc:749).
    pub fn new(controller: Arc<Mutex<SpiiPlusController>>, index: usize) -> Self {
        let mut axis = Self {
            controller,
            axis: index,
            prev_position: 0,
            failure: PollFailure::NONE,
            last_status: MotorStatus {
                has_encoder: true,
                gain_support: true,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
            init_status: None,
        };
        axis.init_status = Some(axis.poll(&AsynUser::new(0)));
        axis
    }

    fn lock(&self) -> MutexGuard<'_, SpiiPlusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// One record transaction as devSPiiPlus.cc builds it into one message:
    /// SET_VELOCITY's `VEL` when the record sends it, SET_ACCEL's `ACC` in
    /// Buffer mode when the record sends it, PREM and `;`
    /// (devSPiiPlus.cc:187-203), then `body`. In Buffer mode `start` follows
    /// alone, as GO or JOG end the message there.
    fn transaction(
        &mut self,
        velocity: Option<f64>,
        acceleration: Option<f64>,
        body: &str,
        start: &str,
    ) -> AsynResult<()> {
        let prem = self.strings.arm();
        let ctrl = self.lock();
        let a = self.axis;
        let buffer = matches!(ctrl.mode, CommandMode::Buffer);
        let mut msg = Vec::new();
        if let Some(v) = velocity {
            msg.extend(format!("VEL{a}={};", nint(v)).into_bytes());
        }
        if let Some(acc) = acceleration.filter(|_| buffer) {
            msg.extend(format!("ACC{a}={};", nint(acc)).into_bytes());
        }
        if !prem.is_empty() {
            msg.extend(prem);
            msg.push(b';');
        }
        msg.extend(body.as_bytes());
        let sent = ctrl.command_bytes(&msg);
        if !buffer {
            return sent.map(drop);
        }
        // GO's `start` is a message of its own, sent whatever the move's
        // returned (devSPiiPlus.cc:256-266, motordrvCom.cc:397-484).
        ctrl.command(start).map(drop)
    }
}

impl Model1Motor for SpiiPlusAxis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, self.last_status.clone(), self.poll_failure())
    }
}

impl AsynMotor for SpiiPlusAxis {
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
        let a = self.axis;
        let p = nint(position);
        let body = match self.lock().mode {
            CommandMode::Buffer => {
                format!("Done({a})=0;target_pos({a})={p};opReq({a})={OP_ABS_MOVE};")
            }
            CommandMode::Connect | CommandMode::Direct => format!("ptp ({a}), {p};"),
        };
        self.transaction(
            Some(velocity),
            move_accel(acceleration),
            &body,
            &format!("start {a}, 1"),
        )
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let a = self.axis;
        let d = nint(distance);
        let body = match self.lock().mode {
            CommandMode::Buffer => {
                format!("Done({a})=0;target_pos({a})={d};opReq({a})={OP_REL_MOVE};")
            }
            CommandMode::Connect | CommandMode::Direct => format!("ptp/r ({a}), {d};"),
        };
        self.transaction(
            Some(velocity),
            move_accel(acceleration),
            &body,
            &format!("start {a}, 1"),
        )
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // The record's jog sends SET_ACCEL and JOG, no SET_VELOCITY.
        let a = self.axis;
        let v = nint(velocity);
        let body = match self.lock().mode {
            CommandMode::Buffer => {
                format!("Done({a})=0;jog_vel({a})={v}; opReq({a})={OP_JOG_MOVE};")
            }
            CommandMode::Connect | CommandMode::Direct => format!("jog/v ({a}), {v};"),
        };
        self.transaction(None, Some(acceleration), &body, &format!("start {a},1"))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // The record's home ends in GO: in Buffer mode it starts the axis
        // buffer; in Direct/Connect mode HOME adds nothing, so C sends only
        // VEL and PREM and the axis reports done in place.
        let a = self.axis;
        let body = match self.lock().mode {
            CommandMode::Buffer => {
                let op = if forward { OP_HOME_F } else { OP_HOME_R };
                format!("Done({a})=0;opReq({a})={op};")
            }
            CommandMode::Connect | CommandMode::Direct => String::new(),
        };
        self.transaction(
            Some(velocity),
            move_accel(acceleration),
            &body,
            &format!("start {a}, 1"),
        )
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let a = self.axis;
        match ctrl.mode {
            CommandMode::Buffer => {
                ctrl.command(&format!("Done({a})=0;stop_all({a})=1;"))?;
            }
            CommandMode::Connect | CommandMode::Direct => {
                ctrl.command(&format!("halt ({a});"))?;
            }
        }
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.command(&format!("set APOS({})={};", self.axis, nint(position)))?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        let a = self.axis;
        if enable {
            ctrl.command(&format!("enable({a});"))?;
        } else {
            ctrl.command(&format!("disable({a});"))?;
        }
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // C: SET_PGAIN/IGAIN/DGAIN are empty (left to the controller MMI).
        Ok(())
    }

    /// `PRIMITIVE`: INIT and a CR ahead of the port's own, so an empty line
    /// follows it on the wire (devSPiiPlus.cc:182-186).
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
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let a = self.axis;
        let mode = ctrl.mode;

        // C `queryCmnds[cmndMode]` in order, the pass ending at the first
        // failed reply (drvSPiiPlus.cc:94-98, :252-269); Buffer mode alone
        // reads opReq, then Done.
        let mut queries = vec![
            format!("?D/MST({a})"),
            format!("?D/FAULT({a})"),
            format!("?APOS({a})"),
            match mode {
                CommandMode::Connect => format!("?FPOS{}", AXIS_LETTERS[a]),
                _ => format!("?FPOS({a})"),
            },
            format!("?FVEL({a})"),
        ];
        if mode == CommandMode::Buffer {
            queries.extend([format!("?opReq({a})"), format!("?Done({a})")]);
        }
        let replies: Vec<String> = queries.iter().map_while(|q| ctrl.query(q)).collect();
        if let Some(failed) = queries.get(replies.len()) {
            (ctrl.comm_state, self.failure) = match ctrl.comm_state {
                CommState::Normal => (CommState::Retry, PollFailure::NONE),
                _ => (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR),
            };
            return Err(spiiplus_err(format!("SPiiPlus axis {a}: {failed} failed")));
        }
        ctrl.comm_state = CommState::Normal;
        let mst = atoi(&replies[0]);
        let fault = atoi(&replies[1]);
        let apos = atof(&replies[2]);
        let fpos = atof(&replies[3]);
        let velocity = atof(&replies[4]);
        let (opreq_val, done_val) = match &replies[5..] {
            [opreq, done] => (atoi(opreq), atoi(done)),
            _ => (0, 0),
        };

        let d = decode_status(mode, mst, fault, done_val, opreq_val, velocity);
        // drvSPiiPlus.cc:403-409.
        let ls_active = (d.plus_ls && d.direction) || (d.minus_ls && !d.direction);
        if (d.done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.command_bytes(&post);
        }
        drop(ctrl);

        // C reads both through `(long)atof` (drvSPiiPlus.cc:369, :386).
        let position = apos.trunc();
        self.prev_position = position as i32;

        let status = MotorStatus {
            position,
            encoder_position: fpos.trunc(),
            velocity,
            done: d.done,
            moving: d.moving,
            msta_moving: d.moving,
            high_limit: d.plus_ls,
            low_limit: d.minus_ls,
            direction: d.direction,
            powered: d.powered,
            has_encoder: true,
            // C motor_init sets GAIN_SUPPORT, so CNEN drives enable/disable.
            gain_support: true,
            ..MotorStatus::default()
        };
        self.last_status = status.clone();
        Ok(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_parse_is_case_insensitive_and_defaults_to_buffer() {
        assert!(matches!(CommandMode::parse("dir"), CommandMode::Direct));
        assert!(matches!(CommandMode::parse("DIRect"), CommandMode::Direct));
        assert!(matches!(CommandMode::parse("con"), CommandMode::Connect));
        assert!(matches!(CommandMode::parse("BUF"), CommandMode::Buffer));
        assert!(matches!(CommandMode::parse("xyz"), CommandMode::Buffer));
        assert!(matches!(CommandMode::parse(""), CommandMode::Buffer));
    }

    #[test]
    fn inmotion_sets_the_msta_moving_bit() {
        use std::sync::atomic::{AtomicI32, Ordering};
        let mst = Arc::new(AtomicI32::new(MST_INMOTION));
        let read = mst.clone();
        let (_rig, io) =
            motor_common::testing::LineRig::new("acsT80Msta", move |sent| match sent {
                "?VR" => "v1".into(),
                "?APOS(0)" => "0".into(),
                s if s.starts_with("?APOS(") => "?".into(),
                "?D/MST(0)" => read.load(Ordering::Relaxed).to_string(),
                _ => "0".into(),
            });
        let ctrl = SpiiPlusController::new(io, CommandMode::Direct).unwrap();
        let mut axis = SpiiPlusAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        let user = AsynUser::default();
        assert!(axis.poll(&user).unwrap().msta_moving);
        mst.store(MST_INPOSITION, Ordering::Relaxed);
        assert!(!axis.poll(&user).unwrap().msta_moving);
    }

    /// C reads `?APOS` and `?FPOS` through `(long)atof`, which truncates
    /// toward zero (drvSPiiPlus.cc:369, :386).
    #[test]
    fn positions_truncate_as_c_casts() {
        let (_rig, io) = motor_common::testing::LineRig::new("acsT80Trunc", |sent| match sent {
            "?VR" => "v1".into(),
            "?APOS(0)" => "12.7".into(),
            s if s.starts_with("?APOS(") => "?".into(),
            "?FPOS(0)" => "-12.7".into(),
            _ => "0".into(),
        });
        let ctrl = SpiiPlusController::new(io, CommandMode::Direct).unwrap();
        let mut axis = SpiiPlusAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        let status = axis.poll(&AsynUser::default()).unwrap();
        assert_eq!((status.position, status.encoder_position), (12.0, -12.0));
    }

    #[test]
    fn direct_done_comes_from_inposition_bit() {
        // Direct mode: done follows MST inposition (0x10), ignores done_val.
        let d = decode_status(CommandMode::Direct, MST_INPOSITION, 0, 0, 0, 0.0);
        assert!(d.done);
        let d = decode_status(CommandMode::Direct, MST_INMOTION, 0, 1, 0, 0.0);
        assert!(!d.done);
        assert!(d.moving);
    }

    #[test]
    fn buffer_done_comes_from_done_query() {
        // Buffer mode: done follows the ?Done value, not the MST bit.
        let d = decode_status(CommandMode::Buffer, 0, 0, 1, 0, 0.0);
        assert!(d.done);
        let d = decode_status(CommandMode::Buffer, MST_INPOSITION, 0, 0, 0, 0.0);
        assert!(!d.done);
    }

    #[test]
    fn limits_combine_hard_and_soft_and_are_masked_while_homing() {
        // Right hard limit -> plus LS; left soft limit -> minus LS.
        let d = decode_status(CommandMode::Direct, 0, FAULT_RL | FAULT_SLL, 0, 0, 0.0);
        assert!(d.plus_ls);
        assert!(d.minus_ls);

        // While homing (Buffer + opReq HOME), the limits are masked off.
        let d = decode_status(
            CommandMode::Buffer,
            0,
            FAULT_RL | FAULT_LL,
            0,
            OP_HOME_F,
            0.0,
        );
        assert!(!d.plus_ls);
        assert!(!d.minus_ls);
    }

    #[test]
    fn direction_follows_velocity_sign() {
        assert!(decode_status(CommandMode::Direct, 0, 0, 0, 0, 5.0).direction);
        assert!(!decode_status(CommandMode::Direct, 0, 0, 0, 0, -5.0).direction);
    }

    use motor_common::testing::LineRig;

    /// Axis 0 of a controller in `mode` on a [`LineRig`].
    fn rig(name: &str, mode: CommandMode) -> (LineRig, SpiiPlusAxis) {
        refusing_rig(name, mode, |_| false)
    }

    /// [`rig`], with the writes `refused` picks refused.
    fn refusing_rig(
        name: &str,
        mode: CommandMode,
        refused: impl Fn(&str) -> bool + Send + Sync + 'static,
    ) -> (LineRig, SpiiPlusAxis) {
        let reply = |last: &str| match last {
            "?VR" => "ACS SPiiPlus".into(),
            "?APOS(0)" => "0".into(),
            _ => String::new(),
        };
        let (line, io) = LineRig::failing(name, reply, refused);
        let ctrl = SpiiPlusController::new(io, mode).unwrap();
        let axis = SpiiPlusAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        (line, axis)
    }

    /// A controller answering every `?APOS(n)` gets one axis per
    /// `ACSPL_axis[]` letter, and its last axis polls in Connect mode.
    #[test]
    fn the_apos_count_stops_at_the_axis_letters() {
        let (_line, io) = LineRig::new("spiiAllApos", |last| match last {
            "?VR" => "ACS SPiiPlus".into(),
            l if l.starts_with("?APOS") || l.starts_with("?FPOS") => "0".into(),
            _ => "0".into(),
        });
        let ctrl = SpiiPlusController::new(io, CommandMode::Connect).unwrap();
        let num_axes = ctrl.num_axes();
        assert_eq!(num_axes, AXIS_LETTERS.len());
        let mut last = SpiiPlusAxis::new(Arc::new(Mutex::new(ctrl)), num_axes - 1);
        let _ = last.poll(&AsynUser::default());
    }

    /// motorRecord.cc:872-884: VEL, ACC, HOME, GO; GO's `start` goes alone.
    #[test]
    fn a_buffer_home_programs_and_starts_the_axis_buffer() {
        let (line, mut axis) = rig("spiiHome", CommandMode::Buffer);
        axis.home(&AsynUser::default(), 0.0, 100.0, 50.0, true)
            .unwrap();
        assert_eq!(
            line.sent(),
            ["VEL0=100;ACC0=50;Done(0)=0;opReq(0)=4;", "start 0, 1"]
        );
    }

    /// GO's `start` is a transaction of its own and motordrvCom ignores
    /// what send_mess returns (devSPiiPlus.cc:256-266, motordrvCom.cc:
    /// 397-484): a refused move message still sends the `start`.
    #[test]
    fn a_refused_move_message_still_sends_the_start() {
        let (line, mut axis) = refusing_rig("spiiRefused", CommandMode::Buffer, |cmd| {
            cmd.starts_with("VEL0=") || cmd == "start 0,1"
        });
        let user = AsynUser::default();
        axis.move_absolute(&user, 7.0, 0.0, 100.0, 0.0).unwrap();
        assert_eq!(
            line.sent(),
            [
                "VEL0=100;Done(0)=0;target_pos(0)=7;opReq(0)=1;",
                "start 0, 1"
            ]
        );
        assert!(axis.move_velocity(&user, 0.0, 10.0, 0.0).is_err());
        assert_eq!(
            line.sent(),
            ["ACC0=0;Done(0)=0;jog_vel(0)=10; opReq(0)=3;", "start 0,1"]
        );
    }

    /// The record sends no SET_ACCEL for a zero acceleration.
    #[test]
    fn a_zero_acceleration_sends_no_acc() {
        let (line, mut axis) = rig("spiiNoAcc", CommandMode::Buffer);
        axis.move_absolute(&AsynUser::default(), 7.0, 0.0, 100.0, 0.0)
            .unwrap();
        assert_eq!(
            line.sent(),
            [
                "VEL0=100;Done(0)=0;target_pos(0)=7;opReq(0)=1;",
                "start 0, 1"
            ]
        );
    }

    /// SET_ACCEL ahead of the jog in Buffer mode; none in Direct mode.
    #[test]
    fn a_jog_carries_the_records_acc_in_buffer_mode_only() {
        let (line, mut axis) = rig("spiiJogBuf", CommandMode::Buffer);
        axis.move_velocity(&AsynUser::default(), 0.0, -20.0, 5.0)
            .unwrap();
        assert_eq!(
            line.sent(),
            ["ACC0=5;Done(0)=0;jog_vel(0)=-20; opReq(0)=3;", "start 0,1"]
        );
        let (line, mut axis) = rig("spiiJogDir", CommandMode::Direct);
        axis.move_velocity(&AsynUser::default(), 0.0, -20.0, 5.0)
            .unwrap();
        assert_eq!(line.sent(), ["jog/v (0), -20;"]);
    }

    /// motorRecord.cc:2141, :3068: a jog's SET_ACCEL goes out at any rate,
    /// a move's only at a positive one (:874).
    #[test]
    fn a_jog_sends_acc_at_a_zero_rate() {
        let (line, mut axis) = rig("spiiJogZeroAcc", CommandMode::Buffer);
        axis.move_velocity(&AsynUser::default(), 0.0, -20.0, 0.0)
            .unwrap();
        assert_eq!(
            line.sent(),
            ["ACC0=0;Done(0)=0;jog_vel(0)=-20; opReq(0)=3;", "start 0,1"]
        );
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// devSPiiPlus.cc:182-186.
    #[test]
    fn init_goes_out_alone() {
        let (line, mut axis) = rig("spiiInit", CommandMode::Direct);
        axis.primitive(&AsynUser::default(), b"enable(0)").unwrap();
        assert_eq!(line.sent(), ["enable(0)\r"]);
    }

    /// devSPiiPlus.cc:187-203 and drvSPiiPlus.cc:403-409: PREM after the
    /// record's VEL and ACC, POST once at the next done poll.
    #[test]
    fn prem_joins_the_move_message_and_post_follows_done() {
        use std::sync::atomic::{AtomicBool, Ordering};

        let done = Arc::new(AtomicBool::new(false));
        let d = done.clone();
        let (line, io) = LineRig::new("spiiPrem", move |last| match last {
            "?VR" => "ACS SPiiPlus".into(),
            "?APOS(0)" => "0".into(),
            "?Done(0)" => u8::from(d.load(Ordering::SeqCst)).to_string(),
            _ => "0".into(),
        });
        let ctrl = SpiiPlusController::new(io, CommandMode::Buffer).unwrap();
        let mut axis = SpiiPlusAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        let user = AsynUser::default();
        let polled = |line: &LineRig| {
            line.sent()
                .into_iter()
                .filter(|l| !l.starts_with('?'))
                .collect::<Vec<_>>()
        };

        axis.set_move_strings(&user, b"P=1", Some(post(b"P=0")))
            .unwrap();
        axis.move_relative(&user, 5.0, 0.0, 100.0, 50.0).unwrap();
        assert_eq!(
            line.sent(),
            [
                "VEL0=100;ACC0=50;P=1;Done(0)=0;target_pos(0)=5;opReq(0)=2;",
                "start 0, 1"
            ]
        );
        axis.poll(&user).unwrap();
        assert!(polled(&line).is_empty());
        done.store(true, Ordering::SeqCst);
        axis.poll(&user).unwrap();
        assert_eq!(polled(&line), ["P=0"]);
        axis.poll(&user).unwrap();
        assert!(polled(&line).is_empty());
    }

    /// drvSPiiPlus.cc:252-298: a reply holding `?` ends the pass, which is
    /// RETRY (nothing raised, nothing posted) on a NORMAL card, COMM_ERR
    /// after, and a full pass clears it.
    #[test]
    fn a_question_mark_reply_is_retry_then_comm_error() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let refuse = Arc::new(AtomicBool::new(false));
        let r = refuse.clone();
        let (line, io) = LineRig::new("spiiRetry", move |last| match last {
            "?VR" => "ACS SPiiPlus".into(),
            "?D/FAULT(0)" if r.load(Ordering::SeqCst) => "?1004".into(),
            _ => "0".into(),
        });
        let ctrl = SpiiPlusController::new(io, CommandMode::Direct).unwrap();
        let mut axis = SpiiPlusAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        let user = AsynUser::default();
        refuse.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(line.sent(), ["?D/MST(0)", "?D/FAULT(0)"]);
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        refuse.store(false, Ordering::SeqCst);
        line.sent();
        let s = axis.poll(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        assert_eq!(
            line.sent(),
            [
                "?D/MST(0)",
                "?D/FAULT(0)",
                "?APOS(0)",
                "?FPOS(0)",
                "?FVEL(0)"
            ]
        );
        refuse.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }

    /// drvSPiiPlus.cc:282-295 exit stores what motor_init prepared
    /// (:727-737, EA_PRESENT and GAIN_SUPPORT), with both COMM_ERR bits once
    /// past RETRY.
    #[test]
    fn a_failed_init_read_keeps_the_encoder_bits() {
        let (_line, io) = LineRig::new("spiiInitFail", |last| match last {
            "?VR" => "ACS SPiiPlus".into(),
            _ => String::new(),
        });
        let ctrl = Arc::new(Mutex::new(
            SpiiPlusController::new(io, CommandMode::Buffer).unwrap(),
        ));
        let mut first = SpiiPlusAxis::new(ctrl.clone(), 0);
        let mut second = SpiiPlusAxis::new(ctrl, 1);
        let prepared = MotorStatus {
            has_encoder: true,
            gain_support: true,
            ..MotorStatus::default()
        };
        assert_eq!(first.take_init_status(), prepared);
        assert_eq!(
            second.take_init_status(),
            MotorStatus {
                problem: true,
                comms_error: true,
                ..prepared
            }
        );
    }

    /// devSPiiPlus.cc: Direct-mode HOME builds nothing, so the home is the
    /// record's VEL and PREM alone, and succeeds.
    #[test]
    fn a_direct_home_sends_vel_and_prem_only() {
        let (line, mut axis) = rig("spiiHomeDir", CommandMode::Direct);
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"P=1", None).unwrap();
        axis.home(&user, 0.0, 100.0, 50.0, false).unwrap();
        assert_eq!(line.sent(), ["VEL0=100;P=1;"]);
    }
}
