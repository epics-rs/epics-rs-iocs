//! ACS Tech80 SPiiPlus motion controller driver (serial/TCP ASCII, ACSPL+).
//!
//! Ported from `motorAcsTech80/acsTech80App/src/drvSPiiPlus.cc` +
//! `devSPiiPlus.cc` (the model-1 dev/drv pair). Commands are ACSPL+ text
//! terminated by CR (`\r`, the driver owns output framing; the startup script
//! sets only the input EOS). Query commands begin with `?` and return a value;
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
//! boundary is counts: positions pass through with `NINT` rounding, the record's
//! `MRES` is 1, and its `EGU` is counts.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the per-mode query set runs
//!   inside [`poll`](AsynMotor::poll).
//! - The C `set_status` velocity readback is marked `NEEDS WORK` and reports the
//!   feedback velocity `?FVEL`; this port reports the same.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 120;

/// Command terminator (C `ACS_EOS`); the driver owns output framing.
const TERMINATOR: &[u8] = b"\r";

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
        };

        // Probe the controller (retry up to 3 times), then stop all motion.
        let mut probed = false;
        for _ in 0..3 {
            let reply = ctrl.command("?VR")?;
            if !reply.is_empty() && !reply.contains('?') {
                probed = true;
                break;
            }
        }
        if !probed {
            return Err(spiiplus_err("SPiiPlus: no valid response to ?VR probe"));
        }
        ctrl.command("halt all")?;
        ctrl.ident = ctrl.command("?VR")?;

        // Auto-detect the axis count: ?APOS(n) succeeds until n is out of range.
        let mut n = 0usize;
        loop {
            let reply = ctrl.command(&format!("?APOS({n})"))?;
            if reply.is_empty() || reply.contains('?') {
                break;
            }
            n += 1;
            if n > AXIS_LETTERS.len() {
                break;
            }
        }
        if n == 0 {
            return Err(spiiplus_err("SPiiPlus: no axes detected (?APOS(0) failed)"));
        }
        ctrl.num_axes = n;
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

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Send a command and return its reply (trimmed). Every command produces an
    /// acknowledgment or value, so this is used for both queries and set
    /// commands (whose reply is discarded) to keep the stream synchronized.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_matches(['\r', '\n', '\0', ' ']).to_string())
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
    last_status: MotorStatus,
}

impl SpiiPlusAxis {
    /// Construct axis `index` (0-based; used directly as the wire axis number).
    pub fn new(controller: Arc<Mutex<SpiiPlusController>>, index: usize) -> Self {
        Self {
            controller,
            axis: index,
            prev_position: 0,
            last_status: MotorStatus {
                done: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
        }
    }

    fn lock(&self) -> MutexGuard<'_, SpiiPlusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for SpiiPlusAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        let a = self.axis;
        let v = nint(velocity);
        let p = nint(position);
        match ctrl.mode {
            CommandMode::Buffer => {
                let acc = nint(acceleration);
                ctrl.command(&format!(
                    "VEL{a}={v};ACC{a}={acc};Done({a})=0;target_pos({a})={p};opReq({a})={OP_ABS_MOVE};"
                ))?;
                ctrl.command(&format!("start {a}, 1"))?;
            }
            CommandMode::Connect | CommandMode::Direct => {
                ctrl.command(&format!("VEL{a}={v};ptp ({a}), {p};"))?;
            }
        }
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
        let ctrl = self.lock();
        let a = self.axis;
        let v = nint(velocity);
        let d = nint(distance);
        match ctrl.mode {
            CommandMode::Buffer => {
                let acc = nint(acceleration);
                ctrl.command(&format!(
                    "VEL{a}={v};ACC{a}={acc};Done({a})=0;target_pos({a})={d};opReq({a})={OP_REL_MOVE};"
                ))?;
                ctrl.command(&format!("start {a}, 1"))?;
            }
            CommandMode::Connect | CommandMode::Direct => {
                ctrl.command(&format!("VEL{a}={v};ptp/r ({a}), {d};"))?;
            }
        }
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        let a = self.axis;
        let v = nint(velocity);
        match ctrl.mode {
            CommandMode::Buffer => {
                ctrl.command(&format!(
                    "Done({a})=0;jog_vel({a})={v}; opReq({a})={OP_JOG_MOVE};"
                ))?;
                ctrl.command(&format!("start {a},1"))?;
            }
            CommandMode::Connect | CommandMode::Direct => {
                ctrl.command(&format!("jog/v ({a}), {v};"))?;
            }
        }
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        let a = self.axis;
        match ctrl.mode {
            CommandMode::Buffer => {
                let op = if forward { OP_HOME_F } else { OP_HOME_R };
                ctrl.command(&format!("Done({a})=0;opReq({a})={op};"))?;
                Ok(())
            }
            CommandMode::Connect | CommandMode::Direct => Err(spiiplus_err(
                "SPiiPlus: homing is only available in Buffer command mode",
            )),
        }
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

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let a = self.axis;
        let mode = ctrl.mode;

        let mst = atoi(&ctrl.command(&format!("?D/MST({a})"))?);
        let fault = atoi(&ctrl.command(&format!("?D/FAULT({a})"))?);
        let apos = atof(&ctrl.command(&format!("?APOS({a})"))?);
        let fpos_reply = match mode {
            CommandMode::Connect => ctrl.command(&format!("?FPOS{}", AXIS_LETTERS[a]))?,
            _ => ctrl.command(&format!("?FPOS({a})"))?,
        };
        let fpos = atof(&fpos_reply);
        let velocity = atof(&ctrl.command(&format!("?FVEL({a})"))?);

        // Done and the home request are only read back in Buffer mode.
        let (done_val, opreq_val) = match mode {
            CommandMode::Buffer => (
                atoi(&ctrl.command(&format!("?Done({a})"))?),
                atoi(&ctrl.command(&format!("?opReq({a})"))?),
            ),
            _ => (0, 0),
        };
        drop(ctrl);

        let d = decode_status(mode, mst, fault, done_val, opreq_val, velocity);

        let position = nint(apos);
        self.prev_position = position;

        let status = MotorStatus {
            position: position as f64,
            encoder_position: nint(fpos) as f64,
            velocity,
            done: d.done,
            moving: d.moving,
            high_limit: d.plus_ls,
            low_limit: d.minus_ls,
            direction: d.direction,
            powered: d.powered,
            has_encoder: true,
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
}
