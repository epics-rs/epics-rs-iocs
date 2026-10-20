//! PI (Physik Instrumente) E-516 digital piezo/nano-positioning controller
//! (`motorPI`).
//!
//! Ported from `drvPIE516.cc` + `devPIE516.cc`. Unlike the C-862 DC-motor
//! path, the E-516 is a **closed-loop piezo** controller: no limit switches,
//! no encoder-count positions — positions are floating-point micrometres and
//! "done" means *on-target* (`ONT?`), not a trajectory-complete status bit.
//! `devPIE516_init_record` sets `NTM = No` (closed-loop, no
//! change-of-direction testing).
//!
//! ## Multi-axis
//! One controller drives up to `MAX_AXES = 3` axes, addressed by the letters
//! `A`/`B`/`C`. Each command carries the axis letter (C `send_mess` substitutes
//! it for the `#` placeholder in the command template); this port builds each
//! command with the letter already in place. The axis count is **probed at
//! connect time** exactly like C `motor_init` (send `POS? {axis}` for each
//! candidate letter, stop at the first that does not reply), so a configured
//! controller installs one motor device support per responding axis
//! (`PIE516_{card}_{axis}`, axis = 0..n).
//!
//! ## Addressing
//! The iocsh `addr` ("asyn address (GPIB)") argument is **vestigial**: C
//! `motor_init`'s `pasynOctetSyncIO->connect(cntrl->asyn_port, 0, ...)`
//! hardcodes the asyn sub-address to 0 and nothing ever reads `asyn_address`
//! on the wire. Axis selection is by the per-command `A`/`B`/`C` letter, not a
//! bus address. The argument is accepted for signature parity and ignored.
//!
//! ## Wire shape — the port owns framing (EOS)
//! C `motor_init` sets **both** `setOutputEos(pasynUser, "\n", 1)` and
//! `setInputEos(pasynUser, "\n", 1)` right after connecting — the port owns
//! the terminator (the `motor-port-eos-ownership` convention; here the C picks
//! port-owned framing, same class as the C-862 but with `"\n"` both ways).
//! `PIE516Config` sets both through `connect_with_eos` and
//! `PIE516Controller::send` writes **bare** command bytes.
//!
//! **Documented deviation — the redundant trailing terminator.** C
//! `devPIE516_build_trans` appends `EOL_E516` (`"\n"`) to every *motion*
//! command string (`strcat(buff, EOL_E516)`) and then the asyn output-EOS
//! layer appends *another* `"\n"` at write time, so the C wire for a move is
//! `MOV A1.000\n\n` — a real command followed by an empty one (harmless: the
//! controller's multi-command delimiter is `"\n"`, so the trailing empty
//! command is ignored). Status queries (`ONL?`/`ONT?`/…) do **not** append the
//! extra `EOL` and go out singly-terminated. This port emits every command
//! singly-terminated via the port's output EOS (no manual `"\n"`), collapsing
//! the harmless double; functionally identical wire, one fewer empty command.
//!
//! ## Connect handshake (C `motor_init` per card)
//! 1. Assure ONLINE: up to 3 iterations of `ONL 1` then `ONL?`; parse the
//!    reply as `atoi == 1`. C's `do/while (online==false && retry<3)` only
//!    increments `retry` on an *empty* read, so a controller that keeps
//!    replying a non-`1` value would spin forever; this port bounds the loop
//!    to 3 total iterations (documented deviation — avoids a startup hang) and
//!    errors out of `new` if the controller never reports online (C nulls
//!    `motor_state[card]` in that case).
//! 2. `VER?` identify → stored as `ident` (the `version >= 311` /
//!    `versionSupport` flag C sets is never read — computed-and-dropped, so
//!    not tracked).
//! 3. Probe axes 0..3 via `POS? {axis}`.
//! 4. `VCO A1 B1 C1` (velocity-control mode ON — required for the `ONT?`
//!    on-target reading to be meaningful).
//!
//! ## Status poll (C `set_status`)
//! Per axis, in order, each gated on a successful read + integer parse
//! (`recv_mess && sscanf("%d")`; any failure drops to the comm-debounce path):
//! `ONL?` (→ if `0`, re-send `ONL 1` + `VCO A1 B1 C1`), `ONT? {axis}`,
//! `OVF? {axis}`, `SVO? {axis}`, then `POS? {axis}` (only non-empty required;
//! `atof` of junk → `0`). Decoded:
//! - `RA_DONE` = `ontarget != 0`; `RA_HOME = RA_DONE`.
//! - `EA_POSITION` (→ `powered`) = `servo != 0`.
//! - `RA_PLUS_LS` = `RA_DONE && overflow != 0` (a servo overflow while
//!   on-target is surfaced as a `+` limit); `RA_MINUS_LS` is always 0 (the
//!   E-516 has no real limit switches).
//! - `RA_DIRECTION` is updated only when the freshly read position differs
//!   from the previous one (`motorData >= prev ? + : -`); it persists across a
//!   no-motion poll. Tracked per axis.
//! - `motorData = NINT(atof(reply) / POS_RES)` — the reply is micrometres, the
//!   record works in `POS_RES`-sized steps, so pair with `MRES = 1`.
//! - `velocity` is always reported 0 (C only sign-flips an otherwise-unset 0).
//!
//! The `NORMAL`/`RETRY`/`COMM_ERR` two-strike debounce is per **controller**
//! (`cntrl->status` is one field shared by all axes in C); a first failed poll
//! fails the poll raising nothing, a second consecutive one flags
//! `comms_error`/`problem`.
//!
//! ## Motion (C `devPIE516_build_trans`)
//! A move issues `SET_VELOCITY` then the move as two writes, mirroring the
//! record's model-1 transaction (`SET_ACCEL` is a no-op — the E-516 has no
//! acceleration command — and `SET_VEL_BASE`/`GO` are no-ops too):
//! - `MOVE_ABS`/`MOVE_REL` → `VEL {axis}{v·res}` then `MOV {axis}{p·res}` /
//!   `MVR {axis}{d·res}`, each `%.3f` (`maxdigits = 3`). Note `SET_VELOCITY`
//!   scales by `res`, but `JOG` below does **not** — a genuine C asymmetry.
//! - `JOG` (`move_velocity`) → `VEL {axis}{velocity}` (raw, **no** `res`
//!   scaling), signed; no move command follows (velocity-control mode moves on
//!   the velocity setpoint).
//! - `HOME_FOR`/`HOME_REV` → C returns `ERROR` (no home command); `home()`
//!   errors.
//! - `LOAD_POS` (`set_position`) → C returns `ERROR`; `set_position()` errors.
//! - `ENABLE_TORQUE`/`DISABL_TORQUE` (`set_closed_loop`) → `SVO {axis}1` /
//!   `SVO {axis}0`.
//! - `STOP_AXIS` (`stop`) → `STP {axis}`.
//! - `SET_PGAIN`/`SET_IGAIN`/`SET_DGAIN`, `SET_HIGH_LIMIT`/`SET_LOW_LIMIT`,
//!   `SET_ENC_RATIO` → no wire command (`send = false`).
//!
//! ## Not modeled (documented deviations)
//! - **`recv_mess(..., FLUSH)`** before each poll and before retry: replaced
//!   by a flush before every query (`exchange`).
//! - **`no_motion_count`** / poll-rate scheduling: not tracked (position is
//!   updated unconditionally from `POS?`).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, Model1Motor, MotorStatus, PollFailure, PostMessage,
};
use epics_rs::asyn::request::OctetRead;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::{MoveStrings, init_poll, init_read};
use motor_common::util::{atof, atoi, nint};

use crate::scan_int;

/// Response buffer size for a single reply (C `BUFF_SIZE`).
const READ_BUF: usize = 100;
/// Position resolution, micrometres per step (C `drvPIE516.h` `POS_RES`).
const POS_RES: f64 = 0.001;
/// Maximum axes per controller (C `MAX_AXES`).
const MAX_AXES: usize = 3;
/// Per-axis command letters (C `PIE516_axis[]`).
const AXIS_LABELS: [&str; MAX_AXES] = ["A", "B", "C"];
/// Command decimal places (C `devPIE516_build_trans` `maxdigits`).
const MAX_DIGITS: usize = 3;

fn pie516_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Controller communication state (C `cntrl->status`): one failed poll is
/// retried silently; a second consecutive one is a hard comm error. Shared by
/// all axes of a controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Raw per-axis readings from one successful `ONT?`/`OVF?`/`SVO?`/`POS?`
/// exchange (direction is computed by the axis, which owns the previous
/// position).
struct AxisReading {
    /// `ONT?` on-target flag (→ `RA_DONE`).
    ontarget: bool,
    /// `OVF?` servo-overflow flag (→ `+` limit when on-target).
    overflow: bool,
    /// `SVO?` servo-enable flag (→ `EA_POSITION`/`powered`).
    servo: bool,
    /// `POS?` position in `POS_RES` steps (`NINT(atof(reply) / POS_RES)`).
    position: i32,
}

/// `VEL {label}{value}` fragment. `SET_VELOCITY` scales the record value by
/// `res`; `JOG` passes it raw — the caller supplies the already-scaled value.
fn vel_cmd(label: &str, value: f64) -> String {
    format!("VEL {label}{:.*}", MAX_DIGITS, CFloat(value))
}

/// `MOV {label}{p·res}` absolute-move fragment.
fn mov_cmd(label: &str, position: f64) -> String {
    format!("MOV {label}{:.*}", MAX_DIGITS, CFloat(position * POS_RES))
}

/// `MVR {label}{d·res}` relative-move fragment.
fn mvr_cmd(label: &str, distance: f64) -> String {
    format!("MVR {label}{:.*}", MAX_DIGITS, CFloat(distance * POS_RES))
}

/// Fold one raw reading into a [`MotorStatus`] (C `set_status` tail). Updates
/// `prev_position` and returns the new direction folded in; `last_direction`
/// persists when the position did not change (C only reassigns `RA_DIRECTION`
/// inside the `motorData != motor_info->position` branch).
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
        msta_moving: false,
        // + limit only surfaces a servo overflow while on-target; no - LS.
        high_limit: done && reading.overflow,
        low_limit: false,
        home: done, // C: RA_HOME = RA_DONE.
        encoder_home: false,
        powered: reading.servo, // C: EA_POSITION = servo ? 1 : 0.
        problem: false,
        direction,
        slip_stall: false,
        slip: false,
        comms_error: false,
        homed: false,
        gain_support: true,
        has_encoder: true,
        vbas_supported: false,
    }
}

/// Shared controller endpoint: owns the octet handle and the (per-controller)
/// comm debounce state, plus the probed axis count.
pub struct PIE516Controller {
    handle: SyncIOHandle,
    ident: String,
    comm_state: CommState,
    num_axes: usize,
}

impl PIE516Controller {
    /// Connect, assure ONLINE, identify, probe axes, and turn on
    /// velocity-control mode (C `motor_init`'s per-card block). Performs
    /// blocking octet I/O. Errors if the controller never reports online (C
    /// nulls `motor_state[card]` in that case).
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            comm_state: CommState::Normal,
            num_axes: 0,
        };

        // 1. Assure ONLINE (bounded to 3 iterations — see module docs).
        let mut online = false;
        for _ in 0..3 {
            let _ = ctrl.send("ONL 1");
            let reply = ctrl.exchange("ONL?");
            if !reply.is_empty() {
                online = atoi(&reply) == 1;
            }
            if online {
                break;
            }
        }
        if !online {
            return Err(pie516_err(
                "PIE516: controller did not report ONLINE (ONL?) after 3 attempts",
            ));
        }

        // 2. Identify.
        ctrl.ident = ctrl.exchange("VER?");

        // 3. Probe axes: POS? for each candidate letter, stop at first silence.
        let mut num_axes = 0;
        for label in AXIS_LABELS {
            if ctrl.exchange(&format!("POS? {label}")).is_empty() {
                break;
            }
            num_axes += 1;
        }
        ctrl.num_axes = num_axes;

        // 4. Velocity-control mode ON (all axes). Both sets are bare
        // send_mess in C (drvPIE516.cc:588, 628).
        let _ = ctrl.send("VCO A1 B1 C1");

        Ok(ctrl)
    }

    /// The controller identification string (from the connect-time `VER?`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes that responded to the connect-time `POS?` probe.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    /// Write a command, no reply expected (C `send_mess`). No terminator is
    /// appended — the port's configured output EOS puts the `"\n"` on the wire.
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
    /// stale input. C flushes only at `set_status` (drvPIE516.cc:243), so a late reply
    /// shifts its answers until the next poll: a C defect, not copied.
    fn exchange(&self, cmd: &str) -> String {
        Self::reply(self.handle.write_read(0, cmd.as_bytes(), READ_BUF))
    }

    /// Read one reply (the port's input EOS already stripped the `"\n"`). Any
    /// transport failure or empty read folds into `""`, matching C
    /// `recv_mess`'s swallow of a bad read into an empty string.
    fn reply(read: OctetRead) -> String {
        match read.into_parts() {
            (raw, Ok(())) if !raw.is_empty() => String::from_utf8_lossy(&raw).into_owned(),
            _ => String::new(),
        }
    }

    /// C `set_status` I/O for one axis, wrapped in the comm debounce. `None`
    /// means the caller fails the poll, raising nothing (RETRY) or a hard
    /// comm error, decided by [`Self::comm_state`] after the call.
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

    /// The raw `set_status` read chain; `None` on the first failed read/parse
    /// (C's nested `recv_mess && sscanf` short-circuit).
    fn read_axis(&mut self, axis: usize) -> Option<AxisReading> {
        let label = AXIS_LABELS[axis];

        // ONLINE gate: a non-1 reply re-asserts ONLINE + velocity-control mode.
        let online = scan_int(&self.exchange("ONL?"))?;
        if online == 0 {
            self.send("ONL 1").ok()?;
            self.send("VCO A1 B1 C1").ok()?;
        }

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

/// One axis of an E-516 controller. Implements [`AsynMotor`]. Holds the shared
/// controller, its command letter, and the per-axis previous position (for the
/// direction bit, which persists across no-motion polls).
pub struct PIE516Axis {
    controller: Arc<Mutex<PIE516Controller>>,
    axis: usize,
    prev_position: i32,
    /// What the last failed poll raises: nothing on C `set_status`'s
    /// first missed reply (RETRY, drvPIE516.cc:285), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :291). Per axis, as the
    /// controller's state can move on under another axis's poll.
    failure: PollFailure,
    last_status: MotorStatus,
    strings: MoveStrings,
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl PIE516Axis {
    /// Construct one axis and read its status, as C `motor_init`'s
    /// `set_status` does (drvPIE516.cc:648). `prev_position` starts at 0 (C
    /// `motor_info->position = 0`).
    pub fn new(controller: Arc<Mutex<PIE516Controller>>, axis: usize) -> Self {
        let mut me = Self {
            controller,
            axis,
            prev_position: 0,
            failure: PollFailure::NONE,
            last_status: MotorStatus {
                gain_support: true,
                has_encoder: true,
                vbas_supported: false,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
            init_status: None,
        };
        me.init_status = Some(init_poll(&mut me, &AsynUser::new(0)));
        me
    }

    fn label(&self) -> &'static str {
        AXIS_LABELS[self.axis]
    }

    fn lock(&self) -> MutexGuard<'_, PIE516Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A record string with its first `#` replaced by the axis letter, as
    /// C's `send_mess` addresses a message (drvPIE516.cc:416).
    fn addressed(&self, text: &[u8]) -> Vec<u8> {
        let mut text = text.to_vec();
        if let Some(b) = text.iter_mut().find(|b| **b == b'#') {
            *b = self.label().as_bytes()[0];
        }
        text
    }

    /// A move's own message: PREM and an `EOL_E516` ahead of its command
    /// (devPIE516.cc:196-205), whose POST waits for the move to end. C
    /// addresses only the first `#` of the whole message, so a PREM holding
    /// one leaves the command's `#` on the wire; not copied.
    fn move_line(&mut self, command: &str) -> Vec<u8> {
        let prem = self.strings.arm();
        let mut line = self.addressed(&prem);
        if !line.is_empty() {
            line.push(b'\n');
        }
        line.extend_from_slice(command.as_bytes());
        line
    }
}

impl Model1Motor for PIE516Axis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, self.last_status.clone(), self.poll_failure())
    }
}

impl AsynMotor for PIE516Axis {
    // devPIE516.cc:127 forces NTM to NO.
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
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let label = self.label();
        let line = self.move_line(&mov_cmd(label, position));
        let ctrl = self.lock();
        // motordrvCom sends each message whatever the one before returned
        // (motordrvCom.cc:397-484).
        let _ = ctrl.send(&vel_cmd(label, velocity * POS_RES));
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
        let _ = ctrl.send(&vel_cmd(label, velocity * POS_RES));
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
        // C `devPIE516_build_trans` HOME_FOR/HOME_REV → ERROR (no home
        // command), after the message already took PREM and its EOL,
        // which C still sends; POST follows the next done poll.
        let line = self.move_line("");
        self.lock().send_bytes(&line)?;
        Err(pie516_err("PIE516: homing is not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let cmd = format!("STP {}", self.label());
        self.lock().send(&cmd)
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C `devPIE516_build_trans` LOAD_POS → ERROR.
        Err(pie516_err(
            "PIE516: set-position (LOAD_POS) is not supported",
        ))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let cmd = format!("SVO {}{}", self.label(), if enable { 1 } else { 0 });
        self.lock().send(&cmd)
    }

    /// `PRIMITIVE`: INIT as its own message, its first `#` addressed
    /// (devPIE516.cc:188-189).
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

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
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
            return Err(pie516_err("no status reply"));
        };
        let last_direction = self.last_status.direction;
        self.last_status = fold_reading(&mut self.prev_position, last_direction, &reading);
        // drvPIE516.cc:326-372: POST, unaddressed, once done or stopped by a
        // limit in the move's direction.
        let s = &self.last_status;
        let ls_active = (s.high_limit && s.direction) || (s.low_limit && !s.direction);
        if (s.done || ls_active)
            && let Some(post) = self.strings.post_due()
        {
            let _ = ctrl.send_bytes(&post);
        }
        drop(ctrl);
        *status = self.last_status.clone();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;
    use motor_common::testing::Polled;
    use std::sync::atomic::{AtomicU8, Ordering};

    /// C sprintf's `%.*f` and `%+.*f` spell a NaN as glibc does: `nan`,
    /// `-nan`, `+nan`.
    #[test]
    fn floats_go_out_as_c_sprintf() {
        let nan = f64::NAN;
        let neg = -f64::NAN;
        assert_eq!(vel_cmd("A", nan), "VEL Anan");
        assert_eq!(mov_cmd("A", neg), "MOV A-nan");
        assert_eq!(mvr_cmd("A", nan), "MVR Anan");
    }

    const QUERIES: [&str; 5] = ["ONL?", "ONT? A", "OVF? A", "SVO? A", "POS? A"];

    /// Axis A of a one-axis E-516 on a [`LineRig`] whose `ONT? A` reply
    /// follows `ont`.
    fn strings_rig(name: &str) -> (LineRig, PIE516Axis, Arc<AtomicU8>) {
        refusing_rig(name, |_| false)
    }

    /// [`strings_rig`] with each write `refused` picks recorded and refused.
    fn refusing_rig(name: &str, refused: fn(&str) -> bool) -> (LineRig, PIE516Axis, Arc<AtomicU8>) {
        let ont = Arc::new(AtomicU8::new(1));
        let o = ont.clone();
        let (line, io) = LineRig::failing(
            name,
            move |last| match last {
                "ONL?" | "SVO? A" => "1".into(),
                "VER?" => "E-516".into(),
                "POS? A" => "0.000".into(),
                "ONT? A" => o.load(Ordering::SeqCst).to_string(),
                "OVF? A" => "0".into(),
                _ => String::new(),
            },
            refused,
        );
        let ctrl = PIE516Controller::new(io).unwrap();
        let axis = PIE516Axis::new(Arc::new(Mutex::new(ctrl)), 0);
        line.sent();
        (line, axis, ont)
    }

    /// The velocity and the move are each a message of their own, whose
    /// status C's motor task drops.
    #[test]
    fn a_refused_velocity_still_sends_the_move() {
        let (line, mut axis, _) = refusing_rig("e516Refused", |cmd| cmd.starts_with("VEL A"));
        let user = AsynUser::default();
        axis.move_absolute(&user, 1.0, 0.0, 2.0, 0.0).unwrap();
        axis.move_relative(&user, -1.0, 0.0, 2.0, 0.0).unwrap();
        let sent: Vec<String> = line.sent().iter().map(|c| c[..3].to_string()).collect();
        assert_eq!(sent, ["VEL", "MOV", "VEL", "MVR"]);
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut PIE516Axis) -> Vec<String> {
        axis.polled(&AsynUser::default()).unwrap();
        line.sent()
            .into_iter()
            .filter(|l| !QUERIES.contains(&l.as_str()))
            .collect()
    }

    /// devPIE516.cc:188-189: INIT as its own message, its first `#`
    /// addressed.
    #[test]
    fn init_goes_out_as_its_own_message() {
        let (line, mut axis, _) = strings_rig("pie516Init");
        axis.primitive(&AsynUser::default(), b"SVO #1").unwrap();
        assert_eq!(line.sent(), ["SVO A1"]);
    }

    /// devPIE516.cc:196-205: PREM and a LF ahead of the move's own command,
    /// for that move only; the velocity goes ahead as its own message.
    #[test]
    fn prem_goes_ahead_of_the_move() {
        let (line, mut axis, _) = strings_rig("pie516Prem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"SVO #1", None).unwrap();
        axis.move_absolute(&user, 1000.0, 0.0, 1000.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["VEL A1.000", "SVO A1\nMOV A1.000"]);
        axis.move_relative(&user, 1000.0, 0.0, 1000.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["VEL A1.000", "MVR A1.000"]);
        axis.set_move_strings(&user, b"SVO #1", None).unwrap();
        axis.move_velocity(&user, 0.0, -2.0, 0.0).unwrap();
        assert_eq!(line.sent(), ["SVO A1\nVEL A-2.000"]);
        axis.set_move_strings(&user, b"SVO #1", None).unwrap();
        // HOME builds no command, but its PREM keeps the EOL.
        assert!(axis.home(&user, 0.0, 1.0, 0.0, true).is_err());
        assert_eq!(line.sent(), ["SVO A1\n"]);
    }

    /// drvPIE516.cc:365-372: POST, unaddressed, once the move is done, not
    /// while it runs, and only once.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, ont) = strings_rig("pie516Post");
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
    fn move_commands_scale_by_res_and_use_three_decimals() {
        // SET_VELOCITY and MOVE_ABS both scale the record value by POS_RES.
        assert_eq!(mov_cmd("A", 1000.0), "MOV A1.000");
        assert_eq!(mov_cmd("B", -500.0), "MOV B-0.500");
        assert_eq!(mvr_cmd("C", 250.0), "MVR C0.250");
        // vel_cmd takes the already-scaled value; a move scales it here.
        assert_eq!(vel_cmd("A", 1000.0 * POS_RES), "VEL A1.000");
    }

    #[test]
    fn jog_velocity_is_raw_not_res_scaled() {
        // C JOG uses cntrl_units directly (no `* res`), unlike SET_VELOCITY.
        assert_eq!(vel_cmd("A", 5.0), "VEL A5.000");
        assert_eq!(vel_cmd("B", -2.5), "VEL B-2.500");
    }

    #[test]
    fn position_reply_converts_um_to_steps() {
        // motorData = NINT(atof(reply) / POS_RES); reply is micrometres.
        assert_eq!(nint(atof("1.234") / POS_RES), 1234);
        assert_eq!(nint(atof("-0.500") / POS_RES), -500);
        assert_eq!(nint(atof("junk") / POS_RES), 0);
    }

    #[test]
    fn scan_int_gates_on_a_parseable_integer() {
        // C `sscanf(buff, "%d", &x)` success vs failure.
        assert_eq!(scan_int("1"), Some(1));
        assert_eq!(scan_int("0"), Some(0));
        assert_eq!(scan_int("-3xyz"), Some(-3));
        assert_eq!(scan_int(""), None);
        assert_eq!(scan_int("abc"), None);
        let overflow = std::os::raw::c_long::MAX as i32;
        assert_eq!(scan_int("99999999999999999999"), Some(overflow));
    }

    /// Fold a raw reading with a known previous position and last direction.
    fn fold(prev: i32, last_dir: bool, reading: AxisReading) -> (MotorStatus, i32) {
        let mut prev_position = prev;
        let status = fold_reading(&mut prev_position, last_dir, &reading);
        (status, prev_position)
    }

    #[test]
    fn on_target_maps_to_done_and_home() {
        let (status, _) = fold(
            0,
            false,
            AxisReading {
                ontarget: true,
                overflow: false,
                servo: true,
                position: 100,
            },
        );
        assert!(status.done);
        assert!(!status.moving);
        assert!(status.home); // RA_HOME = RA_DONE
        assert!(status.powered); // servo -> EA_POSITION
        assert!(!status.high_limit);
    }

    #[test]
    fn overflow_only_flags_plus_limit_while_on_target() {
        // overflow while on-target -> + limit.
        let (on, _) = fold(
            0,
            false,
            AxisReading {
                ontarget: true,
                overflow: true,
                servo: true,
                position: 0,
            },
        );
        assert!(on.high_limit);
        assert!(!on.low_limit);
        // overflow while NOT on-target -> no limit (C gates plusLS on RA_DONE).
        let (off, _) = fold(
            0,
            false,
            AxisReading {
                ontarget: false,
                overflow: true,
                servo: true,
                position: 0,
            },
        );
        assert!(!off.high_limit);
    }

    #[test]
    fn direction_updates_only_on_position_change() {
        // Position increased -> plus direction, prev updated.
        let (status, prev) = fold(
            100,
            false,
            AxisReading {
                ontarget: true,
                overflow: false,
                servo: true,
                position: 200,
            },
        );
        assert!(status.direction);
        assert_eq!(prev, 200);
        // Position decreased -> minus direction.
        let (status, _) = fold(
            200,
            true,
            AxisReading {
                ontarget: true,
                overflow: false,
                servo: true,
                position: 100,
            },
        );
        assert!(!status.direction);
        // Unchanged position -> previous direction persists.
        let (kept_true, prev) = fold(
            150,
            true,
            AxisReading {
                ontarget: true,
                overflow: false,
                servo: true,
                position: 150,
            },
        );
        assert!(kept_true.direction);
        assert_eq!(prev, 150);
        let (kept_false, _) = fold(
            150,
            false,
            AxisReading {
                ontarget: true,
                overflow: false,
                servo: true,
                position: 150,
            },
        );
        assert!(!kept_false.direction);
    }

    #[test]
    fn servo_disabled_clears_powered() {
        let (status, _) = fold(
            0,
            false,
            AxisReading {
                ontarget: true,
                overflow: false,
                servo: false,
                position: 0,
            },
        );
        assert!(!status.powered);
    }

    /// drvPIE516.cc:276-297: a first failed status read is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and a good read clears it.
    #[test]
    fn a_failed_status_read_is_retry_then_comm_error() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("e516Retry", move |last| {
            if q.load(Ordering::SeqCst) {
                return String::new();
            }
            match last {
                "ONL?" | "SVO? A" => "1".into(),
                "VER?" => "E-516".into(),
                "POS? A" => "0.000".into(),
                "ONT? A" => "1".into(),
                "OVF? A" => "0".into(),
                _ => String::new(),
            }
        });
        let ctrl = PIE516Controller::new(io).unwrap();
        let mut axis = PIE516Axis::new(Arc::new(Mutex::new(ctrl)), 0);
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

    /// drvPIE516.cc:276-297 exit stores what motor_init prepared (:635-644,
    /// EA_PRESENT and GAIN_SUPPORT), with both COMM_ERR bits once past
    /// RETRY.
    #[test]
    fn a_failed_init_read_keeps_the_encoder_bits() {
        let (_line, io) = LineRig::new("e516InitFail", |last| match last {
            "ONL?" => "1".into(),
            "VER?" => "E-516".into(),
            _ => String::new(),
        });
        let ctrl = Arc::new(Mutex::new(PIE516Controller::new(io).unwrap()));
        let mut first = PIE516Axis::new(ctrl.clone(), 0);
        let mut second = PIE516Axis::new(ctrl, 1);
        let prepared = MotorStatus {
            gain_support: true,
            has_encoder: true,
            vbas_supported: false,
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
}
