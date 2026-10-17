//! PI (Physik Instrumente) E-710 digital piezo/nano-positioning controller
//! (`motorPI`).
//!
//! Ported from `drvPIE710.cc` + `devPIE710.cc` ("copied from drvPIC848.cc").
//! Unlike the E-516/E-517/E-816 (which poll three separate integer replies),
//! the E-710 is a closed-loop DC-servo controller that reports a single 16-bit
//! **status word** (`#GI8`) plus a target-position query (`#TP`), and it uses a
//! comma-terminated, `%.5f` command syntax. Its own C source is the sole
//! reference for every decision below.
//!
//! ## Addressing — digit axes, first-char substitution
//! `PIE710_axis[] = {"1".."6"}`, `MAX_AXES = 6`. C `send_mess` overwrites only
//! `local_buff[0]` with `*name`, so the leading `#` placeholder of a command
//! becomes the axis digit (`#GI8` → `1GI8`, `#TP` → `1TP`). `substitute_axis`
//! reproduces this.
//!
//! ## EOS ownership
//! C `motor_init` sets output and input EOS to `"\n"` itself, so the asyn port
//! owns framing: `PIE710Config` sets `"\n"` both ways and this port writes
//! bare command bytes. Commands carry a trailing `,` (the E-710 command
//! separator), which is the last byte before the port-appended `"\n"`.
//!
//! ## Connect handshake (C `motor_init`)
//! `connect` → flush → up to 3 iterations of: send `GI` (`GET_IDENT`, sent with
//! `name == NULL` so **no** axis substitution — bare `GI`); read the first
//! reply (the ident, from which the firmware version is parsed via
//! `strchr('V')`, `NINT(atof(V+1)·1000)`); read a **second** reply. The loop
//! continues while the second reply is empty *and* no version was found. C then
//! proceeds only if the second reply was non-empty (`status > 0`), so this port
//! errors the connect when it is not. Axis count is then probed by `#TP` over
//! `1..=6`, stopping at the first silent axis.
//!
//! ## Firmware-version status shift — the `2^8` C bug (fixed here)
//! Older E-710 revisions return a 1-byte status that C intends to shift into
//! the high byte. The version test picks who needs the shift:
//! ```text
//! statusShift = !((version >= 5000 || version == 4019 || version == 4020)
//!                 && version != 5018)
//! ```
//! But upstream writes the shift as `mstat.All = statusInt * (2^8)`, and in C
//! `2^8` is the bitwise-XOR `2 ^ 8 == 10`, **not** `256` — so C multiplies a
//! "shifted" status by ten and its bits never reach the high byte where the
//! meaningful bits live. `decode_status` implements the intended `* 256`
//! (`<< 8`) instead (upstream defect register `doc/upstream-c-defects.md`
//! #45, retro-fixed per the fix-at-source policy).
//!
//! ## Status word decode (`E710_Status_Reg`, C `set_status`)
//! Meaningful bits (LSB-numbered) in the 16-bit word: bit 8 `torque`
//! (servo-control / "torque disabled" flag), bit 10 `moving`, bit 11
//! `minus_ls`, bit 12 `plus_ls`.
//! - `RA_DONE` = `!(moving && !torque)` ("Always DONE if torque disabled"), and
//!   `RA_HOME = RA_DONE`.
//! - `EA_POSITION` (`powered`) = `!torque`.
//! - Limit switches are read only when `RA_DONE`: `RA_PLUS_LS = done &&
//!   plus_ls`, `RA_MINUS_LS = done && minus_ls`.
//! - `motorData = NINT(atof(#TP reply) / POS_RES)`, `POS_RES = 0.0001` →
//!   `MRES = 1`. The `#TP` read is **not** gated (C converts even an empty
//!   reply, `atof("") == 0`); only the `#GI8` parse drives the comm debounce.
//! - `RA_DIRECTION` from the position delta, persisting across no-motion polls.
//! - `RA_PROBLEM`, `EA_SLIP`, `EA_SLIP_STALL`, `EA_HOME` are cleared; velocity
//!   is left 0.
//!
//! Same per-controller `NORMAL`/`RETRY`/`COMM_ERR` two-strike debounce as the
//! other PI ports (first failed `#GI8` parse silently retries the cached
//! status; the second consecutive failure flags `comms_error` + `problem`).
//!
//! ## Motion (C `devPIE710_build_trans`, `maxdigits = 5`)
//! `PIE710_build_trans` starts and ends a transaction of its own for every
//! command, so the record's `SET_VELOCITY` ahead of a move/home goes out as
//! its own message, each with its leading `#` addressed:
//! - `MOVE_ABS` → `{axis}SV{v·res},` then `{axis}MA{p·res},` (`%.5f`).
//! - `MOVE_REL` → `{axis}SV{v·res},` then `{axis}MR{d·res},` (`%+.5f` on `MR`,
//!   forced sign).
//! - `HOME_FOR`/`HOME_REV` → `{axis}SV{v·res},` then `{axis}GH,` (both
//!   directions send `GH`).
//! - `JOG`/`JOG_VELOCITY` (`move_velocity`) → `{axis}SV{velocity},` — raw
//!   velocity, no `res` scaling, single command (the C JOG asymmetry).
//! - `STOP_AXIS` → `{axis}MR0,` (no stop command; a zero relative move).
//! - `ENABLE_TORQUE`/`DISABL_TORQUE` → `{axis}SL1,` / `{axis}SL0,`.
//! - `SET_PGAIN` → `{axis}SP{gain}` (`%.5f`, no trailing `,`).
//! - `LOAD_POS` → C `ERROR`; `set_position()` errors.
//! - `SET_ACCEL`/`GO`/`SET_VEL_BASE`/`SET_IGAIN`/`SET_DGAIN`/`SET_HIGH_LIMIT`/
//!   `SET_LOW_LIMIT`/`SET_ENC_RATIO` → no wire command.
//!
//! ## Not-modeled deviations
//! `recv_mess(FLUSH)` (the pre-poll flush when comms are already errored) is
//! replaced by a flush before every query (`exchange`). `no_motion_count` is
//! not modeled — the moving/idle poll cadence and the record's own retry
//! logic cover the same ground, as in the other PI ports.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::model1::MoveStrings;
use motor_common::util::{atof, nint};

use crate::scan_int;

/// Response buffer size for a single reply (C `BUFF_SIZE`).
const READ_BUF: usize = 100;
/// Position resolution, micrometres per step (C `drvPIE710.h` `POS_RES`).
const POS_RES: f64 = 0.0001;
/// Maximum axes per controller (C `MAX_AXES`).
const MAX_AXES: usize = 6;
/// Per-axis command digits (C `PIE710_axis[]`).
const AXIS_LABELS: [&str; MAX_AXES] = ["1", "2", "3", "4", "5", "6"];
/// Command decimal places (C `devPIE710_build_trans` `maxdigits`).
const MAX_DIGITS: usize = 5;

fn pie710_err(message: impl Into<String>) -> AsynError {
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

/// Substitute the axis digit for the leading command character, reproducing C
/// `send_mess`'s `local_buff[0] = *name`.
fn substitute_axis(msg: &str, label: &str) -> String {
    let mut s = String::with_capacity(msg.len());
    s.push_str(label);
    s.push_str(&msg[1..]);
    s
}

/// Parse the E-710 firmware version from the ident string: C
/// `if ((pbuff = strchr(buff, 'V'))) version = NINT(atof(pbuff+1) * 1000)`.
/// Returns 0 when no `'V'` is present.
fn parse_version(ident: &str) -> i32 {
    match ident.find('V') {
        Some(i) => nint(atof(&ident[i + 1..]) * 1000.0),
        None => 0,
    }
}

/// Whether this firmware version needs the (buggy) status-word shift. C:
/// `statusShift = false` iff `(version >= 5000 || version == 4019 ||
/// version == 4020) && version != 5018`.
fn status_shift_for(version: i32) -> bool {
    !((version >= 5000 || version == 4019 || version == 4020) && version != 5018)
}

/// `#GI8` reply parse: C requires `charcnt > 2` and
/// `sscanf(buff, "%s %s %d", …) == 3`, i.e. at least two whitespace tokens
/// followed by a parseable integer. Returns the status integer (stored as its
/// bit pattern) on success.
fn parse_gi8(reply: &str) -> Option<u32> {
    if reply.len() <= 2 {
        return None;
    }
    let mut tokens = reply.split_ascii_whitespace();
    let _t1 = tokens.next()?;
    let _t2 = tokens.next()?;
    let t3 = tokens.next()?;
    scan_int(t3).map(|v| v as u32)
}

/// Decoded status bits after the version-dependent (buggy) shift and the
/// `RA_DONE` limit-switch gate.
struct StatusBits {
    done: bool,
    powered: bool,
    plus_ls: bool,
    minus_ls: bool,
}

/// Decode the 16-bit status word (C `set_status`), applying the intended
/// high-byte `statusShift` and the done-gated limit-switch read.
fn decode_status(status_int: u32, status_shift: bool) -> StatusBits {
    // C intends `mstat.All = statusInt * 256` but writes `statusInt * (2^8)`
    // (XOR = 10), so the shifted bits never reach the high byte. Implement the
    // intent (`<< 8`, truncated to 16 bits); upstream defect register #45.
    let mstat: u16 = if status_shift {
        status_int.wrapping_mul(256) as u16
    } else {
        status_int as u16
    };

    let torque = mstat & (1 << 8) != 0;
    let moving = mstat & (1 << 10) != 0;
    let plus_ls_bit = mstat & (1 << 12) != 0;
    let minus_ls_bit = mstat & (1 << 11) != 0;

    // C: RA_DONE = (moving && !torque) ? 0 : 1 ("always DONE if torque
    // disabled"); De Morgan of `!(moving && !torque)`.
    let done = !moving || torque;
    let powered = !torque;
    // LS is read only while on-target/done (C gates on RA_DONE).
    let (plus_ls, minus_ls) = if done {
        (plus_ls_bit, minus_ls_bit)
    } else {
        (false, false)
    };

    StatusBits {
        done,
        powered,
        plus_ls,
        minus_ls,
    }
}

/// Raw per-axis readings from one `#GI8` + `#TP` exchange.
struct AxisReading {
    done: bool,
    powered: bool,
    plus_ls: bool,
    minus_ls: bool,
    position: i32,
}

/// `{axis}SV{v},` velocity message; a move scales `velocity` by `res`, a jog
/// sends it raw.
fn sv_msg(label: &str, velocity: f64) -> String {
    substitute_axis(&format!("#SV{:.MAX_DIGITS$},", CFloat(velocity)), label)
}

/// `{axis}MA{p},` absolute-move message (`res`-scaled controller units).
fn ma_msg(label: &str, position: f64) -> String {
    substitute_axis(&format!("#MA{:.MAX_DIGITS$},", CFloat(position)), label)
}

/// `{axis}MR{d},` relative-move message (`MR` forces a sign, C `%+.*f`).
fn mr_msg(label: &str, distance: f64) -> String {
    substitute_axis(&format!("#MR{:+.MAX_DIGITS$},", CFloat(distance)), label)
}

/// Fold one raw reading into a [`MotorStatus`] (C `set_status` tail). Updates
/// `prev_position`; `last_direction` persists when the position did not change.
fn fold_reading(
    prev_position: &mut i32,
    last_direction: bool,
    reading: &AxisReading,
) -> MotorStatus {
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
        done: reading.done,
        moving: false,
        msta_moving: false,
        high_limit: reading.plus_ls,
        low_limit: reading.minus_ls,
        home: reading.done,
        encoder_home: false,
        powered: reading.powered,
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

/// Shared controller endpoint: owns the octet handle, the ident, the comm
/// debounce state, the firmware `status_shift` flag, and the probed axis count.
pub struct PIE710Controller {
    handle: SyncIOHandle,
    ident: String,
    comm_state: CommState,
    status_shift: bool,
    num_axes: usize,
}

impl PIE710Controller {
    /// Connect, identify, and probe axes (C `motor_init`). Performs blocking
    /// octet I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            comm_state: CommState::Normal,
            status_shift: true,
            num_axes: 0,
        };

        let mut ident = String::new();
        let mut version = 0;
        let mut second_ok = false;
        for _ in 0..3 {
            ident = ctrl.exchange("GI"); // GET_IDENT, no axis substitution (name == NULL).
            version = parse_version(&ident);
            second_ok = !ctrl.recv().is_empty();
            // C loops while (status == 0 && !version && retry < 3).
            if second_ok || version != 0 {
                break;
            }
        }

        // C proceeds only if the second reply was non-empty (`status > 0`).
        if !second_ok {
            return Err(pie710_err(
                "PIE710: no identification response from controller",
            ));
        }

        ctrl.ident = ident;
        ctrl.status_shift = status_shift_for(version);

        let mut num_axes = 0;
        for label in AXIS_LABELS {
            if ctrl.exchange(&substitute_axis("#TP", label)).is_empty() {
                break;
            }
            num_axes += 1;
        }
        ctrl.num_axes = num_axes;

        Ok(ctrl)
    }

    /// The controller identification string (from the connect-time `GI`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes that responded to the connect-time `#TP` probe.
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

    /// Read one reply (the port's input EOS already stripped the `"\n"`). Any
    /// transport failure or empty read folds into `""`.
    fn recv(&self) -> String {
        Self::reply(self.handle.read_octet(0, READ_BUF))
    }

    /// Write a query and read its reply as one exchange that first flushes
    /// stale input, so a stray line cannot shift every later reply by one
    /// (C recovers the same way through `recv_mess(FLUSH)`).
    fn exchange(&self, cmd: &str) -> String {
        Self::reply(self.handle.write_read(0, cmd.as_bytes(), READ_BUF))
    }

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

    /// The raw `set_status` read chain: `#GI8` (gates the debounce) then `#TP`
    /// (best-effort, converted even when empty). `None` only when the `#GI8`
    /// reply fails the `charcnt > 2 && sscanf == 3` test.
    fn read_axis(&mut self, axis: usize) -> Option<AxisReading> {
        let label = AXIS_LABELS[axis];

        let status_int = parse_gi8(&self.exchange(&substitute_axis("#GI8", label)))?;
        let bits = decode_status(status_int, self.status_shift);

        // #TP read is NOT gated: C converts even an empty reply (atof("") == 0).
        let position = nint(atof(&self.exchange(&substitute_axis("#TP", label))) / POS_RES);

        Some(AxisReading {
            done: bits.done,
            powered: bits.powered,
            plus_ls: bits.plus_ls,
            minus_ls: bits.minus_ls,
            position,
        })
    }
}

/// One axis of an E-710 controller. Implements [`AsynMotor`].
pub struct PIE710Axis {
    controller: Arc<Mutex<PIE710Controller>>,
    axis: usize,
    prev_position: i32,
    /// What the last failed poll raises: nothing on C `set_status`'s
    /// first missed reply (RETRY, drvPIE710.cc:255), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :261). Per axis, as the
    /// controller's state can move on under another axis's poll.
    failure: PollFailure,
    last_status: MotorStatus,
    strings: MoveStrings,
}

impl PIE710Axis {
    /// Construct one axis, seeding the initial status (C `motor_init`'s
    /// per-axis `set_status`). `prev_position` starts at 0.
    pub fn new(controller: Arc<Mutex<PIE710Controller>>, axis: usize) -> AsynResult<Self> {
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

    fn lock(&self) -> MutexGuard<'_, PIE710Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A record string with the axis digit over its first byte, as C's
    /// `send_mess` addresses a message (drvPIE710.cc:399).
    fn addressed(&self, text: &[u8]) -> Vec<u8> {
        let mut text = text.to_vec();
        if let Some(b) = text.first_mut() {
            *b = self.label().as_bytes()[0];
        }
        text
    }

    /// A move's own message: PREM and a `,` ahead of its command
    /// (devPIE710.cc:196-205), whose POST waits for the move to end. C
    /// addresses only the first byte of the whole message, so behind a PREM
    /// the command keeps its `#`; not copied.
    fn move_line(&mut self, command: &str) -> Vec<u8> {
        let prem = self.strings.arm();
        let mut line = self.addressed(&prem);
        if !line.is_empty() {
            line.push(b',');
        }
        line.extend_from_slice(command.as_bytes());
        line
    }
}

impl AsynMotor for PIE710Axis {
    // devPIE710.cc:127 forces NTM to NO.
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
        let line = self.move_line(&ma_msg(label, position * POS_RES));
        let ctrl = self.lock();
        ctrl.send(&sv_msg(label, velocity * POS_RES))?;
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
        let line = self.move_line(&mr_msg(label, distance * POS_RES));
        let ctrl = self.lock();
        ctrl.send(&sv_msg(label, velocity * POS_RES))?;
        ctrl.send_bytes(&line)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // JOG: raw velocity, no `res` scaling, signed; single command.
        let line = self.move_line(&sv_msg(self.label(), velocity));
        self.lock().send_bytes(&line)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C sends #GH regardless of direction, after the velocity.
        let label = self.label();
        let line = self.move_line(&substitute_axis("#GH,", label));
        let ctrl = self.lock();
        ctrl.send(&sv_msg(label, velocity * POS_RES))?;
        ctrl.send_bytes(&line)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C STOP_AXIS → #MR0 (zero relative move; no stop command).
        let msg = substitute_axis("#MR0,", self.label());
        self.lock().send(&msg)
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        Err(pie710_err(
            "PIE710: set-position (LOAD_POS) is not supported",
        ))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // ENABLE_TORQUE → #SL1, DISABL_TORQUE → #SL0.
        let msg = substitute_axis(if enable { "#SL1," } else { "#SL0," }, self.label());
        self.lock().send(&msg)
    }

    /// SET_PGAIN → `#SP%.5f`, unterminated (devPIE710.cc:274-275);
    /// SET_IGAIN/SET_DGAIN send nothing (276-281).
    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        if kind != PidGainKind::Proportional {
            return Ok(());
        }
        let msg = substitute_axis(&format!("#SP{:.5}", CFloat(gain)), self.label());
        self.lock().send(&msg)
    }

    /// `PRIMITIVE`: INIT as its own message, addressed
    /// (devPIE710.cc:188-189).
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
            return Err(pie710_err("no status reply"));
        };
        let last_direction = self.last_status.direction;
        self.last_status = fold_reading(&mut self.prev_position, last_direction, &reading);
        // drvPIE710.cc:310-356: POST, unaddressed, once done or stopped by a
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
    use std::sync::atomic::{AtomicU32, Ordering};

    /// C sprintf's `%.*f` and `%+.*f` spell a NaN as glibc does: `nan`,
    /// `-nan`, `+nan`.
    #[test]
    fn floats_go_out_as_c_sprintf() {
        let nan = f64::NAN;
        let neg = -f64::NAN;
        assert_eq!(sv_msg("1", nan), "1SVnan,");
        assert_eq!(ma_msg("1", neg), "1MA-nan,");
        assert_eq!(mr_msg("1", nan), "1MR+nan,");
        assert_eq!(mr_msg("1", neg), "1MR-nan,");
    }

    /// `#GI8` status words (no shift, firmware V5): done, and moving.
    const DONE: u32 = 0;
    const MOVING: u32 = 1 << 10;

    /// Axis 1 of a one-axis E-710 on a [`LineRig`] whose `1GI8` reply
    /// follows `gi8`.
    fn rig(name: &str) -> (LineRig, PIE710Axis, Arc<AtomicU32>) {
        let gi8 = Arc::new(AtomicU32::new(DONE));
        let g = gi8.clone();
        let (line, io) = LineRig::new(name, move |last| match last {
            "GI" => "PI E-710 V5.000".into(),
            "1TP" => "0.0".into(),
            "1GI8" => format!("A B {}", g.load(Ordering::SeqCst)),
            _ => String::new(),
        });
        let ctrl = PIE710Controller::new(io).unwrap();
        let axis = PIE710Axis::new(Arc::new(Mutex::new(ctrl)), 0).unwrap();
        line.sent();
        (line, axis, gi8)
    }

    fn post(text: &[u8]) -> PostMessage {
        let p = PostMessage::default();
        p.set(text);
        p
    }

    /// What a poll sent besides its queries.
    fn poll_extra(line: &LineRig, axis: &mut PIE710Axis) -> Vec<String> {
        axis.poll(&AsynUser::default()).unwrap();
        line.sent()
            .into_iter()
            .filter(|l| l != "1GI8" && l != "1TP")
            .collect()
    }

    /// devPIE710.cc:188-189: INIT as its own message, addressed.
    #[test]
    fn init_goes_out_as_its_own_message() {
        let (line, mut axis, _) = rig("pie710Init");
        axis.primitive(&AsynUser::default(), b"#SL1,").unwrap();
        assert_eq!(line.sent(), ["1SL1,"]);
    }

    /// devPIE710.cc:274-282: SET_PGAIN sends `#SP` with the gain at
    /// `%.5f` and no trailing `,`; SET_IGAIN/SET_DGAIN send nothing.
    #[test]
    fn proportional_gain_goes_out_as_sp() {
        let (line, mut axis, _) = rig("pie710Gain");
        let user = AsynUser::default();
        axis.set_pid_gain(&user, PidGainKind::Proportional, 0.25)
            .unwrap();
        assert_eq!(line.sent(), ["1SP0.25000"]);
        axis.set_pid_gain(&user, PidGainKind::Integral, 0.5)
            .unwrap();
        axis.set_pid_gain(&user, PidGainKind::Derivative, 0.5)
            .unwrap();
        assert!(line.sent().is_empty());
    }

    /// devPIE710.cc:196-205: PREM and a `,` ahead of the move's own command,
    /// both addressed, for that move only.
    #[test]
    fn prem_goes_ahead_of_the_move() {
        let (line, mut axis, _) = rig("pie710Prem");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"#SL1", None).unwrap();
        axis.move_absolute(&user, 10000.0, 0.0, 5000.0, 0.0)
            .unwrap();
        assert_eq!(line.sent(), ["1SV0.50000,", "1SL1,1MA1.00000,"]);
        axis.move_relative(&user, 10000.0, 0.0, 5000.0, 0.0)
            .unwrap();
        assert_eq!(line.sent(), ["1SV0.50000,", "1MR+1.00000,"]);
        axis.set_move_strings(&user, b"#SL1", None).unwrap();
        axis.home(&user, 0.0, 3000.0, 0.0, true).unwrap();
        assert_eq!(line.sent(), ["1SV0.30000,", "1SL1,1GH,"]);
        axis.set_move_strings(&user, b"#SL1", None).unwrap();
        axis.move_velocity(&user, 0.0, -2.5, 0.0).unwrap();
        assert_eq!(line.sent(), ["1SL1,1SV-2.50000,"]);
    }

    /// drvPIE710.cc:349-356: POST, unaddressed, once the move is done, not
    /// while it runs, and only once.
    #[test]
    fn post_follows_the_move_once_done() {
        let (line, mut axis, gi8) = rig("pie710Post");
        let user = AsynUser::default();
        axis.set_move_strings(&user, b"", Some(post(b"1SL0,")))
            .unwrap();
        axis.move_absolute(&user, 10000.0, 0.0, 5000.0, 0.0)
            .unwrap();
        line.sent();
        gi8.store(MOVING, Ordering::SeqCst);
        assert!(poll_extra(&line, &mut axis).is_empty());
        gi8.store(DONE, Ordering::SeqCst);
        assert_eq!(poll_extra(&line, &mut axis), ["1SL0,"]);
        assert!(poll_extra(&line, &mut axis).is_empty(), "sent once");
    }

    /// devPIE710.cc's build_trans opens and ends a transaction per command,
    /// so the velocity and the move are two messages, each addressed.
    #[test]
    fn velocity_and_move_are_two_messages() {
        let (line, mut axis, _) = rig("pie710Split");
        let user = AsynUser::default();
        axis.move_absolute(&user, 10000.0, 0.0, 5000.0, 0.0)
            .unwrap();
        assert_eq!(line.sent(), ["1SV0.50000,", "1MA1.00000,"]);
        axis.move_relative(&user, -2500.0, 0.0, 1000.0, 0.0)
            .unwrap();
        assert_eq!(line.sent(), ["1SV0.10000,", "1MR-0.25000,"]);
        axis.home(&user, 0.0, 3000.0, 0.0, false).unwrap();
        assert_eq!(line.sent(), ["1SV0.30000,", "1GH,"]);
    }

    #[test]
    fn substitute_axis_replaces_only_the_first_char() {
        assert_eq!(substitute_axis("#GI8", "1"), "1GI8");
        assert_eq!(substitute_axis("#TP", "3"), "3TP");
    }

    #[test]
    fn move_messages_scale_by_res_and_use_five_decimals() {
        // POS_RES = 0.0001, so 10000 steps -> 1.00000 um.
        assert_eq!(sv_msg("1", 5000.0 * POS_RES), "1SV0.50000,");
        assert_eq!(ma_msg("1", 10000.0 * POS_RES), "1MA1.00000,");
        // MR forces a sign; SV does not.
        assert_eq!(mr_msg("2", 2500.0 * POS_RES), "2MR+0.25000,");
        assert_eq!(mr_msg("2", -2500.0 * POS_RES), "2MR-0.25000,");
    }

    #[test]
    fn jog_velocity_is_raw_not_res_scaled() {
        assert_eq!(sv_msg("1", 5.0), "1SV5.00000,");
        assert_eq!(sv_msg("6", -2.5), "6SV-2.50000,");
    }

    #[test]
    fn stop_is_zero_relative_move() {
        assert_eq!(substitute_axis("#MR0,", "3"), "3MR0,");
    }

    #[test]
    fn parse_gi8_needs_two_tokens_then_an_int() {
        assert_eq!(parse_gi8("PI E710 256"), Some(256));
        assert_eq!(parse_gi8("A B -5"), Some(-5i32 as u32));
        assert_eq!(parse_gi8("A B"), None); // only two tokens
        assert_eq!(parse_gi8("A B xy"), None); // third token not an int
        assert_eq!(parse_gi8("ab"), None); // charcnt <= 2
    }

    #[test]
    fn parse_version_reads_after_the_v() {
        assert_eq!(parse_version("PI E-710 V4.019 build"), 4019);
        assert_eq!(parse_version("V5.000"), 5000);
        assert_eq!(parse_version("no version here"), 0);
    }

    #[test]
    fn status_shift_selects_by_version() {
        // (>=5000 || 4019 || 4020) && !=5018  => no shift.
        assert!(!status_shift_for(5000));
        assert!(!status_shift_for(4019));
        assert!(!status_shift_for(4020));
        assert!(status_shift_for(5018)); // explicit exception -> shift
        assert!(status_shift_for(3000)); // old revision -> shift
    }

    #[test]
    fn status_shift_moves_low_byte_into_high_byte() {
        // Upstream defect register #45 (retro-fixed): C's `2^8` XOR typo
        // multiplied by 10; the intended shift is `* 256`. A 1-byte status
        // must land in the high byte where the meaningful bits live.
        // statusInt bit 0 -> torque (bit 8): powered cleared, done forced.
        let bits = decode_status(1 << 0, true);
        assert!(bits.done);
        assert!(!bits.powered);

        // statusInt bit 2 -> moving (bit 10), no torque -> not done.
        let bits = decode_status(1 << 2, true);
        assert!(!bits.done);
        assert!(bits.powered);

        // statusInt bits 3/4 -> minus/plus LS (bits 11/12), read once done.
        let bits = decode_status(1 << 4, true);
        assert!(bits.done);
        assert!(bits.plus_ls);
        assert!(!bits.minus_ls);
    }

    #[test]
    fn unshifted_status_decodes_high_byte_bits() {
        // torque bit (8) set, no moving -> done, powered cleared (torque disabled).
        let bits = decode_status(1 << 8, false);
        assert!(bits.done);
        assert!(!bits.powered);

        // moving (bit 10) set, torque clear -> not done (moving && !torque).
        let bits = decode_status(1 << 10, false);
        assert!(!bits.done);
        assert!(bits.powered);
    }

    #[test]
    fn limit_switches_gate_on_done() {
        // plus_ls (bit 12) with torque disabled (bit 8) -> done, LS reported.
        let bits = decode_status((1 << 12) | (1 << 8), false);
        assert!(bits.done);
        assert!(bits.plus_ls);

        // minus_ls (bit 11) while moving (bit 10, torque clear) -> not done,
        // LS suppressed.
        let bits = decode_status((1 << 11) | (1 << 10), false);
        assert!(!bits.done);
        assert!(!bits.minus_ls);
    }

    /// Fold a raw reading with a known previous position and last direction.
    fn fold(prev: i32, last_dir: bool, reading: AxisReading) -> (MotorStatus, i32) {
        let mut prev_position = prev;
        let status = fold_reading(&mut prev_position, last_dir, &reading);
        (status, prev_position)
    }

    #[test]
    fn direction_updates_only_on_position_change() {
        let (moved, prev) = fold(
            0,
            false,
            AxisReading {
                done: true,
                powered: true,
                plus_ls: false,
                minus_ls: false,
                position: 100,
            },
        );
        assert!(moved.direction); // 100 >= 0
        assert_eq!(prev, 100);

        let (kept, prev) = fold(
            50,
            false,
            AxisReading {
                done: true,
                powered: true,
                plus_ls: false,
                minus_ls: false,
                position: 50,
            },
        );
        assert!(!kept.direction); // unchanged -> keep last_direction
        assert_eq!(prev, 50);
    }

    /// drvPIE710.cc:246-267: a first failed status read is RETRY (nothing raised,
    /// nothing posted), the next COMM_ERR, and a good read clears it.
    #[test]
    fn a_failed_status_read_is_retry_then_comm_error() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("e710Retry", move |last| {
            if q.load(Ordering::SeqCst) {
                return String::new();
            }
            match last {
                "GI" => "PI E-710 V5.000".into(),
                "1TP" => "0.0".into(),
                "1GI8" => format!("A B {DONE}"),
                _ => String::new(),
            }
        });
        let ctrl = PIE710Controller::new(io).unwrap();
        let mut axis = PIE710Axis::new(Arc::new(Mutex::new(ctrl)), 0).unwrap();
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
