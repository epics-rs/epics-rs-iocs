//! PI (Physik Instrumente) C-862/C-863 DC-motor controller (`motorPI`).
//!
//! Ported from `drvPIC862.cc` + `devPIC862.cc` (a model-1 dev/drv pair) —
//! NOT the GCS2 command set (`motor-pi-gcs2`); the C-862 predates GCS and
//! uses its own two-letter ASCII commands. Single axis per controller (C
//! `MAX_AXES` is hardcoded to 1; `PIC862_NUM_CARDS` allows up to 8 separate
//! controller connections, each its own `PIC862Config` call).
//!
//! ## Transport
//!
//! Despite the iocsh argument being labeled "asyn address (GPIB)"
//! (`PIC862Register.cc`), the driver never touches a GPIB-specific asyn
//! interface — it is pure `pasynOctetSyncIO` (works over serial or IP), and
//! `motor_init`'s `pasynOctetSyncIO->connect(cntrl->asyn_port, 0, ...)`
//! hardcodes the asyn sub-address to 0, ignoring `cntrl->asyn_address`
//! entirely at the transport layer. The "address" is instead a protocol-level
//! multi-drop selector (below) — this port connects via
//! [`motor_common::connect::connect_serial`], exactly like every other octet
//! driver in this workspace.
//!
//! ## Wire shape — the port owns framing (not the driver)
//!
//! `motor_init` calls `pasynOctetSyncIO->setOutputEos(pasynUser,
//! "\r", 1)` **and** `setInputEos(pasynUser, "\n\x03", 2)` right after
//! connecting, and `send_mess`'s `local_buff` never appends a terminator
//! itself (`strcat(local_buff, com)` only) — the port's configured output
//! EOS is what puts the trailing CR on the wire (the asyn EOS layer appends
//! it at write time). This is the opposite framing choice from e.g. the
//! Mclennan PM304 port (which appends its own terminator and leaves the
//! port's output EOS unset) — see the `motor-port-eos-ownership` convention:
//! which one applies is a per-C-driver fact, not a template to copy.
//! `SyncIOHandle` (unlike the real `pasynUser`) exposes no method for a
//! driver to configure a port's EOS itself, so this port pushes both EOS
//! settings into `st.cmd` (`asynOctetSetInputEos`/`asynOctetSetOutputEos`)
//! instead of setting them from [`PIC862Controller::new`] — same wire
//! result, different call site. [`PIC862Controller::send`] therefore writes
//! **bare** command bytes with no manual terminator.
//!
//! Input terminator: `"\n\x03"` (LF then ASCII ETX, 2 bytes) — `\x03` is
//! representable as a 2-hex-digit `\x` escape in `st.cmd` (this port's
//! `asyn-rs` iocsh escape decoder has no octal-triplet form, so the C
//! header's `"\n\003"` spelling doesn't carry over literally).
//!
//! `recv_mess` unconditionally drops the **last** received byte
//! (`com[nread - 1] = '\0'; /* Strip trailing CR. */`) after the port's own
//! EOS strip already removed the `\n\x03` — i.e. every reply carries one
//! extra CR ahead of the terminator that the port's EOS layer doesn't know
//! about. [`PIC862Controller::recv`] reproduces this unconditional
//! single-byte drop (not a `trim_end_matches`, which would strip a variable
//! count) and folds any transport error into an empty reply, matching
//! `recv_mess`'s swallow of `status != asynSuccess` into `""`.
//!
//! ## Multi-drop enable/identify (connect-time only)
//!
//! `motor_init` selects the addressed device on the (potentially shared)
//! serial line by writing `\x01{addr}VE` (SOH, one uppercase hex digit
//! 0-F, then `VE` — "enable device `addr`, then identify"), retrying up to 3
//! times until a non-empty reply arrives; that reply becomes `brdptr->ident`.
//! **This selection is sent once, at connect time only** — `TS`/`TP`/`MA`/…
//! never re-prefix the address. A second `PIC862Config` sharing the same
//! physical port would therefore re-select its own address at its own
//! connect time and leave that address selected on the bus, which is a
//! latent multi-drop hazard already present in the C driver; out of scope
//! here (this round ports one controller end-to-end, i.e. one axis per
//! config call, matching the C `MAX_AXES = 1`).
//!
//! ## Status (`TS` — "Tell Status")
//!
//! Six 2-hex-digit registers: `"S:XX XX XX XX XX XX"` (`sscanf(buff,
//! "S:%2hx %2hx %2hx %2hx %2hx %2hx\n")`, `charcnt > 18` gates a short/bad
//! reply). Bit layout is the header's `#else` (non-`MSB_First`) branch — the
//! one that actually executes on any real (little-endian) build target:
//!
//! | field | register | bit |
//! |-------|----------|-----|
//! | `trty_done` (→ `RA_DONE`) | 1 | 2 (`0x04`) |
//! | `motor_off` (→ `!powered`) | 1 | 7 (`0x80`) |
//! | `mvdir_pol` (→ `RA_DIRECTION`) | 3 | 2 (`0x04`) |
//! | `lmt_high` (limit-active-high select) | 4 | 1 (`0x02`) |
//! | `plus_ls` | 5 | 2 (`0x04`) |
//! | `minus_ls` | 5 | 3 (`0x08`) |
//!
//! Register 2 is read (needed to keep the 6-field `sscanf` count at 6) but
//! never consulted by any status bit — dropped here too. The plus/minus limit
//! switch raw bits are inverted when `lmt_high` is false (limit switches are
//! active-low in that configuration).
//!
//! A comm failure on `TS` runs the C `NORMAL`/`RETRY`/`COMM_ERR` two-strike
//! debounce ([`CommState`]): the first bad `TS` in a row is silently retried
//! (cached status returned unchanged); a second consecutive one sets
//! `comms_error`/`problem`. `TS` succeeding unconditionally proceeds to `TP`
//! with **no further debounce tied to `TP`** — a genuine C asymmetry (`TP`'s
//! `atof(&buff[2])` has no read-failure guard at all) ported as-is: a failed
//! `TP` right after a good `TS` degrades to a `0.0` position reading (the
//! `atof`-on-garbage convention used throughout this port) rather than
//! flagging comm error.
//!
//! ## Position (`TP` — "Tell Position")
//! `motorData = NINT(atof(&buff[2]))` — skip a 2-character reply prefix,
//! parse the rest, then round to the nearest integer count (C `NINT`, not a
//! truncating cast). [`motor_common::util::parse_value_at`] does the skip
//! + parse; [`motor_common::util::nint`] does the rounding.
//!
//! ## Units
//! Positions/velocities/accelerations are raw controller counts (`MA`/`MR`
//! take integer counts, `TP` returns them) with no separate EGU scaling in
//! the driver — like the Mclennan PM304 port, pair this with `MRES = 1`
//! records. Values cross the `AsynMotor` boundary via a plain `as i32`
//! truncating cast (C `cntrl_units = (int) dval;`), NOT `NINT` rounding —
//! this driver never rounds to nearest.
//!
//! ## Motion commands (`devPIC862_build_trans`)
//! The record's generic model-1 move/home transaction always prefixes
//! `SET_VELOCITY`/`SET_ACCEL` ahead of `MOVE_ABS`/`MOVE_REL`/`HOME_FOR`/
//! `HOME_REV` (`SET_VEL_BASE` is a no-op here — DC motor, no base velocity —
//! so `vbas_supported: false`); this port issues the equivalent
//! `"SV{v},SA{a},{move}"` as one write, mirroring the already-ported
//! Newport MM3000 (`motion_preamble`) and PI GCS2 patterns.
//!
//! - `MOVE_ABS`/`MOVE_REL` → `MA{n}`/`MR{n}`.
//! - `HOME_FOR`/`HOME_REV` → `FE0`/`FE1` (no direction-dependent speed
//!   handling beyond the shared SV/SA preamble).
//! - `JOG` (`move_velocity`) — **the C-862 has no jog command**: `SV{|v|}`
//!   then `MA` to whichever record soft limit (`DHLM`/`DLLM`, cached from
//!   [`AsynMotor::set_high_limit`]/[`set_low_limit`]) the velocity sign
//!   selects, exactly like MM3000's jog simulation. `SET_HIGH_LIMIT`/
//!   `SET_LOW_LIMIT` themselves send no wire command (`trans->state =
//!   IDLE_STATE`) — record-side caching only.
//! - `LOAD_POS` (`set_position`) → `DH` only when the **truncated** value is
//!   exactly `0` (`cntrl_units == 0`, so e.g. `0.4` truncates to `0` and is
//!   accepted); any other value is an error (only zero-position load is
//!   supported).
//! - `ENABLE_TORQUE`/`DISABL_TORQUE` (`set_closed_loop`) → `MN`/`MF`.
//! - `STOP_AXIS` (`stop`) → `ST`.
//! - `SET_PGAIN`/`SET_IGAIN`/`SET_DGAIN` (`set_pid_gain`) → `DP{n}`/`DI{n}`/
//!   `DD{n}`, `n = (int)(32767 * gain)`.
//! - `GO` is a no-op here (`send = false`; the C-862 starts moving
//!   immediately on `MA`/`MR`) — the `AsynMotor` interface has no separate GO
//!   step, so nothing to port. `SET_ENC_RATIO`/`GET_INFO`/`PRIMITIVE` have no
//!   `AsynMotor` counterpart either and are not ported.
//!
//! ## Not modeled (documented deviations)
//! - **`recv_mess(..., FLUSH)`**: before retrying after a non-`NORMAL` comm
//!   state, C discards any buffered input via a non-blocking port-level
//!   flush. [`epics_rs::asyn::sync_io::SyncIOHandle`] exposes no synchronous
//!   flush primitive (only `read_octet`/`write_octet`); emulating it with a
//!   blocking read would stall `poll()` for a full timeout when nothing is
//!   buffered, which is worse than the omission. Skipped — a minor hygiene
//!   gap on an already-degraded comm path, not a wire-command difference.
//! - **PREM/POST** (record pre/post-move command strings): handled generically
//!   by this workspace's motor-rs record layer, not per-driver, matching
//!   every other port here.
//! - **`no_motion_count`**: purely a model-1 poll-rate scheduling counter
//!   (decides how eagerly `motor_task` re-polls); the position/encoder
//!   fields it gates are unconditionally set to the same value either way, so
//!   this port always updates `position` from `TP` and doesn't track it.
//! - **`report()`** driver-status iocsh diagnostic: not implemented, matching
//!   every other port in this workspace (the identify string is logged once
//!   at `PIC862Config` time instead, like `PIGCS2CreateController`).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{nint, parse_value_at};

/// Response buffer size for a single reply (C `BUFF_SIZE`).
const READ_BUF: usize = 100;

fn pic862_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Controller communication state (C `cntrl->status`): one failed `TS` read
/// is retried silently; a second consecutive one is a hard comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// The six `TS` status registers, decoded (C
/// `C862_Status_Reg{1,3,4,5}` `#else`/non-`MSB_First` bit layout; register 2
/// is read but never consulted, matching C).
struct StatusBits {
    /// Register 1 bit 2: trajectory complete.
    done: bool,
    /// Register 1 bit 7 (`motor_off`), inverted.
    powered: bool,
    /// Register 3 bit 2: move direction polarity.
    plus_dir: bool,
    /// Register 4 bit 1: limit switches are active-high when set.
    lmt_active_high: bool,
    /// Register 5 bit 2, raw polarity (see `lmt_active_high`).
    plus_ls_raw: bool,
    /// Register 5 bit 3, raw polarity (see `lmt_active_high`).
    minus_ls_raw: bool,
}

fn decode_regs(regs: &[u16; 6]) -> StatusBits {
    StatusBits {
        done: regs[0] & 0x04 != 0,
        powered: regs[0] & 0x80 == 0,
        plus_dir: regs[2] & 0x04 != 0,
        lmt_active_high: regs[3] & 0x02 != 0,
        plus_ls_raw: regs[4] & 0x04 != 0,
        minus_ls_raw: regs[4] & 0x08 != 0,
    }
}

/// Parse a `TS` reply's 6 two-hex-digit registers (`sscanf(buff,
/// "S:%2hx %2hx %2hx %2hx %2hx %2hx")`); `None` if the `"S:"` prefix or all 6
/// whitespace-separated hex fields aren't present (C's `convert_cnt != 6`).
fn parse_ts_reply(reply: &str) -> Option<[u16; 6]> {
    let body = reply.strip_prefix("S:")?;
    let mut regs = [0u16; 6];
    let mut fields = body.split_whitespace();
    for reg in regs.iter_mut() {
        *reg = u16::from_str_radix(fields.next()?, 16).ok()?;
    }
    Some(regs)
}

/// Decoded result of one successful `TS`+`TP` exchange.
struct PolledStatus {
    position: f64,
    done: bool,
    powered: bool,
    direction: bool,
    high_limit: bool,
    low_limit: bool,
}

fn status_from_polled(polled: &PolledStatus) -> MotorStatus {
    MotorStatus {
        position: polled.position,
        encoder_position: polled.position,
        velocity: 0.0, // C only ever flips the sign of an otherwise-unset 0.
        done: polled.done,
        moving: !polled.done,
        high_limit: polled.high_limit,
        low_limit: polled.low_limit,
        direction: polled.direction,
        powered: polled.powered,
        problem: false,
        comms_error: false,
        homed: false,
        gain_support: true,
        has_encoder: true,
        vbas_supported: false,
        ..MotorStatus::default()
    }
}

/// Shared controller endpoint: owns the octet handle and the comm debounce
/// state. One controller drives exactly one axis (C `MAX_AXES` is 1).
pub struct PIC862Controller {
    handle: SyncIOHandle,
    ident: String,
    comm_state: CommState,
}

impl PIC862Controller {
    /// Connect and select/identify a C-862/C-863 (C `motor_init`'s per-card
    /// block): write `\x01{addr}VE` (enable device `addr`, then identify),
    /// retrying up to 3 times until a non-empty reply arrives. Performs
    /// blocking octet I/O. `addr` is the multi-drop bus address (0-F, a
    /// single hex digit — matching C's `%1X` format and the header's documented
    /// range).
    pub fn new(handle: SyncIOHandle, addr: u8) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            comm_state: CommState::Normal,
        };

        let select_cmd = format!("\u{1}{addr:X}VE");
        for _ in 0..3 {
            ctrl.send(&select_cmd)?;
            let (reply, raw_len) = ctrl.recv();
            if raw_len > 0 {
                ctrl.ident = reply;
                break;
            }
        }
        if ctrl.ident.is_empty() {
            return Err(pic862_err(format!(
                "PIC862: no response to enable/identify (\\x01{addr:X}VE)"
            )));
        }
        Ok(ctrl)
    }

    /// The controller identification string (from the connect-time `VE`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Write a command, no reply expected (C `send_mess`). No terminator is
    /// appended here — the port's configured output EOS puts the CR on the
    /// wire (see module docs' "Wire shape" section).
    fn send(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Read one reply, unconditionally dropping the last byte (C
    /// `recv_mess`'s "Strip trailing CR" — always removes the last byte,
    /// not just an actual CR). Returns `(reply, raw_len)`; `raw_len` is the
    /// pre-strip byte count (C's `charcnt`), needed by callers that gate on
    /// it directly. Any transport failure or empty read folds into `("", 0)`,
    /// matching `recv_mess`'s swallow of a bad read into an empty string.
    fn recv(&self) -> (String, usize) {
        match self.handle.read_octet(0, READ_BUF) {
            Ok(raw) if !raw.is_empty() => {
                let stripped = &raw[..raw.len() - 1];
                (String::from_utf8_lossy(stripped).into_owned(), raw.len())
            }
            _ => (String::new(), 0),
        }
    }

    /// C `set_status`: `TS` gated by the `NORMAL`/`RETRY`/`COMM_ERR` comm
    /// debounce, then (unconditionally on a good `TS`) `TP` with no further
    /// debounce — see the module docs' "Status" section for the exact C
    /// asymmetry this reproduces. `None` means the caller should keep its
    /// cached status (debounced retry) or flag a hard comm error, decided by
    /// [`Self::comm_state`] after the call.
    fn poll_status(&mut self) -> Option<PolledStatus> {
        let _ = self.send("TS");
        let (ts_reply, ts_raw_len) = self.recv();
        let regs = if ts_raw_len > 18 {
            parse_ts_reply(&ts_reply)
        } else {
            None
        };

        let Some(regs) = regs else {
            self.comm_state = if self.comm_state == CommState::Normal {
                CommState::Retry
            } else {
                CommState::CommErr
            };
            return None;
        };
        self.comm_state = CommState::Normal;

        let bits = decode_regs(&regs);
        let (high_limit, low_limit) = if bits.lmt_active_high {
            (bits.plus_ls_raw, bits.minus_ls_raw)
        } else {
            (!bits.plus_ls_raw, !bits.minus_ls_raw)
        };

        let _ = self.send("TP");
        let (tp_reply, _) = self.recv();
        let position = parse_value_at(&tp_reply, 2).map_or(0.0, |v| nint(v) as f64);

        Some(PolledStatus {
            position,
            done: bits.done,
            powered: bits.powered,
            direction: bits.plus_dir,
            high_limit,
            low_limit,
        })
    }
}

/// The single axis of a C-862/C-863 controller. Implements [`AsynMotor`].
pub struct PIC862Axis {
    controller: Arc<Mutex<PIC862Controller>>,
    /// Record soft limits (dial EGU = raw counts, `MRES = 1`), cached from
    /// [`AsynMotor::set_high_limit`]/[`set_low_limit`] — the C-862 has no
    /// jog command, so a jog moves to whichever of these the record's DHLM
    /// / DLLM the requested direction selects (C `JOG` case).
    high_limit: Option<f64>,
    low_limit: Option<f64>,
    last_status: MotorStatus,
}

impl PIC862Axis {
    /// Construct the controller's one axis, seeding the initial status (C
    /// `motor_init`'s `set_status(card_index, motor_index)` call). A failed
    /// initial poll is not an error — C leaves the zeroed defaults in place
    /// either way.
    pub fn new(controller: Arc<Mutex<PIC862Controller>>) -> AsynResult<Self> {
        let mut axis = Self {
            controller: controller.clone(),
            high_limit: None,
            low_limit: None,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                has_encoder: true,
                vbas_supported: false,
                ..MotorStatus::default()
            },
        };
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(polled) = ctrl.poll_status() {
            axis.last_status = status_from_polled(&polled);
        }
        Ok(axis)
    }

    fn lock(&self) -> MutexGuard<'_, PIC862Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for PIC862Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.lock().send(&format!(
            "SV{},SA{},MA{}",
            velocity as i32, acceleration as i32, position as i32
        ))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.lock().send(&format!(
            "SV{},SA{},MR{}",
            velocity as i32, acceleration as i32, distance as i32
        ))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C-862 has no jog command: set the (unsigned) jog speed, then move
        // absolute to whichever record soft limit the velocity sign selects.
        let target = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        }
        .ok_or_else(|| pic862_err("PIC862: jog needs the record soft limits (none set yet)"))?;
        // C's jog path issues SET_ACCEL unconditionally before the move
        // (motorRecord.cc:2141, no accel>0 guard — unlike the ordinary move
        // path), so the jog acceleration is always on the wire: `SA,SV,MA`.
        self.lock().send(&format!(
            "SA{},SV{},MA{}",
            acceleration as i32,
            velocity.abs() as i32,
            target as i32
        ))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let cmd = if forward { "FE0" } else { "FE1" };
        self.lock().send(&format!(
            "SV{},SA{},{cmd}",
            velocity as i32, acceleration as i32
        ))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.lock().send("ST")
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `LOAD_POS`: only a *truncated* value of exactly 0 is accepted.
        if position as i32 != 0 {
            return Err(pic862_err("PIC862: only position 0 can be loaded (DH)"));
        }
        self.lock().send("DH")
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.lock().send(if enable { "MN" } else { "MF" })
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C: trans->state = IDLE_STATE — no wire command, record-side cache only.
        self.high_limit = Some(position);
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.low_limit = Some(position);
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let v = (32767.0 * gain) as i32;
        let cmd = match kind {
            PidGainKind::Proportional => format!("DP{v}"),
            PidGainKind::Integral => format!("DI{v}"),
            PidGainKind::Derivative => format!("DD{v}"),
        };
        self.lock().send(&cmd)
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let mut ctrl = self.lock();
        let polled = ctrl.poll_status();
        let comm_err = ctrl.comm_state == CommState::CommErr;
        drop(ctrl);

        let Some(polled) = polled else {
            if comm_err {
                self.last_status.comms_error = true;
                self.last_status.problem = true;
            }
            return Ok(self.last_status.clone());
        };
        self.last_status = status_from_polled(&polled);
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_ts_reply_reads_six_hex_registers() {
        assert_eq!(
            parse_ts_reply("S:04 00 04 02 0c 00"),
            Some([0x04, 0x00, 0x04, 0x02, 0x0c, 0x00])
        );
        assert_eq!(parse_ts_reply("S:04 00"), None);
        assert_eq!(parse_ts_reply("garbage"), None);
    }

    #[test]
    fn decode_regs_extracts_documented_bits() {
        // reg1: trty_done (0x04) set, motor_off (0x80) clear -> done, powered.
        // reg3: mvdir_pol (0x04) set -> plus_dir.
        // reg4: lmt_high (0x02) set -> active-high limits.
        // reg5: plus_ls (0x04) and minus_ls (0x08) both set.
        let regs = [0x04, 0x00, 0x04, 0x02, 0x0c, 0x00];
        let bits = decode_regs(&regs);
        assert!(bits.done);
        assert!(bits.powered);
        assert!(bits.plus_dir);
        assert!(bits.lmt_active_high);
        assert!(bits.plus_ls_raw);
        assert!(bits.minus_ls_raw);
    }

    #[test]
    fn decode_regs_motor_off_clears_powered() {
        let regs = [0x80, 0x00, 0x00, 0x00, 0x00, 0x00];
        assert!(!decode_regs(&regs).powered);
    }

    #[test]
    fn limit_polarity_inverts_when_not_active_high() {
        // lmt_high clear -> raw bits are inverted before reporting.
        let bits = StatusBits {
            done: false,
            powered: true,
            plus_dir: false,
            lmt_active_high: false,
            plus_ls_raw: false,
            minus_ls_raw: true,
        };
        let (high, low) = if bits.lmt_active_high {
            (bits.plus_ls_raw, bits.minus_ls_raw)
        } else {
            (!bits.plus_ls_raw, !bits.minus_ls_raw)
        };
        assert!(high); // raw false -> inverted true
        assert!(!low); // raw true -> inverted false
    }

    #[test]
    fn tp_position_skips_two_char_prefix() {
        assert_eq!(parse_value_at("P:1234", 2), Some(1234.0));
        assert_eq!(parse_value_at("P:-500", 2), Some(-500.0));
    }

    #[test]
    fn tp_position_rounds_to_nearest_not_truncated() {
        // C `NINT(atof(&buff[2]))` rounds; a bare truncating cast would give
        // 1234/-500 here instead of 1235/-501.
        assert_eq!(nint(parse_value_at("P:1234.6", 2).unwrap()), 1235);
        assert_eq!(nint(parse_value_at("P:-500.6", 2).unwrap()), -501);
    }

    #[test]
    fn set_position_truncates_before_the_zero_check() {
        // C `cntrl_units = (int) dval;` truncates toward zero, so 0.4 and
        // -0.4 both pass the `cntrl_units == 0` check.
        assert_eq!(0.4_f64 as i32, 0);
        assert_eq!(-0.4_f64 as i32, 0);
        assert_ne!(1.0_f64 as i32, 0);
    }

    #[test]
    fn status_from_polled_reports_fixed_fields() {
        let polled = PolledStatus {
            position: 42.0,
            done: true,
            powered: true,
            direction: true,
            high_limit: false,
            low_limit: false,
        };
        let status = status_from_polled(&polled);
        assert_eq!(status.position, 42.0);
        assert_eq!(status.encoder_position, 42.0);
        assert_eq!(status.velocity, 0.0);
        assert!(status.done);
        assert!(!status.moving);
        assert!(status.has_encoder);
        assert!(status.gain_support);
        assert!(!status.vbas_supported);
        assert!(!status.problem);
        assert!(!status.comms_error);
    }
}
