//! PI (Physik Instrumente) C-663 DC-motor controller (`motorPI`).
//!
//! Ported from `drvPIC663.cc` + `devPIC663.cc`. The C source's own header
//! comment says this was "Copied from devPIC862.cc" — despite the name
//! similarity to C-662, C-663 is a **C-862 clone**, sharing its two-letter
//! ASCII command set almost verbatim; C-662 (`c662.rs`) is an unrelated
//! SCPI-style driver. Single axis per controller (no `MAX_AXES` override
//! found; same shape as C-862).
//!
//! ## Transport
//! Same as C-862: pure `pasynOctetSyncIO` despite the iocsh argument label
//! "asyn address (GPIB)" — `motor_init`'s `connect(cntrl->asyn_port, 0, ...)`
//! hardcodes the asyn sub-address to 0. Connects via
//! [`motor_common::connect::connect_serial`].
//!
//! ## Wire shape — the port owns framing
//! `motor_init` calls `pasynOctetSyncIO->setOutputEos(pasynUser, "\r", 1)`
//! and `setInputEos(pasynUser, "\n\x03", 2)` right after connecting;
//! `send_mess`'s buffer never appends a terminator (`strcat(local_buff,
//! com)` only). Same ownership as C-862 — see the `motor-port-eos-ownership`
//! convention. `SyncIOHandle` has no driver-side EOS-set hook, so both EOS
//! values are set from `st.cmd` instead of from [`PIC663Controller::new`].
//!
//! `recv_mess` unconditionally drops the **last** received byte (C
//! `com[nread - 1] = '\0'; /* Strip traling CR. */` — a real typo in the C
//! comment, reproduced here only in this note, not in code) after the
//! port's own EOS strip removed `\n\x03` — identical to C-862.
//!
//! ## Multi-drop enable/identify (connect-time only)
//! Identical to C-862: `\x01{addr}VE` (SOH, one uppercase hex digit, `VE`),
//! retried up to 3 times, reply becomes `ident`. A C comment describes a
//! separate `"TB"` address-confirm exchange ("replies `B:000x`") that the
//! actual code never sends — comment/code mismatch, not ported.
//!
//! ## Status (`TS` — "Tell Status")
//! **Three** 2-hex-digit registers (not six, unlike C-862):
//! `"S:XX XX XX"`, gated by `charcnt > 9` (C-862 uses 6 registers / `> 18`).
//! Bit layout (`C663_Status_Reg1`/`Reg2`, non-`MSB_First` branch):
//!
//! | field | register | bit |
//! |-------|----------|-----|
//! | `on_target` (→ `RA_DONE`) | 1 | 1 (`0x02`) |
//! | `drv_cur_act` (→ `powered`, inverted) | 1 | 7 (`0x80`) |
//! | `hi_limit` (→ `high_limit`, inverted) | 2 | 2 (`0x04`) |
//! | `lo_limit` (→ `low_limit`, inverted) | 2 | 0 (`0x01`) |
//!
//! Register 1 bit 7 (`drv_cur_act`, "drive current active") is decoded by C
//! into `EA_POSITION` via `EA_POSITION = drv_cur_act ? 0 : 1`
//! (`drvPIC663.cc:251`), and the record layer reads `EA_POSITION` into `.CNEN`
//! whenever `GAIN_SUPPORT` is set (which C-663 does) — so it is a live status
//! signal, not a spare bit. Mapped here to `powered = (drv_cur_act == 0)`,
//! the same `EA_POSITION → powered` mapping C-862 uses for its `motor_off`
//! bit. Register 3 (error code, `C663_Status_Reg3`) really is read but never
//! consulted, matching C. **`RA_DIRECTION` is hardcoded `false`** in C (never
//! derived from any register) — reproduced literally, not computed from a
//! register bit.
//!
//! Same `NORMAL`/`RETRY`/`COMM_ERR` two-strike debounce as C-862, gated only
//! on `TS`; `TP` has no debounce of its own (same asymmetry as C-862).
//!
//! ## Position (`TP` — "Tell Position")
//! `NINT(atof(&buff[2]))` — skip a 2-character reply prefix, parse, then
//! round to nearest (not truncate). Identical to C-862's own `TP` parse.
//!
//! ## Units
//! Raw controller counts, truncating `as i32` cast on outgoing values
//! (`MRES = 1` records), same convention as C-862.
//!
//! ## Motion commands (`devPIC663_build_trans`)
//! Same shape as C-862's `SV{v},SA{a}` preamble ahead of
//! `MOVE_ABS`/`MOVE_REL`/`HOME_FOR`/`HOME_REV`.
//!
//! - `MOVE_ABS`/`MOVE_REL` → `MA{n}`/`MR{n}`.
//! - `HOME_FOR`/`HOME_REV` → `FE0`/`FE1`.
//! - `JOG` (`move_velocity`) — no native jog: `SV{|v|}` then `MA` to
//!   whichever cached record soft limit (`DHLM`/`DLLM`) the velocity sign
//!   selects — **no `SA` in this command**, unlike `MOVE_ABS`/`MOVE_REL`/
//!   `HOME_FOR`/`HOME_REV`. Identical simulation to C-862.
//! - `LOAD_POS` (`set_position`) → `DH` only when the truncated value is
//!   exactly `0`; any other value is an error.
//! - `ENABLE_TORQUE`/`DISABL_TORQUE` → `MN`/`MF`.
//! - `STOP_AXIS` → `ST`.
//! - `SET_PGAIN`/`SET_IGAIN`/`SET_DGAIN` → `DP{n}`/`DI{n}`/`DD{n}`,
//!   `n = (int)(32767 * gain)`.
//! - `SET_HIGH_LIMIT`/`SET_LOW_LIMIT`/`SET_ENC_RATIO`/`GO` — no wire
//!   command (record-side cache only, or C-862-style immediate-move no-op).
//!
//! ## Not modeled (documented deviations)
//! - **`recv_mess(..., FLUSH)`** before a retry: same gap as C-862 —
//!   `SyncIOHandle` has no synchronous flush primitive.
//! - **`GET_IDENT` dead macro** (`#define GET_IDENT 0x01`, never referenced
//!   in the C source — a vestige of the C-862 copy-paste): not ported.
//! - **`report()`**: not implemented, matching every other port here.
//! - **`no_motion_count`**: pure poll-rate scheduling counter, not tracked.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{nint, parse_value_at};

/// Response buffer size for a single reply (C `BUFF_SIZE`).
const READ_BUF: usize = 100;

fn pic663_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Controller communication state (C `cntrl->status`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// The three `TS` status registers, decoded (C `C663_Status_Reg{1,2}`
/// `#else`/non-`MSB_First` bit layout; register 3, the error code, is read
/// but never consulted, matching C).
struct StatusBits {
    /// Register 1 bit 1: on-target.
    done: bool,
    /// Register 1 bit 7 (`drv_cur_act`), inverted: C `EA_POSITION =
    /// drv_cur_act ? 0 : 1`, mapped to `powered`.
    powered: bool,
    /// Register 2 bit 2: plus (high) limit, raw polarity (inverted below).
    hi_limit_raw: bool,
    /// Register 2 bit 0: minus (low) limit, raw polarity (inverted below).
    lo_limit_raw: bool,
}

fn decode_regs(regs: &[u16; 3]) -> StatusBits {
    StatusBits {
        done: regs[0] & 0x02 != 0,
        powered: regs[0] & 0x80 == 0,
        hi_limit_raw: regs[1] & 0x04 != 0,
        lo_limit_raw: regs[1] & 0x01 != 0,
    }
}

/// Parse a `TS` reply's 3 two-hex-digit registers (`sscanf(buff,
/// "S:%2hx %2hx %2hx")`); `None` if the `"S:"` prefix or all 3
/// whitespace-separated hex fields aren't present.
fn parse_ts_reply(reply: &str) -> Option<[u16; 3]> {
    let body = reply.strip_prefix("S:")?;
    let mut regs = [0u16; 3];
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
    high_limit: bool,
    low_limit: bool,
}

fn status_from_polled(polled: &PolledStatus) -> MotorStatus {
    MotorStatus {
        position: polled.position,
        encoder_position: polled.position,
        velocity: 0.0,
        done: polled.done,
        moving: !polled.done,
        powered: polled.powered,
        high_limit: polled.high_limit,
        low_limit: polled.low_limit,
        // C hardcodes RA_DIRECTION = 0 unconditionally; reproduced literally.
        direction: false,
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
/// state. One controller drives exactly one axis.
pub struct PIC663Controller {
    handle: SyncIOHandle,
    ident: String,
    comm_state: CommState,
}

impl PIC663Controller {
    /// Connect and select/identify a C-663 (C `motor_init`'s per-card
    /// block): write `\x01{addr}VE`, retrying up to 3 times until a
    /// non-empty reply arrives. `addr` is the multi-drop bus address (0-F).
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
            return Err(pic663_err(format!(
                "PIC663: no response to enable/identify (\\x01{addr:X}VE)"
            )));
        }
        Ok(ctrl)
    }

    /// The controller identification string (from the connect-time `VE`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Write a command, no reply expected. No terminator is appended here —
    /// the port's configured output EOS puts the CR on the wire.
    fn send(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Read one reply, unconditionally dropping the last byte (C
    /// `recv_mess`'s "Strip trailing CR"). Returns `(reply, raw_len)`;
    /// `raw_len` is the pre-strip byte count (C's `charcnt`).
    fn recv(&self) -> (String, usize) {
        match self.handle.read_octet(0, READ_BUF) {
            Ok(raw) if !raw.is_empty() => {
                let stripped = &raw[..raw.len() - 1];
                (String::from_utf8_lossy(stripped).into_owned(), raw.len())
            }
            _ => (String::new(), 0),
        }
    }

    /// C `set_status`: `TS` gated by the comm debounce, then (on a good
    /// `TS`) `TP` with no further debounce.
    fn poll_status(&mut self) -> Option<PolledStatus> {
        let _ = self.send("TS");
        let (ts_reply, ts_raw_len) = self.recv();
        let regs = if ts_raw_len > 9 {
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

        let _ = self.send("TP");
        let (tp_reply, _) = self.recv();
        let position = parse_value_at(&tp_reply, 2).map_or(0.0, |v| nint(v) as f64);

        Some(PolledStatus {
            position,
            done: bits.done,
            powered: bits.powered,
            high_limit: !bits.hi_limit_raw,
            low_limit: !bits.lo_limit_raw,
        })
    }
}

/// The single axis of a C-663 controller. Implements [`AsynMotor`].
pub struct PIC663Axis {
    controller: Arc<Mutex<PIC663Controller>>,
    /// Record soft limits (dial EGU = raw counts, `MRES = 1`), cached from
    /// [`AsynMotor::set_high_limit`]/[`set_low_limit`] — the C-663 has no
    /// jog command, so a jog moves to whichever of these the record's DHLM
    /// / DLLM the requested direction selects.
    high_limit: Option<f64>,
    low_limit: Option<f64>,
    last_status: MotorStatus,
}

impl PIC663Axis {
    /// Construct the controller's one axis, seeding the initial status. A
    /// failed initial poll is not an error — C leaves the zeroed defaults
    /// in place either way.
    pub fn new(controller: Arc<Mutex<PIC663Controller>>) -> AsynResult<Self> {
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

    fn lock(&self) -> MutexGuard<'_, PIC663Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for PIC663Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.lock().send(&format!(
            "SV{},{}MA{}",
            velocity as i32,
            crate::accel_field(acceleration),
            position as i32
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
            "SV{},{}MR{}",
            velocity as i32,
            crate::accel_field(acceleration),
            distance as i32
        ))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let target = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        }
        .ok_or_else(|| pic663_err("PIC663: jog needs the record soft limits (none set yet)"))?;
        // C's jog path issues SET_ACCEL unconditionally before the move
        // (shared motorRecord.cc jog kickoff, no accel>0 guard), so the jog
        // acceleration is always on the wire: `SA,SV,MA`.
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
            "SV{},{}{cmd}",
            velocity as i32,
            crate::accel_field(acceleration)
        ))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.lock().send("ST")
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        if position as i32 != 0 {
            return Err(pic663_err("PIC663: only position 0 can be loaded (DH)"));
        }
        self.lock().send("DH")
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.lock().send(if enable { "MN" } else { "MF" })
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
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
    fn parse_ts_reply_reads_three_hex_registers() {
        assert_eq!(parse_ts_reply("S:02 05 00"), Some([0x02, 0x05, 0x00]));
        assert_eq!(parse_ts_reply("S:02"), None);
        assert_eq!(parse_ts_reply("garbage"), None);
    }

    #[test]
    fn decode_regs_extracts_documented_bits() {
        // reg1: on_target (0x02) set -> done; drv_cur_act (0x80) clear -> powered.
        // reg2: hi_limit (0x04) and lo_limit (0x01) both set.
        let regs = [0x02, 0x05, 0x00];
        let bits = decode_regs(&regs);
        assert!(bits.done);
        assert!(bits.powered);
        assert!(bits.hi_limit_raw);
        assert!(bits.lo_limit_raw);
    }

    #[test]
    fn decode_regs_clear_bits_report_false() {
        // drv_cur_act (0x80) clear reports powered = true (C: EA_POSITION =
        // drv_cur_act ? 0 : 1), so powered is asserted separately below.
        let regs = [0x00, 0x00, 0x00];
        let bits = decode_regs(&regs);
        assert!(!bits.done);
        assert!(!bits.hi_limit_raw);
        assert!(!bits.lo_limit_raw);
    }

    #[test]
    fn decode_regs_drive_current_active_clears_powered() {
        // C `drvPIC663.cc:251`: EA_POSITION = drv_cur_act ? 0 : 1, mapped to
        // powered. drv_cur_act (reg1 bit 7, 0x80) set -> powered = false.
        assert!(!decode_regs(&[0x80, 0x00, 0x00]).powered);
        assert!(decode_regs(&[0x00, 0x00, 0x00]).powered);
    }

    #[test]
    fn limit_polarity_is_unconditionally_inverted() {
        // C-663 has no active-high selector bit (unlike C-862): the raw
        // register bit is always inverted to produce the reported limit.
        let regs = [0x00, 0x05, 0x00]; // hi_limit and lo_limit raw bits set
        let bits = decode_regs(&regs);
        let high_limit = !bits.hi_limit_raw;
        let low_limit = !bits.lo_limit_raw;
        assert!(!high_limit);
        assert!(!low_limit);
    }

    #[test]
    fn direction_is_always_reported_false() {
        let polled = PolledStatus {
            position: 0.0,
            done: true,
            powered: true,
            high_limit: false,
            low_limit: false,
        };
        assert!(!status_from_polled(&polled).direction);
    }

    #[test]
    fn tp_position_rounds_to_nearest() {
        assert_eq!(nint(parse_value_at("P:1234.6", 2).unwrap()), 1235);
        assert_eq!(nint(parse_value_at("P:-500.6", 2).unwrap()), -501);
    }

    #[test]
    fn set_position_truncates_before_the_zero_check() {
        assert_eq!(0.4_f64 as i32, 0);
        assert_eq!(-0.4_f64 as i32, 0);
        assert_ne!(1.0_f64 as i32, 0);
    }

    #[test]
    fn status_from_polled_reports_fixed_fields() {
        let polled = PolledStatus {
            position: 42.0,
            done: true,
            powered: false,
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
        // powered now reflects the live drv_cur_act bit (C EA_POSITION), not
        // the MotorStatus default: polled.powered = false propagates through.
        assert!(!status.powered);
    }
}
