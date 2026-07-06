//! PI (Physik Instrumente) C-630 stepper controller (`motorPI`).
//!
//! Ported from `drvPIC630.cc` + `devPIC630.cc` (a model-1 dev/drv pair). The
//! C-630 is a 3-axis stepper controller/driver; up to 3 C-630s can be
//! daisy-chained on one serial port (the C code calls one such chain a
//! "card"), giving up to **9 axes per serial port**, addressed `1`-`9`. This
//! port keeps the same shape: one [`PIC630Controller`] owns the octet handle
//! for one chain, and one [`PIC630Axis`] per address (`1`-`9`) shares it.
//!
//! ## Transport / addressing — real multi-drop, per-command address prefix
//!
//! Unlike the C-862 (whose "address" is a connect-time-only bus selector), the
//! C-630 embeds a **1-digit axis address on every command**: `motor_init`,
//! `set_status` and `devPIC630_build_trans` all `sprintf("%d%s", axis, ...)`
//! (e.g. `1TS`, `2MA1000`). The address is `signal + 1` (device support's
//! `axis++`), so `signal` 0-8 maps to wire address `1`-`9`. The driver never
//! uses an asyn sub-address (`pasynOctetSyncIO->connect(..., 0, ...)` and the
//! driver table's `axis_names` field is `NULL`), so this port connects via
//! [`motor_common::connect::connect_serial`] like every other octet driver.
//!
//! ## Wire shape — the port owns framing, and the device echoes
//!
//! `motor_init` sets `setOutputEos("\n")` **and** `setInputEos("\n")` right
//! after connecting, and `send_mess` never appends a terminator itself
//! (`strcpy(buff, com)` only) — so the port's configured output EOS is what
//! puts the trailing `\n` on the wire. This port has no `SyncIOHandle` hook to
//! set EOS from Rust, so both are pushed into `st.cmd`
//! (`asynOctetSetInputEos`/`asynOctetSetOutputEos`, both `"\n"`), and
//! [`PIC630Controller::send`] writes **bare** command bytes.
//!
//! The reference `PI_C630.iocsh` sets the port EOS to `"\r"` *before* iocInit,
//! but `motor_init` (which runs at iocInit) overwrites both to `"\n"`; the
//! `"\n"` is therefore what is actually on the wire, and this port mirrors that
//! final state, not the transient `"\r"`.
//!
//! Critically, the C-630 **echoes every command it receives**: `send_mess`
//! writes the command and then immediately `recv_mess`'s that echo and
//! discards it ("This thing always echos everything sent to it. Read this
//! response."). [`PIC630Controller::send`] reproduces this: every write is
//! followed by one discarded read. A *query* (`TS`/`TP`) therefore reads
//! twice — the echo (in `send`) then the data reply (in
//! [`PIC630Controller::query`]).
//!
//! ## Status (`{addr}TS` — "Tell Status")
//!
//! Reply `"{addr}TS:N"`, `N` a decimal 0-255 (`cStatus = (char)
//! atoi(&response[4])`, skipping the 4-char `"nTS:"` echo prefix). Bits:
//!
//! | bit | mask | meaning |
//! |-----|------|---------|
//! | 0 | `0x01` | moving (`RA_DONE = !moving`) |
//! | 2 | `0x04` | +limit, **0 = tripped** (`RA_PLUS_LS = !(bit)`) |
//! | 3 | `0x08` | -limit, **0 = tripped** (`RA_MINUS_LS = !(bit)`) |
//!
//! (Bits 1/4/5/6/7 — reference flag, command error, profile error, E-stop —
//! are documented in C but not consulted by `set_status`; dropped here too.)
//!
//! **No comm-error debounce.** Unlike the C-862/C-844, `drvPIC630`'s
//! `set_status` has *no* NORMAL/RETRY/COMM_ERR handling — it parses whatever
//! `recv_mess` leaves in the buffer (`atoi(&response[4])` even on a failed
//! read). This is the "press on with whatever came back" convention; a failed
//! read parses as `0` here (`atoi` of an empty region), so a dead link reports
//! `done` + both limits tripped, and `comms_error` is never set. Mirrored
//! as-is.
//!
//! ## Position (`{addr}TP` — "Tell Position")
//! `motorData = atoi(&response[4])` — skip the 4-char `"nTP:"` prefix and parse
//! the **integer** count directly (C `atoi`, *not* `atof`+`NINT`). Direction is
//! derived from the sign of the position delta versus the previous poll, held
//! across polls when the position is unchanged (C compares against
//! `motor_info->position`).
//!
//! ## Units
//! Raw controller counts throughout (`MA`/`MR`/`TP` are integer counts); pair
//! with `MRES = 1` records. Move/home targets cross the `AsynMotor` boundary
//! through C `NINT` rounding (device support `ival = NINT(dval)`), while the
//! velocity/accel clamps below match `devPIC630_build_trans` exactly.
//!
//! ## Motion (`devPIC630_build_trans`) — separate messages per command
//!
//! `devPIC630_build_trans` self-brackets every command
//! (`motor_start_trans_com` … `motor_end_trans_com` inside a single call), so
//! the record's move preamble is sent as **separate writes**, not one combined
//! command: `SET_VELOCITY`, then `SET_ACCEL` (record guard `accel > 0`), then
//! the move — each its own `send` (and thus its own echo read).
//!
//! - `MOVE_ABS`/`MOVE_REL` → `{addr}MA{n}` / `{addr}MR{n}`, `n = NINT(pos)`.
//! - `HOME_FOR`/`HOME_REV` → `{addr}MA0` (both; "home = move to 0").
//! - `SET_VELOCITY` → `{addr}SV{v}`, `v = NINT(vel)` clamped to `[1, 200000]`.
//! - `SET_ACCEL` → `{addr}SA{a}`, `a = NINT(acc)` clamped to `[0, 500000]`
//!   (move/home: only when `acc > 0`; jog: unconditional, per motorRecord).
//! - `JOG` (`move_velocity`) — the C-630 has no jog command: `{addr}SV{|v|}`
//!   (clamped `[1, 200000]`) then `{addr}MA{limit}` to whichever record soft
//!   limit (`DHLM`/`DLLM`, cached from
//!   [`set_high_limit`](AsynMotor::set_high_limit) /
//!   [`set_low_limit`](AsynMotor::set_low_limit); `MRES = 1` so dial EGU =
//!   counts) the velocity sign selects, preceded by an unconditional
//!   `{addr}SA{a}` (motorRecord jog issues `SET_ACCEL` with no `acc > 0`
//!   guard).
//! - `LOAD_POS` (`set_position`) → `{addr}DH`, but **only** for an exact
//!   `0.0` (C `if (dval == 0.0)`; note this is an exact float test, *not* the
//!   truncated one the C-862 uses); any other value is an error.
//! - `STOP_AXIS` (`stop`) → `{addr}ST`.
//! - `ENABLE_TORQUE`/`DISABL_TORQUE`/`SET_PGAIN`/`SET_IGAIN`/`SET_DGAIN` — C
//!   builds no message (empty `send_mess`, a no-op); mirrored as no-ops.
//! - `GO`/`SET_ENC_RATIO`/`SET_HIGH_LIMIT`/`SET_LOW_LIMIT` — no wire command.
//!
//! ## Connect (`PIC630Config` + `motor_init`)
//! `PIC630Config` records a per-axis drive current (`0`=OFF, `1`=100 mA, …
//! `8`=800 mA). `motor_init` probes each axis with `{addr}TS` (a shared
//! 3-strike retry budget across all axes — a C quirk reproduced in
//! [`PIC630Controller::new`]); if any configured axis never answers the whole
//! chain fails. It then, per axis, sends `{addr}DC{current}` (set current) and
//! `{addr}AB` (stop) before the initial poll — done in
//! [`PIC630Axis::new`] so the per-axis order matches C.
//!
//! ## Not modeled (documented deviations)
//! - **`no_motion_count`** poll-scheduling counter: model-1 record-layer state
//!   (gated on an active `motor_motion` node this interface does not carry);
//!   the position it guards is set identically either way, so this port always
//!   updates `position` from `TP` — matching the C-862 port's treatment.
//! - **`recv_mess(..., FLUSH)`** on the (nonexistent) comm-error path and
//!   **`report()`** iocsh diagnostic: no `SyncIOHandle` equivalent / not
//!   implemented, matching every other port here.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{nint, parse_int_at};

/// Response buffer size for a single reply (C `BUFF_SIZE`).
const READ_BUF: usize = 100;

/// Maximum axes per chain (C `PIC630_NUM_AXIS`).
pub const PIC630_NUM_AXIS: u8 = 9;

fn pic630_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint for one C-630 chain: owns the octet handle. Each
/// axis prefixes its own 1-digit address (`1`-`9`) onto every command, so the
/// controller itself is address-agnostic.
pub struct PIC630Controller {
    handle: SyncIOHandle,
}

impl PIC630Controller {
    /// Connect and probe a C-630 chain (C `motor_init`'s per-card block):
    /// probe each axis `1..=num_axes` with `{addr}TS`. The C retry budget
    /// (`int retry = 0;` declared *once*, before the axis loop, and the
    /// `while (status == 0 && retry < 3)`) is **shared across all axes** — a
    /// C quirk reproduced here: each axis is probed at least once, re-probed
    /// only while the cumulative attempt count is `< 3`. If any probed axis
    /// never answers the whole chain fails (C `if (status == 0) break;` then
    /// the `status > 0` gate).
    pub fn new(handle: SyncIOHandle, num_axes: u8) -> AsynResult<Self> {
        let ctrl = Self { handle };

        let mut retry: u32 = 0;
        let mut last_ok = false;
        for addr in 1..=num_axes {
            let mut reply;
            loop {
                ctrl.send(&format!("{addr}TS"))?;
                reply = ctrl.recv();
                retry += 1;
                if !(reply.is_empty() && retry < 3) {
                    break;
                }
            }
            last_ok = !reply.is_empty();
            if reply.is_empty() {
                break;
            }
        }
        if !last_ok {
            return Err(pic630_err(
                "PIC630: no response to {addr}TS probe (controller off or miswired)",
            ));
        }
        Ok(ctrl)
    }

    /// Write a command and read back the device's echo, discarding it (C
    /// `send_mess`: "This thing always echos everything sent to it. Read this
    /// response."). No terminator is appended — the port's output EOS puts the
    /// `\n` on the wire (see module docs). An empty command is a no-op, like
    /// C's `if (strlen(com) == 0) return OK;`.
    fn send(&self, cmd: &str) -> AsynResult<()> {
        if cmd.is_empty() {
            return Ok(());
        }
        self.handle.write_octet(0, cmd.as_bytes())?;
        let _ = self.recv(); // consume + discard the echo
        Ok(())
    }

    /// Read one EOS-stripped reply, folding any transport failure or empty read
    /// into `""` (C `recv_mess`'s `if (nread < 1) com[0] = '\0'`). The C-630
    /// does *not* strip an extra trailing byte (unlike the C-862).
    fn recv(&self) -> String {
        match self.handle.read_octet(0, READ_BUF) {
            Ok(raw) if !raw.is_empty() => String::from_utf8_lossy(&raw).into_owned(),
            _ => String::new(),
        }
    }

    /// Send a query and read its data reply. `send` already consumed the echo,
    /// so this second read is the actual `{addr}TS:N` / `{addr}TP:N` response.
    fn query(&self, cmd: &str) -> String {
        let _ = self.send(cmd);
        self.recv()
    }
}

/// Decoded result of one `{addr}TS` + `{addr}TP` poll.
struct PolledStatus {
    position: i32,
    done: bool,
    high_limit: bool,
    low_limit: bool,
}

/// One addressed axis (`1`-`9`) of a C-630 chain. Implements [`AsynMotor`].
pub struct PIC630Axis {
    controller: Arc<Mutex<PIC630Controller>>,
    /// Wire address (`signal + 1`, i.e. `1`-`9`).
    addr: u8,
    /// Record soft limits (dial EGU = raw counts, `MRES = 1`), cached for the
    /// jog simulation (C-630 has no jog command).
    high_limit: Option<f64>,
    low_limit: Option<f64>,
    last_status: MotorStatus,
}

impl PIC630Axis {
    /// Construct one axis: send `{addr}DC{current}` (set drive current) then
    /// `{addr}AB` (stop), then seed the initial status — matching the per-axis
    /// order of C `motor_init`'s init loop. `signal` is 0-based; wire address
    /// is `signal + 1`. A failed initial poll is not an error (C leaves the
    /// zeroed defaults in place).
    pub fn new(
        controller: Arc<Mutex<PIC630Controller>>,
        signal: u8,
        current: i32,
    ) -> AsynResult<Self> {
        let addr = signal + 1;
        let mut axis = Self {
            controller: controller.clone(),
            addr,
            high_limit: None,
            low_limit: None,
            last_status: MotorStatus {
                done: true,
                ..MotorStatus::default()
            },
        };

        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        ctrl.send(&format!("{addr}DC{current}"))?;
        ctrl.send(&format!("{addr}AB"))?;
        let polled = axis.poll_locked(&ctrl);
        drop(ctrl);
        axis.apply(polled);
        Ok(axis)
    }

    fn lock(&self) -> MutexGuard<'_, PIC630Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `set_status`: `{addr}TS` then `{addr}TP`, no comm-error debounce
    /// (press on with whatever the buffer holds; a failed read parses as 0).
    fn poll_locked(&self, ctrl: &PIC630Controller) -> PolledStatus {
        let ts = ctrl.query(&format!("{}TS", self.addr));
        // `cStatus = (char) atoi(&response[4])`; skip the "nTS:" echo prefix.
        let c_status = (parse_int_at(&ts, 4).unwrap_or(0) & 0xFF) as u8;
        let done = c_status & 0x01 == 0;
        // "0 = limit tripped": the limit is active when the bit is clear.
        let high_limit = c_status & 0x04 == 0;
        let low_limit = c_status & 0x08 == 0;

        let tp = ctrl.query(&format!("{}TP", self.addr));
        let position = parse_int_at(&tp, 4).unwrap_or(0);

        PolledStatus {
            position,
            done,
            high_limit,
            low_limit,
        }
    }

    /// Fold a fresh poll into `last_status`, deriving direction from the
    /// position delta (held across polls when the position is unchanged, C
    /// `motorData >= motor_info->position` guarded by `motorData != position`).
    fn apply(&mut self, polled: PolledStatus) {
        let prev = self.last_status.position as i32;
        let direction = if polled.position != prev {
            polled.position >= prev
        } else {
            self.last_status.direction
        };
        self.last_status = MotorStatus {
            position: polled.position as f64,
            encoder_position: polled.position as f64,
            velocity: 0.0,
            done: polled.done,
            moving: !polled.done,
            high_limit: polled.high_limit,
            low_limit: polled.low_limit,
            direction,
            // C-630 never reports powered/encoder/gain: DC-stepper, no such
            // status bits; leave `powered` at its default (true) and the
            // capability flags off. No base velocity (`SET_VEL_BASE` no-op).
            powered: true,
            gain_support: false,
            has_encoder: false,
            vbas_supported: false,
            ..MotorStatus::default()
        };
    }

    /// `SET_VELOCITY` fragment: `NINT(vel)` clamped to `[1, 200000]`.
    fn sv(velocity: f64) -> i32 {
        nint(velocity).clamp(1, 200_000)
    }

    /// `SET_ACCEL` fragment: `NINT(acc)` clamped to `[0, 500000]`.
    fn sa(acceleration: f64) -> i32 {
        nint(acceleration).clamp(0, 500_000)
    }
}

impl AsynMotor for PIC630Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let addr = self.addr;
        let ctrl = self.lock();
        ctrl.send(&format!("{addr}SV{}", Self::sv(velocity)))?;
        if acceleration > 0.0 {
            ctrl.send(&format!("{addr}SA{}", Self::sa(acceleration)))?;
        }
        ctrl.send(&format!("{addr}MA{}", nint(position)))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let addr = self.addr;
        let ctrl = self.lock();
        ctrl.send(&format!("{addr}SV{}", Self::sv(velocity)))?;
        if acceleration > 0.0 {
            ctrl.send(&format!("{addr}SA{}", Self::sa(acceleration)))?;
        }
        ctrl.send(&format!("{addr}MR{}", nint(distance)))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // No jog command: SET_ACCEL (unconditional, per motorRecord jog),
        // then the C-630 JOG case's own SV + MA-to-soft-limit simulation.
        let target = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        }
        .ok_or_else(|| pic630_err("PIC630: jog needs the record soft limits (none set yet)"))?;
        let addr = self.addr;
        let ctrl = self.lock();
        ctrl.send(&format!("{addr}SA{}", Self::sa(acceleration)))?;
        ctrl.send(&format!("{addr}SV{}", Self::sv(velocity.abs())))?;
        ctrl.send(&format!("{addr}MA{}", nint(target)))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // HOME_FOR and HOME_REV both map to "{addr}MA0" (home = move to 0).
        let addr = self.addr;
        let ctrl = self.lock();
        ctrl.send(&format!("{addr}SV{}", Self::sv(velocity)))?;
        if acceleration > 0.0 {
            ctrl.send(&format!("{addr}SA{}", Self::sa(acceleration)))?;
        }
        ctrl.send(&format!("{addr}MA0"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let addr = self.addr;
        self.lock().send(&format!("{addr}ST"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `LOAD_POS`: only an exact 0.0 is accepted (`if (dval == 0.0)`).
        if position != 0.0 {
            return Err(pic630_err("PIC630: only position 0 can be loaded (DH)"));
        }
        let addr = self.addr;
        self.lock().send(&format!("{addr}DH"))
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

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let polled = self.poll_locked(&ctrl);
        drop(ctrl);
        self.apply(polled);
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_decode_matches_c_bit_layout() {
        // cStatus = 0x00: not moving (done), both limit bits clear -> both
        // "tripped" (active), since 0 = tripped.
        let c: u8 = 0x00;
        assert!(c & 0x01 == 0); // done
        assert!(c & 0x04 == 0); // +limit active
        assert!(c & 0x08 == 0); // -limit active

        // cStatus = 0x0D = 0b1101: moving (bit0), +limit clear? bit2=1 -> not
        // tripped; bit3=1 -> not tripped.
        let c: u8 = 0x0D;
        assert!(c & 0x01 != 0); // moving -> !done
        assert!(c & 0x04 != 0); // +limit NOT tripped
        assert!(c & 0x08 != 0); // -limit NOT tripped
    }

    #[test]
    fn status_prefix_offset_is_four() {
        // "1TS:5" -> &response[4] = "5"; "1TP:1000" -> "1000".
        assert_eq!(parse_int_at("1TS:5", 4), Some(5));
        assert_eq!(parse_int_at("1TP:1000", 4), Some(1000));
        assert_eq!(parse_int_at("2TP:-750", 4), Some(-750));
        // A too-short / empty reply parses as 0 (press-on, no comm error).
        assert_eq!(parse_int_at("", 4), None);
    }

    #[test]
    fn velocity_clamp_matches_c() {
        assert_eq!(PIC630Axis::sv(0.0), 1); // < 1 -> 1
        assert_eq!(PIC630Axis::sv(-5.0), 1);
        assert_eq!(PIC630Axis::sv(500.0), 500);
        assert_eq!(PIC630Axis::sv(999_999.0), 200_000); // > 200000 -> clamp
    }

    #[test]
    fn accel_clamp_matches_c() {
        assert_eq!(PIC630Axis::sa(-1.0), 0); // < 0 -> 0
        assert_eq!(PIC630Axis::sa(0.4), 0); // NINT(0.4) = 0
        assert_eq!(PIC630Axis::sa(500.0), 500);
        assert_eq!(PIC630Axis::sa(999_999.0), 500_000); // > 500000 -> clamp
    }

    #[test]
    fn move_target_uses_nint_not_truncation() {
        // Device support ival = NINT(dval): 1234.6 -> 1235, -500.6 -> -501.
        assert_eq!(nint(1234.6), 1235);
        assert_eq!(nint(-500.6), -501);
    }
}
