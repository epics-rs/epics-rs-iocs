//! PI (Physik Instrumente) C-848 DC-servo controller (`motorPI`).
//!
//! Ported from `drvPIC848.cc` + `devPIC848.cc` (a model-1 dev/drv pair,
//! itself copied from the C-844). The C-848 is a **multi-axis** (up to
//! `MAX_AXES = 4`) DC-servo controller with a GCS-like command set. One
//! [`PIC848Controller`] owns the octet handle and drives up to four
//! [`PIC848Axis`]es. Unlike the C-844, the axis count is **probed** at connect
//! (`CST?` until `NOSTAGE`).
//!
//! ## Transport / framing — port owns EOS, axis is byte 5 of every command
//!
//! `motor_init` sets `setOutputEos("\n")` **and** `setInputEos("\n")`;
//! `send_mess` writes bare bytes (no terminator, no echo — `cmnd_response =
//! false`), so the port's output EOS puts the `\n` on the wire (both set in
//! `st.cmd`).
//!
//! The axis selector is not a prefix (C-844) but a **single byte at index 5**
//! of a fixed-width command: `send_mess` does `local_buff[5] = *name` where
//! `name` is `PIC848_axis[signal]` = `"A"`/`"B"`/`"C"`/`"D"`. Every command is
//! authored with a `'#'` placeholder at column 5 (e.g. `"STA? #"`,
//! `"MOV  #%.*f"`), overwritten with the axis letter. This port formats the
//! letter directly into column 5 (`n = b'A' + signal`).
//!
//! **One message per command (differs from the C-844).** `PIC848_build_trans`
//! wraps *each* command in its own `motor_start_trans_com`/`motor_end_trans_com`
//! pair, so a move's `SET_VELOCITY` and `MOVE_ABS` are **two separate writes**,
//! each independently getting the column-5 axis byte — whereas the C-844
//! `strcat`s a whole move into one `AXIS n;`-prefixed message. (The
//! `ENABLE_TORQUE` "CLR then SVO" split is the same per-command-message
//! mechanism used twice inside one build call.)
//!
//! ## Connect (`motor_init`)
//! `*IDN?` (retry up to 3× until a non-empty reply → `brdptr->ident`). Then the
//! axis count is discovered: `CST? {A..D}` until a reply whose body (`&buff[2]`)
//! is `"NOSTAGE"`. For each present axis, `REF? {letter}` sets the
//! [reference](PIC848Axis::reference) flag (`&buff[2] == "0"` → `false`, else
//! `true`), which governs `LOAD_POS`. Every axis has encoder + gain support.
//!
//! ## Status
//! Per axis, under one lock (C `set_status`):
//! 1. `STA? {letter}` → reply parsed as `%c=%d` (`"A=1024"`). A reply with
//!    fewer than 3 bytes, no `'='` at index 1, or no integer body drives the
//!    NORMAL/RETRY/COMM_ERR two-strike debounce ([`CommState`], shared across
//!    axes as `cntrl->status` is per-controller). The integer is a
//!    `C848_Status_Reg` (`epicsUInt16`, `#else`/LSB layout): `Done = bit 0`
//!    (→ `RA_DONE`), `plus_ls = bit 5`, `minus_ls = bit 6`, `torque = bit 8`
//!    (→ `EA_POSITION`/powered).
//! 2. `POS? {letter}` → `atof(&buff[2])` (skip the `"A="` header) →
//!    `position = NINT(atof / POS_RES)`. `POS_RES = 1e-6`; both `position` and
//!    `encoder_position` are set to this value. Direction is the sign of the
//!    count delta versus the previous poll, held when unchanged.
//!
//! ## Units / precision
//! `cntrl_units = dval`, `res = POS_RES = 1e-6`, `maxdigits = 5`. Every wire
//! value is `dval × 1e-6` printed with **5** decimals. This is a deliberately
//! faithful reproduction of the C: with a 1e-6 scaler and only 5 printed
//! decimals, the least significant wire digit is 1e-5 physical units = 10
//! counts, so sub-10-count command values round at the wire exactly as in C.
//! Pair with `MRES = 1`, `EGU = counts`.
//!
//! ## Motion (`devPIC848_build_trans`) — each part a separate `#`-addressed write
//! `'#'` below marks column 5 (the axis byte); `L` is the axis letter.
//!
//! - `MOVE_ABS` (`move_absolute`): `SET_VELOCITY` then `MOVE_ABS` →
//!   `VEL  L {v}` (note the space after `L`) then `MOV  L{p}` (`.5f`).
//! - `MOVE_REL` (`move_relative`): `VEL  L {v}` then `MVR  L{+d}` (`%+.5f`).
//! - `HOME_FOR`/`HOME_REV` (`home`): `VEL  L {v}` then `REF  L` (both
//!   directions send the same `REF` — the controller homes to its reference).
//! - `JOG` (`move_velocity`): `VEL  L{v}` — the jog form has **no** space after
//!   `L` (C `"VEL  #%.*f"`, vs `SET_VELOCITY`'s `"VEL  # %.*f"`), value signed;
//!   motorRecord's jog sends only `SET_ACCEL` (here `send = false`) + `JOG`, no
//!   `SET_VELOCITY`.
//! - `STOP_AXIS` (`stop`): `HLT  L`.
//! - `LOAD_POS` (`set_position`): if the axis has a reference switch
//!   ([`reference`](PIC848Axis::reference) `true`), only `0.0` is allowed →
//!   `DFH  L` (define home); a non-zero request is an error. Otherwise
//!   `POS  L{+p}`.
//! - `ENABLE_TORQUE`/`DISABL_TORQUE` (`set_closed_loop`): enabling while torque
//!   is disabled first sends `CLR  L` (clear axis status) then `SVO  L1`;
//!   enabling while already on sends only `SVO  L1`; disabling sends `SVO  L0`.
//! - `SET_PGAIN`/`SET_IGAIN`/`SET_DGAIN` (`set_pid_gain`): `SPA  L1 {g}` /
//!   `SPA  L2 {g}` / `SPA  L3 {g}`, `g = gain × 32767` (`.5f`, **not** scaled by
//!   `res`).
//! - `SET_VEL_BASE`/`SET_ACCEL`/`GO`/`SET_HIGH_LIMIT`/`SET_LOW_LIMIT`/
//!   `SET_ENC_RATIO` — no wire command (DC servo / not supported).
//!
//! ## Not modeled (documented deviations)
//! - **`no_motion_count` motion-timeout**: C, after the target is stable for
//!   `> motionTO` (10) polls *while a move is active*, sets `RA_PROBLEM` and
//!   sends `HLT  #`. That depends on record-layer move state this interface does
//!   not carry; the `Done`-bit primary done signal is honored, the timeout
//!   `HLT` is omitted (as the C-862 port omits its `no_motion_count`).
//! - **`recv_mess(FLUSH)`** on comm recovery / **`report()`**: no `SyncIOHandle`
//!   flush primitive / not implemented, as elsewhere in this workspace.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{nint, parse_int_at, parse_value_at};

/// Response buffer size for a single reply (C `BUFF_SIZE`).
const READ_BUF: usize = 100;

/// Maximum axes probed at connect (C `MAX_AXES`).
pub const PIC848_MAX_AXES: u8 = 4;

/// Position resolution (C `drvPIC848.h` `POS_RES`).
const POS_RES: f64 = 0.000001;

fn pic848_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// The axis letter for a 0-based signal (`0`→`A` … `3`→`D`, C `PIC848_axis`).
fn axis_letter(signal: u8) -> char {
    (b'A' + signal) as char
}

/// Parse a `STA?` reply (`"%c=%d"`, e.g. `"A=1024"`) into the 16-bit status
/// register. `None` when C's `sscanf` would yield `convert_cnt != 2`
/// (`charcnt <= 2`, no `'='` at index 1, or no integer body).
fn parse_status_reg(reply: &str) -> Option<u16> {
    let b = reply.as_bytes();
    if b.len() <= 2 || b[1] != b'=' {
        return None;
    }
    // %d skips leading whitespace, then needs an optional sign and >=1 digit.
    let rest = reply[2..].trim_start();
    let mut chars = rest.chars();
    let first = chars.next()?;
    let has_digit = if first == '+' || first == '-' {
        chars.next().is_some_and(|c| c.is_ascii_digit())
    } else {
        first.is_ascii_digit()
    };
    if !has_digit {
        return None;
    }
    // parse_int_at(rest, 0) mirrors C atoi's leading-integer parse.
    parse_int_at(rest, 0).map(|v| v as u16)
}

/// Controller communication state (C `cntrl->status`, per-controller/shared).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Shared multi-axis controller endpoint: owns the octet handle, the probed
/// axis count + per-axis reference flags, and the (controller-wide) comm
/// debounce state.
pub struct PIC848Controller {
    handle: SyncIOHandle,
    ident: String,
    comm_state: CommState,
    /// Per-axis reference-switch flag (index = signal), from `REF?` at connect.
    references: Vec<bool>,
}

impl PIC848Controller {
    /// Connect, identify, and probe a C-848 (C `motor_init`'s per-card block):
    /// `*IDN?` retried up to 3×, then `CST?` per axis to count stages
    /// (stopping at `NOSTAGE`) and `REF?` per axis for the reference flag.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            comm_state: CommState::Normal,
            references: Vec::new(),
        };

        for _ in 0..3 {
            let reply = ctrl.query("*IDN?");
            if !reply.is_empty() {
                ctrl.ident = reply;
                break;
            }
        }
        if ctrl.ident.is_empty() {
            return Err(pic848_err("PIC848: no response to *IDN? (controller off?)"));
        }

        for signal in 0..PIC848_MAX_AXES {
            let letter = axis_letter(signal);
            // CST? {letter} -> "{L}={stage}"; body "NOSTAGE" ends the probe.
            let cst = ctrl.query(&format!("CST? {letter}"));
            if cst.get(2..) == Some("NOSTAGE") {
                break;
            }
            // REF? {letter} -> "{L}={0|1}"; body "0" => no reference switch.
            let refr = ctrl.query(&format!("REF? {letter}"));
            ctrl.references.push(refr.get(2..) != Some("0"));
        }

        Ok(ctrl)
    }

    /// The controller identification string (from the connect-time `*IDN?`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes with a configured stage (probed via `CST?`).
    pub fn num_axes(&self) -> u8 {
        self.references.len() as u8
    }

    /// Whether axis `signal` has a reference (home) switch (C
    /// `cntrl->reference[signal]`).
    fn reference(&self, signal: u8) -> bool {
        self.references[signal as usize]
    }

    /// Write a command, no reply expected (C `send_mess`). No terminator (port
    /// output EOS) and no echo.
    fn send(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Read one EOS-stripped reply, folding a transport failure / empty read
    /// into `""` (C `recv_mess`).
    fn recv(&self) -> String {
        match self.handle.read_octet(0, READ_BUF) {
            Ok(raw) if !raw.is_empty() => String::from_utf8_lossy(&raw).into_owned(),
            _ => String::new(),
        }
    }

    fn query(&self, cmd: &str) -> String {
        let _ = self.send(cmd);
        self.recv()
    }

    /// C `set_status` for axis `signal`. `None` means keep the cached status
    /// (RETRY) or flag comm error (COMM_ERR), per [`Self::comm_state`].
    fn poll_status(&mut self, signal: u8) -> Option<PolledStatus> {
        let letter = axis_letter(signal);
        let sta = self.query(&format!("STA? {letter}"));
        let Some(reg) = parse_status_reg(&sta) else {
            self.comm_state = if self.comm_state == CommState::Normal {
                CommState::Retry
            } else {
                CommState::CommErr
            };
            return None;
        };
        self.comm_state = CommState::Normal;

        let done = reg & 0x0001 != 0; // Done       (bit 0)
        let high_limit = reg & 0x0020 != 0; // plus_ls   (bit 5)
        let low_limit = reg & 0x0040 != 0; // minus_ls  (bit 6)
        let powered = reg & 0x0100 != 0; // torque    (bit 8)

        let pos = self.query(&format!("POS? {letter}"));
        let position = nint(parse_value_at(&pos, 2).unwrap_or(0.0) / POS_RES);

        Some(PolledStatus {
            position,
            done,
            powered,
            high_limit,
            low_limit,
        })
    }
}

/// Decoded result of one C-848 poll.
struct PolledStatus {
    position: i32,
    done: bool,
    powered: bool,
    high_limit: bool,
    low_limit: bool,
}

/// One axis (`A`-`D`) of a C-848 controller. Implements [`AsynMotor`].
pub struct PIC848Axis {
    controller: Arc<Mutex<PIC848Controller>>,
    signal: u8,
    /// Axis letter (`A`-`D`), byte 5 of every command for this axis.
    letter: char,
    /// C `cntrl->reference[signal]`: `true` ⟹ `LOAD_POS` may only zero.
    reference: bool,
    last_status: MotorStatus,
}

impl PIC848Axis {
    /// Construct axis `signal` (0-based; wire letter `A`+`signal`), seeding the
    /// initial status (C `motor_init`'s `set_status`). A failed initial poll is
    /// not an error.
    pub fn new(controller: Arc<Mutex<PIC848Controller>>, signal: u8) -> AsynResult<Self> {
        let reference = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            ctrl.reference(signal)
        };
        let mut axis = Self {
            controller: controller.clone(),
            signal,
            letter: axis_letter(signal),
            reference,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                has_encoder: true,
                vbas_supported: false,
                ..MotorStatus::default()
            },
        };
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(polled) = ctrl.poll_status(signal) {
            drop(ctrl);
            axis.apply(polled);
        }
        Ok(axis)
    }

    fn lock(&self) -> MutexGuard<'_, PIC848Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

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
            powered: polled.powered,
            gain_support: true,
            has_encoder: true,
            vbas_supported: false,
            ..MotorStatus::default()
        };
    }
}

impl AsynMotor for PIC848Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let l = self.letter;
        let ctrl = self.lock();
        // SET_VELOCITY ("VEL  L {v}") then MOVE_ABS ("MOV  L{p}") — two writes.
        ctrl.send(&format!("VEL  {l} {:.5}", velocity * POS_RES))?;
        ctrl.send(&format!("MOV  {l}{:.5}", position * POS_RES))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let l = self.letter;
        let ctrl = self.lock();
        ctrl.send(&format!("VEL  {l} {:.5}", velocity * POS_RES))?;
        ctrl.send(&format!("MVR  {l}{:+.5}", distance * POS_RES))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // JOG: "VEL  L{v}" (no space after L, signed value). No SET_VELOCITY.
        let l = self.letter;
        self.lock()
            .send(&format!("VEL  {l}{:.5}", velocity * POS_RES))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // Both HOME_FOR/HOME_REV send "REF  L" after SET_VELOCITY.
        let l = self.letter;
        let ctrl = self.lock();
        ctrl.send(&format!("VEL  {l} {:.5}", velocity * POS_RES))?;
        ctrl.send(&format!("REF  {l}"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let l = self.letter;
        self.lock().send(&format!("HLT  {l}"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let l = self.letter;
        if self.reference {
            // Referenced axis: only "define home" at zero is allowed.
            if position == 0.0 {
                self.lock().send(&format!("DFH  {l}"))
            } else {
                Err(pic848_err(
                    "PIC848: referenced axis position can only be set to 0",
                ))
            }
        } else {
            self.lock()
                .send(&format!("POS  {l}{:+.5}", position * POS_RES))
        }
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let l = self.letter;
        let ctrl = self.lock();
        if enable {
            // Enabling while torque is disabled first clears the axis status.
            if !self.last_status.powered {
                ctrl.send(&format!("CLR  {l}"))?;
            }
            ctrl.send(&format!("SVO  {l}1"))
        } else {
            ctrl.send(&format!("SVO  {l}0"))
        }
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let l = self.letter;
        let g = gain * 32767.0;
        let body = match kind {
            PidGainKind::Proportional => format!("SPA  {l}1 {g:.5}"),
            PidGainKind::Integral => format!("SPA  {l}2 {g:.5}"),
            PidGainKind::Derivative => format!("SPA  {l}3 {g:.5}"),
        };
        self.lock().send(&body)
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let mut ctrl = self.lock();
        let polled = ctrl.poll_status(self.signal);
        let comm_err = ctrl.comm_state == CommState::CommErr;
        drop(ctrl);

        let Some(polled) = polled else {
            if comm_err {
                self.last_status.comms_error = true;
                self.last_status.problem = true;
            }
            return Ok(self.last_status.clone());
        };
        self.apply(polled);
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_letters_map_from_signal() {
        assert_eq!(axis_letter(0), 'A');
        assert_eq!(axis_letter(1), 'B');
        assert_eq!(axis_letter(2), 'C');
        assert_eq!(axis_letter(3), 'D');
    }

    #[test]
    fn sta_reply_parses_like_sscanf_c_d() {
        // "%c=%d": valid body -> Some; missing '=' or int body -> None.
        assert_eq!(parse_status_reg("A=1024"), Some(1024));
        assert_eq!(parse_status_reg("D=0"), Some(0));
        assert_eq!(parse_status_reg("B=257junk"), Some(257)); // atoi leading int
        assert_eq!(parse_status_reg("A-1024"), None); // no '=' at index 1
        assert_eq!(parse_status_reg("A="), None); // charcnt <= 2 / no digit
        assert_eq!(parse_status_reg("A=x"), None); // no integer body
        assert_eq!(parse_status_reg(""), None);
    }

    #[test]
    fn status_bits_decode_per_c848_reg() {
        let done = |r: u16| r & 0x0001 != 0;
        let plus = |r: u16| r & 0x0020 != 0;
        let minus = |r: u16| r & 0x0040 != 0;
        let torque = |r: u16| r & 0x0100 != 0;
        assert!(done(0x0001) && !done(0x0002));
        assert!(plus(0x0020) && !plus(0x0010));
        assert!(minus(0x0040) && !minus(0x0020));
        assert!(torque(0x0100) && !torque(0x0080));
    }

    #[test]
    fn position_scales_by_pos_res() {
        // NINT(atof(&buff[2]) / 1e-6): "A=5.000000" -> 5000000.
        assert_eq!(
            nint(parse_value_at("A=5.000000", 2).unwrap() / POS_RES),
            5000000
        );
        assert_eq!(
            nint(parse_value_at("A=-1.500000", 2).unwrap() / POS_RES),
            -1500000
        );
    }

    #[test]
    fn move_and_jog_velocity_formats_differ_by_space() {
        // SET_VELOCITY has a space after the axis letter; JOG does not.
        let v = 10.0 * POS_RES;
        assert_eq!(format!("VEL  A {v:.5}"), "VEL  A 0.00001");
        assert_eq!(format!("VEL  A{v:.5}"), "VEL  A0.00001");
    }

    #[test]
    fn move_targets_scale_and_sign_per_c() {
        // MOV unsigned format, MVR/POS signed (%+).
        assert_eq!(
            format!("MOV  A{:.5}", 3_000_000.0 * POS_RES),
            "MOV  A3.00000"
        );
        assert_eq!(
            format!("MVR  A{:+.5}", 3_000_000.0 * POS_RES),
            "MVR  A+3.00000"
        );
        assert_eq!(
            format!("POS  A{:+.5}", -2_000_000.0 * POS_RES),
            "POS  A-2.00000"
        );
    }

    #[test]
    fn pid_gain_scales_by_32767_not_res() {
        let g = 0.5 * 32767.0;
        assert_eq!(format!("SPA  A1 {g:.5}"), "SPA  A1 16383.50000");
    }
}
