//! piezosystem jena E-516 (PIJEDS) closed-loop piezo controller driver.
//!
//! Ported from `motorPiJena/piJenaApp/src/drvPIJEDS.cc` + `devPIJEDS.cc`. The
//! E-516 is a **closed-loop** capacitive-feedback piezo nanopositioner: it
//! moves to an absolute target immediately on a `set` command and reports its
//! feedback position with `mess`. There is no hardware "done" bit — motion
//! completion is inferred from the feedback position settling (two consecutive
//! polls within a tolerance), exactly as the C `set_status`.
//!
//! ## Wire protocol
//!
//! Commands are `\r`-terminated (C `EDS_OUT_EOS`); replies are framed by an ETX
//! (0x11) input EOS (C `EDS_IN_ETX`), optionally led by an STX (0x13). Each
//! command names the axis in place of `#`:
//!
//! - `set,<axis>,<pos>`   — move to absolute position (µm), write-only
//! - `sr,<axis>,<vel>`    — set slew velocity, write-only
//! - `cl,<axis>,<0|1>`    — closed-loop (torque) off/on, write-only
//! - `mess,<axis>`        — read feedback position → `mess,<axis>,<pos>`
//! - `stat,<axis>`        — read status word → `stat,<axis>,<n>`
//!
//! The `set`/`sr`/`cl` set-forms return no reply; only the `mess`/`stat`
//! query-forms do (this is what lets the port skip the C driver's port flush).
//!
//! ## Units
//!
//! The asyn-rs motor boundary is dial-frame EGU. The C driver's
//! `drive_resolution` (`1/10^EDS_MAX_RES` = 0.001) only bridges the record's
//! raw-step boundary to the controller's physical units (µm) and **cancels** at
//! the EGU boundary, so this port works directly in physical units: positions
//! pass through, the record's `MRES` is `drive_resolution`, and its `EGU` is µm.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the settle detection and the
//!   `mess`/`stat` reads run inside [`poll`](AsynMotor::poll).
//! - C `MOVE_REL` sends nothing (the E-516 has no relative command); this port
//!   synthesizes it as an absolute move to `last position + distance`.
//! - C sets velocity in a separate `SET_VELOCITY` transaction; asyn-rs bundles
//!   the velocity into the move, so [`move_absolute`](AsynMotor::move_absolute)
//!   emits `sr` before `set` when a positive velocity is supplied.
//! - C `STOP_AXIS`, `HOME_*` and `LOAD_POS` send nothing (unsupported on this
//!   absolute closed-loop stage); the port mirrors that as no-ops.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 128;

/// Command terminator (C `EDS_OUT_EOS`).
const TERMINATOR: &[u8] = b"\r";

/// Position resolution significant digits (C `EDS_MAX_RES`); the `set`/`sr`
/// value is formatted with this many decimals.
const MAX_RES: usize = 3;

/// Settle tolerance for done detection, in physical units. C computes
/// `fdbk_tolerance = 10^(EDS_MAX_RES-1)` in raw steps; times `drive_resolution`
/// (`10^-EDS_MAX_RES`) that is `10^-1` = 0.1 physical units.
const TOLERANCE: f64 = 0.1;

/// Status-word bit 7: closed-loop (torque-enabled) mode.
const STATUS_CLOSE_LOOP: i32 = 0x80;

/// Maximum axes per controller (C `EDS_MAX_MOTORS`).
const MAX_MOTORS: usize = 6;

fn eds_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle and command framing.
pub struct PiJedsController {
    handle: SyncIOHandle,
    ident: String,
    version: i32,
    num_axes: usize,
}

impl PiJedsController {
    /// Connect and bring an E-516 online (C `motor_init`): identify it (the
    /// reply must contain `DSM`, retried up to three times), parse its DSM
    /// version, and count present axes by probing `mess` until one answers
    /// `not present`. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            version: 0,
            num_axes: 0,
        };
        let mut online = false;
        for _ in 0..3 {
            // GET_IDENT is the empty command (just the terminator).
            if let Ok(reply) = ctrl.query("")
                && reply.contains("DSM")
            {
                ctrl.ident = reply;
                online = true;
                break;
            }
        }
        if !online {
            return Err(eds_err("PIJEDS: controller not online (no 'DSM' identity)"));
        }
        // Version: NINT(atof(after 'V') * 1000).
        if let Some(pos) = ctrl.ident.find('V') {
            ctrl.version = motor_common::util::nint(atof(&ctrl.ident[pos + 1..]) * 1000.0);
        }
        // Count axes: probe until a stage answers "not present".
        let mut total = 0;
        for axis in 0..MAX_MOTORS {
            match ctrl.query(&format!("mess,{axis}")) {
                Ok(reply) if reply.contains("not present") => break,
                Ok(_) => total += 1,
                Err(_) => break,
            }
        }
        ctrl.num_axes = total;
        Ok(ctrl)
    }

    /// The identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// The DSM firmware version (thousandths, e.g. 1959 for `V1.959`).
    pub fn version(&self) -> i32 {
        self.version
    }

    /// Number of present axes detected at init.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Send a write-only set command (`set`/`sr`/`cl`); the E-516 returns no
    /// reply to these forms.
    fn write_cmd(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Send a query command and return its reply, stripped of framing control
    /// bytes (STX/ETX) and surrounding whitespace.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text
            .trim_matches(|c: char| c.is_control() || c.is_whitespace())
            .to_string())
    }
}

/// Extract the value field of a reply (`mnemonic,<axis>,<value>` → `value`):
/// the substring after the second comma.
fn value_field(reply: &str) -> Option<&str> {
    let mut commas = reply.match_indices(',');
    commas.next()?;
    let (idx, _) = commas.next()?;
    Some(reply[idx + 1..].trim())
}

/// One E-516 axis sharing a controller. Implements [`AsynMotor`].
pub struct PiJedsAxis {
    controller: Arc<Mutex<PiJedsController>>,
    /// 0-based axis index; the wire axis number is the same (E-516 axes 0..5).
    axis: usize,
    /// Last feedback position (physical units); C `motor_info->position`.
    prev_position: f64,
    /// Consecutive settled polls while moving; C `no_motion_count`.
    no_motion_count: u32,
    /// Whether a commanded move is in progress (C `nodeptr != 0`).
    moving: bool,
    last_status: MotorStatus,
}

impl PiJedsAxis {
    /// Construct axis `axis` (0-based). No hardware setup is needed beyond the
    /// controller identification already done in [`PiJedsController::new`].
    pub fn new(controller: Arc<Mutex<PiJedsController>>, axis: usize) -> Self {
        Self {
            controller,
            axis,
            prev_position: 0.0,
            no_motion_count: 0,
            moving: false,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                ..MotorStatus::default()
            },
        }
    }

    fn lock(&self) -> MutexGuard<'_, PiJedsController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Send an absolute move to `position` (physical units), optionally setting
    /// the slew velocity first (C `SET_VELOCITY` + `MOVE_ABS`).
    fn issue_move(&mut self, position: f64, velocity: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        if velocity > 0.0 {
            ctrl.write_cmd(&format!("sr,{},{:.*}", self.axis, MAX_RES, velocity))?;
        }
        ctrl.write_cmd(&format!("set,{},{:.*}", self.axis, MAX_RES, position))?;
        drop(ctrl);
        self.moving = true;
        self.no_motion_count = 0;
        Ok(())
    }
}

impl AsynMotor for PiJedsAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.issue_move(position, velocity)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C sends nothing for MOVE_REL; synthesize from the last feedback
        // position (module Deviations).
        self.issue_move(self.prev_position + distance, velocity)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C JOG only sets the slew rate — the E-516 has no continuous jog.
        let ctrl = self.lock();
        ctrl.write_cmd(&format!("sr,{},{:.*}", self.axis, MAX_RES, velocity.abs()))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C HOME_* send nothing — no home on an absolute closed-loop stage.
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C STOP_AXIS sends nothing — the piezo reaches target immediately.
        self.moving = false;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C LOAD_POS sends nothing — position cannot be redefined on the
        // absolute closed-loop encoder.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_cmd(&format!("cl,{},{}", self.axis, i32::from(enable)))
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let status_reply = ctrl.query(&format!("stat,{}", self.axis));
        let pos_reply = ctrl.query(&format!("mess,{}", self.axis));
        drop(ctrl);

        // A read is OK only when both queries returned a parseable value.
        let status_word = status_reply.ok().as_deref().and_then(value_field).map(atoi);
        let position = pos_reply.ok().as_deref().and_then(value_field).map(atof);

        let (Some(status_word), Some(position)) = (status_word, position) else {
            // C: first failure after NORMAL is a silent RETRY; a repeat failure
            // is a hard comms error. Keep the last position, flag the error.
            self.last_status = MotorStatus {
                comms_error: true,
                problem: true,
                ..self.last_status.clone()
            };
            return Ok(self.last_status.clone());
        };

        // Settle detection (C set_status): done once the feedback position has
        // stayed within tolerance for two consecutive polls while moving.
        let delta = (position - self.prev_position).abs();
        let mut done = false;
        let mut direction = self.last_status.direction;
        if delta < TOLERANCE {
            if self.no_motion_count > 0 {
                done = true;
            }
            if self.moving {
                self.no_motion_count += 1;
            }
        } else {
            direction = position >= self.prev_position;
            self.no_motion_count = 0;
        }
        self.prev_position = position;
        if done {
            self.moving = false;
        }

        let powered = (status_word & STATUS_CLOSE_LOOP) != 0;
        self.last_status = MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0,
            direction,
            done,
            moving: !done,
            // E-516 has no limit or home switches (C set_status).
            high_limit: false,
            low_limit: false,
            home: done,
            powered,
            comms_error: false,
            problem: false,
            gain_support: true,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn value_field_extracts_after_second_comma() {
        assert_eq!(value_field("stat,0,128"), Some("128"));
        assert_eq!(value_field("mess,3,1.234"), Some("1.234"));
        assert_eq!(value_field("sr,0, 5.0 "), Some("5.0"));
        assert_eq!(value_field("noComma"), None);
        assert_eq!(value_field("one,comma"), None);
    }

    #[test]
    fn close_loop_bit_is_bit_seven() {
        // "stat,0,128" → 0x80 set → powered.
        assert_ne!(atoi("128") & STATUS_CLOSE_LOOP, 0);
        // "stat,0,5" (motorExist|fdbk) → bit 7 clear → not powered.
        assert_eq!(atoi("5") & STATUS_CLOSE_LOOP, 0);
    }

    #[test]
    fn tolerance_matches_c_fdbk_tolerance() {
        // 10^(MAX_RES-1) raw steps * drive_resolution (10^-MAX_RES) = 10^-1.
        let steps = 10f64.powi(MAX_RES as i32 - 1);
        let res = 1.0 / 10f64.powi(MAX_RES as i32);
        assert!((steps * res - TOLERANCE).abs() < 1e-12);
    }
}
