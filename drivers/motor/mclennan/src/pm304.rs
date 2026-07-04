//! Mclennan PM304 / PM600 stepper motor controller driver (serial ASCII).
//!
//! Ported from `motorMclennan/mclennanApp/src/drvPM304.cc` + `devPM304.cc` (the
//! model-1 dev/drv pair). One serial line drives a multi-axis controller; each
//! command is prefixed with the 1-based axis number and terminated by CR (the
//! driver owns output framing; the startup script sets only the input EOS). The
//! controller answers **every** command, so each command is written and its
//! reply consumed to keep the stream synchronized.
//!
//! ## Two models
//!
//! `1ID` identifies the controller at startup. A reply containing `PM304`
//! selects [`Model::Pm304`]; anything else is [`Model::Pm600`]. The two differ
//! in wire formatting:
//!
//! - **Status** (`<axis>OS`): PM304 replies with an 8-character `0`/`1` string;
//!   PM600 prefixes it with `01:` and reorders the bits.
//! - **Position** (`<axis>OA` / `<axis>OC`): PM304 replies `AP=<n>`, PM600
//!   replies `01:<n>`; both are parsed by skipping the first three characters.
//! - **Echo**: the PM600 echoes the command followed by a lone CR before its
//!   response, so on that model everything up to and including the first CR is
//!   stripped from each reply.
//! - **Home** (`IX`/`IX-1` on PM304, `HD`/`HD-1` on PM600) and **jog** (`SV`+
//!   `CV1`/`CV-1` on PM304, signed `CV` on PM600) use different commands.
//!
//! ## Units
//!
//! The controller works natively in motor steps / encoder counts (`OA`/`OC`
//! return counts, `MA` takes counts) with no resolution scaling, so the asyn-rs
//! motor boundary is steps: positions pass through with `NINT` rounding, the
//! record's `MRES` is 1, and its `EGU` is steps. Velocity (`SV`, steps/s) and
//! acceleration (`SA`/`SD`) cross the boundary directly as controller values.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `OS`/`OA`/`OC` reads run
//!   inside [`poll`](AsynMotor::poll).
//! - `use_encoder` follows C: always true on the PM304; on the PM600 it is
//!   false only when the per-axis `ID` reply reports `Open loop stepper mode`.
//! - The C `set_status` velocity readback is marked `NEEDS WORK` and always
//!   reports 0; this port reports 0 likewise.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atoi, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 200;

/// Command terminator; the driver owns output framing (PM304/PM600 accept CR).
const TERMINATOR: &[u8] = b"\r";

/// Controller family, selected from the `1ID` reply.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// PM304: bare 8-character status string, `AP=<n>` position, no echo.
    Pm304,
    /// PM600: `01:`-prefixed status/position and a CR-terminated command echo.
    Pm600,
}

fn pm304_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle, the model, and per-axis
/// encoder flags.
pub struct Pm304Controller {
    handle: SyncIOHandle,
    ident: String,
    model: Model,
    n_axes: usize,
    use_encoder: Vec<bool>,
}

impl Pm304Controller {
    /// Connect and identify a Mclennan controller (C `motor_init`): stop axis 1,
    /// read `1ID` to select the model, then read each axis `ID` to decide
    /// whether it reports position from the encoder (`OA`) or the command
    /// counter (`OC`). Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle, n_axes: usize) -> AsynResult<Self> {
        let n_axes = n_axes.max(1);
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            model: Model::Pm304,
            n_axes,
            use_encoder: Vec::with_capacity(n_axes),
        };

        // Stop axis 1 for safety (C sends "1ST;" before identifying).
        let _ = ctrl.write_read_raw("1ST")?;

        // Model detection is robust to the PM600 echo: the reply is
        // "1ID\r<ident>" on the PM600 and "<ident>" on the PM304, and both
        // contain "PM304" only for the PM304.
        let ident = ctrl.write_read_raw("1ID")?;
        ctrl.model = if ident.contains("PM304") {
            Model::Pm304
        } else {
            Model::Pm600
        };
        ctrl.ident = ctrl.strip_echo(&ident).trim().to_string();
        if ctrl.ident.is_empty() {
            return Err(pm304_err("PM304: no response to 1ID identification query"));
        }

        for axis in 1..=n_axes {
            let id = ctrl.write_read_raw(&format!("{axis}ID"))?;
            // C: PM304 always uses the encoder; PM600 uses the command counter
            // only when the axis reports "Open loop stepper mode".
            let use_encoder = match ctrl.model {
                Model::Pm304 => true,
                Model::Pm600 => !id.contains("Open loop stepper mode"),
            };
            ctrl.use_encoder.push(use_encoder);
        }

        Ok(ctrl)
    }

    /// The identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// The detected controller model.
    pub fn model(&self) -> Model {
        self.model
    }

    /// Number of axes (as configured; C `n_axes`).
    pub fn num_axes(&self) -> usize {
        self.n_axes
    }

    /// Whether axis `index` (0-based) reports position from its encoder.
    fn use_encoder(&self, index: usize) -> bool {
        self.use_encoder.get(index).copied().unwrap_or(true)
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// On the PM600, drop the echoed command (everything up to and including the
    /// first CR); on the PM304 the reply has no echo and is returned unchanged.
    fn strip_echo<'a>(&self, reply: &'a str) -> &'a str {
        if self.model == Model::Pm600 {
            match reply.find('\r') {
                Some(i) => &reply[i + 1..],
                None => reply,
            }
        } else {
            reply
        }
    }

    /// Write a command and return its raw reply (input EOS already stripped by
    /// the port), without removing the PM600 echo — used during identification
    /// before the model is known.
    fn write_read_raw(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text
            .trim_end_matches(['\r', '\n', '\0'])
            .to_string())
    }

    /// Write a command and return its reply with the PM600 echo removed. Every
    /// command produces a reply, so this is used for both queries and set
    /// commands (whose reply is discarded) to keep the stream synchronized.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.write_read_raw(cmd)?;
        Ok(self.strip_echo(&raw).trim().to_string())
    }
}

/// Status bits decoded from an `OS` reply.
struct StatusBits {
    done: bool,
    plus_ls: bool,
    minus_ls: bool,
    problem: bool,
}

/// Decode an `OS` status reply for the given model.
///
/// - PM304: an 8-character `0`/`1` string. Index 3 is the busy flag (`0` =
///   moving); index 0/1 are the minus/plus limit switches; any `1` in indices
///   4..8 signals a problem.
/// - PM600: the same fields prefixed with `01:`, reordered to
///   done/problem/plus-LS/minus-LS at indices 3/4/5/6 of the raw reply.
fn parse_status(model: Model, reply: &str) -> StatusBits {
    let b = reply.as_bytes();
    let at = |i: usize| b.get(i).copied().unwrap_or(b'0');
    match model {
        Model::Pm304 => StatusBits {
            done: at(3) != b'0',
            minus_ls: at(0) == b'1',
            plus_ls: at(1) == b'1',
            problem: at(4) == b'1' || at(5) == b'1' || at(6) == b'1' || at(7) == b'1',
        },
        // C strips the leading "01:" (3 chars), then reads [0..4] of the rest;
        // here that is indices 3..7 of the raw reply.
        Model::Pm600 => StatusBits {
            done: at(3) != b'0',
            problem: at(4) == b'1',
            plus_ls: at(5) == b'1',
            minus_ls: at(6) == b'1',
        },
    }
}

/// Parse a position reply (`AP=<n>` on PM304, `01:<n>` on PM600) by skipping the
/// three-character prefix, matching C `atoi(&response[3])`.
fn parse_position(reply: &str) -> i32 {
    match reply.get(3..) {
        Some(rest) => atoi(rest),
        None => atoi(reply),
    }
}

/// One PM304/PM600 axis sharing a controller. Implements [`AsynMotor`].
pub struct Pm304Axis {
    controller: Arc<Mutex<Pm304Controller>>,
    /// 1-based wire axis number.
    axis: u32,
    /// 0-based index into the controller's `use_encoder` table.
    index: usize,
    prev_position: i32,
    last_status: MotorStatus,
}

impl Pm304Axis {
    /// Construct axis `index` (0-based; wire axis = `index + 1`).
    pub fn new(controller: Arc<Mutex<Pm304Controller>>, index: usize) -> Self {
        Self {
            controller,
            axis: index as u32 + 1,
            index,
            prev_position: 0,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                ..MotorStatus::default()
            },
        }
    }

    fn lock(&self) -> MutexGuard<'_, Pm304Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Program the move speeds before a move (C sends `SV`/`SA`/`SD` as separate
    /// transactions ahead of the move command). Non-positive values are skipped.
    fn program_speeds(
        ctrl: &Pm304Controller,
        axis: u32,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if velocity > 0.0 {
            ctrl.command(&format!("{axis}SV{}", nint(velocity)))?;
        }
        if acceleration > 0.0 {
            let accel = nint(acceleration);
            ctrl.command(&format!("{axis}SA{accel}"))?;
            ctrl.command(&format!("{axis}SD{accel}"))?;
        }
        Ok(())
    }
}

impl AsynMotor for Pm304Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        Pm304Axis::program_speeds(&ctrl, self.axis, velocity, acceleration)?;
        ctrl.command(&format!("{}MA{}", self.axis, nint(position)))?;
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
        Pm304Axis::program_speeds(&ctrl, self.axis, velocity, acceleration)?;
        ctrl.command(&format!("{}MR{}", self.axis, nint(distance)))?;
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
        match ctrl.model {
            Model::Pm304 => {
                // PM304: set the (unsigned) slew speed, then a signed direction.
                let speed = nint(velocity.abs());
                if speed > 0 {
                    ctrl.command(&format!("{}SV{speed}", self.axis))?;
                }
                let dir = if velocity >= 0.0 { 1 } else { -1 };
                ctrl.command(&format!("{}CV{dir}", self.axis))?;
            }
            Model::Pm600 => {
                // PM600: signed continuous velocity in one command.
                ctrl.command(&format!("{}CV{}", self.axis, nint(velocity)))?;
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
        let cmd = match (ctrl.model, forward) {
            (Model::Pm304, true) => format!("{}IX", self.axis),
            (Model::Pm304, false) => format!("{}IX-1", self.axis),
            (Model::Pm600, true) => format!("{}HD", self.axis),
            (Model::Pm600, false) => format!("{}HD-1", self.axis),
        };
        ctrl.command(&cmd)?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.command(&format!("{}ST", self.axis))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let cmd = if ctrl.use_encoder(self.index) {
            format!("{}AP{}", self.axis, nint(position))
        } else {
            format!("{}CP{}", self.axis, nint(position))
        };
        ctrl.command(&cmd)?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        // C ENABLE_TORQUE -> "RS", DISABL_TORQUE -> "AB".
        let cmd = if enable {
            format!("{}RS", self.axis)
        } else {
            format!("{}AB", self.axis)
        };
        ctrl.command(&cmd)?;
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let g = nint(gain);
        let cmd = match kind {
            PidGainKind::Proportional => format!("{}KP{g}", self.axis),
            PidGainKind::Integral => format!("{}KS{g}", self.axis),
            PidGainKind::Derivative => format!("{}KV{g}", self.axis),
        };
        ctrl.command(&cmd)?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let use_encoder = ctrl.use_encoder(self.index);
        let model = ctrl.model;

        let status_reply = ctrl.command(&format!("{}OS", self.axis))?;
        let bits = parse_status(model, &status_reply);

        let pos_cmd = if use_encoder {
            format!("{}OA", self.axis)
        } else {
            format!("{}OC", self.axis)
        };
        let pos_reply = ctrl.command(&pos_cmd)?;
        drop(ctrl);

        let position = parse_position(&pos_reply);

        // Direction: a limit switch fixes it (C); otherwise infer from motion.
        let direction = if bits.plus_ls {
            true
        } else if bits.minus_ls {
            false
        } else {
            position >= self.prev_position
        };
        self.prev_position = position;

        let status = MotorStatus {
            position: position as f64,
            encoder_position: position as f64,
            velocity: 0.0,
            done: bits.done,
            moving: !bits.done,
            high_limit: bits.plus_ls,
            low_limit: bits.minus_ls,
            problem: bits.problem,
            direction,
            has_encoder: use_encoder,
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
    fn pm304_status_moving_and_limits() {
        // 8-char string: minus-LS=0, plus-LS=0, [2]=0, busy=0 (moving), 0000.
        let s = parse_status(Model::Pm304, "00000000");
        assert!(!s.done);
        assert!(!s.plus_ls && !s.minus_ls && !s.problem);

        // busy flag (index 3) = 1 -> done; plus-LS (index 1) = 1; a problem bit.
        let s = parse_status(Model::Pm304, "01011000");
        assert!(s.done);
        assert!(s.plus_ls);
        assert!(!s.minus_ls);
        assert!(s.problem); // index 4 == '1'
    }

    #[test]
    fn pm600_status_strips_prefix_and_reorders() {
        // "01:" + done=1, problem=0, plus-LS=1, minus-LS=0, then pad.
        let s = parse_status(Model::Pm600, "01:10100000");
        assert!(s.done);
        assert!(!s.problem);
        assert!(s.plus_ls);
        assert!(!s.minus_ls);
    }

    #[test]
    fn position_skips_three_char_prefix() {
        assert_eq!(parse_position("AP=10234"), 10234);
        assert_eq!(parse_position("01:-512"), -512);
    }
}
