//! ThorLabs MDT693/694/695 piezo controller driver (serial ASCII).
//!
//! Ported from `motorThorLabs/thorlabsApp/src/drvMDT695.cc` + `devMDT695.cc`
//! (the model-1 dev/drv pair). The MDT695/693 drives three piezo channels
//! (`X`/`Y`/`Z`) and the MDT694 a single channel, all over one serial line.
//! Each command is prefixed with the axis letter; commands and replies are
//! CR-terminated (`\r` — the driver owns output framing, the startup script
//! sets the input EOS).
//!
//! ## Open-loop voltage control
//!
//! This is an **open-loop** piezo controller: a "move" sets the channel output
//! voltage (`<axis>V<volts>`) and readback reports the commanded voltage
//! (`<axis>R?`). There is no motion feedback, so the controller is always
//! considered done, with no limit switches, no home, and no encoder — matching
//! the C `set_status`, which reports `RA_DONE` unconditionally. Relative moves,
//! homing, jog, stop, velocity/acceleration and PID are all no-ops (the C
//! `build_trans` emits no message for them).
//!
//! ## Units
//!
//! The controller works in volts (`<axis>R?` returns volts, `<axis>V` takes
//! volts). The C driver scales the record's raw steps by a fixed
//! `drive_resolution` (0.1) to reach volts and divides on readback; at the
//! asyn-rs motor boundary, which is dial-frame EGU, that scaling cancels, so
//! this port works directly in volts: `MRES` is 1 and `EGU` is V.
//!
//! ## Echo
//!
//! The controller can echo commands, which corrupts reply parsing. Startup
//! sends `E` (echo toggle) and retries the device probe until a clean `MDT`
//! identification is seen, matching the C retry loop.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::atof;

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 120;

/// Command terminator (C `MDT695_OUT_EOS`); the driver owns output framing.
const TERMINATOR: &[u8] = b"\r";

/// Axis letters by 0-based signal (C `MDT694_axis`).
const AXIS_LETTERS: [char; 3] = ['X', 'Y', 'Z'];

/// Map a 0-based axis index to its wire letter, clamped to the last channel.
fn axis_letter(index: usize) -> char {
    AXIS_LETTERS[index.min(AXIS_LETTERS.len() - 1)]
}

fn thorlabs_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint owning the serial handle.
pub struct Mdt695Controller {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
}

impl Mdt695Controller {
    /// Connect and identify an MDT693/694/695 (C `motor_init`): probe with `D`
    /// (expecting an `MDT` reply, toggling echo off with `E` on failure), pick
    /// the axis count from the model string (`694` → 1, otherwise 3), then read
    /// the `I` identification. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes: 3,
        };

        let mut model = String::new();
        let mut found = false;
        for _ in 0..3 {
            let reply = ctrl.query("D")?;
            if reply.contains("MDT") {
                model = reply;
                found = true;
                break;
            }
            // Reply corrupted (likely echo on): toggle echo and retry.
            ctrl.write_only("E")?;
            let _ = ctrl.read_line();
        }
        if !found {
            return Err(thorlabs_err(
                "MDT695: no 'MDT' identification response to device (D) command",
            ));
        }

        ctrl.num_axes = if model.contains("694") { 1 } else { 3 };

        // Multi-line identification (Model / Version lines).
        ctrl.write_only("I")?;
        let mut ident_parts = Vec::new();
        while let Ok(line) = ctrl.read_line() {
            if line.is_empty() {
                break;
            }
            if line.contains("Model") || line.contains("Version") {
                ident_parts.push(line);
            }
        }
        ctrl.ident = if ident_parts.is_empty() {
            model
        } else {
            ident_parts.join(", ")
        };

        Ok(ctrl)
    }

    /// The controller identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes (1 for MDT694, otherwise 3).
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command with no reply expected (C `cmnd_response == false`).
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Read one CR-terminated reply line (trimmed).
    fn read_line(&self) -> AsynResult<String> {
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_matches(['\r', '\n', '\0']).to_string())
    }

    /// Write a query and return its reply line.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        self.write_only(cmd)?;
        self.read_line()
    }
}

/// One MDT695 piezo channel sharing a controller. Implements [`AsynMotor`].
pub struct Mdt695Axis {
    controller: Arc<Mutex<Mdt695Controller>>,
    letter: char,
    prev_position: f64,
    comms_error: bool,
}

impl Mdt695Axis {
    /// Construct axis `index` (0-based; wire letter = `X`/`Y`/`Z`).
    pub fn new(controller: Arc<Mutex<Mdt695Controller>>, index: usize) -> Self {
        Self {
            controller,
            letter: axis_letter(index),
            prev_position: 0.0,
            comms_error: false,
        }
    }

    fn lock(&self) -> MutexGuard<'_, Mdt695Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for Mdt695Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // Set the channel output voltage: "<axis>V<volts>" (C "#V%.1f").
        let ctrl = self.lock();
        ctrl.write_only(&format!("{}V{:.1}", self.letter, position))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        _distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C: MOVE_REL emits no message (no relative move on this controller).
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C: JOG emits no message.
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C: HOME_FOR/HOME_REV emit no message.
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C: STOP_AXIS emits no message.
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C: LOAD_POS emits no message.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, _enable: bool) -> AsynResult<()> {
        // Piezo channel has no torque control (C ENABLE/DISABL_TORQUE ERROR).
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // No PID (C SET_PGAIN/IGAIN/DGAIN ERROR).
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let reply = ctrl.query(&format!("{}R?", self.letter))?;
        drop(ctrl);

        // C parses the voltage from reply offset 2 (skips the 2-char prefix).
        let mut comms_error = true;
        let mut position = self.prev_position;
        if reply.len() > 2 {
            let tail = &reply[2..];
            let trimmed = tail.trim_start();
            if trimmed
                .chars()
                .next()
                .is_some_and(|c| c.is_ascii_digit() || c == '+' || c == '-' || c == '.')
            {
                position = atof(trimmed);
                comms_error = false;
            }
        }
        self.comms_error = comms_error;
        self.prev_position = position;

        Ok(MotorStatus {
            position,
            encoder_position: 0.0,
            velocity: 0.0,
            // Open-loop: always done, always "positive" direction, no limits.
            done: true,
            moving: false,
            direction: true,
            comms_error,
            problem: comms_error,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_letters_map_by_index() {
        assert_eq!(axis_letter(0), 'X');
        assert_eq!(axis_letter(1), 'Y');
        assert_eq!(axis_letter(2), 'Z');
        // Out-of-range clamps to the last channel.
        assert_eq!(axis_letter(5), 'Z');
    }
}
