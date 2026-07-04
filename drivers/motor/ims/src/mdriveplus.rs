//! IMS MDrivePlus / MForce / Lexium motor driver (ASCII, over an asyn octet
//! port).
//!
//! Ported from `motorIms/imsApp/src/ImsMDrivePlusMotorController.cpp` +
//! `ImsMDrivePlusMotorAxis.cpp` (the "model 3" asynMotorController /
//! asynMotorAxis driver for Intelligent Motion Systems MDrivePlus M17/M23/M34).
//!
//! The controller speaks the IMS MCode ASCII language. Each controller drives a
//! **single axis** (C `NUM_AXES = 1`), optionally in "party mode" where a device
//! name (`DN`) is prepended to every command to address one drive on a multidrop
//! bus. Set commands (`VM=`, `MA`, …) get no reply; queries (`PR X`) return the
//! value terminated by `\n`.
//!
//! ## Framing
//!
//! The driver owns the output terminator; `st.cmd` sets only the input EOS
//! (`\n`). The output terminator is `\r\n` by default, but a Lexium MDrive
//! (`LMD`) uses `\r` without a device name or `\n` with one (C
//! `readHomeAndLimitConfig` switches `setOutputEos` after reading `PR VR`). The
//! device name, when set, is prepended to every command with no separator.
//!
//! ## Units
//!
//! MCode positions, velocities and accelerations are integer motor steps (the C
//! casts every value to `(long)`). There is no resolution scaling, so the driver
//! works in native steps with `MRES` = 1.
//!
//! ## Not modeled (documented)
//!
//! - **Lexium (`LMM`/`LMD`) `PR IS` switch discovery.** The C reads the whole
//!   `IS` I/O-setup block in one shot by temporarily forcing a NUL input EOS on
//!   the raw octet interface. The asyn-rs `SyncIOHandle` has no per-read EOS
//!   override, so this port discovers home/limit switch inputs only for the
//!   MForce path (`PR S1`..`PR S4`); on a Lexium the switch inputs stay
//!   unconfigured (no home/limit switch status is surfaced, matching a drive
//!   with none of S1-S4 set).
//! - **`saveToNVM` (`S`) / `clearLockedRotor` (`CF`) / locked-rotor readback
//!   (`PR LR`).** These are asyn extra-parameter actions driven by separate PVs,
//!   outside the motor record, so they are not part of the [`AsynMotor`]
//!   boundary.
//! - **Velocity / direction / power-on readback.** The C poller never sets
//!   `motorStatusActualVelocity`, `motorStatusDirection` or `motorStatusPowerOn`,
//!   so those are left at their defaults here too.

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi};

/// Reply read buffer size (C `MAX_BUFF_LEN`).
const READ_BUF: usize = 80;

fn ims_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Build the on-wire command: device name (party mode) + command + terminator.
fn frame(device_name: &str, cmd: &str, terminator: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(device_name.len() + cmd.len() + terminator.len());
    out.extend_from_slice(device_name.as_bytes());
    out.extend_from_slice(cmd.as_bytes());
    out.extend_from_slice(terminator);
    out
}

/// One IMS MDrivePlus controller/axis (1:1, C `NUM_AXES = 1`). Implements
/// [`AsynMotor`].
pub struct ImsMDrivePlusAxis {
    handle: SyncIOHandle,
    /// Device name (`DN`) prepended to every command for party mode; empty when
    /// not in party mode.
    device_name: String,
    /// Output terminator (`\r\n`, or `\r`/`\n` for a Lexium MDrive).
    terminator: Vec<u8>,
    /// I/O input number configured as the home switch (C `homeSwitchInput`).
    home_switch: Option<i32>,
    /// I/O input number configured as the positive-limit switch.
    pos_limit_switch: Option<i32>,
    /// I/O input number configured as the negative-limit switch.
    neg_limit_switch: Option<i32>,
    has_encoder: bool,
    /// True between a `home` command and the move completing (drives the
    /// `R1=1` "homed" latch in [`Self::poll`]).
    homing: bool,
}

impl ImsMDrivePlusAxis {
    /// Connect an axis on `handle`, addressing it by `device_name` (empty for no
    /// party mode). Probes the firmware version (to pick the terminator and,
    /// on MForce, the S1-S4 switch config) and the encoder flag.
    pub fn new(handle: SyncIOHandle, device_name: String) -> AsynResult<Self> {
        let mut axis = Self {
            handle,
            device_name,
            terminator: b"\r\n".to_vec(),
            home_switch: None,
            pos_limit_switch: None,
            neg_limit_switch: None,
            has_encoder: false,
            homing: false,
        };
        axis.setup()?;
        Ok(axis)
    }

    fn framed(&self, cmd: &str) -> Vec<u8> {
        frame(&self.device_name, cmd, &self.terminator)
    }

    /// Write a set command; the controller sends no reply (C `writeController`).
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &self.framed(cmd))?;
        Ok(())
    }

    /// Write a query and read one reply line (C `writeReadController`).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &self.framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let s = String::from_utf8_lossy(&raw);
        Ok(s.trim_matches(['\r', '\n', '\0', ' ']).to_string())
    }

    fn setup(&mut self) -> AsynResult<()> {
        // Firmware version: retry a few times, require a plausible reply (C
        // requires strlen >= 2). MForce 1 answers `PR VR` with an error, so a
        // short/failed reply is not necessarily fatal — but a total silence is.
        let mut version = String::new();
        for _ in 0..3 {
            if let Ok(v) = self.write_read("PR VR")
                && v.len() >= 2
            {
                version = v;
                break;
            }
        }
        if version.len() < 2 {
            return Err(ims_err(
                "ims: version inquiry failed (no response from controller)",
            ));
        }

        let vlow = version.to_lowercase();
        if vlow.contains("lmd") {
            // Lexium MDrive: terminator depends on party mode.
            self.terminator = if self.device_name.is_empty() {
                b"\r".to_vec()
            } else {
                b"\n".to_vec()
            };
        }

        if vlow.contains("lmm") || vlow.contains("lmd") {
            // Lexium switch config comes from `PR IS`, which needs a block read
            // this port cannot do (see module docs) — leave switches unconfigured.
        } else {
            // MForce: read the S1-S4 I/O setup and map any home/limit inputs.
            for input_no in 1..=4 {
                if let Ok(resp) = self.write_read(&format!("PR S{input_no}")) {
                    self.set_switch(atoi(&resp), input_no);
                }
            }
        }

        // Encoder present? (`EE` echo-encoder flag.)
        if let Ok(resp) = self.write_read("PR EE") {
            self.has_encoder = atoi(&resp) != 0;
        }
        Ok(())
    }

    /// Map an I/O setup type code to the switch input it configures (C
    /// `set_switch_vars`: 1 = home, 2 = positive limit, 3 = negative limit).
    fn set_switch(&mut self, type_code: i32, input_no: i32) {
        match type_code {
            1 => self.home_switch = Some(input_no),
            2 => self.pos_limit_switch = Some(input_no),
            3 => self.neg_limit_switch = Some(input_no),
            _ => {}
        }
    }

    /// Set base velocity (`VI`, only if > 0), max velocity (`VM`), acceleration
    /// and deceleration (`A`/`D`, only if non-zero), then reset the stall flag
    /// (`ST=0`). C `setAxisMoveParameters`.
    fn set_move_params(
        &self,
        min_velocity: f64,
        max_velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if min_velocity > 0.0 {
            if min_velocity > max_velocity {
                return Err(ims_err(
                    "ims: base velocity cannot be greater than max velocity",
                ));
            }
            self.write_only(&format!("VI={}", min_velocity as i64))?;
        }
        self.write_only(&format!("VM={}", max_velocity as i64))?;
        if acceleration != 0.0 {
            self.write_only(&format!("A={}", acceleration as i64))?;
            self.write_only(&format!("D={}", acceleration as i64))?;
        }
        self.write_only("ST=0")?;
        Ok(())
    }

    fn do_move(
        &self,
        position: f64,
        relative: bool,
        min_velocity: f64,
        max_velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.set_move_params(min_velocity, max_velocity, acceleration)?;
        let cmd = if relative {
            format!("MR {}", position as i64)
        } else {
            format!("MA {}", position as i64)
        };
        self.write_only(&cmd)
    }
}

impl AsynMotor for ImsMDrivePlusAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false, min_velocity, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true, min_velocity, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C jogs with |maxVelocity| for the parameters (JOGR passes a negative
        // velocity) but the signed value in the SL slew command.
        self.set_move_params(min_velocity, velocity.abs(), acceleration)?;
        self.write_only(&format!("SL {}", velocity as i64))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // C reads `PR VI` to ensure a non-zero creep velocity when min_velocity
        // is 0, but never re-sends it — the creep relies on the drive's stored
        // VI. This port matches that (does not re-send VI).
        self.set_move_params(min_velocity, velocity, acceleration)?;
        // HM 3 homes in the plus direction, HM 1 in the minus direction.
        let direction = if forward { 3 } else { 1 };
        self.write_only(&format!("HM {direction}"))?;
        // Reset the "homed" latch (R1); poll sets it once the move completes.
        self.write_only("R1=0")?;
        self.homing = true;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        if acceleration != 0.0 {
            self.write_only(&format!("D={}", acceleration as i64))?;
        }
        self.write_only("SL 0")
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.write_only(&format!("P={}", position as i64))
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let position = atof(&self.write_read("PR P")?);
        let moving = atoi(&self.write_read("PR MV")?) != 0;

        // On the falling edge of a home move, latch R1 = 1 (homed).
        if self.homing && !moving {
            self.write_only("R1=1")?;
            self.homing = false;
        }

        // Home / limit switch inputs (only those configured on this drive).
        let encoder_home = match self.home_switch {
            Some(n) => atoi(&self.write_read(&format!("PR I{n}"))?) != 0,
            None => false,
        };
        let high_limit = match self.pos_limit_switch {
            Some(n) => atoi(&self.write_read(&format!("PR I{n}"))?) != 0,
            None => false,
        };
        let low_limit = match self.neg_limit_switch {
            Some(n) => atoi(&self.write_read(&format!("PR I{n}"))?) != 0,
            None => false,
        };

        let homed = atoi(&self.write_read("PR R1")?) > 0;
        // ST is the stall flag; C maps it to both following-error and slip. Only
        // slip/stall has a MotorStatus field, so both fold into slip_stall.
        let stall = atoi(&self.write_read("PR ST")?) > 0;
        let encoder_position = atof(&self.write_read("PR C2")?);

        Ok(MotorStatus {
            position,
            encoder_position,
            done: !moving,
            moving,
            high_limit,
            low_limit,
            // C sets motorStatusHome_ (EA_HOME) from the home switch input.
            encoder_home,
            homed,
            slip_stall: stall,
            has_encoder: self.has_encoder,
            gain_support: self.has_encoder,
            // IMS honours a base velocity (VI).
            vbas_supported: true,
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_prepends_device_name_and_terminator() {
        // Party mode: device name prepended with no separator, `\r\n` terminator.
        assert_eq!(frame("M06", "MA 100", b"\r\n"), b"M06MA 100\r\n");
    }

    #[test]
    fn frame_no_device_name_lexium_terminator() {
        // No party mode, Lexium MDrive `\r` terminator.
        assert_eq!(frame("", "PR P", b"\r"), b"PR P\r");
    }
}
