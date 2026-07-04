//! SmarAct MCS (RS-232) controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorSmarAct/smarActApp/src/smarActMCSMotorDriver.cpp` (a
//! model-1-style `asynMotorController`/`asynMotorAxis` pair, configured with a
//! separate controller + per-axis create command). Communication is `\n`
//! terminated (the startup script sets the input EOS; the driver owns output
//! framing).
//!
//! ## Protocol
//!
//! Commands are colon-prefixed, channel-addressed, comma-separated, and every
//! command reads back a reply. Query replies are `:<CMD><axis>,<value>`
//! (e.g. `GP`, `GS`, `GST`, `GPPK`, `GCLS`) or `:<CMD><axis>,<angle>,<rev>` for
//! the rotary angle (`GA`). A reply whose command letters begin with `E` is a
//! status code: `E…,0` is a synchronous acknowledgement (success), `E…,<n>`
//! with `n != 0` is an error. Move/home/stop/set-position/set-speed commands
//! reply with such an acknowledgement.
//!
//! ## Linear vs rotary stages
//!
//! Rotation is detected by querying linear position (`GP`): if it errors the
//! stage is rotary. Rotary stages use `GA`/`MAA`/`MAR` (angle + revolution);
//! linear stages use `GP`/`MPA`/`MPR`. `poll` folds `rev * UDEG_PER_REV + angle`
//! into a single position.
//!
//! ## Units
//!
//! Unlike the SCU, the MCS driver reports the controller's raw integer position
//! directly (nanometres linear / micro-degrees rotary) with no scaling, so the
//! driver boundary is the controller-native unit and `MRES` = 1 (set
//! `MRES` = 1e-3 to read microns / millidegrees). Move targets are rounded to
//! whole controller units.
//!
//! ## Not modeled (documented)
//!
//! The MCS exposes runtime auxiliary asyn parameters that the motor record does
//! not cover: positioner type (`SST`/`ptyp`), calibration (`CS`/`cal`), max
//! closed-loop frequency (`SCLF`/`sclf`), the auto-zero-on-home flag and the
//! post-move hold time. These are not exposed here; motion uses the C startup
//! defaults — hold time `0` (no active hold after a move) and auto-zero `1` on
//! home. Configure the positioner type on the controller beforehand. Exposing
//! these as runtime PVs would need extra asyn parameters (upstream work).
//! `set_closed_loop`/`set_pid_gain` are therefore no-ops (as in C).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::atoi;

/// Response buffer size.
const READ_BUF: usize = 64;

/// Command terminator; the driver owns output framing.
const TERMINATOR: &[u8] = b"\n";

/// Micro-degrees per revolution (C `UDEG_PER_REV`).
const UDEG_PER_REV: i64 = 360_000_000;

/// Very-far linear velocity-move target (nm, C `FAR_AWAY_LIN`).
const FAR_AWAY_LIN: i64 = 1_000_000_000;

/// Very-far rotary velocity-move target (revolutions, C `FAR_AWAY_ROT`).
const FAR_AWAY_ROT: i64 = 32767;

/// Post-move hold time (C startup default of the `holdTime` aux parameter).
const HOLD_TIME: i32 = 0;

/// Auto-zero-on-home flag (C startup default of the `autoZero` aux parameter).
const AUTO_ZERO: i32 = 1;

// SmarActMCSStatus codes returned by `GS` that mean the stage is in motion
// (Stepping, Scanning, Targeting, MoveDelay, Calibrating, FindRefMark). Stopped
// (0), Holding (3) and Locked (9) mean not moving.
fn status_is_moving(code: i32) -> bool {
    matches!(code, 1 | 2 | 4 | 5 | 6 | 7)
}

fn mcs_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Parse a `:<CMD><axis>,<value>` reply into (is-error-command, last value).
/// The MCS error convention is applied by callers: an `E` command with a
/// non-zero value is an error, `E…,0` is an acknowledgement.
fn parse_reply(reply: &str) -> Option<(bool, i32)> {
    let s = reply.trim().strip_prefix(':')?;
    let cmd_end = s.find(|c: char| !c.is_ascii_uppercase())?;
    if cmd_end == 0 {
        return None;
    }
    let is_err = s.as_bytes()[0] == b'E';
    let rest = &s[cmd_end..]; // "<axis>,<value>[,...]"
    let comma = rest.find(',')?;
    let tail = &rest[comma + 1..];
    let val_end = tail.find(',').unwrap_or(tail.len());
    Some((is_err, atoi(tail[..val_end].trim())))
}

/// Parse a `:<CMD><axis>,<angle>,<rev>` rotary reply into (is-error-command,
/// angle, revolutions). Returns `None` unless all three numbers are present
/// (an error reply carries fewer fields and fails to parse, matching C).
fn parse_angle(reply: &str) -> Option<(bool, i32, i32)> {
    let s = reply.trim().strip_prefix(':')?;
    let cmd_end = s.find(|c: char| !c.is_ascii_uppercase())?;
    if cmd_end == 0 {
        return None;
    }
    let is_err = s.as_bytes()[0] == b'E';
    let rest = &s[cmd_end..]; // "<axis>,<angle>,<rev>"
    let parts: Vec<&str> = rest.splitn(3, ',').collect();
    if parts.len() != 3 {
        return None;
    }
    Some((is_err, atoi(parts[1].trim()), atoi(parts[2].trim())))
}

/// Shared MCS controller endpoint owning the asyn octet handle.
pub struct McsController {
    handle: SyncIOHandle,
    /// When set, speed-set commands (`SCLS`) are suppressed (C `disableSpeed`).
    disable_speed: bool,
}

impl McsController {
    /// Wrap a connected octet handle. The C controller constructor connects,
    /// slurps stray telnet negotiation bytes and starts the poller; the telnet
    /// slurp is a workaround for TELNET-mode terminal servers and is omitted
    /// here — use a RAW (or serial) connection.
    pub fn new(handle: SyncIOHandle, disable_speed: bool) -> Self {
        Self {
            handle,
            disable_speed,
        }
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Send a command and read back its reply (every MCS command replies).
    fn transact(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string())
    }

    /// Read an integer parameter (`:<parm><ch>`).
    fn get_val(&self, parm: &str, channel: i32) -> AsynResult<i32> {
        let reply = self.transact(&format!(":{parm}{channel}"))?;
        let (is_err, val) =
            parse_reply(&reply).ok_or_else(|| mcs_err(format!("MCS: bad reply {reply:?}")))?;
        if is_err && val != 0 {
            return Err(mcs_err(format!("MCS: controller error {val} ({reply:?})")));
        }
        Ok(val)
    }

    /// Read the rotary angle (`:GA<ch>` → angle, revolutions).
    fn get_angle(&self, channel: i32) -> AsynResult<(i32, i32)> {
        let reply = self.transact(&format!(":GA{channel}"))?;
        let (is_err, angle, rev) =
            parse_angle(&reply).ok_or_else(|| mcs_err(format!("MCS: bad angle {reply:?}")))?;
        if is_err && angle != 0 {
            return Err(mcs_err(format!(
                "MCS: controller error {angle} ({reply:?})"
            )));
        }
        Ok((angle, rev))
    }

    /// Issue a command whose reply is only an acknowledgement (move/home/stop/
    /// set-position/set-speed); succeed on a non-error reply or `E…,0`.
    fn command(&self, cmd: &str) -> AsynResult<()> {
        let reply = self.transact(cmd)?;
        let (is_err, val) =
            parse_reply(&reply).ok_or_else(|| mcs_err(format!("MCS: bad reply {reply:?}")))?;
        if is_err && val != 0 {
            return Err(mcs_err(format!("MCS: controller error {val} ({reply:?})")));
        }
        Ok(())
    }
}

/// One MCS channel sharing a controller. Implements [`AsynMotor`].
pub struct McsAxis {
    controller: Arc<Mutex<McsController>>,
    /// Wire channel (from the create-axis command; distinct from the axis no.).
    channel: i32,
    /// Rotary vs linear stage (probed at construction).
    is_rot: bool,
    /// Sensor-present flag probed at construction.
    has_encoder: bool,
    /// Whether speed-set commands are suppressed (copied from the controller).
    disable_speed: bool,
    /// Cached closed-loop speed (`GCLS`), to skip redundant `SCLS` writes.
    vel_cached: i32,
}

impl McsAxis {
    /// Construct channel `channel`, probing speed, status, positioner type and
    /// rotation, matching the C `SmarActMCSAxis` constructor. Blocking I/O.
    pub fn new(controller: Arc<Mutex<McsController>>, channel: i32) -> AsynResult<Self> {
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let disable_speed = ctrl.disable_speed;

        let vel_cached = if disable_speed {
            0
        } else {
            ctrl.get_val("GCLS", channel)?
        };

        // Comms check.
        ctrl.get_val("GS", channel)?;

        // Rotation is detected by whether linear position reads back.
        let is_rot = ctrl.get_val("GP", channel).is_err();

        // Sensor type query (also a comms check).
        ctrl.get_val("GST", channel)?;

        let has_encoder = if is_rot {
            ctrl.get_angle(channel).is_ok()
        } else {
            true
        };

        drop(ctrl);
        Ok(Self {
            controller,
            channel,
            is_rot,
            has_encoder,
            disable_speed,
            vel_cached,
        })
    }

    fn lock(&self) -> MutexGuard<'_, McsController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Change the closed-loop speed if it differs from the cached value
    /// (C `setSpeed`); suppressed when `disable_speed` is set.
    fn set_speed(&mut self, velocity: f64) -> AsynResult<()> {
        if self.disable_speed {
            return Ok(());
        }
        let vel = velocity.abs().round() as i32;
        if vel != self.vel_cached {
            let ch = self.channel;
            self.lock().command(&format!(":SCLS{ch},{vel}"))?;
            self.vel_cached = vel;
        }
        Ok(())
    }

    /// Shared move: set speed, round to controller units, and issue the linear
    /// or rotary absolute/relative move with the (default) hold time.
    fn do_move(&mut self, position: f64, relative: bool, velocity: f64) -> AsynResult<()> {
        self.set_speed(velocity)?;
        let ch = self.channel;
        let rpos = position.round() as i64;

        let cmd = if self.is_rot {
            let mut angle = (rpos as i32) % (UDEG_PER_REV as i32);
            let mut rev = (rpos / UDEG_PER_REV) as i32;
            if angle < 0 {
                angle += UDEG_PER_REV as i32;
                rev -= 1;
            }
            let mnem = if relative { "MAR" } else { "MAA" };
            format!(":{mnem}{ch},{angle},{rev},{HOLD_TIME}")
        } else {
            let mnem = if relative { "MPR" } else { "MPA" };
            format!(":{mnem}{ch},{rpos},{HOLD_TIME}")
        };
        self.lock().command(&cmd)
    }
}

impl AsynMotor for McsAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false, velocity)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true, velocity)
    }

    fn move_velocity(
        &mut self,
        user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // No native jog: relative move to a very-far target (C moveVelocity).
        // Speed 0 would mean "unlimited" on the MCS, so treat it as a stop.
        let speed = velocity.abs().round() as i32;
        if speed == 0 {
            return self.stop(user, acceleration);
        }
        self.set_speed(velocity)?;
        let dir: i64 = if velocity < 0.0 { -1 } else { 1 };
        let ch = self.channel;
        let cmd = if self.is_rot {
            format!(":MAR{ch},0,{},0", FAR_AWAY_ROT * dir)
        } else {
            format!(":MPR{ch},{},0", FAR_AWAY_LIN * dir)
        };
        self.lock().command(&cmd)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        self.set_speed(velocity)?;
        let ch = self.channel;
        let dir = if forward { 0 } else { 1 };
        self.lock()
            .command(&format!(":FRM{ch},{dir},{HOLD_TIME},{AUTO_ZERO}"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ch = self.channel;
        self.lock().command(&format!(":S{ch}"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ch = self.channel;
        let rpos = position.round() as i32;
        if self.is_rot && !(0..UDEG_PER_REV as i32).contains(&rpos) {
            // Rotary set-position is only valid within one revolution (C).
            return Err(mcs_err(format!(
                "MCS: rotary set-position {rpos} out of [0, {UDEG_PER_REV})"
            )));
        }
        self.lock().command(&format!(":SP{ch},{rpos}"))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, _enable: bool) -> AsynResult<()> {
        // Not implemented in the C MCS driver.
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // Not implemented in the C MCS driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ch = self.channel;
        let ctrl = self.lock();

        let pos = if self.is_rot {
            let (angle, rev) = ctrl.get_angle(ch)?;
            rev as i64 * UDEG_PER_REV + angle as i64
        } else {
            ctrl.get_val("GP", ch)? as i64
        };

        let moving = status_is_moving(ctrl.get_val("GS", ch)?);
        let homed = ctrl.get_val("GPPK", ch)? != 0;
        drop(ctrl);

        Ok(MotorStatus {
            position: pos as f64,
            encoder_position: pos as f64,
            velocity: 0.0,
            done: !moving,
            moving,
            direction: true,
            has_encoder: self.has_encoder,
            gain_support: self.has_encoder,
            homed,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_value_replies_and_errors() {
        // Normal value reply.
        let (is_err, val) = parse_reply(":P0,12345").unwrap();
        assert!(!is_err);
        assert_eq!(val, 12345);
        // Negative position.
        let (_, val) = parse_reply(":P1,-500").unwrap();
        assert_eq!(val, -500);
        // Acknowledgement E…,0 is not an error.
        let (is_err, val) = parse_reply(":E0,0").unwrap();
        assert!(is_err);
        assert_eq!(val, 0);
        // E…,<n!=0> is an error code.
        let (is_err, val) = parse_reply(":E0,7").unwrap();
        assert!(is_err);
        assert_eq!(val, 7);
    }

    #[test]
    fn parses_angle_and_rejects_short_replies() {
        let (is_err, angle, rev) = parse_angle(":A0,123,2").unwrap();
        assert!(!is_err);
        assert_eq!(angle, 123);
        assert_eq!(rev, 2);
        // An error reply has too few fields to be an angle.
        assert!(parse_angle(":E0,5").is_none());
    }

    #[test]
    fn moving_status_codes() {
        for code in [1, 2, 4, 5, 6, 7] {
            assert!(status_is_moving(code), "code {code} should be moving");
        }
        for code in [0, 3, 9] {
            assert!(!status_is_moving(code), "code {code} should be idle");
        }
    }
}
