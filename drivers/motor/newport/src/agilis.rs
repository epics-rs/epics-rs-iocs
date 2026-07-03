//! Newport Agilis AG-UC2 / AG-UC8 / AG-UC8PC controller driver.
//!
//! Ported from `motorNewport/newportApp/src/AG_UC.cpp` (`AG_UCController` /
//! `AG_UCAxis`). The Agilis is an open-loop piezo stepper controller driving up
//! to 8 axes over one serial line. Axes are grouped into channels of two: an
//! axis is addressed by an **axis ID** (`1` or `2`, the command prefix) within
//! a **channel** selected by the controller-wide `CC` command. The UC2 is a
//! single-channel model (channel 0, no `CC` needed).
//!
//! ## Shared-serial concurrency & channel selection
//!
//! Like [`crate::agap`], every axis is an independent [`AsynMotor`] sharing one
//! [`AgUcController`] behind an `Arc<Mutex<..>>`. Because channel selection is
//! controller-wide state, an axis operation must select its channel and issue
//! its command atomically — both happen under the controller lock, so a second
//! axis cannot change the channel in between.
//!
//! The Agilis needs a short delay between serial writes (C `WRITE_DELAY`); this
//! driver sleeps [`WRITE_DELAY`] after every exchange, matching the C driver.
//!
//! ## Open-loop stepper positioning
//!
//! There is no absolute encoder: `TP` returns an accumulated step count. The
//! driver tracks `current_position` (updated each poll) plus a
//! `position_offset` set by `set_position`, and an absolute move is issued as a
//! relative `PR` of `target - current_position` — faithful to the C driver.
//!
//! ## Parity notes
//!
//! Two C bugs are ported faithfully and flagged at their sites:
//! - `AG_UC.cpp:242`: the reverse-amplitude default clobbers the *forward*
//!   amplitude (see [`setup_amplitudes`]).
//! - `AG_UC.cpp:383`: axis-2 limit test is `(lim == 3 || lim == 3)` — a
//!   duplicated condition (see [`ph_limit`]).

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::util::parse_int_at;

/// Response buffer size for a single controller reply.
const READ_BUF: usize = 256;

/// Command line terminator (CR/LF); this driver owns framing (see
/// [`crate::conex`] for rationale).
const TERMINATOR: &[u8] = b"\r\n";

/// C `WRITE_DELAY`: the Agilis needs a short delay between serial writes.
pub(crate) const WRITE_DELAY: Duration = Duration::from_millis(10);

/// C reset settle time after the `RS` command.
const RESET_SETTLE: Duration = Duration::from_millis(500);

/// C default forward step amplitude when the configured value is non-positive.
const DEFAULT_FORWARD_AMPLITUDE: i32 = 50;

/// Agilis controller model (C `AG_UCModel_t`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AgUcModel {
    /// AG-UC2 — single channel (channel 0), two axes.
    Uc2,
    /// AG-UC8 — four channels of two axes.
    Uc8,
    /// AG-UC8PC — AG-UC8 with PC / piezo-crystal firmware variant.
    Uc8Pc,
}

impl AgUcModel {
    /// C constructor `strstr` chain: `AG-UC2`, then `AG-UC8PC`, then `AG-UC8`
    /// (UC8PC must precede UC8 since `AG-UC8` is a substring of `AG-UC8PC`).
    pub fn from_version(version: &str) -> Option<Self> {
        if version.contains("AG-UC2") {
            Some(Self::Uc2)
        } else if version.contains("AG-UC8PC") {
            Some(Self::Uc8Pc)
        } else if version.contains("AG-UC8") {
            Some(Self::Uc8)
        } else {
            None
        }
    }
}

fn agilis_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

fn parse_error(what: &str) -> AsynError {
    agilis_err(format!("Agilis: could not parse {what} response"))
}

/// C `NINT(f)`: round to nearest integer, away from zero on the half.
fn nint(f: f64) -> i32 {
    (if f > 0.0 { f + 0.5 } else { f - 0.5 }) as i32
}

/// C `AG_UCAxis::velocityToSpeedCode`: bucket the velocity magnitude to an
/// Agilis speed code (1..=4), signed by direction.
fn velocity_to_speed_code(velocity: f64) -> i32 {
    let a = velocity.abs();
    let speed = if a <= 5.0 {
        1
    } else if a <= 100.0 {
        2
    } else if a <= 666.0 {
        4
    } else {
        3
    };
    if velocity < 0.0 { -speed } else { speed }
}

/// C `AG_UCAxis`: axis ID within its channel, `(axisNo % 2) + 1` (1 or 2).
fn axis_id(axis_no: i32) -> i32 {
    (axis_no % 2) + 1
}

/// C `AG_UCAxis`: channel ID — 0 for the single-channel UC2, else
/// `axisNo / 2 + 1`.
fn channel_id(model: AgUcModel, axis_no: i32) -> i32 {
    if model == AgUcModel::Uc2 {
        0
    } else {
        axis_no / 2 + 1
    }
}

/// C `AG_UCAxis` constructor step-amplitude setup, returning the (forward,
/// reverse) amplitudes to send in the two `SU` commands.
///
/// Faithfully reproduces the C bug at `AG_UC.cpp:242`: the code intends to
/// default the reverse amplitude to `-50` when it is non-negative, but assigns
/// the *forward* amplitude instead. So a non-negative reverse amplitude forces
/// the forward `SU` to `-50` and leaves the reverse amplitude unchanged.
fn setup_amplitudes(forward_amplitude: i32, reverse_amplitude: i32) -> (i32, i32) {
    let mut forward = if forward_amplitude <= 0 {
        DEFAULT_FORWARD_AMPLITUDE
    } else {
        forward_amplitude
    };
    if reverse_amplitude >= 0 {
        forward = -50; // C bug: intended `reverse = -50`.
    }
    (forward, reverse_amplitude)
}

/// C `AG_UCAxis::poll` limit test on the `PH` reply value.
///
/// Faithfully reproduces the C bug at `AG_UC.cpp:383`: the axis-2 test is
/// `(lim == 3 || lim == 3)` — a duplicated condition, most likely intended to
/// be `lim == 2 || lim == 3`.
fn ph_limit(axis_id: i32, lim: i32) -> bool {
    match axis_id {
        1 => lim == 1 || lim == 3,
        2 => lim == 3,
        _ => false,
    }
}

// --- Raw framed serial I/O, each followed by WRITE_DELAY (C spacing).

fn framed(cmd: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
    out.extend_from_slice(cmd.as_bytes());
    out.extend_from_slice(TERMINATOR);
    out
}

fn write_raw(handle: &SyncIOHandle, cmd: &str) -> AsynResult<()> {
    handle.write_octet(0, &framed(cmd))?;
    thread::sleep(WRITE_DELAY);
    Ok(())
}

fn write_read_raw(handle: &SyncIOHandle, cmd: &str) -> AsynResult<String> {
    handle.write_octet(0, &framed(cmd))?;
    let reply = handle.read_octet(0, READ_BUF)?;
    thread::sleep(WRITE_DELAY);
    Ok(String::from_utf8_lossy(&reply).into_owned())
}

/// Shared Agilis controller endpoint. Held behind a mutex so an axis's
/// channel-select plus command stay atomic on the shared line.
pub struct AgUcController {
    handle: SyncIOHandle,
    model: AgUcModel,
}

impl AgUcController {
    /// Connect and initialise an Agilis controller: reset (`RS`), enter remote
    /// mode (`MR`), read the firmware version (`VE`) and detect the model.
    /// Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        // Reset, then settle; the controller needs time after RS.
        write_raw(&handle, "RS")?;
        thread::sleep(RESET_SETTLE);
        // Remote mode.
        write_raw(&handle, "MR")?;
        // Firmware version → model.
        let version = write_read_raw(&handle, "VE")?;
        let model = AgUcModel::from_version(&version)
            .ok_or_else(|| agilis_err(format!("Agilis: unknown model, firmware=\"{version}\"")))?;
        Ok(Self { handle, model })
    }

    pub fn model(&self) -> AgUcModel {
        self.model
    }

    fn write(&self, cmd: &str) -> AsynResult<()> {
        write_raw(&self.handle, cmd)
    }

    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        write_read_raw(&self.handle, cmd)
    }

    /// C `setChannel`: unconditionally select `channel_id` via `CC` (the C
    /// driver issues even `CC0` when polling a UC2). Caller holds the lock.
    fn set_channel(&self, channel_id: i32) -> AsynResult<()> {
        self.write(&format!("CC{channel_id}"))
    }

    /// C `writeAgilis(channelID, ..)`: select the channel first when non-zero,
    /// then write the channel-scoped command. Caller holds the lock.
    fn write_on_channel(&self, channel_id: i32, cmd: &str) -> AsynResult<()> {
        if channel_id != 0 {
            self.set_channel(channel_id)?;
        }
        self.write(cmd)
    }
}

/// One Agilis axis sharing a controller. Implements [`AsynMotor`].
pub struct AgUcAxis {
    controller: Arc<Mutex<AgUcController>>,
    /// Command prefix within the channel (`1` or `2`; C `axisID_`).
    axis_id: i32,
    /// Channel selected via `CC` before commands (`0` for UC2; C `channelID_`).
    channel_id: i32,
    /// Whether the actuator has limit switches (C `hasLimits_`).
    has_limits: bool,
    /// Accumulated step position last read from `TP`, plus offset (C
    /// `currentPosition_`).
    current_position: i32,
    /// Offset applied by `set_position` (C `positionOffset_`).
    position_offset: i32,
}

impl AgUcAxis {
    /// Construct axis `axis_no` on a shared controller, sending the forward and
    /// reverse step-amplitude (`SU`) commands. Performs blocking serial I/O
    /// under the controller lock.
    pub fn new(
        controller: Arc<Mutex<AgUcController>>,
        axis_no: i32,
        has_limits: bool,
        forward_amplitude: i32,
        reverse_amplitude: i32,
    ) -> AsynResult<Self> {
        let (axis_id, channel_id) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let id = axis_id(axis_no);
            let ch = channel_id(ctrl.model, axis_no);
            let (forward, reverse) = setup_amplitudes(forward_amplitude, reverse_amplitude);
            ctrl.write_on_channel(ch, &format!("{id}SU{forward}"))?;
            ctrl.write_on_channel(ch, &format!("{id}SU{reverse}"))?;
            (id, ch)
        };

        Ok(Self {
            controller,
            axis_id,
            channel_id,
            has_limits,
            current_position: 0,
            position_offset: 0,
        })
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, AgUcController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for AgUcAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // Open-loop: absolute move is a relative PR from the tracked position.
        let steps = nint(position - self.current_position as f64);
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}PR{steps}", self.axis_id))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let steps = nint(distance);
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}PR{steps}", self.axis_id))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C `moveVelocity`: jog (JA) at the speed code.
        let code = velocity_to_speed_code(velocity);
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}JA{code}", self.axis_id))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `home`: only valid with limit switches; move to limit (MV) at the
        // speed code.
        if !self.has_limits {
            return Err(agilis_err("Agilis: home requires limit switches".into()));
        }
        let code = velocity_to_speed_code(velocity);
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}MV{code}", self.axis_id))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_on_channel(self.channel_id, &format!("{}ST", self.axis_id))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `setPosition`: rebase the tracked position via an offset.
        self.position_offset = nint(position) - self.current_position;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Clone the Arc into a local so the guard borrows it rather than
        // `self`, leaving `self.current_position` free to update below.
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());

        // Select this axis's channel (C issues CC unconditionally, incl. CC0
        // on a UC2).
        ctrl.set_channel(self.channel_id)?;

        // Position (TP): accumulated step count at offset 3 ("1TP").
        let position = parse_int_at(&ctrl.write_read(&format!("{}TP", self.axis_id))?, 3)
            .ok_or_else(|| parse_error("TP"))?;
        self.current_position = position + self.position_offset;

        // Moving status (TS): state char at index 3; '0' means done.
        let ts = ctrl.write_read(&format!("{}TS", self.axis_id))?;
        let state = *ts.as_bytes().get(3).ok_or_else(|| parse_error("TS"))?;
        let moving = state != b'0';

        // Limit status (PH, controller-wide): value at offset 2.
        let lim = parse_int_at(&ctrl.write_read("PH")?, 2).ok_or_else(|| parse_error("PH"))?;
        let limit = ph_limit(self.axis_id, lim);

        Ok(MotorStatus {
            position: self.current_position as f64,
            done: !moving,
            moving,
            // C sets both low and high limit to the same flag.
            low_limit: limit,
            high_limit: limit,
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_detected_from_firmware_string() {
        assert_eq!(AgUcModel::from_version("AG-UC2 v1"), Some(AgUcModel::Uc2));
        assert_eq!(AgUcModel::from_version("AG-UC8 v2"), Some(AgUcModel::Uc8));
        assert_eq!(
            AgUcModel::from_version("AG-UC8PC v2"),
            Some(AgUcModel::Uc8Pc)
        );
        assert_eq!(AgUcModel::from_version("mystery"), None);
    }

    #[test]
    fn uc8pc_detected_before_uc8_substring() {
        // "AG-UC8" is a substring of "AG-UC8PC"; the PC variant must win.
        assert_eq!(AgUcModel::from_version("AG-UC8PC"), Some(AgUcModel::Uc8Pc));
    }

    #[test]
    fn nint_rounds_away_from_zero() {
        assert_eq!(nint(1.5), 2);
        assert_eq!(nint(1.4), 1);
        assert_eq!(nint(-1.5), -2);
        assert_eq!(nint(-1.4), -1);
        assert_eq!(nint(0.0), 0);
    }

    #[test]
    fn velocity_to_speed_code_buckets_and_signs() {
        assert_eq!(velocity_to_speed_code(3.0), 1);
        assert_eq!(velocity_to_speed_code(50.0), 2);
        assert_eq!(velocity_to_speed_code(500.0), 4);
        assert_eq!(velocity_to_speed_code(1000.0), 3);
        // Sign follows direction.
        assert_eq!(velocity_to_speed_code(-3.0), -1);
        assert_eq!(velocity_to_speed_code(-1000.0), -3);
    }

    #[test]
    fn axis_and_channel_ids_match_c() {
        // axisID = axisNo % 2 + 1.
        assert_eq!(axis_id(0), 1);
        assert_eq!(axis_id(1), 2);
        assert_eq!(axis_id(2), 1);
        assert_eq!(axis_id(7), 2);
        // UC2 → channel 0; UC8 → axisNo/2 + 1.
        assert_eq!(channel_id(AgUcModel::Uc2, 0), 0);
        assert_eq!(channel_id(AgUcModel::Uc2, 1), 0);
        assert_eq!(channel_id(AgUcModel::Uc8, 0), 1);
        assert_eq!(channel_id(AgUcModel::Uc8, 1), 1);
        assert_eq!(channel_id(AgUcModel::Uc8, 2), 2);
        assert_eq!(channel_id(AgUcModel::Uc8, 7), 4);
    }

    #[test]
    fn setup_amplitudes_replicates_forward_clobber_bug() {
        // Non-positive forward → default 50; negative reverse leaves it as 50.
        assert_eq!(setup_amplitudes(0, -50), (50, -50));
        assert_eq!(setup_amplitudes(30, -50), (30, -50));
        // C bug: a non-negative reverse forces forward to -50 (reverse kept).
        assert_eq!(setup_amplitudes(30, 0), (-50, 0));
        assert_eq!(setup_amplitudes(30, 20), (-50, 20));
    }

    #[test]
    fn ph_limit_replicates_axis2_duplicate_condition() {
        // Axis 1: limit when lim is 1 or 3.
        assert!(ph_limit(1, 1));
        assert!(ph_limit(1, 3));
        assert!(!ph_limit(1, 2));
        // Axis 2 (C bug): only lim == 3 triggers (duplicate condition).
        assert!(ph_limit(2, 3));
        assert!(!ph_limit(2, 2));
        assert!(!ph_limit(2, 1));
    }
}
