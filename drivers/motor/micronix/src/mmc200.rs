//! Micronix MMC-100/103/110/200 motor controller driver (serial ASCII).
//!
//! Ported from `motorMicronix/micronixApp/src/MMC200Driver.cpp`, a model-3
//! `asynMotorController`/`asynMotorAxis` driver. Axes are numbered from 1 on
//! the wire; commands are `{n}CMD` and queries `{n}CMD?` reply with a leading
//! `#` (e.g. `#MMC-200 …`, `#8`, `#0.000000,0.000000`). Motion commands take
//! no reply — only `?` queries are read — so this port pairs
//! [`Mmc200Controller::write`] (fire-and-forget) with
//! [`Mmc200Controller::query`] (write + read) exactly as the C driver pairs
//! `writeController` with `writeReadController`.
//!
//! ## Units
//!
//! The C driver carries a per-axis `resolution_` (mm-or-deg per microstep or
//! per encoder count, derived from `FBK?`/`REZ?`/`UST?`/`ENC?`) purely to
//! convert between the record's raw-step boundary and the controller's
//! physical units. The asyn-rs motor boundary is dial-frame EGU (the
//! controller's own mm/deg), so every `× resolution_` (positions, velocities,
//! accelerations sent) and `÷ resolution_` (positions read) cancels: values
//! pass through unscaled. Consequently the `FBK?`/`REZ?`/`UST?`/`ENC?` probes,
//! whose only consumer was `resolution_`, are not issued — only `VER?` (model,
//! for the MMC-ETH ignore) and `VMX?` (max velocity, for the jog-percent
//! calculation) survive. The record's `MRES` carries the physical resolution
//! for display and is operator-configured in the substitution file.
//!
//! ## Deviations from C (documented)
//!
//! - C's per-axis constructor sets `motorStatusProblem_` and continues when an
//!   init query fails; this port returns an error from [`Mmc200Axis::new`]
//!   instead (a controller that cannot answer `VER?`/`VMX?` cannot be
//!   operated), naming the failure at controller-create time.
//! - The MMC-ETH module (`#MMC-ETH`, C `model_ == 999`) is not a motor axis;
//!   as in C only its poll is disabled (returns an inert, done status). C does
//!   not guard its motion methods against `999` and neither does this port.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi};

/// Response buffer size for a single controller reply.
const READ_BUF: usize = 256;

/// Command line terminator (st.cmd configures `asynOctetSetOutputEos("\r")`).
const TERMINATOR: &[u8] = b"\r";

/// C `model_ == 999`: the MMC-ETH ethernet module, not a motor axis.
const MODEL_ETH: i32 = 999;

/// Status word bits (C `poll` `status`).
const STA_LOW_LIMIT: i32 = 0x1;
const STA_HIGH_LIMIT: i32 = 0x2;
const STA_DONE: i32 = 0x8;
const STA_ERROR: i32 = 0x80;

/// Shared controller endpoint: owns the serial handle and command framing.
/// The caller holds the `Arc<Mutex<_>>` lock across a write→read exchange.
pub struct Mmc200Controller {
    handle: SyncIOHandle,
}

impl Mmc200Controller {
    /// Wrap a connected octet handle. Axes are created separately (the count
    /// is a `MMC200CreateController` argument, not autodiscovered).
    pub fn new(handle: SyncIOHandle) -> Self {
        Self { handle }
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command message with no reply (C `writeController`); the
    /// terminator is appended here.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Write a query and read its reply (C `writeReadController`), trimming the
    /// trailing line terminator. The leading `#` is left intact — callers skip
    /// it with `reply.get(1..)`, matching C's `&inString_[1]`.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        self.write(cmd)?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let reply = String::from_utf8_lossy(&raw);
        Ok(reply.trim_end_matches(['\r', '\n', '\0']).to_string())
    }
}

/// Parse the `VER?` reply into the C `model_` code: `#MMC-200` → 200,
/// `#MMC-100` → 100, `#MMC-103` → 103, `#MMC-110` → 110, `#MMC-ETH` → 999.
/// A too-short or unrecognized string is `-1` (C behaviour).
fn parse_model(version: &str) -> i32 {
    if version.len() <= 8 {
        return -1;
    }
    match &version[..8] {
        "#MMC-200" => 200,
        "#MMC-100" => 100,
        "#MMC-103" => 103,
        "#MMC-110" => 110,
        "#MMC-ETH" => MODEL_ETH,
        _ => -1,
    }
}

/// Parse a `POS?` reply `#<theoretical>,<encoder>` into `(position, encoder)`
/// in EGU (C `sscanf("#%lf,%lf")`). A missing encoder field reads 0.
fn parse_position(reply: &str) -> (f64, f64) {
    let body = reply.get(1..).unwrap_or("");
    match body.split_once(',') {
        Some((pos, enc)) => (atof(pos), atof(enc)),
        None => (atof(body), 0.0),
    }
}

/// One MMC-200 axis sharing a controller. Implements [`AsynMotor`].
pub struct Mmc200Axis {
    controller: Arc<Mutex<Mmc200Controller>>,
    /// 1-based controller axis number (C `axisIndex_ = axisNo + 1`).
    axis_index: i32,
    /// C `model_` (200/100/103/110/999/-1).
    model: i32,
    /// False for the MMC-ETH module (`model_ == 999`): polling is disabled.
    active: bool,
    /// C `maxVelocity_` from `VMX?`, in EGU/sec — the jog `JOG %` denominator.
    max_velocity: f64,
    /// Controller-wide `ignoreLimits` flag (C `ignoreLimits_`).
    ignore_limits: bool,
    /// Last polled status, reused on the comm-failure early exit.
    last_status: MotorStatus,
}

impl Mmc200Axis {
    /// Construct the axis at 0-based `index`, running the surviving part of the
    /// C per-motor init: `VER?` (model, for the MMC-ETH ignore) and, for a real
    /// axis, `VMX?` (max velocity for jog). Performs blocking serial I/O under
    /// the controller lock.
    pub fn new(
        controller: Arc<Mutex<Mmc200Controller>>,
        index: usize,
        ignore_limits: bool,
    ) -> AsynResult<Self> {
        let axis_index = index as i32 + 1;
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());

        let version = ctrl.query(&format!("{axis_index}VER?"))?;
        let model = parse_model(&version);

        if model == MODEL_ETH {
            drop(ctrl);
            return Ok(Self {
                controller,
                axis_index,
                model,
                active: false,
                max_velocity: 0.0,
                ignore_limits,
                last_status: MotorStatus {
                    done: true,
                    ..MotorStatus::default()
                },
            });
        }

        let vmx = ctrl.query(&format!("{axis_index}VMX?"))?;
        let max_velocity = atof(vmx.get(1..).unwrap_or(""));
        drop(ctrl);

        Ok(Self {
            controller,
            axis_index,
            model,
            active: true,
            max_velocity,
            ignore_limits,
            last_status: MotorStatus::default(),
        })
    }

    fn lock(&self) -> MutexGuard<'_, Mmc200Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `sendAccelAndVelocity`: `VEL`, then `ACC` and `DEC` (accel used for
    /// both). Values are EGU/sec and EGU/sec² (the `× resolution_` cancels at
    /// the EGU boundary — module Units note).
    fn send_accel_vel(
        ctrl: &Mmc200Controller,
        axis: i32,
        acceleration: f64,
        velocity: f64,
    ) -> AsynResult<()> {
        ctrl.write(&format!("{axis}VEL{velocity:.3}"))?;
        ctrl.write(&format!("{axis}ACC{acceleration:.3}"))?;
        ctrl.write(&format!("{axis}DEC{acceleration:.3}"))?;
        Ok(())
    }

    /// The controller model reported by `VER?` (diagnostic).
    pub fn model(&self) -> i32 {
        self.model
    }
}

impl AsynMotor for Mmc200Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        Self::send_accel_vel(&ctrl, n, acceleration, velocity)?;
        ctrl.write(&format!("{n}MVA{position:.6}"))?;
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
        let n = self.axis_index;
        let ctrl = self.lock();
        Self::send_accel_vel(&ctrl, n, acceleration, velocity)?;
        ctrl.write(&format!("{n}MVR{distance:.6}"))?;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C `moveVelocity`: the JOG command takes a speed as a percentage of
        // max velocity. Under the EGU boundary velocity is already EGU/sec, so
        // the percentage is `velocity / maxVelocity_ * 100` (the `×
        // resolution_` in C cancels), clamped to ±100.
        let n = self.axis_index;
        let jog_percent = (velocity / self.max_velocity * 100.0).clamp(-100.0, 100.0);
        let ctrl = self.lock();
        ctrl.write(&format!("{n}JAC{acceleration:.3}"))?;
        ctrl.write(&format!("{n}JOG{jog_percent:.3}"))?;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        Self::send_accel_vel(&ctrl, n, acceleration, velocity)?;
        // C `HCG1` selects the forward home direction, `HCG0` the reverse.
        ctrl.write(&format!("{n}HCG{}", if forward { 1 } else { 0 }))?;
        ctrl.write(&format!("{n}HOM"))?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        ctrl.write(&format!("{n}STP"))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `setPosition`: the MMC-200 can only zero the position (`ZRO`); a
        // non-zero request is a quiet success (no command).
        if position == 0.0 {
            let n = self.axis_index;
            let ctrl = self.lock();
            ctrl.write(&format!("{n}ZRO"))?;
        }
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C `setClosedLoop` toggles motor power (`MOT1`/`MOT0`).
        let n = self.axis_index;
        let ctrl = self.lock();
        ctrl.write(&format!("{n}MOT{}", if enable { 1 } else { 0 }))?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // C ignores the MMC-ETH module in poll (returns asynSuccess without
        // updating any parameter); keep the record inert.
        if !self.active {
            return Ok(self.last_status.clone());
        }

        let n = self.axis_index;
        let ctrl = self.lock();
        // C `poll`: `POS?` (position + encoder), `STA?` (status word, clearing
        // the error buffer with `CER` when the error bit is set), `MOT?`
        // (drive power). Any read failure is C's `goto skip` — mark the record
        // in error and keep the previous readings.
        let readings = (|| -> AsynResult<(f64, f64, i32, i32)> {
            let pos_reply = ctrl.query(&format!("{n}POS?"))?;
            let (position, encoder) = parse_position(&pos_reply);

            let sta_reply = ctrl.query(&format!("{n}STA?"))?;
            let status = atoi(sta_reply.get(1..).unwrap_or(""));
            if status & STA_ERROR != 0 {
                ctrl.write(&format!("{n}CER"))?;
            }

            let mot_reply = ctrl.query(&format!("{n}MOT?"))?;
            let drive_on = atoi(mot_reply.get(1..).unwrap_or(""));
            Ok((position, encoder, status, drive_on))
        })();
        drop(ctrl);

        match readings {
            Ok((position, encoder, status, drive_on)) => {
                let done = status & STA_DONE != 0;
                let (high_limit, low_limit) = if self.ignore_limits {
                    (false, false)
                } else {
                    (status & STA_HIGH_LIMIT != 0, status & STA_LOW_LIMIT != 0)
                };
                self.last_status = MotorStatus {
                    position,
                    encoder_position: encoder,
                    velocity: 0.0,
                    done,
                    moving: !done,
                    high_limit,
                    low_limit,
                    home: false,
                    powered: drive_on != 0,
                    problem: false,
                    comms_error: false,
                    gain_support: true,
                    has_encoder: true,
                    ..MotorStatus::default()
                };
            }
            Err(_) => {
                self.last_status.problem = true;
                self.last_status.comms_error = true;
            }
        }
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_model_recognizes_families() {
        assert_eq!(parse_model("#MMC-200 v1.2.3"), 200);
        assert_eq!(parse_model("#MMC-100 v1"), 100);
        assert_eq!(parse_model("#MMC-103 v1"), 103);
        assert_eq!(parse_model("#MMC-110 v1"), 110);
        assert_eq!(parse_model("#MMC-ETH v1"), MODEL_ETH);
        // Unrecognized model string.
        assert_eq!(parse_model("#MMC-999 v1"), -1);
        // Too short (C: strlen <= 8).
        assert_eq!(parse_model("#MMC-200"), -1);
        assert_eq!(parse_model("#"), -1);
    }

    #[test]
    fn parse_position_reads_theoretical_and_encoder() {
        assert_eq!(parse_position("#1.500000,1.499000"), (1.5, 1.499));
        assert_eq!(parse_position("#-0.250000,-0.250000"), (-0.25, -0.25));
        // Missing encoder field: encoder reads 0.
        assert_eq!(parse_position("#3.000000"), (3.0, 0.0));
    }

    #[test]
    fn status_bits_match_c_masks() {
        // done, high+low limits, error clear.
        assert_ne!(0x8 & STA_DONE, 0);
        assert_ne!(0x2 & STA_HIGH_LIMIT, 0);
        assert_ne!(0x1 & STA_LOW_LIMIT, 0);
        assert_ne!(0x80 & STA_ERROR, 0);
        // done bit isolated from limits.
        assert_eq!(0x8 & (STA_HIGH_LIMIT | STA_LOW_LIMIT), 0);
    }
}
