//! Micos SMC Taurus controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorMicos/micosApp/src/SMCTaurusAxis.cpp` +
//! `SMCTaurusDriver.cpp` (a model-3 `asynMotorController`/`asynMotorAxis` pair).
//! Connects over a `drvAsynIPPort` or `drvAsynSerialPort`; commands are ASCII,
//! CR/LF-terminated (the startup script sets the input EOS, the driver owns
//! output framing).
//!
//! The Taurus command set is the per-axis SMC dialect shared with the hydra
//! sibling (`<value> <axis> <verb>` setters, `<axis> <verb>` queries). Compared
//! to the hydra it is simpler: no motor-form probe (the resolution is always
//! `clPeriod`), `stop` is a bare `nabort`, and `set_closed_loop(true)` sends the
//! closed-loop flag directly as the regulator mode. It additionally supports
//! pushing the record soft limits to the controller (`setnlimit`).
//!
//! ## Units
//!
//! The C driver's `axisRes = clPeriod` only bridges the record's raw-step
//! boundary to the controller's engineering units and cancels at the EGU
//! boundary, so it is dropped: the driver boundary is controller-native units
//! with `MRES` = 1. `SMCTaurusChangeResolution` is not ported — set the record
//! `MRES` instead.
//!
//! ## Deviations (documented)
//!
//! - The C poll computes an e-stop-switch problem flag from status bit `0x200`
//!   but then unconditionally overwrites the problem flag to `0` a few lines
//!   later (a latent bug that discards it). This port honours the intent and
//!   reports bit `0x200` as a problem.
//! - The C poll issues `gnv`/`gna` reads whose replies are discarded; these dead
//!   reads are omitted.
//! - The Taurus-specific limit-switch type parameters (`setsw`/`getsw` driven by
//!   the `SMCTaurusLimit*Type` aux PVs, commented out in the C poll) are not
//!   modeled.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi};

/// Response buffer size.
const READ_BUF: usize = 128;

/// Command terminator; the driver owns output framing.
const TERMINATOR: &[u8] = b"\r\n";

/// `nst` status bits.
const STATUS_MOVING: i32 = 0x1; // axis in motion
const STATUS_POWER_OFF: i32 = 0x100; // drive power disabled
const STATUS_ESTOP_SWITCH: i32 = 0x200; // e-stop switch active

/// Switch-config bit that disables (masks) a limit switch.
const SWITCH_IGNORE: i32 = 0x2;

/// Delay required by the controller after an `init` command.
const INIT_DELAY: Duration = Duration::from_millis(200);

fn taurus_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared Taurus controller endpoint owning the asyn octet handle.
pub struct TaurusController {
    handle: SyncIOHandle,
}

impl TaurusController {
    /// Wrap a connected octet handle.
    pub fn new(handle: SyncIOHandle) -> Self {
        Self { handle }
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command with no reply expected.
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Write a query and read one reply line.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        self.write_only(cmd)?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string())
    }
}

/// One Taurus axis sharing a controller. Implements [`AsynMotor`].
pub struct TaurusAxis {
    controller: Arc<Mutex<TaurusController>>,
    /// 0-based axis index; commands use `axis_no + 1`.
    axis_no: usize,
    /// Cached travel limits (needed as a pair for `setnlimit`).
    neg_travel_limit: f64,
    pos_travel_limit: f64,
}

impl TaurusAxis {
    /// Construct axis `axis_no` (0-based) and read its travel limits.
    pub fn new(controller: Arc<Mutex<TaurusController>>, axis_no: usize) -> AsynResult<Self> {
        let addr = axis_no + 1;
        let reply = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            ctrl.query(&format!("{addr} getnlimit"))?
        };
        let bounds: Vec<f64> = reply
            .split_whitespace()
            .filter_map(|tok| tok.parse::<f64>().ok())
            .collect();
        let (neg, pos) = match bounds.as_slice() {
            [neg, pos, ..] => (*neg, *pos),
            _ => {
                return Err(taurus_err(format!(
                    "taurus: getnlimit returned {reply:?}, expected two values"
                )));
            }
        };
        Ok(Self {
            controller,
            axis_no,
            neg_travel_limit: neg,
            pos_travel_limit: pos,
        })
    }

    fn addr(&self) -> usize {
        self.axis_no + 1
    }

    fn lock(&self) -> MutexGuard<'_, TaurusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Send the per-axis velocity and acceleration (`snv`/`sna`).
    fn send_accel_velocity(
        &self,
        ctrl: &TaurusController,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let addr = self.addr();
        ctrl.write_only(&format!("{:.6} {addr} snv", velocity.abs()))?;
        ctrl.write_only(&format!("{:.6} {addr} sna", acceleration.abs()))
    }

    /// Push the current travel-limit pair to the controller (`setnlimit`).
    fn send_travel_limits(&self, ctrl: &TaurusController) -> AsynResult<()> {
        ctrl.write_only(&format!(
            "{:.6} {:.6} {} setnlimit",
            self.neg_travel_limit,
            self.pos_travel_limit,
            self.addr()
        ))
    }
}

impl AsynMotor for TaurusAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        self.send_accel_velocity(&ctrl, velocity, acceleration)?;
        ctrl.write_only(&format!("{position:.6} {} nm", self.addr()))
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
        self.send_accel_velocity(&ctrl, velocity, acceleration)?;
        ctrl.write_only(&format!("{distance:.6} {} nr", self.addr()))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        // No jog command — move to the cached axis travel limit.
        self.send_accel_velocity(&ctrl, velocity, acceleration)?;
        let target = if velocity > 0.0 {
            self.pos_travel_limit
        } else {
            self.neg_travel_limit
        };
        ctrl.write_only(&format!("{target:.6} {} nm", self.addr()))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        self.send_accel_velocity(&ctrl, velocity, acceleration)?;
        let verb = if forward { "nrm" } else { "ncal" };
        ctrl.write_only(&format!("{} {verb}", self.addr()))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_only(&format!("{} nabort", self.addr()))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        // setnpos takes the distance from the current position to the desired
        // origin, hence the negation (C multiplies by -1).
        ctrl.write_only(&format!("{:.6} {} setnpos", -position, self.addr()))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        let addr = self.addr();
        if enable {
            // C sends the closed-loop flag itself (1) as the regulator mode.
            ctrl.write_only(&format!("1 {addr} setcloop"))?;
            // Reinit so the closed-loop setting takes effect (powers the motor);
            // the controller needs a delay after init.
            ctrl.write_only(&format!("{addr} init"))?;
            std::thread::sleep(INIT_DELAY);
            Ok(())
        } else {
            ctrl.write_only(&format!("{addr} motoroff"))
        }
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.pos_travel_limit = position;
        let ctrl = self.lock();
        self.send_travel_limits(&ctrl)
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.neg_travel_limit = position;
        let ctrl = self.lock();
        self.send_travel_limits(&ctrl)
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // No PID-gain support in the C Taurus driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let addr = self.addr();

        let position = atof(&ctrl.query(&format!("{addr} np"))?);

        let status = atoi(&ctrl.query(&format!("{addr} nst"))?);
        let moving = (status & STATUS_MOVING) != 0;
        let powered = (status & STATUS_POWER_OFF) == 0;
        let problem = (status & STATUS_ESTOP_SWITCH) != 0;

        // Switch config: bit 1 (0x2) masks (disables) the switch.
        let sw = ctrl.query(&format!("{addr} getsw"))?;
        let cfg: Vec<i32> = sw
            .split_whitespace()
            .filter_map(|tok| tok.parse::<i32>().ok())
            .collect();
        let (ignore_low, ignore_high) = match cfg.as_slice() {
            [low, high, ..] => (low & SWITCH_IGNORE, high & SWITCH_IGNORE),
            _ => (0, 0),
        };

        // Switch state: "low high", 0=inactive 1=active.
        let st = ctrl.query(&format!("{addr} getswst"))?;
        let state: Vec<i32> = st
            .split_whitespace()
            .filter_map(|tok| tok.parse::<i32>().ok())
            .collect();
        let (mut low_limit, mut high_limit) = (false, false);
        if let [low, high, ..] = state.as_slice() {
            low_limit = ignore_low == 0 && *low != 0;
            high_limit = ignore_high == 0 && *high != 0;
        }
        drop(ctrl);

        Ok(MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0,
            done: !moving,
            moving,
            direction: true,
            has_encoder: true,
            gain_support: true,
            high_limit,
            low_limit,
            powered,
            problem,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_bits() {
        let moving = |s: i32| (s & STATUS_MOVING) != 0;
        let powered = |s: i32| (s & STATUS_POWER_OFF) == 0;
        let problem = |s: i32| (s & STATUS_ESTOP_SWITCH) != 0;
        assert!(moving(0x1));
        assert!(!powered(0x100));
        assert!(powered(0x1));
        assert!(problem(0x200));
        assert!(!problem(0x1));
    }

    #[test]
    fn switch_ignore_mask() {
        let ignored = |cfg: i32| (cfg & SWITCH_IGNORE) != 0;
        assert!(!ignored(0x1));
        assert!(ignored(0x2));
    }
}
