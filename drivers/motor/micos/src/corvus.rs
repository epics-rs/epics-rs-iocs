//! Micos SMC corvus controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorMicos/micosApp/src/SMCcorvusDriver.cpp` (a model-3
//! `asynMotorController`/`asynMotorAxis` pair). Connects over a `drvAsynIPPort`
//! or `drvAsynSerialPort`; commands are ASCII, CR/LF-terminated (the startup
//! script sets the input EOS, the driver owns output framing).
//!
//! ## Shared controller
//!
//! The corvus exposes only whole-controller operations: `pos` returns every
//! axis' position on one line, `move`/`rmove` take a target for every axis at
//! once, `st` is a global moving status, and `abort`/`sv`/`sa` act on all axes.
//! A single-axis command is therefore a read-modify-write of the full target
//! vector: read all positions, replace the entry for this axis, and send the
//! whole line. All axes share one [`CorvusController`] behind a mutex so that
//! read-modify-write is atomic.
//!
//! ## Units
//!
//! The corvus works in physical units (the `pos`/`move` values are already
//! engineering units). The C driver's `axisRes = pitch / (4 · polePairs)` only
//! bridges the record's raw-step boundary to those units and cancels at the EGU
//! boundary, so it is dropped: the driver boundary is controller-native units
//! with `MRES` = 1. The `SMCcorvusChangeResolution` iocsh command (a runtime
//! `axisRes` override) is likewise not ported — set the record `MRES` instead.
//!
//! ## Deviations (documented)
//!
//! - Relative moves send `0` for the non-moving axes (a true "stay put" delta).
//!   The C code sends each non-moving axis' *current absolute position* as its
//!   `rmove` delta, which would move them by that amount — a latent bug for
//!   multi-axis relative moves; this port sends `0`.
//! - `move_velocity` moves to the axis travel limit read from `getnlimit` (the C
//!   driver uses the record's `HLM`/`LLM`, which are not visible at this
//!   boundary; the controller travel limits carry the same intent). The corvus
//!   has no reliable jog command (its `speed` command is noted as crashing the
//!   controller), so moving to a limit matches C.
//! - Limit and drive-power status are read only when the axis is done (matching
//!   C, which avoids polling switches mid-move to keep the interpreter
//!   responsive); while moving they are reported as not-triggered / powered.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::atoi;

/// Response buffer size.
const READ_BUF: usize = 128;

/// Command terminator; the driver owns output framing.
const TERMINATOR: &[u8] = b"\r\n";

/// `st` status-word bit set while any axis is moving.
const STATUS_MOVING: i32 = 0x1;

/// Switch-config bit that disables (masks) a limit switch.
const SWITCH_IGNORE: i32 = 0x2;

fn corvus_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared corvus controller endpoint owning the asyn octet handle.
pub struct CorvusController {
    handle: SyncIOHandle,
    num_axes: usize,
}

impl CorvusController {
    /// Wrap a connected octet handle for a controller with `num_axes` axes.
    pub fn new(handle: SyncIOHandle, num_axes: usize) -> Self {
        Self { handle, num_axes }
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

    /// Read every axis position (`pos`) as a vector of exactly `num_axes`
    /// engineering-unit values.
    fn read_positions(&self) -> AsynResult<Vec<f64>> {
        let reply = self.query("pos")?;
        let values: Vec<f64> = reply
            .split_whitespace()
            .filter_map(|tok| tok.parse::<f64>().ok())
            .collect();
        if values.len() < self.num_axes {
            return Err(corvus_err(format!(
                "corvus: pos returned {} values, expected {}",
                values.len(),
                self.num_axes
            )));
        }
        Ok(values[..self.num_axes].to_vec())
    }

    /// Send the global velocity and acceleration (`sv`/`sa`, all axes).
    fn send_accel_velocity(&self, velocity: f64, acceleration: f64) -> AsynResult<()> {
        self.write_only(&format!("{:.6} sv", velocity.abs()))?;
        self.write_only(&format!("{:.6} sa", acceleration.abs()))
    }

    /// Join a target vector and append `verb` (`move`, `rmove`, or `setpos`).
    fn send_target_vector(&self, targets: &[f64], verb: &str) -> AsynResult<()> {
        let mut cmd = String::new();
        for t in targets {
            cmd.push_str(&format!("{t:.6} "));
        }
        cmd.push_str(verb);
        self.write_only(&cmd)
    }
}

/// One corvus axis sharing a controller. Implements [`AsynMotor`].
pub struct CorvusAxis {
    controller: Arc<Mutex<CorvusController>>,
    /// 0-based axis index (command prefix uses `axis_no + 1`).
    axis_no: usize,
    num_axes: usize,
}

impl CorvusAxis {
    /// Construct axis `axis_no` (0-based). No per-axis probing is needed since
    /// the resolution scale is dropped at the EGU boundary.
    pub fn new(controller: Arc<Mutex<CorvusController>>, axis_no: usize) -> AsynResult<Self> {
        let num_axes = controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .num_axes;
        if axis_no >= num_axes {
            return Err(corvus_err(format!(
                "corvus: axis {axis_no} out of range (num_axes {num_axes})"
            )));
        }
        Ok(Self {
            controller,
            axis_no,
            num_axes,
        })
    }

    fn lock(&self) -> MutexGuard<'_, CorvusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Absolute move of this axis (others held at their current position).
    fn absolute_move(
        &self,
        ctrl: &CorvusController,
        target: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        ctrl.send_accel_velocity(velocity, acceleration)?;
        let mut targets = ctrl.read_positions()?;
        targets[self.axis_no] = target;
        ctrl.send_target_vector(&targets, "move")
    }
}

impl AsynMotor for CorvusAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        self.absolute_move(&ctrl, position, velocity, acceleration)
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
        ctrl.send_accel_velocity(velocity, acceleration)?;
        // Non-moving axes get a zero delta (stay put); see module deviations.
        let mut targets = vec![0.0; self.num_axes];
        targets[self.axis_no] = distance;
        ctrl.send_target_vector(&targets, "rmove")
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        // No reliable jog on the corvus — move to the axis travel limit.
        let reply = ctrl.query(&format!("{} getnlimit", self.axis_no + 1))?;
        let bounds: Vec<f64> = reply
            .split_whitespace()
            .filter_map(|tok| tok.parse::<f64>().ok())
            .collect();
        let (neg, pos) = match bounds.as_slice() {
            [neg, pos, ..] => (*neg, *pos),
            _ => {
                return Err(corvus_err(format!(
                    "corvus: getnlimit returned {reply:?}, expected two values"
                )));
            }
        };
        let target = if velocity > 0.0 { pos } else { neg };
        self.absolute_move(&ctrl, target, velocity, acceleration)
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
        ctrl.send_accel_velocity(velocity, acceleration)?;
        // Forward homes to the range-measure switch (nrm); reverse calibrates
        // to the low switch (ncal).
        let verb = if forward { "nrm" } else { "ncal" };
        ctrl.write_only(&format!("{} {verb}", self.axis_no + 1))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // The corvus can only abort all axes at once.
        let ctrl = self.lock();
        ctrl.write_only("abort")
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let mut targets = ctrl.read_positions()?;
        // setpos takes the distance from the current position to the desired
        // origin, hence the negation (C multiplies by -1).
        targets[self.axis_no] = -position;
        ctrl.send_target_vector(&targets, "setpos")
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_only(&format!(
            "{} {} setcloop",
            if enable { 1 } else { 0 },
            self.axis_no + 1
        ))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // No PID-gain support in the C corvus driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();

        let position = ctrl.read_positions()?[self.axis_no];

        let status = atoi(&ctrl.query("st")?);
        let moving = (status & STATUS_MOVING) != 0;

        let mut high_limit = false;
        let mut low_limit = false;
        let mut powered = true;
        let mut problem = false;
        if !moving {
            // Switch config: bit 1 (0x2) masks (disables) the switch.
            let sw = ctrl.query(&format!("{} getsw", self.axis_no + 1))?;
            let cfg: Vec<i32> = sw
                .split_whitespace()
                .filter_map(|tok| tok.parse::<i32>().ok())
                .collect();
            let (ignore_low, ignore_high) = match cfg.as_slice() {
                [low, high, ..] => (low & SWITCH_IGNORE, high & SWITCH_IGNORE),
                _ => (0, 0),
            };

            // Switch state: "low high", 0=inactive 1=active.
            let st = ctrl.query(&format!("{} getswst", self.axis_no + 1))?;
            let state: Vec<i32> = st
                .split_whitespace()
                .filter_map(|tok| tok.parse::<i32>().ok())
                .collect();
            if let [low, high, ..] = state.as_slice() {
                low_limit = ignore_low == 0 && *low != 0;
                high_limit = ignore_high == 0 && *high != 0;
            }

            // Drive current enabled?
            let drive_on = atoi(&ctrl.query(&format!("{} getmp", self.axis_no + 1))?);
            powered = drive_on != 0;
            problem = drive_on == 0;
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
    fn status_moving_bit() {
        let moving = |s: i32| (s & STATUS_MOVING) != 0;
        assert!(moving(0x1));
        assert!(moving(0x3));
        assert!(!moving(0x0));
        assert!(!moving(0x2));
    }

    #[test]
    fn switch_ignore_mask() {
        // Bit 1 (0x2) disables the switch; bit 0 (polarity) is irrelevant here.
        let ignored = |cfg: i32| (cfg & SWITCH_IGNORE) != 0;
        assert!(!ignored(0x0));
        assert!(!ignored(0x1));
        assert!(ignored(0x2));
        assert!(ignored(0x3));
    }
}
