//! Kohzu SC-200/400/800 stepper motor controller driver (serial ASCII).
//!
//! Ported from `motorKohzu/kohzuApp/src/drvSC800.cc` + `devSC800.cc`. The SC-800
//! (and its 4-axis SC-400 / 2-axis SC-200 siblings) is a stepper controller.
//! Every command is prefixed with STX (0x02) and terminated by CR+LF (the
//! driver owns this framing); the startup script configures only the input EOS.
//! The controller answers **every** command with a tab-separated `C\t…` reply,
//! so each command is written and its reply consumed to keep the stream
//! synchronized. Queries used here:
//!
//! - `STR1/<axis>`  → `C\tSTR<a>\t1\t<move>\t<norg>\t<orgg>\t<cw>\t<ccw>\t<swng>\t<err>`
//! - `RDP<axis>/0`  → `C\tRDP<a>\t<position>`
//! - `RSY<axis>/21` → `C\tRSY<a>\t21\t<torque>` (0 = torque enabled)
//!
//! Motion commands: `ASI` (set base/slew/accel), `APS`/`RPS` (absolute /
//! relative move), `ORG` (home), `FRP` (jog), `WRP` (set position), `STP`
//! (stop), `COF` (torque on/off). The wire axis number is 1-based.
//!
//! ## Units
//!
//! The controller works natively in motor steps (`RDP` returns steps, `APS`
//! takes steps) with no resolution scaling, so the asyn-rs motor boundary is
//! steps: positions pass through with `NINT` rounding, the record's `MRES` is
//! 1, and its `EGU` is steps. Base/slew speeds cross the boundary in steps/s.
//! The controller's acceleration parameter is the record's acceleration *time*
//! scaled by 100 (C `NINT(mr->accl * 100)`); at the boundary the acceleration
//! *rate* (steps/s²) is supplied instead, so the time is reconstructed as
//! `velocity / acceleration`.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `STR`/`RDP`/`RSY` reads run
//!   inside [`poll`](AsynMotor::poll).
//! - The C move path chooses backlash vs slew acceleration from record fields
//!   (`bvel`/`bacc`/`mres`) not visible at the asyn-rs boundary; this port uses
//!   the acceleration passed with the move.
//! - `HOME_FOR`/`HOME_REV` both emit the same `ORG…/3/1` sequence as in C, so
//!   the requested home direction does not change the command.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::nint;

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 160;

/// Start-of-text prefix on every command (C `STX`).
const STX: u8 = 0x02;

/// Command terminator (C `SC800_OUT_EOS`); the driver owns output framing.
const TERMINATOR: &[u8] = b"\r\n";

/// Jog speed clamp (C `devSC800.cc` JOG validity check).
const JOG_MIN: i32 = 1;
const JOG_MAX: i32 = 4095500;

fn kohzu_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle and STX/CRLF framing.
pub struct KohzuController {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
}

impl KohzuController {
    /// Connect and identify a Kohzu controller (C `motor_init`): `IDN` reports
    /// the model (SC-800/400/200), which fixes the axis count (8/4/2). Performs
    /// blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes: 0,
        };
        let reply = ctrl.command("IDN")?;
        // "C\tIDN0\t<model>\t<version>"
        let fields: Vec<&str> = reply.split('\t').collect();
        let model = fields.get(2).and_then(|s| s.trim().parse::<i32>().ok());
        let num_axes = match model {
            Some(800) => 8,
            Some(400) => 4,
            Some(200) => 2,
            _ => {
                return Err(kohzu_err(format!(
                    "SC800: unrecognized IDN reply '{reply}'"
                )));
            }
        };
        let version = fields.get(3).map(|s| s.trim()).unwrap_or("");
        ctrl.ident = format!("SC-{} Ver{}", model.unwrap(), version);
        ctrl.num_axes = num_axes;
        Ok(ctrl)
    }

    /// The identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes fixed by the controller model.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(1 + cmd.len() + TERMINATOR.len());
        out.push(STX);
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Send a command and return its reply (trimmed of framing). Every SC-800
    /// command produces a `C\t…` reply, so this is used for both queries and
    /// set commands (whose reply is discarded) to keep the stream synchronized.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text
            .trim_matches(|c: char| c == '\r' || c == '\n' || c == '\0' || c == '\u{2}')
            .to_string())
    }
}

/// The three programmable move speeds (C `base_speed`/`slew_speed`/`accl_rate`).
#[derive(Clone, Copy)]
struct Speeds {
    base: i32,
    slew: i32,
    accel: i32,
}

/// Derive the programmable speeds from move parameters. Base/slew are the
/// min/target velocities (steps/s). The controller's acceleration parameter is
/// the acceleration *time* (`velocity / acceleration`) scaled by 100, matching
/// C `NINT(mr->accl * 100)`; when it cannot be derived (non-positive velocity or
/// acceleration) `prev_accel` is kept.
fn compute_speeds(min_velocity: f64, velocity: f64, acceleration: f64, prev_accel: i32) -> Speeds {
    let accel = if velocity > 0.0 && acceleration > 0.0 {
        nint(velocity / acceleration * 100.0)
    } else {
        prev_accel
    };
    Speeds {
        base: nint(min_velocity),
        slew: nint(velocity),
        accel,
    }
}

/// One SC-800 axis sharing a controller. Implements [`AsynMotor`].
pub struct KohzuAxis {
    controller: Arc<Mutex<KohzuController>>,
    /// 1-based wire axis number.
    axis: u32,
    speeds: Speeds,
    prev_position: i32,
    last_status: MotorStatus,
}

impl KohzuAxis {
    /// Construct axis `index` (0-based; wire axis = `index + 1`).
    pub fn new(controller: Arc<Mutex<KohzuController>>, index: usize) -> Self {
        Self {
            controller,
            axis: index as u32 + 1,
            speeds: Speeds {
                base: 0,
                slew: 0,
                accel: 100,
            },
            prev_position: 0,
            last_status: MotorStatus {
                done: true,
                gain_support: false,
                ..MotorStatus::default()
            },
        }
    }

    fn lock(&self) -> MutexGuard<'_, KohzuController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Update the stored speeds from move parameters (keeping the previous
    /// acceleration parameter when it cannot be derived).
    fn set_speeds(&mut self, min_velocity: f64, velocity: f64, acceleration: f64) {
        self.speeds = compute_speeds(min_velocity, velocity, acceleration, self.speeds.accel);
    }

    /// Program the axis speeds (C `write_parms` → `ASI`).
    fn program_speeds(&self, ctrl: &KohzuController) -> AsynResult<()> {
        let s = self.speeds;
        ctrl.command(&format!(
            "ASI{}/{}/{}/{}/{}/0/0/0/0/0/0/0/0/0",
            self.axis, s.base, s.slew, s.accel, s.accel
        ))?;
        Ok(())
    }
}

impl AsynMotor for KohzuAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.set_speeds(min_velocity, velocity, acceleration);
        let ctrl = self.lock();
        self.program_speeds(&ctrl)?;
        ctrl.command(&format!("APS{}/2/0/0/{}/0/0/1", self.axis, nint(position)))?;
        Ok(())
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.set_speeds(min_velocity, velocity, acceleration);
        let ctrl = self.lock();
        self.program_speeds(&ctrl)?;
        ctrl.command(&format!("RPS{}/2/1/0/{}/0/0/1", self.axis, nint(distance)))?;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C JOG: clamp the slew speed, cap base speed to it, program, then FRP
        // with the direction polarity.
        let polarity = if velocity > 0.0 { '1' } else { '0' };
        self.set_speeds(min_velocity, velocity.abs(), acceleration);
        self.speeds.slew = self.speeds.slew.clamp(JOG_MIN, JOG_MAX);
        if self.speeds.base > self.speeds.slew {
            self.speeds.base = self.speeds.slew;
        }
        let ctrl = self.lock();
        self.program_speeds(&ctrl)?;
        ctrl.command(&format!("FRP{}/2/0/0/{}/1", self.axis, polarity))?;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C HOME_FOR/HOME_REV both emit the same ORG sequence.
        self.set_speeds(min_velocity, velocity, acceleration);
        let ctrl = self.lock();
        self.program_speeds(&ctrl)?;
        ctrl.command(&format!("ORG{}/2/0/0/3/1", self.axis))?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.command(&format!("STP{}/0", self.axis))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.command(&format!("WRP{}/{}", self.axis, nint(position)))?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // COF/0 enables torque, COF/1 disables it.
        let ctrl = self.lock();
        ctrl.command(&format!("COF{}/{}", self.axis, i32::from(!enable)))?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let status_reply = ctrl.command(&format!("STR1/{}", self.axis));
        let pos_reply = ctrl.command(&format!("RDP{}/0", self.axis));
        let torque_reply = ctrl.command(&format!("RSY{}/21", self.axis));
        drop(ctrl);

        // Status: "C\tSTR<a>\t1\t<move>\t<norg>\t<orgg>\t<cw>\t<ccw>\t<swng>\t<err>"
        let status_fields: Option<Vec<i32>> = status_reply.ok().and_then(|r| {
            let f: Vec<&str> = r.split('\t').collect();
            if f.len() >= 10 && f[0] == "C" {
                Some(
                    (3..=9)
                        .map(|i| f[i].trim().parse::<i32>().unwrap_or(0))
                        .collect(),
                )
            } else {
                None
            }
        });

        let Some(sf) = status_fields else {
            // C: first failure after NORMAL is a silent RETRY; a repeat is a
            // hard comms error. Keep the last position, flag the error.
            self.last_status = MotorStatus {
                comms_error: true,
                problem: true,
                ..self.last_status.clone()
            };
            return Ok(self.last_status.clone());
        };
        let str_move = sf[0];
        let plus_ls = sf[3] == 1; // cw limit
        let minus_ls = sf[4] == 1; // ccw limit
        let done = str_move == 0;

        // Position: "C\tRDP<a>\t<position>"
        let position = pos_reply
            .ok()
            .and_then(|r| {
                let f: Vec<&str> = r.split('\t').collect();
                f.get(2).and_then(|s| s.trim().parse::<i32>().ok())
            })
            .unwrap_or(self.prev_position);
        let direction = if position != self.prev_position {
            position >= self.prev_position
        } else {
            self.last_status.direction
        };
        self.prev_position = position;

        // Torque: "C\tRSY<a>\t21\t<torque>" (0 = enabled).
        let powered = torque_reply
            .ok()
            .and_then(|r| {
                let f: Vec<&str> = r.split('\t').collect();
                f.get(3).and_then(|s| s.trim().parse::<i32>().ok())
            })
            .map(|v| v == 0)
            .unwrap_or(false);

        self.last_status = MotorStatus {
            position: position as f64,
            encoder_position: position as f64,
            velocity: 0.0,
            direction,
            done,
            moving: !done,
            high_limit: plus_ls,
            low_limit: minus_ls,
            home: false,
            powered,
            comms_error: false,
            problem: false,
            gain_support: false,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framed_wraps_stx_and_crlf() {
        let f = KohzuController::framed("APS1/2/0/0/500/0/0/1");
        assert_eq!(f[0], STX);
        assert_eq!(&f[f.len() - 2..], b"\r\n");
        assert_eq!(&f[1..f.len() - 2], b"APS1/2/0/0/500/0/0/1");
    }

    #[test]
    fn speeds_reconstruct_accel_time() {
        // velocity 1000 steps/s, acceleration 5000 steps/s² → accel time 0.2 s
        // → accel param NINT(0.2 * 100) = 20.
        let s = compute_speeds(100.0, 1000.0, 5000.0, 100);
        assert_eq!(s.slew, 1000);
        assert_eq!(s.base, 100);
        assert_eq!(s.accel, 20);
        // Non-positive acceleration keeps the previous accel parameter.
        let s = compute_speeds(100.0, 1000.0, 0.0, 42);
        assert_eq!(s.accel, 42);
    }
}
