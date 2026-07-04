//! attocube ANC150 piezo stepper controller driver (serial ASCII).
//!
//! Ported from `motorAttocube/attocubeApp/src/drvANC150Asyn.cc`. The ANC150 is
//! an **open-loop** stepper: it reports no position, so the driver tracks
//! position in software (steps) and estimates motion completion from a timer —
//! `moveinterval = |steps| / frequency` seconds, where `frequency` is the
//! axis's step rate read back with `getf`. Commands are `stepu`/`stepd`
//! (up/down by a step count); every command line is `\r\n`-terminated and the
//! controller answers with an echo, a reply line, and an acknowledgement,
//! framed by a `"> "` prompt (input EOS).
//!
//! ## Units
//!
//! Positions are wire steps. The asyn-rs motor boundary is dial-frame EGU, so
//! for this step-native controller EGU ≡ steps: positions pass through with
//! `NINT` rounding (C `NINT`) and the record's `MRES` is 1. The record's
//! velocity/acceleration have no effect — the ANC150 has no velocity command;
//! its step rate (`frequency`) is a controller setting, read but never written
//! from the record.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller runs as a background thread; here the interpolation and the
//!   `getf`/`getm` status reads run inside [`poll`](AsynMotor::poll), whose
//!   cadence supplies the elapsed time from a monotonic clock.
//! - C never stores the high/low limits (`motorAxisSetDouble` breaks on
//!   `motorAxisHighLimit`/`LowLimit`), so its jog targets 0; this port keeps
//!   the same non-storing behaviour (jog targets the unset limit) — see
//!   [`Anc150Axis::high_limit`].
//! - When `frequency` is unknown (0), C divides by it (move never completes);
//!   this port falls back to a one-quantum move interval so the move finishes.
//! - The ANC150 reports no home/limit switches (C `axisStatus` is never set),
//!   so those status bits are always clear.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::nint;

/// Response buffer size (C `BUFFER_SIZE` 100, padded for the multi-line reply).
const READ_BUF: usize = 256;

/// Command terminator (C `ANC150_OUT_EOS`).
const TERMINATOR: &[u8] = b"\r\n";

/// Fallback move interval when the step frequency is unknown
/// (C `epicsThreadSleepQuantum()`, ~1 clock tick).
const MOVE_QUANTUM: f64 = 0.02;

fn anc_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle and command framing.
/// The caller holds the `Arc<Mutex<_>>` lock across a write→read exchange.
pub struct Anc150Controller {
    handle: SyncIOHandle,
    ident: String,
}

impl Anc150Controller {
    /// Connect and identify an ANC150 (C `ANC150AsynConfig`): probe `ver` up to
    /// three times and require an `attocube …` reply. Performs blocking serial
    /// I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
        };
        for _ in 0..3 {
            if let Ok(reply) = ctrl.query("ver")
                && reply.starts_with("attocube")
            {
                ctrl.ident = reply;
                return Ok(ctrl);
            }
        }
        Err(anc_err("ANC150: no 'attocube' identification from 'ver'"))
    }

    /// The `ver` identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Read one prompt-framed response blob (echo, reply, ack, ending at the
    /// `"> "` input EOS).
    fn read_blob(&self) -> AsynResult<String> {
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw).into_owned())
    }

    /// Send a command and discard the reply (C `sendOnly`, which still reads to
    /// keep the stream synchronized).
    fn send_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let _ = self.read_blob()?;
        Ok(())
    }

    /// Send a query and return its reply line (C `sendAndReceive`: the reply is
    /// the second `\r\n`-separated segment, between the echo and the ack).
    fn query(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let blob = self.read_blob()?;
        let mut parts = blob.split("\r\n");
        let _echo = parts.next();
        match parts.next() {
            Some(reply) => Ok(reply.to_string()),
            None => Err(anc_err("ANC150: malformed reply (no reply line)")),
        }
    }
}

/// Compute the move plan (C `motorAxisMove`): the step count magnitude, its
/// direction (`true` = up/positive), and the axis's new target position.
fn plan_move(current: f64, arg: f64, relative: bool) -> (i64, bool, f64) {
    if relative {
        let imove = i64::from(nint(arg));
        (imove.abs(), arg >= 0.0, current + arg)
    } else {
        let imove = i64::from(nint(arg - current));
        (imove.abs(), imove >= 0, arg)
    }
}

/// Interpolate the reported position during a timed move (C poller): once the
/// move timer has elapsed the axis is at the target and done; otherwise the
/// position is linearly interpolated by the fraction of `move_interval`
/// elapsed. Returns `(position, done)`.
fn slew(current: f64, target: f64, time_remaining: f64, move_interval: f64) -> (f64, bool) {
    if time_remaining < 0.0 {
        (target, true)
    } else {
        let proportion = 1.0 - (time_remaining / move_interval);
        (current + (target - current) * proportion, false)
    }
}

/// One ANC150 axis sharing a controller. Implements [`AsynMotor`].
pub struct Anc150Axis {
    controller: Arc<Mutex<Anc150Controller>>,
    /// 0-based axis index; the wire axis number is `axis + 1`.
    axis: usize,
    current_position: f64,
    target_position: f64,
    /// Jog target limits — never updated (C ignores limit sets); default 0.
    high_limit: f64,
    low_limit: f64,
    /// Step rate (Hz) from `getf`, used to time moves.
    frequency: i64,
    moving: bool,
    /// Wall-clock deadline of the in-flight move, and its planned duration.
    move_deadline: Option<Instant>,
    move_interval: f64,
    last_status: MotorStatus,
}

impl Anc150Axis {
    /// Construct axis `axis` (0-based): read its step frequency (`getf`) and put
    /// it in stepping mode (`setm N stp`). Performs blocking serial I/O.
    pub fn new(controller: Arc<Mutex<Anc150Controller>>, axis: usize) -> AsynResult<Self> {
        let mut frequency = 0;
        {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            if let Ok(reply) = ctrl.query(&format!("getf {}", axis + 1)) {
                frequency = parse_frequency(&reply).unwrap_or(0);
            }
            ctrl.send_only(&format!("setm {} stp", axis + 1))?;
        }
        Ok(Self {
            controller,
            axis,
            current_position: 0.0,
            target_position: 0.0,
            high_limit: 0.0,
            low_limit: 0.0,
            frequency,
            moving: false,
            move_deadline: None,
            move_interval: 0.0,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                ..MotorStatus::default()
            },
        })
    }

    fn lock(&self) -> MutexGuard<'_, Anc150Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// The jog limit fields (kept for parity with C's non-storing behaviour).
    pub fn high_limit(&self) -> f64 {
        self.high_limit
    }

    /// Issue a move to `arg` (absolute) or by `arg` (relative), tracking the
    /// software position and starting the completion timer (C `motorAxisMove`).
    fn do_move(&mut self, arg: f64, relative: bool) -> AsynResult<()> {
        let (imove, posdir, new_target) = plan_move(self.current_position, arg, relative);
        self.target_position = new_target;

        let move_command = if posdir { "stepu" } else { "stepd" };
        self.move_interval = if self.frequency > 0 {
            imove as f64 / self.frequency as f64
        } else {
            MOVE_QUANTUM
        };
        if self.move_interval <= 0.0 {
            self.move_interval = MOVE_QUANTUM;
        }
        self.moving = true;
        self.move_deadline = Some(Instant::now() + Duration::from_secs_f64(self.move_interval));

        let ctrl = self.lock();
        ctrl.send_only(&format!("{} {} {}", move_command, self.axis + 1, imove))?;
        Ok(())
    }
}

/// Parse a `getf` reply `frequency = %d` into the step rate.
fn parse_frequency(reply: &str) -> Option<i64> {
    reply.strip_prefix("frequency = ")?.trim().parse().ok()
}

impl AsynMotor for Anc150Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C: the ANC150 has no jog; move to a soft limit instead. The limit
        // fields are never set (module Deviations), so this targets 0.
        let target = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        };
        self.do_move(target, false)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `motorAxisHome` returns an error — the ANC150 has no home.
        Err(anc_err("ANC150: home not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.send_only(&format!("stop {}", self.axis + 1))?;
        drop(ctrl);
        // Reset the timer so the next poll reports done.
        self.moving = false;
        self.move_deadline = None;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `motorAxisSetDouble(motorAxisPosition)`: redefine the software
        // position (no hardware command).
        self.current_position = position;
        self.target_position = position;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        let mode = if enable { "stp" } else { "gnd" };
        ctrl.send_only(&format!("setm {} {}", self.axis + 1, mode))?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Interpolate the software position (C poller).
        let (position, done) = if self.moving {
            let time_remaining = self
                .move_deadline
                .map(|d| {
                    let now = Instant::now();
                    if now >= d {
                        -1.0
                    } else {
                        (d - now).as_secs_f64()
                    }
                })
                .unwrap_or(-1.0);
            let (pos, finished) = slew(
                self.current_position,
                self.target_position,
                time_remaining,
                self.move_interval,
            );
            if finished {
                self.moving = false;
                self.move_deadline = None;
                self.current_position = self.target_position;
            }
            (pos, finished)
        } else {
            self.current_position = self.target_position;
            (self.current_position, true)
        };

        // Update the step frequency and power state. Read into locals first so
        // the controller lock is released before mutating `self.frequency`.
        let axis1 = self.axis + 1;
        let ctrl = self.lock();
        let freq_reply = ctrl.query(&format!("getf {axis1}"));
        let mode_reply = ctrl.query(&format!("getm {axis1}"));
        drop(ctrl);

        let freq_ok = match freq_reply {
            Ok(reply) => {
                if let Some(f) = parse_frequency(&reply) {
                    self.frequency = f;
                    true
                } else {
                    reply.starts_with("Axis not in computer control mode")
                }
            }
            Err(_) => false,
        };
        let powered = matches!(mode_reply, Ok(reply) if reply.starts_with("mode = stp"));

        self.last_status = MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0,
            done,
            moving: !done,
            // ANC150 reports no home/limit switches (module Deviations).
            high_limit: false,
            low_limit: false,
            home: false,
            powered,
            comms_error: !freq_ok,
            problem: !freq_ok,
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
    fn plan_move_absolute_direction_and_target() {
        // Absolute move forward from 10 to 25: 15 steps up.
        let (imove, up, target) = plan_move(10.0, 25.0, false);
        assert_eq!(imove, 15);
        assert!(up);
        assert_eq!(target, 25.0);
        // Absolute move backward from 10 to 4: 6 steps down.
        let (imove, up, target) = plan_move(10.0, 4.0, false);
        assert_eq!(imove, 6);
        assert!(!up);
        assert_eq!(target, 4.0);
    }

    #[test]
    fn plan_move_relative_accumulates_and_rounds() {
        // Relative +3.4 rounds (NINT) to 3 steps up; target accumulates.
        let (imove, up, target) = plan_move(10.0, 3.4, true);
        assert_eq!(imove, 3);
        assert!(up);
        assert_eq!(target, 13.4);
        // Relative -2.6 rounds to 3 steps down.
        let (imove, up, target) = plan_move(10.0, -2.6, true);
        assert_eq!(imove, 3);
        assert!(!up);
        assert!((target - 7.4).abs() < 1e-9);
    }

    #[test]
    fn slew_interpolates_then_completes() {
        // Halfway through a 2 s move from 0 to 10: position 5, not done.
        let (pos, done) = slew(0.0, 10.0, 1.0, 2.0);
        assert!((pos - 5.0).abs() < 1e-9);
        assert!(!done);
        // Timer elapsed: at target, done.
        let (pos, done) = slew(0.0, 10.0, -0.1, 2.0);
        assert_eq!(pos, 10.0);
        assert!(done);
    }

    #[test]
    fn parse_frequency_reads_getf_reply() {
        assert_eq!(parse_frequency("frequency = 400"), Some(400));
        assert_eq!(parse_frequency("frequency = 1000 "), Some(1000));
        assert_eq!(parse_frequency("mode = stp"), None);
    }
}
