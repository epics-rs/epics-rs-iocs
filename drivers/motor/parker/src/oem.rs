//! Parker OEM750 series controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorParker/parkerApp/src/OEMMotorDriver.cpp` (a model-3
//! `asynMotorController`/`asynMotorAxis` pair). Commands are ASCII, prefixed by
//! the 1-based unit address (`<addr>` = axis + 1), CR-terminated (the startup
//! script sets the input EOS; the driver owns output framing). Connects over a
//! `drvAsynIPPort`.
//!
//! ## Reply model
//!
//! The controller echoes each command and then, for a query, sends a data line.
//! Command-only requests (`MN`, `A`, `V`, `D`, `G`, `S`, `H`, `MC`, `FSC`, and
//! the configuration commands) read a single acknowledgement line; queries
//! (`R`, `PR`, `W3`, `IS`) read the echo and then the data line.
//!
//! ## Units
//!
//! The controller's motor resolution (`MR`) and the velocity scale
//! (`V = velocity / pulsesPerUnit`) are the same factor applied in opposite
//! directions: the C driver sets `MR = pulsesPerUnit` and divides the commanded
//! velocity by it, so the physical velocity is independent of `pulsesPerUnit`.
//! It therefore cancels at the EGU boundary — this port uses `MR 1` and reports
//! and commands positions/velocities in controller counts (`MRES` = 1). Position
//! targets (`D`) and readback (`PR`/`W3`) are already in native counts.
//!
//! ## Positioning mode and relative moves
//!
//! `configureController` puts the axis in absolute mode (`MPA`), so `D` is always
//! an absolute target. The C `move` ignores its `relative` flag (a latent bug for
//! relative moves in absolute mode); this port instead honours it by converting a
//! relative move to an absolute target from the last polled position.
//!
//! ## Not modeled (documented)
//!
//! The `OEM_SELECT_SWITCH` aux parameter (which of the two detected switches is
//! the home/low-limit end) is not exposed; the C startup default of `0` is used,
//! so switch 1 maps to the low-limit/home end and switch 2 to the high-limit end.
//! `OEM_RESOLUTION` is subsumed by the `MR 1` choice above; `OEM_SWITCH_DETECTED`
//! is surfaced only through the limit/home status bits.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, leading_hex};

/// Response buffer size.
const READ_BUF: usize = 64;

/// Command terminator; the driver owns output framing.
const TERMINATOR: &[u8] = b"\r";

/// 2^32, subtracted to sign-extend a negative 32-bit step count (C).
const TWO_POW_32: f64 = 4_294_967_296.0;

/// Which detected switch is the home/low-limit end (C `OEM_SELECT_SWITCH`
/// startup default).
const SELECT_SWITCH: i32 = 0;

/// Shared OEM750 controller endpoint owning the asyn octet handle.
pub struct OemController {
    handle: SyncIOHandle,
}

impl OemController {
    /// Wrap a connected octet handle. The C controller constructor connects,
    /// sets the CR EOS, creates the axes and starts the poller; here the EOS is
    /// set by the startup script and axes are created by the ioc command.
    pub fn new(handle: SyncIOHandle) -> Self {
        Self { handle }
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    fn read_line(&self) -> AsynResult<String> {
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string())
    }

    /// Send a command and read its single acknowledgement line.
    fn command(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let _ack = self.read_line()?;
        Ok(())
    }

    /// Send a query and read the echo line then the data line (returned).
    fn query(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let _echo = self.read_line()?;
        self.read_line()
    }
}

/// One OEM750 axis sharing a controller. Implements [`AsynMotor`].
pub struct OemAxis {
    controller: Arc<Mutex<OemController>>,
    /// 1-based unit address used to prefix every command.
    address: i32,
    /// Set by `home`, cleared by `move` (drives the poll's auto-zero step).
    homed: bool,
    /// Persistent base position from `PR` (added to the live `W3` step).
    encoder_base: f64,
    /// Last reported position (used to resolve relative moves to absolute).
    position: f64,
    /// Last polled travel direction (persisted while stopped).
    direction: bool,
}

impl OemAxis {
    /// Construct axis `axis_no` (0-based; unit address = axis_no + 1) and run the
    /// controller configuration sequence, matching the C `OEMAxis` constructor.
    pub fn new(controller: Arc<Mutex<OemController>>, axis_no: usize) -> AsynResult<Self> {
        let axis = Self {
            controller,
            address: axis_no as i32 + 1,
            homed: false,
            encoder_base: 0.0,
            position: 0.0,
            direction: true,
        };
        axis.lock().configure(axis.address)?;
        Ok(axis)
    }

    fn lock(&self) -> MutexGuard<'_, OemController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Shared absolute move: normal mode, ramp time, velocity, target, go.
    fn do_move(
        &mut self,
        target: f64,
        velocity: f64,
        min_velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let a = ramp_time(velocity, min_velocity, acceleration);
        let addr = self.address;
        let ctrl = self.lock();
        ctrl.command(&format!("{addr}MN"))?;
        ctrl.command(&format!("{addr}A{a:.2}"))?;
        ctrl.command(&format!("{addr}V{velocity:.2}"))?;
        ctrl.command(&format!("{addr}D{target:.0}"))?;
        ctrl.command(&format!("{addr}G"))?;
        drop(ctrl);
        self.homed = false;
        Ok(())
    }
}

impl OemController {
    /// Re-apply the axis configuration (echo on, SSA0, absolute mode, MR 1). The
    /// C driver runs this at construction and at the head of every poll.
    fn configure(&self, addr: i32) -> AsynResult<()> {
        self.command(&format!("{addr}E"))?;
        self.command(&format!("{addr}SSA0"))?;
        self.command(&format!("{addr}MPA"))?;
        self.command(&format!("{addr}MR1"))
    }
}

/// Acceleration ramp time `A` = (velocity - min_velocity) / acceleration (C).
fn ramp_time(velocity: f64, min_velocity: f64, acceleration: f64) -> f64 {
    if acceleration != 0.0 {
        (velocity - min_velocity) / acceleration
    } else {
        0.0
    }
}

impl AsynMotor for OemAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, velocity, min_velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // The controller is in absolute mode (MPA); resolve the delta to an
        // absolute target from the last polled position.
        let target = self.position + distance;
        self.do_move(target, velocity, min_velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // Not implemented in the C OEM driver.
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let addr = self.address;
        let ctrl = self.lock();
        ctrl.command(&format!("{addr}MC"))?;
        ctrl.command(&format!("{addr}V{velocity:.2}"))?;
        // C maps forward -> H-, reverse -> H+.
        ctrl.command(&format!("{addr}H{}", if forward { '-' } else { '+' }))?;
        ctrl.command(&format!("{addr}G"))?;
        drop(ctrl);
        self.homed = true;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let addr = self.address;
        self.lock().command(&format!("{addr}S"))
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // The C OEM driver has no set-position support.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let addr = self.address;
        self.lock()
            .command(&format!("{addr}FSC{}", i32::from(enable)))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // Not implemented in the C OEM driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let addr = self.address;
        let ctrl = self.lock();
        ctrl.configure(addr)?;

        // Moving flags: "*B"/"*C" mean in motion.
        let flags = ctrl.query(&format!("{addr}R"))?;
        let done = flags != "*B" && flags != "*C";

        let mut new_base = self.encoder_base;
        let mut new_direction = self.direction;
        let new_position;
        if done {
            // Position readback (skip the leading status char).
            let pr = ctrl.query(&format!("{addr}PR"))?;
            new_base = atof(pr.get(1..).unwrap_or(""));
            new_position = new_base;
        } else {
            // Live step count in hex (skip the leading char); "*F" flags negative.
            let w3 = ctrl.query(&format!("{addr}W3"))?;
            let mut step = leading_hex(w3.get(1..).unwrap_or("")).unwrap_or(0) as f64;
            if w3.contains("*F") {
                step -= TWO_POW_32;
                new_direction = false;
            } else {
                new_direction = true;
            }
            new_position = new_base + step;
        }

        // Switch status (character flags at fixed offsets 6 and 7).
        let is = ctrl.query(&format!("{addr}IS"))?;
        let switch_detected = switch_from_is(&is);

        let mut high_limit = false;
        let mut low_limit = false;
        let mut home = false;
        let mut homed_status = false;
        if switch_detected > 0 {
            if SELECT_SWITCH == switch_detected - 1 {
                if self.homed {
                    ctrl.command(&format!("{addr}PZ"))?;
                    homed_status = true;
                }
                low_limit = true;
                home = true;
            } else {
                high_limit = true;
            }
        }
        drop(ctrl);

        self.encoder_base = new_base;
        self.position = new_position;
        self.direction = new_direction;

        Ok(MotorStatus {
            position: new_position,
            encoder_position: new_position,
            velocity: 0.0,
            done,
            moving: !done,
            direction: new_direction,
            high_limit,
            low_limit,
            home,
            homed: homed_status,
            ..MotorStatus::default()
        })
    }
}

/// Decode the `IS` reply into a detected-switch code: 1 if the flag at offset 6
/// is set, 2 if the flag at offset 7 is set, else 0 (C `poll`).
fn switch_from_is(is: &str) -> i32 {
    let b = is.as_bytes();
    if b.get(6) == Some(&b'1') {
        1
    } else if b.get(7) == Some(&b'1') {
        2
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ramp_time_is_delta_v_over_a() {
        assert_eq!(ramp_time(10.0, 2.0, 4.0), 2.0);
        // Zero acceleration must not divide by zero.
        assert_eq!(ramp_time(10.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn switch_decode() {
        // Offsets 6 and 7 hold the switch flags.
        assert_eq!(switch_from_is("ABCDEF10"), 1);
        assert_eq!(switch_from_is("ABCDEF01"), 2);
        assert_eq!(switch_from_is("ABCDEF00"), 0);
        // Short replies decode as no switch.
        assert_eq!(switch_from_is("AB"), 0);
    }
}
