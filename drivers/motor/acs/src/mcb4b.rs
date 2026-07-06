//! ACS MCB-4B stepper motor controller driver (serial ASCII).
//!
//! Ported from `motorAcs/acsApp/src/MCB4BDriver.cpp` (the asyn **model-3**
//! `asynMotorController` / `asynMotorAxis` driver — not the older model-1
//! `drvMCB4B.cc`/`devMCB4B.cc` pair, which registers `MCB4BSetup`/`MCB4BConfig`
//! instead). One serial line drives up to four axes; every command is prefixed
//! with the 2-digit, **0-based** axis number (`#%02d…`).
//!
//! ## Framing — the port owns it
//!
//! The C `sprintf(pC_->outString_, …)` calls never embed a terminator, and the
//! reference startup script `ACS_MCB4B.iocsh` sets BOTH the input and the
//! output EOS to `\r`:
//!
//! ```text
//! asynOctetSetInputEos( "$(PORT)", -1, "\r")
//! asynOctetSetOutputEos("$(PORT)", -1, "\r")
//! ```
//!
//! So this port sends **bare** command bytes via [`SyncIOHandle::write_octet`]
//! and lets the asyn-rs `EosInterpose` layer append the configured `\r`; the
//! startup script must set both EOS. The driver must NOT append `\r` itself —
//! that would double it on the wire. (Contrast `motor-mclennan`, where the C
//! driver owns framing and appends the terminator.)
//!
//! ## Every command answers
//!
//! The MCB-4B replies to every command (C `writeReadController()` writes
//! `outString_` and reads `inString_`), so each command — query or set — is
//! written and its reply consumed to keep the stream synchronized. A comms
//! failure aborts the remaining reads of a [`poll`](AsynMotor::poll) (C
//! `if (comStatus) goto skip;`), which here is the `?` early return.
//!
//! ## Units
//!
//! The controller works natively in motor steps: `#…P` returns a signed step
//! count, `#…G`/`#…I` take signed step counts, so the asyn-rs motor boundary is
//! steps with `MRES` = 1 and `EGU` = steps. Velocity is programmed as a divisor
//! (`V = 115200/velocity`, clamped 2..=255) and acceleration as a ramp index
//! (`R = 256 - 720000/accel`, clamped 1..=255), exactly as the C computes.
//!
//! ## Fields the C poll sets (and only these)
//!
//! The model-3 `MCB4BAxis::poll` calls `setDoubleParam`/`setIntegerParam` for
//! exactly: `motorPosition_`, `motorStatusDone_`, `motorStatusHighLimit_`,
//! `motorStatusLowLimit_`, `motorStatusAtHome_`, `motorStatusPowerOn_`, and
//! `motorStatusProblem_`. It never sets direction, encoder position, gain
//! support, or the moving-velocity readback, so those are left at their
//! [`MotorStatus`] defaults here (documented deviations, matching C).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, nint};

/// Response buffer size (generous; the MCB-4B replies are short, e.g. `#01P=+1000`).
const READ_BUF: usize = 200;

/// Shared controller endpoint: owns the serial handle and the axis count.
///
/// The C model-3 constructor only connects the octet port and starts the
/// poller — it performs no identification I/O — so construction here is a plain
/// move of the already-connected handle.
pub struct Mcb4bController {
    handle: SyncIOHandle,
    n_axes: usize,
}

impl Mcb4bController {
    /// Wrap an already-connected serial handle. `n_axes` is the configured axis
    /// count (C loops `axis = 0..numAxes`); no probe is performed, matching the
    /// C constructor.
    pub fn new(handle: SyncIOHandle, n_axes: usize) -> Self {
        Self { handle, n_axes }
    }

    /// Number of axes (as configured).
    pub fn num_axes(&self) -> usize {
        self.n_axes
    }

    /// C `writeReadController()`: write the bare command (the port appends the
    /// output EOS) and read the reply (the port has already stripped the input
    /// EOS). Every MCB-4B command produces a reply, so this is used for both
    /// queries and set commands (whose reply is discarded) to keep the stream
    /// synchronized.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_end_matches(['\r', '\n', '\0']).to_string())
    }
}

/// One MCB-4B axis sharing a controller. Implements [`AsynMotor`].
pub struct Mcb4bAxis {
    controller: Arc<Mutex<Mcb4bController>>,
    /// 0-based wire axis number, formatted `#%02d` (C `axisNo_`).
    axis: u32,
}

impl Mcb4bAxis {
    /// Construct axis `index` (0-based; wire prefix `#%02d` = `index`).
    pub fn new(controller: Arc<Mutex<Mcb4bController>>, index: usize) -> Self {
        Self {
            controller,
            axis: index as u32,
        }
    }

    fn lock(&self) -> MutexGuard<'_, Mcb4bController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `MCB4BAxis::sendAccelAndVelocity`: program velocity then acceleration
    /// ahead of a move.
    ///
    /// - Velocity register `V = NINT(|115200/velocity|)`, clamped to `2..=255`.
    /// - Acceleration ramp index `R = NINT(256 - 720000/accel)`, clamped to
    ///   `1..=255` (the MCB step rate is `720000/(256-R)` steps/s²).
    ///
    /// A zero `velocity`/`acceleration` yields a non-finite intermediate; the
    /// `as i32` cast saturates and the clamp then pins it into range, matching
    /// the C clamps that guard the same division.
    fn send_accel_and_velocity(
        ctrl: &Mcb4bController,
        axis: u32,
        acceleration: f64,
        velocity: f64,
    ) -> AsynResult<()> {
        let ival = nint((115200.0 / velocity).abs()).clamp(2, 255);
        ctrl.command(&format!("#{axis:02}V={ival}"))?;

        let ival = nint(256.0 - (720000.0 / acceleration)).clamp(1, 255);
        ctrl.command(&format!("#{axis:02}R={ival}"))?;
        Ok(())
    }
}

impl AsynMotor for Mcb4bAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        Mcb4bAxis::send_accel_and_velocity(&ctrl, self.axis, acceleration, velocity)?;
        // C absolute move: "#%02dG%+d".
        ctrl.command(&format!("#{:02}G{:+}", self.axis, nint(position)))?;
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
        let ctrl = self.lock();
        Mcb4bAxis::send_accel_and_velocity(&ctrl, self.axis, acceleration, velocity)?;
        // C relative move: "#%02dI%+d".
        ctrl.command(&format!("#{:02}I{:+}", self.axis, nint(distance)))?;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        Mcb4bAxis::send_accel_and_velocity(&ctrl, self.axis, acceleration, velocity)?;
        // The MCB-4B has no jog command; C moves a fixed 1,000,000 steps in the
        // requested direction ("#%02dI+1000000" / "#%02dI-1000000").
        let cmd = if velocity > 0.0 {
            format!("#{:02}I+1000000", self.axis)
        } else {
            format!("#{:02}I-1000000", self.axis)
        };
        ctrl.command(&cmd)?;
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
        let ctrl = self.lock();
        Mcb4bAxis::send_accel_and_velocity(&ctrl, self.axis, acceleration, velocity)?;
        // C home: "#%02dH+" (forward) / "#%02dH-" (reverse).
        let cmd = if forward {
            format!("#{:02}H+", self.axis)
        } else {
            format!("#{:02}H-", self.axis)
        };
        ctrl.command(&cmd)?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        // C stop: "#%02dQ".
        ctrl.command(&format!("#{:02}Q", self.axis))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        // C set position: "#%02dP=%+d".
        ctrl.command(&format!("#{:02}P={:+}", self.axis, nint(position)))?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        // C set closed loop (drive power): "#%02dW=%d" with 1 or 0.
        ctrl.command(&format!(
            "#{:02}W={}",
            self.axis,
            if enable { 1 } else { 0 }
        ))?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();

        // Position — reply "#01P=+1000"; C parses atof(&inString_[5]).
        let reply = ctrl.command(&format!("#{:02}P", self.axis))?;
        let position = reply.get(5..).map(atof).unwrap_or(0.0);

        // Moving status — reply "#01X=1"; C: done = (inString_[5]=='0').
        let reply = ctrl.command(&format!("#{:02}X", self.axis))?;
        let done = byte_at(&reply, 5) == b'0';

        // Limit / home status — reply "#01E=..."; C reads indices 5/6/7.
        let reply = ctrl.command(&format!("#{:02}E", self.axis))?;
        let high_limit = byte_at(&reply, 5) == b'1';
        let low_limit = byte_at(&reply, 6) == b'1';
        let at_home = byte_at(&reply, 7) == b'1';

        // Drive power-on status — reply "#01W=1"; C: driveOn = (inString_[5]=='1').
        let reply = ctrl.command(&format!("#{:02}W", self.axis))?;
        let powered = byte_at(&reply, 5) == b'1';
        drop(ctrl);

        Ok(MotorStatus {
            position,
            done,
            moving: !done,
            high_limit,
            low_limit,
            // C sets motorStatusAtHome_ (RA_HOME) from the E reply's index 7.
            home: at_home,
            powered,
            // All reads succeeded here; C sets motorStatusProblem_ = 0 and only
            // flips it to 1 when a read fails (which is the `?` early return).
            problem: false,
            ..MotorStatus::default()
        })
    }
}

/// Byte at index `i` of a reply, or `b'0'` if the reply is too short. The C
/// driver indexes `inString_[i]` unconditionally; guarding avoids reading past
/// a short/garbled reply while preserving the C "not '1'/'0'" outcome for that
/// position.
fn byte_at(reply: &str, i: usize) -> u8 {
    reply.as_bytes().get(i).copied().unwrap_or(b'0')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn position_parses_from_index_five() {
        // "#01P=+1000" -> "+1000" from index 5.
        let reply = "#01P=+1000";
        assert_eq!(reply.get(5..).map(atof).unwrap_or(0.0), 1000.0);
        let reply = "#03P=-512";
        assert_eq!(reply.get(5..).map(atof).unwrap_or(0.0), -512.0);
    }

    #[test]
    fn moving_status_done_when_index_five_zero() {
        assert_eq!(byte_at("#01X=0", 5), b'0'); // done
        assert_eq!(byte_at("#01X=1", 5), b'1'); // moving
    }

    #[test]
    fn limit_and_home_indices() {
        // "#01E=<high><low><home>" at indices 5/6/7.
        let reply = "#01E=101";
        assert_eq!(byte_at(reply, 5), b'1'); // high limit
        assert_eq!(byte_at(reply, 6), b'0'); // low limit
        assert_eq!(byte_at(reply, 7), b'1'); // at home
    }

    #[test]
    fn power_status_index_five() {
        assert_eq!(byte_at("#01W=1", 5), b'1'); // powered
        assert_eq!(byte_at("#01W=0", 5), b'0'); // off
    }

    #[test]
    fn short_reply_defaults_to_zero_byte() {
        assert_eq!(byte_at("#01", 5), b'0');
        assert_eq!("#01".get(5..).map(atof).unwrap_or(0.0), 0.0);
    }

    #[test]
    fn velocity_register_clamps() {
        // V = NINT(|115200/velocity|), clamped 2..=255.
        assert_eq!(nint((115200.0f64 / 1000.0).abs()).clamp(2, 255), 115); // 115.2 -> 115
        assert_eq!(nint((115200.0f64 / 10.0).abs()).clamp(2, 255), 255); // 11520 -> 255
        assert_eq!(nint((115200.0f64 / 100000.0).abs()).clamp(2, 255), 2); // ~1.15 -> 2 (floor of clamp)
    }

    #[test]
    fn accel_ramp_index_clamps() {
        // R = NINT(256 - 720000/accel), clamped 1..=255.
        assert_eq!(nint(256.0 - 720000.0 / 100000.0).clamp(1, 255), 249); // 256-7.2 -> 249
        assert_eq!(nint(256.0 - 720000.0 / 1000.0).clamp(1, 255), 1); // very negative -> 1
        assert_eq!(nint(256.0 - 720000.0 / 720000.0).clamp(1, 255), 255); // 255 -> 255
    }
}
