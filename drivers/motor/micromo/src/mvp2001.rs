//! MicroMo MVP 2001 motion controller driver (serial ASCII).
//!
//! Ported from `motorMicroMo/microMoApp/src/MVP2001Driver.cpp` (the model-3
//! `asynMotorController`/`asynMotorAxis` form; the older `drvMVP2001.cc` model-1
//! pair is not ported). Commands are ASCII `<axis> <CMD> <args>` where `<axis>`
//! is 1-based; commands are CR-terminated (the driver owns output framing, the
//! startup script sets the input EOS).
//!
//! ## Prepended-terminator responses
//!
//! Commands that return data prepend a terminator to the reply. The C
//! `writeRead2xController` handles this by reading twice with a short delay: the
//! first read consumes the prepended terminator, then after ~33 ms the real
//! reply is read. This port replicates that with [`Mvp2001Controller::query`].
//! Replies have the form `"0001 000001F4"`; the value is the hex field starting
//! at byte offset 5 (C `parseReply` copies from `inString[5]` and `sscanf`s
//! `%x`).
//!
//! ## Units
//!
//! Positions are the controller's native encoder counts: `move` sends
//! `NINT(position)` directly (the header's `stepsPerRev_` is declared but never
//! used in the C source), and `POS` reads counts back as a hex value. Positions
//! cross the asyn-rs motor boundary (dial-frame EGU) unscaled: `MRES` is 1,
//! `EGU` is counts. Velocity and acceleration are converted to controller
//! `SP`/`AC` units using the loop sample period and encoder lines-per-rev,
//! matching C.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `POS`/`ST` reads run inside
//!   [`poll`](AsynMotor::poll).
//! - Homing is unimplemented in C (returns success without acting); [`home`] is
//!   likewise a no-op.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::nint;

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 32;

/// Command terminator; the driver owns output framing.
const TERMINATOR: &[u8] = b"\r";

/// Delay between the prepended-terminator read and the real reply (C 0.033 s).
const REPLY_DELAY: Duration = Duration::from_millis(33);

/// Default loop sample period in µs if the `SR` query fails (C fallback 500).
const DEFAULT_SAMPLE_PERIOD: i32 = 500;

fn micromo_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Parse a hex value from a reply of the form `"0001 000001F4"`: the field
/// starts at byte offset 5 and is up to `nchars` long, parsed as `%x` (C
/// `parseReply`). Values wrap through `u32` so negative positions decode
/// correctly.
fn parse_hex_reply(reply: &str, nchars: usize) -> Option<i32> {
    let tail = reply.get(5..)?;
    let end = nchars.min(tail.len());
    let field = tail[..end].trim();
    u32::from_str_radix(field, 16).ok().map(|v| v as i32)
}

/// Max-current command value (C `NINT(maxCurrent * 0.865909 + 2103.431)`).
fn ano_param(max_current_ma: i32) -> i32 {
    nint(max_current_ma as f64 * 0.865909 + 2103.431)
}

/// Velocity command value (C `NINT(samplePeriod * 6e-5 * velocity)`).
fn speed_param(sample_period: i32, velocity: f64) -> i32 {
    nint(sample_period as f64 * 6e-5 * velocity)
}

/// Acceleration command value, clamped to `[1, speed]` (C `AC` logic):
/// `NINT(7.5e-12 * samplePeriod^2 * encoderLinesPerRev * accel)`, then bounded
/// below by 1 and above by the speed parameter.
fn accel_param(sample_period: i32, enc_lines_per_rev: i32, accel: f64, speed: i32) -> i32 {
    let sp = sample_period as f64;
    let mut ac = nint(7.5e-12 * sp * sp * enc_lines_per_rev as f64 * accel);
    if ac < speed {
        if ac <= 0 {
            ac = 1;
        }
    } else {
        ac = speed;
    }
    ac
}

/// Shared controller endpoint owning the serial handle.
pub struct Mvp2001Controller {
    handle: SyncIOHandle,
    num_axes: usize,
}

impl Mvp2001Controller {
    /// Connect to an MVP 2001 controller. Axes are created separately (C
    /// `MVP2001CreateAxis`), so this performs no per-axis I/O.
    pub fn new(handle: SyncIOHandle, num_axes: usize) -> Self {
        Self { handle, num_axes }
    }

    /// Number of axes configured for this controller.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command with no reply expected (C `writeController`).
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Write a data-returning command and read its reply, consuming the
    /// prepended terminator first (C `writeRead2xController`).
    fn query(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        // First read: the prepended terminator (discarded).
        let _ = self.handle.read_octet(0, READ_BUF)?;
        sleep(REPLY_DELAY);
        // Second read: the real reply.
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0'])
            .to_string())
    }
}

/// One MVP 2001 axis sharing a controller. Implements [`AsynMotor`].
pub struct Mvp2001Axis {
    controller: Arc<Mutex<Mvp2001Controller>>,
    /// 1-based wire axis index.
    axis_index: i32,
    encoder_lines_per_rev: i32,
    max_current_ma: i32,
    sample_period: i32,
}

impl Mvp2001Axis {
    /// Construct and initialize axis `axis_no` (0-based). Sets the limit/estop
    /// polarity (`LP`), zeroes the position (`HO`), and reads the loop sample
    /// period (`SR`), matching the C `MVP2001Axis` constructor. Performs
    /// blocking serial I/O.
    ///
    /// `max_current_ma` is clamped to `[100, 2300]` (C accepts 0.1–2.3 A);
    /// `limit_polarity` is normalized to 0 (NC) or 1 (NO).
    pub fn new(
        controller: Arc<Mutex<Mvp2001Controller>>,
        axis_no: usize,
        encoder_lines_per_rev: i32,
        max_current_ma: i32,
        limit_polarity: i32,
    ) -> AsynResult<Self> {
        let axis_index = axis_no as i32 + 1;
        let max_current_ma = if !(100..=2300).contains(&max_current_ma) {
            100
        } else {
            max_current_ma
        };
        let limit_polarity = if limit_polarity != 0 { 1 } else { 0 };

        let sample_period = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // Set limit/estop polarity, then home (zeroes position).
            ctrl.write_only(&format!("{axis_index} LP {limit_polarity}"))?;
            ctrl.write_only(&format!("{axis_index} HO"))?;
            // Query loop sample period (4 hex chars); fall back to the default.
            ctrl.query(&format!("{axis_index} SR"))
                .ok()
                .and_then(|r| parse_hex_reply(&r, 4))
                .unwrap_or(DEFAULT_SAMPLE_PERIOD)
        };

        Ok(Self {
            controller,
            axis_index,
            encoder_lines_per_rev,
            max_current_ma,
            sample_period,
        })
    }

    fn lock(&self) -> MutexGuard<'_, Mvp2001Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Send the max-current, velocity and acceleration commands (C
    /// `sendAccelAndVelocity`). Returns the computed speed parameter for reuse.
    fn send_accel_and_velocity(
        &self,
        ctrl: &Mvp2001Controller,
        accel: f64,
        velocity: f64,
    ) -> AsynResult<i32> {
        let n = self.axis_index;
        ctrl.write_only(&format!("{n} ANO {}", ano_param(self.max_current_ma)))?;
        let sp = speed_param(self.sample_period, velocity);
        ctrl.write_only(&format!("{n} SP {sp}"))?;
        let ac = accel_param(self.sample_period, self.encoder_lines_per_rev, accel, sp);
        ctrl.write_only(&format!("{n} AC {ac}"))?;
        Ok(sp)
    }
}

impl AsynMotor for Mvp2001Axis {
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
        self.send_accel_and_velocity(&ctrl, acceleration, velocity)?;
        ctrl.write_only(&format!("{n} LA {}", nint(position)))?;
        ctrl.write_only(&format!("{n} M"))
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
        self.send_accel_and_velocity(&ctrl, acceleration, velocity)?;
        ctrl.write_only(&format!("{n} LR {}", nint(distance)))?;
        ctrl.write_only(&format!("{n} M"))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        self.send_accel_and_velocity(&ctrl, acceleration, velocity)?;
        let sp = speed_param(self.sample_period, velocity);
        ctrl.write_only(&format!("{n} V {sp}"))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // Homing is unimplemented in the C driver (returns success).
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        ctrl.write_only(&format!("{n} AB"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        ctrl.write_only(&format!("{n} HO {}", nint(position)))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        if enable {
            // An AB is needed before EN (EN fails if status ends in 8, not E).
            ctrl.write_only(&format!("{n} AB"))?;
            drop(ctrl);
            sleep(REPLY_DELAY);
            let ctrl = self.lock();
            ctrl.write_only(&format!("{n} EN"))
        } else {
            ctrl.write_only(&format!("{n} DI"))
        }
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let n = self.axis_index;
        // C gain-to-register conversions.
        let cmd = match kind {
            PidGainKind::Proportional => format!("{n} POR {}", nint(gain * 28000.0 + 4000.0)),
            PidGainKind::Integral => format!("{n} I {}", nint(gain * 31999.0 + 1.0)),
            PidGainKind::Derivative => format!("{n} DER {}", nint(gain * 31000.0 + 1000.0)),
        };
        let ctrl = self.lock();
        ctrl.write_only(&cmd)
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let n = self.axis_index;
        let ctrl = self.lock();

        // Position (8 hex chars, signed via u32 wrap).
        let pos_reply = ctrl.query(&format!("{n} POS"))?;
        let position = parse_hex_reply(&pos_reply, 8)
            .ok_or_else(|| micromo_err(format!("MVP2001: unparseable POS reply {pos_reply:?}")))?;

        // Status word (4 hex chars).
        let status_reply = ctrl.query(&format!("{n} ST"))?;
        drop(ctrl);
        let status = parse_hex_reply(&status_reply, 4).ok_or_else(|| {
            micromo_err(format!("MVP2001: unparseable ST reply {status_reply:?}"))
        })?;

        // Bit 0 set = moving; 0x2000 = high limit; 0x8000 = low limit;
        // 0x100 set = power off. No home bit on this controller.
        let done = (status & 0x1) == 0;
        Ok(MotorStatus {
            position: position as f64,
            encoder_position: position as f64,
            velocity: 0.0,
            done,
            moving: !done,
            direction: true,
            high_limit: (status & 0x2000) != 0,
            low_limit: (status & 0x8000) != 0,
            powered: (status & 0x100) == 0,
            has_encoder: true,
            gain_support: true,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_hex_reply_offset_and_width() {
        assert_eq!(parse_hex_reply("0001 000001F4", 8), Some(500));
        assert_eq!(parse_hex_reply("0001 0008", 4), Some(8));
        // Negative position via u32 wrap.
        assert_eq!(parse_hex_reply("0001 FFFFFFF4", 8), Some(-12));
        // Too short after the offset.
        assert_eq!(parse_hex_reply("0001", 4), None);
    }

    #[test]
    fn accel_param_clamps() {
        // Below speed but positive -> unchanged.
        let ac = accel_param(500, 1000, 1.0, 100);
        assert!((1..=100).contains(&ac));
        // Zero/negative accel -> clamped to 1.
        assert_eq!(accel_param(500, 1000, 0.0, 100), 1);
        // Huge accel -> clamped to the speed.
        assert_eq!(accel_param(500, 1000, 1e9, 100), 100);
    }

    #[test]
    fn speed_and_current_params() {
        assert_eq!(speed_param(500, 100.0), nint(500.0 * 6e-5 * 100.0));
        assert_eq!(ano_param(1000), nint(1000.0 * 0.865909 + 2103.431));
    }
}
