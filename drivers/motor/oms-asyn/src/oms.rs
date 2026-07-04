//! Pro-Dex OMS MAXnet / MXA motor controller driver (ASCII, over an asyn octet
//! port).
//!
//! Ported from `motorOmsAsyn/omsAsynApp/src` (`omsBaseController` /
//! `omsBaseAxis` plus the `omsMAXnet` / `omsMXA` transports, all model-3). The
//! MAXnet and MXA speak the identical OMS ASCII command set over a serial or
//! TCP `drvAsyn*Port`; they differ only in a label and minimum firmware, so one
//! driver covers both. The VME `omsMAXv` sibling is register/interrupt-mapped
//! (not an octet transport) and is out of scope.
//!
//! ## Protocol
//!
//! Commands are `;`-separated OMS mnemonics prefixed with an axis selector:
//! `A<char>` selects one axis (chars `X Y Z T U V R S W K` for axes 0..9), `AM`
//! addresses all. The driver appends the `"\n"` command terminator; the startup
//! script sets the input EOS (`"\n\r"`, the OMS reply terminator). Accepted
//! commands are acknowledged with a leading `0x06` (stripped here).
//!
//! The controller also emits unsolicited `"%000 ..."` *notification* lines when
//! a move completes. The C driver registers an asyn interrupt handler to count
//! them and accelerate the poller; this port instead polls at the configured
//! rate (as every other asyn-rs motor driver does) and simply skips any
//! notification line that arrives in the reply stream. The notification-driven
//! fast-poll optimisation is therefore not modeled — only its correctness
//! effect (move-done detection) matters, and that comes from the polled status.
//!
//! ## Units
//!
//! Positions, encoder counts, velocities and accelerations are controller-
//! native steps / steps-per-second; with the motor record's `MRES` = 1 a step
//! is one EGU, so the record's EGU values pass straight through (clamped to the
//! OMS ranges: velocity 1..4000000, home velocity 1..1000000, acceleration
//! 1..8000000, position magnitude <= 67000000). There is no resolution factor
//! to cancel: the boundary is controller-native steps with `MRES` = 1.
//!
//! ## Not modeled (documented)
//!
//! - The async notification interrupt / watchdog / connection-reset machinery
//!   (`asynCallback`, `waitInterruptible`, `resetConnection`): replaced by plain
//!   periodic polling with notification-line skipping.
//! - The controller-central multi-axis poller: this port polls each axis
//!   independently (single-axis `A<char>` queries), the natural fit for the
//!   per-axis `AsynMotor::poll` boundary. Slightly more traffic; same result.
//! - The auxiliary parameter records exposed through the C `writeInt32` /
//!   `writeFloat64` overrides, closed-loop gain status (`CL?`), the following-
//!   error / encoder-status word (`EA`), and the `setEncoderRatio` hook (ratio
//!   stays 1.0). These are not motor-record motion fields.
//! - The init-time firmware-version gate and axis-count auto-detection: the
//!   driver reads the firmware string once for logging, assumes the modern
//!   (>= 1.30) command forms MAXnet mandates, and trusts the configured axis
//!   count.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::atoi;

/// Reply read buffer size (C `OMSINPUTBUFFERLEN` = 12*10 + 2).
const READ_BUF: usize = 256;

/// Command terminator appended by the driver (C output EOS `"\n"`).
const TERMINATOR: &[u8] = b"\n";

/// Per-axis command selector characters (C `axisChrArr`).
const AXIS_CHARS: [char; 10] = ['X', 'Y', 'Z', 'T', 'U', 'V', 'R', 'S', 'W', 'K'];

/// Move parameter clamps (C `omsBaseAxis`).
const MAX_VELOCITY: i32 = 4_000_000;
const MAX_HOME_VELOCITY: i32 = 1_000_000;
const MAX_ACCEL: i32 = 8_000_000;
const MAX_POSITION: i32 = 67_000_000;

fn oms_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// A line the controller sends unsolicited on move completion
/// (`"%000 SSSSSSSS"`); C `isNotification` keys on the `"000 0"` substring.
fn is_notification(s: &str) -> bool {
    s.contains("000 0")
}

/// An OMS MAXnet/MXA controller endpoint owning the asyn octet handle, shared by
/// its axes behind a mutex so command/reply pairs stay atomic.
pub struct OmsController {
    handle: SyncIOHandle,
    /// `"MAXnet"` or `"MXA"` (label only; the protocol is identical).
    controller_type: &'static str,
}

impl OmsController {
    /// Wrap a connected octet handle.
    pub fn new(handle: SyncIOHandle, controller_type: &'static str) -> Self {
        Self {
            handle,
            controller_type,
        }
    }

    /// Controller label, for logging.
    pub fn controller_type(&self) -> &'static str {
        self.controller_type
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Send a command expecting no reply (C `sendOnly`).
    pub fn send_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Send a command and read one reply, skipping any notification lines and
    /// stripping the leading ACK/CR/LF (C `sendReceive`).
    pub fn send_receive(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        // A move notification may sit ahead of the real reply; skip it. Bound
        // the retries so a chatty controller cannot spin forever.
        for _ in 0..12 {
            let raw = self.handle.read_octet(0, READ_BUF)?;
            let s = String::from_utf8_lossy(&raw);
            let trimmed = s
                .trim_start_matches(['\x06', '\r', '\n'])
                .trim_end_matches(['\r', '\n', '\0', ' ']);
            if is_notification(trimmed) {
                continue;
            }
            return Ok(trimmed.to_string());
        }
        Err(oms_err("oms: only notification lines received"))
    }

    /// Read the firmware version string (`WY`) for a comms check and logging.
    pub fn firmware_version(&self) -> AsynResult<String> {
        self.send_receive("WY")
    }

    /// Interrupt-clear and stop-all, then send an optional init string (C
    /// `Init` preamble). Called once at controller creation.
    pub fn init(&self, init_string: &str) -> AsynResult<()> {
        self.send_only("IC;")?;
        self.send_only("AM SA;")?;
        if !init_string.is_empty() {
            self.send_only(init_string)?;
        }
        Ok(())
    }
}

/// One OMS axis sharing a controller. Implements [`AsynMotor`].
pub struct OmsAxis {
    controller: Arc<Mutex<OmsController>>,
    /// 0-based axis index (limit-flag bit position).
    axis_index: usize,
    /// Command selector char (`A<char>`).
    axis_char: char,
    has_encoder: bool,
    /// Limit true-state inversion (C `invertLimit`, from `LT?`).
    invert_limit: bool,
    /// Last base velocity sent, to order `VL`/`VB` so `VB < VL` always holds.
    last_min_velo: i32,
    /// Consecutive polls seen stopped-but-not-done (C `moveDelay`).
    move_delay: i32,
    /// Whether the previous poll reported motion (C poller `axisMoving`).
    was_moving: bool,
}

impl OmsAxis {
    /// Construct axis `index` and probe its type (`PS?`) and limit true-state
    /// (`LT?`). Assumes the modern (firmware >= 1.30) command forms.
    pub fn new(controller: Arc<Mutex<OmsController>>, index: usize) -> AsynResult<Self> {
        let axis_char = *AXIS_CHARS
            .get(index)
            .ok_or_else(|| oms_err(format!("oms: axis index {index} out of range (max 9)")))?;

        let (has_encoder, invert_limit) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // Axis type: "=O" stepper/no-encoder, "=M" servo, "=E" stepper+enc.
            let ps = ctrl.send_receive(&format!("A{axis_char} PS?"))?;
            let has_encoder = match ps.as_bytes().get(1) {
                Some(b'O') => false,
                Some(b'M') | Some(b'E') => true,
                _ => return Err(oms_err(format!("oms: unknown axis type '{ps}'"))),
            };
            // Limit true-state: "=l"/"=L" invert, "=h"/"=H" no invert.
            let lt = ctrl.send_receive(&format!("A{axis_char} LT?"))?;
            let invert_limit = match lt.as_bytes().get(1) {
                Some(b'l') | Some(b'L') => true,
                Some(b'h') | Some(b'H') => false,
                _ => return Err(oms_err(format!("oms: unknown limit true-state '{lt}'"))),
            };
            (has_encoder, invert_limit)
        };

        Ok(Self {
            controller,
            axis_index: index,
            axis_char,
            has_encoder,
            invert_limit,
            last_min_velo: 0,
            move_delay: 0,
            was_moving: false,
        })
    }

    fn lock(&self) -> MutexGuard<'_, OmsController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Round to nearest step, away from zero (C `(int)(x +/- 0.5)`).
    fn nint(x: f64) -> i32 {
        if x < 0.0 {
            (x - 0.5) as i32
        } else {
            (x + 0.5) as i32
        }
    }

    /// Shared move body (absolute or relative).
    fn do_move(
        &mut self,
        position: f64,
        relative: bool,
        min_v: f64,
        max_v: f64,
        accel: f64,
    ) -> AsynResult<()> {
        // Servo/encoder axes: sync the motor position to the encoder first.
        if self.has_encoder {
            let ctrl = self.lock();
            let ch = self.axis_char;
            let enc = atoi(&ctrl.send_receive(&format!("A{ch};RE;"))?);
            // encoderRatio is fixed at 1.0 (the setEncoderRatio hook is unported).
            ctrl.send_only(&format!("A{ch};LO{enc};"))?;
        }

        let pos = Self::nint(position);
        if pos.abs() > MAX_POSITION {
            return Err(oms_err(format!("oms: position {pos} out of range")));
        }
        let velo = Self::nint(max_v).clamp(1, MAX_VELOCITY);
        let minvelo = Self::nint(min_v).clamp(0, velo - 1);
        let acc = (accel.abs() as i32).clamp(1, MAX_ACCEL);
        let relabs = if relative { "MR" } else { "MA" };

        let ch = self.axis_char;
        let cmd = if velo < self.last_min_velo {
            format!("A{ch};AC{acc};VB{minvelo};VL{velo};{relabs}{pos};GO;ID;")
        } else {
            format!("A{ch};AC{acc};VL{velo};VB{minvelo};{relabs}{pos};GO;ID;")
        };
        self.last_min_velo = minvelo;
        self.lock().send_only(&cmd)?;
        self.was_moving = true;
        self.move_delay = 0;
        Ok(())
    }
}

impl AsynMotor for OmsAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false, min_velocity, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true, min_velocity, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C truncates (int)acceleration / (int)maxVelocity — not round-to-nearest.
        let acc = (acceleration as i32).clamp(1, MAX_ACCEL);
        let velo = (velocity as i32).clamp(-MAX_VELOCITY, MAX_VELOCITY);
        let ch = self.axis_char;
        self.lock()
            .send_only(&format!("A{ch} AC{acc}; JG{velo};"))?;
        self.was_moving = true;
        self.move_delay = 0;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // C home() truncates (int)max_velocity (no +0.5, unlike move()); minvelo
        // does round-to-nearest.
        let velo = (velocity as i32).clamp(1, MAX_HOME_VELOCITY);
        let minvelo = Self::nint(min_velocity).clamp(0, velo - 1);
        let acc = (acceleration.abs() as i32).clamp(1, MAX_ACCEL);
        let dir = if forward { "HM" } else { "HR" };
        let ch = self.axis_char;
        let cmd = if velo < self.last_min_velo {
            format!("A{ch};AC{acc};VB{minvelo};VL{velo};{dir};MA0;GO;ID;")
        } else {
            format!("A{ch};AC{acc};VL{velo};VB{minvelo};{dir};MA0;GO;ID;")
        };
        self.last_min_velo = minvelo;
        self.lock().send_only(&cmd)?;
        self.was_moving = true;
        self.move_delay = 0;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let mut acc = (acceleration.abs() + 0.5) as i32;
        if acc > MAX_ACCEL {
            acc = MAX_ACCEL;
        }
        if acc < 1 {
            acc = 200_000;
        }
        let ch = self.axis_char;
        self.lock().send_only(&format!("A{ch} AC{acc}; ST ID;"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ch = self.axis_char;
        let pos = position as i32;
        self.lock().send_only(&format!("A{ch} LP{pos};"))
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let (pos, status, velo, enc, limit_hex) = {
            let ctrl = self.lock();
            let ch = self.axis_char;
            let pos = atoi(&ctrl.send_receive(&format!("A{ch};RP;"))?);
            let status = ctrl.send_receive(&format!("A{ch};RI;"))?;
            let velo = atoi(&ctrl.send_receive(&format!("A{ch};RV;"))?);
            let enc = if self.has_encoder {
                atoi(&ctrl.send_receive(&format!("A{ch};RE;"))?)
            } else {
                0
            };
            // All-axes limit flags (hex); optional — fall back to the status bit.
            let limit_hex = ctrl
                .send_receive("AM;QL;")
                .ok()
                .and_then(|s| u32::from_str_radix(s.trim(), 16).ok());
            (pos, status, velo, enc, limit_hex)
        };

        let b = status.as_bytes();
        if b.len() < 4 {
            return Err(oms_err(format!("oms: short status '{status}'")));
        }
        // Status field "MDNN": [0] direction P/M, [1] done D, [2] limit L, [3] home H.
        let direction = b[0] == b'P';
        let d_flag = b[1] == b'D';
        let at_limit = b[2] == b'L';
        let at_home = b[3] == b'H';

        // Done / problem detection (C poller): the D flag is authoritative; a
        // stopped axis at a limit is done; a stopped axis that never sets D for
        // several polls is flagged as a problem.
        let done;
        let mut problem = false;
        if d_flag {
            done = true;
            self.move_delay = 0;
        } else if velo == 0 && self.was_moving {
            if at_limit {
                done = true;
                self.move_delay = 0;
            } else {
                self.move_delay += 1;
                if self.move_delay >= 5 {
                    done = true;
                    problem = true;
                    self.move_delay = 0;
                } else {
                    done = false;
                }
            }
        } else if velo == 0 {
            // Idle and not previously moving.
            done = true;
            self.move_delay = 0;
        } else {
            done = false;
            self.move_delay = 0;
        }
        let moving = !done;
        self.was_moving = moving;

        // Limits: prefer the all-axes flag word, else derive from the status bit.
        let (high_limit, low_limit) = if let Some(flags) = limit_hex {
            let off = if self.axis_index > 7 { 8 } else { 0 };
            let low_bit = (flags & (1 << (self.axis_index + off))) != 0;
            let high_bit = (flags & (1 << (self.axis_index + off + 8))) != 0;
            (high_bit ^ self.invert_limit, low_bit ^ self.invert_limit)
        } else if at_limit {
            (direction, !direction)
        } else {
            (false, false)
        };

        Ok(MotorStatus {
            position: pos as f64,
            encoder_position: enc as f64,
            done,
            moving,
            high_limit,
            low_limit,
            home: at_home,
            direction,
            problem,
            has_encoder: self.has_encoder,
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nint_rounds_away_from_zero() {
        assert_eq!(OmsAxis::nint(1.4), 1);
        assert_eq!(OmsAxis::nint(1.5), 2);
        assert_eq!(OmsAxis::nint(-1.4), -1);
        assert_eq!(OmsAxis::nint(-1.5), -2);
        assert_eq!(OmsAxis::nint(0.0), 0);
    }

    #[test]
    fn framed_appends_terminator() {
        assert_eq!(OmsController::framed("AX;RP;"), b"AX;RP;\n");
    }

    #[test]
    fn notification_detection() {
        assert!(is_notification("%000 01000000"));
        assert!(is_notification("000 00000001"));
        assert!(!is_notification("MDNN,MDNN"));
        assert!(!is_notification("12345"));
        assert!(!is_notification("3F"));
    }

    #[test]
    fn axis_chars_cover_ten_axes() {
        assert_eq!(AXIS_CHARS[0], 'X');
        assert_eq!(AXIS_CHARS[3], 'T');
        assert_eq!(AXIS_CHARS[9], 'K');
    }
}
