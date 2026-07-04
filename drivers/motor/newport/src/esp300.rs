//! Newport ESP100/ESP300/ESP301 motor controller driver (serial/GPIB ASCII).
//!
//! Ported from `motorNewport/newportApp/src/drvESP300.cc` + `devESP300.cc`
//! (a model-1 dev/drv pair). Commands are `{axis:02}CC[value]` with a 1-based
//! zero-padded axis prefix; several record transactions join commands with
//! `;` into one write (a move is `VB;VA;AC;AG;PA`). Replies end in CR/LF
//! (C sets input EOS `\n` and strips the `\r`; this driver owns framing like
//! its siblings and trims both).
//!
//! ## Shared-serial concurrency
//!
//! One controller drives up to six axes over a single line: every axis is an
//! independent [`AsynMotor`] sharing the [`Esp300Controller`] behind an
//! `Arc<Mutex<..>>`, each operation holding the lock for its whole
//! write→read exchange (see [`crate::agap`] for the pattern rationale).
//!
//! ## Units
//!
//! The record works in steps; command values are `steps *
//! drive_resolution` and position readbacks divide back (C `cntrl_units` /
//! `set_status`). `drive_resolution` is discovered per axis at startup from
//! the feedback configuration (`ZB?` → `FR?`/`QS?` or `SU?`).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::util::{atof, atoi, leading_hex};

/// Response buffer size for a single controller reply (C `BUFF_SIZE` 100).
const READ_BUF: usize = 256;

/// Command line terminator (C output EOS `"\r"`). This driver owns framing
/// explicitly, like its siblings (see [`crate::smc100`]).
const TERMINATOR: &[u8] = b"\r";

/// Maximum axes per controller (C `ESP300_MAX_AXIS`).
pub(crate) const ESP300_MAX_AXES: usize = 6;

fn esp_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

/// Controller communication state (C `cntrl->status`): one garbled status
/// reply is retried silently; a second consecutive one is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// Shared controller endpoint: owns the serial handle and the cross-axis
/// communication state. Methods take `&self`/`&mut self`; the caller holds
/// the `Arc<Mutex<..>>` lock.
pub struct Esp300Controller {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
    comm_state: CommState,
}

impl Esp300Controller {
    /// Connect and identify an ESP300: flush, read the identity (`VE?`, up to
    /// 3 tries), then discover the axis count by stopping each axis and
    /// checking `TB?` for error 9 ("axis number out of range") — C
    /// `motor_init`. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes: 0,
            comm_state: CommState::Normal,
        };
        for _ in 0..3 {
            if let Ok(ident) = ctrl.write_read("VE?")
                && !ident.is_empty()
            {
                ctrl.ident = ident;
                break;
            }
        }
        if ctrl.ident.is_empty() {
            return Err(esp_err(
                "ESP300: no response to VE? identity query".to_string(),
            ));
        }

        // Stop each axis in turn; the first "axis number out of range" (9)
        // gives the axis count. Any other error aborts the scan there (a
        // missing stage is not an error).
        let mut num_axes = 0;
        for axis in 1..=ESP300_MAX_AXES {
            ctrl.write(&format!("{axis:02}ST"))?;
            let reply = ctrl.write_read("TB?")?;
            let code = atoi(&reply);
            if code == 9 {
                break;
            }
            if code != 0 {
                eprintln!("ESP300: error accessing motor {axis}: {reply}");
                break;
            }
            num_axes = axis;
        }
        ctrl.num_axes = num_axes;
        Ok(ctrl)
    }

    /// Identity string from `VE?`.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes discovered at construction.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command (C `send_mess`); the terminator is appended here.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Write a command and read the reply (C `send_mess` + `recv_mess`).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        self.read_reply()
    }

    /// Read one reply, working around the ESP300 firmware bug where an
    /// unsolicited hard-travel-limit error message (`E35`..`E42`) precedes
    /// the real reply — C `recv_mess` re-reads to flush it.
    fn read_reply(&self) -> AsynResult<String> {
        let reply = self.read_once()?;
        if is_unsolicited_limit_error(&reply) {
            return self.read_once();
        }
        Ok(reply)
    }

    fn read_once(&self) -> AsynResult<String> {
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let s = String::from_utf8_lossy(&raw);
        // Replies end in CR/LF (C strips the LF via input EOS, the CR itself).
        Ok(s.trim_end_matches(['\r', '\n']).to_string())
    }
}

/// C `recv_mess` retry predicate: a reply longer than 3 characters starting
/// with `E` whose number is a hard-travel-limit code (35..=42) is an
/// unsolicited error message to flush.
fn is_unsolicited_limit_error(reply: &str) -> bool {
    reply.len() > 3 && reply.starts_with('E') && (35..=42).contains(&atoi(&reply[1..]))
}

/// The velocity/acceleration preamble every motion transaction carries
/// (record `SET_VEL_BASE`/`SET_VELOCITY`/`SET_ACCEL` before the move):
/// `VB;VA;AC;AG` — the ESP300 takes acceleration and deceleration (`AG`)
/// separately, C sends both. Values are already controller units.
fn motion_preamble(axis: usize, vel_base: f64, velocity: f64, acceleration: f64) -> String {
    format!(
        "{axis:02}VB{vel_base:.6};{axis:02}VA{velocity:.6};{axis:02}AC{acceleration:.6};{axis:02}AG{acceleration:.6}"
    )
}

/// One ESP300 axis sharing a controller. Implements [`AsynMotor`].
pub struct Esp300Axis {
    controller: Arc<Mutex<Esp300Controller>>,
    /// 1-based controller axis number, sent zero-padded (`%.2d`).
    axis: usize,
    /// Controller units per motor step (C `drive_resolution`), discovered at
    /// construction from the feedback configuration.
    drive_resolution: f64,
    /// Stage has an encoder (`ZB?` feedback bits 8/9) — drives the
    /// gain-support/has-encoder status bits (C `EA_PRESENT`/`GAIN_SUPPORT`).
    encoder_present: bool,
    /// Last polled status, reused on the early-exit poll paths where C leaves
    /// the record's other bits stale (TB?/MD errors).
    last_status: MotorStatus,
}

impl Esp300Axis {
    /// Construct axis `axis` (1-based), reading its drive resolution from the
    /// controller (C `motor_init` per-motor block): stepper closed-loop
    /// without encoder feedback (`ZB?` bits 9:8 = `10`) uses full-step (`FR?`)
    /// / microstep (`QS?`); open loop (`00`) uses `FR?` alone; anything else
    /// has an encoder and uses its resolution (`SU?`). Performs blocking
    /// serial I/O under the controller lock.
    pub fn new(controller: Arc<Mutex<Esp300Controller>>, axis: usize) -> AsynResult<Self> {
        let (drive_resolution, encoder_present) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // C reads the controller's EGU (SN?) only for its report(); issue
            // the query for startup wire fidelity and discard the reply.
            let _ = ctrl.write_read(&format!("{axis:02}SN?"))?;
            let feedback = leading_hex(&ctrl.write_read(&format!("{axis:02}ZB?"))?).unwrap_or(0);
            let resolution = match feedback & 0x300 {
                0x200 => {
                    let full_step = atof(&ctrl.write_read(&format!("{axis:02}FR?"))?);
                    let micro_step = atoi(&ctrl.write_read(&format!("{axis:02}QS?"))?);
                    full_step / f64::from(micro_step)
                }
                0x0 => atof(&ctrl.write_read(&format!("{axis:02}FR?"))?),
                _ => atof(&ctrl.write_read(&format!("{axis:02}SU?"))?),
            };
            (resolution, feedback & 0x300 != 0)
        };
        if drive_resolution == 0.0 || !drive_resolution.is_finite() {
            return Err(esp_err(format!(
                "ESP300 axis {axis}: invalid drive resolution {drive_resolution}"
            )));
        }
        Ok(Self {
            controller,
            axis,
            drive_resolution,
            encoder_present,
            last_status: MotorStatus::default(),
        })
    }

    fn lock(&self) -> MutexGuard<'_, Esp300Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Steps → controller units (C `cntrl_units = dval * drive_resolution`).
    fn units(&self, steps: f64) -> f64 {
        steps * self.drive_resolution
    }

    /// [`motion_preamble`] with this axis's step scaling applied.
    fn motion_preamble(&self, min_velocity: f64, velocity: f64, acceleration: f64) -> String {
        motion_preamble(
            self.axis,
            self.units(min_velocity),
            self.units(velocity),
            self.units(acceleration),
        )
    }
}

impl AsynMotor for Esp300Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // One transaction: VB;VA;AC;AG;PA (the ESP300 starts moving on PA;
        // the record's GO is a no-op).
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{};{:02}PA{:.6}",
            self.motion_preamble(min_velocity, velocity, acceleration),
            self.axis,
            self.units(position)
        ))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{};{:02}PR{:.6}",
            self.motion_preamble(min_velocity, velocity, acceleration),
            self.axis,
            self.units(distance)
        ))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // Record JOG transaction: SET_ACCEL then VA with the jog speed and a
        // signed MV (C `JOG` case).
        let a = self.axis;
        let sign = if self.units(velocity) > 0.0 { '+' } else { '-' };
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{a:02}AC{:.6};{a:02}AG{:.6};{a:02}VA{:.6};{a:02}MV{sign}",
            self.units(acceleration),
            self.units(acceleration),
            self.units(velocity).abs(),
        ))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C sends the same `OR` for HOME_FOR and HOME_REV (direction ignored),
        // after the record's velocity/acceleration transaction parts.
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{};{:02}OR",
            self.motion_preamble(min_velocity, velocity, acceleration),
            self.axis
        ))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&format!("{:02}ST", self.axis))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `LOAD_POS`: define home (DH) at the given position.
        let ctrl = self.lock();
        ctrl.write(&format!("{:02}DH{:.6}", self.axis, self.units(position)))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C ENABLE_TORQUE → motor on, DISABL_TORQUE → motor off.
        let ctrl = self.lock();
        if enable {
            ctrl.write(&format!("{:02}MO", self.axis))
        } else {
            ctrl.write(&format!("{:02}MF", self.axis))
        }
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&format!("{:02}SR{:.6}", self.axis, self.units(position)))
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&format!("{:02}SL{:.6}", self.axis, self.units(position)))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        // C SET_[PID]GAIN: KP/KI/KD then UF (update filter), with an
        // *unpadded* axis prefix (C uses %d here, unlike every other command)
        // and the gain scaled by drive resolution like any other value.
        let a = self.axis;
        let cc = match kind {
            PidGainKind::Proportional => "KP",
            PidGainKind::Integral => "KI",
            PidGainKind::Derivative => "KD",
        };
        let ctrl = self.lock();
        ctrl.write(&format!("{a}{cc}{:.6};{a}UF", self.units(gain)))
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Port of C `set_status`, one exchange sequence per axis under the
        // controller lock: TB? (clear/check errors), MD (done), TP?
        // (position), ZH?, MO? (power), TE? (error code).
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let a = self.axis;

        let tb = ctrl.write_read("TB?")?;
        if !tb.starts_with('0') {
            eprintln!("ESP300 status error: {tb}.");
            self.last_status.problem = true;
            return Ok(self.last_status.clone());
        }
        self.last_status.problem = false;

        let md = ctrl.write_read(&format!("{a:02}MD"))?;
        if md == "0" || md == "1" {
            ctrl.comm_state = CommState::Normal;
            self.last_status.comms_error = false;
        } else if ctrl.comm_state == CommState::Normal {
            // One garbled reply: retry silently next poll (C RETRY state),
            // leaving the record's bits stale.
            ctrl.comm_state = CommState::Retry;
            return Ok(self.last_status.clone());
        } else {
            ctrl.comm_state = CommState::CommErr;
            self.last_status.comms_error = true;
            self.last_status.problem = true;
            return Ok(self.last_status.clone());
        }
        let done = md == "1";

        let position = atof(&ctrl.write_read(&format!("{a:02}TP?"))?) / self.drive_resolution;
        // Direction from the position delta; only one position query exists,
        // so the encoder position is the same value (C set_status).
        if position != self.last_status.position {
            self.last_status.direction = position >= self.last_status.position;
        }

        // Hardware limit/home switches: C reads the ZH? configuration and
        // computes `use_limits = (limits&0x1 == 0) || (limits&0x5 == 0)` —
        // with C precedence that is `limits & (0x1==0)`, always false, so the
        // PH limit query is dead code and the switch bits are always cleared.
        // Kept bug-for-bug: issue ZH? (wire parity), report no switches.
        let _ = ctrl.write_read(&format!("{a:02}ZH?"))?;

        let powered = atoi(&ctrl.write_read(&format!("{a:02}MO?"))?) != 0;

        let te = ctrl.write_read("TE?")?;
        let errcode = atoi(&te);
        let problem = errcode != 0;
        if problem {
            eprintln!("ESP300 controller error = {errcode}.");
        }
        drop(ctrl);

        self.last_status = MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0, // C: "Parse motor velocity? NEEDS WORK"
            done,
            moving: !done,
            high_limit: false,
            low_limit: false,
            home: false,
            powered,
            problem,
            direction: self.last_status.direction,
            comms_error: false,
            gain_support: self.encoder_present,
            has_encoder: self.encoder_present,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preamble_matches_c_transaction() {
        // C build_trans: %.2d axis prefix, %f (6-decimal) values, AC and AG
        // both sent with the acceleration.
        assert_eq!(
            motion_preamble(1, 0.5, 2.0, 8.0),
            "01VB0.500000;01VA2.000000;01AC8.000000;01AG8.000000"
        );
        assert_eq!(
            motion_preamble(12, 0.0, 1.0, 4.0),
            "12VB0.000000;12VA1.000000;12AC4.000000;12AG4.000000"
        );
    }

    #[test]
    fn unsolicited_limit_error_predicate() {
        // Hard-travel-limit codes 35..=42 with a message are flushed.
        assert!(is_unsolicited_limit_error("E35 HARDWARE LIMIT"));
        assert!(is_unsolicited_limit_error("E42 x"));
        // Codes outside 35..=42 are real replies.
        assert!(!is_unsolicited_limit_error("E34 x"));
        assert!(!is_unsolicited_limit_error("E43 x"));
        // C requires nread > 3: a bare short reply is never flushed.
        assert!(!is_unsolicited_limit_error("E35"));
        // Ordinary replies pass through.
        assert!(!is_unsolicited_limit_error("0, NO ERROR DETECTED"));
        assert!(!is_unsolicited_limit_error("1"));
    }
}
