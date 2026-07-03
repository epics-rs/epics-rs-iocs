//! Newport CONEX-AGP / CONEX-CC / CONEX-PP / DL / FCL200 controller driver.
//!
//! Ported from `motorNewport/newportApp/src/AG_CONEX.cpp` (`AG_CONEXAxis` /
//! `AG_CONEXController`). The C pair subclasses `asynMotorController` /
//! `asynMotorAxis`; here a single [`ConexAxis`] implements asyn-rs
//! [`AsynMotor`] and the controller/serial-port/record wiring is assembled by
//! the IOC binary (the analogue of `AG_CONEXCreateController`).
//!
//! A CONEX controller is single-axis; commands are prefixed by the numeric
//! controller ID (C `controllerID_`, e.g. `1TP`). The driver holds a
//! [`SyncIOHandle`] to a dedicated serial port and issues CR/LF-terminated
//! ASCII. Five models are supported, each with model-dependent step size,
//! velocity/PID capability, and status parsing — see [`ConexModel`].
//!
//! ## Units
//!
//! The motor record works in raw steps; `step_size` (C `stepSize_`) is the
//! controller EGU-per-step, computed at construction from model-specific
//! queries. Outgoing positions/velocities are multiplied by `step_size` and
//! the polled position divided back, matching the C driver.
//!
//! ## Parity notes
//!
//! Two behaviors are ported faithfully from the C even though they look like C
//! bugs; they are flagged inline at their call sites:
//! - non-DL `TS` parsing reads only the 2-char controller-state byte, so the
//!   `0x100`/`0x200` limit-bit masks are inert (C `AG_CONEX.cpp:533,548`).
//! - `set_pid_gain(Derivative)` on CC/DL emits a `KI` command (C sends `KI`
//!   with `KDMax`, `AG_CONEX.cpp:480`), not `KD`.

use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::util::{leading_hex, parse_int_at, parse_value_at};

/// Response buffer size for a single controller reply.
const READ_BUF: usize = 256;

/// Command line terminator (CR/LF). As with the SMC100 driver, this driver
/// owns framing and appends the terminator itself rather than relying on port
/// EOS (asyn-rs exposes no `asynOctetSetInputEos` iocsh command on the
/// published release). See [`crate::smc100`] for the rationale.
const TERMINATOR: &[u8] = b"\r\n";

// C `CONEX_TIMEOUT` (2 s) is applied where the transport is opened —
// `SyncIOHandle::from_handle` in `crate::ioc` — not per write here.

/// C `HOME_TYPE_MINUS_EOR` — minus end-of-run home type forced for CONEX-PP.
const HOME_TYPE_MINUS_EOR: i32 = 4;

/// CONEX controller model (C `ConexModel_t`), detected from the firmware
/// version string at construction.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConexModel {
    /// CONEX-AGP (piezo, no velocity/accel, PID + low-pass filter).
    Agp,
    /// CONEX-CC (DC servo).
    Cc,
    /// CONEX-PP (stepper).
    Pp,
    /// DL series.
    Dl,
    /// FCL200 (positions in mm).
    Fcl200,
}

/// Closed-loop gain maxima used to scale the 0..=1 motor-record gains onto the
/// controller's native range (C `KPMax_`/`KIMax_`/`KDMax_`/`LFMax_`).
struct GainMax {
    kp: f64,
    ki: f64,
    kd: f64,
    lf: f64,
}

impl ConexModel {
    /// C `AG_CONEXAxis::AG_CONEXAxis`: model detection via substring match on
    /// the firmware version string, in the same priority order as the C
    /// `strstr` chain. Returns `None` for an unrecognized string.
    pub fn from_version(version: &str) -> Option<Self> {
        if version.contains("CONEX-AGP") {
            Some(Self::Agp)
        } else if version.contains("CONEX-CC") {
            Some(Self::Cc)
        } else if version.contains("Conex PP") {
            Some(Self::Pp)
        } else if version.contains("FC series") {
            Some(Self::Fcl200)
        } else if version.contains("DL") {
            Some(Self::Dl)
        } else {
            None
        }
    }

    /// CC and AGP carry an incremental encoder; their step size derives from
    /// the encoder increment / interpolation factor, and they report gain
    /// support to the motor record (C `motorStatusGainSupport_`).
    fn is_cc_or_agp(self) -> bool {
        matches!(self, Self::Cc | Self::Agp)
    }

    /// CC/PP/FCL200/DL accept the `AC`/`VA` accel+velocity preamble on a move;
    /// AGP does not (C `AG_CONEXAxis::move`).
    fn supports_velocity(self) -> bool {
        !matches!(self, Self::Agp)
    }

    /// PID gain commands apply only to AGP/CC/DL (C `setPGain`/`setIGain`/
    /// `setDGain` guard).
    fn supports_pid(self) -> bool {
        matches!(self, Self::Agp | Self::Cc | Self::Dl)
    }

    /// C constructor gain maxima. C sets these only for AGP and CC; DL/PP/
    /// FCL200 leave them uninitialized (read as garbage in the C PID path).
    /// Rust uses a defined `0.0`, so PID writes on DL send `0` — flagged as a
    /// deliberate deviation from C's undefined behavior.
    fn gain_max(self) -> GainMax {
        match self {
            Self::Agp => GainMax {
                kp: 3000.,
                ki: 3000.,
                kd: 0.,
                lf: 1000.,
            },
            Self::Cc => GainMax {
                kp: 1.0e6,
                ki: 1.0e6,
                kd: 1.0e6,
                lf: 0.,
            },
            _ => GainMax {
                kp: 0.,
                ki: 0.,
                kd: 0.,
                lf: 0.,
            },
        }
    }
}

/// C `AG_CONEXAxis::AG_CONEXAxis` step-size computation.
///
/// `encoder_increment`/`interpolation_factor` come from `SU?`/`IF?` (CC/AGP),
/// `full_step_size`/`micro_steps_per_full_step` from `FRS?`/`FRM?` (PP/FCL200).
fn compute_step_size(
    model: ConexModel,
    encoder_increment: f64,
    interpolation_factor: f64,
    full_step_size: f64,
    micro_steps_per_full_step: i32,
) -> f64 {
    match model {
        ConexModel::Agp | ConexModel::Cc => encoder_increment / interpolation_factor,
        ConexModel::Fcl200 => 1.0,
        // DL operates in mm; the motor record wants an integer readback, so C
        // scales by 1e4 and sets resolution 1e-4.
        ConexModel::Dl => 0.0001,
        // CONEX-PP.
        ConexModel::Pp => full_step_size / micro_steps_per_full_step as f64 / 1000.0,
    }
}

/// Decoded `TS` status: motion done + soft-limit flags.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct TsStatus {
    done: bool,
    low_limit: bool,
    high_limit: bool,
}

/// Parse a `TS` (tell-status) reply. C `AG_CONEXAxis::poll`:
/// - DL: `"%*dTS%*6c%x"` — skip 6 chars after `TS`, read the 2-char state hex;
///   moving when state `== 0x3c`.
/// - others: `"%*dTS%*4c%x"` — skip 4 chars after `TS`, read the state byte;
///   moving when `state & 0xff` is `0x1e` or `0x28`. Only CC/PP consult the
///   `0x100`/`0x200` limit bits — which the 2-char read can never set, so they
///   stay inert exactly as in C.
fn parse_ts(model: ConexModel, resp: &str) -> Option<TsStatus> {
    let idx = resp.find("TS")?;
    let after = &resp[idx + 2..];
    let skip = if model == ConexModel::Dl { 6 } else { 4 };
    let hex_part = after.get(skip..)?;
    let status = leading_hex(hex_part)?;

    let done = if model == ConexModel::Dl {
        status != 0x3c
    } else {
        let state = status & 0xff;
        !(state == 0x1e || state == 0x28)
    };

    let (mut low_limit, mut high_limit) = (false, false);
    if matches!(model, ConexModel::Cc | ConexModel::Pp) {
        if status & 0x100 != 0 {
            low_limit = true;
        }
        if status & 0x200 != 0 {
            high_limit = true;
        }
    }
    Some(TsStatus {
        done,
        low_limit,
        high_limit,
    })
}

/// Parse an `MM?` reply into the closed-loop (power-on) flag. C
/// `AG_CONEXAxis::getClosedLoop`: `sscanf(inString_, "%*dMM%x", &status)`, then
/// closed-loop when the state is in `0x1e..=0x34`.
fn parse_closed_loop(resp: &str) -> Option<bool> {
    let idx = resp.find("MM")?;
    let status = leading_hex(&resp[idx + 2..])?;
    Some((0x1e..=0x34).contains(&status))
}

// --- Command formatters (bare text, C `outString_`; the driver frames them).

fn cmd_set_acceleration(id: i32, accel_egu: f64) -> String {
    format!("{id}AC{accel_egu:.6}")
}
fn cmd_set_velocity(id: i32, velocity_egu: f64) -> String {
    format!("{id}VA{velocity_egu:.6}")
}
fn cmd_move_absolute(id: i32, position_egu: f64) -> String {
    format!("{id}PA{position_egu:.6}")
}
fn cmd_move_relative(id: i32, distance_egu: f64) -> String {
    format!("{id}PR{distance_egu:.6}")
}
fn cmd_stop(id: i32) -> String {
    format!("{id}ST")
}
fn cmd_reset(id: i32) -> String {
    format!("{id}RS")
}
fn cmd_home(id: i32) -> String {
    format!("{id}OR")
}
fn cmd_dl_init(id: i32) -> String {
    format!("{id}IE")
}
fn cmd_home_velocity(id: i32, velocity: f64) -> String {
    format!("{id}OH{velocity:.6}")
}
fn cmd_home_type(id: i32, home_type: i32) -> String {
    format!("{id}HT{home_type}")
}
fn cmd_set_closed_loop(id: i32, enable: bool) -> String {
    format!("{id}MM{}", if enable { 1 } else { 0 })
}
fn cmd_set_gain(id: i32, letters: &str, value: f64) -> String {
    format!("{id}{letters}{value:.6}")
}
/// A bare query command: `{id}{code}` (e.g. `1TP`, `1SU?`).
fn query(id: i32, code: &str) -> String {
    format!("{id}{code}")
}

fn conex_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

fn parse_error(what: &str) -> AsynError {
    conex_err(format!("CONEX: could not parse {what} response"))
}

// --- Raw framed serial I/O (usable before `ConexAxis` exists, e.g. in `new`).

fn write_raw(handle: &SyncIOHandle, cmd: &str) -> AsynResult<()> {
    let mut framed = Vec::with_capacity(cmd.len() + TERMINATOR.len());
    framed.extend_from_slice(cmd.as_bytes());
    framed.extend_from_slice(TERMINATOR);
    handle.write_octet(0, &framed)?;
    Ok(())
}

fn write_read_raw(handle: &SyncIOHandle, cmd: &str) -> AsynResult<String> {
    write_raw(handle, cmd)?;
    let reply = handle.read_octet(0, READ_BUF)?;
    Ok(String::from_utf8_lossy(&reply).into_owned())
}

/// A Newport CONEX single-axis driver implementing [`AsynMotor`].
pub struct ConexAxis {
    handle: SyncIOHandle,
    /// Numeric controller ID prefixed on every command (C `controllerID_`).
    controller_id: i32,
    model: ConexModel,
    /// Controller EGU per motor step (C `stepSize_`).
    step_size: f64,
    /// Soft travel limits in controller EGU (C `lowLimit_`/`highLimit_`).
    low_limit: f64,
    high_limit: f64,
}

impl ConexAxis {
    /// Connect and identify a CONEX controller on `handle` with the given
    /// controller ID, running the C `AG_CONEXAxis` constructor's query
    /// sequence (version → model → stage/encoder/limit queries → step size).
    ///
    /// Performs blocking serial I/O; call it where that is acceptable (the
    /// iocsh create command, at IOC init).
    pub fn new(handle: SyncIOHandle, controller_id: i32) -> AsynResult<Self> {
        // Firmware version → model. C reads `&inString_[4]` for the VE reply.
        let version_resp = write_read_raw(&handle, &query(controller_id, "VE"))?;
        let version = version_resp.get(4..).unwrap_or("");
        let model = ConexModel::from_version(version)
            .ok_or_else(|| conex_err(format!("CONEX: unknown model, firmware=\"{version}\"")))?;

        // Stage ID: issued for wire fidelity (C stores it for report only).
        let _ = write_read_raw(&handle, &query(controller_id, "ID?"))?;

        // Encoder increment (CC/AGP only), else 1.0.
        let encoder_increment = if model.is_cc_or_agp() {
            parse_value_at(&write_read_raw(&handle, &query(controller_id, "SU?"))?, 3)
                .ok_or_else(|| parse_error("SU?"))?
        } else {
            1.0
        };

        // Interpolation factor (AGP only), else 1.0.
        let interpolation_factor = if model == ConexModel::Agp {
            parse_value_at(&write_read_raw(&handle, &query(controller_id, "IF?"))?, 3)
                .ok_or_else(|| parse_error("IF?"))?
        } else {
            1.0
        };

        // Full-step size / microsteps (PP and FCL200 only). C reads
        // `&inString_[4]` for the FRM?/FRS? replies.
        let (mut full_step_size, mut micro_steps_per_full_step) = (0.0, 0);
        if matches!(model, ConexModel::Pp | ConexModel::Fcl200) {
            micro_steps_per_full_step =
                parse_int_at(&write_read_raw(&handle, &query(controller_id, "FRM?"))?, 4)
                    .ok_or_else(|| parse_error("FRM?"))?;
            full_step_size =
                parse_value_at(&write_read_raw(&handle, &query(controller_id, "FRS?"))?, 4)
                    .ok_or_else(|| parse_error("FRS?"))?;
        }

        let step_size = compute_step_size(
            model,
            encoder_increment,
            interpolation_factor,
            full_step_size,
            micro_steps_per_full_step,
        );

        // Soft travel limits (already controller EGU).
        let low_limit = parse_value_at(&write_read_raw(&handle, &query(controller_id, "SL?"))?, 3)
            .ok_or_else(|| parse_error("SL?"))?;
        let high_limit = parse_value_at(&write_read_raw(&handle, &query(controller_id, "SR?"))?, 3)
            .ok_or_else(|| parse_error("SR?"))?;

        Ok(Self {
            handle,
            controller_id,
            model,
            step_size,
            low_limit,
            high_limit,
        })
    }

    fn write(&self, cmd: &str) -> AsynResult<()> {
        write_raw(&self.handle, cmd)
    }

    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        write_read_raw(&self.handle, cmd)
    }

    /// C `AG_CONEXAxis::getClosedLoop`.
    fn get_closed_loop(&self) -> AsynResult<bool> {
        let resp = self.write_read(&query(self.controller_id, "MM?"))?;
        parse_closed_loop(&resp).ok_or_else(|| parse_error("MM?"))
    }

    /// C `AG_CONEXAxis::move` velocity/accel preamble (model-gated).
    fn send_accel_and_velocity(&self, acceleration: f64, velocity: f64) -> AsynResult<()> {
        if self.model.supports_velocity() {
            self.write(&cmd_set_acceleration(
                self.controller_id,
                acceleration * self.step_size,
            ))?;
            self.write(&cmd_set_velocity(
                self.controller_id,
                velocity * self.step_size,
            ))?;
        }
        Ok(())
    }
}

impl AsynMotor for ConexAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.send_accel_and_velocity(acceleration, velocity)?;
        self.write(&cmd_move_absolute(
            self.controller_id,
            position * self.step_size,
        ))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C `move()` has a native relative branch (PR); override the trait's
        // poll-then-absolute default to match.
        self.send_accel_and_velocity(acceleration, velocity)?;
        self.write(&cmd_move_relative(
            self.controller_id,
            distance * self.step_size,
        ))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C `moveVelocity`: no jog command; move almost to the soft limit in
        // the jog direction. Limits are already controller EGU.
        let position = if velocity > 0.0 {
            self.high_limit - self.step_size
        } else {
            self.low_limit + self.step_size
        };
        self.write(&cmd_move_absolute(self.controller_id, position))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `home()`: reset to unreferenced state, then home. The blocking
        // sleeps mirror C `epicsThreadSleep` and run in the same blocking
        // context as the driver's serial I/O.
        self.write(&cmd_reset(self.controller_id))?;
        thread::sleep(Duration::from_secs(1));

        if self.model == ConexModel::Dl {
            self.write(&cmd_dl_init(self.controller_id))?;
            thread::sleep(Duration::from_secs(5));
        }

        // CONEX-PP: set home velocity (unscaled, as C) and force minus-EOR
        // home type.
        if self.model == ConexModel::Pp {
            self.write(&cmd_home_velocity(self.controller_id, velocity))?;
            self.write(&cmd_home_type(self.controller_id, HOME_TYPE_MINUS_EOR))?;
        }

        self.write(&cmd_home(self.controller_id))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.write(&cmd_stop(self.controller_id))
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C `setPosition` has no functional body — intentional no-op.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.write(&cmd_set_closed_loop(self.controller_id, enable))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        // C `setPGain`/`setIGain`/`setDGain`: gated to AGP/CC/DL; briefly drop
        // closed loop, write the scaled gain, then restore closed loop.
        if !self.model.supports_pid() {
            return Ok(());
        }
        let max = self.model.gain_max();
        let was_closed = self.get_closed_loop()?;
        self.write(&cmd_set_closed_loop(self.controller_id, false))?;

        let cmd = match kind {
            PidGainKind::Proportional => cmd_set_gain(self.controller_id, "KP", gain * max.kp),
            PidGainKind::Integral => cmd_set_gain(self.controller_id, "KI", gain * max.ki),
            PidGainKind::Derivative => match self.model {
                // Parity: C `setDGain` emits `KI` (with `KDMax`) for CC/DL.
                ConexModel::Cc | ConexModel::Dl => {
                    cmd_set_gain(self.controller_id, "KI", gain * max.kd)
                }
                // AGP repurposes D gain as the low-pass filter frequency (LF).
                ConexModel::Agp => cmd_set_gain(self.controller_id, "LF", gain * max.lf),
                _ => return Ok(()),
            },
        };
        self.write(&cmd)?;

        if was_closed {
            self.write(&cmd_set_closed_loop(self.controller_id, true))?;
        }
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Position (TP): controller EGU → raw steps via step_size.
        let position_egu = parse_value_at(&self.write_read(&query(self.controller_id, "TP"))?, 3)
            .ok_or_else(|| parse_error("TP"))?;
        let position = position_egu / self.step_size;

        // Status (TS): done + limits (model-dependent parse).
        let status = parse_ts(
            self.model,
            &self.write_read(&query(self.controller_id, "TS"))?,
        )
        .ok_or_else(|| parse_error("TS"))?;

        // Power-on = closed-loop enabled (MM?).
        let powered = self.get_closed_loop()?;

        Ok(MotorStatus {
            position,
            encoder_position: position,
            done: status.done,
            moving: !status.done,
            low_limit: status.low_limit,
            high_limit: status.high_limit,
            powered,
            gain_support: self.model.is_cc_or_agp(),
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_detected_from_firmware_string() {
        assert_eq!(
            ConexModel::from_version("CONEX-AGP-blah"),
            Some(ConexModel::Agp)
        );
        assert_eq!(
            ConexModel::from_version("CONEX-CC v2"),
            Some(ConexModel::Cc)
        );
        assert_eq!(ConexModel::from_version("Conex PP"), Some(ConexModel::Pp));
        assert_eq!(
            ConexModel::from_version("FC series"),
            Some(ConexModel::Fcl200)
        );
        assert_eq!(
            ConexModel::from_version("DL something"),
            Some(ConexModel::Dl)
        );
        assert_eq!(ConexModel::from_version("unknown box"), None);
    }

    #[test]
    fn agp_and_cc_precede_dl_in_detection() {
        // "CONEX-CC" contains no "DL", but a stray "DL" substring must not
        // outrank a real CONEX match — order mirrors the C strstr chain.
        assert_eq!(
            ConexModel::from_version("CONEX-AGP DL"),
            Some(ConexModel::Agp)
        );
    }

    #[test]
    fn step_size_is_model_specific() {
        // CC/AGP: encoderIncrement / interpolationFactor.
        assert_eq!(
            compute_step_size(ConexModel::Cc, 2.0e-5, 1.0, 0.0, 0),
            2.0e-5
        );
        assert_eq!(
            compute_step_size(ConexModel::Agp, 1.0e-3, 4.0, 0.0, 0),
            2.5e-4
        );
        // FCL200 in mm.
        assert_eq!(compute_step_size(ConexModel::Fcl200, 0.0, 1.0, 0.0, 0), 1.0);
        // DL fixed 1e-4.
        assert_eq!(compute_step_size(ConexModel::Dl, 0.0, 1.0, 0.0, 0), 0.0001);
        // PP: fullStepSize / microSteps / 1000.
        assert_eq!(
            compute_step_size(ConexModel::Pp, 0.0, 1.0, 2.0, 400),
            2.0 / 400.0 / 1000.0
        );
    }

    #[test]
    fn command_formatting_matches_c_sprintf() {
        assert_eq!(cmd_move_absolute(1, 5.0), "1PA5.000000");
        assert_eq!(cmd_move_relative(2, -1.25), "2PR-1.250000");
        assert_eq!(cmd_set_acceleration(1, 10.0), "1AC10.000000");
        assert_eq!(cmd_set_velocity(1, 2.5), "1VA2.500000");
        assert_eq!(cmd_stop(3), "3ST");
        assert_eq!(cmd_home(1), "1OR");
        assert_eq!(cmd_reset(1), "1RS");
        assert_eq!(cmd_set_closed_loop(1, true), "1MM1");
        assert_eq!(cmd_set_closed_loop(1, false), "1MM0");
        assert_eq!(cmd_home_type(1, HOME_TYPE_MINUS_EOR), "1HT4");
        assert_eq!(cmd_set_gain(1, "KP", 1500.0), "1KP1500.000000");
        assert_eq!(query(1, "TP"), "1TP");
        assert_eq!(query(2, "SU?"), "2SU?");
    }

    #[test]
    fn parse_ts_non_dl_moving_and_ready_states() {
        // "1TS" + 4 error chars (abcd) + 2 state chars. State 0x1e / 0x28 =
        // moving; 0x33 (READY) = done.
        assert_eq!(
            parse_ts(ConexModel::Cc, "1TS00001e"),
            Some(TsStatus {
                done: false,
                low_limit: false,
                high_limit: false,
            })
        );
        assert_eq!(
            parse_ts(ConexModel::Cc, "1TS000028"),
            Some(TsStatus {
                done: false,
                low_limit: false,
                high_limit: false,
            })
        );
        assert_eq!(
            parse_ts(ConexModel::Cc, "1TS000033"),
            Some(TsStatus {
                done: true,
                low_limit: false,
                high_limit: false,
            })
        );
    }

    #[test]
    fn parse_ts_dl_uses_last_two_state_chars() {
        // DL: "1TS" + 6 chars + 2 state chars; moving when state == 0x3c.
        assert_eq!(
            parse_ts(ConexModel::Dl, "1TS0000003c"),
            Some(TsStatus {
                done: false,
                low_limit: false,
                high_limit: false,
            })
        );
        assert_eq!(
            parse_ts(ConexModel::Dl, "1TS00000032"),
            Some(TsStatus {
                done: true,
                low_limit: false,
                high_limit: false,
            })
        );
    }

    #[test]
    fn parse_ts_rejects_reply_too_short_to_hold_state() {
        assert_eq!(parse_ts(ConexModel::Cc, "1TS12"), None);
        assert_eq!(parse_ts(ConexModel::Cc, "garbage"), None);
    }

    #[test]
    fn parse_closed_loop_reads_mm_state_range() {
        // Closed loop for state in 0x1e..=0x34.
        assert_eq!(parse_closed_loop("1MM1e"), Some(true));
        assert_eq!(parse_closed_loop("1MM34"), Some(true));
        assert_eq!(parse_closed_loop("1MM0a"), Some(false));
        assert_eq!(parse_closed_loop("1MM3c"), Some(false));
        assert_eq!(parse_closed_loop("1XX"), None);
    }
}
