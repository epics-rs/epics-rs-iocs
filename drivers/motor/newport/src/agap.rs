//! Newport CONEX-AGAP two-axis piezo gonio controller driver.
//!
//! Ported from `motorNewport/newportApp/src/AGAP_CONEX.cpp`
//! (`AGAP_CONEXController` / `AGAP_CONEXAxis`). Unlike the single-axis SMC100
//! and CONEX drivers, one AGAP controller drives **two** axes (named `U` and
//! `V`) over a single serial line sharing one controller ID. Per-axis commands
//! carry the axis letter (`PA`/`PR`/`TP`/`SL`/`SR`/`KP`/`KI`); a few are
//! controller-wide and carry no letter (`TS` moving status, `MM`/`MM?` closed
//! loop, `LF` low-pass filter). The two axes therefore share moving and
//! power-on status — this is faithful to the C driver.
//!
//! ## Shared-serial concurrency
//!
//! Each axis is an independent [`AsynMotor`] with its own motor record and poll
//! loop, but all axes of a controller share one [`AgapController`] behind an
//! `Arc<Mutex<..>>`. Every axis operation locks the controller for the whole
//! write→read exchange, so a second axis cannot interleave a write between the
//! first axis's command and its reply — the analogue of the C
//! `asynMotorController` lock plus `pasynOctetSyncIO->writeRead`.
//!
//! ## Units
//!
//! The AGAP operates in mm; the C driver fixes `stepSize_ = 1e-4` so the motor
//! record's integer step readback keeps four decimals. Outgoing positions are
//! multiplied by `STEP_SIZE`, the polled position divided back.

use std::sync::{Arc, Mutex};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::util::{leading_hex, parse_value_at};

/// Response buffer size for a single controller reply.
const READ_BUF: usize = 256;

/// Command line terminator (CR/LF); this driver owns framing (see
/// [`crate::conex`] for rationale).
const TERMINATOR: &[u8] = b"\r\n";

/// Fixed controller EGU-per-step (C `stepSize_ = 0.0001`): AGAP is in mm,
/// scaled by 1e4 so the motor record keeps four decimals.
const STEP_SIZE: f64 = 0.0001;

/// Closed-loop gain maxima (C constructor: `KPMax_ = KIMax_ = LFMax_ = 31`).
const KP_MAX: f64 = 31.0;
const KI_MAX: f64 = 31.0;
const LF_MAX: f64 = 31.0;

fn agap_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

fn parse_error(what: &str) -> AsynError {
    agap_err(format!("AGAP: could not parse {what} response"))
}

/// C `AGAP_CONEXAxis` constructor: axis 0 → `'U'`, otherwise `'V'`. Also used
/// by the IOC create command to form the per-axis DTYP key.
pub(crate) fn axis_name(axis_index: u8) -> char {
    if axis_index == 0 { 'U' } else { 'V' }
}

/// Number of axes on an AGAP controller (fixed at 2: `U` and `V`).
pub(crate) const NUM_AXES: u8 = 2;

/// Parse the controller-wide `TS` reply into the moving flag. C
/// `AGAP_CONEXAxis::poll`: `sscanf(inString_, "%*dTS%*4c%x", &status)`, then
/// moving when `status & 0xff` is `0x28`, `0x29`, or `0x46`.
fn parse_moving(resp: &str) -> Option<bool> {
    let idx = resp.find("TS")?;
    let status = leading_hex(resp.get(idx + 2 + 4..)?)?;
    let state = status & 0xff;
    Some(state == 0x28 || state == 0x29 || state == 0x46)
}

/// Parse the controller-wide `MM?` reply into the closed-loop (power-on) flag.
/// C `AGAP_CONEXAxis::getClosedLoop`: `sscanf(inString_, "%*dMM%x", &status)`,
/// closed-loop when the state is in `0x28..=0x36`.
fn parse_closed_loop(resp: &str) -> Option<bool> {
    let idx = resp.find("MM")?;
    let status = leading_hex(&resp[idx + 2..])?;
    Some((0x28..=0x36).contains(&status))
}

/// Shared controller endpoint: owns the serial handle and controller ID. Held
/// behind a mutex so per-axis write→read exchanges stay atomic on the shared
/// line. Methods take `&self`; the caller holds the lock.
pub struct AgapController {
    handle: SyncIOHandle,
    controller_id: i32,
}

impl AgapController {
    /// Connect and identify an AGAP controller: read the firmware version
    /// (`VE`) and verify it reports `CONEX-AGAP`. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle, controller_id: i32) -> AsynResult<Self> {
        let ctrl = Self {
            handle,
            controller_id,
        };
        // C reads `&inString_[4]` for the VE reply.
        let version_resp = ctrl.write_read(&format!("{controller_id}VE"))?;
        let version = version_resp.get(4..).unwrap_or("");
        if !version.contains("CONEX-AGAP") {
            return Err(agap_err(format!(
                "AGAP: unknown model, firmware=\"{version}\""
            )));
        }
        Ok(ctrl)
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    fn write(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let reply = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&reply).into_owned())
    }

    /// C `getClosedLoop`. Caller holds the controller lock.
    fn read_closed_loop(&self) -> AsynResult<bool> {
        let resp = self.write_read(&format!("{}MM?", self.controller_id))?;
        parse_closed_loop(&resp).ok_or_else(|| parse_error("MM?"))
    }
}

/// One AGAP axis (`U` or `V`) sharing a controller. Implements [`AsynMotor`].
pub struct AgapAxis {
    controller: Arc<Mutex<AgapController>>,
    /// `'U'` (axis 0) or `'V'` (axis 1) — the letter embedded in per-axis
    /// commands (C `axisName_`).
    axis_name: char,
}

impl AgapAxis {
    /// Construct axis `axis_index` (0 → `U`, 1 → `V`) on a shared controller,
    /// running the C `AGAP_CONEXAxis` constructor query sequence. Performs
    /// blocking serial I/O under the controller lock.
    pub fn new(controller: Arc<Mutex<AgapController>>, axis_index: u8) -> AsynResult<Self> {
        let axis_name = axis_name(axis_index);

        {
            // C reads stage ID (ID?), system resolution (SU?) and the per-axis
            // soft limits (SL/SR) here, but stores them only for report(),
            // which has no analogue in this driver. Issue the queries for
            // startup wire fidelity and discard the replies; stepSize is the
            // fixed 1e-4 constant regardless.
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let id = ctrl.controller_id;
            let _ = ctrl.write_read(&format!("{id}ID?"))?;
            let _ = ctrl.write_read(&format!("{id}SU?"))?;
            let _ = ctrl.write_read(&format!("{id}SL{axis_name}?"))?;
            let _ = ctrl.write_read(&format!("{id}SR{axis_name}?"))?;
        }

        Ok(Self {
            controller,
            axis_name,
        })
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, AgapController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for AgapAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        // AGAP move sends no AC/VA preamble — position only.
        ctrl.write(&format!(
            "{}PA{}{:.6}",
            ctrl.controller_id,
            self.axis_name,
            position * STEP_SIZE
        ))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C `move()` has a native relative branch (PR); override the trait's
        // poll-then-absolute default.
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{}PR{}{:.6}",
            ctrl.controller_id,
            self.axis_name,
            distance * STEP_SIZE
        ))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C `moveVelocity`: JA with the raw velocity (unscaled by stepSize).
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{}JA{}{:.6}",
            ctrl.controller_id, self.axis_name, velocity
        ))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `home()` is a no-op returning success.
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&format!("{}ST{}", ctrl.controller_id, self.axis_name))
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C `setPosition` is a no-op.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C `setClosedLoop`: controller-wide MM (no axis letter).
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{}MM{}",
            ctrl.controller_id,
            if enable { 1 } else { 0 }
        ))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let id = ctrl.controller_id;
        match kind {
            // C setPGain/setIGain: drop closed loop (controller-wide MM),
            // write the per-axis gain, restore closed loop. Because MM is
            // controller-wide, this toggles closed loop for BOTH axes —
            // faithful to the C driver.
            PidGainKind::Proportional | PidGainKind::Integral => {
                let (letter, max) = match kind {
                    PidGainKind::Proportional => ("KP", KP_MAX),
                    _ => ("KI", KI_MAX),
                };
                let was_closed = ctrl.read_closed_loop()?;
                ctrl.write(&format!("{id}MM0"))?;
                ctrl.write(&format!("{id}{letter}{}{:.6}", self.axis_name, gain * max))?;
                if was_closed {
                    ctrl.write(&format!("{id}MM1"))?;
                }
                Ok(())
            }
            // C setDGain: low-pass filter frequency LF, controller-wide (no
            // axis letter), with no closed-loop save/restore.
            PidGainKind::Derivative => ctrl.write(&format!("{id}LF{:.6}", gain * LF_MAX)),
        }
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let id = ctrl.controller_id;

        // Position (per-axis TP): "1TPU<value>", value at offset 4.
        let position_egu =
            parse_value_at(&ctrl.write_read(&format!("{id}TP{}", self.axis_name))?, 4)
                .ok_or_else(|| parse_error("TP"))?;
        let position = position_egu / STEP_SIZE;

        // Moving status (controller-wide TS) and power-on (controller-wide MM?)
        // are shared by both axes.
        let moving =
            parse_moving(&ctrl.write_read(&format!("{id}TS"))?).ok_or_else(|| parse_error("TS"))?;
        let powered = ctrl.read_closed_loop()?;

        Ok(MotorStatus {
            position,
            done: !moving,
            moving,
            powered,
            // C AGAP always reports both limits clear and does not advertise
            // gain support (leaves motorStatusGainSupport unset).
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_moving_reads_controller_state_byte() {
        // "1TS" + 4 error chars + 2 state chars. 0x28/0x29/0x46 = moving.
        assert_eq!(parse_moving("1TS000028"), Some(true));
        assert_eq!(parse_moving("1TS000029"), Some(true));
        assert_eq!(parse_moving("1TS000046"), Some(true));
        // Any other state = not moving (done).
        assert_eq!(parse_moving("1TS000033"), Some(false));
        assert_eq!(parse_moving("1TS000036"), Some(false));
    }

    #[test]
    fn parse_moving_rejects_short_or_junk_reply() {
        assert_eq!(parse_moving("1TS12"), None);
        assert_eq!(parse_moving("garbage"), None);
    }

    #[test]
    fn parse_closed_loop_reads_mm_state_range() {
        // Closed loop for state in 0x28..=0x36.
        assert_eq!(parse_closed_loop("1MM28"), Some(true));
        assert_eq!(parse_closed_loop("1MM36"), Some(true));
        assert_eq!(parse_closed_loop("1MM27"), Some(false));
        assert_eq!(parse_closed_loop("1MM3c"), Some(false));
        assert_eq!(parse_closed_loop("1XX"), None);
    }

    #[test]
    fn move_command_formatting_matches_c_sprintf() {
        // Verify the per-axis command shape (id, axis letter, %f 6-decimals).
        let id = 1;
        assert_eq!(
            format!("{id}PA{}{:.6}", 'U', 5.0 * STEP_SIZE),
            "1PAU0.000500"
        );
        assert_eq!(
            format!("{id}PR{}{:.6}", 'V', -10.0 * STEP_SIZE),
            "1PRV-0.001000"
        );
        assert_eq!(format!("{id}ST{}", 'U'), "1STU");
        assert_eq!(format!("{id}KP{}{:.6}", 'U', 0.5 * KP_MAX), "1KPU15.500000");
        // Controller-wide (no axis letter).
        assert_eq!(format!("{id}LF{:.6}", 0.5 * LF_MAX), "1LF15.500000");
        assert_eq!(format!("{id}MM{}", 1), "1MM1");
    }

    #[test]
    fn axis_index_maps_to_u_and_v() {
        assert_eq!(axis_name(0), 'U');
        assert_eq!(axis_name(1), 'V');
    }
}
