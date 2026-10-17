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
//! Positions cross the motor boundary in record steps, as in C. `PA`/`PR`
//! send `steps * STEP_SIZE` and `TP` reads back `value / STEP_SIZE`; the
//! IOC's MRES must therefore be `STEP_SIZE`. `JA` sends the record velocity
//! unscaled, as C does.

use std::sync::{Arc, Mutex};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;

use crate::conex::LINUX_WRITE_DELAY;
use crate::util::{leading_hex, parse_value_at};

/// Response buffer size for a single controller reply.
const READ_BUF: usize = 256;

/// C's fixed `stepSize_` (AGAP_CONEX.cpp:219): wire units per record step.
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

/// C `AGAP_CONEXAxis` constructor: axis 0 → `'U'`, otherwise `'V'`.
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
    /// Firmware string from `VE` (C `controllerVersion_`).
    version: String,
    /// C `pasynUserSelf`, which the axes print on too.
    trace: MotorTrace,
}

impl AgapController {
    /// Connect and identify an AGAP controller: read the firmware version
    /// (`VE`). Performs blocking serial I/O. `None` when `VE` fails, where C
    /// returns before creating the axes or the poller.
    pub fn new(handle: SyncIOHandle, controller_id: i32, trace: MotorTrace) -> Option<Self> {
        let mut ctrl = Self {
            handle,
            controller_id,
            version: String::new(),
            trace,
        };
        // C reads `&inString_[4]` for the VE reply.
        let Ok(version_resp) = ctrl.write_read(&format!("{controller_id}VE")) else {
            ctrl.trace.print_self(
                TraceMask::ERROR,
                format_args!(
                    "AGAP_CONEXController::AGAP_CONEXController: cannot read version information from AGAP_CONEX controller\n"
                ),
            );
            return None;
        };
        ctrl.version = version_resp.get(4..).unwrap_or("").to_string();
        Some(ctrl)
    }

    /// C `writeCONEX`: every write, failed or not, is followed by the Linux
    /// `LINUX_WRITE_DELAY` before the next command.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        let status = self.handle.write_octet(0, cmd.as_bytes()).map(drop);
        if cfg!(target_os = "linux") {
            std::thread::sleep(LINUX_WRITE_DELAY);
        }
        status
    }

    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        // C writeReadController: pasynOctetSyncIO->writeRead.
        let reply = self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?;
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
    pub fn new(controller: Arc<Mutex<AgapController>>, axis_index: u8) -> Self {
        let axis_name = axis_name(axis_index);

        {
            // C reads stage ID (ID?), system resolution (SU?) and the per-axis
            // soft limits (SL/SR) here, but stores them only for report(),
            // which has no analogue in this driver. Issue the queries for
            // startup wire fidelity and discard the replies (C ignores their
            // status); stepSize is the fixed 1e-4 constant regardless. An
            // unknown model returns before the queries and is still polled.
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            if ctrl.version.contains("CONEX-AGAP") {
                let id = ctrl.controller_id;
                let _ = ctrl.write_read(&format!("{id}ID?"));
                let _ = ctrl.write_read(&format!("{id}SU?"));
                let _ = ctrl.write_read(&format!("{id}SL{axis_name}?"));
                let _ = ctrl.write_read(&format!("{id}SR{axis_name}?"));
            } else {
                ctrl.trace.print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "AGAP_CONEXAxis::AGAP_CONEXAxis: unknown model, firmware string={}\n",
                        ctrl.version
                    ),
                );
            }
        }

        Self {
            controller,
            axis_name,
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, AgapController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for AgapAxis {
    /// C `AGAP_CONEXAxis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (AGAP_CONEX.cpp).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
    }

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
        let position = parse_value_at(&ctrl.write_read(&format!("{id}TP{}", self.axis_name))?, 4)
            .ok_or_else(|| parse_error("TP"))?
            / STEP_SIZE;

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
        assert_eq!(format!("{id}PA{}{:.6}", 'U', 0.0005), "1PAU0.000500");
        assert_eq!(format!("{id}PR{}{:.6}", 'V', -0.001), "1PRV-0.001000");
        assert_eq!(format!("{id}ST{}", 'U'), "1STU");
        assert_eq!(format!("{id}KP{}{:.6}", 'U', 0.5 * KP_MAX), "1KPU15.500000");
        // Controller-wide (no axis letter).
        assert_eq!(format!("{id}LF{:.6}", 0.5 * LF_MAX), "1LF15.500000");
        assert_eq!(format!("{id}MM{}", 1), "1MM1");
    }

    #[test]
    fn record_steps_scale_by_step_size() {
        use motor_common::testing::LineRig;
        let (line, io) = LineRig::new("agap_steps", |last| {
            match last {
                "1VE" => "1VE CONEX-AGAP 1.0",
                "1TPV" => "1TPV0.25",
                "1TS" => "1TS000033",
                "1MM?" => "1MM33",
                _ => "",
            }
            .to_string()
        });
        let trace = MotorTrace::new("agap_steps", 0);
        let ctrl = Arc::new(Mutex::new(AgapController::new(io, 1, trace).unwrap()));
        let mut axis = AgapAxis::new(ctrl, 1);
        line.sent();
        let user = AsynUser::default();
        axis.move_absolute(&user, 15000.0, 0.0, 5000.0, 2500.0)
            .unwrap();
        axis.move_relative(&user, -100.0, 0.0, 5000.0, 2500.0)
            .unwrap();
        axis.move_velocity(&user, 0.0, 5000.0, 2500.0).unwrap();
        assert_eq!(
            line.sent(),
            ["1PAV1.500000", "1PRV-0.010000", "1JAV5000.000000"]
        );
        assert_eq!(axis.poll(&user).unwrap().position, 2500.0);
    }

    /// AGAP_CONEX.cpp `writeCONEX`: the next command waits out
    /// `LINUX_WRITE_DELAY` after a write.
    #[cfg(target_os = "linux")]
    #[test]
    fn a_write_holds_the_line_for_the_write_delay() {
        use motor_common::testing::LineRig;
        let (_line, io) = LineRig::new("agap_delay", |last| {
            match last {
                "1VE" => "1VE CONEX-AGAP 1.0",
                _ => "",
            }
            .to_string()
        });
        let ctrl = Arc::new(Mutex::new(
            AgapController::new(io, 1, MotorTrace::new("agap_delay", 0)).unwrap(),
        ));
        let mut axis = AgapAxis::new(ctrl, 0);
        let start = std::time::Instant::now();
        axis.stop(&AsynUser::default(), 0.0).unwrap();
        assert!(start.elapsed() >= LINUX_WRITE_DELAY);
    }

    #[test]
    fn axis_index_maps_to_u_and_v() {
        assert_eq!(axis_name(0), 'U');
        assert_eq!(axis_name(1), 'V');
    }
}
