//! Aerotech Ensemble motor controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorAerotech/aerotechApp/src/drvEnsembleAsyn.cc` (the
//! `motorAxisDrvSET_t` "motorAxis" API over `pasynOctetSyncIO`). The Ensemble
//! speaks an ASCII command language over serial or TCP; each reply begins with a
//! status char — `%` ACK, `!` NAK, `#` fault, `$` timeout — followed by the
//! payload. This port drives the per-axis [`AsynMotor`] boundary directly,
//! replacing the C controller-central poller thread with a per-axis poll.
//!
//! ## Units
//!
//! As C, `stepSize = 1 / CountsPerUnit` scales every outgoing record step
//! value into the Ensemble's user units and divides every readback, and
//! values are formatted to `maxDigits` places derived from it.
//!
//! ## Not modeled (documented)
//!
//! - Profile / trajectory moves: drvMotorAsyn never calls `profileMove` /
//!   `triggerProfile`.
//!
//! ## Deviations from C (documented)
//!
//! - `motorAxisHome` in C builds the `HomeRampRate` SETPARM string but never
//!   sends it (the next `sprintf` overwrites the buffer with no intervening
//!   write) — the home acceleration is silently dropped. This port sends it,
//!   honouring the evident intent.
//! - `home` requires the vendor `HomeAsync.bcx` program to be loaded on the
//!   controller (task 5), exactly as the C driver does.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, StatusWrite,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_libcom_rs::cfmt::CFloat;
use motor_common::model2::AxisLog;
use motor_common::util::{atof, atoi, atol};

use crate::axis_report::AxisMembers;
use crate::exchange::Exchange;

/// Reply read buffer size (C `BUFFER_SIZE`).
const READ_BUF: usize = 128;

/// Port EOS set at connect (C `ASCII_EOS_STR`, drvEnsembleAsyn.cc:935-936).
pub const INPUT_EOS: &[u8] = b"\n";
pub const OUTPUT_EOS: &[u8] = b"\n";

/// Reply status characters (C `ASCII_*_CHAR`).
const ACK: u8 = b'%';
const FAULT: u8 = b'#';

/// Axis status word bits (C `Axis_Status.Bits`, little-endian layout).
const AXIS_ENABLED: u32 = 1 << 0;
const HOME_CYCLE_COMPLETE: u32 = 1 << 1;
const MOVE_ACTIVE: u32 = 1 << 3;
const MOTION_CCW: u32 = 1 << 9;
const CW_LIMIT: u32 = 1 << 22;
const CCW_LIMIT: u32 = 1 << 23;
const HOME_LIMIT: u32 = 1 << 24;

/// End-of-travel switch-level bits (C `Switch_Level.Bits`).
const CCW_EOT_SW_STATE: u32 = 1 << 1;
const CW_EOT_SW_STATE: u32 = 1 << 2;

/// Ensemble parameter IDs (C `EnsembleParameterId.h`; the `(0 << 24)` high byte
/// is always zero for these).
const PARAM_REVERSE_MOTION_DIRECTION: i32 = 1;
const PARAM_COUNTS_PER_UNIT: i32 = 2;
const PARAM_POSITION_FEEDBACK_TYPE: i32 = 47;
const PARAM_END_OF_TRAVEL_LIMIT_SETUP: i32 = 61;
const PARAM_ABORT_DECEL_RATE: i32 = 73;
const PARAM_HOME_SETUP: i32 = 75;
const PARAM_HOME_SPEED: i32 = 76;
const PARAM_HOME_OFFSET: i32 = 77;
const PARAM_HOME_RAMP_RATE: i32 = 78;
const PARAM_AXIS_NAME: i32 = 140;

/// Travel-limit fault mask (C `AXISFAULTBITS_Cw/CcwEndOfTravelLimitFaultBit`).
const TRAVEL_LIMIT_FAULT_MASK: i32 = (1 << 2) | (1 << 3);

/// Max axes probed on a controller (C `ENSEMBLE_MAX_AXES`).
pub const ENSEMBLE_MAX_AXES: i32 = 10;

fn ens_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// C `%.*f` with the axis `maxDigits`.
fn fmt(v: f64, digits: usize) -> String {
    format!("{:.digits$}", CFloat(v))
}

/// C's `stepSize` from a `CountsPerUnit` reply, 1 when it was refused.
fn step_size(counts_per_unit: Option<String>) -> f64 {
    counts_per_unit.map_or(1.0, |s| 1.0 / atof(&s))
}

/// C `maxDigits`: two places past the step's, at least one.
fn max_digits(step_size: f64) -> usize {
    ((-step_size.abs().log10()) as i32 + 2).max(1) as usize
}

/// An Ensemble controller endpoint owning the asyn octet handle, shared by its
/// axes behind a mutex so command/reply pairs stay atomic (C
/// `sendReceiveMutex`).
pub struct EnsembleController {
    io: Exchange,
    /// C `inputBuff`: the last reply. A failed exchange leaves it, and C's
    /// callers that ignore the status read it anyway.
    input: String,
}

impl EnsembleController {
    /// Wrap a connected octet handle; `serial_port` and `motor_port` are
    /// the asyn users C prints the exchange errors on.
    pub fn new(handle: SyncIOHandle, serial_port: &str, motor_port: &str) -> Self {
        Self {
            io: Exchange::new(
                handle,
                "drvEnsembleAsyn",
                READ_BUF,
                Some(FAULT),
                serial_port,
                motor_port,
            ),
            input: String::new(),
        }
    }

    /// C `sendAndReceive`: fails only on I/O; a NAK or fault reply is the
    /// caller's to check. Its ACK-then-EOS re-read (drvEnsembleAsyn.cc:1083)
    /// never matches a reply the input EOS already stripped.
    pub fn send(&mut self, cmd: &str) -> AsynResult<()> {
        self.input = self.io.write_read(cmd)?;
        Ok(())
    }

    /// Whether the last reply is an ACK.
    fn ack(&self) -> bool {
        self.input.as_bytes().first() == Some(&ACK)
    }

    /// C `&inputBuff[1]`, whatever the status char.
    fn rest(&self) -> &str {
        self.input.get(1..).unwrap_or("")
    }

    /// `GETPARM(@axis, param)`'s value when the reply is an ACK.
    pub fn get_param(&mut self, axis: i32, param: i32) -> Option<String> {
        let _ = self.send(&format!("GETPARM(@{axis}, {param})"));
        self.ack().then(|| self.rest().to_owned())
    }

    /// Whether an axis exists (its `AxisName` parameter ACKs).
    pub fn axis_exists(&mut self, axis: i32) -> bool {
        self.get_param(axis, PARAM_AXIS_NAME).is_some()
    }

    /// Comms check: an (invalid) `NONE` command that any reply answers (C
    /// config `do { NONE } while` loop).
    pub fn ping(&mut self) -> AsynResult<()> {
        self.send("NONE")
    }

    /// Prevent the ASCII interpreter from blocking during moves (C
    /// `WAIT MODE NOWAIT`, sent at config and on every enable).
    pub fn wait_mode_nowait(&mut self) -> AsynResult<()> {
        self.send("WAIT MODE NOWAIT")
    }
}

/// One Ensemble axis sharing a controller. Implements [`AsynMotor`].
pub struct EnsembleAxis {
    controller: Arc<Mutex<EnsembleController>>,
    /// Controller axis number (the `@n` address).
    axis: i32,
    /// The axis's place among the controller's, C `itera`.
    index: i32,
    /// C `lastFault`, the AXISFAULT word that last raised `problem`.
    last_fault: i32,
    /// C's motorParam view of the axis: the poller's sets, kept when a poll
    /// bails, and `motorAxisProblem` cleared by DISABLE after a travel-limit
    /// fault.
    status: MotorStatus,
    /// The status a failed poll left, for the next post.
    failed: Option<MotorStatus>,
    status_writes: Vec<StatusWrite>,
    /// `HomeSetup` word; bit 0 is the home direction, updated by [`Self::home`].
    home_direction: u32,
    /// `ReverseMotionDirection` parameter.
    reverse_direc: bool,
    /// `EndOfTravelLimitSetup` word (limit-switch active levels).
    swconfig: u32,
    /// C `fabs(stepSize)`: user units per record step.
    step: f64,
    /// C `maxDigits`.
    digits: usize,
    report: Arc<AxisMembers>,
    log: Arc<AxisLog>,
}

impl EnsembleAxis {
    /// Construct axis `axis` and probe its feedback type, home/limit setup and
    /// reverse-direction parameters, then set `RAMP MODE RATE`.
    pub fn new(
        controller: Arc<Mutex<EnsembleController>>,
        axis: i32,
        index: i32,
    ) -> AsynResult<Self> {
        let (has_encoder, step_size, home_preset, home_direction, reverse_direc, swconfig) = {
            let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let has_encoder = ctrl
                .get_param(axis, PARAM_POSITION_FEEDBACK_TYPE)
                .is_some_and(|s| atoi(&s) > 0);
            let step_size = step_size(ctrl.get_param(axis, PARAM_COUNTS_PER_UNIT));
            let home_preset = ctrl
                .get_param(axis, PARAM_HOME_OFFSET)
                .map(|s| atof(&s))
                .unwrap_or(0.0);
            let home_direction = ctrl
                .get_param(axis, PARAM_HOME_SETUP)
                .map(|s| atol(&s) as u32)
                .unwrap_or(0);
            let swconfig = ctrl
                .get_param(axis, PARAM_END_OF_TRAVEL_LIMIT_SETUP)
                .map(|s| atol(&s) as u32)
                .unwrap_or(0);
            let _ = ctrl.send(&format!("RAMP MODE @{axis} RATE"));
            let reverse_direc = ctrl
                .get_param(axis, PARAM_REVERSE_MOTION_DIRECTION)
                .is_some_and(|s| atoi(&s) != 0);
            (
                has_encoder,
                step_size,
                home_preset,
                home_direction,
                reverse_direc,
                swconfig,
            )
        };

        Ok(Self {
            controller,
            axis,
            index,
            last_fault: 0,
            status: MotorStatus {
                has_encoder,
                gain_support: true,
                ..Default::default()
            },
            failed: None,
            status_writes: Vec::new(),
            home_direction,
            reverse_direc,
            swconfig,
            step: step_size.abs(),
            digits: max_digits(step_size),
            report: Arc::new(AxisMembers::new(
                axis.to_string(),
                step_size,
                max_digits(step_size),
                home_preset,
            )),
            log: Arc::default(),
        })
    }

    /// What C's `motorAxisReport` prints of this axis.
    pub fn report(&self) -> Arc<AxisMembers> {
        self.report.clone()
    }

    /// The user C's `PRINT(pAxis->logParam, ...)` lines go through.
    pub fn log(&self) -> Arc<AxisLog> {
        self.log.clone()
    }

    fn lock(&self) -> MutexGuard<'_, EnsembleController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A `motorAxisSetDouble` case C answers with only a TERROR line
    /// (drvEnsembleAsyn.cc:364-399).
    fn refuse(&self, line: &str) {
        self.log.print(TraceMask::ERROR, format_args!("{line}\n"));
    }

    /// A record step value in user units, formatted as C's `%.*f`.
    fn units(&self, v: f64) -> String {
        fmt(v * self.step, self.digits)
    }

    fn do_move(&mut self, position: f64, relative: bool, max_v: f64, accel: f64) -> AsynResult<()> {
        let axis = self.axis;
        let mode = if relative { "INC" } else { "ABS" };
        let mut ctrl = self.lock();
        ctrl.send(mode)?;
        if accel > 0.0 {
            let _ = ctrl.send(&format!("RAMP RATE {}", self.units(accel)));
        }
        ctrl.send(&format!(
            "LINEAR @{axis} {} F{}",
            self.units(position),
            self.units(max_v)
        ))
    }
}

impl EnsembleAxis {
    /// One axis of C `EnsemblePoller` (drvEnsembleAsyn.cc:713-848), setting
    /// [`Self::status`] in its order; an `Err` is its `continue` before
    /// `callCallback`.
    fn poll_status(&mut self) -> AsynResult<()> {
        let axis = self.axis;
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let status = &mut self.status;

        let sent = ctrl.send(&format!("AXISSTATUS(@{axis})"));
        if sent.is_err() || ctrl.input.len() <= 1 {
            status.comms_error = true;
            return sent.and(Err(ens_err("ensemble: AXISSTATUS reply too short")));
        }
        if !ctrl.ack() {
            return Err(ens_err(format!(
                "ensemble: AXISSTATUS refused: '{}'",
                ctrl.input
            )));
        }
        status.comms_error = false;
        let axis_status = atol(ctrl.rest()) as u32;
        let _ = ctrl.send("PLANESTATUS(0)");
        let move_active = (atoi(ctrl.rest()) & 0x01) != 0 || (axis_status & MOVE_ACTIVE) != 0;
        status.done = !move_active;
        status.moving = move_active;
        status.powered = (axis_status & AXIS_ENABLED) != 0;
        status.homed = (axis_status & HOME_CYCLE_COMPLETE) != 0;
        status.home = (axis_status & HOME_LIMIT) != 0;
        let motion_ccw = (axis_status & MOTION_CCW) != 0;
        status.direction = if self.reverse_direc {
            motion_ccw
        } else {
            !motion_ccw
        };
        // Limit switches: XOR the raw limit bit against its configured active
        // level, then map CW/CCW to high/low honouring the reverse-direction flag.
        let cw_sw_active =
            !(((axis_status & CW_LIMIT) != 0) ^ ((self.swconfig & CW_EOT_SW_STATE) != 0));
        let ccw_sw_active =
            !(((axis_status & CCW_LIMIT) != 0) ^ ((self.swconfig & CCW_EOT_SW_STATE) != 0));
        (status.high_limit, status.low_limit) = if self.reverse_direc {
            (ccw_sw_active, cw_sw_active)
        } else {
            (cw_sw_active, ccw_sw_active)
        };
        self.report.set_axis_status(axis_status);

        if let Err(e) = ctrl.send(&format!("PFBKPROG(@{axis})")) {
            status.comms_error = true;
            return Err(e);
        }
        let encoder = if ctrl.ack() { atof(ctrl.rest()) } else { 0.0 };
        status.encoder_position = encoder / self.step;
        let _ = ctrl.send(&format!("PCMDPROG(@{axis})"));
        status.position = atof(ctrl.rest()) / self.step;

        if let Err(e) = ctrl.send(&format!("AXISFAULT(@{axis})")) {
            status.comms_error = true;
            return Err(e);
        }
        if !ctrl.ack() {
            status.problem = true;
            status.comms_error = true;
            return Err(ens_err(format!(
                "ensemble: AXISFAULT refused: '{}'",
                ctrl.input
            )));
        }
        let fault = atoi(ctrl.rest());
        if fault == 0 {
            self.last_fault = 0;
            status.problem = false;
        } else if fault != self.last_fault {
            self.last_fault = fault;
            status.problem = true;
            self.log.print(
                TraceMask::ERROR,
                format_args!(
                    "EnsemblePoller: controller fault on axis={} fault=0x{fault:X}\n",
                    self.index
                ),
            );
        }

        if let Err(e) = ctrl.send(&format!("VFBK(@{axis})")) {
            status.comms_error = true;
            return Err(e);
        }
        status.velocity = atof(ctrl.rest()) / self.step;
        Ok(())
    }
}

impl AsynMotor for EnsembleAxis {
    /// C `EnsemblePoller` sets `motorAxisCommError` and `continue`s past
    /// `callCallback`, posting nothing (drvEnsembleAsyn.cc).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let axis = self.axis;
        let mut ctrl = self.lock();
        let _ = ctrl.send(&format!(
            "SETPARM @{axis}, {PARAM_ABORT_DECEL_RATE}, {}",
            self.units(acceleration)
        ));
        let _ = ctrl.send(&format!("RAMP RATE @{axis} {}", self.units(acceleration)));
        ctrl.send(&format!("FREERUN @{axis} {}", self.units(velocity)))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let axis = self.axis;
        // Adjust home direction for the reverse-direction parameter (C posdir).
        let posdir = forward == self.reverse_direc;
        let hparam = if posdir {
            self.home_direction | 1
        } else {
            self.home_direction & !1
        };
        self.home_direction = hparam;

        let mut ctrl = self.lock();
        if velocity > 0.0 {
            let _ = ctrl.send(&format!(
                "SETPARM @{axis}, {PARAM_HOME_SPEED}, {}",
                self.units(velocity)
            ));
        }
        if acceleration > 0.0 {
            // C builds this SETPARM and overwrites it unsent
            // (drvEnsembleAsyn.cc:572-574, :584); not copied.
            let _ = ctrl.send(&format!(
                "SETPARM @{axis}, {PARAM_HOME_RAMP_RATE}, {}",
                self.units(acceleration)
            ));
        }
        let _ = ctrl.send(&format!("SETPARM @{axis}, {PARAM_HOME_SETUP}, {hparam}"));
        // HomeAsync.bcx protocol: IGLOBAL(32)=1, IGLOBAL(33)=axis, run task 5.
        let _ = ctrl.send("IGLOBAL(32) = 1");
        let _ = ctrl.send(&format!("IGLOBAL(33) = {axis}"));
        ctrl.send("PROGRAM RUN 5, \"HomeAsync.bcx\"")
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let axis = self.axis;
        self.lock().send(&format!("ABORT @{axis}"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let axis = self.axis;
        let mut ctrl = self.lock();
        let sent = ctrl.send(&format!("POSOFFSET SET @{axis}, {}", self.units(position)));
        if !ctrl.ack() {
            return Err(ens_err(format!(
                "ensemble: POSOFFSET refused: '{}'",
                ctrl.input
            )));
        }
        sent
    }

    fn set_encoder_ratio(&mut self, _user: &AsynUser, _ratio: f64) -> AsynResult<()> {
        self.refuse("motorAxisSetDouble: Ensemble does not support setting encoder ratio");
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        self.refuse("Driver does not set Ensemble's Low Limit");
        Ok(())
    }

    fn set_high_limit(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        self.refuse("Driver does not set Ensemble's High Limit");
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        self.refuse(match kind {
            PidGainKind::Proportional => "Ensemble does not support setting proportional gain",
            PidGainKind::Integral => "Ensemble does not support setting integral gain",
            PidGainKind::Derivative => "Ensemble does not support setting derivative gain",
        });
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, closed_loop: bool) -> AsynResult<()> {
        let axis = self.axis;
        let mut ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        if closed_loop {
            // C refreshes stepSize on every enable (not maxDigits).
            if let Some(s) = ctrl.get_param(axis, PARAM_COUNTS_PER_UNIT) {
                self.step = (1.0 / atof(&s)).abs();
            }
            // Acknowledge any pending fault, then enable.
            let sent = ctrl.send(&format!("AXISFAULT @{axis}"));
            let fault = if sent.is_ok() && ctrl.ack() {
                atoi(ctrl.rest())
            } else {
                0
            };
            if fault != 0 {
                self.log.print(
                    TraceMask::ERROR,
                    format_args!("motorAxisSetInteger: FAULTACK = {fault:X}\n"),
                );
                let _ = ctrl.send(&format!("FAULTACK @{axis}"));
            }
            let _ = ctrl.send(&format!("ENABLE @{axis}"));
        } else {
            // C clears it so the user can jog off the limit (:434-435).
            if self.last_fault & TRAVEL_LIMIT_FAULT_MASK != 0 {
                self.status.problem = false;
                self.status_writes.push(StatusWrite::Problem(false));
            }
            let _ = ctrl.send(&format!("DISABLE @{axis}"));
        }
        // C's status is WAIT MODE NOWAIT's; PowerOn follows the request even
        // when ENABLE fails, to force a status update (:461-476).
        self.status.powered = closed_loop;
        ctrl.wait_mode_nowait()?;
        self.status_writes.push(StatusWrite::PowerOn(closed_loop));
        Ok(())
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let polled = self.poll_status();
        if polled.is_err() {
            self.failed = Some(self.status.clone());
        }
        polled.map(|()| self.status.clone())
    }

    fn take_failed_poll_status(&mut self) -> Option<MotorStatus> {
        self.failed.take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn max_digits_follows_the_step() {
        assert_eq!(max_digits(1.0), 2);
        assert_eq!(max_digits(0.001), 5);
        assert_eq!(max_digits(1e-4), 6);
        assert_eq!(max_digits(1000.0), 1);
        assert_eq!(fmt(1.5, 2), "1.50");
        // C `%.*f` of a non-finite value.
        assert_eq!(fmt(f64::NAN, 2), "nan");
        assert_eq!(fmt(f64::NEG_INFINITY, 2), "-inf");
    }

    #[test]
    fn step_size_is_one_for_a_refused_counts_per_unit() {
        assert_eq!(step_size(Some("1000".into())), 0.001);
        assert_eq!(step_size(None), 1.0);
    }

    #[test]
    fn axis_status_bit_positions() {
        assert_eq!(AXIS_ENABLED, 0x0000_0001);
        assert_eq!(HOME_CYCLE_COMPLETE, 0x0000_0002);
        assert_eq!(MOVE_ACTIVE, 0x0000_0008);
        assert_eq!(MOTION_CCW, 0x0000_0200);
        assert_eq!(CW_LIMIT, 0x0040_0000);
        assert_eq!(CCW_LIMIT, 0x0080_0000);
        assert_eq!(HOME_LIMIT, 0x0100_0000);
    }

    #[test]
    fn limit_switch_xor_mapping() {
        // Raw CW limit set, active-high configured (state bit 0) -> !(1 ^ 0) = 0.
        let status = CW_LIMIT;
        let swconfig = 0;
        let cw_active = !(((status & CW_LIMIT) != 0) ^ ((swconfig & CW_EOT_SW_STATE) != 0));
        assert!(!cw_active);
        // Raw CW limit set, active-low configured (state bit set) -> !(1 ^ 1) = 1.
        let swconfig = CW_EOT_SW_STATE;
        let cw_active = !(((status & CW_LIMIT) != 0) ^ ((swconfig & CW_EOT_SW_STATE) != 0));
        assert!(cw_active);
    }

    /// drvEnsembleAsyn.cc:364-399: the setters the Ensemble refuses print
    /// their TERROR line on the axis's drvAsynMotorConfigure user.
    #[epics_rs::base::epics_test]
    async fn refused_setters_print_on_the_axis_s_log_user() {
        use epics_rs::asyn::interfaces::motor::PidGainKind;
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        use epics_rs::base::server::iocsh::registry::ArgValue;
        use motor_common::model2::{Model2, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        static DRIVER: Model2 = Model2::new("ensLogTest", 0);
        let (_line, io) = LineRig::new("ensLogLine", |_| "%0".into());
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensLogLine", "0")));
        let axis = EnsembleAxis::new(controller, 0, 0).unwrap();
        let log = axis.log();
        let axis = Arc::new(Mutex::new(axis));
        let motor: Arc<Mutex<dyn AsynMotor>> = axis.clone();
        DRIVER.set_card(0, vec![motor], 100, 1000);
        DRIVER.set_axis_logs(0, vec![log]);
        let args = vec![
            ArgValue::String("ensLog".into()),
            ArgValue::String("ensLogTest".into()),
            ArgValue::Int(0),
            ArgValue::Int(1),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&DRIVER]), args).await;

        let path = std::env::temp_dir().join(format!("ens-log-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some("ensLog"), TraceMask::ERROR);
        trace.set_trace_file(Some("ensLog"), TraceFile::File(file.clone()));
        trace.set_device_trace_file("ensLog", 0, TraceFile::File(file.clone()));
        let user = AsynUser::new(0);
        {
            let mut axis = axis.lock().unwrap();
            axis.set_encoder_ratio(&user, 1.0).unwrap();
            axis.set_low_limit(&user, -1.0).unwrap();
            axis.set_high_limit(&user, 1.0).unwrap();
            axis.set_pid_gain(&user, PidGainKind::Proportional, 1.0)
                .unwrap();
            axis.set_pid_gain(&user, PidGainKind::Integral, 1.0)
                .unwrap();
            axis.set_pid_gain(&user, PidGainKind::Derivative, 1.0)
                .unwrap();
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        // Each line follows asynPrint's timestamp.
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(
            lines,
            [
                "motorAxisSetDouble: Ensemble does not support setting encoder ratio",
                "Driver does not set Ensemble's Low Limit",
                "Driver does not set Ensemble's High Limit",
                "Ensemble does not support setting proportional gain",
                "Ensemble does not support setting integral gain",
                "Ensemble does not support setting derivative gain",
            ],
            "{text}"
        );
    }

    /// drvEnsembleAsyn.cc:800-833, 432-435: a new non-zero AXISFAULT,
    /// travel-limit faults included, raises problem and prints once; DISABLE
    /// clears it after a travel-limit fault; a zero fault clears it.
    #[epics_rs::base::epics_test]
    async fn a_new_fault_raises_problem_until_disable_or_clear() {
        use epics_rs::asyn::interfaces::motor::StatusWrite;
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;
        use motor_common::testing::LineRig;

        let fault = Arc::new(Mutex::new("%0".to_string()));
        let reply = {
            let fault = fault.clone();
            move |cmd: &str| match cmd.trim_end() {
                "AXISFAULT(@0)" => fault.lock().unwrap().clone(),
                _ => "%0".into(),
            }
        };
        let (_line, io) = LineRig::new("ensFaultLine", reply);
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensFaultLine", "0")));
        let mut axis = EnsembleAxis::new(controller, 0, 2).unwrap();
        let path = std::env::temp_dir().join(format!("ens-fault-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(None, TraceMask::ERROR);
        trace.set_trace_file(None, TraceFile::File(file));
        let user = AsynUser::new(0);
        let poll = |axis: &mut EnsembleAxis, reply: &str| {
            *fault.lock().unwrap() = reply.into();
            axis.poll(&user).unwrap().problem
        };

        assert!(!poll(&mut axis, "%0"));
        assert!(poll(&mut axis, "%4"));
        assert!(poll(&mut axis, "%4"));
        axis.set_closed_loop(&user, false).unwrap();
        assert_eq!(
            axis.take_status_writes(),
            [StatusWrite::Problem(false), StatusWrite::PowerOn(false)]
        );
        assert!(!poll(&mut axis, "%4"));
        assert!(!poll(&mut axis, "%0"));
        assert!(poll(&mut axis, "%1"));
        axis.set_closed_loop(&user, false).unwrap();
        assert_eq!(axis.take_status_writes(), [StatusWrite::PowerOn(false)]);
        assert!(poll(&mut axis, "%1"));

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(
            lines,
            [
                "EnsemblePoller: controller fault on axis=2 fault=0x4",
                "EnsemblePoller: controller fault on axis=2 fault=0x1",
            ],
            "{text}"
        );
    }

    /// drvEnsembleAsyn.cc:972-1015: the axis probe in C's order, RAMP MODE
    /// RATE before ReverseMotionDirection.
    #[test]
    fn the_axis_probe_reads_reverse_direction_after_ramp_mode() {
        use motor_common::testing::LineRig;
        let (line, io) = LineRig::new("ensProbeLine", |_| "%0".into());
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensProbeLine", "0")));
        line.sent();
        EnsembleAxis::new(controller, 0, 0).unwrap();
        assert_eq!(
            line.sent(),
            [
                format!("GETPARM(@0, {PARAM_POSITION_FEEDBACK_TYPE})"),
                format!("GETPARM(@0, {PARAM_COUNTS_PER_UNIT})"),
                format!("GETPARM(@0, {PARAM_HOME_OFFSET})"),
                format!("GETPARM(@0, {PARAM_HOME_SETUP})"),
                format!("GETPARM(@0, {PARAM_END_OF_TRAVEL_LIMIT_SETUP})"),
                "RAMP MODE @0 RATE".to_string(),
                format!("GETPARM(@0, {PARAM_REVERSE_MOTION_DIRECTION})"),
            ]
        );
    }

    /// drvEnsembleAsyn.cc:451-456: an enable that finds a fault prints it
    /// in hex before its FAULTACK.
    #[epics_rs::base::epics_test]
    async fn an_enable_prints_the_fault_it_acknowledges() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;
        use motor_common::testing::LineRig;

        let (line, io) = LineRig::new("ensAckLine", |cmd: &str| match cmd.trim_end() {
            "AXISFAULT @0" => "%26".into(),
            _ => "%0".into(),
        });
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensAckLine", "0")));
        let mut axis = EnsembleAxis::new(controller, 0, 0).unwrap();
        let path = std::env::temp_dir().join(format!("ens-ack-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(None, TraceMask::ERROR);
        trace.set_trace_file(None, TraceFile::File(file));
        line.sent();
        axis.set_closed_loop(&AsynUser::new(0), true).unwrap();
        let sent = line.sent();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(lines, ["motorAxisSetInteger: FAULTACK = 1A"], "{text}");
        assert!(sent.iter().any(|s| s == "FAULTACK @0"), "{sent:?}");
    }

    /// drvEnsembleAsyn.cc:458-476: enable and disable post the requested
    /// PowerOn whether or not ENABLE or DISABLE was accepted; a NAK'd
    /// `WAIT MODE NOWAIT` too, and only a failed one posts nothing and fails
    /// the write.
    #[epics_rs::base::epics_test]
    async fn closed_loop_posts_power_on_unless_nowait_fails() {
        use epics_rs::asyn::interfaces::motor::StatusWrite;
        use motor_common::testing::LineRig;

        let fail = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let reply = move |cmd: &str| match cmd.trim_end() {
            "ENABLE @0" | "WAIT MODE NOWAIT" => "!".into(),
            _ => "%0".into(),
        };
        let fails = {
            let fail = fail.clone();
            move |cmd: &str| {
                cmd.starts_with("WAIT") && fail.load(std::sync::atomic::Ordering::SeqCst)
            }
        };
        let (line, io) = LineRig::failing("ensPowerLine", reply, fails);
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensPowerLine", "0")));
        let mut axis = EnsembleAxis::new(controller, 0, 0).unwrap();
        let user = AsynUser::new(0);
        line.sent();
        axis.set_closed_loop(&user, true).unwrap();
        assert_eq!(line.sent().last().unwrap(), "WAIT MODE NOWAIT");
        assert_eq!(axis.take_status_writes(), [StatusWrite::PowerOn(true)]);
        axis.set_closed_loop(&user, false).unwrap();
        assert_eq!(axis.take_status_writes(), [StatusWrite::PowerOn(false)]);
        fail.store(true, std::sync::atomic::Ordering::SeqCst);
        assert!(axis.set_closed_loop(&user, true).is_err());
        assert!(axis.take_status_writes().is_empty());
    }

    /// An axis 0 on a line answering `reply`, its writes `fails` picks
    /// refused.
    fn rig_axis(
        name: &str,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
        fails: impl Fn(&str) -> bool + Send + Sync + 'static,
    ) -> (motor_common::testing::LineRig, EnsembleAxis) {
        let (line, io) = motor_common::testing::LineRig::failing(name, reply, fails);
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, name, "0")));
        let axis = EnsembleAxis::new(controller, 0, 0).unwrap();
        line.sent();
        (line, axis)
    }

    /// drvEnsembleAsyn.cc:1032-1093: `sendAndReceive` fails only on I/O, so
    /// a NAK fails only the commands whose caller checks the ACK
    /// (POSOFFSET, :358-361); RAMP RATE, the home SETPARMs and IGLOBALs,
    /// AbortDecelRate and RAMP MODE are not checked at all.
    #[test]
    fn a_nak_fails_only_where_c_checks_the_ack() {
        let user = AsynUser::new(0);
        let nak = |cmd: &str| {
            if cmd.starts_with("GETPARM") {
                "%0".into()
            } else {
                "!".into()
            }
        };
        let (line, mut axis) = rig_axis("ensNakLine", nak, |_| false);
        axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        axis.move_relative(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        axis.move_velocity(&user, 0.0, 1.0, 1.0).unwrap();
        axis.home(&user, 0.0, 1.0, 1.0, true).unwrap();
        axis.stop(&user, 1.0).unwrap();
        assert!(axis.set_position(&user, 1.0).is_err());
        assert_eq!(line.sent().len(), 3 + 3 + 3 + 6 + 1 + 1);

        // RAMP MODE's NAK does not refuse the axis.
        let (_line, io) = motor_common::testing::LineRig::new("ensNakProbe", nak);
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensNakProbe", "0")));
        assert!(EnsembleAxis::new(controller, 0, 0).is_ok());
    }

    /// drvEnsembleAsyn.cc:510-533, :584-597, :619-629: a failed exchange
    /// fails a move only at ABS/INC or LINEAR, a home only at PROGRAM RUN, a
    /// velocity move only at FREERUN.
    #[test]
    fn an_io_failure_fails_only_the_checked_exchange() {
        let user = AsynUser::new(0);
        let unchecked = |cmd: &str| {
            cmd.starts_with("RAMP") || cmd.starts_with("SETPARM") || cmd.starts_with("IGLOBAL")
        };
        let (_line, mut axis) = rig_axis("ensIoLine", |_| "%".into(), unchecked);
        axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        axis.move_velocity(&user, 0.0, 1.0, 1.0).unwrap();
        axis.home(&user, 0.0, 1.0, 1.0, true).unwrap();

        let (_line, mut axis) =
            rig_axis("ensIoLinear", |_| "%".into(), |c| c.starts_with("LINEAR"));
        assert!(axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).is_err());
        let (_line, mut axis) = rig_axis("ensIoRun", |_| "%".into(), |c| c.starts_with("PROGRAM"));
        assert!(axis.home(&user, 0.0, 1.0, 1.0, true).is_err());
        let (_line, mut axis) = rig_axis("ensIoFree", |_| "%".into(), |c| c.starts_with("FREERUN"));
        assert!(axis.move_velocity(&user, 0.0, 1.0, 1.0).is_err());
        let (_line, mut axis) = rig_axis("ensIoAbort", |_| "%".into(), |c| c.starts_with("ABORT"));
        assert!(axis.stop(&user, 1.0).is_err());
    }

    /// drvEnsembleAsyn.cc:1050-1092: a bare ACK is the reply, read once (the
    /// ACK-then-EOS re-read never matches a line the input EOS stripped),
    /// and its value is `atoi("")`.
    #[test]
    fn a_bare_ack_is_a_zero_value() {
        use motor_common::testing::LineRig;
        use std::sync::atomic::{AtomicUsize, Ordering};
        let reads = Arc::new(AtomicUsize::new(0));
        let reply = {
            let reads = reads.clone();
            move |_: &str| {
                reads.fetch_add(1, Ordering::SeqCst);
                "%".into()
            }
        };
        let (_line, io) = LineRig::new("ensBareLine", reply);
        let mut ctrl = EnsembleController::new(io, "ensBareLine", "0");
        assert_eq!(ctrl.get_param(0, PARAM_HOME_SETUP).as_deref(), Some(""));
        assert_eq!(reads.load(Ordering::SeqCst), 1);
    }

    /// drvEnsembleAsyn.cc:961-970: any reply to `NONE` answers the ping.
    #[test]
    fn any_reply_answers_the_ping() {
        use motor_common::testing::LineRig;
        let (_line, io) = LineRig::new("ensPingLine", |_| String::new());
        let mut ctrl = EnsembleController::new(io, "ensPingLine", "0");
        assert!(ctrl.ping().is_ok());
    }

    /// drvEnsembleAsyn.cc:972-1015: a probe whose exchange fails reads the
    /// reply C's `inputBuff` still holds.
    #[test]
    fn a_failed_probe_reads_the_last_reply() {
        let reply = |cmd: &str| {
            if cmd.contains(&format!(", {PARAM_COUNTS_PER_UNIT})")) {
                "%1000".into()
            } else {
                "%7".into()
            }
        };
        let home = format!("GETPARM(@0, {PARAM_HOME_OFFSET})");
        let (_line, axis) = rig_axis("ensStaleLine", reply, move |c| c == home);
        let mut text = String::new();
        motor_common::AxisReport::report(&*axis.report(), &mut text, 1);
        assert!(text.contains("home preset: 1000.000000"), "{text}");
    }

    /// The poll's replies by command and the commands whose write fails.
    type PollLine = (
        Arc<Mutex<std::collections::HashMap<String, String>>>,
        Arc<Mutex<Vec<String>>>,
    );

    /// An axis 0 with CountsPerUnit 1 on a line `PollLine` steers; a poll
    /// query it has no reply for answers `%0`.
    fn poll_axis(name: &str) -> (PollLine, EnsembleAxis) {
        let replies: PollLine = Default::default();
        let reply = {
            let replies = replies.0.clone();
            move |cmd: &str| {
                let cmd = cmd.trim_end();
                if cmd == format!("GETPARM(@0, {PARAM_COUNTS_PER_UNIT})") {
                    return "%1".into();
                }
                let replies = replies.lock().unwrap();
                replies.get(cmd).cloned().unwrap_or_else(|| "%0".into())
            }
        };
        let fails = {
            let failing = replies.1.clone();
            move |cmd: &str| failing.lock().unwrap().iter().any(|f| f == cmd.trim_end())
        };
        let (_line, axis) = rig_axis(name, reply, fails);
        (replies, axis)
    }

    fn set_reply((replies, _): &PollLine, cmd: &str, reply: &str) {
        replies.lock().unwrap().insert(cmd.into(), reply.into());
    }

    fn fail_write((_, failing): &PollLine, cmd: Option<&str>) {
        *failing.lock().unwrap() = cmd.into_iter().map(String::from).collect();
    }

    /// drvEnsembleAsyn.cc:714-722, :773-778, :801-806, :837-842: a failed
    /// exchange, or an AXISSTATUS reply of one char, sets motorAxisCommError
    /// and leaves the poll's earlier sets for the next post; a good
    /// AXISSTATUS clears it.
    #[test]
    fn a_failed_poll_exchange_leaves_comm_error_and_the_earlier_sets() {
        let user = AsynUser::new(0);
        let (line, mut axis) = poll_axis("ensPollIo");
        set_reply(&line, "AXISSTATUS(@0)", "%1");
        set_reply(&line, "PFBKPROG(@0)", "%5");
        set_reply(&line, "PCMDPROG(@0)", "%6");
        let good = axis.poll(&user).unwrap();
        assert!(good.powered && !good.comms_error);
        assert_eq!((good.position, good.encoder_position), (6.0, 5.0));

        // AXISSTATUS fails: only the comm error changes.
        fail_write(&line, Some("AXISSTATUS(@0)"));
        assert!(axis.poll(&user).is_err());
        let left = axis.take_failed_poll_status().unwrap();
        assert!(left.comms_error);
        assert_eq!(
            MotorStatus {
                comms_error: false,
                ..left
            },
            good
        );

        // A one-char reply is a comm error too.
        fail_write(&line, None);
        set_reply(&line, "AXISSTATUS(@0)", "%");
        assert!(axis.poll(&user).is_err());
        assert!(axis.take_failed_poll_status().unwrap().comms_error);

        // PFBKPROG fails after a new AXISSTATUS: its bits are left, positions
        // are not.
        set_reply(&line, "AXISSTATUS(@0)", "%0");
        set_reply(&line, "PFBKPROG(@0)", "%7");
        fail_write(&line, Some("PFBKPROG(@0)"));
        assert!(axis.poll(&user).is_err());
        let left = axis.take_failed_poll_status().unwrap();
        assert!(left.comms_error && !left.powered);
        assert_eq!((left.position, left.encoder_position), (6.0, 5.0));

        // VFBK fails: the positions read before it are left.
        fail_write(&line, Some("VFBK(@0)"));
        assert!(axis.poll(&user).is_err());
        let left = axis.take_failed_poll_status().unwrap();
        assert!(left.comms_error);
        assert_eq!((left.position, left.encoder_position), (6.0, 7.0));

        // AXISFAULT fails: comm error, problem untouched.
        fail_write(&line, Some("AXISFAULT(@0)"));
        assert!(axis.poll(&user).is_err());
        let left = axis.take_failed_poll_status().unwrap();
        assert!(left.comms_error && !left.problem);

        fail_write(&line, None);
        assert!(!axis.poll(&user).unwrap().comms_error);
    }

    /// drvEnsembleAsyn.cc:725-730, :779-784, :807-814: a NAK'd AXISSTATUS
    /// bails setting nothing, a NAK'd PFBKPROG is position 0, a NAK'd
    /// AXISFAULT bails with problem and comm error.
    #[test]
    fn a_nak_in_the_poll_is_handled_where_c_checks_it() {
        let user = AsynUser::new(0);
        let (line, mut axis) = poll_axis("ensPollNak");
        set_reply(&line, "PFBKPROG(@0)", "%5");
        let good = axis.poll(&user).unwrap();

        set_reply(&line, "AXISSTATUS(@0)", "!1");
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.take_failed_poll_status().unwrap(), good);

        set_reply(&line, "AXISSTATUS(@0)", "%0");
        set_reply(&line, "PFBKPROG(@0)", "!5");
        assert_eq!(axis.poll(&user).unwrap().encoder_position, 0.0);

        set_reply(&line, "AXISFAULT(@0)", "!");
        assert!(axis.poll(&user).is_err());
        let left = axis.take_failed_poll_status().unwrap();
        assert!(left.problem && left.comms_error);
    }

    /// drvEnsembleAsyn.cc:740-741, :787-789: PLANESTATUS and PCMDPROG
    /// ignore their status and read whatever `inputBuff` holds.
    #[test]
    fn unchecked_poll_exchanges_read_the_last_reply() {
        let user = AsynUser::new(0);
        let (line, mut axis) = poll_axis("ensPollStale");
        set_reply(&line, "PFBKPROG(@0)", "%5");
        set_reply(&line, "PCMDPROG(@0)", "!6");
        // A NAK'd PCMDPROG is still read past its status char.
        assert_eq!(axis.poll(&user).unwrap().position, 6.0);
        fail_write(&line, Some("PCMDPROG(@0)"));
        assert_eq!(axis.poll(&user).unwrap().position, 5.0);
        // AXISSTATUS's reply, odd, stands in for a failed PLANESTATUS.
        set_reply(&line, "AXISSTATUS(@0)", "%1");
        fail_write(&line, Some("PLANESTATUS(0)"));
        assert!(!axis.poll(&user).unwrap().done);
    }

    #[test]
    fn travel_limit_fault_mask_value() {
        assert_eq!(TRAVEL_LIMIT_FAULT_MASK, 0x0C);
    }
}
