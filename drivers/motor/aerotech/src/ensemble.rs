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
use motor_common::drv_motor_asyn::{DrvSet, function};
use motor_common::model2::AxisLog;
use motor_common::param_lib::Params;
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
    /// C `inputBuff`: the last reply, or what a failed read delivered. A
    /// failed write leaves it, and C's callers that ignore the status read
    /// it anyway.
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
        let (reply, status) = self.io.exchange(cmd);
        if let Some(reply) = reply {
            self.input = reply;
        }
        status
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
    /// C `pAxis->card`, the `EnsembleAsynConfig` card.
    card: i32,
    /// Controller axis number (the `@n` address).
    axis: i32,
    /// The axis's place among the controller's, C `itera`.
    index: i32,
    /// C `lastFault`, the AXISFAULT word that last raised `problem`.
    last_fault: i32,
    /// `pAxis->params`: the poller's sets, kept when a poll bails, and
    /// `motorAxisProblem` cleared by DISABLE after a travel-limit fault.
    params: Arc<Params>,
    status_writes: Vec<StatusWrite>,
    /// `HomeSetup` word; bit 0 is the home direction, updated by [`Self::home`].
    /// C's `int homeDirection`, so its `%d` prints a high bit signed.
    home_direction: i32,
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
        card: i32,
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
            // C `atoi`, which is `(int) strtol`.
            let home_direction = ctrl
                .get_param(axis, PARAM_HOME_SETUP)
                .map(|s| atol(&s) as i32)
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

        let params = Arc::new(Params::default());
        // motorAxisInit's GAIN_SUPPORT (drvEnsembleAsyn.cc:262) and the
        // config's encoder (:977).
        let _ = params.set_integer(function::HAS_CLOSED_LOOP, 1);
        if has_encoder {
            let _ = params.set_integer(function::HAS_ENCODER, 1);
        }
        Ok(Self {
            controller,
            params,
            card,
            axis,
            index,
            last_fault: 0,
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

    /// The axis' parameters, which drvAsynMotorConfigure reads and calls
    /// back through.
    pub fn params(&self) -> Arc<Params> {
        self.params.clone()
    }

    /// A command's set, which the poller also posts to the record.
    fn post(&mut self, write: StatusWrite) {
        self.params.apply(write);
        self.status_writes.push(write);
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

    fn do_move(
        &mut self,
        position: f64,
        relative: bool,
        min_v: f64,
        max_v: f64,
        accel: f64,
    ) -> AsynResult<()> {
        let axis = self.axis;
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "Set card {}, axis {axis} move to {:.6}, min vel={:.6}, max_vel={:.6}, accel={:.6}\n",
                self.card,
                CFloat(position),
                CFloat(min_v),
                CFloat(max_v),
                CFloat(accel)
            ),
        );
        // C `currentCmdPos` is the poll's PCMDPROG, as motorAxisPosition.
        let posdir = if relative {
            position >= 0.0
        } else {
            position >= self.params.motor_status().position
        };
        let mode = if relative { "INC" } else { "ABS" };
        {
            let mut ctrl = self.lock();
            ctrl.send(mode)?;
            if accel > 0.0 {
                let _ = ctrl.send(&format!("RAMP RATE {}", self.units(accel)));
            }
            ctrl.send(&format!(
                "LINEAR @{axis} {} F{}",
                self.units(position),
                self.units(max_v)
            ))?;
        }
        self.set_started(posdir);
        Ok(())
    }

    /// `motorAxisDirection` and `motorAxisDone` 0, which a move, a home
    /// and a velocity move set together before their callback, so the
    /// record's next status update sees the motion started.
    fn set_started(&mut self, direction: bool) {
        self.post(StatusWrite::Direction(direction));
        self.post(StatusWrite::Done(false));
        self.params.call_callback();
    }
}

impl EnsembleAxis {
    /// One axis of C `EnsemblePoller` (drvEnsembleAsyn.cc:713-848), setting
    /// [`Self::params`] in its order; an `Err` is its `continue` before
    /// `callCallback`.
    fn poll_status(&mut self) -> AsynResult<()> {
        let axis = self.axis;
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let params = self.params.clone();
        let bit = |f, v: bool| {
            let _ = params.set_integer(f, i32::from(v));
        };

        let sent = ctrl.send(&format!("AXISSTATUS(@{axis})"));
        if sent.is_err() || ctrl.input.len() <= 1 {
            bit(function::COMM_ERROR, true);
            return sent.and(Err(ens_err("ensemble: AXISSTATUS reply too short")));
        }
        if !ctrl.ack() {
            return Err(ens_err(format!(
                "ensemble: AXISSTATUS refused: '{}'",
                ctrl.input
            )));
        }
        bit(function::COMM_ERROR, false);
        let axis_status = atol(ctrl.rest()) as u32;
        let _ = ctrl.send("PLANESTATUS(0)");
        let move_active = (atoi(ctrl.rest()) & 0x01) != 0 || (axis_status & MOVE_ACTIVE) != 0;
        bit(function::DONE, !move_active);
        bit(function::POWER_ON, (axis_status & AXIS_ENABLED) != 0);
        bit(function::HOMED, (axis_status & HOME_CYCLE_COMPLETE) != 0);
        bit(function::HOME_SIGNAL, (axis_status & HOME_LIMIT) != 0);
        let motion_ccw = (axis_status & MOTION_CCW) != 0;
        bit(
            function::DIRECTION,
            if self.reverse_direc {
                motion_ccw
            } else {
                !motion_ccw
            },
        );
        // Limit switches: XOR the raw limit bit against its configured active
        // level, then map CW/CCW to high/low honouring the reverse-direction flag.
        let cw_sw_active =
            !(((axis_status & CW_LIMIT) != 0) ^ ((self.swconfig & CW_EOT_SW_STATE) != 0));
        let ccw_sw_active =
            !(((axis_status & CCW_LIMIT) != 0) ^ ((self.swconfig & CCW_EOT_SW_STATE) != 0));
        let (high, low) = if self.reverse_direc {
            (ccw_sw_active, cw_sw_active)
        } else {
            (cw_sw_active, ccw_sw_active)
        };
        bit(function::HIGH_HARD_LIMIT, high);
        bit(function::LOW_HARD_LIMIT, low);
        self.report.set_axis_status(axis_status);

        if let Err(e) = ctrl.send(&format!("PFBKPROG(@{axis})")) {
            bit(function::COMM_ERROR, true);
            return Err(e);
        }
        let encoder = if ctrl.ack() { atof(ctrl.rest()) } else { 0.0 };
        let _ = params.set_double(function::ENCODER_POSN, encoder / self.step);
        let _ = ctrl.send(&format!("PCMDPROG(@{axis})"));
        let position = atof(ctrl.rest()) / self.step;
        let _ = params.set_double(function::POSITION, position);
        self.log.print(
            TraceMask::IO_DRIVER,
            format_args!(
                "EnsemblePoller: axis {axis} axisStatus={axis_status:x}, position={:.6}\n",
                CFloat(position)
            ),
        );

        if let Err(e) = ctrl.send(&format!("AXISFAULT(@{axis})")) {
            bit(function::COMM_ERROR, true);
            return Err(e);
        }
        if !ctrl.ack() {
            bit(function::PROBLEM, true);
            bit(function::COMM_ERROR, true);
            return Err(ens_err(format!(
                "ensemble: AXISFAULT refused: '{}'",
                ctrl.input
            )));
        }
        let fault = atoi(ctrl.rest());
        if fault == 0 {
            self.last_fault = 0;
            bit(function::PROBLEM, false);
        } else if fault != self.last_fault {
            self.last_fault = fault;
            bit(function::PROBLEM, true);
            self.log.print(
                TraceMask::ERROR,
                format_args!(
                    "EnsemblePoller: controller fault on axis={} fault=0x{fault:X}\n",
                    self.index
                ),
            );
        }

        if let Err(e) = ctrl.send(&format!("VFBK(@{axis})")) {
            bit(function::COMM_ERROR, true);
            return Err(e);
        }
        let _ = params.set_double(function::ACTUAL_VEL, atof(ctrl.rest()) / self.step);
        params.call_callback();
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
        let axis = self.axis;
        let sent = {
            let mut ctrl = self.lock();
            let _ = ctrl.send(&format!(
                "SETPARM @{axis}, {PARAM_ABORT_DECEL_RATE}, {}",
                self.units(acceleration)
            ));
            let _ = ctrl.send(&format!("RAMP RATE @{axis} {}", self.units(acceleration)));
            ctrl.send(&format!("FREERUN @{axis} {}", self.units(velocity)))
        };
        // C sets them whatever FREERUN returned (drvEnsembleAsyn.cc:630-636).
        self.set_started(velocity > 0.0);
        sent
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
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "motorAxisHome: set card {}, axis {axis} to home, forwards = {}\n",
                self.card,
                i32::from(forward)
            ),
        );
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
        ctrl.send("PROGRAM RUN 5, \"HomeAsync.bcx\"")?;
        drop(ctrl);
        self.set_started(forward);
        Ok(())
    }

    /// `motorAxisforceCallback` (drvEnsembleAsyn.cc:671-683); the status update it asks for
    /// is the poll that follows it.
    fn force_callback(&mut self, _user: &AsynUser) {
        self.params.force_callback();
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "motorAxisforceCallback: request card {}, axis {} status update\n",
                self.card, self.axis
            ),
        );
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let axis = self.axis;
        self.log.print(
            TraceMask::FLOW,
            format_args!("Abort on card {}, axis {axis}\n", self.card),
        );
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
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
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
                self.post(StatusWrite::Problem(false));
            }
            let _ = ctrl.send(&format!("DISABLE @{axis}"));
        }
        // C's status is WAIT MODE NOWAIT's; PowerOn follows the request even
        // when ENABLE fails, to force a status update (:461-476).
        self.post(StatusWrite::PowerOn(closed_loop));
        ctrl.wait_mode_nowait()
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        let polled = self.poll_status();
        *status = self.params.motor_status();
        polled
    }
}

/// C `motorAxisSetDouble`/`motorAxisSetInteger` (drvEnsembleAsyn.cc:343-479):
/// a double it does not set prints a TERROR line and still succeeds, an
/// integer other than the closed loop fails; a success goes into the
/// parameters and is called back.
impl DrvSet for EnsembleAxis {
    fn set_double(&mut self, user: &AsynUser, function: i32, value: f64) -> AsynResult<()> {
        match function {
            function::POSITION => self.set_position(user, value)?,
            function::ENCODER_RATIO => self.set_encoder_ratio(user, value)?,
            function::RESOLUTION => self
                .refuse("motorAxisSetDouble: Ensemble does not support setting motor resolution"),
            function::LOW_LIMIT => self.set_low_limit(user, value)?,
            function::HIGH_LIMIT => self.set_high_limit(user, value)?,
            function::P_GAIN => self.set_pid_gain(user, PidGainKind::Proportional, value)?,
            function::I_GAIN => self.set_pid_gain(user, PidGainKind::Integral, value)?,
            function::D_GAIN => self.set_pid_gain(user, PidGainKind::Derivative, value)?,
            _ => self.refuse(&format!("motorAxisSetDouble: unknown function {function}")),
        }
        let _ = self.params.set_double(function, value);
        self.params.call_callback();
        Ok(())
    }

    fn set_integer(&mut self, user: &AsynUser, function: i32, value: i32) -> AsynResult<()> {
        if function != function::CLOSED_LOOP {
            self.refuse(&format!("motorAxisSetInteger: unknown function {function}"));
            return Err(ens_err(format!(
                "motorAxisSetInteger: unknown function {function}"
            )));
        }
        self.set_closed_loop(user, value != 0)?;
        let _ = self.params.set_integer(function, value);
        self.params.call_callback();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::Polled;

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

    /// drvEnsembleAsyn.cc:343-479, 743-849: through the port, a double the
    /// Ensemble does not set succeeds into its parameters, an unknown
    /// integer fails, and a poll and a move set what C's set.
    #[epics_rs::base::epics_test]
    async fn the_port_writes_go_through_the_c_set_switch() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::base::server::iocsh::registry::ArgValue;
        use motor_common::model2::{Model2, Model2Axis, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        static DRIVER: Model2 = Model2::new("ensSetTest", 0);
        let reply = |cmd: &str| match cmd.trim_end() {
            // axis_enabled, home_cycle_complete.
            "AXISSTATUS(@0)" => "%3".into(),
            "PCMDPROG(@0)" => "%2.5".into(),
            "GETPARM(@0, 2)" => "%1".into(),
            _ => "%0".into(),
        };
        let (_line, io) = LineRig::new("ensSetLine", reply);
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensSetLine", "0")));
        let axis = EnsembleAxis::new(controller, 0, 0, 0).unwrap();
        let params = axis.params();
        let axis = Arc::new(Mutex::new(axis));
        let log = axis.lock().unwrap().log();
        let axes = vec![Model2Axis::new(axis.clone(), params.clone(), log)];
        DRIVER.set_card(0, axes, 3_600_000, 3_600_000);
        let args = vec![
            ArgValue::String("ensSet".into()),
            ArgValue::String("ensSetTest".into()),
            ArgValue::Int(0),
            ArgValue::Int(1),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&DRIVER]), args).await;
        let port = PortManager::global().find_port_handle("ensSet").unwrap();
        let reads = std::thread::spawn(move || {
            let gain = port.write_float64_blocking(function::P_GAIN as usize, 0, 0.5);
            let unknown = port.write_float64_blocking(function::SLIP as usize, 0, 2.0);
            let int = port.write_int32_blocking(function::SLIP as usize, 0, 1);
            let read = port.read_float64_blocking(function::P_GAIN as usize, 0);
            (gain.is_ok(), unknown.is_ok(), int.is_err(), read.ok())
        })
        .join()
        .unwrap();
        assert_eq!(reads, (true, true, true, Some(0.5)));
        assert_eq!(params.get_double(function::SLIP), Ok(Some(2.0)));
        assert_eq!(params.get_integer(function::HAS_CLOSED_LOOP), Ok(Some(1)));
        let user = AsynUser::new(0);
        let mut axis = axis.lock().unwrap();
        axis.polled(&user).unwrap();
        assert_eq!(params.get_integer(function::HOMED), Ok(Some(1)));
        assert_eq!(params.get_integer(function::POWER_ON), Ok(Some(1)));
        assert_eq!(params.get_double(function::POSITION), Ok(Some(2.5)));
        axis.move_absolute(&user, 1.0, 0.0, 1.0, 0.0).unwrap();
        assert_eq!(params.get_integer(function::DONE), Ok(Some(0)));
        assert_eq!(params.get_integer(function::DIRECTION), Ok(Some(0)));
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
        use motor_common::model2::{Model2, Model2Axis, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        static DRIVER: Model2 = Model2::new("ensLogTest", 0);
        let (_line, io) = LineRig::new("ensLogLine", |_| "%0".into());
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensLogLine", "0")));
        let axis = EnsembleAxis::new(controller, 0, 0, 0).unwrap();
        let (getter, log) = (axis.params(), axis.log());
        let axis = Arc::new(Mutex::new(axis));
        DRIVER.set_card(
            0,
            vec![Model2Axis::new(axis.clone(), getter, log)],
            100,
            1000,
        );
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
        let mut axis = EnsembleAxis::new(controller, 0, 0, 2).unwrap();
        let path = std::env::temp_dir().join(format!("ens-fault-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(None, TraceMask::ERROR);
        trace.set_trace_file(None, TraceFile::File(file));
        let user = AsynUser::new(0);
        let poll = |axis: &mut EnsembleAxis, reply: &str| {
            *fault.lock().unwrap() = reply.into();
            axis.polled(&user).unwrap().problem
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
        EnsembleAxis::new(controller, 0, 0, 0).unwrap();
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

    /// drvEnsembleAsyn.cc:577-584: `HomeSetup` is a C `int`, so a word with
    /// the high bit set goes out signed.
    #[test]
    fn a_home_sends_the_home_setup_word_signed() {
        let reply = |cmd: &str| match cmd.trim_end() {
            c if c == format!("GETPARM(@0, {PARAM_HOME_SETUP})") => "%-1".into(),
            _ => "%0".into(),
        };
        let (line, mut axis) = rig_axis("ensHomeSetupLine", reply, |_| false);
        axis.home(&AsynUser::new(0), 0.0, 1.0, 1.0, true).unwrap();
        let sent = line.sent();
        let setup = format!("SETPARM @0, {PARAM_HOME_SETUP}, -2");
        assert!(sent.contains(&setup), "{sent:?}");
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
        let mut axis = EnsembleAxis::new(controller, 0, 0, 0).unwrap();
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
    /// `WAIT MODE NOWAIT` too. A failed one fails the write and still
    /// posts: C calls back unless the status is MOTOR_AXIS_ERROR (-1), which
    /// no asyn status is (:479-483).
    #[epics_rs::base::epics_test]
    async fn closed_loop_posts_power_on_even_when_nowait_fails() {
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
        let mut axis = EnsembleAxis::new(controller, 0, 0, 0).unwrap();
        let user = AsynUser::new(0);
        line.sent();
        axis.set_closed_loop(&user, true).unwrap();
        assert_eq!(line.sent().last().unwrap(), "WAIT MODE NOWAIT");
        assert_eq!(axis.take_status_writes(), [StatusWrite::PowerOn(true)]);
        axis.set_closed_loop(&user, false).unwrap();
        assert_eq!(axis.take_status_writes(), [StatusWrite::PowerOn(false)]);
        fail.store(true, std::sync::atomic::Ordering::SeqCst);
        assert!(axis.set_closed_loop(&user, true).is_err());
        assert_eq!(axis.take_status_writes(), [StatusWrite::PowerOn(true)]);
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
        let axis = EnsembleAxis::new(controller, 0, 0, 0).unwrap();
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
        assert!(EnsembleAxis::new(controller, 0, 0, 0).is_ok());
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
        let good = axis.polled(&user).unwrap();
        assert!(good.powered && !good.comms_error);
        assert_eq!((good.position, good.encoder_position), (6.0, 5.0));

        // AXISSTATUS fails: only the comm error changes.
        fail_write(&line, Some("AXISSTATUS(@0)"));
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
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
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
        assert!(left.comms_error);

        // PFBKPROG fails after a new AXISSTATUS: its bits are left, positions
        // are not.
        set_reply(&line, "AXISSTATUS(@0)", "%0");
        set_reply(&line, "PFBKPROG(@0)", "%7");
        fail_write(&line, Some("PFBKPROG(@0)"));
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
        assert!(left.comms_error && !left.powered);
        assert_eq!((left.position, left.encoder_position), (6.0, 5.0));

        // VFBK fails: the positions read before it are left.
        fail_write(&line, Some("VFBK(@0)"));
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
        assert!(left.comms_error);
        assert_eq!((left.position, left.encoder_position), (6.0, 7.0));

        // AXISFAULT fails: comm error, problem untouched.
        fail_write(&line, Some("AXISFAULT(@0)"));
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
        assert!(left.comms_error && !left.problem);

        fail_write(&line, None);
        assert!(!axis.polled(&user).unwrap().comms_error);
    }

    /// drvEnsembleAsyn.cc:725-730, :779-784, :807-814: a NAK'd AXISSTATUS
    /// bails setting nothing, a NAK'd PFBKPROG is position 0, a NAK'd
    /// AXISFAULT bails with problem and comm error.
    #[test]
    fn a_nak_in_the_poll_is_handled_where_c_checks_it() {
        let user = AsynUser::new(0);
        let (line, mut axis) = poll_axis("ensPollNak");
        set_reply(&line, "PFBKPROG(@0)", "%5");
        let good = axis.polled(&user).unwrap();

        set_reply(&line, "AXISSTATUS(@0)", "!1");
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
        assert_eq!(left, good);

        set_reply(&line, "AXISSTATUS(@0)", "%0");
        set_reply(&line, "PFBKPROG(@0)", "!5");
        assert_eq!(axis.polled(&user).unwrap().encoder_position, 0.0);

        set_reply(&line, "AXISFAULT(@0)", "!");
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
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
        assert_eq!(axis.polled(&user).unwrap().position, 6.0);
        fail_write(&line, Some("PCMDPROG(@0)"));
        assert_eq!(axis.polled(&user).unwrap().position, 5.0);
        // AXISSTATUS's reply, odd, stands in for a failed PLANESTATUS.
        set_reply(&line, "AXISSTATUS(@0)", "%1");
        fail_write(&line, Some("PLANESTATUS(0)"));
        assert!(!axis.polled(&user).unwrap().done);
    }

    /// drvEnsembleAsyn.cc:740-741, :787-789: a PLANESTATUS or PCMDPROG
    /// reply cut by a timeout is read from what the failed read delivered.
    #[test]
    fn unchecked_poll_exchanges_read_a_cut_reply() {
        let reply = |cmd: &str| {
            match cmd.trim_end() {
                c if c == format!("GETPARM(@0, {PARAM_COUNTS_PER_UNIT})") => "%1",
                "PFBKPROG(@0)" => "%5",
                "PCMDPROG(@0)" => "%6",
                "PLANESTATUS(0)" => "%1",
                _ => "%0",
            }
            .into()
        };
        let cut = |cmd: &str| matches!(cmd.trim_end(), "PCMDPROG(@0)" | "PLANESTATUS(0)");
        let (line, io) = motor_common::testing::LineRig::cut("ensPollCut", reply, cut);
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensPollCut", "0")));
        let mut axis = EnsembleAxis::new(controller, 0, 0, 0).unwrap();
        line.sent();
        let status = axis.polled(&AsynUser::new(0)).unwrap();
        assert_eq!(status.position, 6.0);
        assert!(!status.done);
    }

    #[test]
    fn travel_limit_fault_mask_value() {
        assert_eq!(TRAVEL_LIMIT_FAULT_MASK, 0x0C);
    }

    /// drvEnsembleAsyn.cc:497-498, :562-563, :662, :676: move, home, abort
    /// and forceCallback print their FLOW line with the card the config gave the axis.
    #[epics_rs::base::epics_test]
    async fn commands_print_their_flow_line_with_the_card() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;
        use motor_common::testing::LineRig;

        let (_line, io) = LineRig::new("ensFlowLine", |_| "%0".into());
        let controller = Arc::new(Mutex::new(EnsembleController::new(io, "ensFlowLine", "3")));
        let mut axis = EnsembleAxis::new(controller, 3, 2, 0).unwrap();
        let path = std::env::temp_dir().join(format!("ens-flow-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(None, TraceMask::FLOW);
        trace.set_trace_file(None, TraceFile::File(file));
        let user = AsynUser::new(0);
        axis.move_absolute(&user, 1.5, 0.5, 2.0, 3.0).unwrap();
        axis.move_relative(&user, -1.0, 0.0, 2.0, 0.0).unwrap();
        axis.home(&user, 0.0, 1.0, 1.0, true).unwrap();
        axis.stop(&user, 1.0).unwrap();
        axis.force_callback(&user);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(
            lines,
            [
                "Set card 3, axis 2 move to 1.500000, min vel=0.500000, max_vel=2.000000, accel=3.000000",
                "Set card 3, axis 2 move to -1.000000, min vel=0.000000, max_vel=2.000000, accel=0.000000",
                "motorAxisHome: set card 3, axis 2 to home, forwards = 1",
                "Abort on card 3, axis 2",
                "motorAxisforceCallback: request card 3, axis 2 status update",
            ],
            "{text}"
        );
    }

    /// drvEnsembleAsyn.cc:797-798: a poll that reads both positions prints
    /// its IODRIVER line with the status word and the commanded position.
    #[epics_rs::base::epics_test]
    async fn a_poll_prints_its_iodriver_line() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;

        let user = AsynUser::new(0);
        let (line, mut axis) = poll_axis("ensIoDrvLine");
        set_reply(&line, "AXISSTATUS(@0)", "%513");
        set_reply(&line, "PCMDPROG(@0)", "%6.5");
        let path = std::env::temp_dir().join(format!("ens-iodrv-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(None, TraceMask::IO_DRIVER);
        trace.set_trace_file(None, TraceFile::File(file));
        axis.polled(&user).unwrap();
        // A poll that bails before PCMDPROG prints nothing.
        fail_write(&line, Some("PFBKPROG(@0)"));
        assert!(axis.polled(&user).is_err());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(
            lines,
            ["EnsemblePoller: axis 0 axisStatus=201, position=6.500000"],
            "{text}"
        );
    }

    /// drvEnsembleAsyn.cc:510-541, :591-603, :630-636: a move and a home set motorAxisDirection and
    /// motorAxisDone 0 once their checked exchange succeeded, a velocity move
    /// whatever FREERUN returned.
    #[test]
    fn commands_post_the_direction_they_set() {
        use epics_rs::asyn::interfaces::motor::StatusWrite::{Direction, Done};
        let failing = Arc::new(Mutex::new(String::new()));
        let fails = {
            let failing = failing.clone();
            move |cmd: &str| {
                let f = failing.lock().unwrap();
                !f.is_empty() && cmd.starts_with(f.as_str())
            }
        };
        let fail = |prefix: &str| *failing.lock().unwrap() = prefix.into();
        let (_line, mut axis) = rig_axis("ensDirLine", |_| "%0".into(), fails);
        let user = AsynUser::new(0);
        axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        assert_eq!(axis.take_status_writes(), [Direction(true), Done(false)]);
        axis.move_relative(&user, -1.0, 0.0, 1.0, 1.0).unwrap();
        assert_eq!(axis.take_status_writes(), [Direction(false), Done(false)]);
        axis.move_velocity(&user, 0.0, 1.0, 1.0).unwrap();
        assert_eq!(axis.take_status_writes(), [Direction(true), Done(false)]);
        axis.home(&user, 0.0, 1.0, 1.0, false).unwrap();
        assert_eq!(axis.take_status_writes(), [Direction(false), Done(false)]);

        fail("LINEAR");
        assert!(axis.move_absolute(&user, -1.0, 0.0, 1.0, 1.0).is_err());
        assert_eq!(axis.take_status_writes(), []);
        fail("FREERUN");
        assert!(axis.move_velocity(&user, 0.0, -1.0, 1.0).is_err());
        assert_eq!(axis.take_status_writes(), [Direction(false), Done(false)]);
        fail("PROGRAM");
        assert!(axis.home(&user, 0.0, 1.0, 1.0, true).is_err());
        assert_eq!(axis.take_status_writes(), []);
    }
}
