//! XPS axis: one positioner exposed as an asyn [`AsynMotor`].
//!
//! Faithful port of `XPSAxis` (`XPSAxis.cpp`). Each axis holds a clone of the
//! shared [`XpsController`] (for the poll socket + group registry) plus its own
//! `Fire`-mode **move socket**. Reads (`poll`, PID, status) go through the
//! locked controller's poll socket; the actual `GroupMove*`/`GroupHomeSearch`
//! go out the axis's own move socket without waiting for completion — the poll
//! observes when the move finishes via `GroupStatusGet`.
//!
//! Record steps cross the motor boundary and `step_size` (C `stepSize_`,
//! `1 / stepsPerUnit`) converts them to positioner units, as in C.

use std::fmt;
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;

use super::controller::{SetPositionFailure, XpsController};
use super::corrector::{self, CorrectorType, PidFailure};
use super::rpc::{XpsError, XpsResult, XpsSocket};

/// C `driverName` in XPSAxis.cpp.
const DRIVER_NAME: &str = "XPSAxis";

/// Velocity below which the axis is considered stopped (C
/// `XPS_VELOCITY_DEADBAND`).
const XPS_VELOCITY_DEADBAND: f64 = 0.0000001;

/// `PositionerErrorGet` bit for the positive hardware end-of-run limit
/// (C `XPSC8_END_OF_RUN_PLUS`).
const XPSC8_END_OF_RUN_PLUS: u32 = 0x8000_0200;
/// `PositionerErrorGet` bit for the negative hardware end-of-run limit
/// (C `XPSC8_END_OF_RUN_MINUS`).
const XPSC8_END_OF_RUN_MINUS: u32 = 0x8000_0100;

/// The XPS group status code (`GroupStatusGet`) that means "home switch
/// active" for the encoder-home (ATHM) signal.
const XPS_STATUS_HOMING_DONE: i32 = 11;

/// `PositionerHardwareStatusGet` bit for "at high level of the home (ZM) zone"
/// (C `XPSC8_ZM_HIGH_LEVEL`). Used by move-to-home to pick the search direction.
const XPSC8_ZM_HIGH_LEVEL: i32 = 0x0000_0004;

/// Short settle between move-to-home referencing steps (C uses `epicsThreadSleep`
/// of 0.05 s to dodge XPS firmware race conditions).
const XPS_REFERENCING_SETTLE: Duration = Duration::from_millis(50);

/// Poll interval while waiting for the move-to-home search to reach the switch
/// (C `epicsThreadSleep(0.2)`).
const XPS_REFERENCING_POLL: Duration = Duration::from_millis(200);

/// Safety cap on the move-to-home switch-crossing wait. C's loop is unbounded
/// (isolated on its own thread); the synchronous port bounds it so a switch
/// that is never reached cannot hang the async worker. 300 × 0.2 s = 60 s.
const XPS_REFERENCING_MAX_POLLS: u32 = 300;

/// One XPS positioner as an asyn motor axis.
pub struct XpsAxis {
    controller: Arc<Mutex<XpsController>>,
    move_sock: XpsSocket,
    positioner_name: String,
    group_name: String,
    step_size: f64,
    /// Cached S-gamma jerk times read at construction, resent on every move
    /// (C caches these in the `XPS_MIN_JERK`/`XPS_MAX_JERK` record params).
    min_jerk: f64,
    max_jerk: f64,
    /// Last group status code seen by [`poll`](AsynMotor::poll); `move` reads it
    /// to decide whether to auto-enable a disabled axis.
    axis_status: i32,
    /// C `moving_`: set by a move this axis sent; in moving mode only the
    /// move socket's answer or a stopped group clears it.
    moving: bool,
    /// The status the last poll left, C's parameter-library values.
    last_status: MotorStatus,
    /// C's `pasynUser_`: the motor port and axis number every error line
    /// prints on and names as `[port,axis]`.
    trace: MotorTrace,
}

impl XpsAxis {
    /// Build an axis for `positioner_name` (`group.positioner`) over its own
    /// `Fire`-mode move socket, registering it with the controller and reading
    /// its S-gamma jerk times and corrector (C `XPSAxis` constructor, which
    /// cannot fail).
    pub fn new(
        controller: Arc<Mutex<XpsController>>,
        move_sock: XpsSocket,
        motor_port: &str,
        axis_no: i32,
        positioner_name: &str,
        step_size: f64,
    ) -> Self {
        let trace = MotorTrace::new(motor_port, axis_no);
        if !positioner_name.contains('.') {
            trace_error(
                &trace,
                "XPSAxis",
                format_args!(
                    "positionerName must be of form group.positioner = {positioner_name}\n"
                ),
            );
        }
        let group_name = group_of(positioner_name).to_string();
        let (min_jerk, max_jerk, pid) = {
            let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // C ignores a failed read; its jerk times are then whatever the
            // uninitialised locals held, here 0.
            let (_vel, _accel, min_jerk, max_jerk) = ctrl
                .poll_socket()
                .positioner_sgamma_parameters_get(positioner_name)
                .unwrap_or_default();
            let pid = corrector::get_pid(ctrl.poll_socket(), positioner_name);
            ctrl.register_axis(axis_no, positioner_name, &group_name);
            (min_jerk, max_jerk, pid)
        };
        let axis = Self {
            controller,
            move_sock,
            positioner_name: positioner_name.to_string(),
            group_name,
            step_size,
            min_jerk,
            max_jerk,
            axis_status: 0,
            moving: false,
            last_status: MotorStatus {
                gain_support: true,
                has_encoder: true,
                // The XPS move ignores base velocity (min_velocity); VBAS unused.
                vbas_supported: false,
                ..Default::default()
            },
            trace,
        };
        // C reads the corrector into `xpsCorrectorInfo_`, which nothing here
        // keeps: setPID re-reads it.
        if let Err(failure) = pid {
            axis.report_get_pid_failure(&failure);
        }
        axis
    }

    /// `asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: ...", driverName,
    /// functionName, ...)`.
    fn error(&self, function: &str, args: fmt::Arguments<'_>) {
        trace_error(&self.trace, function, args);
    }

    /// C's `pC_->portName` and `axisNo_`, as every `[%s,%d]` names them.
    fn port(&self) -> &str {
        self.trace.port()
    }

    fn axis_no(&self) -> i32 {
        self.trace.addr()
    }

    /// C `getXPSError`/`ErrorStringGet` on the poll socket; C leaves the
    /// buffer as it was when that call fails, here empty.
    fn error_string(ctrl: &XpsController, code: i32) -> String {
        ctrl.poll_socket()
            .error_string_get(code)
            .unwrap_or_default()
    }

    fn lock_controller(&self) -> MutexGuard<'_, XpsController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Read the group status over the poll socket, holding the controller lock
    /// only for the single exchange (used by the move-to-home step sequence,
    /// which must release the lock between RPCs and sleeps).
    /// C `GroupStatusGet(pollSocket_, groupName_, &groupStatus)` with its
    /// status ignored: a failed call leaves `group_status` as it was.
    fn refresh_group_status(&self, group_status: &mut i32) {
        let ctrl = self.lock_controller();
        if let Ok(status) = ctrl.poll_socket().group_status_get(&self.group_name) {
            *group_status = status;
        }
    }

    /// `PositionerHardwareStatusGet` into an out-parameter, as
    /// [`Self::refresh_group_status`].
    fn refresh_hardware_status(&self, hardware_status: &mut i32) {
        let ctrl = self.lock_controller();
        if let Ok(status) = ctrl
            .poll_socket()
            .positioner_hardware_status_get(&self.positioner_name)
        {
            *hardware_status = status;
        }
    }

    /// Shared move preamble (C `XPSAxis::move`, before the `GroupMove*` call):
    /// auto-enable a disabled axis, then set the S-gamma velocity/acceleration
    /// profile — both on the poll socket.
    fn prepare_move(
        &self,
        ctrl: &XpsController,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let sock = ctrl.poll_socket();
        // Disabled state (20..=36): auto-enable, or refuse the move.
        if (20..=36).contains(&self.axis_status) {
            if ctrl.auto_enable() {
                sock.group_motion_enable(&self.group_name)
                    .inspect_err(|e| {
                        self.error(
                            "move",
                            format_args!(
                                "motorAxisMove[{},{}]: error performing GroupMotionEnable {}\n",
                                self.port(),
                                self.axis_no(),
                                e.code()
                            ),
                        )
                    })?;
            } else {
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: format!(
                        "XPS axis {} is disabled and auto-enable is off",
                        self.positioner_name
                    ),
                });
            }
        }
        sock.positioner_sgamma_parameters_set(
            &self.positioner_name,
            velocity * self.step_size,
            acceleration * self.step_size,
            self.min_jerk,
            self.max_jerk,
        )
        .inspect_err(|e| {
            self.error(
                "move",
                format_args!(
                    "Error performing PositionerSGammaParametersSet[{},{}] {}: {}\n",
                    self.port(),
                    self.axis_no(),
                    e.code(),
                    Self::error_string(ctrl, e.code())
                ),
            )
        })?;
        Ok(())
    }

    /// The `GroupMoveAbsolute`/`GroupMoveRelative` verdict: C tolerates
    /// `-27` (the motor record reversing direction aborts a move) and prints
    /// every other error.
    fn move_result(&self, command: &str, result: XpsResult<()>) -> AsynResult<()> {
        match result {
            Ok(()) | Err(XpsError::Api(-27)) => Ok(()),
            Err(e) => {
                self.error(
                    "move",
                    format_args!(
                        "Error performing {command}[{},{}] {}\n",
                        self.port(),
                        self.axis_no(),
                        e.code()
                    ),
                );
                Err(e.into())
            }
        }
    }

    fn limits_get_error(&self, function: &str, e: &XpsError) {
        self.error(
            function,
            format_args!(
                "[{},{}]: error performing PositionerUserTravelLimitsGet status={}\n",
                self.port(),
                self.axis_no(),
                e.code()
            ),
        )
    }

    /// C `setPID`'s line for the step that failed.
    fn report_pid_failure(&self, failure: &PidFailure) {
        let (port, axis) = (self.port(), self.axis_no());
        let name = |ct: &CorrectorType| ct.short_name();
        match failure {
            PidFailure::TypeGet(code) => self.error(
                "setPID",
                format_args!(
                    "Error with PositionerCorrectorTypeGet. XPS: {port}, Axis: {axis}, XPS API Error: {code}\n"
                ),
            ),
            PidFailure::Get(CorrectorType::PiPosition, code) => self.error(
                "setPID",
                format_args!(
                    "Error with PositionerCorrectorPIPositionGet. Aborting setting PID XPS: {port}, Axis: {axis}, XPS API Error: {code}\n"
                ),
            ),
            PidFailure::Set(CorrectorType::PiPosition, code) => self.error(
                "setPID",
                format_args!(
                    "Error with PositionerCorrectorPIPositionSet: {port}, Axis: {axis}, XPS API Error: {code}\n"
                ),
            ),
            PidFailure::Get(ct, code) => self.error(
                "setPID",
                format_args!(
                    "Error with PositionerCorrector{}Get. Aborting setting PID {port}, Axis: {axis}, XPS API Error: {code}\n",
                    name(ct)
                ),
            ),
            PidFailure::Set(ct, code) => self.error(
                "setPID",
                format_args!(
                    "Error with PositionerCorrector{}Set. Aborting setting PID {port}, Axis: {axis}, XPS API Error: {code}\n",
                    name(ct)
                ),
            ),
            PidFailure::Unsupported(CorrectorType::NoCorrector, type_name) => self.error(
                "setPID",
                format_args!(
                    ". XPS {port} axis {axis} corrector type is {type_name}. Cannot set PID.\n"
                ),
            ),
            PidFailure::Unsupported(_, type_name) => self.error(
                "setPID",
                format_args!(
                    ". XPS {port} axis {axis} not a valid corrector type: {type_name}. Cannot set PID.\n"
                ),
            ),
        }
    }

    /// C `getPID`'s line for the step that failed; its wording differs from
    /// `setPID`'s.
    fn report_get_pid_failure(&self, failure: &PidFailure) {
        let (port, axis) = (self.port(), self.axis_no());
        match failure {
            PidFailure::TypeGet(code) => self.error(
                "getPID",
                format_args!(
                    "Error with PositionerCorrectorTypeGet. XPS: {port}, Axis: {axis}, XPS API Error: {code}\n"
                ),
            ),
            PidFailure::Get(ct, code) | PidFailure::Set(ct, code) => self.error(
                "getPID",
                format_args!(
                    "XPS {port}, Axis: {axis}, Error with PositionerCorrector{}Get. XPS API Error: {code}\n",
                    ct.short_name()
                ),
            ),
            PidFailure::Unsupported(CorrectorType::NoCorrector, type_name) => self.error(
                "getPID",
                format_args!(
                    ". XPS {port} axis {axis} corrector type is {type_name}. Cannot get PID.\n"
                ),
            ),
            PidFailure::Unsupported(_, type_name) => self.error(
                "getPID",
                format_args!(
                    ". XPS {port} axis {axis} not a valid corrector type: {type_name}. Cannot set PID.\n"
                ),
            ),
        }
    }

    /// C `doMoveToHome`'s failure exit: print, then `GroupKill` on the move
    /// socket when C issues one.
    fn move_to_home_failed(&self, message: &str, kill: bool) -> AsynError {
        self.error("XPSAxis::doMoveToHome", format_args!("{message}\n"));
        if kill {
            let _ = self.move_sock.group_kill(&self.group_name);
        }
        AsynError::Status {
            status: AsynStatus::Error,
            message: format!("XPS move-to-home for {}: {message}", self.group_name),
        }
    }
}

/// `asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: ...")` on an axis user.
fn trace_error(trace: &MotorTrace, function: &str, args: fmt::Arguments<'_>) {
    trace.print(
        TraceMask::ERROR,
        0,
        format_args!("{DRIVER_NAME}:{function}: {args}"),
    );
}

/// The group name is the prefix of `group.positioner` before the first `.`
/// (C `XPSAxis` constructor terminates `groupName_` at the `.`).
fn group_of(positioner: &str) -> &str {
    positioner.split('.').next().unwrap_or(positioner)
}

/// Decode the hardware end-of-run limits from a `PositionerErrorGet` bitmask
/// (`XPSAxis::poll`): `(high_limit, low_limit)`. Faithful to C's `error & mask`
/// test — because both masks include bit 31, a code with only bit 31 set (or
/// either real end-of-run code) reports *both* limits.
fn limits_from_error(error: u32) -> (bool, bool) {
    (
        error & XPSC8_END_OF_RUN_PLUS != 0,
        error & XPSC8_END_OF_RUN_MINUS != 0,
    )
}

/// Motor status bits derived purely from an XPS group status code, factored out
/// for testing against the C boundary values (`XPSAxis::poll`). Assumes
/// `referencingMode == 0` (the standard-home case; the move-to-home referencing
/// mode is not modeled in the core layer).
#[derive(Debug, PartialEq, Eq)]
struct StatusFlags {
    /// Group is moving/homing/jogging (43..=48) → `!done`.
    group_moving: bool,
    /// Encoder home switch active (ATHM).
    encoder_home: bool,
    /// Axis has been homed/referenced.
    homed: bool,
    /// Power is on / closed loop active.
    powered: bool,
    /// Axis cannot move (disabled / uninitialised / not referenced).
    problem: bool,
}

impl StatusFlags {
    fn from_status(status: i32) -> Self {
        StatusFlags {
            group_moving: (43..=48).contains(&status),
            encoder_home: status == XPS_STATUS_HOMING_DONE,
            homed: (10..=21).contains(&status) || status == 44 || status == 45 || status == 47,
            powered: (10..=19).contains(&status)
                || (43..=49).contains(&status)
                || matches!(status, 56 | 64 | 68 | 70 | 77 | 79),
            problem: status < 10 || (20..=42).contains(&status) || status == 50 || status == 64,
        }
    }
}

impl AsynMotor for XpsAxis {
    /// C `XPSAxis::poll`'s `done` label sets only `motorStatusCommsError_`.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::COMMS_ERROR
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let device_units = position * self.step_size;
        {
            let ctrl = self.lock_controller();
            self.prepare_move(&ctrl, velocity, acceleration)?;
        }
        self.move_result(
            "GroupMoveAbsolute",
            self.move_sock
                .group_move_absolute(&self.positioner_name, device_units),
        )?;
        self.moving = true;
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
        // C `move()` has a native relative branch (GroupMoveRelative); override
        // the trait's poll-then-absolute default.
        let device_units = distance * self.step_size;
        {
            let ctrl = self.lock_controller();
            self.prepare_move(&ctrl, velocity, acceleration)?;
        }
        self.move_result(
            "GroupMoveRelative",
            self.move_sock
                .group_move_relative(&self.positioner_name, device_units),
        )?;
        self.moving = true;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let jog_error = |command: &str, e: &XpsError| {
            self.error(
                "moveVelocity",
                format_args!(
                    "[{},{}]: error calling {command} error={}\n",
                    self.port(),
                    self.axis_no(),
                    e.code()
                ),
            )
        };
        {
            let ctrl = self.lock_controller();
            ctrl.poll_socket()
                .group_jog_mode_enable(&self.group_name)
                .inspect_err(|e| jog_error("GroupJogModeEnable", e))?;
        }
        self.move_sock
            .group_jog_parameters_set(
                &self.positioner_name,
                velocity * self.step_size,
                acceleration * self.step_size,
            )
            .inspect_err(|e| jog_error("GroupJogParametersSet", e))?;
        self.moving = true;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // The XPS home search takes no direction; C ignores `forwards`.
        let home_error = |ctrl: &XpsController, command: &str, e: &XpsError| {
            self.error(
                "home",
                format_args!(
                    "[{},{}]: error calling {command} error={}\n",
                    self.port(),
                    self.axis_no(),
                    Self::error_string(ctrl, e.code())
                ),
            )
        };
        {
            let mut ctrl = self.lock_controller();
            // C home() clears referencing mode for every axis in the group.
            ctrl.set_group_referencing(&self.group_name, false);
            let ctrl = &*ctrl;
            let sock = ctrl.poll_socket();
            // A Ready group (10..=18) will refuse home; kill it first.
            let status = sock.group_status_get(&self.group_name)?;
            if (10..=18).contains(&status) {
                sock.group_kill(&self.group_name)
                    .inspect_err(|e| home_error(ctrl, "GroupKill", e))?;
            }
            // If not initialized, initialize it.
            let status = sock.group_status_get(&self.group_name)?;
            if (0..=9).contains(&status) || status == 50 || status == 63 {
                sock.group_initialize(&self.group_name)
                    .inspect_err(|e| home_error(ctrl, "GroupInitialize", e))?;
            }
        }
        if let Err(e) = self.move_sock.group_home_search(&self.group_name) {
            home_error(&self.lock_controller(), "GroupHomeSearch", &e);
            return Err(e.into());
        }
        self.moving = true;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // Captures only `trace`, so `axis_status` stays assignable below.
        let stop_error = |command: &str, e: &XpsError| {
            trace_error(
                &self.trace,
                "stopAxis",
                format_args!(
                    "[{},{}]: error calling {command} status={}\n",
                    self.trace.port(),
                    self.trace.addr(),
                    e.code()
                ),
            )
        };
        // A jog is stopped differently from a move, so read the status first.
        let status = {
            let ctrl = self.lock_controller();
            ctrl.poll_socket()
                .group_status_get(&self.group_name)
                .inspect_err(|e| stop_error("GroupStatusGet", e))?
        };
        self.axis_status = status;
        if (status == 44 || status == 45 || status == 47)
            && let Err(e) = self.move_sock.group_move_abort(&self.group_name)
        {
            stop_error("GroupMoveAbort", &e);
            // C retries the abort once before giving up.
            let _ = self.move_sock.group_move_abort(&self.group_name);
            return Err(e.into());
        }
        if status == 43 {
            self.move_sock
                .group_kill(&self.group_name)
                .inspect_err(|e| stop_error("GroupKill", e))?;
        }
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock_controller();
        if !ctrl.enable_set_position() {
            self.error(
                "setPosition",
                format_args!(
                    "XPS set position is disabled. Enable it by setting enableSetPosition parameter in XPSCreateController).\n"
                ),
            );
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "XPS set position is disabled (enableSetPosition=0)".into(),
            });
        }
        match ctrl.set_position(
            &self.positioner_name,
            &self.group_name,
            position * self.step_size,
        ) {
            Ok(()) => Ok(()),
            Err(SetPositionFailure::CurrentGet(code)) => {
                self.error(
                    "setPosition",
                    format_args!(
                        "Error performing GroupPositionCurrentGet({},{}). Aborting set position. XPS API Error: {code}.\n",
                        self.port(),
                        self.axis_no()
                    ),
                );
                Err(XpsError::Api(code).into())
            }
            Err(SetPositionFailure::Initialize { code, grouped }) => {
                if grouped {
                    self.error(
                        "setPosition",
                        format_args!(
                            "Error performing GroupKill/GroupInitialize({},{}). Aborting set position. XPS API Error: {code}.\n",
                            self.port(),
                            self.axis_no()
                        ),
                    );
                } else {
                    self.error(
                        "setPosition",
                        format_args!(
                            "Error performing GroupKill/GroupInitialize({},{}). XPS API Error: {code}. Aborting set position.\n",
                            self.port(),
                            self.axis_no()
                        ),
                    );
                }
                Err(XpsError::Api(code).into())
            }
            Err(SetPositionFailure::ReferencingStop { code, grouped }) => {
                self.error(
                    "setPosition",
                    format_args!(
                        "Error performing referencing set position ({},{}). XPS API Error: {code}.{}",
                        self.port(),
                        self.axis_no(),
                        if grouped { "\n" } else { "" }
                    ),
                );
                Ok(())
            }
        }
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock_controller();
        let sock = ctrl.poll_socket();
        let (command, result) = if enable {
            (
                "GroupMotionEnable",
                sock.group_motion_enable(&self.group_name),
            )
        } else {
            (
                "GroupMotionDisable",
                sock.group_motion_disable(&self.group_name),
            )
        };
        result.inspect_err(|e| {
            self.error(
                "setClosedLoop",
                format_args!(
                    "[{},{}]: error calling {command} status={}\n",
                    self.port(),
                    self.axis_no(),
                    e.code()
                ),
            )
        })?;
        Ok(())
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `setHighLimit`: read both travel limits, then rewrite with the new
        // (device-unit) high limit and the existing low limit.
        let device_value = position * self.step_size;
        let ctrl = self.lock_controller();
        let sock = ctrl.poll_socket();
        let (low, _high) = sock
            .positioner_user_travel_limits_get(&self.positioner_name)
            .inspect_err(|e| self.limits_get_error("setHighLimit", e))?;
        sock.positioner_user_travel_limits_set(&self.positioner_name, low, device_value)
            .inspect_err(|e| {
                self.error(
                    "setHighLimit",
                    format_args!(
                        "[{},{}]: error performing PositionerUserTravelLimitsSet for highLim={device_value:.6} status={}\n",
                        self.port(),
                        self.axis_no(),
                        e.code()
                    ),
                )
            })?;
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `setLowLimit`: read both travel limits, then rewrite with the new
        // (device-unit) low limit and the existing high limit.
        let device_value = position * self.step_size;
        let ctrl = self.lock_controller();
        let sock = ctrl.poll_socket();
        let (_low, high) = sock
            .positioner_user_travel_limits_get(&self.positioner_name)
            .inspect_err(|e| self.limits_get_error("setLowLimit", e))?;
        sock.positioner_user_travel_limits_set(&self.positioner_name, device_value, high)
            .inspect_err(|e| {
                self.error(
                    "setLowLimit",
                    format_args!(
                        "[{},{}]: error performing PositionerUserTravelLimitsSet for lowLim={device_value:.6} status={}\n",
                        self.port(),
                        self.axis_no(),
                        e.code()
                    ),
                )
            })?;
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let ctrl = self.lock_controller();
        corrector::set_pid(ctrl.poll_socket(), &self.positioner_name, kind, gain).map_err(
            |failure| {
                self.report_pid_failure(&failure);
                AsynError::Status {
                    status: AsynStatus::Error,
                    message: format!("XPS setPID failed: {failure:?}"),
                }
            },
        )
    }

    fn move_to_home(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // Port of XPSAxis::doMoveToHome. The trait `position` is the referencing
        // move distance (C getReferencingModeMove); zero means "not enabled".
        // C computes velocity/2 from SGamma and ignores the passed vel/accel.
        //
        // Runs synchronously. C uses a dedicated background thread; here the
        // controller lock is released between every RPC and during sleeps, so
        // sibling axes keep polling and only this axis's own command task
        // blocks while homing. DEVIATION: C's final wait loop is unbounded
        // (safe on its own thread); we cap it (below) so a home switch that is
        // never reached cannot hang the async worker forever.
        if position == 0.0 {
            let message = format!(
                "This function has not been enabled for axis {}",
                self.axis_no()
            );
            return Err(self.move_to_home_failed(&message, false));
        }
        let mut distance = position * self.step_size;

        // C ignores the status of every call here but GroupInitialize and the
        // ones it prints for; a failed read leaves C's out-parameter unchanged.
        let mut group_status = 0;
        // A Ready group (10..=18) refuses the sequence; kill it (move socket).
        self.refresh_group_status(&mut group_status);
        if (10..=18).contains(&group_status) {
            let _ = self.move_sock.group_kill(&self.group_name);
        }
        thread::sleep(XPS_REFERENCING_SETTLE);
        let initialized = {
            let ctrl = self.lock_controller();
            ctrl.poll_socket().group_initialize(&self.group_name)
        };
        if initialized.is_err() {
            return Err(self.move_to_home_failed("error calling GroupInitialize", false));
        }
        thread::sleep(XPS_REFERENCING_SETTLE);
        let _ = self.move_sock.group_referencing_start(&self.group_name);
        thread::sleep(XPS_REFERENCING_SETTLE);
        let _ = self.move_sock.group_referencing_stop(&self.group_name);
        thread::sleep(XPS_REFERENCING_SETTLE);

        // Must now be in referencing mode (status 11).
        self.refresh_group_status(&mut group_status);
        if group_status != XPS_STATUS_HOMING_DONE {
            return Err(
                self.move_to_home_failed("error putting axis into referencing mode.", false)
            );
        }
        // Flag referencing for the whole group (poll suppresses home/homed).
        {
            let mut ctrl = self.lock_controller();
            ctrl.set_group_referencing(&self.group_name, true);
        }

        // Pick the search direction from which side of the home switch we start.
        let initial_hw = {
            let ctrl = self.lock_controller();
            ctrl.poll_socket()
                .positioner_hardware_status_get(&self.positioner_name)
        };
        let Ok(initial_hw) = initial_hw else {
            return Err(
                self.move_to_home_failed("error calling PositionerHardwareStatusGet.", false)
            );
        };
        distance = referencing_distance(distance, initial_hw);

        // Halve the velocity so the user can intervene before an overrun.
        let slowed = {
            let ctrl = self.lock_controller();
            let sock = ctrl.poll_socket();
            match sock.positioner_sgamma_parameters_get(&self.positioner_name) {
                Err(_) => Err("Error performing PositionerSGammaParametersGet."),
                Ok((vel, accel, min_jerk, max_jerk)) => sock
                    .positioner_sgamma_parameters_set(
                        &self.positioner_name,
                        vel / 2.0,
                        accel,
                        min_jerk,
                        max_jerk,
                    )
                    .map_err(|_| "Error performing PositionerSGammaParametersSet."),
            }
        };
        if let Err(message) = slowed {
            return Err(self.move_to_home_failed(message, true));
        }
        thread::sleep(XPS_REFERENCING_SETTLE);

        // Move toward the home switch (move socket).
        if self
            .move_sock
            .group_move_relative(&self.positioner_name, distance)
            .is_err()
        {
            return Err(self.move_to_home_failed("Error performing GroupMoveRelative.", true));
        }
        thread::sleep(XPS_REFERENCING_POLL);

        // Must be moving (status 44) to begin the switch-crossing wait.
        self.refresh_group_status(&mut group_status);
        if group_status != 44 {
            return Err(self.move_to_home_failed("Error performing GroupMoveRelative.", true));
        }
        // Wait until the hardware status flips (crossed the switch), aborting
        // if the move stops for any other reason. Bounded (see DEVIATION above).
        let mut hardware_status = 0;
        let mut polls = 0;
        loop {
            thread::sleep(XPS_REFERENCING_POLL);
            self.refresh_hardware_status(&mut hardware_status);
            if hardware_status != initial_hw {
                break;
            }
            self.refresh_group_status(&mut group_status);
            if group_status != 44 {
                // Move finished for some other reason.
                return Err(self.move_to_home_failed("Error performing GroupMoveRelative.", true));
            }
            polls += 1;
            if polls >= XPS_REFERENCING_MAX_POLLS {
                let _ = self.move_sock.group_kill(&self.group_name);
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: format!(
                        "XPS move-to-home for {} timed out before reaching home",
                        self.group_name
                    ),
                });
            }
        }
        // Reached the switch: abort the move (poll socket, as C does).
        let aborted = {
            let ctrl = self.lock_controller();
            ctrl.poll_socket().group_move_abort(&self.group_name)
        };
        if aborted.is_err() {
            return Err(self.move_to_home_failed("Error performing GroupMoveAbort.", true));
        }
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // C's parameter library: every field this poll does not reach keeps
        // its last value, and a failed call ends the poll with only
        // `motorStatusCommsError_` raised (XPSAxis.cpp:755-758).
        let mut status = self.last_status.clone();
        let failed = self.poll_into(&mut status).is_err();
        status.comms_error = failed;
        self.last_status = status.clone();
        Ok(status)
    }
}

impl XpsAxis {
    /// C `XPSAxis::poll` up to its `done:` label: writes each field as its
    /// call succeeds, and stops at the first failed call.
    fn poll_into(&mut self, status: &mut MotorStatus) -> XpsResult<()> {
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let sock = ctrl.poll_socket();
        let poll_error = |command: &str, e: &XpsError| {
            trace_error(
                &self.trace,
                "poll",
                format_args!(
                    "[{},{}]: error calling {command} status={}\n",
                    self.trace.port(),
                    self.trace.addr(),
                    e.code()
                ),
            )
        };

        // C prints the GroupStatusGet line for a failed GroupStatusStringGet too.
        let axis_status = sock
            .group_status_get(&self.group_name)
            .inspect(|&code| self.axis_status = code)
            .and_then(|code| sock.group_status_string_get(code).map(|_| code))
            .inspect_err(|e| poll_error("GroupStatusGet", e))?;
        let flags = StatusFlags::from_status(axis_status);
        // C reads the group state for every axis in the group; moving mode
        // instead waits for this axis's move socket to answer the move.
        let moving_mode = ctrl.moving_mode();
        if !flags.group_moving {
            self.moving = false;
        } else if !moving_mode {
            self.moving = true;
        }
        if moving_mode && self.moving && self.move_sock.read_pending() > 0 {
            self.moving = false;
        }
        status.done = !self.moving;

        // A TCL script may change the soft limits; C re-reads them each poll
        // and ignores a failure.
        let _ = sock.positioner_user_travel_limits_get(&self.positioner_name);

        // In referencing (move-to-home) mode C reports the axis as not homed /
        // not at home regardless of status (XPSAxis::poll 636/648).
        let referencing = ctrl.is_group_referencing(&self.group_name);
        status.encoder_home = flags.encoder_home && !referencing;
        status.homed = flags.homed && !referencing;
        status.powered = flags.powered;
        status.problem = flags.problem && !(ctrl.no_disable_error() && axis_status == 20);

        let encoder = sock
            .group_position_current_get(&self.positioner_name)
            .inspect_err(|e| poll_error("GroupPositionCurrentGet", e))?;
        status.encoder_position = encoder / self.step_size;
        let setpoint = sock
            .group_position_setpoint_get(&self.positioner_name)
            .inspect_err(|e| poll_error("GroupPositionSetpointGet", e))?;
        status.position = setpoint / self.step_size;
        let positioner_error = sock
            .positioner_error_get(&self.positioner_name)
            .inspect_err(|e| poll_error("PositionerErrorGet", e))?;
        (status.high_limit, status.low_limit) = limits_from_error(positioner_error);
        // C's line names GroupPositionVelocityGet for this call (XPSAxis.cpp:746).
        let velocity = sock
            .group_velocity_current_get(&self.positioner_name)
            .inspect_err(|e| poll_error("GroupPositionVelocityGet", e))?;
        status.velocity = velocity / self.step_size;
        status.direction = velocity > XPS_VELOCITY_DEADBAND;
        // Motion-done is the group-status view; `moving` is velocity-derived
        // (C `motorStatusDone_` vs `motorStatusMoving_`).
        status.moving = velocity.abs() > XPS_VELOCITY_DEADBAND;
        Ok(())
    }
}

/// Move-to-home search direction (C `doMoveToHome`): the referencing distance
/// keeps its sign when the positioner starts at the high level of the home (ZM)
/// zone, and is reversed otherwise, so the search always drives toward the
/// switch.
fn referencing_distance(distance: f64, hardware_status: i32) -> f64 {
    if hardware_status & XPSC8_ZM_HIGH_LEVEL == 0 {
        -distance
    } else {
        distance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::xps::fake::{Fail, FakeLine, Log};
    use crate::xps::rpc::SocketMode;

    fn fake_axis(port: &str, fail: &'static str) -> (XpsAxis, Fail, Log) {
        let (mut axes, fail, log) = fake_axes(port, fail, false, &["G1.P1"]);
        (axes.remove(0), fail, log)
    }

    /// One axis per positioner, numbered in order, on one controller.
    fn fake_axes(
        port: &str,
        fail: &'static str,
        enable_set_position: bool,
        positioners: &[&str],
    ) -> (Vec<XpsAxis>, Fail, Log) {
        let line = FakeLine::new(port, fail);
        let controller = Arc::new(Mutex::new(
            XpsController::new(
                line.socket(SocketMode::Query),
                enable_set_position,
                Duration::ZERO,
            )
            .unwrap(),
        ));
        let axes = positioners
            .iter()
            .enumerate()
            .map(|(n, p)| {
                XpsAxis::new(
                    controller.clone(),
                    line.socket(SocketMode::Fire),
                    "MOTOR_T",
                    n as i32,
                    p,
                    1.0,
                )
            })
            .collect();
        (axes, line.fail.clone(), line.log.clone())
    }

    /// XPSAxis.cpp:267, 718, 706: record steps times stepSize_ go out, and
    /// the readbacks come back divided by it.
    #[test]
    fn record_steps_scale_by_step_size() {
        let line = FakeLine::new("xps_steps", "");
        let controller = Arc::new(Mutex::new(
            XpsController::new(line.socket(SocketMode::Query), false, Duration::ZERO).unwrap(),
        ));
        let mut axis = XpsAxis::new(
            controller,
            line.socket(SocketMode::Fire),
            "MOTOR_T",
            0,
            "G1.P1",
            0.5,
        );
        let user = AsynUser::new(0);
        line.log.lock().unwrap().clear();
        axis.move_absolute(&user, 4.0, 0.0, 6.0, 8.0).unwrap();
        axis.move_relative(&user, -3.0, 0.0, 6.0, 8.0).unwrap();
        let log = std::mem::take(&mut *line.log.lock().unwrap());
        assert!(
            log.iter().any(|c| c == "GroupMoveAbsolute (G1.P1,2)"),
            "{log:?}"
        );
        assert!(
            log.iter().any(|c| c == "GroupMoveRelative (G1.P1,-1.5)"),
            "{log:?}"
        );
        let status = axis.poll(&user).unwrap();
        assert_eq!((status.position, status.encoder_position), (2.5, 3.0));
    }

    /// A group in state 44 (moving) whose move replies wait until `answer`.
    struct LateMoveXps {
        base: epics_rs::asyn::port::PortDriverBase,
        reply: Option<String>,
        answer: Arc<Mutex<bool>>,
    }

    impl epics_rs::asyn::port::PortDriver for LateMoveXps {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data);
            self.reply = match cmd.split('(').next().unwrap_or("").trim() {
                "GroupMoveAbsolute" => None,
                "GroupStatusGet" => Some("0,44,EndOfAPI".into()),
                _ => Some("0,0,EndOfAPI".into()),
            };
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = self
                .reply
                .take()
                .or_else(|| {
                    std::mem::take(&mut *self.answer.lock().unwrap()).then(|| "0,EndOfAPI".into())
                })
                .ok_or_else(|| AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "no reply".into(),
                })?;
            buf[..reply.len()].copy_from_slice(reply.as_bytes());
            Ok(reply.len())
        }
    }

    /// Two axes of one moving group; the first has sent a move.
    fn late_move_axes(port: &str, moving_mode: bool) -> (XpsAxis, XpsAxis, Arc<Mutex<bool>>) {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let answer = Arc::new(Mutex::new(false));
        let line = LateMoveXps {
            base: PortDriverBase::new(port, 1, PortFlags::default()),
            reply: None,
            answer: answer.clone(),
        };
        let (rt, _jh) =
            create_port_runtime(&claim_port(port).unwrap(), line, RuntimeConfig::default())
                .unwrap();
        let socket = |mode| {
            XpsSocket::new(
                rt.port_handle().clone(),
                port,
                mode,
                Duration::from_millis(100),
            )
        };
        let controller = Arc::new(Mutex::new(
            XpsController::new(socket(SocketMode::Query), false, Duration::ZERO).unwrap(),
        ));
        if moving_mode {
            controller.lock().unwrap().set_moving_mode();
        }
        let axis = |n, p| {
            XpsAxis::new(
                controller.clone(),
                socket(SocketMode::Fire),
                port,
                n,
                p,
                1.0,
            )
        };
        let (mut mover, other) = (axis(0, "G1.P1"), axis(1, "G1.P2"));
        let user = AsynUser::new(0);
        mover.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        (mover, other, answer)
    }

    /// C `XPSAxis::poll` with `enableMovingMode_`: the mover is done when its
    /// move socket answers, and a group mate that sent nothing is not moving.
    #[test]
    fn moving_mode_waits_for_the_move_socket_not_the_group() {
        let (mut mover, mut other, answer) = late_move_axes("xps_moving_mode", true);
        let user = AsynUser::new(0);
        assert!(!mover.poll(&user).unwrap().done);
        assert!(other.poll(&user).unwrap().done);
        *answer.lock().unwrap() = true;
        assert!(mover.poll(&user).unwrap().done);
        assert!(mover.poll(&user).unwrap().done);
    }

    /// Without moving mode every axis of a moving group is moving, and the
    /// move socket is not read.
    #[test]
    fn without_moving_mode_the_group_state_decides() {
        let (mut mover, mut other, answer) = late_move_axes("xps_group_mode", false);
        let user = AsynUser::new(0);
        *answer.lock().unwrap() = true;
        assert!(!mover.poll(&user).unwrap().done);
        assert!(!other.poll(&user).unwrap().done);
        assert!(*answer.lock().unwrap());
    }

    /// C `XPSAxis::poll` (`done:`, XPSAxis.cpp:755-758): a failed call raises
    /// only COMM_ERR, keeps what this poll already wrote and every older value.
    #[test]
    fn a_failed_poll_raises_only_comms_error() {
        let (mut axis, fail, _) = fake_axis("xps_poll_fail", "");
        let user = AsynUser::new(0);

        *fail.lock().unwrap() = "PositionerErrorGet";
        let s = axis.poll(&user).unwrap();
        assert!(s.comms_error);
        assert!(!s.problem, "status 12 is Ready");
        assert!(s.homed);
        assert_eq!((s.encoder_position, s.position), (1.5, 1.25));

        *fail.lock().unwrap() = "";
        assert!(!axis.poll(&user).unwrap().comms_error);

        *fail.lock().unwrap() = "GroupStatusGet";
        let s = axis.poll(&user).unwrap();
        assert!(s.comms_error && s.homed && !s.problem);
        assert_eq!(s.position, 1.25);
    }

    /// The C constructor ignores a failed PositionerSGammaParametersGet and a
    /// failed getPID (XPSAxis.cpp:185-197).
    #[test]
    fn a_failed_constructor_read_still_creates_the_axis() {
        for (port, call) in [
            ("xps_new_sgamma", "PositionerSGammaParametersGet"),
            ("xps_new_pid", "PositionerCorrectorTypeGet"),
        ] {
            let (axis, _, _) = fake_axis(port, call);
            assert_eq!(axis.positioner_name, "G1.P1", "{call}");
        }
    }

    /// C doMoveToHome ignores GroupKill and GroupReferencingStart/Stop
    /// (XPSAxis.cpp:1633-1652); the status-11 check is what refuses.
    #[test]
    fn move_to_home_ignores_the_calls_c_ignores() {
        for (port, call) in [
            ("xps_mth_kill", "GroupKill"),
            ("xps_mth_refstart", "GroupReferencingStart"),
            ("xps_mth_status", "GroupStatusGet"),
        ] {
            let (mut axis, fail, log) = fake_axis(port, "");
            let user = AsynUser::new(0);
            *fail.lock().unwrap() = call;
            log.lock().unwrap().clear();
            assert!(axis.move_to_home(&user, 1.0, 0.0, 1.0, 1.0).is_err());
            let log = log.lock().unwrap();
            for sent in [
                "GroupInitialize",
                "GroupReferencingStart",
                "GroupReferencingStop",
            ] {
                assert!(
                    log.iter().any(|c| c.starts_with(&format!("{sent} ("))),
                    "{call}: {sent} not sent in {log:?}"
                );
            }
            assert!(
                !log.iter()
                    .any(|c| c.starts_with("PositionerHardwareStatusGet ")),
                "{call}: status 12 is not referencing mode"
            );
        }
    }

    /// XPSAxis.cpp:566-575 fails the poll on GroupStatusStringGet; :628 ignores
    /// a failed PositionerUserTravelLimitsGet.
    #[test]
    fn poll_status_string_fails_and_travel_limits_do_not() {
        let (mut axis, fail, _) = fake_axis("xps_poll_string", "");
        let user = AsynUser::new(0);

        *fail.lock().unwrap() = "GroupStatusStringGet";
        assert!(axis.poll(&user).unwrap().comms_error);

        *fail.lock().unwrap() = "PositionerUserTravelLimitsGet";
        let s = axis.poll(&user).unwrap();
        assert!(!s.comms_error);
        assert_eq!(s.position, 1.25);
    }

    #[test]
    fn group_name_is_prefix_before_dot() {
        assert_eq!(group_of("GROUP1.POSITIONER"), "GROUP1");
        assert_eq!(group_of("XY.X"), "XY");
        // No dot: whole string is the group (C leaves it unterminated).
        assert_eq!(group_of("BARE"), "BARE");
    }

    #[test]
    fn status_flags_moving_range() {
        // 43..=48 are moving/homing/jogging.
        for s in 43..=48 {
            assert!(StatusFlags::from_status(s).group_moving, "status {s}");
        }
        assert!(!StatusFlags::from_status(42).group_moving);
        assert!(!StatusFlags::from_status(49).group_moving);
    }

    #[test]
    fn status_flags_homed_and_home() {
        // Ready/enabled range and the specific moving-but-referenced codes.
        assert!(StatusFlags::from_status(10).homed);
        assert!(StatusFlags::from_status(21).homed);
        assert!(StatusFlags::from_status(44).homed);
        assert!(StatusFlags::from_status(47).homed);
        assert!(!StatusFlags::from_status(9).homed);
        assert!(!StatusFlags::from_status(22).homed);
        assert!(!StatusFlags::from_status(46).homed);
        // Encoder home (ATHM) only at status 11.
        assert!(StatusFlags::from_status(11).encoder_home);
        assert!(!StatusFlags::from_status(10).encoder_home);
    }

    #[test]
    fn status_flags_powered() {
        assert!(StatusFlags::from_status(10).powered);
        assert!(StatusFlags::from_status(19).powered);
        assert!(StatusFlags::from_status(43).powered);
        assert!(StatusFlags::from_status(49).powered);
        assert!(StatusFlags::from_status(56).powered);
        assert!(StatusFlags::from_status(79).powered);
        assert!(!StatusFlags::from_status(20).powered);
        assert!(!StatusFlags::from_status(50).powered);
    }

    #[test]
    fn status_flags_problem() {
        // Uninitialised / disabled / not-referenced states.
        assert!(StatusFlags::from_status(0).problem);
        assert!(StatusFlags::from_status(9).problem);
        assert!(StatusFlags::from_status(20).problem);
        assert!(StatusFlags::from_status(42).problem);
        assert!(StatusFlags::from_status(50).problem);
        assert!(StatusFlags::from_status(64).problem);
        // Ready and moving states are not a problem.
        assert!(!StatusFlags::from_status(10).problem);
        assert!(!StatusFlags::from_status(43).problem);
    }

    #[test]
    fn limits_from_error_masks() {
        // No error → neither limit.
        assert_eq!(limits_from_error(0), (false, false));
        // A real positive/negative end-of-run code matches its own mask; both
        // masks share bit 31, so C's `err & mask` reports BOTH limits for a real
        // end-of-run code (or a bit-31-only code). We reproduce that exactly.
        assert_eq!(limits_from_error(0x8000_0200), (true, true));
        assert_eq!(limits_from_error(0x8000_0100), (true, true));
        assert_eq!(limits_from_error(0x8000_0000), (true, true));
        // An unrelated low-bit error touches neither limit.
        assert_eq!(limits_from_error(0x0000_0004), (false, false));
    }

    #[test]
    fn referencing_distance_picks_direction() {
        // Starting at the high level of the ZM zone: keep the sign (drive down).
        assert_eq!(referencing_distance(5.0, XPSC8_ZM_HIGH_LEVEL), 5.0);
        // A status with the ZM bit set among others still counts as "high".
        assert_eq!(referencing_distance(5.0, XPSC8_ZM_HIGH_LEVEL | 0x1), 5.0);
        // ZM bit clear: reverse toward the switch (drive up).
        assert_eq!(referencing_distance(5.0, 0), -5.0);
        assert_eq!(referencing_distance(5.0, 0x2), -5.0);
        // Sign is flipped, not forced: a negative distance reverses to positive.
        assert_eq!(referencing_distance(-5.0, 0), 5.0);
    }

    /// C `XPSAxis::setPosition` with more than one axis in the group
    /// (XPSAxis.cpp:417-476): read every axis, re-initialise the group, set
    /// each sibling back to its position, then this axis to the new one.
    #[test]
    fn grouped_set_position_resets_the_siblings() {
        let (mut axes, fail, log) =
            fake_axes("xps_setpos_grp", "", true, &["G1.P1", "G2.Q", "G1.P2"]);
        let user = AsynUser::new(0);
        log.lock().unwrap().clear();
        axes[0].set_position(&user, 7.0).unwrap();
        let sent: Vec<String> = log.lock().unwrap().drain(..).collect();
        assert_eq!(
            sent,
            [
                "GroupPositionCurrentGet (G1.P1,double *)",
                "GroupPositionCurrentGet (G2.Q,double *)",
                "GroupPositionCurrentGet (G1.P2,double *)",
                "GroupKill (G1)",
                "GroupInitialize (G1)",
                "GroupReferencingStart (G1)",
                "GroupReferencingActionExecute (G1.P2,SetPosition,None,2.5)",
                "GroupReferencingActionExecute (G1.P1,SetPosition,None,7)",
                "GroupReferencingStop (G1)",
            ]
        );

        *fail.lock().unwrap() = "GroupPositionCurrentGet";
        assert!(axes[2].set_position(&user, 7.0).is_err());
        assert!(
            !log.lock()
                .unwrap()
                .iter()
                .any(|c| c.starts_with("GroupKill "))
        );

        // A lone axis in its group skips the reads.
        *fail.lock().unwrap() = "";
        log.lock().unwrap().clear();
        axes[1].set_position(&user, 3.0).unwrap();
        assert_eq!(log.lock().unwrap()[0], "GroupKill (G2)");
    }
}
