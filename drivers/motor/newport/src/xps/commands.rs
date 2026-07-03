//! Typed wrappers for the XPS-C8 RPC functions the motor driver uses.
//!
//! Each method marshals the exact `FuncName (args)` string the vendor
//! `XPS_C8_drivers.cpp` builds (note the space before `(`, and `%.13g` for
//! doubles via [`format_g`]) and parses the reply's out-parameters. Action
//! functions return `Ok(())` on error code `0`; getters return the parsed
//! values. The socket's [`SocketMode`] decides whether the call waits for the
//! reply (poll socket) or fires without waiting (per-axis move socket), so the
//! same wrapper (e.g. [`XpsSocket::group_kill`]) works on either.
//!
//! [`SocketMode`]: super::rpc::SocketMode

use super::rpc::{XpsResult, XpsSocket, format_g};

/// Double precision used by the vendor library (`%.13g`).
const G13: usize = 13;

/// Format a double for the wire exactly as `XPS_C8_drivers.cpp` (`%.13g`).
fn g(value: f64) -> String {
    format_g(value, G13)
}

impl XpsSocket {
    // --- Controller-level getters -----------------------------------------

    /// `FirmwareVersionGet` → firmware version string.
    pub fn firmware_version_get(&self) -> XpsResult<String> {
        let r = self.exec("FirmwareVersionGet (char *)")?.require_ok()?;
        Ok(r.string(1).to_string())
    }

    /// `ErrorStringGet(code)` → the human-readable error string.
    pub fn error_string_get(&self, code: i32) -> XpsResult<String> {
        let cmd = format!("ErrorStringGet ({code},char *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.string(1).to_string())
    }

    // --- Group status / position ------------------------------------------

    /// `GroupStatusGet(group)` → the group status code.
    pub fn group_status_get(&self, group: &str) -> XpsResult<i32> {
        let cmd = format!("GroupStatusGet ({group},int *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.int(1))
    }

    /// `GroupStatusStringGet(code)` → the status description string.
    pub fn group_status_string_get(&self, code: i32) -> XpsResult<String> {
        let cmd = format!("GroupStatusStringGet ({code},char *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.string(1).to_string())
    }

    /// `GroupPositionCurrentGet(positioner, 1)` → current encoder position.
    pub fn group_position_current_get(&self, positioner: &str) -> XpsResult<f64> {
        let cmd = format!("GroupPositionCurrentGet ({positioner},double *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.double(1))
    }

    /// `GroupPositionSetpointGet(positioner, 1)` → commanded setpoint position.
    pub fn group_position_setpoint_get(&self, positioner: &str) -> XpsResult<f64> {
        let cmd = format!("GroupPositionSetpointGet ({positioner},double *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.double(1))
    }

    /// `GroupVelocityCurrentGet(positioner, 1)` → current velocity.
    pub fn group_velocity_current_get(&self, positioner: &str) -> XpsResult<f64> {
        let cmd = format!("GroupVelocityCurrentGet ({positioner},double *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.double(1))
    }

    /// `PositionerErrorGet(positioner)` → the positioner error bitmask
    /// (the end-of-run limit bits are read from here; bit 31 is set, so this is
    /// a raw `u32`).
    pub fn positioner_error_get(&self, positioner: &str) -> XpsResult<u32> {
        let cmd = format!("PositionerErrorGet ({positioner},int *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.bits(1))
    }

    // --- SGamma (velocity/accel/jerk) profile -----------------------------

    /// `PositionerSGammaParametersGet(positioner)` →
    /// `(velocity, acceleration, minJerkTime, maxJerkTime)`.
    pub fn positioner_sgamma_parameters_get(
        &self,
        positioner: &str,
    ) -> XpsResult<(f64, f64, f64, f64)> {
        let cmd = format!(
            "PositionerSGammaParametersGet ({positioner},double *,double *,double *,double *)"
        );
        let r = self.exec(&cmd)?.require_ok()?;
        Ok((r.double(1), r.double(2), r.double(3), r.double(4)))
    }

    /// `PositionerSGammaParametersSet(positioner, vel, accel, minJerk, maxJerk)`.
    pub fn positioner_sgamma_parameters_set(
        &self,
        positioner: &str,
        velocity: f64,
        acceleration: f64,
        min_jerk: f64,
        max_jerk: f64,
    ) -> XpsResult<()> {
        let cmd = format!(
            "PositionerSGammaParametersSet ({positioner},{},{},{},{})",
            g(velocity),
            g(acceleration),
            g(min_jerk),
            g(max_jerk),
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    // --- Travel limits ----------------------------------------------------

    /// `PositionerUserTravelLimitsGet(positioner)` → `(low, high)` in device
    /// units.
    pub fn positioner_user_travel_limits_get(&self, positioner: &str) -> XpsResult<(f64, f64)> {
        let cmd = format!("PositionerUserTravelLimitsGet ({positioner},double *,double *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok((r.double(1), r.double(2)))
    }

    /// `PositionerUserTravelLimitsSet(positioner, low, high)`.
    pub fn positioner_user_travel_limits_set(
        &self,
        positioner: &str,
        low: f64,
        high: f64,
    ) -> XpsResult<()> {
        let cmd = format!(
            "PositionerUserTravelLimitsSet ({positioner},{},{})",
            g(low),
            g(high),
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    // --- Group life-cycle / motion actions --------------------------------

    /// `GroupInitialize(group)`.
    pub fn group_initialize(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("GroupInitialize ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `GroupKill(group)`.
    pub fn group_kill(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("GroupKill ({group})"))?.require_ok()?;
        Ok(())
    }

    /// `GroupMotionEnable(group)`.
    pub fn group_motion_enable(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("GroupMotionEnable ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `GroupMotionDisable(group)`.
    pub fn group_motion_disable(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("GroupMotionDisable ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `GroupHomeSearch(group)`.
    pub fn group_home_search(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("GroupHomeSearch ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `GroupMoveAbort(group)`.
    pub fn group_move_abort(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("GroupMoveAbort ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `GroupMoveAbsolute(positioner, [target])` (one element).
    pub fn group_move_absolute(&self, positioner: &str, target: f64) -> XpsResult<()> {
        let cmd = format!("GroupMoveAbsolute ({positioner},{})", g(target));
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `GroupMoveRelative(positioner, [displacement])` (one element).
    pub fn group_move_relative(&self, positioner: &str, displacement: f64) -> XpsResult<()> {
        let cmd = format!("GroupMoveRelative ({positioner},{})", g(displacement));
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    // --- Jog (velocity mode) ----------------------------------------------

    /// `GroupJogModeEnable(group)`.
    pub fn group_jog_mode_enable(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("GroupJogModeEnable ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `GroupJogParametersSet(positioner, [velocity], [acceleration])`
    /// (one element).
    pub fn group_jog_parameters_set(
        &self,
        positioner: &str,
        velocity: f64,
        acceleration: f64,
    ) -> XpsResult<()> {
        let cmd = format!(
            "GroupJogParametersSet ({positioner},{},{})",
            g(velocity),
            g(acceleration),
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    // --- Referencing (set-position) ---------------------------------------

    /// `GroupReferencingStart(group)`.
    pub fn group_referencing_start(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("GroupReferencingStart ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `GroupReferencingStop(group)`.
    pub fn group_referencing_stop(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("GroupReferencingStop ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `GroupReferencingActionExecute(positioner, action, sensor, parameter)`.
    pub fn group_referencing_action_execute(
        &self,
        positioner: &str,
        action: &str,
        sensor: &str,
        parameter: f64,
    ) -> XpsResult<()> {
        let cmd = format!(
            "GroupReferencingActionExecute ({positioner},{action},{sensor},{})",
            g(parameter),
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    // --- Position compare output (PCO) ------------------------------------

    /// `PositionerPositionCompareDisable(positioner)`.
    pub fn positioner_position_compare_disable(&self, positioner: &str) -> XpsResult<()> {
        self.exec(&format!("PositionerPositionCompareDisable ({positioner})"))?
            .require_ok()?;
        Ok(())
    }

    /// `PositionerPositionCompareEnable(positioner)`.
    pub fn positioner_position_compare_enable(&self, positioner: &str) -> XpsResult<()> {
        self.exec(&format!("PositionerPositionCompareEnable ({positioner})"))?
            .require_ok()?;
        Ok(())
    }

    /// `PositionerPositionComparePulseParametersSet(positioner, pulseWidth, settlingTime)`.
    pub fn positioner_position_compare_pulse_parameters_set(
        &self,
        positioner: &str,
        pulse_width: f64,
        settling_time: f64,
    ) -> XpsResult<()> {
        let cmd = format!(
            "PositionerPositionComparePulseParametersSet ({positioner},{},{})",
            g(pulse_width),
            g(settling_time),
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `PositionerPositionCompareSet(positioner, min, max, step)`.
    pub fn positioner_position_compare_set(
        &self,
        positioner: &str,
        min_position: f64,
        max_position: f64,
        position_step: f64,
    ) -> XpsResult<()> {
        let cmd = format!(
            "PositionerPositionCompareSet ({positioner},{},{},{})",
            g(min_position),
            g(max_position),
            g(position_step),
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    // --- TCL scripting ----------------------------------------------------

    /// `TCLScriptExecute(file, task, parameters)` — run a TCL script from a file
    /// on the controller. C `XPS_C8_drivers.cpp:358` passes task/parameters as
    /// `"0"`/`"0"` when triggered from the driver.
    pub fn tcl_script_execute(&self, file: &str, task: &str, parameters: &str) -> XpsResult<()> {
        let cmd = format!("TCLScriptExecute ({file},{task},{parameters})");
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }
}
