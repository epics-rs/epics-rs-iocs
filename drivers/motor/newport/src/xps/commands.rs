//! Typed wrappers for the XPS-C8 RPC functions the motor driver uses.
//!
//! Each method marshals the exact `FuncName (args)` string the vendor
//! `XPS_C8_drivers.cpp` builds (note the space before `(`, and `%.13g` for
//! doubles via `g13`) and parses the reply's out-parameters. Action
//! functions return `Ok(())` on error code `0`; getters return the parsed
//! values. The socket's [`SocketMode`] decides whether the call waits for the
//! reply (poll socket) or fires without waiting (per-axis move socket), so the
//! same wrapper (e.g. [`XpsSocket::group_kill`]) works on either.
//!
//! [`SocketMode`]: super::rpc::SocketMode

use epics_rs::base::runtime::stdlib::strtod;
use epics_rs::base::types::c_parse::{long_to_int, strtol};

use super::rpc::{XpsResult, XpsSocket, g13 as g};

/// Marshal an extended-event name list: each element is followed by its four
/// (empty) parameters, elements joined by `,` (C
/// `EventExtendedConfiguration{Trigger,Action}Set` with empty parameter lists).
fn join_with_empty_params(names: &[String]) -> String {
    names
        .iter()
        .map(|n| format!("{n},,,,"))
        .collect::<Vec<_>>()
        .join(",")
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

    /// C `HXPErrorStringGet` (hxp_drivers.cpp:271-303): `out` takes the
    /// reply's string and keeps what it held when there is none, as the
    /// move socket's timed-out `"0"`.
    pub fn error_string_get_into(&self, code: i32, out: &mut String) {
        let cmd = format!("ErrorStringGet ({code},char *)");
        if let Ok(reply) = self.exec(&cmd)
            && reply.code() == 0
            && let Some(text) = reply.field(1)
        {
            *out = text.to_string();
        }
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

    /// `GroupPositionCurrentGet(group, pos.len())` into `pos`: one encoder
    /// position per positioner. As C (XPS_C8_drivers.cpp:3556-3600), an
    /// element keeps what it held when its field is missing or no `%lf`.
    pub fn group_positions_current_get(&self, group: &str, pos: &mut [f64]) -> XpsResult<()> {
        let cmd = format!(
            "GroupPositionCurrentGet ({group},{})",
            vec!["double *"; pos.len()].join(",")
        );
        let r = self.exec(&cmd)?.require_ok()?;
        for (i, p) in pos.iter_mut().enumerate() {
            if let Some(read) = r.field(i + 1).map(|f| strtod(f.as_bytes()))
                && read.len > 0
            {
                *p = read.value;
            }
        }
        Ok(())
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

    /// `GroupMoveAbsolute(name, targets)`: one target for a positioner, one
    /// per positioner for a group.
    pub fn group_move_absolute(&self, name: &str, targets: &[f64]) -> XpsResult<()> {
        let targets: Vec<String> = targets.iter().map(|&t| g(t)).collect();
        let cmd = format!("GroupMoveAbsolute ({name},{})", targets.join(","));
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

    /// `PositionerPositionCompareAquadBWindowedSet(positioner, min, max)`.
    pub fn positioner_position_compare_aquadb_windowed_set(
        &self,
        positioner: &str,
        min_position: f64,
        max_position: f64,
    ) -> XpsResult<()> {
        let cmd = format!(
            "PositionerPositionCompareAquadBWindowedSet ({positioner},{},{})",
            g(min_position),
            g(max_position),
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `PositionerPositionCompareAquadBAlwaysEnable(positioner)`.
    pub fn positioner_position_compare_aquadb_always_enable(
        &self,
        positioner: &str,
    ) -> XpsResult<()> {
        let cmd = format!("PositionerPositionCompareAquadBAlwaysEnable ({positioner})");
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `PositionerHardwareStatusGet(positioner)` → the hardware status bitmask.
    pub fn positioner_hardware_status_get(&self, positioner: &str) -> XpsResult<i32> {
        let cmd = format!("PositionerHardwareStatusGet ({positioner},int *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.int(1))
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

    // --- PVT trajectory profiles ------------------------------------------

    /// `MultipleAxesPVTVerification(group, file)` — verify an uploaded
    /// trajectory against the group's dynamics. A non-zero error code carries
    /// the reason (e.g. -69 acceleration too high, -68 velocity too high,
    /// -70 final velocity non-zero, -75 negative/null delta time).
    pub fn multiple_axes_pvt_verification(&self, group: &str, file: &str) -> XpsResult<()> {
        let cmd = format!("MultipleAxesPVTVerification ({group},{file})");
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `MultipleAxesPVTVerificationResultGet(positioner)` → the last verified
    /// trajectory's `(file, minPos, maxPos, maxVel, maxAccel)` for one
    /// positioner, positions relative to its start. The file name is an
    /// out-parameter (XPS_C8_drivers.cpp:7353).
    pub fn multiple_axes_pvt_verification_result_get(
        &self,
        positioner: &str,
    ) -> XpsResult<(String, f64, f64, f64, f64)> {
        let cmd = format!(
            "MultipleAxesPVTVerificationResultGet ({positioner},char *,double *,double *,double *,double *)"
        );
        let r = self.exec(&cmd)?.require_ok()?;
        Ok((
            r.string(1).to_string(),
            r.double(2),
            r.double(3),
            r.double(4),
            r.double(5),
        ))
    }

    /// `MultipleAxesPVTParametersGet(group)` into `file` and `element`: the
    /// running trajectory and its current element. As C
    /// (XPS_C8_drivers.cpp:7439-7473), each keeps what it held when its
    /// field is missing, and `element` too when its field is no `%d`.
    pub fn multiple_axes_pvt_parameters_get(
        &self,
        group: &str,
        file: &mut String,
        element: &mut i32,
    ) -> XpsResult<()> {
        let cmd = format!("MultipleAxesPVTParametersGet ({group},char *,int *)");
        let r = self.exec(&cmd)?.require_ok()?;
        if let Some(name) = r.field(1) {
            *file = name.to_string();
        }
        if let Some((value, len)) = r.field(2).map(|f| strtol(f, 10))
            && len > 0
        {
            *element = long_to_int(value);
        }
        Ok(())
    }

    /// `MultipleAxesPVTExecution(group, file, executionNumber)` — run the
    /// trajectory. The XPS holds the reply until the trajectory finishes, so
    /// this must run on a socket that is not needed for polling meanwhile.
    pub fn multiple_axes_pvt_execution(
        &self,
        group: &str,
        file: &str,
        executions: i32,
    ) -> XpsResult<()> {
        let cmd = format!("MultipleAxesPVTExecution ({group},{file},{executions})");
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `MultipleAxesPVTPulseOutputSet(group, startElement, endElement, period)`
    /// — output a pulse train while the trajectory runs, from the start of
    /// element `startElement` to the end of element `endElement`, one pulse
    /// every `period` seconds (C `executeProfile` passes user pulses + 1 to
    /// skip the acceleration element).
    pub fn multiple_axes_pvt_pulse_output_set(
        &self,
        group: &str,
        start_element: i32,
        end_element: i32,
        period: f64,
    ) -> XpsResult<()> {
        let cmd = format!(
            "MultipleAxesPVTPulseOutputSet ({group},{start_element},{end_element},{})",
            g(period)
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    // --- GPIO (drvXPSAsynAux) ----------------------------------------------

    /// `GPIOAnalogGet(names)` → one value per name, in order.
    pub fn gpio_analog_get(&self, names: &[&str]) -> XpsResult<Vec<f64>> {
        let args: Vec<String> = names.iter().map(|n| format!("{n},double *")).collect();
        let cmd = format!("GPIOAnalogGet ({})", args.join(","));
        let r = self.exec(&cmd)?.require_ok()?;
        Ok((1..=names.len()).map(|n| r.double(n)).collect())
    }

    /// `GPIOAnalogSet(name, value)`.
    pub fn gpio_analog_set(&self, name: &str, value: f64) -> XpsResult<()> {
        let cmd = format!("GPIOAnalogSet ({name},{})", g(value));
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `GPIOAnalogGainGet(name)` → the input's gain.
    pub fn gpio_analog_gain_get(&self, name: &str) -> XpsResult<i32> {
        let cmd = format!("GPIOAnalogGainGet ({name},int *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.int(1))
    }

    /// `GPIOAnalogGainSet(name, gain)`.
    pub fn gpio_analog_gain_set(&self, name: &str, gain: i32) -> XpsResult<()> {
        let cmd = format!("GPIOAnalogGainSet ({name},{gain})");
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `GPIODigitalGet(name)` → the port's bits (C `%hu`).
    pub fn gpio_digital_get(&self, name: &str) -> XpsResult<u16> {
        let cmd = format!("GPIODigitalGet ({name},unsigned short *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.bits(1) as u16)
    }

    /// `GPIODigitalSet(name, mask, value)`.
    pub fn gpio_digital_set(&self, name: &str, mask: u16, value: u16) -> XpsResult<()> {
        let cmd = format!("GPIODigitalSet ({name},{mask},{value})");
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    // --- Gathering + events (trajectory readback) ---------------------------

    /// `GatheringReset()` — empty the in-memory gathering buffer. Required
    /// before each run because `GatheringOneData` only appends.
    pub fn gathering_reset(&self) -> XpsResult<()> {
        self.exec("GatheringReset ()")?.require_ok()?;
        Ok(())
    }

    /// `GatheringConfigurationSet(type, type, ...)` — define what each
    /// `GatheringOneData` event samples. The C wrapper splits its `;`-separated
    /// list and re-joins with `,` on the wire; we take the list directly.
    pub fn gathering_configuration_set(&self, types: &[String]) -> XpsResult<()> {
        let cmd = format!("GatheringConfigurationSet ({})", types.join(","));
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `GatheringCurrentNumberGet` → `(currentSamples, maximumSamples)`.
    pub fn gathering_current_number_get(&self) -> XpsResult<(i32, i32)> {
        let r = self
            .exec("GatheringCurrentNumberGet (int *,int *)")?
            .require_ok()?;
        Ok((r.int(1), r.int(2)))
    }

    /// `GatheringStop()` — stop sampling, leaving the data in memory for
    /// `GatheringDataMultipleLinesGet`. The caller should tolerate error `-30`
    /// (gathering never started, e.g. the trajectory was aborted before one
    /// element completed) like C `executeProfile`.
    pub fn gathering_stop(&self) -> XpsResult<()> {
        self.exec("GatheringStop ()")?.require_ok()?;
        Ok(())
    }

    /// `GatheringDataMultipleLinesGet(indexPoint, numberOfLines, char *)` →
    /// the raw multi-line data block (lines of `;`-separated values separated
    /// by `\n`). The controller rejects requests whose reply would be too
    /// large; C halves `numberOfLines` and retries on any error.
    pub fn gathering_data_multiple_lines_get(&self, index: i32, lines: i32) -> XpsResult<String> {
        let cmd = format!("GatheringDataMultipleLinesGet ({index},{lines},char *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok(r.string(1).to_string())
    }

    /// `EventExtendedConfigurationTriggerSet(...)` — configure the event
    /// triggers. Each name is marshalled with its four (empty) parameters, so
    /// `["Always", "G1.PVT.TrajectoryPulse"]` becomes
    /// `(Always,,,,,G1.PVT.TrajectoryPulse,,,,)` exactly as the C wrapper
    /// emits for the driver's empty parameter lists.
    pub fn event_extended_configuration_trigger_set(&self, events: &[String]) -> XpsResult<()> {
        let cmd = format!(
            "EventExtendedConfigurationTriggerSet ({})",
            join_with_empty_params(events)
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `EventExtendedConfigurationActionSet(...)` — configure the actions run
    /// on each trigger (same four-empty-parameter marshalling as the trigger
    /// set; the driver uses `["GatheringOneData"]`).
    pub fn event_extended_configuration_action_set(&self, actions: &[String]) -> XpsResult<()> {
        let cmd = format!(
            "EventExtendedConfigurationActionSet ({})",
            join_with_empty_params(actions)
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `EventExtendedStart` → the event ID for `event_extended_remove`.
    pub fn event_extended_start(&self) -> XpsResult<i32> {
        let r = self.exec("EventExtendedStart (int *)")?.require_ok()?;
        Ok(r.int(1))
    }

    /// `EventExtendedRemove(id)` — remove the event/action configuration
    /// started by `event_extended_start`.
    pub fn event_extended_remove(&self, id: i32) -> XpsResult<()> {
        let cmd = format!("EventExtendedRemove ({id})");
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    // --- Single-axis slave (gearing), xpsSlave.st -------------------------

    /// `SingleAxisSlaveModeEnable(group)`.
    pub fn single_axis_slave_mode_enable(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("SingleAxisSlaveModeEnable ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `SingleAxisSlaveModeDisable(group)`.
    pub fn single_axis_slave_mode_disable(&self, group: &str) -> XpsResult<()> {
        self.exec(&format!("SingleAxisSlaveModeDisable ({group})"))?
            .require_ok()?;
        Ok(())
    }

    /// `SingleAxisSlaveParametersSet(group, masterPositioner, ratio)`.
    pub fn single_axis_slave_parameters_set(
        &self,
        group: &str,
        master: &str,
        ratio: f64,
    ) -> XpsResult<()> {
        let cmd = format!(
            "SingleAxisSlaveParametersSet ({group},{master},{})",
            g(ratio)
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `SingleAxisSlaveParametersGet(group)` into `master` and `ratio`. As C
    /// (XPS_C8_drivers.cpp:7690-7705), each keeps what it held when its
    /// field is missing, and `ratio` too when its field is no `%lf`.
    pub fn single_axis_slave_parameters_get(
        &self,
        group: &str,
        master: &mut String,
        ratio: &mut f64,
    ) -> XpsResult<()> {
        let cmd = format!("SingleAxisSlaveParametersGet ({group},char *,double *)");
        let r = self.exec(&cmd)?.require_ok()?;
        if let Some(name) = r.field(1) {
            *master = name.to_string();
        }
        if let Some(read) = r.field(2).map(|f| strtod(f.as_bytes()))
            && read.len > 0
        {
            *ratio = read.value;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::super::fake::FakeLine;
    use super::super::rpc::SocketMode;

    /// XPS_C8_drivers.cpp:7353-7385: the file name is an out-parameter, so
    /// the bounds are reply fields 2-5.
    #[test]
    fn pvt_verification_result_get_marshals_as_c() {
        let line = FakeLine::new("xps_pvt_result", "");
        let sock = line.socket(SocketMode::Query);
        assert_eq!(
            sock.multiple_axes_pvt_verification_result_get("G1.P1")
                .unwrap(),
            ("traj.trj".into(), -0.5, 2.5, 1.2, 3.4)
        );
        assert_eq!(
            line.take_log(),
            [
                "MultipleAxesPVTVerificationResultGet (G1.P1,char *,double *,double *,double *,double *)"
            ]
        );
    }

    /// XPS_C8_drivers.cpp:3556-3600: one `double *` per element; an element
    /// whose field is no number, or is missing, keeps its value, and a
    /// failed call leaves them all.
    #[test]
    fn group_positions_current_get_marshals_as_c() {
        let line = FakeLine::new("xps_group_positions", "");
        let sock = line.socket(SocketMode::Query);
        let mut pos = [9.0; 5];
        sock.group_positions_current_get("G1", &mut pos).unwrap();
        assert_eq!(pos, [1.5, 9.0, 3.5, 9.0, 9.0]);
        assert_eq!(
            line.take_log(),
            ["GroupPositionCurrentGet (G1,double *,double *,double *,double *,double *)"]
        );

        *line.fail.lock().unwrap() = "GroupPositionCurrentGet";
        let mut pos = [9.0; 2];
        let err = sock
            .group_positions_current_get("G1", &mut pos)
            .unwrap_err();
        assert_eq!((err.code(), pos), (-3, [9.0; 2]));
    }

    /// XPS_C8_drivers.cpp:7439-7473: the file is reply field 1 and the
    /// element field 2; a missing or non-`%d` field keeps its value.
    #[test]
    fn pvt_parameters_get_marshals_as_c() {
        let line = FakeLine::new("xps_pvt_parameters", "");
        let sock = line.socket(SocketMode::Query);
        let (mut file, mut element) = (String::new(), 7);
        sock.multiple_axes_pvt_parameters_get("G1", &mut file, &mut element)
            .unwrap();
        assert_eq!((file.as_str(), element), ("traj.trj", 42));
        assert_eq!(
            line.take_log(),
            ["MultipleAxesPVTParametersGet (G1,char *,int *)"]
        );

        let (mut file, mut element) = ("kept".to_string(), 7);
        sock.multiple_axes_pvt_parameters_get("NOELEMENT", &mut file, &mut element)
            .unwrap();
        assert_eq!((file.as_str(), element), ("traj.trj", 7));
    }

    /// XPS_C8_drivers.cpp:7574-7710: the SingleAxisSlave* wire strings, and
    /// a failed or short Get leaves its out-parameters alone.
    #[test]
    fn single_axis_slave_calls_marshal_as_c() {
        let line = FakeLine::new("xps_slave_calls", "");
        let sock = line.socket(SocketMode::Query);
        sock.single_axis_slave_parameters_set("G2", "G1.P1", 0.1)
            .unwrap();
        sock.single_axis_slave_mode_enable("G2").unwrap();
        sock.single_axis_slave_mode_disable("G2").unwrap();
        let (mut master, mut ratio) = (String::new(), 0.0);
        sock.single_axis_slave_parameters_get("G2", &mut master, &mut ratio)
            .unwrap();
        assert_eq!((master.as_str(), ratio), ("G1.P1", 2.5));
        assert_eq!(
            line.take_log(),
            [
                "SingleAxisSlaveParametersSet (G2,G1.P1,0.1)",
                "SingleAxisSlaveModeEnable (G2)",
                "SingleAxisSlaveModeDisable (G2)",
                "SingleAxisSlaveParametersGet (G2,char *,double *)",
            ]
        );

        let (mut master, mut ratio) = ("kept".to_string(), 7.0);
        sock.single_axis_slave_parameters_get("NORATIO", &mut master, &mut ratio)
            .unwrap();
        assert_eq!((master.as_str(), ratio), ("G1.P1", 7.0));

        *line.fail.lock().unwrap() = "SingleAxisSlaveParametersGet";
        let (mut master, mut ratio) = ("kept".to_string(), 7.0);
        let err = sock
            .single_axis_slave_parameters_get("G2", &mut master, &mut ratio)
            .unwrap_err();
        assert_eq!((err.code(), master.as_str(), ratio), (-3, "kept", 7.0));
    }
}
