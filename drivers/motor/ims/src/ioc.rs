//! iocsh command for the IMS MDrivePlus driver.
//!
//! Mirrors the C `ImsMDrivePlusCreateController(motorPort, ioPort, deviceName,
//! movingPoll, idlePoll)`: connects the asyn octet port, probes the single axis
//! (firmware version, terminator, switch config, encoder flag) and attaches it
//! to a `DTYP`-keyed motor device support. The `DTYP` key is the `motorPort`
//! string (one controller = one axis). `deviceName` is empty for a
//! non-party-mode drive.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_ip;
use motor_common::iocsh::{
    arg_int_opt, arg_str_opt, arg_str_req, opt_double, opt_string, poll_intervals, req_string,
};

use crate::mdriveplus::ImsMDrivePlusAxis;

/// Default communication timeout when the `timeoutMs` arg is omitted.
const DEFAULT_TIMEOUT_MS: f64 = 2000.0;

/// Build the `ImsMDrivePlusCreateController(motorPort, ioPort, [deviceName],
/// [movingPollMs], [idlePollMs], [timeoutMs])` command bound to `holder`.
pub fn ims_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "ImsMDrivePlusCreateController",
        vec![
            arg_str_req("motorPort"),
            arg_str_req("ioPort"),
            arg_str_opt("deviceName"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
            arg_int_opt("timeoutMs"),
        ],
        "ImsMDrivePlusCreateController(motorPort, ioPort, [deviceName], [movingPollMs], \
         [idlePollMs], [timeoutMs]) - Create an IMS MDrivePlus controller (DTYP motorPort) \
         with one axis; deviceName is prepended in party mode (empty = none)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let io_port = req_string(args, 1, "ioPort")?;
            let device_name = opt_string(args, 2).unwrap_or_default();
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let timeout_ms = opt_double(args, 5, DEFAULT_TIMEOUT_MS, "timeoutMs")?;

            let handle = connect_ip(&io_port, Duration::from_millis(timeout_ms.max(0.0) as u64))?;
            let axis = ImsMDrivePlusAxis::new(handle, device_name.clone())
                .map_err(|e| format!("ImsMDrivePlusCreateController: axis setup failed: {e}"))?;

            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            holder.install(ctx, motor_port.clone(), motor, moving_poll_ms, idle_poll_ms);

            let dev = if device_name.is_empty() {
                "(none)".to_string()
            } else {
                device_name
            };
            println!(
                "ImsMDrivePlusCreateController: motorPort={motor_port} ioPort={io_port} \
                 deviceName={dev} poll=[{moving_poll_ms}/{idle_poll_ms}]ms"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
