//! iocsh command for the IMS MDrivePlus driver.
//!
//! Mirrors the C `ImsMDrivePlusCreateController(motorPort, ioPort, deviceName,
//! movingPoll, idlePoll)`: connects the asyn octet port, probes the single axis
//! (firmware version, output EOS, switch config, encoder flag) and attaches it
//! to a `DTYP`-keyed motor device support. The `DTYP` key is the `motorPort`
//! string (one controller = one axis). `deviceName` is empty for a
//! non-party-mode drive.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use epics_rs::asyn::asyn_record::get_port;
use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_str_opt, arg_str_req, opt_double, opt_string, poll_intervals,
    req_string,
};
use motor_common::{MotorHolder, asyn_user};

use crate::mdriveplus::{INPUT_EOS, ImsMDrivePlusAxis, OUTPUT_EOS};

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
            arg_double_opt("timeoutMs"),
        ],
        "ImsMDrivePlusCreateController(motorPort, ioPort, [deviceName], [movingPollMs], \
         [idlePollMs], [timeoutMs]) - Create an IMS MDrivePlus controller (DTYP motorPort) \
         with one axis; deviceName is prepended in party mode (empty = none)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let io_port = req_string(args, 1, "ioPort")?;
            let device_name = opt_string(args, 2)?.unwrap_or_default();
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let timeout_ms = opt_double(args, 5, DEFAULT_TIMEOUT_MS, "timeoutMs")?;

            let handle = connect_with_eos(
                &io_port,
                Duration::from_millis(timeout_ms.max(0.0) as u64),
                INPUT_EOS,
                OUTPUT_EOS,
                EosRefusal::Ignored,
            )?;
            println!(
                "ImsMDrivePlusMotorDriver:ImsMDrivePlusMotorController(): motorPortName={motor_port}, \
                 IOPortName={io_port}, devName={device_name} "
            );
            let port =
                get_port(&io_port).ok_or_else(|| format!("IP port '{io_port}' not found"))?;
            let axis = ImsMDrivePlusAxis::new(handle, &port.handle, &motor_port, device_name);

            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            holder.install(
                ctx,
                asyn_user(&motor_port, 0),
                motor_port.clone(),
                motor,
                moving_poll_ms,
                idle_poll_ms,
            );

            Ok(CommandOutcome::Continue)
        },
    )
}
