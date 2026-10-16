//! `PI_GCS2_CreateController` iocsh command for the PI GCS2 driver.
//!
//! Mirrors the C `PIasynController` constructor / `PI_GCS2_CreateController`
//! iocsh function: connect the serial/TCP octet port, identify and probe the
//! controller ([`PIGCS2Controller::new`], which auto-discovers connected axis
//! names via `SAI?`), then create controller port `portName` with the first
//! `numAxes` of those axes in discovery order, addressed by records as
//! `@asyn(portName,axis)`. Single-step, matching upstream — see
//! `gcs2.rs`'s "Config" doc section for why this is not a two-step
//! registry/explicit-`CreateAxis` API like `motor-phytron`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};
use motor_common::{asyn_user, install, register_port};

use crate::gcs2::{PIGCS2Axis, PIGCS2Controller};

/// Command timeout (C `PIInterface::TIMEOUT` is 5.0 s).
const PIGCS2_TIMEOUT: Duration = Duration::from_secs(5);

/// Build the `PI_GCS2_CreateController(portName, asynPort, numAxes, priority,
/// stackSize, movingPollingRate, idlePollingRate)` command, rates in ms.
/// `priority` and `stackSize` size C's port thread, which has no counterpart
/// here.
pub fn pigcs2_config_command() -> CommandDef {
    CommandDef::new(
        "PI_GCS2_CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("asyn Port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("priority"),
            arg_int_opt("stackSize"),
            arg_int_opt("moving polling time [msec]"),
            arg_int_opt("idle polling time [msec]"),
        ],
        "PI_GCS2_CreateController(portName, asynPort, numAxes, priority, stackSize, \
         movingPollingRate, idlePollingRate)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let asyn_port = req_string(args, 1, "asyn Port name")?;
            // C: `if (numAxes < 1 ) numAxes = 1;`
            let num_axes = req_int(args, 2, "Number of axes")?.max(1) as usize;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 5, 6)?;

            let handle = connect_with_eos(
                &asyn_port,
                PIGCS2_TIMEOUT,
                b"\n",
                b"",
                EosRefusal::BothFatal,
            )?;
            let controller = PIGCS2Controller::new(handle, &asyn_port)
                .map_err(|e| format!("PIasynController: {e}"))?;
            let found: Vec<String> = controller.axis_names().to_vec();
            if found.len() < num_axes {
                return Err(format!(
                    "PIasynController: requested number of axes ({num_axes}) out of \
                     range, only {} axis/axes supported",
                    found.len()
                ));
            }
            let controller = Arc::new(Mutex::new(controller));

            register_port(ctx, &port_name, num_axes as i32)?;
            for (index, axis_name) in found.into_iter().take(num_axes).enumerate() {
                let axis = PIGCS2Axis::new(controller.clone(), axis_name.clone(), index)
                    .map_err(|e| format!("PIasynController: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                install(
                    ctx,
                    asyn_user(&port_name, index as i32),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                )?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
