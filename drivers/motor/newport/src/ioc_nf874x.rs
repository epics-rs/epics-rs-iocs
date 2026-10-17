//! `nf874xCreateController`, 874xMotorDriver.cpp's iocsh command: controller
//! port `portName` has addresses 0..=numAxes, axis n being `@asyn(portName,n)`
//! and address 0 unused.

use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::connect_serial;
use motor_common::iocsh::{arg_int_opt, arg_str_req, poll_intervals, req_string};
use motor_common::model1::c_int;
use motor_common::{asyn_user, install, register_controller_port};

use crate::nf874x::{NF874X_TIMEOUT, Nf874xAxis, Nf874xController, Nf874xControllerParams};

/// C `nf874xCreateController(portName, nf874xPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)`, periods in ms. A port that is not
/// there is reported and the controller made anyway, as C does.
pub fn nf874x_create_controller_command() -> CommandDef {
    CommandDef::new(
        "nf874xCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("nf874x port name"),
            arg_int_opt("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "nf874xCreateController(portName, nf874xPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let nf_port = req_string(args, 1, "nf874x port name")?;
            let num_axes = c_int(args, 2).max(0);
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&nf_port, NF874X_TIMEOUT)
                .inspect_err(|_| {
                    eprintln!(
                        "nf874xMotorDriver:nf874xController: cannot connect to nf874x controller"
                    )
                })
                .ok();
            sleep(Duration::from_millis(500));
            let controller = Arc::new(Mutex::new(Nf874xController::new(handle)));
            let params = Nf874xControllerParams::new(controller.clone());
            let owner = ctx.db().scan_control().interrupt_gate();
            let port = register_controller_port(owner, &port_name, num_axes + 1, params)?;
            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .attach_port(port);
            for axis in 1..=num_axes {
                let motor: Arc<Mutex<dyn AsynMotor>> =
                    Arc::new(Mutex::new(Nf874xAxis::new(controller.clone(), axis)));
                install(
                    ctx,
                    asyn_user(&port_name, axis),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                    2, // C startPoller, 874xMotorDriver.cpp:89
                )?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
