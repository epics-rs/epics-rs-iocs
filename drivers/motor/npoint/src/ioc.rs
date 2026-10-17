//! `C300CreateController` iocsh command for the nPoint C300.
//!
//! Mirrors C `C300CreateController(portName, C300PortName, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port, unlock the
//! controller ([`C300Controller::new`]), and create controller port `portName`
//! whose `numAxes` axes records address as `@asyn(portName,axis)`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::connect_controller;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};
use motor_common::{Controller, asyn_user, register_port};

use crate::c300::{C300Axis, C300Controller};

/// Command timeout (C `C300_TIMEOUT` 1 s).
const C300_TIMEOUT: Duration = Duration::from_secs(1);

/// Build the `C300CreateController(portName, C300PortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn c300_create_controller_command() -> CommandDef {
    CommandDef::new(
        "C300CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("C300 port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "C300CreateController(portName, C300PortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let asyn_port = req_string(args, 1, "C300 port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("C300CreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port(ctx, &port_name, num_axes as i32)?;

            let (handle, _) = connect_controller(
                &asyn_port,
                C300_TIMEOUT,
                &port_name,
                format_args!("C300MotorDriver:C300Controller: cannot connect to C300 controller"),
            );
            let controller = C300Controller::new(handle, num_axes as usize);
            let controller = Arc::new(Mutex::new(controller));

            let poller = Controller::new();
            for index in 0..num_axes as usize {
                let ax = C300Axis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                poller.install(asyn_user(&port_name, index as i32), motor)?;
            }
            // C startPoller, C300MotorDriver.cpp:82
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 2);
            Ok(CommandOutcome::Continue)
        },
    )
}
