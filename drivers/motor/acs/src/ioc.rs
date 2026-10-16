//! `MCB4BCreateController` iocsh command for the ACS MCB-4B.
//!
//! C `MCB4BCreateController(portName, MCB4BPortName, numAxes,
//! movingPollPeriod, idlePollPeriod)` from `MCB4BDriver.cpp`: connect the serial
//! octet port and create controller port `portName` with `numAxes` axes,
//! addressed by records as `@asyn(portName,axis)`.
//!
//! The C constructor performs no identification I/O (it only connects and starts
//! the poller), so this command connects and installs axes without probing.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};
use motor_common::{PortDriver, asyn_user, install, register_port};

use crate::mcb4b::{Mcb4bAxis, Mcb4bController};

/// Serial command timeout (C `MCB4BDriver` uses the asyn port default; the other
/// ACS/motor ports use 2 s, matched here).
const MCB4B_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `MCB4BCreateController(portName, MCB4BPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command.
pub fn mcb4b_create_controller_command() -> CommandDef {
    CommandDef::new(
        "MCB4BCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("MCB-4B port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "MCB4BCreateController(portName, MCB4BPortName, numAxes, movingPollPeriod, idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let asyn_port = req_string(args, 1, "MCB-4B port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("MCB4BCreateController: numAxes must be > 0".into());
            }
            let num_axes = num_axes as usize;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port(&port_name, PortDriver::Controller, num_axes as i32)?;
            if get_port(&asyn_port).is_none() {
                eprintln!("MCB4BController::MCB4BController: cannot connect to MCB-4B controller");
                return Ok(CommandOutcome::Continue);
            }
            let handle = connect_serial(&asyn_port, MCB4B_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(Mcb4bController::new(handle, num_axes)));

            for index in 0..num_axes {
                let ax = Mcb4bAxis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
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
