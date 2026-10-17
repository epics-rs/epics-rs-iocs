//! `LC400CreateController` / `LC400ConfigAxis`, LC400MotorDriver.cpp's
//! iocsh commands: controller port `portName` has axes 0..numAxes, which
//! records reach as `@asyn(portName,axis)`.
//!
//! As in C, `LC400ConfigAxis` names the controller by its octet port
//! (`LC400PortName`), not by `portName`.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_str_req, opt_double, poll_intervals, req_string,
};
use motor_common::model1::c_int;
use motor_common::{Controller, asyn_user, register_controller_port};

use crate::lc400::{LC400_TIMEOUT, Lc400Axis, Lc400Controller, Lc400ControllerParams, config_axis};

/// C's `controllers` map: each controller by its octet port name.
type Registry = Arc<Mutex<HashMap<String, Arc<Mutex<Lc400Controller>>>>>;

/// Both LC400 commands, sharing their controller map.
pub fn lc400_commands() -> Vec<CommandDef> {
    let registry: Registry = Arc::default();
    vec![
        lc400_create_controller_command(&registry),
        lc400_config_axis_command(&registry),
    ]
}

/// C `LC400CreateController(portName, LC400PortName, numAxes,
/// movingPollPeriod, idlePollPeriod)`, periods in ms. A port that is not
/// there is reported and the controller made anyway, as C does.
fn lc400_create_controller_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "LC400CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("LC400 port name"),
            arg_int_opt("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "LC400CreateController(portName, LC400PortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let lc400_port = req_string(args, 1, "LC400 port name")?;
            let num_axes = c_int(args, 2).max(0);
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&lc400_port, LC400_TIMEOUT)
                .inspect_err(|_| {
                    eprintln!(
                        "LC400MotorDriver:LC400Controller: cannot connect to LC400 controller"
                    )
                })
                .ok();
            let controller = Arc::new(Mutex::new(Lc400Controller::new(handle, num_axes as usize)));
            let params = Lc400ControllerParams::new(controller.clone());
            let owner = ctx.db().scan_control().interrupt_gate();
            let port = register_controller_port(owner, &port_name, num_axes, params)?;
            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .attach_port(port);
            let poller = Controller::new();
            for axis in 0..num_axes {
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(Lc400Axis::new(
                    controller.clone(),
                    axis as usize,
                )));
                poller.install(asyn_user(&port_name, axis), motor)?;
            }
            // C startPoller, LC400MotorDriver.cpp:156
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 2);
            registry
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(lc400_port, controller);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `LC400ConfigAxis(LC400PortName, axis, hiHardLimit, lowHardLimit,
/// tolHardLimit)`, in position units.
fn lc400_config_axis_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "LC400ConfigAxis",
        vec![
            arg_str_req("Post name"),
            arg_int_opt("Axis #"),
            arg_double_opt("High limit"),
            arg_double_opt("Low limit"),
            arg_double_opt("Hard limits tolerance"),
        ],
        "LC400ConfigAxis(LC400PortName, axis, hiHardLimit, lowHardLimit, tolHardLimit)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port = req_string(args, 0, "Post name")?;
            let axis = c_int(args, 1);
            let high = opt_double(args, 2, 0.0, "High limit")?;
            let low = opt_double(args, 3, 0.0, "Low limit")?;
            let tol = opt_double(args, 4, 0.0, "Hard limits tolerance")?;
            let controller = registry
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&port)
                .cloned();
            match controller {
                Some(controller) => config_axis(&controller, axis, high, low, tol),
                None => println!("invalid controller port: {port}"),
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
