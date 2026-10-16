//! `SMCcorvusCreateController` iocsh command for the Micos SMC corvus.
//!
//! Mirrors C `SMCcorvusCreateController(portName, corvusPort, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port and create
//! controller port `portName` whose `numAxes` axes records address as
//! `@asyn(portName,axis)`; hydra and Taurus likewise. The corvus connects over either a
//! `drvAsynIPPort` or a `drvAsynSerialPort`, so `connect_ip` is used (the lookup
//! is transport-agnostic).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::{connect_controller, connect_ip};
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};
use motor_common::{asyn_user, install, register_port};

use crate::corvus::{CorvusAxis, CorvusController};
use crate::hydra::{HydraAxis, HydraController};
use crate::taurus::{TaurusAxis, TaurusController};

/// Command timeout.
const CORVUS_TIMEOUT: Duration = Duration::from_secs(2);

/// Hydra command timeout.
const HYDRA_TIMEOUT: Duration = Duration::from_secs(2);

/// Taurus command timeout.
const TAURUS_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `SMCcorvusCreateController(portName, SMCcorvusPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn corvus_create_controller_command() -> CommandDef {
    CommandDef::new(
        "SMCcorvusCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("SMC corvus port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "SMCcorvusCreateController(portName, SMCcorvusPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let corvus_port = req_string(args, 1, "SMC corvus port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if !(1..=3).contains(&num_axes) {
                return Err("SMCcorvusCreateController: numAxes must be 1..=3".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port(ctx, &port_name, num_axes as i32)?;

            let (handle, _) = connect_controller(
                &corvus_port,
                CORVUS_TIMEOUT,
                &port_name,
                format_args!(
                    "SMCcorvusController::SMCcorvusController: cannot connect to SMC corvus controller"
                ),
            );
            let controller = Arc::new(Mutex::new(CorvusController::new(handle, num_axes as usize)));

            for index in 0..num_axes as usize {
                let ax = CorvusAxis::new(controller.clone(), index)
                    .map_err(|e| format!("SMCcorvusCreateController: axis {index}: {e}"))?;
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

/// Build the `SMChydraCreateController(portName, SMChydraPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn hydra_create_controller_command() -> CommandDef {
    CommandDef::new(
        "SMChydraCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("SMC hydra port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "SMChydraCreateController(portName, SMChydraPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let hydra_port = req_string(args, 1, "SMC hydra port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if !(1..=2).contains(&num_axes) {
                return Err("SMChydraCreateController: numAxes must be 1..=2".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&hydra_port, HYDRA_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(HydraController::new(handle)));

            register_port(ctx, &port_name, num_axes as i32)?;

            for index in 0..num_axes as usize {
                let ax = HydraAxis::new(controller.clone(), index)
                    .map_err(|e| format!("SMChydraCreateController: axis {index}: {e}"))?;
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

/// Build the `SMCTaurusCreateController(portName, SMCTaurusPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn taurus_create_controller_command() -> CommandDef {
    CommandDef::new(
        "SMCTaurusCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("SMC Taurus port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "SMCTaurusCreateController(portName, SMCTaurusPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let taurus_port = req_string(args, 1, "SMC Taurus port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("SMCTaurusCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&taurus_port, TAURUS_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(TaurusController::new(handle)));

            register_port(ctx, &port_name, num_axes as i32)?;

            for index in 0..num_axes as usize {
                let ax = TaurusAxis::new(controller.clone(), index)
                    .map_err(|e| format!("SMCTaurusCreateController: axis {index}: {e}"))?;
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
