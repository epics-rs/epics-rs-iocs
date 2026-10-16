//! `OEMCreateController` iocsh command for the Parker OEM750 series.
//!
//! Mirrors C `OEMCreateController(portName, OEMPortName, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port and create
//! controller port `portName` whose `numAxes` axes (unit address = axis + 1)
//! records address as `@asyn(portName,axis)`; `ACRCreateController` likewise.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::{EosRefusal, connect_ip, connect_with_eos};
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};
use motor_common::{asyn_user, install, register_port};

use crate::acr::{AcrAxis, AcrController};
use crate::oem::{OemAxis, OemController};

/// Command timeout.
const OEM_TIMEOUT: Duration = Duration::from_secs(1);

/// ACR command timeout.
const ACR_TIMEOUT: Duration = Duration::from_secs(1);

/// Build the `OEMCreateController(portName, OEMPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn oem_create_controller_command() -> CommandDef {
    CommandDef::new(
        "OEMCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("OEM port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "OEMCreateController(portName, OEMPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let oem_port = req_string(args, 1, "OEM port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("OEMCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle =
                connect_with_eos(&oem_port, OEM_TIMEOUT, b"\r", b"\r", EosRefusal::Ignored)?;
            let controller = Arc::new(Mutex::new(OemController::new(handle)));

            register_port(ctx, &port_name, num_axes as i32)?;

            for index in 0..num_axes as usize {
                let ax = OemAxis::new(controller.clone(), index)
                    .map_err(|e| format!("OEMCreateController: axis {index}: {e}"))?;
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

/// Build the `ACRCreateController(portName, ACRPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn acr_create_controller_command() -> CommandDef {
    CommandDef::new(
        "ACRCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("ACR port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "ACRCreateController(portName, ACRPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let acr_port = req_string(args, 1, "ACR port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("ACRCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&acr_port, ACR_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(AcrController::new(handle)));

            register_port(ctx, &port_name, num_axes as i32)?;

            for index in 0..num_axes as usize {
                let ax = AcrAxis::new(controller.clone(), index)
                    .map_err(|e| format!("ACRCreateController: axis {index}: {e}"))?;
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
