//! `SMCcorvusCreateController` iocsh command for the Micos SMC corvus.
//!
//! Mirrors C `SMCcorvusCreateController(portName, corvusPort, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port and create
//! controller port `portName` whose `numAxes` axes records address as
//! `@asyn(portName,axis)`; hydra and Taurus likewise. The corvus connects over either a
//! `drvAsynIPPort` or a `drvAsynSerialPort`, so `connect_ip` is used (the lookup
//! is transport-agnostic).

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::connect_controller;
use motor_common::iocsh::{
    arg_double_req, arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_double, req_int,
    req_string,
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

/// An axis whose C `axisRes_` a `*ChangeResolution` command replaces.
trait ChangeResolution: Send {
    fn change_resolution(&mut self, new_resolution: f64);
}

impl ChangeResolution for CorvusAxis {
    fn change_resolution(&mut self, new_resolution: f64) {
        CorvusAxis::change_resolution(self, new_resolution);
    }
}

type ResolutionAxes = HashMap<String, Vec<Arc<Mutex<dyn ChangeResolution>>>>;

/// The axes of every controller port, as C's `findAsynPortDriver` +
/// `getAxis` reach them.
fn resolution_axes() -> &'static Mutex<ResolutionAxes> {
    static AXES: OnceLock<Mutex<ResolutionAxes>> = OnceLock::new();
    AXES.get_or_init(Default::default)
}

fn register_resolution_axis(port: &str, axis: Arc<Mutex<dyn ChangeResolution>>) {
    resolution_axes()
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .entry(port.to_owned())
        .or_default()
        .push(axis);
}

/// C `<name>(portName, axisNo, newResolution)`; `source` is the C file its
/// not-found line names.
fn change_resolution_command(
    name: &'static str,
    port_arg: &'static str,
    source: &'static str,
) -> CommandDef {
    CommandDef::new(
        name,
        vec![
            arg_str_req(port_arg),
            arg_int_req("Axis number"),
            arg_double_req("Axis resolution"),
        ],
        format!("{name}(portName, axisNo, newResolution)"),
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port = req_string(args, 0, port_arg)?;
            let axis_no = req_int(args, 1, "Axis number")?;
            let new_resolution = req_double(args, 2, "Axis resolution")?;
            let axes = resolution_axes().lock().unwrap_or_else(|e| e.into_inner());
            let Some(axes) = axes.get(&port) else {
                println!("{source}:{name}: Error port {port} not found");
                return Ok(CommandOutcome::Continue);
            };
            let axis = usize::try_from(axis_no)
                .ok()
                .and_then(|i| axes.get(i))
                .ok_or_else(|| format!("{name}: no axis {axis_no} on port {port}"))?;
            axis.lock()
                .unwrap_or_else(|e| e.into_inner())
                .change_resolution(new_resolution);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `SMCcorvusChangeResolution(portName, axisNo, newResolution)`
/// command.
pub fn corvus_change_resolution_command() -> CommandDef {
    change_resolution_command(
        "SMCcorvusChangeResolution",
        "SMC corvus port name",
        "SMCcorvusDriver.cpp",
    )
}

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
                let ax = Arc::new(Mutex::new(ax));
                register_resolution_axis(&port_name, ax.clone());
                let motor: Arc<Mutex<dyn AsynMotor>> = ax;
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

            register_port(ctx, &port_name, num_axes as i32)?;

            let (handle, _) = connect_controller(
                &hydra_port,
                HYDRA_TIMEOUT,
                &port_name,
                format_args!(
                    "SMChydraController::SMChydraController: cannot connect to SMC hydra controller"
                ),
            );
            let controller = Arc::new(Mutex::new(HydraController::new(handle)));

            for index in 0..num_axes as usize {
                let ax = HydraAxis::new(controller.clone(), index);
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

            register_port(ctx, &port_name, num_axes as i32)?;

            let (handle, _) = connect_controller(
                &taurus_port,
                TAURUS_TIMEOUT,
                &port_name,
                format_args!(
                    "SMCTaurusController::SMCTaurusController: cannot connect to SMC Taurus controller"
                ),
            );
            let controller = Arc::new(Mutex::new(TaurusController::new(handle)));

            for index in 0..num_axes as usize {
                let ax = TaurusAxis::new(controller.clone(), index);
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
