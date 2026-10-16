//! iocsh commands for the SmarAct controllers, under C's names and
//! arguments: each controller is asyn port `portName` and records reach its
//! axes as `@asyn(portName,axis)`.
//!
//! - **MCS2** (`MCS2CreateController`): creates axes 0..numAxes, skipping
//!   those set in `unusedMask`; channel = axis.
//! - **SCU** / **MCS** (`smarAct{SCU,MCS}CreateController` +
//!   `smarAct{SCU,MCS}CreateAxis`): the controller is found by port name and
//!   each axis names the channel it drives. Their poll periods are seconds.

use std::collections::BTreeSet;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::{EosRefusal, connect_serial, connect_with_eos};
use motor_common::iocsh::{arg_double_req, arg_int_req, arg_str_req, poll_intervals, req_string};
use motor_common::model1::c_int;
use motor_common::{PortDriver, asyn_user, install, register_port};

use crate::mcs::{self, McsAxis, McsController};
use crate::mcs2::{self, Mcs2Axis, Mcs2Controller};
use crate::scu::{ScuAxis, ScuController};

/// Command timeout.
const MCS2_TIMEOUT: Duration = Duration::from_secs(1);

/// SCU command timeout (C `DEFAULT_TIMEOUT` = 2.0 s).
const SCU_TIMEOUT: Duration = Duration::from_secs(2);

/// MCS command timeout (C `DEFLT_TIMEOUT` = 2.0 s).
const MCS_TIMEOUT: Duration = Duration::from_secs(2);

/// An `iocshArgDouble` period in seconds as milliseconds; 0 when absent.
fn period_ms(args: &[ArgValue], i: usize) -> u64 {
    match args.get(i) {
        Some(ArgValue::Double(s)) if *s > 0.0 => (s * 1000.0) as u64,
        Some(ArgValue::Int(s)) if *s > 0 => *s as u64 * 1000,
        _ => 0,
    }
}

/// `MCS2CreateController(portName, MCS2PortName, numAxes, movingPollPeriod,
/// idlePollPeriod, unusedMask)`, periods in ms. A controller that cannot be
/// reached is reported as C's is and keeps its port, with no axes.
pub fn mcs2_create_controller_command() -> CommandDef {
    CommandDef::new(
        "MCS2CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("MCS2 port name"),
            arg_int_req("Number of axes"),
            arg_int_req("Moving poll period (ms)"),
            arg_int_req("Idle poll period (ms)"),
            arg_int_req("Unused bit mask"),
        ],
        "MCS2CreateController(portName, MCS2PortName, numAxes, movingPollPeriod, \
         idlePollPeriod, unusedMask)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let mcs2_port = req_string(args, 1, "MCS2 port name")?;
            let num_axes = c_int(args, 2);
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let unused_mask = c_int(args, 5);

            register_port(&port_name, PortDriver::Controller, num_axes)?;
            let Ok(handle) = connect_with_eos(
                &mcs2_port,
                MCS2_TIMEOUT,
                mcs2::INPUT_EOS,
                mcs2::OUTPUT_EOS,
                EosRefusal::Ignored,
            ) else {
                eprintln!(
                    "SmarActMCS2MotorDriver:MCS2Controller: cannot connect to MCS2 controller"
                );
                return Ok(CommandOutcome::Continue);
            };
            let controller = Arc::new(Mutex::new(
                Mcs2Controller::new(handle).map_err(|e| format!("MCS2CreateController: {e}"))?,
            ));
            for axis in 0..num_axes {
                if unused_mask & (1 << axis) != 0 {
                    continue;
                }
                let motor: Arc<Mutex<dyn AsynMotor>> =
                    Arc::new(Mutex::new(Mcs2Axis::new(controller.clone(), axis as usize)));
                install(
                    ctx,
                    asyn_user(&port_name, axis),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                )?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// A controller `smarAct{SCU,MCS}CreateController` made: `None` when its I/O
/// port could not be connected, as C's constructor then carries on without it.
struct Registered<C> {
    port_name: String,
    controller: Option<Arc<Mutex<C>>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
    axes: BTreeSet<i32>,
}

type Registry<C> = Mutex<Vec<Registered<C>>>;

static SCU_CONTROLLERS: Registry<ScuController> = Mutex::new(Vec::new());
static MCS_CONTROLLERS: Registry<McsController> = Mutex::new(Vec::new());

/// The `CreateController` arguments SCU and MCS share.
fn create_controller_args() -> Vec<ArgDesc> {
    vec![
        arg_str_req("Port name [string]"),
        arg_str_req("I/O port name [string]"),
        arg_int_req("Number of axes [int]"),
        arg_double_req("Moving poll period (s) [double]"),
        arg_double_req("Idle poll period (s) [double]"),
    ]
}

/// The `CreateAxis` arguments SCU and MCS share.
fn create_axis_args() -> Vec<ArgDesc> {
    vec![
        arg_str_req("Controller Port name [string]"),
        arg_int_req("Axis number [int]"),
        arg_int_req("Channel [int]"),
    ]
}

/// Register `port_name` and its controller, or print C's connect refusal:
/// the asynPrint and the `THROW_` that, with exceptions off, only prints.
fn register<C>(
    registry: &Registry<C>,
    args: &[ArgValue],
    controller: Option<C>,
    refusal: (&str, &str),
) -> Result<(), String> {
    let port_name = req_string(args, 0, "Port name [string]")?;
    register_port(&port_name, PortDriver::Controller, c_int(args, 2))?;
    if controller.is_none() {
        eprintln!("{}", refusal.0);
        println!("{}", refusal.1);
    }
    registry
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .push(Registered {
            port_name,
            controller: controller.map(|c| Arc::new(Mutex::new(c))),
            moving_poll_ms: period_ms(args, 3),
            idle_poll_ms: period_ms(args, 4),
            axes: BTreeSet::new(),
        });
    Ok(())
}

/// `smarAct{SCU,MCS}CreateAxis(controllerPortName, axisNumber, channel)`,
/// printing what C's does; `build` makes the axis on a connected controller.
fn create_axis<C>(
    registry: &Registry<C>,
    command: &str,
    duplicate: &str,
    args: &[ArgValue],
    ctx: &CommandContext,
    build: impl FnOnce(Arc<Mutex<C>>, i32, i32) -> Result<Arc<Mutex<dyn AsynMotor>>, String>,
) -> Result<(), String> {
    let controller_port = req_string(args, 0, "Controller Port name [string]")?;
    let axis = c_int(args, 1);
    let channel = c_int(args, 2);
    let mut registry = registry.lock().unwrap_or_else(|e| e.into_inner());
    let Some(reg) = registry.iter_mut().find(|r| r.port_name == controller_port) else {
        println!("{command}: Error port {controller_port} not found");
        return Ok(());
    };
    if !reg.axes.insert(axis) {
        println!("{duplicate} failed:axis {axis} already exists");
        return Ok(());
    }
    let Some(controller) = reg.controller.clone() else {
        return Ok(());
    };
    let motor = build(controller, axis, channel).map_err(|e| format!("{command}: {e}"))?;
    install(
        ctx,
        asyn_user(&controller_port, axis),
        motor,
        reg.moving_poll_ms,
        reg.idle_poll_ms,
    )
}

/// `smarActSCUCreateController(motorPortName, ioPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)`.
pub fn scu_create_controller_command() -> CommandDef {
    CommandDef::new(
        "smarActSCUCreateController",
        create_controller_args(),
        "smarActSCUCreateController(motorPortName, ioPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let io_port = req_string(args, 1, "I/O port name [string]")?;
            let controller = connect_serial(&io_port, SCU_TIMEOUT)
                .ok()
                .map(ScuController::new);
            register(
                &SCU_CONTROLLERS,
                args,
                controller,
                (
                    "SmarActSCUController:SmarActSCUController: cannot connect to SCU controller",
                    "SmarActSCUController: unable to connect serial channel",
                ),
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `smarActSCUCreateAxis(controllerPortName, axisNumber, channel)`.
pub fn scu_create_axis_command() -> CommandDef {
    CommandDef::new(
        "smarActSCUCreateAxis",
        create_axis_args(),
        "smarActSCUCreateAxis(controllerPortName, axisNumber, channel)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port = req_string(args, 0, "Controller Port name [string]")?;
            create_axis(
                &SCU_CONTROLLERS,
                "smarActSCUCreateAxis",
                "SmarActSCUCreateAxis",
                args,
                ctx,
                |controller, axis, channel| {
                    let ax = ScuAxis::new(controller, channel, &port, axis)
                        .map_err(|e| e.to_string())?;
                    Ok(Arc::new(Mutex::new(ax)))
                },
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `smarActMCSCreateController(motorPortName, ioPortName, numAxes,
/// movingPollPeriod, idlePollPeriod, disableSpeed)`.
pub fn mcs_create_controller_command() -> CommandDef {
    let mut args = create_controller_args();
    args.push(arg_int_req("Disable speed cmds [int]"));
    CommandDef::new(
        "smarActMCSCreateController",
        args,
        "smarActMCSCreateController(motorPortName, ioPortName, numAxes, movingPollPeriod, \
         idlePollPeriod, disableSpeed)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name [string]")?;
            let io_port = req_string(args, 1, "I/O port name [string]")?;
            let disable_speed = c_int(args, 5) != 0;
            if disable_speed {
                println!(
                    "SmarActMCSController({port_name}): WARNING - The speed set commands have \
                     been disabled for this controller"
                );
            }
            let controller = connect_with_eos(
                &io_port,
                MCS_TIMEOUT,
                mcs::INPUT_EOS,
                mcs::OUTPUT_EOS,
                EosRefusal::Ignored,
            )
            .ok()
            .map(|handle| McsController::new(handle, disable_speed));
            register(
                &MCS_CONTROLLERS,
                args,
                controller,
                (
                    "SmarActMCSController:SmarActMCSController: cannot connect to MCS controller",
                    "SmarActMCSController: unable to connect serial channel",
                ),
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `smarActMCSCreateAxis(controllerPortName, axisNumber, channel)`.
pub fn mcs_create_axis_command() -> CommandDef {
    CommandDef::new(
        "smarActMCSCreateAxis",
        create_axis_args(),
        "smarActMCSCreateAxis(controllerPortName, axisNumber, channel)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            create_axis(
                &MCS_CONTROLLERS,
                "smarActMCSCreateAxis",
                "SmarActMCSCreateAxis",
                args,
                ctx,
                |controller, _axis, channel| {
                    let ax = McsAxis::new(controller, channel).map_err(|e| e.to_string())?;
                    Ok(Arc::new(Mutex::new(ax)))
                },
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}
