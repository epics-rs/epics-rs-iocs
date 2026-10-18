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
use epics_rs::asyn::manager::PortManager;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::motor::trace::MotorTrace;

use motor_common::connect::{connect_controller, set_eos_ignored};
use motor_common::iocsh::{arg_double_req, arg_int_req, arg_str_req, poll_intervals, req_string};
use motor_common::model1::c_int;
use motor_common::{Controller, ControllerParams, asyn_user, register_port_with};

use crate::mcs::{self, McsAxis, McsController, McsParams, McsReasons};
use crate::mcs2::{self, Mcs2Axis, Mcs2AxisReport, Mcs2Controller, Mcs2Params, Mcs2Reasons};
use crate::scu::{ScuAxis, ScuController};

/// Command timeout (C `DEFAULT_CONTROLLER_TIMEOUT` = 2.0 s).
const MCS2_TIMEOUT: Duration = Duration::from_secs(2);

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

            let params =
                Mcs2Params::new(moving_poll_ms as f64 / 1000.0, idle_poll_ms as f64 / 1000.0);
            let slot = params.controller_slot();
            register_port_with(ctx, &port_name, num_axes, params)?;
            let (handle, _) = connect_controller(
                &mcs2_port,
                MCS2_TIMEOUT,
                &port_name,
                format_args!(
                    "SmarActMCS2MotorDriver:MCS2Controller: cannot connect to MCS2 controller"
                ),
            );
            set_eos_ignored(&handle, mcs2::INPUT_EOS, mcs2::OUTPUT_EOS);
            let mut controller = Mcs2Controller::new(handle, MotorTrace::new(&port_name, 0));
            let port = PortManager::global()
                .find_port_handle(&port_name)
                .map_err(|e| format!("MCS2CreateController: {e}"))?;
            let reasons =
                Mcs2Reasons::lookup(&port).map_err(|e| format!("MCS2CreateController: {e}"))?;
            controller.set_reasons(reasons);
            let controller = Arc::new(Mutex::new(controller));
            let _ = slot.set(controller.clone());
            let poller = Controller::new();
            for axis in 0..num_axes {
                if unused_mask & (1 << axis) != 0 {
                    continue;
                }
                let motor: Arc<Mutex<dyn AsynMotor>> =
                    Arc::new(Mutex::new(Mcs2Axis::new(controller.clone(), axis as usize)));
                let report = Arc::new(Mcs2AxisReport {
                    controller: controller.clone(),
                    channel: axis,
                });
                poller.install_with_report(asyn_user(&port_name, axis), motor, report)?;
            }
            // C startPoller, smarActMCS2MotorDriver.cpp:95
            poller.start(ctx, &port_name, moving_poll_ms, idle_poll_ms, 2);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// A controller `smarAct{SCU,MCS}CreateController` made, connected to its I/O
/// port or not, as C's constructor carries on either way.
struct Registered<C> {
    port_name: String,
    controller: Arc<Mutex<C>>,
    poller: Controller,
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

/// Register `port_name` with the driver's `params`, then connect its
/// controller on it, as C's asynMotorController base constructor runs
/// before the driver's own, and start its poller, which the `CreateAxis`
/// axes join.
fn register<C>(
    ctx: &CommandContext,
    registry: &Registry<C>,
    args: &[ArgValue],
    params: impl ControllerParams,
    connect: impl FnOnce(&str) -> C,
) -> Result<Arc<Mutex<C>>, String> {
    let port_name = req_string(args, 0, "Port name [string]")?;
    register_port_with(ctx, &port_name, c_int(args, 2), params)?;
    let controller = Arc::new(Mutex::new(connect(&port_name)));
    let poller = Controller::new();
    // C startPoller, smarActSCUMotorDriver.cpp:114, smarActMCSMotorDriver.cpp:215
    poller.start(ctx, &port_name, period_ms(args, 3), period_ms(args, 4), 0);
    registry
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .push(Registered {
            port_name,
            controller: controller.clone(),
            poller,
            axes: BTreeSet::new(),
        });
    Ok(controller)
}

/// `smarAct{SCU,MCS}CreateAxis(controllerPortName, axisNumber, channel)`,
/// printing what C's does; `build` makes the axis on a connected controller.
fn create_axis<C>(
    registry: &Registry<C>,
    command: &str,
    duplicate: &str,
    args: &[ArgValue],
    build: impl FnOnce(Arc<Mutex<C>>, i32, i32) -> Arc<Mutex<dyn AsynMotor>>,
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
    let controller = reg.controller.clone();
    let motor = build(controller, axis, channel);
    reg.poller
        .install(asyn_user(&controller_port, axis), motor)
        .map(drop)
}

/// `smarActSCUCreateController(motorPortName, ioPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)`.
pub fn scu_create_controller_command() -> CommandDef {
    CommandDef::new(
        "smarActSCUCreateController",
        create_controller_args(),
        "smarActSCUCreateController(motorPortName, ioPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let io_port = req_string(args, 1, "I/O port name [string]")?;
            register(ctx, &SCU_CONTROLLERS, args, (), |port_name| {
                let (handle, connected) = connect_controller(
                    &io_port,
                    SCU_TIMEOUT,
                    port_name,
                    format_args!(
                        "SmarActSCUController:SmarActSCUController: cannot connect to SCU controller"
                    ),
                );
                // C's THROW_ without ASYN_CANDO_EXCEPTIONS: an epicsPrintf, and on.
                if connected.is_err() {
                    errlog_vprintf(format_args!(
                        "SmarActSCUController: unable to connect serial channel\n"
                    ));
                }
                ScuController::new(handle)
            })?;
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
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port = req_string(args, 0, "Controller Port name [string]")?;
            create_axis(
                &SCU_CONTROLLERS,
                "smarActSCUCreateAxis",
                "SmarActSCUCreateAxis",
                args,
                |controller, axis, channel| {
                    let (ax, probed) = ScuAxis::new(controller, channel, &port, axis);
                    if let Err(e) = probed {
                        errlog_vprintf(format_args!(
                            "SmarActSCUAxis::SmarActSCUAxis -- channel {axis} ASYN error {}\n",
                            e.status() as i32
                        ));
                    }
                    Arc::new(Mutex::new(ax))
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
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name [string]")?;
            let io_port = req_string(args, 1, "I/O port name [string]")?;
            let disable_speed = c_int(args, 5) != 0;
            if disable_speed {
                errlog_vprintf(format_args!(
                    "SmarActMCSController({port_name}): WARNING - The speed set commands have \
                     been disabled for this controller\n"
                ));
            }
            let params = McsParams::default();
            let slot = params.controller_slot();
            let controller = register(ctx, &MCS_CONTROLLERS, args, params, |port_name| {
                let link = mcs::connect_link(port_name, &io_port, MCS_TIMEOUT);
                McsController::new(link, disable_speed)
            })?;
            let port = PortManager::global()
                .find_port_handle(&port_name)
                .map_err(|e| format!("smarActMCSCreateController: {e}"))?;
            let reasons = McsReasons::lookup(&port)
                .map_err(|e| format!("smarActMCSCreateController: {e}"))?;
            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .set_reasons(reasons);
            let _ = slot.set(controller);
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
        move |args: &[ArgValue], _ctx: &CommandContext| {
            create_axis(
                &MCS_CONTROLLERS,
                "smarActMCSCreateAxis",
                "SmarActMCSCreateAxis",
                args,
                |controller, axis, channel| {
                    let (ax, probed) = McsAxis::new(controller, axis, channel);
                    if let Err(e) = probed {
                        errlog_vprintf(format_args!(
                            "SmarActMCSAxis::SmarActMCSAxis -- channel {axis} ASYN error {}\n",
                            e.status() as i32
                        ));
                    }
                    Arc::new(Mutex::new(ax))
                },
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}
