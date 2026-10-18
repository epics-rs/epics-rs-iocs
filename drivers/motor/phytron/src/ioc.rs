//! iocsh commands for the Phytron driver.
//!
//! Mirrors the C two-step configuration: `phytronCreateController` (phyMOTION)
//! / `phytronCreateMCC` connect the asyn octet port and create controller port
//! `controllerName`, then `phytronCreateAxis(controllerName, module, index)`
//! adds axis `module*10 + index`, which records address as
//! `@asyn(controllerName,module*10+index)`. The controller is looked up by
//! name from a registry shared by the three commands (the C `controllers_`
//! list). Both port kinds (`drvAsynIPPort` / `drvAsynSerialPort`) work.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::base::server::ioc_app::{InitHookState, init_hook_register};
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::connect_controller;
use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_int_req, arg_str_req, opt_double, opt_int, poll_intervals,
    req_int, req_string,
};
use motor_common::{AxisReport, Controller, asyn_user, register_port_with};

use crate::phytron::{CtrlType, PhytronAxis, PhytronAxisReport, PhytronController, PhytronParams};

/// Default communication timeout when the `timeoutMs` arg is omitted.
const DEFAULT_TIMEOUT_MS: f64 = 1000.0;

/// A registered controller and the started poller its axes join.
struct ControllerReg {
    controller: Arc<Mutex<PhytronController>>,
    poller: Arc<Controller>,
}

/// Shared name -> controller registry for the three commands.
type Registry = Arc<Mutex<HashMap<String, ControllerReg>>>;

fn registry_lock(reg: &Registry) -> std::sync::MutexGuard<'_, HashMap<String, ControllerReg>> {
    reg.lock().unwrap_or_else(|e| e.into_inner())
}

/// C `phytronController` creates its port with 0xFF axes, room for every
/// `module*10 + index` address.
const PHYTRON_PORT_AXES: i32 = 0xFF;

/// Build all three Phytron iocsh commands, sharing one controller registry.
pub fn phytron_commands() -> Vec<CommandDef> {
    let registry: Registry = Arc::new(Mutex::new(HashMap::new()));
    vec![
        phytron_create_phymotion_command(&registry),
        phytron_create_mcc_command(&registry),
        phytron_create_axis_command(&registry),
    ]
}

/// `phytronCreateController(controllerName, asynPort, movingPollPeriod,
/// idlePollPeriod, timeout, noResetAtBoot)`, C's `phytronCreatePhymotion`;
/// periods and timeout in ms.
fn phytron_create_phymotion_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "phytronCreateController",
        vec![
            arg_str_req("Controller name"),
            arg_str_req("Phytron communication port name"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
            arg_double_opt("Timeout (ms)"),
            arg_int_opt("Do not restart controller with IOC"),
        ],
        "phytronCreateController(controllerName, asynPort, movingPollPeriod, idlePollPeriod, \
         timeout, noResetAtBoot)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller name")?;
            let asyn_port = req_string(args, 1, "Phytron communication port name")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;
            let timeout_ms = opt_double(args, 4, DEFAULT_TIMEOUT_MS, "Timeout (ms)")?;
            let no_reset = opt_int(args, 5, 0, "Do not restart controller with IOC")?;

            register_controller(
                ctx,
                &registry,
                name,
                asyn_port,
                CtrlType::Phymotion,
                0,
                moving_poll_ms,
                idle_poll_ms,
                timeout_ms,
                no_reset != 1,
                "phytronCreateController",
            )
        },
    )
}

/// `phytronCreateMCC(controllerName, asynPort, address, movingPollPeriod,
/// idlePollPeriod, timeout, noResetAtBoot)`, periods and timeout in ms.
fn phytron_create_mcc_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "phytronCreateMCC",
        vec![
            arg_str_req("Controller name"),
            arg_str_req("Phytron communication port name"),
            arg_int_req("Controller address (0..15)"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
            arg_double_opt("Timeout (ms)"),
            arg_int_opt("Do not restart controller with IOC"),
        ],
        "phytronCreateMCC(controllerName, asynPort, address, movingPollPeriod, idlePollPeriod, \
         timeout, noResetAtBoot)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller name")?;
            let asyn_port = req_string(args, 1, "Phytron communication port name")?;
            let address = req_int(args, 2, "Controller address (0..15)")?;
            if !(0..=15).contains(&address) {
                return Err("phytronCreateMCC: address must be 0..=15".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let timeout_ms = opt_double(args, 5, DEFAULT_TIMEOUT_MS, "Timeout (ms)")?;
            let no_reset = opt_int(args, 6, 0, "Do not restart controller with IOC")?;

            register_controller(
                ctx,
                &registry,
                name,
                asyn_port,
                CtrlType::Mcc,
                address as u8,
                moving_poll_ms,
                idle_poll_ms,
                timeout_ms,
                no_reset != 1,
                "phytronCreateMCC",
            )
        },
    )
}

/// Shared body for the two create-controller commands.
#[allow(clippy::too_many_arguments)]
fn register_controller(
    ctx: &CommandContext,
    registry: &Registry,
    name: String,
    asyn_port: String,
    ctrl_type: CtrlType,
    address: u8,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
    timeout_ms: f64,
    reset_at_boot: bool,
    cmd_name: &str,
) -> Result<CommandOutcome, String> {
    if registry_lock(registry).contains_key(&name) {
        return Err(format!(
            "{cmd_name}: controller '{name}' already registered"
        ));
    }
    let params = PhytronParams::new(moving_poll_ms as f64 / 1000.0, idle_poll_ms as f64 / 1000.0);
    let slot = params.controller_slot();
    register_port_with(ctx, &name, PHYTRON_PORT_AXES, params)?;
    // C leaves a controller that misses its port out of the list
    // phytronCreateAxis searches, and skips the reset; its port stays.
    let (handle, connected) = connect_controller(
        &asyn_port,
        Duration::from_millis(timeout_ms.max(0.0) as u64),
        &name,
        format_args!("phytronController::phytronController: cannot connect to phytron controller"),
    );
    if connected.is_err() {
        return Ok(CommandOutcome::Continue);
    }
    let mut controller = PhytronController::new(handle, &name, ctrl_type, address);
    if reset_at_boot {
        controller.reset_at_boot(&name);
    }
    let port = PortManager::global()
        .find_port_handle(&name)
        .map_err(|e| format!("{cmd_name}: {e}"))?;
    let axis_status = port
        .drv_user_create_blocking(&DrvUserRequest::new("AXIS_STATUS", 0))
        .map_err(|e| format!("{cmd_name}: {e}"))?
        .reason;
    controller.set_axis_status_reason(axis_status);
    let controller = Arc::new(Mutex::new(controller));
    let _ = slot.set(controller.clone());
    let poller = Controller::new().controller_poll({
        let controller = controller.clone();
        move || controller.lock().unwrap_or_else(|e| e.into_inner()).poll()
    });
    // C startPoller, phytronAxisMotor.cpp:240
    poller.start(&name, moving_poll_ms, idle_poll_ms, 5);
    let mut controllers = registry_lock(registry);
    // C registers its hook with the first controller (:186-187).
    if controllers.is_empty() {
        let registry = registry.clone();
        init_hook_register(Arc::new(move |state| {
            if state == InitHookState::AfterInitDevSup {
                wait_for_initial_readout(&registry);
            }
        }));
    }
    controllers.insert(
        name.clone(),
        ControllerReg {
            controller,
            poller: Arc::new(poller),
        },
    );
    Ok(CommandOutcome::Continue)
}

/// C `phytronController::epicsInithookFunction` (phytronAxisMotor.cpp:
/// 1003-1024): before motorRecord init_record pass 1, wait until every
/// controller has been read once. The shell thread waits; the pollers run.
fn wait_for_initial_readout(registry: &Registry) {
    let controllers: Vec<_> = registry_lock(registry)
        .values()
        .map(|reg| reg.controller.clone())
        .collect();
    for controller in controllers {
        while !controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .did_initial_readout()
        {
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

/// `phytronCreateAxis(controllerName, module, index)`, printing C's error lines.
fn phytron_create_axis_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "phytronCreateAxis",
        vec![
            arg_str_req("Controller Name"),
            arg_int_req("Module index"),
            arg_int_req("Axis index"),
        ],
        "phytronCreateAxis(controllerName, module, index)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller Name")?;
            let module = req_int(args, 1, "Module index")?;
            let index = req_int(args, 2, "Axis index")?;
            if index <= 0 {
                println!("ERROR: invalid axis specified");
                return Ok(CommandOutcome::Continue);
            }

            let reg = registry_lock(&registry);
            let Some(entry) = reg.get(&name) else {
                println!("ERROR: phytronCreateAxis: Controller {name} is not registered");
                return Ok(CommandOutcome::Continue);
            };
            let controller = entry.controller.clone();
            let poller = entry.poller.clone();
            drop(reg);

            let axis_no = (module * 10 + index) as i32;
            // C constructs the axis under pC->lock() (phytronAxisMotor.cpp:
            // 1262-1265).
            poller.install_built(asyn_user(&name, axis_no), || {
                let axis = PhytronAxis::new(controller.clone(), module as i32, index as i32)
                    .map_err(|e| format!("phytronCreateAxis: {e}"))?;
                let report: Arc<dyn AxisReport> = Arc::new(PhytronAxisReport(axis_no));
                Ok((Arc::new(Mutex::new(axis)), Some(report)))
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}
