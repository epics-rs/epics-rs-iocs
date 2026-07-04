//! iocsh commands for the Phytron driver.
//!
//! Mirrors the C two-step configuration: `phytronCreatePhymotion` /
//! `phytronCreateMCC` connect the asyn octet port and register a controller
//! under a logical name, then `phytronCreateAxis(controllerName, module, index)`
//! attaches each axis to a `DTYP`-keyed motor device support
//! (`PHYTRON_{controllerName}_{module}_{index}`). The controller is looked up by
//! name from a registry shared by the three commands (the C `controllers_`
//! list). Both port kinds (`drvAsynIPPort` / `drvAsynSerialPort`) work, so
//! `connect_ip` is used (the lookup is transport-agnostic).

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_ip;
use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_int_req, arg_str_req, opt_double, opt_int, poll_intervals,
    req_int, req_string,
};

use crate::phytron::{CtrlType, PhytronAxis, PhytronController};

/// Default communication timeout when the `timeoutMs` arg is omitted.
const DEFAULT_TIMEOUT_MS: f64 = 1000.0;

/// A registered controller plus the poll periods to install its axes with.
struct ControllerReg {
    controller: Arc<Mutex<PhytronController>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
}

/// Shared name -> controller registry for the three commands.
type Registry = Arc<Mutex<HashMap<String, ControllerReg>>>;

fn registry_lock(reg: &Registry) -> std::sync::MutexGuard<'_, HashMap<String, ControllerReg>> {
    reg.lock().unwrap_or_else(|e| e.into_inner())
}

/// Build all three Phytron iocsh commands, sharing one controller registry.
pub fn phytron_commands(holder: &Arc<MotorHolder>) -> Vec<CommandDef> {
    let registry: Registry = Arc::new(Mutex::new(HashMap::new()));
    vec![
        phytron_create_phymotion_command(&registry),
        phytron_create_mcc_command(&registry),
        phytron_create_axis_command(&registry, holder),
    ]
}

/// `phytronCreatePhymotion(controllerName, asynPort, [movingPollMs],
/// [idlePollMs], [timeoutMs], [noResetAtBoot])`.
fn phytron_create_phymotion_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "phytronCreatePhymotion",
        vec![
            arg_str_req("controllerName"),
            arg_str_req("asynPort"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
            arg_double_opt("timeoutMs"),
            arg_int_opt("noResetAtBoot"),
        ],
        "phytronCreatePhymotion(controllerName, asynPort, [movingPollMs], [idlePollMs], \
         [timeoutMs], [noResetAtBoot]) - Register a Phytron phyMOTION controller by name",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let asyn_port = req_string(args, 1, "asynPort")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;
            let timeout_ms = opt_double(args, 4, DEFAULT_TIMEOUT_MS, "timeoutMs")?;
            let no_reset = opt_int(args, 5, 0, "noResetAtBoot")?;

            register_controller(
                &registry,
                name,
                asyn_port,
                CtrlType::Phymotion,
                0,
                moving_poll_ms,
                idle_poll_ms,
                timeout_ms,
                no_reset != 1,
                "phytronCreatePhymotion",
            )
        },
    )
}

/// `phytronCreateMCC(controllerName, asynPort, address, [movingPollMs],
/// [idlePollMs], [timeoutMs], [noResetAtBoot])`.
fn phytron_create_mcc_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "phytronCreateMCC",
        vec![
            arg_str_req("controllerName"),
            arg_str_req("asynPort"),
            arg_int_req("address"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
            arg_double_opt("timeoutMs"),
            arg_int_opt("noResetAtBoot"),
        ],
        "phytronCreateMCC(controllerName, asynPort, address, [movingPollMs], [idlePollMs], \
         [timeoutMs], [noResetAtBoot]) - Register a Phytron MCC-1/MCC-2 controller by name",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let asyn_port = req_string(args, 1, "asynPort")?;
            let address = req_int(args, 2, "address")?;
            if !(0..=15).contains(&address) {
                return Err("phytronCreateMCC: address must be 0..=15".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let timeout_ms = opt_double(args, 5, DEFAULT_TIMEOUT_MS, "timeoutMs")?;
            let no_reset = opt_int(args, 6, 0, "noResetAtBoot")?;

            register_controller(
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
    let handle = connect_ip(
        &asyn_port,
        Duration::from_millis(timeout_ms.max(0.0) as u64),
    )?;
    let controller = PhytronController::new(handle, ctrl_type, address);
    if reset_at_boot {
        controller.reset_at_boot();
    }
    let controller = Arc::new(Mutex::new(controller));
    registry_lock(registry).insert(
        name.clone(),
        ControllerReg {
            controller,
            moving_poll_ms,
            idle_poll_ms,
        },
    );
    println!(
        "{cmd_name}: controller='{name}' asynPort={asyn_port} poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
         (attach axes with phytronCreateAxis)"
    );
    Ok(CommandOutcome::Continue)
}

/// `phytronCreateAxis(controllerName, module, index)`.
fn phytron_create_axis_command(registry: &Registry, holder: &Arc<MotorHolder>) -> CommandDef {
    let registry = registry.clone();
    let holder = holder.clone();
    CommandDef::new(
        "phytronCreateAxis",
        vec![
            arg_str_req("controllerName"),
            arg_int_req("module"),
            arg_int_req("index"),
        ],
        "phytronCreateAxis(controllerName, module, index) - Attach one Phytron axis \
         (DTYP PHYTRON_{controllerName}_{module}_{index})",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let module = req_int(args, 1, "module")?;
            let index = req_int(args, 2, "index")?;

            let reg = registry_lock(&registry);
            let entry = reg
                .get(&name)
                .ok_or_else(|| format!("phytronCreateAxis: controller '{name}' not registered"))?;
            let controller = entry.controller.clone();
            let moving_poll_ms = entry.moving_poll_ms;
            let idle_poll_ms = entry.idle_poll_ms;
            drop(reg);

            let axis = PhytronAxis::new(controller, module as i32, index as i32)
                .map_err(|e| format!("phytronCreateAxis: {e}"))?;
            let dtyp_key = format!("PHYTRON_{name}_{module}_{index}");
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            holder.install(ctx, dtyp_key.clone(), motor, moving_poll_ms, idle_poll_ms);
            println!("phytronCreateAxis: {dtyp_key} (module={module} index={index})");
            Ok(CommandOutcome::Continue)
        },
    )
}
