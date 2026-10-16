//! iocsh commands for the OMS MAXnet / MXA driver.
//!
//! Mirrors the C two-step configuration: `omsMAXnetConfig` / `omsMXAConfig`
//! connect the asyn octet port, run the init preamble, and register a
//! controller under a logical name; then `omsCreateAxis(controllerName, axis)`
//! probes and attaches each axis to a `DTYP`-keyed motor device support
//! (`OMS_{controllerName}_{axis}`). The controller is looked up by name from a
//! registry shared by the commands. Both port kinds (`drvAsynIPPort` /
//! `drvAsynSerialPort`) work.
//!
//! `omsMAXnetConfig` and `omsMXAConfig` differ only in the controller label
//! (and, in the C driver, a minimum-firmware value used to select command
//! forms — this port assumes the modern forms both controllers use when their
//! firmware meets its minimum), so both route through one shared body.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_int_req, arg_str_opt, arg_str_req, opt_double, poll_intervals,
    req_int, req_string,
};
use motor_common::{MotorHolder, asyn_user};

use crate::oms::{OmsAxis, OmsController};

/// Default communication timeout when the `timeoutMs` arg is omitted.
const DEFAULT_TIMEOUT_MS: f64 = 2000.0;

/// A registered controller plus the poll periods to install its axes with.
struct ControllerReg {
    controller: Arc<Mutex<OmsController>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
}

/// Shared name -> controller registry for the commands.
type Registry = Arc<Mutex<HashMap<String, ControllerReg>>>;

fn registry_lock(reg: &Registry) -> std::sync::MutexGuard<'_, HashMap<String, ControllerReg>> {
    reg.lock().unwrap_or_else(|e| e.into_inner())
}

/// Build all OMS iocsh commands, sharing one controller registry.
pub fn oms_commands(holder: &Arc<MotorHolder>) -> Vec<CommandDef> {
    let registry: Registry = Arc::new(Mutex::new(HashMap::new()));
    vec![
        oms_config_command(&registry, "omsMAXnetConfig", "MAXnet"),
        oms_config_command(&registry, "omsMXAConfig", "MXA"),
        oms_create_axis_command(&registry, holder),
    ]
}

/// `omsMAXnetConfig` / `omsMXAConfig`
/// `(controllerName, asynPort, [initString], [movingPollMs], [idlePollMs],
/// [timeoutMs])`.
fn oms_config_command(
    registry: &Registry,
    cmd_name: &'static str,
    controller_type: &'static str,
) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        cmd_name,
        vec![
            arg_str_req("controllerName"),
            arg_str_req("asynPort"),
            arg_str_opt("initString"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
            arg_double_opt("timeoutMs"),
        ],
        format!(
            "{cmd_name}(controllerName, asynPort, [initString], [movingPollMs], \
             [idlePollMs], [timeoutMs]) - Register a Pro-Dex OMS {controller_type} \
             controller by name"
        ),
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let asyn_port = req_string(args, 1, "asynPort")?;
            let init_string = req_string(args, 2, "initString").unwrap_or_default();
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let timeout_ms = opt_double(args, 5, DEFAULT_TIMEOUT_MS, "timeoutMs")?;

            if registry_lock(&registry).contains_key(&name) {
                return Err(format!(
                    "{cmd_name}: controller '{name}' already registered"
                ));
            }
            let handle = connect_with_default_eos(
                &asyn_port,
                Duration::from_millis(timeout_ms.max(0.0) as u64),
                &format!("{controller_type}Config"),
            )?;
            let controller = OmsController::new(handle, &asyn_port, controller_type);
            match controller.firmware_version() {
                Ok(fw) => println!("{cmd_name}: {controller_type} firmware: {fw}"),
                Err(e) => println!("{cmd_name}: warning: firmware read failed: {e}"),
            }
            controller
                .init(&init_string)
                .map_err(|e| format!("{cmd_name}: init failed: {e}"))?;

            let controller = Arc::new(Mutex::new(controller));
            registry_lock(&registry).insert(
                name.clone(),
                ControllerReg {
                    controller,
                    moving_poll_ms,
                    idle_poll_ms,
                },
            );
            println!(
                "{cmd_name}: controller='{name}' asynPort={asyn_port} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (attach axes with omsCreateAxis)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `omsCreateAxis(controllerName, axis)`.
fn oms_create_axis_command(registry: &Registry, holder: &Arc<MotorHolder>) -> CommandDef {
    let registry = registry.clone();
    let holder = holder.clone();
    CommandDef::new(
        "omsCreateAxis",
        vec![arg_str_req("controllerName"), arg_int_req("axis")],
        "omsCreateAxis(controllerName, axis) - Attach one OMS axis \
         (DTYP OMS_{controllerName}_{axis}); axis is 0-based",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let axis = req_int(args, 1, "axis")?;
            if !(0..=9).contains(&axis) {
                return Err("omsCreateAxis: axis must be 0..=9".into());
            }

            let reg = registry_lock(&registry);
            let entry = reg
                .get(&name)
                .ok_or_else(|| format!("omsCreateAxis: controller '{name}' not registered"))?;
            let controller = entry.controller.clone();
            let moving_poll_ms = entry.moving_poll_ms;
            let idle_poll_ms = entry.idle_poll_ms;
            drop(reg);

            let axis_dev = OmsAxis::new(controller, axis as usize)
                .map_err(|e| format!("omsCreateAxis: {e}"))?;
            let dtyp_key = format!("OMS_{name}_{axis}");
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_dev));
            holder.install(
                ctx,
                asyn_user(&name, axis as i32),
                dtyp_key.clone(),
                motor,
                moving_poll_ms,
                idle_poll_ms,
            );
            println!("omsCreateAxis: {dtyp_key} (axis={axis})");
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Connect to the octet port, giving it the OMS EOS (input `"\n\r"`, output
/// `"\n"`) only where st.cmd left that direction empty, as
/// `omsMAXnet`/`omsMXA` do after `getInputEos`/`getOutputEos`. As there, a
/// refused get skips that direction and a refused set is printed, never fatal
/// (omsMAXnet.cpp:134-142, omsMXA.cpp:129-137).
fn connect_with_default_eos(
    port_name: &str,
    timeout: Duration,
    who: &str,
) -> Result<SyncIOHandle, String> {
    let port = get_port(port_name).ok_or_else(|| format!("asyn port '{port_name}' not found"))?;
    let user = || AsynUser::new(0).with_addr(0);
    let handle = &port.handle;
    if handle
        .get_input_eos_blocking(user())
        .is_ok_and(|eos| eos.is_empty())
        && let Err(e) = handle.set_input_eos_blocking(user(), b"\n\r")
    {
        println!("{who}: unable to set InputEOS {port_name}: {e}");
    }
    if handle
        .get_output_eos_blocking(user())
        .is_ok_and(|eos| eos.is_empty())
        && let Err(e) = handle.set_output_eos_blocking(user(), b"\n")
    {
        println!("{who}: unable to set OutputEOS {port_name}: {e}");
    }
    Ok(SyncIOHandle::from_handle(handle.clone(), 0, timeout))
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

    /// A port with no EOS support of its own: both sets are refused.
    struct Bare(PortDriverBase);

    impl PortDriver for Bare {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
    }

    #[test]
    fn a_port_refusing_the_default_eos_still_connects() {
        let name = "omsEosRefused";
        let base = PortDriverBase::new(name, 1, PortFlags::default());
        let claim = claim_port(name).unwrap();
        let (rt, _actor) =
            create_port_runtime(&claim, Bare(base), RuntimeConfig::default()).unwrap();
        let port = rt.port_handle().clone();
        assert!(
            port.set_input_eos_blocking(AsynUser::new(0), b"\n\r")
                .is_err()
        );
        claim.publish(port, &open_owner()).unwrap();
        assert!(connect_with_default_eos(name, Duration::from_secs(1), "MAXnetConfig").is_ok());
    }
}
