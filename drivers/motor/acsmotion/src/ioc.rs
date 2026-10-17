//! `AcsMotionConfig` and `SPiiPlusCreateProfile` iocsh commands for the
//! ACS SPiiPlus.
//!
//! C `AcsMotionConfig(acsPort, asynPort, numAxes, movingPollPeriod,
//! idlePollPeriod, virtualAxisList)`: connect the serial/TCP octet port,
//! make its comm, identify the controller ([`SpiiPlusController::new`]), and create controller
//! port `acsPort` whose axes records address as `@asyn(acsPort,axis)`. The poll
//! periods are seconds and `virtualAxisList` is comma-separated 0-based axis
//! indices, as in C.

use std::collections::HashSet;
use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::manager::PortManager;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::motor::controller_port::ControllerPort;

use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_int_req, arg_str_opt, arg_str_req, opt_double, opt_string,
    req_int, req_string,
};
use motor_common::model1::c_int;
use motor_common::{Controller, asyn_user, register_controller_port};

use crate::comm::{SpiiPlusComm, register_comm_port};
use crate::spiiplus::{AcsMotionParams, SpiiPlusAxis, SpiiPlusController};

/// Parse the `virtualAxisList` string (comma-separated 0-based indices)
/// into a set, ignoring blanks and out-of-range/unparseable entries (C warns and
/// continues). Returns the set of valid virtual axis indices.
fn parse_virtual_axes(list: &str, num_axes: usize) -> HashSet<usize> {
    let mut set = HashSet::new();
    for tok in list.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        match tok.parse::<usize>() {
            Ok(idx) if idx < num_axes => {
                set.insert(idx);
            }
            _ => {
                println!("AcsMotionConfig: ignoring invalid virtual axis \"{tok}\"");
            }
        }
    }
    set
}

/// Build the `AcsMotionConfig(acsPort, asynPort, numAxes, movingPollPeriod,
/// idlePollPeriod, virtualAxisList)` command.
pub fn acsmotion_config_command() -> CommandDef {
    CommandDef::new(
        "AcsMotionConfig",
        vec![
            arg_str_req("ACS port name"),
            arg_str_req("asyn port name"),
            arg_int_req("num axes"),
            arg_double_opt("Moving polling rate"),
            arg_double_opt("Idle polling rate"),
            arg_str_opt("Virtual axis list"),
        ],
        "AcsMotionConfig(acsPort, asynPort, numAxes, movingPollPeriod, idlePollPeriod, \
         virtualAxisList)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "ACS port name")?;
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let num_axes = req_int(args, 2, "num axes")?;
            if num_axes < 1 {
                return Err("AcsMotionConfig: numAxes must be > 0".into());
            }
            let num_axes = num_axes as usize;
            let moving_poll = opt_double(args, 3, 0.0, "Moving polling rate")?;
            let idle_poll = opt_double(args, 4, 0.0, "Idle polling rate")?;
            let virtual_list = opt_string(args, 5)?.unwrap_or_default();

            let virtual_axes = parse_virtual_axes(&virtual_list, num_axes);

            // C's constructor makes the comm `<ACSPortName>Comm` first.
            let comm_port = format!("{port_name}Comm");
            let comm = SpiiPlusComm::connect(&comm_port, &asyn_port)?;
            let owner = ctx.db().scan_control().interrupt_gate();
            register_comm_port(owner, &comm_port, num_axes as i32)?;
            let controller = SpiiPlusController::new(comm, &port_name, num_axes, &virtual_axes);
            let controller = Arc::new(Mutex::new(controller));

            let (params, profile_thread) = AcsMotionParams::new(controller.clone());
            let port = register_controller_port(
                ctx.db().scan_control().interrupt_gate(),
                &port_name,
                num_axes as i32,
                params,
            )?;
            // C ignores a failed epicsThreadCreate: the profile never runs.
            if let Err(e) = profile_thread.spawn(port) {
                println!("AcsMotionConfig: SPiiPlusProfile thread not started: {e}");
            }
            let polled = controller.clone();
            // C startPoller, SPiiPlusDriver.cpp:269
            let mut poller = Controller::new().controller_poll(move || {
                // C's poller ignores poll()'s status.
                let _ = polled.lock().unwrap_or_else(|e| e.into_inner()).poll();
            });
            for index in 0..num_axes {
                let ax = SpiiPlusAxis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                poller.install(asyn_user(&port_name, index as i32), motor)?;
            }
            poller.start(
                ctx,
                (moving_poll * 1000.0) as u64,
                (idle_poll * 1000.0) as u64,
                2,
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `SPiiPlusCreateProfile(SPiiPlusName, maxPoints, maxPulses)`: size the
/// profile of controller port `SPiiPlusName` and make its data collection
/// arrays in the controller.
pub fn spiiplus_create_profile_command() -> CommandDef {
    CommandDef::new(
        "SPiiPlusCreateProfile",
        vec![
            arg_str_opt("ACS port name"),
            arg_int_opt("Max points"),
            // C registers "Max points" here as well.
            arg_int_opt("Max pulses"),
        ],
        "SPiiPlusCreateProfile(acsPort, maxPoints, maxPulses)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = opt_string(args, 0)?.unwrap_or_default();
            // C passes a negative count on to its allocations.
            let max_points = c_int(args, 1).max(0) as usize;
            let max_pulses = c_int(args, 2).max(0) as usize;
            if !create_profile(&port_name, max_points, max_pulses) {
                println!(
                    "SPiiPlusController:SPiiPlusCreateProfile: Error port {port_name} not found"
                );
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `SPiiPlusCreateProfile`'s `initializeProfile` under the port's lock;
/// `false` where there is no SPiiPlus controller port `port_name`.
fn create_profile(port_name: &str, max_points: usize, max_pulses: usize) -> bool {
    let Ok(port) = PortManager::global().find_port_handle(port_name) else {
        return false;
    };
    port.with_driver_blocking::<ControllerPort<AcsMotionParams>, _>(move |d| {
        d.with_params(|params, ctx| {
            // C drops the status.
            let _ = params.initialize_profile_pulses(ctx, max_points, max_pulses);
        })
    })
    .is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::tests::{SERVOS, rig};

    #[test]
    fn create_profile_initializes_the_named_controller_port() {
        let rig = rig("spiiCreate", 1, SERVOS);
        assert!(create_profile("spiiCreateCtrl", 4, 2));
        let sent = rig.sent();
        assert!(sent.contains(&"GLOBAL REAL DC_DATA_1 (3)(4)".to_string()));
        let max_points = rig.with(|_, ctx| ctx.profile.max_points);
        assert_eq!(max_points, 4);
    }

    #[test]
    fn create_profile_needs_a_spiiplus_controller_port() {
        let _rig = rig("spiiCreateNot", 1, SERVOS);
        assert!(!create_profile("spiiCreateNone", 4, 2));
        // The octet line is a port, but not a SPiiPlus controller's.
        assert!(!create_profile("spiiCreateNot", 4, 2));
    }
}
