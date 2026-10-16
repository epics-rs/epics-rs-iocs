//! `AcsMotionConfig` iocsh command for the ACS SPiiPlus.
//!
//! C `AcsMotionConfig(acsPort, asynPort, numAxes, movingPollPeriod,
//! idlePollPeriod, virtualAxisList)`: connect the serial/TCP octet port,
//! identify the controller ([`SpiiPlusController::new`]), and create controller
//! port `acsPort` whose axes records address as `@asyn(acsPort,axis)`. The poll
//! periods are seconds and `virtualAxisList` is comma-separated 0-based axis
//! indices, as in C.

use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{
    arg_double_opt, arg_int_req, arg_str_opt, arg_str_req, opt_double, opt_string, req_int,
    req_string,
};
use motor_common::{PortDriver, asyn_user, install, register_port};

use crate::spiiplus::{HOME_LIMIT_INDEX, INPUT_EOS, OUTPUT_EOS, SpiiPlusAxis, SpiiPlusController};

/// Command timeout (C `SPiiPlusComm` uses the asyn default; 2 s here).
const ACSMOTION_TIMEOUT: Duration = Duration::from_secs(2);

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

            let handle = connect_with_eos(
                &asyn_port,
                ACSMOTION_TIMEOUT,
                INPUT_EOS,
                OUTPUT_EOS,
                EosRefusal::Ignored,
            )?;
            let controller =
                SpiiPlusController::new(handle, &port_name, num_axes, HOME_LIMIT_INDEX)
                    .map_err(|e| format!("AcsMotionConfig: {e}"))?;
            let controller = Arc::new(Mutex::new(controller));

            register_port(&port_name, PortDriver::Controller, num_axes as i32)?;
            for index in 0..num_axes {
                let is_virtual = virtual_axes.contains(&index);
                let ax = SpiiPlusAxis::new(controller.clone(), index, is_virtual);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                install(
                    ctx,
                    asyn_user(&port_name, index as i32),
                    motor,
                    (moving_poll * 1000.0) as u64,
                    (idle_poll * 1000.0) as u64,
                )?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
