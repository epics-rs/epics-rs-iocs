//! `AcsMotionConfig` iocsh command for the ACS SPiiPlus.
//!
//! Mirrors C `AcsMotionConfig(acsPort, asynPort, numAxes, movingPoll, idlePoll,
//! virtualAxisList)`: connect the serial/TCP octet port, identify the controller
//! ([`SpiiPlusController::new`]), and register each axis behind a `DTYP`-keyed
//! motor device support (`ACSMOTION_{card}_{index}`, 0-based).
//!
//! Two arguments extend the C signature to carry configuration the C driver
//! reads from runtime aux PVs that are absent at the plain motor boundary:
//! `homingMethod` (the mbbo homing-method selector, applied to every axis;
//! default limit+index) and the poll intervals in milliseconds (the C module's
//! `movingPoll`/`idlePoll` are seconds). The `virtualAxisList` is a
//! comma/space-separated list of 0-based axis indices, as in C.

use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_opt, arg_str_req, opt_int, opt_string, poll_intervals,
    req_int, req_string,
};

use crate::spiiplus::{HOME_LIMIT_INDEX, SpiiPlusAxis, SpiiPlusController};

/// Command timeout (C `SPiiPlusComm` uses the asyn default; 2 s here).
const ACSMOTION_TIMEOUT: Duration = Duration::from_secs(2);

/// Parse the `virtualAxisList` string (comma/space-separated 0-based indices)
/// into a set, ignoring blanks and out-of-range/unparseable entries (C warns and
/// continues). Returns the set of valid virtual axis indices.
fn parse_virtual_axes(list: &str, num_axes: usize) -> HashSet<usize> {
    let mut set = HashSet::new();
    for tok in list.split([',', ' ', '\t']).filter(|s| !s.is_empty()) {
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

/// Build the `AcsMotionConfig(card, asynPort, numAxes, [virtualAxisList],
/// [homingMethod], [movingPollMs], [idlePollMs])` command bound to `holder`.
pub fn acsmotion_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "AcsMotionConfig",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_str_opt("virtualAxisList"),
            arg_int_opt("homingMethod"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "AcsMotionConfig(card, asynPort, numAxes, [virtualAxisList], [homingMethod], \
         [movingPollMs], [idlePollMs]) - Create an ACS SPiiPlus controller (DTYP \
         ACSMOTION_{card}_{0..N-1}); virtualAxisList is comma/space-separated 0-based \
         indices; homingMethod is the mbbo selector 0..6 (default 1=limit+index)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("AcsMotionConfig: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("AcsMotionConfig: numAxes must be > 0".into());
            }
            let num_axes = num_axes as usize;
            let virtual_list = opt_string(args, 3).unwrap_or_default();
            let homing_method = opt_int(args, 4, HOME_LIMIT_INDEX as i64, "homingMethod")? as i32;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 5, 6)?;

            let virtual_axes = parse_virtual_axes(&virtual_list, num_axes);

            let handle = connect_serial(&asyn_port, ACSMOTION_TIMEOUT)?;
            let controller = SpiiPlusController::new(handle, num_axes, homing_method)
                .map_err(|e| format!("AcsMotionConfig: {e}"))?;
            let ident = controller.ident().to_string();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let is_virtual = virtual_axes.contains(&index);
                let ax = SpiiPlusAxis::new(controller.clone(), index, is_virtual);
                let dtyp_key = format!("ACSMOTION_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "AcsMotionConfig: card={card} asynPort={asyn_port} axes={num_axes} \
                 virtual={virtual_axes:?} homingMethod={homing_method} ident=\"{ident}\" \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=ACSMOTION_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
