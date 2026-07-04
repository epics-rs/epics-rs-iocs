//! `SPiiPlusSetup` / `SPiiPlusConfig` iocsh commands for the ACS Tech80
//! SPiiPlus.
//!
//! Mirrors C `SPiiPlusConfig(card, port, modeStr)`: connect the serial/TCP
//! octet port, identify the controller ([`SpiiPlusController::new`]), and
//! register each auto-detected axis behind a `DTYP`-keyed motor device support
//! (`SPIIPLUS_{card}_{index}`, 0-based). `SPiiPlusSetup` (which only sized a
//! controller array in C) is accepted as a no-op for startup-script parity.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_opt, arg_str_req, opt_string, poll_intervals, req_int,
    req_string,
};

use crate::spiiplus::{CommandMode, SpiiPlusAxis, SpiiPlusController};

/// Serial command timeout (C `TIMEOUT` 2 s).
const SPIIPLUS_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `SPiiPlusSetup(maxControllers, [scanRate])` no-op command
/// (startup-script parity; the asyn-rs port allocates per `SPiiPlusConfig`).
pub fn spiiplus_setup_command() -> CommandDef {
    CommandDef::new(
        "SPiiPlusSetup",
        vec![arg_int_req("maxControllers"), arg_int_opt("scanRate")],
        "SPiiPlusSetup(maxControllers, [scanRate]) - Accepted for parity; SPiiPlusConfig allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("SPiiPlusSetup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `SPiiPlusConfig(card, asynPort, [mode], [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn spiiplus_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "SPiiPlusConfig",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_str_opt("mode"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "SPiiPlusConfig(card, asynPort, [mode], [movingPollMs], [idlePollMs]) - \
         Create an ACS Tech80 SPiiPlus controller (DTYP SPIIPLUS_{card}_{0..N-1}); \
         mode is BUF (default), DIR, or CON; axis count is auto-detected",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("SPiiPlusConfig: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let mode = match opt_string(args, 2) {
                Some(s) => CommandMode::parse(&s),
                None => CommandMode::Buffer,
            };
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, SPIIPLUS_TIMEOUT)?;
            let controller = SpiiPlusController::new(handle, mode)
                .map_err(|e| format!("SPiiPlusConfig: {e}"))?;
            let ident = controller.ident().to_string();
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let ax = SpiiPlusAxis::new(controller.clone(), index);
                let dtyp_key = format!("SPIIPLUS_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "SPiiPlusConfig: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=SPIIPLUS_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
