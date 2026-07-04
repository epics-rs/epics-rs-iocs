//! `MDT695Setup` / `MDT695Config` iocsh commands for the ThorLabs MDT693/694/695
//! piezo controller.
//!
//! Mirrors C `MDT695Config(card, port)`: connect the serial octet port, identify
//! the controller ([`Mdt695Controller::new`], which also picks the axis count),
//! and register its axes behind a `DTYP`-keyed motor device support
//! (`MDT695_{card}_{index}`, 0-based). `MDT695Setup` (which only sized a
//! controller array in C) is accepted as a no-op for startup-script parity.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::mdt695::{Mdt695Axis, Mdt695Controller};

/// Serial command timeout (C `TIMEOUT` 2 s).
const MDT695_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `MDT695Setup(maxControllers, [scanRate])` no-op command
/// (startup-script parity; the asyn-rs port allocates per `MDT695Config`).
pub fn mdt695_setup_command() -> CommandDef {
    CommandDef::new(
        "MDT695Setup",
        vec![arg_int_req("maxControllers"), arg_int_opt("scanRate")],
        "MDT695Setup(maxControllers, [scanRate]) - Accepted for parity; MDT695Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("MDT695Setup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `MDT695Config(card, asynPort, [movingPollMs], [idlePollMs])`
/// command bound to `holder`.
pub fn mdt695_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "MDT695Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "MDT695Config(card, asynPort, [movingPollMs], [idlePollMs]) - \
         Create a ThorLabs MDT693/694/695 piezo controller (DTYP MDT695_{card}_{index}); \
         the axis count (1 for MDT694, else 3) is read from the controller",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("MDT695Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;

            let handle = connect_serial(&asyn_port, MDT695_TIMEOUT)?;
            let controller =
                Mdt695Controller::new(handle).map_err(|e| format!("MDT695Config: {e}"))?;
            let ident = controller.ident().to_string();
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let ax = Mdt695Axis::new(controller.clone(), index);
                let dtyp_key = format!("MDT695_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "MDT695Config: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=MDT695_{card}_{{0..{}}})",
                num_axes.saturating_sub(1)
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
