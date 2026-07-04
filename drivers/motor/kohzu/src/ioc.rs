//! `SC800Setup` / `SC800Config` iocsh commands for the Kohzu SC-200/400/800.
//!
//! Mirrors C `SC800Config(card, name, addr)`: connect the serial octet port,
//! identify the controller ([`KohzuController::new`]), and register each axis
//! behind a `DTYP`-keyed motor device support (`SC800_{card}_{index}`, 0-based).
//! The axis count comes from the controller model (SC-800 → 8, SC-400 → 4,
//! SC-200 → 2). `SC800Setup` (which only sized a controller array in C) is
//! accepted as a no-op for startup-script parity.
//!
//! C's GPIB `addr` argument is dropped: this is a serial octet port (asyn
//! address 0).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::kohzu::{KohzuAxis, KohzuController};

/// Serial command timeout (C `TIMEOUT` 3 s).
const SC800_TIMEOUT: Duration = Duration::from_secs(3);

/// Build the `SC800Setup(maxControllers, [scanRate])` no-op command
/// (startup-script parity; the asyn-rs port allocates per `SC800Config`).
pub fn sc800_setup_command() -> CommandDef {
    CommandDef::new(
        "SC800Setup",
        vec![arg_int_req("maxControllers"), arg_int_opt("scanRate")],
        "SC800Setup(maxControllers, [scanRate]) - Accepted for parity; SC800Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("SC800Setup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `SC800Config(card, asynPort, [movingPollMs], [idlePollMs])`
/// command bound to `holder`.
pub fn sc800_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "SC800Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "SC800Config(card, asynPort, [movingPollMs], [idlePollMs]) - \
         Create a Kohzu SC-200/400/800 controller (DTYP SC800_{card}_{0..N-1}); \
         axis count comes from the controller model",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("SC800Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;

            let handle = connect_serial(&asyn_port, SC800_TIMEOUT)?;
            let controller =
                KohzuController::new(handle).map_err(|e| format!("SC800Config: {e}"))?;
            let ident = controller.ident().to_string();
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let ax = KohzuAxis::new(controller.clone(), index);
                let dtyp_key = format!("SC800_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "SC800Config: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=SC800_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
