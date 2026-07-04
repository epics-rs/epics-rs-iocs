//! `PM304Setup` / `PM304Config` iocsh commands for the Mclennan PM304/PM600.
//!
//! Mirrors C `PM304Config(card, port, nAxes)`: connect the serial octet port,
//! identify the controller ([`Pm304Controller::new`]), and register each axis
//! behind a `DTYP`-keyed motor device support (`PM304_{card}_{index}`, 0-based).
//! `PM304Setup` (which only sized a controller array in C) is accepted as a
//! no-op for startup-script parity.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::pm304::{Pm304Axis, Pm304Controller};

/// Serial command timeout (C `TIMEOUT` 2 s).
const PM304_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `PM304Setup(maxControllers, [scanRate])` no-op command
/// (startup-script parity; the asyn-rs port allocates per `PM304Config`).
pub fn pm304_setup_command() -> CommandDef {
    CommandDef::new(
        "PM304Setup",
        vec![arg_int_req("maxControllers"), arg_int_opt("scanRate")],
        "PM304Setup(maxControllers, [scanRate]) - Accepted for parity; PM304Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("PM304Setup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PM304Config(card, asynPort, nAxes, [movingPollMs], [idlePollMs])`
/// command bound to `holder`.
pub fn pm304_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "PM304Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("nAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "PM304Config(card, asynPort, nAxes, [movingPollMs], [idlePollMs]) - \
         Create a Mclennan PM304/PM600 controller (DTYP PM304_{card}_{0..nAxes-1}); \
         the model (PM304 vs PM600) is auto-detected from the 1ID reply",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PM304Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let n_axes = req_int(args, 2, "nAxes")?;
            if n_axes < 1 {
                return Err("PM304Config: nAxes must be > 0".into());
            }
            let n_axes = n_axes as usize;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, PM304_TIMEOUT)?;
            let controller =
                Pm304Controller::new(handle, n_axes).map_err(|e| format!("PM304Config: {e}"))?;
            let ident = controller.ident().to_string();
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let ax = Pm304Axis::new(controller.clone(), index);
                let dtyp_key = format!("PM304_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "PM304Config: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PM304_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
