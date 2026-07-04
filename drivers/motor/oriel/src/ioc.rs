//! `EMC18011Setup` / `EMC18011Config` iocsh commands for the Oriel Encoder
//! Mike 18011.
//!
//! Mirrors C `EMC18011Config(card, port)`: connect the serial octet port,
//! identify the controller ([`Emc18011Controller::new`]), and register its fixed
//! three axes behind a `DTYP`-keyed motor device support
//! (`EMC18011_{card}_{index}`, 0-based). `EMC18011Setup` (which only sized a
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

use crate::emc18011::{Emc18011Axis, Emc18011Controller};

/// Serial command timeout (C `TIMEOUT` 1 s).
const EMC18011_TIMEOUT: Duration = Duration::from_secs(1);

/// Build the `EMC18011Setup(maxControllers, [scanRate])` no-op command
/// (startup-script parity; the asyn-rs port allocates per `EMC18011Config`).
pub fn emc18011_setup_command() -> CommandDef {
    CommandDef::new(
        "EMC18011Setup",
        vec![arg_int_req("maxControllers"), arg_int_opt("scanRate")],
        "EMC18011Setup(maxControllers, [scanRate]) - Accepted for parity; EMC18011Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("EMC18011Setup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `EMC18011Config(card, asynPort, [movingPollMs], [idlePollMs])`
/// command bound to `holder`.
pub fn emc18011_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "EMC18011Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "EMC18011Config(card, asynPort, [movingPollMs], [idlePollMs]) - \
         Create an Oriel Encoder Mike 18011 controller (DTYP EMC18011_{card}_{0..2}); \
         the controller has a fixed three axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("EMC18011Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;

            let handle = connect_serial(&asyn_port, EMC18011_TIMEOUT)?;
            let controller =
                Emc18011Controller::new(handle).map_err(|e| format!("EMC18011Config: {e}"))?;
            let ident = controller.ident().to_string();
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let ax = Emc18011Axis::new(controller.clone(), index);
                let dtyp_key = format!("EMC18011_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "EMC18011Config: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=EMC18011_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
