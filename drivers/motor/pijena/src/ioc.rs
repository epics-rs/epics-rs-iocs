//! `PIJEDSSetup` / `PIJEDSConfig` iocsh commands for the piezosystem jena
//! E-516 controller.
//!
//! Mirrors C `PIJEDSConfig(card, name, addr)`: connect the serial octet port,
//! bring the controller online, detect its axes ([`PiJedsController::new`]), and
//! register each behind a `DTYP`-keyed motor device support
//! (`PIJEDS_{card}_{axis}`, 0-based). `PIJEDSSetup` (which only sized a
//! controller array in C) is accepted as a no-op for startup-script parity.
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

use crate::pijeds::{PiJedsAxis, PiJedsController};

/// Serial command timeout (C `COMM_TIMEOUT` 2 s).
const PIJEDS_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `PIJEDSSetup(maxControllers, scanRate)` no-op command
/// (startup-script parity; the asyn-rs port allocates per `PIJEDSConfig`).
pub fn pijeds_setup_command() -> CommandDef {
    CommandDef::new(
        "PIJEDSSetup",
        vec![arg_int_req("maxControllers"), arg_int_opt("scanRate")],
        "PIJEDSSetup(maxControllers, [scanRate]) - Accepted for parity; PIJEDSConfig allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("PIJEDSSetup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIJEDSConfig(card, asynPort, [movingPollMs], [idlePollMs])`
/// command bound to `holder`.
pub fn pijeds_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "PIJEDSConfig",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "PIJEDSConfig(card, asynPort, [movingPollMs], [idlePollMs]) - \
         Create a piezosystem jena E-516 controller (DTYP PIJEDS_{card}_{0..N-1}); \
         axis count is auto-detected",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PIJEDSConfig: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;

            let handle = connect_serial(&asyn_port, PIJEDS_TIMEOUT)?;
            let controller =
                PiJedsController::new(handle).map_err(|e| format!("PIJEDSConfig: {e}"))?;
            let ident = controller.ident().to_string();
            let version = controller.version();
            let num_axes = controller.num_axes();
            if num_axes == 0 {
                return Err("PIJEDSConfig: no axes detected".into());
            }
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                let ax = PiJedsAxis::new(controller.clone(), axis);
                let dtyp_key = format!("PIJEDS_{card}_{axis}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "PIJEDSConfig: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" version={version} poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PIJEDS_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
