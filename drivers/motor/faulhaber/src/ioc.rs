//! `MCDC2805Setup` / `MCDC2805Config` iocsh commands for the Faulhaber
//! MCDC2805.
//!
//! Mirrors C `MCDC2805Config(card, numMotors, name)`: connect the serial octet
//! port, probe the nodes ([`FaulhaberController::new`]), and register each
//! behind a `DTYP`-keyed motor device support (`MCDC2805_{card}_{node}`,
//! 0-based). `MCDC2805Setup` (which only sized a controller array in C) is
//! accepted as a no-op for startup-script parity.
//!
//! An extra `countsPerRev` argument (not in the C signature) supplies the
//! encoder resolution the C driver took from the record's `SREV` field, which
//! is not visible at the asyn-rs boundary.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::faulhaber::{FaulhaberAxis, FaulhaberController};

/// Serial command timeout (C `COMM_TIMEOUT` 2 s).
const MCDC2805_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `MCDC2805Setup(maxControllers, [scanRate])` no-op command
/// (startup-script parity; the asyn-rs port allocates per `MCDC2805Config`).
pub fn mcdc2805_setup_command() -> CommandDef {
    CommandDef::new(
        "MCDC2805Setup",
        vec![arg_int_req("maxControllers"), arg_int_opt("scanRate")],
        "MCDC2805Setup(maxControllers, [scanRate]) - Accepted for parity; MCDC2805Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("MCDC2805Setup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `MCDC2805Config(card, numMotors, asynPort, countsPerRev,
/// [movingPollMs], [idlePollMs])` command bound to `holder`.
pub fn mcdc2805_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "MCDC2805Config",
        vec![
            arg_int_req("card"),
            arg_int_req("numMotors"),
            arg_str_req("asynPort"),
            arg_int_req("countsPerRev"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "MCDC2805Config(card, numMotors, asynPort, countsPerRev, [movingPollMs], [idlePollMs]) - \
         Create a Faulhaber MCDC2805 controller (DTYP MCDC2805_{card}_{0..N-1}); \
         responding nodes are auto-detected up to numMotors",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("MCDC2805Config: card must be >= 0".into());
            }
            let num_motors = req_int(args, 1, "numMotors")?;
            if !(1..=8).contains(&num_motors) {
                return Err("MCDC2805Config: numMotors must be 1..=8".into());
            }
            let asyn_port = req_string(args, 2, "asynPort")?;
            let counts_per_rev = req_int(args, 3, "countsPerRev")?;
            if counts_per_rev <= 0 {
                return Err("MCDC2805Config: countsPerRev must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 4, 5)?;

            let handle = connect_serial(&asyn_port, MCDC2805_TIMEOUT)?;
            let controller = FaulhaberController::new(handle, num_motors as usize)
                .map_err(|e| format!("MCDC2805Config: {e}"))?;
            let ident = controller.ident().to_string();
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for node in 0..num_axes {
                let ax = FaulhaberAxis::new(controller.clone(), node, counts_per_rev as f64)
                    .map_err(|e| format!("MCDC2805Config: node {node}: {e}"))?;
                let dtyp_key = format!("MCDC2805_{card}_{node}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "MCDC2805Config: card={card} asynPort={asyn_port} nodes={num_axes} \
                 countsPerRev={counts_per_rev} ident=\"{ident}\" \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=MCDC2805_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
