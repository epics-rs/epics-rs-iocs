//! `ANC150AsynSetup` / `ANC150AsynConfig` iocsh commands for the attocube
//! ANC150 stepper.
//!
//! Mirrors C `ANC150AsynConfig(card, portName, numAxes, movingPollPeriod,
//! idlePollPeriod)`: connect the serial octet port, build one [`Anc150Axis`]
//! per axis, and register each behind a `DTYP`-keyed motor device support
//! (`ANC150_{card}_{axis}`, 0-based). `ANC150AsynSetup` (which only sized a
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

use crate::anc150::{Anc150Axis, Anc150Controller};

/// Serial command timeout (C `TIMEOUT` 2.0 s).
const ANC150_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `ANC150AsynSetup(maxControllers)` no-op command (startup-script
/// parity; the asyn-rs port allocates per `ANC150AsynConfig`).
pub fn anc150_setup_command() -> CommandDef {
    CommandDef::new(
        "ANC150AsynSetup",
        vec![arg_int_req("maxControllers")],
        "ANC150AsynSetup(maxControllers) - Accepted for parity; ANC150AsynConfig allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("ANC150AsynSetup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `ANC150AsynConfig(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn anc150_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "ANC150AsynConfig",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "ANC150AsynConfig(card, asynPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create an attocube ANC150 controller (DTYP ANC150_{card}_{0..numAxes-1})",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("ANC150AsynConfig: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if !(1..=6).contains(&num_axes) {
                return Err("ANC150AsynConfig: numAxes must be 1..=6".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, ANC150_TIMEOUT)?;
            let controller =
                Anc150Controller::new(handle).map_err(|e| format!("ANC150AsynConfig: {e}"))?;
            let ident = controller.ident().to_string();
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes as usize {
                let ax = Anc150Axis::new(controller.clone(), axis)
                    .map_err(|e| format!("ANC150AsynConfig: axis {axis}: {e}"))?;
                let dtyp_key = format!("ANC150_{card}_{axis}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "ANC150AsynConfig: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=ANC150_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
