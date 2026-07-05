//! `PIGCS2CreateController` iocsh command for the PI GCS2 driver.
//!
//! Mirrors the C `PIasynController` constructor / `PI_GCS2_CreateController`
//! iocsh function: connect the serial/TCP octet port, identify and probe the
//! controller ([`PIGCS2Controller::new`], which auto-discovers connected axis
//! names via `SAI?`), then create the first `numAxes` of those axes in
//! discovery order behind a `DTYP`-keyed motor device support
//! (`PIGCS2_{card}_{axisName}`). Single-step, matching upstream — see
//! `gcs2.rs`'s "Config" doc section for why this is not a two-step
//! registry/explicit-`CreateAxis` API like `motor-phytron`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::gcs2::{PIGCS2Axis, PIGCS2Controller};

/// Command timeout (C `PIInterface::TIMEOUT` is 5.0 s).
const PIGCS2_TIMEOUT: Duration = Duration::from_secs(5);

/// Build the `PIGCS2CreateController(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn pigcs2_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "PIGCS2CreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "PIGCS2CreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a PI GCS2 stage controller (DTYP PIGCS2_{card}_{axisName}); axes are \
         auto-discovered via SAI? and the first numAxes of them are attached, in \
         controller-reported order",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PIGCS2CreateController: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("PIGCS2CreateController: numAxes must be > 0".into());
            }
            let num_axes = num_axes as usize;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, PIGCS2_TIMEOUT)?;
            let controller = PIGCS2Controller::new(handle)
                .map_err(|e| format!("PIGCS2CreateController: {e}"))?;
            let ident = controller.ident().to_string();
            let found: Vec<String> = controller.axis_names().to_vec();
            if found.len() < num_axes {
                return Err(format!(
                    "PIGCS2CreateController: requested number of axes ({num_axes}) out of \
                     range, only {} axis/axes supported",
                    found.len()
                ));
            }
            let controller = Arc::new(Mutex::new(controller));

            for (index, axis_name) in found.into_iter().take(num_axes).enumerate() {
                let axis = PIGCS2Axis::new(controller.clone(), axis_name.clone(), index)
                    .map_err(|e| format!("PIGCS2CreateController: {e}"))?;
                let dtyp_key = format!("PIGCS2_{card}_{axis_name}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "PIGCS2CreateController: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PIGCS2_{card}_{{axisName}})"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
