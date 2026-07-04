//! `C300CreateController` iocsh command for the nPoint C300.
//!
//! Mirrors C `C300CreateController(portName, C300PortName, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port, unlock the
//! controller ([`C300Controller::new`]), and register `numAxes` axes behind a
//! `DTYP`-keyed motor device support (`C300_{card}_{index}`, 0-based). The C
//! `portName` (the asyn motor port) has no analogue here; `card` keys the DTYP.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_ip;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::c300::{C300Axis, C300Controller};

/// Command timeout (C `C300_TIMEOUT` 1 s).
const C300_TIMEOUT: Duration = Duration::from_secs(1);

/// Build the `C300CreateController(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn c300_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "C300CreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "C300CreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create an nPoint C300 controller (DTYP C300_{card}_{index}) with numAxes axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("C300CreateController: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("C300CreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&asyn_port, C300_TIMEOUT)?;
            let controller = C300Controller::new(handle, num_axes as usize)
                .map_err(|e| format!("C300CreateController: {e}"))?;
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes as usize {
                let ax = C300Axis::new(controller.clone(), index);
                let dtyp_key = format!("C300_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "C300CreateController: card={card} asynPort={asyn_port} axes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=C300_{card}_{{0..{}}})",
                (num_axes - 1).max(0)
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
