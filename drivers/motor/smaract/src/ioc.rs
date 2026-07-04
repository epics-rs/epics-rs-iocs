//! `MCS2CreateController` iocsh command for the SmarAct MCS2.
//!
//! Mirrors C `MCS2CreateController(portName, MCS2PortName, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port, read the
//! controller serial number ([`Mcs2Controller::new`]), and register `numAxes`
//! channels behind a `DTYP`-keyed motor device support (`MCS2_{card}_{index}`,
//! 0-based; the channel number equals the index).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_ip;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::mcs2::{Mcs2Axis, Mcs2Controller};

/// Command timeout.
const MCS2_TIMEOUT: Duration = Duration::from_secs(1);

/// Build the `MCS2CreateController(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn mcs2_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "MCS2CreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "MCS2CreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a SmarAct MCS2 controller (DTYP MCS2_{card}_{index}) with numAxes channels",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("MCS2CreateController: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("MCS2CreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&asyn_port, MCS2_TIMEOUT)?;
            let controller =
                Mcs2Controller::new(handle).map_err(|e| format!("MCS2CreateController: {e}"))?;
            let ident = controller.ident().to_string();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes as usize {
                let ax = Mcs2Axis::new(controller.clone(), index);
                let dtyp_key = format!("MCS2_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "MCS2CreateController: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=MCS2_{card}_{{0..{}}})",
                (num_axes - 1).max(0)
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
