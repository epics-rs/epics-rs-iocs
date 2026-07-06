//! `MCB4BCreateController` iocsh command for the ACS MCB-4B.
//!
//! Mirrors C `MCB4BCreateController(portName, MCB4BPortName, numAxes,
//! movingPollPeriod, idlePollPeriod)` from `MCB4BDriver.cpp`: connect the serial
//! octet port and register `numAxes` axes behind a `DTYP`-keyed motor device
//! support (`MCB4B_{card}_{index}`, 0-based). Following the epics-rs model-3
//! convention (`MCS2CreateController` etc.), the C string `portName` — the new
//! controller's own asyn port name — is replaced by an integer `card` used as
//! the DTYP prefix; the underlying serial port keeps its own name argument.
//!
//! The C constructor performs no identification I/O (it only connects and starts
//! the poller), so this command connects and installs axes without probing.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::mcb4b::{Mcb4bAxis, Mcb4bController};

/// Serial command timeout (C `MCB4BDriver` uses the asyn port default; the other
/// ACS/motor ports use 2 s, matched here).
const MCB4B_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `MCB4BCreateController(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn mcb4b_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "MCB4BCreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "MCB4BCreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create an ACS MCB-4B controller (DTYP MCB4B_{card}_{index}) with numAxes axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("MCB4BCreateController: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("MCB4BCreateController: numAxes must be > 0".into());
            }
            let num_axes = num_axes as usize;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, MCB4B_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(Mcb4bController::new(handle, num_axes)));

            for index in 0..num_axes {
                let ax = Mcb4bAxis::new(controller.clone(), index);
                let dtyp_key = format!("MCB4B_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "MCB4BCreateController: card={card} asynPort={asyn_port} axes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=MCB4B_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
