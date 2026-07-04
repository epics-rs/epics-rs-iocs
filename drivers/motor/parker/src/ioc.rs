//! `OEMCreateController` iocsh command for the Parker OEM750 series.
//!
//! Mirrors C `OEMCreateController(portName, OEMPortName, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port and register
//! `numAxes` axes (unit address = axis + 1) behind a `DTYP`-keyed motor device
//! support (`OEM_{card}_{index}`, 0-based).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_ip;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::acr::{AcrAxis, AcrController};
use crate::oem::{OemAxis, OemController};

/// Command timeout.
const OEM_TIMEOUT: Duration = Duration::from_secs(1);

/// ACR command timeout.
const ACR_TIMEOUT: Duration = Duration::from_secs(1);

/// Build the `OEMCreateController(card, oemPort, numAxes, [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn oem_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "OEMCreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("oemPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "OEMCreateController(card, oemPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a Parker OEM750 controller (DTYP OEM_{card}_{index}) with numAxes axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("OEMCreateController: card must be >= 0".into());
            }
            let oem_port = req_string(args, 1, "oemPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("OEMCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&oem_port, OEM_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(OemController::new(handle)));

            for index in 0..num_axes as usize {
                let ax = OemAxis::new(controller.clone(), index)
                    .map_err(|e| format!("OEMCreateController: axis {index}: {e}"))?;
                let dtyp_key = format!("OEM_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "OEMCreateController: card={card} oemPort={oem_port} axes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=OEM_{card}_{{0..{}}})",
                (num_axes - 1).max(0)
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `ACRCreateController(card, acrPort, numAxes, [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn acr_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "ACRCreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("acrPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "ACRCreateController(card, acrPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a Parker ACR/Aries controller (DTYP ACR_{card}_{index}) with numAxes axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("ACRCreateController: card must be >= 0".into());
            }
            let acr_port = req_string(args, 1, "acrPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("ACRCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&acr_port, ACR_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(AcrController::new(handle)));

            for index in 0..num_axes as usize {
                let ax = AcrAxis::new(controller.clone(), index)
                    .map_err(|e| format!("ACRCreateController: axis {index}: {e}"))?;
                let dtyp_key = format!("ACR_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "ACRCreateController: card={card} acrPort={acr_port} axes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=ACR_{card}_{{0..{}}})",
                (num_axes - 1).max(0)
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
