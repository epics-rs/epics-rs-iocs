//! `SMCcorvusCreateController` iocsh command for the Micos SMC corvus.
//!
//! Mirrors C `SMCcorvusCreateController(portName, corvusPort, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port and register
//! `numAxes` axes sharing one controller behind a `DTYP`-keyed motor device
//! support (`CORVUS_{card}_{index}`, 0-based). The corvus connects over either a
//! `drvAsynIPPort` or a `drvAsynSerialPort`, so `connect_ip` is used (the lookup
//! is transport-agnostic).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_ip;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::corvus::{CorvusAxis, CorvusController};
use crate::hydra::{HydraAxis, HydraController};
use crate::taurus::{TaurusAxis, TaurusController};

/// Command timeout.
const CORVUS_TIMEOUT: Duration = Duration::from_secs(2);

/// Hydra command timeout.
const HYDRA_TIMEOUT: Duration = Duration::from_secs(2);

/// Taurus command timeout.
const TAURUS_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `SMCcorvusCreateController(card, corvusPort, numAxes,
/// [movingPollMs], [idlePollMs])` command bound to `holder`.
pub fn corvus_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "SMCcorvusCreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("corvusPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "SMCcorvusCreateController(card, corvusPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a Micos SMC corvus controller (DTYP CORVUS_{card}_{index}) with numAxes axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("SMCcorvusCreateController: card must be >= 0".into());
            }
            let corvus_port = req_string(args, 1, "corvusPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if !(1..=3).contains(&num_axes) {
                return Err("SMCcorvusCreateController: numAxes must be 1..=3".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&corvus_port, CORVUS_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(CorvusController::new(handle, num_axes as usize)));

            for index in 0..num_axes as usize {
                let ax = CorvusAxis::new(controller.clone(), index)
                    .map_err(|e| format!("SMCcorvusCreateController: axis {index}: {e}"))?;
                let dtyp_key = format!("CORVUS_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "SMCcorvusCreateController: card={card} corvusPort={corvus_port} axes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=CORVUS_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `SMChydraCreateController(card, hydraPort, numAxes, [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn hydra_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "SMChydraCreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("hydraPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "SMChydraCreateController(card, hydraPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a Micos SMC hydra controller (DTYP HYDRA_{card}_{index}) with numAxes axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("SMChydraCreateController: card must be >= 0".into());
            }
            let hydra_port = req_string(args, 1, "hydraPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if !(1..=2).contains(&num_axes) {
                return Err("SMChydraCreateController: numAxes must be 1..=2".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&hydra_port, HYDRA_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(HydraController::new(handle)));

            for index in 0..num_axes as usize {
                let ax = HydraAxis::new(controller.clone(), index)
                    .map_err(|e| format!("SMChydraCreateController: axis {index}: {e}"))?;
                let dtyp_key = format!("HYDRA_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "SMChydraCreateController: card={card} hydraPort={hydra_port} axes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=HYDRA_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `SMCTaurusCreateController(card, taurusPort, numAxes,
/// [movingPollMs], [idlePollMs])` command bound to `holder`.
pub fn taurus_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "SMCTaurusCreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("taurusPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "SMCTaurusCreateController(card, taurusPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a Micos SMC Taurus controller (DTYP TAURUS_{card}_{index}) with numAxes axes",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("SMCTaurusCreateController: card must be >= 0".into());
            }
            let taurus_port = req_string(args, 1, "taurusPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("SMCTaurusCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&taurus_port, TAURUS_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(TaurusController::new(handle)));

            for index in 0..num_axes as usize {
                let ax = TaurusAxis::new(controller.clone(), index)
                    .map_err(|e| format!("SMCTaurusCreateController: axis {index}: {e}"))?;
                let dtyp_key = format!("TAURUS_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "SMCTaurusCreateController: card={card} taurusPort={taurus_port} axes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=TAURUS_{card}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
