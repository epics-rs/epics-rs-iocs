//! `MCDC2805Setup` / `MCDC2805Config` iocsh commands for the Faulhaber
//! MCDC2805, C's model-1 interface: records reach axis `signal` of `card`
//! through `DTYP "MCDC2805"` and `OUT "#C card S signal"`. Config connects
//! and identifies the controller at once, where C's `motor_init` does so at
//! iocInit.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::connect_serial;
use motor_common::iocsh::{arg_int_req, arg_str_req, req_int, req_string};
use motor_common::model1::{Model1, SCAN_RATE};
use motor_common::model1_device;

use crate::faulhaber::{FaulhaberAxis, FaulhaberController};

/// Serial command timeout (C `COMM_TIMEOUT` 2 s).
const MCDC2805_TIMEOUT: Duration = Duration::from_secs(2);

/// The motor record's SREV default: C converts velocities with the record's
/// SREV (devMCDC2805.cc:227), which does not reach the driver here.
const SREV_DEFAULT: f64 = 200.0;

/// `device(motor,VME_IO,devMCDC2805,"MCDC2805")` (devFaulhaberMotor.dbd:2).
static DEV_MCDC2805: DevSup = model1_device("devMCDC2805", "MCDC2805");

/// `MCDC2805_NUM_CARDS` (drvMCDC2805.cc:56).
static MCDC2805: Model1 = Model1::new(&DEV_MCDC2805, 8, SCAN_RATE);

/// `MCDC2805Setup(num_cards, scan_rate)`.
pub fn mcdc2805_setup_command() -> CommandDef {
    MCDC2805.setup_command("MCDC2805Setup")
}

/// `MCDC2805Config(card, modules, name)`.
pub fn mcdc2805_config_command() -> CommandDef {
    CommandDef::new(
        "MCDC2805Config",
        vec![
            arg_int_req("Card being configured"),
            arg_int_req("# modules on this serial port"),
            arg_str_req("asyn port name"),
        ],
        "MCDC2805Config(card, modules, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if !MCDC2805.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let num_motors = req_int(args, 1, "# modules on this serial port")?;
            if !(1..=8).contains(&num_motors) {
                return Err("MCDC2805Config: numMotors must be 1..=8".into());
            }
            let asyn_port = req_string(args, 2, "asyn port name")?;

            let handle = connect_serial(&asyn_port, MCDC2805_TIMEOUT)?;
            let controller = FaulhaberController::new(handle, num_motors as usize)
                .map_err(|e| format!("MCDC2805Config: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for node in 0..num_axes {
                let ax = FaulhaberAxis::new(controller.clone(), node, SREV_DEFAULT)
                    .map_err(|e| format!("MCDC2805Config: node {node}: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                MCDC2805.install(ctx, card, node as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
