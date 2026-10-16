//! `PIJEDSSetup` / `PIJEDSConfig` iocsh commands for the piezosystem jena
//! E-516 controller, C's model-1 interface: records reach axis `signal` of
//! `card` through `DTYP "PIJEDS"` and `OUT "#C card S signal"`. Config
//! connects and identifies the controller at once, where C's `motor_init`
//! does so at iocInit.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, req_int, req_string};
use motor_common::model1::{Model1, SCAN_RATE};
use motor_common::model1_device;

use crate::pijeds::{PiJedsAxis, PiJedsController};

/// Serial command timeout (C `COMM_TIMEOUT` 2 s).
const PIJEDS_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devPIJEDS,"PIJEDS")` (devPIJena.dbd:2).
static DEV_PIJEDS: DevSup = model1_device("devPIJEDS", "PIJEDS");

/// `PIJEDS_NUM_CARDS` (drvPIJEDS.h:32).
static PIJEDS: Model1 = Model1::new(&DEV_PIJEDS, 10, SCAN_RATE);

/// `PIJEDSSetup(num_cards, scan_rate)`.
pub fn pijeds_setup_command() -> CommandDef {
    PIJEDS.setup_command("PIJEDSSetup")
}

/// `PIJEDSConfig(card, name, address)`.
pub fn pijeds_config_command() -> CommandDef {
    CommandDef::new(
        "PIJEDSConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "PIJEDSConfig(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if !PIJEDS.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                PIJEDS_TIMEOUT,
                b"\x11",
                b"\r",
                EosRefusal::Ignored,
            )?;
            let controller =
                PiJedsController::new(handle).map_err(|e| format!("PIJEDSConfig: {e}"))?;
            let num_axes = controller.num_axes();
            if num_axes == 0 {
                return Err("PIJEDSConfig: no axes detected".into());
            }
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                let ax = PiJedsAxis::new(controller.clone(), axis);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                PIJEDS.install(ctx, card, axis as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
