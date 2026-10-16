//! `SPiiPlusSetup` / `SPiiPlusConfig` iocsh commands for the ACS Tech80
//! SPiiPlus, C's model-1 interface: records reach axis `signal` of `card`
//! through `DTYP "SPiiPlus"` and `OUT "#C card S signal"`. Config connects
//! and identifies the controller at once, where C's `motor_init` does so at
//! iocInit.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, opt_string, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::spiiplus::{CommandMode, INPUT_EOS, OUTPUT_EOS, SpiiPlusAxis, SpiiPlusController};

/// Serial command timeout (C `TIMEOUT` 2 s).
const SPIIPLUS_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devSPiiPlus,"SPiiPlus")` (devSPiiPlus.dbd:2).
static DEV_SPIIPLUS: DevSup = model1_device("devSPiiPlus", "SPiiPlus");

/// `SPiiPlus_NUM_CARDS` (drvSPiiPlus.cc:67).
static SPIIPLUS: Model1 = Model1::new(&DEV_SPIIPLUS, 8, SCAN_RATE);

/// `SPiiPlusSetup(num_cards, scan_rate)`.
pub fn spiiplus_setup_command() -> CommandDef {
    SPIIPLUS.setup_command("SPiiPlusSetup")
}

/// `SPiiPlusConfig(card, name, mode)`.
pub fn spiiplus_config_command() -> CommandDef {
    CommandDef::new(
        "SPiiPlusConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_str_req("Command Mode [BUFfer/CONnect/DIRect]"),
        ],
        "SPiiPlusConfig(card, name, mode)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !SPIIPLUS.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let mode = CommandMode::parse(&opt_string(args, 2)?.unwrap_or_default());

            let handle = connect_with_eos(
                &asyn_port,
                SPIIPLUS_TIMEOUT,
                INPUT_EOS,
                OUTPUT_EOS,
                EosRefusal::Ignored,
            )?;
            let controller = SpiiPlusController::new(handle, mode)
                .map_err(|e| format!("SPiiPlusConfig: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let ax = SpiiPlusAxis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                SPIIPLUS.install(ctx, card, index as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
