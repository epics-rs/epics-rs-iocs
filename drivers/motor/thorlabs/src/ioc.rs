//! `MDT695Setup` / `MDT695Config` iocsh commands for the ThorLabs
//! MDT693/694/695 piezo controller, C's model-1 interface: records reach axis
//! `signal` of `card` through `DTYP "MDT695"` and `OUT "#C card S signal"`.
//! Config connects and identifies the controller at once, where C's
//! `motor_init` does so at iocInit.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, req_int, req_string};
use motor_common::model1::{Model1, SCAN_RATE};
use motor_common::model1_device;

use crate::mdt695::{INPUT_EOS, Mdt695Axis, Mdt695Controller, OUTPUT_EOS};

/// Serial command timeout (C `TIMEOUT` 2 s).
const MDT695_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devMDT695,"MDT695")` (devThorLabs.dbd:2).
static DEV_MDT695: DevSup = model1_device("devMDT695", "MDT695");

/// `MDT695_NUM_CARDS` (drvMDT695.cc:61).
static MDT695: Model1 = Model1::new(&DEV_MDT695, 16, SCAN_RATE);

/// `MDT695Setup(num_cards, scan_rate)`.
pub fn mdt695_setup_command() -> CommandDef {
    MDT695.setup_command("MDT695Setup")
}

/// `MDT695Config(card, name)`.
pub fn mdt695_config_command() -> CommandDef {
    CommandDef::new(
        "MDT695Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "MDT695Config(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if !MDT695.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                MDT695_TIMEOUT,
                INPUT_EOS,
                OUTPUT_EOS,
                EosRefusal::Ignored,
            )?;
            let controller =
                Mdt695Controller::new(handle).map_err(|e| format!("MDT695Config: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let ax = Mdt695Axis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                MDT695.install(ctx, card, index as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
