//! `SC800Setup` / `SC800Config` iocsh commands for the Kohzu SC-200/400/800,
//! C's model-1 interface: records reach axis `signal` of `card` through
//! `DTYP "SC-800"` and `OUT "#C card S signal"`.
//!
//! Config connects and identifies the controller at once, where C's
//! `motor_init` does so at iocInit; the axis count comes from the model
//! (SC-800 → 8, SC-400 → 4, SC-200 → 2).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::kohzu::{INPUT_EOS, KohzuAxis, KohzuController, OUTPUT_EOS};

/// Serial command timeout (C `TIMEOUT` 3 s).
const SC800_TIMEOUT: Duration = Duration::from_secs(3);

/// `device(motor,VME_IO, devSC800, "SC-800")` (devKohzuMotor.dbd:4).
static DEV_SC800: DevSup = model1_device("devSC800", "SC-800");

/// `SC800_NUM_CARDS` (drvSC800.cc:59).
static SC800: Model1 = Model1::new(&DEV_SC800, 16, SCAN_RATE);

/// `SC800Setup(num_cards, scan_rate)`.
pub fn sc800_setup_command() -> CommandDef {
    SC800.setup_command("SC800Setup")
}

/// `SC800Config(card, name, address)`; `address` is GPIB's, unused on a
/// serial port.
pub fn sc800_config_command() -> CommandDef {
    CommandDef::new(
        "SC800Config",
        vec![
            arg_int_req("Card# being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "SC800Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !SC800.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let handle = connect_with_eos(
                &asyn_port,
                SC800_TIMEOUT,
                INPUT_EOS,
                OUTPUT_EOS,
                EosRefusal::Ignored,
            )?;
            let controller =
                KohzuController::new(handle).map_err(|e| format!("SC800Config: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));
            for index in 0..num_axes {
                let motor: Arc<Mutex<dyn AsynMotor>> =
                    Arc::new(Mutex::new(KohzuAxis::new(controller.clone(), index)));
                SC800.install(ctx, card, index as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
