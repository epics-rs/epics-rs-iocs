//! `EMC18011Setup` / `EMC18011Config` iocsh commands for the Oriel Encoder
//! Mike 18011, C's model-1 interface: records reach axis `signal` of `card`
//! through `DTYP "EMC18011"` and `OUT "#C card S signal"`.
//!
//! Config connects and identifies the controller at once, where C's
//! `motor_init` does so at iocInit; the controller has a fixed three axes.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::emc18011::{Emc18011Axis, Emc18011Controller};

/// Serial command timeout (C `TIMEOUT` 1 s).
const EMC18011_TIMEOUT: Duration = Duration::from_secs(1);

/// `device(motor,VME_IO,devEMC18011,"EMC18011")` (devOriel.dbd:2).
static DEV_EMC18011: DevSup = model1_device("devEMC18011", "EMC18011");

/// `EMC18011_NUM_CARDS` (drvEMC18011.cc:66).
static EMC18011: Model1 = Model1::new(&DEV_EMC18011, 16, SCAN_RATE);

/// `EMC18011Setup(num_cards, scan_rate)`.
pub fn emc18011_setup_command() -> CommandDef {
    EMC18011.setup_command("EMC18011Setup")
}

/// `EMC18011Config(card, name)`.
pub fn emc18011_config_command() -> CommandDef {
    CommandDef::new(
        "EMC18011Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "EMC18011Config(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !EMC18011.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                EMC18011_TIMEOUT,
                b"\n",
                b"\r",
                EosRefusal::Ignored,
            )?;
            let controller =
                Emc18011Controller::new(handle).map_err(|e| format!("EMC18011Config: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let ax = Emc18011Axis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                EMC18011.install(ctx, card, index as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
