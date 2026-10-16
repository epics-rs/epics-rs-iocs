//! `PM304Setup` / `PM304Config` iocsh commands for the Mclennan PM304/PM600,
//! C's model-1 interface: records reach axis `signal` of `card` through
//! `DTYP "Mclennan PM304"` and `OUT "#C card S signal"`. Config connects and
//! identifies the controller at once, where C's `motor_init` does so at
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

use crate::pm304::{Pm304Axis, Pm304Controller};

/// Serial command timeout (C `TIMEOUT` 2 s).
const PM304_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devPM304,"Mclennan PM304")` (devMclennanMotor.dbd:2).
static DEV_PM304: DevSup = model1_device("devPM304", "Mclennan PM304");

/// `PM304_NUM_CARDS` (drvPM304.h:27).
static PM304: Model1 = Model1::new(&DEV_PM304, 4, SCAN_RATE);

/// `PM304Setup(num_cards, scan_rate)`.
pub fn pm304_setup_command() -> CommandDef {
    PM304.setup_command("PM304Setup")
}

/// `PM304Config(card, name, n_axes)`.
pub fn pm304_config_command() -> CommandDef {
    CommandDef::new(
        "PM304Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("Number of axes"),
        ],
        "PM304Config(card, name, n_axes)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if !PM304.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let mut n_axes = req_int(args, 2, "Number of axes")?;
            if n_axes == 0 {
                n_axes = 1; // C: older startup files omit it.
            }
            let n_axes = usize::try_from(n_axes).map_err(|_| "PM304Config: n_axes < 0")?;

            let handle = connect_serial(&asyn_port, PM304_TIMEOUT)?;
            let controller =
                Pm304Controller::new(handle, n_axes).map_err(|e| format!("PM304Config: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes {
                let ax = Pm304Axis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                PM304.install(ctx, card, index as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
