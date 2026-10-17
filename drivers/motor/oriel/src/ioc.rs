//! `EMC18011Setup` / `EMC18011Config` iocsh commands for the Oriel Encoder
//! Mike 18011, C's model-1 interface: records reach axis `signal` of `card`
//! through `DTYP "EMC18011"` and `OUT "#C card S signal"`.
//!
//! Config keeps the port; `motor_init` at iocInit connects and identifies
//! the controller, which has a fixed three axes.

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
    EMC18011.register_report("drvEMC18011", |cards| {
        cards.report("EMC18011", |card, line| {
            format!("    EMC18011 controller {card}, {line} \n")
        })
    });
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

            EMC18011.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    // C connects at an asyn_address EMC18011Config never sets (drvEMC18011.cc:661).
                    0,
                    EMC18011_TIMEOUT,
                    b"\n",
                    b"\r",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = Emc18011Controller::new(handle) else {
                    return;
                };
                let num_axes = controller.num_axes();
                // C prints `cntrl->asyn_address`, which EMC18011Config never sets
                // (drvEMC18011.cc:605-619), a read of uninitialized memory not
                // copied; this is the address the line is opened at.
                let line = format!("port={asyn_port}, address=0, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));
                for index in 0..num_axes {
                    let ax = Emc18011Axis::new(controller.clone(), index);
                    let build_trans = ax.build_trans();
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    init.axis_with_build_trans(index as i32, motor, build_trans);
                }
                init.describe(line);
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod report {
    use super::*;
    use motor_common::testing::{LineRig, driver_report, drvsup_init, run_command};

    /// drvEMC18011.cc's drvet `report` (drvEMC18011.cc:153-180), which
    /// `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_emc18011_card() {
        let setup = emc18011_setup_command();
        assert_eq!(
            driver_report("drvEMC18011", 0),
            "    No EMC18011 controllers configured.\n"
        );
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("emcRepLine", |cmd| match cmd {
            "R" => "ON LINE".into(),
            _ => String::new(),
        });
        let config = vec![int(1), ArgValue::String("emcRepLine".into())];
        run_command(emc18011_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvEMC18011", 0),
            "    EMC18011 controller 0 connection failed.\n    EMC18011 controller 1, \
             port=emcRepLine, address=0, id: Oriel Encoder Mike 18011 \n"
        );
    }
}
