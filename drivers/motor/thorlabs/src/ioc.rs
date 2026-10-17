//! `MDT695Setup` / `MDT695Config` iocsh commands for the ThorLabs
//! MDT693/694/695 piezo controller, C's model-1 interface: records reach axis
//! `signal` of `card` through `DTYP "MDT695"` and `OUT "#C card S signal"`.
//! Config keeps the port; `motor_init` at iocInit connects and identifies
//! the controller.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::mdt695::{INPUT_EOS, Mdt695Axis, Mdt695Build, Mdt695Controller, OUTPUT_EOS};

/// Serial command timeout (C `TIMEOUT` 2 s).
const MDT695_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devMDT695,"MDT695")` (devThorLabs.dbd:2).
static DEV_MDT695: DevSup = model1_device("devMDT695", "MDT695");

/// `MDT695_NUM_CARDS` (drvMDT695.cc:61).
static MDT695: Model1 = Model1::new(&DEV_MDT695, 16, SCAN_RATE);

/// `MDT695Setup(num_cards, scan_rate)`.
pub fn mdt695_setup_command() -> CommandDef {
    MDT695.register_report("drvMDT695", |cards| {
        cards.report("MDT695", |card, line| {
            format!("    MDT695 controller {card}, {line} \n")
        })
    });
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
            let card = c_int(args, 0);
            if !MDT695.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            MDT695.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    // C connects at an asyn_address MDT695Config never sets (drvMDT695.cc:548).
                    0,
                    MDT695_TIMEOUT,
                    INPUT_EOS,
                    OUTPUT_EOS,
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = Mdt695Controller::new(handle) else {
                    return;
                };
                let num_axes = controller.num_axes();
                // C prints `cntrl->asyn_address`, which MDT695Config never sets
                // (drvMDT695.cc:495-509), a read of uninitialized memory not
                // copied; this is the address the line is opened at.
                let line = format!("port={asyn_port}, address=0, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));
                for index in 0..num_axes {
                    let ax = Mdt695Axis::new(controller.clone(), index);
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    init.axis_with_build_trans(index as i32, motor, Arc::new(Mdt695Build));
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

    /// drvMDT695.cc's drvet `report` (drvMDT695.cc:145-172), which `dbior`
    /// runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_mdt695_card() {
        let setup = mdt695_setup_command();
        assert_eq!(
            driver_report("drvMDT695", 0),
            "    No MDT695 controllers configured.\n"
        );
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::queued("mdtRepLine", |cmd| match cmd.trim_end() {
            "D" => vec!["MDT694".into()],
            _ => Vec::new(),
        });
        let config = vec![int(1), ArgValue::String("mdtRepLine".into())];
        run_command(mdt695_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvMDT695", 0),
            "    MDT695 controller 0 connection failed.\n    MDT695 controller 1, \
             port=mdtRepLine, address=0, id:  \n"
        );
    }
}
