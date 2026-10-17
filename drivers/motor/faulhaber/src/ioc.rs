//! `MCDC2805Setup` / `MCDC2805Config` iocsh commands for the Faulhaber
//! MCDC2805, C's model-1 interface: records reach axis `signal` of `card`
//! through `DTYP "MCDC2805"` and `OUT "#C card S signal"`. Config keeps
//! the port; `motor_init` at iocInit connects and identifies the controller.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::connect_serial;
use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::faulhaber::{FaulhaberAxis, FaulhaberController};

/// Serial command timeout (C `COMM_TIMEOUT` 2 s).
const MCDC2805_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devMCDC2805,"MCDC2805")` (devFaulhaberMotor.dbd:2).
static DEV_MCDC2805: DevSup = model1_device("devMCDC2805", "MCDC2805");

/// `MCDC2805_NUM_CARDS` (drvMCDC2805.cc:56).
static MCDC2805: Model1 = Model1::new(&DEV_MCDC2805, 8, SCAN_RATE);

/// `MCDC2805Setup(num_cards, scan_rate)`.
pub fn mcdc2805_setup_command() -> CommandDef {
    MCDC2805.register_report("drvMCDC2805", |cards| {
        cards.report("MCDC2805", |card, line| {
            format!("    MCDC2805SM controller #{card}, {line} \n")
        })
    });
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
            let card = c_int(args, 0);
            if !MCDC2805.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let num_motors = c_int(args, 1);
            if !(1..=8).contains(&num_motors) {
                return Err("MCDC2805Config: numMotors must be 1..=8".into());
            }
            let asyn_port = req_string(args, 2, "asyn port name")?;

            MCDC2805.config(ctx, card, move |init| {
                let Ok(handle) = connect_serial(&asyn_port, 0, MCDC2805_TIMEOUT) else {
                    return;
                };
                let Ok(controller) = FaulhaberController::new(handle, num_motors as usize) else {
                    return;
                };
                let num_axes = controller.num_axes();
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));
                for node in 0..num_axes {
                    let Ok(ax) = FaulhaberAxis::new(controller.clone(), node) else {
                        return;
                    };
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    init.axis(node as i32, motor);
                }
                if num_axes > 0 {
                    init.describe(line);
                }
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod report {
    use super::*;
    use motor_common::testing::{LineRig, driver_report, drvsup_init, run_command};

    /// drvMCDC2805.cc's drvet `report` (drvMCDC2805.cc:129-153), which
    /// `dbior` runs; C's per-card line says MCDC2805SM.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_mcdc2805_chain() {
        let setup = mcdc2805_setup_command();
        assert_eq!(
            driver_report("drvMCDC2805", 0),
            "    No MCDC2805 controllers configured.\n"
        );
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("mcdcRepLine", |cmd| {
            if cmd.ends_with("VER") {
                "MCDC 2805"
            } else {
                "0"
            }
            .into()
        });
        let config = vec![int(1), int(1), ArgValue::String("mcdcRepLine".into())];
        run_command(mcdc2805_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvMCDC2805", 0),
            "    MCDC2805 controller 0 connection failed.\n    MCDC2805SM controller #1, \
             port=mcdcRepLine, id: MCDC 2805 \n"
        );
    }
}
