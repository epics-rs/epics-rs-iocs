//! `PIJEDSSetup` / `PIJEDSConfig` iocsh commands for the piezosystem jena
//! E-516 controller, C's model-1 interface: records reach axis `signal` of
//! `card` through `DTYP "PIJEDS"` and `OUT "#C card S signal"`. Config
//! keeps the port; `motor_init` at iocInit connects and identifies the
//! controller.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
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
    PIJEDS.register_report("drvPIJEDS", |cards| {
        cards.report("PIJEDS", |card, line| {
            format!("    PIJEDS controller #{card}, {line} \n")
        })
    });
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
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIJEDS.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            PIJEDS.config(card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIJEDS_TIMEOUT,
                    b"\x11",
                    b"\r",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PiJedsController::new(handle) else {
                    return;
                };
                // A card with no stage stays (drvPIJEDS.cc:650-675).
                let num_axes = controller.num_axes();
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));
                for axis in 0..num_axes {
                    let ax = PiJedsAxis::new(controller.clone(), axis);
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    init.axis(axis as i32, motor);
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

    /// drvPIJEDS.cc's drvet `report` (drvPIJEDS.cc:151-176), which `dbior`
    /// runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pijeds_controller() {
        let setup = pijeds_setup_command();
        assert_eq!(
            driver_report("drvPIJEDS", 0),
            "    No PIJEDS controllers configured.\n"
        );
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("jedsRepLine", |cmd| {
            match cmd.trim() {
                "" => "DSM V1.959",
                "mess,0" => "mess,0,1.000",
                _ => "not present",
            }
            .into()
        });
        let config = vec![int(1), ArgValue::String("jedsRepLine".into()), int(0)];
        run_command(pijeds_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvPIJEDS", 0),
            "    PIJEDS controller 0 connection failed.\n    PIJEDS controller #1, \
             port=jedsRepLine, id: DSM V1.959 \n"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::{LineRig, driver_report, drvsup_init, run_command};

    /// drvPIJEDS.cc:643-678: once `DSM` answered, only a `not present`
    /// reply ends the axis count, and a card with no axes stays.
    #[epics_rs::base::epics_test]
    async fn a_card_answering_dsm_survives_its_axis_count() {
        let int = ArgValue::Int;
        run_command(pijeds_setup_command(), vec![int(2), int(10)]).await;
        let rig = |name: &str, axis0: &'static str| {
            LineRig::new(name, move |cmd| match cmd.trim_end() {
                "" => "DSM V1.959".into(),
                "mess,0" => axis0.into(),
                _ => String::new(),
            })
        };
        let (_a, _) = rig("edsSilentMess", "");
        let (_b, _) = rig("edsNoStage", "mess,0,not present");
        for (card, port) in [(0, "edsSilentMess"), (1, "edsNoStage")] {
            let config = vec![int(card), ArgValue::String(port.into()), int(0)];
            run_command(pijeds_config_command(), config).await;
        }
        drvsup_init().await;
        assert_eq!(
            driver_report("drvPIJEDS", 0),
            "    PIJEDS controller #0, port=edsSilentMess, id: DSM V1.959 \n    \
             PIJEDS controller #1, port=edsNoStage, id: DSM V1.959 \n"
        );
    }

    /// drvPIJEDS.cc:461-487: a failed `mess,n` reads as an empty reply,
    /// which is no `not present`, so the axis counts.
    #[epics_rs::base::epics_test]
    async fn a_refused_axis_query_still_counts_the_axis() {
        let (_line, io) = LineRig::failing(
            "edsRefusedMess",
            |cmd| match cmd.trim_end() {
                "" => "DSM V1.959".into(),
                _ => "mess,0,not present".into(),
            },
            |cmd| cmd.starts_with("mess,0"),
        );
        let controller = PiJedsController::new(io).unwrap();
        assert_eq!(controller.num_axes(), 1);
    }
}
