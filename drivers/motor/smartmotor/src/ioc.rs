//! `SmartMotorSetup` / `SmartMotorConfig` iocsh commands for the Animatics
//! SmartMotor, C's model-1 interface: records reach axis `signal` of `card`
//! through `DTYP "SmartMotor"` and `OUT "#C card S signal"`. Config keeps
//! the port; `motor_init` at iocInit connects and identifies the controller.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, Model1Cards, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::smartmotor::{SmartMotorAxis, SmartMotorController};

/// Serial command timeout (C `COMM_TIMEOUT` 2 s).
const SMARTMOTOR_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devSmartMotor,"SmartMotor")` (devSmartMotorMotor.dbd:3).
static DEV_SMART_MOTOR: DevSup = model1_device("devSmartMotor", "SmartMotor");

/// `SmartMotor_NUM_CARDS` (drvSmartMotor.cc:57).
static SMART_MOTOR: Model1 = Model1::new(&DEV_SMART_MOTOR, 7, SCAN_RATE);

/// `SmartMotorSetup(num_cards, scan_rate)`, with the `drvSmartMotor` drvet
/// report its dbd line registers.
pub fn smartmotor_setup_command() -> CommandDef {
    SMART_MOTOR.register_report("drvSmartMotor", report);
    SMART_MOTOR.setup_command("SmartMotorSetup")
}

/// C `report` (drvSmartMotor.cc:143-166). C's `ident` is never filled, so
/// the id is always empty.
fn report(cards: &Model1Cards<'_>) -> String {
    cards.report("SmartMotor", |card, port| {
        format!("    SmartMotorSM controller #{card}, port={port}, id:  \n")
    })
}

/// `SmartMotorConfig(card, name)`.
pub fn smartmotor_config_command() -> CommandDef {
    CommandDef::new(
        "SmartMotorConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "SmartMotorConfig(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !SMART_MOTOR.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            SMART_MOTOR.config(ctx, card, move |init| {
                let Some(port) = get_port(&asyn_port) else {
                    return;
                };
                let Ok(controller) =
                    SmartMotorController::new(port.handle.clone(), SMARTMOTOR_TIMEOUT)
                else {
                    return;
                };
                let num_axes = controller.num_axes();
                let controller = Arc::new(Mutex::new(controller));
                for axis in 0..num_axes {
                    let ax = SmartMotorAxis::new(controller.clone(), axis);
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    init.axis(axis as i32, motor);
                }
                init.describe(asyn_port);
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(all(test, target_os = "linux"))]
mod report {
    use epics_rs::base::server::driver_support::find_driver_support;
    use motor_common::testing::{LineRig, drvsup_init, run_command};

    use super::*;

    /// C drvSmartMotor.cc:143-166, the drvet report `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_smartmotor_card() {
        let setup = smartmotor_setup_command();
        let drvet = find_driver_support("drvSmartMotor").unwrap();
        assert_eq!(
            drvet.report(0).unwrap(),
            "    No SmartMotor controllers configured.\n"
        );
        run_command(setup, vec![ArgValue::Int(2), ArgValue::Int(10)]).await;
        let (_line, _io) = LineRig::new("smReportLine", |_| "0".into());
        let config = smartmotor_config_command();
        let args = vec![ArgValue::Int(1), ArgValue::String("smReportLine".into())];
        run_command(config, args).await;
        drvsup_init().await;
        assert_eq!(
            drvet.report(0).unwrap(),
            "    SmartMotor controller 0 connection failed.\n    \
             SmartMotorSM controller #1, port=smReportLine, id:  \n"
        );
    }
}
