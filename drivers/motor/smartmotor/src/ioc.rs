//! `SmartMotorSetup` / `SmartMotorConfig` iocsh commands for the Animatics
//! SmartMotor, C's model-1 interface: records reach axis `signal` of `card`
//! through `DTYP "SmartMotor"` and `OUT "#C card S signal"`. Config connects
//! and identifies the controller at once, where C's `motor_init` does so at
//! iocInit.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::iocsh::{arg_int_req, arg_str_req, req_int, req_string};
use motor_common::model1::{Model1, SCAN_RATE};
use motor_common::model1_device;

use crate::smartmotor::{SmartMotorAxis, SmartMotorController};

/// Serial command timeout (C `COMM_TIMEOUT` 2 s).
const SMARTMOTOR_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devSmartMotor,"SmartMotor")` (devSmartMotorMotor.dbd:3).
static DEV_SMART_MOTOR: DevSup = model1_device("devSmartMotor", "SmartMotor");

/// `SmartMotor_NUM_CARDS` (drvSmartMotor.cc:57).
static SMART_MOTOR: Model1 = Model1::new(&DEV_SMART_MOTOR, 7, SCAN_RATE);

/// `SmartMotorSetup(num_cards, scan_rate)`.
pub fn smartmotor_setup_command() -> CommandDef {
    SMART_MOTOR.setup_command("SmartMotorSetup")
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
            let card = req_int(args, 0, "card")?;
            if !SMART_MOTOR.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let port = get_port(&asyn_port).ok_or_else(|| {
                format!(
                    "serial port '{asyn_port}' not found (call drvAsynSerialPortConfigure first)"
                )
            })?;
            let controller = SmartMotorController::new(port.handle.clone(), SMARTMOTOR_TIMEOUT)
                .map_err(|e| format!("SmartMotorConfig: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                let ax = SmartMotorAxis::new(controller.clone(), axis);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                SMART_MOTOR.install(ctx, card, axis as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
