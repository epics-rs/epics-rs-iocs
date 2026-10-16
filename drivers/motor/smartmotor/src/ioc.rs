//! `SmartMotorSetup` / `SmartMotorConfig` iocsh commands for the Animatics
//! SmartMotor.
//!
//! Mirrors C `SmartMotorConfig(card, port)`: look up the serial octet port,
//! count the motors on it ([`SmartMotorController::new`]), and register each
//! behind a `DTYP`-keyed motor device support (`SMARTMOTOR_{card}_{axis}`).
//! `SmartMotorSetup` (which only sized a controller array in C) is accepted as a
//! no-op for startup-script parity.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::smartmotor::{SmartMotorAxis, SmartMotorController};

/// Serial command timeout (C `COMM_TIMEOUT` 2 s).
const SMARTMOTOR_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `SmartMotorSetup(maxControllers, [scanRate])` no-op command
/// (startup-script parity; the asyn-rs port allocates per `SmartMotorConfig`).
pub fn smartmotor_setup_command() -> CommandDef {
    CommandDef::new(
        "SmartMotorSetup",
        vec![arg_int_req("maxControllers"), arg_int_opt("scanRate")],
        "SmartMotorSetup(maxControllers, [scanRate]) - Accepted for parity; SmartMotorConfig allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("SmartMotorSetup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `SmartMotorConfig(card, asynPort, [movingPollMs], [idlePollMs])`
/// command bound to `holder`.
pub fn smartmotor_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "SmartMotorConfig",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "SmartMotorConfig(card, asynPort, [movingPollMs], [idlePollMs]) - \
         Create one Animatics SmartMotor or a daisy chain of them \
         (DTYP SMARTMOTOR_{card}_{axis})",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("SmartMotorConfig: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;

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
                let dtyp_key = format!("SMARTMOTOR_{card}_{axis}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }

            println!(
                "SmartMotorConfig: card={card} asynPort={asyn_port} axes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=SMARTMOTOR_{card}_0..)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
