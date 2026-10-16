//! iocsh command for the IMS MDrivePlus driver.
//!
//! Mirrors the C `ImsMDrivePlusCreateController(motorPort, ioPort, deviceName,
//! movingPoll, idlePoll)`: connects the asyn octet port, probes the single axis
//! (firmware version, output EOS, switch config, encoder flag) and creates
//! controller port `motorPort`, whose one axis records address as
//! `@asyn(motorPort,0)`. `deviceName` is empty for a non-party-mode drive.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use epics_rs::asyn::sync_io::SyncIOHandle;
use motor_common::connect::set_eos_ignored;
use motor_common::iocsh::{
    arg_double_opt, arg_str_opt, arg_str_req, opt_double, opt_string, req_string,
};
use motor_common::{asyn_user, install, register_port};

use crate::mdriveplus::{DRIVER_NAME, INPUT_EOS, ImsMDrivePlusAxis, OUTPUT_EOS};

/// C `IMS_TIMEOUT` (ImsMDrivePlusMotorAxis.h:17).
const IMS_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `ImsMDrivePlusCreateController(motorPortName, IOPortName, devName,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn ims_create_controller_command() -> CommandDef {
    CommandDef::new(
        "ImsMDrivePlusCreateController",
        vec![
            arg_str_req("Motor port name"),
            arg_str_req("IO port name"),
            arg_str_opt("Device name"),
            arg_double_opt("Moving poll period (ms)"),
            arg_double_opt("Idle poll period (ms)"),
        ],
        "ImsMDrivePlusCreateController(motorPortName, IOPortName, devName, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "Motor port name")?;
            let io_port = req_string(args, 1, "IO port name")?;
            let device_name = opt_string(args, 2)?.unwrap_or_default();
            let moving_poll_ms = opt_double(args, 3, 0.0, "Moving poll period (ms)")?;
            let idle_poll_ms = opt_double(args, 4, 0.0, "Idle poll period (ms)")?;

            register_port(ctx, &motor_port, 1)?;
            // C prints and carries on with the user of a missing port
            // (ImsMDrivePlusMotorController.cpp:69-74).
            let (handle, connected) = SyncIOHandle::connect_port(&io_port, 0, IMS_TIMEOUT);
            if connected.is_err() {
                println!(
                    "\n\n{DRIVER_NAME}:ImsMDrivePlusMotorController(): ERROR connecting to \
                     Controller's IO port={io_port}\n"
                );
            }
            println!(
                "{DRIVER_NAME}:ImsMDrivePlusMotorController(): motorPortName={motor_port}, \
                 IOPortName={io_port}, devName={device_name} "
            );
            set_eos_ignored(&handle, INPUT_EOS, OUTPUT_EOS);
            let axis = ImsMDrivePlusAxis::new(handle, &motor_port, device_name);

            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            install(
                ctx,
                asyn_user(&motor_port, 0),
                motor,
                moving_poll_ms as u64,
                idle_poll_ms as u64,
            )?;

            Ok(CommandOutcome::Continue)
        },
    )
}
