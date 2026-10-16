//! `MMC200CreateController` iocsh command for the Micronix MMC-100/200 family.
//!
//! Mirrors C `MMC200CreateController(portName, MMC200PortName, numAxes,
//! movingPollPeriod, idlePollPeriod, ignoreLimits)`: connect the serial octet
//! port and create controller port `portName` with one [`Mmc200Axis`] per
//! axis, addressed by records as `@asyn(portName,axis)`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, opt_int, poll_intervals, req_int, req_string,
};
use motor_common::{asyn_user, install, register_port};

use crate::mmc200::{Mmc200Axis, Mmc200Controller};

/// Serial command timeout. The MMC controllers answer queries promptly; the C
/// driver uses the default asyn synchronous-I/O timeout.
const MMC200_TIMEOUT: Duration = Duration::from_secs(1);

/// Build the `MMC200CreateController(portName, MMC200PortName, numAxes,
/// movingPollPeriod, idlePollPeriod, ignoreLimits)` command, periods in ms.
pub fn mmc200_create_controller_command() -> CommandDef {
    CommandDef::new(
        "MMC200CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("MMC-200 port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
            arg_int_opt("Ignore limit flag"),
        ],
        "MMC200CreateController(portName, MMC200PortName, numAxes, movingPollPeriod, \
         idlePollPeriod, ignoreLimits)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "Port name")?;
            let serial_port = req_string(args, 1, "MMC-200 port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes <= 0 {
                return Err("MMC200CreateController: numAxes must be positive".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let ignore_limits = opt_int(args, 5, 0, "Ignore limit flag")? != 0;

            let handle = connect_serial(&serial_port, MMC200_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(Mmc200Controller::new(handle)));

            register_port(ctx, &motor_port, num_axes as i32)?;
            for index in 0..num_axes as usize {
                let ax = Mmc200Axis::new(controller.clone(), index, ignore_limits)
                    .map_err(|e| format!("MMC200CreateController: axis {index}: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                install(
                    ctx,
                    asyn_user(&motor_port, index as i32),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                )?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
