//! `MMC200CreateController` iocsh command for the Micronix MMC-100/200 family.
//!
//! Mirrors C `MMC200CreateController(portName, MMC200PortName, numAxes,
//! movingPollPeriod, idlePollPeriod, ignoreLimits)`: connect the serial octet
//! port, build one [`Mmc200Axis`] per axis, and register each behind a
//! `DTYP`-keyed motor device support (`MMC200_{motorPort}_{index}`, 0-based).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, opt_int, poll_intervals, req_int, req_string,
};

use crate::mmc200::{Mmc200Axis, Mmc200Controller};

/// Serial command timeout. The MMC controllers answer queries promptly; the C
/// driver uses the default asyn synchronous-I/O timeout.
const MMC200_TIMEOUT: Duration = Duration::from_secs(1);

/// Build the `MMC200CreateController` iocsh command bound to `holder`.
pub fn mmc200_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "MMC200CreateController",
        vec![
            arg_str_req("motorPort"),
            arg_str_req("serialPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
            arg_int_opt("ignoreLimits"),
        ],
        "MMC200CreateController(motorPort, serialPort, numAxes, [movingPollMs], [idlePollMs], [ignoreLimits]) - \
         Create a Micronix MMC-100/200 controller (DTYP MMC200_{motorPort}_{0..numAxes-1})",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let serial_port = req_string(args, 1, "serialPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes <= 0 {
                return Err("MMC200CreateController: numAxes must be positive".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let ignore_limits = opt_int(args, 5, 0, "ignoreLimits")? != 0;

            let handle = connect_serial(&serial_port, MMC200_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(Mmc200Controller::new(handle)));

            for index in 0..num_axes as usize {
                let ax = Mmc200Axis::new(controller.clone(), index, ignore_limits)
                    .map_err(|e| format!("MMC200CreateController: axis {index}: {e}"))?;
                let dtyp_key = format!("MMC200_{motor_port}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "MMC200CreateController: motorPort={motor_port} serialPort={serial_port} \
                 axes={num_axes} ignoreLimits={ignore_limits} poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=MMC200_{motor_port}_{{0..{}}})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
