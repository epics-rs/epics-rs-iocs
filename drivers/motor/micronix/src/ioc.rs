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

use motor_common::connect::connect_controller;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, opt_int, poll_intervals, req_int, req_string,
};
use motor_common::{Controller, asyn_user, register_port_with};

use crate::mmc200::{Mmc200Axis, Mmc200Controller, Mmc200Params};

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

            let params = Mmc200Params {
                moving_poll: moving_poll_ms as f64 / 1000.0,
                idle_poll: idle_poll_ms as f64 / 1000.0,
                ignore_limits,
            };
            register_port_with(ctx, &motor_port, num_axes as i32, params)?;

            let (handle, _) = connect_controller(
                &serial_port,
                MMC200_TIMEOUT,
                &motor_port,
                format_args!(
                    "MMC200Controller::MMC200Controller: cannot connect to MMC-200 controller"
                ),
            );
            let controller = Arc::new(Mutex::new(Mmc200Controller::new(handle)));

            let poller = Controller::new();
            for index in 0..num_axes as usize {
                let ax = Mmc200Axis::new(controller.clone(), index, ignore_limits);
                let report = ax.report();
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                poller.install_with_report(asyn_user(&motor_port, index as i32), motor, report)?;
            }
            // C startPoller, MMC200Driver.cpp:62
            poller.start(&motor_port, moving_poll_ms, idle_poll_ms, 2);
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;

    /// `cmd` with `args` on a thread of its own, then the asynReport of
    /// `port` at `level`.
    async fn report_after(cmd: CommandDef, args: Vec<ArgValue>, port: &str, level: i32) -> String {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::base::runtime::task::{BlockingBridge, sleep};
        use epics_rs::base::server::database::PvDatabase;

        let bridge = BlockingBridge::capture();
        let port = port.to_string();
        let shell = std::thread::spawn(move || {
            let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
            cmd.handler.call(&args, &ctx).unwrap();
            motor_common::testing::await_first_polls();
            PortManager::global()
                .find_port_handle(&port)
                .unwrap()
                .report_blocking(level)
                .unwrap()
        });
        while !shell.is_finished() {
            sleep(Duration::from_millis(5)).await;
        }
        shell.join().unwrap()
    }

    /// An MMC-200 axis on line `line`, ignoring limits, reported at `level`.
    async fn mmc200_report(port: &str, level: i32) -> String {
        let line = format!("{port}Line");
        let (_rig, _io) = LineRig::new(&line, |cmd| {
            match cmd.trim_end() {
                "1VER?" => "#MMC-200 v4.1",
                "1FBK?" => "#0",
                "1REZ?" => "#200",
                "1UST?" => "#16",
                "1ENC?" => "#0.5",
                "1VMX?" => "#2.5",
                _ => "#0",
            }
            .into()
        });
        let args = vec![
            ArgValue::String(port.into()),
            ArgValue::String(line.into()),
            ArgValue::Int(1),
            ArgValue::Int(100),
            ArgValue::Int(1000),
            ArgValue::Int(1),
        ];
        report_after(mmc200_create_controller_command(), args, port, level).await
    }

    const DRIVER: &str = "MMC-200 motor driver\n  port name={port}\n  num axes=1\n  \
                          moving poll period=0.100000\n  idle poll period=1.000000\n  \
                          ignore limits=1\n";

    /// C `MMC200Controller::report`, then the base's, which runs each
    /// `MMC200Axis::report` (MMC200Driver.cpp:90-101, 281-298).
    #[epics_rs::base::epics_test]
    async fn the_mmc200_report_prints_the_driver_then_each_axis_as_c() {
        let text = mmc200_report("mmc200Report1", 1).await;
        let axis = "  axis 0\n  axis index 1\n  version #MMC-200 v4.1\n  model 200\n  \
                    feedback mode 0\n  rez 200\n  micro steps 16\n  \
                    encoder resolution 0.500000\n  resolution 0.000000\n  \
                    max velocity 2.500000\n";
        let want = DRIVER.replace("{port}", "mmc200Report1");
        assert!(text.contains(&want), "{text}");
        assert!(text.contains(axis), "{text}");
    }

    #[epics_rs::base::epics_test]
    async fn a_level_0_mmc200_report_prints_no_axis() {
        let text = mmc200_report("mmc200Report0", 0).await;
        assert!(
            text.contains(&DRIVER.replace("{port}", "mmc200Report0")),
            "{text}"
        );
        assert!(!text.contains("  axis index"), "{text}");
    }
}
