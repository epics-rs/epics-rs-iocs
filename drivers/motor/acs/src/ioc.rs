//! `MCB4BCreateController` (MCB4BDriver.cpp) and `MCB4BSetup` /
//! `MCB4BConfig` (drvMCB4B.cc) iocsh commands for the ACS MCB-4B.
//!
//! C `MCB4BCreateController(portName, MCB4BPortName, numAxes,
//! movingPollPeriod, idlePollPeriod)` from `MCB4BDriver.cpp`: connect the serial
//! octet port and create controller port `portName` with `numAxes` axes,
//! addressed by records as `@asyn(portName,axis)`.
//!
//! The C constructor performs no identification I/O (it only connects and starts
//! the poller), so this command connects and installs axes without probing.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{connect_controller, connect_serial};
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};
use motor_common::model1::{Model1, Model1Cards, SCAN_RATE, c_int};
use motor_common::{Controller, asyn_user, model1_device, register_port_with};

use crate::drv_mcb4b::{DrvMcb4bAxis, MCB4B_NUM_AXES, MCB4B_NUM_CARDS, motor_init};
use crate::mcb4b::{Mcb4bAxis, Mcb4bAxisReport, Mcb4bController, Mcb4bParams};

/// Serial command timeout (C `MCB4BDriver` uses the asyn port default; the other
/// ACS/motor ports use 2 s, matched here).
const MCB4B_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `MCB4BCreateController(portName, MCB4BPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command.
pub fn mcb4b_create_controller_command() -> CommandDef {
    CommandDef::new(
        "MCB4BCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("MCB-4B port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "MCB4BCreateController(portName, MCB4BPortName, numAxes, movingPollPeriod, idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let asyn_port = req_string(args, 1, "MCB-4B port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("MCB4BCreateController: numAxes must be > 0".into());
            }
            let num_axes = num_axes as usize;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let params = Mcb4bParams {
                moving_poll: moving_poll_ms as f64 / 1000.0,
                idle_poll: idle_poll_ms as f64 / 1000.0,
            };
            register_port_with(ctx, &port_name, num_axes as i32, params)?;

            let (handle, _) = connect_controller(
                &asyn_port,
                MCB4B_TIMEOUT,
                &port_name,
                format_args!(
                    "MCB4BController::MCB4BController: cannot connect to MCB-4B controller"
                ),
            );
            let controller = Arc::new(Mutex::new(Mcb4bController::new(handle, num_axes)));

            // C startPoller, MCB4BDriver.cpp:58
            let poller = Controller::new();
            for index in 0..num_axes {
                let ax = Mcb4bAxis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                let report = Arc::new(Mcb4bAxisReport(index));
                poller.install_with_report(asyn_user(&port_name, index as i32), motor, report)?;
            }
            poller.start(&port_name, moving_poll_ms, idle_poll_ms, 2);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// drvMCB4B.cc's `TIMEOUT`.
const DRV_MCB4B_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devMCB4B,"ACS MCB-4B")` (devAcsMotor.dbd:4).
static DEV_MCB4B: DevSup = model1_device("devMCB4B", "ACS MCB-4B");

static MCB4B: Model1 = Model1::new(&DEV_MCB4B, MCB4B_NUM_CARDS, SCAN_RATE);

/// `MCB4BSetup(num_cards, scan_rate)`.
pub fn mcb4b_setup_command() -> CommandDef {
    MCB4B.register_report("drvMCB4B", mcb4b_report);
    MCB4B.setup_command("MCB4BSetup")
}

/// drvMCB4B.cc's drvet `report` (:123-140): only the cards that answered.
fn mcb4b_report(cards: &Model1Cards<'_>) -> String {
    if cards.num_cards() <= 0 {
        return "    NO MCB4B controllers found\n".into();
    }
    (0..cards.num_cards())
        .filter_map(|card| {
            let line = cards.card(card)?;
            Some(format!("    MCB4B controller {card}, {line} \n"))
        })
        .collect()
}

/// `MCB4BConfig(card, name)`. `motor_init` at iocInit connects and probes
/// the card; a card that never answers is left unconfigured.
pub fn mcb4b_config_command() -> CommandDef {
    CommandDef::new(
        "MCB4BConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "MCB4BConfig(card, name)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !MCB4B.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            MCB4B.config(card, move |init| {
                let Ok(handle) = connect_serial(&asyn_port, 0, DRV_MCB4B_TIMEOUT) else {
                    return;
                };
                let controller = Mcb4bController::new(handle, MCB4B_NUM_AXES);
                if !motor_init(&controller) {
                    return;
                }
                let controller = Arc::new(Mutex::new(controller));
                for index in 0..MCB4B_NUM_AXES {
                    let motor: Arc<Mutex<dyn AsynMotor>> =
                        Arc::new(Mutex::new(DrvMcb4bAxis::new(controller.clone(), index)));
                    init.axis(index as i32, motor);
                }
                // C's ident is the constant motor_init copies (drvMCB4B.cc:501).
                init.describe(format!("port={asyn_port}, id: MCB-4B"));
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod report {
    use super::*;
    use motor_common::testing::{LineRig, driver_report, drvsup_init, run_command};

    /// drvMCB4B.cc's drvet `report` (:123-140), which `dbior` runs; C
    /// prints nothing for a card that never answered.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_answering_mcb4b_card() {
        let setup = mcb4b_setup_command();
        assert_eq!(
            driver_report("drvMCB4B", 0),
            "    NO MCB4B controllers found\n"
        );
        run_command(setup, vec![ArgValue::Int(3), ArgValue::Int(10)]).await;
        let (_on, _io_on) = LineRig::new("mcbRepLine", |_| "OK".into());
        let (_off, _io_off) = LineRig::new("mcbRepSilent", |_| String::new());
        for (card, line) in [(1, "mcbRepLine"), (2, "mcbRepSilent")] {
            let args = vec![ArgValue::Int(card), ArgValue::String(line.into())];
            run_command(mcb4b_config_command(), args).await;
        }
        drvsup_init().await;
        assert_eq!(
            driver_report("drvMCB4B", 0),
            "    MCB4B controller 1, port=mcbRepLine, id: MCB-4B \n"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `cmd` with `args` on a thread of its own, then the asynReport of
    /// `port` at `level`.
    pub(crate) async fn report_after(
        cmd: CommandDef,
        args: Vec<ArgValue>,
        port: &str,
        level: i32,
    ) -> String {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::base::runtime::task::{BlockingBridge, sleep};
        use epics_rs::base::server::database::PvDatabase;

        let bridge = BlockingBridge::capture();
        let port = port.to_string();
        let shell = std::thread::spawn(move || {
            let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
            cmd.handler.call(&args, &ctx).unwrap();
            motor_common::testing::await_first_polls(&port);
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

    fn create_args(port: &str) -> Vec<ArgValue> {
        vec![
            ArgValue::String(port.into()),
            ArgValue::String("mcb4bNoLine".into()),
            ArgValue::Int(2),
            ArgValue::Int(100),
            ArgValue::Int(1000),
        ]
    }

    /// C `MCB4BController::report`, then the base's, which runs each
    /// `MCB4BAxis::report` (MCB4BDriver.cpp:95-117).
    #[epics_rs::base::epics_test]
    async fn the_port_report_prints_the_driver_line_then_each_axis_as_c() {
        let cmd = mcb4b_create_controller_command();
        let text = report_after(cmd, create_args("mcb4bReport1"), "mcb4bReport1", 1).await;
        let want = "MCB-4B motor driver mcb4bReport1, numAxes=2, \
                    moving poll period=0.100000, idle poll period=1.000000\n  axis 0\n  axis 1\n";
        assert!(text.contains(want), "{text}");
    }

    /// At level 0 C's axis prints nothing.
    #[epics_rs::base::epics_test]
    async fn a_level_0_report_prints_no_axis_line() {
        let cmd = mcb4b_create_controller_command();
        let text = report_after(cmd, create_args("mcb4bReport0"), "mcb4bReport0", 0).await;
        assert!(
            text.contains("MCB-4B motor driver mcb4bReport0, numAxes=2, "),
            "{text}"
        );
        assert!(!text.contains("  axis "), "{text}");
    }
}
