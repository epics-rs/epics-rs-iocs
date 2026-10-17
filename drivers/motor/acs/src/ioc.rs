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
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::{Controller, asyn_user, model1_device, register_port};

use crate::drv_mcb4b::{DrvMcb4bAxis, MCB4B_NUM_AXES, MCB4B_NUM_CARDS, motor_init};
use crate::mcb4b::{Mcb4bAxis, Mcb4bController};

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

            register_port(ctx, &port_name, num_axes as i32)?;

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
            let mut poller = Controller::new(moving_poll_ms, idle_poll_ms, 2);
            for index in 0..num_axes {
                let ax = Mcb4bAxis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                poller.install(asyn_user(&port_name, index as i32), motor)?;
            }
            poller.start(ctx);
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
    MCB4B.setup_command("MCB4BSetup")
}

/// `MCB4BConfig(card, name)`. Connects and probes the card at once, where
/// C's `motor_init` does so at iocInit; a card that never answers is left
/// unconfigured, as there.
pub fn mcb4b_config_command() -> CommandDef {
    CommandDef::new(
        "MCB4BConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "MCB4BConfig(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !MCB4B.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let handle = connect_serial(&asyn_port, DRV_MCB4B_TIMEOUT)?;
            let controller = Mcb4bController::new(handle, MCB4B_NUM_AXES);
            if !motor_init(&controller) {
                return Ok(CommandOutcome::Continue);
            }
            let controller = Arc::new(Mutex::new(controller));
            for index in 0..MCB4B_NUM_AXES {
                let motor: Arc<Mutex<dyn AsynMotor>> =
                    Arc::new(Mutex::new(DrvMcb4bAxis::new(controller.clone(), index)));
                MCB4B.install(ctx, card, index as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
