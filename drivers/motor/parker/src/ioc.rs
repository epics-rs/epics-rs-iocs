//! `OEMCreateController` iocsh command for the Parker OEM750 series.
//!
//! Mirrors C `OEMCreateController(portName, OEMPortName, numAxes,
//! movingPollPeriod, idlePollPeriod)`: connect the asyn octet port and create
//! controller port `portName` whose `numAxes` axes (unit address = axis + 1)
//! records address as `@asyn(portName,axis)`; `ACRCreateController` likewise.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_controller, connect_with_eos, set_eos_ignored};
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_opt, arg_str_req, opt_string, poll_intervals, req_int,
    req_string,
};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::{Controller, asyn_user, model1_device, register_controller_port};

use crate::acr::{AcrAxis, AcrController, AcrParams};
use crate::drv_pc6k::{self, DrvPc6kAxis, PC6K_NUM_CARDS, Pc6kCard};
use crate::oem::{OemAxis, OemController, OemParams};

/// Command timeout: asynMotorController's `DEFAULT_CONTROLLER_TIMEOUT`.
const OEM_TIMEOUT: Duration = Duration::from_secs(2);

/// ACR command timeout: asynMotorController's `DEFAULT_CONTROLLER_TIMEOUT`.
const ACR_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `OEMCreateController(portName, OEMPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn oem_create_controller_command() -> CommandDef {
    CommandDef::new(
        "OEMCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("OEM port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "OEMCreateController(portName, OEMPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let oem_port = req_string(args, 1, "OEM port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("OEMCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let (handle, _) = connect_controller(
                &oem_port,
                OEM_TIMEOUT,
                &port_name,
                format_args!(
                    "OEMMotorDriver:OEMController: cannot connect to OEM series controller"
                ),
            );
            set_eos_ignored(&handle, b"\r", b"\r");
            let controller = Arc::new(Mutex::new(OemController::new(handle, num_axes as usize)));
            register_controller_port(
                ctx.db().scan_control().interrupt_gate(),
                &port_name,
                num_axes as i32,
                OemParams::new(controller.clone()),
            )?;

            let poller = Controller::new();
            for index in 0..num_axes as usize {
                let ax = OemAxis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                poller.install(asyn_user(&port_name, index as i32), motor)?;
            }
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 2); // OEMMotorDriver.cpp:68
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `ACRCreateController(portName, ACRPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn acr_create_controller_command() -> CommandDef {
    CommandDef::new(
        "ACRCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("ACR port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "ACRCreateController(portName, ACRPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let acr_port = req_string(args, 1, "ACR port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("ACRCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let (handle, _) = connect_controller(
                &acr_port,
                ACR_TIMEOUT,
                &port_name,
                format_args!("ACRMotorDriver:ACRController: cannot connect to ACR controller"),
            );
            let controller = Arc::new(Mutex::new(AcrController::new(handle)));
            register_controller_port(
                ctx.db().scan_control().interrupt_gate(),
                &port_name,
                num_axes as i32,
                AcrParams::new(controller.clone()),
            )?;

            let poller = Controller::new();
            for index in 0..num_axes as usize {
                let ax = AcrAxis::new(controller.clone(), index);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                poller.install(asyn_user(&port_name, index as i32), motor)?;
            }
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 2); // ACRMotorDriver.cpp:82
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPC6K,"PC6K")` (devPC6K.dbd).
static DEV_PC6K: DevSup = model1_device("devPC6K", "PC6K");

static PC6K: Model1 = Model1::new(&DEV_PC6K, PC6K_NUM_CARDS, SCAN_RATE);

/// The cards `PC6KConfig` found, for `PC6KUpLoad`.
static PC6K_CARDS: Mutex<BTreeMap<i32, Arc<Mutex<Pc6kCard>>>> = Mutex::new(BTreeMap::new());

/// `PC6KSetup(num_cards, scan_rate)`.
pub fn pc6k_setup_command() -> CommandDef {
    PC6K.setup_command("PC6KSetup")
}

/// `PC6KConfig(card, name)`. Connects and probes the card at once, where
/// C's `motor_init` does so at iocInit; a card that does not answer as a
/// 6K is left unconfigured, as there.
pub fn pc6k_config_command() -> CommandDef {
    CommandDef::new(
        "PC6KConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "PC6KConfig(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PC6K.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let handle = connect_with_eos(
                &asyn_port,
                drv_pc6k::TIMEOUT,
                b">",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller = Pc6kCard::new(handle);
            let Some(axes) = drv_pc6k::motor_init(&controller) else {
                return Ok(CommandOutcome::Continue);
            };
            let controller = Arc::new(Mutex::new(controller));
            for (signal, limits) in axes.into_iter().enumerate() {
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(DrvPc6kAxis::new(
                    controller.clone(),
                    signal,
                    limits,
                )));
                PC6K.install(ctx, card, signal as i32, motor)?;
            }
            PC6K_CARDS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(card, controller);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `PC6KUpLoad(card, file, progName)`: the file's lines sent to the card,
/// wrapped in `DEL`/`DEF progName` ... `END` when a program is named. C
/// refuses every card before iocInit, as only `motor_init` counts them;
/// here a card is there once `PC6KConfig` found it.
pub fn pc6k_upload_command() -> CommandDef {
    CommandDef::new(
        "PC6KUpLoad",
        vec![
            arg_int_req("Controller Card#"),
            arg_str_req("Upload File Path"),
            arg_str_opt("Program Name <NULL=Immediate>"),
        ],
        "PC6KUpLoad(card, file, progName)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            let controller = PC6K_CARDS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&card)
                .cloned()
                .ok_or_else(|| format!("PC6KUpLoad: Controller is not configured - {card}"))?;
            let file = req_string(args, 1, "Upload File Path")?;
            let text = std::fs::read(&file)
                .map_err(|_| format!("PC6KUpLoad: File does not exist - {file}"))?;
            let program = opt_string(args, 2)?.filter(|name| !name.is_empty());
            let controller = controller.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(name) = &program {
                controller.send(format!("DEL {name}"));
                controller.send(format!("DEF {name}"));
            }
            for line in drv_pc6k::upload_lines(&text) {
                controller.send(line);
            }
            if program.is_some() {
                controller.send("END");
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod one_poller {
    use motor_common::testing::pollers_after;

    use super::*;

    fn create_args(port: &str) -> Vec<ArgValue> {
        vec![
            ArgValue::String(port.into()),
            ArgValue::String(format!("{port}NoLine").into()),
            ArgValue::Int(3),
            ArgValue::Int(200),
            ArgValue::Int(1000),
        ]
    }

    /// OEMMotorDriver.cpp:68: one startPoller after the axes.
    #[epics_rs::base::epics_test]
    async fn oem_create_controller_polls_its_axes_on_one_poller() {
        let cmd = oem_create_controller_command();
        assert_eq!(pollers_after(cmd, create_args("oemOnePoller")).await, 1);
    }

    /// ACRMotorDriver.cpp:82: one startPoller after the axes.
    #[epics_rs::base::epics_test]
    async fn acr_create_controller_polls_its_axes_on_one_poller() {
        let cmd = acr_create_controller_command();
        assert_eq!(pollers_after(cmd, create_args("acrOnePoller")).await, 1);
    }
}
