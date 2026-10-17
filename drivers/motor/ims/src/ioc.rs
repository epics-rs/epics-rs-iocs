//! iocsh commands for the IMS MDrivePlus driver and the model-1 IMS
//! drivers (ImsRegister.cc).
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
use epics_rs::base::server::record::DevSup;

use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::sync_io::SyncIOHandle;
use motor_common::connect::{EosRefusal, connect_with_eos, set_eos_ignored};
use motor_common::iocsh::{
    arg_double_opt, arg_int_req, arg_str_opt, arg_str_req, opt_double, opt_string, req_string,
};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::{asyn_user, install, model1_device, register_port};

use crate::drv_im483::{
    DrvIm483Axis, IM483PL_NUM_CARDS, IM483SM_NUM_CARDS, Im483Controller, im483pl_probe,
    im483sm_probe,
};
use crate::drv_mdrive::{DrvMDriveAxis, MDRIVE_NUM_CARDS, MDriveController, mdrive_probe};
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
            let (handle, connected) =
                SyncIOHandle::connect_port(&io_port, 0, IMS_TIMEOUT, InterfaceType::Octet);
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
                2, // C startPoller, ImsMDrivePlusMotorController.cpp:101
            )?;

            Ok(CommandOutcome::Continue)
        },
    )
}

/// drvIM483.h's `COMM_TIMEOUT`.
const COMM_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devIM483PL,"IM483PL")` (devImsMotor.dbd).
static DEV_IM483PL: DevSup = model1_device("devIM483PL", "IM483PL");

static IM483PL: Model1 = Model1::new(&DEV_IM483PL, IM483PL_NUM_CARDS, SCAN_RATE);

/// `IM483PLSetup(num_cards, scan_rate)`.
pub fn im483pl_setup_command() -> CommandDef {
    IM483PL.setup_command("IM483PLSetup")
}

/// ImsRegister.cc's `Config` arguments.
fn config_args() -> Vec<ArgDesc> {
    vec![
        arg_int_req("Card being configured"),
        arg_str_req("asyn port name"),
    ]
}

/// `IM483PLConfig(card, name)`. Connects and probes the card at once, where
/// C's `motor_init` does so at iocInit; a card with no answering axis is left
/// unconfigured, as there.
pub fn im483pl_config_command() -> CommandDef {
    CommandDef::new(
        "IM483PLConfig",
        config_args(),
        "IM483PLConfig(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !IM483PL.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let handle =
                connect_with_eos(&asyn_port, COMM_TIMEOUT, b"\n", b"\n", EosRefusal::Ignored)?;
            let controller = Im483Controller::new(handle);
            let axes = im483pl_probe(&controller);
            let controller = Arc::new(Mutex::new(controller));
            for signal in 0..axes {
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(DrvIm483Axis::im483pl(
                    controller.clone(),
                    signal,
                )));
                IM483PL.install(ctx, card, signal as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devIM483SM,"IM483SM")` (devImsMotor.dbd).
static DEV_IM483SM: DevSup = model1_device("devIM483SM", "IM483SM");

static IM483SM: Model1 = Model1::new(&DEV_IM483SM, IM483SM_NUM_CARDS, SCAN_RATE);

/// `IM483SMSetup(num_cards, scan_rate)`.
pub fn im483sm_setup_command() -> CommandDef {
    IM483SM.setup_command("IM483SMSetup")
}

/// `IM483SMConfig(card, name)`: one axis per card, probed at once as for
/// [`im483pl_config_command`].
pub fn im483sm_config_command() -> CommandDef {
    CommandDef::new(
        "IM483SMConfig",
        config_args(),
        "IM483SMConfig(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !IM483SM.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let handle = connect_with_eos(
                &asyn_port,
                COMM_TIMEOUT,
                b"\r\n",
                b"\r",
                EosRefusal::Ignored,
            )?;
            let controller = Im483Controller::new(handle);
            if !im483sm_probe(&controller) {
                return Ok(CommandOutcome::Continue);
            }
            let controller = Arc::new(Mutex::new(controller));
            let motor: Arc<Mutex<dyn AsynMotor>> =
                Arc::new(Mutex::new(DrvIm483Axis::im483sm(controller)));
            IM483SM.install(ctx, card, 0, motor)?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devMDrive,"MDrive")` (devImsMotor.dbd).
static DEV_MDRIVE: DevSup = model1_device("devMDrive", "MDrive");

static MDRIVE: Model1 = Model1::new(&DEV_MDRIVE, MDRIVE_NUM_CARDS, SCAN_RATE);

/// `MDriveSetup(num_cards, scan_rate)`.
pub fn mdrive_setup_command() -> CommandDef {
    MDRIVE.setup_command("MDriveSetup")
}

/// `MDriveConfig(card, name)`: probes the chain at once as for
/// [`im483pl_config_command`].
pub fn mdrive_config_command() -> CommandDef {
    CommandDef::new(
        "MDriveConfig",
        config_args(),
        "MDriveConfig(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !MDRIVE.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let handle = connect_with_eos(
                &asyn_port,
                COMM_TIMEOUT,
                b"\r\n",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller = MDriveController::new(handle);
            let drives = mdrive_probe(&controller, card);
            let controller = Arc::new(Mutex::new(controller));
            for (signal, drive) in drives.into_iter().enumerate() {
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(DrvMDriveAxis::new(
                    controller.clone(),
                    signal,
                    drive,
                )));
                MDRIVE.install(ctx, card, signal as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
