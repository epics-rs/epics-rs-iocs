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
use motor_common::{Controller, asyn_user, model1_device, register_controller_port};

use crate::drv_im483::{
    DrvIm483Axis, IM483PL_NUM_CARDS, IM483SM_NUM_CARDS, Im483Controller, im483pl_probe,
    im483sm_probe,
};
use crate::drv_mdrive::{DrvMDriveAxis, MDRIVE_NUM_CARDS, MDriveController, mdrive_probe};
use crate::mdriveplus::{
    DRIVER_NAME, INPUT_EOS, ImsLine, ImsMDrivePlusAxis, ImsParams, OUTPUT_EOS,
};

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
            let line = ImsLine::new(handle, device_name);
            // The port C's constructor builds first; it needs the line here.
            let owner = ctx.db().scan_control().interrupt_gate();
            let port =
                register_controller_port(owner, &motor_port, 1, ImsParams::new(line.clone()))?;
            let mut axis = ImsMDrivePlusAxis::new(line, &motor_port);
            axis.post_params_to(port)
                .map_err(|e| format!("ImsMDrivePlusCreateController: {e}"))?;

            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            // C startPoller, ImsMDrivePlusMotorController.cpp:101
            let poller = Controller::new();
            poller.install(asyn_user(&motor_port, 0), motor)?;
            poller.start(ctx, moving_poll_ms as u64, idle_poll_ms as u64, 2);

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
    IM483PL.register_report("drvIM483PL", |cards| {
        cards.report("IM483PL", |card, port| {
            // C clears `ident`: no identification message (drvIM483PL.cc:541).
            format!("    IM483PL controller #{card}, port={port}, id:  \n")
        })
    });
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
            if axes > 0 {
                IM483PL.describe(card, asyn_port);
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
    IM483SM.register_report("drvIM483SM", |cards| {
        cards.report("IM483SM", |card, line| {
            format!("    IM483SM controller #{card}, {line} \n")
        })
    });
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
            let Some(ident) = im483sm_probe(&controller) else {
                return Ok(CommandOutcome::Continue);
            };
            let controller = Arc::new(Mutex::new(controller));
            let motor: Arc<Mutex<dyn AsynMotor>> =
                Arc::new(Mutex::new(DrvIm483Axis::im483sm(controller)));
            IM483SM.install(ctx, card, 0, motor)?;
            IM483SM.describe(card, format!("port={asyn_port}, id: {ident}"));
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devMDrive,"MDrive")` (devImsMotor.dbd).
static DEV_MDRIVE: DevSup = model1_device("devMDrive", "MDrive");

static MDRIVE: Model1 = Model1::new(&DEV_MDRIVE, MDRIVE_NUM_CARDS, SCAN_RATE);

/// `MDriveSetup(num_cards, scan_rate)`.
pub fn mdrive_setup_command() -> CommandDef {
    MDRIVE.register_report("drvMDrive", |cards| {
        cards.report("MDrive", |card, line| {
            format!("    MDrive controller #{card}, {line} \n")
        })
    });
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
            let (ident, drives) = mdrive_probe(&controller, card);
            if !drives.is_empty() {
                MDRIVE.describe(card, format!("port={asyn_port}, id: {ident}"));
            }
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

#[cfg(all(test, target_os = "linux"))]
mod one_poller {
    use super::*;
    use motor_common::testing::{LineRig, pollers_after};

    /// C ImsMDrivePlusMotorController's one asynMotorController poller.
    #[epics_rs::base::epics_test]
    async fn ims_mdrive_plus_create_controller_polls_on_one_poller() {
        let (line, _io) = LineRig::new("mdrivePlusLine", |last| {
            if last.contains("PR VR") {
                "1.000".into()
            } else {
                "0".into()
            }
        });
        let args = vec![
            ArgValue::String("mdrivePlus".into()),
            ArgValue::String("mdrivePlusLine".into()),
            ArgValue::String("".into()),
            ArgValue::Double(100.0),
            ArgValue::Double(1000.0),
        ];
        let cmd = ims_create_controller_command();
        assert_eq!(pollers_after(cmd, args).await, 1);
        assert!(!line.sent().is_empty());
    }
}

#[cfg(test)]
mod report {
    use super::*;
    use motor_common::testing::{LineRig, driver_report, run_command};

    /// drvIM483PL.cc's drvet `report` (drvIM483PL.cc:147-171), which `dbior`
    /// runs; the IM483PL has no identification message.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_im483pl_card() {
        let setup = im483pl_setup_command();
        assert_eq!(
            driver_report("drvIM483PL", 0),
            "    No IM483PL controllers configured.\n"
        );
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("im483plRepLine", |_| "0".into());
        let config = vec![int(1), ArgValue::String("im483plRepLine".into())];
        run_command(im483pl_config_command(), config).await;
        assert_eq!(
            driver_report("drvIM483PL", 0),
            "    IM483PL controller 0 connection failed.\n    IM483PL controller #1, \
             port=im483plRepLine, id:  \n"
        );
    }

    /// drvMDrive.cc's drvet `report` (drvMDrive.cc:162-186), which `dbior`
    /// runs, naming the chain by its first drive's `PR VR`.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_mdrive_chain() {
        let setup = mdrive_setup_command();
        assert_eq!(
            driver_report("drvMDrive", 0),
            "    No MDrive controllers configured.\n"
        );
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("mdriveRepLine", |cmd| {
            if cmd.ends_with("PR VR") { "3.009" } else { "0" }.into()
        });
        let config = vec![int(1), ArgValue::String("mdriveRepLine".into())];
        run_command(mdrive_config_command(), config).await;
        assert_eq!(
            driver_report("drvMDrive", 0),
            "    MDrive controller 0 connection failed.\n    MDrive controller #1, \
             port=mdriveRepLine, id: 3.009 \n"
        );
    }

    /// drvIM483SM.cc's drvet `report` (drvIM483SM.cc:145-169), which `dbior`
    /// runs, naming the card by its sign-on.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_im483sm_card() {
        let setup = im483sm_setup_command();
        assert_eq!(
            driver_report("drvIM483SM", 0),
            "    No IM483SM controllers configured.\n"
        );
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("im483smRepLine", |_| "X".into());
        let config = vec![int(1), ArgValue::String("im483smRepLine".into())];
        run_command(im483sm_config_command(), config).await;
        let ident = vec!["X"; 26].join(" ");
        assert_eq!(
            driver_report("drvIM483SM", 0),
            format!(
                "    IM483SM controller 0 connection failed.\n    IM483SM controller #1, \
                 port=im483smRepLine, id: {ident} \n"
            )
        );
    }
}

#[cfg(test)]
mod extra_params {
    use super::*;
    use epics_rs::asyn::manager::PortManager;
    use motor_common::testing::{LineRig, run_command};

    /// A controller `port` on line `line` with device name `A`, and the
    /// controller port's user at axis 0.
    async fn controller(port: &str, line: &str) -> (LineRig, SyncIOHandle) {
        failing_controller(port, line, |_| false).await
    }

    /// [`controller`], with the writes `fails` picks refused; `PR ER`
    /// answers 73.
    async fn failing_controller(
        port: &str,
        line: &str,
        fails: impl Fn(&str) -> bool + Send + Sync + 'static,
    ) -> (LineRig, SyncIOHandle) {
        let reply = |last: &str| {
            if last.contains("PR VR") {
                "1.000".into()
            } else if last.contains("PR LR") {
                "1".into()
            } else if last.contains("PR ER") {
                "73".into()
            } else {
                "0".into()
            }
        };
        let (rig, _io) = LineRig::failing(line, reply, fails);
        let args = vec![
            ArgValue::String(port.into()),
            ArgValue::String(line.into()),
            ArgValue::String("A".into()),
            ArgValue::Double(100.0),
            ArgValue::Double(1000.0),
        ];
        run_command(ims_create_controller_command(), args).await;
        let handle = PortManager::global().find_port_handle(port).unwrap();
        (rig, SyncIOHandle::from_handle(handle, 0, IMS_TIMEOUT))
    }

    /// The commands among `sent` that the poller does not send.
    fn not_polled(sent: Vec<String>) -> Vec<String> {
        sent.into_iter()
            .filter(|s| !s.starts_with("APR "))
            .collect()
    }

    /// ImsMDrivePlusMotorController.cpp:325-333 and
    /// ImsMDrivePlusMotorAxis.cpp:515-536: IMS_SAVETONVM 1 sends `S`, any
    /// other value only stores it.
    #[epics_rs::base::epics_test]
    async fn save_to_nvm_sends_s_on_one() {
        let (line, io) = controller("imsNvm", "imsNvmLine").await;
        let save = io.drv_user_create("IMS_SAVETONVM").unwrap();
        line.sent();
        io.write_int32(save, 0).unwrap();
        assert_eq!(io.read_int32(save).unwrap(), 0);
        assert!(not_polled(line.sent()).is_empty());
        io.write_int32(save, 1).unwrap();
        assert_eq!(not_polled(line.sent()), ["AS"]);
        assert_eq!(io.read_int32(save).unwrap(), 1);
    }

    /// ImsMDrivePlusMotorAxis.cpp:528-532 then
    /// ImsMDrivePlusMotorController.cpp:328: a failed `S` runs saveToNVM's
    /// handleAxisError before writeInt32's own ERROR line.
    #[epics_rs::base::epics_test]
    async fn a_failed_save_to_nvm_prints_the_drive_s_error_code() {
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let (line, io) = failing_controller("imsNvmErr", "imsNvmErrLine", |l| l == "AS").await;
        let save = io.drv_user_create("IMS_SAVETONVM").unwrap();
        let path = std::env::temp_dir().join(format!("imsNvmErr-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some("imsNvmErr"), TraceMask::ERROR);
        trace.set_trace_file(Some("imsNvmErr"), TraceFile::File(file.clone()));
        trace.set_device_trace_file("imsNvmErr", 0, TraceFile::File(file));
        line.sent();
        assert!(io.write_int32(save, 1).is_err());
        // `APR ER` is among the poll's `APR ` commands; the line below is its reply.
        assert_eq!(not_polled(line.sent()), ["AS"]);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.splitn(3, ' ').nth(2))
            .collect();
        assert_eq!(
            lines,
            [
                "ImsMDrivePlusMotorDriver:handleAxisError(): ImsMDrivePlusMotorDriver:saveToNVM(): \
                 ERROR saving to NVM, 73:'Tried to save while moving'",
                "ImsMDrivePlusMotorDriver:writeInt32: ERROR saving to NVM",
            ],
            "{text}"
        );
    }

    /// ImsMDrivePlusMotorController.cpp:371-373, ImsMDrivePlusMotorAxis.cpp:
    /// 494-497: a failed IO sets the controller's addr-0 COMMS_ERROR param,
    /// not MSTA; a good poll clears it.
    #[epics_rs::base::epics_test]
    async fn a_failed_poll_sets_only_the_comms_error_param() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let dead = Arc::new(AtomicBool::new(false));
        let fails = {
            let dead = dead.clone();
            move |l: &str| l == "APR P" && dead.load(Ordering::Relaxed)
        };
        let (_line, io) = failing_controller("imsComms", "imsCommsLine", fails).await;
        let comms = io.drv_user_create("MOTOR_STATUS_COMMS_ERROR").unwrap();
        let msta = io.drv_user_create("MOTOR_STATUS").unwrap();
        let until = |want: i32| {
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            while io.read_int32(comms).unwrap_or(-1) != want {
                assert!(std::time::Instant::now() < deadline, "comms never {want}");
                std::thread::sleep(Duration::from_millis(10));
            }
        };
        until(0);
        dead.store(true, Ordering::Relaxed);
        until(1);
        assert_eq!(io.read_int32(msta).unwrap() & 1 << 12, 0);
        dead.store(false, Ordering::Relaxed);
        until(0);
    }

    /// ImsMDrivePlusMotorController.cpp:335-343 and
    /// ImsMDrivePlusMotorAxis.cpp:542-563: IMS_CLEARLRERROR 1 sends `CF`.
    #[epics_rs::base::epics_test]
    async fn clear_lr_error_sends_cf_on_one() {
        let (line, io) = controller("imsClrLr", "imsClrLrLine").await;
        let clear = io.drv_user_create("IMS_CLEARLRERROR").unwrap();
        line.sent();
        io.write_int32(clear, 0).unwrap();
        assert!(not_polled(line.sent()).is_empty());
        io.write_int32(clear, 1).unwrap();
        assert_eq!(not_polled(line.sent()), ["ACF"]);
        assert_eq!(io.read_int32(clear).unwrap(), 1);
    }

    /// ImsMDrivePlusMotorAxis.cpp:479-484: the poll reads `PR LR` after
    /// `PR C2` into IMS_LOCKEDROTOR.
    #[epics_rs::base::epics_test]
    async fn the_poll_reads_the_locked_rotor_flag() {
        let (line, io) = controller("imsLr", "imsLrLine").await;
        let locked = io.drv_user_create("IMS_LOCKEDROTOR").unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while io.read_int32(locked).unwrap_or(0) != 1 {
            assert!(std::time::Instant::now() < deadline);
            epics_rs::base::runtime::task::sleep(Duration::from_millis(10)).await;
        }
        let sent = line.sent();
        let c2 = sent.iter().position(|s| s == "APR C2").unwrap();
        assert_eq!(sent[c2 + 1], "APR LR", "{sent:?}");
    }
}
