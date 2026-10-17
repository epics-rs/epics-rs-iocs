//! iocsh commands for the Aerotech Ensemble and A3200 drivers, C model-2
//! drivers: `EnsembleAsynConfig` / `A3200AsynConfig` build card `card` of
//! [`MOTOR_ENSEMBLE`] / [`MOTOR_A3200`], and `drvAsynMotorConfigure(port,
//! "motorEnsemble" | "motorA3200", card, numAxes)` opens its axes as
//! `@asyn(port,0..)`. An Ensemble's addresses count the axes found, not the
//! controller axis numbers (drvEnsembleAsyn.cc:965).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_opt, arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;
use motor_common::model2::{Model2, Model2Axis};

use crate::a3200::{self, A3200_MAX_AXES, A3200Axis, A3200Controller};
use crate::drv_soloist::{self, DrvSoloistAxis, SOLOIST_NUM_CARDS, SoloistCard};
use crate::ensemble::{self, ENSEMBLE_MAX_AXES, EnsembleAxis, EnsembleController};

/// `TIMEOUT` of both drivers (drvEnsembleAsyn.cc:187, drvA3200Asyn.cc:86).
const TIMEOUT: Duration = Duration::from_secs(2);

/// drvEnsembleAsyn.cc's `motorAxisDrvSET_t`.
pub static MOTOR_ENSEMBLE: Model2 = Model2::new("motorEnsemble");

/// drvA3200Asyn.cc's `motorAxisDrvSET_t`.
pub static MOTOR_A3200: Model2 = Model2::new("motorA3200");

/// `numEnsembleControllers` / `numA3200Controllers`, which the `Setup`s set.
static NUM_ENSEMBLE: Mutex<i32> = Mutex::new(0);
static NUM_A3200: Mutex<i32> = Mutex::new(0);

/// A `<Drv>AsynSetup(num_controllers)` setting `count`, printing `refusal`
/// as C's does for a count below 1.
fn setup_command(
    name: &'static str,
    count: &'static Mutex<i32>,
    refusal: &'static str,
) -> CommandDef {
    CommandDef::new(
        name,
        vec![arg_int_req("Max. controller count")],
        format!("{name}(num_controllers)"),
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let num_controllers = c_int(args, 0);
            if num_controllers < 1 {
                println!("{refusal}");
                return Ok(CommandOutcome::Continue);
            }
            *count.lock().unwrap_or_else(|e| e.into_inner()) = num_controllers;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `EnsembleAsynSetup(num_controllers)`.
pub fn ensemble_setup_command() -> CommandDef {
    setup_command(
        "EnsembleAsynSetup",
        &NUM_ENSEMBLE,
        "EnsembleSetup, num_controllers must be > 0",
    )
}

/// `A3200AsynSetup(num_controllers)`.
pub fn a3200_setup_command() -> CommandDef {
    setup_command(
        "A3200AsynSetup",
        &NUM_A3200,
        "A3200AsynSetup, num_controllers must be > 0",
    )
}

/// The `Config` arguments both drivers share (AerotechRegister.cc).
fn config_args() -> Vec<ArgDesc> {
    vec![
        arg_int_req("Card being configured"),
        arg_str_req("asyn port name"),
        arg_int_req("asyn address (GPIB)"),
        arg_int_req("Number of Axes"),
        arg_int_req("Moving poll rate"),
        arg_int_req("Idle poll rate"),
    ]
}

/// `EnsembleAsynConfig(card, portName, asynAddress, numAxes,
/// movingPollPeriod, idlePollPeriod)`, printing what C's does; a controller
/// that never answers fails silently, as in C.
pub fn ensemble_config_command() -> CommandDef {
    CommandDef::new(
        "EnsembleAsynConfig",
        config_args(),
        "EnsembleAsynConfig(card, portName, asynAddress, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            let port_name = req_string(args, 1, "asyn port name")?;
            let num_axes = c_int(args, 3);
            let moving_poll_ms = c_int(args, 4).max(0) as u64;
            let idle_poll_ms = c_int(args, 5).max(0) as u64;

            let num_controllers = *NUM_ENSEMBLE.lock().unwrap_or_else(|e| e.into_inner());
            if num_controllers < 1 {
                println!(
                    "EnsembleConfig: no Ensemble controllers allocated, call EnsembleSetup first"
                );
                return Ok(CommandOutcome::Continue);
            }
            if !(0..num_controllers).contains(&card) {
                println!(
                    "EnsembleConfig: card must in range 0 to {}",
                    num_controllers - 1
                );
                return Ok(CommandOutcome::Continue);
            }
            if !(1..=ENSEMBLE_MAX_AXES).contains(&num_axes) {
                println!("EnsembleConfig: numAxes must in range 1 to {ENSEMBLE_MAX_AXES}");
                return Ok(CommandOutcome::Continue);
            }

            let Ok(handle) = connect_with_eos(
                &port_name,
                TIMEOUT,
                ensemble::INPUT_EOS,
                ensemble::OUTPUT_EOS,
                EosRefusal::Ignored,
            ) else {
                println!("EnsembleAsynConfig: cannot connect to asyn port {port_name}");
                return Ok(CommandOutcome::Continue);
            };
            let controller = Arc::new(Mutex::new(EnsembleController::new(
                handle,
                &port_name,
                &card.to_string(),
            )));
            let answered = {
                let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                (0..3).any(|_| ctrl.ping().is_ok())
            };
            if !answered {
                return Ok(CommandOutcome::Continue);
            }

            let mut axes = Vec::new();
            for axis in 0..ENSEMBLE_MAX_AXES {
                if axes.len() as i32 == num_axes {
                    break;
                }
                let exists = controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .axis_exists(axis);
                if exists {
                    let ax = EnsembleAxis::new(controller.clone(), axis)
                        .map_err(|e| format!("EnsembleAsynConfig: axis {axis}: {e}"))?;
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    axes.push(Model2Axis {
                        motor,
                        moving_poll_ms,
                        idle_poll_ms,
                    });
                }
            }
            // C ignores the reply.
            let _ = controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .wait_mode_nowait();
            MOTOR_ENSEMBLE.set_card(card, axes);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `A3200AsynConfig(card, portName, asynAddress, numAxes, movingPollPeriod,
/// idlePollPeriod, taskNumber, linear)`, printing what C's does; a controller
/// that never answers `~TASK` fails silently, as in C.
pub fn a3200_config_command() -> CommandDef {
    let mut args = config_args();
    args.push(arg_int_req("Task number"));
    args.push(arg_int_req("Linear move commands"));
    CommandDef::new(
        "A3200AsynConfig",
        args,
        "A3200AsynConfig(card, portName, asynAddress, numAxes, movingPollPeriod, \
         idlePollPeriod, taskNumber, linear)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            let port_name = req_string(args, 1, "asyn port name")?;
            let num_axes = c_int(args, 3);
            let moving_poll_ms = c_int(args, 4).max(0) as u64;
            let idle_poll_ms = c_int(args, 5).max(0) as u64;
            let task_number = c_int(args, 6) as u32;
            let linear = c_int(args, 7) != 0;

            let num_controllers = *NUM_A3200.lock().unwrap_or_else(|e| e.into_inner());
            if num_controllers < 1 {
                println!("A3200AsynConfig: no A3200 controllers allocated, call A3200 first");
                return Ok(CommandOutcome::Continue);
            }
            if !(0..num_controllers).contains(&card) {
                println!(
                    "A3200AsynConfig: card must in range 0 to {}",
                    num_controllers - 1
                );
                return Ok(CommandOutcome::Continue);
            }
            if !(1..=A3200_MAX_AXES).contains(&num_axes) {
                println!("A3200AsynConfig: numAxes must be in the range of 1 to {A3200_MAX_AXES}");
                return Ok(CommandOutcome::Continue);
            }

            let Ok(handle) = connect_with_eos(
                &port_name,
                TIMEOUT,
                a3200::INPUT_EOS,
                a3200::OUTPUT_EOS,
                EosRefusal::Ignored,
            ) else {
                println!("A3200AsynConfig: cannot connect to asyn port {port_name}");
                return Ok(CommandOutcome::Continue);
            };
            let controller = Arc::new(Mutex::new(A3200Controller::new(
                handle,
                &port_name,
                &card.to_string(),
                task_number,
                linear,
            )));
            if controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .init_task()
                .is_err()
            {
                return Ok(CommandOutcome::Continue);
            }

            let mut axes = Vec::new();
            for axis in 0..num_axes {
                let axis_name = controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .discover_axis_name(axis)
                    .map_err(|e| format!("A3200AsynConfig: axis {axis}: name discovery: {e}"))?;
                let ax = A3200Axis::new(controller.clone(), axis_name.clone())
                    .map_err(|e| format!("A3200AsynConfig: axis {axis} ({axis_name}): {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                axes.push(Model2Axis {
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                });
            }
            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .finalize()
                .map_err(|e| format!("A3200AsynConfig: finalize failed: {e}"))?;
            MOTOR_A3200.set_card(card, axes);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devSoloist,"Soloist")` (devSoloist.dbd).
static DEV_SOLOIST: DevSup = model1_device("devSoloist", "Soloist");

static SOLOIST: Model1 = Model1::new(&DEV_SOLOIST, SOLOIST_NUM_CARDS, SCAN_RATE);

/// `SoloistSetup(num_cards, scan_rate)`.
pub fn soloist_setup_command() -> CommandDef {
    SOLOIST.setup_command("SoloistSetup")
}

/// `SoloistConfig(card, name, addr)`. C registers the command with two
/// arguments and reads the address past them; it is taken here, 0 when
/// absent. Connects and probes the card at once, where C's `motor_init`
/// does so at iocInit; a card that does not answer is left without its
/// axis, as there.
pub fn soloist_config_command() -> CommandDef {
    CommandDef::new(
        "SoloistConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_opt("asyn address (GPIB)"),
        ],
        "SoloistConfig(card, name, addr)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !SOLOIST.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let addr = c_int(args, 2);
            let (handle, connected) = SyncIOHandle::connect_port(
                &asyn_port,
                addr,
                drv_soloist::TIMEOUT,
                InterfaceType::Octet,
            );
            if connected.is_err() {
                return Ok(CommandOutcome::Continue);
            }
            let controller = SoloistCard::new(handle, addr);
            let Some(setup) = drv_soloist::motor_init(&controller) else {
                return Ok(CommandOutcome::Continue);
            };
            let controller = Arc::new(Mutex::new(controller));
            let motor: Arc<Mutex<dyn AsynMotor>> =
                Arc::new(Mutex::new(DrvSoloistAxis::new(controller, setup)));
            SOLOIST.install(ctx, card, 0, motor)?;
            Ok(CommandOutcome::Continue)
        },
    )
}
