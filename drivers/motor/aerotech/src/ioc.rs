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

use epics_libcom_rs::cfmt::CFloat;
use motor_common::AxisReport;
use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_opt, arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;
use motor_common::model2::Model2;

use crate::a3200::{self, A3200_MAX_AXES, A3200Axis, A3200Controller};
use crate::drv_soloist::{self, DrvSoloistAxis, SOLOIST_NUM_CARDS, SoloistCard};
use crate::ensemble::{self, ENSEMBLE_MAX_AXES, EnsembleAxis, EnsembleController};

/// `TIMEOUT` of both drivers (drvEnsembleAsyn.cc:187, drvA3200Asyn.cc:86).
const TIMEOUT: Duration = Duration::from_secs(2);

/// drvEnsembleAsyn.cc's `motorAxisDrvSET_t`.
/// drvEnsembleAsyn.cc:703's poller forces no fast polls.
pub static MOTOR_ENSEMBLE: Model2 = Model2::new("motorEnsemble", 0);

/// drvA3200Asyn.cc's `motorAxisDrvSET_t`.
/// drvA3200Asyn.cc:723's poller forces no fast polls.
pub static MOTOR_A3200: Model2 = Model2::new("motorA3200", 0);

/// What C's `motorAxisReport` prints of one controller: the `Setup`'s
/// calloc'd entry, whose periods `Config` sets before it connects.
#[derive(Default)]
struct CardReport {
    moving_poll_s: f64,
    idle_poll_s: f64,
    /// The axes `Config` found. C prints `numAxes` entries, the zeroed ones
    /// of axes it never found (or of a card that never answered) as
    /// `Axis 0`; those are not copied.
    axes: Vec<Arc<dyn AxisReport>>,
}

/// `pEnsembleController` / `pA3200Controller`, one entry per controller the
/// `Setup` allocated.
static ENSEMBLE_CARDS: Mutex<Vec<CardReport>> = Mutex::new(Vec::new());
static A3200_CARDS: Mutex<Vec<CardReport>> = Mutex::new(Vec::new());

fn lock_cards(cards: &Mutex<Vec<CardReport>>) -> std::sync::MutexGuard<'_, Vec<CardReport>> {
    cards.lock().unwrap_or_else(|e| e.into_inner())
}

/// C `motorAxisReport` (drvEnsembleAsyn.cc:230-244, drvA3200Asyn.cc:211-228).
fn motor_axis_report(cards: &Mutex<Vec<CardReport>>, level: i32) -> String {
    let mut out = String::new();
    for card in lock_cards(cards).iter() {
        if level != 0 {
            out += &format!(
                "    moving poll period: {:.6}\n    idle poll period: {:.6}\n",
                CFloat(card.moving_poll_s),
                CFloat(card.idle_poll_s)
            );
        }
        for axis in &card.axes {
            axis.report(&mut out, level);
        }
    }
    out
}

/// `card`'s entry as `Config` sets it before connecting.
fn begin_card(cards: &Mutex<Vec<CardReport>>, card: i32, moving_poll_ms: u64, idle_poll_ms: u64) {
    lock_cards(cards)[card as usize] = CardReport {
        moving_poll_s: moving_poll_ms as f64 / 1000.0,
        idle_poll_s: idle_poll_ms as f64 / 1000.0,
        axes: Vec::new(),
    };
}

/// A `<Drv>AsynSetup(num_controllers)` allocating `cards`, printing
/// `refusal` as C's does for a count below 1.
fn setup_command(
    name: &'static str,
    cards: &'static Mutex<Vec<CardReport>>,
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
            *lock_cards(cards) = (0..num_controllers)
                .map(|_| CardReport::default())
                .collect();
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `EnsembleAsynSetup(num_controllers)`.
pub fn ensemble_setup_command() -> CommandDef {
    MOTOR_ENSEMBLE.register_report(|level| motor_axis_report(&ENSEMBLE_CARDS, level));
    setup_command(
        "EnsembleAsynSetup",
        &ENSEMBLE_CARDS,
        "EnsembleSetup, num_controllers must be > 0",
    )
}

/// `A3200AsynSetup(num_controllers)`.
pub fn a3200_setup_command() -> CommandDef {
    MOTOR_A3200.register_report(|level| motor_axis_report(&A3200_CARDS, level));
    setup_command(
        "A3200AsynSetup",
        &A3200_CARDS,
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

            let num_controllers = lock_cards(&ENSEMBLE_CARDS).len() as i32;
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
            begin_card(&ENSEMBLE_CARDS, card, moving_poll_ms, idle_poll_ms);

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
            let mut reports: Vec<Arc<dyn AxisReport>> = Vec::new();
            let mut logs = Vec::new();
            for axis in 0..ENSEMBLE_MAX_AXES {
                if axes.len() as i32 == num_axes {
                    break;
                }
                let exists = controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .axis_exists(axis);
                if exists {
                    let ax = EnsembleAxis::new(controller.clone(), axis, axes.len() as i32)
                        .map_err(|e| format!("EnsembleAsynConfig: axis {axis}: {e}"))?;
                    reports.push(ax.report());
                    logs.push(ax.log());
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                    axes.push(motor);
                }
            }
            // C ignores the reply.
            let _ = controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .wait_mode_nowait();
            lock_cards(&ENSEMBLE_CARDS)[card as usize].axes = reports;
            MOTOR_ENSEMBLE.set_card(card, axes, moving_poll_ms, idle_poll_ms);
            MOTOR_ENSEMBLE.set_axis_logs(card, logs);
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

            let num_controllers = lock_cards(&A3200_CARDS).len() as i32;
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
            begin_card(&A3200_CARDS, card, moving_poll_ms, idle_poll_ms);

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
            let mut reports: Vec<Arc<dyn AxisReport>> = Vec::new();
            let mut logs = Vec::new();
            for axis in 0..num_axes {
                let axis_name = controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .discover_axis_name(axis)
                    .map_err(|e| format!("A3200AsynConfig: axis {axis}: name discovery: {e}"))?;
                let ax = A3200Axis::new(controller.clone(), axis_name.clone())
                    .map_err(|e| format!("A3200AsynConfig: axis {axis} ({axis_name}): {e}"))?;
                reports.push(ax.report());
                logs.push(ax.log());
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                axes.push(motor);
            }
            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .finalize()
                .map_err(|e| format!("A3200AsynConfig: finalize failed: {e}"))?;
            lock_cards(&A3200_CARDS)[card as usize].axes = reports;
            MOTOR_A3200.set_card(card, axes, moving_poll_ms, idle_poll_ms);
            MOTOR_A3200.set_axis_logs(card, logs);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devSoloist,"Soloist")` (devSoloist.dbd).
static DEV_SOLOIST: DevSup = model1_device("devSoloist", "Soloist");

static SOLOIST: Model1 = Model1::new(&DEV_SOLOIST, SOLOIST_NUM_CARDS, SCAN_RATE);

/// `SoloistSetup(num_cards, scan_rate)`.
pub fn soloist_setup_command() -> CommandDef {
    SOLOIST.register_report("drvSoloist", |cards| {
        cards.report("Soloist", |card, line| {
            format!("    Soloist controller {card}, {line} \n")
        })
    });
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
            let Some(init) = drv_soloist::motor_init(&controller) else {
                return Ok(CommandOutcome::Continue);
            };
            let ident = init.user_string.unwrap_or_else(|| format!("Soloist{card}"));
            SOLOIST.describe(
                card,
                format!("port={asyn_port}, address={addr}, id: {ident}"),
            );
            let Some(setup) = init.axis else {
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

#[cfg(test)]
mod report {
    use std::time::{Duration, Instant};

    use super::*;
    use motor_common::model2::drv_asyn_motor_configure_command;
    use motor_common::testing::{LineRig, driver_report, run_command};

    fn config(card: i64, line: &str, num_axes: i64) -> Vec<ArgValue> {
        vec![
            ArgValue::Int(card),
            ArgValue::String(line.into()),
            ArgValue::Int(0),
            ArgValue::Int(num_axes),
            ArgValue::Int(100),
            ArgValue::Int(1000),
        ]
    }

    /// `driver`'s report at `level` once it contains `want`.
    async fn reported(driver: &str, level: i32, want: &str) -> String {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let text = driver_report(driver, level);
            if text.contains(want) || Instant::now() > deadline {
                return text;
            }
            epics_rs::base::runtime::task::sleep(Duration::from_millis(10)).await;
        }
    }

    /// drvEnsembleAsyn.cc's motorAxisReport (:218-244), which `dbior` runs
    /// over every controller `EnsembleAsynSetup` allocated.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_ensemble_controller_and_axis() {
        let setup = ensemble_setup_command();
        assert_eq!(driver_report("motorEnsemble", 1), "");
        run_command(setup, vec![ArgValue::Int(2)]).await;
        let (_line, _io) = LineRig::new("ensRepLine", |cmd| {
            match cmd.trim_end() {
                "GETPARM(@0, 140)" => "%X",
                c if c.starts_with("GETPARM(@") && c.ends_with(", 140)") => "!",
                "GETPARM(@0, 2)" => "%1000",
                "GETPARM(@0, 77)" => "%2.5",
                "AXISSTATUS(@0)" => "%5",
                _ => "%0",
            }
            .into()
        });
        run_command(ensemble_config_command(), config(1, "ensRepLine", 1)).await;
        let axis = "Axis 0\n   axisStatus:  0x0\n   home preset: 2.500000\n   \
                    step size:   0.001000\n   max digits:  5\n";
        assert_eq!(
            driver_report("motorEnsemble", 1),
            format!(
                "    moving poll period: 0.000000\n    idle poll period: 0.000000\n    \
                 moving poll period: 0.100000\n    idle poll period: 1.000000\n{axis}"
            )
        );
        assert_eq!(driver_report("motorEnsemble", 0), "");

        // The poller's AXISSTATUS lands in axisStatus.
        let args = vec![
            ArgValue::String("ensRep".into()),
            ArgValue::String("motorEnsemble".into()),
            ArgValue::Int(1),
            ArgValue::Int(1),
        ];
        run_command(
            drv_asyn_motor_configure_command(vec![&MOTOR_ENSEMBLE]),
            args,
        )
        .await;
        let text = reported("motorEnsemble", 1, "axisStatus:  0x5\n").await;
        assert!(text.contains("Axis 0\n   axisStatus:  0x5\n"), "{text}");
    }

    /// drvA3200Asyn.cc's motorAxisReport (:199-228).
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_a3200_controller_and_axis() {
        let setup = a3200_setup_command();
        assert_eq!(driver_report("motorA3200", 1), "");
        run_command(setup, vec![ArgValue::Int(2)]).await;
        let (_line, _io) = LineRig::new("a32RepLine", |cmd| {
            match cmd.trim_end() {
                "~GETVARIABLE $strtask0" => "%X",
                "CountsPerUnit.X" => "%1000",
                "HomeOffset.X" => "%2.5",
                c if c.starts_with("~STATUS") => "%5 0 0 0 0 0",
                _ => "%0",
            }
            .into()
        });
        let mut args = config(1, "a32RepLine", 1);
        args.extend([ArgValue::Int(1), ArgValue::Int(0)]);
        run_command(a3200_config_command(), args).await;
        let axis = "Axis X\n   axisStatus:  0x0\n   home preset: 2.500000\n   \
                    step size:   0.001000\n   max digits:  5\n";
        assert_eq!(
            driver_report("motorA3200", 1),
            format!(
                "    moving poll period: 0.000000\n    idle poll period: 0.000000\n    \
                 moving poll period: 0.100000\n    idle poll period: 1.000000\n{axis}"
            )
        );
        assert_eq!(driver_report("motorA3200", 0), "");

        // The poller's ~STATUS AxisStatus lands in axisStatus.
        let args = vec![
            ArgValue::String("a32Rep".into()),
            ArgValue::String("motorA3200".into()),
            ArgValue::Int(1),
            ArgValue::Int(1),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&MOTOR_A3200]), args).await;
        let text = reported("motorA3200", 1, "axisStatus:  0x5\n").await;
        assert!(text.contains("Axis X\n   axisStatus:  0x5\n"), "{text}");
    }

    /// drvSoloist.cc's drvet `report` (:148-172). A card that answered
    /// `NONE` is reported though it has no axis, as C's `motor_state` keeps
    /// it; its ident is UserString1, else `Soloist<card>` (:606-615).
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_soloist_card() {
        let setup = soloist_setup_command();
        assert_eq!(
            driver_report("drvSoloist", 0),
            "    No Soloist controllers configured.\n"
        );
        run_command(setup, vec![ArgValue::Int(3), ArgValue::Int(10)]).await;
        let (_a, _io_a) = LineRig::new("solRepA", |cmd| {
            if cmd.trim_end() == "GETPARM(265)" {
                "!"
            } else {
                "%0"
            }
            .into()
        });
        let (_b, _io_b) = LineRig::new("solRepB", |cmd| {
            match cmd.trim_end() {
                "GETPARM(265)" => "%MySoloist",
                "GETPARM(257)" => "!",
                _ => "%0",
            }
            .into()
        });
        for (card, line) in [(0, "solRepA"), (1, "solRepB")] {
            let args = vec![
                ArgValue::Int(card),
                ArgValue::String(line.into()),
                ArgValue::Int(0),
            ];
            run_command(soloist_config_command(), args).await;
        }
        assert_eq!(
            driver_report("drvSoloist", 0),
            "    Soloist controller 0, port=solRepA, address=0, id: Soloist0 \n    \
             Soloist controller 1, port=solRepB, address=0, id: MySoloist \n    \
             Soloist controller 2 connection failed.\n"
        );
    }
}
