//! `MVP2001CreateController` / `MVP2001CreateAxis` iocsh commands for the
//! MicroMo MVP 2001.
//!
//! The MVP 2001 configures its controller and axes with two separate commands
//! (C `MVP2001CreateController` then one `MVP2001CreateAxis` per axis, because
//! each axis carries its own encoder/current/limit configuration). This port
//! keeps that split: `MVP2001CreateController` connects the serial port and
//! creates controller port `portName`; `MVP2001CreateAxis` looks the controller
//! up by port name, constructs and initializes the axis, and installs it as
//! `@asyn(portName,axisNo)`.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_controller, connect_with_eos};
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::{AxisReport, Controller, asyn_user, model1_device, register_port_with};

use crate::drv_mvp2001::{self, DrvMvp2001Axis, MVP2001_NUM_CARDS, Mvp2001Chain};
use crate::mvp2001::{Mvp2001Axis, Mvp2001Controller, Mvp2001Params};

/// Command timeout (C `DEFAULT_CONTROLLER_TIMEOUT`).
const MVP2001_TIMEOUT: Duration = Duration::from_secs(2);

/// A registered controller and the poller its axes join.
struct Registered {
    controller: Arc<Mutex<Mvp2001Controller>>,
    poller: Arc<Controller>,
}

/// Controllers registered by `MVP2001CreateController`, keyed by port name, so
/// that `MVP2001CreateAxis` can attach axes to them (C `findAsynPortDriver`).
fn registry() -> &'static Mutex<HashMap<String, Registered>> {
    static REG: OnceLock<Mutex<HashMap<String, Registered>>> = OnceLock::new();
    REG.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Build the `MVP2001CreateController(portName, MVP2001PortName, numAxes,
/// movingPollPeriod, idlePollPeriod)` command, periods in ms.
pub fn mvp2001_create_controller_command() -> CommandDef {
    CommandDef::new(
        "MVP2001CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("MVP 2001 port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "MVP2001CreateController(portName, MVP2001PortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let asyn_port = req_string(args, 1, "MVP 2001 port name")?;
            let num_axes = req_int(args, 2, "Number of axes")?;
            if num_axes < 1 {
                return Err("MVP2001CreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port_with(
                ctx,
                &port_name,
                num_axes as i32,
                Mvp2001Params::new(moving_poll_ms, idle_poll_ms),
            )?;

            let (handle, _) = connect_controller(
                &asyn_port,
                MVP2001_TIMEOUT,
                &port_name,
                format_args!(
                    "MVP2001Controller::MVP2001Controller: cannot connect to MVP 2001 controller"
                ),
            );
            let controller = Arc::new(Mutex::new(Mvp2001Controller::new(
                handle,
                num_axes as usize,
            )));

            // MVP2001Driver.cpp:57, before any axis exists.
            let poller = Arc::new(Controller::new());
            poller.start(&port_name, moving_poll_ms, idle_poll_ms, 2);
            registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(port_name, Registered { controller, poller });
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `MVP2001CreateAxis(MVP2001Name, axisNo, encLPR, maxCurr, limPol)`
/// command; `limPol` 1=NO, 0=NC.
pub fn mvp2001_create_axis_command() -> CommandDef {
    CommandDef::new(
        "MVP2001CreateAxis",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Axis number"),
            arg_int_req("Encoder lines per rev"),
            arg_int_req("Max current (ma)"),
            arg_int_req("Limit polarity"),
        ],
        "MVP2001CreateAxis(MVP2001Name, axisNo, encLPR, maxCurr, limPol)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Controller port name")?;
            let axis_no = req_int(args, 1, "Axis number")?;
            if axis_no < 0 {
                return Err("MVP2001CreateAxis: axisNo must be >= 0".into());
            }
            let enc_lpr = req_int(args, 2, "Encoder lines per rev")?;
            let max_curr = req_int(args, 3, "Max current (ma)")?;
            let lim_pol = req_int(args, 4, "Limit polarity")?;

            let (controller, poller) = {
                let reg = registry().lock().unwrap_or_else(|e| e.into_inner());
                let Some(entry) = reg.get(&port_name) else {
                    println!("Error port {port_name} not found");
                    return Ok(CommandOutcome::Continue);
                };
                (entry.controller.clone(), entry.poller.clone())
            };

            // C constructs the axis under pC->lock() (MVP2001Driver.cpp:222-224).
            poller.install_built(asyn_user(&port_name, axis_no as i32), || {
                let ax = Mvp2001Axis::new(
                    controller,
                    &port_name,
                    axis_no as usize,
                    enc_lpr as i32,
                    max_curr as i32,
                    lim_pol as i32,
                );
                let report: Arc<dyn AxisReport> = Arc::new(ax.report());
                Ok((Arc::new(Mutex::new(ax)), Some(report)))
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devMVP2001,"MVP2001")` (devMicroMo.dbd).
static DEV_MVP2001: DevSup = model1_device("devMVP2001", "MVP2001");

static MVP2001: Model1 = Model1::new(&DEV_MVP2001, MVP2001_NUM_CARDS, SCAN_RATE);

/// `MicroMoSetup(num_cards, scan_rate)`, MicroMoRegister.cc's name for
/// `MVP2001Setup`.
pub fn micromo_setup_command() -> CommandDef {
    MVP2001.register_report("drvMVP2001", |cards| {
        if cards.num_cards() <= 0 {
            return "    No MVP2001 CHAINS configured.\n".into();
        }
        (0..cards.num_cards())
            .map(|card| match cards.card(card) {
                Some(line) => format!("    MVP2001 controller chain #{card}, {line} \n"),
                None => format!("    MVP2001 controller chain #{card} connection failed.\n"),
            })
            .collect()
    });
    MVP2001.setup_command("MicroMoSetup")
}

/// `MicroMoConfig(card, name)`, MicroMoRegister.cc's name for
/// `MVP2001Config`. C never registers it, and devMicroMo.dbd never names
/// `MicroMomotorRegister`, so neither command reaches C's iocsh. `motor_init` at
/// iocInit connects and probes the chain; a chain with no answering
/// controller is left unconfigured.
pub fn micromo_config_command() -> CommandDef {
    CommandDef::new(
        "MicroMoConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "MicroMoConfig(card, name)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !MVP2001.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            MVP2001.config(card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    drv_mvp2001::COMM_TIMEOUT,
                    b"\n",
                    b"\r",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let chain = Mvp2001Chain::new(handle);
                let limits = drv_mvp2001::motor_init(&chain);
                // drvMVP2001.cc:725: a chain with no controller is NULL again.
                if limits.is_empty() {
                    return;
                }
                let chain = Arc::new(Mutex::new(chain));
                for (signal, limits) in limits.into_iter().enumerate() {
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(
                        DrvMvp2001Axis::new(chain.clone(), signal, limits),
                    ));
                    init.axis(signal as i32, motor);
                }
                // C's `motor_init` clears `ident` and never fills it.
                init.describe(format!("port={asyn_port}, id: "));
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(all(test, target_os = "linux"))]
mod one_poller {
    use motor_common::testing::pollers_after;

    use super::*;

    /// MVP2001Driver.cpp:57: the controller's poller, which every
    /// MVP2001CreateAxis joins.
    #[epics_rs::base::epics_test]
    async fn mvp2001_create_axis_joins_the_controller_poller() {
        let args = vec![
            ArgValue::String("mvpOnePoller".into()),
            ArgValue::String("mvpOnePollerNoLine".into()),
            ArgValue::Int(2),
            ArgValue::Int(200),
            ArgValue::Int(1000),
        ];
        let cmd = mvp2001_create_controller_command();
        assert_eq!(pollers_after(cmd, args).await, 1);
        for axis in 0..2 {
            let args = vec![
                ArgValue::String("mvpOnePoller".into()),
                ArgValue::Int(axis),
                ArgValue::Int(512),
                ArgValue::Int(500),
                ArgValue::Int(1),
            ];
            assert_eq!(pollers_after(mvp2001_create_axis_command(), args).await, 1);
        }
    }
}

#[cfg(test)]
mod report {
    use super::*;
    use motor_common::testing::{LineRig, driver_report, drvsup_init, port_report, run_command};

    /// drvMVP2001.cc's drvet `report` (drvMVP2001.cc:207-231), which `dbior`
    /// runs; `motor_init` leaves `ident` empty.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_mvp2001_chain() {
        let setup = micromo_setup_command();
        assert_eq!(
            driver_report("drvMVP2001", 0),
            "    No MVP2001 CHAINS configured.\n"
        );
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        // Each command answers once, terminator then reply, so a flush ends.
        let reads = Mutex::new((String::new(), 0));
        let (_line, _io) = LineRig::new("mvpChainLine", move |cmd| {
            let mut reads = reads.lock().unwrap();
            if reads.0 != cmd {
                *reads = (cmd.to_string(), 0);
            }
            reads.1 += 1;
            match (cmd.trim_end(), reads.1) {
                ("1 ST", 1) => "\r".into(),
                ("1 ST", 2) => "0001 0000".into(),
                _ => String::new(),
            }
        });
        let config = vec![int(1), ArgValue::String("mvpChainLine".into())];
        run_command(micromo_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvMVP2001", 0),
            "    MVP2001 controller chain #0 connection failed.\n    MVP2001 controller \
             chain #1, port=mvpChainLine, id:  \n"
        );
    }

    /// MVP2001Controller::report then MVP2001Axis::report
    /// (MVP2001Driver.cpp:85-92, :235-248).
    #[epics_rs::base::epics_test]
    async fn mvp2001_report_prints_the_controller_then_each_axis_above_level_0() {
        let (_line, _io) = LineRig::new("mvpRepLine", |cmd| match cmd.trim_end() {
            "2 SR" => "0002 0190".into(),
            _ => String::new(),
        });
        let args = vec![
            ArgValue::String("mvpRep".into()),
            ArgValue::String("mvpRepLine".into()),
            ArgValue::Int(2),
            ArgValue::Int(100),
            ArgValue::Int(1000),
        ];
        run_command(mvp2001_create_controller_command(), args).await;
        let args = vec![
            ArgValue::String("mvpRep".into()),
            ArgValue::Int(1),
            ArgValue::Int(512),
            ArgValue::Int(5000),
            ArgValue::Int(7),
        ];
        run_command(mvp2001_create_axis_command(), args).await;
        let head = "MVP 2001 motor driver mvpRep, numAxes=2, moving poll period=0.100000, \
                    idle poll period=1.000000\n";
        let text = port_report("mvpRep", 0);
        assert!(text.contains(&format!("{head}Port: mvpRep\n")), "{text}");
        let text = port_report("mvpRep", 1);
        let axis = "  axis 1\n  axis index 2\n  encoderLinesPerRev 512\n  maxCurrent 100 (mA)\n  \
                    samplePeriod 400 (us)\n  limitPolarity 1\n";
        assert!(
            text.contains(&format!("{head}{axis}Port: mvpRep\n")),
            "{text}"
        );
    }
}
