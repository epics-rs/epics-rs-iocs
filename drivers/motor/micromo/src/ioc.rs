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
use motor_common::{asyn_user, install, model1_device, register_port};

use crate::drv_mvp2001::{self, DrvMvp2001Axis, MVP2001_NUM_CARDS, Mvp2001Chain};
use crate::mvp2001::{Mvp2001Axis, Mvp2001Controller};

/// Command timeout (C `DEFAULT_CONTROLLER_TIMEOUT`).
const MVP2001_TIMEOUT: Duration = Duration::from_secs(1);

/// A registered controller plus the poll intervals its axes should install with.
struct Registered {
    controller: Arc<Mutex<Mvp2001Controller>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
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

            register_port(ctx, &port_name, num_axes as i32)?;

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

            registry().lock().unwrap_or_else(|e| e.into_inner()).insert(
                port_name,
                Registered {
                    controller,
                    moving_poll_ms,
                    idle_poll_ms,
                },
            );
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
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Controller port name")?;
            let axis_no = req_int(args, 1, "Axis number")?;
            if axis_no < 0 {
                return Err("MVP2001CreateAxis: axisNo must be >= 0".into());
            }
            let enc_lpr = req_int(args, 2, "Encoder lines per rev")?;
            let max_curr = req_int(args, 3, "Max current (ma)")?;
            let lim_pol = req_int(args, 4, "Limit polarity")?;

            let (controller, moving_poll_ms, idle_poll_ms) = {
                let reg = registry().lock().unwrap_or_else(|e| e.into_inner());
                let Some(entry) = reg.get(&port_name) else {
                    println!("Error port {port_name} not found");
                    return Ok(CommandOutcome::Continue);
                };
                (
                    entry.controller.clone(),
                    entry.moving_poll_ms,
                    entry.idle_poll_ms,
                )
            };

            let ax = Mvp2001Axis::new(
                controller,
                axis_no as usize,
                enc_lpr as i32,
                max_curr as i32,
                lim_pol as i32,
            );
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
            install(
                ctx,
                asyn_user(&port_name, axis_no as i32),
                motor,
                moving_poll_ms,
                idle_poll_ms,
            )?;
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
    MVP2001.setup_command("MicroMoSetup")
}

/// `MicroMoConfig(card, name)`, MicroMoRegister.cc's name for
/// `MVP2001Config`. C never registers it, and devMicroMo.dbd never names
/// `MicroMomotorRegister`, so neither command reaches C's iocsh. Connects and probes the chain at once, where
/// C's `motor_init` does so at iocInit; a chain with no answering
/// controller is left unconfigured, as there.
pub fn micromo_config_command() -> CommandDef {
    CommandDef::new(
        "MicroMoConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "MicroMoConfig(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !MVP2001.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let handle = connect_with_eos(
                &asyn_port,
                drv_mvp2001::COMM_TIMEOUT,
                b"\n",
                b"\r",
                EosRefusal::Ignored,
            )?;
            let chain = Mvp2001Chain::new(handle);
            let limits = drv_mvp2001::motor_init(&chain);
            let chain = Arc::new(Mutex::new(chain));
            for (signal, limits) in limits.into_iter().enumerate() {
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(DrvMvp2001Axis::new(
                    chain.clone(),
                    signal,
                    limits,
                )));
                MVP2001.install(ctx, card, signal as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}
