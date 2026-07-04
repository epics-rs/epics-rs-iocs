//! iocsh commands for the SmarAct controllers.
//!
//! - **MCS2** (`MCS2CreateController`): single command — connect the asyn octet
//!   port, read the serial number, register `numAxes` channels behind a
//!   `DTYP`-keyed motor device support (`MCS2_{card}_{index}`, 0-based; channel =
//!   index).
//! - **SCU** (`smarActSCUCreateController` + `smarActSCUCreateAxis`): two
//!   commands, mirroring C — the first connects the serial port and registers a
//!   shared controller keyed by `card`; the second looks it up, probes and
//!   installs one channel (`DTYP` `SCU_{card}_{axisNo}`, explicit channel map).

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::{connect_ip, connect_serial};
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::mcs::{McsAxis, McsController};
use crate::mcs2::{Mcs2Axis, Mcs2Controller};
use crate::scu::{ScuAxis, ScuController};

/// Command timeout.
const MCS2_TIMEOUT: Duration = Duration::from_secs(1);

/// SCU command timeout (C `DEFAULT_TIMEOUT` = 2.0 s).
const SCU_TIMEOUT: Duration = Duration::from_secs(2);

/// MCS command timeout (C `DEFLT_TIMEOUT` = 2.0 s).
const MCS_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `MCS2CreateController(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs])` command bound to `holder`.
pub fn mcs2_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "MCS2CreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "MCS2CreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a SmarAct MCS2 controller (DTYP MCS2_{card}_{index}) with numAxes channels",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("MCS2CreateController: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("MCS2CreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_ip(&asyn_port, MCS2_TIMEOUT)?;
            let controller =
                Mcs2Controller::new(handle).map_err(|e| format!("MCS2CreateController: {e}"))?;
            let ident = controller.ident().to_string();
            let controller = Arc::new(Mutex::new(controller));

            for index in 0..num_axes as usize {
                let ax = Mcs2Axis::new(controller.clone(), index);
                let dtyp_key = format!("MCS2_{card}_{index}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }
            println!(
                "MCS2CreateController: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=MCS2_{card}_{{0..{}}})",
                (num_axes - 1).max(0)
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// A registered SCU controller plus the poll intervals its axes install with.
struct ScuRegistered {
    controller: Arc<Mutex<ScuController>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
}

/// SCU controllers registered by `smarActSCUCreateController`, keyed by `card`,
/// so `smarActSCUCreateAxis` can attach axes (mirrors the C `findAsynPortDriver`
/// lookup by port name).
fn scu_registry() -> &'static Mutex<HashMap<i64, ScuRegistered>> {
    static REG: OnceLock<Mutex<HashMap<i64, ScuRegistered>>> = OnceLock::new();
    REG.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Build `smarActSCUCreateController(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs])`. The C controller only connects and starts the poller; axes
/// are added separately, so `numAxes` is accepted for parity but the registry
/// simply holds the controller for `smarActSCUCreateAxis`.
pub fn scu_create_controller_command() -> CommandDef {
    CommandDef::new(
        "smarActSCUCreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "smarActSCUCreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a SmarAct SCU controller; add axes with smarActSCUCreateAxis",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("smarActSCUCreateController: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("smarActSCUCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, SCU_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(ScuController::new(handle)));

            scu_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(
                    card,
                    ScuRegistered {
                        controller,
                        moving_poll_ms,
                        idle_poll_ms,
                    },
                );
            println!(
                "smarActSCUCreateController: card={card} asynPort={asyn_port} numAxes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (add axes with smarActSCUCreateAxis)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build `smarActSCUCreateAxis(card, axisNo, channel)` bound to `holder`.
pub fn scu_create_axis_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "smarActSCUCreateAxis",
        vec![
            arg_int_req("card"),
            arg_int_req("axisNo"),
            arg_int_req("channel"),
        ],
        "smarActSCUCreateAxis(card, axisNo, channel) - Add axis axisNo (0-based, \
         DTYP SCU_{card}_{axisNo}) driving controller channel `channel`, to a \
         controller created by smarActSCUCreateController",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            let axis_no = req_int(args, 1, "axisNo")?;
            if axis_no < 0 {
                return Err("smarActSCUCreateAxis: axisNo must be >= 0".into());
            }
            let channel = req_int(args, 2, "channel")?;
            if channel < 0 {
                return Err("smarActSCUCreateAxis: channel must be >= 0".into());
            }

            let (controller, moving_poll_ms, idle_poll_ms) = {
                let reg = scu_registry().lock().unwrap_or_else(|e| e.into_inner());
                let entry = reg.get(&card).ok_or_else(|| {
                    format!("smarActSCUCreateAxis: no controller for card={card} (call smarActSCUCreateController first)")
                })?;
                (
                    entry.controller.clone(),
                    entry.moving_poll_ms,
                    entry.idle_poll_ms,
                )
            };

            let ax = ScuAxis::new(controller, channel as i32)
                .map_err(|e| format!("smarActSCUCreateAxis: {e}"))?;
            let dtyp_key = format!("SCU_{card}_{axis_no}");
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
            holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);

            println!(
                "smarActSCUCreateAxis: card={card} axisNo={axis_no} channel={channel} \
                 (DTYP=SCU_{card}_{axis_no})"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// A registered MCS controller plus the poll intervals its axes install with.
struct McsRegistered {
    controller: Arc<Mutex<McsController>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
}

/// MCS controllers registered by `smarActMCSCreateController`, keyed by `card`,
/// so `smarActMCSCreateAxis` can attach axes.
fn mcs_registry() -> &'static Mutex<HashMap<i64, McsRegistered>> {
    static REG: OnceLock<Mutex<HashMap<i64, McsRegistered>>> = OnceLock::new();
    REG.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Build `smarActMCSCreateController(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs], [disableSpeed])`. `disableSpeed` non-zero suppresses `SCLS`
/// speed-set commands (C's 6th argument).
pub fn mcs_create_controller_command() -> CommandDef {
    CommandDef::new(
        "smarActMCSCreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
            arg_int_opt("disableSpeed"),
        ],
        "smarActMCSCreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs], \
         [disableSpeed]) - Create a SmarAct MCS controller; add axes with smarActMCSCreateAxis",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("smarActMCSCreateController: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("smarActMCSCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let disable_speed = matches!(args.get(5), Some(ArgValue::Double(d)) if *d != 0.0);

            let handle = connect_ip(&asyn_port, MCS_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(McsController::new(handle, disable_speed)));

            mcs_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(
                    card,
                    McsRegistered {
                        controller,
                        moving_poll_ms,
                        idle_poll_ms,
                    },
                );
            println!(
                "smarActMCSCreateController: card={card} asynPort={asyn_port} numAxes={num_axes} \
                 disableSpeed={disable_speed} poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (add axes with smarActMCSCreateAxis)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build `smarActMCSCreateAxis(card, axisNo, channel)` bound to `holder`.
pub fn mcs_create_axis_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "smarActMCSCreateAxis",
        vec![
            arg_int_req("card"),
            arg_int_req("axisNo"),
            arg_int_req("channel"),
        ],
        "smarActMCSCreateAxis(card, axisNo, channel) - Add axis axisNo (0-based, \
         DTYP MCS_{card}_{axisNo}) driving controller channel `channel`, to a \
         controller created by smarActMCSCreateController",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            let axis_no = req_int(args, 1, "axisNo")?;
            if axis_no < 0 {
                return Err("smarActMCSCreateAxis: axisNo must be >= 0".into());
            }
            let channel = req_int(args, 2, "channel")?;
            if channel < 0 {
                return Err("smarActMCSCreateAxis: channel must be >= 0".into());
            }

            let (controller, moving_poll_ms, idle_poll_ms) = {
                let reg = mcs_registry().lock().unwrap_or_else(|e| e.into_inner());
                let entry = reg.get(&card).ok_or_else(|| {
                    format!("smarActMCSCreateAxis: no controller for card={card} (call smarActMCSCreateController first)")
                })?;
                (
                    entry.controller.clone(),
                    entry.moving_poll_ms,
                    entry.idle_poll_ms,
                )
            };

            let ax = McsAxis::new(controller, channel as i32)
                .map_err(|e| format!("smarActMCSCreateAxis: {e}"))?;
            let dtyp_key = format!("MCS_{card}_{axis_no}");
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
            holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);

            println!(
                "smarActMCSCreateAxis: card={card} axisNo={axis_no} channel={channel} \
                 (DTYP=MCS_{card}_{axis_no})"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
