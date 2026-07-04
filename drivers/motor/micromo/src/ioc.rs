//! `MVP2001CreateController` / `MVP2001CreateAxis` iocsh commands for the
//! MicroMo MVP 2001.
//!
//! The MVP 2001 configures its controller and axes with two separate commands
//! (C `MVP2001CreateController` then one `MVP2001CreateAxis` per axis, because
//! each axis carries its own encoder/current/limit configuration). This port
//! keeps that split: `MVP2001CreateController` connects the serial port and
//! registers a shared controller keyed by `card`; `MVP2001CreateAxis` looks the
//! controller up, constructs and initializes the axis, and installs it behind a
//! `DTYP`-keyed motor device support (`MVP2001_{card}_{axisNo}`, 0-based).

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::mvp2001::{Mvp2001Axis, Mvp2001Controller};

/// Command timeout (C `DEFAULT_CONTROLLER_TIMEOUT`).
const MVP2001_TIMEOUT: Duration = Duration::from_secs(1);

/// A registered controller plus the poll intervals its axes should install with.
struct Registered {
    controller: Arc<Mutex<Mvp2001Controller>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
}

/// Controllers registered by `MVP2001CreateController`, keyed by `card`, so that
/// `MVP2001CreateAxis` can attach axes to them (mirrors the C `findAsynPortDriver`
/// lookup by port name).
fn registry() -> &'static Mutex<HashMap<i64, Registered>> {
    static REG: OnceLock<Mutex<HashMap<i64, Registered>>> = OnceLock::new();
    REG.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Build the `MVP2001CreateController(card, asynPort, numAxes, [movingPollMs],
/// [idlePollMs])` command.
pub fn mvp2001_create_controller_command() -> CommandDef {
    CommandDef::new(
        "MVP2001CreateController",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "MVP2001CreateController(card, asynPort, numAxes, [movingPollMs], [idlePollMs]) - \
         Create a MicroMo MVP 2001 controller; add axes with MVP2001CreateAxis",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("MVP2001CreateController: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if num_axes < 1 {
                return Err("MVP2001CreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, MVP2001_TIMEOUT)?;
            let controller = Arc::new(Mutex::new(Mvp2001Controller::new(
                handle,
                num_axes as usize,
            )));

            registry().lock().unwrap_or_else(|e| e.into_inner()).insert(
                card,
                Registered {
                    controller,
                    moving_poll_ms,
                    idle_poll_ms,
                },
            );
            println!(
                "MVP2001CreateController: card={card} asynPort={asyn_port} numAxes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (add axes with MVP2001CreateAxis)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `MVP2001CreateAxis(card, axisNo, encLinesPerRev, maxCurrentMa,
/// limitPolarity)` command bound to `holder`.
pub fn mvp2001_create_axis_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "MVP2001CreateAxis",
        vec![
            arg_int_req("card"),
            arg_int_req("axisNo"),
            arg_int_req("encLinesPerRev"),
            arg_int_req("maxCurrentMa"),
            arg_int_req("limitPolarity"),
        ],
        "MVP2001CreateAxis(card, axisNo, encLinesPerRev, maxCurrentMa, limitPolarity) - \
         Add axis axisNo (0-based, DTYP MVP2001_{card}_{axisNo}) to a controller \
         created by MVP2001CreateController; limitPolarity 1=NO, 0=NC",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            let axis_no = req_int(args, 1, "axisNo")?;
            if axis_no < 0 {
                return Err("MVP2001CreateAxis: axisNo must be >= 0".into());
            }
            let enc_lpr = req_int(args, 2, "encLinesPerRev")?;
            let max_curr = req_int(args, 3, "maxCurrentMa")?;
            let lim_pol = req_int(args, 4, "limitPolarity")?;

            let (controller, moving_poll_ms, idle_poll_ms) = {
                let reg = registry().lock().unwrap_or_else(|e| e.into_inner());
                let entry = reg.get(&card).ok_or_else(|| {
                    format!("MVP2001CreateAxis: no controller for card={card} (call MVP2001CreateController first)")
                })?;
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
            )
            .map_err(|e| format!("MVP2001CreateAxis: {e}"))?;
            let dtyp_key = format!("MVP2001_{card}_{axis_no}");
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
            holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);

            println!(
                "MVP2001CreateAxis: card={card} axisNo={axis_no} encLinesPerRev={enc_lpr} \
                 maxCurrentMa={max_curr} limitPolarity={lim_pol} (DTYP=MVP2001_{card}_{axis_no})"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
