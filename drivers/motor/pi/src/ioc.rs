//! iocsh commands for the `motor-pi` driver's ported controllers.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::c663::{PIC663Axis, PIC663Controller};
use crate::c862::{PIC862Axis, PIC862Controller};
use crate::e516::{PIE516Axis, PIE516Controller};

/// Command timeout (C `drvPIC862.h` `COMM_TIMEOUT` 2.0 s).
const PIC862_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIE516.h` `COMM_TIMEOUT` 2.0 s).
const PIE516_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIC663.h` `COMM_TIMEOUT` 2.0 s — C-663 is a C-862
/// clone and shares the same 2.0 s value).
const PIC663_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the `PIC862Setup(maxCards, [scanRate])` no-op command (startup-script
/// parity; the asyn-rs port allocates per `PIC862Config` call, matching every
/// other legacy-driver port in this workspace).
pub fn pic862_setup_command() -> CommandDef {
    CommandDef::new(
        "PIC862Setup",
        vec![arg_int_req("maxCards"), arg_int_opt("scanRate")],
        "PIC862Setup(maxCards, [scanRate]) - Accepted for parity; PIC862Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxCards")?;
            if max < 1 {
                return Err("PIC862Setup: maxCards must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC862Config(card, asynPort, addr, [movingPollMs],
/// [idlePollMs])` command bound to `holder`. `addr` is the controller's
/// multi-drop bus address (0-15, a single hex digit on the wire — C's
/// documented "0-F" range).
pub fn pic862_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "PIC862Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("addr"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "PIC862Config(card, asynPort, addr, [movingPollMs], [idlePollMs]) - \
         Create a PI C-862/C-863 controller (DTYP PIC862_{card}_0); addr is the \
         multi-drop bus address (0-15)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PIC862Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let addr = req_int(args, 2, "addr")?;
            if !(0..=15).contains(&addr) {
                return Err("PIC862Config: addr must be 0-15".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, PIC862_TIMEOUT)?;
            let controller = PIC862Controller::new(handle, addr as u8)
                .map_err(|e| format!("PIC862Config: {e}"))?;
            let ident = controller.ident().to_string();
            let controller = Arc::new(Mutex::new(controller));

            let axis = PIC862Axis::new(controller).map_err(|e| format!("PIC862Config: {e}"))?;
            let dtyp_key = format!("PIC862_{card}_0");
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);

            println!(
                "PIC862Config: card={card} asynPort={asyn_port} addr={addr} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PIC862_{card}_0)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC663Setup(maxCards, [scanRate])` no-op command (startup-script
/// parity; the asyn-rs port allocates per `PIC663Config` call, matching every
/// other legacy-driver port in this workspace).
pub fn pic663_setup_command() -> CommandDef {
    CommandDef::new(
        "PIC663Setup",
        vec![arg_int_req("maxCards"), arg_int_opt("scanRate")],
        "PIC663Setup(maxCards, [scanRate]) - Accepted for parity; PIC663Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxCards")?;
            if max < 1 {
                return Err("PIC663Setup: maxCards must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC663Config(card, asynPort, addr, [movingPollMs],
/// [idlePollMs])` command bound to `holder`. `addr` is the controller's
/// multi-drop bus address (0-15, a single hex digit on the wire — same
/// `\x01{addr}VE` select-at-connect exchange as the C-862 clone it derives
/// from).
pub fn pic663_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "PIC663Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("addr"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "PIC663Config(card, asynPort, addr, [movingPollMs], [idlePollMs]) - \
         Create a PI C-663 controller (DTYP PIC663_{card}_0); addr is the \
         multi-drop bus address (0-15)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PIC663Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let addr = req_int(args, 2, "addr")?;
            if !(0..=15).contains(&addr) {
                return Err("PIC663Config: addr must be 0-15".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, PIC663_TIMEOUT)?;
            let controller = PIC663Controller::new(handle, addr as u8)
                .map_err(|e| format!("PIC663Config: {e}"))?;
            let ident = controller.ident().to_string();
            let controller = Arc::new(Mutex::new(controller));

            let axis = PIC663Axis::new(controller).map_err(|e| format!("PIC663Config: {e}"))?;
            let dtyp_key = format!("PIC663_{card}_0");
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);

            println!(
                "PIC663Config: card={card} asynPort={asyn_port} addr={addr} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PIC663_{card}_0)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIE516Setup(maxCards, [scanRate])` no-op command (startup-script
/// parity; the asyn-rs port allocates per `PIE516Config` call).
pub fn pie516_setup_command() -> CommandDef {
    CommandDef::new(
        "PIE516Setup",
        vec![arg_int_req("maxCards"), arg_int_opt("scanRate")],
        "PIE516Setup(maxCards, [scanRate]) - Accepted for parity; PIE516Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxCards")?;
            if max < 1 {
                return Err("PIE516Setup: maxCards must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIE516Config(card, asynPort, [addr], [movingPollMs],
/// [idlePollMs])` command bound to `holder`. The E-516 is a 3-axis piezo
/// controller; the axis count is probed at connect time and one motor device
/// support is installed per responding axis (`DTYP PIE516_{card}_{axis}`,
/// axis = 0..n). `addr` ("asyn address (GPIB)") is accepted for C signature
/// parity but ignored — the driver hardcodes the asyn sub-address to 0 and
/// selects axes by the per-command A/B/C letter, not a bus address.
pub fn pie516_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "PIE516Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_opt("addr"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "PIE516Config(card, asynPort, [addr], [movingPollMs], [idlePollMs]) - \
         Create a PI E-516 piezo controller; probes axes and installs one motor \
         per axis (DTYP PIE516_{card}_{axis}). addr is accepted for parity but \
         ignored.",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PIE516Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, PIE516_TIMEOUT)?;
            let controller =
                PIE516Controller::new(handle).map_err(|e| format!("PIE516Config: {e}"))?;
            let ident = controller.ident().to_string();
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                let axis_obj = PIE516Axis::new(controller.clone(), axis)
                    .map_err(|e| format!("PIE516Config: {e}"))?;
                let dtyp_key = format!("PIE516_{card}_{axis}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_obj));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }

            println!(
                "PIE516Config: card={card} asynPort={asyn_port} axes={num_axes} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PIE516_{card}_0..{})",
                num_axes.saturating_sub(1)
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
