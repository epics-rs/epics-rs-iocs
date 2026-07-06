//! iocsh commands for the `motor-pi` driver's ported controllers.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::connect::connect_serial;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, opt_int, poll_intervals, req_int, req_string,
};

use crate::c630::{PIC630_NUM_AXIS, PIC630Axis, PIC630Controller};
use crate::c662::{PIC662Axis, PIC662Controller};
use crate::c663::{PIC663Axis, PIC663Controller};
use crate::c844::{PIC844_NUM_AXES, PIC844Axis, PIC844Controller};
use crate::c848::{PIC848Axis, PIC848Controller};
use crate::c862::{PIC862Axis, PIC862Controller};

/// Command timeout (C `drvPIC630.h` `COMM_TIMEOUT` 2 s).
const PIC630_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIC662.h` `COMM_TIMEOUT` 2 s).
const PIC662_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPI.h` `COMM_TIMEOUT` 2 s).
const PIC844_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIC848.h` `COMM_TIMEOUT` 2 s).
const PIC848_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIC862.h` `COMM_TIMEOUT` 2.0 s).
const PIC862_TIMEOUT: Duration = Duration::from_secs(2);

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

/// Build the `PIC630Setup(maxControllers, maxAxes, [scanRate])` no-op command
/// (startup-script parity; the asyn-rs port allocates per `PIC630Config` call).
pub fn pic630_setup_command() -> CommandDef {
    CommandDef::new(
        "PIC630Setup",
        vec![
            arg_int_req("maxControllers"),
            arg_int_req("maxAxes"),
            arg_int_opt("scanRate"),
        ],
        "PIC630Setup(maxControllers, maxAxes, [scanRate]) - Accepted for parity; \
         PIC630Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxControllers")?;
            if max < 1 {
                return Err("PIC630Setup: maxControllers must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC630Config(card, asynPort, numAxes, [cur1..cur9],
/// [movingPollMs], [idlePollMs])` command bound to `holder`.
///
/// `numAxes` (1-9) is the number of daisy-chained axes on this serial line,
/// addressed `1`-`numAxes`. It is passed explicitly here because C derives it
/// from `PIC630Setup`'s global `num_channels`, and this port's `PIC630Setup`
/// is a stateless parity no-op. `cur1..cur9` are the per-axis drive currents
/// (`0`=OFF, `1`=100 mA … `8`=800 mA), sent as `{addr}DC{cur}` at connect;
/// currents beyond `numAxes` are ignored.
pub fn pic630_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    let mut arg_defs = vec![
        arg_int_req("card"),
        arg_str_req("asynPort"),
        arg_int_req("numAxes"),
    ];
    for i in 1..=PIC630_NUM_AXIS {
        arg_defs.push(arg_int_opt(match i {
            1 => "cur1",
            2 => "cur2",
            3 => "cur3",
            4 => "cur4",
            5 => "cur5",
            6 => "cur6",
            7 => "cur7",
            8 => "cur8",
            _ => "cur9",
        }));
    }
    arg_defs.push(arg_int_opt("movingPollMs"));
    arg_defs.push(arg_int_opt("idlePollMs"));

    CommandDef::new(
        "PIC630Config",
        arg_defs,
        "PIC630Config(card, asynPort, numAxes, [cur1..cur9], [movingPollMs], [idlePollMs]) - \
         Create a PI C-630 stepper chain (DTYP PIC630_{card}_{axis}, axis 0..numAxes-1); \
         cur1..cur9 are per-axis drive currents (0=OFF, 1=100mA .. 8=800mA)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PIC630Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let num_axes = req_int(args, 2, "numAxes")?;
            if !(1..=PIC630_NUM_AXIS as i64).contains(&num_axes) {
                return Err(format!("PIC630Config: numAxes must be 1-{PIC630_NUM_AXIS}"));
            }
            let num_axes = num_axes as u8;
            // cur1..cur9 live at arg indices 3..=11.
            let currents: Vec<i32> = (0..PIC630_NUM_AXIS as usize)
                .map(|i| opt_int(args, 3 + i, 0, "cur").map(|v| v as i32))
                .collect::<Result<_, _>>()?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(
                args,
                3 + PIC630_NUM_AXIS as usize,
                4 + PIC630_NUM_AXIS as usize,
            )?;

            let handle = connect_serial(&asyn_port, PIC630_TIMEOUT)?;
            let controller = PIC630Controller::new(handle, num_axes)
                .map_err(|e| format!("PIC630Config: {e}"))?;
            let controller = Arc::new(Mutex::new(controller));

            for signal in 0..num_axes {
                let axis = PIC630Axis::new(controller.clone(), signal, currents[signal as usize])
                    .map_err(|e| format!("PIC630Config: {e}"))?;
                let dtyp_key = format!("PIC630_{card}_{signal}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }

            println!(
                "PIC630Config: card={card} asynPort={asyn_port} numAxes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP=PIC630_{card}_0..{})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC662Setup(maxCards, [scanRate])` no-op command (startup-script
/// parity; the asyn-rs port allocates per `PIC662Config` call).
pub fn pic662_setup_command() -> CommandDef {
    CommandDef::new(
        "PIC662Setup",
        vec![arg_int_req("maxCards"), arg_int_opt("scanRate")],
        "PIC662Setup(maxCards, [scanRate]) - Accepted for parity; PIC662Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxCards")?;
            if max < 1 {
                return Err("PIC662Setup: maxCards must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC662Config(card, asynPort, [movingPollMs], [idlePollMs])`
/// command bound to `holder`. The E-662 is a single-axis SCPI-like piezo
/// controller with no protocol address.
pub fn pic662_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "PIC662Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "PIC662Config(card, asynPort, [movingPollMs], [idlePollMs]) - \
         Create a PI E-662 piezo controller (DTYP PIC662_{card}_0)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PIC662Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;

            let handle = connect_serial(&asyn_port, PIC662_TIMEOUT)?;
            let controller =
                PIC662Controller::new(handle).map_err(|e| format!("PIC662Config: {e}"))?;
            let ident = controller.ident().to_string();
            let controller = Arc::new(Mutex::new(controller));

            let axis = PIC662Axis::new(controller).map_err(|e| format!("PIC662Config: {e}"))?;
            let dtyp_key = format!("PIC662_{card}_0");
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);

            println!(
                "PIC662Config: card={card} asynPort={asyn_port} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PIC662_{card}_0)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC844Setup(maxCards, [scanRate])` no-op command (startup-script
/// parity; the asyn-rs port allocates per `PIC844Config` call).
pub fn pic844_setup_command() -> CommandDef {
    CommandDef::new(
        "PIC844Setup",
        vec![arg_int_req("maxCards"), arg_int_opt("scanRate")],
        "PIC844Setup(maxCards, [scanRate]) - Accepted for parity; PIC844Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxCards")?;
            if max < 1 {
                return Err("PIC844Setup: maxCards must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC844Config(card, asynPort, addr, [movingPollMs], [idlePollMs])`
/// command bound to `holder`. Installs all four axes (DTYP
/// `PIC844_{card}_{axis}`, `axis` 0-3; wire axis is `axis + 1`). `addr` is the
/// C `asyn_address` (GPIB); like the C driver it is stored for signature parity
/// but never used on the wire (the axis is selected by an `AXIS n;` prefix, and
/// the transport connects at asyn sub-address 0).
pub fn pic844_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "PIC844Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("addr"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "PIC844Config(card, asynPort, addr, [movingPollMs], [idlePollMs]) - \
         Create a PI C-844 4-axis controller (DTYP PIC844_{card}_0..3); addr is \
         accepted for parity but unused on the wire",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PIC844Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let _addr = req_int(args, 2, "addr")?; // parity only; unused on wire
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, PIC844_TIMEOUT)?;
            let controller =
                PIC844Controller::new(handle).map_err(|e| format!("PIC844Config: {e}"))?;
            let ident = controller.ident().to_string();
            let controller = Arc::new(Mutex::new(controller));

            for signal in 0..PIC844_NUM_AXES {
                let axis = PIC844Axis::new(controller.clone(), signal)
                    .map_err(|e| format!("PIC844Config: {e}"))?;
                let dtyp_key = format!("PIC844_{card}_{signal}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }

            println!(
                "PIC844Config: card={card} asynPort={asyn_port} \
                 ident=\"{ident}\" poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PIC844_{card}_0..3)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC848Setup(maxCards, [scanRate])` no-op command (startup-script
/// parity; the asyn-rs port allocates per `PIC848Config` call).
pub fn pic848_setup_command() -> CommandDef {
    CommandDef::new(
        "PIC848Setup",
        vec![arg_int_req("maxCards"), arg_int_opt("scanRate")],
        "PIC848Setup(maxCards, [scanRate]) - Accepted for parity; PIC848Config allocates per call",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let max = req_int(args, 0, "maxCards")?;
            if max < 1 {
                return Err("PIC848Setup: maxCards must be > 0".into());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build the `PIC848Config(card, asynPort, addr, [movingPollMs], [idlePollMs])`
/// command bound to `holder`. The axis count is probed at connect (`CST?` until
/// `NOSTAGE`); one axis is installed per present stage (DTYP
/// `PIC848_{card}_{axis}`, `axis` 0-3, wire letter `A`+`axis`). `addr` is the C
/// `asyn_address` (GPIB), stored for signature parity but unused on the wire
/// (the axis is selected by byte 5 of each command; the transport connects at
/// asyn sub-address 0).
pub fn pic848_config_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "PIC848Config",
        vec![
            arg_int_req("card"),
            arg_str_req("asynPort"),
            arg_int_req("addr"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "PIC848Config(card, asynPort, addr, [movingPollMs], [idlePollMs]) - \
         Create a PI C-848 controller; axes are probed via CST? (DTYP \
         PIC848_{card}_0..N); addr is accepted for parity but unused on the wire",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = req_int(args, 0, "card")?;
            if card < 0 {
                return Err("PIC848Config: card must be >= 0".into());
            }
            let asyn_port = req_string(args, 1, "asynPort")?;
            let _addr = req_int(args, 2, "addr")?; // parity only; unused on wire
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            let handle = connect_serial(&asyn_port, PIC848_TIMEOUT)?;
            let controller =
                PIC848Controller::new(handle).map_err(|e| format!("PIC848Config: {e}"))?;
            let ident = controller.ident().to_string();
            let num_axes = controller.num_axes();
            if num_axes == 0 {
                return Err(format!(
                    "PIC848Config: card={card} asynPort={asyn_port} reported no stages \
                     (CST? => NOSTAGE on axis A)"
                ));
            }
            let controller = Arc::new(Mutex::new(controller));

            for signal in 0..num_axes {
                let axis = PIC848Axis::new(controller.clone(), signal)
                    .map_err(|e| format!("PIC848Config: {e}"))?;
                let dtyp_key = format!("PIC848_{card}_{signal}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }

            println!(
                "PIC848Config: card={card} asynPort={asyn_port} \
                 ident=\"{ident}\" numAxes={num_axes} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PIC848_{card}_0..{})",
                num_axes - 1
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
