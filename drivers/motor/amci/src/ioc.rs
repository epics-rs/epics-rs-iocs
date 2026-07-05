//! iocsh commands for the AMCI controllers.
//!
//! - **ANF2** (`ANF2CreateController` + `ANF2CreateAxis` + `ANF2StartPoller`):
//!   three commands, mirroring C — the first connects the In/Out Modbus ports
//!   and registers a shared controller keyed by *port name* (C looks the
//!   controller back up by port name via `findAsynPortDriver`, so the
//!   registry here is `String`-keyed rather than the `i64`-card keying
//!   `smaract`'s two-step commands use); the second looks it up and installs
//!   one axis (`DTYP` `{portName}_{axis}`). Poll intervals move to
//!   `ANF2CreateAxis` — C's `ANF2CreateAxis` has none (they only exist on the
//!   deferred `ANF2StartPoller`) — because `MotorHolder::install` needs them
//!   at axis-install time; `ANF2StartPoller` is accepted for startup-script
//!   parity but is a no-op — see its doc comment.
//! - **ANG1** (`ANG1CreateController`): single command — connects the ports
//!   and installs all `numAxes` axes immediately, matching the C constructor.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::anf2::{Anf2Axis, Anf2Controller, MAX_AXES};
use crate::ang1::{Ang1Axis, Ang1Controller};

/// ANF2 controllers registered by `ANF2CreateController`, keyed by port name
/// (C `findAsynPortDriver(ANF2Name)` — the C driver looks the controller back
/// up by its own asyn port name, not a numeric card).
fn anf2_registry() -> &'static Mutex<HashMap<String, Arc<Mutex<Anf2Controller>>>> {
    static REG: OnceLock<Mutex<HashMap<String, Arc<Mutex<Anf2Controller>>>>> = OnceLock::new();
    REG.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Build `ANF2CreateController(portName, inPort, outPort, numAxes)`.
pub fn anf2_create_controller_command() -> CommandDef {
    CommandDef::new(
        "ANF2CreateController",
        vec![
            arg_str_req("portName"),
            arg_str_req("inPort"),
            arg_str_req("outPort"),
            arg_int_req("numAxes"),
        ],
        "ANF2CreateController(portName, inPort, outPort, numAxes) - Create an AMCI \
         ANF2 controller (Modbus In/Out ports created by drvModbusAsynConfigure); \
         add axes with ANF2CreateAxis",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = req_string(args, 0, "portName")?;
            let in_port = req_string(args, 1, "inPort")?;
            let out_port = req_string(args, 2, "outPort")?;
            let num_axes = req_int(args, 3, "numAxes")?.clamp(0, MAX_AXES as i64) as usize;

            let controller = Anf2Controller::new(&in_port, &out_port, num_axes)
                .map_err(|e| format!("ANF2CreateController: {e}"))?;

            anf2_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(port_name.clone(), Arc::new(Mutex::new(controller)));

            println!(
                "ANF2CreateController: portName={port_name} inPort={in_port} \
                 outPort={out_port} numAxes={num_axes} (add axes with ANF2CreateAxis)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build `ANF2CreateAxis(portName, axis, hexConfig, baseSpeed, homingTimeout,
/// [movingPollMs], [idlePollMs])` bound to `holder`. C's `ANF2CreateAxis` has
/// no poll-interval arguments (those live on `ANF2StartPoller`); they move
/// here because `MotorHolder::install` needs them at axis-install time.
pub fn anf2_create_axis_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "ANF2CreateAxis",
        vec![
            arg_str_req("portName"),
            arg_int_req("axis"),
            arg_str_req("hexConfig"),
            arg_int_req("baseSpeed"),
            arg_int_req("homingTimeout"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "ANF2CreateAxis(portName, axis, hexConfig, baseSpeed, homingTimeout, \
         [movingPollMs], [idlePollMs]) - Add axis `axis` (0-based, DTYP \
         {portName}_{axis}) to a controller created by ANF2CreateController",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "portName")?;
            let axis = req_int(args, 1, "axis")?;
            if axis < 0 {
                return Err("ANF2CreateAxis: axis must be >= 0".into());
            }
            let hex_config = req_string(args, 2, "hexConfig")?;
            let hex_digits = hex_config
                .strip_prefix("0x")
                .or_else(|| hex_config.strip_prefix("0X"))
                .unwrap_or(&hex_config);
            let config = u32::from_str_radix(hex_digits, 16)
                .map_err(|_| format!("ANF2CreateAxis: invalid hexConfig={hex_config}"))?
                as i32;
            let base_speed = req_int(args, 3, "baseSpeed")?.clamp(1, 1_000_000) as i32;
            let homing_timeout = req_int(args, 4, "homingTimeout")?.clamp(0, 300) as i32;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 5, 6)?;

            let controller = anf2_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&port_name)
                .cloned()
                .ok_or_else(|| {
                    format!(
                        "ANF2CreateAxis: no controller for portName={port_name} \
                         (call ANF2CreateController first)"
                    )
                })?;

            let ax = Anf2Axis::new(controller, axis as i32, config, base_speed, homing_timeout)
                .map_err(|e| format!("ANF2CreateAxis: {e}"))?;
            let dtyp_key = format!("{port_name}_{axis}");
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
            holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);

            println!(
                "ANF2CreateAxis: portName={port_name} axis={axis} config=0x{config:x} \
                 baseSpeed={base_speed} homingTimeout={homing_timeout} \
                 poll=[{moving_poll_ms}/{idle_poll_ms}]ms (DTYP={port_name}_{axis})"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build `ANF2StartPoller(portName, movingPollPeriod, idlePollPeriod)`.
/// Accepted for startup-script parity (the C example scripts call this after
/// `iocInit`) but a no-op here: this port's poll loop already starts idle at
/// axis-install time and only begins actually polling once the record's
/// device support runs `init()` post-`iocInit` (the generic mechanism every
/// vendor driver in this workspace relies on) — exactly the timing
/// `ANF2StartPoller` exists to achieve in C. Still validates the port exists,
/// matching C's "port not found" error.
pub fn anf2_start_poller_command() -> CommandDef {
    CommandDef::new(
        "ANF2StartPoller",
        vec![
            arg_str_req("portName"),
            arg_int_req("movingPollPeriod"),
            arg_int_req("idlePollPeriod"),
        ],
        "ANF2StartPoller(portName, movingPollPeriod, idlePollPeriod) - Accepted for \
         startup-script parity; a no-op here (polling already starts automatically \
         after iocInit)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = req_string(args, 0, "portName")?;
            anf2_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&port_name)
                .ok_or_else(|| format!("ANF2StartPoller: port {port_name} not found"))?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build `ANG1CreateController(portName, inPort, outPort, numAxes,
/// movingPollMs, idlePollMs)` bound to `holder`. Matches the C constructor:
/// connects the ports and installs all `numAxes` axes immediately (no
/// separate create-axis command).
pub fn ang1_create_controller_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "ANG1CreateController",
        vec![
            arg_str_req("portName"),
            arg_str_req("inPort"),
            arg_str_req("outPort"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "ANG1CreateController(portName, inPort, outPort, numAxes, [movingPollMs], \
         [idlePollMs]) - Create an AMCI ANG1 controller (Modbus In/Out ports \
         created by drvModbusAsynConfigure) with numAxes axes (DTYP {portName}_{axis})",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "portName")?;
            let in_port = req_string(args, 1, "inPort")?;
            let out_port = req_string(args, 2, "outPort")?;
            let num_axes = req_int(args, 3, "numAxes")?;
            // Stricter than C by design: C's axis loop (ANG1Driver.cpp:94)
            // simply doesn't execute for numAxes<=0, silently creating a
            // zero-axis controller with no error. A zero-axis controller is a
            // no-op in practice, so rejecting numAxes < 1 here only surfaces a
            // likely startup-script mistake earlier rather than diverging in
            // any runtime behavior.
            if num_axes < 1 {
                return Err("ANG1CreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 4, 5)?;

            let controller = Ang1Controller::new(&in_port, &out_port)
                .map_err(|e| format!("ANG1CreateController: {e}"))?;
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                // Infallible, matching C: axis construction ignores the
                // initial setPosition status and never aborts this command.
                let ax = Ang1Axis::new(controller.clone(), axis as i32);
                let dtyp_key = format!("{port_name}_{axis}");
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                holder.install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
            }

            println!(
                "ANG1CreateController: portName={port_name} inPort={in_port} \
                 outPort={out_port} numAxes={num_axes} poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP={port_name}_{{0..{}}})",
                (num_axes - 1).max(0)
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
