//! iocsh commands for the AMCI controllers.
//!
//! - **ANF2** (`ANF2CreateController` + `ANF2CreateAxis` + `ANF2StartPoller`):
//!   three commands, as C. The first connects the In/Out Modbus ports and
//!   creates controller port `portName`; the second looks it up by port name
//!   (C `findAsynPortDriver`) and adds one axis, `@asyn(portName,axis)`.
//! - **ANG1** (`ANG1CreateController`): single command — connects the ports
//!   and installs all `numAxes` axes immediately, matching the C constructor.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::motor::trace::MotorTrace;

use motor_common::iocsh::{
    DEFAULT_IDLE_POLL_MS, DEFAULT_MOVING_POLL_MS, arg_int_opt, arg_int_req, arg_str_req,
    poll_intervals, req_int, req_string,
};
use motor_common::{asyn_user, install, register_port};

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
            arg_str_req("Port name"),
            arg_str_req("ANF2 In port name"),
            arg_str_req("ANF2 Out port name"),
            arg_int_req("Number of axes"),
        ],
        "ANF2CreateController(portName, ANF2InPortName, ANF2OutPortName, numAxes)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let in_port = req_string(args, 1, "In port name")?;
            let out_port = req_string(args, 2, "Out port name")?;
            // Stricter than C by design: C (ANF2Driver.cpp:88-97) clamps only
            // the upper bound and lets a negative/zero numAxes through. Clamp
            // the lower bound to 0 as well. This is a defensive narrowing, not
            // a behavioral divergence — the only runtime use of num_axes is the
            // `axes_created != num_axes` poll gate, which a zero-clamped value
            // and a negative C value both leave permanently un-satisfied.
            let num_axes = req_int(args, 3, "Number of axes")?.clamp(0, MAX_AXES as i64) as usize;

            register_port(ctx, &port_name, num_axes as i32)?;
            let (controller, out_status) = Anf2Controller::new(&in_port, &out_port, num_axes);
            if out_status.is_err() {
                MotorTrace::new(port_name.clone(), 0).print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "ANF2Controller::ANF2Controller: cannot connect to ANF2 controller"
                    ),
                );
            }
            anf2_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(port_name.clone(), Arc::new(Mutex::new(controller)));
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `strtoul(hexConfig, NULL, 16)`: leading blanks and an optional `0x`,
/// then the longest run of hex digits, 0 when there is none; `None` where C's
/// `errno` is set (overflow).
fn strtoul_hex(text: &str) -> Option<u64> {
    let text = text.trim_start();
    let text = text
        .strip_prefix("0x")
        .or_else(|| text.strip_prefix("0X"))
        .unwrap_or(text);
    let end = text
        .find(|c: char| !c.is_ascii_hexdigit())
        .unwrap_or(text.len());
    match &text[..end] {
        "" => Some(0),
        digits => u64::from_str_radix(digits, 16).ok(),
    }
}

/// Build `ANF2CreateAxis(ANF2Name, axis, hexConfig, baseSpeed, homingTimeout)`,
/// printing what C's does. C starts no poller until `ANF2StartPoller`; here the
/// axis polls at the default periods from `iocInit`.
pub fn anf2_create_axis_command() -> CommandDef {
    CommandDef::new(
        "ANF2CreateAxis",
        vec![
            arg_str_req("Port name"),
            arg_int_req("Axis number"),
            arg_str_req("Hex config"),
            arg_int_req("Base speed"),
            arg_int_req("Homing timeout"),
        ],
        "ANF2CreateAxis(ANF2Name, axis, hexConfig, baseSpeed, homingTimeout)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let axis = req_int(args, 1, "Axis number")?;
            let hex_config = req_string(args, 2, "Hex config")?;
            let Some(controller) = anf2_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&port_name)
                .cloned()
            else {
                println!("ANF2MotorDriver:ANF2CreateAxis: Error port {port_name} not found");
                return Ok(CommandOutcome::Continue);
            };
            let Some(config) = strtoul_hex(&hex_config) else {
                println!("ANF2MotorDriver:ANF2CreateAxis: Error invalid config={hex_config}");
                return Ok(CommandOutcome::Continue);
            };
            println!("ANF2MotorDriver:ANF2CreateAxis: Config=0x{config:x}");
            let config = config as i32;
            let base_speed = req_int(args, 3, "Base speed")?.clamp(1, 1_000_000) as i32;
            let homing_timeout = req_int(args, 4, "Homing timeout")?.clamp(0, 300) as i32;

            let ax = Anf2Axis::new(controller, axis as i32, config, base_speed, homing_timeout);
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
            install(
                ctx,
                asyn_user(&port_name, axis as i32),
                motor,
                DEFAULT_MOVING_POLL_MS,
                DEFAULT_IDLE_POLL_MS,
                2, // C startPoller, ANF2Driver.cpp:129
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build `ANF2StartPoller(ANF2Name, movingPollPeriod, idlePollPeriod)`. The
/// axes already poll from `iocInit` at the default periods, so only the port
/// lookup and its error line are C's.
pub fn anf2_start_poller_command() -> CommandDef {
    CommandDef::new(
        "ANF2StartPoller",
        vec![
            arg_str_req("Port name"),
            arg_int_req("Moving poll period (ms)"),
            arg_int_req("Idle poll period (ms)"),
        ],
        "ANF2StartPoller(ANF2Name, movingPollPeriod, idlePollPeriod)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            if !anf2_registry()
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .contains_key(&port_name)
            {
                println!("ANF2MotorDriver:ANF2StartPoller: Error port {port_name} not found");
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Build `ANG1CreateController(portName, ANG1InPortName, ANG1OutPortName,
/// numAxes, movingPollPeriod, idlePollPeriod)`. Matches the C constructor:
/// connects the ports and installs all `numAxes` axes immediately (no
/// separate create-axis command).
pub fn ang1_create_controller_command() -> CommandDef {
    CommandDef::new(
        "ANG1CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("ANG1 In port name"),
            arg_str_req("ANG1 Out port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("Moving poll period (ms)"),
            arg_int_opt("Idle poll period (ms)"),
        ],
        "ANG1CreateController(portName, ANG1InPortName, ANG1OutPortName, numAxes, \
         movingPollPeriod, idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let in_port = req_string(args, 1, "In port name")?;
            let out_port = req_string(args, 2, "Out port name")?;
            let num_axes = req_int(args, 3, "Number of axes")?;
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

            register_port(ctx, &port_name, num_axes as i32)?;
            let (controller, out_status) = Ang1Controller::new(&in_port, &out_port);
            if out_status.is_err() {
                MotorTrace::new(port_name.clone(), 0).print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "ANG1Controller::ANG1Controller: cannot connect to ANG1 controller"
                    ),
                );
            }
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                // Infallible, matching C: axis construction ignores the
                // initial setPosition status and never aborts this command.
                let ax = Ang1Axis::new(controller.clone(), axis as i32);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                install(
                    ctx,
                    asyn_user(&port_name, axis as i32),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                    2, // C startPoller, ANG1Driver.cpp:98
                )?;
            }

            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::strtoul_hex;

    #[test]
    fn hex_config_reads_as_strtoul() {
        assert_eq!(strtoul_hex("0x86280000"), Some(0x8628_0000));
        assert_eq!(strtoul_hex(" 86zz"), Some(0x86));
        assert_eq!(strtoul_hex("zz"), Some(0));
        assert_eq!(strtoul_hex("0x1ffffffffffffffff"), None);
    }
}
