//! iocsh commands for the PMAC driver, under C's names and arguments.
//!
//! `pmacCreateController` / `pmacCreateAxis` / `pmacCreateAxes` /
//! `pmacDisableLimitsCheck` / `pmacSetOpenLoopEncoderAxis` /
//! `pmacCreateCsGroup` / `pmacCsGroupAddAxis` are pmacController.cpp's: the
//! controller is asyn port `portName` and records reach axis n as
//! `@asyn(portName,n)`. `pmacAsynCoordCreate` and the
//! `pmacSetCoord*PollPeriod` pair are pmacAsynCoordRegister.cc's model-2
//! commands: a coordinate system is card `reference` of driver
//! [`PMAC_ASYN_COORD`], which `drvAsynMotorConfigure(port, "pmacAsynCoord",
//! reference, 9)` opens as axes 0..8 (A..Z).
//!
//! `pmacSetCoordStepsPerUnit` and `pmacSetDefaultCoordSteps` are
//! pmacAsynCoord.c's, setting a coordinate-system axis's `stepSize`.
//! `pmacSetAxisScale` is not provided: the driver speaks controller counts
//! and the record's `MRES` does the scaling (see [`crate::axis`]).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::AsynError;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::motor::trace::MotorTrace;

use motor_common::iocsh::{
    arg_double_opt, arg_int_req, arg_str_req, opt_double, poll_intervals, req_string,
};
use motor_common::model1::c_int;
use motor_common::model2::Model2;
use motor_common::{Controller, asyn_user, register_controller_port};

use crate::axis::PmacAxis;
use crate::controller::{PMAC_TIMEOUT, PmacController, PmacControllerParams};
use crate::coord::{CS_AXES, PmacCoordSystem, PmacCsAxis};
use crate::ethernet::pmac_asyn_ip_configure_command;

/// How long a polled coordinate-system status (`&{cs}??`, `Q8{n}`) stays
/// fresh. One read serves every axis polling within the window, which is what
/// C's single poller thread does.
const POLL_CACHE_TTL: Duration = Duration::from_millis(50);

/// pmacAsynCoord.c's `motorAxisDrvSET_t`, its cards keyed by `reference`.
/// pmacAsynCoord.c:729's poller forces no fast polls.
pub static PMAC_ASYN_COORD: Model2 = Model2::new("pmacAsynCoord", 0);

/// `defaultMovingPollPeriod` / `defaultIdlePollPeriod` (pmacAsynCoord.c:122).
const COORD_MOVING_POLL_MS: u64 = 100;
const COORD_IDLE_POLL_MS: u64 = 500;

/// A controller and the poller `pmacCreateController` started for it.
struct ControllerReg {
    name: String,
    controller: Arc<Mutex<PmacController>>,
    poller: Controller,
}

type Registry = Arc<Mutex<Vec<ControllerReg>>>;

/// `pmacController::driverName`, the prefix of the commands' C messages.
const DRIVER_NAME: &str = "pmacController";

/// C `findAsynPortDriver` over the controllers `pmacCreateController` made.
fn find<R>(registry: &Registry, name: &str, f: impl FnOnce(&ControllerReg) -> R) -> Option<R> {
    let registry = registry.lock().unwrap_or_else(|e| e.into_inner());
    registry.iter().find(|r| r.name == name).map(f)
}

/// Which step of C's `lowLevelPortConnect` / `motorAxisAsynConnect` failed.
enum ConnectError {
    NoPort,
    InputEos(AsynError),
    OutputEos(AsynError),
}

/// Connect a [`SyncIOHandle`] to an already-configured octet port at the given
/// asyn address and set its ACK input / CR output EOS, as C
/// `lowLevelPortConnect` does (pmacController.cpp:233, pmacAsynCoord.c:280);
/// every PMAC reply ends in ACK, so without it every read times out.
fn connect_octet(port: &str, addr: i32) -> Result<SyncIOHandle, ConnectError> {
    let entry = get_port(port).ok_or(ConnectError::NoPort)?;
    let user = || AsynUser::new(0).with_addr(addr);
    entry
        .handle
        .set_input_eos_blocking(user(), b"\x06")
        .map_err(ConnectError::InputEos)?;
    entry
        .handle
        .set_output_eos_blocking(user(), b"\r")
        .map_err(ConnectError::OutputEos)?;
    Ok(SyncIOHandle::from_handle(
        entry.handle.clone(),
        addr,
        PMAC_TIMEOUT,
    ))
}

/// C `pmacController`'s constructor (pmacController.cpp:194-200) and its
/// `lowLevelPortConnect`: a missing port keeps the unconnected user, a refused
/// EOS set disconnects it (`None`); either is printed and the controller is
/// still created.
fn low_level_port_connect(self_port: &str, port: &str, addr: i32) -> Option<SyncIOHandle> {
    let trace = MotorTrace::new(self_port, 0);
    let (handle, connected) =
        SyncIOHandle::connect_port(port, addr, PMAC_TIMEOUT, InterfaceType::Octet);
    let kept = match connected.and_then(|()| handle.port()) {
        Err(_) => {
            trace.print_self(
                TraceMask::ERROR,
                format_args!(
                    "pmacController::motorAxisAsynConnect: unable to connect to port {port}\n"
                ),
            );
            Some(handle)
        }
        Ok(entry) => {
            let user = || AsynUser::new(0).with_addr(addr);
            let refused = entry
                .set_input_eos_blocking(user(), b"\x06")
                .map_err(|e| ("input", e))
                .and_then(|()| {
                    entry
                        .set_output_eos_blocking(user(), b"\r")
                        .map_err(|e| ("output", e))
                });
            let Err((which, e)) = refused else {
                return Some(handle);
            };
            trace.print_self(
                TraceMask::ERROR,
                format_args!(
                    "pmacController: unable to set {which} EOS on {port}: {}\n",
                    e.message_display()
                ),
            );
            None
        }
    };
    println!(
        "pmacController::pmacController: Failed to connect to low level \
         asynOctetSyncIO port {port}"
    );
    kept
}

/// Every PMAC iocsh command but `drvAsynMotorConfigure`, which the IOC builds
/// over [`PMAC_ASYN_COORD`] and any other model-2 driver it links.
pub fn pmac_commands() -> Vec<CommandDef> {
    let registry: Registry = Arc::new(Mutex::new(Vec::new()));
    vec![
        pmac_asyn_ip_configure_command(),
        pmac_create_controller_command(&registry),
        pmac_create_axis_command(&registry),
        pmac_create_axes_command(&registry),
        pmac_disable_limits_check_command(&registry),
        pmac_set_open_loop_encoder_axis_command(&registry),
        pmac_create_cs_group_command(&registry),
        pmac_cs_group_add_axis_command(&registry),
        pmac_asyn_coord_create_command(),
        pmac_set_coord_poll_period_command("pmacSetCoordMovingPollPeriod", "Moving poll period"),
        pmac_set_coord_poll_period_command("pmacSetCoordIdlePollPeriod", "Idle poll period"),
        pmac_set_coord_steps_per_unit_command(),
        pmac_set_default_coord_steps_command(),
    ]
}

/// C `pmacCreateController(portName, lowLevelPortName, lowLevelPortAddress,
/// numAxes, movingPollPeriod, idlePollPeriod)`: port `portName` has addresses
/// 0..=numAxes, 0 being the controller's own.
fn pmac_create_controller_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacCreateController",
        vec![
            arg_str_req("Controller port name"),
            arg_str_req("Low level port name"),
            arg_int_req("Low level port address"),
            arg_int_req("Number of axes"),
            arg_int_req("Moving poll rate (ms)"),
            arg_int_req("Idle poll rate (ms)"),
        ],
        "pmacCreateController(portName, lowLevelPortName, lowLevelPortAddress, numAxes, \
         movingPollPeriod, idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller port name")?;
            let low_level_port = req_string(args, 1, "Low level port name")?;
            let low_level_addr = c_int(args, 2);
            let num_axes = c_int(args, 3);
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 4, 5)?;

            let controller = Arc::new(Mutex::new(PmacController::new(low_level_port_connect(
                &name,
                &low_level_port,
                low_level_addr,
            ))));
            let params = PmacControllerParams::new(controller.clone());
            let owner = ctx.db().scan_control().interrupt_gate();
            let handle = register_controller_port(owner, &name, num_axes + 1, params)?;
            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .attach_port(handle);
            let polled = controller.clone();
            let poller = Controller::new().controller_poll(move || {
                polled.lock().unwrap_or_else(|e| e.into_inner()).poll();
            });
            // pmacController.cpp:201, before any axis exists.
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 10);
            registry
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .push(ControllerReg {
                    name,
                    controller,
                    poller,
                });
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Create axis `axis` of controller `reg` as `@asyn(name,axis)`.
fn create_axis(reg: &ControllerReg, command: &str, axis: i32) -> Result<(), String> {
    reg.controller
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .add_axis(axis)
        .map_err(|e| format!("{command}: {e}"))?;
    let motor: Arc<Mutex<dyn AsynMotor>> =
        Arc::new(Mutex::new(PmacAxis::new(reg.controller.clone(), axis)));
    reg.poller
        .install(asyn_user(&reg.name, axis), motor)
        .map(drop)
}

/// C `pmacCreateAxis(pmacName, axis)`, printing what C's does.
fn pmac_create_axis_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacCreateAxis",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Axis number"),
        ],
        "pmacCreateAxis(pmacName, axis)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller port name")?;
            let axis = c_int(args, 1);
            let created = find(&registry, &name, |reg| {
                if axis == 0 {
                    println!(
                        "{DRIVER_NAME}::pmacCreateAxis: ERROR Axis Number 0 Not Allowed. This \
                         Asyn Address Is Reserved For Controller Specific Parameters."
                    );
                    return Ok(());
                }
                create_axis(reg, "pmacCreateAxis", axis)
            });
            match created {
                Some(result) => result?,
                None => {
                    println!("{DRIVER_NAME}::pmacCreateAxis: ERROR Port {name} Not Found.")
                }
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacCreateAxes(pmacName, numAxes)`: axes 1..=numAxes, axis 0 being the
/// controller's own asyn address.
fn pmac_create_axes_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacCreateAxes",
        vec![arg_str_req("Controller port name"), arg_int_req("Num Axes")],
        "pmacCreateAxes(pmacName, numAxes)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller port name")?;
            let num_axes = c_int(args, 1);
            let created = find(&registry, &name, |reg| {
                (1..=num_axes).try_for_each(|axis| create_axis(reg, "pmacCreateAxes", axis))
            });
            match created {
                Some(result) => result?,
                // C's functionName here is "pmacCreateAxis" (pmacController.cpp:964).
                None => println!("{DRIVER_NAME}:pmacCreateAxis: Error port {name} not found"),
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// The `Error port %s not found` every per-controller command prints.
fn port_not_found(function: &str, name: &str) {
    println!("{DRIVER_NAME}:{function}: Error port {name} not found");
}

/// C `pmacDisableLimitsCheck(controller, axis, allAxes)`: allAxes 1 is every
/// axis, 0 is `axis`, anything else does nothing.
fn pmac_disable_limits_check_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacDisableLimitsCheck",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Axis number"),
            arg_int_req("All Axes"),
        ],
        "pmacDisableLimitsCheck(controller, axis, allAxes)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller port name")?;
            let axis = c_int(args, 1);
            let all_axes = c_int(args, 2);
            let done = find(&registry, &name, |reg| {
                let mut controller = reg.controller.lock().unwrap_or_else(|e| e.into_inner());
                match all_axes {
                    1 => {
                        controller.disable_limits_check_all();
                        Ok(())
                    }
                    0 => controller
                        .disable_limits_check(axis)
                        .map_err(|e| format!("pmacDisableLimitsCheck: {e}")),
                    _ => Ok(()),
                }
            });
            match done {
                Some(result) => result?,
                None => port_not_found("pmacDisableLimitsCheck", &name),
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacSetOpenLoopEncoderAxis(controller, axis, encoder_axis)`.
fn pmac_set_open_loop_encoder_axis_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacSetOpenLoopEncoderAxis",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Axis number"),
            arg_int_req("Encoder Axis"),
        ],
        "pmacSetOpenLoopEncoderAxis(controller, axis, encoder_axis)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller port name")?;
            let axis = c_int(args, 1);
            let encoder_axis = c_int(args, 2);
            let done = find(&registry, &name, |reg| {
                reg.controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .set_open_loop_encoder_axis(axis, encoder_axis, 1.0)
                    .map_err(|e| format!("pmacSetOpenLoopEncoderAxis: {e}"))
            });
            match done {
                Some(result) => result?,
                None => port_not_found("pmacSetOpenLoopEncoderAxis", &name),
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacCreateCsGroup(controller, groupNo, groupName, axisCount)`.
/// `axisCount` only pre-sizes C's map, so it is unused.
fn pmac_create_cs_group_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacCreateCsGroup",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Group number"),
            arg_str_req("Group name"),
            arg_int_req("Axis count"),
        ],
        "pmacCreateCsGroup(controller, groupNo, groupName, axisCount)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller port name")?;
            let group = c_int(args, 1);
            let group_name = req_string(args, 2, "Group name")?;
            let found = find(&registry, &name, |reg| {
                reg.controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .cs_groups_mut()
                    .add_group(group, &group_name)
            });
            if found.is_none() {
                port_not_found("pmacCreateCsGroup", &name);
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacCsGroupAddAxis(controller, groupNo, axisNo, mapping, coordinateSysNo)`.
fn pmac_cs_group_add_axis_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacCsGroupAddAxis",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Group number"),
            arg_int_req("Axis number"),
            arg_str_req("Axis CS definition"),
            arg_int_req("CS number"),
        ],
        "pmacCsGroupAddAxis(controller, groupNo, axisNo, mapping, coordinateSysNo)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller port name")?;
            let group = c_int(args, 1);
            let axis = c_int(args, 2);
            let axis_def = req_string(args, 3, "Axis CS definition")?;
            let cs = c_int(args, 4);
            let done = find(&registry, &name, |reg| {
                reg.controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .cs_groups_mut()
                    .add_axis(group, axis, &axis_def, cs)
                    .map_err(|e| format!("pmacCsGroupAddAxis: {e}"))
            });
            match done {
                Some(result) => result?,
                None => port_not_found("pmacCsGroupAddAxis", &name),
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// What each `pmacAsynCoordCreate` made: `(reference, port, address, cs)`
/// and the system.
type CoordSystem = (i32, String, i32, i32, Arc<Mutex<PmacCoordSystem>>);
static COORD_SYSTEMS: Mutex<Vec<CoordSystem>> = Mutex::new(Vec::new());

/// C `pmacAsynCoordCreate(port, addr, cs, ref, program)`: card `ref` of
/// [`PMAC_ASYN_COORD`], its nine axes polled at C's default periods until a
/// `pmacSetCoord*PollPeriod` changes them.
fn pmac_asyn_coord_create_command() -> CommandDef {
    CommandDef::new(
        "pmacAsynCoordCreate",
        vec![
            arg_str_req("port"),
            arg_int_req("address"),
            arg_int_req("C.S. no"),
            arg_int_req("reference"),
            arg_int_req("program"),
        ],
        "pmacAsynCoordCreate(port, address, C.S. no, reference, program)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            coord_create(
                &req_string(args, 0, "port")?,
                c_int(args, 1),
                c_int(args, 2),
                c_int(args, 3),
                c_int(args, 4),
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacAsynCoordCreate`'s body. A refused connect is printed as C's
/// `motorAxisAsynConnect` prints it and creates no driver.
fn coord_create(
    port: &str,
    addr: i32,
    cs: i32,
    reference: i32,
    program: i32,
) -> Result<(), String> {
    if !(1..=16).contains(&cs) {
        println!("Invalid co-ordinate system number: {cs}");
        return Ok(());
    }
    let mut systems = COORD_SYSTEMS.lock().unwrap_or_else(|e| e.into_inner());
    if systems
        .iter()
        .any(|(r, p, a, c, _)| *r == reference || (p == port && *a == addr && *c == cs))
    {
        println!("C.S. reference {reference} already exists");
        return Ok(());
    }
    // drvPrint with no log parameter prints whatever the mask.
    println!(
        "Creating PMAC co-ordinate system driver on port {port}, address {addr}, cs: {cs} \
         (ref = {reference})"
    );
    let trace = MotorTrace::new(port, addr);
    let handle = match connect_octet(port, addr) {
        Ok(handle) => handle,
        Err(ConnectError::NoPort) => {
            println!("drvPmacCreate: unable to connect to port {port}");
            return Ok(());
        }
        Err(ConnectError::InputEos(e)) => {
            trace.print(
                TraceMask::ERROR,
                0,
                format_args!(
                    "drvPmacCreate: unable to set input EOS on {port}: {}\n",
                    e.message_display()
                ),
            );
            return Ok(());
        }
        Err(ConnectError::OutputEos(e)) => {
            trace.print(
                TraceMask::ERROR,
                0,
                format_args!(
                    "drvPmacCreate: unable to set output EOS on {port}: {}\n",
                    e.message_display()
                ),
            );
            return Ok(());
        }
    };
    let system = Arc::new(Mutex::new(PmacCoordSystem::new(
        handle,
        cs,
        program,
        POLL_CACHE_TTL,
    )));
    let axes = (1..=CS_AXES)
        .map(|axis| {
            let motor: Arc<Mutex<dyn AsynMotor>> =
                Arc::new(Mutex::new(PmacCsAxis::new(system.clone(), axis)?));
            trace.print(
                TraceMask::FLOW,
                0,
                format_args!(
                    "Created motor for ref {reference}, C.S. {cs}, signal {} OK\n",
                    axis - 1
                ),
            );
            Ok(motor)
        })
        .collect::<Result<Vec<_>, String>>()?;
    PMAC_ASYN_COORD.set_card(reference, axes, COORD_MOVING_POLL_MS, COORD_IDLE_POLL_MS);
    systems.push((reference, port.to_string(), addr, cs, system));
    Ok(())
}

/// C `pmacSetCoordMovingPollPeriod` / `pmacSetCoordIdlePollPeriod(ref,
/// period)`, silent as C's are.
fn pmac_set_coord_poll_period_command(name: &'static str, period: &'static str) -> CommandDef {
    let moving = name == "pmacSetCoordMovingPollPeriod";
    CommandDef::new(
        name,
        vec![arg_int_req("CS number"), arg_int_req(period)],
        format!("{name}(ref, {period})"),
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let reference = c_int(args, 0);
            let ms = Some(c_int(args, 1).max(0) as u64);
            let (moving_ms, idle_ms) = if moving { (ms, None) } else { (None, ms) };
            PMAC_ASYN_COORD.set_poll_periods(reference, moving_ms, idle_ms);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// The `drvPrint` refusal both step commands share (pmacAsynCoord.c:883).
const STEPS_NOT_POSITIVE: &str = "ERROR: Scaling factor has to be positive.";

/// C `pmacSetCoordStepsPerUnit(ref, axis, stepsPerUnit)`: `stepSize` of
/// 0-based `axis` on coordinate system `ref`.
fn pmac_set_coord_steps_per_unit_command() -> CommandDef {
    CommandDef::new(
        "pmacSetCoordStepsPerUnit",
        vec![
            arg_int_req("CS Ref"),
            arg_int_req("Axis number"),
            arg_double_opt("Steps per unit"),
        ],
        "pmacSetCoordStepsPerUnit(CS Ref, Axis number, Steps per unit)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let steps = opt_double(args, 2, 0.0, "Steps per unit")?;
            set_coord_steps_per_unit(c_int(args, 0), c_int(args, 1), steps);
            Ok(CommandOutcome::Continue)
        },
    )
}

fn set_coord_steps_per_unit(reference: i32, axis: i32, steps_per_unit: f64) {
    if steps_per_unit <= 0.0 {
        println!("{STEPS_NOT_POSITIVE}");
        return;
    }
    let Some(axis) = usize::try_from(axis).ok().filter(|a| *a < CS_AXES) else {
        println!(
            "ERROR: axis number has to be between 0 and {}.",
            CS_AXES - 1
        );
        return;
    };
    let systems = COORD_SYSTEMS.lock().unwrap_or_else(|e| e.into_inner());
    for (.., system) in systems.iter().filter(|(r, ..)| *r == reference) {
        lock_system(system).set_steps_per_unit(axis, steps_per_unit);
        println!("  Setting axis {axis} scaling factor to {steps_per_unit:.6}.");
    }
}

/// C `pmacSetDefaultCoordSteps(defaultSteps)`: `stepSize` of every axis of
/// every coordinate system made so far.
fn pmac_set_default_coord_steps_command() -> CommandDef {
    CommandDef::new(
        "pmacSetDefaultCoordSteps",
        vec![arg_double_opt("Default Steps")],
        "pmacSetDefaultCoordSteps(Default Steps)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            set_default_coord_steps(opt_double(args, 0, 0.0, "Default Steps")?);
            Ok(CommandOutcome::Continue)
        },
    )
}

fn set_default_coord_steps(default_steps: f64) {
    if default_steps <= 0.0 {
        println!("{STEPS_NOT_POSITIVE}");
        return;
    }
    let systems = COORD_SYSTEMS.lock().unwrap_or_else(|e| e.into_inner());
    for (reference, .., system) in systems.iter() {
        let mut system = lock_system(system);
        for axis in 0..CS_AXES {
            system.set_steps_per_unit(axis, default_steps);
        }
        println!("  Set scaling factor to {default_steps:.6} for CS ref {reference}.");
    }
}

fn lock_system(system: &Mutex<PmacCoordSystem>) -> std::sync::MutexGuard<'_, PmacCoordSystem> {
    system.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use std::sync::atomic::{AtomicUsize, Ordering};

    use epics_rs::asyn::error::{AsynResult, AsynStatus};
    use epics_rs::asyn::manager::PortManager;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{PortRuntimeHandle, RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::trace::TraceFile;

    /// A port with no EOS support of its own: both sets are refused.
    struct Bare(PortDriverBase);

    impl PortDriver for Bare {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
    }

    fn publish_bare(name: &str) -> PortRuntimeHandle {
        let base = PortDriverBase::new(name, 1, PortFlags::default());
        let claim = claim_port(name).unwrap();
        let (rt, _actor) =
            create_port_runtime(&claim, Bare(base), RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        rt
    }

    fn coord_exists(reference: i32) -> bool {
        COORD_SYSTEMS
            .lock()
            .unwrap()
            .iter()
            .any(|(r, ..)| *r == reference)
    }

    #[test]
    fn a_refused_eos_set_creates_no_coordinate_system() {
        let _rt = publish_bare("pmacCoordEosRefused");
        coord_create("pmacCoordEosRefused", 0, 2, 9101, 10).unwrap();
        assert!(!coord_exists(9101));
    }

    /// pmacAsynCoord.c pmacSetCoordStepsPerUnit / pmacSetDefaultCoordSteps:
    /// a non-positive factor or an axis past 8 changes nothing, the default
    /// reaches every axis of every system.
    #[test]
    fn step_commands_set_step_size_as_c() {
        let rt = publish_bare("pmacCoordSteps");
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::ZERO);
        let system = Arc::new(Mutex::new(PmacCoordSystem::new(io, 3, 0, POLL_CACHE_TTL)));
        COORD_SYSTEMS
            .lock()
            .unwrap()
            .push((9103, "pmacCoordSteps".into(), 0, 3, system.clone()));
        let step = |axis| lock_system(&system).step_size(axis);
        assert_eq!(step(4), 1e-4);
        set_coord_steps_per_unit(9103, 4, 0.0);
        set_coord_steps_per_unit(9103, 9, 50.0);
        set_coord_steps_per_unit(9104, 4, 50.0);
        assert_eq!(step(4), 1e-4);
        set_coord_steps_per_unit(9103, 4, 50.0);
        assert_eq!((step(4), step(5)), (0.02, 1e-4));
        set_default_coord_steps(-1.0);
        assert_eq!(step(5), 1e-4);
        set_default_coord_steps(1000.0);
        assert_eq!((step(0), step(4), step(8)), (0.001, 0.001, 0.001));
    }

    #[test]
    fn a_missing_port_creates_no_coordinate_system() {
        coord_create("pmacCoordNoSuchPort", 0, 2, 9102, 10).unwrap();
        assert!(!coord_exists(9102));
    }

    fn controller_on(port: &str) -> PmacController {
        PmacController::new(low_level_port_connect("pmacTest", port, 0))
    }

    #[test]
    fn a_refused_eos_set_leaves_a_controller_whose_commands_fail() {
        let name = "pmacEosRefused";
        let base = PortDriverBase::new(name, 1, PortFlags::default());
        let claim = claim_port(name).unwrap();
        let (rt, _actor) =
            create_port_runtime(&claim, Bare(base), RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        assert!(controller_on(name).write_read("I100").is_err());
        drop(rt);
    }

    /// A PMAC that answers the global-status query and, once, the
    /// initial-status query with `initial`.
    struct GlobalOnly {
        base: PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
        reply: Vec<u8>,
        initial: Vec<u8>,
    }

    impl PortDriver for GlobalOnly {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn set_input_eos(&mut self, _: &AsynUser, _: &[u8]) -> AsynResult<()> {
            Ok(())
        }
        fn set_output_eos(&mut self, _: &AsynUser, _: &[u8]) -> AsynResult<()> {
            Ok(())
        }
        fn io_write_octet(&mut self, _: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            self.reply = if cmd == "???" {
                b"000000000000".to_vec()
            } else {
                std::mem::take(&mut self.initial)
            };
            self.written.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    /// pmacController.cpp:594-625: the global status is the controller
    /// poll's, which the poller runs before every axis; the axis reads none.
    #[test]
    fn the_controller_poll_reads_the_global_status_and_the_axis_does_not() {
        let name = "pmacGlobalFirst";
        let written = Arc::new(Mutex::new(Vec::new()));
        let driver = GlobalOnly {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            written: written.clone(),
            reply: Vec::new(),
            initial: Vec::new(),
        };
        let claim = claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        let controller = Arc::new(Mutex::new(controller_on(name)));
        let mut axis = PmacAxis::new(controller.clone(), 1);
        assert_eq!(
            std::mem::take(&mut *written.lock().unwrap()),
            ["I113 I114 I130 I131 I133"]
        );
        assert!(!axis.has_encoder);
        controller.lock().unwrap().poll();
        assert_eq!(std::mem::take(&mut *written.lock().unwrap()), ["???"]);
        assert!(axis.poll(&AsynUser::new(0).with_addr(1)).is_err());
        assert_eq!(*written.lock().unwrap(), ["#1 ? F P"]);
        drop(rt);
    }

    /// pmacAxis.cpp:133-145: only a parsed initial status sets HasEncoder.
    #[test]
    fn a_parsed_initial_status_reports_an_encoder() {
        let name = "pmacInitialStatus";
        let driver = GlobalOnly {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            written: Arc::new(Mutex::new(Vec::new())),
            reply: Vec::new(),
            initial: b"100\r-100\r2000\r1000\r500\r\x06".to_vec(),
        };
        let claim = claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        let controller = Arc::new(Mutex::new(controller_on(name)));
        assert!(PmacAxis::new(controller, 1).has_encoder);
        drop(rt);
    }

    #[test]
    fn a_missing_low_level_port_leaves_a_controller_whose_commands_fail() {
        // C keeps the unconnected user; only a refused EOS set clears it.
        assert!(low_level_port_connect("pmacTest", "pmacNoSuchPort", 0).is_some());
        assert!(controller_on("pmacNoSuchPort").write_read("I100").is_err());
    }

    #[test]
    fn a_refused_eos_set_prints_through_the_controller_trace() {
        let _self_rt = publish_bare("pmacTraceSelf");
        let _low_rt = publish_bare("pmacTraceLow");
        let path = std::env::temp_dir().join(format!("pmacTraceSelf-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        PortManager::global().trace_manager().set_trace_file(
            Some("pmacTraceSelf"),
            TraceFile::File(Arc::new(Mutex::new(file))),
        );
        assert!(low_level_port_connect("pmacTraceSelf", "pmacTraceLow", 0).is_none());
        let trace = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            trace.contains("pmacController: unable to set input EOS on pmacTraceLow: "),
            "{trace:?}"
        );
    }

    /// A PMAC that answers the commands in `replies` and times out on any
    /// other, counting the disconnects its port is asked for.
    struct Scripted {
        base: PortDriverBase,
        rig: Rig,
        reply: Option<Vec<u8>>,
    }

    #[derive(Clone, Default)]
    struct Rig {
        replies: Arc<Mutex<HashMap<String, Vec<u8>>>>,
        written: Arc<Mutex<Vec<String>>>,
        disconnects: Arc<AtomicUsize>,
    }

    impl Rig {
        fn answer(&self, command: &str, reply: &[u8]) {
            let mut replies = self.replies.lock().unwrap();
            replies.insert(command.into(), reply.into());
        }
        fn silence(&self, command: &str) {
            self.replies.lock().unwrap().remove(command);
        }
        fn written(&self) -> Vec<String> {
            std::mem::take(&mut *self.written.lock().unwrap())
        }
        fn disconnects(&self) -> usize {
            self.disconnects.load(Ordering::SeqCst)
        }
    }

    impl PortDriver for Scripted {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn set_input_eos(&mut self, _: &AsynUser, _: &[u8]) -> AsynResult<()> {
            Ok(())
        }
        fn set_output_eos(&mut self, _: &AsynUser, _: &[u8]) -> AsynResult<()> {
            Ok(())
        }
        fn disconnect(&mut self, _: &AsynUser) -> AsynResult<()> {
            self.rig.disconnects.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }
        fn io_write_octet(&mut self, _: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            self.reply = self.rig.replies.lock().unwrap().get(&cmd).cloned();
            self.rig.written.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = self.reply.take().ok_or_else(|| AsynError::Status {
                status: AsynStatus::Timeout,
                message: "timeout".into(),
            })?;
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    fn publish_scripted(name: &str) -> (PortRuntimeHandle, Rig) {
        let rig = Rig::default();
        let driver = Scripted {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            rig: rig.clone(),
            reply: None,
        };
        let claim = claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        (rt, rig)
    }

    /// pmacController.cpp:335-343: a failed transfer disconnects the port; a
    /// reply the PMAC rejected is a transfer that worked.
    #[test]
    fn a_failed_transfer_disconnects_the_low_level_port() {
        let (_rt, rig) = publish_scripted("pmacDisconnect");
        rig.answer("I100", b"\x07ERR003\r");
        let controller = controller_on("pmacDisconnect");
        assert!(controller.write_read("I100").is_err());
        assert_eq!(rig.disconnects(), 0);
        assert!(controller.write_read("I200").is_err());
        assert_eq!(rig.disconnects(), 1);
    }

    /// pmacController.cpp:594-708: an unread global status is 0, the feed
    /// rate is still read after a failed `???`, and the feed-rate read's
    /// verdict is the comms error.
    #[test]
    fn a_failed_global_status_read_clears_the_global_problem() {
        let (_rt, rig) = publish_scripted("pmacGlobalLow");
        rig.answer("???", b"008000\r");
        rig.answer("%", b"100\r");
        let controller = Arc::new(Mutex::new(controller_on("pmacGlobalLow")));
        let params = PmacControllerParams::new(controller.clone());
        let port = epics_rs::motor::devices::register_controller_port(
            &open_owner(),
            "pmacGlobalCtl",
            2,
            params,
        )
        .unwrap();
        controller.lock().unwrap().attach_port(port.clone());
        let io = SyncIOHandle::from_handle(port, 0, Duration::from_secs(1));
        let read = |name| io.read_int32(io.drv_user_create(name).unwrap()).unwrap();
        let posted = || {
            (
                read("PMAC_C_GLOBALSTATUS"),
                read("PMAC_C_COMMSERROR"),
                read("PMAC_C_FEEDRATE"),
            )
        };
        let poll = || controller.lock().unwrap().poll();
        let problem = || controller.lock().unwrap().controller_problem();
        let feed_rate_poll = io.drv_user_create("PMAC_C_FEEDRATE_POLL").unwrap();
        io.write_int32(feed_rate_poll, 1).unwrap();

        poll();
        assert_eq!(rig.written(), ["???", "%"]);
        assert!(problem());
        assert_eq!(posted(), (1, 0, 100));

        rig.silence("???");
        rig.answer("%", b"50\r");
        poll();
        assert_eq!(rig.written(), ["???", "%"]);
        assert!(!problem());
        assert_eq!(posted(), (0, 0, 50));

        rig.answer("???", b"008000\r");
        rig.silence("%");
        poll();
        assert_eq!(posted(), (1, 1, 50));
    }

    /// pmacController.cpp:201: one poller from pmacCreateController, which
    /// pmacCreateAxes joins.
    #[epics_rs::base::epics_test]
    async fn pmac_create_axes_joins_the_controller_poller() {
        use motor_common::testing::pollers_after;

        let commands = pmac_commands();
        let command = |name: &str| commands.iter().find(|c| c.name == name).unwrap().clone();
        let args = vec![
            ArgValue::String("pmacOnePoller".into()),
            ArgValue::String("pmacOnePollerNoLine".into()),
            ArgValue::Int(0),
            ArgValue::Int(3),
            ArgValue::Int(200),
            ArgValue::Int(1000),
        ];
        assert_eq!(
            pollers_after(command("pmacCreateController"), args).await,
            1
        );
        let args = vec![ArgValue::String("pmacOnePoller".into()), ArgValue::Int(3)];
        assert_eq!(pollers_after(command("pmacCreateAxes"), args).await, 1);
    }
}
