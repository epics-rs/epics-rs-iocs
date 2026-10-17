//! iocsh commands for the PMAC driver, under C's names and arguments.
//!
//! `pmacCreateController` / `pmacCreateAxis` / `pmacCreateAxes` /
//! `pmacDisableLimitsCheck` / `pmacSetAxisScale` / `pmacSetOpenLoopEncoderAxis` /
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

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
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
use crate::coord::{CS_AXES, PmacCoordSystem, PmacCsAxis, axis_name};
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
        pmac_set_axis_scale_command(&registry),
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

            let controller = Arc::new(Mutex::new(PmacController::new(
                &name,
                &low_level_port,
                low_level_addr,
                low_level_port_connect(&name, &low_level_port, low_level_addr),
            )));
            let params =
                PmacControllerParams::new(controller.clone(), moving_poll_ms, idle_poll_ms);
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
                    1 => controller.disable_limits_check_all(),
                    0 => controller.disable_limits_check(axis),
                    _ => {}
                }
            });
            if done.is_none() {
                port_not_found("pmacDisableLimitsCheck", &name);
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacSetAxisScale(controller, axis, scale)`.
fn pmac_set_axis_scale_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacSetAxisScale",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Axis number"),
            arg_int_req("Scale"),
        ],
        "pmacSetAxisScale(controller, axis, scale)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller port name")?;
            let axis = c_int(args, 1);
            let scale = c_int(args, 2);
            let done = find(&registry, &name, |reg| {
                reg.controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .set_axis_scale(axis, scale)
            });
            if done.is_none() {
                port_not_found("pmacSetAxisScale", &name);
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
            });
            if done.is_none() {
                port_not_found("pmacSetOpenLoopEncoderAxis", &name);
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

/// What one `pmacAsynCoordCreate` made, C's `drvPmac_t`.
struct CoordSystem {
    reference: i32,
    port: String,
    addr: i32,
    cs: i32,
    system: Arc<Mutex<PmacCoordSystem>>,
    /// Each axis's lock, whose address C's report prints as `axisMutex`.
    axes: Vec<Arc<Mutex<dyn AsynMotor>>>,
}

/// C's `pFirstDrv` list, in creation order.
static COORD_SYSTEMS: Mutex<Vec<CoordSystem>> = Mutex::new(Vec::new());

/// pmacAsynCoord.c `motorAxisReport` (:164-186).
fn coord_report(level: i32) -> String {
    let mut out = String::new();
    for c in COORD_SYSTEMS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
    {
        out += &format!(
            "Found driver for PMAC C.S. {} via {} (ref {})\n",
            c.cs, c.port, c.reference
        );
        if level > 0 {
            for (axis, motor) in c.axes.iter().enumerate() {
                let name = axis_name(axis + 1).unwrap_or(' ');
                out += &format!(
                    "axis {name}:\ndrvPmac->axisMutex = {:p}\n",
                    Arc::as_ptr(motor).cast::<()>()
                );
            }
        }
    }
    out
}

/// C `pmacAsynCoordCreate(port, addr, cs, ref, program)`: card `ref` of
/// [`PMAC_ASYN_COORD`], its nine axes polled at C's default periods until a
/// `pmacSetCoord*PollPeriod` changes them.
fn pmac_asyn_coord_create_command() -> CommandDef {
    PMAC_ASYN_COORD.register_report(coord_report);
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
        .any(|c| c.reference == reference || (c.port == port && c.addr == addr && c.cs == cs))
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
        MotorTrace::new(port, addr),
        reference,
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
    PMAC_ASYN_COORD.set_card(
        reference,
        axes.clone(),
        COORD_MOVING_POLL_MS,
        COORD_IDLE_POLL_MS,
    );
    systems.push(CoordSystem {
        reference,
        port: port.to_string(),
        addr,
        cs,
        system,
        axes,
    });
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
    for c in systems.iter().filter(|c| c.reference == reference) {
        lock_system(&c.system).set_steps_per_unit(axis, steps_per_unit);
        println!(
            "  Setting axis {axis} scaling factor to {steps_per_unit:.6}.",
            steps_per_unit = CFloat(steps_per_unit)
        );
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
    for c in systems.iter() {
        let mut system = lock_system(&c.system);
        for axis in 0..CS_AXES {
            system.set_steps_per_unit(axis, default_steps);
        }
        println!(
            "  Set scaling factor to {default_steps:.6} for CS ref {}.",
            c.reference,
            default_steps = CFloat(default_steps)
        );
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
    use epics_rs::asyn::interfaces::motor::MotorStatus;
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
            .any(|c| c.reference == reference)
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
        let system = Arc::new(Mutex::new(PmacCoordSystem::new(
            io,
            MotorTrace::default(),
            9103,
            3,
            0,
            POLL_CACHE_TTL,
        )));
        COORD_SYSTEMS.lock().unwrap().push(CoordSystem {
            reference: 9103,
            port: "pmacCoordSteps".into(),
            addr: 0,
            cs: 3,
            system: system.clone(),
            axes: Vec::new(),
        });
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

    /// pmacAsynCoord.c `motorAxisReport` (:164-186): each system in
    /// creation order, its nine axes above level 0.
    #[test]
    fn dbior_prints_each_coordinate_system() {
        use motor_common::testing::driver_report;
        let _cmd = pmac_asyn_coord_create_command();
        let (_a, _) = publish_scripted("pmacCoordRepA");
        let (_b, _) = publish_scripted("pmacCoordRepB");
        coord_create("pmacCoordRepA", 0, 2, 9201, 10).unwrap();
        coord_create("pmacCoordRepB", 0, 5, 9202, 10).unwrap();
        let found = "Found driver for PMAC C.S. 2 via pmacCoordRepA (ref 9201)\n\
                     Found driver for PMAC C.S. 5 via pmacCoordRepB (ref 9202)\n";
        assert_eq!(driver_report("pmacAsynCoord", 0), found);
        let text = driver_report("pmacAsynCoord", 1);
        let names: Vec<_> = text
            .lines()
            .filter_map(|l| l.strip_prefix("axis "))
            .collect();
        assert_eq!(names.len(), 18, "{text}");
        assert_eq!(
            names[..9],
            ["A:", "B:", "C:", "U:", "V:", "W:", "X:", "Y:", "Z:"]
        );
        assert!(text.starts_with("Found driver for PMAC C.S. 2 via pmacCoordRepA (ref 9201)\naxis A:\ndrvPmac->axisMutex = 0x"), "{text}");
    }

    #[test]
    fn a_missing_port_creates_no_coordinate_system() {
        coord_create("pmacCoordNoSuchPort", 0, 2, 9102, 10).unwrap();
        assert!(!coord_exists(9102));
    }

    fn controller_on(port: &str) -> PmacController {
        PmacController::new(
            "pmacTest",
            port,
            0,
            low_level_port_connect("pmacTest", port, 0),
        )
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
        assert!(axis.poll(&AsynUser::new(0).with_addr(1)).is_ok());
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

    /// pmacController.cpp:355-375: the head, then axis 0 and every created
    /// axis, before the base report.
    #[test]
    fn port_report_prints_the_controller_then_each_axis() {
        let (_rt, _rig) = publish_scripted("pmacRepLow");
        let controller = Arc::new(Mutex::new(controller_on("pmacRepLow")));
        let params = PmacControllerParams::new(controller.clone(), 200, 3000);
        let port = epics_rs::motor::devices::register_controller_port(
            &open_owner(),
            "pmacRepCtl",
            4,
            params,
        )
        .unwrap();
        controller.lock().unwrap().attach_port(port);
        controller.lock().unwrap().add_axis(2).unwrap();
        let head = "pmac motor driver pmacRepCtl, numAxes=4, moving poll period=0.200000, \
                    idle poll period=3.000000\n";
        let text = motor_common::testing::port_report("pmacRepCtl", 0);
        assert!(text.contains(head), "{text}");
        assert!(!text.contains("scale"), "{text}");
        controller.lock().unwrap().set_axis_scale(2, 3);
        let text = motor_common::testing::port_report("pmacRepCtl", 1);
        let axes = "  axis 0\n    scale = 1\n  axis 2\n    scale = 3\n";
        assert!(text.contains(&format!("{head}{axes}")), "{text}");
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
        let params = PmacControllerParams::new(controller.clone(), 100, 1000);
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

    /// pmacAxis.cpp:494-498, 606-613: a failed status read keeps the status
    /// but the power-on bit, read from zeroed words, and is no failure.
    #[test]
    fn a_failed_axis_status_read_clears_only_power_on() {
        let (_rt, rig) = publish_scripted("pmacAxisSilent");
        rig.answer("#1 ? F P", b"882000000001\r0\r100.0\r");
        rig.answer("i124", b"$0\r");
        let controller = Arc::new(Mutex::new(controller_on("pmacAxisSilent")));
        let mut axis = PmacAxis::new(controller, 1);
        let user = AsynUser::new(0).with_addr(1);
        let read = axis.poll(&user).unwrap();
        assert!(read.powered);
        assert_eq!(read.position, 100.0);

        rig.silence("#1 ? F P");
        let failed = axis.poll(&user).unwrap();
        assert_eq!(
            failed,
            MotorStatus {
                powered: false,
                moving: false,
                ..read
            }
        );
    }

    /// pmacAxis.cpp:578-579: a failed `i{n}24` read skips the limits check.
    #[test]
    fn a_failed_limits_read_is_no_poll_failure() {
        let (_rt, rig) = publish_scripted("pmacIx24Silent");
        rig.answer("#1 ? F P", b"882000000001\r0\r100.0\r");
        let controller = Arc::new(Mutex::new(controller_on("pmacIx24Silent")));
        let mut axis = PmacAxis::new(controller, 1);
        let status = axis.poll(&AsynUser::new(0).with_addr(1)).unwrap();
        assert!(status.powered && !status.problem);
    }

    /// pmacAsynCoord.c:532-543: home and jog print their refusal on the
    /// coordinate system's trace.
    #[test]
    fn cs_home_and_jog_print_c_refusals() {
        let (rt, _rig) = publish_scripted("pmacCsRefuse");
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let trace = MotorTrace::new("pmacCsRefuse", 0);
        let system = Arc::new(Mutex::new(PmacCoordSystem::new(
            io,
            trace,
            9105,
            2,
            0,
            POLL_CACHE_TTL,
        )));
        let mut axis = PmacCsAxis::new(system, 1).unwrap();
        let own = TraceCapture::on("pmacCsRefuse");
        let user = AsynUser::new(0);
        assert!(axis.home(&user, 0.0, 1.0, 1.0, true).is_err());
        assert!(axis.move_velocity(&user, 0.0, 1.0, 1.0).is_err());
        assert_eq!(
            own.take(),
            "motorAxisHome: not implemented for CS axes\n\
             motorAxisVelocityMove: not implemented for CS axes\n"
        );
    }

    /// pmacAsynCoord.c:335-342: a failed transfer prints the C.S. line with
    /// the axis that sent it; a BELL reply is a transfer that worked.
    #[test]
    fn a_failed_cs_transfer_prints_the_c_line() {
        let (rt, rig) = publish_scripted("pmacCsRw");
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let trace = MotorTrace::new("pmacCsRw", 0);
        let system = Arc::new(Mutex::new(PmacCoordSystem::new(
            io,
            trace,
            9106,
            2,
            0,
            POLL_CACHE_TTL,
        )));
        let mut axis = PmacCsAxis::new(system, 4).unwrap();
        let own = TraceCapture::on("pmacCsRw");
        let user = AsynUser::new(0);
        rig.silence("&2A Q74=Q84");
        assert!(axis.stop(&user, 1.0).is_err());
        let text = own.take();
        assert!(
            text.starts_with(
                "Read/write error to PMAC C.S. ref 9106, axis U command &2A Q74=Q84. Status=1, Error="
            ),
            "{text}"
        );
        rig.answer("&2A Q74=Q84", b"\x07ERR003\r");
        assert!(axis.stop(&user, 1.0).is_err());
        assert_eq!(own.take(), "");
    }

    /// pmacAsynCoord.c:584-592: an unparsed or failed `&{cs}??` prints
    /// cmdStatus (0, or MOTOR_AXIS_ERROR) and the response buffer.
    #[test]
    fn a_bad_cs_status_read_prints_the_c_line() {
        let (rt, rig) = publish_scripted("pmacCsStat");
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let trace = MotorTrace::new("pmacCsStat", 0);
        let system = Arc::new(Mutex::new(PmacCoordSystem::new(
            io,
            trace,
            9107,
            2,
            0,
            Duration::ZERO,
        )));
        let mut axis = PmacCsAxis::new(system, 1).unwrap();
        let own = TraceCapture::on("pmacCsStat");
        let user = AsynUser::new(0);
        rig.answer("&2??", b"zz\r");
        assert!(axis.poll(&user).is_err());
        assert_eq!(
            own.take(),
            "drvPmacAxisGetStatus: not all status values returned. Status: 0\n\
             Command :&2??\nResponse:zz\r"
        );
        rig.silence("&2??");
        assert!(axis.poll(&user).is_err());
        let text = own.take();
        assert!(
            text.ends_with(
                "drvPmacAxisGetStatus: not all status values returned. Status: -1\n\
                 Command :&2??\nResponse:"
            ),
            "{text}"
        );
    }

    /// pmacController.cpp:457/463 `%f`: glibc's nan, sign bit included.
    #[test]
    fn a_nan_soft_limit_is_written_as_c_prints_it() {
        let (_rt, rig) = publish_scripted("pmacLimNan");
        let controller = Arc::new(Mutex::new(controller_on("pmacLimNan")));
        let mut axis = PmacAxis::new(controller, 1);
        let user = AsynUser::new(0).with_addr(1);
        let _ = axis.set_high_limit(&user, f64::NAN);
        let _ = axis.set_low_limit(&user, -f64::NAN);
        assert_eq!(rig.written()[1..], ["I113=nan", "I114=-nan"]);
    }

    /// pmacAxis.cpp:178/188/330/509, pmacController.cpp:408/457/463: an axis
    /// scale divides every demand on the wire and multiplies the readback.
    #[test]
    fn an_axis_scale_divides_demands_and_multiplies_readbacks() {
        let (_rt, rig) = publish_scripted("pmacScale");
        rig.answer("#1 ? F P", b"882000000001\r0\r100.0\r");
        rig.answer("i124", b"$0\r");
        for command in [
            "I122=0.500000 I120=500.000000 #1 J=123.40",
            "I122=0.500000 I120=500.000000 #1 J-",
            "I113=10.000000",
            "I114=-10.000000",
            "#1K M161=32*I108 M162=32*I108",
            "#1J/",
        ] {
            rig.answer(command, b"\x06");
        }
        let controller = Arc::new(Mutex::new(controller_on("pmacScale")));
        controller.lock().unwrap().add_axis(1).unwrap();
        controller.lock().unwrap().set_axis_scale(1, 10);
        let mut axis = PmacAxis::new(controller, 1);
        let user = AsynUser::new(0).with_addr(1);
        axis.move_absolute(&user, 1234.0, 0.0, 5000.0, 10000.0)
            .unwrap();
        axis.move_velocity(&user, 0.0, -5000.0, 10000.0).unwrap();
        axis.set_high_limit(&user, 100.0).unwrap();
        axis.set_low_limit(&user, -100.0).unwrap();
        axis.set_position(&user, 10.0).unwrap();
        let status = axis.poll(&user).unwrap();
        assert_eq!((status.position, status.encoder_position), (1000.0, 1000.0));
        assert_eq!(
            rig.written()[1..],
            [
                "I122=0.500000 I120=500.000000 #1 J=123.40",
                "I122=0.500000 I120=500.000000 #1 J-",
                "I113=10.000000",
                "I114=-10.000000",
                "#1K M161=32*I108 M162=32*I108",
                "#1J/",
                "#1 ? F P",
                "i124",
            ]
        );
    }

    /// pmacController.cpp:774-801: the scale's refusals and its FLOW lines.
    #[test]
    fn set_axis_scale_prints_c_lines() {
        let (_rt, _rig) = publish_scripted("pmacScaleLow");
        let mut controller = PmacController::new(
            "pmacScaleCtl",
            "pmacScaleLow",
            0,
            low_level_port_connect("pmacScaleCtl", "pmacScaleLow", 0),
        );
        controller.add_axis(1).unwrap();
        let capture = TraceCapture::on("pmacScaleCtl");
        PortManager::global()
            .trace_manager()
            .set_trace_mask(Some("pmacScaleCtl"), TraceMask::ERROR | TraceMask::FLOW);
        controller.set_axis_scale(1, 0);
        controller.set_axis_scale(2, 4);
        controller.set_axis_scale(1, 4);
        assert_eq!(
            capture.take(),
            "pmacController::pmacSetAxisScale\n\
             pmacController::pmacSetAxisScale: Error: scale factor must be >=1.\n\
             pmacController::pmacSetAxisScale\n\
             pmacController::pmacSetAxisScale: Error: axis 2 has not been configured using \
             pmacCreateAxis.\n\
             pmacController::pmacSetAxisScale\n\
             pmacController::pmacSetAxisScale. Setting scale factor of 4 on axis 1, on \
             controller pmacScaleCtl.\n"
        );
        assert_eq!(controller.axis_scale(1), 4);
    }

    /// `port`'s trace, address 0 included, without the prefix, so what it
    /// holds is the driver's own bytes.
    struct TraceCapture {
        path: std::path::PathBuf,
        seen: std::cell::Cell<usize>,
    }

    impl TraceCapture {
        fn on(port: &str) -> Self {
            use epics_rs::asyn::trace::TraceInfoMask;
            let path = std::env::temp_dir().join(format!("pmac-err-{port}-{}", std::process::id()));
            let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
            let manager = PortManager::global();
            let trace = manager.trace_manager();
            trace.set_trace_info_mask(Some(port), TraceInfoMask::empty());
            trace.set_trace_file(Some(port), TraceFile::File(file.clone()));
            trace.set_device_trace_info_mask(port, 0, TraceInfoMask::empty());
            trace.set_device_trace_file(port, 0, TraceFile::File(file));
            Self {
                path,
                seen: std::cell::Cell::new(0),
            }
        }

        /// What was printed since the last call.
        fn take(&self) -> String {
            let text = std::fs::read_to_string(&self.path).unwrap();
            let new = text[self.seen.get()..].to_owned();
            self.seen.set(text.len());
            new
        }
    }

    impl Drop for TraceCapture {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.path);
        }
    }

    /// Controller port `name` over low-level port `low`, with its parameters.
    fn controller_port(name: &str, low: &str) -> (Arc<Mutex<PmacController>>, SyncIOHandle) {
        let connected = low_level_port_connect(name, low, 0);
        let controller = Arc::new(Mutex::new(PmacController::new(name, low, 0, connected)));
        let params = PmacControllerParams::new(controller.clone(), 100, 1000);
        let port =
            epics_rs::motor::devices::register_controller_port(&open_owner(), name, 2, params)
                .unwrap();
        controller.lock().unwrap().attach_port(port.clone());
        (
            controller,
            SyncIOHandle::from_handle(port, 0, Duration::from_secs(1)),
        )
    }

    /// pmacController.cpp:341, 601-708: C's error lines on the low-level
    /// port's trace, the feed-rate one once per `PMAC_ERROR_PRINT_TIME_`
    /// unless the rate recovered in between.
    #[test]
    fn controller_errors_print_c_lines_on_the_low_level_trace() {
        let (_rt, rig) = publish_scripted("pmacErrLow");
        let low = TraceCapture::on("pmacErrLow");
        let (controller, io) = controller_port("pmacErrCtl", "pmacErrLow");
        let write = |name, value| {
            io.write_int32(io.drv_user_create(name).unwrap(), value)
                .unwrap()
        };
        write("PMAC_C_FEEDRATE_LIMIT", 100);
        write("PMAC_C_FEEDRATE_POLL", 1);
        let poll = || controller.lock().unwrap().poll();
        let failed = |command| {
            format!(
                "pmacController::lowLevelWriteRead: Error from pasynOctetSyncIO->writeRead. \
                 command: {command}, status=1\n"
            )
        };
        let unread_global = format!(
            "{}pmacController::getGlobalStatus: Error reading ???.\n",
            failed("???")
        );
        let below = "pmacController::poll: *** ERROR ***: global feed rate below limit. \
                     feedrate: 50, feedrate limit 100\n";

        rig.answer("%", b"50\r");
        poll();
        assert_eq!(low.take(), format!("{unread_global}{below}"));
        poll();
        assert_eq!(low.take(), unread_global);

        rig.answer("???", b"zz\r");
        rig.answer("%", b"\r");
        poll();
        assert_eq!(
            low.take(),
            "pmacController::getGlobalStatus: Error reading ???. nvals: 0, response: zz\r\n\
             pmacController::getGlobalStatus: Error reading feedrate: nvals: -1, response: \r\n\
             pmacController::poll: Error reading or setting params.\n"
        );

        rig.answer("???", b"000000\r");
        rig.answer("%", b"100\r");
        poll();
        assert_eq!(low.take(), "");
        rig.answer("%", b"50\r");
        poll();
        assert_eq!(low.take(), below);

        rig.silence("%");
        poll();
        assert_eq!(
            low.take(),
            format!(
                "{}pmacController::getGlobalStatus: Error reading feedrate.\n\
                 pmacController::poll: Error reading or setting params.\n",
                failed("%")
            )
        );
    }

    /// pmacAxis.cpp:96, 136, 243-305, 497, 585: C's axis error lines on the
    /// controller's trace, the limits warning throttled as the feed rate's.
    /// pmacController.cpp:871-877, pmacCsGroups.cpp:321-330: a deferred
    /// release line sets `PMAC_C_COMMSERROR` from its status, on the port's
    /// write and on the axis's.
    #[test]
    fn a_deferred_release_sets_the_comms_error() {
        use crate::controller::DEFERRED_FAST_MOVES;
        use epics_rs::motor::controller_port::index;

        let (_rt, rig) = publish_scripted("pmacDeferCommsLow");
        let (controller, io) = controller_port("pmacDeferCommsCtl", "pmacDeferCommsLow");
        let comms_error = io.drv_user_create("PMAC_C_COMMSERROR").unwrap();
        let comms = || io.read_int32(comms_error).unwrap();
        assert_eq!(comms(), 0);
        io.write_int32(index::DEFER_MOVES, DEFERRED_FAST_MOVES)
            .unwrap();
        assert!(io.write_int32(index::DEFER_MOVES, 0).is_err());
        assert_eq!(comms(), 1);
        rig.answer("", b"\x06");
        io.write_int32(index::DEFER_MOVES, DEFERRED_FAST_MOVES)
            .unwrap();
        io.write_int32(index::DEFER_MOVES, 0).unwrap();
        assert_eq!(comms(), 0);

        let mut axis = PmacAxis::new(controller.clone(), 1);
        let user = AsynUser::new(0).with_addr(1);
        rig.silence("");
        axis.set_deferred_moves(&user, true).unwrap();
        controller
            .lock()
            .unwrap()
            .set_moves_deferred(DEFERRED_FAST_MOVES)
            .status
            .unwrap();
        assert!(axis.set_deferred_moves(&user, false).is_err());
        assert_eq!(comms(), 1);
    }

    /// pmacCsGroups.cpp:156-161: the group changes only once `undefine all`
    /// went through.
    #[test]
    fn a_failed_undefine_keeps_the_current_cs_group() {
        let (_rt, rig) = publish_scripted("pmacCsSwitchLow");
        let (controller, _io) = controller_port("pmacCsSwitchCtl", "pmacCsSwitchLow");
        let mut c = controller.lock().unwrap();
        let groups = c.cs_groups_mut();
        groups.add_group(1, "a");
        groups.add_axis(1, 1, "X", 2).unwrap();
        groups.add_group(2, "b");
        groups.add_axis(2, 1, "X", 7).unwrap();
        rig.answer("\x01\nundefine all", b"\x06");
        rig.answer("&2 #1->X", b"\x06");
        c.switch_cs_group(1).unwrap();
        rig.silence("\x01\nundefine all");
        assert!(c.switch_cs_group(2).is_err());
        assert_eq!(c.cs_groups_mut().abort_command(1).unwrap(), "&2A");
    }

    /// pmacCsGroups.cpp:228-233: the release aborts the first deferred
    /// axis's CS before it validates the rest, and drops the abort's status.
    #[test]
    fn coordinated_release_aborts_first_and_ignores_the_abort_status() {
        use crate::controller::DEFERRED_COORDINATED_MOVES;
        use crate::cs_groups::DeferredMove;

        let (_rt, rig) = publish_scripted("pmacCsAbortLow");
        let (controller, _io) = controller_port("pmacCsAbortCtl", "pmacCsAbortLow");
        let mut c = controller.lock().unwrap();
        let groups = c.cs_groups_mut();
        groups.add_group(1, "g");
        groups.add_axis(1, 1, "X", 2).unwrap();
        groups.add_axis(1, 3, "Y", 3).unwrap();
        groups.select(1);
        let to = DeferredMove {
            position: 1.0,
            relative: false,
            time_ms: 0.0,
        };
        rig.written();
        c.set_moves_deferred(DEFERRED_COORDINATED_MOVES)
            .status
            .unwrap();
        c.defer_move(1, to);
        c.defer_move(3, to);
        assert!(c.set_moves_deferred(0).status.is_err());
        assert_eq!(rig.written(), ["&2A"]);

        let line = "&2 Q70=0.000000  Q77=1.000000 B101R";
        rig.answer(line, b"\x06");
        c.set_moves_deferred(DEFERRED_COORDINATED_MOVES)
            .status
            .unwrap();
        c.defer_move(1, to);
        c.set_moves_deferred(0).status.unwrap();
        assert_eq!(rig.written(), ["&2A", line]);
    }

    /// pmacCsGroups.cpp:149, 237-261, 305, 323: the group switch and the
    /// coordinated release print C's lines on the controller's trace.
    #[test]
    fn cs_group_errors_print_c_lines_on_the_controller_trace() {
        use crate::controller::DEFERRED_COORDINATED_MOVES;
        use crate::cs_groups::{AxisPoll, DeferredMove};

        let (_rt, rig) = publish_scripted("pmacCsErrLow");
        let (controller, _io) = controller_port("pmacCsErrCtl", "pmacCsErrLow");
        let own = TraceCapture::on("pmacCsErrCtl");
        let mut c = controller.lock().unwrap();
        assert!(c.switch_cs_group(5).is_err());
        assert_eq!(
            own.take(),
            "pmacCsGroups::switchToGroup: Invalid Coordinate System Group Number\n"
        );

        let groups = c.cs_groups_mut();
        groups.add_group(1, "g");
        for (axis, def, cs) in [(1, "X", 2), (2, "U*2", 2), (3, "Y", 3), (4, "Z", 2)] {
            groups.add_axis(1, axis, def, cs).unwrap();
        }
        groups.select(1);
        rig.answer("&2A", b"\x06");
        rig.answer("&3A", b"\x06");
        let to = DeferredMove {
            position: 1.0,
            relative: false,
            time_ms: 0.0,
        };
        let release = |axes: &[i32], c: &mut PmacController| {
            c.set_moves_deferred(DEFERRED_COORDINATED_MOVES)
                .status
                .unwrap();
            for &axis in axes {
                c.defer_move(axis, to);
            }
            assert!(c.set_moves_deferred(0).status.is_err());
            own.take()
        };
        let line = |text: &str| format!("pmacCsGroups::processDeferredCoordMoves {text}\n");
        assert_eq!(
            release(&[5], &mut c),
            line("Error: deferred coordinated move on real axis 5 not in a coordinate system")
        );
        assert_eq!(
            release(&[1, 3], &mut c),
            line("Error: deferred coordinated move on multiple coordinate systems 2 and 3")
        );
        assert_eq!(
            release(&[2], &mut c),
            line("Error: illegal deferred coordinated move on real axis 2 defined as U*2 in CS 2")
        );
        c.record_poll(
            4,
            AxisPoll {
                position: 0.0,
                moving: true,
            },
        );
        assert_eq!(
            release(&[1], &mut c),
            line(
                "Error: illegal deferred coordinated move - real axis 4 in CS 2 is already moving"
            )
        );
        c.record_poll(4, AxisPoll::default());
        assert_eq!(
            release(&[1], &mut c),
            line("ERROR Sending Deferred Move Command.")
        );
    }

    #[test]
    fn axis_errors_print_c_lines_on_the_controller_trace() {
        let (_rt, rig) = publish_scripted("pmacAxErrLow");
        let (controller, _io) = controller_port("pmacAxErrCtl", "pmacAxErrLow");
        // After the port exists: registering it resets its trace.
        let own = TraceCapture::on("pmacAxErrCtl");
        let mut axis = PmacAxis::new(controller, 1);
        assert_eq!(
            own.take(),
            "pmacAxis::getAxisInitialStatus: Error: initial status poll failed on axis 1.\n\
             pmacAxis::pmacAxis: getAxisInitialStatus failed to return asynSuccess. \
             Controller: pmacAxErrCtl, Axis: 1.\n"
        );

        let user = AsynUser::new(0).with_addr(1);
        rig.answer("#1 ? F P", b"zz\r");
        axis.poll(&user).unwrap();
        assert_eq!(
            own.take(),
            "drvPmacAxisGetStatus: not all status values returned. Status: 0\n\
             Command :#1 ? F P\nResponse:zz\r"
        );
        rig.silence("#1 ? F P");
        axis.poll(&user).unwrap();
        assert_eq!(
            own.take(),
            "drvPmacAxisGetStatus: not all status values returned. Status: 1\n\
             Command :#1 ? F P\nResponse:"
        );

        let warning = "*** WARNING *** Limits are disabled on controller pmacAxErrCtl, axis 1\n";
        rig.answer("#1 ? F P", b"882000000001\r0\r100.0\r");
        // The failed polls took this period's print.
        rig.answer("i124", b"$20000\r");
        axis.poll(&user).unwrap();
        assert_eq!(own.take(), "");
        rig.answer("i124", b"$0\r");
        axis.poll(&user).unwrap();
        rig.answer("i124", b"$20000\r");
        axis.poll(&user).unwrap();
        axis.poll(&user).unwrap();
        assert_eq!(own.take(), warning);

        let home = |axis: &mut PmacAxis| axis.home(&user, 0.0, 2.0, 0.0, false);
        let addr = "Controller pmacAxErrCtl Addr 1. pmacAxis::home: ERROR";
        assert!(home(&mut axis).is_err());
        assert_eq!(own.take(), format!("{addr} Reading Controller Type.\n"));
        rig.answer("cid", b"12\r");
        assert!(home(&mut axis).is_err());
        assert_eq!(
            own.take(),
            format!("{addr} Unknown controller type = 12.\n")
        );
        rig.answer("cid", b"603382\r");
        assert!(home(&mut axis).is_err());
        assert_eq!(own.take(), format!("{addr} Cannot Read Home Flags.\n"));
        rig.answer("I7012 I7013 i124 i123 i126", b"0\r0\r$0\r10\r0\r");
        rig.answer("#1 HOME", b"\r");
        assert!(home(&mut axis).is_ok());
        assert_eq!(
            own.take(),
            "pmacAxis::home: Error: Cannot disable limits to home PMAC controller \
             pmacAxErrCtl, axis 1, type:0, flag:$0, vel:-0.002000, mode:0x0, offset: 0\n"
        );
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
