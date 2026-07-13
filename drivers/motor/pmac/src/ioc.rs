//! iocsh commands for the PMAC driver.
//!
//! The C module registers `pmacAsynIPConfigure` (in `pmacAsynIPPortSrc`),
//! `pmacCreateController` / `pmacCreateAxis` / `pmacCreateAxes` /
//! `pmacDisableLimitsCheck` / `pmacSetAxisScale` /
//! `pmacSetOpenLoopEncoderAxis` / `pmacCreateCsGroup` / `pmacCsGroupAddAxis`
//! (in `pmacAsynMotorPortSrc`) and `pmacAsynCoordCreate` +
//! `pmacSetCoord*` (in `pmacAsynCoordSrc`). All of them are here except:
//!
//! - `pmacSetAxisScale` / `pmacSetCoordStepsPerUnit` / `pmacSetDefaultCoordSteps`
//!   — the raw-step scaling they configure is the motor record's `MRES`; see
//!   [`crate::axis`] and [`crate::coord`]. Accepting the command and ignoring it
//!   would silently halve or double every move, so it is not provided.
//! - `pmacSetCoordMovingPollPeriod` / `pmacSetCoordIdlePollPeriod` — the poll
//!   periods are arguments of `pmacAsynCoordCreate` here, as they already are of
//!   `pmacCreateController` in C.
//!
//! Two commands extend their C signatures with configuration the C driver takes
//! from asyn parameters that the asyn-rs motor boundary does not carry:
//! `pmacCreateController` gains `deferredMode` / `feedRatePoll` /
//! `feedRateLimit` (C `PMAC_C_FEEDRATE_POLL`, `PMAC_C_FEEDRATE_LIMIT` and the
//! motor record's `motorDeferMoves_` value), and `pmacSetOpenLoopEncoderAxis`
//! gains `encoderRatio` (C `motorEncoderRatio_`). One command is new:
//! `pmacCsGroupSwitch`, which C drives by writing the `PMAC_C_COORDINATE_SYS_GROUP`
//! parameter.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_int_req, arg_str_req, opt_double, opt_int, poll_intervals,
    req_int, req_string,
};

use crate::axis::PmacAxis;
use crate::controller::{DeferredMode, PMAC_TIMEOUT, PmacController};
use crate::coord::{CS_AXES, PmacCoordSystem, PmacCsAxis};
use crate::ethernet::pmac_asyn_ip_configure_command;

/// How long a polled global status (`???`, `%`) or coordinate-system status
/// (`&{cs}??`, `Q8{n}`) stays fresh. One controller-wide read serves every axis
/// polling within the window, which is what C's single poller thread does.
const POLL_CACHE_TTL: Duration = Duration::from_millis(50);

/// A registered controller plus the poll periods its axes are installed with.
struct ControllerReg {
    controller: Arc<Mutex<PmacController>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
}

type Registry = Arc<Mutex<HashMap<String, ControllerReg>>>;

fn registry_lock(registry: &Registry) -> std::sync::MutexGuard<'_, HashMap<String, ControllerReg>> {
    registry.lock().unwrap_or_else(|e| e.into_inner())
}

/// Connect a [`SyncIOHandle`] to an already-configured octet port at the given
/// asyn address (C `pasynOctetSyncIO->connect(lowLevelPortName,
/// lowLevelPortAddress, …)`). motor-common's helpers hard-code address 0, and
/// the PMAC commands carry the address explicitly.
fn connect_octet(port: &str, addr: i32) -> Result<SyncIOHandle, String> {
    let entry = get_port(port).ok_or_else(|| {
        format!("low level port '{port}' not found (call pmacAsynIPConfigure or drvAsynIPPortConfigure first)")
    })?;
    Ok(SyncIOHandle::from_handle(
        entry.handle.clone(),
        addr,
        PMAC_TIMEOUT,
    ))
}

/// Every PMAC iocsh command, sharing one controller registry.
pub fn pmac_commands(holder: &Arc<MotorHolder>, trace: Arc<TraceManager>) -> Vec<CommandDef> {
    let registry: Registry = Arc::new(Mutex::new(HashMap::new()));
    vec![
        pmac_asyn_ip_configure_command(trace),
        pmac_create_controller_command(&registry),
        pmac_create_axis_command(&registry, holder),
        pmac_create_axes_command(&registry, holder),
        pmac_disable_limits_check_command(&registry),
        pmac_set_open_loop_encoder_axis_command(&registry),
        pmac_create_cs_group_command(&registry),
        pmac_cs_group_add_axis_command(&registry),
        pmac_cs_group_switch_command(&registry),
        pmac_asyn_coord_create_command(holder),
    ]
}

/// C `pmacCreateController(portName, lowLevelPortName, lowLevelPortAddress,
/// numAxes, movingPollPeriod, idlePollPeriod)`. `numAxes` creates that many axes
/// straight away, as C does (it constructs `numAxes` `pmacAxis` objects).
fn pmac_create_controller_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacCreateController",
        vec![
            arg_str_req("controllerName"),
            arg_str_req("lowLevelPortName"),
            arg_int_req("lowLevelPortAddress"),
            arg_int_req("numAxes"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
            arg_int_opt("deferredMode"),
            arg_int_opt("feedRatePoll"),
            arg_int_opt("feedRateLimit"),
        ],
        "pmacCreateController(controllerName, lowLevelPortName, lowLevelPortAddress, numAxes, \
         [movingPollMs], [idlePollMs], [deferredMode], [feedRatePoll], [feedRateLimit]) - Create a \
         Delta Tau PMAC/Geobrick controller. numAxes is a hint only; axes are created by \
         pmacCreateAxis/pmacCreateAxes. deferredMode is 1 (fast, default) or 2 (coordinated); \
         feedRatePoll != 0 polls the global feed rate and raises PROBLEM below feedRateLimit% \
         (default 100)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let low_level_port = req_string(args, 1, "lowLevelPortName")?;
            let low_level_addr = req_int(args, 2, "lowLevelPortAddress")? as i32;
            let num_axes = req_int(args, 3, "numAxes")?;
            if num_axes < 1 {
                return Err("pmacCreateController: numAxes must be > 0".into());
            }
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 4, 5)?;
            let deferred_mode = DeferredMode::from_code(opt_int(args, 6, 1, "deferredMode")?)
                .map_err(|e| format!("pmacCreateController: {e}"))?;
            let feed_rate_poll = opt_int(args, 7, 0, "feedRatePoll")? != 0;
            let feed_rate_limit = opt_int(args, 8, 100, "feedRateLimit")? as i32;

            if registry_lock(&registry).contains_key(&name) {
                return Err(format!(
                    "pmacCreateController: controller '{name}' already exists"
                ));
            }

            let handle = connect_octet(&low_level_port, low_level_addr)
                .map_err(|e| format!("pmacCreateController: {e}"))?;
            let controller = PmacController::new(
                handle,
                deferred_mode,
                feed_rate_poll,
                feed_rate_limit,
                POLL_CACHE_TTL,
            );

            registry_lock(&registry).insert(
                name.clone(),
                ControllerReg {
                    controller: Arc::new(Mutex::new(controller)),
                    moving_poll_ms,
                    idle_poll_ms,
                },
            );
            println!(
                "pmacCreateController: {name} on {low_level_port}:{low_level_addr} \
                 numAxes={num_axes} deferredMode={deferred_mode:?} feedRatePoll={feed_rate_poll} \
                 feedRateLimit={feed_rate_limit} poll=[{moving_poll_ms}/{idle_poll_ms}]ms"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Create one axis and bind it to a `DTYP`-keyed motor device support
/// (`PMAC_{controllerName}_{axis}`).
fn create_axis(
    registry: &Registry,
    holder: &Arc<MotorHolder>,
    ctx: &CommandContext,
    command: &str,
    name: &str,
    axis: i32,
) -> Result<(), String> {
    let mut registry = registry_lock(registry);
    let reg = registry
        .get_mut(name)
        .ok_or_else(|| format!("{command}: controller '{name}' not found"))?;

    {
        let mut controller = reg.controller.lock().unwrap_or_else(|e| e.into_inner());
        controller
            .add_axis(axis)
            .map_err(|e| format!("{command}: {e}"))?;
    }

    let motor: Arc<Mutex<dyn AsynMotor>> =
        Arc::new(Mutex::new(PmacAxis::new(reg.controller.clone(), axis)));
    holder.install(
        ctx,
        format!("PMAC_{name}_{axis}"),
        motor,
        reg.moving_poll_ms,
        reg.idle_poll_ms,
    );
    Ok(())
}

/// C `pmacCreateAxis(controllerName, axis)`.
fn pmac_create_axis_command(registry: &Registry, holder: &Arc<MotorHolder>) -> CommandDef {
    let registry = registry.clone();
    let holder = holder.clone();
    CommandDef::new(
        "pmacCreateAxis",
        vec![arg_str_req("controllerName"), arg_int_req("axis")],
        "pmacCreateAxis(controllerName, axis) - Create one PMAC axis (1-based; DTYP \
         PMAC_{controllerName}_{axis})",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let axis = req_int(args, 1, "axis")? as i32;
            create_axis(&registry, &holder, ctx, "pmacCreateAxis", &name, axis)?;
            println!("pmacCreateAxis: {name} axis {axis} (DTYP=PMAC_{name}_{axis})");
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacCreateAxes(controllerName, numAxes)`: axes 1..=numAxes. C's loop
/// starts at 1 too — axis 0 is the controller's own asyn address.
fn pmac_create_axes_command(registry: &Registry, holder: &Arc<MotorHolder>) -> CommandDef {
    let registry = registry.clone();
    let holder = holder.clone();
    CommandDef::new(
        "pmacCreateAxes",
        vec![arg_str_req("controllerName"), arg_int_req("numAxes")],
        "pmacCreateAxes(controllerName, numAxes) - Create PMAC axes 1..numAxes (DTYP \
         PMAC_{controllerName}_{axis})",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let num_axes = req_int(args, 1, "numAxes")?;
            if num_axes < 1 {
                return Err("pmacCreateAxes: numAxes must be > 0".into());
            }
            for axis in 1..=num_axes as i32 {
                create_axis(&registry, &holder, ctx, "pmacCreateAxes", &name, axis)?;
            }
            println!(
                "pmacCreateAxes: {name} axes 1..{num_axes} (DTYP=PMAC_{name}_{{1..{num_axes}}})"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacDisableLimitsCheck(controllerName, axis, allAxes)`.
fn pmac_disable_limits_check_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacDisableLimitsCheck",
        vec![
            arg_str_req("controllerName"),
            arg_int_req("axis"),
            arg_int_req("allAxes"),
        ],
        "pmacDisableLimitsCheck(controllerName, axis, allAxes) - Stop raising PROBLEM when the \
         controller reports hardware limits disabled (i{axis}24 bit 17). allAxes != 0 applies to \
         every axis and ignores the axis argument",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let axis = req_int(args, 1, "axis")? as i32;
            let all_axes = req_int(args, 2, "allAxes")? != 0;

            let registry = registry_lock(&registry);
            let reg = registry
                .get(&name)
                .ok_or_else(|| format!("pmacDisableLimitsCheck: controller '{name}' not found"))?;
            let mut controller = reg.controller.lock().unwrap_or_else(|e| e.into_inner());
            if all_axes {
                controller.disable_limits_check_all();
                println!("pmacDisableLimitsCheck: {name} all axes");
            } else {
                controller
                    .disable_limits_check(axis)
                    .map_err(|e| format!("pmacDisableLimitsCheck: {e}"))?;
                println!("pmacDisableLimitsCheck: {name} axis {axis}");
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacSetOpenLoopEncoderAxis(controllerName, axis, encoderAxis)`, plus the
/// encoder ratio C reads from the record's `motorEncoderRatio_` when it forwards
/// a set-position to the encoder axis.
fn pmac_set_open_loop_encoder_axis_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacSetOpenLoopEncoderAxis",
        vec![
            arg_str_req("controllerName"),
            arg_int_req("axis"),
            arg_int_req("encoderAxis"),
            arg_double_opt("encoderRatio"),
        ],
        "pmacSetOpenLoopEncoderAxis(controllerName, axis, encoderAxis, [encoderRatio]) - Read the \
         encoder position of an open-loop axis from another axis (default ratio 1.0)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let axis = req_int(args, 1, "axis")? as i32;
            let encoder_axis = req_int(args, 2, "encoderAxis")? as i32;
            let ratio = opt_double(args, 3, 1.0, "encoderRatio")?;

            let registry = registry_lock(&registry);
            let reg = registry.get(&name).ok_or_else(|| {
                format!("pmacSetOpenLoopEncoderAxis: controller '{name}' not found")
            })?;
            reg.controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .set_open_loop_encoder_axis(axis, encoder_axis, ratio)
                .map_err(|e| format!("pmacSetOpenLoopEncoderAxis: {e}"))?;
            println!(
                "pmacSetOpenLoopEncoderAxis: {name} axis {axis} encoder axis {encoder_axis} \
                 ratio {ratio}"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacCreateCsGroup(controllerName, groupNumber, groupName, axisCount)`.
/// `axisCount` only pre-sizes C's map; the group is defined by the
/// `pmacCsGroupAddAxis` calls that follow, so it is accepted and unused.
fn pmac_create_cs_group_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacCreateCsGroup",
        vec![
            arg_str_req("controllerName"),
            arg_int_req("groupNumber"),
            arg_str_req("groupName"),
            arg_int_req("axisCount"),
        ],
        "pmacCreateCsGroup(controllerName, groupNumber, groupName, axisCount) - Define a \
         coordinate-system axis grouping (axisCount is advisory)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let group = req_int(args, 1, "groupNumber")? as i32;
            let group_name = req_string(args, 2, "groupName")?;
            let _axis_count = req_int(args, 3, "axisCount")?;

            let registry = registry_lock(&registry);
            let reg = registry
                .get(&name)
                .ok_or_else(|| format!("pmacCreateCsGroup: controller '{name}' not found"))?;
            reg.controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .cs_groups_mut()
                .add_group(group, &group_name);
            println!("pmacCreateCsGroup: {name} group {group} \"{group_name}\"");
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacCsGroupAddAxis(controllerName, groupNumber, axis, axisDef, cs)`.
fn pmac_cs_group_add_axis_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacCsGroupAddAxis",
        vec![
            arg_str_req("controllerName"),
            arg_int_req("groupNumber"),
            arg_int_req("axis"),
            arg_str_req("axisDef"),
            arg_int_req("cs"),
        ],
        "pmacCsGroupAddAxis(controllerName, groupNumber, axis, axisDef, cs) - Map a real axis into \
         a coordinate system within a group; axisDef is the PMAC CS definition (e.g. \"X\", \
         \"10000X\")",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let group = req_int(args, 1, "groupNumber")? as i32;
            let axis = req_int(args, 2, "axis")? as i32;
            let axis_def = req_string(args, 3, "axisDef")?;
            let cs = req_int(args, 4, "cs")? as i32;

            let registry = registry_lock(&registry);
            let reg = registry
                .get(&name)
                .ok_or_else(|| format!("pmacCsGroupAddAxis: controller '{name}' not found"))?;
            reg.controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .cs_groups_mut()
                .add_axis(group, axis, &axis_def, cs)
                .map_err(|e| format!("pmacCsGroupAddAxis: {e}"))?;
            println!("pmacCsGroupAddAxis: {name} group {group} axis {axis} -> &{cs} {axis_def}");
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C drives this by writing the `PMAC_C_COORDINATE_SYS_GROUP` asyn parameter
/// (`pmacController::writeInt32` → `pmacCsGroups::switchToGroup`). There is no
/// controller-parameter port at the asyn-rs motor boundary, so the switch is an
/// iocsh command.
fn pmac_cs_group_switch_command(registry: &Registry) -> CommandDef {
    let registry = registry.clone();
    CommandDef::new(
        "pmacCsGroupSwitch",
        vec![arg_str_req("controllerName"), arg_int_req("groupNumber")],
        "pmacCsGroupSwitch(controllerName, groupNumber) - Undefine all coordinate systems and \
         apply the axis mappings of the named group",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = req_string(args, 0, "controllerName")?;
            let group = req_int(args, 1, "groupNumber")? as i32;

            let registry = registry_lock(&registry);
            let reg = registry
                .get(&name)
                .ok_or_else(|| format!("pmacCsGroupSwitch: controller '{name}' not found"))?;
            reg.controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .switch_cs_group(group)
                .map_err(|e| format!("pmacCsGroupSwitch: {e}"))?;
            println!("pmacCsGroupSwitch: {name} group {group}");
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `pmacAsynCoordCreate(port, addr, cs, ref, program)` followed by
/// `drvAsynMotorConfigure(csPort, "pmacAsynCoord", ref, 9)`. Model-3 drivers
/// have no separate `drvAsynMotorConfigure` step, so the two collapse into one
/// command: the CS name replaces the `ref` handle and becomes the `DTYP` key
/// (`PMACCS_{csName}_{1..9}`, 1 = A … 9 = Z; C addresses the same nine axes from
/// 0).
fn pmac_asyn_coord_create_command(holder: &Arc<MotorHolder>) -> CommandDef {
    let holder = holder.clone();
    CommandDef::new(
        "pmacAsynCoordCreate",
        vec![
            arg_str_req("csName"),
            arg_str_req("lowLevelPortName"),
            arg_int_req("lowLevelPortAddress"),
            arg_int_req("cs"),
            arg_int_req("program"),
            arg_int_opt("movingPollMs"),
            arg_int_opt("idlePollMs"),
        ],
        "pmacAsynCoordCreate(csName, lowLevelPortName, lowLevelPortAddress, cs, program, \
         [movingPollMs], [idlePollMs]) - Create the 9 coordinate-system axes A,B,C,U,V,W,X,Y,Z of \
         PMAC coordinate system cs (DTYP PMACCS_{csName}_{1..9}); program is the motion program a \
         move runs, or 0 to leave the start to an external process",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let cs_name = req_string(args, 0, "csName")?;
            let low_level_port = req_string(args, 1, "lowLevelPortName")?;
            let low_level_addr = req_int(args, 2, "lowLevelPortAddress")? as i32;
            let cs = req_int(args, 3, "cs")? as i32;
            let program = req_int(args, 4, "program")? as i32;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 5, 6)?;

            if !(1..=16).contains(&cs) {
                return Err(format!(
                    "pmacAsynCoordCreate: invalid coordinate system number {cs} (1..16)"
                ));
            }

            let handle = connect_octet(&low_level_port, low_level_addr)
                .map_err(|e| format!("pmacAsynCoordCreate: {e}"))?;
            let system = Arc::new(Mutex::new(PmacCoordSystem::new(
                handle,
                cs,
                program,
                POLL_CACHE_TTL,
            )));

            for axis in 1..=CS_AXES {
                let cs_axis = PmacCsAxis::new(system.clone(), axis)
                    .map_err(|e| format!("pmacAsynCoordCreate: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(cs_axis));
                holder.install(
                    ctx,
                    format!("PMACCS_{cs_name}_{axis}"),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                );
            }
            println!(
                "pmacAsynCoordCreate: {cs_name} on {low_level_port}:{low_level_addr} cs={cs} \
                 program={program} poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                 (DTYP=PMACCS_{cs_name}_{{1..{CS_AXES}}})"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}
