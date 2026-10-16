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
//! `pmacSetAxisScale`, `pmacSetCoordStepsPerUnit` and
//! `pmacSetDefaultCoordSteps` are not provided: the driver speaks controller
//! counts and the record's `MRES` does the scaling (see [`crate::axis`]).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::iocsh::{arg_int_req, arg_str_req, poll_intervals, req_string};
use motor_common::model1::c_int;
use motor_common::model2::{Model2, Model2Axis};
use motor_common::{asyn_user, install, register_port};

use crate::axis::PmacAxis;
use crate::controller::{DeferredMode, PMAC_TIMEOUT, PmacController};
use crate::coord::{CS_AXES, PmacCoordSystem, PmacCsAxis};
use crate::ethernet::pmac_asyn_ip_configure_command;

/// How long a polled global status (`???`, `%`) or coordinate-system status
/// (`&{cs}??`, `Q8{n}`) stays fresh. One controller-wide read serves every axis
/// polling within the window, which is what C's single poller thread does.
const POLL_CACHE_TTL: Duration = Duration::from_millis(50);

/// pmacAsynCoord.c's `motorAxisDrvSET_t`, its cards keyed by `reference`.
pub static PMAC_ASYN_COORD: Model2 = Model2::new("pmacAsynCoord");

/// `defaultMovingPollPeriod` / `defaultIdlePollPeriod` (pmacAsynCoord.c:122).
const COORD_MOVING_POLL_MS: u64 = 100;
const COORD_IDLE_POLL_MS: u64 = 500;

/// A controller and the poll periods `pmacCreateController` gave it.
struct ControllerReg {
    name: String,
    controller: Arc<Mutex<PmacController>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
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
    InputEos(String),
    OutputEos(String),
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
        .map_err(|e| ConnectError::InputEos(e.to_string()))?;
    entry
        .handle
        .set_output_eos_blocking(user(), b"\r")
        .map_err(|e| ConnectError::OutputEos(e.to_string()))?;
    Ok(SyncIOHandle::from_handle(
        entry.handle.clone(),
        addr,
        PMAC_TIMEOUT,
    ))
}

/// C `pmacController`'s constructor (pmacController.cpp:194-200): a refused
/// connect or EOS set is printed and the controller is still created, with no
/// low-level user.
fn low_level_port_connect(port: &str, addr: i32) -> Option<SyncIOHandle> {
    connect_octet(port, addr)
        .inspect_err(|e| {
            match e {
                ConnectError::NoPort => eprintln!(
                    "pmacController::motorAxisAsynConnect: unable to connect to port {port}"
                ),
                ConnectError::InputEos(e) => {
                    eprintln!("pmacController: unable to set input EOS on {port}: {e}")
                }
                ConnectError::OutputEos(e) => {
                    eprintln!("pmacController: unable to set output EOS on {port}: {e}")
                }
            }
            println!(
                "pmacController::pmacController: Failed to connect to low level \
                 asynOctetSyncIO port {port}"
            );
        })
        .ok()
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

            register_port(ctx, &name, num_axes + 1)?;
            let controller = PmacController::new(
                low_level_port_connect(&low_level_port, low_level_addr),
                DeferredMode::Fast,
                false,
                100,
                POLL_CACHE_TTL,
            );
            registry
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .push(ControllerReg {
                    name,
                    controller: Arc::new(Mutex::new(controller)),
                    moving_poll_ms,
                    idle_poll_ms,
                });
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Create axis `axis` of controller `reg` as `@asyn(name,axis)`.
fn create_axis(
    reg: &ControllerReg,
    ctx: &CommandContext,
    command: &str,
    axis: i32,
) -> Result<(), String> {
    reg.controller
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .add_axis(axis)
        .map_err(|e| format!("{command}: {e}"))?;
    let motor: Arc<Mutex<dyn AsynMotor>> =
        Arc::new(Mutex::new(PmacAxis::new(reg.controller.clone(), axis)));
    install(
        ctx,
        asyn_user(&reg.name, axis),
        motor,
        reg.moving_poll_ms,
        reg.idle_poll_ms,
    )
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
        move |args: &[ArgValue], ctx: &CommandContext| {
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
                create_axis(reg, ctx, "pmacCreateAxis", axis)
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
        move |args: &[ArgValue], ctx: &CommandContext| {
            let name = req_string(args, 0, "Controller port name")?;
            let num_axes = c_int(args, 1);
            let created = find(&registry, &name, |reg| {
                (1..=num_axes).try_for_each(|axis| create_axis(reg, ctx, "pmacCreateAxes", axis))
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

/// What each `pmacAsynCoordCreate` made: `(reference, port, address, cs)`.
static COORD_SYSTEMS: Mutex<Vec<(i32, String, i32, i32)>> = Mutex::new(Vec::new());

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
            let port = req_string(args, 0, "port")?;
            let addr = c_int(args, 1);
            let cs = c_int(args, 2);
            let reference = c_int(args, 3);
            let program = c_int(args, 4);

            if !(1..=16).contains(&cs) {
                println!("Invalid co-ordinate system number: {cs}");
                return Ok(CommandOutcome::Continue);
            }
            let mut systems = COORD_SYSTEMS.lock().unwrap_or_else(|e| e.into_inner());
            if systems
                .iter()
                .any(|(r, p, a, c)| *r == reference || (*p == port && *a == addr && *c == cs))
            {
                println!("C.S. reference {reference} already exists");
                return Ok(CommandOutcome::Continue);
            }
            let handle = match connect_octet(&port, addr) {
                Ok(handle) => handle,
                Err(ConnectError::NoPort) => {
                    println!("drvPmacCreate: unable to connect to port {port}");
                    return Ok(CommandOutcome::Continue);
                }
                Err(ConnectError::InputEos(e)) => {
                    eprintln!("drvPmacCreate: unable to set input EOS on {port}: {e}");
                    return Ok(CommandOutcome::Continue);
                }
                Err(ConnectError::OutputEos(e)) => {
                    eprintln!("drvPmacCreate: unable to set output EOS on {port}: {e}");
                    return Ok(CommandOutcome::Continue);
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
                    Ok(Model2Axis {
                        motor,
                        moving_poll_ms: COORD_MOVING_POLL_MS,
                        idle_poll_ms: COORD_IDLE_POLL_MS,
                    })
                })
                .collect::<Result<Vec<_>, String>>()?;
            PMAC_ASYN_COORD.set_card(reference, axes);
            systems.push((reference, port, addr, cs));
            Ok(CommandOutcome::Continue)
        },
    )
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

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

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

    fn controller_on(port: &str) -> PmacController {
        PmacController::new(
            low_level_port_connect(port, 0),
            DeferredMode::Fast,
            false,
            100,
            POLL_CACHE_TTL,
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

    #[test]
    fn a_missing_low_level_port_leaves_a_controller_whose_commands_fail() {
        assert!(controller_on("pmacNoSuchPort").write_read("I100").is_err());
    }
}
