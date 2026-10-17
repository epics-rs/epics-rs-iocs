//! iocsh commands for the Newport drivers, under C's names and arguments
//! (NewportRegister.cc, the model-3 `*Create*` files and NewFocusRegister.cc).
//!
//! - **Model 3** (SMC100, CONEX, CONEX-AGAP, Agilis, XPS, HXP): each
//!   controller is asyn port `portName`, and records reach its axes as
//!   `@asyn(portName,axis)`. XPS and HXP open their own TCP sockets from an IP
//!   address and port, as asyn ports named `ip:port:N` (C `ConnectToServer`).
//! - **Model 2** (`MM4000AsynSetup`/`MM4000AsynConfig`): `MM4000AsynConfig`
//!   builds a card of [`MOTOR_MM4000`], which `drvAsynMotorConfigure` opens.
//! - **Model 1** (MM3000, MM4000, PM500, ESP300, PMNC87xx): `<Drv>Setup` +
//!   `<Drv>Config(card, port, addr)`; records reach the axes as
//!   `DTYP "<Drv>"`, `OUT "#C card S signal"`.
//!
//! The XPS profile, position-compare and TCL commands and the HXP
//! move-all/coordinate-system commands are driver-private: C drives them from
//! controller-level records this port does not serve.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::{claim_port, get_port};
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::task::{StackSizeClass, ThreadPriority, spawn_dedicated_thread};
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;
use epics_rs::motor::trace::MotorTrace;
use motor_common::connect::{
    EosConnectError, EosRefusal, connect_controller, connect_serial, connect_with_eos,
};
use motor_common::iocsh::{
    arg_double_opt, arg_double_req, arg_int_opt, arg_int_req, arg_str_opt, arg_str_req, opt_double,
    opt_int, opt_string, poll_intervals, req_double, req_string,
};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;
use motor_common::model2::{Model2, Model2Axis};
use motor_common::util::atof;
use motor_common::{asyn_user, install, register_port};

use crate::agap::{self, AgapAxis, AgapController};
use crate::agilis::{AgUcAxis, AgUcController};
use crate::conex::ConexAxis;
use crate::esp300::{self, Esp300Axis, Esp300Controller};
use crate::hxp::{HXP_GROUP, HxpAxis, HxpController, MoveCoordSys, NUM_HXP_AXES};
use crate::mm3000::{Mm3000Axis, Mm3000Controller};
use crate::mm4000::{self, Mm4000Axis, Mm4000Controller, Mm4000Refusal};
use crate::pm500::{Pm500Axis, Pm500Controller};
use crate::pmnc::{PmncAxis, PmncController};
use crate::smc100::Smc100Axis;
use crate::xps::{
    ExecutionPlan, GatheringReadback, MoveMode, PcoParams, Profile, SocketMode, XpsAxis,
    XpsController, XpsError, XpsSocket, ftp, gathering, gpio, pco, profile,
};

/// Per-command serial I/O timeout: SMC100 (asynMotorController's
/// `DEFAULT_CONTROLLER_TIMEOUT`) and CONEX / AGAP (`CONEX_TIMEOUT`).
const SMC100_TIMEOUT: Duration = Duration::from_secs(2);
const CONEX_TIMEOUT: Duration = Duration::from_secs(2);
const AGAP_TIMEOUT: Duration = Duration::from_secs(2);
const AG_UC_TIMEOUT: Duration = Duration::from_secs(2);

/// XPS poll-socket timeout (C `XPS_POLL_TIMEOUT` = 2 s): waits for the full
/// `,EndOfAPI`-framed reply.
const XPS_POLL_TIMEOUT: Duration = Duration::from_secs(2);
/// XPS move-socket timeout (C sets `-0.1` s): the fire-and-forget write does
/// not wait for the move to finish, so the read times out quickly.
const XPS_MOVE_TIMEOUT: Duration = Duration::from_millis(100);

/// The controller's own move socket, which runs profiles: the moves to the
/// start and `MultipleAxesPVTExecution` block until the motion finishes, so
/// this bounds a whole trajectory run rather than a single RPC.
const XPS_PVT_EXEC_TIMEOUT: Duration = Duration::from_secs(3600);

/// HXP hexapod poll-socket timeout (C `HXP_POLL_TIMEOUT` = 2 s).
const HXP_POLL_TIMEOUT: Duration = Duration::from_secs(2);
/// HXP move-socket timeout (C sets `-0.1` s: fire-and-forget writes).
const HXP_MOVE_TIMEOUT: Duration = Duration::from_millis(100);

/// ESP300 serial command timeout (C `SERIAL_TIMEOUT` 5 s — "the ESP300 does
/// not respond for 2 to 5 seconds after hitting a travel limit").
const ESP300_TIMEOUT: Duration = Duration::from_secs(5);

/// MM3000 serial command timeout (C drvMM3000 `SERIAL_TIMEOUT` 5 s).
const MM3000_TIMEOUT: Duration = Duration::from_secs(5);

/// MM4000 serial command timeout (C drvMM4000Asyn `TIMEOUT` 2 s).
const MM4000_TIMEOUT: Duration = Duration::from_secs(2);
/// C drvMM4000Asyn `MM4000_MAX_AXES`.
const MM4000_MAX_AXES: i32 = 8;

/// PM500 serial command timeout (C drvPM500 `SERIAL_TIMEOUT` 2 s).
const PM500_TIMEOUT: Duration = Duration::from_secs(2);

/// PMNC (New Focus 8750/8752) command timeout (C drvPMNC87xx
/// `PMNC_TIMEOUT` 3 s).
const PMNC_TIMEOUT: Duration = Duration::from_secs(3);

/// Default XPS FTP account and trajectory directory: the factory
/// `Administrator` login C's `XPSCreateProfile` is normally given, and the
/// XPS-C/Q public trajectory folder.
const XPS_FTP_USER: &str = "Administrator";
const XPS_FTP_PASSWORD: &str = "Administrator";
const XPS_TRAJECTORY_DIR: &str = "/Admin/Public/Trajectories";

/// `MAX_SOCKETS` (asynOctetSocket.cpp:36).
const MAX_SOCKETS: usize = 1000;

/// `nextSocket` (asynOctetSocket.cpp).
static NEXT_SOCKET: AtomicUsize = AtomicUsize::new(0);

/// C `ConnectToServer` (asynOctetSocket.cpp:60): asyn IP port
/// `ip:port:N` from `drvAsynIPPortConfigure(name, "ip:port TCP", 0, 0, 1)`.
/// `None` is C's -1, printed as C prints it.
fn connect_to_server(ctx: &CommandContext, ip: &str, ip_port: i32) -> Option<PortHandle> {
    let socket = NEXT_SOCKET.load(Ordering::Relaxed);
    if socket >= MAX_SOCKETS {
        println!("ConnectToServer: too many open sockets, max={MAX_SOCKETS}");
        return None;
    }
    let name = format!("{ip}:{ip_port}:{socket}");
    let refused = |line: &str| {
        println!("{line}");
        println!(
            "ConnectToServer, error calling pasynOctetSyncIO->connect asynManager:connectDevice port {name} not found"
        );
    };
    let driver =
        match DrvAsynIPPort::new_configured(&name, &format!("{ip}:{ip_port} TCP"), false, true) {
            Ok(driver) => driver,
            Err(e) => {
                refused(&format!("drvAsynIPPortConfigure: {e}"));
                return None;
            }
        };
    let registered = claim_port(&name).ok().and_then(|claim| {
        let (handle, _jh) = create_port_runtime(&claim, driver, RuntimeConfig::default()).ok()?;
        match claim.publish(
            handle.port_handle().clone(),
            ctx.db().scan_control().interrupt_gate(),
        ) {
            Ok(()) => Some(handle),
            Err(_) => {
                handle.shutdown();
                None
            }
        }
    });
    let Some(handle) = registered else {
        refused("drvAsynIPPortConfigure: Can't register myself.");
        return None;
    };
    NEXT_SOCKET.store(socket + 1, Ordering::Relaxed);
    Some(handle.port_handle().clone())
}

/// `SMC100CreateController(portName, SMC100PortName, numAxes,
/// movingPollPeriod, idlePollPeriod, eguPerStep)`; axis N is controller
/// address N+1 on the shared line.
pub fn smc100_create_controller_command() -> CommandDef {
    CommandDef::new(
        "SMC100CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("SMC100 port name"),
            arg_int_req("Number of axes"),
            arg_int_req("Moving poll period (ms)"),
            arg_int_req("Idle poll period (ms)"),
            arg_str_req("EGUs per step"),
        ],
        "SMC100CreateController(portName, SMC100PortName, numAxes, movingPollPeriod, \
         idlePollPeriod, eguPerStep)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let serial_port = req_string(args, 1, "SMC100 port name")?;
            let num_axes = c_int(args, 2);
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let step_size = atof(&opt_string(args, 5)?.unwrap_or_default());
            if step_size == 0.0 {
                return Err("SMC100CreateController: EGUs per step must be non-zero".into());
            }

            register_port(ctx, &port_name, num_axes)?;
            let (handle, _) = connect_controller(
                &serial_port,
                SMC100_TIMEOUT,
                &port_name,
                format_args!(
                    "SMC100Controller::SMC100Controller: cannot connect to SMC100 controller"
                ),
            );
            // C builds every axis on pasynUserController_, connected or not.
            let mut controller_user = Some(handle);
            for axis in 0..num_axes {
                let handle = controller_user.take().unwrap_or_else(|| {
                    SyncIOHandle::connect_port(
                        &serial_port,
                        0,
                        SMC100_TIMEOUT,
                        InterfaceType::Octet,
                    )
                    .0
                });
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(Smc100Axis::new(
                    handle,
                    (axis + 1) as u8,
                    step_size,
                )));
                install(
                    ctx,
                    asyn_user(&port_name, axis),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                    2, // C startPoller, SMC100Driver.cpp:67
                )?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// The arguments AG_CONEX and AGAP_CONEX share.
fn conex_args() -> Vec<ArgDesc> {
    vec![
        arg_str_req("Port name"),
        arg_str_req("Serial port name"),
        arg_int_req("Controller ID"),
        arg_int_req("Moving poll period (ms)"),
        arg_int_req("Idle poll period (ms)"),
    ]
}

/// `AG_CONEXCreateController(portName, serialPortName, controllerID,
/// movingPollPeriod, idlePollPeriod)`: one axis, 0.
pub fn ag_conex_create_controller_command() -> CommandDef {
    CommandDef::new(
        "AG_CONEXCreateController",
        conex_args(),
        "AG_CONEXCreateController(portName, serialPortName, controllerID, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let serial_port = req_string(args, 1, "Serial port name")?;
            let controller_id = c_int(args, 2);
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port(ctx, &port_name, 1)?;
            // C returns from the constructor here: the port has no axis.
            let (handle, connected) = connect_controller(
                &serial_port,
                CONEX_TIMEOUT,
                &port_name,
                format_args!(
                    "AG_CONEXController::AG_CONEXController: cannot connect to CONEX controller"
                ),
            );
            if connected.is_err() {
                return Ok(CommandOutcome::Continue);
            }
            let Some(axis) = ConexAxis::new(handle, controller_id, &MotorTrace::new(&port_name, 0))
            else {
                return Ok(CommandOutcome::Continue);
            };
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            install(
                ctx,
                asyn_user(&port_name, 0),
                motor,
                moving_poll_ms,
                idle_poll_ms,
                2, // C startPoller, AG_CONEX.cpp:80
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `AGAP_CONEXCreateController(portName, serialPortName, controllerID,
/// movingPollPeriod, idlePollPeriod)`: axes 0 (U) and 1 (V) on one line.
pub fn agap_conex_create_controller_command() -> CommandDef {
    CommandDef::new(
        "AGAP_CONEXCreateController",
        conex_args(),
        "AGAP_CONEXCreateController(portName, serialPortName, controllerID, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let serial_port = req_string(args, 1, "Serial port name")?;
            let controller_id = c_int(args, 2);
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port(ctx, &port_name, agap::NUM_AXES as i32)?;
            // C returns from the constructor here: the port has no axes.
            let (handle, connected) = connect_controller(
                &serial_port,
                AGAP_TIMEOUT,
                &port_name,
                format_args!(
                    "AGAP_CONEXController::AGAP_CONEXController: cannot connect to CONEX controller"
                ),
            );
            if connected.is_err() {
                return Ok(CommandOutcome::Continue);
            }
            let Some(controller) =
                AgapController::new(handle, controller_id, MotorTrace::new(&port_name, 0))
            else {
                return Ok(CommandOutcome::Continue);
            };
            let controller = Arc::new(Mutex::new(controller));
            for axis in 0..agap::NUM_AXES {
                let motor: Arc<Mutex<dyn AsynMotor>> =
                    Arc::new(Mutex::new(AgapAxis::new(controller.clone(), axis)));
                install(
                    ctx,
                    asyn_user(&port_name, axis as i32),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                    2, // C startPoller, AGAP_CONEX.cpp:79
                )?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// A controller `AG_UCCreateController` made, for `AG_UCCreateAxis` to find
/// by port name.
struct AgUcRegistration {
    port_name: String,
    controller: Arc<Mutex<AgUcController>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
}

static AG_UC_CONTROLLERS: Mutex<Vec<AgUcRegistration>> = Mutex::new(Vec::new());

/// `AG_UCCreateController(portName, serialPortName, numAxes,
/// movingPollPeriod, idlePollPeriod)`; the axes come from `AG_UCCreateAxis`.
pub fn ag_uc_create_controller_command() -> CommandDef {
    CommandDef::new(
        "AG_UCCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("Serial port name"),
            arg_int_req("Number of axes"),
            arg_int_req("Moving poll period (ms)"),
            arg_int_req("Idle poll period (ms)"),
        ],
        "AG_UCCreateController(portName, serialPortName, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let serial_port = req_string(args, 1, "Serial port name")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port(ctx, &port_name, c_int(args, 2))?;
            // C goes on to RS/MR/VE on the unconnected user; VE then fails and
            // the controller is left without a poller.
            let (handle, _) = connect_controller(
                &serial_port,
                AG_UC_TIMEOUT,
                &port_name,
                format_args!(
                    "AG_UCController::AG_UCController: cannot connect to Agilis controller"
                ),
            );
            let controller = Arc::new(Mutex::new(AgUcController::new(
                handle,
                &MotorTrace::new(&port_name, 0),
            )));
            AG_UC_CONTROLLERS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .push(AgUcRegistration {
                    port_name,
                    controller,
                    moving_poll_ms,
                    idle_poll_ms,
                });
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `AG_UCCreateAxis(controllerPortName, axis, hasLimits, forwardAmplitude,
/// reverseAmplitude)`, printing what C's does.
pub fn ag_uc_create_axis_command() -> CommandDef {
    CommandDef::new(
        "AG_UCCreateAxis",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Axis number"),
            arg_int_req("Has Limits"),
            arg_int_req("Forward amplitude"),
            arg_int_req("Reverse amplitude"),
        ],
        "AG_UCCreateAxis(controllerPortName, axis, hasLimits, forwardAmplitude, \
         reverseAmplitude)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Controller port name")?;
            let axis = c_int(args, 1);
            let (controller, moving_poll_ms, idle_poll_ms) = {
                let controllers = AG_UC_CONTROLLERS.lock().unwrap_or_else(|e| e.into_inner());
                let Some(reg) = controllers.iter().find(|r| r.port_name == port_name) else {
                    println!("AG_UCCreateAxis: Error port {port_name} not found");
                    return Ok(CommandOutcome::Continue);
                };
                (reg.controller.clone(), reg.moving_poll_ms, reg.idle_poll_ms)
            };
            let ax = AgUcAxis::new(
                controller,
                axis,
                c_int(args, 2) != 0,
                c_int(args, 3),
                c_int(args, 4),
            );
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
            install(
                ctx,
                asyn_user(&port_name, axis),
                motor,
                moving_poll_ms,
                idle_poll_ms,
                2, // C startPoller, AG_UC.cpp:99
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// A controller `XPSCreateController` made, found by port name by the
/// commands after it.
struct XpsRegistration {
    port_name: String,
    ip: String,
    ip_port: i32,
    /// `None` when the poll socket could not be opened.
    controller: Option<Arc<Mutex<XpsController>>>,
    /// The controller's `moveSocket_`, which runs profiles.
    profile_sock: Option<Arc<XpsSocket>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
    /// `XPSCreateProfile`'s FTP account.
    ftp_login: Option<(String, String)>,
}

static XPS_CONTROLLERS: Mutex<Vec<XpsRegistration>> = Mutex::new(Vec::new());

/// Run `f` on the XPS registered as `port_name`, or print C's
/// `XPSController:<function>: Error port <name> not found`.
fn with_xps<R>(
    port_name: &str,
    function: &str,
    f: impl FnOnce(&mut XpsRegistration) -> R,
) -> Option<R> {
    let mut controllers = XPS_CONTROLLERS.lock().unwrap_or_else(|e| e.into_inner());
    match controllers.iter_mut().find(|r| r.port_name == port_name) {
        Some(reg) => Some(f(reg)),
        None => {
            println!("XPSController:{function}: Error port {port_name} not found");
            None
        }
    }
}

/// `XPSCreateController(portName, IPAddress, IPPort, numAxes,
/// movingPollPeriod, idlePollPeriod, enableSetPosition,
/// setPositionSettlingTime)`; the axes come from `XPSCreateAxis`.
pub fn xps_create_controller_command() -> CommandDef {
    CommandDef::new(
        "XPSCreateController",
        vec![
            arg_str_req("Controller port name"),
            arg_str_req("IP address"),
            arg_int_req("IP port"),
            arg_int_req("Number of axes"),
            arg_int_req("Moving poll rate (ms)"),
            arg_int_req("Idle poll rate (ms)"),
            arg_int_req("Enable set position"),
            arg_int_req("Set position settling time (ms)"),
        ],
        "XPSCreateController(portName, IPAddress, IPPort, numAxes, movingPollPeriod, \
         idlePollPeriod, enableSetPosition, setPositionSettlingTime)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Controller port name")?;
            let ip = req_string(args, 1, "IP address")?;
            let ip_port = c_int(args, 2);
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 4, 5)?;
            let enable_set_position = c_int(args, 6) != 0;
            let settling = Duration::from_millis(c_int(args, 7).max(0) as u64);

            register_port(ctx, &port_name, c_int(args, 3))?;
            let poll = connect_to_server(ctx, &ip, ip_port);
            if poll.is_none() {
                println!(
                    "XPSController:XPSController: error calling TCP_ConnectToServer for pollSocket"
                );
            }
            let profile_sock = connect_to_server(ctx, &ip, ip_port).map(|handle| {
                Arc::new(XpsSocket::new(
                    handle,
                    &port_name,
                    SocketMode::Query,
                    XPS_PVT_EXEC_TIMEOUT,
                ))
            });
            if profile_sock.is_none() {
                println!(
                    "XPSController:XPSController: error calling TCP_ConnectToServer for moveSocket"
                );
            }
            let controller = match poll {
                Some(handle) => Some(Arc::new(Mutex::new(
                    XpsController::new(
                        XpsSocket::new(handle, &port_name, SocketMode::Query, XPS_POLL_TIMEOUT),
                        enable_set_position,
                        settling,
                    )
                    .map_err(|e| format!("XPSCreateController: {e}"))?,
                ))),
                None => None,
            };
            XPS_CONTROLLERS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .push(XpsRegistration {
                    port_name,
                    ip,
                    ip_port,
                    controller,
                    profile_sock,
                    moving_poll_ms,
                    idle_poll_ms,
                    ftp_login: None,
                });
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `XPSCreateAxis(portName, axis, positionerName, stepsPerUnit)`, printing
/// what C's does. Each axis opens its own move socket.
pub fn xps_create_axis_command() -> CommandDef {
    CommandDef::new(
        "XPSCreateAxis",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Axis number"),
            arg_str_req("Axis name"),
            arg_str_req("Steps per unit"),
        ],
        "XPSCreateAxis(portName, axis, positionerName, stepsPerUnit)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Controller port name")?;
            let axis = c_int(args, 1);
            let positioner_name = req_string(args, 2, "Axis name")?;
            let steps_per_unit = atof(&opt_string(args, 3)?.unwrap_or_default());
            if steps_per_unit == 0.0 {
                return Err("XPSCreateAxis: steps per unit must be non-zero".into());
            }
            let Some((controller, ip, ip_port, moving_poll_ms, idle_poll_ms)) =
                with_xps(&port_name, "XPSCreateAxis", |reg| {
                    (
                        reg.controller.clone(),
                        reg.ip.clone(),
                        reg.ip_port,
                        reg.moving_poll_ms,
                        reg.idle_poll_ms,
                    )
                })
            else {
                return Ok(CommandOutcome::Continue);
            };
            let Some(handle) = connect_to_server(ctx, &ip, ip_port) else {
                println!("XPSAxis:XPSAxis: error calling TCP_ConnectToServer for move socket");
                return Ok(CommandOutcome::Continue);
            };
            let Some(controller) = controller else {
                return Ok(CommandOutcome::Continue);
            };
            let move_sock = XpsSocket::new(handle, &port_name, SocketMode::Fire, XPS_MOVE_TIMEOUT);
            let ax = XpsAxis::new(
                controller,
                move_sock,
                &port_name,
                axis,
                &positioner_name,
                1.0 / steps_per_unit,
            );
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
            install(
                ctx,
                asyn_user(&port_name, axis),
                motor,
                moving_poll_ms,
                idle_poll_ms,
                10, // C startPoller, XPSController.cpp:197
            )?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `XPSCreateProfile(portName, maxPoints, ftpUsername, ftpPassword)`: the
/// FTP account `XPSBuildProfile` uploads with. Profiles here are not sized
/// up front, so `maxPoints` has no use.
pub fn xps_create_profile_command() -> CommandDef {
    CommandDef::new(
        "XPSCreateProfile",
        vec![
            arg_str_req("Controller port name"),
            arg_int_req("Max points"),
            arg_str_req("FTP username"),
            arg_str_req("FTP password"),
        ],
        "XPSCreateProfile(portName, maxPoints, ftpUsername, ftpPassword)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Controller port name")?;
            let login = (
                opt_string(args, 2)?.unwrap_or_default(),
                opt_string(args, 3)?.unwrap_or_default(),
            );
            with_xps(&port_name, "XPSCreateProfile", |reg| {
                reg.ftp_login = Some(login)
            });
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `XPSDisableAutoEnable(portName)`.
pub fn xps_disable_auto_enable_command() -> CommandDef {
    xps_controller_flag_command("XPSDisableAutoEnable", |c| c.set_auto_enable(false))
}

/// `XPSNoDisableError(portName)`: a disabled axis (state 20) no longer sets
/// the MSTA problem bit.
pub fn xps_no_disable_error_command() -> CommandDef {
    xps_controller_flag_command("XPSNoDisableError", XpsController::set_no_disable_error)
}

/// `XPSEnableMovingMode(portName)`: an axis's move is done when its move
/// socket answers, so one axis can move while another in its group moves.
pub fn xps_enable_moving_mode_command() -> CommandDef {
    xps_controller_flag_command("XPSEnableMovingMode", XpsController::set_moving_mode)
}

/// A one-argument command that sets a flag on a registered XPS controller.
fn xps_controller_flag_command(name: &'static str, set: fn(&mut XpsController)) -> CommandDef {
    CommandDef::new(
        name,
        vec![arg_str_req("Controller port name")],
        format!("{name}(portName)"),
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Controller port name")?;
            with_xps(&port_name, name, |reg| {
                if let Some(controller) = &reg.controller {
                    set(&mut controller.lock().unwrap_or_else(|e| e.into_inner()));
                }
            });
            Ok(CommandOutcome::Continue)
        },
    )
}

/// The XPS registered as `motor_port`, for the driver-private commands.
fn xps_controller(motor_port: &str, cmd: &str) -> Result<Arc<Mutex<XpsController>>, String> {
    let controllers = XPS_CONTROLLERS.lock().unwrap_or_else(|e| e.into_inner());
    controllers
        .iter()
        .find(|r| r.port_name == motor_port)
        .and_then(|r| r.controller.clone())
        .ok_or_else(|| {
            format!("{cmd}: controller '{motor_port}' not found (call XPSCreateController first)")
        })
}

/// Create the `XPSTclScriptExecute` iocsh command.
///
/// Usage:
/// `XPSTclScriptExecute(motorPort, tclFile, [taskName], [parameters])`
///
/// Runs a TCL script file on the XPS registered under `motorPort`, over its
/// shared poll socket. Ports `TCLScriptExecute`; `taskName`/`parameters`
/// default to `"0"` as the C driver does. DEVIATION from C's record-driven
/// trigger (`XPSTclScript_`/`XPSTclScriptExecute_` asyn params): this is an
/// imperative iocsh command, the natural interface for a one-shot script.
pub fn xps_tcl_script_execute_command() -> CommandDef {
    CommandDef::new(
        "XPSTclScriptExecute",
        vec![
            arg_str_req("motorPort"),
            arg_str_req("tclFile"),
            ArgDesc {
                name: "taskName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "parameters",
                arg_type: ArgType::String,
            },
        ],
        "XPSTclScriptExecute(motorPort, tclFile, [taskName], [parameters]) - Run a TCL script file on a Newport XPS controller",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let tcl_file = req_string(args, 1, "tclFile")?;
            let task_name = opt_string(args, 2)?.unwrap_or_else(|| "0".to_string());
            let parameters = opt_string(args, 3)?.unwrap_or_else(|| "0".to_string());

            let controller = xps_controller(&motor_port, "XPSTclScriptExecute")?;
            let run = controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .poll_socket()
                .tcl_script_execute(&tcl_file, &task_name, &parameters);
            if let Err(e) = run {
                MotorTrace::new(&motor_port, 0).print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "TCLScriptExecute returned error {}, on XPS: {motor_port}\n",
                        e.code()
                    ),
                );
                return Ok(CommandOutcome::Continue);
            }
            println!(
                "XPSTclScriptExecute: motorPort={motor_port} tclFile={tcl_file} task={task_name} params={parameters}"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `tclCall.cc` `getsocket`: the XPS `tclcall` runs on, fixed in C's
/// source, and its `TIMEOUT`.
const TCLCALL_IP: &str = "164.54.160.124";
const TCLCALL_PORT: i32 = 5001;
const TCLCALL_TIMEOUT: Duration = Duration::from_secs(1);

/// `tclcall(tclName, taskName, args)` (tclCall.cc): run a TCL script on
/// the XPS at `TCLCALL_IP` over a socket of its own.
pub fn tclcall_command() -> CommandDef {
    CommandDef::new(
        "tclcall",
        vec![
            arg_str_opt("tcl name"),
            arg_str_opt("Task name"),
            arg_str_opt("Function args"),
        ],
        "tclcall(tclName, taskName, args)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            // printf's "%s" of a missing iocsh string argument.
            let text = |i: usize| match args.get(i) {
                Some(ArgValue::String(s)) => s.to_str().map(str::to_owned),
                _ => Ok("(null)".to_owned()),
            };
            println!("XPS ip = {TCLCALL_IP}");
            let socket = connect_to_server(ctx, TCLCALL_IP, TCLCALL_PORT).map(|handle| {
                let port = handle.port_name().to_owned();
                XpsSocket::new(handle, &port, SocketMode::Query, TCLCALL_TIMEOUT)
            });
            if socket.is_none() {
                println!(" Error TCP_ConnectToServer");
            }
            tclcall(socket.as_ref(), &text(0)?, &text(1)?, &text(2)?);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `XPSAuxConfig(portName, IP address, IP port, polling period)`: C
/// `drvXPSAsynAux.c`, the XPS analog and digital I/O as asyn port `portName`,
/// its inputs posted every `polling period` ms.
pub fn xps_aux_config_command() -> CommandDef {
    CommandDef::new(
        "XPSAuxConfig",
        vec![
            arg_str_opt("portName"),
            arg_str_opt("IP address"),
            arg_int_opt("IP port"),
            arg_int_opt("polling period"),
        ],
        "XPSAuxConfig(portName, IP address, IP port, polling period)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = opt_string(args, 0)?.unwrap_or_default();
            let ip = opt_string(args, 1)?.unwrap_or_default();
            let Some(handle) = connect_to_server(ctx, &ip, c_int(args, 2)) else {
                println!("drvXPSAsynAuxConfig: error calling TCP_ConnectToServer");
                return Ok(CommandOutcome::Continue);
            };
            let period = Duration::from_millis(c_int(args, 3).max(0) as u64);
            let socket = XpsSocket::new(
                handle.clone(),
                handle.port_name(),
                SocketMode::Query,
                gpio::TCP_TIMEOUT,
            );
            let registered = gpio::XpsGpio::new(&port_name, socket)
                .ok()
                .and_then(|driver| {
                    let claim = claim_port(&port_name).ok()?;
                    let (rt, _jh) =
                        create_port_runtime(&claim, driver, RuntimeConfig::default()).ok()?;
                    match claim.publish(
                        rt.port_handle().clone(),
                        ctx.db().scan_control().interrupt_gate(),
                    ) {
                        Ok(()) => Some(rt.port_handle().clone()),
                        Err(_) => {
                            rt.shutdown();
                            None
                        }
                    }
                });
            let Some(port) = registered else {
                eprintln!("XPSAuxConfig ERROR: Can't register port");
                return Ok(CommandOutcome::Continue);
            };
            if spawn_dedicated_thread(
                "XPSAuxPoller".into(),
                ThreadPriority::Medium,
                StackSizeClass::Medium,
                move || gpio::run_poller(port, period),
            )
            .is_err()
            {
                eprintln!("XPSAuxConfig, epicsThreadCreate failed");
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `tclcall` after `getsocket`; returns its status.
fn tclcall(socket: Option<&XpsSocket>, name: &str, task_name: &str, args: &str) -> i32 {
    let status = match socket {
        Some(socket) => socket
            .tcl_script_execute(name, task_name, args)
            .map_or_else(|e| e.code(), |()| 0),
        None => {
            // SendAndReceive on socket -1 (asynOctetSocket.cpp:137-140).
            println!("SendAndReceive: invalid SocketIndex -1");
            -22
        }
    };
    println!("TCL Call Status {status}");
    if status < 0 {
        println!("Error Name called {name}, Task Name {task_name}, Args {args}");
    }
    status
}

/// Create the `XPSPositionCompare` iocsh command.
///
/// `XPSPositionCompare(motorPort, positionerName, mode, [minPosition],
/// [maxPosition], [positionStep], [pulseWidth], [settlingTime])`
///
/// Applies a position-compare-output configuration to one positioner
/// (C `XPSAxis::setPositionCompare`): mode 0=Disable, 1=Pulse,
/// 2=AquadB-windowed, 3=AquadB-always. Positions/step are device units;
/// `pulseWidth`/`settlingTime` are µs (valid widths {0.2,1,2.5,10}, valid
/// settling {0.075,1,4,12}), defaulting to the smallest table entries.
/// Driver-private: the C base-class PCO API (05b25c1d, motor PR #248) is an
/// open, unmerged PR, so PCO is not part of the motor framework.
pub fn xps_position_compare_command() -> CommandDef {
    CommandDef::new(
        "XPSPositionCompare",
        vec![
            arg_str_req("motorPort"),
            arg_str_req("positionerName"),
            ArgDesc {
                name: "mode",
                arg_type: ArgType::Int,
            },
            arg_double_opt("minPosition"),
            arg_double_opt("maxPosition"),
            arg_double_opt("positionStep"),
            arg_double_opt("pulseWidth"),
            arg_double_opt("settlingTime"),
        ],
        "XPSPositionCompare(motorPort, positionerName, mode, [minPosition], [maxPosition], [positionStep], [pulseWidth], [settlingTime]) - Configure position-compare output on an XPS positioner (mode 0=Disable 1=Pulse 2=AquadB-windowed 3=AquadB-always; positions in device units, widths/settling in us)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let positioner = req_string(args, 1, "positionerName")?;
            let mode = match args.get(2) {
                Some(ArgValue::Int(v)) => *v as i32,
                _ => return Err("mode must be an integer".into()),
            };
            let params = PcoParams {
                mode,
                min_position: opt_double(args, 3, 0.0, "minPosition")?,
                max_position: opt_double(args, 4, 0.0, "maxPosition")?,
                position_step: opt_double(args, 5, 0.0, "positionStep")?,
                pulse_width_us: opt_double(
                    args,
                    6,
                    pco::XPS_PCO_DEFAULT_PULSE_WIDTH,
                    "pulseWidth",
                )?,
                settling_time_us: opt_double(
                    args,
                    7,
                    pco::XPS_PCO_DEFAULT_SETTLING_TIME,
                    "settlingTime",
                )?,
            };

            let controller = xps_controller(&motor_port, "XPSPositionCompare")?;
            {
                let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
                pco::set_position_compare(ctrl.poll_socket(), &positioner, &params)
                    .map_err(|e| format!("XPSPositionCompare: {e}"))?;
            }
            println!(
                "XPSPositionCompare: motorPort={motor_port} positioner={positioner} mode={mode} min={} max={} step={} pw={}us settle={}us",
                params.min_position,
                params.max_position,
                params.position_step,
                params.pulse_width_us,
                params.settling_time_us,
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Create the `XPSDefineProfileFromFile` iocsh command.
///
/// `XPSDefineProfileFromFile(motorPort, group, csvFile, [moveMode])`
///
/// Loads a PVT profile for `group` from a CSV points file: each row is
/// `time, pos_0, pos_1, ...` with one position column per positioner in the
/// group (controller registration order), in device units and seconds; `#`
/// comments and blank lines are skipped. `moveMode` is `absolute` (default)
/// or `relative`, selecting whether the execute-time move to the trajectory
/// start is absolute or relative. Driver-private: the epics-rs motor
/// framework has no profileMove record subsystem, so profiles are defined by
/// this command instead of by the C `profileMove` database.
pub fn xps_define_profile_command() -> CommandDef {
    CommandDef::new(
        "XPSDefineProfileFromFile",
        vec![
            arg_str_req("motorPort"),
            arg_str_req("group"),
            arg_str_req("csvFile"),
            ArgDesc {
                name: "moveMode",
                arg_type: ArgType::String,
            },
        ],
        "XPSDefineProfileFromFile(motorPort, group, csvFile, [moveMode]) - Load a PVT profile for an XPS group from a CSV points file (time + one position column per group positioner; moveMode absolute|relative, default absolute)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let group = req_string(args, 1, "group")?;
            let csv_file = req_string(args, 2, "csvFile")?;
            let move_mode = parse_move_mode(opt_string(args, 3)?)?;

            let controller = xps_controller(&motor_port, "XPSDefineProfileFromFile")?;
            let csv = std::fs::read_to_string(&csv_file)
                .map_err(|e| format!("XPSDefineProfileFromFile: cannot read '{csv_file}': {e}"))?;

            let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let positioners = ctrl.positioners_in_group(&group);
            let num_axes = positioners.len();
            let profile = Profile::from_csv(&group, move_mode, &positioners, &csv)
                .map_err(|e| format!("XPSDefineProfileFromFile: {e}"))?;
            let num_points = profile.num_points();
            ctrl.define_profile(profile)
                .map_err(|e| format!("XPSDefineProfileFromFile: {e}"))?;
            drop(ctrl);
            println!(
                "XPSDefineProfileFromFile: motorPort={motor_port} group={group} points={num_points} axes={num_axes} mode={move_mode:?}"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Create the `XPSBuildProfile` iocsh command.
///
/// `XPSBuildProfile(motorPort, fileName, host, [ftpUser], [ftpPassword], [ftpDir])`
///
/// Generates the defined profile's trajectory file, FTP-uploads it to the
/// XPS trajectory folder, then verifies it against the group dynamics and
/// each axis's soft limits (C `buildProfile`). `fileName` is the bare
/// trajectory file name the controller opens; `host` is the XPS IP/hostname
/// (no port). Credentials and directory default to the XPS factory
/// `Administrator` account and `/Admin/Public/Trajectories`; `ftpDir` must
/// name the controller's actual trajectory folder if overridden.
pub fn xps_build_profile_command() -> CommandDef {
    CommandDef::new(
        "XPSBuildProfile",
        vec![
            arg_str_req("motorPort"),
            arg_str_req("fileName"),
            arg_str_req("host"),
            ArgDesc {
                name: "ftpUser",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "ftpPassword",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "ftpDir",
                arg_type: ArgType::String,
            },
        ],
        "XPSBuildProfile(motorPort, fileName, host, [ftpUser], [ftpPassword], [ftpDir]) - Generate the defined profile's trajectory file, FTP it to the XPS, and verify it against dynamics + soft limits",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let file_name = req_string(args, 1, "fileName")?;
            let host = req_string(args, 2, "host")?;
            let (default_user, default_pass) =
                with_xps(&motor_port, "XPSBuildProfile", |reg| reg.ftp_login.clone())
                    .flatten()
                    .unwrap_or_else(|| (XPS_FTP_USER.into(), XPS_FTP_PASSWORD.into()));
            let ftp_user = opt_string(args, 3)?.unwrap_or(default_user);
            let ftp_pass = opt_string(args, 4)?.unwrap_or(default_pass);
            let ftp_dir = opt_string(args, 5)?.unwrap_or_else(|| XPS_TRAJECTORY_DIR.to_string());

            let controller = xps_controller(&motor_port, "XPSBuildProfile")?;

            // Generate and verify hold the controller lock (they use the
            // poll socket); the FTP upload touches no controller state, so
            // it runs between the two without the lock held.
            let build = || {
                let built = controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .build_profile_text()?;
                ftp::upload_trajectory(
                    &host,
                    &ftp_dir,
                    &file_name,
                    &built.text,
                    &ftp_user,
                    &ftp_pass,
                )?;
                controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .verify_profile(&file_name, &built)
            };
            if let Err(message) = build() {
                xps_profile_failed(&motor_port, "buildProfile", &message);
                return Ok(CommandOutcome::Continue);
            }
            println!(
                "XPSBuildProfile: motorPort={motor_port} file={file_name} uploaded to {host}:{ftp_dir} and verified"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Create the `XPSExecuteProfile` iocsh command.
///
/// `XPSExecuteProfile(motorPort, [executions], [startPulses], [endPulses],
/// [numPulses])`
///
/// Moves the group to the trajectory start, configures gathering (each
/// trajectory pulse samples every registered positioner's
/// `SetpointPosition`+`CurrentPosition` via a `GatheringOneData` event),
/// then runs the built PVT profile with `MultipleAxesPVTExecution` and
/// moves each axis to the profile's end (C `runProfile` flyback). The
/// moves and the execution block until the motion finishes, so everything
/// runs on a background thread over the controller's move socket, as C's
/// does — the shared poll socket keeps polling meanwhile (C configures
/// gathering on the poll socket; same RPCs, the move socket keeps the poll
/// socket uncontended). `executions` (default 1)
/// repeats the trajectory. `startPulses`/`endPulses` (1-based trajectory
/// elements, defaults: the whole profile) window the pulse output and
/// `numPulses` (default one per element) spreads that many pulses over it,
/// exactly as C `executeProfile`. Read the samples back afterwards with
/// `XPSReadbackProfile`.
pub fn xps_execute_profile_command() -> CommandDef {
    CommandDef::new(
        "XPSExecuteProfile",
        vec![
            arg_str_req("motorPort"),
            arg_int_opt("executions"),
            arg_int_opt("startPulses"),
            arg_int_opt("endPulses"),
            arg_int_opt("numPulses"),
        ],
        "XPSExecuteProfile(motorPort, [executions], [startPulses], [endPulses], [numPulses]) - Move the group to the trajectory start and run the built PVT profile with gathering on a background thread (executions default 1; pulse window defaults to one pulse per trajectory element)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let executions = match args.get(1) {
                Some(ArgValue::Int(v)) if *v >= 1 => *v as i32,
                None | Some(ArgValue::Missing) => 1,
                Some(ArgValue::Int(_)) => return Err("executions must be >= 1".into()),
                _ => return Err("executions must be an integer".into()),
            };

            let controller = xps_controller(&motor_port, "XPSExecuteProfile")?;
            let plan = controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .execution_plan();
            let Some(plan) = plan else {
                xps_profile_failed(
                    &motor_port,
                    "runProfile",
                    "no built profile (call XPSBuildProfile first)",
                );
                return Ok(CommandOutcome::Continue);
            };

            // Pulse window: default one pulse per trajectory element over
            // the whole profile (C reads these from records).
            let num_elements = plan.times.len().saturating_sub(1) as i64;
            let start_pulses = opt_int(args, 2, 1, "startPulses")?;
            let end_pulses = opt_int(args, 3, num_elements, "endPulses")?;
            let num_pulses = opt_int(args, 4, num_elements, "numPulses")?;
            let Some(exec_sock) = with_xps(&motor_port, "XPSExecuteProfile", |reg| {
                reg.profile_sock.clone()
            })
            .flatten() else {
                return Ok(CommandOutcome::Continue);
            };

            println!(
                "XPSExecuteProfile: motorPort={motor_port} group={} file={} executions={executions} pulses={num_pulses} over elements {start_pulses}..{end_pulses} (running on background thread)",
                plan.group, plan.file_name
            );
            // C's XPSProfile thread (XPSController.cpp:206-209).
            spawn_dedicated_thread(
                "XPSProfile".into(),
                ThreadPriority::Low,
                StackSizeClass::Medium,
                move || {
                    match xps_run_profile(
                        &exec_sock,
                        &plan,
                        executions,
                        (start_pulses as i32, end_pulses as i32, num_pulses as i32),
                    ) {
                        None => println!(
                            "XPSExecuteProfile: trajectory '{}' complete (read samples with XPSReadbackProfile)",
                            plan.file_name
                        ),
                        Some(message) => xps_profile_failed(&motor_port, "runProfile", &message),
                    }
                },
            )
            .map_err(|e| format!("XPSExecuteProfile: cannot create XPSProfile thread: {e}"))?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Run a built trajectory: move to the start, configure gathering, execute,
/// remove the event, stop gathering and fly back to the end (C
/// `runProfile`, XPSController.cpp:862-1145). `pulses` is
/// `(startPulses, endPulses, numPulses)`. Returns C's message for the last
/// failed step, which C prints once at the end.
fn xps_run_profile(
    sock: &XpsSocket,
    plan: &ExecutionPlan,
    executions: i32,
    pulses: (i32, i32, i32),
) -> Option<String> {
    // C ignores both moves' status.
    xps_profile_moves(sock, plan.move_mode, &plan.start_moves);
    let message = xps_execute_gathered(sock, plan, executions, pulses);
    xps_profile_moves(sock, plan.move_mode, &plan.end_moves);
    message
}

/// Move each axis in turn; a move on the blocking socket returns when the
/// motion is done, as C's `waitMotors` waits after it.
fn xps_profile_moves(sock: &XpsSocket, mode: MoveMode, moves: &[(String, f64)]) {
    for (positioner, target) in moves {
        let _ = match mode {
            MoveMode::Absolute => sock.group_move_absolute(positioner, *target),
            MoveMode::Relative => sock.group_move_relative(positioner, *target),
        };
    }
}

/// The part of C `runProfile` between the start move and `done:`.
fn xps_execute_gathered(
    sock: &XpsSocket,
    plan: &ExecutionPlan,
    executions: i32,
    pulses: (i32, i32, i32),
) -> Option<String> {
    let event_id = match xps_start_gathering(sock, plan, pulses) {
        Ok(id) => id,
        Err(message) => return Some(message),
    };

    let mut message =
        match sock.multiple_axes_pvt_execution(&plan.group, &plan.file_name, executions) {
            Ok(()) => None,
            // -27: the trajectory was aborted.
            Err(XpsError::Api(-27)) => Some("MultipleAxesPVTExecution aborted".to_string()),
            Err(e) => Some(format!(
                "Error performing MultipleAxesPVTExecution, status={}",
                e.code()
            )),
        };
    // C removes the event and stops gathering whatever the run returned.
    if let Err(e) = sock.event_extended_remove(event_id) {
        message = Some(format!(
            "Error performing ExtendedEventRemove, status={}",
            e.code()
        ));
    }
    match sock.gathering_stop() {
        // -30: gathering never started (aborted before one element
        // completed); C tolerates it.
        Ok(()) | Err(XpsError::Api(-30)) => {}
        Err(e) => {
            message = Some(format!(
                "Error performing GatheringStop, status={}",
                e.code()
            ))
        }
    }
    message
}

/// Configure and start gathering for a trajectory run: reset the buffer,
/// declare `SetpointPosition`+`CurrentPosition` per registered positioner,
/// window the trajectory pulse train, and fire `GatheringOneData` on every
/// pulse (C `runProfile`, XPSController.cpp:935-1060). Returns the
/// extended-event ID to remove after the run, or C's message.
fn xps_start_gathering(
    sock: &XpsSocket,
    plan: &ExecutionPlan,
    (start_pulses, end_pulses, num_pulses): (i32, i32, i32),
) -> Result<i32, String> {
    sock.gathering_reset()
        .map_err(|e| format!("Error performing GatheringReset, status={}", e.code()))?;
    let types: Vec<String> = plan
        .gathering_positioners
        .iter()
        .flat_map(|p| {
            [
                format!("{p}.SetpointPosition"),
                format!("{p}.CurrentPosition"),
            ]
        })
        .collect();
    sock.gathering_configuration_set(&types).map_err(|e| {
        let buffer: String = types.iter().map(|t| format!("{t};")).collect();
        format!(
            "Error performing GatheringConfigurationSet, status={}, buffer={buffer}",
            e.code()
        )
    })?;
    let (pulse_start, pulse_end, pulse_period) =
        profile::pulse_output_window(&plan.times, start_pulses, end_pulses, num_pulses)?;
    // C ignores its status.
    let _ =
        sock.multiple_axes_pvt_pulse_output_set(&plan.group, pulse_start, pulse_end, pulse_period);
    let triggers = vec![
        "Always".to_string(),
        format!("{}.PVT.TrajectoryPulse", plan.group),
    ];
    sock.event_extended_configuration_trigger_set(&triggers)
        .map_err(|e| {
            let buffer = triggers.join(";");
            format!(
                "Error performing EventExtendedConfigurationTriggerSet, status={}, buffer={buffer}, strlen(buffer)={}",
                e.code(),
                buffer.len()
            )
        })?;
    sock.event_extended_configuration_action_set(&["GatheringOneData".to_string()])
        .map_err(|e| {
            format!(
                "Error performing EventExtendedConfigurationActionSet, status={}",
                e.code()
            )
        })?;
    sock.event_extended_start()
        .map_err(|e| format!("Error performing EventExtendedStart, status={}", e.code()))
}

/// Create the `XPSReadbackProfile` iocsh command.
///
/// `XPSReadbackProfile(motorPort, outputFile)`
///
/// Reads the gathering samples collected during the last
/// `XPSExecuteProfile` run over the shared poll socket
/// (`GatheringCurrentNumberGet` + chunked `GatheringDataMultipleLinesGet`,
/// halving the request on a controller error exactly as C
/// `readbackProfile`) and writes them to `outputFile` as CSV — one row per
/// trajectory pulse with `actual, following_error` per registered
/// positioner, the file-based counterpart of C posting the readback
/// waveforms (positions are device units, like the profile CSV).
pub fn xps_readback_profile_command() -> CommandDef {
    CommandDef::new(
        "XPSReadbackProfile",
        vec![arg_str_req("motorPort"), arg_str_req("outputFile")],
        "XPSReadbackProfile(motorPort, outputFile) - Read the gathering samples from the last XPSExecuteProfile run and write them to outputFile as CSV (actual position + following error per registered positioner, device units)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let output_file = req_string(args, 1, "outputFile")?;

            let controller = xps_controller(&motor_port, "XPSReadbackProfile")?;
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let positioners = ctrl.registered_positioners();
            if positioners.is_empty() {
                return Err("XPSReadbackProfile: no axes registered".into());
            }
            let sock = ctrl.poll_socket();

            let (current_samples, _max_samples) = match sock.gathering_current_number_get() {
                Ok(n) => n,
                Err(e) => {
                    let message =
                        format!("Error calling GatherCurrentNumberGet, status={}", e.code());
                    xps_profile_failed(&motor_port, "readbackProfile", &message);
                    return Ok(CommandOutcome::Continue);
                }
            };

            let mut readback = GatheringReadback::new(positioners.len());
            let mut num_read = 0i32;
            let mut read = || -> Result<(), String> {
                while num_read < current_samples {
                    // Ask for everything left; on a controller error halve
                    // the request until it fits (C readbackProfile).
                    let mut lines = current_samples - num_read;
                    let buffer = loop {
                        match sock.gathering_data_multiple_lines_get(num_read, lines) {
                            Ok(buf) => break buf,
                            Err(_) if lines > 1 => lines /= 2,
                            Err(_) => {
                                return Err("Error reading gathering data, numInBuffer = 0".into());
                            }
                        }
                    };
                    let parsed = gathering::parse_gathering_buffer(
                        &buffer,
                        positioners.len(),
                        &mut readback,
                    )?;
                    if parsed == 0 {
                        return Err("gathering returned no lines".into());
                    }
                    num_read += parsed as i32;
                }
                Ok(())
            };
            if let Err(message) = read() {
                xps_profile_failed(&motor_port, "readbackProfile", &message);
                return Ok(CommandOutcome::Continue);
            }

            let csv = gathering::readback_csv(&positioners, &readback);
            std::fs::write(&output_file, csv)
                .map_err(|e| format!("XPSReadbackProfile: writing {output_file}: {e}"))?;
            println!(
                "XPSReadbackProfile: wrote {num_read} samples x {} positioners to {output_file}",
                positioners.len()
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// A hexapod `HXPCreateController` made. Keeps a `Fire`-mode move socket of
/// its own for the driver-private commands.
struct HxpRegistration {
    port_name: String,
    controller: Arc<Mutex<HxpController>>,
    move_sock: XpsSocket,
}

static HXP_CONTROLLERS: Mutex<Vec<HxpRegistration>> = Mutex::new(Vec::new());

/// `HXPCreateController(portName, IPAddress, IPPort, movingPollPeriod,
/// idlePollPeriod)`: the controller and its six axes X, Y, Z, U, V, W
/// (0..5), each axis on a move socket of its own.
pub fn hxp_create_controller_command() -> CommandDef {
    CommandDef::new(
        "HXPCreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("IP address"),
            arg_int_req("Port"),
            arg_int_req("Moving poll period (ms)"),
            arg_int_req("Idle poll period (ms)"),
        ],
        "HXPCreateController(portName, IPAddress, IPPort, movingPollPeriod, idlePollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let ip = req_string(args, 1, "IP address")?;
            let ip_port = c_int(args, 2);
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;

            register_port(ctx, &port_name, NUM_HXP_AXES as i32)?;
            let Some(handle) = connect_to_server(ctx, &ip, ip_port) else {
                println!(
                    "HXPDriver:HXPController::HXPController: error calling TCP_ConnectToServer for pollSocket"
                );
                return Ok(CommandOutcome::Continue);
            };
            let poll_sock = XpsSocket::new(handle, &port_name, SocketMode::Query, HXP_POLL_TIMEOUT);
            // One group poll serves all six axis polls per period: cache for
            // half the fastest poll interval.
            let cache_ttl = Duration::from_millis(moving_poll_ms / 2);
            let controller = Arc::new(Mutex::new(HxpController::new(poll_sock, cache_ttl)));

            for axis_no in 0..NUM_HXP_AXES {
                let Some(handle) = connect_to_server(ctx, &ip, ip_port) else {
                    continue;
                };
                let move_sock =
                    XpsSocket::new(handle, &port_name, SocketMode::Fire, HXP_MOVE_TIMEOUT);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(HxpAxis::new(
                    controller.clone(),
                    move_sock,
                    axis_no,
                )));
                install(
                    ctx,
                    asyn_user(&port_name, axis_no as i32),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                    2, // C startPoller, HXPDriver.cpp:116
                )?;
            }

            let Some(handle) = connect_to_server(ctx, &ip, ip_port) else {
                return Ok(CommandOutcome::Continue);
            };
            HXP_CONTROLLERS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .push(HxpRegistration {
                    port_name: port_name.clone(),
                    controller,
                    move_sock: XpsSocket::new(
                        handle,
                        &port_name,
                        SocketMode::Fire,
                        HXP_MOVE_TIMEOUT,
                    ),
                });
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Run `f` with the HXP registration for `motor_port` (the sockets are not
/// clonable, so callers borrow the registration under the lock — every
/// hexapod command RPC is a single bounded exchange).
fn with_hxp_registration<R>(
    motor_port: &str,
    cmd: &str,
    f: impl FnOnce(&HxpRegistration) -> Result<R, String>,
) -> Result<R, String> {
    let controllers = HXP_CONTROLLERS.lock().unwrap_or_else(|e| e.into_inner());
    let reg = controllers
        .iter()
        .find(|r| r.port_name == motor_port)
        .ok_or_else(|| {
            format!("{cmd}: controller '{motor_port}' not found (call HXPCreateController first)")
        })?;
    f(reg)
}

/// Create the `HXPMoveAll` iocsh command.
///
/// `HXPMoveAll(motorPort, x, y, z, u, v, w)`
///
/// Moves all six hexapod axes to absolute Work-coordinate targets with a
/// single `HexapodMoveAbsolute` (C `HXPController::moveAll`, which reads
/// the targets from the `HXP_MOVE_ALL_TARGET_*` records; here they are
/// arguments). Fire-and-forget on the move socket — poll the motor records
/// for completion.
pub fn hxp_move_all_command() -> CommandDef {
    CommandDef::new(
        "HXPMoveAll",
        vec![
            arg_str_req("motorPort"),
            arg_double_req("x"),
            arg_double_req("y"),
            arg_double_req("z"),
            arg_double_req("u"),
            arg_double_req("v"),
            arg_double_req("w"),
        ],
        "HXPMoveAll(motorPort, x, y, z, u, v, w) - Move all six hexapod axes to absolute Work-coordinate targets in one motion (device units)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let mut targets = [0.0; 6];
            for (i, t) in targets.iter_mut().enumerate() {
                *t = req_double(args, i + 1, ["x", "y", "z", "u", "v", "w"][i])?;
            }
            with_hxp_registration(&motor_port, "HXPMoveAll", |reg| {
                reg.move_sock
                    .hexapod_move_absolute(HXP_GROUP, "Work", &targets)
                    .map_err(|e| format!("HXPMoveAll: {e}"))
            })?;
            println!(
                "HXPMoveAll: motorPort={motor_port} targets={targets:?} (Work coordinates, fire-and-forget)"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Create the `HXPCoordSysRead` iocsh command.
///
/// `HXPCoordSysRead(motorPort)`
///
/// Reads and prints the Tool, Work, and Base coordinate-system definitions
/// (C `HXPController::readAllCS`, which posts them to the
/// `HXP_COORD_SYS_*` records; here they are printed). Runs on the poll
/// socket (DEVIATION: C reads over the move socket's 0.1 s best-effort
/// read; the `Query` poll socket waits for the reply reliably).
pub fn hxp_coord_sys_read_command() -> CommandDef {
    CommandDef::new(
        "HXPCoordSysRead",
        vec![arg_str_req("motorPort")],
        "HXPCoordSysRead(motorPort) - Read and print the hexapod Tool/Work/Base coordinate-system definitions",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let controller = with_hxp_registration(&motor_port, "HXPCoordSysRead", |reg| {
                Ok(reg.controller.clone())
            })?;
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let sock = ctrl.poll_socket();
            for cs in ["Tool", "Work", "Base"] {
                let p = sock
                    .hexapod_coordinate_system_get(HXP_GROUP, cs)
                    .map_err(|e| format!("HXPCoordSysRead: {cs}: {e}"))?;
                println!(
                    "HXPCoordSysRead: {cs:<4} X={:.6} Y={:.6} Z={:.6} U={:.6} V={:.6} W={:.6}",
                    p[0], p[1], p[2], p[3], p[4], p[5]
                );
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Create the `HXPCoordSysSet` iocsh command.
///
/// `HXPCoordSysSet(motorPort, coordSystem, x, y, z, u, v, w)`
///
/// Redefines the origin of one hexapod coordinate system
/// (`Work`/`Tool`/`Base`, or C's record encoding `1`/`2`/`3`) —
/// C `HXPController::setCS`. Runs on the poll socket (same DEVIATION as
/// `HXPCoordSysRead`).
pub fn hxp_coord_sys_set_command() -> CommandDef {
    CommandDef::new(
        "HXPCoordSysSet",
        vec![
            arg_str_req("motorPort"),
            arg_str_req("coordSystem"),
            arg_double_req("x"),
            arg_double_req("y"),
            arg_double_req("z"),
            arg_double_req("u"),
            arg_double_req("v"),
            arg_double_req("w"),
        ],
        "HXPCoordSysSet(motorPort, coordSystem, x, y, z, u, v, w) - Redefine the origin of a hexapod coordinate system (Work/Tool/Base)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let cs_arg = req_string(args, 1, "coordSystem")?;
            let cs = match cs_arg.to_ascii_lowercase().as_str() {
                "work" | "1" => "Work",
                "tool" | "2" => "Tool",
                "base" | "3" => "Base",
                other => {
                    return Err(format!(
                        "coordSystem must be Work, Tool or Base, got '{other}'"
                    ));
                }
            };
            let mut origin = [0.0; 6];
            for (i, t) in origin.iter_mut().enumerate() {
                *t = req_double(args, i + 2, ["x", "y", "z", "u", "v", "w"][i])?;
            }
            let controller = with_hxp_registration(&motor_port, "HXPCoordSysSet", |reg| {
                Ok(reg.controller.clone())
            })?;
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            ctrl.poll_socket()
                .hexapod_coordinate_system_set(HXP_GROUP, cs, &origin)
                .map_err(|e| format!("HXPCoordSysSet: {e}"))?;
            println!("HXPCoordSysSet: motorPort={motor_port} {cs} origin={origin:?}");
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Create the `HXPSetMoveCoordSys` iocsh command.
///
/// `HXPSetMoveCoordSys(motorPort, coordSystem)`
///
/// Selects the coordinate system motor-record moves use: `Work`/`0`
/// (default) or `Tool`/`1` — the C `HXP_MOVE_COORD_SYS` record parameter
/// as a command.
pub fn hxp_set_move_coord_sys_command() -> CommandDef {
    CommandDef::new(
        "HXPSetMoveCoordSys",
        vec![arg_str_req("motorPort"), arg_str_req("coordSystem")],
        "HXPSetMoveCoordSys(motorPort, coordSystem) - Select the coordinate system for motor-record moves: Work/0 (default) or Tool/1",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let motor_port = req_string(args, 0, "motorPort")?;
            let cs_arg = req_string(args, 1, "coordSystem")?;
            let cs = match cs_arg.to_ascii_lowercase().as_str() {
                "work" | "0" => MoveCoordSys::Work,
                "tool" | "1" => MoveCoordSys::Tool,
                other => {
                    return Err(format!("coordSystem must be Work or Tool, got '{other}'"));
                }
            };
            let controller = with_hxp_registration(&motor_port, "HXPSetMoveCoordSys", |reg| {
                Ok(reg.controller.clone())
            })?;
            controller
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .set_move_coord_sys(cs);
            println!("HXPSetMoveCoordSys: motorPort={motor_port} coordSystem={cs:?}");
            Ok(CommandOutcome::Continue)
        },
    )
}

/// The `Config(card, name, addr)` arguments every model-1 driver here takes
/// (NewportRegister.cc:41-43).
fn model1_config_args() -> Vec<ArgDesc> {
    vec![
        arg_int_req("Card being configured"),
        arg_str_req("asyn port name"),
        arg_int_req("asyn address (GPIB)"),
    ]
}

/// A model-1 `<name>(card, port, addr)` whose card `build` fills, if the
/// driver's `Setup` sized a table holding `card`. `addr` is the GPIB address,
/// which an octet port at address 0 has no use for.
fn model1_config_command(
    name: &'static str,
    driver: &'static Model1,
    build: fn(&CommandContext, i32, &str) -> Result<(), String>,
) -> CommandDef {
    CommandDef::new(
        name,
        model1_config_args(),
        format!("{name}(card, name, addr)"),
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !driver.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let port = req_string(args, 1, "asyn port name")?;
            build(ctx, card, &port)?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devMM3000,"MM3000")` (devNewport.dbd).
static DEV_MM3000: DevSup = model1_device("devMM3000", "MM3000");
/// `MM3000_NUM_CARDS` (drvMM3000.cc:97).
static MM3000: Model1 = Model1::new(&DEV_MM3000, 4, SCAN_RATE);

/// `MM300Setup(num_cards, scan_rate)` — C registers it under this name
/// (NewportRegister.cc:56).
pub fn mm3000_setup_command() -> CommandDef {
    MM3000.setup_command("MM300Setup")
}

/// `MM3000Config(card, name, addr)`: one signal per axis slot the
/// controller's `RC` reports, including the trailing `unused` slot C counts.
pub fn mm3000_config_command() -> CommandDef {
    model1_config_command("MM3000Config", &MM3000, |ctx, card, port| {
        // C drops a card it cannot connect to without a word.
        let Ok(handle) = connect_serial(port, MM3000_TIMEOUT) else {
            return Ok(());
        };
        let Some(controller) = Mm3000Controller::new(handle) else {
            return Ok(());
        };
        let num_axes = controller.axis_types().len();
        let controller = Arc::new(Mutex::new(controller));
        for axis in 1..=num_axes {
            let ax = Mm3000Axis::new(controller.clone(), axis);
            MM3000.install(ctx, card, (axis - 1) as i32, Arc::new(Mutex::new(ax)))?;
        }
        Ok(())
    })
}

/// `device(motor,VME_IO,devMM4000,"MM4000")` (devNewport.dbd).
static DEV_MM4000: DevSup = model1_device("devMM4000", "MM4000");
/// `MM4000_NUM_CARDS` (drvMM4000.cc:111).
const MM4000_NUM_CARDS: i32 = 10;
static MM4000: Model1 = Model1::new(&DEV_MM4000, MM4000_NUM_CARDS, SCAN_RATE);

/// `MM4000Setup(num_cards, scan_rate)`, which alone of these warns about a
/// card count it clamps, between two 5 s sleeps (drvMM4000.cc:606-611).
pub fn mm4000_setup_command() -> CommandDef {
    MM4000.register_row();
    CommandDef::new(
        "MM4000Setup",
        vec![
            arg_int_req("Max. controller count"),
            arg_int_req("Polling rate"),
        ],
        "MM4000Setup(num_cards, scan_rate)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let num_cards = c_int(args, 0);
            if !(1..=MM4000_NUM_CARDS).contains(&num_cards) {
                std::thread::sleep(Duration::from_secs(5));
                errlog_vprintf(format_args!(
                    "\n*** ERROR *** Number specified ({num_cards}) exceeds maximum allowed ({MM4000_NUM_CARDS}).\n\n"
                ));
                std::thread::sleep(Duration::from_secs(5));
            }
            MM4000.setup(num_cards, c_int(args, 1));
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `MM4000Config(card, name, addr)`: one signal per axis the controller's
/// `TH;` reports.
pub fn mm4000_config_command() -> CommandDef {
    model1_config_command("MM4000Config", &MM4000, |ctx, card, port| {
        // C drops a card it cannot connect to without a word.
        let Ok(handle) = connect_serial(port, MM4000_TIMEOUT) else {
            return Ok(());
        };
        // Six per-axis polls share one controller-wide exchange: cache for
        // half the default scan period.
        let cache_ttl = Duration::from_millis(1000 / SCAN_RATE as u64 / 2);
        let controller = match Mm4000Controller::new_model1(handle, port, cache_ttl) {
            Ok(controller) => controller,
            Err(Mm4000Refusal::InvalidModel(ident)) => {
                errlog_vprintf(format_args!(
                    "drvMM4000.c:motor_init() - invalid model = {ident}\n"
                ));
                return Ok(());
            }
            Err(_) => return Ok(()),
        };
        let num_axes = controller.num_axes();
        let controller = Arc::new(Mutex::new(controller));
        for axis in 1..=num_axes {
            let mut ax = Mm4000Axis::new(controller.clone(), axis, mm4000::MODEL1_POLL_FAILURE);
            ax.model1_set_status();
            MM4000.install(ctx, card, (axis - 1) as i32, Arc::new(Mutex::new(ax)))?;
        }
        Ok(())
    })
}

/// drvMM4000Asyn.c's `motorAxisDrvSET_t`.
/// Its poller forces 10 fast polls (drvMM4000Asyn.c:630).
pub static MOTOR_MM4000: Model2 = Model2::new("motorMM4000", 10);

/// `numMM4000Controllers`, which `MM4000AsynSetup` sets.
static NUM_MM4000_ASYN: Mutex<i32> = Mutex::new(0);

/// `MM4000AsynSetup(num_controllers)`, printing what C's does.
pub fn mm4000_asyn_setup_command() -> CommandDef {
    CommandDef::new(
        "MM4000AsynSetup",
        vec![arg_int_req("Max. controller count")],
        "MM4000AsynSetup(num_controllers)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let num_controllers = c_int(args, 0);
            if num_controllers < 1 {
                println!("MM4000Setup, num_controllers must be > 0");
                return Ok(CommandOutcome::Continue);
            }
            *NUM_MM4000_ASYN.lock().unwrap_or_else(|e| e.into_inner()) = num_controllers;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `MM4000AsynConfig(card, portName, asynAddress, numAxes,
/// movingPollPeriod, idlePollPeriod)`, printing what C's does; builds card
/// `card` of [`MOTOR_MM4000`].
pub fn mm4000_asyn_config_command() -> CommandDef {
    CommandDef::new(
        "MM4000AsynConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
            arg_int_req("Number of Axes"),
            arg_int_req("Moving poll rate"),
            arg_int_req("Idle poll rate"),
        ],
        "MM4000AsynConfig(card, portName, asynAddress, numAxes, movingPollPeriod, \
         idlePollPeriod)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            let port_name = req_string(args, 1, "asyn port name")?;
            let num_axes = c_int(args, 3);
            let moving_poll_ms = c_int(args, 4).max(0) as u64;
            let idle_poll_ms = c_int(args, 5).max(0) as u64;

            let num_controllers = *NUM_MM4000_ASYN.lock().unwrap_or_else(|e| e.into_inner());
            if num_controllers < 1 {
                println!("MM4000Config: no MM4000 controllers allocated, call MM4000Setup first");
                return Ok(CommandOutcome::Continue);
            }
            if !(0..num_controllers).contains(&card) {
                println!(
                    "MM4000Config: card must in range 0 to {}",
                    num_controllers - 1
                );
                return Ok(CommandOutcome::Continue);
            }
            if !(1..=MM4000_MAX_AXES).contains(&num_axes) {
                println!("MM4000Config: numAxes must in range 1 to {MM4000_MAX_AXES}");
                return Ok(CommandOutcome::Continue);
            }
            let Ok(handle) = connect_serial(&port_name, MM4000_TIMEOUT) else {
                println!("MM4000AsynConfig: cannot connect to asyn port {port_name}");
                return Ok(CommandOutcome::Continue);
            };
            let cache_ttl = Duration::from_millis(moving_poll_ms / 2);
            let controller = match Mm4000Controller::new(
                handle,
                &port_name,
                &port_name,
                num_axes as usize,
                cache_ttl,
            ) {
                Ok(controller) => controller,
                Err(Mm4000Refusal::NoAnswer) => return Ok(CommandOutcome::Continue),
                Err(Mm4000Refusal::InvalidModel(firmware)) => {
                    println!("MM4000AsynConfig: invalid model = {firmware}");
                    return Ok(CommandOutcome::Continue);
                }
                Err(Mm4000Refusal::TooFewAxes { total_axes }) => {
                    println!(
                        "MM4000AsynConfig: actual number of axes={total_axes} < numAxes={num_axes}"
                    );
                    return Ok(CommandOutcome::Continue);
                }
            };
            let controller = Arc::new(Mutex::new(controller));
            let axes = (1..=num_axes as usize)
                .map(|axis| Model2Axis {
                    motor: Arc::new(Mutex::new(Mm4000Axis::new(
                        controller.clone(),
                        axis,
                        mm4000::MODEL2_POLL_FAILURE,
                    ))),
                    moving_poll_ms,
                    idle_poll_ms,
                })
                .collect();
            MOTOR_MM4000.set_card(card, axes);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPM500,"PM500")` (devNewport.dbd).
static DEV_PM500: DevSup = model1_device("devPM500", "PM500");
/// `PM500_NUM_CARDS` (drvPM500.cc:80).
static PM500: Model1 = Model1::new(&DEV_PM500, 4, SCAN_RATE);

/// `PM500Setup(num_cards, scan_rate)`.
pub fn pm500_setup_command() -> CommandDef {
    PM500.setup_command("PM500Setup")
}

/// `PM500Config(card, name, addr)`: one signal per channel the `STAT?`
/// scan finds. C `motor_init` drops a controller it cannot connect to or
/// that leaves `SVN?` unanswered without a word (drvPM500.cc:577, :671-672).
pub fn pm500_config_command() -> CommandDef {
    model1_config_command("PM500Config", &PM500, |ctx, card, port| {
        let Ok(handle) = connect_serial(port, PM500_TIMEOUT) else {
            return Ok(());
        };
        let Some(controller) = Pm500Controller::new(handle) else {
            return Ok(());
        };
        let num_axes = controller.num_axes();
        let controller = Arc::new(Mutex::new(controller));
        for index in 0..num_axes {
            let ax = Pm500Axis::new(controller.clone(), index);
            PM500.install(ctx, card, index as i32, Arc::new(Mutex::new(ax)))?;
        }
        Ok(())
    })
}

/// `device(motor,VME_IO,devESP300,"ESP300")` (devNewport.dbd).
static DEV_ESP300: DevSup = model1_device("devESP300", "ESP300");
/// `ESP300_NUM_CARDS` (drvESP300.cc:71).
static ESP300: Model1 = Model1::new(&DEV_ESP300, 4, SCAN_RATE);

/// `ESP300Setup(num_cards, scan_rate)`.
pub fn esp300_setup_command() -> CommandDef {
    ESP300.setup_command("ESP300Setup")
}

/// `ESP300Config(card, name, addr)`: one signal per axis C `motor_init`
/// finds (stop each axis until "axis number out of range").
pub fn esp300_config_command() -> CommandDef {
    model1_config_command("ESP300Config", &ESP300, |ctx, card, port| {
        const ERRMSG: &str = "drvESP300.c:motor_init: ASYN";
        let handle = match connect_with_eos(
            port,
            ESP300_TIMEOUT,
            esp300::INPUT_EOS,
            esp300::OUTPUT_EOS,
            EosRefusal::InputFatal,
        ) {
            Ok(handle) => handle,
            Err(EosConnectError::InputEos(_, e)) => {
                errlog_vprintf(format_args!(
                    "{ERRMSG} setInputEos error = {}\n",
                    e.status() as i32
                ));
                return Ok(());
            }
            Err(_) => {
                errlog_vprintf(format_args!(
                    "{ERRMSG} connect error = {}\n",
                    AsynStatus::Error as i32
                ));
                return Ok(());
            }
        };
        let Some(controller) = Esp300Controller::new(handle) else {
            return Ok(());
        };
        let num_axes = controller.num_axes();
        let controller = Arc::new(Mutex::new(controller));
        for axis in 1..=num_axes {
            let ax = Esp300Axis::new(controller.clone(), axis);
            ESP300.install(ctx, card, (axis - 1) as i32, Arc::new(Mutex::new(ax)))?;
        }
        Ok(())
    })
}

/// `device(motor,VME_IO,devPMNC87xx,"PMNC87xx")` (motorNewFocus
/// devNewFocus.dbd).
static DEV_PMNC87XX: DevSup = model1_device("devPMNC87xx", "PMNC87xx");
/// `PMNC87xx_NUM_CARDS` (drvPMNCCom.h:87).
static PMNC87XX: Model1 = Model1::new(&DEV_PMNC87XX, 10, SCAN_RATE);

/// `PMNC87xxSetup(num_cards, num_drivers, scan_rate)`. The driver modules
/// are discovered, so `num_drivers` has no use.
pub fn pmnc87xx_setup_command() -> CommandDef {
    PMNC87XX.register_row();
    CommandDef::new(
        "PMNC87xxSetup",
        vec![
            arg_int_req("Max. controller count"),
            arg_int_req("Max. drivers per controller count"),
            arg_int_req("Polling rate"),
        ],
        "PMNC87xxSetup(num_cards, num_drivers, scan_rate)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            PMNC87XX.setup(c_int(args, 0), c_int(args, 2));
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `PMNC87xxConfig(card, name, addr)`: one signal per channel of the driver
/// modules found (`MPV` for the 8750, `DRT` for the 8752). The controller
/// sets the port EOS itself, as C `motor_init` does.
pub fn pmnc87xx_config_command() -> CommandDef {
    model1_config_command("PMNC87xxConfig", &PMNC87XX, |ctx, card, port| {
        let asyn_port = get_port(port).ok_or_else(|| {
            format!("IP port '{port}' not found (call drvAsynIPPortConfigure first)")
        })?;
        let controller = PmncController::new(asyn_port.handle.clone(), PMNC_TIMEOUT)
            .map_err(|e| format!("PMNC87xxConfig: {e}"))?;
        let num_axes = controller.num_axes();
        let controller = Arc::new(Mutex::new(controller));
        for index in 0..num_axes {
            let ax = PmncAxis::new(controller.clone(), index)
                .map_err(|e| format!("PMNC87xxConfig: axis {index}: {e}"))?;
            PMNC87XX.install(ctx, card, index as i32, Arc::new(Mutex::new(ax)))?;
        }
        Ok(())
    })
}

/// Parse the optional `moveMode` arg for the PVT commands: `absolute`/`abs`/`0`
/// (default) or `relative`/`rel`/`1`.
fn parse_move_mode(arg: Option<String>) -> Result<MoveMode, String> {
    match arg.as_deref().map(str::to_ascii_lowercase).as_deref() {
        None | Some("") | Some("absolute") | Some("abs") | Some("0") => Ok(MoveMode::Absolute),
        Some("relative") | Some("rel") | Some("1") => Ok(MoveMode::Relative),
        Some(other) => Err(format!(
            "moveMode must be absolute or relative, got '{other}'"
        )),
    }
}

/// C `asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n", driverName,
/// functionName, message)` for a failed XPS profile build, run or readback.
fn xps_profile_failed(motor_port: &str, function: &str, message: &str) {
    MotorTrace::new(motor_port, 0).print_self(
        TraceMask::ERROR,
        format_args!("XPSController:{function}: {message}\n"),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xps::fake::FakeLine;

    fn plan() -> ExecutionPlan {
        ExecutionPlan {
            file_name: "traj.trj".into(),
            group: "G1".into(),
            start_moves: vec![("G1.P1".into(), -0.25)],
            end_moves: vec![("G1.P1".into(), 3.0)],
            move_mode: MoveMode::Absolute,
            times: vec![0.5, 0.5, 1.0],
            gathering_positioners: vec!["G1.P1".into()],
        }
    }

    fn run(line: &FakeLine, fail: &'static str) -> (Option<String>, Vec<String>) {
        *line.fail.lock().unwrap() = fail;
        let message = xps_run_profile(&line.socket(SocketMode::Query), &plan(), 1, (1, 2, 2));
        (message, line.take_log())
    }

    /// C `runProfile` (XPSController.cpp:905-1135): the start move, the pulse
    /// output and the flyback are unchecked, a failed gathering step skips
    /// to `done:` and the flyback, and the last failed step's message wins.
    #[test]
    fn run_profile_flies_back_and_keeps_the_last_message() {
        let line = FakeLine::new("xps_run_profile", "");
        let (message, log) = run(&line, "");
        assert_eq!(message, None);
        assert_eq!(log.first().unwrap(), "GroupMoveAbsolute (G1.P1,-0.25)");
        assert_eq!(log.last().unwrap(), "GroupMoveAbsolute (G1.P1,3)");

        for ignored in ["GroupMoveAbsolute", "MultipleAxesPVTPulseOutputSet"] {
            let (message, log) = run(&line, ignored);
            assert_eq!(message, None, "{ignored}");
            assert!(
                log.iter()
                    .any(|c| c.starts_with("MultipleAxesPVTExecution "))
            );
        }

        let (message, log) = run(&line, "GatheringReset");
        assert_eq!(
            message.as_deref(),
            Some("Error performing GatheringReset, status=-3")
        );
        assert!(
            !log.iter()
                .any(|c| c.starts_with("MultipleAxesPVTExecution "))
        );
        assert_eq!(log.last().unwrap(), "GroupMoveAbsolute (G1.P1,3)");

        let (message, _) = run(&line, "EventExtendedRemove");
        assert_eq!(
            message.as_deref(),
            Some("Error performing ExtendedEventRemove, status=-3")
        );
    }

    /// C `tclcall`: the controller's code is the status, and a missing
    /// socket is SendAndReceive's -22.
    #[test]
    fn tclcall_returns_the_script_status() {
        let line = FakeLine::new("xps_tclcall", "");
        let socket = line.socket(SocketMode::Query);
        assert_eq!(tclcall(Some(&socket), "a.tcl", "t", "1"), 0);
        assert_eq!(line.take_log(), ["TCLScriptExecute (a.tcl,t,1)"]);
        *line.fail.lock().unwrap() = "TCLScriptExecute";
        assert_eq!(tclcall(Some(&socket), "a.tcl", "t", "1"), -3);
        assert_eq!(tclcall(None, "a.tcl", "t", "1"), -22);
    }
}
