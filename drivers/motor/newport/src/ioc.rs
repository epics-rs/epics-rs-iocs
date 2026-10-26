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
//! The XPS profile, position-compare and TCL commands are driver-private: C
//! drives them from controller-level records this port does not serve.

use std::fmt::Write;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::asyn_record::{claim_port, get_port};
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::stdlib::strtod;
use epics_rs::base::runtime::task::{StackSizeClass, ThreadPriority, spawn_dedicated_thread};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;
use epics_rs::motor::controller_port::{ControllerCtx, ControllerPort};
use epics_rs::motor::trace::MotorTrace;
use motor_common::connect::{
    EosConnectError, EosRefusal, connect_controller, connect_serial, connect_with_eos,
};
use motor_common::iocsh::{
    arg_double_opt, arg_int_opt, arg_int_req, arg_str_opt, arg_str_req, opt_double, opt_int,
    opt_string, poll_intervals, req_string,
};
use motor_common::model1::{CardInit, Model1, SCAN_RATE, c_int};
use motor_common::model1_device;
use motor_common::model2::{Model2, Model2Axis};
use motor_common::{
    AxisReport, Controller, ControllerParams, ControllerReport, DEFAULT_CONTROLLER_TIMEOUT,
    asyn_user, install_with_report, register_controller_port, register_port_with,
};

use crate::agap::{self, AgapAxis, AgapController};
use crate::agilis::{AgUcAxis, AgUcController};
use crate::conex::ConexAxis;
use crate::esp300::{self, Esp300Axis, Esp300Controller};
use crate::hxp::{HxpAxis, HxpController, HxpControllerParams, MoveLine, NUM_HXP_AXES};
use crate::mm3000::{Mm3000Axis, Mm3000Controller};
use crate::mm4000::{self, Mm4000Axis, Mm4000AxisReport, Mm4000Controller, Mm4000Refusal};
use crate::pm500::{Pm500Axis, Pm500Controller};
use crate::pmnc::{PmncAxis, PmncController};
use crate::report::{AxisLines, ControllerHead, period_s};
use crate::smc100::Smc100Axis;
use crate::xps::profile_move::{
    ProfileHost, XpsProfile, run_flow, start_gathering, stop_gathering, strerror, trajectory_upload,
};
use crate::xps::{
    ExecutionPlan, GatheringReadback, MoveMode, PcoParams, Profile, SocketMode, XpsAxis,
    XpsController, XpsError, XpsSocket, ftp, gathering, gpio, pco, sftp,
};

/// `CONEX_TIMEOUT` of AG_CONEX.cpp:30 and AGAP_CONEX.cpp:31.
const CONEX_TIMEOUT: Duration = Duration::from_secs(2);
const AGAP_TIMEOUT: Duration = Duration::from_secs(2);
/// AG_UC.cpp:29 `AGILIS_TIMEOUT`.
const AG_UC_TIMEOUT: Duration = Duration::from_secs(2);

/// XPS poll-socket timeout (C `XPS_POLL_TIMEOUT` = 2 s): waits for the full
/// `,EndOfAPI`-framed reply.
const XPS_POLL_TIMEOUT: Duration = Duration::from_secs(2);
/// XPS move-socket timeout (C sets `-0.1` s): the fire-and-forget write does
/// not wait for the move to finish, so the read times out quickly.
const XPS_MOVE_TIMEOUT: Duration = Duration::from_millis(100);

/// The controller's own `moveSocket_`, on which `MultipleAxesPVTExecution`
/// blocks until the trajectory ends (C `XPS_MOVE_TIMEOUT` 100000 s).
const XPS_PVT_EXEC_TIMEOUT: Duration = Duration::from_secs(100_000);

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

/// Default XPS FTP account: the factory `Administrator` login C's
/// `XPSCreateProfile` is normally given.
const XPS_FTP_USER: &str = "Administrator";
const XPS_FTP_PASSWORD: &str = "Administrator";

/// `MAX_SOCKETS` (asynOctetSocket.cpp:36).
const MAX_SOCKETS: usize = 1000;

/// `nextSocket` (asynOctetSocket.cpp), held for a whole `connect_to_server`.
/// C reads and bumps it unlocked, so two sequencer threads (two xpsSlave
/// instances) could both name port `ip:port:N` and the second fail.
static NEXT_SOCKET: Mutex<usize> = Mutex::new(0);

/// C `ConnectToServer` (asynOctetSocket.cpp:60): asyn IP port
/// `ip:port:N` from `drvAsynIPPortConfigure(name, "ip:port TCP", 0, 0, 1)`
/// in `db`'s IOC. The socket index and its port; `None` is C's -1, printed
/// as C prints it.
pub(crate) fn connect_to_server(
    db: &PvDatabase,
    ip: &str,
    ip_port: i32,
) -> Option<(i32, PortHandle)> {
    let mut next = NEXT_SOCKET.lock().unwrap();
    let socket = *next;
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
            db.scan_control().interrupt_gate(),
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
    *next = socket + 1;
    Some((socket as i32, handle.port_handle().clone()))
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
            let text = opt_string(args, 5)?.unwrap_or_default();
            // C prints this after building the controller, on an errno it never
            // cleared; it is refused here before the build.
            let Some(step_size) = step_size(&text) else {
                println!("SMC100: Error invalid steps per unit={text}");
                return Ok(CommandOutcome::Continue);
            };

            let (moving_s, idle_s) = (period_s(moving_poll_ms), period_s(idle_poll_ms));
            // SMC100Controller::report (SMC100Driver.cpp:103-110).
            let head = ControllerHead(move |ctx: &ControllerReport<'_>, out: &mut dyn Write| {
                let _ = writeln!(
                    out,
                    "SMC100 motor driver {}, numAxes={}, moving poll period={moving_s:.6}, \
                     idle poll period={idle_s:.6}",
                    ctx.port_name(),
                    ctx.num_axes(),
                    moving_s = CFloat(moving_s),
                    idle_s = CFloat(idle_s)
                );
            });
            register_port_with(ctx, &port_name, num_axes, head)?;
            let (handle, _) = connect_controller(
                &serial_port,
                DEFAULT_CONTROLLER_TIMEOUT,
                &port_name,
                format_args!(
                    "SMC100Controller::SMC100Controller: cannot connect to SMC100 controller"
                ),
            );
            // C builds every axis on pasynUserController_, connected or not.
            let poller = Controller::new();
            let mut controller_user = Some(handle);
            for axis in 0..num_axes {
                let handle = controller_user.take().unwrap_or_else(|| {
                    SyncIOHandle::connect_port(
                        &serial_port,
                        0,
                        DEFAULT_CONTROLLER_TIMEOUT,
                        InterfaceType::Octet,
                    )
                    .0
                });
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(Smc100Axis::new(
                    handle,
                    (axis + 1) as u8,
                    step_size,
                    MotorTrace::new(&port_name, axis),
                )));
                // SMC100Axis::report (SMC100Driver.cpp:150-159).
                let report = Arc::new(AxisLines(move |out: &mut dyn Write, level| {
                    if level > 0 {
                        let _ = writeln!(out, "  axis {}", axis + 1);
                    }
                }));
                poller.install_with_report(asyn_user(&port_name, axis), motor, report)?;
            }
            poller.start(&port_name, moving_poll_ms, idle_poll_ms, 2); // SMC100Driver.cpp:67
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

/// `AG_CONEXController::report` / `AGAP_CONEXController::report`
/// (AG_CONEX.cpp:138-146, AGAP_CONEX.cpp:137-145), its quote unclosed as
/// C's, and the cell for C's `controllerVersion_`, unset until VE answers
/// (empty here).
fn conex_head(
    controller_id: i32,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
) -> (Arc<Mutex<String>>, impl ControllerParams) {
    let version = Arc::new(Mutex::new(String::new()));
    let (moving_s, idle_s) = (period_s(moving_poll_ms), period_s(idle_poll_ms));
    let shown = version.clone();
    let head = ControllerHead(move |ctx: &ControllerReport<'_>, out: &mut dyn Write| {
        let version = shown.lock().unwrap_or_else(|e| e.into_inner());
        let _ = write!(
            out,
            "CONEX motor driver {}, controllerID={controller_id}, version=\"{version}\n  \
             moving poll period={moving_s:.6}, idle poll period={idle_s:.6}\n",
            ctx.port_name(),
            moving_s = CFloat(moving_s),
            idle_s = CFloat(idle_s)
        );
    });
    (version, head)
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

            let (version, head) = conex_head(controller_id, moving_poll_ms, idle_poll_ms);
            register_port_with(ctx, &port_name, 1, head)?;
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
            axis.version()
                .clone_into(&mut version.lock().unwrap_or_else(|e| e.into_inner()));
            let report = Arc::new(axis.report());
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            install_with_report(
                asyn_user(&port_name, 0),
                motor,
                report,
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

            let (version, head) = conex_head(controller_id, moving_poll_ms, idle_poll_ms);
            register_port_with(ctx, &port_name, agap::NUM_AXES as i32, head)?;
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
            controller
                .version()
                .clone_into(&mut version.lock().unwrap_or_else(|e| e.into_inner()));
            let controller = Arc::new(Mutex::new(controller));
            let poller = Controller::new();
            for axis in 0..agap::NUM_AXES {
                let ax = AgapAxis::new(controller.clone(), axis);
                let report = Arc::new(ax.report());
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
                poller.install_with_report(asyn_user(&port_name, axis as i32), motor, report)?;
            }
            poller.start(&port_name, moving_poll_ms, idle_poll_ms, 2); // AGAP_CONEX.cpp:79
            Ok(CommandOutcome::Continue)
        },
    )
}

/// A controller `AG_UCCreateController` made, for `AG_UCCreateAxis` to find
/// by port name.
struct AgUcRegistration {
    port_name: String,
    controller: Arc<Mutex<AgUcController>>,
    poller: Arc<Controller>,
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

            // Filled once the constructor has read the model and version.
            let model_version = Arc::new(Mutex::new((0, String::new())));
            let (moving_s, idle_s) = (period_s(moving_poll_ms), period_s(idle_poll_ms));
            let shown = model_version.clone();
            // AG_UCController::report (AG_UC.cpp:192-201).
            let head = ControllerHead(move |ctx: &ControllerReport<'_>, out: &mut dyn Write| {
                let (model, version) = &*shown.lock().unwrap_or_else(|e| e.into_inner());
                let _ = write!(
                    out,
                    "Agilis UC motor driver {}, model={model}, numAxes={}, moving poll \
                     period={moving_s:.6}, idle poll period={idle_s:.6}\ncontroller version {version}\n",
                    ctx.port_name(),
                    ctx.num_axes(),
                    moving_s = CFloat(moving_s),
                    idle_s = CFloat(idle_s)
                );
            });
            register_port_with(ctx, &port_name, c_int(args, 2), head)?;
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
            let controller = AgUcController::new(handle, &MotorTrace::new(&port_name, 0));
            // C prints an unset `AG_UCModel_` when VE failed or named no
            // model; 0 stands for it.
            *model_version.lock().unwrap_or_else(|e| e.into_inner()) = (
                controller.model().map_or(0, |model| model as i32),
                controller.version().to_string(),
            );
            // C's constructor returns before startPoller without a model
            // (AG_UC.cpp:79, :96, :99); the axes created later stay unpolled.
            let poller = Arc::new(Controller::new());
            if controller.model().is_some() {
                poller.start(&port_name, moving_poll_ms, idle_poll_ms, 2);
            }
            AG_UC_CONTROLLERS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .push(AgUcRegistration {
                    port_name,
                    controller: Arc::new(Mutex::new(controller)),
                    poller,
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
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Controller port name")?;
            let axis = c_int(args, 1);
            let (controller, poller) = {
                let controllers = AG_UC_CONTROLLERS.lock().unwrap_or_else(|e| e.into_inner());
                let Some(reg) = controllers.iter().find(|r| r.port_name == port_name) else {
                    println!("AG_UCCreateAxis: Error port {port_name} not found");
                    return Ok(CommandOutcome::Continue);
                };
                (reg.controller.clone(), reg.poller.clone())
            };
            // C constructs the axis under pC->lock() (AG_UC.cpp:135-137).
            poller.install_built(asyn_user(&port_name, axis), || {
                let ax = AgUcAxis::new(
                    controller,
                    axis,
                    c_int(args, 2) != 0,
                    c_int(args, 3),
                    c_int(args, 4),
                );
                let report: Arc<dyn AxisReport> = Arc::new(ax.report());
                Ok((Arc::new(Mutex::new(ax)), Some(report)))
            })?;
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
    /// The controller's asyn port.
    handle: PortHandle,
    controller: Arc<Mutex<XpsController>>,
    /// The controller's `moveSocket_`, which runs profiles.
    profile_sock: Arc<XpsSocket>,
    /// The poller `XPSCreateController` starts; `XPSCreateAxis` joins it.
    poller: Arc<Controller>,
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

/// The controller and IP address `XPSCreateController` registered as a port.
struct XpsLink {
    controller: Arc<Mutex<XpsController>>,
    ip: String,
    ip_port: i32,
}

fn xps_link(port_name: &str) -> Option<XpsLink> {
    XPS_CONTROLLERS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|r| r.port_name == port_name)
        .map(|r| XpsLink {
            controller: r.controller.clone(),
            ip: r.ip.clone(),
            ip_port: r.ip_port,
        })
}

/// The hooks' controller, which `XPSCreateController` registers right
/// after the port.
fn xps_hook_link(ctx: &ControllerCtx<'_>) -> AsynResult<(Arc<Mutex<XpsController>>, String)> {
    xps_link(&ctx.base.port_name)
        .map(|link| (link.controller, link.ip))
        .ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("XPS port {} has no controller", ctx.base.port_name),
        })
}

/// The parameters and overrides of C `XPSController` (XPSController.cpp);
/// its sockets and axes come from the registration `XPSCreateController`
/// pushes after the port.
struct XpsHead {
    moving_s: f64,
    idle_s: f64,
    enable_set_position: bool,
    settling_s: f64,
    profile: XpsProfile,
}

impl ProfileHost for XpsHead {
    fn xps_profile(&mut self) -> &mut XpsProfile {
        &mut self.profile
    }
}

impl ControllerParams for XpsHead {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        self.profile.create_params(base)
    }

    fn build_profile(&mut self, ctx: &mut ControllerCtx<'_>) -> AsynResult<()> {
        let (controller, ip) = xps_hook_link(ctx)?;
        self.profile.build_profile(ctx, &controller, &ip)
    }

    fn execute_profile(&mut self, _ctx: &mut ControllerCtx<'_>) -> AsynResult<()> {
        self.profile.execute_profile()
    }

    fn abort_profile(&mut self, ctx: &mut ControllerCtx<'_>) -> AsynResult<()> {
        let (controller, _) = xps_hook_link(ctx)?;
        self.profile.abort_profile(ctx, &controller)
    }

    fn readback_profile(&mut self, ctx: &mut ControllerCtx<'_>) -> AsynResult<()> {
        let (controller, _) = xps_hook_link(ctx)?;
        self.profile.readback_profile(ctx, &controller)
    }

    fn define_profile(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        addr: i32,
        positions: &[f64],
    ) -> AsynResult<()> {
        let (controller, _) = xps_hook_link(ctx)?;
        self.profile
            .define_profile(ctx, &controller, addr, positions)
    }

    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn Write, level: i32) {
        if let Some(XpsLink {
            controller,
            ip,
            ip_port,
        }) = xps_link(ctx.port_name())
        {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let _ = write!(
                out,
                "XPS motor driver: {}\n                 numAxes: {}\n        firmware version: \
                 {}\n      moving poll period: {:.6}\n        idle poll period: {:.6}\n",
                ctx.port_name(),
                ctx.num_axes(),
                ctrl.firmware(),
                CFloat(self.moving_s),
                CFloat(self.idle_s)
            );
            if level > 0 {
                // glibc prints a NULL %s, C's ftpUsername_ before
                // XPSCreateProfile, as "(null)".
                let (user, password) = self
                    .profile
                    .ftp_login()
                    .cloned()
                    .unwrap_or_else(|| ("(null)".into(), "(null)".into()));
                let _ = write!(
                    out,
                    "       enableSetPosition: {}\n setPositionSettlingTime: {:.6}\n\
                     \x20              IPAddress: {ip}\n                  IPPort: {ip_port}\n\
                     \x20            ftpUserName: {user}\n         ftpUserPassword: {password}\n\
                     \x20                useSFTP: {}\n           movesDeferred: {}\n\
                     \x20             autoEnable: {}\n          noDisableError: {}\n",
                    i32::from(self.enable_set_position),
                    CFloat(self.settling_s),
                    ctrl.use_sftp(),
                    i32::from(ctrl.moves_deferred()),
                    i32::from(ctrl.auto_enable()),
                    i32::from(ctrl.no_disable_error())
                );
            }
        }
        ctx.report(out, level);
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

            let (profile, profile_thread) = XpsProfile::new(&port_name);
            let head = XpsHead {
                moving_s: period_s(moving_poll_ms),
                idle_s: period_s(idle_poll_ms),
                enable_set_position,
                settling_s: f64::from(c_int(args, 7)) / 1000.0,
                profile,
            };
            let owner = ctx.db().scan_control().interrupt_gate();
            let handle = register_controller_port(owner, &port_name, c_int(args, 3), head)?;
            // C goes on with socket -1 for either socket
            // (XPSController.cpp:179-193).
            let poll = match connect_to_server(ctx.db(), &ip, ip_port) {
                Some((index, handle)) => XpsSocket::new(
                    handle,
                    index,
                    &port_name,
                    SocketMode::Query,
                    XPS_POLL_TIMEOUT,
                ),
                None => {
                    println!(
                        "XPSController:XPSController: error calling TCP_ConnectToServer for pollSocket"
                    );
                    XpsSocket::invalid(&port_name, SocketMode::Query)
                }
            };
            let profile_sock = Arc::new(match connect_to_server(ctx.db(), &ip, ip_port) {
                Some((index, handle)) => XpsSocket::new(
                    handle,
                    index,
                    &port_name,
                    SocketMode::Query,
                    XPS_PVT_EXEC_TIMEOUT,
                ),
                None => {
                    println!(
                        "XPSController:XPSController: error calling TCP_ConnectToServer for moveSocket"
                    );
                    XpsSocket::invalid(&port_name, SocketMode::Query)
                }
            });
            let controller = Arc::new(Mutex::new(
                XpsController::new(poll, enable_set_position, settling)
                    .map_err(|e| format!("XPSCreateController: {e}"))?,
            ));
            // C starts the poller before any axis exists (XPSController.cpp:197).
            let poller = Arc::new(
                Controller::new().controller_poll(profile_thread.poll(controller.clone())),
            );
            poller.start(&port_name, moving_poll_ms, idle_poll_ms, 10);
            // XPSController.cpp:200, the only C driver that starts it.
            poller
                .start_move_to_home_thread()
                .map_err(|e| format!("XPSCreateController: {e}"))?;
            // XPSController.cpp:206; C ignores epicsThreadCreate's answer.
            let _ = profile_thread.spawn::<XpsHead>(
                handle.clone(),
                controller.clone(),
                profile_sock.clone(),
            );
            XPS_CONTROLLERS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .push(XpsRegistration {
                    port_name,
                    ip,
                    ip_port,
                    handle,
                    controller,
                    profile_sock,
                    poller,
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
            let Some((controller, ip, ip_port, poller)) =
                with_xps(&port_name, "XPSCreateAxis", |reg| {
                    (
                        reg.controller.clone(),
                        reg.ip.clone(),
                        reg.ip_port,
                        reg.poller.clone(),
                    )
                })
            else {
                return Ok(CommandOutcome::Continue);
            };
            let text = opt_string(args, 3)?.unwrap_or_default();
            let Some(steps_per_unit) = step_size(&text) else {
                println!("XPSController:XPSCreateAxis: Error invalid steps per unit={text}");
                return Ok(CommandOutcome::Continue);
            };
            // C constructs the axis, its move socket included, under
            // pC->lock() (XPSController.cpp:1379-1381).
            poller.install_built(asyn_user(&port_name, axis), || {
                // C goes on with socket -1, whose TCP_SetTimeout prints
                // (XPSAxis.cpp:146-152).
                let move_sock = match connect_to_server(ctx.db(), &ip, ip_port) {
                    Some((index, handle)) => XpsSocket::new(
                        handle,
                        index,
                        &port_name,
                        SocketMode::Fire,
                        XPS_MOVE_TIMEOUT,
                    ),
                    None => {
                        println!(
                            "XPSAxis:XPSAxis: error calling TCP_ConnectToServer for move socket"
                        );
                        println!(
                            "SetTCPTimeout, SocketIndex=-1, must be >=0 and < {}",
                            *NEXT_SOCKET.lock().unwrap()
                        );
                        XpsSocket::invalid(&port_name, SocketMode::Fire)
                    }
                };
                let ax = XpsAxis::new(
                    controller,
                    move_sock,
                    &port_name,
                    axis,
                    &positioner_name,
                    1.0 / steps_per_unit,
                );
                let report: Arc<dyn AxisReport> = Arc::new(ax.report());
                Ok((Arc::new(Mutex::new(ax)), Some(report)))
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `XPSCreateProfile(portName, maxPoints, ftpUsername, ftpPassword)`: C
/// `XPSController::initializeProfile`, with the FTP account `buildProfile`
/// uploads with.
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
            let max_points = c_int(args, 1).max(0) as usize;
            let (user, password) = (
                opt_string(args, 2)?.unwrap_or_default(),
                opt_string(args, 3)?.unwrap_or_default(),
            );
            let login = (user.clone(), password.clone());
            // The port thread looks the registration up, so the call runs
            // after with_xps lets it go.
            let Some(handle) = with_xps(&port_name, "XPSCreateProfile", |reg| {
                reg.ftp_login = Some(login);
                reg.handle.clone()
            }) else {
                return Ok(CommandOutcome::Continue);
            };
            let _ = handle.with_driver_blocking::<ControllerPort<XpsHead>, _>(move |d| {
                d.with_params(|head, ctx| {
                    head.profile
                        .initialize_profile(ctx, max_points, &user, &password)
                })
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
                set(&mut reg.controller.lock().unwrap_or_else(|e| e.into_inner()));
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
        .map(|r| r.controller.clone())
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
            let ran = xps_tcl_script_execute(
                &motor_port,
                controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .poll_socket(),
                &tcl_file,
                &task_name,
                &parameters,
            );
            if !ran {
                return Ok(CommandOutcome::Continue);
            }
            println!(
                "XPSTclScriptExecute: motorPort={motor_port} tclFile={tcl_file} task={task_name} params={parameters}"
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// XPSController.cpp:267-277: `TCLScriptExecute` between C's FLOW and
/// ERROR lines on the controller's own user.
fn xps_tcl_script_execute(
    motor_port: &str,
    sock: &XpsSocket,
    tcl_file: &str,
    task_name: &str,
    parameters: &str,
) -> bool {
    let trace = MotorTrace::new(motor_port, 0);
    trace.print_self(
        TraceMask::FLOW,
        format_args!("Executing TCL script {tcl_file} on XPS: {motor_port}\n"),
    );
    match sock.tcl_script_execute(tcl_file, task_name, parameters) {
        Ok(()) => true,
        Err(e) => {
            trace.print_self(
                TraceMask::ERROR,
                format_args!(
                    "TCLScriptExecute returned error {}, on XPS: {motor_port}\n",
                    e.code()
                ),
            );
            false
        }
    }
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
            let socket =
                connect_to_server(ctx.db(), TCLCALL_IP, TCLCALL_PORT).map(|(index, handle)| {
                    let port = handle.port_name().to_owned();
                    XpsSocket::new(handle, index, &port, SocketMode::Query, TCLCALL_TIMEOUT)
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
            let Some((index, handle)) = connect_to_server(ctx.db(), &ip, c_int(args, 2)) else {
                println!("drvXPSAsynAuxConfig: error calling TCP_ConnectToServer");
                return Ok(CommandOutcome::Continue);
            };
            let period = Duration::from_millis(c_int(args, 3).max(0) as u64);
            let socket = XpsSocket::new(
                handle.clone(),
                index,
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
                epics_rs::base::stderr_println!("XPSAuxConfig ERROR: Can't register port");
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
                epics_rs::base::stderr_println!("XPSAuxConfig, epicsThreadCreate failed");
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
                CFloat(params.min_position).g(),
                CFloat(params.max_position).g(),
                CFloat(params.position_step).g(),
                CFloat(params.pulse_width_us).g(),
                CFloat(params.settling_time_us).g(),
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
/// (no port). Credentials default to the XPS factory `Administrator`
/// account and the directory to the firmware's trajectory folder.
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
            let ftp_dir = opt_string(args, 5)?;

            let controller = xps_controller(&motor_port, "XPSBuildProfile")?;

            let upload = TrajectoryUpload {
                motor_port: &motor_port,
                host: &host,
                file_name: &file_name,
                user: &ftp_user,
                password: &ftp_pass,
            };
            let build =
                || xps_build_profile(&controller, &upload, ftp_dir.as_deref(), upload_trajectory);
            let directory = match build() {
                Ok(directory) => directory,
                Err(message) => {
                    xps_profile_failed(&motor_port, "buildProfile", &message);
                    return Ok(CommandOutcome::Continue);
                }
            };
            println!(
                "XPSBuildProfile: motorPort={motor_port} file={file_name} uploaded to {host}:{directory} and verified"
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
            }) else {
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
                        &MotorTrace::new(&motor_port, 0),
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
    trace: &MotorTrace,
    sock: &XpsSocket,
    plan: &ExecutionPlan,
    executions: i32,
    pulses: (i32, i32, i32),
) -> Option<String> {
    // C ignores both moves' status.
    xps_profile_moves(sock, plan.move_mode, &plan.start_moves);
    let message = xps_execute_gathered(trace, sock, plan, executions, pulses);
    xps_profile_moves(sock, plan.move_mode, &plan.end_moves);
    message
}

/// Move each axis in turn; a move on the blocking socket returns when the
/// motion is done, as C's `waitMotors` waits after it.
fn xps_profile_moves(sock: &XpsSocket, mode: MoveMode, moves: &[(String, f64)]) {
    for (positioner, target) in moves {
        let _ = match mode {
            MoveMode::Absolute => sock.group_move_absolute(positioner, &[*target]),
            MoveMode::Relative => sock.group_move_relative(positioner, *target),
        };
    }
}

/// The part of C `runProfile` between the start move and `done:`.
fn xps_execute_gathered(
    trace: &MotorTrace,
    sock: &XpsSocket,
    plan: &ExecutionPlan,
    executions: i32,
    pulses: (i32, i32, i32),
) -> Option<String> {
    let event_id = match start_gathering(
        trace,
        sock,
        &plan.group,
        &plan.gathering_positioners,
        &plan.times,
        pulses,
    ) {
        Ok(id) => id,
        Err(message) => return Some(message),
    };

    run_flow(
        trace,
        format_args!(
            "calling MultipleAxesPVTExecution({}, {}, {}, {executions})\n",
            sock.index(),
            plan.group,
            plan.file_name
        ),
    );
    let message = match sock.multiple_axes_pvt_execution(&plan.group, &plan.file_name, executions) {
        Ok(()) => None,
        // -27: the trajectory was aborted.
        Err(XpsError::Api(-27)) => Some("MultipleAxesPVTExecution aborted".to_string()),
        Err(e) => Some(format!(
            "Error performing MultipleAxesPVTExecution, status={}",
            e.code()
        )),
    };
    // C removes the event and stops gathering whatever the run returned.
    stop_gathering(trace, sock, event_id).or(message)
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
            let trace = MotorTrace::new(&motor_port, 0);
            let (readback, num_read) = match xps_readback(&trace, ctrl.poll_socket(), &positioners)
            {
                Ok(read) => read,
                Err(message) => {
                    xps_profile_failed(&motor_port, "readbackProfile", &message);
                    return Ok(CommandOutcome::Continue);
                }
            };

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

/// C `readbackProfile` up to its parse: the samples and how many were read.
fn xps_readback(
    trace: &MotorTrace,
    sock: &XpsSocket,
    positioners: &[String],
) -> Result<(GatheringReadback, i32), String> {
    let flow = |args: std::fmt::Arguments<'_>| {
        trace.print_self(
            TraceMask::FLOW,
            format_args!("XPSController:readbackProfile: {args}"),
        )
    };
    flow(format_args!("entry\n"));
    let read = sock.gathering_current_number_get();
    let (status, (current_samples, max_samples)) = match &read {
        Ok(n) => (0, *n),
        Err(e) => (e.code(), (0, 0)),
    };
    flow(format_args!(
        "GatheringCurrentNumberGet, status={status}, currentSamples={current_samples}, maxSamples={max_samples}\n"
    ));
    if status != 0 {
        return Err(format!(
            "Error calling GatherCurrentNumberGet, status={status}"
        ));
    }

    let mut readback = GatheringReadback::new(positioners.len());
    let mut num_read = 0i32;
    while num_read < current_samples {
        // Ask for everything left; on a controller error halve the request
        // until it fits (C readbackProfile).
        let mut lines = current_samples - num_read;
        let buffer = loop {
            let got = sock.gathering_data_multiple_lines_get(num_read, lines);
            let status = got.as_ref().map_or_else(|e| e.code(), |_| 0);
            flow(format_args!(
                "GatheringDataMultipleLinesGet, status={status}, numInBuffer={lines}\n"
            ));
            match got {
                Ok(buf) => break buf,
                Err(_) if lines > 1 => lines /= 2,
                Err(_) => return Err("Error reading gathering data, numInBuffer = 0".into()),
            }
        };
        let parsed = gathering::parse_gathering_buffer(&buffer, positioners.len(), &mut readback)?;
        if parsed == 0 {
            return Err("gathering returned no lines".into());
        }
        num_read += parsed as i32;
    }
    Ok((readback, num_read))
}

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

            // C goes on with socket -1 for either socket (HXPDriver.cpp:106-116,
            // 559-562).
            let poll_sock = match connect_to_server(ctx.db(), &ip, ip_port) {
                Some((index, handle)) => XpsSocket::new(
                    handle,
                    index,
                    &port_name,
                    SocketMode::Query,
                    HXP_POLL_TIMEOUT,
                ),
                None => {
                    println!(
                        "HXPDriver:HXPController::HXPController: error calling TCP_ConnectToServer for pollSocket"
                    );
                    XpsSocket::invalid(&port_name, SocketMode::Query)
                }
            };
            let controller = Arc::new(Mutex::new(HxpController::new(poll_sock)));
            let owner = ctx.db().scan_control().interrupt_gate();
            register_controller_port(
                owner,
                &port_name,
                NUM_HXP_AXES as i32,
                HxpControllerParams::new(controller.clone(), moving_poll_ms, idle_poll_ms),
            )?;
            // C startPoller, HXPDriver.cpp:116.
            let polled = controller.clone();
            let poller = Controller::new()
                .controller_poll(move || polled.lock().unwrap_or_else(|e| e.into_inner()).poll());

            for axis_no in 0..NUM_HXP_AXES {
                let move_sock = match connect_to_server(ctx.db(), &ip, ip_port) {
                    Some((index, handle)) => XpsSocket::new(
                        handle,
                        index,
                        &port_name,
                        SocketMode::Fire,
                        HXP_MOVE_TIMEOUT,
                    ),
                    None => {
                        println!(
                            "SetTCPTimeout, SocketIndex=-1, must be >=0 and < {}",
                            *NEXT_SOCKET.lock().unwrap()
                        );
                        XpsSocket::invalid(&port_name, SocketMode::Fire)
                    }
                };
                let line = Arc::new(MoveLine::new(move_sock));
                controller
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .add_line(axis_no, line.clone());
                let motor: Arc<Mutex<dyn AsynMotor>> =
                    Arc::new(Mutex::new(HxpAxis::new(controller.clone(), line, axis_no)));
                // HXPAxis::report (HXPDriver.cpp:578-587).
                let name = char::from(b"XYZUVW"[axis_no]);
                let report = Arc::new(AxisLines(move |out: &mut dyn Write, level| {
                    if level > 0 {
                        let _ = write!(out, "  axis {axis_no}\n    axisName {name}\n");
                    }
                }));
                poller.install_with_report(asyn_user(&port_name, axis_no as i32), motor, report)?;
            }
            poller.start(&port_name, moving_poll_ms, idle_poll_ms, 2);
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

/// A model-1 `<name>(card, port, addr)` whose `motor_init` pass `build` is,
/// if the driver's `Setup` sized a table holding `card`. `addr` is the asyn
/// address C connects at.
fn model1_config_command(
    name: &'static str,
    driver: &'static Model1,
    build: fn(&mut CardInit, &str, i32),
) -> CommandDef {
    CommandDef::new(
        name,
        model1_config_args(),
        format!("{name}(card, name, addr)"),
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !driver.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let port = req_string(args, 1, "asyn port name")?;
            let addr = c_int(args, 2);
            driver.config(card, move |init| build(init, &port, addr))?;
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
    MM3000.register_report("drvMM3000", |cards| {
        cards.report("MM3000", |card, line| {
            format!("    MM3000 controller {card} asyn port= {line} \n")
        })
    });
    MM3000.setup_command("MM300Setup")
}

/// `MM3000Config(card, name, addr)`: one signal per axis slot the
/// controller's `RC` reports, including the trailing `unused` slot C counts.
pub fn mm3000_config_command() -> CommandDef {
    model1_config_command("MM3000Config", &MM3000, |init, port, addr| {
        // C drops a card it cannot connect to without a word.
        let Ok(handle) = connect_serial(port, addr, MM3000_TIMEOUT) else {
            return;
        };
        let Some(controller) = Mm3000Controller::new(handle) else {
            return;
        };
        let num_axes = controller.axis_types().len();
        let line = format!("{port}, address={addr}, id: {}", controller.ident());
        let controller = Arc::new(Mutex::new(controller));
        for axis in 1..=num_axes {
            let ax = Mm3000Axis::new(controller.clone(), axis);
            init.axis((axis - 1) as i32, Arc::new(Mutex::new(ax)));
        }
        init.describe(line);
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
    MM4000.register_report("drvMM4000", |cards| {
        cards.report("MM4000", |card, line| {
            format!("    MM4000 controller {card}, {line} \n")
        })
    });
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
    model1_config_command("MM4000Config", &MM4000, |init, port, addr| {
        // C drops a card it cannot connect to without a word.
        let Ok(handle) = connect_serial(port, addr, MM4000_TIMEOUT) else {
            return;
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
                return;
            }
            Err(_) => return,
        };
        let num_axes = controller.num_axes();
        let line = format!("port={port}, address={addr}, id: {}", controller.firmware());
        let controller = Arc::new(Mutex::new(controller));
        for axis in 1..=num_axes {
            let ax = Mm4000Axis::new(
                controller.clone(),
                init.card(),
                axis,
                mm4000::MODEL1_POLL_FAILURE,
            );
            init.axis((axis - 1) as i32, Arc::new(Mutex::new(ax)));
        }
        init.describe(line);
    })
}

/// drvMM4000Asyn.c's `motorAxisDrvSET_t`.
/// Its poller forces 10 fast polls (drvMM4000Asyn.c:630).
pub static MOTOR_MM4000: Model2 = Model2::new("motorMM4000", 10);

/// C's `MM4000Controller`, as `motorAxisReport` prints it: zeroed by
/// `MM4000AsynSetup`'s calloc, filled by `MM4000AsynConfig` up to where it
/// stops.
#[derive(Default)]
struct Mm4000AsynCard {
    firmware: String,
    /// C's `MM_model`: MM4000 0, MM4005 1.
    model: i32,
    moving_s: f64,
    idle_s: f64,
    axes: Vec<Mm4000AxisReport>,
}

/// `pMM4000Controller`, `numMM4000Controllers` long.
static MM4000_ASYN: Mutex<Vec<Mm4000AsynCard>> = Mutex::new(Vec::new());

/// drvMM4000Asyn.c `motorAxisReport` (:208-224).
fn mm4000_asyn_report(level: i32) -> String {
    let mut out = String::new();
    for (i, card) in MM4000_ASYN
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .enumerate()
    {
        let firmware = format!("Controller {i} firmware version: {}\n", card.firmware);
        out.push_str(&firmware);
        if level != 0 {
            out.push_str(&format!(
                "    model: {}\n    moving poll period: {:.6}\n    idle poll period: {:.6}\n",
                card.model,
                CFloat(card.moving_s),
                CFloat(card.idle_s)
            ));
            out.push_str(&firmware);
        }
        for axis in &card.axes {
            axis.report(&mut out, level);
        }
    }
    out
}

/// `MM4000AsynSetup(num_controllers)`, printing what C's does.
pub fn mm4000_asyn_setup_command() -> CommandDef {
    MOTOR_MM4000.register_report(mm4000_asyn_report);
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
            *MM4000_ASYN.lock().unwrap_or_else(|e| e.into_inner()) = (0..num_controllers)
                .map(|_| Mm4000AsynCard::default())
                .collect();
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

            let num_controllers =
                MM4000_ASYN.lock().unwrap_or_else(|e| e.into_inner()).len() as i32;
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
            // C fills its controller as it goes, so a Config stopping below
            // leaves what the report then prints; a re-Config keeps the
            // firmware and model until VE; answers (drvMM4000Asyn.c:804-808).
            let shown = |f: &mut dyn FnMut(&mut Mm4000AsynCard)| {
                f(&mut MM4000_ASYN.lock().unwrap_or_else(|e| e.into_inner())[card as usize]);
            };
            shown(&mut |c| {
                c.axes = (0..num_axes).map(|_| Mm4000AxisReport::default()).collect();
                c.moving_s = f64::from(c_int(args, 4)) / 1000.0;
                c.idle_s = f64::from(c_int(args, 5)) / 1000.0;
            });
            let Ok(handle) = connect_serial(&port_name, c_int(args, 2), MM4000_TIMEOUT) else {
                println!("MM4000AsynConfig: cannot connect to asyn port {port_name}");
                return Ok(CommandOutcome::Continue);
            };
            let cache_ttl = Duration::from_millis(moving_poll_ms / 2);
            let controller = match Mm4000Controller::new(
                handle,
                &port_name,
                num_axes as usize,
                cache_ttl,
            ) {
                Ok(controller) => controller,
                Err(Mm4000Refusal::NoAnswer) => return Ok(CommandOutcome::Continue),
                Err(Mm4000Refusal::InvalidModel(firmware)) => {
                    println!("MM4000AsynConfig: invalid model = {firmware}");
                    shown(&mut |c| c.firmware = firmware.clone());
                    return Ok(CommandOutcome::Continue);
                }
                Err(Mm4000Refusal::TooFewAxes {
                    total_axes,
                    firmware,
                    model,
                }) => {
                    shown(&mut |c| {
                        c.firmware = firmware.clone();
                        c.model = model as i32;
                    });
                    println!(
                        "MM4000AsynConfig: actual number of axes={total_axes} < numAxes={num_axes}"
                    );
                    return Ok(CommandOutcome::Continue);
                }
            };
            let (firmware, model) = (controller.firmware().to_owned(), controller.model());
            let controller = Arc::new(Mutex::new(controller));
            let (axes, reports): (Vec<_>, Vec<_>) = (1..=num_axes as usize)
                .map(|axis| {
                    let ax = Mm4000Axis::new(
                        controller.clone(),
                        card,
                        axis,
                        mm4000::MODEL2_POLL_FAILURE,
                    );
                    let report = ax.report();
                    let (getter, log) = (ax.params(), ax.log());
                    (
                        Model2Axis::new(Arc::new(Mutex::new(ax)), getter, log),
                        report,
                    )
                })
                .unzip();
            let mut reports = Some(reports);
            shown(&mut |c| {
                c.firmware = firmware.clone();
                c.model = model as i32;
                c.axes = reports.take().unwrap_or_default();
            });
            MOTOR_MM4000.start_card(card, axes, moving_poll_ms, idle_poll_ms)?;
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
    PM500.register_report("drvPM500", |cards| {
        if cards.num_cards() <= 0 {
            return "    No PM500 controllers configured.\n".into();
        }
        (0..cards.num_cards())
            .map(|card| match cards.card(card) {
                Some(line) => format!("    PM500 controller {card} {line} \n"),
                None => format!("    PM500 controller #{card} connection failed.\n"),
            })
            .collect()
    });
    PM500.setup_command("PM500Setup")
}

/// `PM500Config(card, name, addr)`: one signal per channel the `STAT?`
/// scan finds. C `motor_init` drops a controller it cannot connect to or
/// that leaves `SVN?` unanswered without a word (drvPM500.cc:577, :671-672).
pub fn pm500_config_command() -> CommandDef {
    model1_config_command("PM500Config", &PM500, |init, port, addr| {
        let Ok(handle) = connect_serial(port, addr, PM500_TIMEOUT) else {
            return;
        };
        let Some(controller) = Pm500Controller::new(handle) else {
            return;
        };
        let num_axes = controller.num_axes();
        let line = format!("port={port}, address={addr}, id: {}", controller.ident());
        let controller = Arc::new(Mutex::new(controller));
        for index in 0..num_axes {
            let ax = Pm500Axis::new(controller.clone(), index);
            init.axis(index as i32, Arc::new(Mutex::new(ax)));
        }
        init.describe(line);
    })
}

/// `device(motor,VME_IO,devESP300,"ESP300")` (devNewport.dbd).
static DEV_ESP300: DevSup = model1_device("devESP300", "ESP300");
/// `ESP300_NUM_CARDS` (drvESP300.cc:71).
static ESP300: Model1 = Model1::new(&DEV_ESP300, 4, SCAN_RATE);

/// `ESP300Setup(num_cards, scan_rate)`.
pub fn esp300_setup_command() -> CommandDef {
    ESP300.register_report("drvESP300", |cards| {
        cards.report("ESP300", |card, line| {
            format!("    ESP300 controller {card} {line} \n")
        })
    });
    ESP300.setup_command("ESP300Setup")
}

/// `ESP300Config(card, name, addr)`: one signal per axis C `motor_init`
/// finds (stop each axis until "axis number out of range").
pub fn esp300_config_command() -> CommandDef {
    model1_config_command("ESP300Config", &ESP300, |init, port, addr| {
        const ERRMSG: &str = "drvESP300.c:motor_init: ASYN";
        let handle = match connect_with_eos(
            port,
            addr,
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
                return;
            }
            Err(_) => {
                errlog_vprintf(format_args!(
                    "{ERRMSG} connect error = {}\n",
                    AsynStatus::Error as i32
                ));
                return;
            }
        };
        let Some(controller) = Esp300Controller::new(handle) else {
            return;
        };
        let num_axes = controller.num_axes();
        let line = format!("port={port}, address={addr}, id: {}", controller.ident());
        let controller = Arc::new(Mutex::new(controller));
        for axis in 1..=num_axes {
            let ax = Esp300Axis::new(controller.clone(), axis);
            init.axis((axis - 1) as i32, Arc::new(Mutex::new(ax)));
        }
        init.describe(line);
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
    // The `break` at drvPMNC87xx.cc:233 ends C's card loop after the first
    // configured card, leaving the rest unreported; a C defect not copied.
    PMNC87XX.register_report("drvPMNC87xx", |cards| {
        cards.report("PMNC87xx", |card, line| {
            format!("    PMNC87xx controller {card} asyn port= {line} \n")
        })
    });
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
    model1_config_command("PMNC87xxConfig", &PMNC87XX, |init, port, addr| {
        let Some(asyn_port) = get_port(port) else {
            // A missing port is the one connect refusal; connectDevice
            // returns asynError for it.
            println!(
                "drvPMNC:motor_init(), error calling pasynOctetSyncIO->connect port={port} \
                 error={}",
                AsynStatus::Error as i32
            );
            return;
        };
        let Ok(controller) = PmncController::new(asyn_port.handle.clone(), addr, PMNC_TIMEOUT)
        else {
            return;
        };
        let line = format!("{port}, address={addr}, id: {}", controller.ident());
        for (index, ax) in PmncAxis::all(Arc::new(Mutex::new(controller)))
            .into_iter()
            .enumerate()
        {
            init.axis(index as i32, Arc::new(Mutex::new(ax)));
        }
        init.describe(line);
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
/// What one `XPSBuildProfile` uploads, besides its folder and contents.
struct TrajectoryUpload<'a> {
    motor_port: &'a str,
    host: &'a str,
    file_name: &'a str,
    user: &'a str,
    password: &'a str,
}

/// C `buildProfile` from the trajectory text to the verify: the text goes
/// to `file_name` in the working directory, the firmware picks the folder
/// (unless `ftp_dir` names one) and `useSFTP_` the transport, and `upload`
/// sends it with the controller unlocked. Answers the folder.
fn xps_build_profile(
    controller: &Mutex<XpsController>,
    target: &TrajectoryUpload<'_>,
    ftp_dir: Option<&str>,
    upload: impl Fn(&TrajectoryUpload<'_>, &str, &str, bool) -> Result<(), String>,
) -> Result<String, String> {
    let trace = MotorTrace::new(target.motor_port, 0);
    trace.print_self(
        TraceMask::FLOW,
        format_args!("XPSController:buildProfile: entry\n"),
    );
    let (built, directory, use_sftp) = {
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let built = ctrl.build_profile_text(&trace)?;
        std::fs::write(target.file_name, &built.text).map_err(|e| {
            format!(
                "Error creating trajectory file {}, error={}\n",
                target.file_name,
                strerror(&e)
            )
        })?;
        let (directory, sftp) = trajectory_upload(ctrl.firmware())?;
        if sftp {
            ctrl.set_use_sftp();
        }
        (built, directory, ctrl.use_sftp())
    };
    let directory = ftp_dir.unwrap_or(directory).to_string();
    upload(target, &directory, &built.text, use_sftp)?;
    controller
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .verify_profile(&trace, target.file_name, &built)?;
    Ok(directory)
}

/// C `strtod(text, NULL)` for a step size, `None` where C's `errno` refuses
/// it (XPSController.cpp:1371-1376, SMC100Driver.cpp:84-89). C builds a
/// zero, infinite or NaN step and divides by it on every move or readback;
/// those are refused too.
fn step_size(text: &str) -> Option<f64> {
    let read = strtod(text.as_bytes());
    (!read.erange && read.value.is_finite() && read.value != 0.0).then_some(read.value)
}

/// XPSController.cpp:709-747: SFTP when `useSFTP_`, else FTP.
fn upload_trajectory(
    target: &TrajectoryUpload<'_>,
    directory: &str,
    contents: &str,
    use_sftp: bool,
) -> Result<(), String> {
    if use_sftp {
        xps_sftp_upload_traced(target, directory)
    } else {
        ftp::upload_trajectory(
            target.host,
            directory,
            target.file_name,
            contents,
            target.user,
            target.password,
        )
    }
}

/// XPSController.cpp:710-720: `xpsSFTPUpload` between C's FLOW lines,
/// verbose when the controller's own user traces IO_DRIVER.
fn xps_sftp_upload_traced(target: &TrajectoryUpload<'_>, directory: &str) -> Result<(), String> {
    let trace = MotorTrace::new(target.motor_port, 0);
    let verbose = PortManager::global().trace_manager().is_enabled_device(
        target.motor_port,
        0,
        TraceMask::IO_DRIVER,
    );
    trace.print_self(
        TraceMask::FLOW,
        format_args!("XPSController::buildProfile calling xpsSFTPUpload\n"),
    );
    let status = sftp::xps_sftp_upload(
        target.host,
        directory,
        target.file_name,
        target.user,
        target.password,
        verbose,
    );
    trace.print_self(
        TraceMask::FLOW,
        format_args!("XPSController::buildProfile xpsSFTPUpload returned {status}\n"),
    );
    if status != 0 {
        return Err(format!("Error calling xpsSFTPUpload, status={status}\n"));
    }
    Ok(())
}

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

    /// XPSController.cpp:1371-1376: C's strtod, refused on its ERANGE, a
    /// subnormal included.
    #[test]
    fn a_step_size_is_cs_strtod() {
        assert_eq!(step_size("0x10"), Some(16.0));
        assert_eq!(step_size(" 2.5e3mm"), Some(2500.0));
        assert_eq!(step_size("1e-310"), None);
        assert_eq!(step_size("1e-400"), None);
        assert_eq!(step_size("1e400"), None);
        assert_eq!(step_size("inf"), None);
        assert_eq!(step_size("nan"), None);
        assert_eq!(step_size("abc"), None);
    }

    /// XPSController.cpp:695-747: XPSBuildProfile uploads to the firmware's
    /// folder, over SFTP once an XPS-D build sets `useSFTP_` and over FTP
    /// otherwise; an ftpDir names the folder instead.
    /// XPSController.cpp:642-648: the build writes the trajectory file under
    /// its name in the working directory before the upload, and fails when
    /// it cannot.
    #[test]
    fn the_build_writes_the_trajectory_file() {
        let dir = std::env::temp_dir().join(format!("xps-traj-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("traj.trj");
        let file = file.to_str().unwrap();
        let (sent, _) = build_with("xpsBuildFile", "XPS-C8 V2", None, file);
        assert_eq!(sent.len(), 1);
        let text = std::fs::read_to_string(file).unwrap();
        assert!(text.starts_with("0.") && text.ends_with('\n'), "{text}");
        let _ = std::fs::remove_dir_all(&dir);

        let missing = "/nonexistent-xps-dir/traj.trj";
        assert_eq!(
            build_result("xpsBuildNoFile", "XPS-C8 V2", missing),
            Err(format!(
                "Error creating trajectory file {missing}, error=No such file or directory\n"
            ))
        );
    }

    /// A controller over a fake XPS with one G1 axis and a three-point
    /// profile.
    fn profiled(port: &str, firmware: &'static str) -> Mutex<XpsController> {
        profiled_at(port, firmware, vec![0.0, 1.0, 2.0])
    }

    /// [`profiled`] with the profile's `positions`.
    fn profiled_at(
        port: &str,
        firmware: &'static str,
        positions: Vec<f64>,
    ) -> Mutex<XpsController> {
        // The IP port's own FLOW lines stay off the motor port's trace.
        let line = FakeLine::new(&format!("{port}_ip"), "");
        *line.firmware.lock().unwrap() = firmware;
        let mut ctrl =
            XpsController::new(line.socket(SocketMode::Query), false, Duration::ZERO).unwrap();
        ctrl.register_axis(
            0,
            "G1.P1",
            "G1",
            Arc::new(line.socket(SocketMode::Fire)),
            Default::default(),
            1.0,
        );
        ctrl.define_profile(Profile {
            group: "G1".into(),
            move_mode: MoveMode::Absolute,
            times: vec![1.0, 1.0, 1.0],
            axes: vec![crate::xps::ProfileAxis {
                positioner: "G1.P1".into(),
                positions,
            }],
        })
        .unwrap();
        Mutex::new(ctrl)
    }

    /// Run the build to `file_name` with a recording uploader; answer the
    /// uploads and the controller's `useSFTP_`.
    fn build_with(
        port: &str,
        firmware: &'static str,
        ftp_dir: Option<&str>,
        file_name: &str,
    ) -> (Vec<(String, String, bool)>, bool) {
        let controller = profiled(port, firmware);
        let target = TrajectoryUpload {
            motor_port: port,
            host: "10.0.0.5",
            file_name,
            user: "u",
            password: "p",
        };
        let sent = Mutex::new(Vec::new());
        let _ = xps_build_profile(&controller, &target, ftp_dir, |t, dir, _text, sftp| {
            sent.lock()
                .unwrap()
                .push((t.file_name.to_string(), dir.to_string(), sftp));
            Ok(())
        });
        let use_sftp = controller.lock().unwrap().use_sftp();
        (sent.into_inner().unwrap(), use_sftp)
    }

    /// The build's verdict, the upload always succeeding.
    fn build_result(port: &str, firmware: &'static str, file_name: &str) -> Result<(), String> {
        build_result_of(profiled(port, firmware), port, file_name)
    }

    fn build_result_of(
        controller: Mutex<XpsController>,
        port: &str,
        file_name: &str,
    ) -> Result<(), String> {
        let target = TrajectoryUpload {
            motor_port: port,
            host: "10.0.0.5",
            file_name,
            user: "u",
            password: "p",
        };
        xps_build_profile(&controller, &target, None, |_, _, _, _| Ok(())).map(|_| ())
    }

    #[test]
    fn the_build_uploads_by_the_firmwares_transport() {
        let dir = std::env::temp_dir().join(format!("xps-transport-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("traj.trj");
        let file = file.to_str().unwrap().to_string();
        let build = |port: &str, firmware: &'static str, ftp_dir: Option<&str>| {
            build_with(port, firmware, ftp_dir, &file)
        };
        let upload = |d: &str, sftp| vec![(file.clone(), d.to_string(), sftp)];
        assert_eq!(
            build("xpsBuildC", "XPS-C8 V2", None),
            (upload("/Admin/Public/Trajectories", false), false)
        );
        assert_eq!(
            build("xpsBuildQ", "XPS-Q8 V1", None),
            (upload("/Public/Trajectories", false), false)
        );
        assert_eq!(
            build("xpsBuildD", "XPS-D8 V1", None),
            (upload("/Admin/Public/Trajectories", true), true)
        );
        assert_eq!(
            build("xpsBuildDir", "XPS-D8 V1", Some("/Mine")),
            (upload("/Mine", true), true)
        );
        assert_eq!(build("xpsBuildNone", "0", None), (vec![], false));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// XPSController.cpp:801-817 checks the start plus the verified travel
    /// (the fake XPS's -0.5..2.5) against the fake's -100..100 soft limits.
    #[test]
    fn the_build_checks_the_verified_travel_against_the_soft_limits() {
        let dir = std::env::temp_dir().join(format!("xps-travel-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("traj.trj");
        let file = file.to_str().unwrap();
        let build = |port: &str, start: f64| {
            let controller = profiled_at(port, "XPS-C8 V2", vec![start, start + 0.5, start + 1.0]);
            build_result_of(controller, port, file)
        };
        assert_eq!(build("xpsTravelIn", 0.0), Ok(()));
        assert_eq!(
            build("xpsTravelHigh", 98.0),
            Err(
                "High soft limit violation for axis G1.P1, position=100.500000, limit=100.000000\n"
                    .into()
            )
        );
        assert_eq!(
            build("xpsTravelLow", -99.6),
            Err("Low soft limit violation for axis G1.P1, position=-100.100000, limit=-100.000000\n".into())
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// XPSController.cpp:695-706: the trajectory folder and transport follow
    /// the firmware string, and any other firmware fails the build.
    #[test]
    fn the_firmware_picks_the_trajectory_folder() {
        assert_eq!(
            trajectory_upload("XPS-C8 Firmware V2.6.x"),
            Ok(("/Admin/Public/Trajectories", false))
        );
        assert_eq!(
            trajectory_upload("XPS-Q8 Firmware V1.4.x"),
            Ok(("/Public/Trajectories", false))
        );
        assert_eq!(
            trajectory_upload("XPS-D8 Firmware V1.0.x"),
            Ok(("/Admin/Public/Trajectories", true))
        );
        assert_eq!(
            trajectory_upload("0"),
            Err("Firmware version does not contain XPS-C, XPS-Q, or XPS-D=0\n".to_string())
        );
    }

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
        let trace = MotorTrace::new("xps_run_profile", 0);
        let message = xps_run_profile(
            &trace,
            &line.socket(SocketMode::Query),
            &plan(),
            1,
            (1, 2, 2),
        );
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

    /// FLOW on `port` addr 0 into a fresh file; returns its path.
    fn trace_flow(port: &str) -> std::path::PathBuf {
        use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};

        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_device_trace_mask(port, 0, TraceMask::FLOW);
        trace.set_device_trace_info_mask(port, 0, TraceInfoMask::empty());
        trace.set_device_trace_file(port, 0, TraceFile::File(file));
        path
    }

    fn read_trace(path: &std::path::Path) -> String {
        let text = std::fs::read_to_string(path).unwrap();
        let _ = std::fs::remove_file(path);
        text
    }

    /// XPSController.cpp:565, 624, 750: buildProfile's FLOW lines.
    #[test]
    fn the_build_traces_as_c() {
        let port = "xpsBuildTrace";
        let path = trace_flow(port);
        let file = std::env::temp_dir().join(format!("xps-build-trace-{}", std::process::id()));
        let file = file.to_str().unwrap().to_string();
        assert_eq!(build_result(port, "XPS-C8 V2", &file), Ok(()));
        let _ = std::fs::remove_file(&file);
        assert_eq!(
            read_trace(&path),
            format!(
                "XPSController:buildProfile: entry\n\
                 XPSController:buildProfile: axis 0 profilePositions[0]=0.000000, profilePositions[2]=2.000000, maxAcceleration=36.000000, preTimeMax=0.027778, postTimeMax=0.027778\n\
                 XPSController:buildProfile: calling MultipleAxesPVTVerification(0, G1, {file})\n"
            )
        );
    }

    /// XPSController.cpp:938-1093: runProfile's FLOW lines.
    #[test]
    fn the_run_traces_as_c() {
        let port = "xps_run_trace";
        let line = FakeLine::new(&format!("{port}_ip"), "");
        let path = trace_flow(port);
        let trace = MotorTrace::new(port, 0);
        let message = xps_run_profile(
            &trace,
            &line.socket(SocketMode::Query),
            &plan(),
            1,
            (1, 2, 2),
        );
        assert_eq!(message, None);
        let text = read_trace(&path);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 9, "{text}");
        assert_eq!(
            lines[..5],
            [
                "XPSController:runProfile: calling GatheringReset(0)",
                "XPSController:runProfile: calling GatheringConfigurationSet(0, 2, G1.P1.SetpointPosition;G1.P1.CurrentPosition;)",
                "XPSController:runProfile: calling MultipleAxesPVTPulseOutputSet(0, G1, 2, 3, 0.500000)",
                "XPSController:runProfile: calling EventExtendedConfigurationTriggerSet(0, 2, Always;G1.PVT.TrajectoryPulse, , , . )",
                "XPSController:runProfile: calling EventExtendedConfigurationActionSet(0, 1, GatheringOneData, , , , )",
            ]
        );
        // C prints &eventId with %p.
        assert!(
            lines[5].starts_with("XPSController:runProfile: calling EventExtendedStart(0, 0x"),
            "{text}"
        );
        assert_eq!(
            lines[6..],
            [
                "XPSController:runProfile: calling MultipleAxesPVTExecution(0, G1, traj.trj, 1)",
                "XPSController:runProfile: calling EventExtendedRemove(0, 0)",
                "XPSController:runProfile: calling GatheringStop(0)",
            ]
        );
    }

    /// XPSController.cpp:1210, 1229, 1254: readbackProfile's FLOW lines,
    /// the halving retry included.
    #[test]
    fn the_readback_traces_as_c() {
        let port = "xps_readback_trace";
        let line = FakeLine::new(&format!("{port}_ip"), "");
        let path = trace_flow(port);
        let trace = MotorTrace::new(port, 0);
        let sock = line.socket(SocketMode::Query);
        let positioners = ["G1.P1".to_string()];
        let (_, num_read) = xps_readback(&trace, &sock, &positioners).unwrap();
        assert_eq!(num_read, 2);
        *line.fail.lock().unwrap() = "GatheringDataMultipleLinesGet";
        assert_eq!(
            xps_readback(&trace, &sock, &positioners).map(|(_, n)| n),
            Err("Error reading gathering data, numInBuffer = 0".to_string())
        );
        assert_eq!(
            read_trace(&path),
            "XPSController:readbackProfile: entry\n\
             XPSController:readbackProfile: GatheringCurrentNumberGet, status=0, currentSamples=2, maxSamples=100\n\
             XPSController:readbackProfile: GatheringDataMultipleLinesGet, status=0, numInBuffer=2\n\
             XPSController:readbackProfile: entry\n\
             XPSController:readbackProfile: GatheringCurrentNumberGet, status=0, currentSamples=2, maxSamples=100\n\
             XPSController:readbackProfile: GatheringDataMultipleLinesGet, status=-3, numInBuffer=2\n\
             XPSController:readbackProfile: GatheringDataMultipleLinesGet, status=-3, numInBuffer=1\n"
        );
    }

    /// XPSController.cpp:267: the TCL script's FLOW line, before the call.
    #[test]
    fn the_tcl_script_traces_as_c() {
        let port = "xps_tcl_trace";
        let line = FakeLine::new(&format!("{port}_ip"), "");
        let path = trace_flow(port);
        let sock = line.socket(SocketMode::Query);
        assert!(xps_tcl_script_execute(port, &sock, "a.tcl", "0", "0"));
        assert_eq!(
            read_trace(&path),
            "Executing TCL script a.tcl on XPS: xps_tcl_trace\n"
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

    /// Two xpsSlave instances connect from their own sequencer threads:
    /// each gets its own socket, where C's unlocked nextSocket could hand
    /// both one port name.
    #[epics_rs::base::epics_test]
    async fn concurrent_connects_take_distinct_sockets() {
        let db = PvDatabase::new();
        let mut indices: Vec<i32> = std::thread::scope(|s| {
            let joins: Vec<_> = (0..4)
                .map(|_| s.spawn(|| connect_to_server(&db, "127.0.0.1", 1).map(|(i, _)| i)))
                .collect();
            joins
                .into_iter()
                .map(|j| j.join().unwrap().unwrap())
                .collect()
        });
        indices.sort();
        indices.dedup();
        assert_eq!(indices.len(), 4);
    }
}

#[cfg(all(test, target_os = "linux"))]
mod one_poller {
    use std::net::TcpListener;

    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::user::AsynUser;
    use epics_rs::base::server::iocsh::registry::ArgValue;
    use motor_common::testing::pollers_after;

    use super::*;

    /// A serial port under `name` that answers every read with `reply`.
    fn publish_line(name: &str, reply: &'static str) {
        struct Line(PortDriverBase, &'static str);
        impl PortDriver for Line {
            fn base(&self) -> &PortDriverBase {
                &self.0
            }
            fn base_mut(&mut self) -> &mut PortDriverBase {
                &mut self.0
            }
            fn io_write_octet(&mut self, _: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
                Ok(data.len())
            }
            fn io_read_octet(
                &mut self,
                _: &AsynUser,
                buf: &mut [u8],
            ) -> epics_rs::asyn::interpose::OctetReadResult {
                let n = self.1.len().min(buf.len());
                buf[..n].copy_from_slice(&self.1.as_bytes()[..n]);
                epics_rs::asyn::interpose::OctetReadResult::ok(
                    n,
                    epics_rs::asyn::interpose::EomReason::empty(),
                )
            }
        }
        let base = PortDriverBase::new(
            name,
            1,
            PortFlags::default(),
            InterfaceMask::UNMIGRATED_ALL,
            InterfaceMask::UNMIGRATED_INTERRUPTS,
        );
        let claim = claim_port(name).unwrap();
        let (rt, _actor) =
            create_port_runtime(&claim, Line(base, reply), RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        std::mem::forget(rt);
    }

    /// HXPDriver.cpp:116: one startPoller for the six axes.
    #[epics_rs::base::epics_test]
    async fn hxp_create_controller_polls_its_axes_on_one_poller() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        std::thread::spawn(move || {
            let held: Vec<_> = listener.incoming().take(8).collect();
            std::thread::park();
            drop(held);
        });
        let args = vec![
            ArgValue::String("hxpOnePoller".into()),
            ArgValue::String("127.0.0.1".into()),
            ArgValue::Int(port.into()),
            ArgValue::Int(200),
            ArgValue::Int(1000),
        ];
        let cmd = hxp_create_controller_command();
        assert_eq!(pollers_after(cmd, args).await, 1);
    }

    /// HXPDriver.cpp:106-116: a failed poll or move socket still builds the
    /// controller, its axes and its poller.
    #[epics_rs::base::epics_test]
    async fn hxp_create_controller_without_sockets_still_polls_its_axes() {
        // Every ConnectToServer takes socket 0, whose port name is held.
        let _held = epics_rs::asyn::registry::claim_port("hxp.nosocket:1:0").unwrap();
        let args = vec![
            ArgValue::String("hxpNoSocket".into()),
            ArgValue::String("hxp.nosocket".into()),
            ArgValue::Int(1),
            ArgValue::Int(200),
            ArgValue::Int(1000),
        ];
        let cmd = hxp_create_controller_command();
        assert_eq!(pollers_after(cmd, args).await, 1);
    }

    /// XPSController.cpp:179-197, XPSAxis.cpp:146-152: failed poll and
    /// move sockets still build the controller and its axes.
    #[epics_rs::base::epics_test]
    async fn xps_create_axis_without_sockets_still_builds_the_axis() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::motor::controller_port::index;

        // Every ConnectToServer takes socket 0, whose port name is held.
        let _held = epics_rs::asyn::registry::claim_port("xps.nosocket:1:0").unwrap();
        let args = vec![
            ArgValue::String("xpsNoSocket".into()),
            ArgValue::String("xps.nosocket".into()),
            ArgValue::Int(1),
            ArgValue::Int(1),
            ArgValue::Int(200),
            ArgValue::Int(1000),
            ArgValue::Int(0),
            ArgValue::Int(0),
        ];
        assert_eq!(
            pollers_after(xps_create_controller_command(), args).await,
            1
        );
        let args = vec![
            ArgValue::String("xpsNoSocket".into()),
            ArgValue::Int(0),
            ArgValue::String("G1.P1".into()),
            ArgValue::String("1000".into()),
        ];
        pollers_after(xps_create_axis_command(), args).await;
        let port = PortManager::global()
            .find_port_handle("xpsNoSocket")
            .unwrap();
        // A write to an address with no axis is refused.
        port.write_float64(index::VELOCITY, 0, 1.0).await.unwrap();
    }

    /// SMC100Driver.cpp:67: one startPoller after the axes, which C builds
    /// on an unconnected line too.
    #[epics_rs::base::epics_test]
    async fn smc100_create_controller_polls_its_axes_on_one_poller() {
        let args = vec![
            ArgValue::String("smcOnePoller".into()),
            ArgValue::String("smcOnePollerNoLine".into()),
            ArgValue::Int(3),
            ArgValue::Int(200),
            ArgValue::Int(1000),
            ArgValue::String("0.001".into()),
        ];
        let cmd = smc100_create_controller_command();
        assert_eq!(pollers_after(cmd, args).await, 1);
    }

    /// AGAP_CONEX.cpp:79: one startPoller for axes U and V.
    #[epics_rs::base::epics_test]
    async fn agap_conex_create_controller_polls_its_axes_on_one_poller() {
        publish_line("agapOnePollerLine", "1VE CONEX-AGAP");
        let args = vec![
            ArgValue::String("agapOnePoller".into()),
            ArgValue::String("agapOnePollerLine".into()),
            ArgValue::Int(1),
            ArgValue::Int(200),
            ArgValue::Int(1000),
        ];
        let cmd = agap_conex_create_controller_command();
        assert_eq!(pollers_after(cmd, args).await, 1);
    }

    /// Run AG_UCCreateController on a line answering VE with `version`, then
    /// two AG_UCCreateAxis, returning the poller count after each.
    async fn ag_uc_pollers(prefix: &str, version: &'static str) -> Vec<usize> {
        let line = format!("{prefix}Line");
        publish_line(&line, version);
        let args = vec![
            ArgValue::String(prefix.into()),
            ArgValue::String(line.into()),
            ArgValue::Int(2),
            ArgValue::Int(200),
            ArgValue::Int(1000),
        ];
        let mut counts = vec![pollers_after(ag_uc_create_controller_command(), args).await];
        for axis in 0..2 {
            let args = vec![
                ArgValue::String(prefix.into()),
                ArgValue::Int(axis),
                ArgValue::Int(0),
                ArgValue::Int(50),
                ArgValue::Int(-50),
            ];
            counts.push(pollers_after(ag_uc_create_axis_command(), args).await);
        }
        counts
    }

    /// AG_UC.cpp:99: the controller's poller, which every AG_UCCreateAxis
    /// joins.
    #[epics_rs::base::epics_test]
    async fn ag_uc_create_axis_joins_the_controller_poller() {
        assert_eq!(ag_uc_pollers("agUcOne", "AG-UC8 v1").await, [1, 1, 1]);
    }

    /// AG_UC.cpp:96: no model, no poller.
    #[epics_rs::base::epics_test]
    async fn ag_uc_unknown_model_starts_no_poller() {
        assert_eq!(ag_uc_pollers("agUcNone", "XX").await, [0, 0, 0]);
    }

    /// XPSController.cpp:197: the controller's poller, which every
    /// XPSCreateAxis joins.
    #[epics_rs::base::epics_test]
    async fn xps_create_axis_joins_the_controller_poller() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        std::thread::spawn(move || {
            let held: Vec<_> = listener.incoming().take(4).collect();
            std::thread::park();
            drop(held);
        });
        let args = vec![
            ArgValue::String("xpsOnePoller".into()),
            ArgValue::String("127.0.0.1".into()),
            ArgValue::Int(port.into()),
            ArgValue::Int(2),
            ArgValue::Int(200),
            ArgValue::Int(1000),
            ArgValue::Int(0),
            ArgValue::Int(0),
        ];
        assert_eq!(
            pollers_after(xps_create_controller_command(), args).await,
            1
        );
        for (axis, name) in [(0, "G1.P1"), (1, "G1.P2")] {
            let args = vec![
                ArgValue::String("xpsOnePoller".into()),
                ArgValue::Int(axis),
                ArgValue::String(name.into()),
                ArgValue::String("1000".into()),
            ];
            assert_eq!(pollers_after(xps_create_axis_command(), args).await, 1);
        }
    }
}

#[cfg(test)]
mod report {
    use super::*;
    use motor_common::testing::{drvsup_init, port_report, run_command};

    /// AG_UCController::report then AG_UCAxis::report (AG_UC.cpp:192-201,
    /// :255-267); currentPosition is the last poll's.
    #[epics_rs::base::epics_test]
    async fn ag_uc_report_prints_the_controller_then_each_axis_above_level_0() {
        use motor_common::testing::LineRig;
        let (_line, _io) = LineRig::new("agucRepLine", |cmd| {
            match cmd.trim_end() {
                "VE" => "AG-UC8 v2.2.1",
                "1TP" => "1TP42",
                "1TS" => "1TS0",
                "PH" => "PH0",
                _ => "",
            }
            .into()
        });
        let args = vec![
            ArgValue::String("agucRep".into()),
            ArgValue::String("agucRepLine".into()),
            ArgValue::Int(2),
            ArgValue::Int(100),
            ArgValue::Int(100),
        ];
        run_command(ag_uc_create_controller_command(), args).await;
        let int = ArgValue::Int;
        let axis = vec![
            ArgValue::String("agucRep".into()),
            int(0),
            int(1),
            int(30),
            int(-20),
        ];
        run_command(ag_uc_create_axis_command(), axis).await;
        let head = "Agilis UC motor driver agucRep, model=1, numAxes=2, moving poll \
                    period=0.100000, idle poll period=0.100000\ncontroller version AG-UC8 v2.2.1\n";
        let text = port_report("agucRep", 0);
        assert!(text.contains(&format!("{head}Port: agucRep\n")), "{text}");
        let axis = "  axisID=1, channelID=1, hasLimits=1\n  forwardAmplitude=30, \
                    reverseAmplitude=-20\n  currentPosition=42, positionOffset=0\n";
        let want = format!("{head}{axis}Port: agucRep\n");
        let mut text = String::new();
        for _ in 0..300 {
            text = port_report("agucRep", 1);
            if text.contains(&want) {
                break;
            }
            epics_rs::base::runtime::task::sleep(Duration::from_millis(10)).await;
        }
        assert!(text.contains(&want), "{text}");
    }

    /// AG_CONEXController::report then AG_CONEXAxis::report, which reads
    /// the gains (AG_CONEX.cpp:138-146, :283-319).
    #[epics_rs::base::epics_test]
    async fn ag_conex_report_prints_the_controller_then_reads_the_gains_above_level_0() {
        use motor_common::testing::LineRig;
        let (line, _io) = LineRig::new("cxRepLine", |cmd| {
            match cmd.trim_end() {
                "1VE" => "1VE CONEX-CC 2.0.1",
                "1ID?" => "1IDMFA-CC",
                "1SU?" => "1SU0.0001",
                "1SL?" => "1SL-12.5",
                "1SR?" => "1SR12.5",
                "1TP" => "1TP1.5",
                "1KP?" => "1KP12",
                "1KI?" => "1KI3",
                "1KD?" => "1KD0.5",
                _ => "",
            }
            .into()
        });
        let args = vec![
            ArgValue::String("cxRep".into()),
            ArgValue::String("cxRepLine".into()),
            ArgValue::Int(1),
            ArgValue::Int(100),
            ArgValue::Int(100),
        ];
        run_command(ag_conex_create_controller_command(), args).await;
        let head = "CONEX motor driver cxRep, controllerID=1, version=\"CONEX-CC 2.0.1\n  \
                    moving poll period=0.100000, idle poll period=0.100000\n";
        let text = port_report("cxRep", 0);
        assert!(text.contains(&format!("{head}Port: cxRep\n")), "{text}");
        let axis = "  stageID=MFA-CC\n  currentPosition=15000.000000, encoderIncrement=0.000100\n  \
                    interpolationFactor=1.000000, stepSize=0.000100, lowLimit=-12.500000, \
                    highLimit=12.500000\n  KP=12.000000, KI=3.000000, KD/LF=0.500000\n  \
                    fullStepSize=0.000000, microStepsPerFullStep=0\n";
        let want = format!("{head}{axis}Port: cxRep\n");
        let mut text = String::new();
        for _ in 0..300 {
            line.sent();
            text = port_report("cxRep", 1);
            if text.contains(&want) {
                break;
            }
            epics_rs::base::runtime::task::sleep(Duration::from_millis(10)).await;
        }
        assert!(text.contains(&want), "{text}");
        let sent = line.sent();
        for query in ["1KP?", "1KI?", "1KD?"] {
            assert!(sent.iter().any(|s| s == query), "{sent:?}");
        }
    }

    /// AGAP_CONEXController::report then AGAP_CONEXAxis::report, which reads
    /// each axis's gains (AGAP_CONEX.cpp:137-145, :237-261).
    #[epics_rs::base::epics_test]
    async fn agap_conex_report_prints_the_controller_then_reads_each_axis_above_level_0() {
        use motor_common::testing::LineRig;
        let (_line, _io) = LineRig::new("agapRepLine", |cmd| {
            match cmd.trim_end() {
                "1VE" => "1VE CONEX-AGAP 1.0",
                "1ID?" => "1IDAG-M100",
                "1SU?" => "1SU0.00001",
                "1SLU?" | "1SLV?" => "1SLU-0.75",
                "1SRU?" | "1SRV?" => "1SRU0.75",
                "1TPU" => "1TPU0.01",
                "1TPV" => "1TPV-0.02",
                "1KPU?" => "1KPU5",
                "1KIU?" => "1KIU6",
                "1LFU?" => "1LFU7",
                "1KPV?" => "1KPV8",
                "1KIV?" => "1KIV9",
                "1LFV?" => "1LFV10",
                _ => "",
            }
            .into()
        });
        let args = vec![
            ArgValue::String("agapRep".into()),
            ArgValue::String("agapRepLine".into()),
            ArgValue::Int(1),
            ArgValue::Int(100),
            ArgValue::Int(100),
        ];
        run_command(agap_conex_create_controller_command(), args).await;
        let head = "CONEX motor driver agapRep, controllerID=1, version=\"CONEX-AGAP 1.0\n  \
                    moving poll period=0.100000, idle poll period=0.100000\n";
        let text = port_report("agapRep", 0);
        assert!(text.contains(&format!("{head}Port: agapRep\n")), "{text}");
        let axis = |name, position, kp, ki, lf| {
            format!(
                "axisName={name}\n  stageID=AG-M100\n  currentPosition={position}, \
                 encoderIncrement=0.000010\n  interpolationFactor=0.000000, stepSize=0.000100, \
                 lowLimit=-0.750000, highLimit=0.750000\n  KP={kp}, KI={ki}, KD/LF={lf}\n"
            )
        };
        let want = format!(
            "{head}{}{}Port: agapRep\n",
            axis("U", "100.000000", "5.000000", "6.000000", "7.000000"),
            axis("V", "-200.000000", "8.000000", "9.000000", "10.000000")
        );
        let mut text = String::new();
        for _ in 0..300 {
            text = port_report("agapRep", 1);
            if text.contains(&want) {
                break;
            }
            epics_rs::base::runtime::task::sleep(Duration::from_millis(10)).await;
        }
        assert!(text.contains(&want), "{text}");
    }

    /// HXPController::report then HXPAxis::report (HXPDriver.cpp:143-154,
    /// :578-587); coordSys is the address-0 HXP_MOVE_COORD_SYS as written.
    #[epics_rs::base::epics_test]
    async fn hxp_report_prints_the_coord_sys_then_each_axis_above_level_0() {
        let args = vec![
            ArgValue::String("hxpRep".into()),
            ArgValue::String("127.0.0.1".into()),
            ArgValue::Int(1),
            ArgValue::Int(100),
            ArgValue::Int(1000),
        ];
        run_command(hxp_create_controller_command(), args).await;
        let head = |cs| {
            format!(
                "Newport hexapod motor driver hxpRep, numAxes=6, moving poll period=0.100000, \
                 idle poll period=1.000000, coordSys={cs}\n"
            )
        };
        let text = port_report("hxpRep", 0);
        assert!(
            text.contains(&format!("{}Port: hxpRep\n", head(0))),
            "{text}"
        );
        let port = epics_rs::asyn::manager::PortManager::global()
            .find_port_handle("hxpRep")
            .unwrap();
        let io = SyncIOHandle::from_handle(port.clone(), 0, Duration::from_secs(1));
        let reason = io.drv_user_create("HXP_MOVE_COORD_SYS").unwrap();
        io.write_int32(reason, 2).unwrap();
        SyncIOHandle::from_handle(port, 1, Duration::from_secs(1))
            .write_int32(reason, 1)
            .unwrap();
        let axes: String = "XYZUVW"
            .chars()
            .enumerate()
            .map(|(n, name)| format!("  axis {n}\n    axisName {name}\n"))
            .collect();
        let text = port_report("hxpRep", 1);
        assert!(
            text.contains(&format!("{}{axes}Port: hxpRep\n", head(2))),
            "{text}"
        );
    }

    /// XPSController::report then XPSAxis::report (XPSController.cpp:222-246,
    /// XPSAxis.cpp:207-219); the axis prints at every level.
    #[epics_rs::base::epics_test]
    async fn xps_report_prints_the_controller_then_each_axis() {
        let first = *NEXT_SOCKET.lock().unwrap();
        let args = vec![
            ArgValue::String("xpsRep".into()),
            ArgValue::String("127.0.0.1".into()),
            ArgValue::Int(1),
            ArgValue::Int(2),
            ArgValue::Int(100),
            ArgValue::Int(1000),
            ArgValue::Int(1),
            ArgValue::Int(500),
        ];
        run_command(xps_create_controller_command(), args).await;
        let args = vec![
            ArgValue::String("xpsRep".into()),
            ArgValue::Int(0),
            ArgValue::String("G1.P1".into()),
            ArgValue::String("1000".into()),
        ];
        run_command(xps_create_axis_command(), args).await;
        let head = "XPS motor driver: xpsRep\n\
                    \x20                numAxes: 2\n\
                    \x20       firmware version: \n\
                    \x20     moving poll period: 0.100000\n\
                    \x20       idle poll period: 1.000000\n";
        let axis = format!(
            "  axis 0\n    name = G1.P1\n    step size = 0.001\n    \
             poll socket = {first}, moveSocket = {}\n    status = 0\n",
            first + 2
        );
        let text = port_report("xpsRep", 0);
        assert!(
            text.contains(&format!("{head}{axis}Port: xpsRep\n")),
            "{text}"
        );
        let detail = |user: &str, password: &str, no_disable_error: i32| {
            format!(
                "       enableSetPosition: 1\n\
                 \x20setPositionSettlingTime: 0.500000\n\
                 \x20              IPAddress: 127.0.0.1\n\
                 \x20                 IPPort: 1\n\
                 \x20            ftpUserName: {user}\n\
                 \x20        ftpUserPassword: {password}\n\
                 \x20                useSFTP: false\n\
                 \x20          movesDeferred: 0\n\
                 \x20             autoEnable: 1\n\
                 \x20         noDisableError: {no_disable_error}\n"
            )
        };
        let text = port_report("xpsRep", 1);
        assert!(
            text.contains(&format!("{head}{}{axis}", detail("(null)", "(null)", 0))),
            "{text}"
        );
        let args = vec![
            ArgValue::String("xpsRep".into()),
            ArgValue::Int(100),
            ArgValue::String("user".into()),
            ArgValue::String("pass".into()),
        ];
        run_command(xps_create_profile_command(), args).await;
        run_command(
            xps_no_disable_error_command(),
            vec![ArgValue::String("xpsRep".into())],
        )
        .await;
        let text = port_report("xpsRep", 1);
        assert!(
            text.contains(&format!("{head}{}{axis}", detail("user", "pass", 1))),
            "{text}"
        );
        // XPSController::setDeferredMoves sets movesDeferred_.
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::motor::controller_port::index;
        let port = PortManager::global().find_port_handle("xpsRep").unwrap();
        port.write_int32(index::DEFER_MOVES, 0, 1).await.unwrap();
        let deferred = "           movesDeferred: 1\n";
        let mut text = String::new();
        for _ in 0..200 {
            text = port_report("xpsRep", 1);
            if text.contains(deferred) {
                break;
            }
            epics_rs::base::runtime::task::sleep(Duration::from_millis(10)).await;
        }
        assert!(text.contains(deferred), "{text}");
    }

    /// drvESP300.cc's drvet `report` (drvESP300.cc:153-178), which `dbior`
    /// runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_esp300_controller() {
        use motor_common::testing::{LineRig, driver_report};
        let setup = esp300_setup_command();
        let none = "    No ESP300 controllers configured.\n";
        assert_eq!(driver_report("drvESP300", 0), none);
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("esp300RepLine", |last| {
            match last {
                "VE?" => "ESP300 Version 3.08",
                "TB?" => "9, 0, AXIS NUMBER OUT OF RANGE",
                _ => "",
            }
            .to_string()
                + "\r"
        });
        let config = vec![int(1), ArgValue::String("esp300RepLine".into()), int(3)];
        run_command(esp300_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvESP300", 0),
            "    ESP300 controller 0 connection failed.\n    ESP300 controller 1 \
             port=esp300RepLine, address=3, id: ESP300 Version 3.08 \n"
        );
    }

    /// drvMM3000.cc's drvet `report` (drvMM3000.cc:173-199).
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_mm3000_controller() {
        use motor_common::testing::{LineRig, driver_report};
        let setup = mm3000_setup_command();
        let none = "    No MM3000 controllers configured.\n";
        assert_eq!(driver_report("drvMM3000", 0), none);
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("mm3000RepLine", |last| match last {
            "VE" => "MM3000 Version 2.4".into(),
            "RC" => "1=dc".into(),
            _ => String::new(),
        });
        let config = vec![int(0), ArgValue::String("mm3000RepLine".into()), int(4)];
        run_command(mm3000_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvMM3000", 0),
            "    MM3000 controller 0 asyn port= mm3000RepLine, address=4, id: MM3000 Version \
             2.4 \n    MM3000 controller 1 connection failed.\n"
        );
    }

    /// drvMM4000.cc's drvet `report` (drvMM4000.cc:198-224); `id` is the
    /// `VE;` reply past "VE".
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_mm4000_controller() {
        use motor_common::testing::{LineRig, driver_report};
        let setup = mm4000_setup_command();
        let none = "    No MM4000 controllers configured.\n";
        assert_eq!(driver_report("drvMM4000", 0), none);
        let int = ArgValue::Int;
        run_command(setup, vec![int(1), int(10)]).await;
        let (_line, _io) = LineRig::new("mm4000RepLine", |last| match last {
            "TH;" => "1TH0".into(),
            "VE;" => "VEMM4005 Version 2.0".into(),
            _ => String::new(),
        });
        let config = vec![int(0), ArgValue::String("mm4000RepLine".into()), int(0)];
        run_command(mm4000_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvMM4000", 0),
            "    MM4000 controller 0, port=mm4000RepLine, address=0, id: MM4005 Version 2.0 \n"
        );
    }

    /// drvPM500.cc's drvet `report` (drvPM500.cc:163-189), which `dbior`
    /// runs at every level; `strncpy` keeps 50 bytes of the ident (:584).
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pm500_controller() {
        use motor_common::testing::{LineRig, driver_report, drvsup_init};
        let setup = pm500_setup_command();
        let none = "    No PM500 controllers configured.\n";
        assert_eq!(driver_report("drvPM500", 0), none);
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let ident = format!("PM500 {}", "v".repeat(50));
        let svn = format!("XD{ident}");
        let (_line, _io) = LineRig::new("pm500RepLine", move |last| match last.trim_end() {
            "SVN?" => svn.clone(),
            "XSTAT?" => "XB".into(),
            _ => "YE".into(),
        });
        let config = vec![int(1), ArgValue::String("pm500RepLine".into()), int(3)];
        run_command(pm500_config_command(), config).await;
        drvsup_init().await;
        for level in [0, 1, 2] {
            assert_eq!(
                driver_report("drvPM500", level),
                format!(
                    "    PM500 controller #0 connection failed.\n    PM500 controller 1 \
                     port=pm500RepLine, address=3, id: {} \n",
                    &ident[..50]
                )
            );
        }
    }

    /// drvESP300.cc:671-676, now at motor_init rather than ESP300Config.
    #[epics_rs::base::epics_test]
    async fn esp300_errlogs_a_connect_error_at_ioc_init() {
        use motor_common::testing::ioc_init_errlog;
        let int = ArgValue::Int;
        run_command(esp300_setup_command(), vec![int(1), int(10)]).await;
        let config = vec![int(0), ArgValue::String("espNowhere".into()), int(0)];
        run_command(esp300_config_command(), config).await;
        assert_eq!(
            ioc_init_errlog("").await,
            ["drvESP300.c:motor_init: ASYN connect error = 3"]
        );
    }

    /// drvMM4000.cc:728-733, now at motor_init rather than MM4000Config.
    #[epics_rs::base::epics_test]
    async fn mm4000_errlogs_an_invalid_model_at_ioc_init() {
        use motor_common::testing::{LineRig, ioc_init_errlog};
        let int = ArgValue::Int;
        run_command(mm4000_setup_command(), vec![int(1), int(10)]).await;
        let (_line, _io) = LineRig::new("mm4000Odd", |last| match last {
            "TH;" => "TH 0".into(),
            "VE;" => "VEXPS 1.0".into(),
            _ => String::new(),
        });
        let config = vec![int(0), ArgValue::String("mm4000Odd".into()), int(0)];
        run_command(mm4000_config_command(), config).await;
        assert_eq!(
            ioc_init_errlog("").await,
            ["drvMM4000.c:motor_init() - invalid model = XPS 1.0"]
        );
    }

    /// drvPMNC87xx.cc:853-856: a refused connect at motor_init is printf'd.
    #[epics_rs::base::epics_test]
    async fn pmnc87xx_prints_a_connect_error_at_ioc_init() {
        use motor_common::testing::{child_stdout, drvsup_init, in_child};
        if !in_child() {
            let out = child_stdout(
                module_path!(),
                "pmnc87xx_prints_a_connect_error_at_ioc_init",
            );
            let line = "drvPMNC:motor_init(), error calling pasynOctetSyncIO->connect \
                        port=pmncNowhere error=3\n";
            assert_eq!(out.matches(line).count(), 1, "{out}");
            return;
        }
        let int = ArgValue::Int;
        run_command(pmnc87xx_setup_command(), vec![int(1), int(1), int(10)]).await;
        let config = vec![int(0), ArgValue::String("pmncNowhere".into()), int(0)];
        run_command(pmnc87xx_config_command(), config).await;
        drvsup_init().await;
    }

    /// The errlog drvPMNC87xx.cc's motor_init prints for a card that
    /// answers `VER` with `ver` and `DRT` with `drt`.
    async fn pmnc87xx_init_errlog(port: &str, ver: &'static str, drt: &'static str) -> Vec<String> {
        use motor_common::testing::{LineRig, ioc_init_errlog};
        let int = ArgValue::Int;
        run_command(pmnc87xx_setup_command(), vec![int(1), int(1), int(10)]).await;
        let (_line, _io) = LineRig::new(port, move |last| match last {
            "VER" => ver.into(),
            "DRT" => drt.into(),
            _ => ">".into(),
        });
        let config = vec![int(0), ArgValue::String(port.into()), int(0)];
        run_command(pmnc87xx_config_command(), config).await;
        ioc_init_errlog("").await
    }

    /// drvPMNC87xx.cc:919-924: no `VER_STR` in the ident.
    #[epics_rs::base::epics_test]
    async fn pmnc87xx_errlogs_an_unknown_version() {
        assert_eq!(
            pmnc87xx_init_errlog("pmncNoVer", "Picomotor 2.0", "A0=2>").await,
            ["drvPMNC87xx.c:motor_init() - unknown version = Picomotor 2.0"]
        );
    }

    /// drvPMNC87xx.cc:935-938: a firmware digit other than 0, 5 or 6.
    #[epics_rs::base::epics_test]
    async fn pmnc87xx_errlogs_an_invalid_model() {
        assert_eq!(
            pmnc87xx_init_errlog("pmncBadModel", "Version 1.9.0", "A0=2>").await,
            ["drvPMNC87xx:motor_init() - invalid model = Version 1.9.0"]
        );
    }

    /// drvPMNC87xx.cc:984-987: a DRT driver type other than 8753 or 8751.
    #[epics_rs::base::epics_test]
    async fn pmnc87xx_errlogs_an_invalid_driver() {
        assert_eq!(
            pmnc87xx_init_errlog("pmncBadDrt", "Version 1.6.0", "A0=7>").await,
            ["drvPMNC87xx:motor_init() - invalid driver = 7"]
        );
    }

    /// drvPMNC87xx.cc's drvet `report` (drvPMNC87xx.cc:211-238), past the
    /// `break` after the first configured card.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pmnc87xx_controller() {
        use motor_common::testing::{LineRig, driver_report};
        let setup = pmnc87xx_setup_command();
        let none = "    No PMNC87xx controllers configured.\n";
        assert_eq!(driver_report("drvPMNC87xx", 0), none);
        let int = ArgValue::Int;
        run_command(setup, vec![int(3), int(1), int(10)]).await;
        let rig = |name: &str| {
            LineRig::new(name, |last| match last {
                "VER" => "Version 1.6.0".into(),
                "DRT" => "A0=2>".into(),
                _ => ">".into(),
            })
        };
        let (_a, _) = rig("pmncRepA");
        let (_b, _) = rig("pmncRepB");
        for (card, port) in [(0, "pmncRepA"), (2, "pmncRepB")] {
            let config = vec![int(card), ArgValue::String(port.into()), int(0)];
            run_command(pmnc87xx_config_command(), config).await;
        }
        let id = |card, port| {
            format!(
                "    PMNC87xx controller {card} asyn port= {port}, address=0, id: Version 1.6.0 \n"
            )
        };
        drvsup_init().await;
        assert_eq!(
            driver_report("drvPMNC87xx", 0),
            format!(
                "{}    PMNC87xx controller 1 connection failed.\n{}",
                id(0, "pmncRepA"),
                id(2, "pmncRepB")
            )
        );
    }

    /// drvPMNC87xx.cc:944-993: once `VER` answered, a silent `DRT` leaves
    /// the card with no axes; only an invalid model or driver drops it.
    #[epics_rs::base::epics_test]
    async fn pmnc87xx_card_answering_ver_survives_silent_drt() {
        use motor_common::testing::{LineRig, driver_report};
        let int = ArgValue::Int;
        run_command(pmnc87xx_setup_command(), vec![int(1), int(1), int(10)]).await;
        let (_line, _io) = LineRig::new("pmncNoDrt", |last| match last {
            "VER" => "Version 1.6.0".into(),
            _ => String::new(),
        });
        let config = vec![int(0), ArgValue::String("pmncNoDrt".into()), int(0)];
        run_command(pmnc87xx_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvPMNC87xx", 0),
            "    PMNC87xx controller 0 asyn port= pmncNoDrt, address=0, id: Version 1.6.0 \n"
        );
    }

    /// drvMM4000Asyn.c `motorAxisReport` (:194-224), every controller
    /// `MM4000AsynSetup` allocated; a Config stopping after its range checks
    /// leaves its calloc'd axes.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_mm4000_asyn_controller() {
        use motor_common::testing::{LineRig, driver_report};
        let int = ArgValue::Int;
        run_command(mm4000_asyn_setup_command(), vec![int(3)]).await;
        let (_line, _io) = LineRig::new("mm4kAsynLine", |last| {
            match last {
                "VE;" => "VEMM4005 Version 2.0",
                "TP;" => "1TP0.0",
                "1TU" => "1TU0.001",
                "1XH" => "1XH1.5",
                "1TL" => "1TL-10",
                "1TR" => "1TR10",
                _ => "",
            }
            .to_string()
        });
        let config = |card, port: &str, num_axes, moving, idle| {
            vec![
                int(card),
                ArgValue::String(port.into()),
                int(0),
                int(num_axes),
                int(moving),
                int(idle),
            ]
        };
        let cmd = mm4000_asyn_config_command;
        run_command(cmd(), config(0, "mm4kAsynLine", 1, 100, 1000)).await;
        run_command(cmd(), config(1, "mm4kAsynNoPort", 2, 200, 2000)).await;
        let controller =
            |n, firmware: &str| format!("Controller {n} firmware version: {firmware}\n");
        assert_eq!(
            driver_report("motorMM4000", 0),
            format!(
                "{}{}{}",
                controller(0, "MM4005 Version 2.0"),
                controller(1, ""),
                controller(2, "")
            )
        );
        let detail = |n, firmware: &str, model, moving: &str, idle: &str| {
            format!(
                "{}    model: {model}\n    moving poll period: {moving}\n    idle poll period: \
                 {idle}\n{}",
                controller(n, firmware),
                controller(n, firmware)
            )
        };
        let axis = |high: &str, low: &str, home: &str, step: &str, digits| {
            format!(
                "Axis 0\n   axisStatus:  0x0\n   high limit:  {high}\n   low limit:   {low}\n   \
                 home preset: {home}\n   step size:   {step}\n   max digits:  {digits}\n"
            )
        };
        let zero = axis("0.000000", "0.000000", "0.000000", "0.000000", 0);
        assert_eq!(
            driver_report("motorMM4000", 1),
            format!(
                "{}{}{}{zero}{zero}{}",
                detail(0, "MM4005 Version 2.0", 1, "0.100000", "1.000000"),
                axis("10.000000", "-10.000000", "1.500000", "0.001000", 5),
                detail(1, "", 0, "0.200000", "2.000000"),
                detail(2, "", 0, "0.000000", "0.000000"),
            )
        );
        // drvMM4000Asyn.c:804-808: a re-Config resets only the axes and the
        // poll periods before it connects; the firmware and model stay.
        run_command(cmd(), config(0, "mm4kAsynNoPort", 2, 300, 3000)).await;
        let text = driver_report("motorMM4000", 1);
        let again = detail(0, "MM4005 Version 2.0", 1, "0.300000", "3.000000");
        assert!(text.starts_with(&format!("{again}{zero}{zero}")), "{text}");
    }

    /// drvMM4000Asyn.c:899-903: MM4000AsynConfig starts MM4000Poller, whose
    /// `MS;` polls the axes before any drvAsynMotorConfigure.
    #[epics_rs::base::epics_test]
    async fn mm4000_asyn_config_starts_its_poller() {
        use motor_common::testing::LineRig;
        let int = ArgValue::Int;
        run_command(mm4000_asyn_setup_command(), vec![int(1)]).await;
        let (line, _io) = LineRig::new("mm4kPollLine", |last| {
            match last {
                "VE;" => "VEMM4005 Version 2.0",
                "TP;" => "1TP0.0",
                "MS;" => "1MS@",
                _ => "",
            }
            .to_string()
        });
        let port = ArgValue::String("mm4kPollLine".into());
        let config = vec![int(0), port, int(0), int(1), int(100), int(1000)];
        run_command(mm4000_asyn_config_command(), config).await;
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        let mut sent = line.sent();
        while !sent.iter().any(|c| c == "MS;") && std::time::Instant::now() < deadline {
            epics_rs::base::runtime::task::sleep(Duration::from_millis(10)).await;
            sent.extend(line.sent());
        }
        assert!(sent.iter().any(|c| c == "MS;"), "{sent:?}");
    }

    /// SMC100Controller::report then SMC100Axis::report
    /// (SMC100Driver.cpp:103-110, :150-159).
    #[epics_rs::base::epics_test]
    async fn smc100_report_prints_the_controller_then_each_axis_above_level_0() {
        let args = vec![
            ArgValue::String("smcRep".into()),
            ArgValue::String("smcRepNoLine".into()),
            ArgValue::Int(2),
            ArgValue::Int(100),
            ArgValue::Int(1000),
            ArgValue::String("0.001".into()),
        ];
        run_command(smc100_create_controller_command(), args).await;
        let head = "SMC100 motor driver smcRep, numAxes=2, moving poll period=0.100000, \
                    idle poll period=1.000000\n";
        let text = port_report("smcRep", 0);
        assert!(text.contains(&format!("{head}Port: smcRep\n")), "{text}");
        let text = port_report("smcRep", 1);
        assert!(
            text.contains(&format!("{head}  axis 1\n  axis 2\nPort: smcRep\n")),
            "{text}"
        );
    }
}

#[cfg(test)]
mod init_status {
    use super::*;
    use epics_rs::base::types::EpicsValue;
    use motor_common::testing::{LineRig, drvsup_init, record_init, run_command};
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// The RMPs of `dtyp`'s card 0 signals 0 and 1 once the records start.
    async fn rmps(dtyp: &str) -> [EpicsValue; 2] {
        let db = record_init(&format!(
            r##"
record(motor, "m1") {{ field(DTYP, "{dtyp}") field(OUT, "#C0 S0 @") }}
record(motor, "m2") {{ field(DTYP, "{dtyp}") field(OUT, "#C0 S1 @") }}
"##
        ))
        .await;
        ["m1.RMP", "m2.RMP"].map(|pv| db.get_pv(pv).unwrap())
    }

    fn wire(cmds: &[&str]) -> Vec<String> {
        cmds.iter().map(|c| c.to_string()).collect()
    }

    /// drvMM3000.cc:634-711: `motor_init` reads each axis's status once,
    /// after its `TPE`, and the record starts from that read.
    #[epics_rs::base::epics_test]
    async fn mm3000_motor_init_reads_each_status_after_its_tpe() {
        let int = ArgValue::Int;
        run_command(mm3000_setup_command(), vec![int(1), int(10)]).await;
        let (line, _io) = LineRig::new("mm3000InitLine", |last| match last {
            "VE" => "MM3000 Version 2.4".into(),
            "RC" => "1=dc 2=dc".into(),
            "1MS" | "2MS" => "D".into(),
            "1TP" => "14 ".into(),
            "2TP" => "3 ".into(),
            _ => String::new(),
        });
        let config = vec![int(0), ArgValue::String("mm3000InitLine".into()), int(0)];
        run_command(mm3000_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            line.sent(),
            wire(&[
                "VE", "ST", "VE", "RC", "1TPE", "1MS", "1TP", "2TPE", "2MS", "2TP"
            ])
        );
        assert_eq!(
            rmps("MM3000").await,
            [EpicsValue::Long(14), EpicsValue::Long(3)]
        );
    }

    /// drvMM4000.cc:757-811: `motor_init` reads each axis's status once,
    /// after its TC/TU/XH/TL/TR, and the record starts from that read.
    #[epics_rs::base::epics_test]
    async fn mm4000_motor_init_reads_each_status_after_its_queries() {
        let int = ArgValue::Int;
        run_command(mm4000_setup_command(), vec![int(1), int(10)]).await;
        let (line, _io) = LineRig::new("mm4000InitLine", |last| match last {
            "TH;" => "1TH0.014,2TH0.003".into(),
            "VE;" => "VEMM4005 Version 2.0".into(),
            "MS;" => "1MSA,2MSA".into(),
            "TP;" => "1TP0.014,2TP0.003".into(),
            "TE;" => "TE@".into(),
            c if c.ends_with("TU") => format!("{c}0.001"),
            _ => String::new(),
        });
        let config = vec![int(0), ArgValue::String("mm4000InitLine".into()), int(0)];
        run_command(mm4000_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            line.sent(),
            wire(&[
                "TH;", "ST;", "VE;", "TH;", "MS;", "TH;", "TP;", "1TC", "1TU", "1XH", "1TL", "1TR",
                "TE;", "2TC", "2TU", "2XH", "2TL", "2TR", "TE;"
            ])
        );
        assert_eq!(
            rmps("MM4000").await,
            [EpicsValue::Long(14), EpicsValue::Long(3)]
        );
    }

    /// drvPM500.cc:611-668: `motor_init` reads each axis's status once,
    /// after its `CONFIG?`, and the record starts from that read.
    #[epics_rs::base::epics_test]
    async fn pm500_motor_init_reads_each_status_after_its_config() {
        let int = ArgValue::Int;
        run_command(pm500_setup_command(), vec![int(1), int(10)]).await;
        let (line, _io) = LineRig::new("pm500InitLine", |last| match last.trim_end() {
            "SVN?" => "XDPM500 v1".into(),
            "XSTAT?" | "YSTAT?" => "XB".into(),
            "XCONFIG?" => "XCONFIG 302".into(),
            "YCONFIG?" => "YCONFIG 302".into(),
            "XR" => "XD+0.14".into(),
            "YR" => "YD+0.03".into(),
            "XM?" | "YM?" => "XM1".into(),
            _ => "ZE".into(),
        });
        let config = vec![int(0), ArgValue::String("pm500InitLine".into()), int(0)];
        run_command(pm500_config_command(), config).await;
        drvsup_init().await;
        let sent: Vec<String> = line
            .sent()
            .iter()
            .map(|c| c.trim_end().to_owned())
            .collect();
        assert_eq!(
            sent,
            wire(&[
                "SCUM 1",
                "SENAINT $AF",
                "SVN?",
                "SVN?",
                "XSTAT?",
                "YSTAT?",
                "ZSTAT?",
                "XCONFIG?",
                "XR",
                "XM?",
                "YCONFIG?",
                "YR",
                "YM?"
            ])
        );
        assert_eq!(
            rmps("PM500").await,
            [EpicsValue::Long(14), EpicsValue::Long(3)]
        );
    }

    /// drvESP300.cc:757-816: `motor_init` reads each axis's status once,
    /// after its SN?/ZB?/FR?, and the record starts from that read.
    #[epics_rs::base::epics_test]
    async fn esp300_motor_init_reads_each_status_after_its_queries() {
        let int = ArgValue::Int;
        run_command(esp300_setup_command(), vec![int(1), int(10)]).await;
        let tb = AtomicUsize::new(0);
        let (line, _io) = LineRig::new("esp300InitLine", move |last| {
            match last {
                "VE?" => "ESP300 Version 3.08",
                // The third axis is out of range.
                "TB?" if tb.fetch_add(1, Ordering::SeqCst) == 2 => "9, 0, AXIS NUMBER OUT OF RANGE",
                "TB?" | "TE?" | "01ZB?" | "02ZB?" => "0",
                "01FR?" | "02FR?" => "0.001",
                "01MD" | "02MD" | "01MO?" | "02MO?" => "1",
                "01TP?" => "0.014",
                "02TP?" => "0.003",
                _ => "",
            }
            .to_string()
                + "\r"
        });
        let config = vec![int(0), ArgValue::String("esp300InitLine".into()), int(0)];
        run_command(esp300_config_command(), config).await;
        drvsup_init().await;
        let axis = |a: &str| {
            ["SN?", "ZB?", "FR?", "TB?", "MD", "TP?", "ZH?", "MO?", "TE?"].map(|c| match c {
                "TB?" | "TE?" => c.to_owned(),
                c => format!("{a}{c}"),
            })
        };
        let mut want = wire(&["VE?", "01ST", "TB?", "02ST", "TB?", "03ST", "TB?"]);
        want.extend(axis("01"));
        want.extend(axis("02"));
        assert_eq!(line.sent(), want);
        assert_eq!(
            rmps("ESP300").await,
            [EpicsValue::Long(14), EpicsValue::Long(3)]
        );
    }

    /// drvPMNC87xx.cc:1010-1027: `motor_init` reads each axis's status
    /// once, and the record starts from that read.
    #[epics_rs::base::epics_test]
    async fn pmnc87xx_motor_init_reads_each_status_once() {
        let int = ArgValue::Int;
        run_command(pmnc87xx_setup_command(), vec![int(1), int(2), int(10)]).await;
        let (line, _io) = LineRig::new("pmncInitLine", |last| match last {
            "VER" => "Version 1.6.0".into(),
            "DRT" => "A0=2\nA1=2>".into(),
            "STA 0" | "DIAG 0" => "A0=0x04>".into(),
            "STA 1" | "DIAG 1" => "A1=0x04>".into(),
            "POS 0" => "A0=14>".into(),
            "POS 1" => "A1=3>".into(),
            _ => ">".into(),
        });
        let config = vec![int(0), ArgValue::String("pmncInitLine".into()), int(0)];
        run_command(pmnc87xx_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            line.sent(),
            wire(&[
                "VER", "DRT", "STA 0", "DIAG 0", "POS 0", "STA 1", "DIAG 1", "POS 1"
            ])
        );
        assert_eq!(
            rmps("PMNC87xx").await,
            [EpicsValue::Long(14), EpicsValue::Long(3)]
        );
    }
}

#[cfg(test)]
mod xps_profile_tests {
    use std::sync::atomic::Ordering;
    use std::sync::mpsc::Receiver;
    use std::time::Instant;

    use epics_rs::asyn::interfaces::motor::{AsynMotor, PortSets};
    use epics_rs::asyn::port::PortDriver;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::user::AsynUser;
    use epics_rs::motor::builder::MotorBuilder;
    use epics_rs::motor::controller_port::{
        PROFILE_BUILD_DONE, PROFILE_EXECUTE_DONE, PROFILE_STATUS_ABORT, PROFILE_STATUS_SUCCESS,
        index,
    };
    use epics_rs::motor::devices::register_axis;

    use super::*;
    use crate::xps::controller::verify_message;
    use crate::xps::fake::FakeLine;

    /// Two axes, G1.P1 and G1.P2, 0.5 user units a step and 0.25 XPS units
    /// a step, on an XPS-C whose FTP server is `ftp`; the profile is
    /// initialized for 8 points.
    struct Rig {
        port: PortHandle,
        line: FakeLine,
        poll: Box<dyn FnMut() -> PortSets + Send>,
        file: String,
        ftp: Receiver<String>,
    }

    fn rig(name: &str) -> Rig {
        let line = FakeLine::new(&format!("{name}Line"), "");
        *line.firmware.lock().unwrap() = "XPS-C8 Precision Platform";
        let exec = line.twin(&format!("{name}Exec"));
        let (ftp_addr, ftp) = ftp::tests::server("331 password\r\n");
        ftp::FTP_PORT.store(ftp_addr.port(), Ordering::Relaxed);
        let port_name = format!("{name}Ctrl");
        let controller = Arc::new(Mutex::new(
            XpsController::new(line.socket(SocketMode::Query), false, Duration::ZERO).unwrap(),
        ));
        let (profile, thread) = XpsProfile::new(&port_name);
        let head = XpsHead {
            moving_s: 0.1,
            idle_s: 1.0,
            enable_set_position: false,
            settling_s: 0.0,
            profile,
        };
        let port = register_controller_port(&open_owner(), &port_name, 2, head).unwrap();
        let profile_sock = Arc::new(exec.socket_with(SocketMode::Query, Duration::from_secs(30)));
        let poll = Box::new(thread.poll(controller.clone()));
        thread
            .spawn::<XpsHead>(port.clone(), controller.clone(), profile_sock.clone())
            .unwrap();
        for (addr, positioner) in [(0, "G1.P1"), (1, "G1.P2")] {
            let axis = XpsAxis::new(
                controller.clone(),
                line.socket(SocketMode::Fire),
                &port_name,
                addr,
                positioner,
                0.25,
            );
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            let user = asyn_user(&port_name, addr);
            register_axis(MotorBuilder::new(motor).trace_user(user).build_axis().axis).unwrap();
            port.write_float64_blocking(index::REC_RESOLUTION, addr, 0.5)
                .unwrap();
            port.write_float64_blocking(index::REC_OFFSET, addr, 0.0)
                .unwrap();
            port.write_int32_blocking(index::REC_DIRECTION, addr, 0)
                .unwrap();
        }
        XPS_CONTROLLERS.lock().unwrap().push(XpsRegistration {
            port_name: port_name.clone(),
            ip: "127.0.0.1".into(),
            ip_port: 5001,
            handle: port.clone(),
            controller,
            profile_sock,
            poller: Arc::new(Controller::new()),
            ftp_login: None,
        });
        let rig = Rig {
            port,
            line,
            poll,
            file: std::env::temp_dir()
                .join(format!("{name}-{}.trj", std::process::id()))
                .to_string_lossy()
                .into_owned(),
            ftp,
        };
        rig.with(|head, ctx| {
            head.profile
                .initialize_profile(ctx, 8, "Administrator", "Administrator")
        })
        .unwrap();
        rig.line.take_log();
        rig
    }

    impl Rig {
        fn with<R: Send + 'static>(
            &self,
            f: impl FnOnce(&mut XpsHead, &mut ControllerCtx<'_>) -> R + Send + 'static,
        ) -> R {
            self.port
                .with_driver_blocking::<ControllerPort<XpsHead>, _>(|d| d.with_params(f))
                .unwrap()
        }

        fn int(&self, reason: usize, addr: i32, value: i32) {
            self.port.write_int32_blocking(reason, addr, value).unwrap();
        }

        fn read_int(&self, reason: usize) -> i32 {
            self.port.read_int32_blocking(reason, 0).unwrap()
        }

        fn float_at(&self, name: &'static str, addr: i32) -> f64 {
            let reason = self.with(move |_, ctx| ctx.base.find_param(name).unwrap());
            self.port.read_float64_blocking(reason, addr).unwrap()
        }

        fn message(&self, reason: usize) -> String {
            let bytes = self.port.get_string_param_blocking(reason, 0, 256).unwrap();
            String::from_utf8(bytes).unwrap()
        }

        /// Profile [0, 1, 2] user units on both axes, 0.5 s a point, in
        /// group G1, written to `self.file`.
        fn define(&self) {
            self.int(index::PROFILE_NUM_POINTS, 0, 3);
            self.int(index::PROFILE_TIME_MODE, 0, 0);
            self.port
                .write_float64_blocking(index::PROFILE_FIXED_TIME, 0, 0.5)
                .unwrap();
            self.int(index::PROFILE_MOVE_MODE, 0, 0);
            self.int(index::PROFILE_START_PULSES, 0, 1);
            self.int(index::PROFILE_END_PULSES, 0, 3);
            self.int(index::PROFILE_NUM_PULSES, 0, 2);
            let file = self.file.clone();
            self.with(move |_, ctx| {
                for (name, value) in [
                    ("XPS_PROFILE_GROUP_NAME", "G1"),
                    ("XPS_TRAJECTORY_FILE", &file),
                ] {
                    let reason = ctx.base.find_param(name).unwrap();
                    ctx.base.set_string_param(reason, 0, value).unwrap();
                }
            });
            for addr in 0..2 {
                self.int(index::PROFILE_USE_AXIS, addr, 1);
                self.port
                    .with_driver_blocking::<ControllerPort<XpsHead>, _>(move |d| {
                        let user = AsynUser::new(index::PROFILE_POSITIONS).with_addr(addr);
                        d.write_float64_array(&user, &[0.0, 1.0, 2.0])
                    })
                    .unwrap()
                    .unwrap();
            }
        }

        fn build(&self) {
            self.define();
            self.int(index::PROFILE_BUILD, 0, 1);
            assert_eq!(
                self.read_int(index::PROFILE_BUILD_STATUS),
                PROFILE_STATUS_SUCCESS
            );
            self.line.take_log();
        }

        fn wait_executed(&self) {
            let deadline = Instant::now() + Duration::from_secs(20);
            while self.read_int(index::PROFILE_EXECUTE_STATE) != PROFILE_EXECUTE_DONE
                || self.read_int(index::PROFILE_EXECUTE) != 0
            {
                assert!(Instant::now() < deadline, "the run never ended");
                std::thread::sleep(Duration::from_millis(10));
            }
        }

        fn count(log: &[String], call: &str) -> usize {
            log.iter().filter(|c| c.starts_with(call)).count()
        }
    }

    /// XPSAxis::defineProfile: the base's steps times stepSize.
    #[test]
    fn define_profile_converts_steps_to_xps_units() {
        let rig = rig("XpsProfDef");
        rig.define();
        let positions = rig.with(|_, ctx| ctx.profile.axes[1].positions[..3].to_vec());
        assert_eq!(positions, [0.0, 0.5, 1.0]);
    }

    /// buildProfile: the file goes up by FTP, is verified, and each axis's
    /// verified range and limits land at its address.
    #[test]
    fn build_profile_uploads_verifies_and_sets_the_axis_params() {
        let rig = rig("XpsProfBuild");
        rig.define();
        rig.int(index::PROFILE_BUILD, 0, 1);

        assert_eq!(
            rig.read_int(index::PROFILE_BUILD_STATUS),
            PROFILE_STATUS_SUCCESS
        );
        assert_eq!(rig.read_int(index::PROFILE_BUILD_STATE), PROFILE_BUILD_DONE);
        assert_eq!(rig.read_int(index::PROFILE_BUILD), 0);
        assert_eq!(rig.message(index::PROFILE_BUILD_MESSAGE), verify_message(0));
        for addr in 0..2 {
            assert_eq!(rig.float_at("XPS_PROFILE_MIN_POSITION", addr), -0.5);
            assert_eq!(rig.float_at("XPS_PROFILE_MAX_POSITION", addr), 2.5);
            assert_eq!(rig.float_at("XPS_PROFILE_MAX_VELOCITY", addr), 1.2);
            assert_eq!(rig.float_at("XPS_PROFILE_MAX_ACCELERATION", addr), 3.4);
        }
        let log = rig.line.take_log();
        assert!(
            log.contains(&format!("MultipleAxesPVTVerification (G1,{})", rig.file)),
            "{log:?}"
        );
        let ftp: Vec<String> = rig.ftp.iter().collect();
        assert_eq!(ftp[0], "USER Administrator");
        assert_eq!(ftp[1], "PASS Administrator");
        assert_eq!(
            ftp[2..5],
            ["PASV", "TYPE I", "CWD /Admin/Public/Trajectories"]
        );
        let text = std::fs::read_to_string(&rig.file).unwrap();
        assert!(ftp.contains(&text), "{ftp:?}");
        assert!(text.lines().count() >= 4, "{text}");
    }

    /// buildProfile: a failed verify is C's message and FAILURE.
    #[test]
    fn build_profile_reports_a_failed_verify() {
        let rig = rig("XpsProfBadVerify");
        rig.define();
        *rig.line.fail.lock().unwrap() = "MultipleAxesPVTVerification";
        assert!(
            rig.port
                .write_int32_blocking(index::PROFILE_BUILD, 0, 1)
                .is_err()
        );
        assert_eq!(
            rig.read_int(index::PROFILE_BUILD_STATUS),
            epics_rs::motor::controller_port::PROFILE_STATUS_FAILURE
        );
        assert_eq!(
            rig.message(index::PROFILE_BUILD_MESSAGE),
            verify_message(-3)
        );
        assert_eq!(rig.read_int(index::PROFILE_BUILD_STATE), PROFILE_BUILD_DONE);
    }

    /// executeProfile wakes the XPSProfile thread, whose runProfile moves
    /// to the start, gathers through MultipleAxesPVTExecution and moves off
    /// the end.
    #[test]
    fn execute_profile_runs_on_the_profile_thread() {
        let rig = rig("XpsProfExec");
        rig.build();
        rig.int(index::PROFILE_EXECUTE, 0, 1);
        rig.wait_executed();

        let log = rig.line.take_log();
        assert_eq!(rig.message(index::PROFILE_EXECUTE_MESSAGE), " ", "{log:?}");
        assert_eq!(
            rig.read_int(index::PROFILE_EXECUTE_STATUS),
            PROFILE_STATUS_SUCCESS
        );
        let at = |call: &str| log.iter().position(|c| c.starts_with(call)).unwrap();
        assert_eq!(Rig::count(&log, "GroupMoveAbsolute (G1.P1,"), 2, "{log:?}");
        assert_eq!(Rig::count(&log, "GroupMoveAbsolute (G1.P2,"), 2, "{log:?}");
        assert!(log.contains(&format!("MultipleAxesPVTExecution (G1,{},1)", rig.file)));
        assert!(at("GroupMoveAbsolute") < at("GatheringReset"));
        assert!(at("EventExtendedStart") < at("MultipleAxesPVTExecution"));
        assert!(at("MultipleAxesPVTExecution") < at("GatheringStop"));
        // The end moves go to the last point, 1.0 XPS units.
        assert_eq!(log.last().unwrap(), "GroupStatusGet (G1,int *)");
        assert!(
            log.contains(&"GroupMoveAbsolute (G1.P1,1)".to_string()),
            "{log:?}"
        );
    }

    /// buildProfile and runProfile: an axis useAxis turns off writes zero
    /// columns, reads no SGamma or travel limits, and does not move.
    #[test]
    fn use_axis_off_keeps_the_axis_still() {
        let rig = rig("XpsProfUnused");
        rig.define();
        rig.int(index::PROFILE_USE_AXIS, 1, 0);
        rig.int(index::PROFILE_BUILD, 0, 1);
        assert_eq!(
            rig.read_int(index::PROFILE_BUILD_STATUS),
            PROFILE_STATUS_SUCCESS
        );
        let log = rig.line.take_log();
        for (call, n) in [
            ("PositionerSGammaParametersGet (G1.P1,", 1),
            ("PositionerSGammaParametersGet (G1.P2,", 0),
            ("MultipleAxesPVTVerificationResultGet (G1.P2,", 1),
            ("PositionerUserTravelLimitsGet (G1.P1,", 1),
            ("PositionerUserTravelLimitsGet (G1.P2,", 0),
        ] {
            assert_eq!(Rig::count(&log, call), n, "{call} {log:?}");
        }
        let text = std::fs::read_to_string(&rig.file).unwrap();
        assert!(
            text.lines().all(|l| l.ends_with(", 0.000000, 0.000000")),
            "{text}"
        );

        rig.int(index::PROFILE_EXECUTE, 0, 1);
        rig.wait_executed();
        let log = rig.line.take_log();
        assert_eq!(Rig::count(&log, "GroupMoveAbsolute (G1.P1,"), 2, "{log:?}");
        assert_eq!(Rig::count(&log, "GroupMoveAbsolute (G1.P2,"), 0, "{log:?}");
    }

    /// abortProfile: GroupMoveAbort on the poll socket ends the run with
    /// ABORT and C's message; the poll reads the current element meanwhile.
    #[test]
    fn abort_profile_aborts_the_running_trajectory() {
        let mut rig = rig("XpsProfAbort");
        rig.build();
        assert!((rig.poll)().is_empty());
        rig.line.hold.0.lock().unwrap().armed = true;
        rig.int(index::PROFILE_EXECUTE, 0, 1);
        {
            let (state, changed) = &*rig.line.hold;
            let state = state.lock().unwrap();
            let _held = changed
                .wait_timeout_while(state, Duration::from_secs(20), |s| !s.entered)
                .unwrap();
        }
        assert!(!(rig.poll)().is_empty());
        assert!(
            rig.line
                .take_log()
                .iter()
                .any(|c| c.starts_with("MultipleAxesPVTParametersGet (G1,"))
        );

        rig.int(index::PROFILE_ABORT, 0, 1);
        rig.wait_executed();
        assert_eq!(
            rig.read_int(index::PROFILE_EXECUTE_STATUS),
            PROFILE_STATUS_ABORT
        );
        assert_eq!(
            rig.message(index::PROFILE_EXECUTE_MESSAGE),
            "MultipleAxesPVTExecution aborted"
        );
        let log = rig.line.take_log();
        assert!(log.contains(&"GroupMoveAbort (G1)".to_string()), "{log:?}");
        assert!(log.iter().any(|c| c.starts_with("GatheringStop")));
        assert!((rig.poll)().is_empty());
    }

    /// readbackProfile: the gathered actuals and following errors, XPS
    /// units to steps to user units, and NUM_READBACKS.
    #[test]
    fn readback_profile_posts_user_unit_arrays() {
        let rig = rig("XpsProfReadback");
        rig.int(index::PROFILE_NUM_PULSES, 0, 2);
        rig.line.replies.lock().unwrap().insert(
            "GatheringDataMultipleLinesGet",
            "0,1;1.5;2;2.25\n2;2.5;3;3.5".into(),
        );
        rig.int(index::PROFILE_READBACK, 0, 1);

        assert_eq!(
            rig.read_int(index::PROFILE_READBACK_STATUS),
            PROFILE_STATUS_SUCCESS
        );
        assert_eq!(rig.read_int(index::PROFILE_NUM_READBACKS), 2);
        assert_eq!(rig.read_int(index::PROFILE_ACTUAL_PULSES), 2);
        let (readbacks, errors) = rig.with(|_, ctx| {
            let axes = &ctx.profile.axes;
            (
                [
                    axes[0].readbacks[..2].to_vec(),
                    axes[1].readbacks[..2].to_vec(),
                ],
                [
                    axes[0].following_errors[..2].to_vec(),
                    axes[1].following_errors[..2].to_vec(),
                ],
            )
        });
        assert_eq!(readbacks, [vec![3.0, 5.0], vec![4.5, 7.0]]);
        assert_eq!(errors, [vec![1.0, 1.0], vec![0.5, 1.0]]);
    }

    /// readbackProfile: fewer samples than pulses fails with C's message.
    #[test]
    fn readback_profile_reports_missing_pulses() {
        let rig = rig("XpsProfShort");
        rig.int(index::PROFILE_NUM_PULSES, 0, 3);
        rig.line.replies.lock().unwrap().insert(
            "GatheringDataMultipleLinesGet",
            "0,1;1.5;2;2.25\n2;2.5;3;3.5".into(),
        );
        rig.int(index::PROFILE_READBACK, 0, 1);
        assert_eq!(
            rig.read_int(index::PROFILE_READBACK_STATUS),
            epics_rs::motor::controller_port::PROFILE_STATUS_FAILURE
        );
        assert_eq!(
            rig.message(index::PROFILE_READBACK_MESSAGE),
            "Error, numPulses=3, currentSamples=2"
        );
    }
}
