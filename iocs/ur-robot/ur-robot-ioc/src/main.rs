//! Universal Robots IOC binary.
//!
//! Usage:
//!   cargo run -p ur-robot-ioc -- iocs/ur-robot/ur-robot-ioc/st.cmd
//!
//! The five iocsh commands mirror urRobot's, argument for argument:
//!
//! ```text
//! URDashboardConfig(port, robot_ip, poll_period)
//! RTDEReceiveConfig(port, robot_ip, poll_period)
//! RTDEInOutConfig(port, robot_ip, poll_period)
//! RTDEControlConfig(port, dashboard_port, receive_port, poll_period)
//! URGripperConfig(port, dashboard_port, poll_period)
//! ```

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::error::AsynError;
use epics_rs::asyn::manager::claim_port_driver;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::base::error::CaResult;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use ur_robot::drivers::runtime::{
    UrPortRuntime, create_control, create_dashboard, create_gripper, create_io, create_receive,
};

/// Every port the startup script creates, kept alive for the life of the IOC.
type Ports = Arc<Mutex<Vec<UrPortRuntime>>>;

fn string_arg(args: &[ArgValue], i: usize) -> Result<String, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(s.to_str()?.to_owned()),
        _ => Err(format!("argument {i} must be a string")),
    }
}

/// The poll period is a double in seconds; urRobot's defaults are 0.1 s for the
/// dashboard and I/O ports and 0.02 s for receive, control and the gripper.
fn poll_arg(args: &[ArgValue], i: usize, default: f64) -> Result<Duration, String> {
    let seconds = match args.get(i) {
        Some(ArgValue::Double(v)) => *v,
        Some(ArgValue::Int(v)) => *v as f64,
        None => default,
        _ => return Err(format!("argument {i} must be a number")),
    };
    if seconds <= 0.0 {
        return Err(format!("the poll period must be positive, got {seconds}"));
    }
    Ok(Duration::from_secs_f64(seconds))
}

fn register(
    ports: &Ports,
    claim: PortClaim,
    owner: &Arc<InterruptGate>,
    port: UrPortRuntime,
) -> Result<(), String> {
    claim
        .publish(port.port_handle().clone(), owner)
        .map_err(|e| e.to_string())?;
    ports.lock().expect("port list poisoned").push(port);
    Ok(())
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: ur-robot-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("URROBOT", env!("CARGO_MANIFEST_DIR"));

    let ports: Ports = Arc::new(Mutex::new(Vec::new()));

    let mut app = IocApplication::new();
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    // `busy` and `sseq` are opt-in in epics-rs (dropped from the default
    // registry with the stdRecords.dbd manifest); the db files this IOC
    // loads use them, as a C IOC links the owning module's .dbd.
    app = app.register_record_type("busy", || Box::new(epics_rs::busy::BusyRecord::default()));
    app = app.register_record_type("sseq", || {
        Box::new(epics_rs::base::server::records::sseq::SseqRecord::default())
    });
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // URDashboardConfig(port, robot_ip, poll_period)
    {
        let ports = ports.clone();
        app = app.register_startup_command(CommandDef::new(
            "URDashboardConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "robotIP",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                },
            ],
            "URDashboardConfig portName robotIP [pollPeriod]",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let ip = string_arg(args, 1)?;
                let poll = poll_arg(args, 2, 0.1)?;
                // asynPortDriver registers the port before the driver body
                // connects to the robot (asynPortDriver.cpp:4031-4040).
                let claim = match claim_port_driver(&name) {
                    Ok(claim) => claim,
                    Err(e) => return Ok(ctx.cxx_error(e)),
                };
                let port = match create_dashboard(&claim, &ip, poll) {
                    Ok(port) => port,
                    // The constructor throws (dashboard_driver.cpp:73-75).
                    Err(AsynError::Status { message, .. }) => return Ok(ctx.cxx_error(message)),
                    Err(e) => return Err(format!("URDashboardConfig failed: {e}")),
                };
                register(
                    &ports,
                    claim,
                    ctx.db().scan_control().interrupt_gate(),
                    port,
                )?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // RTDEReceiveConfig(port, robot_ip, poll_period)
    {
        let ports = ports.clone();
        app = app.register_startup_command(CommandDef::new(
            "RTDEReceiveConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "robotIP",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                },
            ],
            "RTDEReceiveConfig portName robotIP [pollPeriod]",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let ip = string_arg(args, 1)?;
                let poll = poll_arg(args, 2, 0.02)?;
                // asynPortDriver registers the port before the driver body
                // connects to the robot (asynPortDriver.cpp:4031-4040).
                let claim = match claim_port_driver(&name) {
                    Ok(claim) => claim,
                    Err(e) => return Ok(ctx.cxx_error(e)),
                };
                let port = create_receive(&claim, &ip, poll)
                    .map_err(|e| format!("RTDEReceiveConfig failed: {e}"))?;
                register(
                    &ports,
                    claim,
                    ctx.db().scan_control().interrupt_gate(),
                    port,
                )?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // RTDEInOutConfig(port, robot_ip, poll_period). The period is accepted for
    // command compatibility: the I/O port is write-only and never polls, exactly
    // as in rtde_io_driver.cpp.
    {
        let ports = ports.clone();
        app = app.register_startup_command(CommandDef::new(
            "RTDEInOutConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "robotIP",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                },
            ],
            "RTDEInOutConfig portName robotIP [pollPeriod]",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let ip = string_arg(args, 1)?;
                // Unused (the io port never polls) but validated, so a bad
                // period is rejected here exactly like on the other ports.
                poll_arg(args, 2, 0.1)?;
                // asynPortDriver registers the port before the driver body
                // connects to the robot (asynPortDriver.cpp:4031-4040).
                let claim = match claim_port_driver(&name) {
                    Ok(claim) => claim,
                    Err(e) => return Ok(ctx.cxx_error(e)),
                };
                let port =
                    create_io(&claim, &ip).map_err(|e| format!("RTDEInOutConfig failed: {e}"))?;
                register(
                    &ports,
                    claim,
                    ctx.db().scan_control().interrupt_gate(),
                    port,
                )?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // RTDEControlConfig(port, dashboard_port, receive_port, poll_period)
    {
        let ports = ports.clone();
        app = app.register_startup_command(CommandDef::new(
            "RTDEControlConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "dashboardPort",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "receivePort",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                },
            ],
            "RTDEControlConfig portName dashboardPort receivePort [pollPeriod]",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let dash = string_arg(args, 1)?;
                let recv = string_arg(args, 2)?;
                let poll = poll_arg(args, 3, 0.02)?;
                // asynPortDriver registers the port before the driver body
                // connects to the robot (asynPortDriver.cpp:4031-4040).
                let claim = match claim_port_driver(&name) {
                    Ok(claim) => claim,
                    Err(e) => return Ok(ctx.cxx_error(e)),
                };
                let port = match create_control(&claim, &dash, &recv, poll) {
                    Ok(port) => port,
                    // The constructor throws (rtde_control_driver.cpp:96,227).
                    Err(AsynError::Status { message, .. }) => return Ok(ctx.cxx_error(message)),
                    Err(e) => return Err(format!("RTDEControlConfig failed: {e}")),
                };
                register(
                    &ports,
                    claim,
                    ctx.db().scan_control().interrupt_gate(),
                    port,
                )?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // URGripperConfig(port, dashboard_port, poll_period)
    {
        let ports = ports.clone();
        app = app.register_startup_command(CommandDef::new(
            "URGripperConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "dashboardPort",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                },
            ],
            "URGripperConfig portName dashboardPort [pollPeriod]",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let dash = string_arg(args, 1)?;
                let poll = poll_arg(args, 2, 0.02)?;
                // asynPortDriver registers the port before the driver body
                // connects to the robot (asynPortDriver.cpp:4031-4040).
                let claim = match claim_port_driver(&name) {
                    Ok(claim) => claim,
                    Err(e) => return Ok(ctx.cxx_error(e)),
                };
                let port = create_gripper(&claim, &dash, poll)
                    .map_err(|e| format!("URGripperConfig failed: {e}"))?;
                register(
                    &ports,
                    claim,
                    ctx.db().scan_control().interrupt_gate(),
                    port,
                )?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
