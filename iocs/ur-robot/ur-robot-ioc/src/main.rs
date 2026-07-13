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

use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use ur_robot::drivers::runtime::{
    UrPortRuntime, create_control, create_dashboard, create_gripper, create_io, create_receive,
};

/// Every port the startup script creates, kept alive for the life of the IOC.
type Ports = Arc<Mutex<Vec<UrPortRuntime>>>;

fn string_arg(args: &[ArgValue], i: usize) -> Result<String, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(s.clone()),
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
    name: &str,
    trace: &Arc<TraceManager>,
    port: UrPortRuntime,
) -> Result<(), String> {
    epics_rs::asyn::asyn_record::register_port(name, port.port_handle().clone(), trace.clone())
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

    let trace = Arc::new(TraceManager::new());
    let ports: Ports = Arc::new(Mutex::new(Vec::new()));

    let mut app = IocApplication::new();
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // URDashboardConfig(port, robot_ip, poll_period)
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "URDashboardConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "robotIP",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "URDashboardConfig portName robotIP [pollPeriod]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let ip = string_arg(args, 1)?;
                let poll = poll_arg(args, 2, 0.1)?;
                let port = create_dashboard(&name, &ip, poll)
                    .map_err(|e| format!("URDashboardConfig failed: {e}"))?;
                register(&ports, &name, &trace, port)?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // RTDEReceiveConfig(port, robot_ip, poll_period)
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "RTDEReceiveConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "robotIP",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "RTDEReceiveConfig portName robotIP [pollPeriod]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let ip = string_arg(args, 1)?;
                let poll = poll_arg(args, 2, 0.02)?;
                let port = create_receive(&name, &ip, poll)
                    .map_err(|e| format!("RTDEReceiveConfig failed: {e}"))?;
                register(&ports, &name, &trace, port)?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // RTDEInOutConfig(port, robot_ip, poll_period). The period is accepted for
    // command compatibility: the I/O port is write-only and never polls, exactly
    // as in rtde_io_driver.cpp.
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "RTDEInOutConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "robotIP",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "RTDEInOutConfig portName robotIP [pollPeriod]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let ip = string_arg(args, 1)?;
                let port =
                    create_io(&name, &ip).map_err(|e| format!("RTDEInOutConfig failed: {e}"))?;
                register(&ports, &name, &trace, port)?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // RTDEControlConfig(port, dashboard_port, receive_port, poll_period)
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "RTDEControlConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "dashboardPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "receivePort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "RTDEControlConfig portName dashboardPort receivePort [pollPeriod]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let dash = string_arg(args, 1)?;
                let recv = string_arg(args, 2)?;
                let poll = poll_arg(args, 3, 0.02)?;
                let port = create_control(&name, &dash, &recv, poll)
                    .map_err(|e| format!("RTDEControlConfig failed: {e}"))?;
                register(&ports, &name, &trace, port)?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // URGripperConfig(port, dashboard_port, poll_period)
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "URGripperConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "dashboardPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "URGripperConfig portName dashboardPort [pollPeriod]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let dash = string_arg(args, 1)?;
                let poll = poll_arg(args, 2, 0.02)?;
                let port = create_gripper(&name, &dash, poll)
                    .map_err(|e| format!("URGripperConfig failed: {e}"))?;
                register(&ports, &name, &trace, port)?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
