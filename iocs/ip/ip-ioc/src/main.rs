//! IOC for the serial devices of the EPICS `ip` module.
//!
//! Usage:
//!   cargo run -p ip-ioc -- iocs/ip/ip-ioc/st.cmd
//!
//! Each device family gets one iocsh command that puts an asyn port on top of an
//! octet port created earlier by `drvAsynSerialPortConfigure` /
//! `drvAsynIPPortConfigure`:
//!
//! ```text
//! MPCConfig(port, octetPort, address, [pollPeriod])
//! TPG261Config(port, octetPort, [pollPeriod])
//! TelevacConfig(port, octetPort, numStations, numRelays, [pollPeriod])
//! MKSConfig(port, octetPort, numGauges, [pollPeriod])
//! ND261Config(port, octetPort, [pollPeriod])
//! ```

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use ip_devices::mks::create_mks;
use ip_devices::mpc::create_mpc;
use ip_devices::nd261::create_nd261;
use ip_devices::runtime::IpPortRuntime;
use ip_devices::televac::create_televac;
use ip_devices::tpg261::create_tpg261;

/// Every port the startup script creates, kept alive for the life of the IOC.
type Ports = Arc<Mutex<Vec<IpPortRuntime>>>;

fn string_arg(args: &[ArgValue], i: usize) -> Result<String, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(s.clone()),
        _ => Err(format!("argument {i} must be a string")),
    }
}

fn int_arg(args: &[ArgValue], i: usize) -> Result<i64, String> {
    match args.get(i) {
        Some(ArgValue::Int(v)) => Ok(*v),
        _ => Err(format!("argument {i} must be an integer")),
    }
}

/// Poll period in seconds; the C device support polled at the record's SCAN
/// rate, so this is the port's equivalent knob.
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

fn register(ports: &Ports, name: &str, trace: &Arc<TraceManager>, port: IpPortRuntime) {
    epics_rs::asyn::asyn_record::register_port(name, port.port_handle().clone(), trace.clone());
    ports.lock().expect("port list poisoned").push(port);
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: ip-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("IP", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());
    let ports: Ports = Arc::new(Mutex::new(Vec::new()));

    let mut app = IocApplication::new();
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // MPCConfig(port, octetPort, address, [pollPeriod])
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "MPCConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "octetPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "address",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "MPCConfig portName octetPort address [pollPeriod] - MPC/Digitel ion-pump controller",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let octet = string_arg(args, 1)?;
                let address = int_arg(args, 2)?;
                let address = u8::try_from(address)
                    .map_err(|_| format!("MPCConfig: address {address} is not 0..255"))?;
                let poll = poll_arg(args, 3, 1.0)?;
                let port = create_mpc(&name, &octet, address, poll)
                    .map_err(|e| format!("MPCConfig failed: {e}"))?;
                register(&ports, &name, &trace, port);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // TPG261Config(port, octetPort, [pollPeriod])
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "TPG261Config",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "octetPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "TPG261Config portName octetPort [pollPeriod] - Pfeiffer TPG261/TPG262 gauge controller",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let octet = string_arg(args, 1)?;
                let poll = poll_arg(args, 2, 2.0)?;
                let port = create_tpg261(&name, &octet, poll)
                    .map_err(|e| format!("TPG261Config failed: {e}"))?;
                register(&ports, &name, &trace, port);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // TelevacConfig(port, octetPort, numStations, numRelays, [pollPeriod])
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "TelevacConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "octetPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "numStations",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "numRelays",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "TelevacConfig portName octetPort numStations numRelays [pollPeriod] - \
             Televac vacuum gauge controller",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let octet = string_arg(args, 1)?;
                let stations = int_arg(args, 2)?;
                let relays = int_arg(args, 3)?;
                let stations = u8::try_from(stations)
                    .map_err(|_| format!("TelevacConfig: numStations {stations} is not 1..9"))?;
                let relays = u8::try_from(relays)
                    .map_err(|_| format!("TelevacConfig: numRelays {relays} is not 0..8"))?;
                let poll = poll_arg(args, 4, 1.0)?;
                let port = create_televac(&name, &octet, stations, relays, poll)
                    .map_err(|e| format!("TelevacConfig failed: {e}"))?;
                register(&ports, &name, &trace, port);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // MKSConfig(port, octetPort, numGauges, [pollPeriod])
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "MKSConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "octetPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "numGauges",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "MKSConfig portName octetPort numGauges [pollPeriod] - MKS/HPS SensaVac 937 \
             gauge controller",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let octet = string_arg(args, 1)?;
                let gauges = int_arg(args, 2)?;
                let gauges = u8::try_from(gauges)
                    .map_err(|_| format!("MKSConfig: numGauges {gauges} is not 1..5"))?;
                let poll = poll_arg(args, 3, 1.0)?;
                let port = create_mks(&name, &octet, gauges, poll)
                    .map_err(|e| format!("MKSConfig failed: {e}"))?;
                register(&ports, &name, &trace, port);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // ND261Config(port, octetPort, [pollPeriod])
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "ND261Config",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "octetPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                    optional: true,
                },
            ],
            "ND261Config portName octetPort [pollPeriod] - Heidenhain ND261 display unit",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let octet = string_arg(args, 1)?;
                let poll = poll_arg(args, 2, 1.0)?;
                let port = create_nd261(&name, &octet, poll)
                    .map_err(|e| format!("ND261Config failed: {e}"))?;
                register(&ports, &name, &trace, port);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
