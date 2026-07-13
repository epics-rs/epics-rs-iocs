//! Love PID controller IOC — RS-485 multi-drop, generic asyn Int32/
//! UInt32Digital device support.
//!
//! Usage:
//!   cargo run -p love-ioc -- iocs/love-ioc/st.cmd

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::drivers::serial_port::DrvAsynSerialPort;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use love::connect::connect_octet;
use love::driver::{INPUT_EOS, LoveDriver, OUTPUT_EOS};
use love::registry::{K_INSTRMAX, Model};

/// C `#define K_COMTMO (1.0)` (`drvLove.c:116`) — the `pasynUser->timeout`
/// used for every Love command transaction.
const COMMAND_TIMEOUT: Duration = Duration::from_secs(1);

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: love-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("LOVE", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands — provides drvAsynSerialPortConfigure /
    // drvAsynIPPortConfigure / asynOctetSetInputEos / asynOctetSetOutputEos.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager.clone());

    // Same asyn-rs 0.22.1 startup-command / dual-registry framework gap
    // already worked around in delaygen-ioc (see its main.rs for the full
    // explanation): register_asyn_commands's set is startup-command-visible
    // only via this rebuild, and drvAsynSerialPortConfigure needs shadowing
    // so PortManager-resolved commands (asynSetOption etc.) can find a port
    // it created.
    for def in epics_rs::asyn::iocsh::build_asyn_commands(port_manager.clone()) {
        app = app.register_startup_command(def);
    }

    {
        let trace_c = trace.clone();
        let mgr_c = port_manager.clone();
        app = app.register_startup_command(CommandDef::new(
            "drvAsynSerialPortConfigure",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "ttyName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "priority",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "noAutoConnect",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "noProcessEos",
                    arg_type: ArgType::Int,
                    optional: true,
                },
            ],
            "drvAsynSerialPortConfigure portName ttyName [priority] [noAutoConnect] [noProcessEos] \
             - create a serial octet port",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let port = match args.first() {
                    Some(ArgValue::String(s)) if !s.is_empty() => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let tty = match args.get(1) {
                    Some(ArgValue::String(s)) if !s.is_empty() => s.clone(),
                    _ => return Err("ttyName required".into()),
                };
                let no_auto_connect = matches!(args.get(3), Some(ArgValue::Int(n)) if *n != 0);
                let no_process_eos = matches!(args.get(4), Some(ArgValue::Int(n)) if *n != 0);

                let driver = match DrvAsynSerialPort::configure(
                    &port,
                    &tty,
                    no_auto_connect,
                    no_process_eos,
                ) {
                    Ok(d) => d,
                    Err(e) => {
                        ctx.println(&format!("drvAsynSerialPortConfigure: {e}"));
                        return Ok(CommandOutcome::Continue);
                    }
                };

                let runtime_handle =
                    match mgr_c.register_port_with_config(driver, RuntimeConfig::default()) {
                        Ok(h) => h,
                        Err(e) => {
                            ctx.println(&format!("drvAsynSerialPortConfigure: {e}"));
                            return Ok(CommandOutcome::Continue);
                        }
                    };
                epics_rs::asyn::asyn_record::register_port(
                    &port,
                    runtime_handle.port_handle().clone(),
                    trace_c.clone(),
                )
                .map_err(|e| e.to_string())?;
                ctx.println(&format!(
                    "drvAsynSerialPortConfigure: octet port '{port}' -> {tty}"
                ));
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // LoveInit(lovPort,serPort,serAddr) -- C drvLoveInit(lovPort,serPort,serAddr)
    {
        let trace_c = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "LoveInit",
            vec![
                ArgDesc {
                    name: "lovPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "serPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "serAddr",
                    arg_type: ArgType::Int,
                    optional: false,
                },
            ],
            "LoveInit lovPort serPort serAddr",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let lov_port = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("lovPort required".into()),
                };
                let ser_port = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("serPort required".into()),
                };
                let ser_addr = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("serAddr required".into()),
                };

                let handle =
                    connect_octet(&ser_port, ser_addr, COMMAND_TIMEOUT, INPUT_EOS, OUTPUT_EOS)
                        .map_err(|e| format!("LoveInit: {e}"))?;

                let models = Arc::new(Mutex::new([Model::default(); K_INSTRMAX]));
                let driver = LoveDriver::new(&lov_port, handle, models.clone())
                    .map_err(|e| format!("LoveInit: failed to initialize {lov_port}: {e}"))?;
                love::registry::register(&lov_port, models);

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(driver, RuntimeConfig::default());
                epics_rs::asyn::asyn_record::register_port(
                    &lov_port,
                    runtime_handle.port_handle().clone(),
                    trace_c.clone(),
                )
                .map_err(|e| e.to_string())?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // LoveConfig(lovPort,addr,model) -- C drvLoveConfig(lovPort,addr,model)
    {
        app = app.register_startup_command(CommandDef::new(
            "LoveConfig",
            vec![
                ArgDesc {
                    name: "lovPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "addr",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "model",
                    arg_type: ArgType::String,
                    optional: false,
                },
            ],
            "LoveConfig lovPort addr model",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let lov_port = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("lovPort required".into()),
                };
                let addr = match &args[1] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("addr required".into()),
                };
                let model_str = match &args[2] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("model required".into()),
                };

                let table = love::registry::lookup(&lov_port)
                    .ok_or_else(|| format!("LoveConfig::failure to locate port {lov_port}"))?;
                let model = Model::parse(&model_str)
                    .ok_or_else(|| format!("LoveConfig::unsupported model \"{model_str}\""))?;
                if !(1..=K_INSTRMAX as i32).contains(&addr) {
                    return Err(format!("LoveConfig: addr {addr} out of range"));
                }
                table.lock().unwrap()[(addr - 1) as usize] = model;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
