//! Delay/pulse generator IOC — SRS DG645, Colby PDL-100A, Coherent SDG.
//!
//! Usage:
//!   cargo run -p delaygen-ioc -- iocs/delaygen-ioc/st.cmd

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::drivers::serial_port::DrvAsynSerialPort;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use delaygen::connect::connect_octet;
use delaygen::dg645::Dg645Driver;

/// C `TIMEOUT` (`drvAsynDG645.cpp:104`, also used by the Colby/CoherentSDG
/// drivers) — the `pasynOctetSyncIO` command timeout.
const COMMAND_TIMEOUT: Duration = Duration::from_secs(3);

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: delaygen-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("DELAYGEN", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands — provides drvAsynSerialPortConfigure /
    // drvAsynIPPortConfigure / asynOctetSetInputEos / asynOctetSetOutputEos.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager.clone());

    // asyn-rs 0.22.1's `register_asyn_commands` registers asynSetOption /
    // asynOctetSetInputEos / asynOctetSetOutputEos / asynReport / the trace
    // mutators only as *shell* commands (interactive, post-iocInit) even
    // though upstream `asynShellCommands.c` makes all of these usable from
    // st.cmd before iocInit too — `IocApplication::run` only runs
    // `startup_commands` on the startup-script shell; `shell_commands` are
    // registered on the interactive shell only after the script has
    // already executed (epics-base-rs `server/ioc_app.rs`). Every
    // delaygen startup script sets EOS/serial options via these commands
    // before the device `*Config` call runs, so register the same
    // command set as startup commands too.
    for def in epics_rs::asyn::iocsh::build_asyn_commands(port_manager.clone()) {
        app = app.register_startup_command(def);
    }

    // Second half of the same framework gap: asyn-rs 0.22.1's
    // `drvAsynSerialPortConfigure` (from `register_asyn_commands`) registers
    // the port it creates only in the global `asyn_record` registry, never
    // in the `PortManager` instance passed to `register_asyn_commands` — so
    // `asynSetOption`/`asynOctetSetInputEos`/`asynOctetSetOutputEos`/
    // `asynReport`, which all resolve their port via
    // `PortManager::find_port_handle`, can never find a port created this
    // way ("port not found: serial1"), even after the fix above. Shadow
    // `drvAsynSerialPortConfigure` (last registration for a given command
    // name wins — see `IocApplication::run`/`CommandRegistry::register`)
    // with an equivalent that registers into *both* registries:
    // `PortManager::register_port_with_config` for the `PortManager` half,
    // plus the same `asyn_record::register_port` call the original command
    // makes. No delaygen or serial-port logic is reimplemented — this only
    // bridges two registries the vendored crate itself left disjoint.
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
                );
                ctx.println(&format!(
                    "drvAsynSerialPortConfigure: octet port '{port}' -> {tty}"
                ));
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // DG645Config(myport,ioport,ioaddr) -- C drvAsynDG645(myport,ioport,ioaddr)
    {
        let trace_c = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "DG645Config",
            vec![
                ArgDesc {
                    name: "myport",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "ioport",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "ioaddr",
                    arg_type: ArgType::Int,
                    optional: false,
                },
            ],
            "DG645Config myport ioport ioaddr",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let my_port = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("myport required".into()),
                };
                let io_port = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("ioport required".into()),
                };
                let io_addr = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("ioaddr required".into()),
                };

                let handle = connect_octet(&io_port, io_addr, COMMAND_TIMEOUT)
                    .map_err(|e| format!("DG645Config: {e}"))?;
                let driver = Dg645Driver::new(&my_port, handle)
                    .map_err(|e| format!("DG645Config: failed to initialize {my_port}: {e}"))?;

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(driver, RuntimeConfig::default());
                epics_rs::asyn::asyn_record::register_port(
                    &my_port,
                    runtime_handle.port_handle().clone(),
                    trace_c.clone(),
                );

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
