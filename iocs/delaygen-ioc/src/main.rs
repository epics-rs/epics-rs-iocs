//! Delay/pulse generator IOC — SRS DG645, Colby PDL-100A, Coherent SDG.
//!
//! Usage:
//!   cargo run -p delaygen-ioc -- iocs/delaygen-ioc/st.cmd

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::drivers::serial_port::DrvAsynSerialPort;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use delaygen::coherent_sdg::CoherentSdgDriver;
use delaygen::colby::ColbyDriver;
use delaygen::connect::connect_octet;
use delaygen::dg645::Dg645Driver;

/// C `TIMEOUT` (`drvAsynDG645.cpp:104`, also `drvAsynColby.cpp:124`) — the
/// `pasynOctetSyncIO` command timeout for DG645 and Colby. Coherent SDG uses
/// a distinct, longer timeout — see [`COHERENT_SDG_COMMAND_TIMEOUT`].
const COMMAND_TIMEOUT: Duration = Duration::from_secs(3);

/// C `TIMEOUT` (`drvAsynCoherentSDG.cpp:87`) — `10.0`, not the `3.0` used by
/// DG645/Colby.
const COHERENT_SDG_COMMAND_TIMEOUT: Duration = Duration::from_secs(10);

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

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    // transform is opt-in in epics-rs (dropped from the default
    // registry with the stdRecords.dbd manifest); the db files this IOC
    // loads use it, as a C IOC links the owning module's .dbd.
    app = app.register_record_type("transform", || {
        Box::new(epics_rs::base::server::records::transform::TransformRecord::default())
    });
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
    // `drvAsynSerialPortConfigure` (from `register_asyn_commands`) registered
    // the port it creates only in the global `asyn_record` registry, never
    // in the `PortManager` instance passed to `register_asyn_commands` — so
    // `asynSetOption`/`asynOctetSetInputEos`/`asynOctetSetOutputEos`/
    // `asynReport`, which all resolve their port via
    // `PortManager::find_port_handle`, could never find a port created this
    // way ("port not found: serial1"), even after the fix above. Shadow
    // `drvAsynSerialPortConfigure` (last registration for a given command
    // name wins — see `IocApplication::run`/`CommandRegistry::register`)
    // with an equivalent that registers into the `PortManager` instance via
    // `register_port_with_config`. As of asyn-rs 0.24.0,
    // `register_port_with_config` is itself the single owner of both halves
    // — it publishes into the `PortManager`'s own map *and* the process-wide
    // `asyn_record` registry in one atomic call (`asyn-rs::manager.rs`), so
    // this shim must not also call `asyn_record::register_port` itself: that
    // used to be required (0.22.1) but is now a duplicate the registry
    // rejects with "port already registered", failing this command's very
    // first invocation.
    {
        let mgr_c = port_manager.clone();
        app = app.register_startup_command(CommandDef::new(
            "drvAsynSerialPortConfigure",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "ttyName",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "priority",
                    arg_type: ArgType::Int,
                },
                ArgDesc {
                    name: "noAutoConnect",
                    arg_type: ArgType::Int,
                },
                ArgDesc {
                    name: "noProcessEos",
                    arg_type: ArgType::Int,
                },
            ],
            "drvAsynSerialPortConfigure portName ttyName [priority] [noAutoConnect] [noProcessEos] \
             - create a serial octet port",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let port = match args.first() {
                    Some(ArgValue::String(s)) if !s.is_empty() => s.to_str()?.to_owned(),
                    _ => return Err("portName required".into()),
                };
                let tty = match args.get(1) {
                    Some(ArgValue::String(s)) if !s.is_empty() => s.to_str()?.to_owned(),
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

                if let Err(e) = mgr_c.register_port_with_config(
                    driver,
                    RuntimeConfig::default(),
                    ctx.db().scan_control().interrupt_gate(),
                ) {
                    ctx.println(&format!("drvAsynSerialPortConfigure: {e}"));
                    return Ok(CommandOutcome::Continue);
                }
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // C drvAsynDG645(myport,ioport,ioaddr)
    {
        app = app.register_startup_command(CommandDef::new(
            "drvAsynDG645",
            vec![
                ArgDesc {
                    name: "myport",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "ioport",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "ioaddr",
                    arg_type: ArgType::Int,
                },
            ],
            "drvAsynDG645 myport ioport ioaddr",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let my_port = match &args[0] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("myport required".into()),
                };
                let io_port = match &args[1] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("ioport required".into()),
                };
                let io_addr = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("ioaddr required".into()),
                };

                let handle = connect_octet(&io_port, io_addr, COMMAND_TIMEOUT)
                    .map_err(|e| format!("drvAsynDG645: {e}"))?;
                let driver = Dg645Driver::new(&my_port, handle)
                    .map_err(|e| format!("drvAsynDG645: failed to initialize {my_port}: {e}"))?;

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(driver, RuntimeConfig::default())
                        .map_err(|e| format!("drvAsynDG645: {e}"))?;
                epics_rs::asyn::asyn_record::register_port(
                    &my_port,
                    runtime_handle.port_handle().clone(),
                    ctx.db().scan_control().interrupt_gate(),
                )
                .map_err(|e| e.to_string())?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // C drvAsynColby(myport,ioport,addr,units,iface)
    {
        app = app.register_startup_command(CommandDef::new(
            "drvAsynColby",
            vec![
                ArgDesc {
                    name: "myport",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "ioport",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "addr",
                    arg_type: ArgType::Int,
                },
                ArgDesc {
                    name: "units",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "iface",
                    arg_type: ArgType::Int,
                },
            ],
            "drvAsynColby myport ioport addr units iface",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let my_port = match &args[0] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("myport required".into()),
                };
                let io_port = match &args[1] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("ioport required".into()),
                };
                let io_addr = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("addr required".into()),
                };
                let units = match &args[3] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("units required".into()),
                };
                let iface = match &args[4] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("iface required".into()),
                };

                let handle = connect_octet(&io_port, io_addr, COMMAND_TIMEOUT)
                    .map_err(|e| format!("drvAsynColby: {e}"))?;
                let driver = ColbyDriver::new(&my_port, handle, &units, iface)
                    .map_err(|e| format!("drvAsynColby: failed to initialize {my_port}: {e}"))?;

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(driver, RuntimeConfig::default())
                        .map_err(|e| format!("drvAsynColby: {e}"))?;
                epics_rs::asyn::asyn_record::register_port(
                    &my_port,
                    runtime_handle.port_handle().clone(),
                    ctx.db().scan_control().interrupt_gate(),
                )
                .map_err(|e| e.to_string())?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // C drvAsynCoherentSDG(myport,ioport,ioaddr)
    {
        app = app.register_startup_command(CommandDef::new(
            "drvAsynCoherentSDG",
            vec![
                ArgDesc {
                    name: "myport",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "ioport",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "ioaddr",
                    arg_type: ArgType::Int,
                },
            ],
            "drvAsynCoherentSDG myport ioport ioaddr",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let my_port = match &args[0] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("myport required".into()),
                };
                let io_port = match &args[1] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("ioport required".into()),
                };
                let io_addr = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("ioaddr required".into()),
                };

                let handle = connect_octet(&io_port, io_addr, COHERENT_SDG_COMMAND_TIMEOUT)
                    .map_err(|e| format!("drvAsynCoherentSDG: {e}"))?;
                let driver = CoherentSdgDriver::new(&my_port, handle).map_err(|e| {
                    format!("drvAsynCoherentSDG: failed to initialize {my_port}: {e}")
                })?;

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(driver, RuntimeConfig::default())
                        .map_err(|e| format!("drvAsynCoherentSDG: {e}"))?;
                epics_rs::asyn::asyn_record::register_port(
                    &my_port,
                    runtime_handle.port_handle().clone(),
                    ctx.db().scan_control().interrupt_gate(),
                )
                .map_err(|e| e.to_string())?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
