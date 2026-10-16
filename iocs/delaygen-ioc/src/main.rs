//! Delay/pulse generator IOC — SRS DG645, Colby PDL-100A, Coherent SDG.
//!
//! Usage:
//!   cargo run -p delaygen-ioc -- iocs/delaygen-ioc/st.cmd

use std::sync::Arc;
use std::time::Duration;

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
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

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
