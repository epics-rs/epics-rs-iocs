//! Delay/pulse generator IOC — SRS DG645, Colby PDL-100A, Coherent SDG.
//!
//! Usage:
//!   cargo run -p delaygen-ioc -- iocs/delaygen-ioc/st.cmd

use std::time::Duration;

use epics_rs::asyn::asyn_record::{PortClaim, claim_port};
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::base::error::CaResult;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use delaygen::coherent_sdg::CoherentSdgDriver;
use delaygen::colby::{ColbyDriver, Iface};
use delaygen::connect::connect_octet;
use delaygen::dg645::Dg645Driver;

/// C `TIMEOUT` (`drvAsynDG645.cpp:104`, also `drvAsynColby.cpp:124`) — the
/// `pasynOctetSyncIO` command timeout for DG645 and Colby. Coherent SDG uses
/// a distinct, longer timeout — see [`COHERENT_SDG_COMMAND_TIMEOUT`].
const COMMAND_TIMEOUT: Duration = Duration::from_secs(3);

/// C `TIMEOUT` (`drvAsynCoherentSDG.cpp:87`) — `10.0`, not the `3.0` used by
/// DG645/Colby.
const COHERENT_SDG_COMMAND_TIMEOUT: Duration = Duration::from_secs(10);

/// Run `driver` as port `claim` and publish it. Every C constructor here
/// calls `registerPort` before its init query and has no unregister, so the
/// port stays whether or not that query was answered.
fn serve(ctx: &CommandContext, claim: PortClaim, driver: impl PortDriver) -> Result<(), String> {
    let (runtime_handle, _actor_jh) =
        create_port_runtime(&claim, driver, RuntimeConfig::default()).map_err(|e| e.to_string())?;
    claim
        .publish(
            runtime_handle.port_handle().clone(),
            ctx.db().scan_control().interrupt_gate(),
        )
        .map_err(|e| e.to_string())
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: delaygen-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default_path("DELAYGEN", env!("CARGO_MANIFEST_DIR"));

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
    let port_manager = epics_rs::asyn::manager::PortManager::global();
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

                let Ok(handle) = connect_octet(&io_port, io_addr, COMMAND_TIMEOUT) else {
                    errlog_vprintf(format_args!(
                        "drvAsynDG645::drvAsynDG645 port {my_port} can't connect to asynCommon \
                         on Octet server {io_port} address {io_addr}.\n"
                    ));
                    return Ok(CommandOutcome::Continue);
                };
                let Ok(claim) = claim_port(&my_port) else {
                    errlog_vprintf(format_args!(
                        "drvAsynDG645::drvAsynDG645 port {my_port} can't register port\n"
                    ));
                    return Ok(CommandOutcome::Continue);
                };
                let mut driver = match Dg645Driver::new(&my_port, handle) {
                    Ok(driver) => driver,
                    Err(e) => {
                        errlog_vprintf(format_args!(
                            "drvAsynDG645::drvAsynDG645 port {my_port} can't register standard \
                             interfaces: {e}\n"
                        ));
                        return Ok(CommandOutcome::Continue);
                    }
                };
                if let Err(step) = driver.initialize() {
                    errlog_vprintf(format_args!(
                        "drvAsynDG645::drvAsynDG645 port {my_port} {step}\n"
                    ));
                }
                serve(ctx, claim, driver)?;

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

                let Some(iface) = Iface::from_arg(iface) else {
                    ctx.println(&format!(
                        "drvAsynColby:init {my_port}: invalid interface type {iface} specified"
                    ));
                    return Ok(CommandOutcome::Continue);
                };
                let Ok(handle) = connect_octet(&io_port, io_addr, COMMAND_TIMEOUT) else {
                    ctx.println(&format!(
                        "drvAsynColby:init {my_port}: cannot connect to asyn port {io_port}"
                    ));
                    return Ok(CommandOutcome::Continue);
                };
                let Ok(claim) = claim_port(&my_port) else {
                    ctx.println(&format!(
                        "drvAsynColby::init {my_port}: failure to register port"
                    ));
                    return Ok(CommandOutcome::Continue);
                };
                let mut driver = ColbyDriver::new(&my_port, handle, &units, iface);
                // C returns -1 with no line of its own when *IDN? fails.
                let _ = driver.initialize();
                serve(ctx, claim, driver)?;

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

                let Ok(handle) = connect_octet(&io_port, io_addr, COHERENT_SDG_COMMAND_TIMEOUT)
                else {
                    ctx.println(&format!(
                        "drvAsynCoherentSDG:init {my_port}: cannot connect to asyn port {io_port}"
                    ));
                    return Ok(CommandOutcome::Continue);
                };
                let Ok(claim) = claim_port(&my_port) else {
                    ctx.println(&format!(
                        "drvAsynCoherentSDG::init {my_port}: failure to register port"
                    ));
                    return Ok(CommandOutcome::Continue);
                };
                let mut driver = CoherentSdgDriver::new(&my_port, handle);
                if let Err(step) = driver.initialize() {
                    ctx.println(&format!("drvAsynCoherentSDG::init {my_port}: {step}"));
                }
                serve(ctx, claim, driver)?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
