//! MicroEpsilon capaNCDT6200 IOC — L0 config port (`capaNCDT6200.proto`
//! StreamDevice commands, translated to a native asyn command table -- no
//! StreamDevice engine in epics-rs 0.22.1) + L1 native data port
//! (`capaNCDT6200Sup.c`, ported byte-for-byte).
//!
//! Usage:
//!   cargo run -p microepsilon-ioc -- st.cmd

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::asyn_record;
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use microepsilon::config_driver::ConfigDriver;
use microepsilon::connect::connect_octet;
use microepsilon::data_driver::{configure as data_configure, keep_port_runtime_alive};

/// `capaNCDT6200.proto` sets no file-level `Timeout` directive, so
/// StreamDevice's own documented default (1s) applies here as the L0 config
/// port's blocking-transaction timeout.
const COMMAND_TIMEOUT: Duration = Duration::from_secs(1);

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: microepsilon-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("MICROEPSILON", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands. `drvAsynIPPortConfigure` (needed by
    // st.cmd for the L0 config port's raw TCP transport) is already
    // registered as BOTH a shell and a startup command directly by
    // `register_asyn_commands` itself (asyn-rs 0.22.1 `iocsh.rs:46-48`),
    // independent of `PortManager` -- unlike delaygen-ioc/love-ioc/
    // syringepump-ioc/scaler974-ioc, this IOC's st.cmd never calls a
    // PortManager-resolved command (`asynSetOption`/`asynOctetSetInputEos`/
    // `asynOctetSetOutputEos`/the trace mutators), so neither the
    // `build_asyn_commands`-as-startup-commands rebuild nor the
    // `drvAsynSerialPortConfigure` dual-registry shim documented in
    // asynrs-0221-serial-ioc-boilerplate-gap is needed here: the L0 config
    // port's EOS is set programmatically by
    // `microepsilon::connect::connect_octet` (the `.proto`'s fixed
    // InTerminator/OutTerminator), not via iocsh -- see
    // `motor-port-eos-ownership`'s "driver-programmatic EOS" case.
    let port_manager = Arc::new(PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // CapaNCDT6200ConfigInit(cfgPort, ioPort, ioAddr) -- no upstream C
    // equivalent (StreamDevice attaches directly to the raw port upstream's
    // own `drvAsynIPPortConfigure` creates). This driver needs its own named
    // asyn port for `ConfigDriver` (the db-facing port every
    // xxCapaNCDT6200.template record binds to), wrapping a raw octet port
    // created separately via `drvAsynIPPortConfigure`/
    // `drvAsynSerialPortConfigure`. Mirrors `love-ioc`'s
    // `LoveInit(lovPort, serPort, serAddr)` shape.
    {
        let trace_c = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "CapaNCDT6200ConfigInit",
            vec![
                ArgDesc {
                    name: "cfgPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "ioPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "ioAddr",
                    arg_type: ArgType::Int,
                    optional: false,
                },
            ],
            "CapaNCDT6200ConfigInit cfgPort ioPort ioAddr",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let cfg_port = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("cfgPort required".into()),
                };
                let io_port = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("ioPort required".into()),
                };
                let io_addr = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("ioAddr required".into()),
                };

                let handle = connect_octet(&io_port, io_addr, COMMAND_TIMEOUT)
                    .map_err(|e| format!("CapaNCDT6200ConfigInit: {e}"))?;
                let driver = ConfigDriver::new(&cfg_port, handle).map_err(|e| {
                    format!("CapaNCDT6200ConfigInit: failed to initialize {cfg_port}: {e}")
                })?;

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(driver, RuntimeConfig::default());
                asyn_record::register_port(
                    &cfg_port,
                    runtime_handle.port_handle().clone(),
                    trace_c.clone(),
                )
                .map_err(|e| e.to_string())?;
                // See drivers/microepsilon::data_driver's module doc
                // (`PortRuntimeHandle` gap note): dropping the last handle
                // to a port runtime closes its shutdown channel and the
                // actor thread exits. Unlike syringepump-ioc's
                // `TeledyneDInit`/`TeledyneHInit`, love-ioc's `LoveInit`, and
                // delaygen-ioc's `DG645Config`/`ColbyConfig`/
                // `CoherentSdgConfig` (all of which drop their local
                // `runtime_handle` at the end of the closure without
                // retaining it anywhere), this port's runtime is retained
                // explicitly here.
                keep_port_runtime_alive(runtime_handle);

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // capaNCDT6200Configure(portName, IPaddress, IPport) -- C
    // `capaNCDT6200Configure` (`capaNCDT6200Sup.c:682-791`). Thin wrapper:
    // `microepsilon::data_driver::configure` builds both the outer L1 port
    // and its internal `_RBK` transport port, and retains both runtimes
    // itself.
    {
        let trace_c = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "capaNCDT6200Configure",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "IPaddress",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "IPport",
                    arg_type: ArgType::String,
                    optional: false,
                },
            ],
            "capaNCDT6200Configure portName IPaddress IPport",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let ip_address = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("IPaddress required".into()),
                };
                let ip_port = match &args[2] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("IPport required".into()),
                };

                data_configure(&port_name, &ip_address, &ip_port, trace_c.clone())
                    .map_err(|e| format!("capaNCDT6200Configure: {e}"))?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
