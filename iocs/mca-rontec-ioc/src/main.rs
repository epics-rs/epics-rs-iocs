//! Rontec MCA IOC — serial ASCII/binary asyn MCA driver
//! (`mca-rontec`/`drvMcaRontec.c`), the shared `mca` foundation crate's
//! `devMcaAsyn` bridge, and one `mca` record.
//!
//! Usage:
//!   cargo run -p mca-rontec-ioc -- iocs/mca-rontec-ioc/st.cmd
//!
//! iocsh commands:
//! * `drvAsynSerialPortConfigure(portName,ttyName,[priority],
//!   [noAutoConnect],[noProcessEos])` — standard asyn serial port setup,
//!   provided directly by `epics_rs::asyn::iocsh::register_asyn_commands`
//!   (see this file's own comment on that call for why this IOC does not
//!   also hand-roll a second copy of the command, unlike some sibling IOCs).
//! * `RontecConfig(portName,serialPort,serialPortAddress)` — C
//!   `RontecConfig` (`drvMcaRontec.c:165`). Unlike Love, this is a single
//!   combined command (no Init/Config split): C's `RontecConfig` itself
//!   both creates the port and connects it to `serialPort` in one call.
//!   The startup script must configure the serial port's input/output EOS
//!   (`asynOctetSetInputEos`/`asynOctetSetOutputEos`) before calling this,
//!   since C `RontecConfig` only *reads* the already-configured input EOS
//!   (`pasynOctetSyncIO->getInputEos`) rather than setting it itself.

use std::sync::{Arc, Mutex};

use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::device_support::DeviceSupport;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::{DeviceSupportContext, IocApplication};

use mca::dev_mca_asyn;
use mca::interface::ASYN_MCA_DTYP;
use mca_rontec::driver::RontecDriver;

/// Every asyn port `RontecConfig` creates, kept alive for the life of the
/// process. Dropping a `PortRuntimeHandle` closes its actor's shutdown
/// channel and ends the actor thread even though a `PortHandle` clone
/// survives in `asyn_record` -- love-ioc's `LoveInit`/syringepump-ioc/
/// delaygen-ioc all drop theirs at the end of their command closure and hit
/// this (flagged, unfixed there); this IOC follows mca-ioc's own `Ports`
/// pattern instead since it is new code, not a preserved defect.
type Ports = Arc<Mutex<Vec<PortRuntimeHandle>>>;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: mca-rontec-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("MCA_RONTEC_IOC", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());
    let ports: Ports = Arc::new(Mutex::new(Vec::new()));

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    let (mca_name, mca_factory) = mca_rs::mca_record_factory();
    app = app.register_record_type(mca_name, move || mca_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands, including drvAsynSerialPortConfigure --
    // provides asynOctetSetInputEos/asynOctetSetOutputEos/asynSetOption and
    // serial port creation for both the startup script and interactive
    // shell (see `register_asyn_commands`'s own doc). Earlier IOC crates in
    // this workspace (love-ioc, syringepump-ioc, microepsilon-ioc,
    // delaygen-ioc) additionally hand-roll a *second*
    // `drvAsynSerialPortConfigure` startup command that calls
    // `PortManager::register_port_with_config` and then
    // `asyn_record::register_port` again on the same name -- asyn-rs
    // 0.24.0's `register_port_with_config` now performs that
    // `asyn_record::register_port` call itself (`asyn-rs::manager.rs`), so
    // the second call in that shim double-registers and fails with "port
    // already registered" the first time it runs. Confirmed live (this
    // IOC hit exactly that failure before this was simplified to a single
    // `register_asyn_commands` call); not applied to the other IOC crates
    // carrying the same now-redundant/broken shim, out of scope here.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager.clone());

    // RontecConfig(portName,serialPort,serialPortAddress) -- C
    // RontecConfig (`drvMcaRontec.c:165`).
    {
        let trace_c = trace.clone();
        let ports_c = ports.clone();
        app = app.register_startup_command(CommandDef::new(
            "RontecConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "serialPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "serialPortAddress",
                    arg_type: ArgType::Int,
                    optional: false,
                },
            ],
            "RontecConfig portName serialPort serialPortAddress",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match args.first() {
                    Some(ArgValue::String(s)) if !s.is_empty() => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let serial_port = match args.get(1) {
                    Some(ArgValue::String(s)) if !s.is_empty() => s.clone(),
                    _ => return Err("serialPort required".into()),
                };
                let serial_addr = match args.get(2) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => return Err("serialPortAddress required".into()),
                };

                let driver = RontecDriver::connect(&port_name, &serial_port, serial_addr)
                    .map_err(|e| format!("RontecConfig: {e}"))?;

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(driver, RuntimeConfig::default());
                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    runtime_handle.port_handle().clone(),
                    trace_c.clone(),
                )
                .map_err(|e| e.to_string())?;
                ports_c
                    .lock()
                    .expect("port list poisoned")
                    .push(runtime_handle);

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // Binds every mca record with DTYP "asynMCA" to the asyn port its own
    // INP link names (see mca-ioc's own comment on this block).
    app = app.register_dynamic_device_support(|ctx: &DeviceSupportContext| {
        if ctx.dtyp != ASYN_MCA_DTYP {
            return None;
        }
        let dev = dev_mca_asyn::connect(ctx.inp)?;
        Some(Box::new(dev) as Box<dyn DeviceSupport>)
    });

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
