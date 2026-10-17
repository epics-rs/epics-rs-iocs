//! SyringePump IOC — Teledyne ISCO D/H-series (native asyn port driver) plus
//! ISCO and Vindum (Modbus-only upstream, wired directly against
//! `epics-modbus-rs`).
//!
//! # Scope split (per task decision)
//! `epics-modules/SyringePump` ships three pump families. Only Teledyne
//! D/H's `teled_d.proto`/`teled_h.proto` are StreamDevice protocol files
//! needing genuine protocol translation -- that logic lives in
//! `drivers/syringepump` (see its module doc). ISCO and Vindum have no
//! `.proto`/StreamDevice anywhere in their db: both are Modbus register maps
//! wired entirely through `drvModbusAsynConfigure` + generic db templates
//! (`epics-modules/SyringePump/SPApp/Db/ISCO*.template`,
//! `VindumController.template`, `VindumPumpN.template`, and their
//! `.substitutions` register-map files). They need no driver code of their
//! own and are wired here directly against `epics-modbus-rs`'s existing
//! `drvModbusAsynConfigure`/`modbusInterposeConfig` iocsh commands.
//!
//! # dbLoadTemplate (epics-base-rs 0.24.3)
//! `ISCO*.substitutions`/`Vindum*.substitutions` load via the `dbLoadTemplate`
//! iocsh command, registered as a framework builtin as of epics-base-rs
//! 0.24.3 (`server::iocsh::commands::register_builtins`). Earlier ports of
//! this IOC worked around the then-missing command by mechanically
//! transcribing each substitutions row into an explicit `dbLoadRecords` call
//! in `st.cmd`; that expansion has been replaced by the direct
//! `dbLoadTemplate` calls now that the command exists. See `st.cmd`'s own
//! header comment for the per-block upstream-file citations.
//!
//! # Vendored modbus db/ templates
//! A crates.io dependency has no installed, runtime-resolvable path to its
//! own bundled `db/` directory (unlike a classic EPICS support module's
//! install tree, which is what `$(MODBUS)/db/...` resolves against
//! upstream). `db/aiFloat64.template`, `aoFloat64.template`, `bi_bit.template`,
//! `bo_bit.template`, `longinInt32.template`, `longoutInt32.template`,
//! `ai.template`, `mbbi.template` and `mbbo.template` are therefore local,
//! verbatim copies of the C modbus module's `modbusApp/Db` templates (see
//! each file's header).
//!
//! Usage:
//!   cargo run -p syringepump-ioc -- st.cmd

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use syringepump::connect::connect_octet;
use syringepump::driver::{Family, TeledyneDriver};

/// Fixed command timeout for every Teledyne transaction (no upstream
/// `st.cmd`/`iocBoot` ships for either Teledyne family to source a real
/// value from; chosen to match `drivers/love`'s established convention for
/// this class of RS-485 ASCII device).
const COMMAND_TIMEOUT: Duration = Duration::from_secs(1);

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: syringepump-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("SYRINGEPUMP", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    // sseq is opt-in in epics-rs (dropped from the default
    // registry with the stdRecords.dbd manifest); the db files this IOC
    // loads use it, as a C IOC links the owning module's .dbd.
    app = app.register_record_type("sseq", || {
        Box::new(epics_rs::base::server::records::sseq::SseqRecord::default())
    });
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands -- provides drvAsynSerialPortConfigure /
    // drvAsynIPPortConfigure / asynOctetSetInputEos / asynOctetSetOutputEos /
    // asynSetTraceMask / asynSetTraceIOMask / asynSetTraceIOTruncateSize /
    // asynSetTraceFile / asynSetOption.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // Modbus iocsh commands: modbusInterposeConfig, drvModbusAsynConfigure
    // (ISCO/Vindum wiring goes entirely through these -- see this file's
    // module doc for the scope-split rationale).
    let runtime_handle = epics_rs::base::runtime::task::runtime_handle();
    app = modbus_rs::ioc::register_modbus_commands(app, runtime_handle);

    // TeledyneDInit(port, serPort, serAddr, unit) -- creates a D-series
    // TeledyneDriver on a pre-configured octet port. No separate *Config
    // command: unlike drivers/love's per-address Model (a genuinely
    // runtime-configurable fact), a D/H pump's letter is fixed by which
    // template instantiation created the record (see driver.rs's module
    // doc), and `unit` is a per-driver-instance constant set once here.
    {
        app = app.register_startup_command(CommandDef::new(
            "TeledyneDInit",
            vec![
                ArgDesc {
                    name: "port",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "serPort",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "serAddr",
                    arg_type: ArgType::Int,
                },
                ArgDesc {
                    name: "unit",
                    arg_type: ArgType::Int,
                },
            ],
            "TeledyneDInit port serPort serAddr unit",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let port = match &args[0] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("port required".into()),
                };
                let ser_port = match &args[1] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("serPort required".into()),
                };
                let ser_addr = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("serAddr required".into()),
                };
                let unit = match &args[3] {
                    ArgValue::Int(n) => *n as u8,
                    _ => return Err("unit required".into()),
                };

                let handle = connect_octet(&ser_port, ser_addr, COMMAND_TIMEOUT)
                    .map_err(|e| format!("TeledyneDInit: {e}"))?;
                let claim = claim_port(&port).map_err(|e| format!("TeledyneDInit: {e}"))?;
                let driver = TeledyneDriver::new(&port, handle, Family::D, unit);

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(&claim, driver, RuntimeConfig::default())
                        .map_err(|e| format!("TeledyneDInit: {e}"))?;
                claim
                    .publish(
                        runtime_handle.port_handle().clone(),
                        ctx.db().scan_control().interrupt_gate(),
                    )
                    .map_err(|e| e.to_string())?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // TeledyneHInit(port, serPort, serAddr, unit) -- same as TeledyneDInit,
    // H-series.
    {
        app = app.register_startup_command(CommandDef::new(
            "TeledyneHInit",
            vec![
                ArgDesc {
                    name: "port",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "serPort",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "serAddr",
                    arg_type: ArgType::Int,
                },
                ArgDesc {
                    name: "unit",
                    arg_type: ArgType::Int,
                },
            ],
            "TeledyneHInit port serPort serAddr unit",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let port = match &args[0] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("port required".into()),
                };
                let ser_port = match &args[1] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("serPort required".into()),
                };
                let ser_addr = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("serAddr required".into()),
                };
                let unit = match &args[3] {
                    ArgValue::Int(n) => *n as u8,
                    _ => return Err("unit required".into()),
                };

                let handle = connect_octet(&ser_port, ser_addr, COMMAND_TIMEOUT)
                    .map_err(|e| format!("TeledyneHInit: {e}"))?;
                let claim = claim_port(&port).map_err(|e| format!("TeledyneHInit: {e}"))?;
                let driver = TeledyneDriver::new(&port, handle, Family::H, unit);

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(&claim, driver, RuntimeConfig::default())
                        .map_err(|e| format!("TeledyneHInit: {e}"))?;
                claim
                    .publish(
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
