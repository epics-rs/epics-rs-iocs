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
//! # dbLoadTemplate gap (Deviation, not a feasibility blocker)
//! epics-base-rs 0.22.1 has a working `.substitutions` parser
//! (`db_loader::substitution::{parse_substitutions, load_substitution_file}`)
//! but no `dbLoadTemplate` iocsh command wraps it. `dbLoadTemplate` itself is,
//! in both real EPICS and here, nothing more than "macro-expand each pattern
//! row, then `dbLoadRecords` the target template once per row" -- a pure
//! textual expansion, not a distinct capability. `st.cmd` reproduces that
//! expansion directly: every `ISCO*.substitutions`/`Vindum*.substitutions`
//! row (mechanically transcribed from the upstream file, byte-for-byte) is
//! one explicit `dbLoadRecords` call. This is textually equivalent to what
//! `dbLoadTemplate` would have produced; see `st.cmd`'s own header comment
//! for the per-block upstream-file citations.
//!
//! # Vendored modbus-rs db/ templates
//! A crates.io dependency has no installed, runtime-resolvable path to its
//! own bundled `db/` directory (unlike a classic EPICS support module's
//! install tree, which is what `$(MODBUS)/db/...` resolves against
//! upstream). `db/aiFloat64.template`, `aoFloat64.template`, `bi_bit.template`,
//! `bo_bit.template`, `longinInt32.template`, `longoutInt32.template`, and
//! `ai.template` are therefore local, byte-for-byte copies of
//! `epics-modbus-rs` 0.22.1's bundled templates (see each file's header).
//! `db/vindumMbbi.template`/`vindumMbbo.template` are new: epics-modbus-rs
//! 0.22.1 only bundles the bit-packed `mbbiDirect`/`mbboDirect` templates,
//! not a plain 4-state enumerated-register `mbbi`/`mbbo` (the shape Vindum's
//! holding-register Direction/PressureUnits/RateUnits records need).
//!
//! Usage:
//!   cargo run -p syringepump-ioc -- st.cmd

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::drivers::serial_port::DrvAsynSerialPort;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::trace::TraceManager;
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

    let trace = Arc::new(TraceManager::new());

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands -- provides drvAsynSerialPortConfigure /
    // drvAsynIPPortConfigure / asynOctetSetInputEos / asynOctetSetOutputEos /
    // asynSetTraceMask / asynSetTraceIOMask / asynSetTraceIOTruncateSize /
    // asynSetTraceFile / asynSetOption.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager.clone());

    // asyn-rs 0.22.1 startup-command / dual-registry framework gap (same one
    // worked around in delaygen-ioc/love-ioc/scaler974-ioc -- see those
    // main.rs files): register_asyn_commands's set is startup-command-visible
    // only via this rebuild (needed here because st.cmd calls
    // asynSetTraceIOMask/asynSetTraceMask/asynSetTraceIOTruncateSize/
    // asynSetTraceFile before iocInit, matching upstream's own ISCO/Vindum
    // st.cmd), and drvAsynSerialPortConfigure needs shadowing below so
    // PortManager-resolved commands can find a port it created.
    for def in epics_rs::asyn::iocsh::build_asyn_commands(port_manager.clone()) {
        app = app.register_startup_command(def);
    }

    {
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

                // `register_port_with_config` is the sole registration owner:
                // it publishes the port into both the PortManager's own map
                // (so PortManager-resolved commands like asynSetOption find
                // it directly) and the process-wide asyn_record registry
                // (asyn-rs 0.24.0 manager.rs) in one atomic call. A second
                // manual `asyn_record::register_port` on the same name used
                // to be required (asyn-rs 0.22.1: register_port_with_config
                // did not touch asyn_record and PortManager::find_port_handle
                // had no registry fallback) but is now a duplicate that the
                // registry rejects with "port already registered", failing
                // this command's very first invocation.
                if let Err(e) = mgr_c.register_port_with_config(driver, RuntimeConfig::default()) {
                    ctx.println(&format!("drvAsynSerialPortConfigure: {e}"));
                    return Ok(CommandOutcome::Continue);
                }
                ctx.println(&format!(
                    "drvAsynSerialPortConfigure: octet port '{port}' -> {tty}"
                ));
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // Modbus iocsh commands: modbusInterposeConfig, drvModbusAsynConfigure
    // (ISCO/Vindum wiring goes entirely through these -- see this file's
    // module doc for the scope-split rationale).
    let runtime_handle = epics_rs::base::runtime::task::runtime_handle();
    app = modbus_rs::ioc::register_modbus_commands(app, runtime_handle, trace.clone());

    // TeledyneDInit(port, serPort, serAddr, unit) -- creates a D-series
    // TeledyneDriver on a pre-configured octet port. No separate *Config
    // command: unlike drivers/love's per-address Model (a genuinely
    // runtime-configurable fact), a D/H pump's letter is fixed by which
    // template instantiation created the record (see driver.rs's module
    // doc), and `unit` is a per-driver-instance constant set once here.
    {
        let trace_c = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "TeledyneDInit",
            vec![
                ArgDesc {
                    name: "port",
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
                ArgDesc {
                    name: "unit",
                    arg_type: ArgType::Int,
                    optional: false,
                },
            ],
            "TeledyneDInit port serPort serAddr unit",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("port required".into()),
                };
                let ser_port = match &args[1] {
                    ArgValue::String(s) => s.clone(),
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
                let driver = TeledyneDriver::new(&port, handle, Family::D, unit);

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(driver, RuntimeConfig::default());
                epics_rs::asyn::asyn_record::register_port(
                    &port,
                    runtime_handle.port_handle().clone(),
                    trace_c.clone(),
                )
                .map_err(|e| e.to_string())?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // TeledyneHInit(port, serPort, serAddr, unit) -- same as TeledyneDInit,
    // H-series.
    {
        let trace_c = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "TeledyneHInit",
            vec![
                ArgDesc {
                    name: "port",
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
                ArgDesc {
                    name: "unit",
                    arg_type: ArgType::Int,
                    optional: false,
                },
            ],
            "TeledyneHInit port serPort serAddr unit",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("port required".into()),
                };
                let ser_port = match &args[1] {
                    ArgValue::String(s) => s.clone(),
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
                let driver = TeledyneDriver::new(&port, handle, Family::H, unit);

                let (runtime_handle, _actor_jh) =
                    create_port_runtime(driver, RuntimeConfig::default());
                epics_rs::asyn::asyn_record::register_port(
                    &port,
                    runtime_handle.port_handle().clone(),
                    trace_c.clone(),
                )
                .map_err(|e| e.to_string())?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
