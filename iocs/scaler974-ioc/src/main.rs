//! Ortec 974 counter/timer IOC — serial/GPIB octet port, `scaler-rs`'s
//! `scalerRecord` + `ScalerAsynDeviceSupport` (DTYP "Asyn Scaler") bound to
//! this crate's [`scaler974::driver::Scaler974Driver`] via
//! `register_dynamic_device_support` + `initScaler974`.
//!
//! Usage:
//!   cargo run -p scaler974-ioc -- iocs/scaler974-ioc/st.cmd

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::drivers::serial_port::DrvAsynSerialPort;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::device_support::DeviceSupport;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::{DeviceSupportContext, IocApplication};

use scaler974::connect::connect_octet;
use scaler974::driver::Scaler974Driver;

/// C `sendCommand`'s local `double timeout = 1.0` (`drvScaler974.cpp`) —
/// the only timeout the driver actually uses (the file-level `timeOut`
/// macro is dead code, see `driver`'s module doc).
const COMMAND_TIMEOUT: Duration = Duration::from_secs(1);

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: scaler974-ioc <st.cmd>");
        std::process::exit(1);
    };

    // scaler-rs ships its own ready-to-use db/ directory (scaler.db,
    // scaler16.db, ...) -- no template is authored in this crate.
    epics_rs::base::runtime::env::set_default("SCALER", epics_rs::scaler::SCALER_DB_DIR);

    let trace = Arc::new(TraceManager::new());

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    let (scaler_name, scaler_factory) = epics_rs::scaler::scaler_record_factory();
    app = app.register_record_type(scaler_name, move || scaler_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands — provides drvAsynSerialPortConfigure /
    // drvAsynIPPortConfigure / asynOctetSetInputEos / asynOctetSetOutputEos.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager.clone());

    // Same asyn-rs 0.22.1 startup-command / dual-registry framework gap
    // already worked around in delaygen-ioc/love-ioc (see love-ioc's
    // main.rs for the full explanation): register_asyn_commands's set is
    // startup-command-visible only via this rebuild, and
    // drvAsynSerialPortConfigure needs shadowing so PortManager-resolved
    // commands (asynSetOption etc.) can find a port it created.
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
                );
                ctx.println(&format!(
                    "drvAsynSerialPortConfigure: octet port '{port}' -> {tty}"
                ));
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // initScaler974(portName,serialPort,serialAddr,poll)
    // -- C `initScaler974(const char *portName, const char *serialPort,
    //    int serialAddr, int poll)` (`drvScaler974.cpp:256`). `portName` is
    //    the asyn port name that a scalerRecord's OUT link
    //    `@asyn(portName,0)` targets; `serialPort`/`serialAddr` name the
    //    already-configured underlying octet port this driver talks to.
    app = app.register_startup_command(CommandDef::new(
        "initScaler974",
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
                name: "serialAddr",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "poll",
                arg_type: ArgType::Int,
                optional: false,
            },
        ],
        "initScaler974 portName serialPort serialAddr poll",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) => s.clone(),
                _ => return Err("portName required".into()),
            };
            let serial_port = match &args[1] {
                ArgValue::String(s) => s.clone(),
                _ => return Err("serialPort required".into()),
            };
            let serial_addr = match &args[2] {
                ArgValue::Int(n) => *n as i32,
                _ => return Err("serialAddr required".into()),
            };
            let poll = match &args[3] {
                ArgValue::Int(n) => *n as i32,
                _ => return Err("poll required".into()),
            };

            // Two independent SyncIOHandles onto the same underlying octet
            // port: one for the synchronous command path (reset/arm/
            // write_preset), one moved into the background SHOW_COUNTS
            // poll thread (see driver::Scaler974Driver::new's doc — a
            // SyncIOHandle is not Clone).
            let handle = connect_octet(&serial_port, serial_addr, COMMAND_TIMEOUT)
                .map_err(|e| format!("initScaler974: {e}"))?;
            let poll_handle = connect_octet(&serial_port, serial_addr, COMMAND_TIMEOUT)
                .map_err(|e| format!("initScaler974: {e}"))?;

            let driver = Scaler974Driver::new(handle, poll_handle, poll);
            scaler974::registry::register(&port_name, Box::new(driver))
                .map_err(|e| format!("initScaler974: {e}"))?;

            Ok(CommandOutcome::Continue)
        },
    ));

    // Binds the sole pending initScaler974 driver to the first scalerRecord
    // with DTYP "Asyn Scaler" that iocInit wires. Cannot key off the
    // record's own OUT link the way C devScalerAsyn.c::scaler_init_record
    // does -- see scaler974::registry's module doc for why
    // DeviceSupportContext::out is unusable for a scalerRecord, and why
    // this crate's registry is deliberately single-instance-only.
    app = app.register_dynamic_device_support(|ctx: &DeviceSupportContext| {
        if ctx.dtyp != "Asyn Scaler" {
            return None;
        }
        let driver = scaler974::registry::take()?;
        Some(Box::new(
            epics_rs::scaler::device_support::scaler_asyn::ScalerAsynDeviceSupport::new(driver),
        ) as Box<dyn DeviceSupport>)
    });

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
