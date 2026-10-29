//! Ortec 974 counter/timer IOC — serial/GPIB octet port, `scaler-rs`'s
//! `scalerRecord` + `ScalerAsynDeviceSupport` (DTYP "Asyn Scaler") bound to
//! this crate's [`scaler974::driver::Scaler974Driver`] via
//! `register_dynamic_device_support` + `initScaler974`.
//!
//! Usage:
//!   cargo run -p scaler974-ioc -- iocs/scaler974-ioc/st.cmd

use std::time::Duration;

use epics_rs::base::error::CaResult;
use epics_rs::base::server::device_support::DeviceSupport;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::{DeviceSupportContext, IocApplication};

use scaler974::driver::Scaler974Driver;

/// C `sendCommand`'s local `double timeout = 1.0` (`drvScaler974.cpp`) —
/// the only timeout the driver actually uses (the file-level `timeOut`
/// macro is dead code, see `driver`'s module doc).
const COMMAND_TIMEOUT: Duration = Duration::from_secs(1);

/// `scalerSupport.dbd`'s `device(scaler,INST_IO,devScalerAsyn,"Asyn Scaler")`.
static SCALER_DEVICES: &[epics_rs::base::server::record::DevSup] =
    &[epics_rs::base::server::record::DevSup {
        name: "devScalerAsyn",
        choice: "Asyn Scaler",
        link_type: epics_rs::base::server::record::DbLinkType::InstIo,
        dset: epics_rs::base::server::record::Dset::new(7, &[2, 4, 5, 6], false),
    }];

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: scaler974-ioc <st.cmd>");
        std::process::exit(1);
    };

    // scaler-rs ships its own ready-to-use db/ directory (scaler.db,
    // scaler16.db, ...) -- no template is authored in this crate. Its
    // const names the db dir itself; SCALER names the crate dir so
    // st.cmd loads follow the repo-wide $(MACRO)/db/<file> form.
    let scaler_dir = std::path::Path::new(epics_rs::scaler::SCALER_DB_DIR)
        .parent()
        .expect("SCALER_DB_DIR is <crate>/db");
    epics_rs::base::runtime::env::set_default_path(
        "SCALER",
        scaler_dir.to_str().expect("cargo paths are UTF-8"),
    );

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    let (scaler_name, scaler_factory) = epics_rs::scaler::scaler_record_factory();
    app = app.register_record_type(scaler_name, move || scaler_factory());
    // transform is opt-in in epics-rs (dropped from the default
    // registry with the stdRecords.dbd manifest); the db files this IOC
    // loads use it, as a C IOC links the owning module's .dbd.
    app = app.register_record_type("transform", || {
        Box::new(epics_rs::base::server::records::transform::TransformRecord::default())
    });
    // Contribute the scaler record type's DTYP menu. The device support is
    // bound dynamically by DTYP string at iocInit (register_dynamic_device_support
    // below), which is name-blind and runs AFTER st.cmd's
    // `dbpf(...DTYP,"Asyn Scaler")`, so the name is not in any device menu when
    // that put validates. Register it here at app setup, before the startup
    // script runs, so the put resolves against the record type's live device
    // menu instead of failing S_db_noRSET. (asyn contributes its own menus via
    // register_asyn_device_support; the dynamic scaler support has no such hook.)
    epics_rs::base::server::record::register_device_menu(scaler_name, SCALER_DEVICES);
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands — provides drvAsynSerialPortConfigure /
    // drvAsynIPPortConfigure / asynOctetSetInputEos / asynOctetSetOutputEos.
    let port_manager = epics_rs::asyn::manager::PortManager::global();
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

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
            },
            ArgDesc {
                name: "serialPort",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "serialAddr",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "poll",
                arg_type: ArgType::Int,
            },
        ],
        "initScaler974 portName serialPort serialAddr poll",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let serial_port = match &args[1] {
                ArgValue::String(s) => s.to_str()?.to_owned(),
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

            // C prints and keeps going when the octet port is missing; no
            // driver is left for a scaler record to bind.
            let Some(driver) =
                Scaler974Driver::create(&serial_port, serial_addr, poll, COMMAND_TIMEOUT)
            else {
                return Ok(CommandOutcome::Continue);
            };
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
    app = app.register_dynamic_device_support(
        &[(
            "scaler",
            std::slice::from_ref(
                &epics_rs::scaler::device_support::scaler_asyn::ASYN_SCALER_DEVICE,
            ),
        )],
        |ctx: &DeviceSupportContext| {
            if ctx.dtyp != "Asyn Scaler" {
                return None;
            }
            let driver = scaler974::registry::take()?;
            Some(Box::new(
                epics_rs::scaler::device_support::scaler_asyn::ScalerAsynDeviceSupport::new(driver),
            ) as Box<dyn DeviceSupport>)
        },
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
