//! Delta Tau Turbo PMAC / Geobrick motor IOC (tpmac).
//!
//! Assembles: an octet port (`pmacAsynIPConfigure` for a PMAC ethernet
//! controller, or the standard `drvAsynIPPortConfigure` /
//! `drvAsynSerialPortConfigure` for a raw-ASCII link), the PMAC iocsh commands
//! (`pmacCreateController`, `pmacCreateAxis`/`pmacCreateAxes`,
//! `pmacDisableLimitsCheck`, `pmacSetOpenLoopEncoderAxis`, `pmacCreateCsGroup`,
//! `pmacCsGroupAddAxis`, `pmacAsynCoordCreate`, `pmacSetCoord*PollPeriod`),
//! `drvAsynMotorConfigure` over the `pmacAsynCoord` driver, the motor record
//! type, autosave, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p pmac-ioc -- st.cmd

use std::sync::{Arc, Mutex};

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_common::model2::drv_asyn_motor_configure_command;
use motor_pmac::ioc::{PMAC_ASYN_COORD, pmac_commands};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: pmac-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("PMAC_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);
    // motorSupport.dbd's asynMotorControllerRegister.
    for cmd in epics_rs::motor::ioc::asyn_motor_controller_commands() {
        app = app.register_startup_command(cmd);
    }

    // asyn's devEpics, for the controller records at address 0.
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands — the serial/IP port configure verbs and the
    // EOS setters a raw-ASCII PMAC link needs.
    let port_manager = epics_rs::asyn::manager::PortManager::global();
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // PMAC iocsh commands (including pmacAsynIPConfigure, which builds an IP
    // port with the PMAC ethernet framing interpose installed), drvAsynMotorConfigure
    // + motor device support.
    for command in pmac_commands() {
        app = app.register_startup_command(command);
    }
    app = app.register_startup_command(drv_asyn_motor_configure_command(vec![&PMAC_ASYN_COORD]));
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    // Autosave
    let autosave_config = Arc::new(Mutex::new(
        epics_rs::base::server::autosave::startup::AutosaveStartupConfig::new(),
    ));
    app = app.autosave_startup(autosave_config);

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
