//! Aerotech motor IOC (Ensemble + A3200).
//!
//! Assembles: an asyn octet port (`drvAsynIPPortConfigure` or
//! `drvAsynSerialPortConfigure`), the Aerotech iocsh commands
//! (`EnsembleAsynSetup`/`Config`, `A3200AsynSetup`/`Config`),
//! `drvAsynMotorConfigure` over their drivers, the motor record type, and the
//! CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p aerotech-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_aerotech::ioc::{
    MOTOR_A3200, MOTOR_ENSEMBLE, a3200_config_command, a3200_setup_command,
    ensemble_config_command, ensemble_setup_command,
};
use motor_common::model2::drv_asyn_motor_configure_command;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: aerotech-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("AEROTECH_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);

    // Standard asyn iocsh commands — provides the drvAsyn*PortConfigure family.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // Aerotech Setup/Config commands build the cards drvAsynMotorConfigure
    // opens + motor device support.
    app = app.register_startup_command(ensemble_setup_command());
    app = app.register_startup_command(ensemble_config_command());
    app = app.register_startup_command(a3200_setup_command());
    app = app.register_startup_command(a3200_config_command());
    app = app.register_startup_command(drv_asyn_motor_configure_command(vec![
        &MOTOR_ENSEMBLE,
        &MOTOR_A3200,
    ]));
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
