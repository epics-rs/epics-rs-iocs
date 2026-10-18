//! IMS motor IOC.
//!
//! Assembles: an asyn octet port (`drvAsynIPPortConfigure` or
//! `drvAsynSerialPortConfigure`), the IMS iocsh commands
//! (`ImsMDrivePlusCreateController` and the model-1 `IM483PLSetup` /
//! `IM483PLConfig`, `IM483SMSetup` / `IM483SMConfig`, `MDriveSetup` /
//! `MDriveConfig`), the motor record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p ims-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_ims::ioc::{
    im483pl_config_command, im483pl_setup_command, im483sm_config_command, im483sm_setup_command,
    ims_create_controller_command, mdrive_config_command, mdrive_setup_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: ims-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("IMS_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);
    // motorSupport.dbd's asynMotorControllerRegister.
    for cmd in epics_rs::motor::ioc::asyn_motor_controller_commands() {
        app = app.register_startup_command(cmd);
    }

    // Standard asyn iocsh commands — provides the drvAsyn*PortConfigure family.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // IMS config command creates the controller + its single axis on one motor
    // device support (DTYP = motorPort).
    app = app.register_startup_command(ims_create_controller_command());
    // ImsRegister.cc's model-1 commands and their DTYPs.
    app = app.register_startup_command(im483pl_setup_command());
    app = app.register_startup_command(im483pl_config_command());
    app = app.register_startup_command(im483sm_setup_command());
    app = app.register_startup_command(im483sm_config_command());
    app = app.register_startup_command(mdrive_setup_command());
    app = app.register_startup_command(mdrive_config_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
