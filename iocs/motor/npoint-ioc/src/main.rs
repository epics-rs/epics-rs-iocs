//! nPoint C300 motor IOC.
//!
//! Assembles: an asyn IP octet port (`drvAsynIPPortConfigure`), the C300 iocsh
//! command (`C300CreateController`), the LC400 ones (`LC400CreateController`,
//! `LC400ConfigAxis`) with asyn's devEpics for `LC400_auxParameters.db`, the
//! motor record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p npoint-ioc -- st.cmd

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_npoint::ioc::c300_create_controller_command;
use motor_npoint::ioc_lc400::lc400_commands;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: npoint-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("NPOINT_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);
    // motorSupport.dbd's asynMotorControllerRegister.
    for cmd in epics_rs::motor::ioc::asyn_motor_controller_commands() {
        app = app.register_startup_command(cmd);
    }

    // asyn.dbd, which every C motor IOC links: the asyn record and the
    // devEpics asyn_motor.db and LC400_auxParameters.db use.
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands — provides `drvAsynIPPortConfigure`.
    let port_manager = epics_rs::asyn::manager::PortManager::global();
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // C300 iocsh command + motor device support.
    app = app.register_startup_command(c300_create_controller_command());
    for command in lc400_commands() {
        app = app.register_startup_command(command);
    }
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
