//! PI GCS2 stage-controller IOC (motorPIGCS2).
//!
//! Assembles: a serial/TCP octet port (`drvAsynSerialPortConfigure` /
//! `drvAsynIPPortConfigure`), the `PI_GCS2_CreateController` iocsh command, the
//! motor record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p pigcs2-ioc -- st.cmd

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_pi_gcs2::ioc::pigcs2_config_command;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: pigcs2-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("PIGCS2_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);
    // motorSupport.dbd's asynMotorControllerRegister.
    for cmd in epics_rs::motor::ioc::asyn_motor_controller_commands() {
        app = app.register_startup_command(cmd);
    }
    // motorUtilRegister: motorUtilInit, printChIDlist, listMovingMotors.
    for cmd in epics_rs::motor::motor_util::motor_util_commands() {
        app = app.register_startup_command(cmd);
    }

    // asyn.dbd, which every C motor IOC links: the asyn record, the devEpics
    // asyn_motor.db use, and the iocsh commands (the serial/IP port configure
    // verbs).
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // PI GCS2 iocsh command + motor device support.
    app = app.register_startup_command(pigcs2_config_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
