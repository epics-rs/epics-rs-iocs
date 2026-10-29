//! ACS SPiiPlus motion controller IOC (motorAcsMotion).
//!
//! Assembles: a serial/TCP octet port (`drvAsynSerialPortConfigure` /
//! `drvAsynIPPortConfigure`), the `AcsMotionConfig`, `SPiiPlusCreateProfile`,
//! `SPiiPlusComm` and `AcsMotionAuxIOConfig` iocsh commands, the motor record type, and the
//! CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p acsmotion-ioc -- st.cmd

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_acsmotion::ioc::{acsmotion_config_command, spiiplus_create_profile_command};
use motor_acsmotion::ioc_aux::acsmotion_aux_io_config_command;
use motor_acsmotion::ioc_comm::spiiplus_comm_command;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: acsmotion-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default_path("ACSMOTION_IOC", env!("CARGO_MANIFEST_DIR"));

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

    // profileMoveController.template's busy records: opt-in in epics-rs, as
    // a C IOC links busy's .dbd.
    app = app.register_record_type("busy", || Box::new(epics_rs::busy::BusyRecord::default()));

    // asyn.dbd, as C's IOC links it: the asyn record and devEpics, for
    // asyn_motor.db, the SPiiPlus databases and asynRecord.db, and the iocsh
    // commands (`drvAsynIPPortConfigure`).
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, asyn_factory);
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // SPiiPlus iocsh commands + motor device support.
    app = app.register_startup_command(acsmotion_config_command());
    app = app.register_startup_command(spiiplus_create_profile_command());
    app = app.register_startup_command(spiiplus_comm_command());
    app = app.register_startup_command(acsmotion_aux_io_config_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
