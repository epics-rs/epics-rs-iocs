//! PI (Physik Instrumente) legacy motor IOC (`motorPI`).
//!
//! Assembles: a serial/TCP octet port (`drvAsynSerialPortConfigure` /
//! `drvAsynIPPortConfigure`), the PIC862 and PIC663 iocsh commands
//! (`PIC862Setup`/`PIC862Config`, `PIC663Setup`/`PIC663Config`), the motor
//! record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p pi-ioc -- st.cmd

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_pi::ioc::{
    pic630_config_command, pic630_setup_command, pic662_config_command, pic662_setup_command,
    pic663_config_command, pic663_setup_command, pic844_config_command, pic844_setup_command,
    pic848_config_command, pic848_setup_command, pic862_config_command, pic862_setup_command,
    pie516_config_command, pie516_setup_command, pie517_config_command, pie517_setup_command,
    pie710_config_command, pie710_setup_command, pie816_config_command, pie816_setup_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: pi-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("PI_IOC", env!("CARGO_MANIFEST_DIR"));

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

    // asyn.dbd, as C's IOC links it: the asyn record, devEpics and the
    // iocsh commands (the serial/IP port configure verbs).
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, asyn_factory);
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // PIC862 + PIC663 iocsh commands + motor device support.
    app = app.register_startup_command(pic862_setup_command());
    app = app.register_startup_command(pic862_config_command());
    app = app.register_startup_command(pic663_setup_command());
    app = app.register_startup_command(pic663_config_command());
    app = app.register_startup_command(pic630_setup_command());
    app = app.register_startup_command(pic630_config_command());
    app = app.register_startup_command(pic662_setup_command());
    app = app.register_startup_command(pic662_config_command());
    app = app.register_startup_command(pic844_setup_command());
    app = app.register_startup_command(pic844_config_command());
    app = app.register_startup_command(pic848_setup_command());
    app = app.register_startup_command(pic848_config_command());
    app = app.register_startup_command(pie516_setup_command());
    app = app.register_startup_command(pie516_config_command());
    app = app.register_startup_command(pie517_setup_command());
    app = app.register_startup_command(pie517_config_command());
    app = app.register_startup_command(pie710_setup_command());
    app = app.register_startup_command(pie710_config_command());
    app = app.register_startup_command(pie816_setup_command());
    app = app.register_startup_command(pie816_config_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
