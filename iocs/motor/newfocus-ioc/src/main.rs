//! Newport New Focus picomotor motor IOC: PMNC 8750/8752 and 8742/8743-CL.
//!
//! Assembles: a TCP octet port (`drvAsynIPPortConfigure`), the New Focus
//! PMNC controller (`PMNC87xxSetup`/`PMNC87xxConfig`), the 874x controller
//! (`nf874xCreateController`), the motor record type, asyn's device support,
//! and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p newfocus-ioc -- st.cmd

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_newport::ioc::{pmnc87xx_config_command, pmnc87xx_setup_command};
use motor_newport::ioc_nf874x::nf874x_create_controller_command;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: newfocus-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default_path("NEWFOCUS_IOC", env!("CARGO_MANIFEST_DIR"));

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
    // iocsh commands (`drvAsynIPPortConfigure`).
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, asyn_factory);
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Newport iocsh commands + motor device support.
    app = app.register_startup_command(pmnc87xx_setup_command());
    app = app.register_startup_command(pmnc87xx_config_command());
    app = app.register_startup_command(nf874x_create_controller_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
