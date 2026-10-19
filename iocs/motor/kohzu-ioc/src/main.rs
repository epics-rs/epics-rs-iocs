//! Kohzu SC-200/400/800 stepper motor IOC.
//!
//! Assembles: a serial octet port (`drvAsynSerialPortConfigure`), the SC800
//! iocsh commands (`SC800Setup`, `SC800Config`), the motor record type, and the
//! CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p kohzu-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_kohzu::ioc::{sc800_config_command, sc800_setup_command};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: kohzu-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("KOHZU_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);
    // motorSupport.dbd's asynMotorControllerRegister.
    for cmd in epics_rs::motor::ioc::asyn_motor_controller_commands() {
        app = app.register_startup_command(cmd);
    }

    // Standard asyn iocsh commands — provides `drvAsynSerialPortConfigure`.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // SC800 iocsh commands, and the motor DTYPs their axes serve.
    app = app.register_startup_command(sc800_setup_command());
    app = app.register_startup_command(sc800_config_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
