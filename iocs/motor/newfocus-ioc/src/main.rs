//! Newport New Focus PMNC 8750/8752 picomotor motor IOC.
//!
//! Assembles: a TCP octet port (`drvAsynIPPortConfigure`), the New Focus
//! PMNC controller (`PMNC87xxSetup`/`PMNC87xxConfig`), the motor record type, and
//! the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p newfocus-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_newport::ioc::{pmnc87xx_config_command, pmnc87xx_setup_command};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: newfocus-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("NEWFOCUS_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);

    // Standard asyn iocsh commands — provides `drvAsynIPPortConfigure`.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // Newport iocsh commands + motor device support.
    app = app.register_startup_command(pmnc87xx_setup_command());
    app = app.register_startup_command(pmnc87xx_config_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
