//! PI GCS2 stage-controller IOC (motorPIGCS2).
//!
//! Assembles: a serial/TCP octet port (`drvAsynSerialPortConfigure` /
//! `drvAsynIPPortConfigure`), the `PIGCS2CreateController` iocsh command, the
//! motor record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p pigcs2-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_common::MotorHolder;
use motor_pi_gcs2::ioc::pigcs2_config_command;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: pigcs2-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("PIGCS2_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);

    // Standard asyn iocsh commands — provides the serial/IP port configure verbs.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // PI GCS2 iocsh command + motor device support.
    let holder = MotorHolder::new();
    app = app.register_startup_command(pigcs2_config_command(&holder));
    app = app.register_dynamic_device_support(holder.device_support_factory());

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
