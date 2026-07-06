//! PI (Physik Instrumente) legacy motor IOC (`motorPI`).
//!
//! Assembles: a serial/TCP octet port (`drvAsynSerialPortConfigure` /
//! `drvAsynIPPortConfigure`), the PIC862 and PIC663 iocsh commands
//! (`PIC862Setup`/`PIC862Config`, `PIC663Setup`/`PIC663Config`), the motor
//! record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p pi-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_common::MotorHolder;
use motor_pi::ioc::{
    pic630_config_command, pic630_setup_command, pic662_config_command, pic662_setup_command,
    pic663_config_command, pic663_setup_command, pic844_config_command, pic844_setup_command,
    pic848_config_command, pic848_setup_command, pic862_config_command, pic862_setup_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: pi-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("PI_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);

    // Standard asyn iocsh commands — provides the serial/IP port configure verbs.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // PIC862 + PIC663 iocsh commands + motor device support.
    let holder = MotorHolder::new();
    app = app.register_startup_command(pic862_setup_command());
    app = app.register_startup_command(pic862_config_command(&holder));
    app = app.register_startup_command(pic663_setup_command());
    app = app.register_startup_command(pic663_config_command(&holder));
    app = app.register_startup_command(pic630_setup_command());
    app = app.register_startup_command(pic630_config_command(&holder));
    app = app.register_startup_command(pic662_setup_command());
    app = app.register_startup_command(pic662_config_command(&holder));
    app = app.register_startup_command(pic844_setup_command());
    app = app.register_startup_command(pic844_config_command(&holder));
    app = app.register_startup_command(pic848_setup_command());
    app = app.register_startup_command(pic848_config_command(&holder));
    app = app.register_dynamic_device_support(holder.device_support_factory());

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
