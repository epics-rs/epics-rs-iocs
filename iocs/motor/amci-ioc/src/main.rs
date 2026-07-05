//! AMCI ANF2 / ANG1 stepper motor IOC.
//!
//! Assembles: an asyn IP octet port (`drvAsynIPPortConfigure`) carrying a
//! Modbus/TCP link (`modbusInterposeConfig`), one or more `drvModbusAsynConfigure`
//! register ports, the AMCI iocsh commands (`ANF2CreateController` +
//! `ANF2CreateAxis` + `ANF2StartPoller`; `ANG1CreateController`), the motor
//! record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p amci-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_amci::ioc::{
    anf2_create_axis_command, anf2_create_controller_command, anf2_start_poller_command,
    ang1_create_controller_command,
};
use motor_common::MotorHolder;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: amci-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("AMCI_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);

    // Standard asyn iocsh commands — provides `drvAsynIPPortConfigure`, the
    // underlying octet port a Modbus link runs over.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // Modbus iocsh commands: modbusInterposeConfig, drvModbusAsynConfigure.
    let trace = Arc::new(TraceManager::new());
    let handle = epics_rs::base::runtime::task::runtime_handle();
    app = modbus_rs::ioc::register_modbus_commands(app, handle, trace);

    // AMCI iocsh commands + motor device support.
    let holder = MotorHolder::new();
    app = app.register_startup_command(anf2_create_controller_command());
    app = app.register_startup_command(anf2_create_axis_command(&holder));
    app = app.register_startup_command(anf2_start_poller_command());
    app = app.register_startup_command(ang1_create_controller_command(&holder));
    app = app.register_dynamic_device_support(holder.device_support_factory());

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
