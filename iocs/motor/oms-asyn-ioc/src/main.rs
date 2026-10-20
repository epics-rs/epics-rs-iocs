//! Pro-Dex OMS MAXnet / MXA motor IOC.
//!
//! Assembles: an asyn octet port (`drvAsynIPPortConfigure` or
//! `drvAsynSerialPortConfigure`), the OMS iocsh commands (`omsMAXnetConfig`,
//! `omsMXAConfig`), the motor record type, and the CA + PVA
//! (QSRV) server.
//!
//! Usage:
//!   cargo run -p oms-asyn-ioc -- st.cmd          # MAXnet
//!   cargo run -p oms-asyn-ioc -- st.mxa.cmd      # MXA

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_oms_asyn::ioc::oms_commands;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: oms-asyn-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("OMS_ASYN_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);
    // motorSupport.dbd's asynMotorControllerRegister.
    for cmd in epics_rs::motor::ioc::asyn_motor_controller_commands() {
        app = app.register_startup_command(cmd);
    }

    // Standard asyn iocsh commands — provides the drvAsyn*PortConfigure family.
    let port_manager = epics_rs::asyn::manager::PortManager::global();
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // OMS config commands + motor device support.
    for cmd in oms_commands() {
        app = app.register_startup_command(cmd);
    }
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
