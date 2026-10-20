//! Phytron motor IOC (phyMOTION MCM and MCC-1/MCC-2).
//!
//! Assembles: an asyn octet port (`drvAsynIPPortConfigure` or
//! `drvAsynSerialPortConfigure`), the Phytron iocsh commands
//! (`phytronCreateController`, `phytronCreateMCC`, `phytronCreateAxis`,
//! `phytronCreateAnalog`, `phytronCreateDigital`), the motor record type,
//! asyn's devEpics for the module's aux dbs, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p phytron-ioc -- st.cmd          # phyMOTION MCM
//!   cargo run -p phytron-ioc -- st.mcc.cmd      # MCC-1 / MCC-2

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_phytron::ioc::phytron_commands;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: phytron-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("PHYTRON_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);
    // motorSupport.dbd's asynMotorControllerRegister.
    for cmd in epics_rs::motor::ioc::asyn_motor_controller_commands() {
        app = app.register_startup_command(cmd);
    }

    // asyn's devEpics, for the Phytron_*.db aux records.
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands — provides the drvAsyn*PortConfigure family.
    let port_manager = epics_rs::asyn::manager::PortManager::global();
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // Phytron create-controller / create-axis commands share a controller-name
    // registry.
    for cmd in phytron_commands() {
        app = app.register_startup_command(cmd);
    }
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
