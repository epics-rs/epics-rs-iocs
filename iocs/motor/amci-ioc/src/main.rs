//! AMCI ANF2 / ANG1 stepper motor IOC.
//!
//! Assembles: an asyn IP octet port (`drvAsynIPPortConfigure`) carrying a
//! Modbus/TCP link (`modbusInterposeConfig`), one or more `drvModbusAsynConfigure`
//! register ports, the AMCI iocsh commands (`ANF2CreateController` +
//! `ANF2CreateAxis` + `ANF2StartPoller`; `ANG1CreateController`), the motor
//! record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p amci-ioc -- st.cmd        # ANF2
//!   cargo run -p amci-ioc -- st.ang1.cmd   # ANG1

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_amci::ioc::{
    anf2_create_axis_command, anf2_create_controller_command, anf2_start_poller_command,
    ang1_create_controller_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: amci-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("AMCI_IOC", env!("CARGO_MANIFEST_DIR"));

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

    // asyn.dbd, as C's IOC links it: the asyn record and devEpics, for
    // asyn_motor.db and ANF2Aux.template, and the iocsh commands
    // (`drvAsynIPPortConfigure`, the octet port a Modbus link runs over).
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, asyn_factory);
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Modbus iocsh commands: modbusInterposeConfig, drvModbusAsynConfigure.
    let handle = epics_rs::base::runtime::task::runtime_handle();
    app = modbus_rs::ioc::register_modbus_commands(app, handle);

    // AMCI iocsh commands + motor device support.
    app = app.register_startup_command(anf2_create_controller_command());
    app = app.register_startup_command(anf2_create_axis_command());
    app = app.register_startup_command(anf2_start_poller_command());
    app = app.register_startup_command(ang1_create_controller_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
