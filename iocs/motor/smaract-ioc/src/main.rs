//! SmarAct motor IOC (MCS2 + SCU).
//!
//! Assembles: an asyn octet port (`drvAsynIPPortConfigure` for the MCS2 or
//! `drvAsynSerialPortConfigure` for the SCU), the SmarAct iocsh commands
//! (`MCS2CreateController`; `smarActSCUCreateController` + `smarActSCUCreateAxis`),
//! the motor record type, motorUtil, autosave, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p smaract-ioc -- st.cmd        # MCS2
//!   cargo run -p smaract-ioc -- st.scu.cmd    # SCU

use std::sync::{Arc, Mutex};

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_smaract::ioc::{
    mcs_create_axis_command, mcs_create_controller_command, mcs2_create_controller_command,
    scu_create_axis_command, scu_create_controller_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: smaract-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("SMARACT_IOC", env!("CARGO_MANIFEST_DIR"));

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

    // asyn's devEpics, for basic_asyn_motor.db's and MCS2_Extra.db's records,
    // and the asyn iocsh commands (`drvAsynIPPortConfigure`).
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // SmarAct iocsh commands (MCS2 + SCU + MCS) + motor device support.
    app = app.register_startup_command(mcs2_create_controller_command());
    app = app.register_startup_command(scu_create_controller_command());
    app = app.register_startup_command(scu_create_axis_command());
    app = app.register_startup_command(mcs_create_controller_command());
    app = app.register_startup_command(mcs_create_axis_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    // Autosave
    let autosave_config = Arc::new(Mutex::new(
        epics_rs::base::server::autosave::startup::AutosaveStartupConfig::new(),
    ));
    app = app.autosave_startup(autosave_config);

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
