//! Newport MM4000/MM4005/MM4006 motor IOC.
//!
//! Assembles: a serial octet port (`drvAsynSerialPortConfigure`), the Newport
//! MM4000 controller (model 1: `MM4000Setup`/`MM4000Config`; model 2:
//! `MM4000AsynSetup`/`MM4000AsynConfig` + `drvAsynMotorConfigure`), the motor record type, the
//! `MM4005_trajectoryScan` sequencer program, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p mm4000-ioc -- st.cmd

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_common::model2::drv_asyn_motor_configure_command;
use motor_newport::ioc::{
    MOTOR_MM4000, mm4000_asyn_config_command, mm4000_asyn_setup_command, mm4000_config_command,
    mm4000_setup_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: mm4000-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("MM4000_IOC", env!("CARGO_MANIFEST_DIR"));

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

    // trajectoryScan.db's busy records: opt-in in epics-rs, as a C IOC links
    // busy's .dbd.
    app = app.register_record_type("busy", || Box::new(epics_rs::busy::BusyRecord::default()));

    // asyn.dbd, as C's IOC links it: the asyn record, devEpics and the
    // iocsh commands (`drvAsynSerialPortConfigure`).
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, asyn_factory);
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Newport iocsh commands + motor device support.
    app = app.register_startup_command(mm4000_setup_command());
    app = app.register_startup_command(mm4000_config_command());
    app = app.register_startup_command(mm4000_asyn_setup_command());
    app = app.register_startup_command(mm4000_asyn_config_command());
    app = app.register_startup_command(drv_asyn_motor_configure_command(vec![&MOTOR_MM4000]));
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    // devNewportSeq.dbd's registrar(MM4005_trajectoryScanRegistrar): the seq
    // shell commands and MM4005_trajectoryScan.st.
    motor_newport::snl::mm4005_trajectory_scan::MM4005_trajectoryScanRegistrar();

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
