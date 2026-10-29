//! Micos motor IOC (SMC corvus, SMC hydra, SMC Taurus and the model-1 MoCo).
//!
//! Assembles: an asyn octet port (`drvAsynIPPortConfigure` or
//! `drvAsynSerialPortConfigure`), the corvus/hydra/Taurus iocsh commands
//! (`SMCcorvusCreateController`, `SMChydraCreateController`,
//! `SMCTaurusCreateController`, and the MoCo's `MicosSetup` / `MicosConfig`),
//! the motor record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p micos-ioc -- st.cmd           # SMC corvus
//!   cargo run -p micos-ioc -- st.hydra.cmd     # SMC hydra
//!   cargo run -p micos-ioc -- st.taurus.cmd    # SMC Taurus
//!   cargo run -p micos-ioc -- st.moco.cmd      # MoCo

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_micos::ioc::{
    corvus_change_resolution_command, corvus_create_controller_command,
    hydra_change_resolution_command, hydra_create_controller_command, micos_config_command,
    micos_setup_command, taurus_change_resolution_command, taurus_create_controller_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: micos-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default_path("MICOS_IOC", env!("CARGO_MANIFEST_DIR"));

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
    // asyn_motor.db, SMChydraAxis.db and asynRecord.db, and the iocsh
    // commands (the drvAsyn*PortConfigure family).
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, asyn_factory);
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Corvus, hydra and Taurus iocsh commands share one motor device support.
    app = app.register_startup_command(corvus_create_controller_command());
    app = app.register_startup_command(corvus_change_resolution_command());
    app = app.register_startup_command(hydra_create_controller_command());
    app = app.register_startup_command(hydra_change_resolution_command());
    app = app.register_startup_command(taurus_create_controller_command());
    app = app.register_startup_command(taurus_change_resolution_command());
    // MicosRegister.cc's model-1 commands and the "Micos MoCo" DTYP.
    app = app.register_startup_command(micos_setup_command());
    app = app.register_startup_command(micos_config_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
