//! Parker motor IOC (OEM750, ACR/Aries and the model-1 6K).
//!
//! Assembles: an asyn IP octet port (`drvAsynIPPortConfigure`), the OEM and ACR
//! iocsh commands (`OEMCreateController`, `ACRCreateController`), the 6K's
//! `PC6KSetup`, `PC6KConfig` and `PC6KUpLoad`, the motor record type, and the
//! CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p parker-ioc -- st.cmd        # OEM750
//!   cargo run -p parker-ioc -- st.acr.cmd    # ACR / Aries
//!   cargo run -p parker-ioc -- st-pc6k.cmd   # 6K

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_parker::ioc::{
    acr_create_controller_command, oem_create_controller_command, pc6k_config_command,
    pc6k_setup_command, pc6k_upload_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: parker-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("PARKER_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);
    // motorSupport.dbd's asynMotorControllerRegister.
    for cmd in epics_rs::motor::ioc::asyn_motor_controller_commands() {
        app = app.register_startup_command(cmd);
    }

    // asyn.dbd, which every C motor IOC links: the asyn record, the devEpics
    // basic_asyn_motor.db and the OEM_RESOLUTION records use, and the iocsh
    // commands (`drvAsynIPPortConfigure`).
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // OEM750, ACR and 6K iocsh commands share one motor device support.
    app = app.register_startup_command(oem_create_controller_command());
    app = app.register_startup_command(acr_create_controller_command());
    app = app.register_startup_command(pc6k_setup_command());
    app = app.register_startup_command(pc6k_config_command());
    app = app.register_startup_command(pc6k_upload_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
