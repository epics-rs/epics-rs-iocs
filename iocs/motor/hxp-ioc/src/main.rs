//! Newport HXP hexapod controller IOC.
//!
//! Assembles: an HXP controller with its six axes (`HXPCreateController`,
//! which opens its TCP sockets from an IP address and port), the motor record type, and the CA + PVA
//! (QSRV) server.
//!
//! Usage:
//!   cargo run -p hxp-ioc -- st.cmd

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_newport::ioc::hxp_create_controller_command;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: hxp-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default_path("HXP_IOC", env!("CARGO_MANIFEST_DIR"));

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

    // asyn.dbd, as C's IOC links it: the asyn record, devEpics for
    // HXP_extra.db / HXP_coords.db at address 0, and the iocsh commands
    // (`drvAsynIPPortConfigure`). Then the sseq HXP_coords.db syncs the
    // motors with after setting a coordinate system (opt-in in epics-rs).
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, asyn_factory);
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);
    app = app.register_record_type("sseq", || {
        Box::new(epics_rs::base::server::records::sseq::SseqRecord::default())
    });

    // HXP controller create command (creates all six axes) and the motor
    // device support.
    app = app.register_startup_command(hxp_create_controller_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
