//! SmarAct motor IOC (MCS2 + SCU).
//!
//! Assembles: an asyn octet port (`drvAsynIPPortConfigure` for the MCS2 or
//! `drvAsynSerialPortConfigure` for the SCU), the SmarAct iocsh commands
//! (`MCS2CreateController`; `smarActSCUCreateController` + `smarActSCUCreateAxis`),
//! the motor record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p smaract-ioc -- st.cmd        # MCS2
//!   cargo run -p smaract-ioc -- st.scu.cmd    # SCU

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_common::MotorHolder;
use motor_smaract::ioc::{
    mcs2_create_controller_command, scu_create_axis_command, scu_create_controller_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: smaract-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("SMARACT_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);

    // Standard asyn iocsh commands — provides `drvAsynIPPortConfigure`.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // SmarAct iocsh commands (MCS2 + SCU) + motor device support.
    let holder = MotorHolder::new();
    app = app.register_startup_command(mcs2_create_controller_command(&holder));
    app = app.register_startup_command(scu_create_controller_command());
    app = app.register_startup_command(scu_create_axis_command(&holder));
    app = app.register_dynamic_device_support(holder.device_support_factory());

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
