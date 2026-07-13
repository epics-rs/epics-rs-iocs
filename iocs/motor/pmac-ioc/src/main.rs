//! Delta Tau Turbo PMAC / Geobrick motor IOC (tpmac).
//!
//! Assembles: an octet port (`pmacAsynIPConfigure` for a PMAC ethernet
//! controller, or the standard `drvAsynIPPortConfigure` /
//! `drvAsynSerialPortConfigure` for a raw-ASCII link), the PMAC iocsh commands
//! (`pmacCreateController`, `pmacCreateAxis`/`pmacCreateAxes`,
//! `pmacDisableLimitsCheck`, `pmacSetOpenLoopEncoderAxis`, `pmacCreateCsGroup`,
//! `pmacCsGroupAddAxis`, `pmacCsGroupSwitch`, `pmacAsynCoordCreate`), the motor
//! record type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p pmac-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_common::MotorHolder;
use motor_pmac::ioc::pmac_commands;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: pmac-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("PMAC_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);

    // Standard asyn iocsh commands — the serial/IP port configure verbs and the
    // EOS setters a raw-ASCII PMAC link needs.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // PMAC iocsh commands (including pmacAsynIPConfigure, which builds an IP
    // port with the PMAC ethernet framing interpose installed) + motor device
    // support.
    let trace = Arc::new(TraceManager::new());
    let holder = MotorHolder::new();
    for command in pmac_commands(&holder, trace) {
        app = app.register_startup_command(command);
    }
    app = app.register_dynamic_device_support(holder.device_support_factory());

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
