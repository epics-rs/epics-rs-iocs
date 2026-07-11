//! Ion-pump IOC: the `digitel` record type + `devDigitelPump`
//! (`asyn DigitelPump`) device support for Perkin-Elmer Digitel 500/1500 and
//! Gamma Vacuum MPC/QPC controllers.
//!
//! Assembles: the standard asyn iocsh commands (`drvAsynSerialPortConfigure`,
//! `drvAsynIPPortConfigure`, `asynSetOption`, `asynOctetSetInputEos` /
//! `asynOctetSetOutputEos`), the `digitel` record type, the dynamic
//! `asyn DigitelPump` device support, and the CA + PVA (QSRV) server.
//!
//! EOS is configured entirely by the startup script — the device support emits
//! bare command bytes — so the shipped `st.cmd` carries one
//! `asynOctetSet{Input,Output}Eos` pair per port with the device's terminators
//! (the Digitels want an asymmetric `\n\r` in / `\r` out split).
//!
//! Usage:
//!   cargo run -p digitel-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use vac::ioc::{VAC_DB_DIR, device_support_factory, digitel_record_factory};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: digitel-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("VAC", VAC_DB_DIR);

    let mut app = IocApplication::new();

    // The `digitel` record type.
    let (name, factory) = digitel_record_factory();
    app = app.register_record_type(name, factory);

    // Standard asyn iocsh commands — serial/IP port configuration and the
    // per-port EOS setters the startup script uses.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // `asyn DigitelPump` device support, resolved from each record's INP link.
    app = app.register_dynamic_device_support(device_support_factory());

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
