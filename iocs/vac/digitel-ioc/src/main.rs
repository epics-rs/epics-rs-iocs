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

use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use vac::ioc::{device_support_factory, digitel_record_factory};

/// `vacApp/src/devVacSen.dbd` and `devDigitelPump.dbd`'s `device()` lines.
static DEVICES: &[(&str, &[DevSup])] = &[
    ("vs", &[row("devVacSen", "asyn VacSen", 5, &[2, 4])]),
    (
        "digitel",
        &[row("devDigitelPump", "asyn DigitelPump", 5, &[2, 4])],
    ),
];

const fn row(name: &'static str, choice: &'static str, number: u8, funcs: &'static [u8]) -> DevSup {
    DevSup {
        name,
        choice,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(number, funcs, false),
    }
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: digitel-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("VAC", concat!(env!("CARGO_MANIFEST_DIR"), "/.."));

    let mut app = IocApplication::new();

    // The `digitel` record type.
    let (name, factory) = digitel_record_factory();
    app = app.register_record_type(name, factory);

    // Standard asyn iocsh commands — serial/IP port configuration and the
    // per-port EOS setters the startup script uses.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // `asyn DigitelPump` device support, resolved from each record's INP link.
    app = app.register_dynamic_device_support(DEVICES, device_support_factory());

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
