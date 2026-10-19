//! SyringePump IOC — Teledyne ISCO D/H-series on StreamDevice (SPApp/Db's
//! teled_d.proto/teled_h.proto through stream-rs, as C) plus ISCO and
//! Vindum (Modbus-only upstream, wired directly against `epics-modbus-rs`).
//!
//! # dbLoadTemplate (epics-base-rs 0.24.3)
//! `ISCO*.substitutions`/`Vindum*.substitutions` load via the `dbLoadTemplate`
//! iocsh command, registered as a framework builtin as of epics-base-rs
//! 0.24.3 (`server::iocsh::commands::register_builtins`). Earlier ports of
//! this IOC worked around the then-missing command by mechanically
//! transcribing each substitutions row into an explicit `dbLoadRecords` call
//! in `st.cmd`; that expansion has been replaced by the direct
//! `dbLoadTemplate` calls now that the command exists. See `st.cmd`'s own
//! header comment for the per-block upstream-file citations.
//!
//! # Vendored modbus db/ templates
//! A crates.io dependency has no installed, runtime-resolvable path to its
//! own bundled `db/` directory (unlike a classic EPICS support module's
//! install tree, which is what `$(MODBUS)/db/...` resolves against
//! upstream). `db/aiFloat64.template`, `aoFloat64.template`, `bi_bit.template`,
//! `bo_bit.template`, `longinInt32.template`, `longoutInt32.template`,
//! `ai.template`, `mbbi.template` and `mbbo.template` are therefore local,
//! verbatim copies of the C modbus module's `modbusApp/Db` templates (see
//! each file's header).
//!
//! Usage:
//!   cargo run -p syringepump-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: syringepump-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("SYRINGEPUMP", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    // sseq is opt-in in epics-rs (dropped from the default
    // registry with the stdRecords.dbd manifest); the db files this IOC
    // loads use it, as a C IOC links the owning module's .dbd.
    app = app.register_record_type("sseq", || {
        Box::new(epics_rs::base::server::records::sseq::SseqRecord::default())
    });
    // streamApp.dbd: asyn device support, DTYP "stream" and the stream
    // iocsh commands; Teledyne D/H run on SPApp/Db's teled_d/teled_h.proto.
    app = stream_rs::dev::register_app(app);

    // Standard asyn iocsh commands -- provides drvAsynSerialPortConfigure /
    // drvAsynIPPortConfigure / asynOctetSetInputEos / asynOctetSetOutputEos /
    // asynSetTraceMask / asynSetTraceIOMask / asynSetTraceIOTruncateSize /
    // asynSetTraceFile / asynSetOption.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // Modbus iocsh commands: modbusInterposeConfig, drvModbusAsynConfigure
    // (ISCO/Vindum wiring goes entirely through these -- see this file's
    // module doc for the scope-split rationale).
    let runtime_handle = epics_rs::base::runtime::task::runtime_handle();
    app = modbus_rs::ioc::register_modbus_commands(app, runtime_handle);

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
