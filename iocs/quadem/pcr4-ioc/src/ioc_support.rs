//! iocsh commands for the PCR4 IOC.
//!
//! `drvPCR4Configure` lives in `quadem::iocsh`.

use std::sync::{Arc, Mutex};

use epics_rs::ad_plugins::ioc::AdIoc;

use quadem::Pcr4Runtime;
use quadem::iocsh::pcr4_configure_command;

/// Register the PCR4 configure command.
pub fn register(ioc: &mut AdIoc) {
    epics_rs::base::runtime::env::set_default("QUADEM", concat!(env!("CARGO_MANIFEST_DIR"), "/.."));

    let runtime: Arc<Mutex<Option<Pcr4Runtime>>> = Arc::new(Mutex::new(None));
    let cmd = pcr4_configure_command(ioc.mgr().clone(), runtime.clone());
    ioc.register_startup_command(cmd);

    // Keep the runtime (read thread, callback thread, port actor) alive.
    ioc.keep_alive(runtime);
}
