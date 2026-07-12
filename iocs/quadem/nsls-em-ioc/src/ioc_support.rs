//! iocsh commands for the NSLS_EM IOC.
//!
//! `drvNSLS_EMConfigure` lives in `quadem::iocsh`. The driver discovers the
//! module by UDP broadcast and builds its own three asyn IP ports, so unlike
//! the other quadEM IOCs no `drvAsynIPPortConfigure` runs first.

use std::sync::{Arc, Mutex};

use epics_rs::ad_plugins::ioc::AdIoc;

use quadem::NslsEmRuntime;
use quadem::iocsh::nsls_em_configure_command;

/// Register the NSLS_EM configure command.
pub fn register(ioc: &mut AdIoc) {
    epics_rs::base::runtime::env::set_default("QUADEM", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<Mutex<Option<NslsEmRuntime>>> = Arc::new(Mutex::new(None));
    let cmd = nsls_em_configure_command(ioc.mgr().clone(), ioc.trace().clone(), runtime.clone());
    ioc.register_startup_command(cmd);

    // Keep the runtime (read thread, callback thread, port actor) alive.
    ioc.keep_alive(runtime);
}
