//! iocsh commands for the AH501 IOC.
//!
//! `drvAHxxxConfigure` lives in `quadem::iocsh` — the
//! AH401 and AH501 IOCs drive the same upstream driver and register the same
//! command.

use std::sync::{Arc, Mutex};

use epics_rs::ad_plugins::ioc::AdIoc;

use quadem::AhxxxRuntime;
use quadem::iocsh::ahxxx_configure_command;

/// Register the AHxxx configure command.
pub fn register(ioc: &mut AdIoc) {
    epics_rs::base::runtime::env::set_default_path(
        "QUADEM",
        concat!(env!("CARGO_MANIFEST_DIR"), "/.."),
    );

    let runtime: Arc<Mutex<Option<AhxxxRuntime>>> = Arc::new(Mutex::new(None));
    let cmd = ahxxx_configure_command(ioc.mgr().clone(), runtime.clone());
    ioc.register_startup_command(cmd);

    // Keep the runtime (read thread, callback thread, port actor) alive.
    ioc.keep_alive(runtime);
}
