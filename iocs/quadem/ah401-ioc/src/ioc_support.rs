//! iocsh commands for the AH401 IOC.
//!
//! `drvAHxxxConfigure` and the octet-port verbs live in `quadem::iocsh` — the
//! AH401 and AH501 IOCs drive the same upstream driver and register the same
//! commands.

use std::sync::{Arc, Mutex};

use epics_rs::ad_plugins::ioc::AdIoc;

use quadem::AhxxxRuntime;
use quadem::iocsh::{ahxxx_configure_command, octet_port_commands};

/// Register the AHxxx configure command and the octet-port verbs it needs.
pub fn register(ioc: &mut AdIoc) {
    epics_rs::base::runtime::env::set_default("QUADEM", env!("CARGO_MANIFEST_DIR"));

    for cmd in octet_port_commands(ioc.trace().clone()) {
        ioc.register_startup_command(cmd);
    }

    let runtime: Arc<Mutex<Option<AhxxxRuntime>>> = Arc::new(Mutex::new(None));
    let cmd = ahxxx_configure_command(ioc.mgr().clone(), ioc.trace().clone(), runtime.clone());
    ioc.register_startup_command(cmd);

    // Keep the runtime (read thread, callback thread, port actor) alive.
    ioc.keep_alive(runtime);
}
