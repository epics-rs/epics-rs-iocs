//! iocsh commands for the FX4 IOC.
//!
//! `drvFX4Configure` lives in `quadem::iocsh`. The driver opens its own
//! WebSocket to the meter, so unlike most quadEM IOCs no
//! `drvAsynIPPortConfigure` runs first.

use std::sync::{Arc, Mutex};

use epics_rs::ad_plugins::ioc::AdIoc;

use quadem::Fx4Runtime;
use quadem::iocsh::fx4_configure_command;

/// Register the FX4 configure command.
pub fn register(ioc: &mut AdIoc) {
    epics_rs::base::runtime::env::set_default("QUADEM", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<Mutex<Option<Fx4Runtime>>> = Arc::new(Mutex::new(None));
    let cmd = fx4_configure_command(ioc.mgr().clone(), ioc.trace().clone(), runtime.clone());
    ioc.register_startup_command(cmd);

    // Keep the runtime (socket thread, data thread, callback thread, port
    // actor) alive.
    ioc.keep_alive(runtime);
}
