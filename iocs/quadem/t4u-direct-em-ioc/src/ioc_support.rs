//! iocsh commands for the T4UDirect_EM IOC.
//!
//! `drvT4UDirect_EMConfigure` lives in `quadem::iocsh`. The driver builds its
//! own asyn IP ports (a TCP command port to the meter's telnet socket and a UDP
//! data port it binds locally), so unlike most quadEM IOCs no
//! `drvAsynIPPortConfigure` runs first.

use std::sync::{Arc, Mutex};

use epics_rs::ad_plugins::ioc::AdIoc;

use quadem::T4uRuntime;
use quadem::iocsh::t4u_direct_em_configure_command;

/// Register the T4UDirect_EM configure command.
pub fn register(ioc: &mut AdIoc) {
    epics_rs::base::runtime::env::set_default("QUADEM", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<Mutex<Option<T4uRuntime>>> = Arc::new(Mutex::new(None));
    let cmd =
        t4u_direct_em_configure_command(ioc.mgr().clone(), ioc.trace().clone(), runtime.clone());
    ioc.register_startup_command(cmd);

    // Keep the runtime (command thread, data thread, callback thread, port
    // actor) alive.
    ioc.keep_alive(runtime);
}
