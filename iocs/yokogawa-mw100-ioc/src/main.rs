//! Yokogawa MW100 data-acquisition-unit IOC binary (`drvMW100.c`/`devMW100_*.c`).
//!
//! MW100's wire protocol is a proprietary TCP ASCII/binary framing (port
//! 34318), not Modbus — so this IOC wires a dynamic `DeviceSupport` factory
//! (`yokogawa_mw100::device_support::factory`) instead of an asyn port. The
//! `mw100Init` startup command mirrors the C `mw100Init(netDevice, address)`
//! iocsh command: it connects, then registers the resulting `Instrument`
//! under `netDevice` in a shared name-keyed `Registry` that every record's
//! `@netDevice CMD:ADDRESS` link resolves against at `iocInit`.
//!
//! Usage:
//!   cargo run -p yokogawa-mw100-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use yokogawa_mw100::device_support;
use yokogawa_mw100::instrument::{Instrument, Registry};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: yokogawa-mw100-ioc <st.cmd>");
        std::process::exit(1);
    };

    let registry = Arc::new(Registry::new());

    let mut app = IocApplication::new();
    app = app.register_dynamic_device_support(device_support::factory(registry.clone()));

    app = app.register_startup_command(CommandDef::new(
        "mw100Init",
        vec![
            ArgDesc {
                name: "netDevice",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "address",
                arg_type: ArgType::String,
                optional: false,
            },
        ],
        "mw100Init netDevice address",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let net_device = match &args[0] {
                ArgValue::String(s) => s.clone(),
                _ => return Err("netDevice required".into()),
            };
            let address = match &args[1] {
                ArgValue::String(s) => s.clone(),
                _ => return Err("address required".into()),
            };

            let instrument = Instrument::connect(&address)
                .map_err(|e| format!("mw100Init: connect to {address} failed: {e}"))?;
            registry
                .insert(net_device.clone(), instrument)
                .map_err(|e| format!("mw100Init: {net_device}: {e}"))?;
            Ok(CommandOutcome::Continue)
        },
    ));

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
