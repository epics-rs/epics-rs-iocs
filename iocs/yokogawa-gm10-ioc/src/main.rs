//! Yokogawa GM10 data-acquisition-unit IOC binary (`drvGM10.c`/`devGM10_*.c`).
//!
//! GM10's wire protocol is a proprietary TCP ASCII/binary framing (port
//! 34434), not Modbus — so this IOC wires a dynamic `DeviceSupport` factory
//! (`yokogawa_gm10::device_support::factory`) instead of an asyn port. The
//! `gm10Init` startup command mirrors the C `gm10Init(netDevice, address)`
//! iocsh command: it connects, then registers the resulting `Instrument`
//! under `netDevice` in a shared name-keyed `Registry` that every record's
//! `@netDevice CMD:ADDRESS` link resolves against at `iocInit`.
//!
//! Usage:
//!   cargo run -p yokogawa-gm10-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use yokogawa_gm10::device_support;
use yokogawa_gm10::instrument::{Instrument, Registry};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: yokogawa-gm10-ioc <st.cmd>");
        std::process::exit(1);
    };

    let registry = Arc::new(Registry::new());

    let mut app = IocApplication::new();
    app = app.register_dynamic_device_support(device_support::factory(registry.clone()));

    app = app.register_startup_command(CommandDef::new(
        "gm10Init",
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
        "gm10Init netDevice address",
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
                .map_err(|e| format!("gm10Init: connect to {address} failed: {e}"))?;
            registry
                .insert(net_device.clone(), instrument)
                .map_err(|e| format!("gm10Init: {net_device}: {e}"))?;
            Ok(CommandOutcome::Continue)
        },
    ));

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
