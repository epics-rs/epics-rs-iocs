//! Yokogawa GM10 data-acquisition-unit IOC binary (`drvGM10.c`/`devGM10_*.c`).
//!
//! GM10's wire protocol is a proprietary TCP ASCII/binary framing (port
//! 34434), not Modbus — so this IOC wires a dynamic `DeviceSupport` factory
//! (`yokogawa_gm10::device_support::factory`) instead of an asyn port. The
//! `gm10Init` startup command is the C `gm10Init(netDevice, address)` iocsh
//! command: it registers an `Instrument` under `netDevice` in a shared
//! name-keyed `Registry` that every record's `@netDevice CMD:ADDRESS` link
//! resolves against at `iocInit`, then connects.
//!
//! Usage:
//!   cargo run -p yokogawa-gm10-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use yokogawa_gm10::device_support;
use yokogawa_gm10::instrument::{Registry, gm10_init};

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
    epics_rs::base::server::driver_support::register_driver_support("drvGM10", registry.clone());

    epics_rs::base::runtime::env::set_default("GM10", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();
    app = app.register_dynamic_device_support(device_support::factory(registry.clone()));

    app = app.register_startup_command(CommandDef::new(
        "gm10Init",
        vec![
            ArgDesc {
                name: "netDevice",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "address",
                arg_type: ArgType::String,
            },
        ],
        "gm10Init netDevice address",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let arg = |v: &ArgValue| match v {
                ArgValue::String(s) => Some(s.as_bytes().to_vec()),
                _ => None,
            };
            let (net_device, address) = (arg(&args[0]), arg(&args[1]));
            gm10_init(&registry, net_device.as_deref(), address.as_deref());
            Ok(CommandOutcome::Continue)
        },
    ));

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
