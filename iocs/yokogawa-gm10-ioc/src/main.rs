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

use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use yokogawa_gm10::device_support;
use yokogawa_gm10::instrument::{Registry, gm10_init};

/// `yokogawaDasApp/src/gm10Support.dbd`'s `device()` lines.
static DEVICES: &[(&str, &[DevSup])] = &[
    (
        "ai",
        &[row("devGM10_ai", device_support::DTYP, 6, &[1, 2, 3, 4])],
    ),
    (
        "ao",
        &[row("devGM10_ao", device_support::DTYP, 6, &[1, 2, 4])],
    ),
    (
        "bi",
        &[row("devGM10_bi", device_support::DTYP, 5, &[1, 2, 3, 4])],
    ),
    (
        "bo",
        &[row("devGM10_bo", device_support::DTYP, 5, &[1, 2, 4])],
    ),
    (
        "mbbi",
        &[row("devGM10_mbbi", device_support::DTYP, 5, &[1, 2, 3, 4])],
    ),
    (
        "mbbo",
        &[row("devGM10_mbbo", device_support::DTYP, 5, &[1, 2, 4])],
    ),
    (
        "longin",
        &[row(
            "devGM10_longin",
            device_support::DTYP,
            5,
            &[1, 2, 3, 4],
        )],
    ),
    (
        "stringin",
        &[row(
            "devGM10_stringin",
            device_support::DTYP,
            6,
            &[1, 2, 3, 4],
        )],
    ),
];

const fn row(name: &'static str, choice: &'static str, number: u8, funcs: &'static [u8]) -> DevSup {
    DevSup {
        name,
        choice,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(number, funcs, false),
    }
}

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
    app = app.register_dynamic_device_support(DEVICES, device_support::factory(registry.clone()));

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
