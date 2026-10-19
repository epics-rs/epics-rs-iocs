//! Yokogawa MW100 data-acquisition-unit IOC binary (`drvMW100.c`/`devMW100_*.c`).
//!
//! MW100's wire protocol is a proprietary TCP ASCII/binary framing (port
//! 34318), not Modbus — so this IOC wires a dynamic `DeviceSupport` factory
//! (`yokogawa_mw100::device_support::factory`) instead of an asyn port. The
//! `mw100Init` startup command mirrors the C `mw100Init(netDevice, address)`
//! iocsh command: it registers `netDevice` in the shared `Registry` and
//! connects it; every record's `@netDevice CMD:ADDRESS` link resolves
//! against that registry at `iocInit`, and `dbior` reports it as
//! `drvMW100`.
//!
//! Usage:
//!   cargo run -p yokogawa-mw100-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::base::server::driver_support::register_driver_support;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use yokogawa_mw100::device_support;
use yokogawa_mw100::instrument::Registry;

/// `yokogawaDasApp/src/mw100Support.dbd`'s `device()` lines.
static DEVICES: &[(&str, &[DevSup])] = &[
    (
        "ai",
        &[row("devMW100_ai", device_support::DTYP, 6, &[2, 3, 4])],
    ),
    (
        "ao",
        &[row("devMW100_ao", device_support::DTYP, 6, &[2, 4])],
    ),
    (
        "bi",
        &[row("devMW100_bi", device_support::DTYP, 5, &[2, 3, 4])],
    ),
    (
        "bo",
        &[row("devMW100_bo", device_support::DTYP, 5, &[2, 4])],
    ),
    (
        "mbbi",
        &[row("devMW100_mbbi", device_support::DTYP, 5, &[2, 3, 4])],
    ),
    (
        "mbbo",
        &[row("devMW100_mbbo", device_support::DTYP, 5, &[2, 4])],
    ),
    (
        "longin",
        &[row("devMW100_longin", device_support::DTYP, 5, &[2, 3, 4])],
    ),
    (
        "stringin",
        &[row(
            "devMW100_stringin",
            device_support::DTYP,
            6,
            &[2, 3, 4],
        )],
    ),
];

const fn row(name: &'static str, choice: &'static str, number: u8, funcs: &'static [u8]) -> DevSup {
    DevSup {
        name,
        choice,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(number, funcs, false).with_init(device_support::dev_init),
    }
}

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
    register_driver_support("drvMW100", registry.clone());
    let init_registry = registry.clone();

    epics_rs::base::runtime::env::set_default("MW100", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();
    app = app.register_dynamic_device_support(DEVICES, device_support::factory(registry.clone()));

    app = app.register_startup_command(CommandDef::new(
        "mw100Init",
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
        "mw100Init netDevice address",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            // A NULL `sval` for an absent argument; C ignores the result.
            let arg = |v: &ArgValue| match v {
                ArgValue::String(s) => Some(s.as_bytes().to_vec()),
                _ => None,
            };
            init_registry.init_mw100(arg(&args[0]).as_deref(), arg(&args[1]).as_deref());
            Ok(CommandOutcome::Continue)
        },
    ));

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every devMW100_* dset has `init` (devMW100_*.c), so dbDumpDevice shows
    /// slot 1 and the dset init passes reach it.
    #[test]
    fn every_dset_runs_dev_init() {
        let dev_init: fn(bool) = device_support::dev_init;
        for (record_type, rows) in DEVICES {
            for row in *rows {
                assert_eq!(
                    row.dset.init.map(|f| f as usize),
                    Some(dev_init as usize),
                    "{record_type} {}",
                    row.name
                );
            }
        }
    }
}
