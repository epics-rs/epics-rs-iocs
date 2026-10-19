//! EtherIP IOC binary -- Allen-Bradley ControlLogix / PLC-5 over EtherNet/IP.
//!
//! Usage:
//!   cargo run -p ether-ip-ioc -- iocs/ether-ip/ether-ip-ioc/st.cmd

use std::sync::atomic::Ordering;

use epics_rs::base::error::CaResult;
use epics_rs::base::server::ioc_app::init_hook_register;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::ca::server::ioc_app::IocApplication;

use ether_ip::device;
use ether_ip::driver;

/// `ether_ipApp/src/ether_ip-{3-14,lint,lsio}.dbd`'s `device()` lines.
static DEVICES: &[(&str, &[DevSup])] = &[
    ("ai", &[row("devAiEtherIP", device::DTYP, 6, &[2, 3, 4])]),
    ("bi", &[row("devBiEtherIP", device::DTYP, 5, &[2, 3, 4])]),
    (
        "int64in",
        &[row("devInt64inEtherIP", device::DTYP, 5, &[2, 3, 4])],
    ),
    ("lsi", &[row("devLsiEtherIP", device::DTYP, 5, &[2, 3, 4])]),
    (
        "mbbi",
        &[row("devMbbiEtherIP", device::DTYP, 5, &[2, 3, 4])],
    ),
    (
        "mbbiDirect",
        &[row("devMbbiDirectEtherIP", device::DTYP, 5, &[2, 3, 4])],
    ),
    (
        "stringin",
        &[row("devSiEtherIP", device::DTYP, 5, &[2, 3, 4])],
    ),
    (
        "waveform",
        &[row("devWfEtherIP", device::DTYP, 5, &[2, 3, 4])],
    ),
    ("ao", &[row("devAoEtherIP", device::DTYP, 6, &[2, 4])]),
    (
        "bo",
        &[
            row("devBoEtherIP", device::DTYP, 6, &[2, 4]),
            DevSup {
                name: "devBoEtherIPReset",
                choice: device::DTYP_RESET,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[4], false),
            },
        ],
    ),
    (
        "int64out",
        &[row("devInt64outEtherIP", device::DTYP, 5, &[2, 4])],
    ),
    ("lso", &[row("devLsoEtherIP", device::DTYP, 5, &[2, 4])]),
    ("mbbo", &[row("devMbboEtherIP", device::DTYP, 5, &[2, 4])]),
    (
        "mbboDirect",
        &[row("devMbboDirectEtherIP", device::DTYP, 5, &[2, 4])],
    ),
    (
        "stringout",
        &[row("devSoEtherIP", device::DTYP, 5, &[2, 4])],
    ),
];

/// A devEtherIP row whose `init` is the shared `init(run)`, which calls
/// `devExtend` at run 0 (`devEtherIP.c:1227-1230` and its twins).
const fn row(name: &'static str, choice: &'static str, number: u8, funcs: &'static [u8]) -> DevSup {
    DevSup {
        name,
        choice,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(number, funcs, true).with_init(driver::dev_init),
    }
}

fn int_arg(args: &[ArgValue], i: usize) -> Option<i64> {
    match args.get(i) {
        Some(ArgValue::Int(n)) => Some(*n),
        _ => None,
    }
}

fn string_arg(args: &[ArgValue], i: usize) -> Result<Option<String>, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(Some(s.to_str()?.to_owned())),
        _ => Ok(None),
    }
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: ether-ip-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("ETHERIP", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // One dynamic factory covers every record type: the DTYP is always
    // "EtherIP" (or "EtherIPReset"), and the tag comes from the INP/OUT text.
    app = app.register_dynamic_device_support(DEVICES, device::device_factory);

    // drvEtherIPRegister.cpp: every command, in its registration order.
    let int = |name| ArgDesc {
        name,
        arg_type: ArgType::Int,
    };
    let string = |name| ArgDesc {
        name,
        arg_type: ArgType::String,
    };
    let commands = vec![
        CommandDef::new(
            "drvEtherIP_default_rate",
            vec![ArgDesc {
                name: "value",
                arg_type: ArgType::Double,
            }],
            "drvEtherIP_default_rate value",
            |args: &[ArgValue], _ctx: &CommandContext| {
                driver::set_default_rate(match args.first() {
                    Some(ArgValue::Double(v)) => *v,
                    _ => 0.0,
                });
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "EIP_verbosity",
            vec![int("value")],
            "EIP_verbosity value",
            |args: &[ArgValue], _ctx: &CommandContext| {
                let value = int_arg(args, 0).unwrap_or(0) as i32;
                ether_ip::EIP_VERBOSITY.store(value, Ordering::Relaxed);
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "EIP_timeout",
            vec![int("value")],
            "EIP_timeout value",
            |args: &[ArgValue], _ctx: &CommandContext| {
                let ms = int_arg(args, 0).unwrap_or(0);
                driver::TIMEOUT_MS.store(ms as u32, Ordering::Relaxed);
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "EIP_buffer_limit",
            vec![int("bytes")],
            "EIP_buffer_limit bytes",
            |args: &[ArgValue], _ctx: &CommandContext| {
                print!(
                    "Changing buffer limit from {} ",
                    driver::BUFFER_LIMIT.load(Ordering::Relaxed)
                );
                let bytes = int_arg(args, 0).unwrap_or(0) as u32;
                driver::BUFFER_LIMIT.store(bytes, Ordering::Relaxed);
                println!("to {bytes} bytes");
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_help",
            vec![],
            "drvEtherIP_help",
            |_args: &[ArgValue], _ctx: &CommandContext| {
                driver::help();
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_init",
            vec![],
            "drvEtherIP_init",
            |_args: &[ArgValue], _ctx: &CommandContext| {
                driver::init();
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_restart",
            vec![],
            "drvEtherIP_restart",
            |_args: &[ArgValue], _ctx: &CommandContext| {
                driver::restart();
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_dump",
            vec![],
            "drvEtherIP_dump",
            |_args: &[ArgValue], _ctx: &CommandContext| {
                driver::dump();
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_list",
            vec![],
            "drvEtherIP_list",
            |_args: &[ArgValue], _ctx: &CommandContext| {
                driver::list();
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_describe",
            vec![int("type ID")],
            "drvEtherIP_describe 'type ID'",
            |args: &[ArgValue], _ctx: &CommandContext| {
                driver::describe(int_arg(args, 0).unwrap_or(0) as u16);
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_reset_statistics",
            vec![],
            "drvEtherIP_reset_statistics",
            |_args: &[ArgValue], _ctx: &CommandContext| {
                driver::reset_statistics();
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_report",
            vec![int("value")],
            "drvEtherIP_report value",
            |args: &[ArgValue], _ctx: &CommandContext| {
                driver::report(int_arg(args, 0).unwrap_or(0) as i32);
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_define_PLC",
            vec![string("plc_name"), string("ip_addr"), int("slot")],
            "drvEtherIP_define_PLC plc_name ip_addr slot",
            |args: &[ArgValue], _ctx: &CommandContext| {
                let name = string_arg(args, 0)?.ok_or("plc_name required")?;
                let ip = string_arg(args, 1)?.ok_or("ip_addr required")?;
                // C keeps an int and sends it as a CN_USINT.
                let slot = int_arg(args, 2).unwrap_or(0) as u8;
                driver::define_plc(&name, &ip, slot);
                Ok(CommandOutcome::Continue)
            },
        ),
        CommandDef::new(
            "drvEtherIP_read_tag",
            vec![
                string("ip_addr"),
                int("slot"),
                string("tag_name"),
                int("elements"),
                int("timeout"),
            ],
            "drvEtherIP_read_tag ip_addr slot tag_name elements timeout",
            |args: &[ArgValue], _ctx: &CommandContext| {
                let ip = string_arg(args, 0)?.unwrap_or_default();
                let tag = string_arg(args, 2)?.unwrap_or_default();
                driver::read_tag(
                    &ip,
                    int_arg(args, 1).unwrap_or(0) as u8,
                    &tag,
                    int_arg(args, 3).unwrap_or(0) as u16,
                    int_arg(args, 4).unwrap_or(0) as u64,
                );
                Ok(CommandOutcome::Continue)
            },
        ),
    ];
    for command in commands {
        app = app.register_startup_command(command.clone());
        app = app.register_shell_command(command);
    }

    // drvEtherIP_drvInit's initHookRegister.
    init_hook_register(std::sync::Arc::new(driver::init_hook));

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every devEtherIP dset but devBoEtherIPReset has `init` (devEtherIP.c),
    /// so `finishDevSup` starts the driver through any of them.
    #[test]
    fn every_dset_but_reset_runs_dev_init() {
        let dev_init: fn(bool) = driver::dev_init;
        for (record_type, rows) in DEVICES {
            for row in *rows {
                assert_eq!(
                    row.dset.init.map(|f| f as usize),
                    (row.name != "devBoEtherIPReset").then_some(dev_init as usize),
                    "{record_type} {}",
                    row.name
                );
            }
        }
    }

    /// Every devEtherIP dset but devBoEtherIPReset calls `devExtend` from its
    /// `init(0)`, so a runtime INP/OUT put passes `dbPutFieldLink`'s gate.
    #[test]
    fn every_dset_but_reset_extends() {
        for (record_type, rows) in DEVICES {
            for row in *rows {
                assert_eq!(
                    row.has_dsxt(),
                    row.name != "devBoEtherIPReset",
                    "{record_type} {}",
                    row.name
                );
            }
        }
    }
}
