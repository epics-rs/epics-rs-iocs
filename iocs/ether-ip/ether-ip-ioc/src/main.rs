//! EtherIP IOC binary -- Allen-Bradley ControlLogix / PLC-5 over EtherNet/IP.
//!
//! Usage:
//!   cargo run -p ether-ip-ioc -- iocs/ether-ip/ether-ip-ioc/st.cmd

use std::sync::atomic::Ordering;

use epics_rs::base::error::CaResult;
use epics_rs::base::server::ioc_app::init_hook_register;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use ether_ip::device;
use ether_ip::driver;

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
    app = app.register_dynamic_device_support(device::device_factory);

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
