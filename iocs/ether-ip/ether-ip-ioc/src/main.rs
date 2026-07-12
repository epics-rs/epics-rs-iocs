//! EtherIP IOC binary -- Allen-Bradley ControlLogix / PLC-5 over EtherNet/IP.
//!
//! Usage:
//!   cargo run -p ether-ip-ioc -- iocs/ether-ip/ether-ip-ioc/st.cmd

use std::sync::atomic::Ordering;

use epics_rs::base::error::CaResult;
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

fn string_arg(args: &[ArgValue], i: usize) -> Option<String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Some(s.clone()),
        _ => None,
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

    let mut app = IocApplication::new();

    // One dynamic factory covers every record type: the DTYP is always
    // "EtherIP" (or "EtherIPReset"), and the tag comes from the INP/OUT text.
    app = app.register_dynamic_device_support(device::device_factory);

    // drvEtherIP_define_PLC(name, ip_addr, slot)
    app = app.register_startup_command(CommandDef::new(
        "drvEtherIP_define_PLC",
        vec![
            ArgDesc {
                name: "name",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "ip_addr",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "slot",
                arg_type: ArgType::Int,
                optional: false,
            },
        ],
        "drvEtherIP_define_PLC name ip_addr slot",
        |args: &[ArgValue], _ctx: &CommandContext| {
            let name = string_arg(args, 0).ok_or("name required")?;
            let ip = string_arg(args, 1).ok_or("ip_addr required")?;
            let slot = int_arg(args, 2).ok_or("slot required")?;
            if !(0..=255).contains(&slot) {
                return Err(format!("slot {slot} out of range 0..255"));
            }
            driver::define_plc(&name, &ip, slot as u8);
            Ok(CommandOutcome::Continue)
        },
    ));

    // EIP_timeout(<milliseconds>)
    app = app.register_startup_command(CommandDef::new(
        "EIP_timeout",
        vec![ArgDesc {
            name: "milliseconds",
            arg_type: ArgType::Int,
            optional: false,
        }],
        "EIP_timeout milliseconds",
        |args: &[ArgValue], _ctx: &CommandContext| {
            let ms = int_arg(args, 0).ok_or("milliseconds required")?;
            if ms <= 0 {
                return Err("timeout must be positive".into());
            }
            driver::TIMEOUT_MS.store(ms as u32, Ordering::Relaxed);
            Ok(CommandOutcome::Continue)
        },
    ));

    // EIP_buffer_limit(<bytes>) -- only meaningful before the scan tasks start.
    app = app.register_startup_command(CommandDef::new(
        "EIP_buffer_limit",
        vec![ArgDesc {
            name: "bytes",
            arg_type: ArgType::Int,
            optional: false,
        }],
        "EIP_buffer_limit bytes",
        |args: &[ArgValue], _ctx: &CommandContext| {
            let bytes = int_arg(args, 0).ok_or("bytes required")?;
            if bytes <= 0 || bytes as usize > ether_ip::encap::BUFFER_SIZE {
                return Err(format!(
                    "buffer limit must be in 1..={}",
                    ether_ip::encap::BUFFER_SIZE
                ));
            }
            driver::BUFFER_LIMIT.store(bytes as u32, Ordering::Relaxed);
            Ok(CommandOutcome::Continue)
        },
    ));

    // drvEtherIP_default_rate(<seconds>) -- the scan period for records whose
    // SCAN gives none and whose link carries no `S <period>` flag.
    app = app.register_startup_command(CommandDef::new(
        "drvEtherIP_default_rate",
        vec![ArgDesc {
            name: "seconds",
            arg_type: ArgType::Double,
            optional: false,
        }],
        "drvEtherIP_default_rate seconds",
        |args: &[ArgValue], _ctx: &CommandContext| {
            let secs = match args.first() {
                Some(ArgValue::Double(v)) => *v,
                Some(ArgValue::Int(v)) => *v as f64,
                _ => return Err("seconds required".into()),
            };
            if secs <= 0.0 {
                return Err("rate must be positive".into());
            }
            driver::DEFAULT_RATE_MS.store((secs * 1000.0) as u32, Ordering::Relaxed);
            Ok(CommandOutcome::Continue)
        },
    ));

    // drvEtherIP_report(<level>)
    let report = CommandDef::new(
        "drvEtherIP_report",
        vec![ArgDesc {
            name: "level",
            arg_type: ArgType::Int,
            optional: true,
        }],
        "drvEtherIP_report [level]",
        |args: &[ArgValue], _ctx: &CommandContext| {
            driver::report(int_arg(args, 0).unwrap_or(0) as i32);
            Ok(CommandOutcome::Continue)
        },
    );
    app = app.register_startup_command(report.clone());
    app = app.register_shell_command(report);

    // The scan tasks must not start until every record has registered its tag,
    // so they start after iocInit -- the C does the same from its device-support
    // `init(run==1)` hook, which calls drvEtherIP_restart().
    app = app.register_after_init(|| {
        let n = driver::start_scan_tasks();
        log::info!("EIP: started {n} scan task(s)");
    });

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
