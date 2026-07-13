//! TwinCAT ADS IOC.
//!
//! Usage:
//!   cargo run -p twincat-ads-ioc -- iocs/twincat-ads-ioc/st.cmd
//!
//! The iocsh commands are the C driver's:
//!
//! * `adsSetLocalAddress("192.168.88.44.1.1")` — the AMS Net Id this IOC answers
//!   to. Optional: without it the driver derives `<own ip>.1.1`, as Beckhoff's
//!   router does.
//! * `adsAsynPortDriverConfigure(port, ip, amsNetId, amsPort, paramTableSize,
//!   priority, disableAutoConnect, sampleTimeMS, maxDelayTimeMS, timeoutMS,
//!   timeSource)`
//! * `adsPollInfo(name)` — what the bulk reader is polling.

use std::str::FromStr;
use std::sync::{Arc, Mutex};

use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use twincat_ads::{AdsConfig, AdsRuntime, AmsNetId, TimeBase, create_ads_port};

fn arg_string(args: &[ArgValue], i: usize) -> Option<String> {
    match args.get(i) {
        Some(ArgValue::String(s)) if !s.is_empty() => Some(s.clone()),
        _ => None,
    }
}

fn arg_int(args: &[ArgValue], i: usize) -> Option<i64> {
    match args.get(i) {
        Some(ArgValue::Int(n)) => Some(*n),
        _ => None,
    }
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: twincat-ads-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("TWINCAT_ADS", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());
    // One IOC can serve several PLCs; each port's runtime is kept alive here.
    let runtimes: Arc<Mutex<Vec<AdsRuntime>>> = Arc::new(Mutex::new(Vec::new()));

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    let autosave_config = Arc::new(Mutex::new(
        epics_rs::base::server::autosave::startup::AutosaveStartupConfig::new(),
    ));
    app = app.autosave_startup(autosave_config);

    // adsSetLocalAddress
    {
        app = app.register_startup_command(CommandDef::new(
            "adsSetLocalAddress",
            vec![ArgDesc {
                name: "localAmsNetId",
                arg_type: ArgType::String,
                optional: false,
            }],
            "adsSetLocalAddress localAmsNetId",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let text = arg_string(args, 0).ok_or("localAmsNetId required")?;
                let net_id =
                    AmsNetId::from_str(&text).map_err(|e| format!("localAmsNetId: {e}"))?;
                twincat_ads::set_local_address(net_id);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // adsAsynPortDriverConfigure
    {
        let trace_c = trace.clone();
        let rts = runtimes.clone();
        app = app.register_startup_command(CommandDef::new(
            "adsAsynPortDriverConfigure",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "ipAddr",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "amsNetId",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "amsPort",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "paramTableSize",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "priority",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "disableAutoConnect",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "defaultSampleTimeMS",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "maxDelayTimeMS",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "adsTimeoutMS",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "defaultTimeSource",
                    arg_type: ArgType::Int,
                    optional: true,
                },
            ],
            "adsAsynPortDriverConfigure portName ipAddr amsNetId [amsPort] \
             [paramTableSize] [priority] [disableAutoConnect] [defaultSampleTimeMS] \
             [maxDelayTimeMS] [adsTimeoutMS] [defaultTimeSource]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = arg_string(args, 0).ok_or("portName required")?;
                let ip_addr = arg_string(args, 1).ok_or("ipAddr required")?;
                let ams_text = arg_string(args, 2).ok_or("amsNetId required")?;
                let remote_net_id =
                    AmsNetId::from_str(&ams_text).map_err(|e| format!("amsNetId: {e}"))?;
                let ams_port = arg_int(args, 3).unwrap_or(851) as u16;

                let mut cfg = AdsConfig::new(&port_name, &ip_addr, remote_net_id, ams_port);
                // paramTableSize (4) and priority (5) are C implementation
                // limits — the parameter table grows on demand here and the
                // port actor sets its own thread priority — so they are
                // accepted for st.cmd compatibility and ignored.
                if let Some(disable) = arg_int(args, 6) {
                    cfg.auto_connect = disable == 0;
                }
                if let Some(ms) = arg_int(args, 7) {
                    cfg.default_sample_time_ms = ms as f64;
                }
                if let Some(ms) = arg_int(args, 8) {
                    cfg.default_max_delay_time_ms = ms as f64;
                }
                if let Some(ms) = arg_int(args, 9) {
                    cfg.ads_timeout_ms = ms as u64;
                }
                if let Some(src) = arg_int(args, 10) {
                    // C `ADS_TIME_BASE_PLC = 0`, `ADS_TIME_BASE_EPICS = 1`
                    // (adsAsynPortDriverUtils.h:59-60).
                    cfg.default_time_base = if src == 0 {
                        TimeBase::Plc
                    } else {
                        TimeBase::Epics
                    };
                }

                let runtime = create_ads_port(cfg).map_err(|e| e.to_string())?;
                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    runtime.port_handle().clone(),
                    trace_c.clone(),
                )
                .map_err(|e| e.to_string())?;
                rts.lock().unwrap().push(runtime);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // adsPollInfo
    {
        app = app.register_startup_command(CommandDef::new(
            "adsPollInfo",
            vec![ArgDesc {
                name: "name",
                arg_type: ArgType::String,
                optional: true,
            }],
            "adsPollInfo [name]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                twincat_ads::poll_info(&arg_string(args, 0).unwrap_or_default());
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
