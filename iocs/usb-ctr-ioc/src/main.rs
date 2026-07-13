//! USB-CTR08 IOC binary.
//!
//! Usage:
//!   cargo run -p usb-ctr-ioc -- iocs/usb-ctr-ioc/st.cmd

use std::sync::{Arc, Mutex};

use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use usb_ctr::{CtrRuntime, create_usb_ctr};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: usb-ctr-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("MEASCOMP", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());

    // Runtime kept alive by being captured in the startup command closure
    let runtime: Arc<Mutex<Option<CtrRuntime>>> = Arc::new(Mutex::new(None));

    let mut app = IocApplication::new();

    // Register record types
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());

    // Universal asyn device support
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Autosave
    let autosave_config = Arc::new(Mutex::new(
        epics_rs::base::server::autosave::startup::AutosaveStartupConfig::new(),
    ));
    app = app.autosave_startup(autosave_config);

    // USBCTRConfig command
    {
        let trace_c = trace.clone();
        let rt = runtime.clone();
        app = app.register_startup_command(CommandDef::new(
            "USBCTRConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "uniqueID",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "maxTimePoints",
                    arg_type: ArgType::Int,
                    optional: true,
                },
            ],
            "USBCTRConfig portName uniqueID [maxTimePoints]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let unique_id = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("uniqueID required".into()),
                };
                let max_points = match args.get(2) {
                    Some(ArgValue::Int(n)) => *n as usize,
                    _ => 2048,
                };

                let ctr_rt = create_usb_ctr(&port_name, &unique_id, max_points)?;

                let port_handle = ctr_rt.port_handle().clone();
                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    port_handle,
                    trace_c.clone(),
                )
                .map_err(|e| e.to_string())?;

                *rt.lock().unwrap() = Some(ctr_rt);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
