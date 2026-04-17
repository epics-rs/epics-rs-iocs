//! USB-2408-2AO IOC binary.
//!
//! Usage:
//!   cargo run -p usb-2408-ioc -- iocs/usb-2408-ioc/st.cmd

use std::sync::{Arc, Mutex};

use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use usb_2408::{MultiFunctionRuntime, create_usb_2408};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: usb-2408-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("MEASCOMP", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());
    let runtime: Arc<Mutex<Option<MultiFunctionRuntime>>> = Arc::new(Mutex::new(None));

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    let autosave_config = Arc::new(Mutex::new(
        epics_rs::base::server::autosave::startup::AutosaveStartupConfig::new(),
    ));
    app = app.autosave_startup(autosave_config);

    // MultiFunctionConfig command
    {
        let trace_c = trace.clone();
        let rt = runtime.clone();
        app = app.register_startup_command(CommandDef::new(
            "MultiFunctionConfig",
            vec![
                ArgDesc { name: "portName", arg_type: ArgType::String, optional: false },
                ArgDesc { name: "uniqueID", arg_type: ArgType::String, optional: false },
                ArgDesc { name: "maxInputPoints", arg_type: ArgType::Int, optional: true },
                ArgDesc { name: "maxOutputPoints", arg_type: ArgType::Int, optional: true },
            ],
            "MultiFunctionConfig portName uniqueID [maxInputPoints] [maxOutputPoints]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let unique_id = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("uniqueID required".into()),
                };
                let max_in = match args.get(2) {
                    Some(ArgValue::Int(n)) => *n as usize,
                    _ => 2048,
                };
                let max_out = match args.get(3) {
                    Some(ArgValue::Int(n)) => *n as usize,
                    _ => 2048,
                };

                let mf_rt = create_usb_2408(&port_name, &unique_id, max_in, max_out)?;

                let port_handle = mf_rt.port_handle().clone();
                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    port_handle,
                    trace_c.clone(),
                );

                *rt.lock().unwrap() = Some(mf_rt);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
