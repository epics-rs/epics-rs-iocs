use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use ad_mar345::{Mar345Runtime, create_mar345_detector};

/// Register `mar345Config` on an `AdIoc`.
///
/// C signature: `mar345Config(portName, serverPort, maxBuffers, maxMemory,
/// priority, stackSize)`. `maxBuffers`, `priority` and `stackSize` have no
/// equivalent in `ad-core-rs` (the pool is memory-bounded and the worker thread
/// is a plain OS thread); they are accepted and ignored so the C `st.cmd` line
/// works unchanged. The sensor size is not a config argument — mar345 fixes
/// `ADMaxSizeX`/`ADMaxSizeY` at 3450 and learns the scan size from mode changes.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADMAR345", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<Mar345Runtime>>> =
        Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt_slot = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "mar345Config",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "serverPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "maxBuffers",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "maxMemory",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "priority",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "stackSize",
                    arg_type: ArgType::Int,
                    optional: true,
                },
            ],
            "mar345Config portName serverPort [maxBuffers] [maxMemory] \
             [priority] [stackSize]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let server_port = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("serverPort required".into()),
                };
                // C: maxMemory == 0 means unlimited. The Rust pool needs a
                // finite bound, so 0 selects a 100 MB default.
                let max_memory = match args.get(3) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 100_000_000,
                };

                println!(
                    "mar345Config: port={port_name}, server={server_port}, \
                     maxMemory={max_memory}"
                );

                let det = create_mar345_detector(&port_name, &server_port, max_memory)?;

                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    det.port_handle().clone(),
                    trace.clone(),
                )
                .map_err(|e| e.to_string())?;

                mgr.set_driver(Arc::new(GenericDriverContext::new(
                    det.pool().clone(),
                    det.array_output().clone(),
                    &port_name,
                    mgr.wiring(),
                )));

                *rt_slot.lock().unwrap() = Some(det);

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // Keep the runtime alive for the IOC's lifetime.
    ioc.keep_alive(runtime);
}
