use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use ad_pilatus::{PilatusRuntime, create_pilatus_detector};

/// Register `pilatusDetectorConfig` on an `AdIoc`.
///
/// C signature: `pilatusDetectorConfig(portName, camserverPort, maxSizeX,
/// maxSizeY, maxBuffers, maxMemory, priority, stackSize)`. `maxBuffers`,
/// `priority` and `stackSize` have no equivalent in `ad-core-rs` (the pool is
/// memory-bounded and the worker threads are plain OS threads); they are
/// accepted and ignored so the C `st.cmd` line works unchanged.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADPILATUS", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<PilatusRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt_slot = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "pilatusDetectorConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "camserverPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "maxSizeX",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "maxSizeY",
                    arg_type: ArgType::Int,
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
            "pilatusDetectorConfig portName camserverPort maxSizeX maxSizeY \
             [maxBuffers] [maxMemory] [priority] [stackSize]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let camserver_port = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("camserverPort required".into()),
                };
                let size_x = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("maxSizeX required".into()),
                };
                let size_y = match &args[3] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("maxSizeY required".into()),
                };
                // C: maxMemory == 0 means unlimited. The Rust pool needs a
                // finite bound, so 0 selects a 100 MB default.
                let max_memory = match args.get(5) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 100_000_000,
                };

                println!(
                    "pilatusDetectorConfig: port={port_name}, camserver={camserver_port}, \
                     size={size_x}x{size_y}, maxMemory={max_memory}"
                );

                let det = create_pilatus_detector(
                    &port_name,
                    &camserver_port,
                    size_x,
                    size_y,
                    max_memory,
                )?;

                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    det.port_handle().clone(),
                    trace.clone(),
                );

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
