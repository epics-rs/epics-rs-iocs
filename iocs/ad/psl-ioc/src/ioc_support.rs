use std::sync::Arc;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use psl::{PslRuntime, create_psl_detector};

/// Register `PSLConfig` on an `AdIoc`.
///
/// C parity: `PSLConfig(portName, serverPort, maxBuffers, maxMemory, priority,
/// stackSize)`. `maxBuffers`, `priority` and `stackSize` are accepted for
/// startup-script compatibility and ignored: the Rust pool is bounded by memory
/// alone and the runtime schedules the port actors itself.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADPSL", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<PslRuntime>>> = Arc::new(std::sync::Mutex::new(None));

    let mgr = ioc.mgr().clone();
    let trace = ioc.trace().clone();
    let rt_slot = runtime.clone();
    ioc.register_startup_command(CommandDef::new(
        "PSLConfig",
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
        "PSLConfig portName serverPort [maxBuffers] [maxMemory] [priority] [stackSize]",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) if !s.is_empty() => s.clone(),
                _ => return Err("portName required".into()),
            };
            let server_port = match &args[1] {
                ArgValue::String(s) if !s.is_empty() => s.clone(),
                _ => return Err("serverPort required".into()),
            };
            // 0 means "unlimited" in the C startup scripts.
            let max_memory = match args.get(3) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };

            let server = get_port(&server_port).ok_or_else(|| {
                format!(
                    "server port '{server_port}' not found \
                     (call drvAsynIPPortConfigure first)"
                )
            })?;

            println!("PSLConfig: port={port_name}, server={server_port}");

            let rt = create_psl_detector(&port_name, server.handle.clone(), max_memory)
                .map_err(|e| format!("failed to create the PSL detector: {e}"))?;

            epics_rs::asyn::asyn_record::register_port(
                &port_name,
                rt.port_handle().clone(),
                trace.clone(),
            );

            mgr.set_driver(Arc::new(GenericDriverContext::new(
                rt.pool().clone(),
                rt.array_output().clone(),
                &port_name,
                mgr.wiring(),
            )));

            *rt_slot.lock().unwrap() = Some(rt);
            Ok(CommandOutcome::Continue)
        },
    ));

    ioc.keep_alive(runtime);
}
