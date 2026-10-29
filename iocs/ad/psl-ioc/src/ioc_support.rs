use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::sync_io::SyncIOHandle;
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
    epics_rs::base::runtime::env::set_default_path("ADPSL", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<PslRuntime>>> = Arc::new(std::sync::Mutex::new(None));

    let mgr = ioc.mgr().clone();
    let rt_slot = runtime.clone();
    ioc.register_startup_command(CommandDef::new(
        "PSLConfig",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "serverPort",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "maxBuffers",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "priority",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "stackSize",
                arg_type: ArgType::Int,
            },
        ],
        "PSLConfig portName serverPort [maxBuffers] [maxMemory] [priority] [stackSize]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) if !s.is_empty() => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let server_port = match &args[1] {
                ArgValue::String(s) if !s.is_empty() => s.to_str()?.to_owned(),
                _ => return Err("serverPort required".into()),
            };
            // 0 means "unlimited" in the C startup scripts.
            let max_memory = match args.get(3) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };

            let claim = match epics_rs::asyn::manager::claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            // C ignores the connect status (PSL.cpp:166-167): a missing server
            // port fails getVersion and leaves the port up.
            let (server, _) =
                SyncIOHandle::connect_port(&server_port, 0, Duration::ZERO, InterfaceType::Octet);

            let rt = create_psl_detector(&claim, server, max_memory)
                .map_err(|e| format!("failed to create the PSL detector: {e}"))?;

            claim
                .publish(
                    rt.port_handle().clone(),
                    ctx.db().scan_control().interrupt_gate(),
                )
                .map_err(|e| e.to_string())?;

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
