use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use bruker::{BrukerRuntime, create_bruker_detector};

/// Register `BISDetectorConfig` on an `AdIoc`.
///
/// C parity: `BISDetectorConfig(portName, BISPortName, statusPortName,
/// maxBuffers, maxMemory, priority, stackSize)`. `maxBuffers`, `priority` and
/// `stackSize` are accepted for startup-script compatibility and ignored: the
/// Rust pool is bounded by memory alone and the runtime schedules the port
/// actors itself.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADBRUKER", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<BrukerRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    let mgr = ioc.mgr().clone();
    let rt_slot = runtime.clone();
    ioc.register_startup_command(CommandDef::new(
        "BISDetectorConfig",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "BISPortName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "statusPortName",
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
        "BISDetectorConfig portName BISPortName statusPortName \
         [maxBuffers] [maxMemory] [priority] [stackSize]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) if !s.is_empty() => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let bis_port = match &args[1] {
                ArgValue::String(s) if !s.is_empty() => s.to_str()?.to_owned(),
                _ => return Err("BISPortName required".into()),
            };
            let status_port = match &args[2] {
                ArgValue::String(s) if !s.is_empty() => s.to_str()?.to_owned(),
                _ => return Err("statusPortName required".into()),
            };
            // 0 means "unlimited" in the C startup scripts.
            let max_memory = match args.get(4) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };

            // asynPortDriver registers the port before BISDetector's body looks
            // up either socket (asynPortDriver.cpp:4030-4040, BISDetector.cpp:759).
            let claim = match epics_rs::asyn::manager::claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            // C ignores both connect statuses (BISDetector.cpp:759-760): a
            // missing socket port leaves a user whose every transfer fails.
            let (command, _) =
                SyncIOHandle::connect_port(&bis_port, 0, Duration::ZERO, InterfaceType::Octet);
            let (status, _) =
                SyncIOHandle::connect_port(&status_port, 0, Duration::ZERO, InterfaceType::Octet);

            let rt = create_bruker_detector(&claim, command, status, max_memory)
                .map_err(|e| format!("failed to create the BIS detector: {e}"))?;

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
