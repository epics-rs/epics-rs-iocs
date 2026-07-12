//! `mythenConfig` — the iocsh command that creates a Mythen port
//! (C `mythenConfig`, mythen.cpp:1400).

use std::sync::Arc;

use epics_rs::ad_core::ioc::GenericDriverContext;
use epics_rs::base::server::iocsh::registry::*;

use mythen::{MythenRuntime, create_mythen_detector};

/// Register the Mythen configure command on an `AdIoc`.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADMYTHEN", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Vec<MythenRuntime>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "mythenConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "IPPortName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                // C also takes maxBuffers, priority and stackSize. The Rust
                // NDArray pool is not buffer-count limited and the acquisition
                // task is an OS thread with the runtime's own priority, so only
                // maxMemory carries over.
                ArgDesc {
                    name: "maxMemory",
                    arg_type: ArgType::Int,
                    optional: true,
                },
            ],
            "mythenConfig portName IPPortName [maxMemory]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let ip_port_name = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("IPPortName required".into()),
                };
                let max_memory = match args.get(2) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 0,
                };

                println!(
                    "mythenConfig: port={port_name}, ipPort={ip_port_name}, maxMemory={max_memory}"
                );

                let runtime = create_mythen_detector(&port_name, &ip_port_name, max_memory)
                    .map_err(|e| format!("failed to create the Mythen detector: {e}"))?;

                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    runtime.port_handle().clone(),
                    trace.clone(),
                );

                mgr.set_driver(Arc::new(GenericDriverContext::new(
                    runtime.pool.clone(),
                    runtime.array_output().clone(),
                    &port_name,
                    mgr.wiring(),
                )));

                rt.lock().unwrap().push(runtime);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // Keep the runtime (and its acquisition task) alive for the IOC's lifetime.
    ioc.keep_alive(runtime);
}
