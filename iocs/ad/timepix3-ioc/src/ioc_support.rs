//! `ADTimePixConfig` — the iocsh command that creates a TimePix3 port
//! (C `ADTimePixConfig`, ADTimePix.cpp:1601).

use std::sync::Arc;

use epics_rs::ad_core::ioc::GenericDriverContext;
use epics_rs::base::server::iocsh::registry::*;

use timepix3::{TimePix3Runtime, create_timepix3_detector};

/// Register the TimePix3 configure command on an `AdIoc`.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADTIMEPIX", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Vec<TimePix3Runtime>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "ADTimePixConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "serverURL",
                    arg_type: ArgType::String,
                    optional: false,
                },
                // C also takes maxBuffers, priority and stackSize. The Rust
                // NDArray pool is not buffer-count limited and the background
                // threads run at the runtime's own priority, so only maxMemory
                // carries over. C's `ADTimePixConfigWithFlags` seventh argument
                // (asynFlags) has no counterpart either: the port is always
                // ASYN_MULTIDEVICE | ASYN_CANBLOCK, which is the only
                // combination the driver works under.
                ArgDesc {
                    name: "maxMemory",
                    arg_type: ArgType::Int,
                    optional: true,
                },
            ],
            "ADTimePixConfig portName serverURL [maxMemory]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let server_url = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("serverURL required".into()),
                };
                let max_memory = match args.get(2) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 0,
                };

                println!(
                    "ADTimePixConfig: port={port_name}, serverURL={server_url}, maxMemory={max_memory}"
                );

                let runtime = create_timepix3_detector(&port_name, &server_url, max_memory)
                    .map_err(|e| format!("failed to create the TimePix3 detector: {e}"))?;

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

    // Keep the runtime (and its background threads) alive for the IOC's life.
    ioc.keep_alive(runtime);
}
