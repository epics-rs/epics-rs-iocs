use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use ad_std_arrays_driver::{NdStdArraysRuntime, create_nd_std_arrays};

/// Register `NDDriverStdArraysConfig` on an `AdIoc`.
///
/// Signature matches the C iocsh command (NDDriverStdArrays.cpp:424-450):
///   `NDDriverStdArraysConfig(portName, maxBuffers, maxMemory, priority, stackSize)`
///
/// DEVIATION: `priority` and `stackSize` configure `epicsThreadCreate`; the
/// Rust publisher task runs on a `run_thread_named` worker with no equivalent
/// knobs, so both are accepted and ignored.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("NDDRIVERSTDARRAYS", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<NdStdArraysRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "NDDriverStdArraysConfig",
            vec![
                ArgDesc {
                    name: "portName",
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
            "NDDriverStdArraysConfig portName [maxBuffers] [maxMemory] [priority] [stackSize]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                // C: `(maxBuffers < 0) ? 0 : maxBuffers`. The Rust pool is
                // bounded by maxMemory only, so this value is unused.
                let max_buffers = match args.get(1) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as i32,
                    _ => 0,
                };
                // C: `(maxMemory < 0) ? 0 : maxMemory`; 0 means unlimited,
                // which `NDArrayPool` uses too.
                let max_memory = match args.get(2) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 0,
                };

                println!(
                    "NDDriverStdArraysConfig: port={port_name}, maxBuffers={max_buffers}, \
                     maxMemory={max_memory}"
                );

                let nd_rt = create_nd_std_arrays(&port_name, max_buffers, max_memory)
                    .map_err(|e| format!("failed to create NDDriverStdArrays: {e}"))?;

                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    nd_rt.port_handle().clone(),
                    trace.clone(),
                );

                // The single address-0 NDArray output doubles as the driver
                // context for downstream plugin wiring.
                mgr.set_driver(Arc::new(GenericDriverContext::new(
                    nd_rt.pool().clone(),
                    nd_rt.array_output().clone(),
                    &port_name,
                    mgr.wiring(),
                )));

                *rt.lock().unwrap() = Some(nd_rt);

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    ioc.keep_alive(runtime);
}
