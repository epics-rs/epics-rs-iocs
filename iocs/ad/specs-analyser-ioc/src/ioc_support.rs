use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use ad_specs_analyser::{SpecsAnalyserRuntime, create_specs_analyser_detector};

/// Register `specsAnalyserConfig` on an `AdIoc`.
///
/// C signature: `specsAnalyserConfig(portName, driverPort, maxBuffers,
/// maxMemory, priority, stackSize)` (`specsAnalyser.cpp:8`). `priority` and
/// `stackSize` have no equivalent in `ad-core-rs` (the acquisition task is a
/// plain OS thread with no configurable priority/stack size); they are
/// accepted and ignored so the C `st.cmd` line works unchanged.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADSPECSANALYSER", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<SpecsAnalyserRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt_slot = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "specsAnalyserConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "driverPort",
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
            "specsAnalyserConfig portName driverPort [maxBuffers] [maxMemory] \
             [priority] [stackSize]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let driver_port = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("driverPort required".into()),
                };
                let max_buffers = match args.get(2) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as i32,
                    _ => 16,
                };
                // C: maxMemory == 0 means unlimited. The Rust pool needs a
                // finite bound, so 0 selects a 100 MB default.
                let max_memory = match args.get(3) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 100_000_000,
                };

                println!(
                    "specsAnalyserConfig: port={port_name}, driverPort={driver_port}, \
                     maxBuffers={max_buffers}, maxMemory={max_memory}"
                );

                let det = create_specs_analyser_detector(
                    &port_name,
                    &driver_port,
                    max_buffers,
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
