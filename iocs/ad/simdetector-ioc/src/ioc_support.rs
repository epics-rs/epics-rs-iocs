use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;
use epics_rs::ad_core::ndarray::NDDataType;

use ad_simdetector::{SimDetectorRuntime, create_sim_detector};

/// Register `simDetectorConfig` on an `AdIoc`.
///
/// Signature matches the C iocsh command (simDetector.cpp:1160-1196):
///   `simDetectorConfig(portName, maxSizeX, maxSizeY, dataType, maxBuffers, maxMemory)`
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADSIMDETECTOR", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<SimDetectorRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "simDetectorConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "maxSizeX",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "maxSizeY",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "dataType",
                    arg_type: ArgType::Int,
                    optional: true,
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
            ],
            "simDetectorConfig portName [maxSizeX] [maxSizeY] [dataType] [maxBuffers] [maxMemory]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let size_x = match args.get(1) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => 1024,
                };
                let size_y = match args.get(2) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => 1024,
                };
                let data_type_code = match args.get(3) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => 1, // NDUInt8
                };
                let data_type = u8::try_from(data_type_code)
                    .ok()
                    .and_then(NDDataType::from_ordinal)
                    .ok_or_else(|| format!("invalid dataType {data_type_code}"))?;
                // C's `maxBuffers == 0` means "unlimited"; the Rust pool is
                // bounded by maxMemory only, so this value is unused.
                let max_buffers = match args.get(4) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => 0,
                };
                // C's `maxMemory == 0` means "unlimited"; `NDArrayPool` uses the
                // same convention.
                let max_memory = match args.get(5) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 0,
                };

                println!(
                    "simDetectorConfig: port={port_name}, size={size_x}x{size_y}, \
                     dataType={data_type_code}, maxBuffers={max_buffers}, maxMemory={max_memory}"
                );

                let sim_rt = create_sim_detector(
                    &port_name,
                    size_x,
                    size_y,
                    data_type,
                    max_buffers,
                    max_memory,
                )
                .map_err(|e| format!("failed to create simDetector: {e}"))?;

                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    sim_rt.port_handle().clone(),
                    trace.clone(),
                );

                mgr.set_driver(Arc::new(GenericDriverContext::new(
                    sim_rt.pool().clone(),
                    sim_rt.array_output().clone(),
                    &port_name,
                    mgr.wiring(),
                )));

                *rt.lock().unwrap() = Some(sim_rt);

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    ioc.keep_alive(runtime);
}
