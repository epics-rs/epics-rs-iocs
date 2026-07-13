use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::plugin::wiring::upstream_key;

use ad_csimdetector::{CSimDetectorRuntime, MAX_SIGNALS, create_c_sim_detector};

/// Register `ADCSimDetectorConfig` on an `AdIoc`.
///
/// Signature matches the C iocsh command (ADCSimDetector.cpp:412-450):
///   `ADCSimDetectorConfig(portName, numTimePoints, dataType, maxBuffers, maxMemory, priority, stackSize)`
///
/// DEVIATION: `priority` and `stackSize` configure `epicsThreadCreate`; the
/// Rust simulation task runs on a `run_thread_named` worker with no equivalent
/// knobs, so both are accepted and ignored.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADCSIMDETECTOR", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<CSimDetectorRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "ADCSimDetectorConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "numTimePoints",
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
            "ADCSimDetectorConfig portName [numTimePoints] [dataType] [maxBuffers] [maxMemory] \
             [priority] [stackSize]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let num_time_points = match args.get(1) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => 1000,
                };
                let data_type_code = match args.get(2) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => NDDataType::Float64 as u8 as i32,
                };
                let data_type = u8::try_from(data_type_code)
                    .ok()
                    .and_then(NDDataType::from_ordinal)
                    .ok_or_else(|| format!("invalid dataType {data_type_code}"))?;
                // C's `maxBuffers == 0` means "unlimited"; the Rust pool is
                // bounded by maxMemory only, so this value is unused.
                let max_buffers = match args.get(3) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => 0,
                };
                // C's `maxMemory == 0` means "unlimited"; `NDArrayPool` uses the
                // same convention.
                let max_memory = match args.get(4) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 0,
                };

                println!(
                    "ADCSimDetectorConfig: port={port_name}, numTimePoints={num_time_points}, \
                     dataType={data_type_code}, maxBuffers={max_buffers}, maxMemory={max_memory}"
                );

                let sim_rt = create_c_sim_detector(
                    &port_name,
                    num_time_points,
                    data_type,
                    max_buffers,
                    max_memory,
                )
                .map_err(|e| format!("failed to create ADCSimDetector: {e}"))?;

                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    sim_rt.port_handle().clone(),
                    trace.clone(),
                )
                .map_err(|e| e.to_string())?;

                // Address 0 is the 2-D array and doubles as the driver context.
                mgr.set_driver(Arc::new(GenericDriverContext::new(
                    sim_rt.pool().clone(),
                    sim_rt.array_output(0).clone(),
                    &port_name,
                    mgr.wiring(),
                )));
                // Addresses 1..=MAX_SIGNALS carry the per-signal 1-D arrays.
                // `register_output` keys on `upstream_key(name, 0)`, i.e. the
                // bare name, so the already-suffixed key is passed through.
                for addr in 1..=MAX_SIGNALS {
                    mgr.wiring().register_output(
                        &upstream_key(&port_name, addr as i32),
                        sim_rt.array_output(addr).clone(),
                    );
                }

                *rt.lock().unwrap() = Some(sim_rt);

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    ioc.keep_alive(runtime);
}
