//! iocsh commands for the AH401 IOC.
//!
//! `drvAHxxxConfigure(portName, QEPortName, ringBufferSize, modelName[,
//! maxMemory])` mirrors C++ `drvAHxxx.cpp::drvAHxxxConfigure`; the trailing
//! `maxMemory` argument has no C++ analogue (the C++ `NDArrayPool` is
//! unbounded) and bounds the Rust pool. Only the ported models — `AH401B` and
//! `AH401D` — are accepted.

use std::sync::Arc;

use epics_rs::ad_core::ioc::GenericDriverContext;
use epics_rs::ad_plugins::ioc::AdIoc;
use epics_rs::base::server::iocsh::registry::*;

use quadem::drv_quad_em::QE_ADDR_ALL;
use quadem::iocsh::octet_port_commands;
use quadem::{AhxxxRuntime, create_ahxxx};

fn ahxxx_configure_command(
    ioc: &AdIoc,
    runtime: Arc<std::sync::Mutex<Option<AhxxxRuntime>>>,
) -> CommandDef {
    let mgr = ioc.mgr().clone();
    let trace = ioc.trace().clone();
    CommandDef::new(
        "drvAHxxxConfigure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "QEPortName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "ringBufferSize",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "modelName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
                optional: true,
            },
        ],
        "drvAHxxxConfigure portName QEPortName ringBufferSize modelName [maxMemory]",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) => s.clone(),
                _ => return Err("portName required".into()),
            };
            let qe_port_name = match &args[1] {
                ArgValue::String(s) => s.clone(),
                _ => return Err("QEPortName required".into()),
            };
            // C++ passes ringBufferSize straight to drvQuadEM, which
            // substitutes QE_DEFAULT_RING_BUFFER_SIZE when it is <= 0.
            let ring_buffer_size = match args.get(2) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };
            let model_name = match args.get(3) {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("modelName required".into()),
            };
            let max_memory = match args.get(4) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 100_000_000,
            };

            let rt = create_ahxxx(
                &port_name,
                &qe_port_name,
                ring_buffer_size,
                max_memory,
                &model_name,
            )
            .map_err(|e| format!("drvAHxxxConfigure: {e}"))?;

            epics_rs::asyn::asyn_record::register_port(
                &port_name,
                rt.port_handle().clone(),
                trace.clone(),
            );

            // Address 0 (Current1 time series) is the port's default NDArray
            // source; addresses 1..=10 are the remaining data items and
            // address 11 is the full 2-D [11 x numAverage] array. Each has its
            // own fan-out, selected downstream by NDArrayAddr.
            mgr.set_driver(Arc::new(GenericDriverContext::new(
                rt.pool.clone(),
                rt.outputs[0].clone(),
                &port_name,
                mgr.wiring(),
            )));
            for addr in 1..=QE_ADDR_ALL {
                mgr.wiring()
                    .register_output(&format!("{port_name}:{addr}"), rt.outputs[addr].clone());
            }

            *runtime.lock().unwrap() = Some(rt);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Register the AHxxx configure command and the octet-port verbs it needs.
pub fn register(ioc: &mut AdIoc) {
    epics_rs::base::runtime::env::set_default("QUADEM", env!("CARGO_MANIFEST_DIR"));

    for cmd in octet_port_commands(ioc.trace().clone()) {
        ioc.register_startup_command(cmd);
    }

    let runtime: Arc<std::sync::Mutex<Option<AhxxxRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));
    let cmd = ahxxx_configure_command(ioc, runtime.clone());
    ioc.register_startup_command(cmd);

    // Keep the runtime (read thread, callback thread, port actor) alive.
    ioc.keep_alive(runtime);
}
