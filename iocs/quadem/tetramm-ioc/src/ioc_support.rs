//! iocsh commands for the TetrAMM IOC.
//!
//! `drvTetrAMMConfigure(portName, QEPortName, ringBufferSize[, maxMemory])`
//! mirrors C++ `drvTetrAMM.cpp::drvTetrAMMConfigure`; the trailing
//! `maxMemory` argument has no C++ analogue (the C++ `NDArrayPool` is
//! unbounded) and bounds the Rust pool.

use std::sync::Arc;

use epics_rs::ad_plugins::ioc::AdIoc;
use epics_rs::asyn::manager::claim_port_driver;
use epics_rs::base::server::iocsh::registry::*;

use quadem::iocsh::{max_memory_arg, register_quadem_port};
use quadem::{TetrAmmRuntime, create_tetramm};

fn tetramm_configure_command(
    ioc: &AdIoc,
    runtime: Arc<std::sync::Mutex<Option<TetrAmmRuntime>>>,
) -> CommandDef {
    let mgr = ioc.mgr().clone();
    CommandDef::new(
        "drvTetrAMMConfigure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "QEPortName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "ringBufferSize",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
            },
        ],
        "drvTetrAMMConfigure portName QEPortName [ringBufferSize] [maxMemory]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let qe_port_name = match &args[1] {
                ArgValue::String(s) => s.to_str()?.to_owned(),
                _ => return Err("QEPortName required".into()),
            };
            // C++ passes ringBufferSize straight to drvQuadEM, which
            // substitutes QE_DEFAULT_RING_BUFFER_SIZE when it is <= 0.
            let ring_buffer_size = match args.get(2) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };
            let max_memory = max_memory_arg(args.get(3));

            // asynPortDriver registers the port before the driver body runs
            // (asynPortDriver.cpp:4031-4040).
            let claim = match claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            let rt = create_tetramm(&claim, &qe_port_name, ring_buffer_size, max_memory)
                .map_err(|e| format!("drvTetrAMMConfigure: {e}"))?;

            register_quadem_port(
                &mgr,
                ctx.db().scan_control().interrupt_gate(),
                claim,
                rt.port_handle().clone(),
                rt.pool.clone(),
                &rt.outputs,
            )
            .map_err(|e| e.to_string())?;

            *runtime.lock().unwrap() = Some(rt);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Register the TetrAMM configure command.
pub fn register(ioc: &mut AdIoc) {
    epics_rs::base::runtime::env::set_default_path(
        "QUADEM",
        concat!(env!("CARGO_MANIFEST_DIR"), "/.."),
    );

    let runtime: Arc<std::sync::Mutex<Option<TetrAmmRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));
    let cmd = tetramm_configure_command(ioc, runtime.clone());
    ioc.register_startup_command(cmd);

    // Keep the runtime (read thread, callback thread, port actor) alive.
    ioc.keep_alive(runtime);
}
