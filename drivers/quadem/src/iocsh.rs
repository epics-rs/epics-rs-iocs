//! iocsh `drv*Configure` verbs for the quadEM drivers. The octet port and
//! EOS verbs an `st.cmd` uses first come from `AdIoc`'s asyn command set.

use std::sync::{Arc, Mutex};

use epics_rs::ad_core::ioc::{GenericDriverContext, PluginManager};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::asyn::manager::claim_port_driver;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;
use epics_rs::base::server::iocsh::registry::{
    ArgDesc, ArgType, ArgValue, CommandContext, CommandDef, CommandOutcome,
};

use crate::ahxxx::{AhxxxRuntime, create_ahxxx};
use crate::drv_quad_em::QE_ADDR_ALL;
use crate::fx4::{Fx4Runtime, create_fx4};
use crate::nsls_em::{NslsEmRuntime, create_nsls_em};
use crate::pcr4::{Pcr4Runtime, create_pcr4};
use crate::t4u::{T4uDirectError, T4uRuntime, create_t4u_direct_em, create_t4u_em};

/// Publish a configured quadEM port: the asyn port itself, then its twelve
/// NDArray fan-outs. The sub-ports a driver builds for itself are published
/// as it builds them.
///
/// Address 0 (the Current1 time series) is the port's default NDArray source;
/// addresses 1..=10 carry the remaining data items and address 11 the full 2-D
/// `[11 x numAverage]` array. Each has its own fan-out, selected downstream by
/// `NDArrayAddr`.
pub fn register_quadem_port(
    mgr: &Arc<PluginManager>,
    owner: &Arc<InterruptGate>,
    claim: PortClaim,
    handle: PortHandle,
    pool: Arc<NDArrayPool>,
    outputs: &[Arc<parking_lot::Mutex<NDArrayOutput>>],
) -> epics_rs::asyn::error::AsynResult<()> {
    let port_name = claim.name().to_string();
    claim.publish(handle, owner)?;

    mgr.set_driver(Arc::new(GenericDriverContext::new(
        pool,
        outputs[0].clone(),
        &port_name,
        mgr.wiring(),
    )));
    for (addr, output) in outputs.iter().enumerate().take(QE_ADDR_ALL + 1).skip(1) {
        mgr.wiring()
            .register_output_at(port_name, addr as i32, output.clone());
    }
    Ok(())
}

/// C++ `drvAHxxxConfigure(portName, QEPortName, ringBufferSize, modelName)`.
///
/// The trailing `maxMemory` argument has no C++ analogue (the C++ `NDArrayPool`
/// is unbounded) and bounds the Rust pool. Upstream drives both AHxxx families
/// from one driver, so one command serves the AH401 and AH501 IOCs.
pub fn ahxxx_configure_command(
    mgr: Arc<PluginManager>,
    runtime: Arc<Mutex<Option<AhxxxRuntime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvAHxxxConfigure",
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
                name: "modelName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
            },
        ],
        "drvAHxxxConfigure portName QEPortName ringBufferSize modelName [maxMemory]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let qe_port_name = match args.get(1) {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("QEPortName required".into()),
            };
            // C++ passes ringBufferSize straight to drvQuadEM, which
            // substitutes QE_DEFAULT_RING_BUFFER_SIZE when it is <= 0.
            let ring_buffer_size = match args.get(2) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };
            let model_name = match args.get(3) {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("modelName required".into()),
            };
            let max_memory = match args.get(4) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 100_000_000,
            };

            // asynPortDriver registers the port before the driver body runs
            // (asynPortDriver.cpp:4031-4040).
            let claim = match claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            let rt = create_ahxxx(
                &claim,
                &qe_port_name,
                ring_buffer_size,
                max_memory,
                &model_name,
            )
            .map_err(|e| format!("drvAHxxxConfigure: {e}"))?;

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

/// C++ `drvNSLS_EMConfigure(portName, broadcastAddress, moduleID,
/// ringBufferSize)`.
///
/// Unlike the other quadEM devices the NSLS_EM builds its own three asyn IP
/// ports (UDP discovery, TCP command, TCP data), so no `drvAsynIPPortConfigure`
/// precedes this verb.
pub fn nsls_em_configure_command(
    mgr: Arc<PluginManager>,
    runtime: Arc<Mutex<Option<NslsEmRuntime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvNSLS_EMConfigure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "broadcastAddress",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "moduleID",
                arg_type: ArgType::Int,
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
        "drvNSLS_EMConfigure portName broadcastAddress moduleID ringBufferSize [maxMemory]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let broadcast_address = match args.get(1) {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("broadcastAddress required".into()),
            };
            let module_id = match args.get(2) {
                Some(ArgValue::Int(n)) => *n as i32,
                _ => return Err("moduleID required".into()),
            };
            let ring_buffer_size = match args.get(3) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };
            let max_memory = match args.get(4) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 100_000_000,
            };

            // asynPortDriver registers the port before the driver body runs
            // (asynPortDriver.cpp:4031-4040).
            let claim = match claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            let rt = create_nsls_em(
                &claim,
                &broadcast_address,
                module_id,
                ring_buffer_size,
                max_memory,
                ctx.db().scan_control().interrupt_gate(),
            )
            .map_err(|e| format!("drvNSLS_EMConfigure: {e}"))?;

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

/// C++ `drvFX4Configure(portName, FX4_IP, ringBufferSize)`.
///
/// The FX4 driver opens its own WebSocket, so no `drvAsynIPPortConfigure`
/// precedes this verb. The trailing `maxMemory` argument has no C++ analogue
/// and bounds the Rust `NDArrayPool`.
pub fn fx4_configure_command(
    mgr: Arc<PluginManager>,
    runtime: Arc<Mutex<Option<Fx4Runtime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvFX4Configure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "FX4_IP",
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
        "drvFX4Configure portName FX4_IP ringBufferSize [maxMemory]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let fx4_ip = match args.get(1) {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("FX4_IP required".into()),
            };
            let ring_buffer_size = match args.get(2) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };
            let max_memory = match args.get(3) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 100_000_000,
            };

            // asynPortDriver registers the port before the driver body runs
            // (asynPortDriver.cpp:4031-4040).
            let claim = match claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            let rt = create_fx4(&claim, &fx4_ip, ring_buffer_size, max_memory)
                .map_err(|e| format!("drvFX4Configure: {e}"))?;

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

/// C++ `drvPCR4Configure(portName, QEPortName, ringBufferSize)`.
///
/// The trailing `maxMemory` argument has no C++ analogue (the C++ `NDArrayPool`
/// is unbounded) and bounds the Rust pool.
pub fn pcr4_configure_command(
    mgr: Arc<PluginManager>,
    runtime: Arc<Mutex<Option<Pcr4Runtime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvPCR4Configure",
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
        "drvPCR4Configure portName QEPortName ringBufferSize [maxMemory]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let qe_port_name = match args.get(1) {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("QEPortName required".into()),
            };
            let ring_buffer_size = match args.get(2) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };
            let max_memory = match args.get(3) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 100_000_000,
            };

            // asynPortDriver registers the port before the driver body runs
            // (asynPortDriver.cpp:4031-4040).
            let claim = match claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            let rt = create_pcr4(&claim, &qe_port_name, ring_buffer_size, max_memory)
                .map_err(|e| format!("drvPCR4Configure: {e}"))?;

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

/// C++ `drvT4U_EMConfigure(portName, qtHostAddress, ringBufferSize,
/// basePortNum)`.
///
/// The T4U drivers build their own asyn IP ports, so no
/// `drvAsynIPPortConfigure` precedes this verb. The trailing `maxMemory`
/// argument has no C++ analogue and bounds the Rust `NDArrayPool`.
pub fn t4u_em_configure_command(
    mgr: Arc<PluginManager>,
    runtime: Arc<Mutex<Option<T4uRuntime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvT4U_EMConfigure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "qtHostAddress",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "ringBufferSize",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "basePortNum",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
            },
        ],
        "drvT4U_EMConfigure portName qtHostAddress ringBufferSize basePortNum [maxMemory]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let qt_host_address = match args.get(1) {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("qtHostAddress required".into()),
            };
            let ring_buffer_size = match args.get(2) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };
            let base_port_num = port_number(args.get(3), "basePortNum")?;
            let max_memory = match args.get(4) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 100_000_000,
            };

            // asynPortDriver registers the port before the driver body runs
            // (asynPortDriver.cpp:4031-4040).
            let claim = match claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            let rt = create_t4u_em(
                &claim,
                &qt_host_address,
                ring_buffer_size,
                base_port_num,
                max_memory,
                ctx.db().scan_control().interrupt_gate(),
            )
            .map_err(|e| format!("drvT4U_EMConfigure: {e}"))?;

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

/// C++ `drvT4UDirect_EMConfigure(portName, T4UAddress, ringBufferSize,
/// basePortNum, cfgFileName)`.
pub fn t4u_direct_em_configure_command(
    mgr: Arc<PluginManager>,
    runtime: Arc<Mutex<Option<T4uRuntime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvT4UDirect_EMConfigure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "T4UAddress",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "ringBufferSize",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "basePortNum",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "cfgFileName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
            },
        ],
        "drvT4UDirect_EMConfigure portName T4UAddress ringBufferSize basePortNum cfgFileName \
         [maxMemory]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let t4u_address = match args.get(1) {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("T4UAddress required".into()),
            };
            let ring_buffer_size = match args.get(2) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };
            let base_port_num = port_number(args.get(3), "basePortNum")?;
            let cfg_file_name = match args.get(4) {
                Some(ArgValue::String(s)) => s.to_str()?.to_owned(),
                _ => return Err("cfgFileName required".into()),
            };
            let max_memory = match args.get(5) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 100_000_000,
            };

            // asynPortDriver registers the port before the driver body runs
            // (asynPortDriver.cpp:4031-4040).
            let claim = match claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            let rt = match create_t4u_direct_em(
                &claim,
                &t4u_address,
                ring_buffer_size,
                base_port_num,
                &cfg_file_name,
                max_memory,
                ctx.db().scan_control().interrupt_gate(),
            ) {
                Ok(rt) => rt,
                Err(T4uDirectError::Thrown(what)) => {
                    ctx.eprintln(&cpp_error(&what));
                    return Ok(CommandOutcome::Failed);
                }
                Err(e) => return Err(format!("drvT4UDirect_EMConfigure: {e}")),
            };

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

/// iocsh's line for an exception a command throws (iocsh.cpp:1276-1278).
fn cpp_error(what: &str) -> String {
    let line = format!("C++ error: {what}");
    if epics_rs::base::server::iocsh::use_ansi_color() {
        use epics_rs::base::runtime::log::{ANSI_ESC_RED, ANSI_ESC_RESET};
        format!("{ANSI_ESC_RED}{line}{ANSI_ESC_RESET}")
    } else {
        line
    }
}

/// C++ takes the base port number as an `int` and passes it to `%d`; a value
/// outside the TCP/UDP port range is a typo in `st.cmd`, so it is rejected here
/// rather than wrapped.
fn port_number(arg: Option<&ArgValue>, name: &str) -> Result<u16, String> {
    match arg {
        Some(ArgValue::Int(n)) => {
            u16::try_from(*n).map_err(|_| format!("{name}: {n} is not a valid port number"))
        }
        _ => Err(format!("{name} required")),
    }
}
