//! iocsh verbs every quadEM IOC needs to build the octet port its driver
//! talks through.
//!
//! `drvAsynIPPortConfigure` and `drvAsynSerialPortConfigure` come from
//! `asyn-rs`. `asynOctetSetInputEos` / `asynOctetSetOutputEos` are provided
//! here instead: the `asyn-rs` versions resolve a port name through a
//! `PortManager`, which the port-configure verbs do not populate — they
//! register into the `asyn_record` registry, which is what these look in.

use std::sync::{Arc, Mutex};

use epics_rs::ad_core::ioc::{GenericDriverContext, PluginManager};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::server::iocsh::registry::{
    ArgDesc, ArgType, ArgValue, CommandContext, CommandDef, CommandOutcome,
};

use crate::ahxxx::{AhxxxRuntime, create_ahxxx};
use crate::drv_quad_em::QE_ADDR_ALL;
use crate::fx4::{Fx4Runtime, create_fx4};
use crate::nsls_em::{NslsEmRuntime, create_nsls_em};
use crate::pcr4::{Pcr4Runtime, create_pcr4};
use crate::t4u::{T4uRuntime, create_t4u_direct_em, create_t4u_em};

/// Rust port of EPICS `epicsStrnRawFromEscaped` (libcom `epicsString.c`),
/// restricted to the escapes an octet EOS can carry.
pub fn raw_from_escaped(src: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(src.len());
    let mut chars = src.chars();
    while let Some(c) = chars.next() {
        if c != '\\' {
            let mut buf = [0u8; 4];
            out.extend_from_slice(c.encode_utf8(&mut buf).as_bytes());
            continue;
        }
        match chars.next() {
            Some('r') => out.push(b'\r'),
            Some('n') => out.push(b'\n'),
            Some('t') => out.push(b'\t'),
            Some('a') => out.push(0x07),
            Some('b') => out.push(0x08),
            Some('f') => out.push(0x0c),
            Some('v') => out.push(0x0b),
            Some('0') => out.push(0),
            Some('\\') => out.push(b'\\'),
            Some('\'') => out.push(b'\''),
            Some('"') => out.push(b'"'),
            Some(other) => {
                let mut buf = [0u8; 4];
                out.extend_from_slice(other.encode_utf8(&mut buf).as_bytes());
            }
            None => out.push(b'\\'),
        }
    }
    out
}

fn eos_command(name: &'static str, input: bool) -> CommandDef {
    CommandDef::new(
        name,
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "addr",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "eos",
                arg_type: ArgType::String,
                optional: false,
            },
        ],
        format!("{name} portName addr eos"),
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port = match args.first() {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("portName required".into()),
            };
            let eos = match args.get(2) {
                Some(ArgValue::String(s)) => raw_from_escaped(s),
                _ => Vec::new(),
            };
            let Some(entry) = epics_rs::asyn::asyn_record::get_port(&port) else {
                ctx.println(&format!("{name}: port '{port}' not found"));
                return Ok(CommandOutcome::Continue);
            };
            let res = if input {
                entry
                    .handle
                    .set_input_eos_blocking(epics_rs::asyn::user::AsynUser::default(), &eos)
            } else {
                entry
                    .handle
                    .set_output_eos_blocking(epics_rs::asyn::user::AsynUser::default(), &eos)
            };
            if let Err(e) = res {
                ctx.println(&format!("{name}: {e}"));
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// The four startup verbs a quadEM `st.cmd` uses before its `drv*Configure`
/// call: create the octet port, then frame it.
pub fn octet_port_commands(trace: Arc<TraceManager>) -> Vec<CommandDef> {
    vec![
        epics_rs::asyn::iocsh::drv_asyn_ip_port_configure_command(
            epics_rs::asyn::services::PortServices::new(trace.clone()),
        ),
        epics_rs::asyn::iocsh::drv_asyn_serial_port_configure_command(
            epics_rs::asyn::services::PortServices::new(trace),
        ),
        eos_command("asynOctetSetInputEos", true),
        eos_command("asynOctetSetOutputEos", false),
    ]
}

/// Publish a configured quadEM port: the asyn port itself, then its twelve
/// NDArray fan-outs.
///
/// Address 0 (the Current1 time series) is the port's default NDArray source;
/// addresses 1..=10 carry the remaining data items and address 11 the full 2-D
/// `[11 x numAverage]` array. Each has its own fan-out, selected downstream by
/// `NDArrayAddr`.
pub fn register_quadem_port(
    mgr: &Arc<PluginManager>,
    trace: &Arc<TraceManager>,
    port_name: &str,
    handle: PortHandle,
    pool: Arc<NDArrayPool>,
    outputs: &[Arc<parking_lot::Mutex<NDArrayOutput>>],
) -> epics_rs::asyn::error::AsynResult<()> {
    epics_rs::asyn::asyn_record::register_port(port_name, handle, trace.clone())?;

    mgr.set_driver(Arc::new(GenericDriverContext::new(
        pool,
        outputs[0].clone(),
        port_name,
        mgr.wiring(),
    )));
    for (addr, output) in outputs.iter().enumerate().take(QE_ADDR_ALL + 1).skip(1) {
        mgr.wiring()
            .register_output(&format!("{port_name}:{addr}"), output.clone());
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
    trace: Arc<TraceManager>,
    runtime: Arc<Mutex<Option<AhxxxRuntime>>>,
) -> CommandDef {
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
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("portName required".into()),
            };
            let qe_port_name = match args.get(1) {
                Some(ArgValue::String(s)) => s.clone(),
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

            register_quadem_port(
                &mgr,
                &trace,
                &port_name,
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
    trace: Arc<TraceManager>,
    runtime: Arc<Mutex<Option<NslsEmRuntime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvNSLS_EMConfigure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "broadcastAddress",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "moduleID",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "ringBufferSize",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
                optional: true,
            },
        ],
        "drvNSLS_EMConfigure portName broadcastAddress moduleID ringBufferSize [maxMemory]",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("portName required".into()),
            };
            let broadcast_address = match args.get(1) {
                Some(ArgValue::String(s)) => s.clone(),
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

            let rt = create_nsls_em(
                &port_name,
                &broadcast_address,
                module_id,
                ring_buffer_size,
                max_memory,
            )
            .map_err(|e| format!("drvNSLS_EMConfigure: {e}"))?;

            register_quadem_port(
                &mgr,
                &trace,
                &port_name,
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
    trace: Arc<TraceManager>,
    runtime: Arc<Mutex<Option<Fx4Runtime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvFX4Configure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "FX4_IP",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "ringBufferSize",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
                optional: true,
            },
        ],
        "drvFX4Configure portName FX4_IP ringBufferSize [maxMemory]",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("portName required".into()),
            };
            let fx4_ip = match args.get(1) {
                Some(ArgValue::String(s)) => s.clone(),
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

            let rt = create_fx4(&port_name, &fx4_ip, ring_buffer_size, max_memory)
                .map_err(|e| format!("drvFX4Configure: {e}"))?;

            register_quadem_port(
                &mgr,
                &trace,
                &port_name,
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
    trace: Arc<TraceManager>,
    runtime: Arc<Mutex<Option<Pcr4Runtime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvPCR4Configure",
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
                name: "maxMemory",
                arg_type: ArgType::Int,
                optional: true,
            },
        ],
        "drvPCR4Configure portName QEPortName ringBufferSize [maxMemory]",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("portName required".into()),
            };
            let qe_port_name = match args.get(1) {
                Some(ArgValue::String(s)) => s.clone(),
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

            let rt = create_pcr4(&port_name, &qe_port_name, ring_buffer_size, max_memory)
                .map_err(|e| format!("drvPCR4Configure: {e}"))?;

            register_quadem_port(
                &mgr,
                &trace,
                &port_name,
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
    trace: Arc<TraceManager>,
    runtime: Arc<Mutex<Option<T4uRuntime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvT4U_EMConfigure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "qtHostAddress",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "ringBufferSize",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "basePortNum",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
                optional: true,
            },
        ],
        "drvT4U_EMConfigure portName qtHostAddress ringBufferSize basePortNum [maxMemory]",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("portName required".into()),
            };
            let qt_host_address = match args.get(1) {
                Some(ArgValue::String(s)) => s.clone(),
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

            let rt = create_t4u_em(
                &port_name,
                &qt_host_address,
                ring_buffer_size,
                base_port_num,
                max_memory,
            )
            .map_err(|e| format!("drvT4U_EMConfigure: {e}"))?;

            register_quadem_port(
                &mgr,
                &trace,
                &port_name,
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
    trace: Arc<TraceManager>,
    runtime: Arc<Mutex<Option<T4uRuntime>>>,
) -> CommandDef {
    CommandDef::new(
        "drvT4UDirect_EMConfigure",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "T4UAddress",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "ringBufferSize",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "basePortNum",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "cfgFileName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
                optional: true,
            },
        ],
        "drvT4UDirect_EMConfigure portName T4UAddress ringBufferSize basePortNum cfgFileName \
         [maxMemory]",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match args.first() {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("portName required".into()),
            };
            let t4u_address = match args.get(1) {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("T4UAddress required".into()),
            };
            let ring_buffer_size = match args.get(2) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };
            let base_port_num = port_number(args.get(3), "basePortNum")?;
            let cfg_file_name = match args.get(4) {
                Some(ArgValue::String(s)) => s.clone(),
                _ => return Err("cfgFileName required".into()),
            };
            let max_memory = match args.get(5) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 100_000_000,
            };

            let rt = create_t4u_direct_em(
                &port_name,
                &t4u_address,
                ring_buffer_size,
                base_port_num,
                &cfg_file_name,
                max_memory,
            )
            .map_err(|e| format!("drvT4UDirect_EMConfigure: {e}"))?;

            register_quadem_port(
                &mgr,
                &trace,
                &port_name,
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

#[cfg(test)]
mod tests {
    use super::raw_from_escaped;

    #[test]
    fn decodes_crlf_input_eos() {
        assert_eq!(raw_from_escaped("\\r\\n"), b"\r\n");
    }

    #[test]
    fn decodes_cr_output_eos() {
        assert_eq!(raw_from_escaped("\\r"), b"\r");
    }

    #[test]
    fn passes_through_plain_text_and_backslash() {
        assert_eq!(raw_from_escaped("ab"), b"ab");
        assert_eq!(raw_from_escaped("\\\\"), b"\\");
    }

    #[test]
    fn empty_eos_decodes_to_empty() {
        assert_eq!(raw_from_escaped(""), b"");
    }
}
