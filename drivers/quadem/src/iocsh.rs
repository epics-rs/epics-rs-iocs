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
use crate::pcr4::{Pcr4Runtime, create_pcr4};

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
                entry.handle.set_input_eos_blocking(&eos)
            } else {
                entry.handle.set_output_eos_blocking(&eos)
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
        epics_rs::asyn::iocsh::drv_asyn_ip_port_configure_command(trace.clone()),
        epics_rs::asyn::iocsh::drv_asyn_serial_port_configure_command(trace),
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
) {
    epics_rs::asyn::asyn_record::register_port(port_name, handle, trace.clone());

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
            );

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
            );

            *runtime.lock().unwrap() = Some(rt);
            Ok(CommandOutcome::Continue)
        },
    )
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
