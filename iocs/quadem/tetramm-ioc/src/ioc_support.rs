//! iocsh commands for the TetrAMM IOC.
//!
//! `drvTetrAMMConfigure(portName, QEPortName, ringBufferSize[, maxMemory])`
//! mirrors C++ `drvTetrAMM.cpp::drvTetrAMMConfigure`; the trailing
//! `maxMemory` argument has no C++ analogue (the C++ `NDArrayPool` is
//! unbounded) and bounds the Rust pool.
//!
//! The octet-port verbs (`drvAsynIPPortConfigure`,
//! `drvAsynSerialPortConfigure`) come from `asyn-rs`. `asynOctetSetInputEos`
//! / `asynOctetSetOutputEos` are re-implemented here against the
//! `asyn_record` port registry, because the `asyn-rs` versions resolve names
//! through a `PortManager` that the port-configure verbs do not populate.

use std::sync::Arc;

use epics_rs::ad_core::ioc::GenericDriverContext;
use epics_rs::ad_plugins::ioc::AdIoc;
use epics_rs::base::server::iocsh::registry::*;

use quadem::drv_quad_em::QE_ADDR_ALL;
use quadem::{TetrAmmRuntime, create_tetramm};

/// Rust port of EPICS `epicsStrnRawFromEscaped` (libcom `epicsString.c`),
/// restricted to the escapes an octet EOS can carry.
fn raw_from_escaped(src: &str) -> Vec<u8> {
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
            let port = match &args[0] {
                ArgValue::String(s) => s.clone(),
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

fn tetramm_configure_command(
    ioc: &AdIoc,
    runtime: Arc<std::sync::Mutex<Option<TetrAmmRuntime>>>,
) -> CommandDef {
    let mgr = ioc.mgr().clone();
    let trace = ioc.trace().clone();
    CommandDef::new(
        "drvTetrAMMConfigure",
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
                optional: true,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
                optional: true,
            },
        ],
        "drvTetrAMMConfigure portName QEPortName [ringBufferSize] [maxMemory]",
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
            let max_memory = match args.get(3) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 100_000_000,
            };

            let rt = create_tetramm(&port_name, &qe_port_name, ring_buffer_size, max_memory)
                .map_err(|e| format!("drvTetrAMMConfigure: {e}"))?;

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

/// Register the TetrAMM configure command and the octet-port verbs it needs.
pub fn register(ioc: &mut AdIoc) {
    epics_rs::base::runtime::env::set_default("QUADEM", env!("CARGO_MANIFEST_DIR"));

    let trace = ioc.trace().clone();
    ioc.register_startup_command(epics_rs::asyn::iocsh::drv_asyn_ip_port_configure_command(
        trace.clone(),
    ));
    ioc.register_startup_command(
        epics_rs::asyn::iocsh::drv_asyn_serial_port_configure_command(trace),
    );
    ioc.register_startup_command(eos_command("asynOctetSetInputEos", true));
    ioc.register_startup_command(eos_command("asynOctetSetOutputEos", false));

    let runtime: Arc<std::sync::Mutex<Option<TetrAmmRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));
    let cmd = tetramm_configure_command(ioc, runtime.clone());
    ioc.register_startup_command(cmd);

    // Keep the runtime (read thread, callback thread, port actor) alive.
    ioc.keep_alive(runtime);
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
