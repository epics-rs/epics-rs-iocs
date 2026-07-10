//! iocsh verbs every quadEM IOC needs to build the octet port its driver
//! talks through.
//!
//! `drvAsynIPPortConfigure` and `drvAsynSerialPortConfigure` come from
//! `asyn-rs`. `asynOctetSetInputEos` / `asynOctetSetOutputEos` are provided
//! here instead: the `asyn-rs` versions resolve a port name through a
//! `PortManager`, which the port-configure verbs do not populate — they
//! register into the `asyn_record` registry, which is what these look in.

use std::sync::Arc;

use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::server::iocsh::registry::{
    ArgDesc, ArgType, ArgValue, CommandContext, CommandDef, CommandOutcome,
};

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
