//! Argument-parsing helpers for the vendor `*CreateController`/`*CreateAxis`
//! iocsh commands. These mirror the small readers the C `<driver>Config`
//! functions perform on their positional arguments, with defaulting and
//! type checks appropriate to `iocsh`.

use epics_rs::base::server::iocsh::registry::{ArgDesc, ArgType, ArgValue};

/// Default poll intervals when a create command omits the trailing
/// `movingPollMs`/`idlePollMs` args.
pub const DEFAULT_MOVING_POLL_MS: u64 = 100;
pub const DEFAULT_IDLE_POLL_MS: u64 = 1000;

/// A required string argument.
pub fn arg_str_req(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::String,
        optional: false,
    }
}

/// An optional string argument.
pub fn arg_str_opt(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::String,
        optional: true,
    }
}

/// A required integer argument.
pub fn arg_int_req(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Int,
        optional: false,
    }
}

/// An optional integer argument.
pub fn arg_int_opt(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Int,
        optional: true,
    }
}

/// A required double argument.
pub fn arg_double_req(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Double,
        optional: false,
    }
}

/// An optional double argument.
pub fn arg_double_opt(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Double,
        optional: true,
    }
}

/// Read a required string arg.
pub fn req_string(args: &[ArgValue], i: usize, name: &str) -> Result<String, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(s.clone()),
        _ => Err(format!("{name} must be a string")),
    }
}

/// Read an optional string arg: `Some` for a non-empty string, else `None`
/// (absent or `Missing`).
pub fn opt_string(args: &[ArgValue], i: usize) -> Option<String> {
    match args.get(i) {
        Some(ArgValue::String(s)) if !s.is_empty() => Some(s.clone()),
        _ => None,
    }
}

/// Read a required double arg.
pub fn req_double(args: &[ArgValue], i: usize, name: &str) -> Result<f64, String> {
    match args.get(i) {
        Some(ArgValue::Double(v)) => Ok(*v),
        _ => Err(format!("{name} must be a number")),
    }
}

/// Read an optional double arg, defaulting when absent.
pub fn opt_double(args: &[ArgValue], i: usize, default: f64, name: &str) -> Result<f64, String> {
    match args.get(i) {
        Some(ArgValue::Double(v)) => Ok(*v),
        None | Some(ArgValue::Missing) => Ok(default),
        _ => Err(format!("{name} must be a number")),
    }
}

/// Read a required integer arg.
pub fn req_int(args: &[ArgValue], i: usize, name: &str) -> Result<i64, String> {
    match args.get(i) {
        Some(ArgValue::Int(v)) => Ok(*v),
        _ => Err(format!("{name} must be an integer")),
    }
}

/// Read an optional integer arg, defaulting when absent.
pub fn opt_int(args: &[ArgValue], i: usize, default: i64, name: &str) -> Result<i64, String> {
    match args.get(i) {
        Some(ArgValue::Int(v)) => Ok(*v),
        None | Some(ArgValue::Missing) => Ok(default),
        _ => Err(format!("{name} must be an integer")),
    }
}

/// Parse the optional `movingPollMs`/`idlePollMs` trailing args, defaulting and
/// rejecting non-positive values.
pub fn poll_intervals(
    args: &[ArgValue],
    moving_i: usize,
    idle_i: usize,
) -> Result<(u64, u64), String> {
    let moving = poll_ms(args.get(moving_i), DEFAULT_MOVING_POLL_MS, "movingPollMs")?;
    let idle = poll_ms(args.get(idle_i), DEFAULT_IDLE_POLL_MS, "idlePollMs")?;
    Ok((moving, idle))
}

fn poll_ms(arg: Option<&ArgValue>, default: u64, name: &str) -> Result<u64, String> {
    match arg {
        Some(ArgValue::Int(v)) if *v > 0 => Ok(*v as u64),
        Some(ArgValue::Int(_)) => Err(format!("{name} must be positive")),
        None | Some(ArgValue::Missing) => Ok(default),
        _ => Err(format!("{name} must be an integer")),
    }
}
