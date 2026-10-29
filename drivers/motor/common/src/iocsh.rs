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
    }
}

/// An optional string argument.
pub fn arg_str_opt(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::String,
    }
}

/// A required integer argument.
pub fn arg_int_req(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Int,
    }
}

/// An optional integer argument.
pub fn arg_int_opt(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Int,
    }
}

/// A required double argument.
pub fn arg_double_req(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Double,
    }
}

/// An optional double argument.
pub fn arg_double_opt(name: &'static str) -> ArgDesc {
    ArgDesc {
        name,
        arg_type: ArgType::Double,
    }
}

/// Read a required string arg.
pub fn req_string(args: &[ArgValue], i: usize, name: &str) -> Result<String, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(s.to_str()?.to_owned()),
        _ => Err(format!("{name} must be a string")),
    }
}

/// Read an optional string arg: `Some` for a non-empty string, else `None`
/// (absent or `Missing`).
pub fn opt_string(args: &[ArgValue], i: usize) -> Result<Option<String>, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) if !s.is_empty() => Ok(Some(s.to_str()?.to_owned())),
        _ => Ok(None),
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

/// Read an integer arg; a missing one is 0, as C iocsh passes it
/// (iocsh.cpp:839-840).
pub fn req_int(args: &[ArgValue], i: usize, name: &str) -> Result<i64, String> {
    match args.get(i) {
        Some(ArgValue::Int(v)) => Ok(*v),
        None | Some(ArgValue::Missing) => Ok(0),
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
/// rejecting negative values.
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
        // 0 is C's "wait for a poll event only" (asynMotorController.cpp:633).
        Some(ArgValue::Int(v)) if *v >= 0 => Ok(*v as u64),
        Some(ArgValue::Int(_)) => Err(format!("{name} must not be negative")),
        None | Some(ArgValue::Missing) => Ok(default),
        _ => Err(format!("{name} must be an integer")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_missing_int_arg_is_c_s_0() {
        assert_eq!(req_int(&[ArgValue::Missing], 0, "n"), Ok(0));
        assert_eq!(req_int(&[], 0, "n"), Ok(0));
    }

    #[test]
    fn a_given_int_arg_is_its_value() {
        assert_eq!(req_int(&[ArgValue::Int(-3)], 0, "n"), Ok(-3));
    }
}
