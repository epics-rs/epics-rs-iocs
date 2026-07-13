//! C-runtime numeric parsing and write/read composition shared by the three
//! delaygen drivers.
//!
//! None of the C drivers embed a terminator in their command strings — every
//! `.cmd` startup fragment configures the input/output EOS on the underlying
//! octet port instead (`asynOctetSetInputEos`/`asynOctetSetOutputEos`), so
//! the port itself appends the output EOS on write and frames replies on the
//! input EOS on read. [`write_read`] therefore sends the bare command text
//! and does not append anything.

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::sync_io::SyncIOHandle;

/// asyn reason used for every octet transaction against the underlying
/// serial/IP port — that port has a single octet stream, so the reason
/// value is not meaningful to it (mirrors the precedent in
/// `motor_common::connect`/`motor-mclennan`).
const OCTET_REASON: usize = 0;

/// Send `cmd` and return the reply with the port's configured input EOS
/// already stripped (C `pasynOctetSyncIO->writeRead`, composed from the two
/// primitives asyn-rs exposes).
pub fn write_read(handle: &SyncIOHandle, cmd: &str) -> AsynResult<String> {
    handle.write_octet(OCTET_REASON, cmd.as_bytes())?;
    let raw = handle.read_octet(OCTET_REASON, 4096)?;
    Ok(String::from_utf8_lossy(&raw).into_owned())
}

/// Send `cmd` without reading a reply (C `pasynOctetSyncIO->write`).
pub fn write_only(handle: &SyncIOHandle, cmd: &str) -> AsynResult<()> {
    handle.write_octet(OCTET_REASON, cmd.as_bytes())?;
    Ok(())
}

/// Read a reply with no command sent first (used where a caller already
/// wrote and only needs to drain an extra line, e.g. Colby's serial echo).
pub fn read_only(handle: &SyncIOHandle) -> AsynResult<String> {
    let raw = handle.read_octet(OCTET_REASON, 4096)?;
    Ok(String::from_utf8_lossy(&raw).into_owned())
}

/// Mimic C `atoi`: parse the leading integer prefix, `0` on junk.
pub fn atoi(s: &str) -> i32 {
    let t = s.trim_start();
    let b = t.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    t.get(..i).and_then(|p| p.parse::<i32>().ok()).unwrap_or(0)
}

/// Extract the leading numeric token (sign, digits, optional `.digits`,
/// optional exponent) that C `atof`/`sscanf("%e", ...)` would consume,
/// shared by [`atof`] and [`atof_f32`] since they differ only in which
/// float width parses that token.
fn leading_float_token(s: &str) -> &str {
    let t = s.trim_start();
    let b = t.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    if i < b.len() && b[i] == b'.' {
        i += 1;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
    }
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let mut j = i + 1;
        if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
            j += 1;
        }
        let exp_start = j;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_start {
            i = j;
        }
    }
    t.get(..i).unwrap_or("")
}

/// Mimic C `atof`: parse the leading numeric prefix as `f64`, `0.0` on junk.
pub fn atof(s: &str) -> f64 {
    leading_float_token(s).parse::<f64>().unwrap_or(0.0)
}

/// Mimic C `sscanf(s, "%e", &float_var)`: parse the leading numeric prefix
/// directly into a 32-bit `float`, `0.0` on junk. Distinct from [`atof`] —
/// callers that store the result in a C `float` (single rounding,
/// ASCII-to-`f32`) must use this rather than `atof(s) as f32` (which would
/// round twice: ASCII-to-`f64`, then `f64`-to-`f32`).
pub fn atof_f32(s: &str) -> f32 {
    leading_float_token(s).parse::<f32>().unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn atoi_parses_leading_int_and_ignores_junk() {
        assert_eq!(atoi("42"), 42);
        assert_eq!(atoi("  -7abc"), -7);
        assert_eq!(atoi("abc"), 0);
        assert_eq!(atoi(""), 0);
    }

    #[test]
    fn atof_parses_leading_float_and_ignores_junk() {
        assert!((atof("3.25") - 3.25).abs() < 1e-12);
        assert!((atof("-1.5e-3 extra") - (-1.5e-3)).abs() < 1e-15);
        assert_eq!(atof("junk"), 0.0);
    }

    #[test]
    fn atof_f32_parses_scientific_notation_and_ignores_junk() {
        assert!((atof_f32("5.000000E-09") - 5E-9_f32).abs() < 1e-15);
        assert_eq!(atof_f32("junk"), 0.0);
    }
}
