//! C-runtime numeric parsing and write/read composition shared by the three
//! delaygen drivers.
//!
//! None of the C drivers embed a terminator in their command strings — every
//! `.cmd` startup fragment configures the input/output EOS on the underlying
//! octet port instead (`asynOctetSetInputEos`/`asynOctetSetOutputEos`), so
//! the port itself appends the output EOS on write and frames replies on the
//! input EOS on read. [`write_read`] therefore sends the bare command text
//! and does not append anything.

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};

use crate::connect::OctetLink;

/// asyn reason used for every octet transaction against the underlying
/// serial/IP port — that port has a single octet stream, so the reason
/// value is not meaningful to it (mirrors the precedent in
/// `motor_common::connect`/`motor-mclennan`).
const OCTET_REASON: usize = 0;

/// None of the C delaygen drivers writes the record's
/// `pasynUser->errorMessage`, so a failed transfer reaches the record as its
/// status alone.
pub(crate) fn record_error(status: AsynStatus) -> AsynError {
    AsynError::Status {
        status,
        message: String::new(),
    }
}

/// C's `if( pport->init==0 ) return asynError;` at the top of every
/// DG645 and Coherent SDG transfer: a port whose constructor gave up after
/// `registerPort` stays registered and refuses all I/O.
pub(crate) fn require_init(init: bool) -> AsynResult<()> {
    if init {
        Ok(())
    } else {
        Err(record_error(AsynStatus::Error))
    }
}

/// A failed C `pasynOctetSyncIO` transaction with the byte counts the C
/// drivers' error lines print.
#[derive(Debug)]
pub struct OctetFailure {
    pub error: AsynError,
    pub sent: usize,
    pub received: usize,
}

impl OctetFailure {
    /// The C `asynStatus` number the error lines print.
    pub fn status(&self) -> i32 {
        self.error.status() as i32
    }
}

impl From<OctetFailure> for AsynError {
    fn from(f: OctetFailure) -> Self {
        f.error
    }
}

/// Write `cmd`; any write that sent fewer bytes than `cmd` is `asynError`,
/// as every C driver turns `nActual != nRequested` into one — including a
/// refused `queueLockPort`, whose own status (e.g. asynDisconnected) C drops.
fn write_all(link: &OctetLink, cmd: &str) -> Result<usize, OctetFailure> {
    let short = |n: usize, message: String| OctetFailure {
        error: AsynError::Status {
            status: AsynStatus::Error,
            message,
        },
        sent: n,
        received: 0,
    };
    match link.sync.write_octet(OCTET_REASON, cmd.as_bytes()) {
        Ok(n) if n == cmd.len() => Ok(n),
        Ok(n) => Err(short(n, format!("wrote {n} of {} bytes", cmd.len()))),
        Err(error) if cmd.is_empty() => Err(OctetFailure {
            error,
            sent: 0,
            received: 0,
        }),
        Err(error) => Err(short(0, error.message())),
    }
}

/// Send `cmd` and return the reply with the port's configured input EOS
/// already stripped (C `pasynOctetSyncIO->writeRead`, composed from the two
/// primitives asyn-rs exposes).
pub fn write_read(link: &OctetLink, cmd: &str) -> Result<String, OctetFailure> {
    write_read_bytes(link, cmd).map(|raw| String::from_utf8_lossy(&raw).into_owned())
}

/// [`write_read`]'s reply as the device sent it.
pub fn write_read_bytes(link: &OctetLink, cmd: &str) -> Result<Vec<u8>, OctetFailure> {
    let (raw, status) = exchange(link, cmd);
    status.map(|()| raw)
}

/// [`write_read_bytes`] with the bytes a failed read delivered, which C's
/// buffer holds; a failed write reads none.
pub fn exchange(link: &OctetLink, cmd: &str) -> (Vec<u8>, Result<(), OctetFailure>) {
    let sent = match write_all(link, cmd) {
        Ok(sent) => sent,
        Err(f) => return (Vec::new(), Err(f)),
    };
    let (raw, status) = link.sync.read_octet(OCTET_REASON, 4096).into_parts();
    let status = status.map_err(|error| OctetFailure {
        received: raw.len(),
        error,
        sent,
    });
    (raw, status)
}

/// Send `cmd` without reading a reply (C `pasynOctetSyncIO->write`).
pub fn write_only(link: &OctetLink, cmd: &str) -> Result<(), OctetFailure> {
    write_all(link, cmd).map(drop)
}

/// Read a reply with no command sent first (used where a caller already
/// wrote and only needs to drain an extra line, e.g. Colby's serial echo).
pub fn read_only(link: &OctetLink) -> AsynResult<String> {
    let (raw, status) = link.sync.read_octet(OCTET_REASON, 4096).into_parts();
    status?;
    Ok(String::from_utf8_lossy(&raw).into_owned())
}

pub use epics_rs::base::types::c_parse::atoi;

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
    fn atoi_wraps_an_overflow_as_glibc_does() {
        // glibc's atoi is `(int) strtol(s, NULL, 10)`.
        use std::ffi::c_long;
        let past_int = if c_long::BITS == 64 {
            i32::MIN
        } else {
            i32::MAX
        };
        assert_eq!(atoi("2147483648"), past_int);
        assert_eq!(atoi("99999999999999999999"), c_long::MAX as i32);
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
