//! XPS-C8 controller RPC transport: ASCII commands over a TCP octet port.
//!
//! Faithful port of the EPICS build's `asynOctetSocket.cpp` shim plus the
//! `sprintf`/`sscanf` marshalling in `XPS_C8_drivers.cpp`. Each RPC call sends
//! a `FuncName (arg,arg,...)` string and the controller replies
//! `errorCode,value,value,...,EndOfAPI`. The first field is the integer error
//! code (`0` = OK, `< 0` = NOK); the remaining comma-separated fields are the
//! out-parameters in declaration order.
//!
//! # Sockets
//!
//! The XPS driver opens several TCP connections to the same controller
//! (`XPSController.cpp` / `XPSAxis.cpp`):
//!
//! - a shared **poll socket** (positive timeout, [`XpsSocket::query`]) used by
//!   the controller poll and every axis for reads;
//! - one **move socket per axis** (negative timeout, [`XpsSocket::fire`]) used
//!   for the actual `GroupMove*` / `GroupHomeSearch` commands. The controller
//!   does not wait for the move to finish; completion is observed by polling
//!   `GroupStatusGet` on the poll socket.
//!
//! # Framing
//!
//! Like `asynOctetSocket.cpp` (which configures the port with `noProcessEos`),
//! [`XpsSocket::query`] accumulates reads until the reply ends with
//! [`XPS_TERMINATOR`] and strips it. Framing is therefore self-contained — the
//! port needs no input-EOS configuration, so the IOC `st.cmd` only has to
//! register a plain `drvAsynIPPort` per socket.

use std::fmt;

use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;

use crate::util::{atof, atoi};

/// XPS reply terminator (`asynOctetSocket.cpp:40 XPS_TERMINATOR`). Configure it
/// as the port's asyn input EOS so framed reads return one reply each.
pub const XPS_TERMINATOR: &str = ",EndOfAPI";

/// Octet reason index for the single-device TCP port (C parity: reason/addr 0).
const OCTET_REASON: usize = 0;

/// Per-read buffer size (C `SIZE_SMALL`, the `asynOctetSocket` default).
const READ_BUF: usize = 1024;

/// Cap on an accumulated reply, to bound a controller that never sends the
/// terminator (C `SIZE_HUGE`).
const MAX_REPLY: usize = 65536;

/// Fire-and-forget move retries (C `asynOctetSocket.cpp:42 MAX_RETRIES`).
const MOVE_MAX_RETRIES: usize = 2;

/// An XPS RPC failure: a transport error, or a nonzero controller error code
/// (C convention: a return value `< 0` means NOK).
#[derive(Debug)]
pub enum XpsError {
    /// Transport-level failure talking to the port.
    Transport(AsynError),
    /// The controller returned this nonzero error code.
    Api(i32),
}

impl fmt::Display for XpsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            XpsError::Transport(e) => write!(f, "XPS transport error: {e}"),
            XpsError::Api(code) => write!(f, "XPS API error {code}"),
        }
    }
}

impl std::error::Error for XpsError {}

impl From<AsynError> for XpsError {
    fn from(e: AsynError) -> Self {
        XpsError::Transport(e)
    }
}

/// Convert back into an `AsynError` so RPC calls can be used with `?` inside
/// `AsynMotor` methods (which return `AsynResult`).
impl From<XpsError> for AsynError {
    fn from(e: XpsError) -> Self {
        match e {
            XpsError::Transport(a) => a,
            XpsError::Api(code) => AsynError::Status {
                status: AsynStatus::Error,
                message: format!("XPS API error {code}"),
            },
        }
    }
}

/// Result of an XPS RPC call.
pub type XpsResult<T> = Result<T, XpsError>;

/// A raw XPS reply, `errorCode,value,value,...` with the `,EndOfAPI` terminator
/// already stripped by the port's input EOS. Field 0 is the error code;
/// subsequent comma-separated fields are the out-parameters in declaration
/// order (C walks them with `strchr(pt, ',')`).
pub struct XpsReply(String);

impl XpsReply {
    /// The leading error code (C `sscanf(reply, "%i", &ret)`); `0` = OK. An
    /// empty reply yields `0` (C leaves `ret` at its `-1` init, but for our
    /// EOS-framed transport an empty reply cannot occur; `atoi("") == 0`).
    pub fn code(&self) -> i32 {
        atoi(&self.0)
    }

    /// Require a success code, returning `self` for out-param extraction or
    /// `Err(Api(code))` on any nonzero code.
    pub fn require_ok(self) -> XpsResult<Self> {
        match self.code() {
            0 => Ok(self),
            code => Err(XpsError::Api(code)),
        }
    }

    /// The nth comma-separated field (`0` = error code, `1` = first out-param).
    /// C string out-params take everything up to the next comma; for XPS values
    /// (numbers, and status strings that never contain commas) that equals this
    /// split.
    fn field(&self, n: usize) -> Option<&str> {
        self.0.split(',').nth(n)
    }

    /// Out-param `n` (`1` = first value after the code) as `f64`, C `atof`.
    pub fn double(&self, n: usize) -> f64 {
        self.field(n).map(atof).unwrap_or(0.0)
    }

    /// Out-param `n` as `i32`, C `atoi`.
    pub fn int(&self, n: usize) -> i32 {
        self.field(n).map(atoi).unwrap_or(0)
    }

    /// Out-param `n` as a raw 32-bit bitmask. Parses the field as a (possibly
    /// `> i32::MAX` or negative) decimal and truncates to the low 32 bits,
    /// matching C reading an unsigned XPS error code into a signed `int`
    /// (`PositionerErrorGet`; the end-of-run masks set bit 31).
    pub fn bits(&self, n: usize) -> u32 {
        self.field(n).map(atoi64).unwrap_or(0) as u32
    }

    /// Out-param `n` as a string slice (C `strcpy` up to the next comma).
    pub fn string(&self, n: usize) -> &str {
        self.field(n).unwrap_or("")
    }
}

/// How an [`XpsSocket`] sends commands, mirroring the sign of the C socket's
/// timeout (`asynOctetSocket.cpp`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SocketMode {
    /// Positive timeout: write, wait for the full `,EndOfAPI`-framed reply.
    /// The shared poll socket and all reads use this.
    Query,
    /// Negative timeout: write and do not wait for completion (per-axis move
    /// socket). A read timeout counts as success.
    Fire,
}

/// One TCP connection to the XPS controller, wrapping a [`SyncIOHandle`] bound
/// to a `drvAsynIPPort`. Construct one per XPS socket: a shared [`Query`] poll
/// socket, and one [`Fire`] move socket per axis.
///
/// [`Query`]: SocketMode::Query
/// [`Fire`]: SocketMode::Fire
pub struct XpsSocket {
    handle: SyncIOHandle,
    mode: SocketMode,
}

impl XpsSocket {
    /// Wrap an already-connected octet handle with the given send mode.
    pub fn new(handle: SyncIOHandle, mode: SocketMode) -> Self {
        Self { handle, mode }
    }

    /// Send a marshalled command and return its reply. The socket's [`mode`]
    /// selects the C `SendAndReceive` path: [`Query`] waits for the framed
    /// reply; [`Fire`] writes without waiting and synthesizes a `"0"` success
    /// reply (so callers can uniformly `require_ok()`). Move completion on a
    /// [`Fire`] socket is observed later by polling `GroupStatusGet`.
    ///
    /// [`mode`]: XpsSocket::mode
    /// [`Query`]: SocketMode::Query
    /// [`Fire`]: SocketMode::Fire
    pub fn exec(&self, cmd: &str) -> XpsResult<XpsReply> {
        match self.mode {
            SocketMode::Query => self.query(cmd),
            SocketMode::Fire => {
                self.fire(cmd)?;
                Ok(XpsReply("0".to_string()))
            }
        }
    }

    /// This socket's send mode.
    pub fn mode(&self) -> SocketMode {
        self.mode
    }

    /// Query path — C `SendAndReceive` with positive timeout. Write the command,
    /// then accumulate reads until the reply ends with [`XPS_TERMINATOR`]
    /// (`asynOctetSocket.cpp` uses `noProcessEos` and loops the same way), and
    /// return it with the terminator stripped. Self-contained: it does not rely
    /// on any port-level input EOS.
    fn query(&self, cmd: &str) -> XpsResult<XpsReply> {
        self.handle.write_octet(OCTET_REASON, cmd.as_bytes())?;
        let mut buf = Vec::new();
        loop {
            let chunk = self.handle.read_octet(OCTET_REASON, READ_BUF)?;
            buf.extend_from_slice(&chunk);
            if buf.ends_with(XPS_TERMINATOR.as_bytes()) {
                break;
            }
            if buf.len() > MAX_REPLY {
                return Err(XpsError::Transport(AsynError::Status {
                    status: AsynStatus::Overflow,
                    message: "XPS reply exceeded maximum length without terminator".into(),
                }));
            }
        }
        let text = String::from_utf8_lossy(&buf).into_owned();
        let text = text
            .strip_suffix(XPS_TERMINATOR)
            .map(str::to_string)
            .unwrap_or(text);
        Ok(XpsReply(text))
    }

    /// Fire-and-forget path — C `SendAndReceive` with negative timeout on the
    /// per-axis move socket. Write the command and do not wait for the move to
    /// finish: a read timeout is success (C fakes `"0"`), a `-1` reply
    /// ("previous command not complete") is retried up to [`MOVE_MAX_RETRIES`],
    /// and any other reply's error code is honored.
    ///
    /// The move handle's per-call timeout should be short (C sets `-0.1` s on
    /// the move socket) so the write returns promptly.
    fn fire(&self, cmd: &str) -> XpsResult<()> {
        for _ in 0..MOVE_MAX_RETRIES {
            self.handle.write_octet(OCTET_REASON, cmd.as_bytes())?;
            match self.handle.read_octet(OCTET_REASON, READ_BUF) {
                Err(AsynError::Status {
                    status: AsynStatus::Timeout,
                    ..
                }) => return Ok(()),
                Err(e) => return Err(XpsError::Transport(e)),
                Ok(raw) => {
                    let reply = XpsReply(String::from_utf8_lossy(&raw).into_owned());
                    match reply.code() {
                        0 => return Ok(()),
                        -1 => continue,
                        code => return Err(XpsError::Api(code)),
                    }
                }
            }
        }
        // C: after MAX_RETRIES with no clean response, forces "0" (success).
        Ok(())
    }
}

/// Format `value` like C `printf("%.*g", precision, value)`, used to marshal
/// doubles into XPS command strings (the vendor library uses `%.13g`), so the
/// wire bytes match. C `%g` rules: let `p` be the precision (min 1) and `x` the
/// decimal exponent; use `%e` with precision `p-1` when `x < -4 || x >= p`,
/// else `%f` with precision `p-1-x`; then strip trailing fractional zeros and a
/// trailing decimal point.
pub(crate) fn format_g(value: f64, precision: usize) -> String {
    let p = precision.max(1);

    if value == 0.0 {
        return "0".to_string();
    }
    if value.is_nan() {
        return "nan".to_string();
    }
    if value.is_infinite() {
        return if value < 0.0 { "-inf" } else { "inf" }.to_string();
    }

    // Extract the rounded decimal exponent via a scientific formatting at the
    // target precision (rounding may bump the exponent, so read it back).
    let sci = format!("{:.*e}", p - 1, value);
    let exp: i32 = sci
        .rsplit_once('e')
        .and_then(|(_, e)| e.parse().ok())
        .unwrap_or(0);

    if exp < -4 || exp >= p as i32 {
        // Scientific style, %e with precision p-1, mantissa zero-stripped.
        let (mantissa, _) = sci.split_once('e').unwrap_or((sci.as_str(), ""));
        let mantissa = strip_trailing_zeros(mantissa);
        format!("{mantissa}e{}", format_exponent(exp))
    } else {
        // Fixed style, %f with precision p-1-exp, zero-stripped.
        let frac_digits = (p as i32 - 1 - exp).max(0) as usize;
        let fixed = format!("{value:.frac_digits$}");
        strip_trailing_zeros(&fixed).to_string()
    }
}

/// Strip trailing fractional zeros and a dangling decimal point from a decimal
/// string (no exponent). Leaves integers untouched.
fn strip_trailing_zeros(s: &str) -> &str {
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.')
    } else {
        s
    }
}

/// Format an exponent C-style: always a sign, at least two digits
/// (`e+05`, `e-13`, `e+100`).
fn format_exponent(exp: i32) -> String {
    let sign = if exp < 0 { '-' } else { '+' };
    format!("{sign}{:02}", exp.abs())
}

/// C `atoi` widened to `i64`: parse the leading (optionally signed) integer
/// prefix, `0` on junk. Used for 32-bit bitmasks that exceed `i32::MAX`.
fn atoi64(s: &str) -> i64 {
    let t = s.trim_start();
    let b = t.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    t.get(..i).and_then(|p| p.parse::<i64>().ok()).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reply_parses_code_and_fields() {
        let r = XpsReply("0,42,-0.1234,Ready".to_string());
        assert_eq!(r.code(), 0);
        assert_eq!(r.int(1), 42);
        assert_eq!(r.double(2), -0.1234);
        assert_eq!(r.string(3), "Ready");
        // Missing field → defaults.
        assert_eq!(r.double(9), 0.0);
        assert_eq!(r.string(9), "");
    }

    #[test]
    fn reply_error_code_negative() {
        let r = XpsReply("-17".to_string());
        assert_eq!(r.code(), -17);
        assert!(matches!(r.require_ok(), Err(XpsError::Api(-17))));
    }

    #[test]
    fn reply_require_ok_passes_through() {
        let r = XpsReply("0,1.5".to_string());
        let r = r.require_ok().expect("code 0");
        assert_eq!(r.double(1), 1.5);
    }

    #[test]
    fn format_g_matches_c_printf() {
        // Reference values from C printf("%.13g", x).
        assert_eq!(format_g(0.0, 13), "0");
        assert_eq!(format_g(50.0, 13), "50");
        assert_eq!(format_g(-50.0, 13), "-50");
        assert_eq!(format_g(0.1, 13), "0.1");
        assert_eq!(format_g(-0.5, 13), "-0.5");
        assert_eq!(format_g(1.5e-3, 13), "0.0015");
        assert_eq!(format_g(1234567890123.0, 13), "1234567890123");
        // exp -5 < -4 → scientific.
        assert_eq!(format_g(1e-5, 13), "1e-05");
        // exp 20 >= 13 → scientific.
        assert_eq!(format_g(1e20, 13), "1e+20");
        assert_eq!(format_g(1.25, 13), "1.25");
    }

    #[test]
    fn format_g_rounds_to_precision() {
        // 1/3 at 13 significant digits.
        assert_eq!(format_g(1.0 / 3.0, 13), "0.3333333333333");
    }
}
