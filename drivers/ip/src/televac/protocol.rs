//! What `devTelevac.c` puts on the wire and takes off it.
//!
//! Commands are bare ASCII; the octet port's EOS frames both directions:
//!
//! ```text
//! R<n>     a station's pressure
//! SP<n>N   a relay's ON pressure
//! SP<n>F   a relay's OFF pressure
//! RY       the relay states, as hex
//! ```

use crate::scan::Scan;
use crate::user_param::c_str;

/// `devTelevac.h`'s command numbers, by `televacCommands[]`'s names
/// (`devTelevac.c:47-52`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    GetPressure = 0,
    GetRelayOn = 1,
    GetRelayOff = 2,
    GetRelayStatus = 3,
}

impl Command {
    const NAMES: [(Self, &'static str); 4] = [
        (Self::GetPressure, "GET_PRESSURE"),
        (Self::GetRelayOn, "GET_RELAYON"),
        (Self::GetRelayOff, "GET_RELAYOFF"),
        (Self::GetRelayStatus, "GET_RELAYSTATUS"),
    ];

    /// `epicsStrCaseCmp` against each name.
    pub fn from_name(name: &str) -> Option<Self> {
        Self::NAMES
            .iter()
            .find(|(_, n)| n.eq_ignore_ascii_case(name))
            .map(|(c, _)| *c)
    }

    pub fn number(self) -> i32 {
        self as i32
    }

    /// The `pact == 0` `sendBuf` of `readAi` (`ai`) or `readBi`; `None` is
    /// their "Wrong record type" arm.
    pub fn request(self, ai: bool, parameter: i32) -> Option<String> {
        match (ai, self) {
            (true, Self::GetPressure) => Some(format!("R{parameter}")),
            (true, Self::GetRelayOn) => Some(format!("SP{parameter}N")),
            (true, Self::GetRelayOff) => Some(format!("SP{parameter}F")),
            (false, Self::GetRelayStatus) => Some("RY".to_owned()),
            _ => None,
        }
    }
}

/// `readAi`'s decode of a pressure reply (`devTelevac.c:244-265`): `n=n.nn-eT`
/// when it is 9 characters, `n.nn-e` / `n.n-e` when 6 or 5, anything else
/// 9.9e+9, as is the over-range exponent `B`. The mantissa is a C `float`.
///
/// C leaves `sign` and `exp` uninitialised when the reply stops short of
/// them; they read `+` and 0 here.
pub fn pressure(rec_buf: &[u8]) -> f64 {
    let buf = c_str(rec_buf);
    let over_range = (9.9f32, b'+', 9);
    // `%f%c%x`, each conversion landing until the first that fails.
    let tail = |scan: &mut Scan| {
        let value = scan.float().unwrap_or(0.0);
        (value, scan.char().unwrap_or(b'+'), scan.hex().unwrap_or(0))
    };
    let mut scan = Scan::new(buf);
    let (mut value, mut sign, mut exp) = match buf.len() {
        9 => {
            scan.int();
            tail(scan.literal(b'='))
        }
        5 | 6 => tail(&mut scan),
        _ => over_range,
    };
    if exp == 11 {
        (value, sign, exp) = over_range;
    }
    if sign == b'-' {
        exp = -exp;
    }
    f64::from(value) * 10f64.powf(f64::from(exp))
}

/// `readBi`'s RVAL for `relay` (1-based) from the `RY` reply
/// (`devTelevac.c:310-317`). C's second `'n'` test rewrites `recBuf[0]`, not
/// `recBuf[1]`, so a low digit `n` clobbered the high one; both digits are
/// rewritten in place here. A relay outside 1..32 is an undefined C shift and
/// selects no bit.
pub fn relay_rval(rec_buf: &[u8], relay: i32) -> u32 {
    let mut buf = c_str(rec_buf).to_vec();
    for digit in buf.iter_mut().take(2) {
        if *digit == b'n' {
            *digit = b'0';
        }
    }
    let value = Scan::new(&buf).hex().unwrap_or(0) as u32;
    let mask = u32::try_from(relay - 1)
        .ok()
        .and_then(|shift| 1u32.checked_shl(shift))
        .unwrap_or(0);
    value & mask
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requests_are_the_c_send_buffers() {
        use Command::*;
        assert_eq!(GetPressure.request(true, 3).as_deref(), Some("R3"));
        assert_eq!(GetRelayOn.request(true, 2).as_deref(), Some("SP2N"));
        assert_eq!(GetRelayOff.request(true, 2).as_deref(), Some("SP2F"));
        assert_eq!(GetRelayStatus.request(false, 2).as_deref(), Some("RY"));
        assert_eq!(GetRelayStatus.request(true, 2), None);
        assert_eq!(GetPressure.request(false, 1), None);
        assert_eq!(Command::from_name("get_relayon"), Some(GetRelayOn));
        assert_eq!(Command::from_name("GET_FOO"), None);
    }

    #[test]
    fn pressure_replies_decode_as_read_ai_does() {
        let near = |raw: &[u8], want: f64| {
            let got = pressure(raw);
            assert!((got - want).abs() <= want.abs() * 1e-12, "{raw:?}: {got}");
        };
        near(b"1=7.60-2T", f64::from(7.6f32) * 1e-2);
        near(b"7.60-2", f64::from(7.6f32) * 1e-2);
        near(b"7.6-2", f64::from(7.6f32) * 1e-2);
        near(b"1.00+3", 1e3);
        let over = f64::from(9.9f32) * 1e9;
        near(b"9.90-B", over);
        near(b"7.600-2", over);
        near(b"", over);
        assert_eq!(pressure(b"abcdef"), 0.0);
    }

    #[test]
    fn relay_rval_is_the_relay_s_bit() {
        assert_eq!(relay_rval(b"0F", 1), 1);
        assert_eq!(relay_rval(b"0F", 4), 8);
        assert_eq!(relay_rval(b"0F", 5), 0);
        // C rewrote the high digit for a low `n`, and read 0.
        assert_eq!(relay_rval(b"1n", 5), 0x10);
        assert_eq!(relay_rval(b"nn", 1), 0);
        assert_eq!(relay_rval(b"0F", 0), 0);
    }
}
