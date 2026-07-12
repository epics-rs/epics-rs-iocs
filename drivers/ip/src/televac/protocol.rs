//! Televac vacuum-gauge-controller protocol (`devTelevac.c`).
//!
//! Commands are bare ASCII, framed by the octet port's EOS:
//!
//! ```text
//! R<station>    read the pressure of a station (1..9)
//! SP<relay>N    read a relay's ON pressure
//! SP<relay>F    read a relay's OFF pressure
//! RY            read the relay states as a hex bitmask
//! ```
//!
//! A pressure reply is either `n=n.nn-e` (station echoed) or `n.nn-e`; the
//! exponent digit is hexadecimal, and the over-range marker is `B` = 11
//! (`devTelevac.c:246-261`).

/// Stations a controller can carry (`parameter` 1..9 in the C link).
pub const MAX_STATIONS: usize = 9;

/// Relays a controller can carry (`parameter` 1..8 in the C link).
pub const MAX_RELAYS: usize = 8;

/// The value the C substitutes when a reply is unparseable or over-range:
/// 9.9e+9 (`devTelevac.c:252-261`).
pub const OVER_RANGE: f64 = 9.9e9;

/// The exponent digit that means over-range (`B` read as hex = 11).
const OVER_RANGE_EXPONENT: u32 = 11;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum TelevacError {
    #[error("cannot parse {field} from {raw:?}")]
    Parse { field: &'static str, raw: String },
}

pub fn read_pressure(station: u8) -> String {
    format!("R{station}")
}

pub fn read_relay_on(relay: u8) -> String {
    format!("SP{relay}N")
}

pub fn read_relay_off(relay: u8) -> String {
    format!("SP{relay}F")
}

pub fn read_relay_states() -> String {
    "RY".to_string()
}

/// Decode a pressure reply.
///
/// `n=n.nn-e` (9 bytes) or `n.nn-e` / `n.n-e` (6 or 5 bytes); anything else is
/// the over-range value, as in the C. The mantissa is decimal, the exponent one
/// hex digit, and `-` before it negates the exponent.
pub fn parse_pressure(raw: &str) -> f64 {
    let body = raw.trim();
    let body = match body.split_once('=') {
        Some((_station, rest)) => rest,
        None => body,
    };
    let bytes = body.as_bytes();
    // The mantissa runs up to the sign character that introduces the exponent.
    let Some(sign_at) = bytes.iter().position(|&b| b == b'-' || b == b'+') else {
        return OVER_RANGE;
    };
    let (Ok(mantissa), Some(exponent)) = (
        body[..sign_at].parse::<f64>(),
        body[sign_at + 1..]
            .chars()
            .next()
            .and_then(|c| c.to_digit(16)),
    ) else {
        return OVER_RANGE;
    };
    if exponent == OVER_RANGE_EXPONENT {
        return OVER_RANGE;
    }
    let exponent = if bytes[sign_at] == b'-' {
        -(exponent as i32)
    } else {
        exponent as i32
    };
    mantissa * 10f64.powi(exponent)
}

/// Decode the `RY` relay bitmask.
///
/// The reply is two hex digits; the C rewrote an `n` digit to `0` before the
/// hex conversion, but its second test wrote to `recBuf[0]` again instead of
/// `recBuf[1]` (`devTelevac.c:312-313`), so an `n` in the low nibble was left
/// in place and the whole byte failed to convert. Both digits are sanitised
/// here.
pub fn parse_relay_states(raw: &str) -> Result<u32, TelevacError> {
    let sanitised: String = raw
        .trim()
        .chars()
        .map(|c| if c == 'n' { '0' } else { c })
        .collect();
    u32::from_str_radix(&sanitised, 16).map_err(|_| TelevacError::Parse {
        field: "relay states",
        raw: raw.to_string(),
    })
}

/// Is `relay` (1-based) closed in the `RY` bitmask (`devTelevac.c:316`)?
pub fn relay_is_set(states: u32, relay: u8) -> bool {
    states & (1 << (relay - 1)) != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commands_match_the_c_strings() {
        assert_eq!(read_pressure(3), "R3");
        assert_eq!(read_relay_on(2), "SP2N");
        assert_eq!(read_relay_off(2), "SP2F");
        assert_eq!(read_relay_states(), "RY");
    }

    #[test]
    fn pressure_with_the_station_echoed() {
        // "1=7.60-2T" -> 7.60e-2
        assert!((parse_pressure("1=7.60-2T") - 7.6e-2).abs() < 1e-12);
    }

    #[test]
    fn pressure_without_the_station() {
        assert!((parse_pressure("7.60-2") - 7.6e-2).abs() < 1e-12);
        assert!((parse_pressure("7.6-2") - 7.6e-2).abs() < 1e-12);
        assert!((parse_pressure("1.00+3") - 1.0e3).abs() < 1e-9);
    }

    #[test]
    fn over_range_and_garbage_give_the_c_sentinel() {
        // Exponent 'B' = 11 is the over-range marker.
        assert_eq!(parse_pressure("9.90-B"), OVER_RANGE);
        assert_eq!(parse_pressure("junk"), OVER_RANGE);
        assert_eq!(parse_pressure(""), OVER_RANGE);
    }

    #[test]
    fn relay_states_sanitise_both_nibbles() {
        assert_eq!(parse_relay_states("0F").unwrap(), 0x0f);
        // The C only fixed the high nibble, so "1n" never converted.
        assert_eq!(parse_relay_states("1n").unwrap(), 0x10);
        assert_eq!(parse_relay_states("nn").unwrap(), 0);
        assert!(parse_relay_states("zz").is_err());
    }

    #[test]
    fn relay_bits_are_one_based() {
        let states = 0b0000_0101;
        assert!(relay_is_set(states, 1));
        assert!(!relay_is_set(states, 2));
        assert!(relay_is_set(states, 3));
        assert!(!relay_is_set(states, 8));
    }
}
