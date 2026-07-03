//! SMC100 serial ASCII protocol: command formatting and response parsing.
//!
//! Pure functions with no I/O and no asyn-rs dependency, so the wire format is
//! unit-testable without hardware. Ported byte-for-byte from
//! `motorNewport/newportApp/src/SMC100Driver.cpp` (`SMC100Axis`).
//!
//! All commands are prefixed with the 1-based controller axis number (the C
//! driver sends `axisNo_ + 1`). The controller line terminator (CR/LF) is
//! appended by the asyn serial port's output EOS, not by these functions —
//! exactly as the C `sprintf(outString_, …)` writes a bare command that
//! `pasynOctetSyncIO` terminates.
//!
//! Numeric fields use C `%f` formatting (fixed 6 decimals) so the wire bytes
//! match the reference driver.

/// `<axis>VA<velocity>` — set moving velocity (controller EGU/sec).
pub fn cmd_set_velocity(axis: u8, velocity_egu: f64) -> String {
    format!("{axis}VA{velocity_egu:.6}")
}

/// `<axis>AC<acceleration>` — set acceleration (controller EGU/sec²).
pub fn cmd_set_acceleration(axis: u8, acceleration_egu: f64) -> String {
    format!("{axis}AC{acceleration_egu:.6}")
}

/// `<axis>PA<position>` — move to an absolute position (controller EGU).
pub fn cmd_move_absolute(axis: u8, position_egu: f64) -> String {
    format!("{axis}PA{position_egu:.6}")
}

/// `<axis>PR<distance>` — move a relative distance (controller EGU).
pub fn cmd_move_relative(axis: u8, distance_egu: f64) -> String {
    format!("{axis}PR{distance_egu:.6}")
}

/// `<axis>OR` — execute the home search sequence.
pub fn cmd_home(axis: u8) -> String {
    format!("{axis}OR")
}

/// `<axis>ST` — stop motion.
pub fn cmd_stop(axis: u8) -> String {
    format!("{axis}ST")
}

/// `<axis>TP` — query current position. Response: `<axis>TP<value>`.
pub fn cmd_query_position(axis: u8) -> String {
    format!("{axis}TP")
}

/// `<axis>TS` — query controller status. Response: `<axis>TS<6 status chars>`.
pub fn cmd_query_status(axis: u8) -> String {
    format!("{axis}TS")
}

/// `<axis>SR?` — query the configured positive (right) travel limit.
pub fn cmd_query_high_limit(axis: u8) -> String {
    format!("{axis}SR?")
}

/// `<axis>SL?` — query the configured negative (left) travel limit.
pub fn cmd_query_low_limit(axis: u8) -> String {
    format!("{axis}SL?")
}

/// Parse a `TP`/`SR?`/`SL?` numeric response.
///
/// The controller echoes the command mnemonic before the value
/// (`"1TP-0.123"`, `"1SR25.0"`), so the C driver reads `atof(&inString_[3])` —
/// skip the 3-char prefix, then take the leading number. Returns `None` only
/// when the response is shorter than the prefix; a prefix followed by
/// non-numeric text yields `0.0`, matching C `atof`.
pub fn parse_value(resp: &str) -> Option<f64> {
    let tail = resp.get(3..)?;
    Some(atof(tail))
}

/// Decoded `TS` controller-status response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TsStatus {
    /// The controller reports state `28` (MOVING).
    pub moving: bool,
    /// Positive hardware travel limit active.
    pub high_limit: bool,
    /// Negative hardware travel limit active.
    pub low_limit: bool,
    /// The controller reports state `32` (READY from HOMING / at home).
    pub at_home: bool,
}

/// Parse a `TS` status response of the form `"1TS000028"`.
///
/// The C driver indexes the raw response bytes directly
/// (`inString_[6]`/`[7]`/`[8]`): the last two characters are the controller
/// state code, and byte 6 carries the limit bit. Returns `None` if the
/// response is too short to hold the full status field.
pub fn parse_status(resp: &str) -> Option<TsStatus> {
    let b = resp.as_bytes();
    if b.len() < 9 {
        return None;
    }
    // C SMC100Axis::poll: done = (inString_[7]=='2' && inString_[8]=='8') ? 0:1
    let moving = b[7] == b'2' && b[8] == b'8';
    // limit = (inString_[6]=='2') high, (inString_[6]=='1') low
    let high_limit = b[6] == b'2';
    let low_limit = b[6] == b'1';
    // atHome = (inString_[7]=='3' && inString_[8]=='2')
    let at_home = b[7] == b'3' && b[8] == b'2';
    Some(TsStatus {
        moving,
        high_limit,
        low_limit,
        at_home,
    })
}

/// Parse the leading floating-point number of `s`, in the manner of C `atof`:
/// leading whitespace is skipped, the longest leading numeric run
/// (`[+-]? digits '.' digits ('e'/'E' [+-]? digits)?`) is parsed, and any
/// trailing bytes (e.g. a stray CR the port did not strip) are ignored.
/// Returns `0.0` when no number is present, as `atof` does.
fn atof(s: &str) -> f64 {
    let s = s.trim_start();
    let bytes = s.as_bytes();
    let mut end = 0;
    let mut seen_dot = false;
    let mut seen_exp = false;
    while end < bytes.len() {
        let c = bytes[end];
        let ok = match c {
            b'0'..=b'9' => true,
            b'+' | b'-' => end == 0 || bytes[end - 1] == b'e' || bytes[end - 1] == b'E',
            b'.' => !seen_dot && !seen_exp,
            b'e' | b'E' => !seen_exp && end > 0,
            _ => false,
        };
        if !ok {
            break;
        }
        match c {
            b'.' => seen_dot = true,
            b'e' | b'E' => seen_exp = true,
            _ => {}
        }
        end += 1;
    }
    s[..end].parse().unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_formatting_matches_c_sprintf() {
        // C: sprintf(outString_, "%1dPA%f", axisNo_+1, val)
        assert_eq!(cmd_move_absolute(1, 2.5), "1PA2.500000");
        assert_eq!(cmd_move_relative(1, -0.25), "1PR-0.250000");
        assert_eq!(cmd_set_velocity(1, 1.0), "1VA1.000000");
        assert_eq!(cmd_set_acceleration(1, 10.0), "1AC10.000000");
        assert_eq!(cmd_home(1), "1OR");
        assert_eq!(cmd_stop(1), "1ST");
        assert_eq!(cmd_query_position(1), "1TP");
        assert_eq!(cmd_query_status(1), "1TS");
        assert_eq!(cmd_query_high_limit(1), "1SR?");
        assert_eq!(cmd_query_low_limit(1), "1SL?");
    }

    #[test]
    fn parse_value_skips_prefix_like_atof() {
        // Response forms from the C comments: "1TP-0.123", "1SR25.0", "1SL-5.0"
        assert_eq!(parse_value("1TP-0.123"), Some(-0.123));
        assert_eq!(parse_value("1TP12.5"), Some(12.5));
        assert_eq!(parse_value("1SR25.0"), Some(25.0));
        assert_eq!(parse_value("1SL-5.0"), Some(-5.0));
    }

    #[test]
    fn parse_value_tolerates_trailing_bytes_and_junk() {
        // A stray CR the input EOS did not strip must not defeat the parse.
        assert_eq!(parse_value("1TP-0.123\r"), Some(-0.123));
        // Non-numeric tail yields 0.0, as C atof does.
        assert_eq!(parse_value("1TPxyz"), Some(0.0));
    }

    #[test]
    fn parse_value_rejects_response_shorter_than_prefix() {
        assert_eq!(parse_value("1T"), None);
    }

    #[test]
    fn parse_status_moving_state_28() {
        // "1TS000028": state 28 == MOVING, no limits, not at home.
        let s = parse_status("1TS000028").unwrap();
        assert!(s.moving);
        assert!(!s.high_limit);
        assert!(!s.low_limit);
        assert!(!s.at_home);
    }

    #[test]
    fn parse_status_ready_and_home_state_32() {
        // "1TS000032": state 32, not moving; bytes 7/8 == '3'/'2' -> at home.
        let s = parse_status("1TS000032").unwrap();
        assert!(!s.moving);
        assert!(s.at_home);
        assert!(!s.high_limit);
        assert!(!s.low_limit);
    }

    #[test]
    fn parse_status_limit_bit_in_byte_6() {
        // byte[6]=='2' -> high limit; byte[6]=='1' -> low limit.
        let hi = parse_status("1TS000200").unwrap();
        assert!(hi.high_limit);
        assert!(!hi.low_limit);
        assert!(!hi.moving);

        let lo = parse_status("1TS000100").unwrap();
        assert!(lo.low_limit);
        assert!(!lo.high_limit);
    }

    #[test]
    fn parse_status_rejects_short_response() {
        assert_eq!(parse_status("1TS28"), None);
    }
}
