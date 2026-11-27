//! Numeric parsing helpers shared by the Newport controller drivers.
//!
//! The C drivers parse controller replies with `atof`/`atoi`/`sscanf("%x")`
//! applied at a fixed byte offset into the reply (skipping the echoed command
//! prefix). These helpers reproduce those C semantics faithfully so the
//! per-driver modules can share one implementation.

pub(crate) use motor_common::util::{atof, atoi, leading_hex};

/// `atof(&resp[offset..])`; `None` only when `offset` is past the reply.
pub(crate) fn parse_value_at(resp: &str, offset: usize) -> Option<f64> {
    resp.get(offset..).map(atof)
}

/// `atoi(&resp[offset..])`; `None` only when `offset` is past the reply.
pub(crate) fn parse_int_at(resp: &str, offset: usize) -> Option<i32> {
    resp.get(offset..).map(atoi)
}

/// C `NINT(f)`: round to nearest integer, away from zero on the half.
pub(crate) fn nint(f: f64) -> i32 {
    (if f > 0.0 { f + 0.5 } else { f - 0.5 }) as i32
}

/// Shared MM4000/PM500 `maxDigits`/`res_decpts`: command decimal precision
/// from the drive resolution, `(int)(-log10(stepSize)) + 2`, floored at 1.
pub(crate) fn max_digits(step_size: f64) -> usize {
    let x = -step_size.log10();
    // C `(int)` on x86-64 turns NaN/inf (res <= 0) into INT_MIN, hence the floor.
    if !x.is_finite() {
        return 1;
    }
    (x as i32 + 2).max(1) as usize
}

/// Shared ESP300/MM3000 `recv_mess` retry predicate: a reply longer than 3
/// characters starting with `E` whose number is a hard-travel-limit code
/// (35..=42), read by `strtol(&com[1], NULL, 0)`, is an unsolicited error
/// message to flush with a re-read.
pub(crate) fn is_unsolicited_limit_error(reply: &str) -> bool {
    reply.len() > 3
        && reply.starts_with('E')
        && (35..=42).contains(&motor_common::util::strtol(&reply[1..], 0).0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn max_digits_matches_c_truncation() {
        // C: (int)(-log10(step)) + 2, floored at 1.
        assert_eq!(max_digits(0.001), 5);
        assert_eq!(max_digits(0.0005), 5); // -log10 = 3.30 → (int) 3 → 5
        assert_eq!(max_digits(0.01), 4); // the PM500 fixed resolution
        assert_eq!(max_digits(0.1), 3);
        assert_eq!(max_digits(1.0), 2);
        assert_eq!(max_digits(100.0), 1); // -2 + 2 = 0 → floored at 1
        assert_eq!(max_digits(0.0), 1); // C x86: (int)inf = INT_MIN → floored
        assert_eq!(max_digits(-1.0), 1); // log10 NaN
    }

    #[test]
    fn nint_rounds_away_from_zero() {
        assert_eq!(nint(1.5), 2);
        assert_eq!(nint(1.4), 1);
        assert_eq!(nint(-1.5), -2);
        assert_eq!(nint(-1.4), -1);
        assert_eq!(nint(0.0), 0);
    }

    #[test]
    fn unsolicited_limit_error_predicate() {
        // Hard-travel-limit codes 35..=42 with a message are flushed.
        assert!(is_unsolicited_limit_error("E35 HARDWARE LIMIT"));
        assert!(is_unsolicited_limit_error("E42 x"));
        // Codes outside 35..=42 are real replies.
        assert!(!is_unsolicited_limit_error("E34 x"));
        assert!(!is_unsolicited_limit_error("E43 x"));
        // C requires nread > 3: a bare short reply is never flushed.
        assert!(!is_unsolicited_limit_error("E35"));
        // Ordinary replies pass through.
        assert!(!is_unsolicited_limit_error("0, NO ERROR DETECTED"));
        assert!(!is_unsolicited_limit_error("1"));
    }

    /// C reads the code with `strtol(&com[1], NULL, 0)`: `035` is octal 29.
    #[test]
    fn unsolicited_limit_error_reads_strtol_base_0() {
        assert!(!is_unsolicited_limit_error("E035 x"));
        assert!(is_unsolicited_limit_error("E0x23 x"));
        assert!(is_unsolicited_limit_error("E043 x"));
    }

    #[test]
    fn atof_parses_leading_numeric_prefix() {
        assert_eq!(atof("-0.1234junk"), -0.1234);
        assert_eq!(atof("  5"), 5.0);
        assert_eq!(atof("1.5e-3x"), 1.5e-3);
        // No valid number → 0.0, like C atof.
        assert_eq!(atof("abc"), 0.0);
        assert_eq!(atof("+"), 0.0);
    }

    #[test]
    fn atof_reads_what_strtod_reads() {
        assert_eq!(atof("0x1A"), 26.0);
        assert_eq!(atof(" -inf"), f64::NEG_INFINITY);
        assert!(atof("nan").is_nan());
        assert_eq!(atof("\u{a0}5"), 0.0);
    }

    #[test]
    fn leading_hex_reads_hex_digits_or_none() {
        assert_eq!(leading_hex("1e"), Some(0x1e));
        assert_eq!(leading_hex("ffXY"), Some(0xff));
        assert_eq!(leading_hex("xy"), None);
        assert_eq!(leading_hex(""), None);
    }

    #[test]
    fn leading_hex_reads_what_sscanf_x_reads() {
        assert_eq!(leading_hex(" 1e"), Some(0x1e));
        assert_eq!(leading_hex("0x1F"), Some(31));
        assert_eq!(leading_hex("+5"), Some(5));
        assert_eq!(leading_hex("-5"), None);
    }

    #[test]
    fn parse_at_offset_skips_command_prefix() {
        // "1TP-0.1234": value at offset 3.
        assert_eq!(parse_value_at("1TP-0.1234", 3), Some(-0.1234));
        // "1FRM400": int at offset 4.
        assert_eq!(parse_int_at("1FRM400", 4), Some(400));
        // Offset past the reply.
        assert_eq!(parse_value_at("1TP", 5), None);
        assert_eq!(parse_int_at("1TP", 5), None);
    }
}
