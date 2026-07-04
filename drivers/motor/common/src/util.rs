//! C-runtime numeric helpers shared by the controller drivers.
//!
//! The C drivers parse controller replies with `atof`/`atoi`/`sscanf("%x")`
//! applied at a fixed byte offset into the reply (skipping the echoed command
//! prefix). These reproduce those C semantics faithfully — in particular the
//! "parse the leading numeric prefix, `0` on junk" behaviour of `atof`/`atoi`,
//! which controller reply parsing relies on.

/// Mimic C `atof`: parse the leading numeric prefix as `f64`, returning `0.0`
/// on junk (as C `atof` does).
pub fn atof(s: &str) -> f64 {
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
    t.get(..i)
        .and_then(|p| p.parse::<f64>().ok())
        .unwrap_or(0.0)
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

/// Mimic C `atol`/`strtol` into `i64`: parse the leading integer prefix, `0`
/// on junk. Used by controllers whose positions overflow `i32` (picomotor
/// step counts, encoder counts).
pub fn atol(s: &str) -> i64 {
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

/// Parse leading hex digits as `u32` (C `sscanf` `%x`); `None` if there is no
/// hex digit (C `sscanf` count `!= 1`).
pub fn leading_hex(s: &str) -> Option<u32> {
    let n = s.bytes().take_while(u8::is_ascii_hexdigit).count();
    if n == 0 {
        return None;
    }
    u32::from_str_radix(&s[..n], 16).ok()
}

/// `atof(&resp[offset..])`; `None` only when `offset` is past the reply.
pub fn parse_value_at(resp: &str, offset: usize) -> Option<f64> {
    resp.get(offset..).map(atof)
}

/// `atoi(&resp[offset..])`; `None` only when `offset` is past the reply.
pub fn parse_int_at(resp: &str, offset: usize) -> Option<i32> {
    resp.get(offset..).map(atoi)
}

/// C `NINT(f)`: round to nearest integer, away from zero on the half.
pub fn nint(f: f64) -> i32 {
    (if f > 0.0 { f + 0.5 } else { f - 0.5 }) as i32
}

/// Command decimal precision from a drive resolution, `(int)(-log10(res)) + 2`,
/// floored at 1 (C `maxDigits`/`res_decpts`).
pub fn max_digits(step_size: f64) -> usize {
    let digits = (-step_size.log10()) as i32 + 2;
    digits.max(1) as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn atof_parses_leading_numeric_prefix() {
        assert_eq!(atof("-0.1234junk"), -0.1234);
        assert_eq!(atof("  5"), 5.0);
        assert_eq!(atof("1.5e-3x"), 1.5e-3);
        assert_eq!(atof("abc"), 0.0);
        assert_eq!(atof("+"), 0.0);
    }

    #[test]
    fn atoi_parses_leading_integer_prefix() {
        assert_eq!(atoi("400rpm"), 400);
        assert_eq!(atoi("-7"), -7);
        assert_eq!(atoi("junk"), 0);
    }

    #[test]
    fn atol_parses_wide_integers() {
        assert_eq!(atol("10000000000x"), 10_000_000_000);
        assert_eq!(atol("-2"), -2);
        assert_eq!(atol("junk"), 0);
    }

    #[test]
    fn leading_hex_reads_hex_digits_or_none() {
        assert_eq!(leading_hex("1e"), Some(0x1e));
        assert_eq!(leading_hex("ffXY"), Some(0xff));
        assert_eq!(leading_hex("xy"), None);
        assert_eq!(leading_hex(""), None);
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
    fn max_digits_matches_c_truncation() {
        assert_eq!(max_digits(0.001), 5);
        assert_eq!(max_digits(0.01), 4);
        assert_eq!(max_digits(0.1), 3);
        assert_eq!(max_digits(1.0), 2);
        assert_eq!(max_digits(100.0), 1);
    }

    #[test]
    fn parse_at_offset_skips_command_prefix() {
        assert_eq!(parse_value_at("1TP-0.1234", 3), Some(-0.1234));
        assert_eq!(parse_int_at("1FRM400", 4), Some(400));
        assert_eq!(parse_value_at("1TP", 5), None);
        assert_eq!(parse_int_at("1TP", 5), None);
    }
}
