//! Minimal C `sscanf` conversion primitives.
//!
//! The vac device supports parse fixed-layout replies with `sscanf` format
//! strings such as `"%f%c%x"`, `"%2d%c%x"` and `"%3d%c%2d"`. Reproducing the
//! wire parse faithfully means reproducing C's conversion rules, not Rust's
//! `str::parse` rules:
//!
//! * numeric conversions (`%d`, `%x`, `%e`, `%f`) skip leading whitespace;
//!   `%c` does not;
//! * a conversion consumes the longest valid prefix and leaves the rest for
//!   the next conversion — `"1.23-7"` under `"%f%c%x"` yields `1.23`, `'-'`,
//!   `7`;
//! * a failed conversion leaves the caller's variable untouched. Callers get
//!   `None` here and keep the previous value themselves.
//!
//! Widths (`%2d`, `%3d`) cap the number of characters the conversion may
//! consume after the whitespace skip, as in C.

/// C's `isspace` for the "C" locale.
fn is_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | b'\x0b' | b'\x0c' | b'\r')
}

fn skip_ws(s: &[u8]) -> &[u8] {
    let n = s.iter().take_while(|&&b| is_space(b)).count();
    &s[n..]
}

/// Bytes a width-limited conversion may look at, after the whitespace skip.
fn window(s: &[u8], width: Option<usize>) -> &[u8] {
    match width {
        Some(w) => &s[..w.min(s.len())],
        None => s,
    }
}

/// C `%c`: take exactly one character, no whitespace skip.
pub fn scan_char(s: &[u8]) -> Option<(u8, &[u8])> {
    s.split_first().map(|(&c, rest)| (c, rest))
}

/// Consume an optional `+`/`-` sign, returning `(negative, rest)`.
fn sign(s: &[u8]) -> (bool, &[u8]) {
    match s.split_first() {
        Some((b'-', rest)) => (true, rest),
        Some((b'+', rest)) => (false, rest),
        _ => (false, s),
    }
}

/// C `%d` (optionally width-limited). Returns the value and the unconsumed
/// tail of the *original* slice.
pub fn scan_int(s: &[u8], width: Option<usize>) -> Option<(i64, &[u8])> {
    let after_ws = skip_ws(s);
    let win = window(after_ws, width);
    let (neg, digits) = sign(win);
    let n = digits.iter().take_while(|b| b.is_ascii_digit()).count();
    if n == 0 {
        return None;
    }
    let mag: i64 = std::str::from_utf8(&digits[..n]).ok()?.parse().ok()?;
    let consumed = (win.len() - digits.len()) + n;
    Some((if neg { -mag } else { mag }, &after_ws[consumed..]))
}

/// C `%x` (optionally width-limited).
pub fn scan_hex(s: &[u8], width: Option<usize>) -> Option<(i64, &[u8])> {
    let after_ws = skip_ws(s);
    let win = window(after_ws, width);
    let (neg, digits) = sign(win);
    let n = digits.iter().take_while(|b| b.is_ascii_hexdigit()).count();
    if n == 0 {
        return None;
    }
    let mag = i64::from_str_radix(std::str::from_utf8(&digits[..n]).ok()?, 16).ok()?;
    let consumed = (win.len() - digits.len()) + n;
    Some((if neg { -mag } else { mag }, &after_ws[consumed..]))
}

/// C `%e` / `%f` / `%le` / `%lf` (optionally width-limited).
///
/// Accepts `[+-]?digits[.digits][(e|E)[+-]?digits]`. An exponent marker with
/// no following digits is not part of the number (C backs up), so `"1.2e"`
/// converts `1.2` and leaves `"e"`.
pub fn scan_float(s: &[u8], width: Option<usize>) -> Option<(f64, &[u8])> {
    let after_ws = skip_ws(s);
    let win = window(after_ws, width);

    let mut i = 0;
    if matches!(win.first(), Some(b'+' | b'-')) {
        i += 1;
    }
    let int_digits = win[i..].iter().take_while(|b| b.is_ascii_digit()).count();
    i += int_digits;
    let mut frac_digits = 0;
    if win.get(i) == Some(&b'.') {
        let after_dot = i + 1;
        frac_digits = win[after_dot..]
            .iter()
            .take_while(|b| b.is_ascii_digit())
            .count();
        i = after_dot + frac_digits;
    }
    if int_digits == 0 && frac_digits == 0 {
        return None;
    }

    // Exponent, only if it is complete.
    if matches!(win.get(i), Some(b'e' | b'E')) {
        let mut j = i + 1;
        if matches!(win.get(j), Some(b'+' | b'-')) {
            j += 1;
        }
        let exp_digits = win[j..].iter().take_while(|b| b.is_ascii_digit()).count();
        if exp_digits > 0 {
            i = j + exp_digits;
        }
    }

    let value: f64 = std::str::from_utf8(&win[..i]).ok()?.parse().ok()?;
    Some((value, &after_ws[i..]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn float_then_char_then_hex_splits_mm200_field() {
        // MM200 cold-cathode field "1.23-7" under C "%f%c%x".
        let (f, rest) = scan_float(b"1.23-7", None).unwrap();
        assert_eq!(f, 1.23);
        let (c, rest) = scan_char(rest).unwrap();
        assert_eq!(c, b'-');
        let (e, rest) = scan_hex(rest, None).unwrap();
        assert_eq!(e, 7);
        assert!(rest.is_empty());
    }

    #[test]
    fn hex_exponent_accepts_letters() {
        // MM200 exponents are read with %x, so 'a' is 10.
        let (f, rest) = scan_float(b"9.9+a", None).unwrap();
        assert_eq!(f, 9.9);
        let (c, rest) = scan_char(rest).unwrap();
        assert_eq!(c, b'+');
        assert_eq!(scan_hex(rest, None).unwrap().0, 10);
    }

    #[test]
    fn width_limited_int_stops_at_width() {
        // CC10 "%2d%c%x" on "1005".
        let (v, rest) = scan_int(b"1005", Some(2)).unwrap();
        assert_eq!(v, 10);
        let (c, rest) = scan_char(rest).unwrap();
        assert_eq!(c, b'0');
        assert_eq!(scan_hex(rest, None).unwrap().0, 5);
    }

    #[test]
    fn mx200_hi_resolution_triplet() {
        // "%3d%c%2d" on "123105".
        let (v, rest) = scan_int(b"123105", Some(3)).unwrap();
        assert_eq!(v, 123);
        let (c, rest) = scan_char(rest).unwrap();
        assert_eq!(c, b'1');
        assert_eq!(scan_int(rest, Some(2)).unwrap().0, 5);
    }

    #[test]
    fn leading_whitespace_skipped_for_numbers_not_chars() {
        assert_eq!(scan_int(b"   42", None).unwrap().0, 42);
        assert_eq!(scan_hex(b" ff", None).unwrap().0, 255);
        assert_eq!(scan_char(b" f").unwrap().0, b' ');
    }

    #[test]
    fn scientific_notation_parses_and_incomplete_exponent_backs_up() {
        assert_eq!(scan_float(b"1.23E-07", None).unwrap().0, 1.23e-7);
        let (v, rest) = scan_float(b"1.2e", None).unwrap();
        assert_eq!(v, 1.2);
        assert_eq!(rest, b"e");
    }

    #[test]
    fn failed_conversion_returns_none() {
        assert!(scan_int(b"abc", None).is_none());
        assert!(scan_float(b"", None).is_none());
        assert!(scan_hex(b"zz", None).is_none());
        assert!(scan_char(b"").is_none());
    }

    #[test]
    fn digitel_time_online_prefix() {
        // "%d %d:%d %lfV%leI" on "12 03:45 5600V 1.2E-08I"
        let s = b"12 03:45 5600V 1.2E-08I";
        let (d, rest) = scan_int(s, None).unwrap();
        let (h, rest) = scan_int(rest, None).unwrap();
        assert_eq!(scan_char(rest).unwrap().0, b':');
        let (m, rest) = scan_int(&rest[1..], None).unwrap();
        let (v, rest) = scan_float(rest, None).unwrap();
        assert_eq!(scan_char(rest).unwrap().0, b'V');
        let (i, _) = scan_float(&rest[1..], None).unwrap();
        assert_eq!((d, h, m), (12, 3, 45));
        assert_eq!(v, 5600.0);
        assert_eq!(i, 1.2e-8);
    }
}
