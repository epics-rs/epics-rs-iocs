//! C `sscanf` conversions one at a time.
//!
//! The vac device supports parse fixed-layout replies with `sscanf` format
//! strings such as `"%f%c%x"`, `"%2d%c%x"` and `"%3d%c%2d"`. Each primitive
//! here is one [`sscanf`] call with the conversion, its width and `%n`, so
//! blanks, widths and the longest valid prefix are glibc's: `"1.23-7"` under
//! `"%f%c%x"` yields `1.23`, `'-'`, `7`. A failed conversion is `None`, and
//! the caller keeps its previous value as C leaves its variable untouched.

use std::cell::Cell;

use epics_libcom_rs::runtime::scanf::{ScanNum, sscanf};
use epics_libcom_rs::runtime::stdlib::strtoll_errno;

/// `sscanf(s, "%<width><conv>%n", &v, &n) == 1`: the value and the bytes read.
fn scan<T: ScanNum + Default>(s: &[u8], width: Option<usize>, conv: &str) -> Option<(T, usize)> {
    let fmt = match width {
        Some(w) => format!("%{w}{conv}%n"),
        None => format!("%{conv}%n"),
    };
    let (v, n) = (Cell::new(T::default()), Cell::new(0i32));
    (sscanf(s, fmt, &[&v, &n]) == 1).then(|| (v.get(), n.get() as usize))
}

/// The field `conv` reads, valued by `strtoll` so that one past `long long`,
/// which glibc saturates, is `None`.
fn integer<'a>(
    s: &'a [u8],
    width: Option<usize>,
    conv: &str,
    base: u32,
) -> Option<(i64, &'a [u8])> {
    let (_, n) = scan::<i64>(s, width, conv)?;
    let (v, _) = strtoll_errno(&s[..n], base)?;
    Some((v, &s[n..]))
}

/// C `%c`: take exactly one character, no whitespace skip.
pub fn scan_char(s: &[u8]) -> Option<(u8, &[u8])> {
    scan(s, None, "c").map(|(c, n)| (c, &s[n..]))
}

/// C `%d` (optionally width-limited). Returns the value and the unconsumed
/// tail.
pub fn scan_int(s: &[u8], width: Option<usize>) -> Option<(i64, &[u8])> {
    integer(s, width, "lld", 10)
}

/// C `%x` (optionally width-limited).
pub fn scan_hex(s: &[u8], width: Option<usize>) -> Option<(i64, &[u8])> {
    integer(s, width, "llx", 16)
}

/// C `%e` / `%f` / `%le` / `%lf` (optionally width-limited).
pub fn scan_float(s: &[u8], width: Option<usize>) -> Option<(f64, &[u8])> {
    scan(s, width, "lf").map(|(v, n)| (v, &s[n..]))
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

    /// glibc 2.43's `%f` fails an exponent cut short, where `strtod` would
    /// back up to the mantissa.
    #[test]
    fn scientific_notation_parses_and_a_cut_short_exponent_fails() {
        assert_eq!(scan_float(b"1.23E-07", None).unwrap().0, 1.23e-7);
        assert!(scan_float(b"1.2e", None).is_none());
        assert!(scan_float(b"1.2e+", None).is_none());
    }

    #[test]
    fn conversions_read_what_glibc_sscanf_reads() {
        assert_eq!(scan_hex(b"0x1f", None), Some((31, &b""[..])));
        assert_eq!(scan_float(b"inf", None), Some((f64::INFINITY, &b""[..])));
        assert_eq!(scan_float(b"0x1p3V", None), Some((8.0, &b"V"[..])));
        assert_eq!(scan_hex(b" -5", None), Some((-5, &b""[..])));
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
