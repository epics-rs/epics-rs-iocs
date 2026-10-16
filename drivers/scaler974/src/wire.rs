//! ASCII command/reply helpers ported from `Scaler974::writeInt32`/
//! `Scaler974::eventThread` (`drvScaler974.cpp`). The Ortec 974 protocol
//! used by this driver is line-oriented ASCII; EOS framing itself is not
//! set by the C driver (see `crate::connect`'s module doc) so it is not
//! reproduced here either -- this module only covers the two pieces of
//! payload text the driver actually parses/builds: the `SHOW_COUNTS` reply
//! and the `SET_COUNT_PRESET` command body.

/// C `Scaler974::eventThread` (`drvScaler974.cpp:234-235`):
/// `sscanf(response, "%d;%d;%d;%d;", &counts[0], ...)` into the thread's
/// own `counts[]`. `sscanf` stops at the first field it cannot convert and
/// leaves that field and the rest untouched, so they keep the previous
/// poll's values.
pub fn scan_show_counts(data: &[u8], counts: &mut [i32; 4]) {
    let text = String::from_utf8_lossy(data);
    let mut rest = text.as_ref();
    for slot in counts.iter_mut() {
        let Some((value, after)) = scan_int(rest) else {
            break;
        };
        *slot = value;
        rest = match after.strip_prefix(';') {
            Some(r) => r,
            None => break,
        };
    }
}

/// C `sscanf` `%d`: skip leading whitespace, an optional sign, then decimal
/// digits. Returns `None` (no conversion) if no digit follows the
/// optional sign, matching C leaving the target unset in that case.
fn scan_int(s: &str) -> Option<(i32, &str)> {
    let s = s.trim_start();
    let bytes = s.as_bytes();
    let mut end = 0;
    if end < bytes.len() && (bytes[end] == b'-' || bytes[end] == b'+') {
        end += 1;
    }
    let digits_start = end;
    while end < bytes.len() && bytes[end].is_ascii_digit() {
        end += 1;
    }
    if end == digits_start {
        return None;
    }
    let value: i32 = s[..end].parse().ok()?;
    Some((value, &s[end..]))
}

/// C `Scaler974::writeInt32`'s `scalerPreset` branch (`drvScaler974.cpp:164-176`):
/// ```c
/// n = (int)log10((double)value);
/// m = (int)(value / pow(10.0, n));
/// sprintf(newstr, "SET_COUNT_PRESET %d,%d", m, n);
/// ```
/// **Lossy**: only a single leading significant digit `m` (0-9) survives —
/// any preset that isn't exactly `m * 10^n` is silently truncated toward
/// that form on the wire (e.g. `12345` -> `"SET_COUNT_PRESET 1,4"`, i.e.
/// `10000`). C never reads this back or reports the loss to the record —
/// see `Scaler974Driver::write_preset`'s doc.
///
/// `preset == 0`: C's `log10(0.0)` is `-HUGE_VAL`; casting that to `int` is
/// implementation-defined (UB-adjacent) in C. Rust's `as i32` float-to-int
/// cast is a defined saturating conversion (`f64::NEG_INFINITY as i32 ==
/// i32::MIN`), so this substitutes a deterministic (if nonsensical) command
/// string instead of UB. Not reachable by a real nonzero preset.
pub fn encode_set_count_preset(preset: u32) -> String {
    let value = preset as f64;
    let n = value.log10() as i32;
    let m = (value / 10f64.powi(n)) as i32;
    format!("SET_COUNT_PRESET {m},{n}")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scan(data: &[u8], prev: [i32; 4]) -> [i32; 4] {
        let mut counts = prev;
        scan_show_counts(data, &mut counts);
        counts
    }

    #[test]
    fn scan_show_counts_reads_four_semicolon_terminated_fields() {
        assert_eq!(scan(b"1234;5678;91;23;", [0; 4]), [1234, 5678, 91, 23]);
    }

    #[test]
    fn scan_show_counts_keeps_the_fields_it_cannot_convert() {
        let prev = [7, 8, 9, 10];
        assert_eq!(scan(b"10;20;", prev), [10, 20, 9, 10]);
        assert_eq!(scan(b"", prev), prev);
        assert_eq!(scan(b"garbage", prev), prev);
    }

    #[test]
    fn scan_show_counts_stops_at_missing_semicolon() {
        // No ';' after "10": sscanf converts field 0 only.
        assert_eq!(scan(b"10 20;30;40;", [1, 2, 3, 4]), [10, 2, 3, 4]);
    }

    #[test]
    fn scan_show_counts_handles_negative_fields() {
        assert_eq!(scan(b"-1;2;-3;4;", [0; 4]), [-1, 2, -3, 4]);
    }

    #[test]
    fn encode_set_count_preset_extracts_leading_digit_and_exponent() {
        assert_eq!(encode_set_count_preset(5000), "SET_COUNT_PRESET 5,3");
        assert_eq!(encode_set_count_preset(1), "SET_COUNT_PRESET 1,0");
        assert_eq!(encode_set_count_preset(9), "SET_COUNT_PRESET 9,0");
    }

    #[test]
    fn encode_set_count_preset_is_lossy_for_non_power_of_ten_mantissas() {
        // 12345 -> n=4, m=12345/10^4 truncated to 1 -> "1,4" (== 10000).
        assert_eq!(encode_set_count_preset(12345), "SET_COUNT_PRESET 1,4");
    }

    #[test]
    fn encode_set_count_preset_does_not_panic_on_zero() {
        // C: log10(0)=-inf, (int)-inf is UB. Rust saturates instead of
        // panicking or invoking UB.
        let cmd = encode_set_count_preset(0);
        assert!(cmd.starts_with("SET_COUNT_PRESET"));
    }
}
