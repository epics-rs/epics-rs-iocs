//! C `printf`-compatible float formatting.
//!
//! The vacuum controllers are fed exponent-form numbers built with `%7.1E`
//! (`devMPC.c:626`) and `%8.4E` (`devTPG261.c:517`). Rust's `{:E}` writes the
//! exponent without the C two-digit zero padding (`1.0E-6` vs `1.0E-06`), which
//! would change the bytes on the wire, so format the exponent explicitly.

/// Format `value` as C's `%.<precision>E` does: one leading digit, `precision`
/// decimals, `E`, an explicit sign and at least two exponent digits.
pub fn exp_c(value: f64, precision: usize) -> String {
    let text = format!("{value:.precision$E}");
    let (mantissa, exponent) = text
        .split_once('E')
        .expect("Rust's {:E} always emits an exponent");
    let exponent: i32 = exponent
        .parse()
        .expect("Rust's {:E} always emits an integer exponent");
    let sign = if exponent < 0 { '-' } else { '+' };
    format!("{mantissa}E{sign}{:02}", exponent.abs())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_c_percent_e() {
        // printf("%7.1E", 1.0e-6) -> "1.0E-06"
        assert_eq!(exp_c(1.0e-6, 1), "1.0E-06");
        assert_eq!(exp_c(5.5e-8, 1), "5.5E-08");
        assert_eq!(exp_c(0.0, 1), "0.0E+00");
        assert_eq!(exp_c(760.0, 1), "7.6E+02");
        // printf("%8.4E", 1.0e-6) -> "1.0000E-06"
        assert_eq!(exp_c(1.0e-6, 4), "1.0000E-06");
        assert_eq!(exp_c(1.2345e-9, 4), "1.2345E-09");
        // Three-digit exponents keep all their digits, as in C.
        assert_eq!(exp_c(1.0e-120, 1), "1.0E-120");
    }
}
