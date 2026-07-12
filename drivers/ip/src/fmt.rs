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

/// One `printf` conversion of a `double`, as parsed out of a C format string.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Spec {
    left: bool,
    plus: bool,
    space: bool,
    zero: bool,
    alt: bool,
    width: usize,
    precision: Option<usize>,
    conversion: char,
}

/// Split a C format string into `(prefix, spec, suffix)`.
///
/// `devXxEurotherm.c` takes the payload format from the record link (`FMT=`,
/// default `%f`), so the format is data, not a literal: `SL%4.0lf` must produce
/// the same bytes C's `sprintf` would. Exactly one conversion is allowed and it
/// must take a `double` — the C passes `pr->val`, so `%d` and friends would be
/// undefined behaviour there and are rejected here.
fn parse_format(format: &str) -> Result<(String, Spec, String), String> {
    let mut prefix = String::new();
    let mut chars = format.chars().peekable();

    // Literal text, with "%%" standing for a literal '%'.
    loop {
        match chars.next() {
            None => return Err(format!("format {format:?} has no conversion")),
            Some('%') if chars.peek() == Some(&'%') => {
                chars.next();
                prefix.push('%');
            }
            Some('%') => break,
            Some(c) => prefix.push(c),
        }
    }

    let mut spec = Spec {
        left: false,
        plus: false,
        space: false,
        zero: false,
        alt: false,
        width: 0,
        precision: None,
        conversion: ' ',
    };
    while let Some(&c) = chars.peek() {
        match c {
            '-' => spec.left = true,
            '+' => spec.plus = true,
            ' ' => spec.space = true,
            '0' => spec.zero = true,
            '#' => spec.alt = true,
            _ => break,
        }
        chars.next();
    }
    while let Some(&c) = chars.peek() {
        let Some(digit) = c.to_digit(10) else { break };
        spec.width = spec.width * 10 + digit as usize;
        chars.next();
    }
    if chars.peek() == Some(&'.') {
        chars.next();
        let mut precision = 0;
        while let Some(&c) = chars.peek() {
            let Some(digit) = c.to_digit(10) else { break };
            precision = precision * 10 + digit as usize;
            chars.next();
        }
        spec.precision = Some(precision);
    }
    // Length modifiers: the C databases write "%4.0lf".
    while let Some(&c) = chars.peek() {
        if matches!(c, 'l' | 'L' | 'h') {
            chars.next();
        } else {
            break;
        }
    }
    spec.conversion = match chars.next() {
        Some(c @ ('f' | 'F' | 'e' | 'E' | 'g' | 'G')) => c,
        Some(c) => {
            return Err(format!(
                "format {format:?} converts a double with %{c}, which takes another type"
            ));
        }
        None => return Err(format!("format {format:?} ends inside its conversion")),
    };

    let mut suffix = String::new();
    while let Some(c) = chars.next() {
        match c {
            '%' if chars.peek() == Some(&'%') => {
                chars.next();
                suffix.push('%');
            }
            '%' => return Err(format!("format {format:?} has a second conversion")),
            c => suffix.push(c),
        }
    }
    Ok((prefix, spec, suffix))
}

/// C `%g`: the shorter of `%e` and `%f`, with the trailing zeros removed unless
/// `#` was given.
fn g_format(magnitude: f64, precision: usize, alt: bool) -> String {
    let precision = precision.max(1);
    let exponent: i32 = if magnitude == 0.0 {
        0
    } else {
        exp_c(magnitude, precision - 1)
            .split_once('E')
            .and_then(|(_, e)| e.parse().ok())
            .unwrap_or(0)
    };

    let mut text = if exponent < -4 || exponent >= precision as i32 {
        exp_c(magnitude, precision - 1)
    } else {
        let decimals = (precision as i32 - 1 - exponent).max(0) as usize;
        format!("{magnitude:.decimals$}")
    };
    if !alt {
        let (mantissa, exponent) = match text.split_once('E') {
            Some((mantissa, exponent)) => (mantissa.to_string(), Some(exponent.to_string())),
            None => (text, None),
        };
        let mut mantissa = mantissa;
        if mantissa.contains('.') {
            mantissa = mantissa
                .trim_end_matches('0')
                .trim_end_matches('.')
                .to_string();
        }
        text = match exponent {
            Some(exponent) => format!("{mantissa}E{exponent}"),
            None => mantissa,
        };
    }
    text
}

/// Format `value` with the C format string `format`, which must carry exactly
/// one `double` conversion (`%f`, `%e`, `%E`, `%g`, `%G`, with the usual flags,
/// width, precision and `l` modifier).
pub fn format_c_double(format: &str, value: f64) -> Result<String, String> {
    let (prefix, spec, suffix) = parse_format(format)?;

    let magnitude = value.abs();
    let mut body = if !value.is_finite() {
        if value.is_nan() { "nan" } else { "inf" }.to_string()
    } else {
        let precision = spec.precision.unwrap_or(6);
        match spec.conversion {
            'f' | 'F' => format!("{magnitude:.precision$}"),
            'e' | 'E' => exp_c(magnitude, precision),
            _ => g_format(magnitude, precision, spec.alt),
        }
    };
    if spec.conversion.is_lowercase() {
        body = body.to_lowercase();
    } else {
        body = body.to_uppercase();
    }

    let sign = if value.is_sign_negative() && !value.is_nan() {
        "-"
    } else if spec.plus {
        "+"
    } else if spec.space {
        " "
    } else {
        ""
    };

    let width = spec.width.saturating_sub(sign.len() + body.chars().count());
    let padded = if width == 0 {
        format!("{sign}{body}")
    } else if spec.left {
        format!("{sign}{body}{}", " ".repeat(width))
    } else if spec.zero && value.is_finite() {
        format!("{sign}{}{body}", "0".repeat(width))
    } else {
        format!("{}{sign}{body}", " ".repeat(width))
    };
    Ok(format!("{prefix}{padded}{suffix}"))
}

/// Read the leading number of `text` as C's `sscanf("%lf")` does: skip leading
/// whitespace, take the longest prefix that is a number, ignore the rest.
/// `None` when there is no number at all (C: the conversion fails).
pub fn scan_f64(text: &str) -> Option<f64> {
    let text = text.trim_start();
    let mut value = None;
    for (end, _) in text.char_indices().skip(1) {
        match text[..end].parse::<f64>() {
            Ok(parsed) => value = Some(parsed),
            // Keep going: "1e" does not parse but "1e-3" does.
            Err(_) => continue,
        }
    }
    text.parse::<f64>().ok().or(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every expectation here is the output of the same `printf` call compiled
    /// with gcc and run on this host.
    #[test]
    fn matches_c_printf() {
        assert_eq!(format_c_double("%4.0lf", 123.4).unwrap(), " 123");
        assert_eq!(format_c_double("SL%4.0lf", 123.4).unwrap(), "SL 123");
        assert_eq!(format_c_double("%f", 1.5).unwrap(), "1.500000");
        assert_eq!(format_c_double("%f", -1.5).unwrap(), "-1.500000");
        assert_eq!(format_c_double("%8.2f", -1.5).unwrap(), "   -1.50");
        assert_eq!(format_c_double("%-8.2f", 1.5).unwrap(), "1.50    ");
        assert_eq!(format_c_double("%08.2f", -1.5).unwrap(), "-0001.50");
        assert_eq!(format_c_double("%+.1f", 1.5).unwrap(), "+1.5");
        assert_eq!(format_c_double("% .1f", 1.5).unwrap(), " 1.5");
        assert_eq!(format_c_double("%e", 1234.5).unwrap(), "1.234500e+03");
        assert_eq!(format_c_double("%.2E", -0.00012345).unwrap(), "-1.23E-04");
        assert_eq!(format_c_double("%g", 100.0).unwrap(), "100");
        assert_eq!(format_c_double("%g", 0.0001).unwrap(), "0.0001");
        assert_eq!(format_c_double("%g", 0.00001).unwrap(), "1e-05");
        assert_eq!(format_c_double("%g", 123456789.0).unwrap(), "1.23457e+08");
        assert_eq!(format_c_double("%g", 0.0).unwrap(), "0");
        assert_eq!(format_c_double("%.3g", 1234.0).unwrap(), "1.23e+03");
        assert_eq!(format_c_double("%#g", 100.0).unwrap(), "100.000");
        assert_eq!(format_c_double("%G", 0.00001).unwrap(), "1E-05");
    }

    #[test]
    fn rejects_formats_that_do_not_take_one_double() {
        assert!(format_c_double("SL", 1.0).is_err());
        assert!(format_c_double("%d", 1.0).is_err());
        assert!(format_c_double("%f%f", 1.0).is_err());
        assert!(format_c_double("%", 1.0).is_err());
        // "%%" is a literal percent, not a conversion.
        assert!(format_c_double("100%%", 1.0).is_err());
        assert_eq!(format_c_double("%.0f%%", 12.0).unwrap(), "12%");
    }

    #[test]
    fn scans_the_leading_number_like_sscanf() {
        assert_eq!(scan_f64("12.3456"), Some(12.3456));
        assert_eq!(scan_f64("  12.3456 mm"), Some(12.3456));
        assert_eq!(scan_f64("1.2E-06 Torr"), Some(1.2e-6));
        assert_eq!(scan_f64("-4.5"), Some(-4.5));
        assert_eq!(scan_f64("0.0"), Some(0.0));
        assert_eq!(scan_f64("mm"), None);
        assert_eq!(scan_f64(""), None);
    }

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
