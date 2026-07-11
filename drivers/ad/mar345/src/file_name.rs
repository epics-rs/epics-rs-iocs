//! File-name construction.
//!
//! `ADDriverBase` in ad-core-rs 0.22.1 does not expose `createFileName` (it
//! lives on the C `asynNDArrayDriver`, and the crate's `sprintf_template`
//! helper is private), so the printf subset `NDFileTemplate` needs is
//! reimplemented here. `mar345.cpp` calls `createFileName(MAX_FILENAME_LEN,
//! tempFileName)`, which expands the default `NDFileTemplate` `"%s%s_%3.3d"`
//! over `(NDFilePath, NDFileName, NDFileNumber)`; the driver then appends the
//! `.mar<pixels>` extension.

use crate::types::MAX_FILENAME_LEN;

#[derive(Default)]
struct Spec {
    minus: bool,
    zero: bool,
    plus: bool,
    space: bool,
    width: Option<usize>,
    prec: Option<usize>,
}

fn pad(spec: &Spec, body: String) -> String {
    let width = spec.width.unwrap_or(0);
    if body.chars().count() >= width {
        return body;
    }
    let fill = width - body.chars().count();
    if spec.minus {
        format!("{body}{}", " ".repeat(fill))
    } else {
        format!("{}{body}", " ".repeat(fill))
    }
}

/// `%d` / `%i`: precision is a minimum digit count (zero-padded), applied before
/// the sign; the `0` flag pads the *field* with zeros but is ignored when a
/// precision is present or when `-` is given.
fn format_signed(spec: &Spec, value: i64) -> String {
    let digits = value.unsigned_abs().to_string();
    let digits = match spec.prec {
        Some(0) if value == 0 => String::new(),
        Some(p) if digits.len() < p => format!("{}{digits}", "0".repeat(p - digits.len())),
        _ => digits,
    };
    let sign = if value < 0 {
        "-"
    } else if spec.plus {
        "+"
    } else if spec.space {
        " "
    } else {
        ""
    };
    let width = spec.width.unwrap_or(0);
    if spec.zero && !spec.minus && spec.prec.is_none() && sign.len() + digits.len() < width {
        let fill = width - sign.len() - digits.len();
        return format!("{sign}{}{digits}", "0".repeat(fill));
    }
    pad(spec, format!("{sign}{digits}"))
}

fn format_unsigned(spec: &Spec, value: u64, radix: u32, upper: bool) -> String {
    let mut digits = match radix {
        8 => format!("{value:o}"),
        16 if upper => format!("{value:X}"),
        16 => format!("{value:x}"),
        _ => value.to_string(),
    };
    if let Some(p) = spec.prec
        && digits.len() < p
    {
        digits = format!("{}{digits}", "0".repeat(p - digits.len()));
    }
    let width = spec.width.unwrap_or(0);
    if spec.zero && !spec.minus && spec.prec.is_none() && digits.len() < width {
        let fill = width - digits.len();
        return format!("{}{digits}", "0".repeat(fill));
    }
    pad(spec, digits)
}

fn format_string(spec: &Spec, value: &str) -> String {
    let body = match spec.prec {
        Some(p) => value.chars().take(p).collect::<String>(),
        None => value.to_string(),
    };
    pad(spec, body)
}

/// Expand a printf template the way `asynNDArrayDriver::createFileName` does:
/// `epicsSnprintf(out, maxChars, template, string0, string1, integer)`.
///
/// Returns `None` when the template needs more than the two string arguments or
/// uses a conversion these templates never contain (C would read past the
/// argument list — undefined behaviour we refuse instead of imitating). The
/// result is truncated to `MAX_FILENAME_LEN - 1` bytes, matching
/// `epicsSnprintf`'s bound.
pub fn format_file_template(
    template: &str,
    string0: &str,
    string1: &str,
    integer: i32,
) -> Option<String> {
    let strings = [string0, string1];
    let mut next_string = 0usize;
    let mut out = String::new();
    let mut it = template.chars().peekable();

    while let Some(c) = it.next() {
        if c != '%' {
            out.push(c);
            continue;
        }
        if it.peek() == Some(&'%') {
            it.next();
            out.push('%');
            continue;
        }

        let mut spec = Spec::default();
        loop {
            match it.peek() {
                Some('-') => spec.minus = true,
                Some('0') => spec.zero = true,
                Some('+') => spec.plus = true,
                Some(' ') => spec.space = true,
                Some('#') => {}
                _ => break,
            }
            it.next();
        }
        let mut width = String::new();
        while let Some(d) = it.peek().copied() {
            if d.is_ascii_digit() {
                width.push(d);
                it.next();
            } else {
                break;
            }
        }
        if !width.is_empty() {
            spec.width = Some(width.parse().ok()?);
        }
        if it.peek() == Some(&'.') {
            it.next();
            let mut prec = String::new();
            while let Some(d) = it.peek().copied() {
                if d.is_ascii_digit() {
                    prec.push(d);
                    it.next();
                } else {
                    break;
                }
            }
            spec.prec = Some(if prec.is_empty() {
                0
            } else {
                prec.parse().ok()?
            });
        }
        // Length modifiers are accepted and ignored; every integer argument is
        // an `int`.
        while matches!(
            it.peek(),
            Some('h') | Some('l') | Some('L') | Some('z') | Some('j')
        ) {
            it.next();
        }

        match it.next()? {
            's' => {
                let s = strings.get(next_string)?;
                next_string += 1;
                out.push_str(&format_string(&spec, s));
            }
            'd' | 'i' => out.push_str(&format_signed(&spec, integer as i64)),
            'u' => out.push_str(&format_unsigned(&spec, integer as u32 as u64, 10, false)),
            'o' => out.push_str(&format_unsigned(&spec, integer as u32 as u64, 8, false)),
            'x' => out.push_str(&format_unsigned(&spec, integer as u32 as u64, 16, false)),
            'X' => out.push_str(&format_unsigned(&spec, integer as u32 as u64, 16, true)),
            _ => return None,
        }
    }

    if out.len() >= MAX_FILENAME_LEN {
        out.truncate(
            (0..MAX_FILENAME_LEN)
                .rev()
                .find(|&i| out.is_char_boundary(i))
                .unwrap_or(0),
        );
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn template_default_mar345_form() {
        // C default NDFileTemplate is "%s%s_%3.3d".
        assert_eq!(
            format_file_template("%s%s_%3.3d", "/data/", "test", 7).unwrap(),
            "/data/test_007"
        );
    }

    #[test]
    fn template_plain_d() {
        assert_eq!(
            format_file_template("%s%s_%d", "/data/", "image", 7).unwrap(),
            "/data/image_7"
        );
    }

    #[test]
    fn template_zero_flag_width() {
        assert_eq!(
            format_file_template("%s%s%05d", "/d/", "i", 42).unwrap(),
            "/d/i00042"
        );
    }

    #[test]
    fn template_literal_percent() {
        assert_eq!(format_file_template("100%%", "", "", 0).unwrap(), "100%");
    }

    #[test]
    fn template_too_many_strings_is_rejected() {
        assert_eq!(format_file_template("%s%s%s", "a", "b", 0), None);
    }

    #[test]
    fn template_unsupported_conversion_is_rejected() {
        assert_eq!(format_file_template("%f", "", "", 0), None);
    }
}
