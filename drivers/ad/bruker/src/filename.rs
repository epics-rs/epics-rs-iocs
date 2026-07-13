//! `FilePath` + `FileName` + `FileNumber` through `FileTemplate`
//! (C `asynNDArrayDriver::createFileName`, which is
//! `epicsSnprintf(buf, max, template, path, name, number)`).
//!
//! `ADDriverBase` in ad-core 0.23 does not carry `NDArrayDriverBase`'s
//! `create_file_name`, so the printf expansion lives here.

/// Expand a template of the form `%s%s_%3.3d.sfrm`: the first `%s` is the path,
/// the second the name, and the integer conversion the file number.
pub fn expand(template: &str, path: &str, name: &str, number: i32) -> String {
    let mut out = String::with_capacity(template.len() + path.len() + name.len() + 16);
    let mut chars = template.chars().peekable();
    let mut strings = [path, name].into_iter();

    while let Some(c) = chars.next() {
        if c != '%' {
            out.push(c);
            continue;
        }

        let mut left = false;
        let mut zero = false;
        while let Some(&c) = chars.peek() {
            match c {
                '-' => left = true,
                '0' => zero = true,
                '+' | ' ' | '#' => {}
                _ => break,
            }
            chars.next();
        }

        let mut width = String::new();
        while chars.peek().is_some_and(|c| c.is_ascii_digit()) {
            width.push(chars.next().unwrap());
        }
        let mut precision = String::new();
        if chars.peek() == Some(&'.') {
            chars.next();
            while chars.peek().is_some_and(|c| c.is_ascii_digit()) {
                precision.push(chars.next().unwrap());
            }
        }
        let width: usize = width.parse().unwrap_or(0);
        let precision: Option<usize> = precision.parse().ok();

        let body = match chars.next() {
            Some('%') => {
                out.push('%');
                continue;
            }
            Some('s') => {
                let mut s = strings.next().unwrap_or("").to_string();
                if let Some(p) = precision {
                    s.truncate(p);
                }
                s
            }
            Some('d') | Some('i') | Some('u') => {
                let digits = number.unsigned_abs().to_string();
                let digits = match precision {
                    Some(p) if digits.len() < p => "0".repeat(p - digits.len()) + &digits,
                    _ => digits,
                };
                if number < 0 {
                    format!("-{digits}")
                } else {
                    digits
                }
            }
            // A conversion the template has no argument for: C would have read
            // whatever was next on the stack. Keep it verbatim instead.
            Some(other) => {
                out.push('%');
                out.push(other);
                continue;
            }
            None => {
                out.push('%');
                break;
            }
        };

        if body.len() >= width {
            out.push_str(&body);
        } else if left {
            out.push_str(&body);
            out.push_str(&" ".repeat(width - body.len()));
        } else {
            out.push_str(&(if zero { "0" } else { " " }).repeat(width - body.len()));
            out.push_str(&body);
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_area_detector_default_template_names_a_numbered_file() {
        assert_eq!(
            expand("%s%s_%3.3d.sfrm", "/data/", "test", 1),
            "/data/test_001.sfrm"
        );
        assert_eq!(
            expand("%s%s_%3.3d.sfrm", "/data/", "test", 1234),
            "/data/test_1234.sfrm"
        );
    }

    #[test]
    fn width_flags_and_a_literal_percent_are_honoured() {
        assert_eq!(expand("%s%s%06d", "/d/", "a", 42), "/d/a000042");
        assert_eq!(expand("%s%s%6d", "/d/", "a", 42), "/d/a    42");
        assert_eq!(expand("%s%s%-6d|", "/d/", "a", 42), "/d/a42    |");
        assert_eq!(expand("100%%", "", "", 0), "100%");
    }

    #[test]
    fn an_empty_template_yields_an_empty_name() {
        // C parity: epicsSnprintf of an empty format writes nothing. No default
        // template is invented.
        assert_eq!(expand("", "/data/", "test", 1), "");
    }
}
