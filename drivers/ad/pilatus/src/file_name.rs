//! File-name construction.
//!
//! `ADDriverBase` in ad-core-rs 0.22.1 does not expose `createFileName` /
//! `checkPath` (they live on the C `asynNDArrayDriver`, and the crate's
//! `sprintf_template` helper is private), so the printf subset that
//! `NDFileTemplate` needs is reimplemented here together with
//! `pilatusDetector::makeMultipleFileFormat`.

use crate::types::MAX_FILENAME_LEN;

// ---------------------------------------------------------------------------
// printf subset for NDFileTemplate
// ---------------------------------------------------------------------------

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

/// `%d` / `%i`: precision is a minimum digit count (zero-padded), applied
/// before the sign; the `0` flag pads the *field* with zeros but is ignored
/// when a precision is present or when `-` is given.
fn format_signed(spec: &Spec, value: i64) -> String {
    let digits = value.unsigned_abs().to_string();
    let digits = match spec.prec {
        Some(p) if digits.len() < p => format!("{}{digits}", "0".repeat(p - digits.len())),
        Some(0) if value == 0 => String::new(),
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

/// Expand `NDFileTemplate` the way `asynNDArrayDriver::createFileName` does:
/// `epicsSnprintf(fullFileName, maxChars, fileTemplate, filePath, fileName, fileNumber)`.
///
/// Returns `None` when the template needs more than the two string arguments or
/// uses a conversion this driver's templates never contain (C would read past
/// the argument list — undefined behaviour we refuse instead of imitating).
/// The result is truncated to `MAX_FILENAME_LEN - 1` bytes, matching
/// `epicsSnprintf`'s bound; C returns `asynError` when the value is truncated,
/// which the caller reproduces by comparing lengths.
pub fn format_file_template(
    template: &str,
    file_path: &str,
    file_name: &str,
    file_number: i32,
) -> Option<String> {
    let strings = [file_path, file_name];
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
            'd' | 'i' => out.push_str(&format_signed(&spec, file_number as i64)),
            'u' => out.push_str(&format_unsigned(
                &spec,
                file_number as u32 as u64,
                10,
                false,
            )),
            'o' => out.push_str(&format_unsigned(&spec, file_number as u32 as u64, 8, false)),
            'x' => out.push_str(&format_unsigned(
                &spec,
                file_number as u32 as u64,
                16,
                false,
            )),
            'X' => out.push_str(&format_unsigned(&spec, file_number as u32 as u64, 16, true)),
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

/// C `asynNDArrayDriver::checkPath` — strips one trailing separator and reports
/// whether the result is an existing directory.
pub fn check_path(file_path: &str) -> bool {
    if file_path.is_empty() {
        return false;
    }
    let trimmed = match file_path.chars().last() {
        Some('/') | Some('\\') => &file_path[..file_path.len() - 1],
        _ => file_path,
    };
    if trimmed.is_empty() {
        // C stat("")s and fails.
        return false;
    }
    std::path::Path::new(trimmed).is_dir()
}

// ---------------------------------------------------------------------------
// makeMultipleFileFormat
// ---------------------------------------------------------------------------

/// Result of `pilatusDetector::makeMultipleFileFormat` — the equivalent of
/// C's `multipleFileFormat` (`"<prefix>%.<digits>d<extension>"`) together with
/// the initial `multipleFileNumber`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MultipleFileFormat {
    pub prefix: String,
    pub digits: usize,
    pub extension: String,
    /// C `multipleFileNumber` after the call.
    pub start_number: i32,
}

impl MultipleFileFormat {
    /// C `epicsSnprintf(fullFileName, ..., multipleFileFormat, multipleFileNumber)`.
    pub fn file_name(&self, number: i32) -> String {
        let spec = Spec {
            prec: Some(self.digits),
            ..Spec::default()
        };
        format!(
            "{}{}{}",
            self.prefix,
            format_signed(&spec, number as i64),
            self.extension
        )
    }
}

fn is_digit(bytes: &[u8], i: usize) -> bool {
    bytes.get(i).is_some_and(u8::is_ascii_digit)
}

/// C `pilatusDetector::makeMultipleFileFormat`, ported verbatim including its
/// truncation of trailing non-digit characters after a numeric suffix.
pub fn make_multiple_file_format(base_file_name: &str, num_images: i32) -> MultipleFileFormat {
    // strncpy(mfTempFormat, baseFileName, sizeof(mfTempFormat))
    let mut temp: Vec<u8> = base_file_name.as_bytes().to_vec();
    temp.truncate(MAX_FILENAME_LEN - 1);

    // p = mfTempFormat + strlen(mfTempFormat) - 5;  if ((q = strrchr(p, '.')))
    // Only the last five bytes are searched for the extension.
    let tail_start = temp.len().saturating_sub(5);
    let dot = temp[tail_start..]
        .iter()
        .rposition(|&b| b == b'.')
        .map(|rel| tail_start + rel);
    let extension = match dot {
        Some(dot) => {
            let ext = String::from_utf8_lossy(&temp[dot..]).into_owned();
            temp.truncate(dot);
            ext
        }
        None => String::new(),
    };

    let mut multiple_file_number = 0i32;
    let mut fmt = 5usize;

    // p = strrchr(mfTempFormat, '/') ? : mfTempFormat
    let p = temp.iter().rposition(|&b| b == b'/').unwrap_or(0);

    match temp[p..].iter().rposition(|&b| b == b'_') {
        Some(rel) => {
            let q = p + rel + 1; // q++
            if is_digit(&temp, q) && is_digit(&temp, q + 1) && is_digit(&temp, q + 2) {
                // atoi(q) stops at the first non-digit.
                let digits_end = (q..temp.len())
                    .find(|&i| !temp[i].is_ascii_digit())
                    .unwrap_or(temp.len());
                multiple_file_number = std::str::from_utf8(&temp[q..digits_end])
                    .ok()
                    .and_then(|s| s.parse::<i32>().ok())
                    .unwrap_or(0);
                fmt = digits_end - q;
                // *p = '\0' with p == q: drops the digits *and* anything after.
                temp.truncate(q);
                if fmt < 3 || (fmt == 3 && num_images > 999) || (fmt == 4 && num_images > 9999) {
                    fmt = 5;
                }
            } else if q < temp.len() {
                // else if (*q) — the name does not already end with '_'.
                temp.push(b'_');
            }
        }
        None => temp.push(b'_'),
    }

    MultipleFileFormat {
        prefix: String::from_utf8_lossy(&temp).into_owned(),
        digits: fmt,
        extension,
        start_number: multiple_file_number,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn template_default_pilatus_form() {
        assert_eq!(
            format_file_template("%s%s_%3.3d.tif", "/data/", "image", 7).unwrap(),
            "/data/image_007.tif"
        );
    }

    #[test]
    fn template_plain_d() {
        assert_eq!(
            format_file_template("%s%s_%d.tif", "/data/", "image", 7).unwrap(),
            "/data/image_7.tif"
        );
    }

    #[test]
    fn template_zero_flag_width() {
        assert_eq!(
            format_file_template("%s%s%05d.tif", "/d/", "i", 42).unwrap(),
            "/d/i00042.tif"
        );
    }

    #[test]
    fn template_left_justified_width() {
        assert_eq!(
            format_file_template("[%-6d]", "", "", 42).unwrap(),
            "[42    ]"
        );
    }

    #[test]
    fn template_string_precision_truncates() {
        assert_eq!(
            format_file_template("%.3s%s", "abcdef", "X", 0).unwrap(),
            "abcX"
        );
    }

    #[test]
    fn template_literal_percent() {
        assert_eq!(format_file_template("100%%", "", "", 0).unwrap(), "100%");
    }

    #[test]
    fn template_negative_number_precision() {
        // C: printf("%.5d", -1) -> "-00001"
        assert_eq!(format_file_template("%.5d", "", "", -1).unwrap(), "-00001");
    }

    #[test]
    fn template_hex_and_octal() {
        assert_eq!(format_file_template("%x", "", "", 255).unwrap(), "ff");
        assert_eq!(format_file_template("%X", "", "", 255).unwrap(), "FF");
        assert_eq!(format_file_template("%o", "", "", 8).unwrap(), "10");
    }

    #[test]
    fn template_too_many_strings_is_rejected() {
        assert_eq!(format_file_template("%s%s%s", "a", "b", 0), None);
    }

    #[test]
    fn template_unsupported_conversion_is_rejected() {
        assert_eq!(format_file_template("%f", "", "", 0), None);
    }

    #[test]
    fn multiple_format_numeric_suffix_three_digits() {
        let m = make_multiple_file_format("/data/image_001.tif", 100);
        assert_eq!(m.prefix, "/data/image_");
        assert_eq!(m.digits, 3);
        assert_eq!(m.extension, ".tif");
        assert_eq!(m.start_number, 1);
        assert_eq!(m.file_name(1), "/data/image_001.tif");
        assert_eq!(m.file_name(42), "/data/image_042.tif");
        assert_eq!(m.file_name(1234), "/data/image_1234.tif");
    }

    #[test]
    fn multiple_format_three_digits_promoted_when_num_images_large() {
        let m = make_multiple_file_format("/data/image_001.tif", 1000);
        assert_eq!(m.digits, 5);
        assert_eq!(m.file_name(1), "/data/image_00001.tif");
    }

    #[test]
    fn multiple_format_four_digits_kept_and_promoted() {
        assert_eq!(make_multiple_file_format("/d/i_0001.tif", 9999).digits, 4);
        assert_eq!(make_multiple_file_format("/d/i_0001.tif", 10000).digits, 5);
    }

    #[test]
    fn multiple_format_two_digit_suffix_is_not_numeric() {
        // isdigit(*q) && isdigit(*(q+1)) && isdigit(*(q+2)) fails on "12.".
        // Wait: the extension has already been stripped, so *(q+2) is NUL.
        let m = make_multiple_file_format("/d/i_12.tif", 10);
        assert_eq!(m.prefix, "/d/i_12_");
        assert_eq!(m.digits, 5);
        assert_eq!(m.file_name(0), "/d/i_12_00000.tif");
    }

    #[test]
    fn multiple_format_trailing_junk_after_digits_is_dropped() {
        // C: p = q; while (isdigit(*q)) ...; *p = '\0'  -> "abc" is lost.
        let m = make_multiple_file_format("/d/i_001abc.tif", 10);
        assert_eq!(m.prefix, "/d/i_");
        assert_eq!(m.digits, 3);
        assert_eq!(m.start_number, 1);
    }

    #[test]
    fn multiple_format_no_underscore_appends_one() {
        let m = make_multiple_file_format("/data/image.tif", 10);
        assert_eq!(m.prefix, "/data/image_");
        assert_eq!(m.digits, 5);
        assert_eq!(m.start_number, 0);
        assert_eq!(m.file_name(0), "/data/image_00000.tif");
    }

    #[test]
    fn multiple_format_underscore_at_end_is_kept_once() {
        let m = make_multiple_file_format("/data/image_.tif", 10);
        assert_eq!(m.prefix, "/data/image_");
        assert_eq!(m.digits, 5);
    }

    #[test]
    fn multiple_format_non_numeric_suffix_appends_underscore() {
        let m = make_multiple_file_format("/data/image_abc.tif", 10);
        assert_eq!(m.prefix, "/data/image_abc_");
        assert_eq!(m.digits, 5);
    }

    #[test]
    fn multiple_format_underscore_only_in_directory_part() {
        // strrchr(p, '_') searches only after the last '/', so the directory's
        // underscore is invisible and a '_' is appended to the file name.
        let m = make_multiple_file_format("/my_data/image.tif", 10);
        assert_eq!(m.prefix, "/my_data/image_");
    }

    #[test]
    fn multiple_format_tiff_extension() {
        let m = make_multiple_file_format("/d/i_001.tiff", 10);
        assert_eq!(m.extension, ".tiff");
        assert_eq!(m.prefix, "/d/i_");
    }

    #[test]
    fn multiple_format_no_extension() {
        // Only the last five bytes are searched for '.', so an early dot is
        // not treated as an extension.
        let m = make_multiple_file_format("/d.x/imagefile", 10);
        assert_eq!(m.extension, "");
        assert_eq!(m.prefix, "/d.x/imagefile_");
    }

    #[test]
    fn multiple_format_alignment_file() {
        let m = make_multiple_file_format("/data/alignment.tif", 1);
        assert_eq!(m.prefix, "/data/alignment_");
        assert_eq!(m.extension, ".tif");
    }

    #[test]
    fn check_path_reports_directories() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().to_str().unwrap();
        assert!(check_path(p));
        assert!(check_path(&format!("{p}/")));
        assert!(!check_path(&format!("{p}/nope")));
        assert!(!check_path(""));
        assert!(!check_path("/"));
    }
}
