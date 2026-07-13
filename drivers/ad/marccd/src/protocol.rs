//! marccd server ASCII command formatting and reply parsing.
//!
//! Every function here is pure so the wire format can be checked against
//! `marCCD.cpp` with fixture strings and no hardware.
//!
//! Framing note: `marCCD` never appends a terminator itself. The server asyn
//! port is created by `st.cmd` with `asynOctetSetInputEos("marServer",0,"\n")`
//! and `asynOctetSetOutputEos("marServer",0,"\n")`, so the port appends `\n` to
//! every command and splits replies on `\n`. The strings below carry no
//! terminator.

// ---------------------------------------------------------------------------
// scanf-style primitives
// ---------------------------------------------------------------------------

/// Skip the characters a `' '` directive (and the implicit skip before `%d`)
/// consumes in a scanf format.
fn skip_ws(s: &str) -> &str {
    s.trim_start_matches(|c: char| c.is_ascii_whitespace())
}

/// Match a literal that a scanf format matches verbatim (no whitespace skip).
fn expect_lit<'a>(s: &'a str, lit: &str) -> Option<&'a str> {
    s.strip_prefix(lit)
}

/// `%d`: skip leading whitespace, then read an optionally-signed decimal.
fn scan_i32(s: &str) -> Option<(i32, &str)> {
    let s = skip_ws(s);
    let bytes = s.as_bytes();
    let mut end = 0;
    if end < bytes.len() && (bytes[end] == b'+' || bytes[end] == b'-') {
        end += 1;
    }
    let digits_start = end;
    while end < bytes.len() && bytes[end].is_ascii_digit() {
        end += 1;
    }
    if end == digits_start {
        return None;
    }
    s[..end].parse::<i32>().ok().map(|v| (v, &s[end..]))
}

/// `%lf`: skip leading whitespace, then read the longest prefix `strtod` would
/// accept, backtracking on a trailing exponent marker the way `strtod` does.
fn scan_f64(s: &str) -> Option<(f64, &str)> {
    let s = skip_ws(s);
    let candidate_len = s
        .bytes()
        .take_while(|b| b.is_ascii_digit() || matches!(b, b'+' | b'-' | b'.' | b'e' | b'E'))
        .count();
    for end in (1..=candidate_len).rev() {
        if let Ok(v) = s[..end].parse::<f64>() {
            return Some((v, &s[end..]));
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

/// C `epicsSnprintf(toServer, ..., "%f", v)` — the plain `printf` `%f`, six
/// fractional digits.
fn f(v: f64) -> String {
    format!("{v:.6}")
}

/// C: `"readout,%d"` / `"readout,%d,%s"`.
pub fn cmd_readout(buffer_number: i32, file_name: Option<&str>) -> String {
    match file_name {
        Some(name) if !name.is_empty() => format!("readout,{buffer_number},{name}"),
        _ => format!("readout,{buffer_number}"),
    }
}

/// C: `"writefile,%s,%d"`.
pub fn cmd_writefile(full_file_name: &str, corrected_flag: i32) -> String {
    format!("writefile,{full_file_name},{corrected_flag}")
}

/// C: `"set_bin,%d,%d"`.
pub fn cmd_set_bin(bin_x: i32, bin_y: i32) -> String {
    format!("set_bin,{bin_x},{bin_y}")
}

/// C: `"set_gating,%d"`.
pub fn cmd_set_gating(value: i32) -> String {
    format!("set_gating,{value}")
}

/// C: `"set_readout_mode,%d"`.
pub fn cmd_set_readout_mode(value: i32) -> String {
    format!("set_readout_mode,{value}")
}

/// C: `"set_frameshift,%d"`.
pub fn cmd_set_frameshift(value: i32) -> String {
    format!("set_frameshift,{value}")
}

/// C: `"set_stability,%f"`.
pub fn cmd_set_stability(value: f64) -> String {
    format!("set_stability,{}", f(value))
}

/// C: `"shutter,1"` / `"shutter,0"`.
pub fn cmd_shutter(open: bool) -> &'static str {
    if open { "shutter,1" } else { "shutter,0" }
}

/// C: `"dezinger,%d"` (`dezinger,0` for double-correlation, `dezinger,1` for
/// background).
pub fn cmd_dezinger(value: i32) -> String {
    format!("dezinger,{value}")
}

/// C `writeHeader` first line: `header,detector_distance=%f,beam_x=%f,`
/// `beam_y=%f,exposure_time=%f,` (note the trailing comma).
pub fn cmd_header_1(
    detector_distance: f64,
    beam_x: f64,
    beam_y: f64,
    exposure_time: f64,
) -> String {
    format!(
        "header,detector_distance={},beam_x={},beam_y={},exposure_time={},",
        f(detector_distance),
        f(beam_x),
        f(beam_y),
        f(exposure_time),
    )
}

/// C `writeHeader` second line: `header,start_phi=%f,rotation_axis=%s,`
/// `rotation_range=%f,twotheta=%f,source_wavelength=%f,file_comments=%s,`
/// `dataset_comments=%s`.
pub fn cmd_header_2(
    start_phi: f64,
    rotation_axis: &str,
    rotation_range: f64,
    two_theta: f64,
    wavelength: f64,
    file_comments: &str,
    dataset_comments: &str,
) -> String {
    format!(
        "header,start_phi={},rotation_axis={},rotation_range={},twotheta={},\
         source_wavelength={},file_comments={},dataset_comments={}",
        f(start_phi),
        rotation_axis,
        f(rotation_range),
        f(two_theta),
        f(wavelength),
        file_comments,
        dataset_comments,
    )
}

/// C `collectSeries`, `marCCDImageSeriesTriggered` branch. When `trigger_mode`
/// is `Timed` the first field is the exposure time (`%f`); otherwise it is the
/// integer `itemp` (`%d`). `first`, `base_file_name`, `file_suffix` and
/// `digits` follow.
pub fn cmd_start_series_triggered_timed(
    acquire_time: f64,
    num_images: i32,
    first: i32,
    base_file_name: &str,
    file_suffix: &str,
    digits: i32,
) -> String {
    format!(
        "start_series_triggered,{},{num_images},{first},{base_file_name},{file_suffix},{digits}",
        f(acquire_time),
    )
}

/// C `collectSeries`, non-`Timed` triggered branch (`itemp` is `0` for
/// internal/frame, `1` for bulb).
pub fn cmd_start_series_triggered_itemp(
    itemp: i32,
    num_images: i32,
    first: i32,
    base_file_name: &str,
    file_suffix: &str,
    digits: i32,
) -> String {
    format!(
        "start_series_triggered,{itemp},{num_images},{first},{base_file_name},{file_suffix},{digits}"
    )
}

/// C `collectSeries`, `marCCDImageSeriesTimed` branch:
/// `start_series_timed,%d,%d,%f,%f,%s,%s,%d`.
pub fn cmd_start_series_timed(
    num_images: i32,
    first: i32,
    acquire_time: f64,
    acquire_period: f64,
    base_file_name: &str,
    file_suffix: &str,
    digits: i32,
) -> String {
    format!(
        "start_series_timed,{num_images},{first},{},{},{base_file_name},{file_suffix},{digits}",
        f(acquire_time),
        f(acquire_period),
    )
}

// ---------------------------------------------------------------------------
// Reply parsing
// ---------------------------------------------------------------------------

/// C `getState`: `strtol(fromServer, NULL, 0)`. Base 0 auto-detects a `0x`/`0X`
/// hex prefix, a leading-`0` octal prefix, or decimal; skips leading whitespace
/// and an optional sign; stops at the first character not valid for the radix.
/// Returns `0` when no digits convert, matching `strtol`.
pub fn parse_state(reply: &str) -> i32 {
    let s = skip_ws(reply);
    let bytes = s.as_bytes();
    let mut i = 0;
    let mut negative = false;
    if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
        negative = bytes[i] == b'-';
        i += 1;
    }

    let (radix, digits_start) = if i < bytes.len()
        && bytes[i] == b'0'
        && i + 1 < bytes.len()
        && (bytes[i + 1] == b'x' || bytes[i + 1] == b'X')
    {
        (16u32, i + 2)
    } else if i < bytes.len() && bytes[i] == b'0' {
        // Octal. The leading '0' is itself a valid octal digit, so the run
        // starts at the '0'.
        (8u32, i)
    } else {
        (10u32, i)
    };

    let mut end = digits_start;
    while end < bytes.len() && (bytes[end] as char).is_digit(radix) {
        end += 1;
    }
    if end == digits_start {
        return 0;
    }
    let magnitude = i64::from_str_radix(&s[digits_start..end], radix).unwrap_or(0);
    let signed = if negative { -magnitude } else { magnitude };
    signed as i32
}

/// C `sscanf(fromServer, "%d", &value)`. `None` when no integer converts (C
/// leaves the destination unchanged).
pub fn parse_int(reply: &str) -> Option<i32> {
    scan_i32(reply).map(|(v, _)| v)
}

/// C `sscanf(fromServer, "%d,%d", &a, &b)`. Returns both values, or `None` if
/// the two-integer, comma-separated form does not match.
pub fn parse_pair(reply: &str) -> Option<(i32, i32)> {
    let (a, rest) = scan_i32(reply)?;
    let rest = expect_lit(rest, ",")?;
    let (b, _) = scan_i32(rest)?;
    Some((a, b))
}

/// C `sscanf(fromServer, "%lf", &value)`. `None` when no float converts.
pub fn parse_f64(reply: &str) -> Option<f64> {
    scan_f64(reply).map(|(v, _)| v)
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- command formatting -------------------------------------------------

    #[test]
    fn readout_with_and_without_file() {
        assert_eq!(cmd_readout(0, None), "readout,0");
        assert_eq!(cmd_readout(3, Some("")), "readout,3");
        assert_eq!(
            cmd_readout(0, Some("/data/img_001.tif")),
            "readout,0,/data/img_001.tif"
        );
    }

    #[test]
    fn writefile_command() {
        assert_eq!(
            cmd_writefile("/data/img_001.tif", 1),
            "writefile,/data/img_001.tif,1"
        );
        assert_eq!(
            cmd_writefile("/data/raw.tif", 0),
            "writefile,/data/raw.tif,0"
        );
    }

    #[test]
    fn set_commands() {
        assert_eq!(cmd_set_bin(2, 2), "set_bin,2,2");
        assert_eq!(cmd_set_gating(1), "set_gating,1");
        assert_eq!(cmd_set_readout_mode(4), "set_readout_mode,4");
        assert_eq!(cmd_set_frameshift(7), "set_frameshift,7");
        assert_eq!(cmd_set_stability(0.5), "set_stability,0.500000");
    }

    #[test]
    fn shutter_and_dezinger() {
        assert_eq!(cmd_shutter(true), "shutter,1");
        assert_eq!(cmd_shutter(false), "shutter,0");
        assert_eq!(cmd_dezinger(0), "dezinger,0");
        assert_eq!(cmd_dezinger(1), "dezinger,1");
    }

    #[test]
    fn header_lines_match_c_format() {
        assert_eq!(
            cmd_header_1(100.0, 1000.5, 1024.25, 1.0),
            "header,detector_distance=100.000000,beam_x=1000.500000,\
             beam_y=1024.250000,exposure_time=1.000000,"
        );
        assert_eq!(
            cmd_header_2(0.0, "phi", 1.0, 0.0, 1.54, "fc", "dc"),
            "header,start_phi=0.000000,rotation_axis=phi,rotation_range=1.000000,\
             twotheta=0.000000,source_wavelength=1.540000,file_comments=fc,\
             dataset_comments=dc"
        );
    }

    #[test]
    fn series_triggered_timed_form() {
        // trigger mode Timed uses the exposure time (%f) as the first field.
        assert_eq!(
            cmd_start_series_triggered_timed(0.25, 10, 1, "/d/base", ".tif", 5),
            "start_series_triggered,0.250000,10,1,/d/base,.tif,5"
        );
    }

    #[test]
    fn series_triggered_itemp_form() {
        assert_eq!(
            cmd_start_series_triggered_itemp(0, 10, 1, "/d/base", ".tif", 5),
            "start_series_triggered,0,10,1,/d/base,.tif,5"
        );
        assert_eq!(
            cmd_start_series_triggered_itemp(1, 5, 2, "/d/b", ".tif", 4),
            "start_series_triggered,1,5,2,/d/b,.tif,4"
        );
    }

    #[test]
    fn series_timed_form() {
        assert_eq!(
            cmd_start_series_timed(10, 1, 0.5, 1.0, "/d/base", ".tif", 5),
            "start_series_timed,10,1,0.500000,1.000000,/d/base,.tif,5"
        );
    }

    // --- reply parsing ------------------------------------------------------

    #[test]
    fn state_decimal() {
        assert_eq!(parse_state("0"), 0);
        assert_eq!(parse_state("8"), 8);
        assert_eq!(parse_state("32"), 32);
        assert_eq!(parse_state("  514 "), 514);
    }

    #[test]
    fn state_hex() {
        // strtol base 0 auto-detects 0x.
        assert_eq!(parse_state("0x22"), 0x22);
        assert_eq!(parse_state("0X200"), 0x200);
        // acquire executing (task 0 nibble = 2) + state busy(8): 0x20 | 0x8.
        assert_eq!(parse_state("0x28"), 0x28);
    }

    #[test]
    fn state_octal() {
        // Leading 0 -> octal.
        assert_eq!(parse_state("010"), 8);
        assert_eq!(parse_state("017"), 15);
    }

    #[test]
    fn state_stops_at_invalid_char() {
        assert_eq!(parse_state("32 something"), 32);
        assert_eq!(parse_state("0x1fZ"), 0x1f);
        // '8' and '9' are not octal digits: "089" stops after "0".
        assert_eq!(parse_state("089"), 0);
    }

    #[test]
    fn state_no_digits_is_zero() {
        assert_eq!(parse_state(""), 0);
        assert_eq!(parse_state("error"), 0);
    }

    #[test]
    fn int_and_pair() {
        assert_eq!(parse_int("1"), Some(1));
        assert_eq!(parse_int("2 high speed"), Some(2));
        assert_eq!(parse_int("junk"), None);
        assert_eq!(parse_pair("2048,2048"), Some((2048, 2048)));
        assert_eq!(parse_pair("512,1024"), Some((512, 1024)));
        // Missing the second field: sscanf would assign only the first; we
        // require both, so this is None.
        assert_eq!(parse_pair("512"), None);
    }

    #[test]
    fn float_reply() {
        assert_eq!(parse_f64("0.5"), Some(0.5));
        assert_eq!(parse_f64("  1.25 "), Some(1.25));
        assert_eq!(parse_f64("-3.0 C"), Some(-3.0));
        assert_eq!(parse_f64("nan-ish"), None);
    }
}
