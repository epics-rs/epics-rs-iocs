//! camserver ASCII command formatting and reply parsing.
//!
//! Every function here is pure so the wire format can be checked against
//! `pilatusDetector.cpp` with fixture strings and no hardware.
//!
//! Framing note: `pilatusDetector` never appends a terminator itself. The
//! camserver asyn port is created by `st.cmd` with
//! `asynOctetSetOutputEos("camserver", 0, "\n")` and
//! `asynOctetSetInputEos("camserver", 0, "\x18")`, so the port appends `\n` to
//! every command and splits replies on the `0x18` (CAN) byte camserver sends
//! after each reply. The strings below therefore carry no terminator.

use crate::types::{GAIN_STRINGS, TriggerMode};

// ---------------------------------------------------------------------------
// scanf-style primitives
// ---------------------------------------------------------------------------

/// Skip the characters a `' '` directive in a scanf format skips.
pub(crate) fn skip_ws(s: &str) -> &str {
    s.trim_start_matches(|c: char| c.is_ascii_whitespace())
}

/// Match a literal that a scanf format would match verbatim (no whitespace
/// skipping of its own).
pub(crate) fn expect_lit<'a>(s: &'a str, lit: &str) -> Option<&'a str> {
    s.strip_prefix(lit)
}

/// `%d`: skip leading whitespace, then read an optionally-signed decimal.
pub(crate) fn scan_i32(s: &str) -> Option<(i32, &str)> {
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

/// `%f`: skip leading whitespace, then read the longest prefix `strtod` would
/// accept from the decimal grammar. Backtracks like `strtod` does, so
/// `"31.4C"` yields `31.4` and leaves `"C"`.
fn scan_f32(s: &str) -> Option<(f32, &str)> {
    let s = skip_ws(s);
    let candidate_len = s
        .bytes()
        .take_while(|b| b.is_ascii_digit() || matches!(b, b'+' | b'-' | b'.' | b'e' | b'E'))
        .count();
    for end in (1..=candidate_len).rev() {
        if let Ok(v) = s[..end].parse::<f32>() {
            return Some((v, &s[end..]));
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

/// C: `epicsSnprintf(toCamserver, ..., "nimages %d", ival)`
pub fn cmd_nimages(n: i32) -> String {
    format!("nimages {n}")
}

/// C: `"nexpframe %d"`
pub fn cmd_nexpframe(n: i32) -> String {
    format!("nexpframe {n}")
}

/// C: `"exptime %11.8f"`
pub fn cmd_exptime(v: f64) -> String {
    format!("exptime {v:11.8}")
}

/// C: `"expperiod %11.8f"`
pub fn cmd_expperiod(v: f64) -> String {
    format!("expperiod {v:11.8}")
}

/// C: `"delay %f"`
pub fn cmd_delay(v: f64) -> String {
    format!("delay {v:.6}")
}

/// C: `"gapfill %d"`
pub fn cmd_gapfill(v: i32) -> String {
    format!("gapfill {v}")
}

/// C `setThreshold`: `"SetThreshold energy %.0f %s %.0f"` with keV values
/// scaled to eV and the gain index clamped to `0..=3`.
pub fn cmd_set_threshold(energy_kev: f64, gain_index: i32, threshold_kev: f64) -> String {
    let gain = GAIN_STRINGS[gain_index.clamp(0, 3) as usize];
    format!(
        "SetThreshold energy {:.0} {} {:.0}",
        energy_kev * 1000.0,
        gain,
        threshold_kev * 1000.0
    )
}

/// C `setThreshold`: `igain = (int)(dgain + 0.5)`, clamped to `0..=3`.
pub fn gain_index(gain: f64) -> i32 {
    ((gain + 0.5) as i32).clamp(0, 3)
}

/// C: `"ResetModulePower %d"`
pub fn cmd_reset_module_power(reset_time: i32) -> String {
    format!("ResetModulePower {reset_time}")
}

/// C: `"HeaderString \"%s\""`
pub fn cmd_header_string(header: &str) -> String {
    format!("HeaderString \"{header}\"")
}

/// C `pilatusTask` trigger-mode switch: `"<verb> <fullFileName>"`.
pub fn cmd_exposure(mode: TriggerMode, full_file_name: &str) -> String {
    format!("{} {}", mode.command(), full_file_name)
}

/// C: `"imgpath %s"`
pub fn cmd_imgpath(path: &str) -> String {
    format!("imgpath {path}")
}

/// C: `"mxsettings <Name> %f"`
pub fn cmd_mx_f64(name: &str, v: f64) -> String {
    format!("mxsettings {name} {v:.6}")
}

/// C: `"mxsettings <Name> %f,%f"` (`Energy_range`, `Beam_xy`)
pub fn cmd_mx_pair(name: &str, a: f64, b: f64) -> String {
    format!("mxsettings {name} {a:.6},{b:.6}")
}

/// C: `"mxsettings N_oscillations %d"`
pub fn cmd_mx_i32(name: &str, v: i32) -> String {
    format!("mxsettings {name} {v}")
}

/// C: `"mxsettings Oscillation_axis %s"` with an empty value replaced by `(nil)`.
pub fn cmd_oscill_axis(value: &str) -> String {
    let v = if value.is_empty() { "(nil)" } else { value };
    format!("mxsettings Oscillation_axis {v}")
}

/// C: `"mxsettings cbf_template_file %s"` with an empty value replaced by `0`.
pub fn cmd_cbf_template_file(value: &str) -> String {
    let v = if value.is_empty() { "0" } else { value };
    format!("mxsettings cbf_template_file {v}")
}

// ---------------------------------------------------------------------------
// Reply parsing
// ---------------------------------------------------------------------------

/// C `readCamserver`: `if (!strstr(fromCamserver, "OK"))` → error.
pub fn reply_is_ok(reply: &str) -> bool {
    reply.contains("OK")
}

/// C `setAcquireParams`, `Tau` reply: `strstr(reply, "cutoff")` then
/// `sscanf(substr, "cutoff = %d counts", &pixelCutOff)`.
///
/// Returns `None` when `"cutoff"` is absent (C leaves `PIXEL_CUTOFF`
/// untouched). Returns `Some(0)` when the substring is present but the numeric
/// scan fails, because C's `pixelCutOff` is initialised to 0 and written
/// unconditionally once `strstr` matched.
pub fn parse_tau_cutoff(reply: &str) -> Option<i32> {
    let idx = reply.find("cutoff")?;
    let rest = &reply[idx + "cutoff".len()..];
    let rest = skip_ws(rest);
    let Some(rest) = expect_lit(rest, "=") else {
        return Some(0);
    };
    Some(scan_i32(rest).map(|(v, _)| v).unwrap_or(0))
}

/// C `setThreshold`, `SetThreshold` reply: `strstr(reply, "threshold: ")` then
/// `sscanf(strtok(substr, ";"), "threshold: %d eV", &threshold_readback)`.
///
/// Value is in eV. `None` when the marker is absent.
pub fn parse_threshold_ev(reply: &str) -> Option<i32> {
    let idx = reply.find("threshold: ")?;
    let sub = &reply[idx..];
    // strtok(substr, ";") truncates at the first ';'.
    let token = sub.split(';').next().unwrap_or(sub);
    let rest = expect_lit(token, "threshold:")?;
    Some(scan_i32(rest).map(|(v, _)| v).unwrap_or(0))
}

/// C `setThreshold`, `SetEnergy` reply:
/// `sscanf(fromCamserver, "15 OK Energy setting: %d eV", &energy_readback)`.
///
/// C initialises `energy_readback` to 0 and writes `ENERGY` unconditionally on
/// a successful read, so a non-matching reply yields 0. Value is in eV.
pub fn parse_energy_setting(reply: &str) -> i32 {
    let parsed = (|| {
        let s = expect_lit(reply, "15")?;
        let s = expect_lit(skip_ws(s), "OK")?;
        let s = expect_lit(skip_ws(s), "Energy")?;
        let s = expect_lit(skip_ws(s), "setting:")?;
        scan_i32(s).map(|(v, _)| v)
    })();
    parsed.unwrap_or(0)
}

/// Result of parsing the `version` reply.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VersionInfo {
    /// The text written to `TVXVERSION` and `SDK_VERSION`: everything after the
    /// last space of the reply (C `strrchr(fromCamserver, ' ') + 1`).
    pub version_string: String,
    /// `%d.%d.%d` fields. A field that failed to convert is `None`; C leaves the
    /// corresponding member at its previous value.
    pub major: Option<i32>,
    pub minor: Option<i32>,
    pub patch: Option<i32>,
}

/// C `pilatusStatus`, `version` reply.
///
/// Old firmware: `"Code release:  tvx-7.3.13-121212"`, new: `"Code release: 7.9.0"`.
/// C takes everything after the last space, publishes it verbatim, then skips a
/// leading `tvx-` (`if (substr[0] == 't') substr += 4`) before the numeric scan.
///
/// `None` when the reply contains no space at all (C would dereference
/// `NULL + 1`).
pub fn parse_version(reply: &str) -> Option<VersionInfo> {
    let idx = reply.rfind(' ')?;
    let version_string = reply[idx + 1..].to_string();

    let mut t: &str = &version_string;
    if t.starts_with('t') {
        t = t.get(4..).unwrap_or("");
    }

    let mut major = None;
    let mut minor = None;
    let mut patch = None;
    if let Some((v, rest)) = scan_i32(t) {
        major = Some(v);
        if let Some(rest) = expect_lit(rest, ".")
            && let Some((v, rest)) = scan_i32(rest)
        {
            minor = Some(v);
            if let Some(rest) = expect_lit(rest, ".")
                && let Some((v, _)) = scan_i32(rest)
            {
                patch = Some(v);
            }
        }
    }

    Some(VersionInfo {
        version_string,
        major,
        minor,
        patch,
    })
}

/// C `pilatusStatus`, `thread` reply, one channel:
/// `sscanf(substr, "Channel N: Temperature = %fC, Rel. Humidity = %f", &temp, &humid)`.
///
/// Returns `None` when `"Channel N"` is absent (C skips the whole `if` block).
/// The tuple mirrors sscanf's partial-assignment behaviour: a `None` field was
/// never converted, so C leaves its `temp` / `humid` local at the previous
/// channel's value.
pub fn parse_thread_channel(reply: &str, channel: u8) -> Option<(Option<f32>, Option<f32>)> {
    let key = format!("Channel {channel}");
    let idx = reply.find(&key)?;
    let s = &reply[idx + key.len()..];

    let scan = || -> (Option<f32>, Option<f32>) {
        let Some(s) = expect_lit(s, ":") else {
            return (None, None);
        };
        let Some(s) = expect_lit(skip_ws(s), "Temperature") else {
            return (None, None);
        };
        let Some(s) = expect_lit(skip_ws(s), "=") else {
            return (None, None);
        };
        let Some((temp, s)) = scan_f32(s) else {
            return (None, None);
        };
        let Some(s) = expect_lit(s, "C,") else {
            return (Some(temp), None);
        };
        let Some(s) = expect_lit(skip_ws(s), "Rel.") else {
            return (Some(temp), None);
        };
        let Some(s) = expect_lit(skip_ws(s), "Humidity") else {
            return (Some(temp), None);
        };
        let Some(s) = expect_lit(skip_ws(s), "=") else {
            return (Some(temp), None);
        };
        match scan_f32(s) {
            Some((humid, _)) => (Some(temp), Some(humid)),
            None => (Some(temp), None),
        }
    };

    Some(scan())
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- command formatting -------------------------------------------------

    #[test]
    fn exptime_matches_c_percent_11_8f() {
        // C: epicsSnprintf("exptime %11.8f", 1.0) -> "exptime  1.00000000"
        assert_eq!(cmd_exptime(1.0), "exptime  1.00000000");
        assert_eq!(cmd_exptime(0.5), "exptime  0.50000000");
        assert_eq!(cmd_exptime(12.25), "exptime 12.25000000");
        // Width 11 is a minimum, not a truncation.
        assert_eq!(cmd_exptime(1234.5), "exptime 1234.50000000");
    }

    #[test]
    fn expperiod_matches_c_percent_11_8f() {
        assert_eq!(cmd_expperiod(2.0), "expperiod  2.00000000");
    }

    #[test]
    fn delay_matches_c_percent_f() {
        assert_eq!(cmd_delay(0.0), "delay 0.000000");
        assert_eq!(cmd_delay(0.001), "delay 0.001000");
    }

    #[test]
    fn nimages_nexpframe_gapfill() {
        assert_eq!(cmd_nimages(10), "nimages 10");
        assert_eq!(cmd_nexpframe(1), "nexpframe 1");
        assert_eq!(cmd_gapfill(-1), "gapfill -1");
        assert_eq!(cmd_gapfill(0), "gapfill 0");
    }

    #[test]
    fn set_threshold_command() {
        // energy 20 keV, gain index 1 (midG), threshold 10 keV
        assert_eq!(
            cmd_set_threshold(20.0, 1, 10.0),
            "SetThreshold energy 20000 midG 10000"
        );
        assert_eq!(
            cmd_set_threshold(6.0, 3, 3.0),
            "SetThreshold energy 6000 uhighG 3000"
        );
    }

    #[test]
    fn gain_index_rounds_and_clamps() {
        assert_eq!(gain_index(0.0), 0);
        assert_eq!(gain_index(1.0), 1);
        assert_eq!(gain_index(2.6), 3);
        assert_eq!(gain_index(-5.0), 0);
        assert_eq!(gain_index(9.0), 3);
    }

    #[test]
    fn reset_module_power_command() {
        assert_eq!(cmd_reset_module_power(1), "ResetModulePower 1");
    }

    #[test]
    fn header_string_is_quoted() {
        assert_eq!(cmd_header_string(""), "HeaderString \"\"");
        assert_eq!(cmd_header_string("beam 1"), "HeaderString \"beam 1\"");
    }

    #[test]
    fn exposure_commands_per_trigger_mode() {
        let f = "/data/img_001.tif";
        assert_eq!(
            cmd_exposure(TriggerMode::Internal, f),
            "Exposure /data/img_001.tif"
        );
        assert_eq!(
            cmd_exposure(TriggerMode::ExternalEnable, f),
            "ExtEnable /data/img_001.tif"
        );
        assert_eq!(
            cmd_exposure(TriggerMode::ExternalTrigger, f),
            "ExtTrigger /data/img_001.tif"
        );
        assert_eq!(
            cmd_exposure(TriggerMode::MultipleExternalTrigger, f),
            "ExtMTrigger /data/img_001.tif"
        );
        assert_eq!(
            cmd_exposure(TriggerMode::Alignment, f),
            "Exposure /data/img_001.tif"
        );
    }

    #[test]
    fn mxsettings_commands() {
        assert_eq!(
            cmd_mx_f64("Start_angle", 1.5),
            "mxsettings Start_angle 1.500000"
        );
        assert_eq!(
            cmd_mx_f64("Wavelength", 1.54),
            "mxsettings Wavelength 1.540000"
        );
        assert_eq!(
            cmd_mx_pair("Energy_range", 0.0, 0.0),
            "mxsettings Energy_range 0.000000,0.000000"
        );
        assert_eq!(
            cmd_mx_pair("Beam_xy", 100.5, 200.25),
            "mxsettings Beam_xy 100.500000,200.250000"
        );
        assert_eq!(
            cmd_mx_i32("N_oscillations", 1),
            "mxsettings N_oscillations 1"
        );
        // Detector_distance / Detector_Voffset are sent in metres (value/1000).
        assert_eq!(
            cmd_mx_f64("Detector_distance", 1000.0 / 1000.0),
            "mxsettings Detector_distance 1.000000"
        );
    }

    #[test]
    fn oscill_axis_empty_becomes_nil() {
        assert_eq!(cmd_oscill_axis(""), "mxsettings Oscillation_axis (nil)");
        assert_eq!(
            cmd_oscill_axis("X, CW"),
            "mxsettings Oscillation_axis X, CW"
        );
    }

    #[test]
    fn cbf_template_file_empty_becomes_zero() {
        assert_eq!(cmd_cbf_template_file(""), "mxsettings cbf_template_file 0");
        assert_eq!(
            cmd_cbf_template_file("/tmp/t.cbf"),
            "mxsettings cbf_template_file /tmp/t.cbf"
        );
    }

    #[test]
    fn imgpath_command() {
        assert_eq!(cmd_imgpath("/data/"), "imgpath /data/");
    }

    // --- reply parsing ------------------------------------------------------

    #[test]
    fn ok_detection() {
        assert!(reply_is_ok("15 OK"));
        assert!(reply_is_ok("7 OK /data/img_001.tif"));
        assert!(!reply_is_ok("1 ERR *** Unrecognized command"));
        assert!(!reply_is_ok(""));
    }

    #[test]
    fn tau_cutoff() {
        assert_eq!(
            parse_tau_cutoff(
                "15 OK Rate correction is on; tau = 1.9e-07 s, cutoff = 1221026 counts"
            ),
            Some(1221026)
        );
        // Marker present, numeric scan fails -> C's zero-initialised local.
        assert_eq!(parse_tau_cutoff("cutoff = counts"), Some(0));
        assert_eq!(parse_tau_cutoff("cutoff"), Some(0));
        // Marker absent -> C never calls setIntegerParam.
        assert_eq!(parse_tau_cutoff("15 OK Rate correction is off"), None);
    }

    #[test]
    fn threshold_readback() {
        assert_eq!(
            parse_threshold_ev(
                "15 OK  Settings: mid gain; threshold: 9000 eV; vcmp: 0.700 V\n Trim file:"
            ),
            Some(9000)
        );
        assert_eq!(parse_threshold_ev("threshold: 4024 eV;"), Some(4024));
        assert_eq!(parse_threshold_ev("15 OK Settings: mid gain"), None);
    }

    #[test]
    fn threshold_readback_stops_at_semicolon() {
        // strtok truncates at ';' — a later "threshold:" must not leak in.
        assert_eq!(
            parse_threshold_ev("threshold: 9000 eV; threshold: 1111 eV"),
            Some(9000)
        );
    }

    #[test]
    fn energy_setting_readback() {
        assert_eq!(
            parse_energy_setting("15 OK Energy setting: 12000 eV"),
            12000
        );
        // Any deviation from the literal prefix leaves C's zero-initialised local.
        assert_eq!(parse_energy_setting("15 OK Energy setting: eV"), 0);
        assert_eq!(parse_energy_setting("1 ERR"), 0);
        assert_eq!(parse_energy_setting(""), 0);
    }

    #[test]
    fn version_old_tvx_format() {
        let v = parse_version("15 OK Code release:  tvx-7.3.13-121212").unwrap();
        assert_eq!(v.version_string, "tvx-7.3.13-121212");
        assert_eq!((v.major, v.minor, v.patch), (Some(7), Some(3), Some(13)));
    }

    #[test]
    fn version_new_format() {
        let v = parse_version("24 OK Code release: 7.9.0").unwrap();
        assert_eq!(v.version_string, "7.9.0");
        assert_eq!((v.major, v.minor, v.patch), (Some(7), Some(9), Some(0)));
    }

    #[test]
    fn version_partial_scan_leaves_fields_unset() {
        let v = parse_version("Code release: 7.9").unwrap();
        assert_eq!(v.version_string, "7.9");
        assert_eq!((v.major, v.minor, v.patch), (Some(7), Some(9), None));
    }

    #[test]
    fn version_without_space_is_none() {
        assert_eq!(parse_version("7.9.0"), None);
    }

    #[test]
    fn thread_channels() {
        let reply = "215 OK Channel 0: Temperature = 31.4C, Rel. Humidity = 22.1%;\n\
                     Channel 1: Temperature = 25.8C, Rel. Humidity = 33.5%;\n\
                     Channel 2: Temperature = 28.6C, Rel. Humidity = 2.0%";
        assert_eq!(
            parse_thread_channel(reply, 0),
            Some((Some(31.4), Some(22.1)))
        );
        assert_eq!(
            parse_thread_channel(reply, 1),
            Some((Some(25.8), Some(33.5)))
        );
        assert_eq!(
            parse_thread_channel(reply, 2),
            Some((Some(28.6), Some(2.0)))
        );
        assert_eq!(parse_thread_channel(reply, 3), None);
    }

    #[test]
    fn thread_channel_partial_conversion() {
        // Temperature converts, humidity never does -> C keeps the old humid.
        assert_eq!(
            parse_thread_channel("Channel 0: Temperature = 31.4C, Rel. Humidity = ", 0),
            Some((Some(31.4), None))
        );
        // Nothing converts.
        assert_eq!(
            parse_thread_channel("Channel 0: Temp = 31.4C", 0),
            Some((None, None))
        );
    }

    #[test]
    fn thread_channel_negative_temperature() {
        assert_eq!(
            parse_thread_channel("Channel 0: Temperature = -1.5C, Rel. Humidity = 10.0%", 0),
            Some((Some(-1.5), Some(10.0)))
        );
    }

    #[test]
    fn scan_f32_backtracks_like_strtod() {
        assert_eq!(scan_f32("31.4C").unwrap().0, 31.4);
        assert_eq!(scan_f32("31.4C").unwrap().1, "C");
        // Trailing 'e' has no exponent digits; strtod backtracks to "31.4".
        assert_eq!(scan_f32("31.4eC").unwrap().0, 31.4);
        assert_eq!(scan_f32("22.1%").unwrap().1, "%");
        assert!(scan_f32("abc").is_none());
    }

    #[test]
    fn scan_i32_skips_leading_whitespace() {
        assert_eq!(scan_i32("   42 counts").unwrap(), (42, " counts"));
        assert_eq!(scan_i32("-7").unwrap(), (-7, ""));
        assert!(scan_i32("  eV").is_none());
    }
}
