//! What goes over the two wires: the ASCII command channel (C
//! `writeReadServer` and the `epicsSnprintf` call sites) and the status
//! broadcast (C `statusTask`).
//!
//! Everything here is a pure function of bytes in and bytes out, so it is
//! tested without a detector.

use crate::types::{
    CoolingStatus, DEW_POINT_ERROR, DEW_POINT_WARNING, FrameType, HVMode, SyncOutFunction,
    SyncPolarity, TCOLD_ERROR, TCOLD_WARNING, THOT_ERROR, THOT_WARNING, TriggerMode,
};

/// `DAQ:! SYSTEM_RESET`.
pub fn system_reset() -> String {
    "DAQ:! SYSTEM_RESET".into()
}

/// `DAQ:! INIT <cooling> <coolingOn> <HV> <HVOn>` (C `setCoolingAndHV`).
pub fn init(cooling_value: f64, cooling_state: i32, hv_value: f64, hv_state: i32) -> String {
    format!("DAQ:! INIT {cooling_value:.1} {cooling_state} {hv_value:.1} {hv_state}")
}

/// `DAQ:! SET_SYNC <in> <out> <function>` (C `setSync`).
pub fn set_sync(
    sync_in: SyncPolarity,
    sync_out: SyncPolarity,
    function: SyncOutFunction,
) -> String {
    format!(
        "DAQ:! SET_SYNC {} {} {}",
        sync_in.as_str(),
        sync_out.as_str(),
        function.as_str()
    )
}

/// `DAQ:! SET_PIII_CONF ...` (C `setAutoCalParams`).
pub fn set_piii_conf(ofs0: i32, fs0: i32, ofs2: i32, fs1: i32, fs2: i32, ibias: i32) -> String {
    format!("DAQ:! SET_PIII_CONF {ofs0} {fs0} {ofs2} {fs1} {fs2} {ibias}")
}

/// The Pixie-III form of `SET_SENSOR_OPERATINGS` (C `setThresholds`).
pub fn set_sensor_operatings_piii(
    threshold_regs: &[i32; 5],
    readout_mode: &str,
    count_mode: &str,
    vbg_mcal_dac: i32,
) -> String {
    format!(
        "DAQ:! SET_SENSOR_OPERATINGS {} {} {} {} {} {readout_mode} {count_mode} {vbg_mcal_dac}",
        threshold_regs[3],
        threshold_regs[2],
        threshold_regs[1],
        threshold_regs[0],
        threshold_regs[4],
    )
}

/// The Pixie-II form of `SET_SENSOR_OPERATINGS` (C `setThresholds`; `auFS` is
/// hard-coded to 7 there, and the count mode is always `NONBI`).
pub fn set_sensor_operatings_pii(
    threshold_regs: &[i32; 5],
    vth_max: i32,
    reference: i32,
    au_fs: i32,
    readout_mode: &str,
) -> String {
    format!(
        "DAQ:! SET_SENSOR_OPERATINGS {} {} {} {} {vth_max} {reference} {au_fs} {readout_mode} NONBI",
        threshold_regs[3], threshold_regs[2], threshold_regs[1], threshold_regs[0],
    )
}

/// `DAQ:! AUTOCAL NOCODES`.
pub fn autocal() -> String {
    "DAQ:! AUTOCAL NOCODES".into()
}

/// `DAQ:! LOOP ...` (C `startAcquire`).
pub fn loop_acquire(
    num_images: i32,
    acquire_time: f64,
    shutter_pause: f64,
    frame_type: FrameType,
    trigger_mode: TriggerMode,
    hv_mode: HVMode,
) -> String {
    format!(
        "DAQ:! LOOP {num_images} {} {} {} {} UNMOD {}",
        (acquire_time * 1000.0) as i32,
        (shutter_pause * 1000.0) as i32,
        frame_type.as_str(),
        trigger_mode.as_str(),
        hv_mode.as_str()
    )
}

/// `DAQ:!!ACQUISITIONBREAK`.
pub fn acquisition_break() -> String {
    "DAQ:!!ACQUISITIONBREAK".into()
}

pub fn get_firmware_version() -> String {
    "SYS:? GET_FIRMWARE_VERSION".into()
}

pub fn get_additional_info() -> String {
    "SYS:? GET_ADDITIONAL_INFO".into()
}

/// The C string in a reply buffer: its bytes up to the first NUL, as
/// `setStringParam`, `strstr` and `sscanf` see `fromServer_`.
pub fn c_string(bytes: &[u8]) -> &[u8] {
    bytes.split(|&b| b == 0).next().unwrap_or_default()
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

/// Did the box accept the command? (C `writeReadServer`: every reply to a
/// non-`SYS:?` command has to contain `GOT:`.)
pub fn reply_is_ok(command: &str, reply: &[u8]) -> bool {
    command.contains("SYS:?") || find(reply, b"GOT:").is_some()
}

/// `DETECTOR <serial> FRMW_VER: <version>` → (serial, version), each present
/// as far as C's `sscanf(fromServer_, "DETECTOR %s FRMW_VER: %s")` got
/// before the reply stopped matching.
pub fn parse_firmware_version(reply: &[u8]) -> (Option<Vec<u8>>, Option<Vec<u8>>) {
    // C `isspace` in the "C" locale.
    let is_space = |c: &u8| matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r');
    let skip_space = |s: &[u8]| -> usize { s.iter().take_while(|c| is_space(c)).count() };
    // `%s ` : skip blanks, take the longest non-blank run (not empty), then the
    // format blank after it.
    let word = |rest: &mut &[u8]| {
        let s = &rest[skip_space(rest)..];
        let end = s.iter().position(is_space).unwrap_or(s.len());
        *rest = &s[end..];
        *rest = &rest[skip_space(rest)..];
        (end > 0).then(|| s[..end].to_vec())
    };
    // A literal matches as written; a blank after it matches none or more.
    let literal = |rest: &mut &[u8], text: &[u8]| match rest.strip_prefix(text) {
        Some(s) => {
            *rest = &s[skip_space(s)..];
            true
        }
        None => false,
    };
    let mut rest = c_string(reply);
    if !literal(&mut rest, b"DETECTOR") {
        return (None, None);
    }
    let Some(serial) = word(&mut rest) else {
        return (None, None);
    };
    let version = if literal(&mut rest, b"FRMW_VER:") {
        word(&mut rest)
    } else {
        None
    };
    (Some(serial), version)
}

/// Everything after `ADDITIONAL INFO:`, byte for byte.
///
/// C bug, pixirad.cpp:569-570: `strstr` returns NULL for a reply without the
/// marker and C publishes the string at address 0x10; here the parameter is
/// left unset.
pub fn parse_additional_info(reply: &[u8]) -> Option<&[u8]> {
    const MARKER: &[u8] = b"ADDITIONAL INFO:";
    let reply = c_string(reply);
    find(reply, MARKER).map(|i| &reply[i + MARKER.len()..])
}

/// The value the status broadcast carries for `key`, e.g. `READ_TCOLD -21.5`
/// (C's `sscanf(pString, "%s %f")`).
pub fn status_value(message: &str, key: &str) -> Option<f64> {
    let start = message.find(key)?;
    let rest = &message[start + key.len()..];
    rest.split_whitespace().next()?.parse().ok()
}

/// The dew point of the box (Magnus formula, C `statusTask`).
///
/// C divided by the literal `0.4343`, which is `log10(e)` to four places; the
/// constant itself is used here.
pub fn dew_point(humidity: f64, box_temp: f64) -> f64 {
    let h = (humidity.log10() - 2.0) / std::f64::consts::LOG10_E
        + (17.62 * box_temp) / (243.12 + box_temp);
    243.12 * h / (17.62 - h)
}

/// How worried to be about the cooling (C `statusTask`'s ladder — later tests
/// override earlier ones, so the most serious condition wins).
pub fn cooling_status(cold_temp: f64, hot_temp: f64, dew_point: f64) -> CoolingStatus {
    let mut status = CoolingStatus::Ok;
    if cold_temp <= dew_point + DEW_POINT_WARNING {
        status = CoolingStatus::DewPointWarning;
    }
    if cold_temp <= dew_point + DEW_POINT_ERROR {
        status = CoolingStatus::DewPointError;
    }
    if hot_temp >= THOT_WARNING {
        status = CoolingStatus::THotWarning;
    }
    if hot_temp >= THOT_ERROR {
        status = CoolingStatus::THotError;
    }
    if cold_temp >= TCOLD_WARNING {
        status = CoolingStatus::TColdWarning;
    }
    if cold_temp >= TCOLD_ERROR {
        status = CoolingStatus::TColdError;
    }
    status
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fw(reply: &str) -> (Option<String>, Option<String>) {
        let text = |v: Vec<u8>| String::from_utf8(v).unwrap();
        let (serial, version) = parse_firmware_version(reply.as_bytes());
        (serial.map(text), version.map(text))
    }

    #[test]
    fn init_command_matches_c() {
        assert_eq!(init(15.0, 1, 350.0, 1), "DAQ:! INIT 15.0 1 350.0 1");
    }

    #[test]
    fn loop_command_matches_c() {
        assert_eq!(
            loop_acquire(
                10,
                0.5,
                0.25,
                FrameType::TwoColors,
                TriggerMode::Internal,
                HVMode::Auto
            ),
            "DAQ:! LOOP 10 500 250 2COL INT UNMOD AUTOHV"
        );
    }

    #[test]
    fn sensor_operatings_sends_the_registers_high_colour_first() {
        let regs = [11, 22, 33, 44, 55];
        assert_eq!(
            set_sensor_operatings_piii(&regs, "DTF", "NPI", 1850),
            "DAQ:! SET_SENSOR_OPERATINGS 44 33 22 11 55 DTF NPI 1850"
        );
        assert_eq!(
            set_sensor_operatings_pii(&regs, 1500, 1, 7, "NODTF"),
            "DAQ:! SET_SENSOR_OPERATINGS 44 33 22 11 1500 1 7 NODTF NONBI"
        );
    }

    #[test]
    fn sync_command_matches_c() {
        assert_eq!(
            set_sync(SyncPolarity::Pos, SyncPolarity::Neg, SyncOutFunction::Read),
            "DAQ:! SET_SYNC POS NEG READ"
        );
    }

    #[test]
    fn reply_is_ok_only_when_the_box_said_got() {
        assert!(reply_is_ok("DAQ:! SYSTEM_RESET", b"GOT: SYSTEM_RESET"));
        assert!(!reply_is_ok("DAQ:! SYSTEM_RESET", b"ERR"));
        // SYS:? queries answer with data, not with an acknowledgement.
        assert!(reply_is_ok(
            "SYS:? GET_FIRMWARE_VERSION",
            b"DETECTOR 007 FRMW_VER: 2.1"
        ));
    }

    #[test]
    fn firmware_version_is_split_into_serial_and_version() {
        assert_eq!(
            fw("DETECTOR PX8-007 FRMW_VER: 3.14\r\n"),
            (Some("PX8-007".to_string()), Some("3.14".to_string()))
        );
        assert_eq!(fw("DETECTOR PX8-007"), (Some("PX8-007".to_string()), None));
        assert_eq!(fw("nonsense"), (None, None));
    }

    /// The edges of C's `sscanf(fromServer_, "DETECTOR %s FRMW_VER: %s")`:
    /// a literal does not skip leading space, a format blank matches none or
    /// more, and only `isspace` separates.
    #[test]
    fn firmware_version_follows_sscanf() {
        let both = |s: &str, v: &str| (Some(s.to_string()), Some(v.to_string()));
        assert_eq!(fw(" DETECTOR PX8 FRMW_VER: 3"), (None, None));
        assert_eq!(fw("DETECTORPX8 FRMW_VER: 3"), both("PX8", "3"));
        assert_eq!(fw("DETECTOR PX8 FRMW_VER:3"), both("PX8", "3"));
        assert_eq!(
            fw("DETECTOR PX8FRMW_VER: 3"),
            (Some("PX8FRMW_VER:".into()), None)
        );
        assert_eq!(fw("DETECTOR\x0bPX8\x0bFRMW_VER:\x0b3"), both("PX8", "3"));
        assert_eq!(
            fw("DETECTOR PX8\u{a0}1 FRMW_VER: 3"),
            both("PX8\u{a0}1", "3")
        );
        assert_eq!(fw("DETECTOR PX8 FRMW_VERS 3"), (Some("PX8".into()), None));
        assert_eq!(fw("DETECTOR "), (None, None));
    }

    #[test]
    fn additional_info_is_the_tail_and_none_when_absent() {
        assert_eq!(
            parse_additional_info(b"HEAD ADDITIONAL INFO: box 3 \r\0junk"),
            Some(&b" box 3 \r"[..])
        );
        assert_eq!(parse_additional_info(b"HEAD"), None);
        assert_eq!(parse_additional_info(b"HEAD\0ADDITIONAL INFO: x"), None);
    }

    #[test]
    fn status_broadcast_values_are_found_by_key() {
        let msg = "READ_TCOLD -21.5 READ_THOT 33.0 READ_BOX_HUM 12.5";
        assert_eq!(status_value(msg, "READ_TCOLD"), Some(-21.5));
        assert_eq!(status_value(msg, "READ_THOT"), Some(33.0));
        assert_eq!(status_value(msg, "READ_HV"), None);
    }

    #[test]
    fn dew_point_matches_the_magnus_formula() {
        // 50 % relative humidity at 20 C is a dew point of 9.3 C.
        let dp = dew_point(50.0, 20.0);
        assert!((dp - 9.26).abs() < 0.05, "dew point {dp}");
    }

    #[test]
    fn cooling_status_reports_the_most_serious_condition() {
        assert_eq!(cooling_status(-20.0, 25.0, -30.0), CoolingStatus::Ok);
        assert_eq!(
            cooling_status(-28.0, 25.0, -30.0),
            CoolingStatus::DewPointWarning
        );
        assert_eq!(
            cooling_status(-31.0, 25.0, -30.0),
            CoolingStatus::DewPointError
        );
        assert_eq!(
            cooling_status(-20.0, 45.0, -30.0),
            CoolingStatus::THotWarning
        );
        assert_eq!(cooling_status(-20.0, 55.0, -30.0), CoolingStatus::THotError);
        // A cold sensor that is not cold at all outranks everything else.
        assert_eq!(
            cooling_status(35.0, 55.0, -30.0),
            CoolingStatus::TColdWarning
        );
        assert_eq!(cooling_status(45.0, 55.0, -30.0), CoolingStatus::TColdError);
        assert!(CoolingStatus::TColdError.is_error());
        assert!(!CoolingStatus::THotWarning.is_error());
    }
}
