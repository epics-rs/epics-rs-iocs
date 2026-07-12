//! The bracketed ASCII BIS talks: the commands the driver sends on the command
//! socket, and the messages BIS broadcasts on the status socket.
//!
//! Both are pure functions of their text, so the whole conversation can be
//! tested without a server.

use std::str::FromStr;

use crate::types::FrameType;

/// `[Scan ...]` or `[Dark ...]` for this frame type (C `BISTask`'s switch).
///
/// `file_name` is ignored for a dark: BIS names the dark frames itself.
pub fn acquire(
    frame_type: FrameType,
    file_name: &str,
    acquire_time: f64,
    num_darks: i32,
) -> String {
    match frame_type {
        FrameType::Normal => {
            format!("[Scan /Filename={file_name} /scantime={acquire_time:.6} /Rescan=0]")
        }
        FrameType::Dark => {
            format!("[Dark /AddTime={acquire_time:.6} /Repetitions={num_darks}]")
        }
        FrameType::Raw => {
            format!(
                "[Scan /Filename={file_name} /scantime={acquire_time:.6} /Rescan=0 /DarkFlood=0]"
            )
        }
        FrameType::DoubleCorrelation => {
            format!("[Scan /Filename={file_name} /scantime={acquire_time:.6} /Rescan=1]")
        }
    }
}

/// `[Shutter /Status=n]` (C `BISDetector::setShutter`).
pub fn shutter(open: bool) -> String {
    format!("[Shutter /Status={}]", if open { 1 } else { 0 })
}

/// `[ChangeFrameSize /FrameSize=n]` (C `writeInt32`, `ADBinX`).
pub fn change_frame_size(frame_size: i32) -> String {
    format!("[ChangeFrameSize /FrameSize={frame_size}]")
}

/// Everything one status message from BIS says.
///
/// C read each field with `sscanf(strstr(response, "KEY="), "KEY=%d", &value)`
/// and never checked `strstr` for null: a message that carried the message name
/// but not the key dereferenced null, and a `sscanf` that matched nothing left
/// the value uninitialised and published it. Here a key that is not there is
/// `None` and nothing is published for it.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct StatusReport {
    /// BIS has finished processing the frame (C's `readoutEventId`).
    pub processing_done: bool,
    pub temperature: Option<f64>,
    /// The detector is square; this is the side, in pixels.
    pub frame_size: Option<i32>,
    pub shutter_open: Option<i32>,
}

/// The number that follows `key` (C's `strstr` + `sscanf` pairs).
fn value_after<T: FromStr>(message: &str, key: &str) -> Option<T> {
    let at = message.find(key)? + key.len();
    let token: String = message[at..]
        .chars()
        .take_while(|c| c.is_ascii_digit() || "+-.eE".contains(*c))
        .collect();
    token.parse().ok()
}

/// Read one status message (C `statusTask`'s `strstr` chain).
pub fn parse_status(message: &str) -> StatusReport {
    let mut report = StatusReport::default();

    if message.contains("[INSTRUMENTQUEUE /PROCESSING=0]") {
        report.processing_done = true;
    } else if message.contains("[CCDTEMPERATURE") {
        report.temperature = value_after(message, "DEGREESC=");
    } else if message.contains("[DETECTORSTATUS") {
        report.frame_size = value_after(message, "FRAMESIZE=");
        report.temperature = value_after(message, "CCDTEMP=");
    } else if message.contains("[SHUTTERSTATUS") {
        report.shutter_open = value_after(message, "STATUS=");
    }

    report
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_normal_scan_names_the_file_and_the_exposure() {
        assert_eq!(
            acquire(FrameType::Normal, "/data/test_001.sfrm", 2.5, 2),
            "[Scan /Filename=/data/test_001.sfrm /scantime=2.500000 /Rescan=0]"
        );
    }

    #[test]
    fn a_raw_scan_turns_the_dark_flood_correction_off() {
        assert_eq!(
            acquire(FrameType::Raw, "/data/a.sfrm", 1.0, 2),
            "[Scan /Filename=/data/a.sfrm /scantime=1.000000 /Rescan=0 /DarkFlood=0]"
        );
    }

    #[test]
    fn a_double_correlation_scan_rescans() {
        assert_eq!(
            acquire(FrameType::DoubleCorrelation, "/data/a.sfrm", 1.0, 2),
            "[Scan /Filename=/data/a.sfrm /scantime=1.000000 /Rescan=1]"
        );
    }

    #[test]
    fn a_dark_takes_the_repetition_count_and_no_file_name() {
        assert_eq!(
            acquire(FrameType::Dark, "/data/a.sfrm", 0.25, 3),
            "[Dark /AddTime=0.250000 /Repetitions=3]"
        );
    }

    #[test]
    fn the_shutter_and_the_frame_size_are_bracketed_too() {
        assert_eq!(shutter(true), "[Shutter /Status=1]");
        assert_eq!(shutter(false), "[Shutter /Status=0]");
        assert_eq!(change_frame_size(2048), "[ChangeFrameSize /FrameSize=2048]");
    }

    #[test]
    fn the_processing_message_ends_the_readout() {
        let report = parse_status("[INSTRUMENTQUEUE /PROCESSING=0]");
        assert!(report.processing_done);
        assert_eq!(report.temperature, None);
    }

    #[test]
    fn a_temperature_message_carries_degrees() {
        let report = parse_status("[CCDTEMPERATURE /DEGREESC=-40.25]");
        assert_eq!(report.temperature, Some(-40.25));
    }

    #[test]
    fn a_detector_status_message_carries_the_frame_size_and_the_temperature() {
        let report = parse_status("[DETECTORSTATUS /FRAMESIZE=1024 /CCDTEMP=-39.5]");
        assert_eq!(report.frame_size, Some(1024));
        assert_eq!(report.temperature, Some(-39.5));
    }

    #[test]
    fn a_shutter_message_carries_its_state() {
        assert_eq!(
            parse_status("[SHUTTERSTATUS /STATUS=1]").shutter_open,
            Some(1)
        );
        assert_eq!(
            parse_status("[SHUTTERSTATUS /STATUS=0]").shutter_open,
            Some(0)
        );
    }

    #[test]
    fn a_message_without_its_key_publishes_nothing() {
        // C ran `sscanf` on a null pointer here.
        let report = parse_status("[DETECTORSTATUS /BUSY=1]");
        assert_eq!(report.frame_size, None);
        assert_eq!(report.temperature, None);
        assert_eq!(parse_status("[CCDTEMPERATURE]").temperature, None);
        assert_eq!(parse_status("[SHUTTERSTATUS]").shutter_open, None);
    }
}
