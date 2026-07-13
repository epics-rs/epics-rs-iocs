//! Wire protocol of the SenSiC PCR4 4-channel picoammeter (`drvPCR4.cpp`).
//!
//! ASCII line protocol over an asyn octet port: every command is answered
//! either with `ACK` or with an echo of the queried setting, and the data
//! stream is one `\r\n`-terminated line of space-separated floating-point
//! currents per sample.

use std::time::Duration;

use crate::drv_quad_em::{QE_MAX_INPUTS, QeTriggerPolarity};

/// C++ `PCR4_TIMEOUT` (1 s).
pub const PCR4_TIMEOUT: Duration = Duration::from_secs(1);
/// C++ `MAX_COMMAND_LEN`.
pub const MAX_COMMAND_LEN: usize = 256;
/// C++ `ASCII_BUFFER_SIZE`.
pub const ASCII_BUFFER_SIZE: usize = 150;
/// C++ `MIN_VALUES_PER_READ_ASCII`.
pub const MIN_VALUES_PER_READ: i32 = 10;
/// C++ `MAX_VALUES_PER_READ`.
pub const MAX_VALUES_PER_READ: i32 = 52734;
/// C++ `sampleTime = 19e-6 * valuesPerRead` (the comment above it says 10 us,
/// the code says 19 us; the code is what the meter is configured from).
pub const SECONDS_PER_VALUE: f64 = 19e-6;
/// C++ constructor: `setIntegerParam(P_ValuesPerRead, 5)`.
pub const DEFAULT_VALUES_PER_READ: i32 = 5;
/// C++ constructor: `resolution_ = 24`.
pub const RESOLUTION: i32 = 24;
/// C++ `reset`: wait up to 20 s for the meter to come back.
pub const RESET_WAIT_LOOPS: usize = 20;

/// C++ `ranges_v1` — the range enum of a PCR4v1.
pub const RANGES_V1: &[&str] = &["+- 50 uA"];
/// C++ `ranges_v2` — the range enum of a PCR4v2.
pub const RANGES_V2: &[&str] = &["+- 50 mA", "+- 250 uA", "+- 2.5 uA", "+- 25 nA"];

/// C++ `readEnum`: the range choices depend on the firmware revision, and an
/// unrecognised revision is an error rather than a guess.
pub fn ranges_for_version(version: i32) -> Option<&'static [&'static str]> {
    match version {
        1 => Some(RANGES_V1),
        2 => Some(RANGES_V2),
        _ => None,
    }
}

// ===========================================================================
// Commands
// ===========================================================================

pub const CMD_VERSION: &str = "VERSION:?";
pub const CMD_RESET: &str = "RESET";
pub const CMD_ACQUIRE_START: &str = "ACQC:START";
pub const CMD_ACQUIRE_STOP: &str = "ACQC:STOP";
pub const CMD_TRIGGER_STOP: &str = "TRIGGER:STOP";
pub const CMD_TRIGGER_START: &str = "TRIGGER:START";
pub const CMD_RANGE_QUERY: &str = "RANGE:?";
pub const CMD_CHANNELS_QUERY: &str = "CHANNELS:?";
pub const CMD_SPR_QUERY: &str = "SPR:?";
pub const CMD_BIAS_QUERY: &str = "BIASSTATUS:?";
pub const CMD_NETCONFIG: &str = "NETCONFIG";
/// C++ `sendCommand` requires this reply.
pub const ACK: &str = "ACK";

/// C++ `setAcquireParams`: `"SETRANGE:%d"`.
pub fn cmd_range(value: i32) -> String {
    format!("SETRANGE:{value}")
}

/// C++ `setAcquireParams`: `"SETCHANNELS:%d"`.
pub fn cmd_num_channels(value: i32) -> String {
    format!("SETCHANNELS:{value}")
}

/// C++ `setAcquireParams`: clamp to `[MIN_VALUES_PER_READ_ASCII,
/// MAX_VALUES_PER_READ]`, then `"SPR:%d"`.
///
/// The clamp is local to the command; C++ does not write it back to the
/// parameter library, so `QE_VALUES_PER_READ` keeps the requested value while
/// the meter runs at the clamped one.
pub fn clamp_values_per_read(value: i32) -> i32 {
    value.clamp(MIN_VALUES_PER_READ, MAX_VALUES_PER_READ)
}

pub fn cmd_values_per_read(value: i32) -> String {
    format!("SPR:{}", clamp_values_per_read(value))
}

/// C++ `setAcquireParams`: `"SETTRIGGER:%s"`, `RIS` or `FALL`.
pub fn cmd_trigger_polarity(polarity: QeTriggerPolarity) -> &'static str {
    match polarity {
        QeTriggerPolarity::Positive => "SETTRIGGER:RIS",
        QeTriggerPolarity::Negative => "SETTRIGGER:FALL",
    }
}

/// C++ `setAcquireParams`: `"TRIGGER:%s"`, `START` when a trigger mode is
/// selected, `STOP` in free-run.
pub fn cmd_trigger(trigger_mode: i32) -> &'static str {
    if trigger_mode == 0 {
        CMD_TRIGGER_STOP
    } else {
        CMD_TRIGGER_START
    }
}

/// C++ `setBiasState`: `"BIAS:%s"`.
pub fn cmd_bias_state(on: bool) -> &'static str {
    if on { "BIAS:ON" } else { "BIAS:OFF" }
}

/// C++ `setBiasVoltage`: `"SETBIAS:%f"` — C's `%f` is six decimals.
pub fn cmd_bias_voltage(volts: f64) -> String {
    format!("SETBIAS:{volts:.6}")
}

/// C++ `setAcquireParams`: `sampleTime = 19e-6 * valuesPerRead`.
///
/// Computed from the *requested* values-per-read, not the clamped one — that
/// is what C++ does, and the ring-buffer averaging is sized from it.
pub fn sample_time(values_per_read: i32) -> f64 {
    SECONDS_PER_VALUE * values_per_read as f64
}

// ===========================================================================
// Responses
// ===========================================================================

/// C++ `getFirmwareVersion`.
///
/// Upstream does `strcpy(firmwareVersion_, &inString_[8])` — an out-of-bounds
/// read when the reply is shorter than the `"VERSION:"` prefix — and then
/// `atoi(strstr(inString_, "PCR4v") + 5)` with no NULL check, which segfaults
/// on any reply that does not name the model. Here a reply that carries
/// neither is simply not a version reply.
///
/// Returns the firmware text (everything after `VERSION:`) and the revision
/// number parsed out of `PCR4v<n>`.
pub fn parse_version(resp: &str) -> Option<(String, i32)> {
    let firmware = resp.strip_prefix("VERSION:")?;
    let idx = firmware.find("PCR4v")?;
    let digits = &firmware[idx + "PCR4v".len()..];
    let end = digits
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(digits.len());
    let version = digits[..end].parse::<i32>().ok()?;
    Some((firmware.to_string(), version))
}

/// C++ `readStatus`: `sscanf(inString_, "RANGE:%1d", &range)` — the meter can
/// answer with four ranges, only the first digit is taken.
pub fn parse_range(resp: &str) -> Option<i32> {
    let rest = resp.strip_prefix("RANGE:")?;
    rest.chars().next()?.to_digit(10).map(|d| d as i32)
}

/// C++ `readStatus`: `sscanf(inString_, "CHANNELS:%d", &numChannels)`.
pub fn parse_num_channels(resp: &str) -> Option<i32> {
    parse_int_after(resp, "CHANNELS:")
}

/// C++ `readStatus`: `sscanf(inString_, "SPR:%d", &valuesPerRead)`.
pub fn parse_values_per_read(resp: &str) -> Option<i32> {
    parse_int_after(resp, "SPR:")
}

fn parse_int_after(resp: &str, prefix: &str) -> Option<i32> {
    let rest = resp.strip_prefix(prefix)?.trim_start();
    let (value, consumed) = strtod(rest);
    (consumed > 0).then_some(value as i32)
}

/// C++ `readStatus`: `"BIASSTATUS:OFF"` means the bias is off, anything else
/// carries the voltage. `Some(None)` is bias off, `Some(Some(v))` is bias on.
pub fn parse_bias_status(resp: &str) -> Option<Option<f64>> {
    if resp.starts_with("BIASSTATUS:OFF") {
        return Some(None);
    }
    let rest = resp.strip_prefix("BIASSTATUS:")?;
    let (value, consumed) = strtod(rest);
    (consumed > 0).then_some(Some(value))
}

/// C++ `setAcquire(0)`'s resynchronisation loop: it re-reads until a line ends
/// in `ACK`.
pub fn ends_with_ack(line: &[u8]) -> bool {
    line.len() >= 3 && &line[line.len() - 3..] == ACK.as_bytes()
}

// ===========================================================================
// Data stream
// ===========================================================================

/// One line of the PCR4's data stream, as C++ `readThread` classifies it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DataLine {
    /// `strstr(ASCIIData, "TRGEVENTON")`: rising edge of an external trigger.
    TriggerOn,
    /// `strstr(ASCIIData, "TRGEVENTOFF")`: falling edge.
    TriggerOff,
    /// One sample: `numChannels` currents, the remaining channels zeroed.
    Sample([f64; QE_MAX_INPUTS]),
}

/// C++ `readThread`.
///
/// Note the ordering: upstream tests `TRGEVENTON` first, and `TRGEVENTOFF`
/// contains no `TRGEVENTON` substring, so the two tests are unambiguous.
pub fn parse_data_line(line: &str, num_channels: i32) -> DataLine {
    if line.contains("TRGEVENTON") {
        return DataLine::TriggerOn;
    }
    if line.contains("TRGEVENTOFF") {
        return DataLine::TriggerOff;
    }
    DataLine::Sample(parse_sample(line, num_channels))
}

/// C++ `readThread`'s `strtod(inPtr, &inPtr)` loop over `numChannels_`
/// currents, with the unread channels zeroed.
///
/// Upstream parses into `epicsFloat64 *f64Data = (epicsFloat64 *)ASCIIData` —
/// the very buffer it is still parsing from — so each stored double overwrites
/// eight characters of the input line. With the meter's usual field widths the
/// stored bytes land behind the parse pointer and the corruption is invisible;
/// with short fields (`"0 0 0 0"`) the second and later channels are parsed out
/// of the binary image of the first double. This port parses into its own
/// array, so the input is never overwritten.
pub fn parse_sample(line: &str, num_channels: i32) -> [f64; QE_MAX_INPUTS] {
    let mut raw = [0.0f64; QE_MAX_INPUTS];
    let n = (num_channels.max(0) as usize).min(QE_MAX_INPUTS);
    let mut rest = line;
    for r in raw.iter_mut().take(n) {
        let (value, consumed) = strtod(rest);
        *r = value;
        rest = &rest[consumed..];
    }
    raw
}

/// `strtod(nptr, &endptr)` restricted to what the meter emits: optional
/// whitespace, optional sign, digits with an optional fraction and an optional
/// decimal exponent.
///
/// Returns the value and the bytes consumed. As in C, "no conversion" leaves
/// the pointer parked (`0` consumed) and yields `0.0`, which makes every later
/// field of the same line read `0.0` as well.
pub fn strtod(s: &str) -> (f64, usize) {
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() && b[i].is_ascii_whitespace() {
        i += 1;
    }
    let start = i;
    if matches!(b.get(i), Some(b'+') | Some(b'-')) {
        i += 1;
    }
    let mut digits = 0;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
        digits += 1;
    }
    if b.get(i) == Some(&b'.') {
        i += 1;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
            digits += 1;
        }
    }
    if digits == 0 {
        return (0.0, 0);
    }
    // The exponent only counts if it is complete; otherwise strtod backs up to
    // the end of the mantissa.
    if matches!(b.get(i), Some(b'e') | Some(b'E')) {
        let mut j = i + 1;
        if matches!(b.get(j), Some(b'+') | Some(b'-')) {
            j += 1;
        }
        let exp_start = j;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_start {
            i = j;
        }
    }
    match s[start..i].parse::<f64>() {
        Ok(v) => (v, i),
        Err(_) => (0.0, 0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_strings() {
        assert_eq!(cmd_range(2), "SETRANGE:2");
        assert_eq!(cmd_num_channels(4), "SETCHANNELS:4");
        assert_eq!(
            cmd_trigger_polarity(QeTriggerPolarity::Positive),
            "SETTRIGGER:RIS"
        );
        assert_eq!(
            cmd_trigger_polarity(QeTriggerPolarity::Negative),
            "SETTRIGGER:FALL"
        );
        assert_eq!(cmd_trigger(0), "TRIGGER:STOP");
        assert_eq!(cmd_trigger(1), "TRIGGER:START");
        assert_eq!(cmd_bias_state(true), "BIAS:ON");
        assert_eq!(cmd_bias_state(false), "BIAS:OFF");
        assert_eq!(cmd_bias_voltage(12.5), "SETBIAS:12.500000");
    }

    #[test]
    fn values_per_read_is_clamped_in_the_command_only() {
        assert_eq!(cmd_values_per_read(5), "SPR:10");
        assert_eq!(cmd_values_per_read(100), "SPR:100");
        assert_eq!(cmd_values_per_read(1_000_000), "SPR:52734");
        // The sample time uses the requested value, not the clamp — C++
        // computes it before clamping.
        assert_eq!(sample_time(5), 19e-6 * 5.0);
    }

    #[test]
    fn version_reply_yields_firmware_and_revision() {
        let (fw, v) = parse_version("VERSION:PCR4v2 1.3.4").unwrap();
        assert_eq!(fw, "PCR4v2 1.3.4");
        assert_eq!(v, 2);
        let (fw, v) = parse_version("VERSION:SenSiC PCR4v1").unwrap();
        assert_eq!(fw, "SenSiC PCR4v1");
        assert_eq!(v, 1);
    }

    #[test]
    fn version_reply_without_model_is_rejected_not_dereferenced() {
        // Upstream calls atoi(strstr(...) + 5) with no NULL check here.
        assert_eq!(parse_version("VERSION:garbage"), None);
        assert_eq!(parse_version("ACK"), None);
        assert_eq!(parse_version("VER"), None);
    }

    #[test]
    fn status_replies_parse() {
        assert_eq!(parse_range("RANGE:3"), Some(3));
        // The meter can answer with four ranges; only the first digit is taken.
        assert_eq!(parse_range("RANGE:1234"), Some(1));
        assert_eq!(parse_range("ACK"), None);
        assert_eq!(parse_num_channels("CHANNELS:4"), Some(4));
        assert_eq!(parse_num_channels("RANGE:0"), None);
        assert_eq!(parse_values_per_read("SPR:100"), Some(100));
        assert_eq!(parse_values_per_read("SPR:x"), None);
    }

    #[test]
    fn bias_status_off_and_on() {
        assert_eq!(parse_bias_status("BIASSTATUS:OFF"), Some(None));
        assert_eq!(parse_bias_status("BIASSTATUS:24.500000"), Some(Some(24.5)));
        assert_eq!(parse_bias_status("ACK"), None);
    }

    #[test]
    fn ack_is_matched_at_the_end_of_a_line() {
        assert!(ends_with_ack(b"ACK"));
        assert!(ends_with_ack(b"1.0 2.0ACK"));
        assert!(!ends_with_ack(b"AC"));
        assert!(!ends_with_ack(b"ACK "));
    }

    #[test]
    fn trigger_event_lines_are_recognised() {
        assert_eq!(parse_data_line("TRGEVENTON", 4), DataLine::TriggerOn);
        assert_eq!(parse_data_line("TRGEVENTOFF", 4), DataLine::TriggerOff);
    }

    #[test]
    fn sample_line_parses_every_channel() {
        let line = "-1.234567e-09 2.000000e-09 3.000000e-09 4.000000e-09";
        assert_eq!(
            parse_data_line(line, 4),
            DataLine::Sample([-1.234567e-9, 2e-9, 3e-9, 4e-9])
        );
    }

    #[test]
    fn sample_line_zeroes_the_channels_the_meter_is_not_sending() {
        assert_eq!(parse_sample("1.0 2.0", 2), [1.0, 2.0, 0.0, 0.0]);
    }

    #[test]
    fn short_fields_survive_the_parse() {
        // The C parses these out of the binary image of the first double,
        // because f64Data aliases the input buffer. Here they are exact.
        assert_eq!(parse_sample("1 2 3 4", 4), [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(parse_sample("0 0 0 0", 4), [0.0; 4]);
    }

    #[test]
    fn strtod_matches_c_on_the_boundaries() {
        // The consumed count includes the skipped leading whitespace: it is
        // what the caller slices the rest of the line by.
        assert_eq!(strtod(" 1.5e-3rest"), (1.5e-3, 7));
        assert_eq!(&" 1.5e-3rest"[7..], "rest");
        assert_eq!(strtod("-2"), (-2.0, 2));
        // A truncated exponent backs up to the end of the mantissa.
        assert_eq!(strtod("2e"), (2.0, 1));
        assert_eq!(strtod("2e+"), (2.0, 1));
        // No conversion: value 0, pointer parked.
        assert_eq!(strtod("abc"), (0.0, 0));
        assert_eq!(strtod(""), (0.0, 0));
    }

    #[test]
    fn range_choices_depend_on_the_firmware_revision() {
        assert_eq!(ranges_for_version(1), Some(RANGES_V1));
        assert_eq!(ranges_for_version(2), Some(RANGES_V2));
        assert_eq!(ranges_for_version(0), None);
        assert_eq!(ranges_for_version(3), None);
    }
}
