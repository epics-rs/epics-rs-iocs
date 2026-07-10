//! Wire protocol of the Elettra/CaenEls AHxxx picoammeters (`drvAHxxx.cpp`).
//!
//! Currently covers the AH401 series (AH401B, AH401D). The AH501 series shares
//! the same command grammar but a different data encoding; it is not ported
//! yet.

use std::time::Duration;

use crate::drv_quad_em::{QE_MAX_INPUTS, QeAcquireMode, QeModel, QeReadFormat};

/// C++ `AHxxx_TIMEOUT` (0.05 s).
pub const AHXXX_TIMEOUT: Duration = Duration::from_millis(50);
/// C++ `MAX_COMMAND_LEN`.
pub const MAX_COMMAND_LEN: usize = 256;
/// C++ `ASCIIData[150]`.
pub const ASCII_BUFFER_SIZE: usize = 150;
/// C++ `MIN_INTEGRATION_TIME`.
pub const MIN_INTEGRATION_TIME: f64 = 0.001;
/// C++ `MAX_INTEGRATION_TIME` — declared upstream but never enforced.
pub const MAX_INTEGRATION_TIME: f64 = 1.0;

/// The AH401 series always reads 4 channels of 3 bytes each.
pub const AH401_BYTES_PER_VALUE: usize = 3;
pub const AH401_NUM_CHANNELS: usize = 4;

// ===========================================================================
// C `strtol` emulation
// ===========================================================================

/// `strtol(nptr, &endptr, base)` restricted to bases 10 and 16.
///
/// Returns the converted value and the number of bytes consumed. On "no
/// conversion" C leaves `endptr == nptr` and returns 0; the read thread calls
/// `strtol` in a loop with `inPtr` as both input and output, so a field that
/// fails to convert leaves the pointer parked and every later field also reads
/// 0. That behaviour is reproduced here rather than fixed: it is what the
/// meter's output has to avoid triggering.
pub fn strtol(s: &str, base: u32) -> (i64, usize) {
    debug_assert!(base == 10 || base == 16);
    let bytes = s.as_bytes();
    let mut i = 0;
    while i < bytes.len() && bytes[i].is_ascii_whitespace() {
        i += 1;
    }
    let negative = match bytes.get(i) {
        Some(b'-') => {
            i += 1;
            true
        }
        Some(b'+') => {
            i += 1;
            false
        }
        _ => false,
    };
    if base == 16
        && bytes.get(i) == Some(&b'0')
        && matches!(bytes.get(i + 1), Some(b'x') | Some(b'X'))
        && bytes
            .get(i + 2)
            .is_some_and(|c| (*c as char).is_digit(base))
    {
        i += 2;
    }
    let digits_start = i;
    let mut value: i64 = 0;
    while let Some(d) = bytes.get(i).and_then(|c| (*c as char).to_digit(base)) {
        value = value.saturating_mul(base as i64).saturating_add(d as i64);
        i += 1;
    }
    if i == digits_start {
        // No conversion performed: endptr = nptr.
        return (0, 0);
    }
    (if negative { -value } else { value }, i)
}

// ===========================================================================
// Command formatting
// ===========================================================================

/// C++ `setRange`: `"RNG %d"`.
pub fn cmd_range(value: i32) -> String {
    format!("RNG {value}")
}

/// C++ `setPingPong`: `"HLF %s", value ? "OFF" : "ON"`.
///
/// `PingPong == 1` means "use just ping", which the meter calls `HLF OFF`.
pub fn cmd_ping_pong(value: i32) -> &'static str {
    if value != 0 { "HLF OFF" } else { "HLF ON" }
}

/// C++ `setIntegrationTime`: clamp to `MIN_INTEGRATION_TIME`, then
/// `"ITM %d", (int)(value * 10000)`.
///
/// Returns the clamped seconds (which C++ writes back into the parameter
/// library) and the command.
pub fn cmd_integration_time(value: f64) -> (f64, String) {
    let clamped = if value < MIN_INTEGRATION_TIME {
        MIN_INTEGRATION_TIME
    } else {
        value
    };
    (clamped, format!("ITM {}", (clamped * 10000.0) as i32))
}

/// C++ `setReadFormat` / `setAcquire`: `"BIN ON"` or `"BIN OFF"`.
pub fn cmd_read_format(format: QeReadFormat) -> &'static str {
    match format {
        QeReadFormat::Binary => "BIN ON",
        QeReadFormat::Ascii => "BIN OFF",
    }
}

/// C++ `setAcquire`: `"NAQ %d"`.
pub fn cmd_naq(num_acquire: i32) -> String {
    format!("NAQ {num_acquire}")
}

/// C++ `setAcquire`: in one-shot mode the meter is asked for exactly
/// `numAverage` samples, otherwise for a free-running stream.
pub fn naq_value(acquire_mode: QeAcquireMode, num_average: i32) -> i32 {
    if acquire_mode == QeAcquireMode::Single {
        num_average
    } else {
        0
    }
}

// ===========================================================================
// Response parsing
// ===========================================================================

/// C++ `sscanf(inString_, "RNG %1d", &range)`.
///
/// The AH401D answers with two ranges; only the first digit is parsed.
pub fn parse_range(resp: &str) -> Option<i32> {
    let rest = resp.strip_prefix("RNG")?;
    let rest = rest.trim_start();
    let c = rest.chars().next()?;
    c.to_digit(10).map(|d| d as i32)
}

/// C++ `strcmp("HLF ON", inString_)` / `strcmp("HLF OFF", inString_)`.
///
/// Returns the `PingPong` parameter value: `HLF ON` -> 0, `HLF OFF` -> 1.
pub fn parse_hlf(resp: &str) -> Option<i32> {
    match resp {
        "HLF ON" => Some(0),
        "HLF OFF" => Some(1),
        _ => None,
    }
}

/// C++ `sscanf(inString_, "ITM %lf", &integrationTime)` followed by
/// `integrationTime = integrationTime/10000.`
pub fn parse_itm(resp: &str) -> Option<f64> {
    let rest = resp.strip_prefix("ITM")?;
    let rest = rest.trim_start();
    let end = rest
        .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+'))
        .unwrap_or(rest.len());
    rest[..end].parse::<f64>().ok().map(|v| v / 10000.0)
}

/// C++ `readStatus`, AH401 branch: `sampleTime = pingPong ? integrationTime :
/// integrationTime*2.`
pub fn sample_time_ah401(ping_pong: i32, integration_time: f64) -> f64 {
    if ping_pong != 0 {
        integration_time
    } else {
        integration_time * 2.0
    }
}

/// C++ `reset`: derive the model from the firmware string when the
/// constructor was not given one. Only the AH401 models are recognised here;
/// the AH501 strings are not ported yet.
pub fn model_from_firmware(firmware: &str) -> Option<QeModel> {
    if firmware.contains("PicoNew") {
        Some(QeModel::Ah401b)
    } else if firmware.contains("401D") {
        Some(QeModel::Ah401d)
    } else {
        None
    }
}

/// C++ `drvAHxxxConfigure`'s `modelName` argument.
pub fn model_from_name(name: &str) -> QeModel {
    match name {
        "AH401B" => QeModel::Ah401b,
        "AH401D" => QeModel::Ah401d,
        _ => QeModel::Unknown,
    }
}

// ===========================================================================
// Data decoding
// ===========================================================================

/// Bytes the read thread requests per pass: `numBytes * numChannels *
/// valuesPerRead`.
pub fn ah401_read_len(values_per_read: usize) -> usize {
    AH401_BYTES_PER_VALUE * AH401_NUM_CHANNELS * values_per_read
}

/// C++ `readThread`, AH401 binary branch: little-endian unsigned 24-bit
/// samples, accumulated per channel over `valuesPerRead` readings.
///
/// Returns `None` when `input` is shorter than the frame the caller asked for.
pub fn accumulate_binary_ah401(
    input: &[u8],
    values_per_read: usize,
) -> Option<[f64; QE_MAX_INPUTS]> {
    if input.len() < ah401_read_len(values_per_read) {
        return None;
    }
    let mut raw = [0.0f64; QE_MAX_INPUTS];
    let mut offset = 0;
    for _ in 0..values_per_read {
        for r in raw.iter_mut().take(AH401_NUM_CHANNELS) {
            let v = ((input[offset + 2] as u32) << 16)
                | ((input[offset + 1] as u32) << 8)
                | (input[offset] as u32);
            *r += v as f64;
            offset += AH401_BYTES_PER_VALUE;
        }
    }
    Some(raw)
}

/// C++ `readThread`, AH401 ASCII branch: `strtol(inPtr, &inPtr, 10)` once per
/// channel across one EOS-terminated line.
pub fn parse_ascii_ah401(line: &str, num_channels: usize) -> [f64; QE_MAX_INPUTS] {
    let mut raw = [0.0f64; QE_MAX_INPUTS];
    let mut rest = line;
    for r in raw.iter_mut().take(num_channels.min(QE_MAX_INPUTS)) {
        let (value, consumed) = strtol(rest, 10);
        *r = value as f64;
        rest = &rest[consumed..];
    }
    raw
}

/// C++ `readThread`: `if (valuesPerRead_ > 1) raw[i] = raw[i] / valuesPerRead_`.
pub fn average_over_values_per_read(
    raw: &mut [f64; QE_MAX_INPUTS],
    num_channels: usize,
    values_per_read: i32,
) {
    if values_per_read > 1 {
        for r in raw.iter_mut().take(num_channels.min(QE_MAX_INPUTS)) {
            *r /= values_per_read as f64;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strtol_base10_consumes_digits_and_leading_space() {
        assert_eq!(strtol("  123 456", 10), (123, 5));
    }

    #[test]
    fn strtol_stops_at_non_digit_without_consuming_it() {
        let (v, n) = strtol("42abc", 10);
        assert_eq!((v, n), (42, 2));
        assert_eq!(&"42abc"[n..], "abc");
    }

    #[test]
    fn strtol_no_conversion_returns_zero_and_parks_pointer() {
        assert_eq!(strtol(",123", 10), (0, 0));
        assert_eq!(strtol("", 10), (0, 0));
    }

    #[test]
    fn strtol_handles_sign() {
        assert_eq!(strtol("-17", 10), (-17, 3));
        assert_eq!(strtol("+17", 10), (17, 3));
    }

    #[test]
    fn strtol_base16_parses_hex_with_optional_prefix() {
        assert_eq!(strtol("ffffff", 16), (0xff_ffff, 6));
        assert_eq!(strtol("0x1A", 16), (0x1a, 4));
    }

    #[test]
    fn range_command_and_response() {
        assert_eq!(cmd_range(2), "RNG 2");
        assert_eq!(parse_range("RNG 3"), Some(3));
        // The AH401D answers with two ranges; only the first digit is taken.
        assert_eq!(parse_range("RNG 12"), Some(1));
        assert_eq!(parse_range("ACK"), None);
    }

    #[test]
    fn ping_pong_command_is_inverted() {
        assert_eq!(cmd_ping_pong(1), "HLF OFF");
        assert_eq!(cmd_ping_pong(0), "HLF ON");
        assert_eq!(parse_hlf("HLF ON"), Some(0));
        assert_eq!(parse_hlf("HLF OFF"), Some(1));
        assert_eq!(parse_hlf("HLF"), None);
    }

    #[test]
    fn integration_time_command_truncates_to_hundred_microseconds() {
        let (clamped, cmd) = cmd_integration_time(0.5);
        assert_eq!(clamped, 0.5);
        assert_eq!(cmd, "ITM 5000");
        // Below the minimum the value is clamped, as C++ does before printing.
        let (clamped, cmd) = cmd_integration_time(0.0001);
        assert_eq!(clamped, MIN_INTEGRATION_TIME);
        assert_eq!(cmd, "ITM 10");
    }

    #[test]
    fn integration_time_response_scales_by_ten_thousand() {
        assert_eq!(parse_itm("ITM 5000"), Some(0.5));
        assert_eq!(parse_itm("ITM 10"), Some(0.001));
        assert_eq!(parse_itm("RNG 0"), None);
    }

    #[test]
    fn sample_time_doubles_when_both_halves_are_used() {
        assert_eq!(sample_time_ah401(1, 0.001), 0.001);
        assert_eq!(sample_time_ah401(0, 0.001), 0.002);
    }

    #[test]
    fn read_format_and_naq_commands() {
        assert_eq!(cmd_read_format(QeReadFormat::Binary), "BIN ON");
        assert_eq!(cmd_read_format(QeReadFormat::Ascii), "BIN OFF");
        assert_eq!(cmd_naq(17), "NAQ 17");
        assert_eq!(naq_value(QeAcquireMode::Single, 17), 17);
        assert_eq!(naq_value(QeAcquireMode::Continuous, 17), 0);
        assert_eq!(naq_value(QeAcquireMode::Multiple, 17), 0);
    }

    #[test]
    fn binary_frame_is_little_endian_24_bit() {
        // Channel 1 = 0x030201, 2 = 0x060504, 3 = 0x090807, 4 = 0x0c0b0a.
        let frame: Vec<u8> = (1u8..=12).collect();
        let raw = accumulate_binary_ah401(&frame, 1).unwrap();
        assert_eq!(raw[0], 0x03_02_01 as f64);
        assert_eq!(raw[1], 0x06_05_04 as f64);
        assert_eq!(raw[2], 0x09_08_07 as f64);
        assert_eq!(raw[3], 0x0c_0b_0a as f64);
    }

    #[test]
    fn binary_frame_accumulates_across_values_per_read() {
        let mut frame: Vec<u8> = (1u8..=12).collect();
        frame.extend(1u8..=12);
        let raw = accumulate_binary_ah401(&frame, 2).unwrap();
        assert_eq!(raw[0], 2.0 * 0x03_02_01 as f64);
        assert_eq!(raw[3], 2.0 * 0x0c_0b_0a as f64);
    }

    #[test]
    fn binary_frame_full_scale_is_unsigned() {
        let frame = [0xffu8; 12];
        let raw = accumulate_binary_ah401(&frame, 1).unwrap();
        assert_eq!(raw[0], 16_777_215.0);
    }

    #[test]
    fn short_binary_frame_is_rejected() {
        assert!(accumulate_binary_ah401(&[0u8; 11], 1).is_none());
        assert!(accumulate_binary_ah401(&[0u8; 12], 2).is_none());
    }

    #[test]
    fn ascii_line_is_decimal_per_channel() {
        let raw = parse_ascii_ah401(" 100 200 300 400", 4);
        assert_eq!(raw, [100.0, 200.0, 300.0, 400.0]);
    }

    #[test]
    fn ascii_line_short_of_channels_leaves_zeros() {
        // strtol parks on the first unconvertible field, so every later
        // channel reads 0 — the C behaviour, reproduced.
        let raw = parse_ascii_ah401(" 100 200", 4);
        assert_eq!(raw, [100.0, 200.0, 0.0, 0.0]);
    }

    #[test]
    fn averaging_divides_only_when_values_per_read_exceeds_one() {
        let mut raw = [10.0, 20.0, 30.0, 40.0];
        average_over_values_per_read(&mut raw, 4, 1);
        assert_eq!(raw, [10.0, 20.0, 30.0, 40.0]);
        average_over_values_per_read(&mut raw, 4, 2);
        assert_eq!(raw, [5.0, 10.0, 15.0, 20.0]);
    }

    #[test]
    fn model_names_and_firmware_strings() {
        assert_eq!(model_from_name("AH401B"), QeModel::Ah401b);
        assert_eq!(model_from_name("AH401D"), QeModel::Ah401d);
        assert_eq!(model_from_name("AH501"), QeModel::Unknown);
        assert_eq!(model_from_firmware("PicoNew 1.2"), Some(QeModel::Ah401b));
        assert_eq!(model_from_firmware("AH401D v2"), Some(QeModel::Ah401d));
        assert_eq!(model_from_firmware("AH501D v1"), None);
    }

    #[test]
    fn read_len_matches_three_bytes_four_channels() {
        assert_eq!(ah401_read_len(1), 12);
        assert_eq!(ah401_read_len(5), 60);
    }
}
