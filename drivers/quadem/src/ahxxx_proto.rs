//! Wire protocol of the Elettra/CaenEls AHxxx picoammeters (`drvAHxxx.cpp`).
//!
//! Covers both families: the AH401 series (AH401B, AH401D), which reports
//! little-endian unsigned 24-bit counts, and the AH501 series (AH501,
//! AH501BE, AH501C, AH501D), which reports big-endian 16- or 24-bit counts
//! through a sign transform.

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

/// C++ `setNumChannels`: `"CHN %d"` (AH501 series only).
pub fn cmd_num_channels(value: i32) -> String {
    format!("CHN {value}")
}

/// C++ `setResolution`: `"RES %d"` (AH501 series only).
pub fn cmd_resolution(value: i32) -> String {
    format!("RES {value}")
}

/// C++ `setBiasState`: `"HVS %s", value ? "ON" : "OFF"`.
pub fn cmd_bias_state(on: bool) -> &'static str {
    if on { "HVS ON" } else { "HVS OFF" }
}

/// C++ `setBiasVoltage`: `"HVS %f"` — C's `%f` is six decimals.
pub fn cmd_bias_voltage(volts: f64) -> String {
    format!("HVS {volts:.6}")
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
/// constructor was not given one. Upstream has no AH501BE branch here, so a
/// meter of that model must be named in `drvAHxxxConfigure`.
pub fn model_from_firmware(firmware: &str) -> Option<QeModel> {
    if firmware.contains("PicoNew") {
        Some(QeModel::Ah401b)
    } else if firmware.contains("401D") {
        Some(QeModel::Ah401d)
    } else if firmware.contains("501 ") {
        Some(QeModel::Ah501)
    } else if firmware.contains("501C") {
        Some(QeModel::Ah501c)
    } else if firmware.contains("501D") {
        Some(QeModel::Ah501d)
    } else {
        None
    }
}

/// C++ `drvAHxxxConfigure`'s `modelName` argument.
pub fn model_from_name(name: &str) -> QeModel {
    match name {
        "AH401B" => QeModel::Ah401b,
        "AH401D" => QeModel::Ah401d,
        "AH501" => QeModel::Ah501,
        "AH501BE" => QeModel::Ah501be,
        "AH501C" => QeModel::Ah501c,
        "AH501D" => QeModel::Ah501d,
        _ => QeModel::Unknown,
    }
}

/// C++ `AH401Series_`.
pub fn is_ah401_series(model: QeModel) -> bool {
    matches!(model, QeModel::Ah401b | QeModel::Ah401d)
}

/// C++ `AH501Series_`: everything the driver accepts that is not an AH401.
pub fn is_ah501_series(model: QeModel) -> bool {
    matches!(
        model,
        QeModel::Ah501 | QeModel::Ah501be | QeModel::Ah501c | QeModel::Ah501d
    )
}

/// C++ `setBiasState` / `setBiasVoltage` reject the plain AH501, which has no
/// bias supply, and the whole AH401 series.
pub fn has_bias_supply(model: QeModel) -> bool {
    matches!(model, QeModel::Ah501be | QeModel::Ah501c | QeModel::Ah501d)
}

/// C++ `readStatus` reads `HVS ?` back only from these models.
pub fn reads_bias_status(model: QeModel) -> bool {
    matches!(model, QeModel::Ah501be | QeModel::Ah501c | QeModel::Ah501d)
}

/// C++ `sscanf(inString_, "CHN %d", &numChannels)`.
pub fn parse_chn(resp: &str) -> Option<i32> {
    parse_int_after(resp, "CHN")
}

/// C++ `sscanf(inString_, "RES %d", &resolution)`.
pub fn parse_res(resp: &str) -> Option<i32> {
    parse_int_after(resp, "RES")
}

fn parse_int_after(resp: &str, prefix: &str) -> Option<i32> {
    let rest = resp.strip_prefix(prefix)?.trim_start();
    let (value, consumed) = strtol(rest, 10);
    (consumed > 0).then_some(value as i32)
}

/// C++ `readStatus`: `"HVS OFF"` means the bias is off; anything else carries
/// the voltage. `Some(None)` is bias off, `Some(Some(v))` is bias on at `v`.
pub fn parse_hvs(resp: &str) -> Option<Option<f64>> {
    if resp == "HVS OFF" {
        return Some(None);
    }
    let rest = resp.strip_prefix("HVS")?.trim_start();
    let end = rest
        .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+'))
        .unwrap_or(rest.len());
    rest[..end].parse::<f64>().ok().map(Some)
}

/// C++ `readStatus`, AH501 branch: the sample time is a function of the
/// resolution, the channel count and the read format.
///
/// The binary case recomputes the same `38.4 us * numChannels` (doubled at 24
/// bits) that seeds the ASCII case; an ASCII combination outside the table —
/// three channels, or a resolution that is neither 16 nor 24 — falls through
/// C++'s `switch` and keeps that seed.
pub fn sample_time_ah501(read_format: QeReadFormat, resolution: i32, num_channels: i32) -> f64 {
    let mut seed = 38.4e-6 * num_channels as f64;
    if resolution == 24 {
        seed *= 2.0;
    }
    if read_format == QeReadFormat::Binary {
        return seed;
    }
    match (resolution, num_channels) {
        (16, 1) => 384e-6,
        (16, 2) => 806.4e-6,
        (16, 4) => 1.6128e-3,
        (24, 1) => 499.2e-6,
        (24, 2) => 998.4e-6,
        (24, 4) => 1.9968e-3,
        _ => seed,
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

// ===========================================================================
// AH501 data decoding
// ===========================================================================

/// C++ `readThread`: `numBytes = 3; if (resolution_ == 16) numBytes = 2;`
pub fn ah501_bytes_per_value(resolution: i32) -> usize {
    if resolution == 16 { 2 } else { 3 }
}

/// Bytes the AH501 read thread requests per pass.
pub fn ah501_read_len(resolution: i32, num_channels: usize, values_per_read: usize) -> usize {
    ah501_bytes_per_value(resolution) * num_channels * values_per_read
}

/// C++ 16-bit sign transform: `value <= 32767 ? -value : 65536 - value`.
pub fn sign_transform_16(value: i64) -> i64 {
    if value <= 32767 {
        -value
    } else {
        65536 - value
    }
}

/// C++ 24-bit sign transform: `value <= 8388607 ? -value : 16777216 - value`.
pub fn sign_transform_24(value: i64) -> i64 {
    if value <= 8_388_607 {
        -value
    } else {
        16_777_216 - value
    }
}

/// C++ `readThread`, AH501 binary branch: big-endian counts through the
/// resolution's sign transform, accumulated per channel over `valuesPerRead`
/// readings.
///
/// Returns `None` when `input` is shorter than the frame the caller asked for.
pub fn accumulate_binary_ah501(
    input: &[u8],
    resolution: i32,
    num_channels: usize,
    values_per_read: usize,
) -> Option<[f64; QE_MAX_INPUTS]> {
    let num_bytes = ah501_bytes_per_value(resolution);
    if input.len() < ah501_read_len(resolution, num_channels, values_per_read) {
        return None;
    }
    let mut raw = [0.0f64; QE_MAX_INPUTS];
    let mut offset = 0;
    for _ in 0..values_per_read {
        for r in raw.iter_mut().take(num_channels.min(QE_MAX_INPUTS)) {
            let value = if num_bytes == 2 {
                let v = ((input[offset] as i64) << 8) | (input[offset + 1] as i64);
                sign_transform_16(v)
            } else {
                let v = ((input[offset] as i64) << 16)
                    | ((input[offset + 1] as i64) << 8)
                    | (input[offset + 2] as i64);
                sign_transform_24(v)
            };
            *r += value as f64;
            offset += num_bytes;
        }
    }
    Some(raw)
}

/// C++ `readThread`, AH501 ASCII branch: `nExpected = (resolution/4) *
/// numChannels + (numChannels - 1)` bytes per line.
pub fn ah501_ascii_expected_len(resolution: i32, num_channels: usize) -> usize {
    (resolution as usize / 4) * num_channels + (num_channels - 1)
}

/// C++ `readThread`'s AH501BE Ext. Gate literal: `strncmp((const char *)
/// input, "ACK\r\n", 5)`.
pub const ACK_PREAMBLE: &[u8] = b"ACK\r\n";

/// C++ `readThread` (drvAHxxx.cpp:252-259): a binary read that timed out
/// after transferring exactly the 5-byte ACK preamble and nothing else.
pub fn is_ack_preamble_only(partial: &[u8]) -> bool {
    partial == ACK_PREAMBLE
}

/// C++ `readThread` (drvAHxxx.cpp:265-267): a completed read whose first 5
/// bytes are the ACK preamble — the gate fired between the ACK and the rest
/// of the frame arriving.
pub fn starts_with_ack_preamble(data: &[u8]) -> bool {
    data.starts_with(ACK_PREAMBLE)
}

/// C++ `readThread`, AH501 ASCII branch: `strtol(inPtr, &inPtr, 16)` per
/// channel, then the resolution's sign transform.
pub fn parse_ascii_ah501(line: &str, resolution: i32, num_channels: usize) -> [f64; QE_MAX_INPUTS] {
    let mut raw = [0.0f64; QE_MAX_INPUTS];
    let mut rest = line;
    for r in raw.iter_mut().take(num_channels.min(QE_MAX_INPUTS)) {
        let (value, consumed) = strtol(rest, 16);
        rest = &rest[consumed..];
        *r = if resolution == 16 {
            sign_transform_16(value) as f64
        } else {
            sign_transform_24(value) as f64
        };
    }
    raw
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
        assert_eq!(model_from_firmware("PicoNew 1.2"), Some(QeModel::Ah401b));
        assert_eq!(model_from_firmware("AH401D v2"), Some(QeModel::Ah401d));
    }

    #[test]
    fn ah501_command_strings() {
        assert_eq!(cmd_num_channels(2), "CHN 2");
        assert_eq!(cmd_resolution(24), "RES 24");
        assert_eq!(cmd_bias_state(true), "HVS ON");
        assert_eq!(cmd_bias_state(false), "HVS OFF");
        assert_eq!(cmd_bias_voltage(12.5), "HVS 12.500000");
    }

    #[test]
    fn ah501_status_responses_parse() {
        assert_eq!(parse_chn("CHN 4"), Some(4));
        assert_eq!(parse_res("RES 24"), Some(24));
        assert_eq!(parse_chn("RES 24"), None);
        assert_eq!(parse_hvs("HVS OFF"), Some(None));
        assert_eq!(parse_hvs("HVS 12.500000"), Some(Some(12.5)));
        assert_eq!(parse_hvs("RNG 0"), None);
    }

    #[test]
    fn ah501_binary_sample_time_scales_with_channels_and_resolution() {
        assert_eq!(
            sample_time_ah501(QeReadFormat::Binary, 16, 4),
            38.4e-6 * 4.0
        );
        assert_eq!(
            sample_time_ah501(QeReadFormat::Binary, 24, 4),
            38.4e-6 * 4.0 * 2.0
        );
    }

    #[test]
    fn ah501_ascii_sample_time_uses_the_table() {
        assert_eq!(sample_time_ah501(QeReadFormat::Ascii, 16, 1), 384e-6);
        assert_eq!(sample_time_ah501(QeReadFormat::Ascii, 16, 2), 806.4e-6);
        assert_eq!(sample_time_ah501(QeReadFormat::Ascii, 16, 4), 1.6128e-3);
        assert_eq!(sample_time_ah501(QeReadFormat::Ascii, 24, 1), 499.2e-6);
        assert_eq!(sample_time_ah501(QeReadFormat::Ascii, 24, 2), 998.4e-6);
        assert_eq!(sample_time_ah501(QeReadFormat::Ascii, 24, 4), 1.9968e-3);
    }

    #[test]
    fn ah501_ascii_sample_time_falls_through_for_three_channels() {
        // C++'s switch has no case 3, so sampleTime keeps the seed value.
        assert_eq!(
            sample_time_ah501(QeReadFormat::Ascii, 24, 3),
            38.4e-6 * 3.0 * 2.0
        );
    }

    #[test]
    fn ah501_sign_transform_is_upstream_exact() {
        assert_eq!(sign_transform_16(0), 0);
        assert_eq!(sign_transform_16(1), -1);
        assert_eq!(sign_transform_16(32767), -32767);
        assert_eq!(sign_transform_16(32768), 32768);
        assert_eq!(sign_transform_16(65535), 1);
        assert_eq!(sign_transform_24(0), 0);
        assert_eq!(sign_transform_24(8_388_607), -8_388_607);
        assert_eq!(sign_transform_24(8_388_608), 8_388_608);
        assert_eq!(sign_transform_24(16_777_215), 1);
    }

    #[test]
    fn ah501_binary_16_bit_is_big_endian() {
        // 0x0001 -> -1, 0xFFFF -> +1.
        let frame = [0x00, 0x01, 0xff, 0xff];
        let raw = accumulate_binary_ah501(&frame, 16, 2, 1).unwrap();
        assert_eq!(raw[0], -1.0);
        assert_eq!(raw[1], 1.0);
    }

    #[test]
    fn ah501_binary_24_bit_is_big_endian() {
        // 0x000001 -> -1, 0xFFFFFF -> +1.
        let frame = [0x00, 0x00, 0x01, 0xff, 0xff, 0xff];
        let raw = accumulate_binary_ah501(&frame, 24, 2, 1).unwrap();
        assert_eq!(raw[0], -1.0);
        assert_eq!(raw[1], 1.0);
    }

    #[test]
    fn ah501_binary_accumulates_across_values_per_read() {
        let frame = [0x00, 0x00, 0x01, 0x00, 0x00, 0x01];
        let raw = accumulate_binary_ah501(&frame, 24, 1, 2).unwrap();
        assert_eq!(raw[0], -2.0);
    }

    #[test]
    fn ah501_short_binary_frame_is_rejected() {
        assert!(accumulate_binary_ah501(&[0u8; 5], 24, 2, 1).is_none());
        assert!(accumulate_binary_ah501(&[0u8; 3], 16, 2, 1).is_none());
    }

    #[test]
    fn ah501_read_len_depends_on_resolution() {
        assert_eq!(ah501_read_len(16, 4, 1), 8);
        assert_eq!(ah501_read_len(24, 4, 1), 12);
        assert_eq!(ah501_read_len(24, 2, 5), 30);
    }

    #[test]
    fn ah501_ascii_line_is_hex_per_channel() {
        // 24-bit: 000001 -> -1, FFFFFF -> +1.
        let raw = parse_ascii_ah501("000001 FFFFFF", 24, 2);
        assert_eq!(raw[0], -1.0);
        assert_eq!(raw[1], 1.0);
    }

    #[test]
    fn ah501_ascii_line_16_bit() {
        let raw = parse_ascii_ah501("0001 FFFF", 16, 2);
        assert_eq!(raw[0], -1.0);
        assert_eq!(raw[1], 1.0);
    }

    #[test]
    fn ah501_ascii_expected_length_counts_separators() {
        assert_eq!(ah501_ascii_expected_len(24, 4), 6 * 4 + 3);
        assert_eq!(ah501_ascii_expected_len(16, 2), 4 * 2 + 1);
    }

    #[test]
    fn ah501_series_membership() {
        assert!(is_ah401_series(QeModel::Ah401b));
        assert!(!is_ah501_series(QeModel::Ah401d));
        for m in [
            QeModel::Ah501,
            QeModel::Ah501be,
            QeModel::Ah501c,
            QeModel::Ah501d,
        ] {
            assert!(is_ah501_series(m));
            assert!(!is_ah401_series(m));
        }
        assert!(!has_bias_supply(QeModel::Ah501));
        assert!(has_bias_supply(QeModel::Ah501be));
        assert!(reads_bias_status(QeModel::Ah501d));
        assert!(!reads_bias_status(QeModel::Ah501));
    }

    #[test]
    fn ah501_model_names_and_firmware_strings() {
        assert_eq!(model_from_name("AH501"), QeModel::Ah501);
        assert_eq!(model_from_name("AH501BE"), QeModel::Ah501be);
        assert_eq!(model_from_name("AH501C"), QeModel::Ah501c);
        assert_eq!(model_from_name("AH501D"), QeModel::Ah501d);
        assert_eq!(model_from_firmware("AH501 v1"), Some(QeModel::Ah501));
        assert_eq!(model_from_firmware("AH501C v1"), Some(QeModel::Ah501c));
        assert_eq!(model_from_firmware("AH501D v1"), Some(QeModel::Ah501d));
        // Upstream has no AH501BE firmware branch.
        assert_eq!(model_from_firmware("AH501BE v1"), None);
    }

    #[test]
    fn read_len_matches_three_bytes_four_channels() {
        assert_eq!(ah401_read_len(1), 12);
        assert_eq!(ah401_read_len(5), 60);
    }

    #[test]
    fn ack_preamble_only_matches_exactly_five_bytes() {
        assert!(is_ack_preamble_only(b"ACK\r\n"));
        // A timeout that landed mid-preamble, or with trailing frame bytes,
        // is not the ACK case — C++'s `nRead == 5` guard requires the exact
        // count.
        assert!(!is_ack_preamble_only(b"ACK\r"));
        assert!(!is_ack_preamble_only(b"ACK\r\nA"));
        assert!(!is_ack_preamble_only(b""));
    }

    #[test]
    fn starts_with_ack_preamble_ignores_trailing_frame_bytes() {
        let mut frame = b"ACK\r\n".to_vec();
        frame.extend([0u8; 12]);
        assert!(starts_with_ack_preamble(&frame));
        assert!(!starts_with_ack_preamble(b"AK\r\n"));
        assert!(!starts_with_ack_preamble(b""));
    }
}
