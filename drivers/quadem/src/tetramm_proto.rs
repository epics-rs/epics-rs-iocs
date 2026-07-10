//! Wire protocol of the CaenEls TetrAMM 4-channel picoammeter.
//!
//! Ported from `quadEMApp/caenSrc/drvTetrAMM.cpp`. Everything here is pure:
//! command strings in, byte slices out, no I/O — so every frame layout and
//! scaling rule is covered by the tests at the bottom of this file.

use crate::drv_quad_em::{
    QE_MAX_INPUTS, QeAcquireMode, QeReadFormat, QeTriggerMode, QeTriggerPolarity,
};

/// C++ `TetrAMM_TIMEOUT` (seconds).
pub const TETRAMM_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(50);
/// C++ `MIN_VALUES_PER_READ_BINARY`.
pub const MIN_VALUES_PER_READ_BINARY: i32 = 5;
/// C++ `MIN_VALUES_PER_READ_ASCII`.
pub const MIN_VALUES_PER_READ_ASCII: i32 = 500;
/// C++ `MAX_VALUES_PER_READ`.
pub const MAX_VALUES_PER_READ: i32 = 100_000;
/// C++ `BINARY_BUFFER_SIZE`: twice the largest sample (4 doubles + trailer), so
/// a resync read is guaranteed to contain an intact trailer.
pub const BINARY_BUFFER_SIZE: usize = 80;
/// C++ `ASCII_BUFFER_SIZE`.
pub const ASCII_BUFFER_SIZE: usize = 150;
/// Every value on the wire is an IEEE-754 double, big-endian.
pub const BYTES_PER_VALUE: usize = 8;

/// The signalling-NaN trailer that terminates every binary sample.
///
/// C++ reads `i64Data[numChannels_]` *after* the big-endian→host swap, so these
/// constants are the host-order values of the trailing 8 wire bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trailer {
    /// `0xfff40002ffffffff` — end of a normal data sample.
    Data,
    /// `0xfff40000ffffffff` — rising edge of an external trigger.
    TriggerStart,
    /// `0xfff40001ffffffff` — falling edge of an external trigger.
    TriggerEnd,
    /// `0xfff40003ffffffff` — acquisition stopped.
    AcqDone,
    /// Anything else: the stream has lost sync (dropped packet).
    LostSync,
}

impl Trailer {
    pub fn from_u64(v: u64) -> Self {
        match v {
            0xfff4_0002_ffff_ffff => Self::Data,
            0xfff4_0000_ffff_ffff => Self::TriggerStart,
            0xfff4_0001_ffff_ffff => Self::TriggerEnd,
            0xfff4_0003_ffff_ffff => Self::AcqDone,
            _ => Self::LostSync,
        }
    }
}

/// The wire bytes of the `Data` trailer, in transmission order. `readThread`
/// scans for this pattern byte-by-byte when resynchronising.
pub const DATA_TRAILER_BYTES: [u8; 8] = [0xff, 0xf4, 0x00, 0x02, 0xff, 0xff, 0xff, 0xff];

/// Number of bytes in one binary sample: `numChannels` doubles plus the trailer.
pub fn binary_frame_len(num_channels: usize) -> usize {
    (num_channels + 1) * BYTES_PER_VALUE
}

/// A decoded binary sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BinaryFrame {
    /// Channels beyond `numChannels` are zero-filled, as C++ does before
    /// calling `computePositions`.
    pub currents: [f64; QE_MAX_INPUTS],
    pub trailer: Trailer,
}

/// Decode one binary sample of `num_channels` channels.
///
/// Returns `None` when `data` is shorter than the frame. Currents are only
/// meaningful when `trailer == Trailer::Data`; C++ likewise leaves `f64Data`
/// untouched for the trigger/stop trailers.
pub fn parse_binary_frame(data: &[u8], num_channels: usize) -> Option<BinaryFrame> {
    let n = num_channels.min(QE_MAX_INPUTS);
    let need = binary_frame_len(n);
    if data.len() < need {
        return None;
    }

    let mut currents = [0.0f64; QE_MAX_INPUTS];
    for (ch, slot) in currents.iter_mut().enumerate().take(n) {
        let start = ch * BYTES_PER_VALUE;
        let bytes: [u8; 8] = data[start..start + 8].try_into().ok()?;
        *slot = f64::from_be_bytes(bytes);
    }

    let start = n * BYTES_PER_VALUE;
    let bytes: [u8; 8] = data[start..start + 8].try_into().ok()?;
    let trailer = Trailer::from_u64(u64::from_be_bytes(bytes));

    Some(BinaryFrame { currents, trailer })
}

/// C++ resync scan: look for the `Data` trailer anywhere in the first
/// `BINARY_BUFFER_SIZE - BYTES_PER_VALUE` byte positions of the oversized read.
///
/// Returns the byte offset of the trailer's first byte.
pub fn find_resync_offset(buf: &[u8]) -> Option<usize> {
    let limit = BINARY_BUFFER_SIZE - BYTES_PER_VALUE;
    (0..limit).find(|&i| buf.len() >= i + 8 && buf[i..i + 8] == DATA_TRAILER_BYTES)
}

/// Bytes to consume after finding the trailer at `offset`, so the next read
/// starts on a frame boundary. C++: `nRequested - bytesPerValue - i` where
/// `nRequested` is the doubled frame length.
pub fn resync_remainder(num_channels: usize, offset: usize) -> usize {
    binary_frame_len(num_channels.min(QE_MAX_INPUTS)) * 2 - BYTES_PER_VALUE - offset
}

/// One line of the ASCII data stream.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AsciiRecord {
    /// Line contained `SEQNR`: rising edge of an external trigger.
    TriggerStart,
    /// Line contained `EOTRG`: falling edge of an external trigger.
    TriggerEnd,
    /// Whitespace-separated doubles; channels beyond `numChannels` zero-filled.
    Data([f64; QE_MAX_INPUTS]),
}

/// Parse one ASCII line. C++ tests `SEQNR` before `EOTRG` before falling
/// through to `strtod`; a short line yields zeros for the missing channels
/// because `strtod` returns 0.0 when it cannot convert.
pub fn parse_ascii_line(line: &str, num_channels: usize) -> AsciiRecord {
    if line.contains("SEQNR") {
        return AsciiRecord::TriggerStart;
    }
    if line.contains("EOTRG") {
        return AsciiRecord::TriggerEnd;
    }
    let mut currents = [0.0f64; QE_MAX_INPUTS];
    let mut fields = line.split_whitespace();
    for slot in currents.iter_mut().take(num_channels.min(QE_MAX_INPUTS)) {
        *slot = fields
            .next()
            .and_then(|f| f.parse::<f64>().ok())
            .unwrap_or(0.0);
    }
    AsciiRecord::Data(currents)
}

// ===========================================================================
// Commands
// ===========================================================================

/// C++ `sampleTime = 10e-6 * valuesPerRead`.
pub fn sample_time(values_per_read: i32) -> f64 {
    10e-6 * values_per_read as f64
}

/// C++ `setAcquireParams` clamp applied to the `NRSAMP` argument only.
///
/// Note the ordering bug preserved from upstream: `P_SampleTime` is computed
/// from the *unclamped* `valuesPerRead`, while `NRSAMP` carries the clamped
/// value. Do not "fix" this — the readback loop depends on it.
pub fn clamp_values_per_read(values_per_read: i32, format: QeReadFormat) -> i32 {
    let mut v = values_per_read;
    if v > MAX_VALUES_PER_READ {
        v = MAX_VALUES_PER_READ;
    }
    let min = match format {
        QeReadFormat::Binary => MIN_VALUES_PER_READ_BINARY,
        QeReadFormat::Ascii => MIN_VALUES_PER_READ_ASCII,
    };
    if v < min { min } else { v }
}

/// C++ `setAcquireParams`: `naq` is the sample count for one triggered
/// acquisition, or 0 for free-running.
pub fn naq_value(
    trigger_mode: QeTriggerMode,
    acquire_mode: QeAcquireMode,
    num_average: i32,
) -> i32 {
    let triggered = trigger_mode == QeTriggerMode::ExtTrigger;
    let single_shot =
        trigger_mode == QeTriggerMode::FreeRun && acquire_mode == QeAcquireMode::Single;
    if triggered || single_shot {
        num_average
    } else {
        0
    }
}

/// C++ `setAcquireParams`: `NTRG` does not affect continuous mode.
pub fn ntrg_value(acquire_mode: QeAcquireMode, num_acquire: i32) -> i32 {
    match acquire_mode {
        QeAcquireMode::Single => 1,
        QeAcquireMode::Multiple => num_acquire,
        QeAcquireMode::Continuous => 0,
    }
}

/// C++ `numAverage`: external-bulb mode averages over the gate, not a count.
pub fn num_average(trigger_mode: QeTriggerMode, averaging_time: f64, sample_time: f64) -> i32 {
    if trigger_mode == QeTriggerMode::ExtBulb {
        0
    } else {
        crate::drv_quad_em::num_average_from(averaging_time, sample_time)
    }
}

pub fn cmd_range(channel_1_based: i32, range: i32) -> String {
    format!("RNG:CH{channel_1_based}:{range}")
}

pub fn cmd_range_query(channel_1_based: i32) -> String {
    format!("RNG:CH{channel_1_based}:?")
}

pub fn cmd_num_channels(num_channels: i32) -> String {
    format!("CHN:{num_channels}")
}

pub fn cmd_read_format(format: QeReadFormat) -> &'static str {
    match format {
        QeReadFormat::Binary => "ASCII:OFF",
        QeReadFormat::Ascii => "ASCII:ON",
    }
}

pub fn cmd_nrsamp(values_per_read: i32) -> String {
    format!("NRSAMP:{values_per_read}")
}

pub fn cmd_trigger(trigger_mode: QeTriggerMode) -> &'static str {
    if trigger_mode == QeTriggerMode::FreeRun {
        "TRG:OFF"
    } else {
        "TRG:ON"
    }
}

pub fn cmd_trigger_polarity(polarity: QeTriggerPolarity) -> &'static str {
    match polarity {
        QeTriggerPolarity::Positive => "TRGPOL:POS",
        QeTriggerPolarity::Negative => "TRGPOL:NEG",
    }
}

pub fn cmd_naq(naq: i32) -> String {
    format!("NAQ:{naq}")
}

pub fn cmd_ntrg(ntrg: i32) -> String {
    format!("NTRG:{ntrg}")
}

pub fn cmd_bias_state(on: bool) -> &'static str {
    if on { "HVS:ON" } else { "HVS:OFF" }
}

/// C++ `epicsSnprintf(outString_, …, "HVS:%f", value)` — C's `%f` is always six
/// fractional digits.
pub fn cmd_bias_voltage(volts: f64) -> String {
    format!("HVS:{volts:.6}")
}

pub fn cmd_bias_interlock(on: bool) -> &'static str {
    if on { "INTERLOCK:ON" } else { "INTERLOCK:OFF" }
}

// ===========================================================================
// Status responses
// ===========================================================================

/// `sscanf(inString_, "CHN:%d", …)` and friends: match a literal prefix then
/// parse the tail.
fn after_prefix<'a>(resp: &'a str, prefix: &str) -> Option<&'a str> {
    resp.strip_prefix(prefix).map(str::trim)
}

pub fn parse_chn(resp: &str) -> Option<i32> {
    after_prefix(resp, "CHN:")?.parse().ok()
}

/// `sscanf(inString_, "RNG:CH%*1c:%d", …)` — the channel digit is skipped.
pub fn parse_range(resp: &str) -> Option<i32> {
    let rest = after_prefix(resp, "RNG:CH")?;
    let mut chars = rest.chars();
    chars.next()?; // %*1c
    chars.as_str().strip_prefix(':')?.trim().parse().ok()
}

pub fn parse_nrsamp(resp: &str) -> Option<i32> {
    after_prefix(resp, "NRSAMP:")?.parse().ok()
}

/// `HVS:?` answers either `HVS:OFF` or `HVS:<volts>`.
pub fn parse_hvs(resp: &str) -> Option<Option<f64>> {
    if resp == "HVS:OFF" {
        return Some(None);
    }
    Some(Some(after_prefix(resp, "HVS:")?.parse().ok()?))
}

pub fn parse_hvv(resp: &str) -> Option<f64> {
    after_prefix(resp, "HVV:")?.parse().ok()
}

pub fn parse_hvi(resp: &str) -> Option<f64> {
    after_prefix(resp, "HVI:")?.parse().ok()
}

pub fn parse_temp(resp: &str) -> Option<f64> {
    after_prefix(resp, "TEMP:")?.parse().ok()
}

/// `sscanf(inString_, "STATUS:%lx", …)`.
pub fn parse_status(resp: &str) -> Option<u64> {
    u64::from_str_radix(after_prefix(resp, "STATUS:")?, 16).ok()
}

/// C++ `setIntegerParam(P_InterlockStatus, (unitStatus>>8)&0x80)` — the general
/// fault bit, TetrAMM manual p. 40.
pub fn interlock_status(unit_status: u64) -> i32 {
    ((unit_status >> 8) & 0x80) as i32
}

/// `VER:?` answers `VER:<version>`; C++ takes `&inString_[4]`, i.e. everything
/// after the four-character `VER:` prefix — with no bounds check.
pub fn parse_version(resp: &str) -> &str {
    if resp.len() >= 4 { &resp[4..] } else { "" }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a 4-channel binary frame: four big-endian doubles + trailer.
    fn frame(currents: &[f64], trailer: u64) -> Vec<u8> {
        let mut v = Vec::new();
        for c in currents {
            v.extend_from_slice(&c.to_be_bytes());
        }
        v.extend_from_slice(&trailer.to_be_bytes());
        v
    }

    #[test]
    fn binary_frame_len_counts_trailer() {
        assert_eq!(binary_frame_len(1), 16);
        assert_eq!(binary_frame_len(2), 24);
        assert_eq!(binary_frame_len(4), 40);
    }

    #[test]
    fn parse_four_channel_data_frame() {
        let bytes = frame(&[1.5, -2.5, 3.25, -4.75], 0xfff4_0002_ffff_ffff);
        let f = parse_binary_frame(&bytes, 4).expect("full frame");
        assert_eq!(f.trailer, Trailer::Data);
        assert_eq!(f.currents, [1.5, -2.5, 3.25, -4.75]);
    }

    #[test]
    fn parse_one_channel_frame_zero_fills_rest() {
        let bytes = frame(&[7.0], 0xfff4_0002_ffff_ffff);
        assert_eq!(bytes.len(), 16);
        let f = parse_binary_frame(&bytes, 1).expect("full frame");
        assert_eq!(f.trailer, Trailer::Data);
        assert_eq!(f.currents, [7.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn parse_two_channel_frame_zero_fills_rest() {
        let bytes = frame(&[7.0, 8.0], 0xfff4_0002_ffff_ffff);
        let f = parse_binary_frame(&bytes, 2).expect("full frame");
        assert_eq!(f.currents, [7.0, 8.0, 0.0, 0.0]);
    }

    #[test]
    fn all_four_trailers_decode() {
        for (raw, want) in [
            (0xfff4_0002_ffff_ffffu64, Trailer::Data),
            (0xfff4_0000_ffff_ffff, Trailer::TriggerStart),
            (0xfff4_0001_ffff_ffff, Trailer::TriggerEnd),
            (0xfff4_0003_ffff_ffff, Trailer::AcqDone),
            (0x0000_0000_0000_0000, Trailer::LostSync),
            (0xfff4_0004_ffff_ffff, Trailer::LostSync),
        ] {
            let bytes = frame(&[0.0; 4], raw);
            assert_eq!(
                parse_binary_frame(&bytes, 4).unwrap().trailer,
                want,
                "{raw:#x}"
            );
        }
    }

    #[test]
    fn short_frame_is_rejected() {
        let bytes = frame(&[1.0, 2.0, 3.0, 4.0], 0xfff4_0002_ffff_ffff);
        assert!(parse_binary_frame(&bytes[..39], 4).is_none());
        assert!(parse_binary_frame(&bytes, 4).is_some());
    }

    #[test]
    fn data_trailer_wire_bytes_are_big_endian() {
        assert_eq!(0xfff4_0002_ffff_ffffu64.to_be_bytes(), DATA_TRAILER_BYTES);
    }

    #[test]
    fn resync_finds_trailer_at_offset() {
        // 3 junk bytes, then a full data frame.
        let mut buf = vec![0xaa, 0xbb, 0xcc];
        buf.extend_from_slice(&frame(&[1.0, 2.0, 3.0, 4.0], 0xfff4_0002_ffff_ffff));
        buf.resize(BINARY_BUFFER_SIZE, 0);
        // Trailer starts after 3 junk + 32 data bytes.
        assert_eq!(find_resync_offset(&buf), Some(35));
        // C++: nRequested(80) - 8 - 35 = 37
        assert_eq!(resync_remainder(4, 35), 37);
    }

    #[test]
    fn resync_scan_stops_before_last_partial_trailer() {
        // Trailer starting at offset 72 == BINARY_BUFFER_SIZE - 8 is out of the
        // scan range: C++ loops `i < BINARY_BUFFER_SIZE - bytesPerValue`.
        let mut buf = vec![0u8; BINARY_BUFFER_SIZE];
        buf[72..80].copy_from_slice(&DATA_TRAILER_BYTES);
        assert_eq!(find_resync_offset(&buf), None);
        buf[71..79].copy_from_slice(&DATA_TRAILER_BYTES);
        assert_eq!(find_resync_offset(&buf), Some(71));
    }

    #[test]
    fn resync_absent_trailer_returns_none() {
        assert_eq!(find_resync_offset(&[0u8; BINARY_BUFFER_SIZE]), None);
    }

    #[test]
    fn ascii_trigger_markers() {
        assert_eq!(parse_ascii_line("SEQNR 12", 4), AsciiRecord::TriggerStart);
        assert_eq!(parse_ascii_line("EOTRG", 4), AsciiRecord::TriggerEnd);
    }

    #[test]
    fn ascii_data_line_parses_channels() {
        let r = parse_ascii_line(" 1.234e-09 -5.6e-10 7.8e-11 -9.0e-12", 4);
        assert_eq!(
            r,
            AsciiRecord::Data([1.234e-9, -5.6e-10, 7.8e-11, -9.0e-12])
        );
    }

    #[test]
    fn ascii_data_line_zero_fills_unused_channels() {
        let r = parse_ascii_line("1.0 2.0 3.0 4.0", 2);
        assert_eq!(r, AsciiRecord::Data([1.0, 2.0, 0.0, 0.0]));
    }

    #[test]
    fn ascii_short_line_yields_zeros_like_strtod() {
        let r = parse_ascii_line("1.0", 4);
        assert_eq!(r, AsciiRecord::Data([1.0, 0.0, 0.0, 0.0]));
    }

    #[test]
    fn sample_time_is_ten_microseconds_per_value() {
        assert_eq!(sample_time(1), 10e-6);
        assert_eq!(sample_time(1000), 10e-3);
    }

    #[test]
    fn values_per_read_clamped_per_format() {
        assert_eq!(clamp_values_per_read(1, QeReadFormat::Binary), 5);
        assert_eq!(clamp_values_per_read(10, QeReadFormat::Binary), 10);
        assert_eq!(clamp_values_per_read(1, QeReadFormat::Ascii), 500);
        assert_eq!(clamp_values_per_read(600, QeReadFormat::Ascii), 600);
        assert_eq!(
            clamp_values_per_read(200_000, QeReadFormat::Binary),
            100_000
        );
        // Max clamp is applied before the min clamp, exactly as in C++.
        assert_eq!(clamp_values_per_read(200_000, QeReadFormat::Ascii), 100_000);
    }

    #[test]
    fn naq_only_for_ext_trigger_or_freerun_single() {
        use QeAcquireMode::*;
        use QeTriggerMode::*;
        assert_eq!(naq_value(ExtTrigger, Continuous, 77), 77);
        assert_eq!(naq_value(ExtTrigger, Single, 77), 77);
        assert_eq!(naq_value(FreeRun, Single, 77), 77);
        assert_eq!(naq_value(FreeRun, Continuous, 77), 0);
        assert_eq!(naq_value(FreeRun, Multiple, 77), 0);
        assert_eq!(naq_value(ExtBulb, Single, 77), 0);
        assert_eq!(naq_value(ExtGate, Single, 77), 0);
    }

    #[test]
    fn ntrg_per_acquire_mode() {
        assert_eq!(ntrg_value(QeAcquireMode::Single, 9), 1);
        assert_eq!(ntrg_value(QeAcquireMode::Multiple, 9), 9);
        assert_eq!(ntrg_value(QeAcquireMode::Continuous, 9), 0);
    }

    #[test]
    fn num_average_zero_in_ext_bulb() {
        assert_eq!(num_average(QeTriggerMode::ExtBulb, 1.0, 50e-6), 0);
        assert_eq!(num_average(QeTriggerMode::FreeRun, 1.0, 50e-6), 20000);
    }

    #[test]
    fn command_strings_match_cpp() {
        assert_eq!(cmd_range(1, 0), "RNG:CH1:0");
        assert_eq!(cmd_range(4, 2), "RNG:CH4:2");
        assert_eq!(cmd_range_query(3), "RNG:CH3:?");
        assert_eq!(cmd_num_channels(2), "CHN:2");
        assert_eq!(cmd_read_format(QeReadFormat::Binary), "ASCII:OFF");
        assert_eq!(cmd_read_format(QeReadFormat::Ascii), "ASCII:ON");
        assert_eq!(cmd_nrsamp(5), "NRSAMP:5");
        assert_eq!(cmd_trigger(QeTriggerMode::FreeRun), "TRG:OFF");
        assert_eq!(cmd_trigger(QeTriggerMode::ExtGate), "TRG:ON");
        assert_eq!(
            cmd_trigger_polarity(QeTriggerPolarity::Positive),
            "TRGPOL:POS"
        );
        assert_eq!(
            cmd_trigger_polarity(QeTriggerPolarity::Negative),
            "TRGPOL:NEG"
        );
        assert_eq!(cmd_naq(0), "NAQ:0");
        assert_eq!(cmd_ntrg(3), "NTRG:3");
        assert_eq!(cmd_bias_state(true), "HVS:ON");
        assert_eq!(cmd_bias_state(false), "HVS:OFF");
        assert_eq!(cmd_bias_voltage(12.5), "HVS:12.500000");
        assert_eq!(cmd_bias_voltage(-3.0), "HVS:-3.000000");
        assert_eq!(cmd_bias_interlock(true), "INTERLOCK:ON");
        assert_eq!(cmd_bias_interlock(false), "INTERLOCK:OFF");
    }

    #[test]
    fn status_responses_parse() {
        assert_eq!(parse_chn("CHN:4"), Some(4));
        assert_eq!(parse_chn("CHN:junk"), None);
        assert_eq!(parse_range("RNG:CH1:2"), Some(2));
        assert_eq!(parse_range("RNG:CH4:0"), Some(0));
        assert_eq!(parse_nrsamp("NRSAMP:1000"), Some(1000));
        assert_eq!(parse_hvv("HVV:99.5"), Some(99.5));
        assert_eq!(parse_hvi("HVI:-0.002"), Some(-0.002));
        assert_eq!(parse_temp("TEMP:36.75"), Some(36.75));
    }

    #[test]
    fn hvs_query_distinguishes_off_from_voltage() {
        assert_eq!(parse_hvs("HVS:OFF"), Some(None));
        assert_eq!(parse_hvs("HVS:100.0"), Some(Some(100.0)));
    }

    #[test]
    fn status_word_yields_interlock_bit() {
        // (unitStatus >> 8) & 0x80 → bit 15 of the word.
        assert_eq!(parse_status("STATUS:8000"), Some(0x8000));
        assert_eq!(interlock_status(0x8000), 0x80);
        assert_eq!(interlock_status(0x0000), 0);
        assert_eq!(interlock_status(0x7f00), 0);
    }

    #[test]
    fn version_strips_four_char_prefix() {
        assert_eq!(parse_version("VER:2.4.0"), "2.4.0");
        assert_eq!(parse_version("VER"), "");
    }
}
