//! L1 binary data-packet framing/parsing and the duplicate/missed-packet +
//! averaging/throttle state machine, ported from `capaNCDT6200Sup.c`'s
//! `readerThread`/`processDataPacket`. Pure, unit-tested functions -- no I/O
//! (the actual socket read loop lives in [`crate::data_driver`]).
//!
//! ## Preserved upstream quirks
//!
//! 1. **Non-power-of-2 channel-availability masks**
//!    (`capaNCDT6200Sup.c:358,368,378,388`): channel 1 tests
//!    `channelBitField & 1`, channel 2 tests `& 5` (0b101), channel 3 tests
//!    `& 21` (0b10101), channel 4 tests `& 85` (0b1010101) -- each test also
//!    matches some of its neighbors' bits. Reproduced verbatim in
//!    [`CHANNEL_MASKS`] / [`channel_available`].
//! 2. **Asymmetric raw-value truncation**
//!    (`capaNCDT6200Sup.c:361,371,381,391`): channel 1's raw dword is masked
//!    `& 0xFFFFFFFF` (32-bit, a no-op); channels 2-4 are masked `& 0xFFFFFF`
//!    (24-bit, discarding the top byte of whatever was read). Reproduced in
//!    [`raw_channel_mask`] / [`parse_channel_raw`].
//! 3. **Stale bytes beyond the confirmed packet length** (self-derived this
//!    port, not a citation from an upstream bug tracker): the reader's byte
//!    buffer (`pdpvt->cbuf`) is never zeroed between packets, and a read
//!    cycle only guarantees `32 + numMeasChansAvail*4` FRESH bytes were
//!    written (`capaNCDT6200Sup.c:324`). The per-channel raw-dword reads at
//!    fixed offsets 32/36/40/44 (`capaNCDT6200Sup.c:358-395`) are gated only
//!    by `channelBitField`, NOT by `numMeasChansAvail` -- so if
//!    `numMeasChansAvail < 4` while `channelBitField` still claims a higher
//!    channel present, that channel's raw dword is read from bytes left
//!    over from a PREVIOUS packet. This module always parses whatever
//!    48-byte window it is handed (matching the C code's unconditional
//!    fixed-offset reads); [`crate::data_driver`]'s reader loop is
//!    responsible for handing it a persistent, only-partially-overwritten
//!    buffer rather than a freshly-zeroed one.
//! 4. **Float32-precision intermediate in the channel scaling formula**
//!    (`capaNCDT6200Sup.c:362-364` et al.): `chanNMeasValue = (dispN *
//!    (float) chanNMeasRange) / capaNCDT6200_MEASURING_VALUE_RANGE` -- the
//!    `(float)` cast on the range forces the *multiplication* itself into
//!    32-bit float precision; only the subsequent division by the `double`
//!    constant `16777215.0` happens at double precision. For large `dispN`
//!    (up to `0xFFFFFF` = 16777215, at the edge of `f32`'s 24-bit
//!    exact-integer mantissa) this rounds differently than a straight `f64`
//!    multiplication would. Reproduced in [`scale_channel`].

pub const HEADER_LEN: usize = 32;
pub const MAX_CHANNELS: usize = 4;
pub const PACKET_BUF_LEN: usize = HEADER_LEN + MAX_CHANNELS * 4;

/// `capaNCDT6200Protocol.h`: `capaNCDT6200_MEASURING_VALUE_RANGE (16777215.0)`, i.e. `0xFFFFFF`.
pub const MEASURING_VALUE_RANGE: f64 = 16_777_215.0;

/// Quirk 1 (see module doc): non-power-of-2 per-channel availability masks.
pub const CHANNEL_MASKS: [u64; MAX_CHANNELS] = [1, 5, 21, 85];

/// `capaNCDT6200Sup.c:324`: `tread >= 32 + numMeasChansAvail*4`.
pub fn packet_len(num_meas_chans_avail: u32) -> usize {
    HEADER_LEN + num_meas_chans_avail as usize * 4
}

fn u16_le(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

fn u32_le(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn u64_le(b: &[u8]) -> u64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[..8]);
    u64::from_le_bytes(a)
}

/// `capaNCDT6200Sup.c:397-399`: the preamble check builds a 4-char string
/// from `cbuf[3], cbuf[2], cbuf[1], cbuf[0]` (in that order) and compares
/// it to the literal `"MEAS"` -- equivalent to checking that the raw wire
/// bytes at offsets 0..4 spell `"SAEM"`.
pub fn preamble_ok(buf: &[u8]) -> bool {
    buf.len() >= 4 && &buf[0..4] == b"SAEM"
}

/// Fixed 32-byte packet header (`capaNCDT6200Sup.c:332-355`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PacketHeader {
    pub preamble: u32,
    pub order_number: u32,
    pub serial_number: u32,
    pub channel_bit_field: u64,
    pub status: u32,
    pub frame_number_m: u16,
    pub bytes_per_frame: u16,
    pub meas_value_counter: u32,
}

/// `capaNCDT6200Sup.c:332-355`. `buf` must be at least [`HEADER_LEN`] bytes.
pub fn parse_header(buf: &[u8]) -> PacketHeader {
    PacketHeader {
        preamble: u32_le(&buf[0..4]),
        order_number: u32_le(&buf[4..8]),
        serial_number: u32_le(&buf[8..12]),
        channel_bit_field: u64_le(&buf[12..20]),
        status: u32_le(&buf[20..24]),
        frame_number_m: u16_le(&buf[24..26]),
        bytes_per_frame: u16_le(&buf[26..28]),
        meas_value_counter: u32_le(&buf[28..32]),
    }
}

/// `channel` is 0-based (0 = channel 1). Quirk 1 (see module doc).
pub fn channel_available(channel_bit_field: u64, channel: usize) -> bool {
    channel_bit_field & CHANNEL_MASKS[channel] != 0
}

/// Quirk 2 (see module doc): channel 1 (`channel == 0`) keeps all 32 raw
/// bits; channels 2-4 are truncated to 24 bits.
pub fn raw_channel_mask(channel: usize) -> u32 {
    if channel == 0 {
        0xFFFF_FFFF
    } else {
        0x00FF_FFFF
    }
}

/// `capaNCDT6200Sup.c:358-395`, the raw-dword half only (masking applied,
/// scaling not yet). `buf` must be at least [`PACKET_BUF_LEN`] bytes.
/// `channel` is 0-based.
pub fn parse_channel_raw(buf: &[u8], channel: usize) -> u32 {
    let offset = HEADER_LEN + channel * 4;
    u32_le(&buf[offset..offset + 4]) & raw_channel_mask(channel)
}

/// Quirk 4 (see module doc): `(raw * (float) meas_range) / MEASURING_VALUE_RANGE`,
/// with the multiplication forced into `f32` precision before the `f64` divide.
pub fn scale_channel(raw: u32, meas_range: i32) -> f64 {
    let product_f32 = raw as f32 * meas_range as f32;
    product_f32 as f64 / MEASURING_VALUE_RANGE
}

/// A fully-decoded packet: header plus each channel's scaled measurement
/// value (`0.0` -- matching `capaNCDT6200Sup.c:327-330` -- for any channel
/// `channel_bit_field` marks unavailable).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodedPacket {
    pub header: PacketHeader,
    pub chan_values: [f64; MAX_CHANNELS],
}

/// `buf` must be at least [`PACKET_BUF_LEN`] bytes; bytes beyond
/// `packet_len(numMeasChansAvail)` may be stale leftovers from a previous
/// packet (quirk 3, see module doc) -- this function does not know or care,
/// exactly like the C code.
pub fn decode_packet(buf: &[u8], meas_ranges: [i32; MAX_CHANNELS]) -> DecodedPacket {
    let header = parse_header(buf);
    let mut chan_values = [0.0; MAX_CHANNELS];
    for (ch, value) in chan_values.iter_mut().enumerate() {
        if channel_available(header.channel_bit_field, ch) {
            let raw = parse_channel_raw(buf, ch);
            *value = scale_channel(raw, meas_ranges[ch]);
        }
    }
    DecodedPacket {
        header,
        chan_values,
    }
}

/// Running connection-health + averaging/throttle state, mirroring the
/// scattered fields on `drvPvt`/`portLink` that `readerThread`/
/// `processDataPacket` mutate every cycle.
#[derive(Debug, Clone, PartialEq)]
pub struct ReaderState {
    pub bad_packet_count: u32,
    pub is_communicating: bool,
    pub read_timeout_secs: f64,
    meas_count_initialized: bool,
    expected_meas_value_counter: u32,
    disp_sum: [f64; MAX_CHANNELS],
    disp_sum_count: u32,
    pub pv_throttle: i32,
    pv_throttle_counter: i32,
    pub stats: LinkStats,
}

/// `capaNCDT6200Sup.c`'s `dataPacket{Good,Bad,Timeout,BadRead}Count`,
/// `{duplicate,missed,dataPacketOutOfSequence}...Count` fields.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LinkStats {
    pub good_count: u64,
    pub bad_preamble_count: u64,
    pub timeout_count: u64,
    pub bad_read_count: u64,
    pub out_of_sequence_count: u64,
    pub duplicate_count: u64,
    pub missed_count: u64,
}

impl ReaderState {
    /// `pdpvt->pvThrottle = 5;` (`capaNCDT6200Sup.c:714`, the constructor's default).
    pub fn new(pv_throttle: i32) -> Self {
        ReaderState {
            bad_packet_count: 0,
            is_communicating: false,
            read_timeout_secs: 5.0,
            meas_count_initialized: false,
            expected_meas_value_counter: 0,
            disp_sum: [0.0; MAX_CHANNELS],
            disp_sum_count: 0,
            pv_throttle,
            pv_throttle_counter: pv_throttle,
            stats: LinkStats::default(),
        }
    }
}

/// One read attempt's classification, feeding [`apply_read_outcome`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadOutcome {
    /// `eopReached && preambleOK == 0` (`capaNCDT6200Sup.c:403-408`).
    GoodPacket,
    /// `eopReached && preambleOK != 0` (`capaNCDT6200Sup.c:409-416`).
    BadPreamble,
    /// `status == asynTimeout` (`capaNCDT6200Sup.c:423-426`).
    Timeout,
    /// A non-timeout read failure (`capaNCDT6200Sup.c:427-432`).
    ReadError,
}

/// `capaNCDT6200Sup.c:402-443`: update the running health/stat counters for
/// one read attempt and report the resulting `isCommunicating` state plus
/// whether `processDataPacket` should run this cycle (`Some(isValid)`), or
/// not at all (`None`).
pub fn apply_read_outcome(state: &mut ReaderState, outcome: ReadOutcome) -> Option<bool> {
    let mut should_process = None;
    match outcome {
        ReadOutcome::GoodPacket => {
            state.stats.good_count += 1;
            should_process = Some(true);
            state.bad_packet_count = 0;
        }
        ReadOutcome::BadPreamble => {
            state.stats.bad_preamble_count += 1;
            state.bad_packet_count += 1;
        }
        ReadOutcome::Timeout => {
            state.stats.timeout_count += 1;
            state.bad_packet_count += 1;
            state.read_timeout_secs = 30.0;
        }
        ReadOutcome::ReadError => {
            state.stats.bad_read_count += 1;
            state.bad_packet_count += 1;
        }
    }
    if state.bad_packet_count == 0 {
        state.is_communicating = true;
        state.read_timeout_secs = 5.0;
    } else if state.bad_packet_count >= 2 {
        state.is_communicating = false;
        should_process = Some(false);
    }
    should_process
}

/// Result of one [`process_data_packet`] call that reached the push stage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PushUpdate {
    /// Averaged-since-last-push value per channel.
    pub values: [f64; MAX_CHANNELS],
    /// `isValid` -- `false` means this push came from the
    /// `badPacketCount >= 2` invalid/alarm path (`capaNCDT6200Sup.c:442`),
    /// and the caller should mark the record COMM_ALARM/INVALID rather
    /// than pushing `values` as good data.
    pub valid: bool,
}

/// `capaNCDT6200Sup.c:145-233` (`processDataPacket`). `meas_value_counter`/
/// `chan_values` are only meaningful when `is_valid`; pass the last-decoded
/// packet's values regardless (mirroring the C code, which reads
/// `dp->measValueCounter` and `capaNCDT6200Data.chanNMeasValue`
/// unconditionally -- on the `isValid == 0` invalid-packet path those
/// fields are simply stale, matching quirk 3's staleness one level up).
/// `interrupt_accept` mirrors EPICS's global `interruptAccept` (records not
/// yet ready to receive I/O Intr callbacks during iocInit).
pub fn process_data_packet(
    state: &mut ReaderState,
    meas_value_counter: u32,
    chan_values: [f64; MAX_CHANNELS],
    is_valid: bool,
    interrupt_accept: bool,
) -> Option<PushUpdate> {
    if !state.meas_count_initialized {
        state.expected_meas_value_counter = meas_value_counter;
        state.meas_count_initialized = true;
    }
    let diff = meas_value_counter.wrapping_sub(state.expected_meas_value_counter) as i32;
    if diff != 0 && state.is_communicating {
        if diff == -1 {
            state.stats.duplicate_count += 1;
            return None;
        }
        if diff > 0 {
            state.stats.missed_count += diff as u64;
        }
        state.stats.out_of_sequence_count += 1;
    }
    state.expected_meas_value_counter = meas_value_counter.wrapping_add(1);

    if !interrupt_accept {
        return None;
    }

    if is_valid {
        for (sum, value) in state.disp_sum.iter_mut().zip(chan_values.iter()) {
            *sum += value;
        }
        state.disp_sum_count += 1;
        state.pv_throttle_counter -= 1;
        if state.pv_throttle_counter > 0 {
            return None;
        }
    }
    state.pv_throttle_counter = state.pv_throttle;

    let count = state.disp_sum_count as f64;
    let mut values = [0.0; MAX_CHANNELS];
    for (value, sum) in values.iter_mut().zip(state.disp_sum.iter()) {
        *value = sum / count;
    }
    state.disp_sum = [0.0; MAX_CHANNELS];
    state.disp_sum_count = 0;
    Some(PushUpdate {
        values,
        valid: is_valid,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header_bytes(preamble_ok: bool, channel_bit_field: u64, meas_value_counter: u32) -> Vec<u8> {
        let mut b = vec![0u8; PACKET_BUF_LEN];
        if preamble_ok {
            b[0..4].copy_from_slice(b"SAEM");
        } else {
            b[0..4].copy_from_slice(b"XXXX");
        }
        b[12..20].copy_from_slice(&channel_bit_field.to_le_bytes());
        b[28..32].copy_from_slice(&meas_value_counter.to_le_bytes());
        b
    }

    #[test]
    fn preamble_ok_matches_saem_bytes() {
        assert!(preamble_ok(b"SAEM"));
        assert!(!preamble_ok(b"MEAS"));
        assert!(!preamble_ok(b"xxxx"));
    }

    #[test]
    fn packet_len_matches_c_formula() {
        assert_eq!(packet_len(0), 32);
        assert_eq!(packet_len(4), 48);
        assert_eq!(packet_len(2), 40);
    }

    #[test]
    fn channel_availability_masks_are_non_power_of_two() {
        // channelBitField == 1 (bit 0 only) lights up ALL FOUR channels,
        // because every mask (1, 5, 21, 85) has bit 0 set -- the most
        // visible consequence of the non-power-of-2 masks: a device
        // reporting only channel 1 present makes every channel's
        // availability test pass.
        assert!(channel_available(1, 0));
        assert!(channel_available(1, 1));
        assert!(channel_available(1, 2));
        assert!(channel_available(1, 3));
        // bit 2 (value 4) alone: chan1 mask (1) misses it; chan2 (5 =
        // 0b101), chan3 (21 = 0b10101) and chan4 (85 = 0b1010101) all have
        // a bit-2 component and catch it too.
        assert!(!channel_available(4, 0));
        assert!(channel_available(4, 1));
        assert!(channel_available(4, 2));
        assert!(channel_available(4, 3));
        // bit 6 (value 64) is only in channel 4's mask (85 = 0b1010101).
        assert!(!channel_available(64, 0));
        assert!(!channel_available(64, 1));
        assert!(!channel_available(64, 2));
        assert!(channel_available(64, 3));
        // bit field 0 (nothing reported present): no channel is available.
        assert!(!channel_available(0, 0));
        assert!(!channel_available(0, 1));
        assert!(!channel_available(0, 2));
        assert!(!channel_available(0, 3));
    }

    #[test]
    fn raw_channel_mask_is_asymmetric() {
        let buf = vec![0xFFu8; PACKET_BUF_LEN];
        assert_eq!(parse_channel_raw(&buf, 0), 0xFFFF_FFFF);
        assert_eq!(parse_channel_raw(&buf, 1), 0x00FF_FFFF);
        assert_eq!(parse_channel_raw(&buf, 2), 0x00FF_FFFF);
        assert_eq!(parse_channel_raw(&buf, 3), 0x00FF_FFFF);
    }

    #[test]
    fn scale_channel_matches_f64_for_small_products() {
        // raw * range well within f32's exact-integer range: f32 and f64
        // multiplication agree exactly.
        let raw = 1000u32;
        let range = 10i32;
        let expected = (raw as f64 * range as f64) / MEASURING_VALUE_RANGE;
        assert_eq!(scale_channel(raw, range), expected);
    }

    #[test]
    fn scale_channel_diverges_from_f64_for_large_products_quirk4() {
        // raw at the top of its 24-bit range times a plausible measuring
        // range: the product overflows f32's 24-bit exact-integer mantissa,
        // so the f32-intermediate result provably differs from a pure f64
        // computation -- proving this reproduces the C float32 cast rather
        // than silently "improving" precision.
        let raw = 0x00FF_FFFFu32; // 16_777_215, the 24-bit truncation quirk's own boundary
        let range = 137i32;
        let f32_result = scale_channel(raw, range);
        let f64_result = (raw as f64 * range as f64) / MEASURING_VALUE_RANGE;
        assert_ne!(f32_result, f64_result);
    }

    #[test]
    fn decode_packet_zeroes_unavailable_channels() {
        // channel_bit_field == 0: no mask test can pass, so every channel
        // stays at its `capaNCDT6200Sup.c:327-330` zero default regardless
        // of whatever raw bytes sit in the channel region of the buffer.
        let mut buf = header_bytes(true, 0, 7);
        buf[32..36].copy_from_slice(&123u32.to_le_bytes());
        buf[36..40].copy_from_slice(&456u32.to_le_bytes());
        let decoded = decode_packet(&buf, [1, 1, 1, 1]);
        assert_eq!(decoded.chan_values, [0.0; 4]);
    }

    #[test]
    fn decode_packet_bit0_alone_lights_up_every_channel() {
        // Direct consequence of the non-power-of-2 masks (quirk 1): a
        // device reporting only channel 1 present (channel_bit_field == 1)
        // makes every channel's availability test pass, so all four raw
        // dwords get read and scaled even though only one channel is
        // actually present.
        let mut buf = header_bytes(true, 1, 7);
        buf[32..36].copy_from_slice(&123u32.to_le_bytes());
        buf[36..40].copy_from_slice(&456u32.to_le_bytes());
        buf[40..44].copy_from_slice(&789u32.to_le_bytes());
        buf[44..48].copy_from_slice(&321u32.to_le_bytes());
        let decoded = decode_packet(&buf, [1, 1, 1, 1]);
        assert!(decoded.chan_values.iter().all(|v| *v != 0.0));
    }

    #[test]
    fn decode_packet_parses_header_fields() {
        let mut buf = header_bytes(true, 0, 99);
        buf[4..8].copy_from_slice(&11u32.to_le_bytes());
        buf[8..12].copy_from_slice(&22u32.to_le_bytes());
        buf[20..24].copy_from_slice(&33u32.to_le_bytes());
        buf[24..26].copy_from_slice(&44u16.to_le_bytes());
        buf[26..28].copy_from_slice(&55u16.to_le_bytes());
        let decoded = decode_packet(&buf, [0, 0, 0, 0]);
        assert_eq!(decoded.header.order_number, 11);
        assert_eq!(decoded.header.serial_number, 22);
        assert_eq!(decoded.header.status, 33);
        assert_eq!(decoded.header.frame_number_m, 44);
        assert_eq!(decoded.header.bytes_per_frame, 55);
        assert_eq!(decoded.header.meas_value_counter, 99);
    }

    #[test]
    fn apply_read_outcome_good_packet_clears_bad_count_and_communicating() {
        let mut state = ReaderState::new(5);
        state.bad_packet_count = 3;
        state.is_communicating = false;
        let outcome = apply_read_outcome(&mut state, ReadOutcome::GoodPacket);
        assert_eq!(outcome, Some(true));
        assert_eq!(state.bad_packet_count, 0);
        assert!(state.is_communicating);
        assert_eq!(state.read_timeout_secs, 5.0);
        assert_eq!(state.stats.good_count, 1);
    }

    #[test]
    fn apply_read_outcome_single_bad_preamble_does_not_flip_communicating() {
        let mut state = ReaderState::new(5);
        state.is_communicating = true;
        let outcome = apply_read_outcome(&mut state, ReadOutcome::BadPreamble);
        assert_eq!(outcome, None);
        assert_eq!(state.bad_packet_count, 1);
        assert!(state.is_communicating);
        assert_eq!(state.stats.bad_preamble_count, 1);
    }

    #[test]
    fn apply_read_outcome_second_consecutive_bad_flips_communicating_and_forces_invalid_push() {
        let mut state = ReaderState::new(5);
        state.is_communicating = true;
        apply_read_outcome(&mut state, ReadOutcome::BadPreamble);
        let outcome = apply_read_outcome(&mut state, ReadOutcome::ReadError);
        assert_eq!(outcome, Some(false));
        assert_eq!(state.bad_packet_count, 2);
        assert!(!state.is_communicating);
        assert_eq!(state.stats.bad_read_count, 1);
    }

    #[test]
    fn apply_read_outcome_timeout_bumps_read_timeout_to_30s() {
        let mut state = ReaderState::new(5);
        apply_read_outcome(&mut state, ReadOutcome::Timeout);
        assert_eq!(state.read_timeout_secs, 30.0);
        assert_eq!(state.stats.timeout_count, 1);
    }

    #[test]
    fn process_data_packet_first_call_initializes_expected_counter_without_gap_counting() {
        let mut state = ReaderState::new(1);
        state.is_communicating = true;
        let update = process_data_packet(&mut state, 42, [1.0, 2.0, 3.0, 4.0], true, true);
        assert!(update.is_some());
        assert_eq!(state.stats.out_of_sequence_count, 0);
        assert_eq!(state.stats.missed_count, 0);
    }

    #[test]
    fn process_data_packet_duplicate_counter_is_ignored_and_not_pushed() {
        let mut state = ReaderState::new(1);
        state.is_communicating = true;
        // First call to counter 10 advances expected_meas_value_counter to
        // 11; a genuine duplicate is the SAME counter (10) arriving again,
        // giving diff = 10 - 11 == -1.
        process_data_packet(&mut state, 10, [1.0, 1.0, 1.0, 1.0], true, true);
        let update = process_data_packet(&mut state, 10, [1.0, 1.0, 1.0, 1.0], true, true);
        assert_eq!(update, None);
        assert_eq!(state.stats.duplicate_count, 1);
    }

    #[test]
    fn process_data_packet_missed_packets_are_counted_by_gap_size() {
        let mut state = ReaderState::new(1);
        state.is_communicating = true;
        // First call to counter 100 advances expected_meas_value_counter to
        // 101; the next real packet at 104 is a gap of 3 (104 - 101).
        process_data_packet(&mut state, 100, [0.0; 4], true, true);
        process_data_packet(&mut state, 104, [0.0; 4], true, true);
        assert_eq!(state.stats.missed_count, 3);
        assert_eq!(state.stats.out_of_sequence_count, 1);
    }

    #[test]
    fn process_data_packet_gap_counting_gated_on_is_communicating() {
        let mut state = ReaderState::new(1);
        state.is_communicating = false;
        process_data_packet(&mut state, 10, [0.0; 4], true, true);
        // Big jump, but isCommunicating was false throughout -- no counting.
        process_data_packet(&mut state, 999, [0.0; 4], true, true);
        assert_eq!(state.stats.missed_count, 0);
        assert_eq!(state.stats.out_of_sequence_count, 0);
    }

    #[test]
    fn process_data_packet_not_pushed_before_interrupt_accept() {
        let mut state = ReaderState::new(1);
        let update = process_data_packet(&mut state, 1, [5.0; 4], true, false);
        assert_eq!(update, None);
        // Averaging still doesn't happen (early return before the isValid block).
        assert_eq!(state.disp_sum_count, 0);
    }

    #[test]
    fn process_data_packet_throttle_holds_back_push_until_counter_expires() {
        let mut state = ReaderState::new(3);
        state.is_communicating = true;
        let mut pushes = 0;
        for counter in 1u32..=3 {
            let update = process_data_packet(&mut state, counter, [1.0; 4], true, true);
            if update.is_some() {
                pushes += 1;
            }
        }
        assert_eq!(pushes, 1);
    }

    #[test]
    fn process_data_packet_averages_across_throttled_cycles() {
        let mut state = ReaderState::new(2);
        state.is_communicating = true;
        process_data_packet(&mut state, 1, [10.0, 0.0, 0.0, 0.0], true, true);
        let update = process_data_packet(&mut state, 2, [20.0, 0.0, 0.0, 0.0], true, true).unwrap();
        assert_eq!(update.values[0], 15.0);
        assert!(update.valid);
    }

    #[test]
    fn process_data_packet_invalid_push_does_not_accumulate_averages() {
        let mut state = ReaderState::new(1);
        state.is_communicating = true;
        let update = process_data_packet(&mut state, 1, [7.0; 4], false, true).unwrap();
        assert!(!update.valid);
        // isValid == false means the (--pvThrottleCounter) decrement never
        // runs and the disp sums are never accumulated, matching
        // `capaNCDT6200Sup.c:181-198`'s isValid-gated block.
        assert_eq!(state.disp_sum_count, 0);
    }
}
