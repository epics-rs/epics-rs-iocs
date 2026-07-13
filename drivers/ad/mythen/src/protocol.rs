//! The Mythen socket protocol as pure functions (port of the command builders,
//! reply decoders and `decodeRawReadout` in `mythen.cpp`).
//!
//! Commands are ASCII (`-get status`, `-time 10000000`); the output EOS the
//! detector needs is appended by the asyn IP port (`asynOctetSetOutputEos`),
//! not here. Replies are raw binary, little-endian, and fixed width: 4-byte
//! `int32` for almost everything, 4-byte `float32` for `-get tau`, and a
//! 7-byte ASCII string for `-get version`.

use epics_rs::ad_core::driver::ADStatus;

/// Channels per module (C `MAX_DIMS`).
pub const CHANNELS_PER_MODULE: usize = 1280;
/// Width of an integer reply (C `sizeof(int)`).
pub const INT_REPLY_LEN: usize = 4;
/// Width of the `-get version` reply (C `firmwareVersion_[7]`).
pub const VERSION_REPLY_LEN: usize = 7;

/// Read modes (C `SD_READ_MODE`, `readmode_`).
pub const READ_MODE_RAW: i32 = 0;

/// Trigger modes (C `SD_TRIGGER`).
pub const TRIGGER_NONE: i32 = 0;
pub const TRIGGER_SINGLE: i32 = 1;
pub const TRIGGER_CONTINUOUS: i32 = 2;

/// Image modes (C `ADImageMode`).
pub const IMAGE_MODE_SINGLE: i32 = 0;

/// The detector counts time in units of 100 ns (C `value * 1E+7`).
pub fn to_hundred_ns(seconds: f64) -> i32 {
    (seconds * 1e7) as i32
}

/// …and back (C `aux * 1E-7`).
pub fn from_hundred_ns(hns: i32) -> f64 {
    f64::from(hns) * 1e-7
}

/// Decode a 4-byte integer reply.
///
/// C reinterpret_casts the reply buffer and byte-swaps only when the *host* is
/// big-endian (`isBigEndian_ = EPICS_BYTE_ORDER == EPICS_ENDIAN_BIG`), i.e. it
/// treats the wire as little-endian. Reading it as little-endian here is the
/// same thing, and is host-independent by construction.
pub fn decode_i32(reply: &[u8]) -> Option<i32> {
    let b: [u8; 4] = reply.get(..4)?.try_into().ok()?;
    Some(i32::from_le_bytes(b))
}

/// Decode a 4-byte float reply (`-get tau`).
pub fn decode_f32(reply: &[u8]) -> Option<f32> {
    let b: [u8; 4] = reply.get(..4)?.try_into().ok()?;
    Some(f32::from_le_bytes(b))
}

/// Decode the 7-byte `-get version` reply into a string.
pub fn decode_version(reply: &[u8]) -> String {
    let end = reply.iter().position(|&b| b == 0).unwrap_or(reply.len());
    String::from_utf8_lossy(&reply[..end]).trim().to_string()
}

/// The firmware major version (C `(int)firmwareVersion_[1] % 48`).
///
/// The version string is `M<major>.<minor>.<patch>`, so byte 1 is the major
/// digit and `% 48` turns its ASCII code into the digit. Firmware 3 and later
/// support `-energy`, `-get delafter`, `-get conttrig` and the short settings
/// names (`Cu` rather than `StdCu`).
pub fn firmware_major(version: &str) -> u32 {
    version
        .as_bytes()
        .get(1)
        .map(|b| u32::from(*b) % 48)
        .unwrap_or(0)
}

/// The `-nbits` value behind a `SD_BIT_DEPTH` menu index (C `setBitDepth`).
pub fn nbits_of_bit_depth(index: i32) -> i32 {
    match index {
        1 => 16,
        2 => 8,
        3 => 4,
        _ => 24,
    }
}

/// How many channels one 32-bit raw word carries (C `decodeRawReadout`).
pub fn channels_per_word(nbits: i32) -> usize {
    match nbits {
        16 => 2,
        8 => 4,
        4 => 8,
        _ => 1,
    }
}

/// The mask of one channel within a raw word (C `decodeRawReadout`).
pub fn channel_mask(nbits: i32) -> u32 {
    match nbits {
        16 => 0xffff,
        8 => 0xff,
        4 => 0xf,
        _ => 0x00ff_ffff,
    }
}

/// How many bytes a readout of `nmodules` modules at `nbits` is expected to be.
///
/// C computes this from `chanperline_ = 32 / nbits` (integer division, so 24
/// bits gives 1 channel per word), which is [`channels_per_word`] for every
/// bit depth the detector accepts.
pub fn readout_len(read_mode: i32, nmodules: usize, nbits: i32) -> usize {
    let words_per_module = if read_mode == READ_MODE_RAW {
        CHANNELS_PER_MODULE / channels_per_word(nbits)
    } else {
        CHANNELS_PER_MODULE
    };
    INT_REPLY_LEN * nmodules * words_per_module
}

/// Unpack a `-readoutraw` reply into one count per channel
/// (C `decodeRawReadout`, mythen.cpp:1026).
///
/// The raw readout packs `32 / nbits` channels into each 32-bit word; the
/// corrected (`-readout`) reply is one 24-bit count per word, which is the same
/// function with `nbits = 24`.
pub fn decode_raw_readout(nmodules: usize, nbits: i32, data: &[u32]) -> Vec<u32> {
    let per_word = channels_per_word(nbits);
    let mask = channel_mask(nbits);
    let words = (CHANNELS_PER_MODULE / per_word) * nmodules;
    let words = words.min(data.len());

    let mut result = vec![0u32; words * per_word];
    for (j, shift) in (0..per_word).map(|j| (j, (nbits as usize * j) as u32)) {
        for i in 0..words {
            result[i * per_word + j] = (data[i] >> shift) & mask;
        }
    }
    result
}

/// Reinterpret a readout reply as little-endian 32-bit words.
pub fn words_from_bytes(bytes: &[u8]) -> Vec<u32> {
    bytes
        .chunks_exact(4)
        .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect()
}

/// What `-get status` reports (C `getStatus`, mythen.cpp:509).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusBits {
    /// Bit 0: an acquisition is running.
    pub running: bool,
    /// Bit 3: waiting for a trigger.
    pub waiting_for_trigger: bool,
    /// Bit 16: *no* data is available.
    pub no_data: bool,
}

pub fn status_bits(aux: i32) -> StatusBits {
    StatusBits {
        running: aux & 1 != 0,
        waiting_for_trigger: aux & (1 << 3) != 0,
        no_data: aux & (1 << 16) != 0,
    }
}

/// A detector that is neither running nor holding data is idle
/// (C `!(m_status || !d_status)`, mythen.cpp:523).
pub fn is_idle(bits: StatusBits) -> bool {
    !bits.running && bits.no_data
}

/// What a non-idle detector is doing, once the wait for a trigger is over
/// (C, mythen.cpp:542-545). `timed_out` is C's
/// `triggerWaitCnt == MAX_TRIGGER_TIMEOUT_COUNT`.
pub fn status_after_wait(bits: StatusBits, timed_out: bool) -> ADStatus {
    if timed_out {
        ADStatus::Error
    } else if !bits.no_data {
        ADStatus::Readout
    } else {
        ADStatus::Acquire
    }
}

/// How long to wait before polling the status again, on the `count`-th poll
/// spent waiting for a trigger (C `triggerWait`, mythen.cpp:531).
pub fn trigger_backoff(count: u32) -> std::time::Duration {
    std::time::Duration::from_secs_f64(0.0001 * 10f64.powf(f64::from(count / 10) + 1.0))
}

/// The settings command for a `SD_SETTING` menu index (C `loadSettings`).
///
/// Firmware 2 and older spell the standard settings `StdCu` / `StdMo` / `HgCr`;
/// firmware 3 and later use the element names. Silver is firmware-3-only, and
/// an unknown index resets the module.
pub fn settings_command(index: i32, firmware_major: u32) -> &'static str {
    let modern = firmware_major >= 3;
    match index {
        0 if modern => "-settings Cu",
        0 => "-settings StdCu",
        1 if modern => "-settings Mo",
        1 => "-settings StdMo",
        2 => "-settings Ag",
        3 if modern => "-settings Cr",
        3 => "-settings HgCr",
        _ => "-reset",
    }
}

/// The trigger command for a `SD_TRIGGER` menu index (C `setTrigger`).
///
/// Clearing `trigen` clears `conttrig` as well, so "None" needs only the one
/// command. An index outside the menu sends nothing.
pub fn trigger_command(mode: i32) -> Option<&'static str> {
    match mode {
        TRIGGER_NONE => Some("-trigen 0"),
        TRIGGER_SINGLE => Some("-trigen 1"),
        TRIGGER_CONTINUOUS => Some("-conttrigen 1"),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integer_replies_are_little_endian() {
        assert_eq!(decode_i32(&[0x01, 0x00, 0x00, 0x00]), Some(1));
        assert_eq!(decode_i32(&[0xff, 0xff, 0xff, 0xff]), Some(-1));
        assert_eq!(decode_i32(&[0x00, 0x00, 0x01, 0x00]), Some(65536));
        assert_eq!(decode_i32(&[0x01, 0x00]), None);
    }

    #[test]
    fn tau_replies_are_little_endian_floats() {
        assert_eq!(decode_f32(&(-1.0f32).to_le_bytes()), Some(-1.0));
        assert_eq!(decode_f32(&(1.23f32).to_le_bytes()), Some(1.23));
        assert_eq!(decode_f32(&[0u8; 3]), None);
    }

    #[test]
    fn the_version_reply_stops_at_the_first_nul() {
        assert_eq!(decode_version(b"M3.0.0\0"), "M3.0.0");
        assert_eq!(decode_version(b"M2.0.5\0"), "M2.0.5");
        // A reply that fills all seven bytes is still a valid string.
        assert_eq!(decode_version(b"M3.0.10"), "M3.0.10");
    }

    #[test]
    fn the_firmware_major_is_the_second_character() {
        assert_eq!(firmware_major("M3.0.0"), 3);
        assert_eq!(firmware_major("M2.0.5"), 2);
        assert_eq!(firmware_major(""), 0);
    }

    #[test]
    fn settings_names_depend_on_the_firmware() {
        assert_eq!(settings_command(0, 3), "-settings Cu");
        assert_eq!(settings_command(0, 2), "-settings StdCu");
        assert_eq!(settings_command(1, 3), "-settings Mo");
        assert_eq!(settings_command(1, 2), "-settings StdMo");
        assert_eq!(settings_command(2, 3), "-settings Ag");
        assert_eq!(settings_command(2, 2), "-settings Ag");
        assert_eq!(settings_command(3, 3), "-settings Cr");
        assert_eq!(settings_command(3, 2), "-settings HgCr");
        assert_eq!(settings_command(9, 3), "-reset");
    }

    #[test]
    fn trigger_commands() {
        assert_eq!(trigger_command(TRIGGER_NONE), Some("-trigen 0"));
        assert_eq!(trigger_command(TRIGGER_SINGLE), Some("-trigen 1"));
        assert_eq!(trigger_command(TRIGGER_CONTINUOUS), Some("-conttrigen 1"));
        assert_eq!(trigger_command(7), None);
    }

    #[test]
    fn exposure_times_are_hundreds_of_nanoseconds() {
        assert_eq!(to_hundred_ns(1.0), 10_000_000);
        assert_eq!(to_hundred_ns(0.1), 1_000_000);
        assert_eq!(from_hundred_ns(10_000_000), 1.0);
    }

    #[test]
    fn bit_depth_menu_maps_to_nbits() {
        assert_eq!(nbits_of_bit_depth(0), 24);
        assert_eq!(nbits_of_bit_depth(1), 16);
        assert_eq!(nbits_of_bit_depth(2), 8);
        assert_eq!(nbits_of_bit_depth(3), 4);
        assert_eq!(nbits_of_bit_depth(99), 24);
    }

    #[test]
    fn a_raw_readout_is_sized_by_the_bit_depth() {
        // 24 bits: one channel per word.
        assert_eq!(readout_len(READ_MODE_RAW, 1, 24), 4 * 1280);
        assert_eq!(readout_len(READ_MODE_RAW, 2, 24), 4 * 2560);
        // 4 bits: eight channels per word.
        assert_eq!(readout_len(READ_MODE_RAW, 1, 4), 4 * 160);
        // The corrected readout is always one word per channel.
        assert_eq!(readout_len(1, 2, 4), 4 * 2560);
    }

    #[test]
    fn a_24_bit_raw_readout_is_one_channel_per_word() {
        let data: Vec<u32> = (0..1280).map(|i| i as u32 | 0xff00_0000).collect();
        let out = decode_raw_readout(1, 24, &data);
        assert_eq!(out.len(), 1280);
        // The top byte is not part of a 24-bit count.
        assert_eq!(out[0], 0);
        assert_eq!(out[7], 7);
        assert_eq!(out[1279], 1279);
    }

    #[test]
    fn an_8_bit_raw_readout_unpacks_four_channels_per_word() {
        // Word i carries channels 4i..4i+3 in ascending byte order.
        let data: Vec<u32> = (0..320)
            .map(|i| {
                let b = |k: u32| (4 * i + k) % 251;
                b(0) | (b(1) << 8) | (b(2) << 16) | (b(3) << 24)
            })
            .collect();
        let out = decode_raw_readout(1, 8, &data);
        assert_eq!(out.len(), 1280);
        for (i, v) in out.iter().enumerate() {
            assert_eq!(*v, (i as u32) % 251, "channel {i}");
        }
    }

    #[test]
    fn a_4_bit_raw_readout_unpacks_eight_channels_per_word() {
        let data: Vec<u32> = (0..160)
            .map(|i| {
                (0..8).fold(0u32, |acc, k| {
                    let v = ((8 * i + k) % 16) as u32;
                    acc | (v << (4 * k))
                })
            })
            .collect();
        let out = decode_raw_readout(1, 4, &data);
        assert_eq!(out.len(), 1280);
        for (i, v) in out.iter().enumerate() {
            assert_eq!(*v, (i as u32) % 16, "channel {i}");
        }
    }

    #[test]
    fn a_16_bit_raw_readout_covers_two_modules() {
        let data: Vec<u32> = (0..1280)
            .map(|i| {
                let lo = (2 * i) as u32 & 0xffff;
                let hi = (2 * i + 1) as u32 & 0xffff;
                lo | (hi << 16)
            })
            .collect();
        let out = decode_raw_readout(2, 16, &data);
        assert_eq!(out.len(), 2560);
        for (i, v) in out.iter().enumerate() {
            assert_eq!(*v, (i as u32) & 0xffff, "channel {i}");
        }
    }

    #[test]
    fn a_short_readout_decodes_only_the_words_that_arrived() {
        let data = vec![7u32; 100];
        let out = decode_raw_readout(1, 24, &data);
        assert_eq!(out.len(), 100);
        assert!(out.iter().all(|&v| v == 7));
    }

    #[test]
    fn status_bits_are_independent() {
        let s = status_bits(0);
        assert_eq!(
            s,
            StatusBits {
                running: false,
                waiting_for_trigger: false,
                no_data: false
            }
        );
        assert!(status_bits(1).running);
        assert!(status_bits(1 << 3).waiting_for_trigger);
        assert!(status_bits(1 << 16).no_data);
        let all = status_bits(1 | (1 << 3) | (1 << 16));
        assert!(all.running && all.waiting_for_trigger && all.no_data);
    }

    #[test]
    fn only_a_stopped_detector_with_no_data_is_idle() {
        // C: idle iff !(m_status || !d_status).
        assert!(is_idle(status_bits(1 << 16)));
        assert!(!is_idle(status_bits(0))); // data waiting, not running
        assert!(!is_idle(status_bits(1))); // running
        assert!(!is_idle(status_bits(1 | (1 << 16))));
    }

    #[test]
    fn the_status_after_the_trigger_wait() {
        assert_eq!(
            status_after_wait(status_bits(1), false),
            ADStatus::Readout,
            "running, data available"
        );
        assert_eq!(
            status_after_wait(status_bits(1 | (1 << 16)), false),
            ADStatus::Acquire,
            "running, nothing to read yet"
        );
        // A trigger that never arrived beats every other bit.
        assert_eq!(status_after_wait(status_bits(1), true), ADStatus::Error);
        assert_eq!(
            status_after_wait(status_bits(1 | (1 << 16)), true),
            ADStatus::Error
        );
    }

    #[test]
    fn the_trigger_backoff_grows_every_ten_polls() {
        // C: 0.0001 * 10^((cnt/10) + 1).
        assert_eq!(trigger_backoff(0).as_secs_f64(), 0.001);
        assert_eq!(trigger_backoff(9).as_secs_f64(), 0.001);
        assert_eq!(trigger_backoff(10).as_secs_f64(), 0.01);
        assert_eq!(trigger_backoff(40).as_secs_f64(), 10.0);
        // 50 polls is MAX_TRIGGER_TIMEOUT_COUNT: about a minute in total.
        let total: f64 = (0..50).map(|n| trigger_backoff(n).as_secs_f64()).sum();
        assert!((total - 111.11).abs() < 0.01, "total wait {total}");
    }

    #[test]
    fn readout_bytes_become_little_endian_words() {
        let bytes = [1u8, 0, 0, 0, 0, 1, 0, 0];
        assert_eq!(words_from_bytes(&bytes), vec![1, 256]);
        // A trailing partial word is dropped, as a short reply is rejected
        // before this point.
        assert_eq!(words_from_bytes(&[1, 0, 0, 0, 9]), vec![1]);
    }
}
