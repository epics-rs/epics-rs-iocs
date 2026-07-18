//! DP5 status-packet decode, ported from `DP5Status.cpp`'s `Process_Status`
//! and the small helpers it and `drvAmptek.cpp` call
//! (`DppUtilities.cpp`, `DppConst.h`, `DP5Status.cpp:653-682`).
//!
//! Only the fields `drvAmptek.cpp` actually reads out of the 64-byte DP4
//! Format Status block are decoded (feasibility gate: "port only what the
//! driver uses") -- `DP4_FORMAT_STATUS` has ~50 fields, of which
//! `drvAmptek.cpp::processStatus` (`drvAmptek.cpp:964-1004` /
//! `readStatus` call sites) reads the set captured in [`Dp5Status`].

/// `DppConst.h enum dp5DppTypes`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DppType {
    Dp5,
    Px5,
    Dp5G,
    Mca8000D,
    Tb5,
    Dp5X,
}

impl DppType {
    /// `CDP5Status::GetDeviceNameFromVal` (`DP5Status.cpp:653-682`): any
    /// value not in `0..=5` names as `"DP5"`, matching C's `default` arm.
    pub fn from_device_id(device_id: u8) -> Self {
        match device_id {
            0 => DppType::Dp5,
            1 => DppType::Px5,
            2 => DppType::Dp5G,
            3 => DppType::Mca8000D,
            4 => DppType::Tb5,
            5 => DppType::Dp5X,
            _ => DppType::Dp5,
        }
    }

    /// `CDP5Status::GetDeviceNameFromVal` (`DP5Status.cpp:653-682`).
    pub fn name(self) -> &'static str {
        match self {
            DppType::Dp5 => "DP5",
            DppType::Px5 => "PX5",
            DppType::Dp5G => "DP5G",
            DppType::Mca8000D => "MCA8000D",
            DppType::Tb5 => "TB5",
            DppType::Dp5X => "DP5-X",
        }
    }
}

/// `CDppUtilities::LongWordToDouble` (`DppUtilities.cpp`): little-endian
/// 4-byte accumulate starting at `start`, `sum(buf[start+i] * 256^i for i
/// in 0..4)`. Panics if `start + 4 > buf.len()` -- every call site in
/// [`process_status`] uses a fixed, in-bounds `start` against a
/// known-64-byte block, so an out-of-range slice indicates a malformed
/// caller, not device data to tolerate.
fn long_word_to_double(start: usize, buf: &[u8]) -> f64 {
    let mut total = 0.0_f64;
    let mut mult = 1.0_f64;
    for &byte in &buf[start..start + 4] {
        total += f64::from(byte) * mult;
        mult *= 256.0;
    }
    total
}

/// `CDppUtilities::BYTEVersionToString` (`DppUtilities.cpp`): `"{major}.
/// {minor:02}"` with `major = (v & 240) / 16`, `minor = v & 15`.
pub fn byte_version_to_string(v: u8) -> String {
    format!("{}.{:02}", (v & 240) / 16, v & 15)
}

/// The decoded DP4-format status block, restricted to the fields
/// `drvAmptek.cpp` reads. Field names mirror `DP4_FORMAT_STATUS`
/// (`DP5Status.h`) / `drvAmptek.cpp`'s param names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dp5Status {
    pub device_id: u8,
    pub dpp_type: DppType,
    pub fast_count: f64,
    pub slow_count: f64,
    pub accumulation_time: f64,
    pub real_time: f64,
    pub firmware: u8,
    pub fpga: u8,
    pub build: u8,
    /// `None` when `RAW[29] >= 128` (C's serial-number-invalid sentinel,
    /// represented there as `-1`; `Option` makes "no valid serial" a type
    /// state instead of a magic negative value).
    pub serial_number: Option<u32>,
    pub high_voltage: f64,
    pub det_temp: f64,
    pub dp5_temp: f64,
    pub preset_rt_done: bool,
    pub preset_lt_done: bool,
    pub mca_enabled: bool,
    pub precnt_reached: bool,
    pub mcs_done: bool,
    pub pc5_present: bool,
    pub dpp_eco: u8,
    pub is_dp5_rev_dx_gains: bool,
}

/// `CDP5Status::Process_Status` (`DP5Status.cpp`), restricted to the
/// fields `drvAmptek.cpp` reads (see the module doc). `raw` is the 64-byte
/// DP4-format status block extracted as
/// [`crate::protocol::ReceivedPacket::Status`]'s `data`.
///
/// Panics if `raw.len() < 64` -- C indexes the fixed-size buffer directly
/// without a length check (`DP5Status.cpp`'s `Process_Status`); a status
/// packet is only ever produced by [`crate::protocol::parse_packet`] from
/// wire bytes whose `LEN` field the sender fixes at 64, so a short buffer
/// here means the transport handed over a malformed/truncated packet, not
/// legitimate device data.
pub fn process_status(raw: &[u8]) -> Dp5Status {
    assert!(
        raw.len() >= 64,
        "DP5 status block must be 64 bytes, got {}",
        raw.len()
    );

    let device_id = raw[39];
    let dpp_type = DppType::from_device_id(device_id);
    let fast_count = long_word_to_double(0, raw);
    let slow_count = long_word_to_double(4, raw);
    let accumulation_time = f64::from(raw[12]) * 0.001
        + (f64::from(raw[13]) + f64::from(raw[14]) * 256.0 + f64::from(raw[15]) * 65536.0) * 0.1;
    let real_time = (f64::from(raw[20])
        + f64::from(raw[21]) * 256.0
        + f64::from(raw[22]) * 65536.0
        + f64::from(raw[23]) * 16_777_216.0)
        * 0.001;
    let firmware = raw[24];
    let fpga = raw[25];
    let build = if firmware > 0x65 { raw[37] & 0xF } else { 0 };
    let serial_number = if raw[29] < 128 {
        Some(long_word_to_double(26, raw) as u32)
    } else {
        None
    };
    let high_voltage = if raw[30] < 128 {
        (f64::from(raw[31]) + f64::from(raw[30]) * 256.0) * 0.5
    } else {
        ((f64::from(raw[31]) + f64::from(raw[30]) * 256.0) - 65536.0) * 0.5
    };
    let det_temp = (f64::from(raw[33]) + f64::from(raw[32] & 15) * 256.0) * 0.1;
    // sign-extend an 8-bit value: DP5_TEMP = RAW[34] - (RAW[34]&128)*2
    let dp5_temp = f64::from(raw[34]) - f64::from(raw[34] & 128) * 2.0;
    let preset_rt_done = raw[35] & 128 != 0;
    let preset_lt_done = raw[35] & 64 != 0;
    let mca_enabled = raw[35] & 32 != 0;
    let precnt_reached = raw[35] & 16 != 0;
    let mcs_done = raw[36] & 64 != 0;
    let pc5_present = raw[38] & 128 != 0;
    let dpp_eco = raw[49];
    let is_dp5_rev_dx_gains = dpp_type == DppType::Dp5
        && (u16::from(firmware) << 8 | u16::from(build)) >= 0x686
        && dpp_eco < 0xFF;

    Dp5Status {
        device_id,
        dpp_type,
        fast_count,
        slow_count,
        accumulation_time,
        real_time,
        firmware,
        fpga,
        build,
        serial_number,
        high_voltage,
        det_temp,
        dp5_temp,
        preset_rt_done,
        preset_lt_done,
        mca_enabled,
        precnt_reached,
        mcs_done,
        pc5_present,
        dpp_eco,
        is_dp5_rev_dx_gains,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raw_fixture() -> [u8; 64] {
        [0u8; 64]
    }

    #[test]
    fn zeroed_block_decodes_to_zeroed_status() {
        let s = process_status(&raw_fixture());
        assert_eq!(s.device_id, 0);
        assert_eq!(s.dpp_type, DppType::Dp5);
        assert_eq!(s.fast_count, 0.0);
        assert_eq!(s.slow_count, 0.0);
        assert_eq!(s.serial_number, Some(0));
        assert_eq!(s.high_voltage, 0.0);
        assert!(!s.preset_rt_done);
        assert!(!s.mca_enabled);
    }

    /// `FastCount`/`SlowCount`: `LongWordToDouble` little-endian accumulate.
    #[test]
    fn fast_and_slow_count_are_little_endian() {
        let mut raw = raw_fixture();
        raw[0..4].copy_from_slice(&[0x01, 0x00, 0x00, 0x01]); // 1 + 1*256^3
        raw[4..8].copy_from_slice(&[0xFF, 0x00, 0x00, 0x00]); // 255
        let s = process_status(&raw);
        assert_eq!(s.fast_count, 1.0 + 256.0_f64.powi(3));
        assert_eq!(s.slow_count, 255.0);
    }

    /// `Build = Firmware>0x65 ? (RAW[37]&0xF) : 0`.
    #[test]
    fn build_is_gated_by_firmware_version() {
        let mut raw = raw_fixture();
        raw[37] = 0xAB;
        raw[24] = 0x65; // firmware == 0x65, not > 0x65
        assert_eq!(process_status(&raw).build, 0);
        raw[24] = 0x66;
        assert_eq!(process_status(&raw).build, 0xB);
    }

    /// `SerialNumber` sentinel: `RAW[29] >= 128` means invalid/unknown.
    /// `RAW[29]` is simultaneously the validity gate and the top byte of
    /// the little-endian 4-byte value (`LongWordToDouble(26, RAW)` reads
    /// `RAW[26..30]`), so a valid `RAW[29]` still contributes its own
    /// magnitude to the decoded number.
    #[test]
    fn serial_number_invalid_sentinel() {
        let mut raw = raw_fixture();
        raw[29] = 128;
        assert_eq!(process_status(&raw).serial_number, None);
        raw[26] = 0x11;
        raw[27] = 0x22;
        raw[28] = 0x33;
        raw[29] = 0x44; // < 128: valid
        let expected = 0x11 + 0x22 * 256 + 0x33 * 256 * 256 + 0x44 * 256 * 256 * 256;
        assert_eq!(process_status(&raw).serial_number, Some(expected));
    }

    /// `HV`: two's-complement-ish sign convention on `RAW[30]>=128`.
    #[test]
    fn high_voltage_sign_convention() {
        let mut raw = raw_fixture();
        raw[30] = 0x00;
        raw[31] = 0x0A; // (10 + 0*256) * 0.5 = 5.0
        assert_eq!(process_status(&raw).high_voltage, 5.0);

        raw[30] = 0xFF; // >= 128 branch
        raw[31] = 0x00;
        // ((0 + 255*256) - 65536) * 0.5 = (65280 - 65536) * 0.5 = -128.0
        assert_eq!(process_status(&raw).high_voltage, -128.0);
    }

    /// `DP5_TEMP`: sign-extend an 8-bit value.
    #[test]
    fn dp5_temp_sign_extends() {
        let mut raw = raw_fixture();
        raw[34] = 10;
        assert_eq!(process_status(&raw).dp5_temp, 10.0);
        raw[34] = 0xF6; // 246 -> -10 after sign extension
        assert_eq!(process_status(&raw).dp5_temp, -10.0);
    }

    /// Status bitflags packed into `RAW[35]`/`RAW[36]`/`RAW[38]`.
    #[test]
    fn status_bitflags() {
        let mut raw = raw_fixture();
        raw[35] = 128 | 64 | 32 | 16;
        raw[36] = 64;
        raw[38] = 128;
        let s = process_status(&raw);
        assert!(s.preset_rt_done);
        assert!(s.preset_lt_done);
        assert!(s.mca_enabled);
        assert!(s.precnt_reached);
        assert!(s.mcs_done);
        assert!(s.pc5_present);
    }

    #[test]
    fn device_names() {
        assert_eq!(DppType::from_device_id(0).name(), "DP5");
        assert_eq!(DppType::from_device_id(1).name(), "PX5");
        assert_eq!(DppType::from_device_id(2).name(), "DP5G");
        assert_eq!(DppType::from_device_id(3).name(), "MCA8000D");
        assert_eq!(DppType::from_device_id(4).name(), "TB5");
        assert_eq!(DppType::from_device_id(5).name(), "DP5-X");
        assert_eq!(DppType::from_device_id(200).name(), "DP5");
    }

    #[test]
    fn byte_version_formats_as_major_dot_minor() {
        assert_eq!(byte_version_to_string(0x67), "6.07");
        assert_eq!(byte_version_to_string(0x00), "0.00");
    }

    #[test]
    #[should_panic(expected = "DP5 status block must be 64 bytes")]
    fn process_status_rejects_short_buffer() {
        process_status(&[0u8; 10]);
    }
}
