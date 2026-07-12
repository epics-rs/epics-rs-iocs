//! Wire protocol of the Sydor T4U electrometer, shared by `drvT4U_EM.cpp` (the
//! Qt middle-layer driver) and `drvT4UDirect_EM.cpp` (the direct driver).
//!
//! Both talk the same register language on the command socket — `wr`, `bs`,
//! `bc`, `tr`, and the device's `rr`/`tr` replies — and publish the same data
//! samples; they differ in the transport (see [`crate::t4u`]).

use std::time::Duration;

use crate::drv_quad_em::QE_MAX_INPUTS;

/// C++ `MAX_COMMAND_LEN`.
pub const MAX_COMMAND_LEN: usize = 256;
/// C++ `T4U_EM_TIMEOUT` in `drvT4U_EM.cpp` (0.2 s).
pub const T4U_EM_TIMEOUT: Duration = Duration::from_millis(200);
/// C++ `T4U_EM_TIMEOUT` in `drvT4UDirect_EM.cpp` (0.1 s).
pub const T4U_DIRECT_TIMEOUT: Duration = Duration::from_millis(100);
/// C++ `T4U_CMD_PORT` — the direct driver's command socket is telnet.
pub const T4U_CMD_PORT: u16 = 23;
/// C++ `T4U_CMD_QUEUE_LEN`.
pub const CMD_QUEUE_LEN: usize = 300;
/// C++ `kT4U_MAX_DATA_SIZE`: the largest number of reads one frame can carry.
pub const MAX_DATA_SIZE: usize = 500;
/// C++ `NUM_RANGES` (`drvT4UDirect_EM.h`).
pub const NUM_RANGES: usize = 3;
/// The largest UDP frame the direct data thread accepts (C++ `MAX_PACKET_SIZE`).
pub const MAX_PACKET_SIZE: usize = 65535;

/// C++ `ranges_[MAX_RANGES]`: the transimpedance of each gain setting.
pub const RANGES: [f64; 8] = [5e6, 14955.12, 47.0, 1.0, 1.0, 1.0, 1.0, 1.0];
/// C++ `rawToCurrent`: `kVREF`.
pub const VREF: f64 = 1.50;
/// C++ `rawToCurrent`: the 20-bit ADC's half-scale.
pub const ADC_HALF_SCALE: f64 = 524288.0;

// --- Registers and bit masks (identical in both drivers) -------------------

pub const REG_T4U_CTRL: i32 = 0;
pub const BIAS_N_EN_MASK: u32 = 1 << 9;
pub const BIAS_P_EN_MASK: u32 = 1 << 10;
pub const PULSE_BIAS_EN_MASK: u32 = 1 << 18;
pub const PULSE_BIAS_OFF_REG: i32 = 22;
pub const PULSE_BIAS_ON_REG: i32 = 23;
pub const REG_T4U_FREQ: i32 = 1;
pub const REG_T4U_RANGE: i32 = 3;
pub const RANGE_SEL_MASK: u32 = 0x3;
pub const REG_BIAS_P_VOLTAGE: i32 = 4;
pub const REG_BIAS_N_VOLTAGE: i32 = 5;

/// C++ `WAIT_STATE_MASK` and friends (direct driver only).
pub const WAIT_STATE_MASK: u32 = 0x7 << 12;
pub const WAIT_STATE_INHIBIT_MASK: u32 = 0x3 << 12;
pub const WAIT_STATE_TRIGGER_MASK: u32 = 0x5 << 12;
pub const WAIT_STATE_MODE_NONE: i32 = 0;
pub const WAIT_STATE_MODE_INHIBIT: i32 = 1;
pub const WAIT_STATE_MODE_TRIGGER: i32 = 2;
/// C++ `REG_T4U_READS_PER_PACKET` (direct driver only).
pub const REG_READS_PER_PACKET: i32 = 24;

pub const REG_PID_CTRL: i32 = 19;
pub const PID_EN_MASK: u32 = 0x4;
pub const PID_CUTOUT_EN_MASK: u32 = 0x2;
pub const PID_HYST_REENABLE_MASK: u32 = 0x1;
pub const PID_POS_TRACK_MASK: u32 = 0x3;
pub const PID_POS_TRACK_SHIFT: u32 = 3;
pub const PID_CTRL_POL_MASK: u32 = 0x40;
pub const PID_EXT_CTRL_MASK: u32 = 0x80;
pub const REG_OUTPUT_MODE: i32 = 93;
pub const OUTPUT_MODE_MASK: u32 = 0x7;

pub const TXC_CALIB_SLOPE_BASE: i32 = 100;
pub const TXC_CALIB_OFFSET_BASE: i32 = 104;

// ===========================================================================
// PID register table
// ===========================================================================

/// C++ `T4U_Reg_T`, minus the asyn parameter index: one row of
/// `T4U_param_list.txt`, which the upstream build turns into
/// `gc_t4u_cpp_params.cpp`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct T4uReg {
    /// The register's address on the T4U.
    pub reg_num: i32,
    /// The asyn parameter name (`gc_t4u_hdr_string.h`).
    pub param: &'static str,
    pub pv_min: f64,
    pub pv_max: f64,
    pub reg_min: f64,
    pub reg_max: f64,
}

const fn reg(
    reg_num: i32,
    param: &'static str,
    pv_min: f64,
    pv_max: f64,
    reg_min: f64,
    reg_max: f64,
) -> T4uReg {
    T4uReg {
        reg_num,
        param,
        pv_min,
        pv_max,
        reg_min,
        reg_max,
    }
}

/// Rows in [`PID_REGS`].
pub const PID_REG_COUNT: usize = 19;

/// C++ `pidRegData_`, in the order of `T4U_param_list.txt`.
pub const PID_REGS: [T4uReg; PID_REG_COUNT] = [
    reg(50, "QE_PIDX_SP", -1.0, 1.0, -10000.0, 10000.0),
    reg(51, "QE_PIDX_KP", 0.0, 1.0, 0.0, 10000.0),
    reg(52, "QE_PIDX_KI", 0.0, 1.0, 0.0, 10000.0),
    reg(53, "QE_PIDX_KD", 0.0, 1.0, 0.0, 10000.0),
    reg(54, "QE_PIDX_SCALE", 0.0, 1.0, 0.0, 1.0),
    reg(56, "QE_PIDX_VSCALE", -5.0, 5.0, -50000.0, 50000.0),
    reg(57, "QE_PIDX_VOFFSET", 0.0, 10.0, 0.0, 100000.0),
    reg(60, "QE_PIDY_SP", -1.0, 1.0, -10000.0, 10000.0),
    reg(61, "QE_PIDY_KP", 0.0, 1.0, 0.0, 10000.0),
    reg(62, "QE_PIDY_KI", 0.0, 1.0, 0.0, 10000.0),
    reg(63, "QE_PIDY_KD", 0.0, 1.0, 0.0, 10000.0),
    reg(64, "QE_PIDY_SCALE", 0.0, 1.0, 0.0, 1.0),
    reg(66, "QE_PIDY_VSCALE", -5.0, 5.0, -50000.0, 50000.0),
    reg(67, "QE_PIDY_VOFFSET", 0.0, 10.0, 0.0, 100000.0),
    reg(90, "QE_PID_CUTOUT", 0.0, 1.0, 0.0, 1000.0),
    reg(91, "QE_PID_HYST", 0.0, 1.0, 0.0, 1000.0),
    reg(92, "QE_DAC_ITOV", 0.0, 1.0, 0.0, 1.0),
    reg(76, "QE_DAC_ITOV_OFFSET", 0.0, 10.0, 0.0, 100000.0),
    reg(20, "QE_POS_TRACK_RAD", -1.0, 1.0, -10000.0, 10000.0),
];

/// C++ `findRegByNum`.
pub fn find_reg_by_num(reg_num: i32) -> Option<usize> {
    PID_REGS.iter().position(|r| r.reg_num == reg_num)
}

/// C++ `scaleParamToReg`: linear map from the PV's range to the register's.
///
/// `clip` is C++'s defaulted-to-false second argument; no caller passes true.
pub fn scale_param_to_reg(value: f64, r: &T4uReg, clip: bool) -> f64 {
    let percent = (value - r.pv_min) / (r.pv_max - r.pv_min);
    let scaled = percent * (r.reg_max - r.reg_min) + r.reg_min;
    if !clip {
        return scaled;
    }
    scaled.clamp(r.reg_min, r.reg_max)
}

/// C++ `processRegVal`'s PID branch: the inverse map, register to PV.
pub fn scale_reg_to_param(reg_val: u32, r: &T4uReg) -> f64 {
    // C++ computes `(reg_val - reg_min)` in double after promoting the
    // uint32_t, so a register holding a two's-complement negative reads as a
    // large positive. That is what the device's `tr` reply means for these
    // registers only if reg_min >= 0; for the signed setpoints C++ is wrong by
    // 2^32. It is left alone here: the fix would need the device's declared
    // register signedness, which no source on hand states.
    let raw_percent = (reg_val as f64 - r.reg_min) / (r.reg_max - r.reg_min);
    raw_percent * (r.pv_max - r.pv_min) + r.pv_min
}

/// C++ `rawToCurrent`: 20-bit ADC counts through the range's transimpedance.
pub fn raw_to_current(raw: i32, range: i32) -> f64 {
    let idx = (range.clamp(0, RANGES.len() as i32 - 1)) as usize;
    raw as f64 / ADC_HALF_SCALE * VREF / RANGES[idx]
}

// ===========================================================================
// Commands
// ===========================================================================

/// C++ `"wr %i %i"`.
pub fn cmd_write(reg_num: i32, value: i32) -> String {
    format!("wr {reg_num} {value}")
}

/// C++ `"bs %i %i"` / `"bc %i %i"`.
///
/// C++ indexes `const char *enable_cmd[2] = {"bc", "bs"}` with the raw EPICS
/// value, so any value outside 0/1 reads past the end of the array. Here the
/// test is `value != 0`, which is what the two-element table means.
pub fn cmd_bits(set: bool, reg_num: i32, mask: u32) -> String {
    let verb = if set { "bs" } else { "bc" };
    format!("{verb} {reg_num} {}", mask as i32)
}

/// C++ writes the two bias enables with a hex literal mask
/// (`"bs 0 0x200"`/`"bc 0 0x400"`) where every other bit command uses decimal.
/// The device's parser is not on hand, so the bytes are kept as they are.
pub fn cmd_bias_enable(set: bool, positive: bool) -> String {
    let verb = if set { "bs" } else { "bc" };
    let mask = if positive { "0x400" } else { "0x200" };
    format!("{verb} 0 {mask}")
}

/// C++ `"tr %i %i"`: ask the device to dump a register range.
pub fn cmd_read_regs(first: i32, last: i32) -> String {
    format!("tr {first} {last}")
}

// ===========================================================================
// Register decoding
// ===========================================================================

/// What one `(reg_num, reg_val)` pair from the device means.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RegUpdate {
    /// A PID register: index into [`PID_REGS`], already scaled to PV units.
    Pid {
        index: usize,
        value: f64,
    },
    /// `REG_T4U_CTRL`: the three bias enables.
    Ctrl {
        bias_n: bool,
        bias_p: bool,
        pulse_bias: bool,
    },
    /// `REG_T4U_FREQ`: the sample frequency in Hz.
    SampleFreq(u32),
    /// `REG_T4U_RANGE`: the selected range.
    Range(i32),
    PulseBiasOff(i32),
    PulseBiasOn(i32),
    /// `REG_OUTPUT_MODE`: the DAC output mode.
    DacMode(i32),
    /// `REG_PID_CTRL`: the PID enables and the position-tracking mode.
    PidCtrl {
        cutout_en: bool,
        hyst_en: bool,
        pid_en: bool,
        ctrl_pol: bool,
        ext_ctrl: bool,
        pos_track: i32,
    },
    /// `TXC_CH{A..D}_CALIB_SLOPE`: the channel's calibration slope, a float in
    /// the register's bit pattern.
    CalSlope {
        channel: usize,
        value: f64,
    },
    /// `TXC_CH{A..D}_CALIB_OFFSET`.
    CalOffset {
        channel: usize,
        value: f64,
    },
}

/// C++ `processRegVal`: `None` is C++'s `return -1` (an unhandled register).
pub fn decode_reg(reg_num: i32, reg_val: u32) -> Option<RegUpdate> {
    if let Some(index) = find_reg_by_num(reg_num) {
        return Some(RegUpdate::Pid {
            index,
            value: scale_reg_to_param(reg_val, &PID_REGS[index]),
        });
    }
    match reg_num {
        REG_T4U_CTRL => Some(RegUpdate::Ctrl {
            bias_n: reg_val & BIAS_N_EN_MASK != 0,
            bias_p: reg_val & BIAS_P_EN_MASK != 0,
            pulse_bias: reg_val & PULSE_BIAS_EN_MASK != 0,
        }),
        REG_T4U_FREQ => Some(RegUpdate::SampleFreq(reg_val)),
        REG_T4U_RANGE => Some(RegUpdate::Range((reg_val & RANGE_SEL_MASK) as i32)),
        PULSE_BIAS_OFF_REG => Some(RegUpdate::PulseBiasOff(reg_val as i32)),
        PULSE_BIAS_ON_REG => Some(RegUpdate::PulseBiasOn(reg_val as i32)),
        REG_OUTPUT_MODE => Some(RegUpdate::DacMode((reg_val & OUTPUT_MODE_MASK) as i32)),
        REG_PID_CTRL => Some(RegUpdate::PidCtrl {
            cutout_en: reg_val & PID_CUTOUT_EN_MASK != 0,
            hyst_en: reg_val & PID_HYST_REENABLE_MASK != 0,
            pid_en: reg_val & PID_EN_MASK != 0,
            ctrl_pol: reg_val & PID_CTRL_POL_MASK != 0,
            ext_ctrl: reg_val & PID_EXT_CTRL_MASK != 0,
            pos_track: ((reg_val >> PID_POS_TRACK_SHIFT) & PID_POS_TRACK_MASK) as i32,
        }),
        n if (TXC_CALIB_SLOPE_BASE..TXC_CALIB_SLOPE_BASE + 4).contains(&n) => {
            Some(RegUpdate::CalSlope {
                channel: (n - TXC_CALIB_SLOPE_BASE) as usize,
                value: f32::from_bits(reg_val) as f64,
            })
        }
        n if (TXC_CALIB_OFFSET_BASE..TXC_CALIB_OFFSET_BASE + 4).contains(&n) => {
            Some(RegUpdate::CalOffset {
                channel: (n - TXC_CALIB_OFFSET_BASE) as usize,
                value: f32::from_bits(reg_val) as f64,
            })
        }
        _ => None,
    }
}

/// C++ `cmdReadThread`'s `kGOT_FULL_TR` branch: the `tr` payload is a run of
/// six-byte records, a little-endian register number followed by a
/// little-endian value. A trailing partial record is dropped, as in C++
/// (`tr_len / 6`).
pub fn parse_reg_records(payload: &[u8]) -> Vec<(i32, u32)> {
    payload
        .chunks_exact(6)
        .map(|c| {
            let num = u16::from_le_bytes([c[0], c[1]]) as i32;
            let val = u32::from_le_bytes([c[2], c[3], c[4], c[5]]);
            (num, val)
        })
        .collect()
}

/// C++ `parseCmdName`: the device's replies start with a two-character verb.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CmdName {
    /// `tr`: a binary register dump follows (the middle-layer driver only —
    /// the direct driver's `parseCmdName` maps `tr` to the ASCII branch).
    Tr,
    /// `wr`, `rr`, `bc`, `bs`: an ASCII line terminated by `\n`.
    Ascii,
    /// Anything else: flush the socket.
    Unknown,
}

pub fn parse_cmd_name(name: &str) -> CmdName {
    match name {
        "tr" => CmdName::Tr,
        "wr" | "rr" | "bc" | "bs" => CmdName::Ascii,
        _ => CmdName::Unknown,
    }
}

// ===========================================================================
// Data stream
// ===========================================================================

/// C++ `readTextCurrVals`: `sscanf(InData, "ead %lf , %lf , %lf , %lf ")`,
/// where the leading `r` was consumed by the caller.
///
/// Takes the whole line, `r` included.
pub fn parse_read_line(line: &str) -> Option<[f64; QE_MAX_INPUTS]> {
    let rest = line.trim_start().strip_prefix("read")?;
    let mut out = [0.0f64; QE_MAX_INPUTS];
    let mut fields = rest.split(',');
    for slot in out.iter_mut() {
        *slot = fields.next()?.trim().parse::<f64>().ok()?;
    }
    Some(out)
}

/// The header of the middle-layer driver's binary data frame: `B\x01`, then
/// the units flag and the payload length, both big-endian (C++ `readDataParam`
/// runs them through `ntohs`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BroadcastHeader {
    /// 0 = raw ADC counts, non-zero = microamps as IEEE floats.
    pub units: u16,
    pub payload_len: usize,
}

impl BroadcastHeader {
    /// C++ `bc_hdr_.num_reads = payload_len/4/4`.
    pub fn num_reads(&self) -> usize {
        self.payload_len / 4 / 4
    }
}

/// Parse the four header bytes that follow `B\x01`.
pub fn parse_broadcast_header(bytes: &[u8]) -> Option<BroadcastHeader> {
    if bytes.len() < 4 {
        return None;
    }
    Some(BroadcastHeader {
        units: u16::from_be_bytes([bytes[0], bytes[1]]),
        payload_len: u16::from_be_bytes([bytes[2], bytes[3]]) as usize,
    })
}

/// The direct driver's UDP frame metadata (C++ `T4UMetadata`, `#pragma
/// pack(1)`, little-endian on the wire).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct T4uMetadata {
    pub timestamp: u64,
    pub frame_number: u32,
    pub status: u32,
    pub gain: u16,
    pub over_samples: u16,
    pub adc_gain: u16,
    pub decimation: u16,
    /// 0 = raw counts, 1 = microamps.
    pub units: u16,
    pub number_of_reads: u32,
}

/// Wire size of [`T4uMetadata`] (C++ `sizeof(T4UMetadata)` under `pack(1)`).
pub const METADATA_LEN: usize = 30;
/// Wire size of C++ `T4UErrors` (`int16_t data[16]`).
pub const ERRORS_LEN: usize = 32;

impl T4uMetadata {
    pub fn parse(b: &[u8]) -> Option<Self> {
        if b.len() < METADATA_LEN {
            return None;
        }
        let u16at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let u32at = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        Some(Self {
            timestamp: u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]),
            frame_number: u32at(8),
            status: u32at(12),
            gain: u16at(16),
            over_samples: u16at(18),
            adc_gain: u16at(20),
            decimation: u16at(22),
            units: u16at(24),
            number_of_reads: u32at(26),
        })
    }
}

/// One frame off the direct driver's UDP data socket.
#[derive(Debug, Clone, PartialEq)]
pub enum DirectFrame<'a> {
    /// `B\x01`: a data frame. `image` is `number_of_reads * 4` little-endian
    /// 32-bit words.
    Data {
        metadata: T4uMetadata,
        image: &'a [u8],
    },
    /// `B\x03`: a register dump.
    Registers(Vec<(i32, u32)>),
}

/// Why a UDP frame was rejected. C++ funnels all of these into `bad_type` /
/// `Failed checksum`, which flushes the socket and drops the frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FrameError {
    /// The frame does not start with `B\x01` or `B\x03`.
    BadType,
    /// The datagram ended before the frame did.
    Truncated,
    /// The trailing checksum byte is not `'*'`.
    BadChecksum,
}

/// Parse one datagram from the direct driver's UDP data socket.
///
/// C++ reads the frame back off the socket in five separate `read()` calls
/// (`drvT4UDirect_EM.cpp:1107-1176`). On a datagram socket each `read` is one
/// `recvfrom`, which discards whatever of the datagram did not fit in the
/// buffer it was given — so every read after the two-byte header lands on the
/// *next* packet, and the frame is reassembled out of five different packets.
/// This port reads the datagram whole and parses it out of the buffer, which is
/// what the unused `DataBuffer_T` / `readDataBuf` pair in the C++ was written
/// for.
pub fn parse_direct_frame(buf: &[u8]) -> Result<DirectFrame<'_>, FrameError> {
    if buf.len() < 2 || buf[0] != b'B' {
        return Err(FrameError::BadType);
    }
    match buf[1] {
        1 => {
            // u16 packet length, metadata, errors, image, checksum.
            if buf.len() < 4 + METADATA_LEN + ERRORS_LEN {
                return Err(FrameError::Truncated);
            }
            let metadata =
                T4uMetadata::parse(&buf[4..4 + METADATA_LEN]).ok_or(FrameError::Truncated)?;
            // C++ clamps numberOfReads to kT4U_MAX_DATA_SIZE before reading.
            let num_reads = (metadata.number_of_reads as usize).min(MAX_DATA_SIZE);
            let start = 4 + METADATA_LEN + ERRORS_LEN;
            let end = start + num_reads * 4 * 4;
            if buf.len() < end + 1 {
                return Err(FrameError::Truncated);
            }
            if buf[end] != b'*' {
                return Err(FrameError::BadChecksum);
            }
            let mut metadata = metadata;
            metadata.number_of_reads = num_reads as u32;
            Ok(DirectFrame::Data {
                metadata,
                image: &buf[start..end],
            })
        }
        3 => {
            if buf.len() < 4 {
                return Err(FrameError::Truncated);
            }
            let len = u16::from_le_bytes([buf[2], buf[3]]) as usize;
            let end = 4 + len;
            if buf.len() < end {
                return Err(FrameError::Truncated);
            }
            Ok(DirectFrame::Registers(parse_reg_records(&buf[4..end])))
        }
        _ => Err(FrameError::BadType),
    }
}

/// The per-channel calibration the data path applies to raw counts (C++
/// `calSlope_` / `calOffset_`, as read back from registers 100-107).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelCal {
    pub slope: [f64; QE_MAX_INPUTS],
    pub offset: [f64; QE_MAX_INPUTS],
}

impl Default for ChannelCal {
    fn default() -> Self {
        Self {
            slope: [1.0; QE_MAX_INPUTS],
            offset: [0.0; QE_MAX_INPUTS],
        }
    }
}

/// C++ `dataReadThread`'s binary branch: one sample per four 32-bit words.
///
/// `units != 0` means the device already converted to microamps and sends IEEE
/// floats; otherwise the words are signed ADC counts and go through
/// [`raw_to_current`] and the channel calibration.
pub fn decode_samples(
    image: &[u8],
    units: u16,
    range: i32,
    cal: &ChannelCal,
) -> Vec<[f64; QE_MAX_INPUTS]> {
    image
        .chunks_exact(4 * QE_MAX_INPUTS)
        .map(|sample| {
            let mut out = [0.0f64; QE_MAX_INPUTS];
            for (ch, slot) in out.iter_mut().enumerate() {
                let word = [
                    sample[ch * 4],
                    sample[ch * 4 + 1],
                    sample[ch * 4 + 2],
                    sample[ch * 4 + 3],
                ];
                *slot = if units != 0 {
                    f32::from_bits(u32::from_le_bytes(word)) as f64
                } else {
                    (raw_to_current(i32::from_le_bytes(word), range) - cal.offset[ch])
                        / cal.slope[ch]
                };
            }
            out
        })
        .collect()
}

// ===========================================================================
// Calibration file (direct driver)
// ===========================================================================

/// One mode's calibration: a slope and an offset per range and channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalTable {
    pub slope: [[f32; QE_MAX_INPUTS]; NUM_RANGES],
    pub offset: [[f32; QE_MAX_INPUTS]; NUM_RANGES],
}

impl Default for CalTable {
    fn default() -> Self {
        Self {
            slope: [[f32::NAN; QE_MAX_INPUTS]; NUM_RANGES],
            offset: [[f32::NAN; QE_MAX_INPUTS]; NUM_RANGES],
        }
    }
}

/// C++ `parseConfigFile`'s result: the continuous-wave table and the pulsed
/// table, which the wait-state mode selects between.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Calibration {
    pub cw: CalTable,
    pub pulsed: CalTable,
}

/// C++ `drvT4UDirect_EM::parseConfigFile` with `inicpp`.
///
/// The file names a `selected` calibration set (and optionally a separate
/// `pulsed` one) in its `[config]` section; each set has one section per range
/// (`<name>_range<N>`) holding `ChannelA`..`ChannelD` = `"<slope>, <offset>"`.
/// Every range and channel of both sets must be present — C++ aborts the IOC
/// otherwise, and so does the caller here.
pub fn parse_calibration(text: &str) -> Result<Calibration, String> {
    let ini = parse_ini(text);
    let get = |section: &str, key: &str| -> Option<&String> {
        ini.iter()
            .find(|(s, _, _)| s == section)
            .map(|_| ())
            .and(ini.iter().find(|(s, k, _)| s == section && k == key))
            .map(|(_, _, v)| v)
    };

    let cw_name = get("config", "selected")
        .ok_or_else(|| {
            "No selected calibration discovered. Missing key selected=<name> in section [config]."
                .to_string()
        })?
        .clone();
    // A T4U without pulsed mode has no separate pulsed set; C++ falls back to
    // the CW one.
    let pulsed_name = get("config", "pulsed").cloned().unwrap_or_else(|| {
        log::info!(
            "drvT4UDirect_EM: no pulsed-mode calibration selected; using the CW set \"{cw_name}\""
        );
        cw_name.clone()
    });

    let load = |name: &str| -> Result<CalTable, String> {
        let mut table = CalTable::default();
        for range in 0..NUM_RANGES {
            let section = format!("{name}_range{range}");
            for channel in 0..QE_MAX_INPUTS {
                let key = format!("Channel{}", (b'A' + channel as u8) as char);
                let value = get(&section, &key).ok_or_else(|| {
                    format!(
                        "Calibration {name} entry for range {range} channel {} not found.",
                        (b'A' + channel as u8) as char
                    )
                })?;
                let (slope, offset) = parse_cal_pair(value)
                    .ok_or_else(|| format!("Invalid value string: {value}"))?;
                table.slope[range][channel] = slope;
                table.offset[range][channel] = offset;
            }
        }
        Ok(table)
    };

    Ok(Calibration {
        cw: load(&cw_name)?,
        pulsed: load(&pulsed_name)?,
    })
}

/// C++ `sscanf(curr_val.c_str(), " \" %f , %f \" ")`: the quotes are optional.
fn parse_cal_pair(value: &str) -> Option<(f32, f32)> {
    let body = value.trim().trim_matches('"');
    let (slope, offset) = body.split_once(',')?;
    Some((
        slope.trim().parse::<f32>().ok()?,
        offset.trim().parse::<f32>().ok()?,
    ))
}

/// A minimal INI reader: `[section]` headers, `key = value` pairs, `#`/`;`
/// comments. Enough for the calibration file `inicpp` reads upstream.
fn parse_ini(text: &str) -> Vec<(String, String, String)> {
    let mut out = Vec::new();
    let mut section = String::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with(';') {
            continue;
        }
        if let Some(name) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            section = name.trim().to_string();
            continue;
        }
        if let Some((key, value)) = line.split_once('=') {
            out.push((
                section.clone(),
                key.trim().to_string(),
                value.trim().to_string(),
            ));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pid_table_matches_the_upstream_parameter_list() {
        assert_eq!(PID_REGS.len(), 19);
        assert_eq!(find_reg_by_num(50), Some(0));
        assert_eq!(PID_REGS[0].param, "QE_PIDX_SP");
        assert_eq!(find_reg_by_num(20), Some(18));
        assert_eq!(PID_REGS[18].param, "QE_POS_TRACK_RAD");
        // 55, 58, 59 are gaps in the upstream list.
        assert_eq!(find_reg_by_num(55), None);
    }

    #[test]
    fn param_scales_linearly_into_the_register_range() {
        let r = PID_REGS[0]; // PIDX_Sp: PV -1..1 -> reg -10000..10000
        assert_eq!(scale_param_to_reg(0.0, &r, false), 0.0);
        assert_eq!(scale_param_to_reg(1.0, &r, false), 10000.0);
        assert_eq!(scale_param_to_reg(-1.0, &r, false), -10000.0);
        // Unclipped by default, as every C++ caller leaves it.
        assert_eq!(scale_param_to_reg(2.0, &r, false), 20000.0);
        assert_eq!(scale_param_to_reg(2.0, &r, true), 10000.0);
    }

    #[test]
    fn register_scales_back_into_the_pv_range() {
        let r = PID_REGS[1]; // PIDX_Kp: PV 0..1 -> reg 0..10000
        assert_eq!(scale_reg_to_param(0, &r), 0.0);
        assert_eq!(scale_reg_to_param(10000, &r), 1.0);
        assert_eq!(scale_reg_to_param(5000, &r), 0.5);
    }

    #[test]
    fn commands_are_the_upstream_strings() {
        assert_eq!(cmd_write(REG_T4U_FREQ, 1000), "wr 1 1000");
        assert_eq!(cmd_bits(true, 0, 0x200), "bs 0 512");
        assert_eq!(cmd_bits(false, 0, 0x400), "bc 0 1024");
        assert_eq!(cmd_bits(true, 0, PULSE_BIAS_EN_MASK), "bs 0 262144");
        assert_eq!(cmd_read_regs(100, 107), "tr 100 107");
        // The bias enables keep the upstream hex literal.
        assert_eq!(cmd_bias_enable(true, false), "bs 0 0x200");
        assert_eq!(cmd_bias_enable(false, true), "bc 0 0x400");
    }

    #[test]
    fn raw_counts_convert_through_the_range_transimpedance() {
        // Range 2: 47 ohms; half scale at 1.5 V reference.
        assert_eq!(raw_to_current(524288, 2), 1.5 / 47.0);
        assert_eq!(raw_to_current(-524288, 2), -1.5 / 47.0);
        assert_eq!(raw_to_current(0, 0), 0.0);
        assert_eq!(raw_to_current(524288, 0), 1.5 / 5e6);
    }

    #[test]
    fn ctrl_register_decodes_the_bias_enables() {
        assert_eq!(
            decode_reg(REG_T4U_CTRL, BIAS_N_EN_MASK | PULSE_BIAS_EN_MASK),
            Some(RegUpdate::Ctrl {
                bias_n: true,
                bias_p: false,
                pulse_bias: true,
            })
        );
    }

    #[test]
    fn pid_ctrl_register_decodes_every_field() {
        let val = PID_EN_MASK | PID_CTRL_POL_MASK | (0x2 << PID_POS_TRACK_SHIFT);
        assert_eq!(
            decode_reg(REG_PID_CTRL, val),
            Some(RegUpdate::PidCtrl {
                cutout_en: false,
                hyst_en: false,
                pid_en: true,
                ctrl_pol: true,
                ext_ctrl: false,
                pos_track: 2,
            })
        );
    }

    #[test]
    fn calibration_registers_carry_float_bit_patterns() {
        let bits = 1.25f32.to_bits();
        assert_eq!(
            decode_reg(101, bits),
            Some(RegUpdate::CalSlope {
                channel: 1,
                value: 1.25
            })
        );
        assert_eq!(
            decode_reg(107, (-0.5f32).to_bits()),
            Some(RegUpdate::CalOffset {
                channel: 3,
                value: -0.5
            })
        );
    }

    #[test]
    fn range_and_frequency_registers_decode() {
        assert_eq!(decode_reg(REG_T4U_RANGE, 0x83), Some(RegUpdate::Range(3)));
        assert_eq!(
            decode_reg(REG_T4U_FREQ, 10000),
            Some(RegUpdate::SampleFreq(10000))
        );
        // An unhandled register is C++'s "return -1".
        assert_eq!(decode_reg(999, 0), None);
    }

    #[test]
    fn tr_payload_is_little_endian_six_byte_records() {
        // reg 3 = 2, reg 1 = 10000.
        let payload = [
            0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x27, 0x00, 0x00,
        ];
        assert_eq!(parse_reg_records(&payload), vec![(3, 2), (1, 10000)]);
        // A trailing partial record is dropped, as tr_len/6 does upstream.
        let mut short = payload.to_vec();
        short.push(0xff);
        assert_eq!(parse_reg_records(&short).len(), 2);
    }

    #[test]
    fn command_names_route_the_parser() {
        assert_eq!(parse_cmd_name("tr"), CmdName::Tr);
        assert_eq!(parse_cmd_name("wr"), CmdName::Ascii);
        assert_eq!(parse_cmd_name("bs"), CmdName::Ascii);
        assert_eq!(parse_cmd_name("zz"), CmdName::Unknown);
    }

    #[test]
    fn text_data_line_parses_four_currents() {
        assert_eq!(
            parse_read_line("read 1.0,-2.5, 3e-9 ,4\n"),
            Some([1.0, -2.5, 3e-9, 4.0])
        );
        assert_eq!(parse_read_line("read 1.0,2.0"), None);
        assert_eq!(parse_read_line("B\x01"), None);
    }

    #[test]
    fn broadcast_header_is_big_endian() {
        let hdr = parse_broadcast_header(&[0x00, 0x01, 0x00, 0x20]).unwrap();
        assert_eq!(hdr.units, 1);
        assert_eq!(hdr.payload_len, 32);
        assert_eq!(hdr.num_reads(), 2);
        assert_eq!(parse_broadcast_header(&[0, 1]), None);
    }

    #[test]
    fn samples_decode_as_floats_when_the_device_reports_microamps() {
        let mut image = Vec::new();
        for v in [1.0f32, 2.0, 3.0, 4.0] {
            image.extend_from_slice(&v.to_bits().to_le_bytes());
        }
        let out = decode_samples(&image, 1, 2, &ChannelCal::default());
        assert_eq!(out, vec![[1.0, 2.0, 3.0, 4.0]]);
    }

    #[test]
    fn samples_decode_as_counts_through_calibration_when_units_are_raw() {
        let mut image = Vec::new();
        for v in [524288i32, -524288, 0, 524288] {
            image.extend_from_slice(&v.to_le_bytes());
        }
        let cal = ChannelCal {
            slope: [2.0, 1.0, 1.0, 1.0],
            offset: [0.0, 0.0, 0.5, 0.0],
        };
        let out = decode_samples(&image, 0, 2, &cal);
        let full = 1.5 / 47.0;
        assert_eq!(out[0][0], full / 2.0);
        assert_eq!(out[0][1], -full);
        assert_eq!(out[0][2], -0.5);
        assert_eq!(out[0][3], full);
    }

    #[test]
    fn samples_decode_one_entry_per_four_words() {
        let image = vec![0u8; 32];
        assert_eq!(
            decode_samples(&image, 1, 0, &ChannelCal::default()).len(),
            2
        );
        // A trailing partial sample is dropped.
        let image = vec![0u8; 36];
        assert_eq!(
            decode_samples(&image, 1, 0, &ChannelCal::default()).len(),
            2
        );
    }

    fn direct_data_frame(num_reads: u32, units: u16, checksum: u8) -> Vec<u8> {
        let mut f = vec![b'B', 1, 0, 0];
        f.extend_from_slice(&0u64.to_le_bytes()); // timestamp
        f.extend_from_slice(&7u32.to_le_bytes()); // frame number
        f.extend_from_slice(&0u32.to_le_bytes()); // status
        f.extend_from_slice(&0u16.to_le_bytes()); // gain
        f.extend_from_slice(&0u16.to_le_bytes()); // over samples
        f.extend_from_slice(&0u16.to_le_bytes()); // adc gain
        f.extend_from_slice(&0u16.to_le_bytes()); // decimation
        f.extend_from_slice(&units.to_le_bytes());
        f.extend_from_slice(&num_reads.to_le_bytes());
        f.extend_from_slice(&[0u8; ERRORS_LEN]);
        f.extend(std::iter::repeat_n(0u8, num_reads as usize * 16));
        f.push(checksum);
        f
    }

    #[test]
    fn direct_data_frame_parses_metadata_and_image() {
        let frame = direct_data_frame(2, 1, b'*');
        match parse_direct_frame(&frame).unwrap() {
            DirectFrame::Data { metadata, image } => {
                assert_eq!(metadata.frame_number, 7);
                assert_eq!(metadata.units, 1);
                assert_eq!(metadata.number_of_reads, 2);
                assert_eq!(image.len(), 32);
            }
            other => panic!("expected a data frame, got {other:?}"),
        }
    }

    #[test]
    fn direct_frame_rejects_a_bad_checksum_a_bad_type_and_a_short_datagram() {
        assert_eq!(
            parse_direct_frame(&direct_data_frame(1, 0, b'x')),
            Err(FrameError::BadChecksum)
        );
        assert_eq!(parse_direct_frame(b"C\x01"), Err(FrameError::BadType));
        assert_eq!(parse_direct_frame(b"B\x02ab"), Err(FrameError::BadType));
        let frame = direct_data_frame(2, 1, b'*');
        assert_eq!(
            parse_direct_frame(&frame[..frame.len() - 4]),
            Err(FrameError::Truncated)
        );
    }

    #[test]
    fn direct_frame_clamps_an_oversized_read_count() {
        // numberOfReads beyond kT4U_MAX_DATA_SIZE would index past the image.
        let mut frame = direct_data_frame(MAX_DATA_SIZE as u32, 1, b'*');
        let off = 4 + METADATA_LEN - 4;
        frame[off..off + 4].copy_from_slice(&(MAX_DATA_SIZE as u32 + 100).to_le_bytes());
        match parse_direct_frame(&frame).unwrap() {
            DirectFrame::Data { metadata, image } => {
                assert_eq!(metadata.number_of_reads, MAX_DATA_SIZE as u32);
                assert_eq!(image.len(), MAX_DATA_SIZE * 16);
            }
            other => panic!("expected a data frame, got {other:?}"),
        }
    }

    #[test]
    fn direct_register_frame_parses() {
        let mut frame = vec![b'B', 3];
        frame.extend_from_slice(&6u16.to_le_bytes());
        frame.extend_from_slice(&[0x03, 0x00, 0x02, 0x00, 0x00, 0x00]);
        assert_eq!(
            parse_direct_frame(&frame),
            Ok(DirectFrame::Registers(vec![(3, 2)]))
        );
    }

    const CAL_FILE: &str = "\
[config]
selected = default
pulsed = pulse1

[default_range0]
ChannelA = \"1.0, 0.1\"
ChannelB = \"1.1, 0.2\"
ChannelC = \"1.2, 0.3\"
ChannelD = \"1.3, 0.4\"
[default_range1]
ChannelA = \"2.0, 0.1\"
ChannelB = \"2.1, 0.2\"
ChannelC = \"2.2, 0.3\"
ChannelD = \"2.3, 0.4\"
[default_range2]
ChannelA = \"3.0, 0.1\"
ChannelB = \"3.1, 0.2\"
ChannelC = \"3.2, 0.3\"
ChannelD = \"3.3, 0.4\"

[pulse1_range0]
ChannelA = \"9.0, 0.9\"
ChannelB = \"9.1, 0.9\"
ChannelC = \"9.2, 0.9\"
ChannelD = \"9.3, 0.9\"
[pulse1_range1]
ChannelA = \"9.0, 0.9\"
ChannelB = \"9.1, 0.9\"
ChannelC = \"9.2, 0.9\"
ChannelD = \"9.3, 0.9\"
[pulse1_range2]
ChannelA = \"9.0, 0.9\"
ChannelB = \"9.1, 0.9\"
ChannelC = \"9.2, 0.9\"
ChannelD = \"9.3, 0.9\"
";

    #[test]
    fn calibration_file_loads_both_modes() {
        let cal = parse_calibration(CAL_FILE).unwrap();
        assert_eq!(cal.cw.slope[0][0], 1.0);
        assert_eq!(cal.cw.offset[0][3], 0.4);
        assert_eq!(cal.cw.slope[2][3], 3.3);
        assert_eq!(cal.pulsed.slope[1][2], 9.2);
    }

    #[test]
    fn calibration_file_without_a_pulsed_set_falls_back_to_cw() {
        let text = CAL_FILE.replace("pulsed = pulse1\n", "");
        let cal = parse_calibration(&text).unwrap();
        assert_eq!(cal.pulsed, cal.cw);
    }

    #[test]
    fn calibration_file_missing_a_channel_is_an_error() {
        let text = CAL_FILE.replace("ChannelC = \"1.2, 0.3\"\n", "");
        let err = parse_calibration(&text).unwrap_err();
        assert!(err.contains("channel C"), "{err}");
        let text = CAL_FILE.replace("selected = default\n", "");
        assert!(parse_calibration(&text).is_err());
    }

    /// `iocBoot/iocT4UDirect_EM/DBPM_Settings.ini`, verbatim.
    const UPSTREAM_CAL_FILE: &str = "\
[config]
selected=direct

[direct_range0]
ChannelA=\"0.8823024931401017, 1.0223252432686801e-10\"
ChannelB=\"0.8822969078219268, 1.1540957859584427e-10\"
ChannelC=\"0.8825603068244604, 1.4080320085798082e-10\"
ChannelD=\"0.8822436043497636, 1.2449537004743768e-10\"

[direct_range1]
ChannelA=\"0.8808044564388182, 3.715184399856713e-08\"
ChannelB=\"0.8808623492298324, 4.181399867542052e-08\"
ChannelC=\"0.8810291910888772, 5.01167279727309e-08\"
ChannelD=\"0.8809629509769801, 4.477734606747846e-08\"

[direct_range2]
ChannelA=\"0.952110410518152, 1.1741499045699521e-05\"
ChannelB=\"0.9524212459605307, 1.331462686185057e-05\"
ChannelC=\"0.9521789712575454, 1.5998248995980937e-05\"
ChannelD=\"0.9527919082805758, 1.4346421308430505e-05\"
";

    #[test]
    fn the_upstream_calibration_file_loads() {
        // The table is f32, as C++'s is; the file's digits are compared through
        // the same narrowing rather than as f32 literals.
        let f32_of = |s: &str| s.parse::<f32>().unwrap();
        let cal = parse_calibration(UPSTREAM_CAL_FILE).unwrap();
        assert_eq!(cal.cw.slope[0][0], f32_of("0.8823024931401017"));
        assert_eq!(cal.cw.offset[0][0], f32_of("1.0223252432686801e-10"));
        assert_eq!(cal.cw.slope[2][3], f32_of("0.9527919082805758"));
        assert_eq!(cal.cw.offset[2][3], f32_of("1.4346421308430505e-05"));
        // The file has no pulsed set, so the CW table is used for both.
        assert_eq!(cal.pulsed, cal.cw);
    }

    #[test]
    fn calibration_values_may_be_unquoted() {
        let text = CAL_FILE.replace("ChannelA = \"1.0, 0.1\"", "ChannelA = 1.0,0.1");
        let cal = parse_calibration(&text).unwrap();
        assert_eq!(cal.cw.slope[0][0], 1.0);
        assert_eq!(cal.cw.offset[0][0], 0.1);
    }
}
