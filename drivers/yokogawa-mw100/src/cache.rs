//! In-memory cache mirroring `struct devqueue` (`drvMW100.c:398-440`). All
//! wire-facing reads land here; every `DeviceSupport::read()` for a
//! cache-only command is a plain lookup, no wire I/O (mirrors
//! `devMW100_mbbi.c`'s own "not really needed, just run everything, no
//! async here").

use crate::codec::ErrorEntry;

pub const MAX_MODULES: usize = 6;
pub const MAX_SIGNAL: usize = 60;
pub const MAX_MATH: usize = 300;
pub const MAX_COMM: usize = 300;
pub const MAX_CONST: usize = 60;

/// `CH_TYPE_*` (`drvMW100.c:60-62`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ChannelType {
    #[default]
    None,
    InputAnalog,
    InputBinary,
    InputInteger,
    OutputAnalog,
    OutputBinary,
    Unknown,
}

/// One module slot (`struct module`, `drvMW100.c:36-58`). `set_message`/
/// `status_message`/`error_message` are copied byte-for-byte from the wire
/// with no space-trimming (unlike GM10's 17-byte space-stopped fields) —
/// `drvMW100.c:587-601`'s copy loops run a fixed 13 iterations regardless of
/// content.
#[derive(Debug, Clone, Default)]
pub struct Module {
    pub set_message: String,
    pub status_message: String,
    pub error_message: String,
    pub use_flag: bool,
    pub module_string: String,
    /// 110/112/114/115/120/125, or 0 if empty (`drvMW100.c:53`).
    pub model: i32,
    /// 3-char code field (`drvMW100.c:627-631`).
    pub code: String,
    /// 0=Low, 1=Medium, 2=High, -1=unrecognized (`drvMW100.c:633-646`).
    pub speed: i32,
    pub number: i32,
}

/// `CH_STATUS_*` (`drvMW100.c:66`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ChStatus {
    #[default]
    Skip,
    Normal,
    Diff,
    Unknown,
}

#[derive(Debug, Clone, Default)]
pub struct ChannelInfo {
    pub ch_status: ChStatus,
    /// Relay (`CH_MODE_RELAY_*`, 0-6) or DAC (`CH_MODE_DAC_*`, 0-3) mode,
    /// meaning depends on the channel's `ChannelType` (`drvMW100.c:67-71`).
    pub ch_mode: i32,
    pub unit: String,
    /// 0-4, indexes `scaled_value`'s `scaler` table.
    pub scale: u8,
}

/// `VL_*` (`drvMW100.c:84-85`): only 6 variants, detected via magic 32-bit
/// sentinel values (`input_value_flag`, `drvMW100.c:1008-1034`) — a
/// fundamentally different mechanism from GM10's status-byte bitmask.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataStatus {
    #[default]
    Normal,
    Overrange,
    Underrange,
    SkipOff,
    Error,
    Uncertain,
}

impl DataStatus {
    /// `input_value_flag` (`drvMW100.c:1008-1034`): any raw value below the
    /// sentinel range is normal; only 5 exact sentinel values are
    /// recognized as special, and any other large value falls through to
    /// `Normal` ("nothing should get here", but the fallthrough is real
    /// wire-observable behavior and must not be "corrected").
    pub fn from_wire(value: u32) -> Self {
        if value < 0x7fff_7fff {
            return Self::Normal;
        }
        match value {
            0x7fff_7fff => Self::Overrange,
            0x8001_8001 => Self::Underrange,
            0x8002_8002 => Self::SkipOff,
            0x8004_8004 => Self::Error,
            0x8005_8005 => Self::Uncertain,
            _ => Self::Normal,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ChannelData {
    pub alarm_status: u8,
    /// Raw, un-shifted nibble values (`drvMW100.c:1110-1113`):
    /// `alarm[0]=alarms1&0xF`, `alarm[1]=alarms1&0xF0` (upper nibble, NOT
    /// shifted down), `alarm[2]=alarms2&0xF`, `alarm[3]=alarms2&0xF0`. This
    /// asymmetry (odd slots hold un-shifted multiples of 16) is the C
    /// driver's own on-wire-derived behavior, not a bug — preserved
    /// verbatim since it is what `ALARM.2`/`ALARM.4` mbbi records have
    /// always reported.
    pub alarm: [u8; 4],
    pub data_status: DataStatus,
    pub value: i32,
}

/// `scaled_value`/`unscaled_value` (`drvMW100.c:110-123`): scale is 0-4, a
/// narrower range than GM10's 0-6.
pub fn scaled_value(value: i32, scale: u8) -> f64 {
    const SCALER: [f64; 5] = [1.0, 0.1, 0.01, 0.001, 0.0001];
    f64::from(value) * SCALER[(scale as usize).min(4)]
}

pub fn unscaled_value(value: f64, scale: u8) -> i32 {
    const SCALER: [f64; 5] = [1.0, 10.0, 100.0, 1000.0, 10000.0];
    (value * SCALER[(scale as usize).min(4)]) as i32
}

#[derive(Debug, Clone, Default)]
pub struct ExprInfo {
    pub on_flag: bool,
    /// A single `String`, unlike the C driver's split `calc_expr[60]`
    /// (121-byte buffer, channels 1-60) / `short_calc_expr[240]` (11-byte
    /// buffer, channels 61-300) — see [`crate::codec`] module docs for why
    /// that split is a genuine upstream buffer-overflow defect this port
    /// structurally eliminates rather than reproduces.
    pub expr: String,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ErrorState {
    pub flag: bool,
    /// `None` when the wire's `E1 <code> ...` error code has no match in
    /// the local `errors[]` table (`drvMW100.c:546-547`: `dq->error` stays
    /// `NULL`) — every real `ERROR` stringin call passes channel 1-3, so
    /// the C source's `channel == 0` "Unknown error." fallback
    /// (`drvMW100.c:2035-2036`) is dead code, not reproduced here.
    pub entry: Option<&'static ErrorEntry>,
}

pub struct Cache {
    pub ch_type: Vec<ChannelType>,
    pub ch_info: Vec<ChannelInfo>,
    pub ch_data: Vec<ChannelData>,

    pub calc_info: Vec<ChannelInfo>,
    pub calc_data: Vec<ChannelData>,
    pub calc_expr: Vec<ExprInfo>,

    pub comm_input: Vec<f64>,
    pub constant: Vec<f64>,

    pub modules: Vec<Module>,

    /// Only `status[4]`'s bits 0-2 are used (`drvMW100.c:696-723`).
    pub settings_mode: bool,
    pub measurement_mode: bool,
    pub compute_mode: bool,

    pub alarm_flag: bool,

    pub error: ErrorState,
}

impl Default for Cache {
    fn default() -> Self {
        Self {
            ch_type: vec![ChannelType::default(); MAX_SIGNAL],
            ch_info: vec![ChannelInfo::default(); MAX_SIGNAL],
            ch_data: vec![ChannelData::default(); MAX_SIGNAL],
            calc_info: vec![ChannelInfo::default(); MAX_MATH],
            calc_data: vec![ChannelData::default(); MAX_MATH],
            calc_expr: vec![ExprInfo::default(); MAX_MATH],
            comm_input: vec![0.0; MAX_COMM],
            constant: vec![0.0; MAX_CONST],
            modules: vec![Module::default(); MAX_MODULES],
            settings_mode: false,
            measurement_mode: false,
            compute_mode: false,
            alarm_flag: false,
            error: ErrorState::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scaled_value_matches_c_table() {
        assert_eq!(scaled_value(12345, 0), 12345.0);
        assert_eq!(scaled_value(12345, 3), 12.345);
        assert_eq!(scaled_value(-500, 2), -5.0);
    }

    #[test]
    fn unscaled_value_matches_c_table() {
        assert_eq!(unscaled_value(1.5, 3), 1500);
        assert_eq!(unscaled_value(-5.0, 2), -500);
    }

    #[test]
    fn data_status_from_wire_sentinels() {
        assert_eq!(DataStatus::from_wire(1234), DataStatus::Normal);
        assert_eq!(DataStatus::from_wire(0x7fff_7fff), DataStatus::Overrange);
        assert_eq!(DataStatus::from_wire(0x8001_8001), DataStatus::Underrange);
        assert_eq!(DataStatus::from_wire(0x8002_8002), DataStatus::SkipOff);
        assert_eq!(DataStatus::from_wire(0x8004_8004), DataStatus::Error);
        assert_eq!(DataStatus::from_wire(0x8005_8005), DataStatus::Uncertain);
        // Unrecognized-but-large falls through to Normal (matches C's own
        // "nothing should get here" fallthrough, preserved verbatim).
        assert_eq!(DataStatus::from_wire(0x8000_0000), DataStatus::Normal);
    }
}
