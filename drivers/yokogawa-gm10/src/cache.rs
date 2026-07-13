//! In-memory cache mirroring `struct devqueue` (`drvGM10.c:197-246`). All
//! wire-facing reads land here; every `DeviceSupport::read()` for a
//! cache-only command is a plain lookup, no wire I/O (`devGM10_mbbi.c:61`:
//! "not really needed, just run everything, no async here").

pub const MAX_MODULES: usize = 10;
pub const MAX_SIGNAL: usize = 999;
pub const MAX_MATH: usize = 200;
pub const MAX_COMM: usize = 500;
pub const MAX_CONST: usize = 100;
pub const MAX_VARCONST: usize = 100;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ModuleType {
    #[default]
    Unknown,
    InputAnalog,
    InputDigital,
    InputPulse,
    OutputAnalog,
    OutputDigital,
    InputOutputDigital,
    Pid,
}

#[derive(Debug, Clone, Default)]
pub struct Module {
    pub module_string: String,
    pub use_flag: bool,
    pub mod_type: ModuleType,
    /// [input channel count, output channel count] (`channel_number[2]`).
    pub channel_number: [i32; 2],
}

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
    /// Relay (`CH_MODE_RELAY_*`) or DAC (`CH_MODE_DAC_*`) mode, meaning
    /// depends on the channel's `ModuleType`; `16` is the shared "unknown"
    /// sentinel both C enums use.
    pub ch_mode: i32,
    pub unit: String,
    /// 0-6, indexes `scaled_value`'s `scaler` table.
    pub scale: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataStatus {
    #[default]
    Normal,
    Skip,
    PosOverrange,
    NegOverrange,
    PosBurnout,
    NegBurnout,
    AdError,
    InvalidData,
    MathNan,
    CommError,
    Unknown,
}

impl DataStatus {
    /// `load_data_values`'s `switch(status & 0x1F)` (`drvGM10.c:1016-1050`).
    pub fn from_wire(status: u8) -> Self {
        match status & 0x1F {
            0 => Self::Normal,
            1 => Self::Skip,
            2 => Self::PosOverrange,
            3 => Self::NegOverrange,
            4 => Self::PosBurnout,
            5 => Self::NegBurnout,
            6 => Self::AdError,
            7 => Self::InvalidData,
            16 => Self::MathNan,
            17 => Self::CommError,
            _ => Self::Unknown,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ChannelData {
    pub alarm_status: u8,
    pub alarm: [u8; 4],
    pub data_status: DataStatus,
    pub value: i32,
}

/// `scaled_value` (`drvGM10.c:83-89`).
pub fn scaled_value(value: i32, scale: u8) -> f64 {
    const SCALER: [f64; 7] = [1.0, 0.1, 0.01, 1.0e-3, 1.0e-4, 1.0e-5, 1.0e-6];
    f64::from(value) * SCALER[scale as usize & 0x7]
}

#[derive(Debug, Clone, Default)]
pub struct ExprInfo {
    pub on_flag: bool,
    pub expr: String,
}

#[derive(Debug, Clone, Default)]
pub struct ErrorState {
    pub code: i32,
    pub parameter: i32,
    pub strings: [String; 3],
}

pub struct Cache {
    pub meas_type: Vec<ChannelType>,
    pub meas_info: Vec<ChannelInfo>,
    pub meas_data: Vec<ChannelData>,

    pub calc_info: Vec<ChannelInfo>,
    pub calc_data: Vec<ChannelData>,
    pub calc_expr: Vec<ExprInfo>,

    pub comm_info: Vec<ChannelInfo>,
    pub comm_data: Vec<ChannelData>,

    pub constant: Vec<f64>,
    pub varconstant: Vec<f64>,

    pub modules: Vec<Module>,

    pub recording_mode: bool,
    pub compute_mode: i32,
    pub settings_mode: bool,
    pub alarm_flag: bool,

    pub error_flag: bool,
    pub error: ErrorState,
}

impl Default for Cache {
    fn default() -> Self {
        Self {
            meas_type: vec![ChannelType::default(); MAX_SIGNAL],
            meas_info: vec![ChannelInfo::default(); MAX_SIGNAL],
            meas_data: vec![ChannelData::default(); MAX_SIGNAL],
            calc_info: vec![ChannelInfo::default(); MAX_MATH],
            calc_data: vec![ChannelData::default(); MAX_MATH],
            calc_expr: vec![ExprInfo::default(); MAX_MATH],
            comm_info: vec![ChannelInfo::default(); MAX_COMM],
            comm_data: vec![ChannelData::default(); MAX_COMM],
            constant: vec![0.0; MAX_CONST],
            varconstant: vec![0.0; MAX_VARCONST],
            modules: vec![Module::default(); MAX_MODULES],
            recording_mode: false,
            compute_mode: 0,
            settings_mode: false,
            alarm_flag: false,
            error_flag: false,
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
    fn data_status_from_wire_masks_top_bits() {
        assert_eq!(DataStatus::from_wire(0), DataStatus::Normal);
        assert_eq!(DataStatus::from_wire(0x20 | 6), DataStatus::AdError);
        assert_eq!(DataStatus::from_wire(9), DataStatus::Unknown);
        assert_eq!(DataStatus::from_wire(16), DataStatus::MathNan);
    }
}
