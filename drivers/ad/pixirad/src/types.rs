//! Constants, enumerations and detector geometry (C `pixirad.cpp` and
//! `pxrd2_interface_misc.h`).

use std::time::Duration;

/// Command/reply buffer to the box (C `MAX_MESSAGE_SIZE`).
pub const MAX_MESSAGE_SIZE: usize = 256;
/// Timeout on a command (C `SERVER_DEFAULT_TIMEOUT`).
pub const SERVER_TIMEOUT: Duration = Duration::from_secs(1);
/// How long the box takes to come back after `SYSTEM_RESET`
/// (C `DETECTOR_RESET_TIME`).
pub const DETECTOR_RESET_TIME: Duration = Duration::from_secs(5);
/// Size of one UDP data packet (C `MAX_UDP_PACKET_LEN`).
pub const MAX_UDP_PACKET_LEN: usize = 1448;
/// Receive buffer asked of the kernel (C `MAX_UDP_DATA_BUFFER`).
pub const MAX_UDP_DATA_BUFFER: usize = 256_217_728;
/// Packets per identifier group (C `DAQ_PACKET_FRAGMENT`).
pub const DAQ_PACKET_FRAGMENT: usize = 45;
/// Bit in the packet tag that says the frame is autocalibration data.
pub const AUTOCAL_DATA: u8 = 0x40;
/// Bit the driver sets in the tag it hands on when packets were misordered.
pub const FRAME_HAS_ALIGN_ERRORS: u8 = 0x20;
pub const PACKET_TAG_BYTES: usize = 2;
pub const PACKET_ID_BYTES: usize = 2;
pub const PACKET_ID_OFFSET: usize = 2;
pub const PACKET_CRC_BYTES: usize = 4;
pub const PACKET_SENSOR_DATA_OFFSET: usize = PACKET_TAG_BYTES + PACKET_ID_BYTES;
pub const PACKET_EXTRA_BYTES: usize = PACKET_ID_BYTES + PACKET_TAG_BYTES + PACKET_CRC_BYTES;
/// Payload bytes each packet carries.
pub const PACKET_SENSOR_DATA_BYTES: usize = MAX_UDP_PACKET_LEN - PACKET_EXTRA_BYTES;

/// Thresholds the driver computes: four colours plus the Pixie-III hit
/// threshold (C `NUM_THRESHOLDS`).
pub const NUM_THRESHOLDS: usize = 5;

// Environmental limits (C).
pub const DEW_POINT_WARNING: f64 = 3.0;
pub const DEW_POINT_ERROR: f64 = 0.0;
pub const THOT_WARNING: f64 = 40.0;
pub const THOT_ERROR: f64 = 50.0;
pub const TCOLD_WARNING: f64 = 30.0;
pub const TCOLD_ERROR: f64 = 40.0;

// Initial values (C).
pub const INITIAL_HV_VALUE: f64 = 350.0;
pub const INITIAL_COOLING_VALUE: f64 = 15.0;

// Threshold calibration (C).
pub const THRESH_B_COEFF: f64 = 39.3;
pub const THRESH_A_COEFF: f64 = 36.6;
pub const EXTDAC_LSB: f64 = 0.000781;
pub const VAGND: f64 = 0.6;
pub const VTHMAX_UPPER_LIMIT: i32 = 2200;
pub const VTHMAX_LOWER_LIMIT: i32 = 1000;
pub const VTHMAX_DECR_STEP: i32 = 1;
pub const VTH1_ACCURACY: f64 = 0.001;
pub const INT_DAC_STEPS: usize = 32;
pub const PIII_P0: f64 = 494.70;
pub const PIII_P1: f64 = 19.36;

/// The internal-DAC fractions of VTHMAX the Pixie-II can select
/// (C `thresholdFractions`).
pub const THRESHOLD_FRACTIONS: [f64; INT_DAC_STEPS] = [
    0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.12, 0.14, 0.16, 0.18, 0.2,
    0.22, 0.24, 0.26, 0.28, 0.32, 0.36, 0.40, 0.44, 0.48, 0.52, 0.56, 0.60, 0.7, 0.8, 0.9, 1.0,
];

/// Depth of both conversion tables (C `PSTABLE_DEPTH` / `CONVERSIONTABLEDEPTH`).
pub const CONVERSION_TABLE_DEPTH: usize = 32768;
/// Width of the pseudo-random counter (C `PSCNT_WIDTH`).
pub const PS_COUNTER_WIDTH: usize = 15;
/// Taps of the Pixie-III 15-bit pseudo-random sequence.
pub const PIII_PRANDOM_15BITS_B0: usize = 13;
pub const PIII_PRANDOM_15BITS_B1: usize = 14;
/// Taps of the Pixie-III 7-bit pseudo-random sequence.
pub const PIII_PRANDOM_07BITS_B0: usize = 5;
pub const PIII_PRANDOM_07BITS_B1: usize = 6;

// Parameter names.
pub const PIXIRAD_SYSTEM_RESET: &str = "SYSTEM_RESET";
pub const PIXIRAD_SYSTEM_INFO: &str = "SYSTEM_INFO";
pub const PIXIRAD_COLORS_COLLECTED: &str = "COLORS_COLLECTED";
pub const PIXIRAD_UDP_BUFFERS_READ: &str = "UDP_BUFFERS_READ";
pub const PIXIRAD_UDP_BUFFERS_MAX: &str = "UDP_BUFFERS_MAX";
pub const PIXIRAD_UDP_BUFFERS_FREE: &str = "UDP_BUFFERS_FREE";
pub const PIXIRAD_UDP_SPEED: &str = "UDP_SPEED";
pub const PIXIRAD_THRESHOLD: [&str; 4] = ["THRESHOLD1", "THRESHOLD2", "THRESHOLD3", "THRESHOLD4"];
pub const PIXIRAD_HIT_THRESHOLD: &str = "HIT_THRESHOLD";
pub const PIXIRAD_THRESHOLD_ACTUAL: [&str; 4] = [
    "THRESHOLD_ACTUAL1",
    "THRESHOLD_ACTUAL2",
    "THRESHOLD_ACTUAL3",
    "THRESHOLD_ACTUAL4",
];
pub const PIXIRAD_HIT_THRESHOLD_ACTUAL: &str = "HIT_THRESHOLD_ACTUAL";
pub const PIXIRAD_COUNT_MODE: &str = "COUNT_MODE";
pub const PIXIRAD_AUTO_CALIBRATE: &str = "AUTO_CALIBRATE";
pub const PIXIRAD_HV_VALUE: &str = "HV_VALUE";
pub const PIXIRAD_HV_STATE: &str = "HV_STATE";
pub const PIXIRAD_HV_MODE: &str = "HV_MODE";
pub const PIXIRAD_HV_ACTUAL: &str = "HV_ACTUAL";
pub const PIXIRAD_HV_CURRENT: &str = "HV_CURRENT";
pub const PIXIRAD_SYNC_IN_POLARITY: &str = "SYNC_IN_POLARITY";
pub const PIXIRAD_SYNC_OUT_POLARITY: &str = "SYNC_OUT_POLARITY";
pub const PIXIRAD_SYNC_OUT_FUNCTION: &str = "SYNC_OUT_FUNCTION";
pub const PIXIRAD_COOLING_STATE: &str = "COOLING_STATE";
pub const PIXIRAD_HOT_TEMPERATURE: &str = "HOT_TEMPERATURE";
pub const PIXIRAD_BOX_TEMPERATURE: &str = "BOX_TEMPERATURE";
pub const PIXIRAD_BOX_HUMIDITY: &str = "BOX_HUMIDITY";
pub const PIXIRAD_DEW_POINT: &str = "DEW_POINT";
pub const PIXIRAD_COOLING_STATUS: &str = "COOLING_STATUS";
pub const PIXIRAD_PELTIER_POWER: &str = "PELTIER_POWER";
/// Internal, record-less: the `pixiradAutoCal` iocsh command writes its
/// arguments here (C reached into the driver through `findAsynPortDriver`).
pub const PIXIRAD_AUTOCAL_CONF: &str = "AUTOCAL_CONF";

/// Trigger mode (C `PixiradTriggerMode_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerMode {
    Internal,
    External,
    Bulb,
}

impl TriggerMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Internal),
            1 => Some(Self::External),
            2 => Some(Self::Bulb),
            _ => None,
        }
    }

    /// What the box calls it (C `PixiradTriggerModeStrings`).
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Internal => "INT",
            Self::External => "EXT1",
            Self::Bulb => "EXT2",
        }
    }
}

/// High-voltage mode (C `PixiradHVMode_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HVMode {
    Manual,
    Auto,
}

impl HVMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Manual),
            1 => Some(Self::Auto),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Manual => "STDHV",
            Self::Auto => "AUTOHV",
        }
    }
}

/// Sync polarity (C `PixiradSyncPolarity_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncPolarity {
    Pos,
    Neg,
}

impl SyncPolarity {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Pos),
            1 => Some(Self::Neg),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Pos => "POS",
            Self::Neg => "NEG",
        }
    }
}

/// What the sync output signals (C `PixiradSyncOutFunction_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncOutFunction {
    Shutter,
    ReadoutDone,
    Read,
}

impl SyncOutFunction {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Shutter),
            1 => Some(Self::ReadoutDone),
            2 => Some(Self::Read),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Shutter => "SHUTTER",
            Self::ReadoutDone => "RODONE",
            Self::Read => "READ",
        }
    }
}

/// Frame type — how many colours the box collects per image
/// (C `PixiradFrameType_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    OneColorLow,
    OneColorHigh,
    TwoColors,
    FourColors,
    OneColorDTF,
    TwoColorsDTF,
}

impl FrameType {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::OneColorLow),
            1 => Some(Self::OneColorHigh),
            2 => Some(Self::TwoColors),
            3 => Some(Self::FourColors),
            4 => Some(Self::OneColorDTF),
            5 => Some(Self::TwoColorsDTF),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::OneColorLow => "1COL0",
            Self::OneColorHigh => "1COL1",
            Self::TwoColors => "2COL",
            Self::FourColors => "4COL",
            Self::OneColorDTF => "DTF",
            Self::TwoColorsDTF => "2COLDTF",
        }
    }

    /// Colours the box sends per image (C `FTNumColors`).
    pub fn num_colors(self) -> usize {
        match self {
            Self::OneColorLow | Self::OneColorHigh | Self::OneColorDTF => 1,
            Self::TwoColors | Self::TwoColorsDTF => 2,
            Self::FourColors => 4,
        }
    }

    /// Whether the readout runs in dead-time-free mode (C `readoutModeString`).
    pub fn readout_mode(self) -> &'static str {
        match self {
            Self::OneColorDTF | Self::TwoColorsDTF => "DTF",
            _ => "NODTF",
        }
    }
}

/// Count mode (C `PixiradCountModeType_t`); Pixie-II only ever sends `NONBI`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CountMode {
    Normal,
    NPI,
    NPISum,
}

impl CountMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Normal),
            1 => Some(Self::NPI),
            2 => Some(Self::NPISum),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Normal => "Normal",
            Self::NPI => "NPI",
            Self::NPISum => "NPISUM",
        }
    }
}

/// Cooling status (C `PixiradCoolingStatus_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoolingStatus {
    Ok = 0,
    DewPointWarning = 1,
    DewPointError = 2,
    THotWarning = 3,
    THotError = 4,
    TColdWarning = 5,
    TColdError = 6,
}

impl CoolingStatus {
    /// Whether the driver must switch the cooling off (C `statusTask`).
    pub fn is_error(self) -> bool {
        matches!(
            self,
            Self::DewPointError | Self::THotError | Self::TColdError
        )
    }
}

/// Which ASIC the detector is built on (C `ASIC_TYPE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Asic {
    PII,
    PIII,
}

/// Which detector of the family (C `DETECTOR_BUILD`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Build {
    PX1,
    PX2,
    PX4,
    PX8,
}

impl Build {
    /// C `PixiradModelNames`.
    pub fn model_name(self) -> &'static str {
        match self {
            Self::PX1 => "Pixirad-1",
            Self::PX2 => "Pixirad-2",
            Self::PX4 => "Pixirad-4",
            Self::PX8 => "Pixirad-8",
        }
    }
}

/// Everything about the sensor that the decoder and the packet reader need
/// (C `SENSOR`, filled from `maxSizeX` / `maxSizeY` in the constructor).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sensor {
    pub asic: Asic,
    pub build: Build,
    pub modules: usize,
    pub rows: usize,
    pub cols: usize,
    pub dout: usize,
    pub cols_per_dout: usize,
    /// Pixels of one module (C `matrix_size_pxls`).
    pub matrix_size_pxls: usize,
    /// Bit planes per counter in a normal frame (C `bit_per_cnt_std`).
    pub bit_per_cnt_std: usize,
    /// Bit planes per counter in an autocalibration frame
    /// (C `autocal_bit_cnt`).
    pub autocal_bit_cnt: usize,
    /// UDP packets in a normal frame (C `numUDPPackets_`).
    pub num_udp_packets: usize,
    /// UDP packets in an autocalibration frame (C `numAutocalUDPPackets_`).
    pub num_autocal_udp_packets: usize,
}

impl Sensor {
    /// The sensor a `maxSizeX` × `maxSizeY` detector has (C's constructor
    /// switch). X selects the ASIC, Y the number of modules.
    pub fn from_size(max_size_x: i32, max_size_y: i32) -> Option<Self> {
        let (asic, rows) = match max_size_x {
            476 => (Asic::PII, 476usize),
            402 => (Asic::PIII, 402usize),
            _ => return None,
        };
        let (build, modules, num_udp_packets, num_autocal_udp_packets) = match (asic, max_size_y) {
            (Asic::PII, 512) => (Build::PX1, 1, 360, 135),
            (Asic::PII, 1024) => (Build::PX2, 2, 720, 270),
            (Asic::PII, 4096) => (Build::PX8, 8, 2539, 1080),
            (Asic::PIII, 512) => (Build::PX1, 1, 270, 180),
            (Asic::PIII, 1024) => (Build::PX2, 2, 540, 360),
            (Asic::PIII, 4096) => (Build::PX8, 8, 2539, 1080),
            _ => return None,
        };
        let cols = 512;
        let (bit_per_cnt_std, autocal_bit_cnt) = match asic {
            Asic::PII => (15, 5),
            Asic::PIII => (15, 9),
        };
        Some(Self {
            asic,
            build,
            modules,
            rows,
            cols,
            dout: 16,
            cols_per_dout: 32,
            matrix_size_pxls: rows * cols,
            bit_per_cnt_std,
            autocal_bit_cnt,
            num_udp_packets,
            num_autocal_udp_packets,
        })
    }

    /// Bit planes in a frame of this kind.
    pub fn code_depth(&self, is_autocal: bool) -> usize {
        if is_autocal {
            self.autocal_bit_cnt
        } else {
            self.bit_per_cnt_std
        }
    }

    /// Packets in a frame of this kind.
    pub fn packets(&self, is_autocal: bool) -> usize {
        if is_autocal {
            self.num_autocal_udp_packets
        } else {
            self.num_udp_packets
        }
    }

    /// Pixels in the whole detector (all modules).
    pub fn image_pixels(&self) -> usize {
        self.modules * self.matrix_size_pxls
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_from_size_knows_every_documented_detector() {
        let px1 = Sensor::from_size(476, 512).unwrap();
        assert_eq!(px1.asic, Asic::PII);
        assert_eq!(px1.build, Build::PX1);
        assert_eq!(px1.modules, 1);
        assert_eq!(px1.num_udp_packets, 360);
        assert_eq!(px1.image_pixels(), 476 * 512);

        let px2 = Sensor::from_size(402, 1024).unwrap();
        assert_eq!(px2.asic, Asic::PIII);
        assert_eq!(px2.modules, 2);
        assert_eq!(px2.num_udp_packets, 540);
        assert_eq!(px2.autocal_bit_cnt, 9);
        assert_eq!(px2.image_pixels(), 402 * 512 * 2);
    }

    #[test]
    fn sensor_from_size_rejects_an_unknown_geometry() {
        // C printed "Illegal maxSizeX" and carried on with an uninitialised
        // SENSOR; an unknown geometry is refused here.
        assert!(Sensor::from_size(1024, 1024).is_none());
        assert!(Sensor::from_size(476, 640).is_none());
    }

    #[test]
    fn frame_type_colors_match_c() {
        assert_eq!(FrameType::OneColorLow.num_colors(), 1);
        assert_eq!(FrameType::TwoColors.num_colors(), 2);
        assert_eq!(FrameType::FourColors.num_colors(), 4);
        assert_eq!(FrameType::TwoColorsDTF.num_colors(), 2);
        assert_eq!(FrameType::TwoColorsDTF.readout_mode(), "DTF");
        assert_eq!(FrameType::TwoColors.readout_mode(), "NODTF");
    }
}
