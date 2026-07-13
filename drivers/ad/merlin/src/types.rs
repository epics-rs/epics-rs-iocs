//! Constants, enums and on-wire names for the Merlin (Medipix) MPX protocol.
//!
//! Ported from `merlinApp/src/merlin_low.h` and `merlinDetector.h`.

/// Longest legal command-channel message, header included
/// (C `MPX_MAXLINE`).
pub const MPX_MAXLINE: usize = 256;

/// `MPX` + `,` + 10 decimal length digits + `,` — the fixed part of every
/// frame that precedes the body (C: `strlen(MPX_HEADER) +
/// MPX_MSG_LEN_DIGITS + 2`).
pub const MPX_FRAME_HEADER_LEN: usize = 15;

/// Leading magic of every MPX frame.
pub const MPX_HEADER: &[u8] = b"MPX";

/// Number of decimal digits in the length field.
pub const MPX_MSG_LEN_DIGITS: usize = 10;

/// Bytes of a data-frame body that identify its type (`HDR`, `MQ1`, `PR1`).
pub const MPX_MSG_DATATYPE_LEN: usize = 3;

/// Acquisition-header body length kept for the "Acquisition Header"
/// attribute (C `MPX_ACQUISITION_HEADER_LEN`).
pub const MPX_ACQUISITION_HEADER_LEN: usize = 2044;

/// Largest data-channel frame we will accept, per detector family.
pub const MAX_BUFF_MERLIN_QUAD: usize = 2_000_000;
pub const MAX_BUFF_UOM: usize = 5_300_000;

/// C `MPX_IMG_FRAME_LEN24`: 256-byte header + 256*256 32-bit pixels + type
/// field + commas.
pub const MPX_IMG_FRAME_LEN24: usize = 256 + 65536 * 2 * 2 + 3 + 2;

/// Default command timeout (C `Labview_DEFAULT_TIMEOUT`).
pub const LABVIEW_DEFAULT_TIMEOUT_SEC: f64 = 2.0;

/// Data-channel read timeout (C: `mpxRead(..., 10)` in `merlinTask`).
pub const DATA_READ_TIMEOUT_SEC: f64 = 10.0;

// --- MPX error codes (C `merlin_low.h`) ---------------------------------
pub const MPX_OK: i32 = 0;
pub const MPX_ERR_UNEXPECTED: i32 = 111;

// --- Command-channel message kinds --------------------------------------
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MpxKind {
    Set,
    Get,
    Cmd,
}

impl MpxKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Set => "SET",
            Self::Get => "GET",
            Self::Cmd => "CMD",
        }
    }
}

// --- Data-frame header kinds --------------------------------------------
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataHeader {
    /// `HDR` — the per-acquisition text header.
    Acquisition,
    /// `MQ1` — a Merlin Quad image frame.
    QuadData,
    /// `PR1` — an X/Y profile frame.
    Profile,
    Unknown,
}

// --- Detector families ---------------------------------------------------
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum DetectorType {
    Merlin = 0,
    MerlinXbpm = 1,
    UomXbpm = 2,
    MerlinQuad = 3,
}

impl DetectorType {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Merlin),
            1 => Some(Self::MerlinXbpm),
            2 => Some(Self::UomXbpm),
            3 => Some(Self::MerlinQuad),
            _ => None,
        }
    }

    /// The Merlin FPGA sends pixels big-endian; the XBPM variants send them
    /// host-order (C `merlinDetector::endian_swap` only swaps for `Merlin`
    /// and `MerlinQuad`).
    pub fn swaps_pixels(self) -> bool {
        matches!(self, Self::Merlin | Self::MerlinQuad)
    }

    /// Buffer budget for one data-channel frame (C `merlinTask`).
    pub fn max_frame_bytes(self) -> usize {
        match self {
            Self::Merlin | Self::MerlinXbpm => MPX_IMG_FRAME_LEN24,
            Self::MerlinQuad => MAX_BUFF_MERLIN_QUAD,
            Self::UomXbpm => MAX_BUFF_UOM,
        }
    }

    pub fn manufacturer(self) -> &'static str {
        match self {
            Self::UomXbpm => "University of Manchester",
            _ => "Merlin Consortium",
        }
    }

    pub fn model(self) -> &'static str {
        match self {
            Self::Merlin => "Merlin",
            Self::MerlinXbpm => "Lancelot XBPM",
            Self::UomXbpm => "UoM XBPM",
            Self::MerlinQuad => "Merlin Quad",
        }
    }

    pub fn gui(self) -> &'static str {
        match self {
            Self::MerlinQuad => "merlinQuadEmbedded.edl",
            _ => "merlinEmbedded.edl",
        }
    }
}

/// EPICS `ImageMode` values, extended by Merlin (C `MPXImageMode_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MerlinImageMode {
    Single = 0,
    Multiple = 1,
    Continuous = 2,
    ThresholdScan = 3,
    BackgroundCalibrate = 4,
}

impl MerlinImageMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Single),
            1 => Some(Self::Multiple),
            2 => Some(Self::Continuous),
            3 => Some(Self::ThresholdScan),
            4 => Some(Self::BackgroundCalibrate),
            _ => None,
        }
    }
}

/// C `merlinTriggerMode`. The device controls the start and the stop trigger
/// separately; `setAcquireParams` maps each EPICS mode onto a start/stop pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerMode {
    Internal = 0,
    ExternalEnable = 1,
    ExternalTriggerHigh = 2,
    ExternalTriggerLow = 3,
    ExternalTriggerRising = 4,
    SoftwareTrigger = 5,
}

impl TriggerMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Internal),
            1 => Some(Self::ExternalEnable),
            2 => Some(Self::ExternalTriggerHigh),
            3 => Some(Self::ExternalTriggerLow),
            4 => Some(Self::ExternalTriggerRising),
            5 => Some(Self::SoftwareTrigger),
            _ => None,
        }
    }
}

/// Individual trigger-edge codes sent for `TRIGGERSTART` / `TRIGGERSTOP`.
pub const TM_TRIG_INTERNAL: &str = "0";
pub const TM_TRIG_RISING: &str = "1";
pub const TM_TRIG_FALLING: &str = "2";
pub const TM_TRIG_SOFTWARE: &str = "3";

/// C `MPXQuadMode_t`: the six named Quad modes, each a canned combination of
/// five device settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuadModeSettings {
    pub counter_depth: i32,
    pub colour_mode: i32,
    pub enable_counter1: i32,
    pub continuous_rw: i32,
    pub charge_summing: i32,
    pub frames_per_acquire: i32,
}

/// Decode a `QUADMERLINMODE` selection (C `merlinDetector::SetQuadMode`).
pub fn quad_mode_settings(mode: i32) -> Option<QuadModeSettings> {
    let base = QuadModeSettings {
        counter_depth: 12,
        colour_mode: 0,
        enable_counter1: 0,
        continuous_rw: 0,
        charge_summing: 0,
        frames_per_acquire: 1,
    };
    match mode {
        // 12 bit
        0 => Some(base),
        // 24 bit
        1 => Some(QuadModeSettings {
            counter_depth: 24,
            ..base
        }),
        // Two threshold: both counters, two frames per acquire.
        2 => Some(QuadModeSettings {
            enable_counter1: 2,
            frames_per_acquire: 2,
            ..base
        }),
        // Continuous read/write
        3 => Some(QuadModeSettings {
            continuous_rw: 1,
            ..base
        }),
        // Colour: both counters, eight frames per acquire.
        4 => Some(QuadModeSettings {
            colour_mode: 1,
            enable_counter1: 2,
            frames_per_acquire: 8,
            ..base
        }),
        // Charge summing
        5 => Some(QuadModeSettings {
            charge_summing: 1,
            enable_counter1: 1,
            ..base
        }),
        _ => None,
    }
}

// --- Bits in the PROFILES selection mask (C `merlin_low.h`) --------------
pub const MPXPROFILES_IMAGE: i32 = 1;
pub const MPXPROFILES_XPROFILE: i32 = 2;
pub const MPXPROFILES_YPROFILE: i32 = 4;
pub const MPXPROFILES_SUM: i32 = 8;

// --- Device variable names (SET / GET) ----------------------------------
pub const MPXVAR_GETSOFTWAREVERSION: &str = "SOFTWAREVERSION";
pub const MPXVAR_NUMFRAMESTOACQUIRE: &str = "NUMFRAMESTOACQUIRE";
pub const MPXVAR_ACQUISITIONTIME: &str = "ACQUISITIONTIME";
pub const MPXVAR_ACQUISITIONPERIOD: &str = "ACQUISITIONPERIOD";
pub const MPXVAR_TRIGGERSTART: &str = "TRIGGERSTART";
pub const MPXVAR_TRIGGERSTOP: &str = "TRIGGERSTOP";
pub const MPXVAR_NUMFRAMESPERTRIGGER: &str = "NUMFRAMESPERTRIGGER";
pub const MPXVAR_COUNTERDEPTH: &str = "COUNTERDEPTH";
pub const MPXVAR_ENABLECOUNTER1: &str = "ENABLECOUNTER1";
pub const MPXVAR_CONTINUOUSRW: &str = "CONTINUOUSRW";
pub const MPXVAR_ROI: &str = "ROI";
pub const MPXVAR_ENABLEBACKROUNDCORR: &str = "BCKGRNDCORRECTION";
pub const MPXVAR_BACKGROUNDCOUNT: &str = "BCKGRND";
pub const MPXVAR_ENABLEIMAGEAVERAGE: &str = "IMGAVERAGE";
pub const MPXVAR_IMAGESTOSUM: &str = "IMAGESTOSUM";
pub const MPXVAR_COLOURMODE: &str = "COLOURMODE";
pub const MPXVAR_CHARGESUMMING: &str = "CHARGESUMMING";
pub const MPXVAR_THSSCAN: &str = "THSCAN";
pub const MPXVAR_THSTART: &str = "THSTART";
pub const MPXVAR_THSTOP: &str = "THSTOP";
pub const MPXVAR_THSTEP: &str = "THSTEP";
pub const MPXVAR_OPERATINGENERGY: &str = "OPERATINGENERGY";
/// `THRESHOLD0` .. `THRESHOLD7`.
pub const MPXVAR_THRESHOLD: [&str; 8] = [
    "THRESHOLD0",
    "THRESHOLD1",
    "THRESHOLD2",
    "THRESHOLD3",
    "THRESHOLD4",
    "THRESHOLD5",
    "THRESHOLD6",
    "THRESHOLD7",
];

// --- Device commands (CMD) ----------------------------------------------
pub const MPXCMD_STARTACQUISITION: &str = "STARTACQUISITION";
pub const MPXCMD_STOPACQUISITION: &str = "STOPACQUISITION";
pub const MPXCMD_THSCAN: &str = "THSCAN";
pub const MPXCMD_SOFTWARETRIGGER: &str = "SWTRIGGER";
pub const MPXCMD_RESET: &str = "RESET";
/// Also the variable name written by the `PROFILECONTROL` parameter.
pub const MPXCMD_PROFILES: &str = "PROFILES";
pub const MPXCMD_BACKGROUNDACQUIRE: &str = "BCKGRND";
