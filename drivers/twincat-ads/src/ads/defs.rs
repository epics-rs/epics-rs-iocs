//! ADS/AMS protocol constants and enums.
//!
//! Ported from Beckhoff `AdsLib/standalone/AdsDef.h` (MIT). Only the subset the
//! twincat-ads driver actually uses is reproduced here — no C/C++ linking.

use std::fmt;
use std::str::FromStr;

/// TCP port of the AMS router on a TwinCAT system.
pub const ADS_TCP_SERVER_PORT: u16 = 0xBF02; // 48898

/// Local AMS port range handed out by the AMS router (`Router.h`).
pub const LOCAL_PORT_BASE: u16 = 30000;

// ADS command ids (`ADSSRVID_*`).
pub const CMD_READ_DEVICE_INFO: u16 = 0x0001;
pub const CMD_READ: u16 = 0x0002;
pub const CMD_WRITE: u16 = 0x0003;
pub const CMD_READ_STATE: u16 = 0x0004;
pub const CMD_WRITE_CONTROL: u16 = 0x0005;
pub const CMD_ADD_DEVICE_NOTIFICATION: u16 = 0x0006;
pub const CMD_DEL_DEVICE_NOTIFICATION: u16 = 0x0007;
pub const CMD_DEVICE_NOTIFICATION: u16 = 0x0008;
pub const CMD_READ_WRITE: u16 = 0x0009;

// AoE header state flags.
pub const STATE_FLAG_REQUEST: u16 = 0x0004;
pub const STATE_FLAG_RESPONSE: u16 = 0x0005;

// ADS reserved index groups (`ADSIGRP_*`) used by this driver.
pub const ADSIGRP_SYM_HNDBYNAME: u32 = 0xF003;
pub const ADSIGRP_SYM_VALBYHND: u32 = 0xF005;
pub const ADSIGRP_SYM_RELEASEHND: u32 = 0xF006;
pub const ADSIGRP_SYM_VERSION: u32 = 0xF008;
pub const ADSIGRP_SYM_INFOBYNAMEEX: u32 = 0xF009;
/// Sum-up read: write `{iGroup, iOffset, len}` triples, read `{results}` then `{data}`.
pub const ADSIGRP_SUMUP_READ: u32 = 0xF080;

/// `%M` memory area — the fallback bulk-read timestamp source when the PLC has
/// no `MAIN.fbSystemTime` symbols (adsAsynPortDriver.cpp:1359).
pub const ADSIGRP_MEMORY_BYTE: u32 = 0x4020;

// Error codes referenced by name in the driver's control flow.
pub const GLOBALERR_TARGET_PORT: u32 = 0x06;
pub const ADSERR_CLIENT_ERROR: u32 = 0x0740;

/// ADS notification transmission mode (`ADSTRANSMODE`).
pub const ADSTRANS_SERVERONCHA: u32 = 4;

/// AMS Net Id — six bytes, conventionally `<ipv4>.1.1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, PartialOrd, Ord)]
pub struct AmsNetId(pub [u8; 6]);

impl fmt::Display for AmsNetId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let b = self.0;
        write!(f, "{}.{}.{}.{}.{}.{}", b[0], b[1], b[2], b[3], b[4], b[5])
    }
}

/// Parse failure for an AMS Net Id string.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AmsNetIdParseError(pub String);

impl fmt::Display for AmsNetIdParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "invalid AMS Net Id: {}", self.0)
    }
}

impl std::error::Error for AmsNetIdParseError {}

impl FromStr for AmsNetId {
    type Err = AmsNetIdParseError;

    /// Mirrors the C constructor's `sscanf("%hhu.%hhu.%hhu.%hhu.%hhu.%hhu")`
    /// (adsAsynPortDriver.cpp:343): exactly six decimal octets.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut out = [0u8; 6];
        let mut n = 0;
        for part in s.split('.') {
            let v: u8 = part
                .parse()
                .map_err(|_| AmsNetIdParseError(s.to_string()))?;
            if n == 6 {
                return Err(AmsNetIdParseError(s.to_string()));
            }
            out[n] = v;
            n += 1;
        }
        if n != 6 {
            return Err(AmsNetIdParseError(s.to_string()));
        }
        Ok(AmsNetId(out))
    }
}

/// Full AMS address: net id plus AMS port (851 = first TC3 PLC runtime).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, PartialOrd, Ord)]
pub struct AmsAddr {
    pub net_id: AmsNetId,
    pub port: u16,
}

/// ADS device state (`ADSSTATE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum AdsState {
    Invalid = 0,
    Idle = 1,
    Reset = 2,
    Init = 3,
    Start = 4,
    Run = 5,
    Stop = 6,
    SaveCfg = 7,
    LoadCfg = 8,
    PowerFailure = 9,
    PowerGood = 10,
    Error = 11,
    Shutdown = 12,
    Suspend = 13,
    Resume = 14,
    Config = 15,
    Reconfig = 16,
    Stopping = 17,
    Incompatible = 18,
    Exception = 19,
    MaxStates = 20,
    /// Not an upstream `ADSSTATE`. The C driver seeds a fresh `amsPortInfo`
    /// with `ADSSTATE_MAXSTATES + 1` as an "unknown" sentinel
    /// (adsAsynPortDriver.cpp:1848); `adsStateToString` has no arm for it and
    /// falls through to `"UNKNOWN ADSSTATE"`. Naming the sentinel keeps that
    /// state representable without an out-of-range cast.
    Unknown = 21,
}

impl AdsState {
    pub fn from_u16(v: u16) -> Self {
        match v {
            0 => Self::Invalid,
            1 => Self::Idle,
            2 => Self::Reset,
            3 => Self::Init,
            4 => Self::Start,
            5 => Self::Run,
            6 => Self::Stop,
            7 => Self::SaveCfg,
            8 => Self::LoadCfg,
            9 => Self::PowerFailure,
            10 => Self::PowerGood,
            11 => Self::Error,
            12 => Self::Shutdown,
            13 => Self::Suspend,
            14 => Self::Resume,
            15 => Self::Config,
            16 => Self::Reconfig,
            17 => Self::Stopping,
            18 => Self::Incompatible,
            19 => Self::Exception,
            20 => Self::MaxStates,
            _ => Self::Unknown,
        }
    }

    /// `adsStateToString` (adsAsynPortDriverUtils.cpp:307).
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Invalid => "ADSSTATE_INVALID",
            Self::Idle => "ADSSTATE_IDLE",
            Self::Reset => "ADSSTATE_RESET",
            Self::Init => "ADSSTATE_INIT",
            Self::Start => "ADSSTATE_START",
            Self::Run => "ADSSTATE_RUN",
            Self::Stop => "ADSSTATE_STOP",
            Self::SaveCfg => "ADSSTATE_SAVECFG",
            Self::LoadCfg => "ADSSTATE_LOADCFG",
            Self::PowerFailure => "ADSSTATE_POWERFAILURE",
            Self::PowerGood => "ADSSTATE_POWERGOOD",
            Self::Error => "ADSSTATE_ERROR",
            Self::Shutdown => "ADSSTATE_SHUTDOWN",
            Self::Suspend => "ADSSTATE_SUSPEND",
            Self::Resume => "ADSSTATE_RESUME",
            Self::Config => "ADSSTATE_CONFIG",
            Self::Reconfig => "ADSSTATE_RECONFIG",
            Self::Stopping => "ADSSTATE_STOPPING",
            Self::Incompatible => "ADSSTATE_INCOMPATIBLE",
            Self::Exception => "ADSSTATE_EXCEPTION",
            Self::MaxStates => "ADSSTATE_MAXSTATES",
            Self::Unknown => "UNKNOWN ADSSTATE",
        }
    }
}

/// ADS data type id (`ADSDATATYPEID`, adsAsynPortDriverUtils.h:152).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdsType {
    Void,
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Int64,
    UInt64,
    Real32,
    Real64,
    Real80,
    BigType,
    String,
    WString,
    Bit,
    /// Any id the PLC reports that is not in the table above.
    Unknown(u32),
}

impl AdsType {
    pub fn from_u32(v: u32) -> Self {
        match v {
            0 => Self::Void,
            2 => Self::Int16,
            3 => Self::Int32,
            4 => Self::Real32,
            5 => Self::Real64,
            16 => Self::Int8,
            17 => Self::UInt8,
            18 => Self::UInt16,
            19 => Self::UInt32,
            20 => Self::Int64,
            21 => Self::UInt64,
            30 => Self::String,
            31 => Self::WString,
            32 => Self::Real80,
            33 => Self::Bit,
            65 => Self::BigType,
            other => Self::Unknown(other),
        }
    }

    pub fn to_u32(self) -> u32 {
        match self {
            Self::Void => 0,
            Self::Int16 => 2,
            Self::Int32 => 3,
            Self::Real32 => 4,
            Self::Real64 => 5,
            Self::Int8 => 16,
            Self::UInt8 => 17,
            Self::UInt16 => 18,
            Self::UInt32 => 19,
            Self::Int64 => 20,
            Self::UInt64 => 21,
            Self::String => 30,
            Self::WString => 31,
            Self::Real80 => 32,
            Self::Bit => 33,
            Self::BigType => 65,
            Self::Unknown(v) => v,
        }
    }

    /// Size of one element, or `None` where the C `adsTypeSize` returns
    /// `(size_t)-1` (adsAsynPortDriverUtils.cpp:382) — a sentinel meaning
    /// "no fixed element size", which every C caller compares against
    /// `plcSize` and so never treats as a real length.
    pub fn element_size(self) -> Option<usize> {
        match self {
            Self::Void => Some(0),
            Self::Int8 | Self::UInt8 | Self::Bit => Some(1),
            // ADST_STRING is an array of char (C returns 1).
            Self::String => Some(1),
            Self::Int16 | Self::UInt16 => Some(2),
            Self::Int32 | Self::UInt32 | Self::Real32 => Some(4),
            Self::Int64 | Self::UInt64 | Self::Real64 => Some(8),
            Self::Real80 => Some(10),
            Self::WString | Self::BigType | Self::Unknown(_) => None,
        }
    }

    /// `adsTypeToString` (adsAsynPortDriverUtils.cpp:222).
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Void => "ADST_VOID",
            Self::Int16 => "ADST_INT16",
            Self::Int32 => "ADST_INT32",
            Self::Real32 => "ADST_REAL32",
            Self::Real64 => "ADST_REAL64",
            Self::Int8 => "ADST_INT8",
            Self::UInt8 => "ADST_UINT8",
            Self::UInt16 => "ADST_UINT16",
            Self::UInt32 => "ADST_UINT32",
            Self::Int64 => "ADST_INT64",
            Self::UInt64 => "ADST_UINT64",
            Self::String => "ADST_STRING",
            Self::WString => "ADST_WSTRING",
            Self::Real80 => "ADST_REAL80",
            Self::Bit => "ADST_BIT",
            Self::BigType => "ADST_BIGTYPE",
            Self::Unknown(_) => "ADS_UNKNOWN_DATATYPE",
        }
    }
}

/// Router/device version triple returned by `ReadDeviceInfo`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AdsVersion {
    pub version: u8,
    pub revision: u8,
    pub build: u16,
}

/// `adsErrorToString` (adsAsynPortDriverUtils.cpp:73).
pub fn error_to_string(error: u32) -> &'static str {
    match error {
        0x06 => "GLOBALERR_TARGET_PORT",
        0x07 => "GLOBALERR_MISSING_ROUTE",
        0x19 => "GLOBALERR_NO_MEMORY",
        0x1A => "GLOBALERR_TCP_SEND",
        0x0700 => "ADSERR_DEVICE_ERROR",
        0x0701 => "ADSERR_DEVICE_SRVNOTSUPP",
        0x0702 => "ADSERR_DEVICE_INVALIDGRP",
        0x0703 => "ADSERR_DEVICE_INVALIDOFFSET",
        0x0704 => "ADSERR_DEVICE_INVALIDACCESS",
        0x0705 => "ADSERR_DEVICE_INVALIDSIZE",
        0x0706 => "ADSERR_DEVICE_INVALIDDATA",
        0x0707 => "ADSERR_DEVICE_NOTREADY",
        0x0708 => "ADSERR_DEVICE_BUSY",
        0x0709 => "ADSERR_DEVICE_INVALIDCONTEXT",
        0x070A => "ADSERR_DEVICE_NOMEMORY",
        0x070B => "ADSERR_DEVICE_INVALIDPARM",
        0x070C => "ADSERR_DEVICE_NOTFOUND",
        0x070D => "ADSERR_DEVICE_SYNTAX",
        0x070E => "ADSERR_DEVICE_INCOMPATIBLE",
        0x070F => "ADSERR_DEVICE_EXISTS",
        0x0710 => "ADSERR_DEVICE_SYMBOLNOTFOUND",
        0x0711 => "ADSERR_DEVICE_SYMBOLVERSIONINVALID",
        0x0712 => "ADSERR_DEVICE_INVALIDSTATE",
        0x0713 => "ADSERR_DEVICE_TRANSMODENOTSUPP",
        0x0714 => "ADSERR_DEVICE_NOTIFYHNDINVALID",
        0x0715 => "ADSERR_DEVICE_CLIENTUNKNOWN",
        0x0716 => "ADSERR_DEVICE_NOMOREHDLS",
        0x0717 => "ADSERR_DEVICE_INVALIDWATCHSIZE",
        0x0718 => "ADSERR_DEVICE_NOTINIT",
        0x0719 => "ADSERR_DEVICE_TIMEOUT",
        0x071A => "ADSERR_DEVICE_NOINTERFACE",
        0x071B => "ADSERR_DEVICE_INVALIDINTERFACE",
        0x071C => "ADSERR_DEVICE_INVALIDCLSID",
        0x071D => "ADSERR_DEVICE_INVALIDOBJID",
        0x071E => "ADSERR_DEVICE_PENDING",
        0x071F => "ADSERR_DEVICE_ABORTED",
        0x0720 => "ADSERR_DEVICE_WARNING",
        0x0721 => "ADSERR_DEVICE_INVALIDARRAYIDX",
        0x0722 => "ADSERR_DEVICE_SYMBOLNOTACTIVE",
        0x0723 => "ADSERR_DEVICE_ACCESSDENIED",
        0x0724 => "ADSERR_DEVICE_LICENSENOTFOUND",
        0x0725 => "ADSERR_DEVICE_LICENSEEXPIRED",
        0x0726 => "ADSERR_DEVICE_LICENSEEXCEEDED",
        0x0727 => "ADSERR_DEVICE_LICENSEINVALID",
        0x0728 => "ADSERR_DEVICE_LICENSESYSTEMID",
        0x0729 => "ADSERR_DEVICE_LICENSENOTIMELIMIT",
        0x072A => "ADSERR_DEVICE_LICENSEFUTUREISSUE",
        0x072B => "ADSERR_DEVICE_LICENSETIMETOLONG",
        0x072C => "ADSERR_DEVICE_EXCEPTION",
        0x072D => "ADSERR_DEVICE_LICENSEDUPLICATED",
        0x072E => "ADSERR_DEVICE_SIGNATUREINVALID",
        0x072F => "ADSERR_DEVICE_CERTIFICATEINVALID",
        0x0740 => "ADSERR_CLIENT_ERROR",
        0x0741 => "ADSERR_CLIENT_INVALIDPARM",
        0x0742 => "ADSERR_CLIENT_LISTEMPTY",
        0x0743 => "ADSERR_CLIENT_VARUSED",
        0x0744 => "ADSERR_CLIENT_DUPLINVOKEID",
        0x0745 => "ADSERR_CLIENT_SYNCTIMEOUT",
        0x0746 => "ADSERR_CLIENT_W32ERROR",
        0x0747 => "ADSERR_CLIENT_TIMEOUTINVALID",
        0x0748 => "ADSERR_CLIENT_PORTNOTOPEN",
        0x0749 => "ADSERR_CLIENT_NOAMSADDR",
        0x0750 => "ADSERR_CLIENT_SYNCINTERNAL",
        0x0751 => "ADSERR_CLIENT_ADDHASH",
        0x0752 => "ADSERR_CLIENT_REMOVEHASH",
        0x0753 => "ADSERR_CLIENT_NOMORESYM",
        0x0754 => "ADSERR_CLIENT_SYNCRESINVALID",
        0x0755 => "ADSERR_CLIENT_SYNCPORTLOCKED",
        _ => "ADSERR_ERROR_UNKNOWN",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn net_id_parses_six_octets() {
        assert_eq!(
            "192.168.88.44.1.1".parse::<AmsNetId>().unwrap(),
            AmsNetId([192, 168, 88, 44, 1, 1])
        );
    }

    #[test]
    fn net_id_rejects_wrong_octet_count() {
        assert!("192.168.88.44.1".parse::<AmsNetId>().is_err());
        assert!("192.168.88.44.1.1.1".parse::<AmsNetId>().is_err());
        assert!("".parse::<AmsNetId>().is_err());
    }

    #[test]
    fn net_id_rejects_out_of_range_octet() {
        // C `%hhu` would silently truncate 256 to 0; Rust's u8 parse rejects it.
        assert!("192.168.88.256.1.1".parse::<AmsNetId>().is_err());
    }

    #[test]
    fn net_id_display_roundtrips() {
        let id: AmsNetId = "10.0.0.1.1.1".parse().unwrap();
        assert_eq!(id.to_string(), "10.0.0.1.1.1");
    }

    #[test]
    fn ads_type_sizes_match_c_table() {
        // adsAsynPortDriverUtils.cpp:382 `adsTypeSize`.
        assert_eq!(AdsType::Void.element_size(), Some(0));
        assert_eq!(AdsType::Int8.element_size(), Some(1));
        assert_eq!(AdsType::UInt8.element_size(), Some(1));
        assert_eq!(AdsType::Bit.element_size(), Some(1));
        assert_eq!(AdsType::String.element_size(), Some(1));
        assert_eq!(AdsType::Int16.element_size(), Some(2));
        assert_eq!(AdsType::UInt16.element_size(), Some(2));
        assert_eq!(AdsType::Int32.element_size(), Some(4));
        assert_eq!(AdsType::UInt32.element_size(), Some(4));
        assert_eq!(AdsType::Real32.element_size(), Some(4));
        assert_eq!(AdsType::Int64.element_size(), Some(8));
        assert_eq!(AdsType::UInt64.element_size(), Some(8));
        assert_eq!(AdsType::Real64.element_size(), Some(8));
        assert_eq!(AdsType::Real80.element_size(), Some(10));
        // C returns (size_t)-1 for these.
        assert_eq!(AdsType::WString.element_size(), None);
        assert_eq!(AdsType::BigType.element_size(), None);
        assert_eq!(AdsType::Unknown(99).element_size(), None);
    }

    #[test]
    fn ads_type_id_roundtrips() {
        for t in [
            AdsType::Void,
            AdsType::Int8,
            AdsType::UInt8,
            AdsType::Int16,
            AdsType::UInt16,
            AdsType::Int32,
            AdsType::UInt32,
            AdsType::Int64,
            AdsType::UInt64,
            AdsType::Real32,
            AdsType::Real64,
            AdsType::Real80,
            AdsType::BigType,
            AdsType::String,
            AdsType::WString,
            AdsType::Bit,
        ] {
            assert_eq!(AdsType::from_u32(t.to_u32()), t, "{t:?}");
        }
    }

    #[test]
    fn ads_state_unknown_sentinel_matches_c_string() {
        // C seeds amsPortInfo with ADSSTATE_MAXSTATES+1 == 21 and
        // adsStateToString falls through to the default.
        assert_eq!(AdsState::from_u16(21), AdsState::Unknown);
        assert_eq!(AdsState::from_u16(21).as_str(), "UNKNOWN ADSSTATE");
        assert_eq!(AdsState::from_u16(5), AdsState::Run);
    }

    #[test]
    fn error_strings_match_c_table() {
        assert_eq!(error_to_string(0x06), "GLOBALERR_TARGET_PORT");
        assert_eq!(error_to_string(0x0710), "ADSERR_DEVICE_SYMBOLNOTFOUND");
        assert_eq!(error_to_string(0x0745), "ADSERR_CLIENT_SYNCTIMEOUT");
        assert_eq!(error_to_string(0xDEAD), "ADSERR_ERROR_UNKNOWN");
    }
}
