//! `drvInfo` grammar — the string an EPICS record's INP/OUT link carries after
//! `@asyn(PORT,addr,timeout)`.
//!
//! ```text
//! [OPTION=VALUE/]... <address><?|=>
//! ```
//!
//! e.g. `ADSPORT=851/TIMEBASE=PLC/T_DLY_MS=500/TS_MS=10/Main.fTest?`
//!
//! The address is either a PLC symbol name (`Main.fTest`), an absolute
//! `.ADR.16#<group>,16#<offset>,<size>,<type>` command, or the driver-local
//! `.AMSPORTSTATE.` pseudo-variable. A trailing `?` means the record reads
//! (inputs, and outputs with readback); `=` means write-only.
//!
//! Divergence from C (upstream defect, fixed at source here): the C
//! `parsePlcInfofromDrvInfo` (adsAsynPortDriver.cpp:1557) locates every option
//! with `strstr(drvInfo, "TS_MS")` etc. over the *whole* drvInfo, so a PLC
//! symbol whose name contains an option keyword — `Main.TS_MS_setpoint`,
//! `Main.bADSPORT_OK` — is mis-parsed as if it carried that option, and the
//! following `sscanf("=%lf/")` either fails (returning `asynError`, so the
//! record never binds) or silently captures garbage. We split on `/` first and
//! only parse the leading segments as options, so the address text can never be
//! mistaken for one.

use std::fmt;

use crate::ads::defs::AdsType;

/// Where a parameter's data comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataSource {
    /// Normal case: the value lives in the PLC.
    Plc,
    /// `.AMSPORTSTATE.` — the ADS state of the AMS port, known to the driver
    /// itself, never read from the PLC.
    AmsState,
}

/// Which clock stamps the value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeBase {
    /// Timestamp delivered by the PLC (`TIMEBASE=PLC`, the C default).
    Plc,
    /// Timestamp taken in the IOC when the value arrives (`TIMEBASE=EPICS`).
    Epics,
}

/// The PLC address a parameter refers to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlcAddress {
    /// A symbol name, resolved to a handle via `SYM_HNDBYNAME`.
    Symbolic(String),
    /// `.ADR.16#<group>,16#<offset>,<size>,<type>` — absolute address, no
    /// symbol lookup, size and type given up front.
    Absolute {
        index_group: u32,
        index_offset: u32,
        size: u32,
        data_type: AdsType,
    },
    /// `.AMSPORTSTATE.` — driver-local.
    AmsPortState,
}

/// A parsed `drvInfo`.
#[derive(Debug, Clone, PartialEq)]
pub struct DrvInfo {
    /// Verbatim drvInfo, used as the asyn parameter name (C `createParam`).
    pub raw: String,
    pub address: PlcAddress,
    /// Address text as written, minus the trailing `?`/`=` (C `plcAdrStr`).
    pub address_str: String,
    /// `true` when the drvInfo ends in `?` — the record reads this parameter.
    pub has_input: bool,
    pub ams_port: u16,
    pub sample_time_ms: f64,
    pub max_delay_time_ms: f64,
    pub time_base: TimeBase,
    pub data_source: DataSource,
    /// `POLL_RATE=` given: read this parameter in the sum-up bulk read rather
    /// than subscribing to a device notification.
    pub is_bulk_read: bool,
    /// `POLL_RATE` value (C `pollClass`).
    pub poll_class: f64,
}

/// Defaults a `drvInfo` inherits from `adsAsynPortDriverConfigure`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DrvInfoDefaults {
    pub ams_port: u16,
    pub sample_time_ms: f64,
    pub max_delay_time_ms: f64,
    pub time_base: TimeBase,
}

/// Why a `drvInfo` could not be parsed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DrvInfoError {
    /// Neither `?` nor `=` terminates the string.
    MissingTerminator,
    /// The address part is empty.
    EmptyAddress,
    /// An option segment is not `KEY=VALUE`.
    MalformedOption(String),
    /// A known option's value did not parse.
    BadOptionValue { key: String, value: String },
    /// An option key the driver does not know.
    UnknownOption(String),
    /// `.ADR.` present but its four fields did not parse.
    BadAdrCommand(String),
}

impl fmt::Display for DrvInfoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingTerminator => {
                write!(f, "drvInfo must end with '?' (input) or '=' (output)")
            }
            Self::EmptyAddress => write!(f, "drvInfo has no PLC address"),
            Self::MalformedOption(s) => write!(f, "malformed option '{s}' (expected KEY=VALUE)"),
            Self::BadOptionValue { key, value } => {
                write!(f, "bad value '{value}' for option {key}")
            }
            Self::UnknownOption(k) => write!(f, "unknown option '{k}'"),
            Self::BadAdrCommand(s) => write!(
                f,
                "bad .ADR. command '{s}' \
                 (expected .ADR.16#<group>,16#<offset>,<size>,<type>)"
            ),
        }
    }
}

impl std::error::Error for DrvInfoError {}

const ADR_PREFIX: &str = ".ADR.";
const AMS_STATE_COMMAND: &str = ".AMSPORTSTATE.";

impl DrvInfo {
    /// Parse a drvInfo string against the port's configured defaults.
    pub fn parse(raw: &str, defaults: DrvInfoDefaults) -> Result<Self, DrvInfoError> {
        let (body, has_input) = match raw.chars().last() {
            Some('?') => (&raw[..raw.len() - 1], true),
            Some('=') => (&raw[..raw.len() - 1], false),
            _ => return Err(DrvInfoError::MissingTerminator),
        };

        // Split options from the address: everything before the last '/' is
        // options, the rest is the address (C takes the address after the last
        // '/' the same way, but hunts options across the whole string).
        let (options, address_str) = match body.rfind('/') {
            Some(i) => (&body[..i], &body[i + 1..]),
            None => ("", body),
        };
        if address_str.is_empty() {
            return Err(DrvInfoError::EmptyAddress);
        }

        let mut info = DrvInfo {
            raw: raw.to_string(),
            address: PlcAddress::Symbolic(address_str.to_string()),
            address_str: address_str.to_string(),
            has_input,
            ams_port: defaults.ams_port,
            sample_time_ms: defaults.sample_time_ms,
            max_delay_time_ms: defaults.max_delay_time_ms,
            time_base: defaults.time_base,
            data_source: DataSource::Plc,
            is_bulk_read: false,
            poll_class: 1.0,
        };

        for seg in options.split('/').filter(|s| !s.is_empty()) {
            let (key, value) = seg
                .split_once('=')
                .ok_or_else(|| DrvInfoError::MalformedOption(seg.to_string()))?;
            let bad = || DrvInfoError::BadOptionValue {
                key: key.to_string(),
                value: value.to_string(),
            };
            match key {
                "ADSPORT" => info.ams_port = value.parse().map_err(|_| bad())?,
                "TS_MS" => info.sample_time_ms = value.parse().map_err(|_| bad())?,
                "T_DLY_MS" => info.max_delay_time_ms = value.parse().map_err(|_| bad())?,
                "POLL_RATE" => {
                    info.is_bulk_read = true;
                    info.poll_class = value.parse().map_err(|_| bad())?;
                }
                "TIMEBASE" => {
                    info.time_base = match value {
                        "PLC" => TimeBase::Plc,
                        "EPICS" => TimeBase::Epics,
                        _ => return Err(bad()),
                    }
                }
                _ => return Err(DrvInfoError::UnknownOption(key.to_string())),
            }
        }

        if address_str == AMS_STATE_COMMAND {
            info.address = PlcAddress::AmsPortState;
            info.data_source = DataSource::AmsState;
            // The AMS port state is a driver-side UINT16, always stamped by the
            // IOC clock (adsAsynPortDriver.cpp:1794-1798).
            info.time_base = TimeBase::Epics;
        } else if let Some(rest) = address_str.strip_prefix(ADR_PREFIX) {
            info.address = parse_adr_command(rest)
                .ok_or_else(|| DrvInfoError::BadAdrCommand(address_str.to_string()))?;
        }

        Ok(info)
    }

    /// PLC data type when the drvInfo states it (`.ADR.`) or the driver owns it
    /// (`.AMSPORTSTATE.`); `None` when it must be read from the PLC via
    /// `SYM_INFOBYNAMEEX`.
    pub fn declared_type(&self) -> Option<(AdsType, u32)> {
        match &self.address {
            PlcAddress::Absolute {
                size, data_type, ..
            } => Some((*data_type, *size)),
            // adsAsynPortDriver.cpp:1795 — ADST_UINT16, 2 bytes.
            PlcAddress::AmsPortState => Some((AdsType::UInt16, 2)),
            PlcAddress::Symbolic(_) => None,
        }
    }
}

/// `16#<group>,16#<offset>,<size>,<type>` (C `sscanf("16#%x,16#%x,%u,%u")`).
fn parse_adr_command(rest: &str) -> Option<PlcAddress> {
    let mut parts = rest.split(',');
    let group = parse_hex16(parts.next()?)?;
    let offset = parse_hex16(parts.next()?)?;
    let size: u32 = parts.next()?.trim().parse().ok()?;
    let type_id: u32 = parts.next()?.trim().parse().ok()?;
    if parts.next().is_some() {
        return None;
    }
    Some(PlcAddress::Absolute {
        index_group: group,
        index_offset: offset,
        size,
        data_type: AdsType::from_u32(type_id),
    })
}

/// IEC `16#XXXX` hex literal.
fn parse_hex16(s: &str) -> Option<u32> {
    let hex = s.trim().strip_prefix("16#")?;
    u32::from_str_radix(hex, 16).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn defaults() -> DrvInfoDefaults {
        DrvInfoDefaults {
            ams_port: 851,
            sample_time_ms: 50.0,
            max_delay_time_ms: 100.0,
            time_base: TimeBase::Plc,
        }
    }

    fn parse(s: &str) -> DrvInfo {
        DrvInfo::parse(s, defaults()).unwrap()
    }

    #[test]
    fn bare_symbol_input_uses_defaults() {
        let d = parse("Main.fAmplitude?");
        assert_eq!(d.address, PlcAddress::Symbolic("Main.fAmplitude".into()));
        assert_eq!(d.address_str, "Main.fAmplitude");
        assert!(d.has_input);
        assert_eq!(d.ams_port, 851);
        assert_eq!(d.sample_time_ms, 50.0);
        assert_eq!(d.max_delay_time_ms, 100.0);
        assert_eq!(d.time_base, TimeBase::Plc);
        assert_eq!(d.data_source, DataSource::Plc);
        assert!(!d.is_bulk_read);
        // The param name is the verbatim drvInfo, terminator included.
        assert_eq!(d.raw, "Main.fAmplitude?");
    }

    #[test]
    fn trailing_equals_is_write_only() {
        let d = parse("ADSPORT=851/Main.fAmplitude=");
        assert!(!d.has_input);
        assert_eq!(d.address_str, "Main.fAmplitude");
    }

    #[test]
    fn missing_terminator_is_rejected() {
        assert_eq!(
            DrvInfo::parse("Main.fAmplitude", defaults()),
            Err(DrvInfoError::MissingTerminator)
        );
    }

    #[test]
    fn parses_full_option_list() {
        let d = parse("TIMEBASE=EPICS/T_DLY_MS=500/TS_MS=10/ADSPORT=852/Main.fTest?");
        assert_eq!(d.ams_port, 852);
        assert_eq!(d.sample_time_ms, 10.0);
        assert_eq!(d.max_delay_time_ms, 500.0);
        assert_eq!(d.time_base, TimeBase::Epics);
        assert_eq!(d.address_str, "Main.fTest");
    }

    #[test]
    fn poll_rate_selects_bulk_read() {
        let d = parse("ADSPORT=851/POLL_RATE=1.0/Main.fAmplitude?");
        assert!(d.is_bulk_read);
        assert_eq!(d.poll_class, 1.0);

        let d = parse("ADSPORT=851/Main.fAmplitude?");
        assert!(!d.is_bulk_read);
        assert_eq!(d.poll_class, 1.0);
    }

    /// The upstream defect this parser fixes: C `strstr(drvInfo, "TS_MS")`
    /// matches inside the *symbol name*, so these drvInfos mis-parse there.
    #[test]
    fn option_keyword_inside_symbol_name_is_not_an_option() {
        let d = parse("ADSPORT=851/Main.TS_MS_setpoint?");
        assert_eq!(d.address_str, "Main.TS_MS_setpoint");
        assert_eq!(d.sample_time_ms, 50.0, "must keep the port default");

        let d = parse("Main.bADSPORT_OK?");
        assert_eq!(d.address_str, "Main.bADSPORT_OK");
        assert_eq!(d.ams_port, 851, "must keep the port default");

        let d = parse("Main.stPOLL_RATE_cfg?");
        assert_eq!(d.address_str, "Main.stPOLL_RATE_cfg");
        assert!(!d.is_bulk_read, "must not be pulled into the bulk read");

        let d = parse("Main.fTIMEBASE?");
        assert_eq!(d.address_str, "Main.fTIMEBASE");
        assert_eq!(d.time_base, TimeBase::Plc);
    }

    #[test]
    fn ams_port_state_is_driver_local() {
        let d = parse("ADSPORT=851/.AMSPORTSTATE.?");
        assert_eq!(d.address, PlcAddress::AmsPortState);
        assert_eq!(d.data_source, DataSource::AmsState);
        // Forced regardless of the port default, which is PLC here.
        assert_eq!(d.time_base, TimeBase::Epics);
        assert_eq!(d.declared_type(), Some((AdsType::UInt16, 2)));
    }

    #[test]
    fn adr_command_parses_absolute_address() {
        let d = parse("ADSPORT=851/.ADR.16#4020,16#1A,2,2?");
        assert_eq!(
            d.address,
            PlcAddress::Absolute {
                index_group: 0x4020,
                index_offset: 0x1A,
                size: 2,
                data_type: AdsType::Int16,
            }
        );
        assert_eq!(d.declared_type(), Some((AdsType::Int16, 2)));
    }

    #[test]
    fn malformed_adr_command_is_rejected() {
        // C assigns -1 to the unsigned plcSize/plcDataType fields on this path
        // (adsAsynPortDriver.cpp:1622-1625), leaving SIZE_MAX behind before it
        // returns the error. Rejecting outright cannot leave that state.
        for bad in [
            ".ADR.16#4020,16#0,2?",         // too few fields
            ".ADR.16#4020,16#0,2,2,3?",     // too many
            ".ADR.4020,16#0,2,2?",          // missing 16# prefix
            ".ADR.16#XYZ,16#0,2,2?",        // bad hex
            ".ADR.16#4020,16#0,notanum,2?", // bad size
        ] {
            assert!(
                matches!(
                    DrvInfo::parse(bad, defaults()),
                    Err(DrvInfoError::BadAdrCommand(_))
                ),
                "{bad} should be rejected"
            );
        }
    }

    #[test]
    fn unknown_and_malformed_options_are_rejected() {
        assert_eq!(
            DrvInfo::parse("NOSUCH=1/Main.x?", defaults()),
            Err(DrvInfoError::UnknownOption("NOSUCH".into()))
        );
        assert_eq!(
            DrvInfo::parse("ADSPORT/Main.x?", defaults()),
            Err(DrvInfoError::MalformedOption("ADSPORT".into()))
        );
        assert_eq!(
            DrvInfo::parse("ADSPORT=abc/Main.x?", defaults()),
            Err(DrvInfoError::BadOptionValue {
                key: "ADSPORT".into(),
                value: "abc".into()
            })
        );
        assert_eq!(
            DrvInfo::parse("TIMEBASE=UTC/Main.x?", defaults()),
            Err(DrvInfoError::BadOptionValue {
                key: "TIMEBASE".into(),
                value: "UTC".into()
            })
        );
    }

    #[test]
    fn empty_address_is_rejected() {
        assert_eq!(
            DrvInfo::parse("ADSPORT=851/?", defaults()),
            Err(DrvInfoError::EmptyAddress)
        );
    }

    #[test]
    fn symbolic_address_has_no_declared_type() {
        assert_eq!(parse("Main.fTest?").declared_type(), None);
    }
}
