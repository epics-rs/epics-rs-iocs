//! The asynOctet ASCII command layer.
//!
//! The driver exposes a text protocol on parameter 0 ("Default access"), so a
//! `stringout`/`stringin` pair — or the ecmc motor record's `asynOctet` link —
//! can read and write arbitrary PLC symbols without a dedicated record. A line
//! is a sequence of commands separated by `;` (or, if the line has none, by a
//! space), and each command is one of:
//!
//! ```text
//! .THIS.sFeatures?                      → "ads"
//! [ADSPORT=<n>/]<symbol>?               → the symbol's value as text
//! [ADSPORT=<n>/]<symbol>=<value>        → "OK"
//! [ADSPORT=<n>/].ADR.16#<g>,16#<o>,<len>,<type>[=<value>]
//! ```
//!
//! This module owns the pure text↔bytes half (splitting, formatting, parsing);
//! the driver performs the ADS I/O in between.
//!
//! Upstream defects fixed at source (all in `adsAsynPortDriverUtils.cpp`):
//!
//! * **:818 / :850 — `"% PRId64"` / `"% PRIu64"`.** The `PRId64` macro sits
//!   *inside* the string literal, so it is never expanded: `printf` sees the
//!   format `"% PRId64"`, consumes ` ` and the invalid conversion ` P`, and the
//!   64-bit value is never printed. Reading an `LINT`/`ULINT` returns the
//!   literal text ` PRId64` instead of a number.
//! * **:837 / :845 — `"%d"` for `UINT16`/`UINT32`.** A `UDINT` above 2^31
//!   reaches `printf` as `%d` and prints negative (`4000000000` → `-294967296`).
//! * **adsAsynPortDriver.cpp:2229 — unterminated `strncpy`.** The symbolic
//!   *write* path copies the variable name with
//!   `strncpy(variableName, myarg_1, adr - myarg_1)` and never NUL-terminates,
//!   while the symbolic *read* path two arms below does
//!   (`variableName[adr - myarg_1] = 0`). The write buffer is a 255-byte stack
//!   array zeroed by `memset` at entry, so the bug is latent only while the name
//!   is shorter than the buffer — but the same buffer is reused across stacked
//!   commands within one line, so a long name followed by a short one leaves the
//!   previous name's tail attached. Slicing cannot reproduce the class of bug.

use std::fmt;

use crate::ads::defs::AdsType;

/// C `ADS_COM_ERROR_*` (adsAsynPortDriverUtils.h:37-40).
pub const ERROR_INVALID_DATA_TYPE: i32 = 1004;
pub const ERROR_BUFFER_INDEX_EXCEEDED_SIZE: i32 = 1005;
pub const ERROR_OCTET_ADSPORT_OPTION_FAIL: i32 = 1007;

/// The PLC struct the octet layer special-cases (`DUT_AxisStatus_v0_01`).
pub const DUT_AXIS_STATUS: &str = "DUT_AxisStatus_v0_01";

/// One parsed ASCII command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// `.THIS.sFeatures?` — answered by the driver itself.
    Features,
    /// `<symbol>?`
    ReadSymbol { ams_port: u16, name: String },
    /// `<symbol>=<value>`
    WriteSymbol {
        ams_port: u16,
        name: String,
        value: String,
    },
    /// `.ADR.16#<g>,16#<o>,<len>,<type>?`
    ReadAdr {
        ams_port: u16,
        index_group: u32,
        index_offset: u32,
        length: u32,
        data_type: AdsType,
    },
    /// `.ADR.16#<g>,16#<o>,<len>,<type>=<value>`
    WriteAdr {
        ams_port: u16,
        index_group: u32,
        index_offset: u32,
        length: u32,
        data_type: AdsType,
        value: String,
    },
    /// Anything else. C answers "Error: Bad command" and keeps going.
    Bad,
}

/// Text-layer failures.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OctetError {
    /// C `ADS_COM_ERROR_OCTET_ADSPORT_OPTION_FAIL`.
    BadAdsPort(String),
    /// C `ADS_COM_ERROR_INVALID_DATA_TYPE`.
    InvalidDataType(AdsType),
    /// The value text did not parse as the PLC's type.
    BadValue { text: String, plc: AdsType },
    /// C `ADS_COM_ERROR_ADS_READ_BUFFER_INDEX_EXCEEDED_SIZE`.
    BufferExceeded { size: usize, need: usize },
}

impl OctetError {
    /// The numeric code the C parser puts in its "Error: <n>" reply.
    pub fn code(&self) -> i32 {
        match self {
            Self::BadAdsPort(_) => ERROR_OCTET_ADSPORT_OPTION_FAIL,
            Self::InvalidDataType(_) | Self::BadValue { .. } => ERROR_INVALID_DATA_TYPE,
            Self::BufferExceeded { .. } => ERROR_BUFFER_INDEX_EXCEEDED_SIZE,
        }
    }
}

impl fmt::Display for OctetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadAdsPort(s) => write!(f, "ADS_COM_ERROR_OCTET_ADSPORT_OPTION_FAIL ({s})"),
            Self::InvalidDataType(t) => write!(f, "invalid data type {}", t.as_str()),
            Self::BadValue { text, plc } => {
                write!(f, "cannot parse '{}' as {}", text, plc.as_str())
            }
            Self::BufferExceeded { size, need } => {
                write!(f, "buffer of {size} bytes cannot hold {need}")
            }
        }
    }
}

impl std::error::Error for OctetError {}

/// Split a line into commands, keeping the separator that followed each.
///
/// C `octetCreateArgvSepv` (adsAsynPortDriverUtils.cpp:684): if the line
/// contains `;` that is the separator; otherwise, if it contains a space, that
/// is; otherwise the whole line is one command. The separators are echoed back
/// between the replies, so a `a?;b?` request answers `<a>;<b>`.
pub fn split_commands(line: &str) -> Vec<(String, &'static str)> {
    let sep = if line.contains(';') {
        ';'
    } else if line.contains(' ') {
        ' '
    } else {
        return if line.is_empty() {
            Vec::new()
        } else {
            vec![(line.to_string(), "")]
        };
    };
    let sep_str: &'static str = if sep == ';' { ";" } else { " " };

    let mut out = Vec::new();
    let parts: Vec<&str> = line.split(sep).collect();
    for (i, part) in parts.iter().enumerate() {
        // C stops at the first empty tail segment, so a trailing separator does
        // not produce an extra empty command.
        if part.is_empty() && i > 0 {
            break;
        }
        let last = i == parts.len() - 1;
        out.push((part.to_string(), if last { "" } else { sep_str }));
    }
    out
}

/// Parse one command. `default_ams_port` is the port's configured default.
pub fn parse_command(arg: &str, default_ams_port: u16) -> Result<Command, OctetError> {
    let mut ams_port = default_ams_port;
    let mut rest = arg;

    // Optional `ADSPORT=<n>/` prefix.
    if let Some(after) = rest.strip_prefix("ADSPORT=") {
        let (port_txt, tail) = after
            .split_once('/')
            .ok_or_else(|| OctetError::BadAdsPort(arg.to_string()))?;
        ams_port = port_txt
            .parse()
            .map_err(|_| OctetError::BadAdsPort(arg.to_string()))?;
        rest = tail;
    }

    if rest == ".THIS.sFeatures?" {
        return Ok(Command::Features);
    }

    if let Some(adr) = rest.strip_prefix(".ADR.") {
        return Ok(parse_adr(adr, ams_port).unwrap_or(Command::Bad));
    }

    // A symbolic write is `name=value`; a symbolic read is `name?`. C checks
    // '=' first, so `name=?` is a write of the text "?".
    if let Some((name, value)) = rest.split_once('=') {
        return Ok(Command::WriteSymbol {
            ams_port,
            name: name.to_string(),
            value: value.to_string(),
        });
    }
    if let Some((name, _)) = rest.split_once('?') {
        return Ok(Command::ReadSymbol {
            ams_port,
            name: name.to_string(),
        });
    }
    Ok(Command::Bad)
}

fn parse_adr(adr: &str, ams_port: u16) -> Option<Command> {
    // `16#<g>,16#<o>,<len>,<type>` then either `=<value>` or `?`.
    let (head, value) = match adr.split_once('=') {
        Some((h, v)) => (h, Some(v.to_string())),
        None => (adr.trim_end_matches('?'), None),
    };
    let mut parts = head.split(',');
    let group = u32::from_str_radix(parts.next()?.trim().strip_prefix("16#")?, 16).ok()?;
    let offset = u32::from_str_radix(parts.next()?.trim().strip_prefix("16#")?, 16).ok()?;
    let length: u32 = parts.next()?.trim().parse().ok()?;
    let type_id: u32 = parts.next()?.trim().parse().ok()?;
    if parts.next().is_some() {
        return None;
    }
    let data_type = AdsType::from_u32(type_id);
    Some(match value {
        Some(value) => Command::WriteAdr {
            ams_port,
            index_group: group,
            index_offset: offset,
            length,
            data_type,
            value,
        },
        None => Command::ReadAdr {
            ams_port,
            index_group: group,
            index_offset: offset,
            length,
            data_type,
        },
    })
}

/// PLC bytes → the ASCII reply text (C `octetBinary2ascii`).
///
/// Arrays are rendered as comma-separated elements. `type_name` is the symbol's
/// PLC type name, used only to recognize `DUT_AxisStatus_v0_01`.
pub fn binary_to_ascii(
    data: &[u8],
    plc: AdsType,
    size: usize,
    name: &str,
    type_name: &str,
) -> Result<String, OctetError> {
    if plc == AdsType::BigType {
        if type_name.contains(DUT_AXIS_STATUS) {
            return axis_status_to_ascii(data, name);
        }
        return Err(OctetError::InvalidDataType(plc));
    }

    if plc == AdsType::String {
        let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
        return Ok(String::from_utf8_lossy(&data[..end]).into_owned());
    }

    let elem = text_width(plc).ok_or(OctetError::InvalidDataType(plc))?;
    let total = size.min(data.len());
    if total < elem {
        return Err(OctetError::BufferExceeded {
            size: data.len(),
            need: elem,
        });
    }

    let mut out = String::new();
    for (i, c) in data[..total].chunks_exact(elem).enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str(&format_element(plc, c));
    }
    Ok(out)
}

/// Byte width of a PLC type the ASCII layer can render one element of.
///
/// `None` for every type the C `binary2ascii`/`ascii2binary` switches leave to
/// their `default:` arm (`ADS_COM_ERROR_INVALID_DATA_TYPE`). `REAL80` matters
/// here: it has a defined element size (10 bytes) yet no `printf` arm, so
/// sizing off `element_size` alone would walk into a formatter that has no case
/// for it.
fn text_width(plc: AdsType) -> Option<usize> {
    match plc {
        AdsType::Int8 | AdsType::UInt8 | AdsType::Bit => Some(1),
        AdsType::Int16 | AdsType::UInt16 => Some(2),
        AdsType::Int32 | AdsType::UInt32 | AdsType::Real32 => Some(4),
        AdsType::Int64 | AdsType::UInt64 | AdsType::Real64 => Some(8),
        _ => None,
    }
}

fn format_element(plc: AdsType, c: &[u8]) -> String {
    match plc {
        AdsType::Int8 => (c[0] as i8).to_string(),
        AdsType::UInt8 => c[0].to_string(),
        // C prints "1" only for an exact 1; TwinCAT sends 1 for TRUE, and any
        // other non-zero byte is still TRUE, so test against zero.
        AdsType::Bit => u8::from(c[0] != 0).to_string(),
        AdsType::Int16 => i16::from_le_bytes([c[0], c[1]]).to_string(),
        AdsType::UInt16 => u16::from_le_bytes([c[0], c[1]]).to_string(),
        AdsType::Int32 => i32::from_le_bytes([c[0], c[1], c[2], c[3]]).to_string(),
        AdsType::UInt32 => u32::from_le_bytes([c[0], c[1], c[2], c[3]]).to_string(),
        AdsType::Int64 => i64::from_le_bytes(c.try_into().unwrap()).to_string(),
        AdsType::UInt64 => u64::from_le_bytes(c.try_into().unwrap()).to_string(),
        // C's "%f" / "%lf": six digits after the point.
        AdsType::Real32 => format!("{:.6}", f32::from_le_bytes([c[0], c[1], c[2], c[3]])),
        AdsType::Real64 => format!("{:.6}", f64::from_le_bytes(c.try_into().unwrap())),
        _ => unreachable!("text_width admitted only the types handled above"),
    }
}

/// ASCII text → PLC bytes (C `octetAscii2binary`).
///
/// Comma-separated elements fill an array; `buf_size` is the PLC variable's
/// size and caps the result, as in C.
pub fn ascii_to_binary(text: &str, plc: AdsType, buf_size: usize) -> Result<Vec<u8>, OctetError> {
    if plc == AdsType::String {
        let mut out = text.as_bytes().to_vec();
        out.truncate(buf_size);
        // Keep room for the terminator the PLC expects on a STRING.
        if out.len() == buf_size && buf_size > 0 {
            out[buf_size - 1] = 0;
        } else {
            out.push(0);
        }
        return Ok(out);
    }

    let elem = text_width(plc).ok_or(OctetError::InvalidDataType(plc))?;

    let mut out = Vec::new();
    for field in text.split(',') {
        let t = field.trim();
        let bytes = parse_element(plc, t).ok_or_else(|| OctetError::BadValue {
            text: t.to_string(),
            plc,
        })?;
        if out.len() + elem > buf_size {
            return Err(OctetError::BufferExceeded {
                size: buf_size,
                need: out.len() + elem,
            });
        }
        out.extend_from_slice(&bytes);
    }
    Ok(out)
}

fn parse_element(plc: AdsType, t: &str) -> Option<Vec<u8>> {
    Some(match plc {
        AdsType::Int8 => t.parse::<i8>().ok()?.to_le_bytes().to_vec(),
        AdsType::UInt8 => t.parse::<u8>().ok()?.to_le_bytes().to_vec(),
        // C reads a BIT with "%hhu": the text is a number, not "true"/"false".
        AdsType::Bit => vec![u8::from(t.parse::<u8>().ok()? != 0)],
        AdsType::Int16 => t.parse::<i16>().ok()?.to_le_bytes().to_vec(),
        AdsType::UInt16 => t.parse::<u16>().ok()?.to_le_bytes().to_vec(),
        AdsType::Int32 => t.parse::<i32>().ok()?.to_le_bytes().to_vec(),
        AdsType::UInt32 => t.parse::<u32>().ok()?.to_le_bytes().to_vec(),
        AdsType::Int64 => t.parse::<i64>().ok()?.to_le_bytes().to_vec(),
        AdsType::UInt64 => t.parse::<u64>().ok()?.to_le_bytes().to_vec(),
        AdsType::Real32 => t.parse::<f32>().ok()?.to_le_bytes().to_vec(),
        AdsType::Real64 => t.parse::<f64>().ok()?.to_le_bytes().to_vec(),
        _ => return None,
    })
}

/// `DUT_AxisStatus_v0_01`, as the C octet layer renders it.
///
/// Field offsets are those of the C struct (adsAsynPortDriverUtils.cpp:34) under
/// natural alignment — three `char`s, one pad byte, two `uint16_t`, then the
/// 8-byte-aligned doubles. Naming the offsets keeps the padding explicit rather
/// than implied by a `#[repr(C)]` cast over PLC-supplied bytes.
const AXIS_STATUS_SIZE: usize = 96;

fn axis_status_to_ascii(data: &[u8], name: &str) -> Result<String, OctetError> {
    if data.len() < AXIS_STATUS_SIZE {
        return Err(OctetError::BufferExceeded {
            size: data.len(),
            need: AXIS_STATUS_SIZE,
        });
    }
    let b = |off: usize| u8::from(data[off] != 0);
    let u16_at = |off: usize| u16::from_le_bytes([data[off], data[off + 1]]);
    let u32_at =
        |off: usize| u32::from_le_bytes([data[off], data[off + 1], data[off + 2], data[off + 3]]);
    let f64_at = |off: usize| f64::from_le_bytes(data[off..off + 8].try_into().unwrap());

    let mut s = format!("{name}=");
    s.push_str(&format!("{},", b(0))); // bEnable
    s.push_str(&format!("{},", b(1))); // bReset
    s.push_str(&format!("{},", b(2))); // bExecute
    s.push_str(&format!("{},", u16_at(4))); // nCommand   (1 pad byte at 3)
    s.push_str(&format!("{},", u16_at(6))); // nCmdData
    s.push_str(&format!("{:.6},", f64_at(8))); // fVelocity
    s.push_str(&format!("{:.6},", f64_at(16))); // fPosition
    s.push_str(&format!("{:.6},", f64_at(24))); // fAcceleration
    s.push_str(&format!("{:.6},", f64_at(32))); // fDeceleration
    s.push_str(&format!("{},", b(40))); // bJogFwd
    s.push_str(&format!("{},", b(41))); // bJogBwd
    s.push_str(&format!("{},", b(42))); // bLimitFwd
    s.push_str(&format!("{},", b(43))); // bLimitBwd
    s.push_str(&format!("{:.6},", f64_at(48))); // fOverride  (4 pad bytes at 44)
    s.push_str(&format!("{},", b(56))); // bHomeSensor
    s.push_str(&format!("{},", b(57))); // bEnabled
    s.push_str(&format!("{},", b(58))); // bError
    s.push_str(&format!("{},", u32_at(60))); // nErrorId   (1 pad byte at 59)
    s.push_str(&format!("{:.6},", f64_at(64))); // fActVelocity
    s.push_str(&format!("{:.6},", f64_at(72))); // fActPosition
    s.push_str(&format!("{:.6},", f64_at(80))); // fActDiff
    s.push_str(&format!("{},", b(88))); // bHomed
    s.push_str(&format!("{};", b(89))); // bBusy — C ends this one with ';'
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn semicolon_beats_space_as_separator() {
        assert_eq!(
            split_commands("a?;b?"),
            vec![("a?".to_string(), ";"), ("b?".to_string(), "")]
        );
        assert_eq!(
            split_commands("a? b?"),
            vec![("a?".to_string(), " "), ("b?".to_string(), "")]
        );
        // With both present, ';' wins and the spaces stay inside the commands.
        assert_eq!(
            split_commands("a b;c"),
            vec![("a b".to_string(), ";"), ("c".to_string(), "")]
        );
    }

    #[test]
    fn a_line_without_separators_is_one_command() {
        assert_eq!(
            split_commands("Main.fTest?"),
            vec![("Main.fTest?".to_string(), "")]
        );
        assert!(split_commands("").is_empty());
    }

    #[test]
    fn trailing_separator_does_not_add_an_empty_command() {
        assert_eq!(
            split_commands("a?;"),
            vec![("a?".to_string(), ";")],
            "C's loop breaks on the empty tail"
        );
    }

    #[test]
    fn parses_features_command() {
        assert_eq!(
            parse_command(".THIS.sFeatures?", 851).unwrap(),
            Command::Features
        );
    }

    #[test]
    fn parses_symbolic_read_and_write() {
        assert_eq!(
            parse_command("Main.fTest?", 851).unwrap(),
            Command::ReadSymbol {
                ams_port: 851,
                name: "Main.fTest".into()
            }
        );
        assert_eq!(
            parse_command("Main.fTest=1.5", 851).unwrap(),
            Command::WriteSymbol {
                ams_port: 851,
                name: "Main.fTest".into(),
                value: "1.5".into()
            }
        );
    }

    #[test]
    fn adsport_prefix_overrides_the_default_port() {
        assert_eq!(
            parse_command("ADSPORT=852/Main.fTest?", 851).unwrap(),
            Command::ReadSymbol {
                ams_port: 852,
                name: "Main.fTest".into()
            }
        );
        assert_eq!(
            parse_command("ADSPORT=xyz/Main.fTest?", 851),
            Err(OctetError::BadAdsPort("ADSPORT=xyz/Main.fTest?".into()))
        );
        // No '/' after the port number.
        assert!(matches!(
            parse_command("ADSPORT=852", 851),
            Err(OctetError::BadAdsPort(_))
        ));
    }

    /// The C write path copies the name with a `strncpy` that never terminates,
    /// into a buffer reused across the commands of one line. A long name
    /// followed by a short one leaves the long name's tail behind.
    #[test]
    fn a_short_write_after_a_long_one_does_not_inherit_the_previous_name() {
        let cmds: Vec<Command> = split_commands("Main.aVeryLongVariableName=1;Main.x=2")
            .iter()
            .map(|(c, _)| parse_command(c, 851).unwrap())
            .collect();
        assert_eq!(
            cmds[1],
            Command::WriteSymbol {
                ams_port: 851,
                name: "Main.x".into(),
                value: "2".into()
            }
        );
    }

    #[test]
    fn parses_adr_read_and_write() {
        assert_eq!(
            parse_command(".ADR.16#4020,16#1A,2,2?", 851).unwrap(),
            Command::ReadAdr {
                ams_port: 851,
                index_group: 0x4020,
                index_offset: 0x1A,
                length: 2,
                data_type: AdsType::Int16,
            }
        );
        assert_eq!(
            parse_command("ADSPORT=852/.ADR.16#4020,16#0,8,5=1.25", 851).unwrap(),
            Command::WriteAdr {
                ams_port: 852,
                index_group: 0x4020,
                index_offset: 0,
                length: 8,
                data_type: AdsType::Real64,
                value: "1.25".into(),
            }
        );
    }

    #[test]
    fn a_bad_command_is_reported_not_fatal() {
        assert_eq!(parse_command("gibberish", 851).unwrap(), Command::Bad);
        assert_eq!(parse_command(".ADR.nonsense?", 851).unwrap(), Command::Bad);
    }

    /// C prints an LINT with the format string `"% PRId64"`, which never
    /// expands: the value is dropped and the literal text is echoed.
    #[test]
    fn int64_prints_its_value() {
        let v = -9_007_199_254_740_993i64;
        let s = binary_to_ascii(&v.to_le_bytes(), AdsType::Int64, 8, "x", "LINT").unwrap();
        assert_eq!(s, "-9007199254740993");
        assert!(!s.contains("PRId64"));
    }

    #[test]
    fn uint64_prints_its_value() {
        let v = u64::MAX;
        let s = binary_to_ascii(&v.to_le_bytes(), AdsType::UInt64, 8, "x", "ULINT").unwrap();
        assert_eq!(s, "18446744073709551615");
        assert!(!s.contains("PRIu64"));
    }

    /// C passes a `uint32_t` to `%d`, so anything above INT32_MAX prints signed.
    #[test]
    fn large_unsigned_values_print_unsigned() {
        let s = binary_to_ascii(
            &4_000_000_000u32.to_le_bytes(),
            AdsType::UInt32,
            4,
            "x",
            "UDINT",
        )
        .unwrap();
        assert_eq!(s, "4000000000");

        let s = binary_to_ascii(&65535u16.to_le_bytes(), AdsType::UInt16, 2, "x", "UINT").unwrap();
        assert_eq!(s, "65535");
    }

    #[test]
    fn reals_print_with_six_decimals_like_printf() {
        let s = binary_to_ascii(&1.5f64.to_le_bytes(), AdsType::Real64, 8, "x", "LREAL").unwrap();
        assert_eq!(s, "1.500000");
        let s =
            binary_to_ascii(&(-0.25f32).to_le_bytes(), AdsType::Real32, 4, "x", "REAL").unwrap();
        assert_eq!(s, "-0.250000");
    }

    #[test]
    fn arrays_are_comma_separated() {
        let data: Vec<u8> = [1i32, -2, 3].iter().flat_map(|v| v.to_le_bytes()).collect();
        let s = binary_to_ascii(&data, AdsType::Int32, 12, "x", "ARRAY").unwrap();
        assert_eq!(s, "1,-2,3");
    }

    #[test]
    fn strings_stop_at_the_nul() {
        let s = binary_to_ascii(b"Hello\0junk", AdsType::String, 10, "x", "STRING(9)").unwrap();
        assert_eq!(s, "Hello");
    }

    #[test]
    fn bit_prints_zero_or_one() {
        assert_eq!(
            binary_to_ascii(&[0], AdsType::Bit, 1, "x", "BOOL").unwrap(),
            "0"
        );
        assert_eq!(
            binary_to_ascii(&[1], AdsType::Bit, 1, "x", "BOOL").unwrap(),
            "1"
        );
    }

    #[test]
    fn unsupported_types_are_rejected() {
        // REAL80 has a defined element size (10 bytes) but no printf arm in C.
        // Sizing off `element_size` would reach a formatter with no case for it.
        assert_eq!(
            binary_to_ascii(&[0; 10], AdsType::Real80, 10, "x", "?"),
            Err(OctetError::InvalidDataType(AdsType::Real80))
        );
        assert_eq!(
            ascii_to_binary("1.0", AdsType::Real80, 10),
            Err(OctetError::InvalidDataType(AdsType::Real80))
        );
        assert_eq!(
            binary_to_ascii(&[0; 8], AdsType::WString, 8, "x", "?"),
            Err(OctetError::InvalidDataType(AdsType::WString))
        );
        // BIGTYPE that is not the known DUT.
        assert_eq!(
            binary_to_ascii(&[0; 8], AdsType::BigType, 8, "x", "ST_Other"),
            Err(OctetError::InvalidDataType(AdsType::BigType))
        );
    }

    #[test]
    fn ascii_to_binary_roundtrips_scalars() {
        assert_eq!(
            ascii_to_binary("-2", AdsType::Int16, 2).unwrap(),
            (-2i16).to_le_bytes()
        );
        assert_eq!(
            ascii_to_binary("1.25", AdsType::Real64, 8).unwrap(),
            1.25f64.to_le_bytes()
        );
        assert_eq!(ascii_to_binary("1", AdsType::Bit, 1).unwrap(), vec![1]);
        assert_eq!(ascii_to_binary("0", AdsType::Bit, 1).unwrap(), vec![0]);
    }

    #[test]
    fn ascii_to_binary_fills_arrays() {
        assert_eq!(
            ascii_to_binary("1,2,3", AdsType::Int16, 6).unwrap(),
            [1i16, 2, 3]
                .iter()
                .flat_map(|v| v.to_le_bytes())
                .collect::<Vec<u8>>()
        );
    }

    #[test]
    fn ascii_to_binary_refuses_to_overrun_the_plc_variable() {
        // Three INTs into a 4-byte (2-element) PLC array.
        assert!(matches!(
            ascii_to_binary("1,2,3", AdsType::Int16, 4),
            Err(OctetError::BufferExceeded { .. })
        ));
    }

    #[test]
    fn ascii_to_binary_rejects_unparseable_text() {
        assert_eq!(
            ascii_to_binary("abc", AdsType::Int32, 4),
            Err(OctetError::BadValue {
                text: "abc".into(),
                plc: AdsType::Int32
            })
        );
        // Out of range for the PLC type.
        assert!(matches!(
            ascii_to_binary("70000", AdsType::UInt16, 2),
            Err(OctetError::BadValue { .. })
        ));
    }

    #[test]
    fn ascii_to_binary_nul_terminates_a_string() {
        assert_eq!(ascii_to_binary("Hi", AdsType::String, 10).unwrap(), b"Hi\0");
        // Exactly filling the slot still leaves room for the terminator.
        let out = ascii_to_binary("abcd", AdsType::String, 4).unwrap();
        assert_eq!(out, b"abc\0");
    }

    #[test]
    fn axis_status_renders_every_field_at_its_padded_offset() {
        let mut d = vec![0u8; AXIS_STATUS_SIZE];
        d[0] = 1; // bEnable
        d[2] = 1; // bExecute
        d[4..6].copy_from_slice(&3u16.to_le_bytes()); // nCommand
        d[6..8].copy_from_slice(&4u16.to_le_bytes()); // nCmdData
        d[8..16].copy_from_slice(&10.5f64.to_le_bytes()); // fVelocity
        d[48..56].copy_from_slice(&100.0f64.to_le_bytes()); // fOverride
        d[60..64].copy_from_slice(&42u32.to_le_bytes()); // nErrorId
        d[72..80].copy_from_slice(&(-1.5f64).to_le_bytes()); // fActPosition
        d[89] = 1; // bBusy

        let s = axis_status_to_ascii(&d, "ax1").unwrap();
        assert_eq!(
            s,
            "ax1=1,0,1,3,4,10.500000,0.000000,0.000000,0.000000,0,0,0,0,\
             100.000000,0,0,0,42,0.000000,-1.500000,0.000000,0,1;"
        );
    }

    #[test]
    fn axis_status_is_reached_through_the_type_name() {
        let d = vec![0u8; AXIS_STATUS_SIZE];
        let s = binary_to_ascii(
            &d,
            AdsType::BigType,
            AXIS_STATUS_SIZE,
            "ax1",
            "DUT_AxisStatus_v0_01",
        )
        .unwrap();
        assert!(s.starts_with("ax1=0,0,0,"));
        assert!(s.ends_with(';'));
    }

    #[test]
    fn a_truncated_axis_status_is_rejected() {
        assert!(matches!(
            axis_status_to_ascii(&[0u8; 50], "ax1"),
            Err(OctetError::BufferExceeded { need: 96, .. })
        ));
    }
}
