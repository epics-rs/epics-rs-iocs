//! Pure command-string builders and response parsers for the GM10
//! protocol. Every parser here operates on an ASCII payload that has
//! already had [`crate::wire::ascii_payload`]'s leading-4-byte-header /
//! trailing-`"EN\r\n"` strip applied (matching every `drvGM10.c` consumer's
//! own `ptr += 4`), or on a full raw binary frame (`FData`, which indexes
//! from the frame start).

use crate::cache::ModuleType;

// ---------------------------------------------------------------------
// Command builders
// ---------------------------------------------------------------------

pub fn cmd_fsysconf() -> String {
    "FSysConf\r\n".to_string()
}
pub fn cmd_orec_query() -> String {
    "ORec?\r\n".to_string()
}
pub fn cmd_omath_query() -> String {
    "OMath?\r\n".to_string()
}
pub fn cmd_fchinfo() -> String {
    "FChInfo\r\n".to_string()
}
pub fn cmd_srangeao_query() -> String {
    "SRangeAO?\r\n".to_string()
}
pub fn cmd_srangedo_query() -> String {
    "SRangeDO?\r\n".to_string()
}
pub fn cmd_srangemath_query() -> String {
    "SRangeMath?\r\n".to_string()
}
pub fn cmd_fdata_all() -> String {
    "FData,1\r\n".to_string()
}
pub fn cmd_fdata_signal(channel: u32) -> String {
    format!("FData,1,{channel:04},{channel:04}\r\n")
}
pub fn cmd_fdata_math(channel: u32) -> String {
    format!("FData,1,A{channel:03},A{channel:03}\r\n")
}
pub fn cmd_fdata_comm(channel: u32) -> String {
    format!("FData,1,C{channel:03},C{channel:03}\r\n")
}
pub fn cmd_skconst_query_all() -> String {
    "SKConst?\r\n".to_string()
}
pub fn cmd_skconst_query(channel: u32) -> String {
    format!("SKConst,{channel}?\r\n")
}
pub fn cmd_swconst_query_all() -> String {
    "SWConst?\r\n".to_string()
}
pub fn cmd_swconst_query(channel: u32) -> String {
    format!("SWConst,{channel}?\r\n")
}
/// `set_output_value(CMD_SET_SIGNAL_OUTPUT, ...)` (`drvGM10.c:1164`):
/// value sent as a fixed 3-decimal integer (`value * 1000`).
pub fn cmd_ocmdao(channel: u32, value: f64) -> String {
    format!("OCmdAO,{channel:04},{}\r\n", (value * 1000.0) as i64)
}
pub fn cmd_ocommch(channel: u32, value: f64) -> String {
    format!("OCommCh,C{channel:03},{}\r\n", format_g(value))
}
pub fn cmd_skconst_set(channel: u32, value: f64) -> String {
    format!("SKConst,{channel},{}\r\n", format_g(value))
}
pub fn cmd_swconst_set(channel: u32, value: f64) -> String {
    format!("SWConst,{channel},{}\r\n", format_g(value))
}
pub fn cmd_ocmdrelay(channel: u32, on: bool) -> String {
    format!(
        "OCmdRelay,{channel:04}-{}\r\n",
        if on { "On" } else { "Off" }
    )
}
pub fn cmd_orec_set(on: bool) -> String {
    format!("ORec,{}\r\n", if on { '1' } else { '0' })
}
/// `set_mode(CMD_SET_COMPUTE, ...)` (`drvGM10.c:1222`): `value` is 0-3.
pub fn cmd_omath_set(mode: u8) -> String {
    format!("OMath,{}\r\n", (b'0' + (mode & 0x3)) as char)
}
pub fn cmd_oerrorclear() -> String {
    "OErrorClear,0\r\n".to_string()
}
pub fn cmd_oalarmack() -> String {
    "OAlarmAck,0\r\n".to_string()
}
pub fn cmd_err_query(code: i32) -> String {
    format!("_ERR,{code}\r\n")
}

/// C `sprintf("%G", value)` — default precision (6 significant digits),
/// trailing zeros stripped (`drvGM10.c:1168,1171,1174`).
pub fn format_g(value: f64) -> String {
    if value == 0.0 {
        return "0".to_string();
    }
    const PRECISION: i32 = 6;
    let exponent = value.abs().log10().floor() as i32;
    if !(-4..PRECISION).contains(&exponent) {
        let digits = (PRECISION - 1).max(0) as usize;
        let s = format!("{value:.digits$e}");
        let (mantissa, exp) = s.split_once('e').expect("Rust {:e} always emits 'e'");
        let mantissa = strip_trailing_zeros(mantissa);
        let exp: i32 = exp
            .parse()
            .expect("Rust {:e} exponent is always an integer");
        format!("{mantissa}E{exp:+03}")
    } else {
        let decimals = (PRECISION - 1 - exponent).max(0) as usize;
        strip_trailing_zeros(&format!("{value:.decimals$}"))
    }
}

fn strip_trailing_zeros(s: &str) -> String {
    if !s.contains('.') {
        return s.to_string();
    }
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

// ---------------------------------------------------------------------
// Error decode (`response_reader`, `drvGM10.c:360-413`)
// ---------------------------------------------------------------------

/// `sscanf(inbuffer, "E1,%d:1:%d\r\n", &code, &parameter) >= 1` — `code` is
/// mandatory, `parameter` defaults to 0 if the `:1:` suffix is absent or
/// malformed (matching C's tolerant `>= 1` accept).
pub fn parse_error_header(payload: &str) -> Option<(i32, i32)> {
    let rest = payload.strip_prefix("E1,")?;
    let code_end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    if code_end == 0 {
        return None;
    }
    let code: i32 = rest[..code_end].parse().ok()?;
    let param = rest[code_end..]
        .strip_prefix(":1:")
        .and_then(|p| {
            let end = p.find(|c: char| !c.is_ascii_digit()).unwrap_or(p.len());
            p[..end].parse().ok()
        })
        .unwrap_or(0);
    Some((code, param))
}

/// `chopstring40` (`drvGM10.c:330-356`): split off a chunk of at most 39
/// chars, breaking at the last space within the first 40 chars when `src`
/// is 40 chars or longer. Returns `(chunk, remainder)`.
pub fn chop_string_40(src: &str) -> (String, Option<String>) {
    if src.len() < 40 {
        return (src.to_string(), None);
    }
    let bytes = src.as_bytes();
    let mut i = 39usize;
    loop {
        if bytes[i] == b' ' {
            return (src[..i].to_string(), Some(src[i + 1..].to_string()));
        }
        if i == 0 {
            return (src[..39].to_string(), Some(src[39..].to_string()));
        }
        i -= 1;
    }
}

/// The `_ERR,<code>` reply's single-quoted message, chopped into up to 3
/// 40-char-max chunks (`response_reader`, `drvGM10.c:384-403`).
pub fn split_error_message(msg: &str) -> [String; 3] {
    let mut out = [String::new(), String::new(), String::new()];
    let (first, rest) = chop_string_40(msg);
    out[0] = first;
    if let Some(rest) = rest {
        let (second, rest2) = chop_string_40(&rest);
        out[1] = second;
        if let Some(rest2) = rest2 {
            out[2] = chop_string_40(&rest2).0;
        }
    }
    out
}

/// Extract the single-quoted message body from an `_ERR,<code>` ASCII
/// reply payload (`drvGM10.c:384-388`: `ptr = strchr(ptr,'\'')+1; eptr =
/// strchr(ptr,'\'')`).
pub fn extract_quoted_message(payload: &str) -> Option<&str> {
    let start = payload.find('\'')? + 1;
    let rest = &payload[start..];
    let end = rest.find('\'')?;
    Some(&rest[..end])
}

// ---------------------------------------------------------------------
// FSysConf (`load_modules`, `drvGM10.c:416-579`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModuleLine {
    pub index: usize,
    pub set_message: String,
    pub status_message: String,
    pub error_message: String,
}

const NO_ERROR: &str = "----------------";

/// Parses the `FSysConf` ASCII body (after the `"Unit:00"` prefix check).
/// Returns `None` on a malformed body (matches C `load_modules`'s `return 1`
/// paths), matching the required `"Unit:00"` header.
pub fn parse_fsysconf(payload: &[u8]) -> Option<Vec<ModuleLine>> {
    if payload.len() < 9 || &payload[0..7] != b"Unit:00" {
        return None;
    }
    let mut p = 9usize;
    let mut modules = Vec::new();
    while p < payload.len() && payload[p] != b'E' {
        if p + 3 > payload.len() {
            return None;
        }
        if !payload[p].is_ascii_digit() || !payload[p + 1].is_ascii_digit() {
            return None;
        }
        let which = 10 * usize::from(payload[p] - b'0') + usize::from(payload[p + 1] - b'0');
        p += 3;

        let (set_message, np) = read_field_17(payload, p)?;
        p = np;
        let (status_message, np) = read_field_17(payload, p)?;
        p = np;
        let (error_message, np) = read_field_until(payload, p, b'\r')?;
        p = np + 2;

        modules.push(ModuleLine {
            index: which,
            set_message,
            status_message,
            error_message,
        });
    }
    Some(modules)
}

/// A fixed-17-byte-wide field: text up to the first space, then padding to
/// fill out the 17-byte slot (`drvGM10.c:459-463`: `ptr += (17 - i)` after
/// a copy loop that already advanced by `i`, so the field is 17 bytes wide
/// in total regardless of the string's own length).
fn read_field_17(payload: &[u8], start: usize) -> Option<(String, usize)> {
    let mut i = 0usize;
    while start + i < payload.len() && payload[start + i] != b' ' {
        i += 1;
        if i > 16 {
            return None;
        }
    }
    let s = String::from_utf8_lossy(&payload[start..start + i]).into_owned();
    Some((s, start + 17))
}

fn read_field_until(payload: &[u8], start: usize, terminator: u8) -> Option<(String, usize)> {
    let mut i = 0usize;
    while start + i < payload.len() && payload[start + i] != terminator {
        i += 1;
    }
    if start + i >= payload.len() {
        return None;
    }
    let s = String::from_utf8_lossy(&payload[start..start + i]).into_owned();
    Some((s, start + i))
}

/// Whether a parsed [`ModuleLine`] indicates a present, error-free module
/// (`drvGM10.c:490-496`: set == status, no error, and set isn't itself the
/// all-dashes "unset" sentinel).
pub fn module_line_ok(line: &ModuleLine) -> bool {
    line.set_message == line.status_message
        && line.error_message == NO_ERROR
        && line.set_message != NO_ERROR
}

/// Decode a module type string (`"GX90{U,X,Y,W}{T,A,D,P}-<n>"`) into its
/// type and `[input_count, output_count]` (`drvGM10.c:500-574`). Returns
/// `None` for anything not matching the `GX90` + recognized-subtype +
/// `-`-separator shape (C's `continue` paths).
pub fn classify_module(set_message: &str) -> Option<(ModuleType, [i32; 2])> {
    let rest = set_message.strip_prefix("GX90")?;
    let mut chars = rest.chars();
    let c0 = chars.next()?;
    let c1 = chars.next()?;
    let module_type = match (c0, c1) {
        ('U', 'T') => ModuleType::Pid,
        ('X', 'A') => ModuleType::InputAnalog,
        ('X', 'D') => ModuleType::InputDigital,
        ('X', 'P') => ModuleType::InputPulse,
        ('Y', 'A') => ModuleType::OutputAnalog,
        ('Y', 'D') => ModuleType::OutputDigital,
        ('W', 'D') => ModuleType::InputOutputDigital,
        _ => return None,
    };
    let after_subtype = chars.as_str();
    let after_dash = after_subtype.strip_prefix('-')?;
    if module_type == ModuleType::Pid {
        // C `continue`s before assigning channel_number for PID/UNKNOWN.
        return Some((module_type, [0, 0]));
    }
    let digits_end = after_dash
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(after_dash.len());
    let v: i32 = after_dash[..digits_end].parse().ok()?;
    let channel_number = match module_type {
        ModuleType::InputAnalog | ModuleType::InputDigital | ModuleType::InputPulse => [v, 0],
        ModuleType::OutputAnalog | ModuleType::OutputDigital => [0, v],
        ModuleType::InputOutputDigital => [v / 100, v % 100],
        ModuleType::Pid | ModuleType::Unknown => [0, 0],
    };
    Some((module_type, channel_number))
}

// ---------------------------------------------------------------------
// FChInfo (`load_infos`, `drvGM10.c:661-771`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InfoFamily {
    Signal,
    Math,
    Comm,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelInfoLine {
    pub family: InfoFamily,
    /// 0-based array index (`address = atoi(ptr) - 1`).
    pub index: usize,
    pub status: u8, // CH_STATUS_{SKIP=0,NORMAL=1,DIFF=2,UNKNOWN=3}
    pub unit: String,
    pub scale: u8,
}

pub const CH_STATUS_SKIP: u8 = 0;
pub const CH_STATUS_NORMAL: u8 = 1;
pub const CH_STATUS_DIFF: u8 = 2;
pub const CH_STATUS_UNKNOWN: u8 = 3;

fn find_byte(payload: &[u8], from: usize, target: u8) -> Option<usize> {
    payload
        .get(from..)?
        .iter()
        .position(|&b| b == target)
        .map(|i| from + i)
}

fn scan_digits(payload: &[u8], from: usize) -> usize {
    let mut i = from;
    while i < payload.len() && payload[i].is_ascii_digit() {
        i += 1;
    }
    i
}

/// `atoi` over `payload[from..to]`, rejecting an empty digit run (an
/// address of 0 or less would otherwise wrap to a huge `usize` at the
/// `- 1` cast below; C's `atoi` on empty/garbage input silently returns 0
/// and indexes `array[-1]`, which is undefined behavior we do not
/// reproduce — this is a parse-boundary safety net, not a protocol guess).
fn parse_1based_index(payload: &[u8], from: usize, to: usize) -> Option<usize> {
    if to <= from {
        return None;
    }
    let n: i64 = std::str::from_utf8(&payload[from..to]).ok()?.parse().ok()?;
    if n < 1 {
        return None;
    }
    Some((n - 1) as usize)
}

/// Every line, regardless of channel family, is dropped by C when the
/// owning module's `use_flag` is false (`drvGM10.c:712`, meas channels
/// only — Math/Comm have no such gate). That check needs `Cache::modules`,
/// which this pure parser does not have; the caller must apply it before
/// storing a `Signal`-family line (`index / 100` selects the module).
pub fn parse_fchinfo(payload: &[u8]) -> Vec<ChannelInfoLine> {
    let mut out = Vec::new();
    let mut p = 0usize;
    while p < payload.len() && payload[p] != b'E' {
        let status = match payload[p] {
            b'N' => CH_STATUS_NORMAL,
            b'D' => CH_STATUS_DIFF,
            b'S' => CH_STATUS_SKIP,
            _ => CH_STATUS_UNKNOWN,
        };
        p += 2;
        if p >= payload.len() {
            break;
        }
        let family_ch = payload[p];
        p += 1;
        let family = match family_ch {
            b'0' => InfoFamily::Signal,
            b'A' => InfoFamily::Math,
            b'C' => InfoFamily::Comm,
            _ => {
                let Some(nl) = find_byte(payload, p, b'\n') else {
                    break;
                };
                p = nl + 1;
                continue;
            }
        };
        let digits_end = scan_digits(payload, p);
        let Some(index) = parse_1based_index(payload, p, digits_end) else {
            break;
        };
        // Fixed 4-byte jump from where `atoi` started reading, independent
        // of how many digits it actually consumed (`drvGM10.c:717,724,731`:
        // `ptr += 4` after `atoi(ptr)`, not after the digit run's own width).
        p += 4;

        if status == CH_STATUS_SKIP {
            let Some(nl) = find_byte(payload, p, b'\n') else {
                break;
            };
            out.push(ChannelInfoLine {
                family,
                index,
                status,
                unit: "----".to_string(),
                scale: 0,
            });
            p = nl + 1;
            continue;
        }

        // Unit field: up to 10 chars, stopping early at a space
        // (`drvGM10.c:754-759` — note the source `char unit[7]` buffer is
        // only sized for 6 chars + NUL, a stack buffer overflow the Rust
        // `String` here structurally cannot reproduce).
        let unit_start = p;
        let mut i = 0usize;
        while i < 10 && unit_start + i < payload.len() && payload[unit_start + i] != b' ' {
            i += 1;
        }
        let unit = String::from_utf8_lossy(&payload[unit_start..unit_start + i]).into_owned();
        p = unit_start + i;
        let Some(comma) = find_byte(payload, p, b',') else {
            break;
        };
        let scale_start = comma + 1;
        let scale_end = scan_digits(payload, scale_start);
        let scale = std::str::from_utf8(&payload[scale_start..scale_end])
            .ok()
            .and_then(|s| s.parse::<u8>().ok())
            .unwrap_or(0);

        out.push(ChannelInfoLine {
            family,
            index,
            status,
            unit,
            scale,
        });

        let Some(nl) = find_byte(payload, scale_end, b'\n') else {
            break;
        };
        p = nl + 1;
    }
    out
}

// ---------------------------------------------------------------------
// SRangeAO / SRangeDO (`load_infos`, `drvGM10.c:774-869`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelModeLine {
    /// 0-based `meas_info` index (signal channels only).
    pub index: usize,
    pub mode: i32,
}

pub const CH_MODE_DAC_SKIP: i32 = 0;
pub const CH_MODE_DAC_RETRANS: i32 = 1;
pub const CH_MODE_DAC_MANUAL: i32 = 2;
pub const CH_MODE_UNKNOWN: i32 = 16;

pub const CH_MODE_RELAY_ALARM: i32 = 0;
pub const CH_MODE_RELAY_MANUAL: i32 = 1;
pub const CH_MODE_RELAY_FAIL: i32 = 2;

/// Same module-`use_flag` caveat as [`parse_fchinfo`] applies here
/// (`drvGM10.c:789-793`/`:835-839`) — this parser returns every candidate
/// line; the caller gates storage on `Cache::modules[index / 100].use_flag`.
fn parse_range_lines(payload: &[u8], mode_of: impl Fn(u8) -> i32) -> Vec<ChannelModeLine> {
    let mut out = Vec::new();
    let mut p = 0usize;
    while p < payload.len() && payload[p] != b'E' {
        // `ptr += 9; if (*(ptr++) != '0') return 1;` (`drvGM10.c:784-785`).
        p += 9;
        if p >= payload.len() || payload[p] != b'0' {
            break;
        }
        p += 1;
        let digits_end = scan_digits(payload, p);
        let Some(index) = parse_1based_index(payload, p, digits_end) else {
            break;
        };
        // Fixed 4-byte jump from where `atoi` started (`drvGM10.c:794`:
        // `ptr += 4` after `atoi(ptr)`), not from the digit run's own end.
        p += 4;
        if p >= payload.len() {
            break;
        }
        let mode = mode_of(payload[p]);
        out.push(ChannelModeLine { index, mode });
        let Some(nl) = find_byte(payload, p, b'\n') else {
            break;
        };
        p = nl + 1;
    }
    out
}

pub fn parse_srangeao(payload: &[u8]) -> Vec<ChannelModeLine> {
    parse_range_lines(payload, |c| match c {
        b'S' => CH_MODE_DAC_SKIP,
        b'M' => CH_MODE_DAC_MANUAL,
        b'T' => CH_MODE_DAC_RETRANS,
        _ => CH_MODE_UNKNOWN,
    })
}

pub fn parse_srangedo(payload: &[u8]) -> Vec<ChannelModeLine> {
    parse_range_lines(payload, |c| match c {
        b'A' => CH_MODE_RELAY_ALARM,
        b'M' => CH_MODE_RELAY_MANUAL,
        b'F' => CH_MODE_RELAY_FAIL,
        _ => CH_MODE_UNKNOWN,
    })
}

// ---------------------------------------------------------------------
// SRangeMath (`load_infos`, `drvGM10.c:874-909`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExprLine {
    pub index: usize,
    pub on_flag: bool,
    pub expr: String,
}

pub fn parse_srangemath(payload: &[u8]) -> Vec<ExprLine> {
    let mut out = Vec::new();
    let mut p = 0usize;
    while p < payload.len() && payload[p] != b'E' {
        p += 11;
        let digits_end = scan_digits(payload, p);
        let Some(index) = parse_1based_index(payload, p, digits_end) else {
            break;
        };
        // Fixed 5-byte jump from where `atoi` started (`drvGM10.c:885`:
        // `ptr += 5` after `atoi(ptr)`), not from the digit run's own end.
        p += 5;
        if p >= payload.len() {
            break;
        }
        if payload[p] == b'f' {
            out.push(ExprLine {
                index,
                on_flag: false,
                expr: String::new(),
            });
            let Some(nl) = find_byte(payload, p, b'\n') else {
                break;
            };
            p = nl + 1;
            continue;
        }
        let Some(q1) = find_byte(payload, p, b'\'') else {
            break;
        };
        let Some(q2) = find_byte(payload, q1 + 1, b'\'') else {
            break;
        };
        let expr = String::from_utf8_lossy(&payload[q1 + 1..q2]).into_owned();
        out.push(ExprLine {
            index,
            on_flag: true,
            expr,
        });
        let Some(nl) = find_byte(payload, p, b'\n') else {
            break;
        };
        p = nl + 1;
    }
    out
}

// ---------------------------------------------------------------------
// FData binary decode (`load_data_values`, `drvGM10.c:965-1069`)
// ---------------------------------------------------------------------

/// Every field here is the raw wire value. The caller (owns `Cache`) must,
/// in this order, exactly as `load_data_values` does:
/// 1. Drop the record unless `data_type == 1` (`drvGM10.c:995`) — other
///    values are never stored.
/// 2. Drop the record unless `Cache::modules[(address-1)/100].use_flag`
///    (same line — applied uniformly, even to Math/Comm channel_types).
/// 3. Select the target array by `channel_type` (1=meas,2=calc,3=comm),
///    index `address - 1`.
/// 4. Derive `data_status` via [`DataStatus::from_wire`]; if not `Normal`,
///    store `value = 0` instead of the wire value (`drvGM10.c:1051-1054`).
/// 5. Mask each alarm byte with `0x3F` before storing (`drvGM10.c:1056-1059`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataRecord {
    pub data_type: u8,
    /// 1 = meas, 2 = calc, 3 = comm.
    pub channel_type: u8,
    pub status: u8,
    /// 1-based wire address, already masked `& 0x03FF`.
    pub address: u16,
    pub alarms: [u8; 4],
    pub value: i32,
}

/// Decode the `FData` binary frame. `raw` is the *full* frame, header
/// included (offsets below are absolute within it, matching
/// `q = inbuffer + 18`).
pub fn parse_fdata_binary(raw: &[u8]) -> Option<Vec<DataRecord>> {
    if raw.len() < 36 {
        return None;
    }
    let length = u16::from_be_bytes([raw[18], raw[19]]) as i32;
    let number_values = ((length - 16) / 12).max(0) as usize;
    let mut q = 36usize;
    let mut out = Vec::with_capacity(number_values);
    for _ in 0..number_values {
        if q + 12 > raw.len() {
            break;
        }
        let data_type = (raw[q] & 0xF0) >> 4;
        let channel_type = raw[q] & 0x0F;
        let status = raw[q + 1];
        let address = u16::from_be_bytes([raw[q + 2], raw[q + 3]]) & 0x03FF;
        let alarms = [raw[q + 4], raw[q + 5], raw[q + 6], raw[q + 7]];
        let value = i32::from_be_bytes([raw[q + 8], raw[q + 9], raw[q + 10], raw[q + 11]]);
        out.push(DataRecord {
            data_type,
            channel_type,
            status,
            address,
            alarms,
            value,
        });
        q += 12;
    }
    Some(out)
}

// ---------------------------------------------------------------------
// SKConst / SWConst (`load_misc_values`, `drvGM10.c:1086-1152`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConstLine {
    pub index: usize,
    pub value: f64,
}

/// Shared by `SKConst`/`SWConst` replies: each line is an opaque 8-byte
/// prefix (`drvGM10.c:1109,1137`: `p += 8`, unexamined by C either), then
/// `<digits>,<float>\n`.
pub fn parse_const_lines(payload: &[u8]) -> Vec<ConstLine> {
    let text = String::from_utf8_lossy(payload);
    let mut out = Vec::new();
    let mut rest: &str = text.as_ref();
    while !rest.is_empty() && !rest.starts_with('E') {
        if rest.len() < 8 {
            break;
        }
        let after_prefix = &rest[8..];
        let digits_end = after_prefix
            .find(|c: char| !c.is_ascii_digit())
            .unwrap_or(after_prefix.len());
        let Ok(addr1): Result<i64, _> = after_prefix[..digits_end].parse() else {
            break;
        };
        if addr1 < 1 {
            break;
        }
        let Some(comma) = after_prefix.find(',') else {
            break;
        };
        let after_comma = &after_prefix[comma + 1..];
        let Some(nl) = after_comma.find('\n') else {
            break;
        };
        let value_str = after_comma[..nl].trim_end_matches('\r');
        let Ok(value) = value_str.parse::<f64>() else {
            break;
        };
        out.push(ConstLine {
            index: (addr1 - 1) as usize,
            value,
        });
        rest = &after_comma[nl + 1..];
    }
    out
}

// ---------------------------------------------------------------------
// ORec? / OMath? (`load_status`, `drvGM10.c:625-656`)
// ---------------------------------------------------------------------

pub fn parse_orec(payload: &str) -> Option<i32> {
    parse_int_after(payload, "ORec,")
}

pub fn parse_omath(payload: &str) -> Option<i32> {
    parse_int_after(payload, "OMath,")
}

fn parse_int_after(payload: &str, prefix: &str) -> Option<i32> {
    let idx = payload.find(prefix)?;
    let rest = &payload[idx + prefix.len()..];
    let end = rest
        .find(|c: char| !(c.is_ascii_digit() || c == '-'))
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_builders_match_c_format_strings() {
        assert_eq!(cmd_fdata_signal(27), "FData,1,0027,0027\r\n");
        assert_eq!(cmd_fdata_math(5), "FData,1,A005,A005\r\n");
        assert_eq!(cmd_fdata_comm(123), "FData,1,C123,C123\r\n");
        assert_eq!(cmd_ocmdao(27, 1.5), "OCmdAO,0027,1500\r\n");
        assert_eq!(cmd_ocmdrelay(5, true), "OCmdRelay,0005-On\r\n");
        assert_eq!(cmd_ocmdrelay(5, false), "OCmdRelay,0005-Off\r\n");
        assert_eq!(cmd_orec_set(true), "ORec,1\r\n");
        assert_eq!(cmd_orec_set(false), "ORec,0\r\n");
        assert_eq!(cmd_omath_set(3), "OMath,3\r\n");
        assert_eq!(cmd_err_query(205), "_ERR,205\r\n");
    }

    #[test]
    fn format_g_matches_printf_g() {
        assert_eq!(format_g(3.25), "3.25");
        assert_eq!(format_g(100.0), "100");
        assert_eq!(format_g(0.0001234), "0.0001234");
        assert_eq!(format_g(1.0), "1");
        assert_eq!(format_g(0.0), "0");
        assert_eq!(format_g(123456.0), "123456");
        assert_eq!(format_g(1234567.0), "1.23457E+06");
    }

    #[test]
    fn error_header_with_and_without_parameter() {
        assert_eq!(parse_error_header("E1,205:1:12\r\n"), Some((205, 12)));
        assert_eq!(parse_error_header("E1,205\r\n"), Some((205, 0)));
        assert_eq!(parse_error_header("E0\r\n"), None);
    }

    #[test]
    fn chop_string_40_short_string_no_remainder() {
        let (chunk, rest) = chop_string_40("short message");
        assert_eq!(chunk, "short message");
        assert_eq!(rest, None);
    }

    #[test]
    fn chop_string_40_breaks_at_last_space_within_40() {
        // 45-char message; the space nearest-but-before index 39 is at 33.
        let msg = "This is a long error message that continues";
        let (chunk, rest) = chop_string_40(msg);
        assert_eq!(chunk, "This is a long error message that");
        assert_eq!(rest, Some("continues".to_string()));
    }

    #[test]
    fn split_error_message_cascades_and_clears_unused_slots() {
        let out = split_error_message("short");
        assert_eq!(out[0], "short");
        assert_eq!(out[1], "");
        assert_eq!(out[2], "");
    }

    #[test]
    fn extract_quoted_message_pulls_body_between_quotes() {
        assert_eq!(
            extract_quoted_message("junk 'hello world' junk"),
            Some("hello world")
        );
        assert_eq!(extract_quoted_message("no quotes"), None);
    }

    #[test]
    fn fsysconf_parses_one_present_module() {
        let mut payload = b"Unit:00".to_vec(); // 7-byte header check
        payload.extend_from_slice(b"  "); // 2 unexamined bytes (ptr += 9 total)
        payload.extend_from_slice(b"00:"); // module index "00" + 1 unexamined byte
        // Both fields are 17 bytes wide by construction, so `start + 17`
        // always lands exactly on the next field regardless of content
        // width (`drvGM10.c:463,469`).
        payload.extend_from_slice(format!("{:<17}", "GX90XA-06").as_bytes());
        payload.extend_from_slice(format!("{:<17}", "GX90XA-06").as_bytes());
        payload.extend_from_slice(b"----------------\r\n"); // error message + CRLF
        payload.push(b'E'); // sentinel terminating the loop
        let modules = parse_fsysconf(&payload).unwrap();
        assert_eq!(modules.len(), 1);
        assert_eq!(modules[0].index, 0);
        assert_eq!(modules[0].set_message, "GX90XA-06");
        assert!(module_line_ok(&modules[0]));
        let (mtype, count) = classify_module(&modules[0].set_message).unwrap();
        assert_eq!(mtype, ModuleType::InputAnalog);
        assert_eq!(count, [6, 0]);
    }

    #[test]
    fn fsysconf_rejects_missing_unit_header() {
        assert_eq!(parse_fsysconf(b"Something:00 rest"), None);
    }

    #[test]
    fn classify_module_rejects_non_gx90() {
        assert_eq!(classify_module("OTHER-01"), None);
    }

    #[test]
    fn classify_module_input_output_digital_splits_count() {
        let (mtype, count) = classify_module("GX90WD-0816").unwrap();
        assert_eq!(mtype, ModuleType::InputOutputDigital);
        assert_eq!(count, [8, 16]);
    }

    #[test]
    fn fchinfo_normal_line_parses_unit_and_scale() {
        // status='N' skip 1, family='0' (leading digit of "0001"), digits
        // "001" (3 more), then a fixed 4-byte jump (`drvGM10.c:717`) lands
        // on the unit field regardless of the digit run's own width.
        let payload = b"N 0001 DEGC ,3\nE";
        let lines = parse_fchinfo(payload);
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].family, InfoFamily::Signal);
        assert_eq!(lines[0].index, 0);
        assert_eq!(lines[0].status, CH_STATUS_NORMAL);
        assert_eq!(lines[0].unit, "DEGC");
        assert_eq!(lines[0].scale, 3);
    }

    #[test]
    fn fchinfo_skip_line_uses_sentinel_unit() {
        let payload = b"S 0001 \nE";
        let lines = parse_fchinfo(payload);
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].index, 0);
        assert_eq!(lines[0].status, CH_STATUS_SKIP);
        assert_eq!(lines[0].unit, "----");
        assert_eq!(lines[0].scale, 0);
    }

    #[test]
    fn srangeao_decodes_dac_modes() {
        // 9 unexamined bytes, then the mandatory '0', then a 4-digit
        // address, then the mode char directly (no delimiter byte, per
        // the fixed `ptr += 4` in `drvGM10.c:794`).
        let payload = b"xxxxxxxxx00001T\nE";
        let lines = parse_srangeao(payload);
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].index, 0);
        assert_eq!(lines[0].mode, CH_MODE_DAC_RETRANS);
    }

    #[test]
    fn srangedo_decodes_relay_modes() {
        let payload = b"xxxxxxxxx00003M\nE";
        let lines = parse_srangedo(payload);
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].index, 2);
        assert_eq!(lines[0].mode, CH_MODE_RELAY_MANUAL);
    }

    #[test]
    fn srangemath_off_channel_has_no_expr() {
        // 11 unexamined bytes, 4-digit address, 1 delimiter, then a fixed
        // 5-byte jump (`drvGM10.c:885`) lands on the flag byte.
        let payload = b"xxxxxxxxxxx0001,f\nE";
        let lines = parse_srangemath(payload);
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].index, 0);
        assert!(!lines[0].on_flag);
        assert_eq!(lines[0].expr, "");
    }

    #[test]
    fn srangemath_on_channel_extracts_quoted_expr() {
        let payload = "xxxxxxxxxxx0001,'A002+A003'\nE".as_bytes();
        let lines = parse_srangemath(payload);
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].index, 0);
        assert!(lines[0].on_flag);
        assert_eq!(lines[0].expr, "A002+A003");
    }

    #[test]
    fn const_lines_parses_multiple_entries() {
        let payload = b"aaaaaaaa001,12.5\nbbbbbbbb002,-3.25\nE";
        let lines = parse_const_lines(payload);
        assert_eq!(lines.len(), 2);
        assert_eq!(
            lines[0],
            ConstLine {
                index: 0,
                value: 12.5
            }
        );
        assert_eq!(
            lines[1],
            ConstLine {
                index: 1,
                value: -3.25
            }
        );
    }

    #[test]
    fn orec_omath_parse() {
        assert_eq!(parse_orec("ORec,1\r\n"), Some(1));
        assert_eq!(parse_omath("OMath,3\r\n"), Some(3));
        assert_eq!(parse_orec("garbage"), None);
    }

    #[test]
    fn fdata_binary_decodes_one_record() {
        let mut raw = vec![b'E', b'B', 0, 0];
        // total length placeholder, filled below
        raw.extend_from_slice(&0u32.to_be_bytes());
        raw.extend_from_slice(&[0u8; 10]); // bytes 8..18: unexamined preamble
        // length field at offset 18: 16 + 12*1
        raw.extend_from_slice(&28u16.to_be_bytes());
        raw.extend_from_slice(&[0u8; 8]); // input_poll_time[8]
        raw.extend_from_slice(&[0u8; 8]); // extra skip
        // one 12-byte record: data_type=1, channel_type=1, status=0,
        // address=27, alarms all 0, value=1234
        raw.push((1 << 4) | 1);
        raw.push(0);
        raw.extend_from_slice(&27u16.to_be_bytes());
        raw.extend_from_slice(&[0, 0, 0, 0]);
        raw.extend_from_slice(&1234i32.to_be_bytes());
        let total = (raw.len() - 8) as u32;
        raw[4..8].copy_from_slice(&total.to_be_bytes());

        let records = parse_fdata_binary(&raw).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].data_type, 1);
        assert_eq!(records[0].channel_type, 1);
        assert_eq!(records[0].address, 27);
        assert_eq!(records[0].value, 1234);
    }
}
