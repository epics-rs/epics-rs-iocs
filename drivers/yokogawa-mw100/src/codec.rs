//! Pure command-string builders and response parsers for the MW100
//! protocol. Every parser here operates on an ASCII payload that has
//! already had [`crate::wire::ascii_payload`]'s leading-4-byte-header /
//! trailing-`"EN\r\n"` strip applied (matching every `drvMW100.c`
//! consumer's own `ptr += 4`), or on a full raw binary frame (`FD1`/`FO1`,
//! which indexes from the frame start).

// ---------------------------------------------------------------------
// Command builders
// ---------------------------------------------------------------------

pub fn cmd_cf0() -> String {
    "CF0\r\n".to_string()
}
pub fn cmd_is0() -> String {
    "IS0\r\n".to_string()
}
/// `load_infos`'s channel-info query (`drvMW100.c:748`): always the full
/// fixed range, never parameterized by channel.
pub fn cmd_fe1() -> String {
    "FE1,001,A300\r\n".to_string()
}
/// `load_infos`'s output-channel-status query (`drvMW100.c:828`).
pub fn cmd_fo0() -> String {
    "FO0,001,060\r\n".to_string()
}
pub fn cmd_ao_query() -> String {
    "AO?\r\n".to_string()
}
pub fn cmd_xd_query() -> String {
    "XD?\r\n".to_string()
}
pub fn cmd_so_query() -> String {
    "SO?\r\n".to_string()
}
pub fn cmd_fd1_all() -> String {
    "FD1,001,A300\r\n".to_string()
}
pub fn cmd_fd1_signal(channel: u32) -> String {
    format!("FD1,{channel:03},{channel:03}\r\n")
}
pub fn cmd_fd1_math(channel: u32) -> String {
    format!("FD1,A{channel:03},A{channel:03}\r\n")
}
pub fn cmd_fo1_all() -> String {
    "FO1,001,060\r\n".to_string()
}
pub fn cmd_fo1_signal(channel: u32) -> String {
    format!("FO1,{channel:03},{channel:03}\r\n")
}
pub fn cmd_cm_query_all() -> String {
    "CM?\r\n".to_string()
}
pub fn cmd_cmc_query(channel: u32) -> String {
    format!("CMC{channel:03}?\r\n")
}
pub fn cmd_sk_query_all() -> String {
    "SK?\r\n".to_string()
}
pub fn cmd_skk_query(channel: u32) -> String {
    format!("SKK{channel:02}?\r\n")
}
/// `set_output_value(CMD_SET_SIGNAL_OUTPUT, ...)` (`drvMW100.c:1262`):
/// `sval` is the already-unscaled integer (see [`crate::cache::unscaled_value`]).
pub fn cmd_sp_set(channel: u32, sval: i32) -> String {
    format!("SP{channel:03},{sval}\r\n")
}
pub fn cmd_cmc_set(channel: u32, value: f64) -> String {
    format!("CMC{channel:03},{}\r\n", format_g(value))
}
pub fn cmd_skk_set(channel: u32, value: f64) -> String {
    format!("SKK{channel:02},{}\r\n", format_g(value))
}
pub fn cmd_vd_set(channel: u32, on: bool) -> String {
    format!("VD{channel:03},{}\r\n", if on { "ON" } else { "OFF" })
}
/// `set_mode(CMD_SET_OPMODE, ...)` (`drvMW100.c:1303`).
pub fn cmd_ds_set(on: bool) -> String {
    format!("DS{}\r\n", if on { '1' } else { '0' })
}
/// `set_mode(CMD_SET_COMPUTE, ...)` (`drvMW100.c:1312`): `mode` is 0-3.
pub fn cmd_ex_set(mode: u8) -> String {
    format!("EX{}\r\n", (b'0' + (mode & 0x3)) as char)
}
pub fn cmd_ce0() -> String {
    "CE0\r\n".to_string()
}
pub fn cmd_ak0() -> String {
    "AK0\r\n".to_string()
}
/// Byte-order negotiation (`init_mw100`, `drvMW100.c:1538-1543`): the
/// original driver sends `BO1` only on a little-endian host, `BO0`
/// otherwise, and decodes binary payloads accordingly. This port always
/// decodes little-endian (see [`crate::wire`]), so it always negotiates
/// `BO1`.
pub fn cmd_bo1() -> String {
    "BO1\r\n".to_string()
}

/// C `sprintf("%G", value)` — default precision (6 significant digits),
/// trailing zeros stripped (`drvMW100.c:1265,1268`).
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
// Error decode (`response_reader`, `drvMW100.c:526-562`) plus the local
// static error-message table (`drvMW100.c:194-394`).
// ---------------------------------------------------------------------

/// `sscanf(inbuffer, "E1 %d %*s\r\n", &error_code) == 1` — space-separated
/// (not GM10's comma/`:1:`-suffixed format); the trailing token is
/// discarded, and `sscanf`'s return count is already satisfied once the
/// leading `%d` assigns, regardless of what (if anything) follows.
pub fn parse_error_code(payload: &str) -> Option<i32> {
    let rest = payload.strip_prefix("E1 ")?;
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    if end == 0 {
        return None;
    }
    rest[..end].parse().ok()
}

#[derive(Debug)]
pub struct ErrorEntry {
    pub id: i32,
    pub strings: [&'static str; 3],
}

/// Look up an error code in the local static table (`response_reader`,
/// `drvMW100.c:544-548`: a linear scan with no `break`, so with unique ids
/// the first match is also the only/last match).
pub fn lookup_error(code: i32) -> Option<&'static ErrorEntry> {
    ERRORS.iter().find(|e| e.id == code)
}

/// Transcribed verbatim from `drvMW100.c:194-394`, with one upstream defect
/// fixed: entry id=11's source line (`drvMW100.c:209`) is a malformed
/// adjacent-string-literal concatenation —
/// `{ "Time value exceeds the setting range., "", "" "}}` has zero comma
/// tokens between its three intended string members, so the C compiler
/// concatenates all three into one garbled `string[0]` and leaves
/// `string[1]`/`string[2]` as NULL. Fixed here to the clean single-sentence
/// shape used by every sibling entry (e.g. id=9).
pub static ERRORS: &[ErrorEntry] = &[
    ErrorEntry {
        id: 0,
        strings: ["Invalid response from DAU.", "", ""],
    },
    ErrorEntry {
        id: 1,
        strings: ["Invalid function parameter.", "", ""],
    },
    ErrorEntry {
        id: 2,
        strings: ["Value exceeds the setting range.", "", ""],
    },
    ErrorEntry {
        id: 3,
        strings: ["Incorrect real number format.", "", ""],
    },
    ErrorEntry {
        id: 4,
        strings: ["Real number value exceeds the setting", "range.", ""],
    },
    ErrorEntry {
        id: 5,
        strings: ["Incorrect character string.", "", ""],
    },
    ErrorEntry {
        id: 6,
        strings: ["Character string too long.", "", ""],
    },
    ErrorEntry {
        id: 7,
        strings: ["Incorrect display color format.", "", ""],
    },
    ErrorEntry {
        id: 8,
        strings: ["Incorrect date format.", "", ""],
    },
    ErrorEntry {
        id: 9,
        strings: ["Date value exceeds the setting range.", "", ""],
    },
    ErrorEntry {
        id: 10,
        strings: ["Incorrect time format.", "", ""],
    },
    ErrorEntry {
        id: 11,
        strings: ["Time value exceeds the setting range.", "", ""],
    },
    ErrorEntry {
        id: 12,
        strings: ["Incorrect time zone format.", "", ""],
    },
    ErrorEntry {
        id: 13,
        strings: ["Time zone value exceeds the setting", "range.", ""],
    },
    ErrorEntry {
        id: 14,
        strings: ["Incorrect IP address format.", "", ""],
    },
    ErrorEntry {
        id: 20,
        strings: ["Invalid channel number.", "", ""],
    },
    ErrorEntry {
        id: 21,
        strings: ["Invalid sequence of first and last", "channel.", ""],
    },
    ErrorEntry {
        id: 22,
        strings: ["Invalid alarm number.", "", ""],
    },
    ErrorEntry {
        id: 23,
        strings: ["Invalid relay number.", "", ""],
    },
    ErrorEntry {
        id: 24,
        strings: ["Invalid sequence of first and last", "relay.", ""],
    },
    ErrorEntry {
        id: 25,
        strings: ["Invalid MATH group number.", "", ""],
    },
    ErrorEntry {
        id: 26,
        strings: ["Invalid box number.", "", ""],
    },
    ErrorEntry {
        id: 27,
        strings: ["Invalid timer number.", "", ""],
    },
    ErrorEntry {
        id: 28,
        strings: ["Invalid match time number.", "", ""],
    },
    ErrorEntry {
        id: 29,
        strings: ["Invalid measurement group number.", "", ""],
    },
    ErrorEntry {
        id: 30,
        strings: ["Invalid module number.", "", ""],
    },
    ErrorEntry {
        id: 31,
        strings: ["Invalid start and end time of DST.", "", ""],
    },
    ErrorEntry {
        id: 32,
        strings: ["Invalid display group number.", "", ""],
    },
    ErrorEntry {
        id: 33,
        strings: ["Invalid tripline number.", "", ""],
    },
    ErrorEntry {
        id: 34,
        strings: ["Invalid message number.", "", ""],
    },
    ErrorEntry {
        id: 35,
        strings: ["Invalid user number.", "", ""],
    },
    ErrorEntry {
        id: 36,
        strings: ["Invalid server type.", "", ""],
    },
    ErrorEntry {
        id: 37,
        strings: ["Invalid e-mail contents.", "", ""],
    },
    ErrorEntry {
        id: 38,
        strings: ["Invalid server number.", "", ""],
    },
    ErrorEntry {
        id: 39,
        strings: ["Invalid command number.", "", ""],
    },
    ErrorEntry {
        id: 40,
        strings: ["Invalid client type.", "", ""],
    },
    ErrorEntry {
        id: 41,
        strings: ["Invalid server type.", "", ""],
    },
    ErrorEntry {
        id: 50,
        strings: ["Invalid input type.", "", ""],
    },
    ErrorEntry {
        id: 51,
        strings: [
            "Module of an invalid input type found",
            "in the range of specified channels.",
            "",
        ],
    },
    ErrorEntry {
        id: 52,
        strings: ["Invalid measuring range.", "", ""],
    },
    ErrorEntry {
        id: 53,
        strings: [
            "Module of an invalid measuring range",
            "found in the range of specified",
            "channels.",
        ],
    },
    ErrorEntry {
        id: 54,
        strings: ["Upper and lower limits of span cannot", "be equal.", ""],
    },
    ErrorEntry {
        id: 55,
        strings: ["Upper and lower limits of scale cannot", "be equal.", ""],
    },
    ErrorEntry {
        id: 56,
        strings: ["Invalid reference channel number.", "", ""],
    },
    ErrorEntry {
        id: 60,
        strings: ["Cannot set an alarm for a skipped", "channel.", ""],
    },
    ErrorEntry {
        id: 61,
        strings: [
            "Cannot set an alarm for a channel on",
            "which MATH function is turned OFF.",
            "",
        ],
    },
    ErrorEntry {
        id: 62,
        strings: ["Invalid alarm type.", "", ""],
    },
    ErrorEntry {
        id: 63,
        strings: ["Invalid alarm relay number.", "", ""],
    },
    ErrorEntry {
        id: 65,
        strings: [
            "Cannot set hysteresis for a channel on",
            "which alarms are turned OFF.",
            "",
        ],
    },
    ErrorEntry {
        id: 70,
        strings: ["Nonexistent channel specified in MATH", "expression.", ""],
    },
    ErrorEntry {
        id: 71,
        strings: ["Nonexistent constant specified in MATH", "expression.", ""],
    },
    ErrorEntry {
        id: 72,
        strings: ["Invalid syntax found in MATH", "expression.", ""],
    },
    ErrorEntry {
        id: 73,
        strings: ["Too many operators for MATH", "expression.", ""],
    },
    ErrorEntry {
        id: 74,
        strings: ["Invalid order of operators.", "", ""],
    },
    ErrorEntry {
        id: 75,
        strings: ["Upper an lower limits of MATH span", "cannot be equal.", ""],
    },
    ErrorEntry {
        id: 80,
        strings: ["Incorrect MATH group format.", "", ""],
    },
    ErrorEntry {
        id: 81,
        strings: ["Incorrect channels for MATH group.", "", ""],
    },
    ErrorEntry {
        id: 82,
        strings: ["Too many channels for MATH group.", "", ""],
    },
    ErrorEntry {
        id: 90,
        strings: ["Incorrect break point format.", "", ""],
    },
    ErrorEntry {
        id: 91,
        strings: [
            "Time value of break point exceeds the",
            "setting range.",
            "",
        ],
    },
    ErrorEntry {
        id: 92,
        strings: [
            "Output value of break point exceeds",
            "the setting range.",
            "",
        ],
    },
    ErrorEntry {
        id: 93,
        strings: ["No break point found.", "", ""],
    },
    ErrorEntry {
        id: 94,
        strings: ["Invalid time value of first break", "point.", ""],
    },
    ErrorEntry {
        id: 95,
        strings: ["Invalid time sequence found in break", "points.", ""],
    },
    ErrorEntry {
        id: 100,
        strings: ["Invalid output type.", "", ""],
    },
    ErrorEntry {
        id: 101,
        strings: [
            "Module of an invalid output type found",
            "in the range of specified channels.",
            "",
        ],
    },
    ErrorEntry {
        id: 102,
        strings: ["Invalid output range.", "", ""],
    },
    ErrorEntry {
        id: 103,
        strings: [
            "Module of an invalid output range",
            "found in the range of specified",
            "channels.",
        ],
    },
    ErrorEntry {
        id: 104,
        strings: [
            "Upper and lower limits of output span",
            "cannot be equal.",
            "",
        ],
    },
    ErrorEntry {
        id: 105,
        strings: ["Invalid transmission reference", "channel.", ""],
    },
    ErrorEntry {
        id: 110,
        strings: ["Invalid channel number for contact", "input event.", ""],
    },
    ErrorEntry {
        id: 111,
        strings: ["Invalid channel number for alarm", "event.", ""],
    },
    ErrorEntry {
        id: 112,
        strings: ["Invalid relay number for relay event.", "", ""],
    },
    ErrorEntry {
        id: 113,
        strings: ["Invalid action type.", "", ""],
    },
    ErrorEntry {
        id: 114,
        strings: [
            "Invalid combination of edge and level",
            "detection actions.",
            "",
        ],
    },
    ErrorEntry {
        id: 115,
        strings: ["Invalid combination of level detection", "actions.", ""],
    },
    ErrorEntry {
        id: 116,
        strings: ["Invalid flag number", "", ""],
    },
    ErrorEntry {
        id: 120,
        strings: ["Invalid measurement group number.", "", ""],
    },
    ErrorEntry {
        id: 121,
        strings: ["Invalid measurement group number for", "MATH interval.", ""],
    },
    ErrorEntry {
        id: 130,
        strings: [
            "Size of data file for measurement group",
            "1 exceeds the upper limit.",
            "",
        ],
    },
    ErrorEntry {
        id: 131,
        strings: [
            "Size of data file for measurement group",
            "2 exceeds the upper limit.",
            "",
        ],
    },
    ErrorEntry {
        id: 132,
        strings: [
            "Size of data file for measurement group",
            "3 exceeds the upper limit.",
            "",
        ],
    },
    ErrorEntry {
        id: 133,
        strings: ["Size of MATH data file exceeds the", "upper limit.", ""],
    },
    ErrorEntry {
        id: 134,
        strings: ["Size of thinned data file exceeds the", "upper limit.", ""],
    },
    ErrorEntry {
        id: 135,
        strings: [
            "Cannot set smaller value for thinning",
            "recording interval than measuring or",
            "MATH interval.",
        ],
    },
    ErrorEntry {
        id: 136,
        strings: [
            "Invalid combination of thinning",
            "recording, measuring and MATH interval.",
            "",
        ],
    },
    ErrorEntry {
        id: 137,
        strings: [
            "The combination of the thinning",
            "recording interval and the thinning",
            "recording data length is not correct.",
        ],
    },
    ErrorEntry {
        id: 138,
        strings: [
            "Cannot set recording operation for",
            "measurement group with no measuring",
            "interval.",
        ],
    },
    ErrorEntry {
        id: 139,
        strings: ["Invalid recording interval.", "", ""],
    },
    ErrorEntry {
        id: 140,
        strings: [
            "Upper and lower limits of the display",
            "zone cannot be equal.",
            "",
        ],
    },
    ErrorEntry {
        id: 141,
        strings: [
            "Cannot set smaller value than lower",
            "limit of display zone for upper limit.",
            "",
        ],
    },
    ErrorEntry {
        id: 142,
        strings: [
            "Width of display zone must be 5% of",
            "that of the entire display or more.",
            "",
        ],
    },
    ErrorEntry {
        id: 145,
        strings: ["Incorrect display group format.", "", ""],
    },
    ErrorEntry {
        id: 150,
        strings: ["IP address must belong to class A, B,", "or C.", ""],
    },
    ErrorEntry {
        id: 151,
        strings: ["Net or host part of IP address is all", "0's or 1's.", ""],
    },
    ErrorEntry {
        id: 152,
        strings: ["Invalid subnet mask.", "", ""],
    },
    ErrorEntry {
        id: 153,
        strings: ["Invalid gateway address.", "", ""],
    },
    ErrorEntry {
        id: 160,
        strings: ["Incorrect alarm e-mail channel format.", "", ""],
    },
    ErrorEntry {
        id: 165,
        strings: ["Invalid channel number for Modbus", "command.", ""],
    },
    ErrorEntry {
        id: 166,
        strings: [
            "Invalid combination of start and end",
            "channel for Modbus command.",
            "",
        ],
    },
    ErrorEntry {
        id: 167,
        strings: [
            "Invalid sequence of start and end",
            "channel for Modbus command.",
            "",
        ],
    },
    ErrorEntry {
        id: 168,
        strings: ["Too many channels for command number.", "", ""],
    },
    ErrorEntry {
        id: 170,
        strings: ["Invalid channel number for report.", "", ""],
    },
    ErrorEntry {
        id: 201,
        strings: ["Cannot execute due to different", "operation mode.", ""],
    },
    ErrorEntry {
        id: 202,
        strings: ["Cannot execute while in setting mode.", "", ""],
    },
    ErrorEntry {
        id: 203,
        strings: ["Cannot execute while in measurement", "mode.", ""],
    },
    ErrorEntry {
        id: 204,
        strings: ["Cannot change or execute during memory", "sampling.", ""],
    },
    ErrorEntry {
        id: 205,
        strings: ["Cannot execute during MATH operation.", "", ""],
    },
    ErrorEntry {
        id: 206,
        strings: ["Cannot change or execute during MATH", "operation.", ""],
    },
    ErrorEntry {
        id: 207,
        strings: [
            "Cannot change or execute while",
            "saving/loading settings.",
            "",
        ],
    },
    ErrorEntry {
        id: 209,
        strings: ["Cannot execute while memory sample is", "stopped.", ""],
    },
    ErrorEntry {
        id: 211,
        strings: ["No relays for communication input", "found.", ""],
    },
    ErrorEntry {
        id: 212,
        strings: ["Initial balance failed.", "", ""],
    },
    ErrorEntry {
        id: 213,
        strings: ["No channels for initial balance found.", "", ""],
    },
    ErrorEntry {
        id: 214,
        strings: ["No channels for transmission output", "found.", ""],
    },
    ErrorEntry {
        id: 215,
        strings: ["No channels for arbitrary output", "found.", ""],
    },
    ErrorEntry {
        id: 221,
        strings: ["No measurement channels found.", "", ""],
    },
    ErrorEntry {
        id: 222,
        strings: ["Invalid measurement interval.", "", ""],
    },
    ErrorEntry {
        id: 223,
        strings: ["Too many measurement channels.", "", ""],
    },
    ErrorEntry {
        id: 224,
        strings: ["No MATH channels found.", "", ""],
    },
    ErrorEntry {
        id: 225,
        strings: ["Invalid MATH interval.", "", ""],
    },
    ErrorEntry {
        id: 226,
        strings: ["Cannot start/stop MATH operation.", "", ""],
    },
    ErrorEntry {
        id: 227,
        strings: ["Cannot start/stop recording.", "", ""],
    },
    ErrorEntry {
        id: 301,
        strings: ["CF card error detected.", "", ""],
    },
    ErrorEntry {
        id: 302,
        strings: ["Not enough free space on CF card.", "", ""],
    },
    ErrorEntry {
        id: 303,
        strings: ["CF card is write-protected.", "", ""],
    },
    ErrorEntry {
        id: 311,
        strings: ["CF card not inserted.", "", ""],
    },
    ErrorEntry {
        id: 312,
        strings: ["CF card format damaged.", "", ""],
    },
    ErrorEntry {
        id: 313,
        strings: ["CF card damaged or not formatted.", "", ""],
    },
    ErrorEntry {
        id: 314,
        strings: ["File is write-protected.", "", ""],
    },
    ErrorEntry {
        id: 315,
        strings: ["No such file or directory.", "", ""],
    },
    ErrorEntry {
        id: 316,
        strings: ["Number of files exceeds the upper", "limit.", ""],
    },
    ErrorEntry {
        id: 317,
        strings: ["Invalid file or directory name.", "", ""],
    },
    ErrorEntry {
        id: 318,
        strings: ["Unknown file type.", "", ""],
    },
    ErrorEntry {
        id: 319,
        strings: ["Same name of file or directory already", "exists.", ""],
    },
    ErrorEntry {
        id: 320,
        strings: ["Invalid file or directory operation.", "", ""],
    },
    ErrorEntry {
        id: 321,
        strings: ["File in use.", "", ""],
    },
    ErrorEntry {
        id: 331,
        strings: ["Setting file not found.", "", ""],
    },
    ErrorEntry {
        id: 332,
        strings: ["Setting file is broken.", "", ""],
    },
    ErrorEntry {
        id: 341,
        strings: ["FIFO buffer overflow.", "", ""],
    },
    ErrorEntry {
        id: 342,
        strings: ["Data to be save to file not found.", "", ""],
    },
    ErrorEntry {
        id: 343,
        strings: ["Power failed while opening file.", "", ""],
    },
    ErrorEntry {
        id: 344,
        strings: [
            "Some or all data prior to power outage",
            "could not be recovered.",
            "",
        ],
    },
    ErrorEntry {
        id: 345,
        strings: [
            "Could not restart recording after",
            "recovery from power failure.",
            "",
        ],
    },
    ErrorEntry {
        id: 346,
        strings: ["Recording could not be started due to", "power outage.", ""],
    },
    ErrorEntry {
        id: 401,
        strings: ["Command string too long.", "", ""],
    },
    ErrorEntry {
        id: 402,
        strings: ["Too many commands enumerated.", "", ""],
    },
    ErrorEntry {
        id: 403,
        strings: ["Invalid type of commands enumerated.", "", ""],
    },
    ErrorEntry {
        id: 404,
        strings: ["Invalid command.", "", ""],
    },
    ErrorEntry {
        id: 405,
        strings: ["Not allowed to execute this command.", "", ""],
    },
    ErrorEntry {
        id: 406,
        strings: ["Cannot execute due to different", "operation mode.", ""],
    },
    ErrorEntry {
        id: 407,
        strings: ["Invalid number of parameters.", "", ""],
    },
    ErrorEntry {
        id: 408,
        strings: ["Parameter string too long.", "", ""],
    },
    ErrorEntry {
        id: 411,
        strings: ["Daylight saving time function not", "available.", ""],
    },
    ErrorEntry {
        id: 412,
        strings: ["Temperature unit selection not", "available.", ""],
    },
    ErrorEntry {
        id: 413,
        strings: ["MATH operation not available.", "", ""],
    },
    ErrorEntry {
        id: 414,
        strings: [
            "Serial communication interface option",
            "not available.",
            "",
        ],
    },
    ErrorEntry {
        id: 415,
        strings: ["Report option not available.", "", ""],
    },
    ErrorEntry {
        id: 501,
        strings: ["Login first.", "", ""],
    },
    ErrorEntry {
        id: 502,
        strings: ["Login failed, try again.", "", ""],
    },
    ErrorEntry {
        id: 503,
        strings: ["Connection count exceeded the upper", "limit.", ""],
    },
    ErrorEntry {
        id: 504,
        strings: ["Connection has been lost.", "", ""],
    },
    ErrorEntry {
        id: 505,
        strings: ["Connection has timed out.", "", ""],
    },
    ErrorEntry {
        id: 520,
        strings: ["FTP function not available.", "", ""],
    },
    ErrorEntry {
        id: 521,
        strings: ["FTP control connection failed.", "", ""],
    },
    ErrorEntry {
        id: 530,
        strings: ["SMTP function not available.", "", ""],
    },
    ErrorEntry {
        id: 531,
        strings: ["SMTP connection failed.", "", ""],
    },
    ErrorEntry {
        id: 532,
        strings: ["POP3 connection failed.", "", ""],
    },
    ErrorEntry {
        id: 550,
        strings: ["SNTP function not available.", "", ""],
    },
    ErrorEntry {
        id: 551,
        strings: ["SNTP command/response failed.", "", ""],
    },
    ErrorEntry {
        id: 999,
        strings: ["System error.", "", ""],
    },
];

// ---------------------------------------------------------------------
// CF0 (`load_modules`, `drvMW100.c:565-684`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModuleLine {
    pub index: usize,
    pub set_message: String,
    pub status_message: String,
    pub error_message: String,
}

/// The all-dashes sentinel for "no module configured" (`drvMW100.c:616`) —
/// 13 dashes, unlike GM10's 16-dash sentinel.
pub const NO_MODULE: &str = "-------------";

/// Parses the `CF0` ASCII body. Each line: 1-digit module index (0-5), 3
/// unexamined bytes, a fixed 13-byte `set_message` (no space-trimming), 3
/// unexamined bytes, a fixed 13-byte `status_message`, 1 unexamined byte,
/// then `error_message` read until `\r` (`drvMW100.c:579-603`).
pub fn parse_cf0(payload: &[u8]) -> Option<Vec<ModuleLine>> {
    let mut p = 4usize;
    let mut modules = Vec::new();
    while p < payload.len() && payload[p] != b'E' {
        if !payload[p].is_ascii_digit() {
            return None;
        }
        let which = usize::from(payload[p] - b'0');
        p += 4;

        if p + 13 > payload.len() {
            return None;
        }
        let set_message = String::from_utf8_lossy(&payload[p..p + 13]).into_owned();
        p += 13 + 3;

        if p + 13 > payload.len() {
            return None;
        }
        let status_message = String::from_utf8_lossy(&payload[p..p + 13]).into_owned();
        p += 13 + 1;

        let start = p;
        while p < payload.len() && payload[p] != b'\r' {
            p += 1;
        }
        if p >= payload.len() {
            return None;
        }
        let error_message = String::from_utf8_lossy(&payload[start..p]).into_owned();
        p += 2;

        modules.push(ModuleLine {
            index: which,
            set_message,
            status_message,
            error_message,
        });
    }
    Some(modules)
}

/// Whether a parsed [`ModuleLine`] indicates a present, error-free module
/// (`drvMW100.c:609-619`: set == status, no error, and set isn't itself the
/// all-dashes "unset" sentinel).
pub fn module_line_ok(line: &ModuleLine) -> bool {
    line.set_message == line.status_message
        && line.error_message.is_empty()
        && line.set_message != NO_MODULE
}

/// Decode a module type string (e.g. `"MX110-UNV-M10"`) into
/// `(model, code, speed, number)` (`drvMW100.c:622-649`). `speed`: 0=Low,
/// 1=Medium, 2=High, -1=unrecognized. Never fails — any malformed
/// sub-field falls back to a default (`atoi` on garbage returns 0, an
/// unrecognized speed char yields -1), matching the C source's own
/// tolerant parse.
pub fn classify_module_string(module_string: &str) -> (i32, String, i32, i32) {
    let bytes = module_string.as_bytes();
    // `ptr = module_string + 2` then `atoi(ptr)` (`drvMW100.c:623-624`):
    // the model number starts at byte offset 2 (e.g. "MX110..." -> "110...").
    let model_start = 2.min(bytes.len());
    let model_str = &module_string[model_start..];
    let digits_end = model_str
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(model_str.len());
    let model: i32 = model_str[..digits_end].parse().unwrap_or(0);

    // `ptr += 4` from the model start (`drvMW100.c:625`), then a 3-byte
    // code field (`drvMW100.c:627-630`).
    let code_start = (model_start + 4).min(bytes.len());
    let code_end = (code_start + 3).min(bytes.len());
    let code = module_string[code_start..code_end].to_string();

    // `ptr++` past the code (`drvMW100.c:631`, the extra increment after the
    // copy loop skips the `-` separator), then the speed char
    // (`drvMW100.c:633-646`).
    let speed_pos = code_start + 3 + 1;
    let speed = match bytes.get(speed_pos) {
        Some(b'L') => 0,
        Some(b'M') => 1,
        Some(b'H') => 2,
        _ => -1,
    };

    // `ptr++` past the speed char, then `atoi(ptr)` (`drvMW100.c:647-649`).
    let number_start = (speed_pos + 1).min(bytes.len());
    let number_str = &module_string[number_start.min(module_string.len())..];
    let digits_end = number_str
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(number_str.len());
    let number: i32 = number_str[..digits_end].parse().unwrap_or(0);

    (model, code, speed, number)
}

/// `CH_TYPE_*` for a given model number (`drvMW100.c:657-677`).
pub fn channel_type_for_model(model: i32) -> crate::cache::ChannelType {
    use crate::cache::ChannelType;
    match model {
        110 | 112 => ChannelType::InputAnalog,
        114 => ChannelType::InputInteger,
        115 => ChannelType::InputBinary,
        120 => ChannelType::OutputAnalog,
        125 => ChannelType::OutputBinary,
        _ => ChannelType::Unknown,
    }
}

// ---------------------------------------------------------------------
// FE1 (`load_infos`, `drvMW100.c:748-821`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InfoFamily {
    Signal,
    Math,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelInfoLine {
    pub family: InfoFamily,
    /// 0-based array index (`address = atoi(channel) - 1`).
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

pub fn parse_fe1(payload: &[u8]) -> Option<Vec<ChannelInfoLine>> {
    let mut out = Vec::new();
    let mut p = 4usize;
    while p < payload.len() && payload[p] != b'E' {
        let status = match payload[p] {
            b'N' => CH_STATUS_NORMAL,
            b'D' => CH_STATUS_DIFF,
            b'S' => CH_STATUS_SKIP,
            _ => CH_STATUS_UNKNOWN,
        };
        p += 2;
        if p + 4 > payload.len() {
            return None;
        }
        let channel = &payload[p..p + 4];
        p += 4;
        let (family, index) = if channel[0] == b'A' {
            let digits = std::str::from_utf8(&channel[1..4]).ok()?;
            (
                InfoFamily::Math,
                digits.trim_start_matches('0').parse::<i64>().unwrap_or(0),
            )
        } else {
            let digits = std::str::from_utf8(channel).ok()?;
            (
                InfoFamily::Signal,
                digits.trim_start_matches('0').parse::<i64>().unwrap_or(0),
            )
        };
        let index = (index - 1).max(0) as usize;

        if status == CH_STATUS_SKIP {
            let nl = find_byte(payload, p, b'\n')?;
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

        let unit_start = p;
        let mut i = 0usize;
        while i < 6 && unit_start + i < payload.len() && payload[unit_start + i] != b' ' {
            i += 1;
        }
        let unit = String::from_utf8_lossy(&payload[unit_start..unit_start + i]).into_owned();
        p = unit_start + i;
        let comma = find_byte(payload, p, b',')?;
        let scale_start = comma + 1;
        if scale_start + 3 > payload.len() {
            return None;
        }
        let scale: u8 = std::str::from_utf8(&payload[scale_start..scale_start + 3])
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(0);

        out.push(ChannelInfoLine {
            family,
            index,
            status,
            unit,
            scale,
        });

        let nl = find_byte(payload, scale_start + 3, b'\n')?;
        p = nl + 1;
    }
    Some(out)
}

// ---------------------------------------------------------------------
// FO0 (`load_infos`, `drvMW100.c:828-852`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputStatusLine {
    pub index: usize,
    pub status: u8,
}

/// Any status char other than `'N'`/`'S'` carries forward the previous
/// line's status unchanged (`drvMW100.c:832,838-845`: `ch_status` is a
/// single local variable with no `default` case in the switch) — the first
/// line therefore inherits `CH_STATUS_UNKNOWN`, its initial value.
pub fn parse_fo0(payload: &[u8]) -> Option<Vec<OutputStatusLine>> {
    let mut out = Vec::new();
    let mut p = 4usize;
    let mut status = CH_STATUS_UNKNOWN;
    while p < payload.len() && payload[p] != b'E' {
        match payload[p] {
            b'N' => status = CH_STATUS_NORMAL,
            b'S' => status = CH_STATUS_SKIP,
            _ => {}
        }
        p += 2;
        let digits_end = scan_digits(payload, p);
        let index = parse_1based_index(payload, p, digits_end)?;
        out.push(OutputStatusLine { index, status });
        let nl = find_byte(payload, digits_end, b'\n')?;
        p = nl + 1;
    }
    Some(out)
}

fn scan_digits(payload: &[u8], from: usize) -> usize {
    let mut i = from;
    while i < payload.len() && payload[i].is_ascii_digit() {
        i += 1;
    }
    i
}

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

// ---------------------------------------------------------------------
// AO? / XD? (`load_infos`, `drvMW100.c:859-943`)
// ---------------------------------------------------------------------

pub const CH_MODE_DAC_SKIP: i32 = 0;
pub const CH_MODE_DAC_TRANS: i32 = 1;
pub const CH_MODE_DAC_COM: i32 = 2;
pub const CH_MODE_DAC_UNKNOWN: i32 = 3;

pub const CH_MODE_RELAY_SKIP: i32 = 0;
pub const CH_MODE_RELAY_ALARM: i32 = 1;
pub const CH_MODE_RELAY_COM: i32 = 2;
pub const CH_MODE_RELAY_MEDIA: i32 = 3;
pub const CH_MODE_RELAY_FAIL: i32 = 4;
pub const CH_MODE_RELAY_ERROR: i32 = 5;
pub const CH_MODE_RELAY_UNKNOWN: i32 = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelModeLine {
    pub index: usize,
    pub mode: i32,
}

/// `AO?` DAC-mode decode (`drvMW100.c:859-896`): `'A'` at the fixed offset
/// means "not skipped", any other char means `CH_MODE_DAC_SKIP` outright
/// (the mode char after it is never even inspected in that case).
pub fn parse_ao(payload: &[u8]) -> Option<Vec<ChannelModeLine>> {
    let mut out = Vec::new();
    let mut p = 4usize;
    while p < payload.len() && payload[p] != b'E' {
        p += 2;
        let digits_end = scan_digits(payload, p).min(p + 3);
        let index = parse_1based_index(payload, p, digits_end)?;
        p += 4;
        let mode = if payload.get(p) != Some(&b'A') {
            CH_MODE_DAC_SKIP
        } else {
            p += 3;
            match payload.get(p) {
                Some(b'C') => CH_MODE_DAC_COM,
                Some(b'T') => CH_MODE_DAC_TRANS,
                _ => CH_MODE_DAC_UNKNOWN,
            }
        };
        out.push(ChannelModeLine { index, mode });
        let nl = find_byte(payload, p, b'\n')?;
        p = nl + 1;
    }
    Some(out)
}

/// `XD?` relay-mode decode (`drvMW100.c:900-943`): if the channel's
/// already-stored [`crate::cache::ChStatus`] is `Skip` (set by the earlier
/// `FE1`/`FO0` passes in the same `load_infos` call), the mode char is
/// never inspected at all. Caller supplies `ch_status_of` for that lookup.
pub fn parse_xd(
    payload: &[u8],
    ch_status_of: impl Fn(usize) -> ChStatusWire,
) -> Option<Vec<ChannelModeLine>> {
    let mut out = Vec::new();
    let mut p = 4usize;
    while p < payload.len() && payload[p] != b'E' {
        p += 2;
        let digits_end = scan_digits(payload, p).min(p + 3);
        let index = parse_1based_index(payload, p, digits_end)?;
        p += 4;
        let mode = if ch_status_of(index) == CH_STATUS_SKIP {
            CH_MODE_RELAY_SKIP
        } else {
            match payload.get(p) {
                Some(b'A') => CH_MODE_RELAY_ALARM,
                Some(b'C') => CH_MODE_RELAY_COM,
                Some(b'M') => CH_MODE_RELAY_MEDIA,
                Some(b'F') => CH_MODE_RELAY_FAIL,
                Some(b'E') => CH_MODE_RELAY_ERROR,
                _ => CH_MODE_RELAY_UNKNOWN,
            }
        };
        out.push(ChannelModeLine { index, mode });
        let nl = find_byte(payload, p, b'\n')?;
        p = nl + 1;
    }
    Some(out)
}

pub type ChStatusWire = u8;

// ---------------------------------------------------------------------
// SO? (`load_infos`, `drvMW100.c:950-997`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExprLine {
    pub index: usize,
    pub on_flag: bool,
    pub expr: String,
}

/// `SO?` calc-expression decode. Unlike GM10's `SRangeMath`, the
/// expression text is comma-delimited, not single-quoted
/// (`drvMW100.c:958-994`).
pub fn parse_so(payload: &[u8]) -> Option<Vec<ExprLine>> {
    let mut out = Vec::new();
    let mut p = 4usize;
    while p < payload.len() && payload[p] != b'E' {
        p += 3;
        let digits_end = (p + 3).min(payload.len());
        let index = parse_1based_index(payload, p, digits_end)?;
        p += 5;
        if payload.get(p) == Some(&b'F') {
            out.push(ExprLine {
                index,
                on_flag: false,
                expr: String::new(),
            });
            let nl = find_byte(payload, p, b'\n')?;
            p = nl + 1;
            continue;
        }
        p += 2;
        let comma = find_byte(payload, p, b',')?;
        let expr = String::from_utf8_lossy(&payload[p..comma]).into_owned();
        p = comma + 1;
        out.push(ExprLine {
            index,
            on_flag: true,
            expr,
        });
        let nl = find_byte(payload, p, b'\n')?;
        p = nl + 1;
    }
    Some(out)
}

// ---------------------------------------------------------------------
// FD1 / FO1 binary decode (`load_input_values`/`load_output_values`,
// `drvMW100.c:1037-1244`)
// ---------------------------------------------------------------------

/// One `FD1` (input) record: 1-based wire `address` (Signal 1-60, or
/// Math wire-encoded as `100 + index` per `drvMW100.c:1099-1102`), raw
/// `alarms1`/`alarms2` bytes, and the raw `value` (status not yet derived
/// — the caller applies [`crate::cache::DataStatus::from_wire`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputRecord {
    pub address: u16,
    pub alarms1: u8,
    pub alarms2: u8,
    pub value: u32,
}

/// Decode the `FD1` binary frame (`drvMW100.c:1077-1097`). `raw` is the
/// full frame, header included. The `-22` constant (not `-20`, the
/// preamble's own byte width) is copied verbatim from `drvMW100.c:1079` —
/// not re-derived, since both give the same floored record count for any
/// real frame and only the literal C constant is a proven-correct value.
pub fn parse_fd1_binary(raw: &[u8]) -> Option<Vec<InputRecord>> {
    if raw.len() < 28 {
        return None;
    }
    let length = i64::from(u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]));
    let number_values = ((length - 22) / 8).max(0) as usize;
    let mut q = 28usize;
    let mut out = Vec::with_capacity(number_values);
    for _ in 0..number_values {
        if q + 8 > raw.len() {
            break;
        }
        let address = u16::from_le_bytes([raw[q], raw[q + 1]]);
        let alarms1 = raw[q + 2];
        let alarms2 = raw[q + 3];
        let value = u32::from_le_bytes([raw[q + 4], raw[q + 5], raw[q + 6], raw[q + 7]]);
        out.push(InputRecord {
            address,
            alarms1,
            alarms2,
            value,
        });
        q += 8;
    }
    Some(out)
}

/// One `FO1` (output) record: 1-based Signal wire `address`, raw `value`
/// (`drvMW100.c:1173-1185`: no alarm bytes, and a 2-byte pad between the
/// address and value fields since `q` advances by 4 after a 2-byte read).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputRecord {
    pub address: u16,
    pub value: u32,
}

pub fn parse_fo1_binary(raw: &[u8]) -> Option<Vec<OutputRecord>> {
    if raw.len() < 28 {
        return None;
    }
    let length = i64::from(u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]));
    let number_values = ((length - 22) / 8).max(0) as usize;
    let mut q = 28usize;
    let mut out = Vec::with_capacity(number_values);
    for _ in 0..number_values {
        if q + 8 > raw.len() {
            break;
        }
        let address = u16::from_le_bytes([raw[q], raw[q + 1]]);
        let value = u32::from_le_bytes([raw[q + 4], raw[q + 5], raw[q + 6], raw[q + 7]]);
        out.push(OutputRecord { address, value });
        q += 8;
    }
    Some(out)
}

// ---------------------------------------------------------------------
// CM? / CMC%03d? and SK? / SKK%02d? (`load_output_values`, `drvMW100.c:1188-1238`)
// ---------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValueLine {
    pub index: usize,
    pub value: f64,
}

/// Shared by `CM?`/`CMC%03d?`: 3 unexamined bytes, a 3-digit address, 1
/// delimiter byte, then a float up to the next `\n` (`drvMW100.c:1200-1211`).
pub fn parse_comm_lines(payload: &[u8]) -> Vec<ValueLine> {
    parse_addr_value_lines(payload, 3, 3)
}

/// Shared by `SK?`/`SKK%02d?`: 3 unexamined bytes, a 2-digit address, 1
/// delimiter byte, then a float up to the next `\n` (`drvMW100.c:1226-1237`).
pub fn parse_const_lines(payload: &[u8]) -> Vec<ValueLine> {
    parse_addr_value_lines(payload, 3, 2)
}

fn parse_addr_value_lines(payload: &[u8], skip: usize, addr_width: usize) -> Vec<ValueLine> {
    let mut out = Vec::new();
    let mut p = 4usize;
    while p < payload.len() && payload[p] != b'E' {
        p += skip;
        if p + addr_width > payload.len() {
            break;
        }
        let Ok(addr1) = std::str::from_utf8(&payload[p..p + addr_width])
            .unwrap_or("")
            .parse::<i64>()
        else {
            break;
        };
        if addr1 < 1 {
            break;
        }
        p += addr_width + 1;
        let value_start = p;
        let mut i = value_start;
        while i < payload.len() && payload[i] != b'\n' {
            i += 1;
        }
        if i >= payload.len() {
            break;
        }
        let value_str = std::str::from_utf8(&payload[value_start..i])
            .unwrap_or("")
            .trim_end_matches('\r');
        let Ok(value) = value_str.parse::<f64>() else {
            break;
        };
        out.push(ValueLine {
            index: (addr1 - 1) as usize,
            value,
        });
        p = i + 1;
    }
    out
}

// ---------------------------------------------------------------------
// IS0 (`load_status`, `drvMW100.c:686-728`)
// ---------------------------------------------------------------------

/// Fixed 8-entry status decode: each entry is 3 ASCII decimal digits, plus
/// a 1-byte delimiter (`drvMW100.c:696-700`). Only `status[4]` is used by
/// the driver (bit 0 = settings mode, bit 1 = measurement mode, bit 2 =
/// compute mode).
pub fn parse_is0(payload: &[u8]) -> Option<[u8; 8]> {
    let mut status = [0u8; 8];
    let mut p = 4usize;
    for slot in status.iter_mut() {
        if p + 3 > payload.len() {
            return None;
        }
        let d0 = payload[p].checked_sub(b'0')?;
        let d1 = payload[p + 1].checked_sub(b'0')?;
        let d2 = payload[p + 2].checked_sub(b'0')?;
        *slot = 100 * d0 + 10 * d1 + d2;
        p += 4;
    }
    Some(status)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cache::ChannelType;

    #[test]
    fn command_builders_match_c_format_strings() {
        assert_eq!(cmd_fd1_signal(27), "FD1,027,027\r\n");
        assert_eq!(cmd_fd1_math(5), "FD1,A005,A005\r\n");
        assert_eq!(cmd_fo1_signal(3), "FO1,003,003\r\n");
        assert_eq!(cmd_cmc_query(27), "CMC027?\r\n");
        assert_eq!(cmd_skk_query(5), "SKK05?\r\n");
        assert_eq!(cmd_sp_set(27, 1500), "SP027,1500\r\n");
        assert_eq!(cmd_cmc_set(1, 3.25), "CMC001,3.25\r\n");
        assert_eq!(cmd_skk_set(1, -7.5), "SKK01,-7.5\r\n");
        assert_eq!(cmd_vd_set(5, true), "VD005,ON\r\n");
        assert_eq!(cmd_vd_set(5, false), "VD005,OFF\r\n");
        assert_eq!(cmd_ds_set(true), "DS1\r\n");
        assert_eq!(cmd_ds_set(false), "DS0\r\n");
        assert_eq!(cmd_ex_set(3), "EX3\r\n");
    }

    #[test]
    fn format_g_matches_printf_g() {
        assert_eq!(format_g(3.25), "3.25");
        assert_eq!(format_g(100.0), "100");
        assert_eq!(format_g(0.0), "0");
        assert_eq!(format_g(1234567.0), "1.23457E+06");
    }

    #[test]
    fn error_code_parses_space_separated_format() {
        assert_eq!(parse_error_code("E1 205 blah\r\n"), Some(205));
        assert_eq!(parse_error_code("E1 205\r\n"), Some(205));
        assert_eq!(parse_error_code("E0\r\n"), None);
    }

    #[test]
    fn error_table_lookup_finds_fixed_entry_11() {
        let e = lookup_error(11).unwrap();
        assert_eq!(e.strings[0], "Time value exceeds the setting range.");
        assert_eq!(e.strings[1], "");
        assert_eq!(e.strings[2], "");
    }

    #[test]
    fn error_table_lookup_misses_unknown_code() {
        assert!(lookup_error(-1).is_none());
    }

    #[test]
    fn classify_module_string_matches_doc_examples() {
        assert_eq!(
            classify_module_string("MX110-UNV-M10"),
            (110, "UNV".to_string(), 1, 10)
        );
        assert_eq!(
            classify_module_string("MX115-D05-H10"),
            (115, "D05".to_string(), 2, 10)
        );
        assert_eq!(
            classify_module_string("MX120-VAO-M08"),
            (120, "VAO".to_string(), 1, 8)
        );
        assert_eq!(
            classify_module_string("MX125-MKC-M10"),
            (125, "MKC".to_string(), 1, 10)
        );
    }

    #[test]
    fn channel_type_for_model_matches_c_switch() {
        assert_eq!(channel_type_for_model(110), ChannelType::InputAnalog);
        assert_eq!(channel_type_for_model(112), ChannelType::InputAnalog);
        assert_eq!(channel_type_for_model(114), ChannelType::InputInteger);
        assert_eq!(channel_type_for_model(115), ChannelType::InputBinary);
        assert_eq!(channel_type_for_model(120), ChannelType::OutputAnalog);
        assert_eq!(channel_type_for_model(125), ChannelType::OutputBinary);
        assert_eq!(channel_type_for_model(999), ChannelType::Unknown);
    }

    #[test]
    fn cf0_parses_one_present_module() {
        let mut payload = b"xxxx".to_vec(); // 4-byte ASCII header skip
        payload.push(b'0'); // module index 0
        payload.extend_from_slice(b"xxx"); // 3 unexamined bytes
        payload.extend_from_slice(b"MX110-UNV-M10"); // 13-byte set_message
        payload.extend_from_slice(b"xxx"); // 3 unexamined bytes
        payload.extend_from_slice(b"MX110-UNV-M10"); // 13-byte status_message
        payload.push(b'x'); // 1 unexamined byte
        payload.extend_from_slice(b"\r\n"); // empty error_message + CRLF
        payload.push(b'E');
        let modules = parse_cf0(&payload).unwrap();
        assert_eq!(modules.len(), 1);
        assert_eq!(modules[0].index, 0);
        assert_eq!(modules[0].set_message, "MX110-UNV-M10");
        assert!(module_line_ok(&modules[0]));
    }

    #[test]
    fn fe1_normal_signal_line_parses_unit_and_scale() {
        let payload = b"xxxxN 0001DEGC ,003\nE";
        let lines = parse_fe1(payload).unwrap();
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].family, InfoFamily::Signal);
        assert_eq!(lines[0].index, 0);
        assert_eq!(lines[0].status, CH_STATUS_NORMAL);
        assert_eq!(lines[0].unit, "DEGC");
        assert_eq!(lines[0].scale, 3);
    }

    #[test]
    fn fe1_math_line_parses_family() {
        let payload = b"xxxxN A001DEGC ,000\nE";
        let lines = parse_fe1(payload).unwrap();
        assert_eq!(lines[0].family, InfoFamily::Math);
        assert_eq!(lines[0].index, 0);
    }

    #[test]
    fn fe1_skip_line_uses_sentinel_unit() {
        let payload = b"xxxxS 0001\nE";
        let lines = parse_fe1(payload).unwrap();
        assert_eq!(lines[0].status, CH_STATUS_SKIP);
        assert_eq!(lines[0].unit, "----");
        assert_eq!(lines[0].scale, 0);
    }

    #[test]
    fn fo0_carries_forward_status_on_unrecognized_char() {
        // First line 'N' -> Normal; second line 'X' (unrecognized) carries
        // forward the previous Normal status rather than resetting.
        let payload = b"xxxxN 0001\nX0002\nE";
        let lines = parse_fo0(payload).unwrap();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[0].status, CH_STATUS_NORMAL);
        assert_eq!(lines[1].status, CH_STATUS_NORMAL);
    }

    #[test]
    fn ao_decodes_dac_modes() {
        let payload = b"xxxx000010AXXC\nE";
        let lines = parse_ao(payload).unwrap();
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].index, 0);
        assert_eq!(lines[0].mode, CH_MODE_DAC_COM);
    }

    #[test]
    fn ao_non_a_char_means_skip_without_reading_mode() {
        let payload = b"xxxx00001-XXX\nE";
        let lines = parse_ao(payload).unwrap();
        assert_eq!(lines[0].mode, CH_MODE_DAC_SKIP);
    }

    #[test]
    fn xd_decodes_relay_modes() {
        let payload = b"xxxx00001XM\nE";
        let lines = parse_xd(payload, |_| CH_STATUS_NORMAL).unwrap();
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].index, 0);
        assert_eq!(lines[0].mode, CH_MODE_RELAY_MEDIA);
    }

    #[test]
    fn xd_skip_status_short_circuits_mode() {
        let payload = b"xxxx00001XM\nE";
        let lines = parse_xd(payload, |_| CH_STATUS_SKIP).unwrap();
        assert_eq!(lines[0].mode, CH_MODE_RELAY_SKIP);
    }

    #[test]
    fn so_off_channel_has_no_expr() {
        let payload = b"xxxxxxx001xxF\nE";
        let lines = parse_so(payload).unwrap();
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].index, 0);
        assert!(!lines[0].on_flag);
        assert_eq!(lines[0].expr, "");
    }

    #[test]
    fn so_on_channel_extracts_comma_delimited_expr() {
        let payload = b"xxxxxxx001xxxxA001+A002,\nE";
        let lines = parse_so(payload).unwrap();
        assert_eq!(lines.len(), 1);
        assert!(lines[0].on_flag);
        assert_eq!(lines[0].expr, "A001+A002");
    }

    #[test]
    fn fd1_binary_decodes_one_signal_record() {
        let mut raw = vec![b'E', b'B', 0, 0];
        raw.extend_from_slice(&30u32.to_le_bytes()); // length = 22 + 8*1
        raw.extend_from_slice(&[0u8; 20]); // preamble up to offset 28
        raw.extend_from_slice(&1u16.to_le_bytes()); // address = 1
        raw.push(0x01); // alarms1
        raw.push(0x00); // alarms2
        raw.extend_from_slice(&1234u32.to_le_bytes()); // value
        let records = parse_fd1_binary(&raw).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].address, 1);
        assert_eq!(records[0].alarms1, 0x01);
        assert_eq!(records[0].value, 1234);
    }

    #[test]
    fn fd1_binary_math_address_is_100_plus_index() {
        let mut raw = vec![b'E', b'B', 0, 0];
        raw.extend_from_slice(&30u32.to_le_bytes());
        raw.extend_from_slice(&[0u8; 20]);
        raw.extend_from_slice(&101u16.to_le_bytes()); // math channel 1 -> wire 101
        raw.push(0);
        raw.push(0);
        raw.extend_from_slice(&5000u32.to_le_bytes());
        let records = parse_fd1_binary(&raw).unwrap();
        assert_eq!(records[0].address, 101);
    }

    #[test]
    fn fo1_binary_decodes_one_record() {
        let mut raw = vec![b'E', b'B', 0, 0];
        raw.extend_from_slice(&30u32.to_le_bytes());
        raw.extend_from_slice(&[0u8; 20]);
        raw.extend_from_slice(&3u16.to_le_bytes());
        raw.extend_from_slice(&[0u8; 2]); // 2-byte pad before value
        raw.extend_from_slice(&777u32.to_le_bytes());
        let records = parse_fo1_binary(&raw).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].address, 3);
        assert_eq!(records[0].value, 777);
    }

    #[test]
    fn comm_lines_parses_multiple_entries() {
        let payload = b"xxxxxxx001,12.5\nxxx002,-3.25\nE";
        let lines = parse_comm_lines(payload);
        assert_eq!(
            lines,
            vec![
                ValueLine {
                    index: 0,
                    value: 12.5
                },
                ValueLine {
                    index: 1,
                    value: -3.25
                },
            ]
        );
    }

    #[test]
    fn const_lines_parses_two_digit_address() {
        let payload = b"xxxxxxx01,7.5\nE";
        let lines = parse_const_lines(payload);
        assert_eq!(
            lines,
            vec![ValueLine {
                index: 0,
                value: 7.5
            }]
        );
    }

    #[test]
    fn is0_decodes_eight_fixed_fields() {
        let mut payload = b"xxxx".to_vec();
        for v in [0u8, 0, 0, 0, 5, 0, 0, 0] {
            payload.extend_from_slice(format!("{v:03}x").as_bytes());
        }
        let status = parse_is0(&payload).unwrap();
        assert_eq!(status[4], 5);
        assert_eq!(status[0], 0);
    }
}
