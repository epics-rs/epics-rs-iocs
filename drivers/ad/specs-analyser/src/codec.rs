//! Wire protocol codec for the SPECS Phoibos analyser ASCII/TCP protocol.
//!
//! Ported from `specsAnalyser.cpp`'s `asynWriteRead`/`commandResponse`/
//! `cleanString`/command builders. Every function here is pure (no I/O), so
//! the request framing, response parsing and command-string construction can
//! be unit tested without a live device. The driver's connect/acquisition
//! code supplies the raw bytes (via the underlying asyn octet port) and calls
//! these functions to interpret them.
//!
//! Upstream's C++ string-trim/split helpers (`cleanString`'s `substr`
//! underflow on an all-separator/empty string, the backslash-escape lookback
//! in `commandResponse` re-indexing from 0 on every continuation chunk
//! instead of carrying the previous chunk's last character, the ERROR
//! digit-extraction loop looping past an empty string when no space is ever
//! found, and `commandResponse` calling `cleanString` on every continuation
//! chunk regardless of parse state — silently eating a leading/trailing
//! separator character that is semantically part of an in-progress quoted or
//! array value) are the same defect family: an unguarded length/index/parse-
//! state assumption in ad-hoc C string manipulation. Every function below
//! uses bounds-checked Rust string operations instead, so none of the four
//! can reproduce here.

use std::collections::HashMap;

/// Characters `cleanString` strips by default (`specsAnalyser.cpp:170`).
pub const DEFAULT_CLEAN_CHARS: &str = ": \n";

/// Message counter wraps to 1, never to 0 (`specsAnalyser.cpp:2058-2060`).
pub const MSG_COUNTER_MAX: u32 = 9999;

/// Advance the wire message counter, wrapping to 1 above 9999
/// (`specsAnalyser.cpp:2056-2060`).
pub fn next_counter(current: u32) -> u32 {
    let next = current + 1;
    if next > MSG_COUNTER_MAX { 1 } else { next }
}

/// Build the outgoing request frame: `?nnnn command`, a 4-digit
/// zero-padded counter (`specsAnalyser.cpp:2069`, `sprintf("?%04d %s", ...)`).
pub fn format_request(counter: u32, command: &str) -> String {
    format!("?{counter:04} {command}")
}

/// Strip characters in `search` from both ends of `s`
/// (`SpecsAnalyser::cleanString`, `specsAnalyser.cpp:2126-2141`, always
/// called here with `where=0`, i.e. both ends).
///
/// Upstream throws (`string::substr` underflow) once the string reduces to
/// empty mid-strip; this returns `""` instead.
pub fn clean_string(s: &str, search: &str) -> String {
    let chars: Vec<char> = s.chars().collect();
    let mut start = 0;
    let mut end = chars.len();
    while start < end && search.contains(chars[start]) {
        start += 1;
    }
    while end > start && search.contains(chars[end - 1]) {
        end -= 1;
    }
    chars[start..end].iter().collect()
}

/// Error strip/clean using the default charset.
pub fn clean_default(s: &str) -> String {
    clean_string(s, DEFAULT_CLEAN_CHARS)
}

/// Reasons a raw reply frame failed to validate
/// (`SpecsAnalyser::asynWriteRead`, `specsAnalyser.cpp:2093-2111`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Reply did not start with `'!'`.
    BadPrefix,
    /// The echoed message counter did not match the counter that was sent.
    CounterMismatch { expected: u32, received: u32 },
}

/// Validate and strip a raw `!nnnn payload` reply frame, returning the
/// payload (`specsAnalyser.cpp:2090-2111`). Byte 0 is `'!'`, bytes 1-4 are
/// the 4-digit echoed counter, byte 5 is a space, payload starts at byte 6.
pub fn strip_response_frame(raw: &str, expected_counter: u32) -> Result<&str, FrameError> {
    if !raw.starts_with('!') {
        return Err(FrameError::BadPrefix);
    }
    let received = raw
        .get(1..5)
        .and_then(|f| f.parse::<u32>().ok())
        .unwrap_or(0);
    if received != expected_counter {
        return Err(FrameError::CounterMismatch {
            expected: expected_counter,
            received,
        });
    }
    Ok(raw.get(6..).unwrap_or(""))
}

/// Parsed `ERROR` response body (`specsAnalyser.cpp:2011-2030`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ErrorInfo {
    pub code: String,
    pub message: String,
}

/// Parse an `ERROR ...` payload (payload already known to start with
/// `"ERROR"`) into its code and message. Upstream accumulates the code by
/// scanning character-by-character until the first space, looping past the
/// end of the string (`substr(1)` on an empty string) when no space is ever
/// found — `split_once` handles that case without a panic.
pub fn parse_error_response(payload: &str) -> ErrorInfo {
    let rest = clean_default(payload.get(5..).unwrap_or(""));
    match rest.split_once(' ') {
        Some((code, message)) => ErrorInfo {
            code: code.to_string(),
            message: clean_default(message),
        },
        None => ErrorInfo {
            code: rest,
            message: String::new(),
        },
    }
}

/// Incremental name:value pair parser state
/// (`SpecsAnalyser::commandResponse`, `specsAnalyser.cpp:1925-2008`).
///
/// `last_char` persists the previous character across `feed` calls so the
/// backslash-escape lookback (`replyString.substr(index-1,1)`) is correct at
/// the start of a continuation chunk — upstream re-indexes from the fresh
/// chunk each time, underflowing `substr` if a quoted value continues across
/// a continuation read.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct NameValueParser {
    inside_quotes: bool,
    in_array: bool,
    parsing_name: bool,
    name: String,
    value: String,
    last_char: Option<char>,
}

impl NameValueParser {
    pub fn new() -> Self {
        Self {
            parsing_name: true,
            ..Default::default()
        }
    }

    /// Feed one chunk of response text (the initial OK-stripped remainder,
    /// or a continuation read). Returns the pairs completed by this chunk and
    /// whether the caller must read more from the wire to finish the value
    /// currently in progress (mirrors the `while(parsing)` continuation
    /// condition, `specsAnalyser.cpp:1985-2008`).
    pub fn feed(&mut self, chunk: &str) -> (Vec<(String, String)>, bool) {
        let mut pairs = Vec::new();
        for cc in chunk.chars() {
            if self.parsing_name {
                if cc == ':' {
                    self.parsing_name = false;
                } else {
                    self.name.push(cc);
                }
            } else {
                self.value.push(cc);
                if self.inside_quotes {
                    if self.last_char != Some('\\') && cc == '"' {
                        self.inside_quotes = false;
                    }
                } else {
                    match cc {
                        '[' => self.in_array = true,
                        ']' => self.in_array = false,
                        '"' => self.inside_quotes = true,
                        ' ' if !self.in_array => {
                            pairs.push(self.take_pair());
                        }
                        _ => {}
                    }
                }
            }
            self.last_char = Some(cc);
        }

        let needs_more = !self.parsing_name && (self.in_array || self.inside_quotes);
        if !self.parsing_name && !needs_more {
            // End of reply, not inside quotes/brackets: the trailing value is
            // complete even without a terminating space.
            pairs.push(self.take_pair());
        }
        (pairs, needs_more)
    }

    fn take_pair(&mut self) -> (String, String) {
        let name = clean_default(&self.name);
        let value = clean_default(&self.value);
        self.name.clear();
        self.value.clear();
        self.parsing_name = true;
        (name, value)
    }
}

/// Parse a complete OK-response name:value section in one shot (test/simple
/// callers that already have the whole response). Panics-free; returns
/// whatever pairs are complete plus whether the section was left incomplete.
pub fn parse_name_value_section(section: &str) -> (HashMap<String, String>, bool) {
    let mut parser = NameValueParser::new();
    let (pairs, needs_more) = parser.feed(section);
    (pairs.into_iter().collect(), needs_more)
}

/// Decode a `GetAcquisitionData` `Data` field: `[v1,v2,...]`
/// (`SpecsAnalyser::readAcquisitionData`, `specsAnalyser.cpp:1286-1305`).
/// Upstream's `strtod` returns `0.0` on total parse failure; `unwrap_or(0.0)`
/// matches that for the well-formed comma-separated protocol this driver
/// expects (a partially-numeric token, e.g. `"1.5abc"`, is not something this
/// wire protocol is documented to send, so this does not attempt `strtod`'s
/// prefix-parse behavior for that case).
pub fn parse_data_array(data: &str) -> Vec<f64> {
    let cleaned = clean_string(data, "[]");
    if cleaned.is_empty() {
        return Vec::new();
    }
    cleaned
        .split(',')
        .map(|tok| tok.trim().parse::<f64>().unwrap_or(0.0))
        .collect()
}

/// Outcome of feeding the first (frame-stripped) reply chunk to a
/// [`ResponseAssembler`] (`SpecsAnalyser::commandResponse`,
/// `specsAnalyser.cpp:1911-2030`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BeginOutcome {
    /// The `OK` response is fully parsed (or had nothing to parse).
    Ok,
    /// An `OK` response's last value is inside quotes/brackets; the
    /// transport must read another chunk and call
    /// [`ResponseAssembler::continue_with`].
    NeedsMore,
    /// The device reported an `ERROR` response (never needs continuation —
    /// upstream's ERROR branch parses only the first chunk).
    Error(ErrorInfo),
}

/// Drives `SpecsAnalyser::commandResponse`'s OK/ERROR decision and name:value
/// state machine to completion across however many raw chunks the transport
/// had to read (`specsAnalyser.cpp:1911-2030`'s `while(parsing)` loop). Pure:
/// the transport (blocking `wire.rs` or async `task.rs`) supplies each
/// already-frame-stripped chunk and performs the actual reads.
#[derive(Debug)]
pub struct ResponseAssembler {
    parser: NameValueParser,
    pairs: Vec<(String, String)>,
}

impl Default for ResponseAssembler {
    fn default() -> Self {
        Self::new()
    }
}

impl ResponseAssembler {
    pub fn new() -> Self {
        Self {
            parser: NameValueParser::new(),
            pairs: Vec::new(),
        }
    }

    /// Feed the first reply chunk (already stripped of the `!nnnn ` frame by
    /// [`strip_response_frame`]). Mirrors `replyString.substr(0,2)=="OK"`
    /// (`specsAnalyser.cpp:1915`); an unrecognised (non-"OK") payload is
    /// treated as an ERROR response, matching upstream's unconditional
    /// `else` branch.
    pub fn begin(&mut self, payload: &str) -> BeginOutcome {
        match payload.strip_prefix("OK") {
            // Upstream only parses further if `replyString.length() > 3`,
            // i.e. there is content beyond "OK" (specsAnalyser.cpp:1918).
            Some(rest) if clean_default(rest).is_empty() => BeginOutcome::Ok,
            Some(rest) => {
                let rest = clean_default(rest);
                let (pairs, needs_more) = self.parser.feed(&rest);
                self.pairs.extend(pairs);
                if needs_more {
                    BeginOutcome::NeedsMore
                } else {
                    BeginOutcome::Ok
                }
            }
            None => BeginOutcome::Error(parse_error_response(payload)),
        }
    }

    /// Feed a continuation chunk (`specsAnalyser.cpp:1991-1999`). A
    /// continuation is only ever requested mid-value (inside quotes or an
    /// array, `needs_more`'s precondition), so unlike the first chunk this is
    /// fed raw: upstream calls `cleanString` on every continuation chunk
    /// regardless of parse state, which would silently eat a leading/trailing
    /// space, `:` or newline that is semantically part of the in-progress
    /// value — the same "unguarded string manipulation" defect family as the
    /// three noted in this module's top doc comment. Returns `true` if yet
    /// more data is needed.
    pub fn continue_with(&mut self, chunk: &str) -> bool {
        let (pairs, needs_more) = self.parser.feed(chunk);
        self.pairs.extend(pairs);
        needs_more
    }

    /// Collect all pairs parsed so far into the response map (C's `data`).
    pub fn finish(self) -> HashMap<String, String> {
        self.pairs.into_iter().collect()
    }
}

/// Decode a `data[name]` field as an integer (`SpecsAnalyser::readIntegerData`,
/// `specsAnalyser.cpp:1635-1647`, `stringstream >> value` then `.fail()`
/// check). Upstream's stream extraction tolerates leading whitespace and
/// stops at the first non-digit (a trailing-garbage prefix parse); `.trim()`
/// plus a whole-token `parse` is stricter about trailing garbage than C++
/// stream extraction, which this wire protocol's well-formed numeric fields
/// never exercise (same simplification already noted for `strtod` above).
pub fn parse_integer_field(raw: &str) -> Option<i32> {
    raw.trim().parse::<i32>().ok()
}

/// Decode a `data[name]` field as a double (`SpecsAnalyser::readDoubleData`,
/// `specsAnalyser.cpp:1649-1661`); see [`parse_integer_field`] for the
/// stream-extraction-vs-`parse` tolerance note.
pub fn parse_double_field(raw: &str) -> Option<f64> {
    raw.trim().parse::<f64>().ok()
}

/// Decode a spectrum-parameter `Values` field (lens modes / scan ranges):
/// comma-separated tokens with `"`, `[`, `]` removed from *anywhere* in each
/// token (`SpecsAnalyser::readSpectrumParameter`, `specsAnalyser.cpp:1695-1709`,
/// `std::remove` over the whole token, not just the ends).
pub fn parse_values_list(values: &str) -> Vec<String> {
    values
        .split(',')
        .map(|tok| tok.chars().filter(|c| !"\"[]".contains(*c)).collect())
        .collect()
}

/// Decode the `ParameterNames` field into (`EPICS_NAME`, `raw device name`)
/// pairs (`SpecsAnalyser::setupEPICSParameters`, `specsAnalyser.cpp:1382-1418`).
/// `names` is already `clean_string(names, "[]")`-stripped by the caller, as
/// upstream does before running this state machine.
pub fn parse_parameter_names(names: &str) -> Vec<(String, String)> {
    let mut result = Vec::new();
    let mut inside_quotes = false;
    let mut name = String::new();
    let mut rawname = String::new();
    for cc in names.chars() {
        if inside_quotes {
            match cc {
                ' ' | '[' | ']' | '/' => rawname.push(cc),
                '"' => {
                    result.push((std::mem::take(&mut name), std::mem::take(&mut rawname)));
                    inside_quotes = false;
                }
                _ => {
                    name.extend(cc.to_uppercase());
                    rawname.push(cc);
                }
            }
        } else if cc == '"' {
            inside_quotes = true;
        }
    }
    result
}

/// `%.6g`-style formatting matching C++'s default `stringstream <<` for a
/// `double` (`defaultfloat`, precision 6) — the on-wire numeric format the
/// SPECS server expects for command arguments (`defineSpectrumFAT` etc.,
/// e.g. `specsAnalyser.cpp:1121` `command << "StartEnergy:" << dvalue`).
pub fn format_double(value: f64) -> String {
    if value == 0.0 {
        return "0".to_string();
    }
    const PRECISION: i32 = 6;
    let neg = value.is_sign_negative();
    let abs = value.abs();

    let sci = format!("{:.*e}", (PRECISION - 1) as usize, abs);
    let e_pos = sci.find('e').expect("LowerExp formatting always emits 'e'");
    let mantissa = &sci[..e_pos];
    let exp: i32 = sci[e_pos + 1..]
        .parse()
        .expect("exponent is always an integer");

    let body = if !(-4..PRECISION).contains(&exp) {
        let mantissa = trim_trailing_zeros(mantissa);
        format!(
            "{mantissa}e{}{:02}",
            if exp >= 0 { "+" } else { "-" },
            exp.abs()
        )
    } else {
        let decimals = (PRECISION - 1 - exp).max(0) as usize;
        trim_trailing_zeros(&format!("{abs:.decimals$}"))
    };

    if neg { format!("-{body}") } else { body }
}

fn trim_trailing_zeros(s: &str) -> String {
    if !s.contains('.') {
        return s.to_string();
    }
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

// --- Command builders (specsAnalyser.h SPECS_CMD_* strings + argument formatting) ---

pub fn start_command(safe_after: bool) -> String {
    format!("Start SetSafeStateAfter:\"{safe_after}\"")
}

/// Numeric fields of `DefineSpectrumFAT` (`specsAnalyser.h` `SPECS_CMD_DEFINE_FAT`).
/// `lens_mode`/`scan_range` stay separate parameters: they select an axis
/// enum choice rather than carry a measured energy quantity.
#[derive(Debug, Clone, Copy)]
pub struct DefineFatArgs {
    pub start_energy: f64,
    pub end_energy: f64,
    pub step_width: f64,
    pub dwell_time: f64,
    pub pass_energy: f64,
}

pub fn define_fat_command(args: DefineFatArgs, lens_mode: &str, scan_range: &str) -> String {
    format!(
        "DefineSpectrumFAT StartEnergy:{} EndEnergy:{} StepWidth:{} DwellTime:{} PassEnergy:{} LensMode:\"{lens_mode}\" ScanRange:\"{scan_range}\"",
        format_double(args.start_energy),
        format_double(args.end_energy),
        format_double(args.step_width),
        format_double(args.dwell_time),
        format_double(args.pass_energy),
    )
}

pub fn define_sfat_command(
    start_energy: f64,
    end_energy: f64,
    samples: i32,
    dwell_time: f64,
    lens_mode: &str,
    scan_range: &str,
) -> String {
    format!(
        "DefineSpectrumSFAT StartEnergy:{} EndEnergy:{} Samples:{samples} DwellTime:{} LensMode:\"{lens_mode}\" ScanRange:\"{scan_range}\"",
        format_double(start_energy),
        format_double(end_energy),
        format_double(dwell_time),
    )
}

/// Numeric fields of `DefineSpectrumFRR` (`specsAnalyser.h` `SPECS_CMD_DEFINE_FRR`).
#[derive(Debug, Clone, Copy)]
pub struct DefineFrrArgs {
    pub start_energy: f64,
    pub end_energy: f64,
    pub step_width: f64,
    pub dwell_time: f64,
    pub retarding_ratio: f64,
}

pub fn define_frr_command(args: DefineFrrArgs, lens_mode: &str, scan_range: &str) -> String {
    format!(
        "DefineSpectrumFRR StartEnergy:{} EndEnergy:{} StepWidth:{} DwellTime:{} RetardingRatio:{} LensMode:\"{lens_mode}\" ScanRange:\"{scan_range}\"",
        format_double(args.start_energy),
        format_double(args.end_energy),
        format_double(args.step_width),
        format_double(args.dwell_time),
        format_double(args.retarding_ratio),
    )
}

pub fn define_fe_command(
    kinetic_energy: f64,
    samples: i32,
    dwell_time: f64,
    pass_energy: f64,
    lens_mode: &str,
    scan_range: &str,
) -> String {
    format!(
        "DefineSpectrumFE KinEnergy:{} Samples:{samples} DwellTime:{} PassEnergy:{} LensMode:\"{lens_mode}\" ScanRange:\"{scan_range}\"",
        format_double(kinetic_energy),
        format_double(dwell_time),
        format_double(pass_energy),
    )
}

pub fn get_data_command(from_index: i32, to_index: i32) -> String {
    format!("GetAcquisitionData FromIndex:{from_index} ToIndex:{to_index}")
}

pub fn get_value_command(name: &str) -> String {
    format!("GetAnalyzerParameterValue ParameterName:\"{name}\"")
}

pub fn get_info_command(name: &str) -> String {
    format!("GetAnalyzerParameterInfo ParameterName:\"{name}\"")
}

pub fn get_spectrum_command(name: &str) -> String {
    format!("GetSpectrumParameterInfo ParameterName:\"{name}\"")
}

pub fn get_data_info_command(name: &str) -> String {
    format!("GetSpectrumDataInfo ParameterName:\"{name}\"")
}

pub fn set_value_int_command(name: &str, value: i32) -> String {
    format!("SetAnalyzerParameterValue ParameterName:\"{name}\" Value:{value}")
}

pub fn set_value_double_command(name: &str, value: f64) -> String {
    format!(
        "SetAnalyzerParameterValue ParameterName:\"{name}\" Value:{}",
        format_double(value)
    )
}

pub fn set_value_string_command(name: &str, value: &str) -> String {
    format!("SetAnalyzerParameterValue ParameterName:\"{name}\" Value:{value}")
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- counter / framing ---

    #[test]
    fn counter_increments() {
        assert_eq!(next_counter(0), 1);
        assert_eq!(next_counter(41), 42);
    }

    #[test]
    fn counter_wraps_above_9999_to_1_not_0() {
        assert_eq!(next_counter(9999), 1);
        assert_eq!(next_counter(10000), 1);
    }

    #[test]
    fn request_frame_is_query_counter_space_command() {
        assert_eq!(format_request(1, "Connect"), "?0001 Connect");
        assert_eq!(
            format_request(42, "GetAcquisitionStatus"),
            "?0042 GetAcquisitionStatus"
        );
        assert_eq!(format_request(9999, "Abort"), "?9999 Abort");
    }

    #[test]
    fn strip_response_frame_extracts_payload() {
        assert_eq!(strip_response_frame("!0001 OK", 1), Ok("OK"));
        assert_eq!(
            strip_response_frame("!0042 ERROR 3: not connected", 42),
            Ok("ERROR 3: not connected")
        );
    }

    #[test]
    fn strip_response_frame_rejects_bad_prefix() {
        assert_eq!(strip_response_frame("XOK", 1), Err(FrameError::BadPrefix));
        assert_eq!(strip_response_frame("", 1), Err(FrameError::BadPrefix));
    }

    #[test]
    fn strip_response_frame_rejects_counter_mismatch() {
        assert_eq!(
            strip_response_frame("!0002 OK", 1),
            Err(FrameError::CounterMismatch {
                expected: 1,
                received: 2
            })
        );
    }

    #[test]
    fn strip_response_frame_handles_truncated_reply_without_panicking() {
        // Upstream's substr(6) on a <6-byte reply throws; this must not panic.
        assert_eq!(strip_response_frame("!0001", 1), Ok(""));
        assert_eq!(
            strip_response_frame("!", 1),
            Err(FrameError::CounterMismatch {
                expected: 1,
                received: 0
            })
        );
    }

    // --- clean_string ---

    #[test]
    fn clean_string_strips_both_ends() {
        assert_eq!(clean_default(" : hello : \n"), "hello");
        assert_eq!(clean_string("[[data]]", "[]"), "data");
    }

    #[test]
    fn clean_string_on_all_separator_input_returns_empty_without_panic() {
        assert_eq!(clean_default(": \n: \n"), "");
        assert_eq!(clean_default(""), "");
    }

    #[test]
    fn clean_string_no_separators_present_is_unchanged() {
        assert_eq!(clean_default("hello"), "hello");
    }

    // --- ERROR responses ---

    #[test]
    fn parse_error_response_splits_code_and_message() {
        let info = parse_error_response("ERROR 3 Not connected");
        assert_eq!(info.code, "3");
        assert_eq!(info.message, "Not connected");
    }

    #[test]
    fn parse_error_response_with_no_space_has_empty_message_without_panic() {
        // Upstream loops substr(1) past an empty string here and throws.
        let info = parse_error_response("ERROR 7");
        assert_eq!(info.code, "7");
        assert_eq!(info.message, "");
    }

    // --- OK name:value parsing ---

    #[test]
    fn parses_simple_name_value_pairs() {
        let (data, needs_more) = parse_name_value_section("ServerName:SPECS ProtocolVersion:1.11");
        assert!(!needs_more);
        assert_eq!(data.get("ServerName"), Some(&"SPECS".to_string()));
        assert_eq!(data.get("ProtocolVersion"), Some(&"1.11".to_string()));
    }

    #[test]
    fn trailing_value_without_terminating_space_is_still_complete() {
        let (data, needs_more) = parse_name_value_section("Code:3");
        assert!(!needs_more);
        assert_eq!(data.get("Code"), Some(&"3".to_string()));
    }

    #[test]
    fn quoted_value_containing_a_space_is_not_split() {
        let (data, needs_more) =
            parse_name_value_section("LensMode:\"Wide Angle Mode\" ScanRange:\"Fixed\"");
        assert!(!needs_more);
        assert_eq!(
            data.get("LensMode"),
            Some(&"\"Wide Angle Mode\"".to_string())
        );
        assert_eq!(data.get("ScanRange"), Some(&"\"Fixed\"".to_string()));
    }

    #[test]
    fn array_value_containing_a_space_is_not_split() {
        let (data, needs_more) = parse_name_value_section("Data:[1,2, 3] Code:0");
        assert!(!needs_more);
        assert_eq!(data.get("Data"), Some(&"[1,2, 3]".to_string()));
        assert_eq!(data.get("Code"), Some(&"0".to_string()));
    }

    #[test]
    fn escaped_quote_inside_value_does_not_close_the_quote() {
        let (data, needs_more) = parse_name_value_section("Message:\"a \\\" b\" Code:0");
        assert!(!needs_more);
        assert_eq!(data.get("Message"), Some(&"\"a \\\" b\"".to_string()));
        assert_eq!(data.get("Code"), Some(&"0".to_string()));
    }

    #[test]
    fn incomplete_quoted_value_requests_continuation() {
        let mut parser = NameValueParser::new();
        let (pairs, needs_more) = parser.feed("Message:\"first part");
        assert!(pairs.is_empty());
        assert!(needs_more);

        let (pairs, needs_more) = parser.feed(" second part\" Code:0");
        assert!(!needs_more);
        assert_eq!(pairs.len(), 2);
        assert_eq!(
            pairs[0],
            (
                "Message".to_string(),
                "\"first part second part\"".to_string()
            )
        );
        assert_eq!(pairs[1], ("Code".to_string(), "0".to_string()));
    }

    #[test]
    fn incomplete_array_value_requests_continuation() {
        let mut parser = NameValueParser::new();
        let (pairs, needs_more) = parser.feed("Data:[1,2,");
        assert!(pairs.is_empty());
        assert!(needs_more);

        let (pairs, needs_more) = parser.feed("3,4]");
        assert!(!needs_more);
        assert_eq!(pairs, vec![("Data".to_string(), "[1,2,3,4]".to_string())]);
    }

    #[test]
    fn escaped_quote_split_across_a_continuation_boundary_does_not_close_early() {
        // The backslash lands as the very last char of chunk 1, and the quote
        // it escapes is the very first char of chunk 2. Upstream re-indexes
        // from 0 on each chunk and would underflow `substr(index-1,1)` here;
        // this must carry the previous chunk's last char instead and must
        // NOT treat the escaped quote as closing the value.
        let mut parser = NameValueParser::new();
        let (pairs, needs_more) = parser.feed("Message:\"a \\");
        assert!(pairs.is_empty());
        assert!(needs_more);

        let (pairs, needs_more) = parser.feed("\" still inside\" Code:0");
        assert!(!needs_more);
        assert_eq!(
            pairs[0],
            ("Message".to_string(), "\"a \\\" still inside\"".to_string())
        );
        assert_eq!(pairs[1], ("Code".to_string(), "0".to_string()));
    }

    // --- ResponseAssembler ---

    #[test]
    fn response_assembler_parses_simple_ok_reply_in_one_chunk() {
        let mut asm = ResponseAssembler::new();
        assert_eq!(
            asm.begin("OK ServerName:SPECS ProtocolVersion:1.11"),
            BeginOutcome::Ok
        );
        let data = asm.finish();
        assert_eq!(data.get("ServerName"), Some(&"SPECS".to_string()));
        assert_eq!(data.get("ProtocolVersion"), Some(&"1.11".to_string()));
    }

    #[test]
    fn response_assembler_ok_with_nothing_to_parse_yields_empty_map() {
        let mut asm = ResponseAssembler::new();
        assert_eq!(asm.begin("OK"), BeginOutcome::Ok);
        assert!(asm.finish().is_empty());
    }

    #[test]
    fn response_assembler_requests_continuation_for_incomplete_value() {
        let mut asm = ResponseAssembler::new();
        assert_eq!(
            asm.begin("OK Message:\"first part"),
            BeginOutcome::NeedsMore
        );
        assert!(!asm.continue_with(" second part\" Code:0"));
        let data = asm.finish();
        assert_eq!(
            data.get("Message"),
            Some(&"\"first part second part\"".to_string())
        );
        assert_eq!(data.get("Code"), Some(&"0".to_string()));
    }

    #[test]
    fn response_assembler_treats_non_ok_prefix_as_error() {
        let mut asm = ResponseAssembler::new();
        match asm.begin("ERROR 3 Not connected") {
            BeginOutcome::Error(info) => {
                assert_eq!(info.code, "3");
                assert_eq!(info.message, "Not connected");
            }
            other => panic!("expected Error, got {other:?}"),
        }
    }

    // --- readIntegerData / readDoubleData ---

    #[test]
    fn parse_integer_field_decodes_and_rejects_non_numeric() {
        assert_eq!(parse_integer_field("42"), Some(42));
        assert_eq!(parse_integer_field(" 42 "), Some(42));
        assert_eq!(parse_integer_field("not a number"), None);
        assert_eq!(parse_integer_field(""), None);
    }

    #[test]
    fn parse_double_field_decodes_and_rejects_non_numeric() {
        assert_eq!(parse_double_field("3.25"), Some(3.25));
        assert_eq!(parse_double_field(" 3.25 "), Some(3.25));
        assert_eq!(parse_double_field("not a number"), None);
        assert_eq!(parse_double_field(""), None);
    }

    // --- data array / values list / parameter names ---

    #[test]
    fn parse_data_array_decodes_bracketed_csv_doubles() {
        assert_eq!(parse_data_array("[1.5,2.25,3]"), vec![1.5, 2.25, 3.0]);
    }

    #[test]
    fn parse_data_array_on_empty_bracket_pair_is_empty() {
        assert_eq!(parse_data_array("[]"), Vec::<f64>::new());
    }

    #[test]
    fn parse_data_array_non_numeric_token_defaults_to_zero() {
        assert_eq!(parse_data_array("[1,garbage,3]"), vec![1.0, 0.0, 3.0]);
    }

    #[test]
    fn parse_values_list_strips_quotes_and_brackets_anywhere_in_token() {
        assert_eq!(
            parse_values_list("[\"WideAngle\"],[\"Narrow\"]"),
            vec!["WideAngle".to_string(), "Narrow".to_string()]
        );
    }

    #[test]
    fn parse_parameter_names_extracts_epics_name_and_raw_name() {
        // Space/'['/']'/'/' feed only rawname, not the EPICS name (matches
        // `setupEPICSParameters`'s separate `if` branch for those characters).
        let names = parse_parameter_names("\"kinetic energy/base\",\"aux voltage\"");
        assert_eq!(
            names,
            vec![
                (
                    "KINETICENERGYBASE".to_string(),
                    "kinetic energy/base".to_string()
                ),
                ("AUXVOLTAGE".to_string(), "aux voltage".to_string()),
            ]
        );
    }

    // --- format_double (%.6g parity with C++ stringstream defaults) ---

    #[test]
    fn format_double_matches_cpp_stringstream_defaults() {
        assert_eq!(format_double(0.0), "0");
        assert_eq!(format_double(10.0), "10");
        assert_eq!(format_double(300.0), "300");
        assert_eq!(format_double(82.5), "82.5");
        assert_eq!(format_double(0.2), "0.2");
        assert_eq!(format_double(-10.0), "-10");
        assert_eq!(format_double(1.0), "1");
        assert_eq!(format_double(0.001), "0.001");
    }

    #[test]
    fn format_double_uses_scientific_outside_g_range() {
        assert_eq!(format_double(0.00001), "1e-05");
        assert_eq!(format_double(1_234_567.0), "1.23457e+06");
    }

    // --- command builders ---

    #[test]
    fn start_command_encodes_safe_after() {
        assert_eq!(start_command(true), "Start SetSafeStateAfter:\"true\"");
        assert_eq!(start_command(false), "Start SetSafeStateAfter:\"false\"");
    }

    #[test]
    fn define_fat_command_matches_expected_wire_text() {
        assert_eq!(
            define_fat_command(
                DefineFatArgs {
                    start_energy: 82.0,
                    end_energy: 86.0,
                    step_width: 0.2,
                    dwell_time: 1.0,
                    pass_energy: 10.0
                },
                "WideAngleMode",
                "Fixed"
            ),
            "DefineSpectrumFAT StartEnergy:82 EndEnergy:86 StepWidth:0.2 DwellTime:1 PassEnergy:10 LensMode:\"WideAngleMode\" ScanRange:\"Fixed\""
        );
    }

    #[test]
    fn define_sfat_command_matches_expected_wire_text() {
        assert_eq!(
            define_sfat_command(82.0, 86.0, 500, 1.0, "WideAngleMode", "Fixed"),
            "DefineSpectrumSFAT StartEnergy:82 EndEnergy:86 Samples:500 DwellTime:1 LensMode:\"WideAngleMode\" ScanRange:\"Fixed\""
        );
    }

    #[test]
    fn define_frr_command_matches_expected_wire_text() {
        assert_eq!(
            define_frr_command(
                DefineFrrArgs {
                    start_energy: 82.0,
                    end_energy: 86.0,
                    step_width: 0.2,
                    dwell_time: 1.0,
                    retarding_ratio: 10.0
                },
                "WideAngleMode",
                "Fixed"
            ),
            "DefineSpectrumFRR StartEnergy:82 EndEnergy:86 StepWidth:0.2 DwellTime:1 RetardingRatio:10 LensMode:\"WideAngleMode\" ScanRange:\"Fixed\""
        );
    }

    #[test]
    fn define_fe_command_matches_expected_wire_text() {
        assert_eq!(
            define_fe_command(300.0, 500, 1.0, 10.0, "WideAngleMode", "Fixed"),
            "DefineSpectrumFE KinEnergy:300 Samples:500 DwellTime:1 PassEnergy:10 LensMode:\"WideAngleMode\" ScanRange:\"Fixed\""
        );
    }

    #[test]
    fn get_data_command_matches_expected_wire_text() {
        assert_eq!(
            get_data_command(0, 99),
            "GetAcquisitionData FromIndex:0 ToIndex:99"
        );
    }

    #[test]
    fn value_commands_match_expected_wire_text() {
        assert_eq!(
            get_value_command("NumNonEnergyChannels"),
            "GetAnalyzerParameterValue ParameterName:\"NumNonEnergyChannels\""
        );
        assert_eq!(
            set_value_int_command("NumNonEnergyChannels", 5),
            "SetAnalyzerParameterValue ParameterName:\"NumNonEnergyChannels\" Value:5"
        );
        assert_eq!(
            set_value_double_command("SomeGain", 1.5),
            "SetAnalyzerParameterValue ParameterName:\"SomeGain\" Value:1.5"
        );
        assert_eq!(
            set_value_string_command("SomeName", "abc"),
            "SetAnalyzerParameterValue ParameterName:\"SomeName\" Value:abc"
        );
    }

    #[test]
    fn info_spectrum_and_data_info_commands_match_expected_wire_text() {
        assert_eq!(
            get_info_command("NumNonEnergyChannels"),
            "GetAnalyzerParameterInfo ParameterName:\"NumNonEnergyChannels\""
        );
        assert_eq!(
            get_spectrum_command("LensMode"),
            "GetSpectrumParameterInfo ParameterName:\"LensMode\""
        );
        assert_eq!(
            get_data_info_command("OrdinateRange"),
            "GetSpectrumDataInfo ParameterName:\"OrdinateRange\""
        );
    }
}
