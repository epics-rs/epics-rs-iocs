//! L0 config-port wire format/reply-parse functions, transcribed byte-for-byte
//! from `capaNCDT6200Sup/Db/capaNCDT6200.proto` (`InTerminator = CR LF;
//! OutTerminator = CR;` -- both terminators are applied by the port's EOS,
//! never part of the strings here). Every function pair mirrors one `.proto`
//! command block; `ExtraInput = Ignore` (set on every block in the original)
//! means the framework accepts and discards trailing bytes past what a
//! command's own format string consumes -- the parse functions below stop
//! exactly where the corresponding StreamDevice format directive would, not
//! where a literal "OK" happens to sit, since two directives are field
//! character-classes rather than literal suffixes (see
//! [`parse_chan_info`] and [`parse_query_ver`]'s docs).
//!
//! # Preserved `.proto` oddity: `[^OK]`/`[^,\s]` are character classes
//! `queryChan{1,2,3,4}Info`'s `UNT%(...)[^OK]` is StreamDevice glob syntax
//! for "capture characters that are none of {O, K}" -- a *character class*
//! exclusion, not "stop at the literal substring OK". A unit string
//! containing an 'O' or 'K' anywhere (e.g. a hypothetical "Ohm") would
//! truncate at that character. In practice every real capaNCDT6200 unit
//! string is "mm" or similar and never hits this, but [`parse_chan_info`]
//! reproduces the exact class-based stop rather than a literal-suffix strip,
//! since that's the wire-parity-correct translation of the `.proto` glob.
//! Likewise `NAM%(...)[^,\s]` stops at the first comma OR whitespace
//! character, not a literal delimiter string.

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError(pub String);

fn err(msg: impl Into<String>) -> ParseError {
    ParseError(msg.into())
}

fn as_str(reply: &[u8]) -> Result<&str, ParseError> {
    std::str::from_utf8(reply).map_err(|_| err("reply is not valid UTF-8"))
}

fn strip_prefix<'a>(s: &'a str, prefix: &str) -> Result<&'a str, ParseError> {
    s.strip_prefix(prefix)
        .ok_or_else(|| err(format!("expected prefix {prefix:?}, got {s:?}")))
}

fn strip_suffix<'a>(s: &'a str, suffix: &str) -> Result<&'a str, ParseError> {
    s.strip_suffix(suffix)
        .ok_or_else(|| err(format!("expected suffix {suffix:?}, got {s:?}")))
}

/// `%d` -- an optionally-signed decimal integer, stopping at the first
/// non-digit character (comma, letter, end of string). Returns the parsed
/// value and the unconsumed remainder.
fn take_int(s: &str) -> Result<(i64, &str), ParseError> {
    let mut end = 0;
    let bytes = s.as_bytes();
    if end < bytes.len() && (bytes[end] == b'-' || bytes[end] == b'+') {
        end += 1;
    }
    let digits_start = end;
    while end < bytes.len() && bytes[end].is_ascii_digit() {
        end += 1;
    }
    if end == digits_start {
        return Err(err(format!("expected a decimal integer in {s:?}")));
    }
    let value: i64 = s[..end]
        .parse()
        .map_err(|_| err(format!("integer out of range in {s:?}")))?;
    Ok((value, &s[end..]))
}

/// `[^,\s]` -- capture until the first comma or whitespace character.
fn take_until_comma_or_space(s: &str) -> (&str, &str) {
    let idx = s
        .find(|c: char| c == ',' || c.is_whitespace())
        .unwrap_or(s.len());
    s.split_at(idx)
}

/// `[^OK]` -- capture until the first 'O' or 'K' character (see the module
/// doc's "Preserved `.proto` oddity" section -- a character-class
/// exclusion, not a literal-substring stop).
fn take_until_o_or_k(s: &str) -> (&str, &str) {
    let idx = s.find(['O', 'K']).unwrap_or(s.len());
    s.split_at(idx)
}

// --- grabWelcomeText: no `out` clause; pure "read whatever line is
// waiting" primitive, reused by both `welcome` and `measDataM`. ---

pub fn parse_welcome_text(reply: &[u8]) -> Result<String, ParseError> {
    Ok(as_str(reply)?.to_string())
}

// --- queryVer ---

pub fn format_query_ver() -> String {
    "$VER".to_string()
}

/// `in "$VER%(...)[^;];" "%(...)[^OK]OK";` -- version1M's own VAL is the
/// text before the first `;`; version2M's VAL (a Soft Channel record with no
/// `INP` of its own) is cross-written with the text before the trailing
/// `OK`. Returns `(version1, version2)`.
pub fn parse_query_ver(reply: &[u8]) -> Result<(String, String), ParseError> {
    let s = strip_prefix(as_str(reply)?, "$VER")?;
    let (v1, rest) = s
        .split_once(';')
        .ok_or_else(|| err(format!("expected ';' separator in {s:?}")))?;
    let v2 = strip_suffix(rest, "OK")?;
    Ok((v1.to_string(), v2.to_string()))
}

// --- querySampleTime / setSampleTime ---

pub fn format_query_sample_time() -> String {
    "$STI?".to_string()
}

pub fn parse_query_sample_time(reply: &[u8]) -> Result<i32, ParseError> {
    let s = strip_prefix(as_str(reply)?, "$STI?")?;
    let s = strip_suffix(s, "OK")?;
    let (v, rest) = take_int(s)?;
    if !rest.is_empty() {
        return Err(err(format!("unexpected trailing bytes {rest:?}")));
    }
    Ok(v as i32)
}

pub fn format_set_sample_time(value: i32) -> String {
    format!("$STI{value}")
}

/// Ack shape is `"$STI%d,%dOK"` -- two comma-separated integers, unlike
/// every other `set*` ack in this file (a single integer). Both values are
/// validated for wire-format correctness but neither is otherwise
/// interpreted -- `sampleTimeC` is a plain trigger `mbbo`, not read back.
pub fn parse_set_sample_time_ack(reply: &[u8]) -> Result<(i32, i32), ParseError> {
    let s = strip_prefix(as_str(reply)?, "$STI")?;
    let (a, rest) = take_int(s)?;
    let rest = strip_prefix(rest, ",")?;
    let (b, rest) = take_int(rest)?;
    let rest = strip_suffix(rest, "OK")?;
    if !rest.is_empty() {
        return Err(err(format!("unexpected trailing bytes {rest:?}")));
    }
    Ok((a as i32, b as i32))
}

// --- Generic single-`%d`-ack query/set pair, shared shape for
// TrigMode/AvgTypeMode/AvgNumMode/DataPort/AnalogFilter and the
// LinearMode/LinearPoint/ClearMathFunc/AnalogFilter `set*` acks. ---

fn parse_prefixed_int(reply: &[u8], prefix: &str, suffix: &str) -> Result<i32, ParseError> {
    let s = strip_prefix(as_str(reply)?, prefix)?;
    let s = strip_suffix(s, suffix)?;
    let (v, rest) = take_int(s)?;
    if !rest.is_empty() {
        return Err(err(format!("unexpected trailing bytes {rest:?}")));
    }
    Ok(v as i32)
}

// --- queryTrigMode / setTrigMode ---

pub fn format_query_trig_mode() -> String {
    "$TRG?".to_string()
}

pub fn parse_query_trig_mode(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$TRG?", "OK")
}

pub fn format_set_trig_mode(value: i32) -> String {
    format!("$TRG{value}")
}

pub fn parse_set_trig_mode_ack(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$TRG", "OK")
}

// --- queryAvgTypeMode / setAvgTypeMode ---

pub fn format_query_avg_type_mode() -> String {
    "$AVT?".to_string()
}

pub fn parse_query_avg_type_mode(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$AVT?", "OK")
}

pub fn format_set_avg_type_mode(value: i32) -> String {
    format!("$AVT{value}")
}

pub fn parse_set_avg_type_mode_ack(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$AVT", "OK")
}

// --- queryAvgNumMode / setAvgNumMode ---

pub fn format_query_avg_num_mode() -> String {
    "$AVN?".to_string()
}

pub fn parse_query_avg_num_mode(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$AVN?", "OK")
}

pub fn format_set_avg_num_mode(value: i32) -> String {
    format!("$AVN{value}")
}

pub fn parse_set_avg_num_mode_ack(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$AVN", "OK")
}

// --- queryChanStatus / queryStatus: both fan out into the same four
// chan{1,2,3,4}StatM targets -- a genuine duplicate-target design (two
// distinct wire commands, `$CHS` and `$STS`), not a typo. ---

pub fn format_query_chan_status() -> String {
    "$CHS".to_string()
}

pub fn format_query_status() -> String {
    "$STS".to_string()
}

fn parse_four_ints(reply: &[u8], prefix: &str) -> Result<[i32; 4], ParseError> {
    let s = strip_prefix(as_str(reply)?, prefix)?;
    let (a, rest) = take_int(s)?;
    let rest = strip_prefix(rest, ",")?;
    let (b, rest) = take_int(rest)?;
    let rest = strip_prefix(rest, ",")?;
    let (c, rest) = take_int(rest)?;
    let rest = strip_prefix(rest, ",")?;
    let (d, rest) = take_int(rest)?;
    let rest = strip_suffix(rest, "OK")?;
    if !rest.is_empty() {
        return Err(err(format!("unexpected trailing bytes {rest:?}")));
    }
    Ok([a as i32, b as i32, c as i32, d as i32])
}

pub fn parse_chan_status(reply: &[u8]) -> Result<[i32; 4], ParseError> {
    parse_four_ints(reply, "$CHS")
}

pub fn parse_status(reply: &[u8]) -> Result<[i32; 4], ParseError> {
    parse_four_ints(reply, "$STS")
}

// --- queryChan{1,2,3,4}Info ---

/// Parsed reply of `queryChanNInfo`: article number, name, serial number,
/// measuring-range offset, measuring range, and unit string.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChanInfo {
    pub article_number: i32,
    pub name: String,
    pub serial_number: i32,
    pub offset: i32,
    pub range: i32,
    pub unit: String,
}

pub fn format_query_chan_info(channel: u8) -> String {
    format!("$CHI{channel}")
}

/// `in "$CHIn:ANO%d,NAM%[^,\s],SNO%d,OFS%d,RNG%d,UNT%[^OK]";` -- see the
/// module doc for why `NAM`/`UNT` stop at a character class, not a literal
/// delimiter, and why the trailing "OK" is deliberately left unconsumed
/// (`ExtraInput = Ignore`).
pub fn parse_chan_info(reply: &[u8], channel: u8) -> Result<ChanInfo, ParseError> {
    let s = as_str(reply)?;
    let s = strip_prefix(s, &format!("$CHI{channel}:ANO"))?;
    let (article_number, s) = take_int(s)?;
    let s = strip_prefix(s, ",NAM")?;
    let (name, s) = take_until_comma_or_space(s);
    let s = strip_prefix(s, ",SNO")?;
    let (serial_number, s) = take_int(s)?;
    let s = strip_prefix(s, ",OFS")?;
    let (offset, s) = take_int(s)?;
    let s = strip_prefix(s, ",RNG")?;
    let (range, s) = take_int(s)?;
    let s = strip_prefix(s, ",UNT")?;
    let (unit, _rest) = take_until_o_or_k(s);
    Ok(ChanInfo {
        article_number: article_number as i32,
        name: name.to_string(),
        serial_number: serial_number as i32,
        offset: offset as i32,
        range: range as i32,
        unit: unit.to_string(),
    })
}

// --- queryLinMode ---

pub fn format_query_lin_mode() -> String {
    "$LIN?".to_string()
}

pub fn parse_lin_mode(reply: &[u8]) -> Result<[i32; 4], ParseError> {
    parse_four_ints(reply, "$LIN?")
}

// --- setCh{1,2,3,4}LinearMode ---

pub fn format_set_linear_mode(channel: u8, value: i32) -> String {
    format!("$LIN{channel}:{value}")
}

pub fn parse_set_linear_mode_ack(reply: &[u8], channel: u8) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, &format!("$LIN{channel}:"), "OK")
}

// --- clearMathFunc: one wire command shared by 4 records, each supplying
// the channel number as its own hardcoded VAL (see `config_driver`'s doc).

pub fn format_clear_math_func(channel: i32) -> String {
    format!("$CMF{channel}")
}

pub fn parse_clear_math_func_ack(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$CMF", "OK")
}

// --- queryDataPort / setDataPort ---

pub fn format_query_data_port() -> String {
    "$GDP".to_string()
}

pub fn parse_query_data_port(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$GDP", "OK")
}

pub fn format_set_data_port(value: i32) -> String {
    format!("$SDP{value}")
}

pub fn parse_set_data_port_ack(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$SDP", "OK")
}

// --- queryMeasData: the one command whose ack literal ("OK") comes BEFORE
// the value instead of after -- every other command in this file is
// `<echo>OK`; this alone is `OK<value>`. ---

pub fn format_query_meas_data() -> String {
    "$GMD".to_string()
}

pub fn parse_query_meas_data(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$GMDOK", "")
}

// --- setCh{1,2,3,4}LinearPoint ---

pub fn format_set_linear_point(channel: u8, value: i32) -> String {
    format!("$SLP{channel}:{value}")
}

pub fn parse_set_linear_point_ack(reply: &[u8], channel: u8) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, &format!("$SLP{channel}:"), "OK")
}

// --- queryAnalogFilter / setAnalogFilter ---

pub fn format_query_analog_filter() -> String {
    "$ALP?".to_string()
}

pub fn parse_query_analog_filter(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$ALP?", "OK")
}

pub fn format_set_analog_filter(value: i32) -> String {
    format!("$ALP{value}")
}

pub fn parse_set_analog_filter_ack(reply: &[u8]) -> Result<i32, ParseError> {
    parse_prefixed_int(reply, "$ALP", "OK")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn welcome_text_passes_through_verbatim() {
        assert_eq!(
            parse_welcome_text(b"MicroEpsilon capaNCDT6200 v1.0").unwrap(),
            "MicroEpsilon capaNCDT6200 v1.0"
        );
    }

    #[test]
    fn query_ver_splits_own_and_cross_written_value() {
        assert_eq!(format_query_ver(), "$VER");
        let (v1, v2) = parse_query_ver(b"$VERApp1.2;Boot3.4OK").unwrap();
        assert_eq!(v1, "App1.2");
        assert_eq!(v2, "Boot3.4");
    }

    #[test]
    fn query_ver_rejects_missing_separator() {
        assert!(parse_query_ver(b"$VERApp1.2Boot3.4OK").is_err());
    }

    #[test]
    fn sample_time_query_and_set_roundtrip() {
        assert_eq!(format_query_sample_time(), "$STI?");
        assert_eq!(parse_query_sample_time(b"$STI?96000OK").unwrap(), 96000);
        assert_eq!(format_set_sample_time(96000), "$STI96000");
        assert_eq!(
            parse_set_sample_time_ack(b"$STI96000,5OK").unwrap(),
            (96000, 5)
        );
    }

    #[test]
    fn trig_mode_query_and_set_roundtrip() {
        assert_eq!(format_query_trig_mode(), "$TRG?");
        assert_eq!(parse_query_trig_mode(b"$TRG?2OK").unwrap(), 2);
        assert_eq!(format_set_trig_mode(2), "$TRG2");
        assert_eq!(parse_set_trig_mode_ack(b"$TRG2OK").unwrap(), 2);
    }

    #[test]
    fn avg_type_mode_query_and_set_roundtrip() {
        assert_eq!(format_query_avg_type_mode(), "$AVT?");
        assert_eq!(parse_query_avg_type_mode(b"$AVT?1OK").unwrap(), 1);
        assert_eq!(format_set_avg_type_mode(1), "$AVT1");
        assert_eq!(parse_set_avg_type_mode_ack(b"$AVT1OK").unwrap(), 1);
    }

    #[test]
    fn avg_num_mode_query_and_set_roundtrip() {
        assert_eq!(format_query_avg_num_mode(), "$AVN?");
        assert_eq!(parse_query_avg_num_mode(b"$AVN?8OK").unwrap(), 8);
        assert_eq!(format_set_avg_num_mode(8), "$AVN8");
        assert_eq!(parse_set_avg_num_mode_ack(b"$AVN8OK").unwrap(), 8);
    }

    #[test]
    fn chan_status_and_status_share_reply_shape_but_distinct_wire_commands() {
        assert_eq!(format_query_chan_status(), "$CHS");
        assert_eq!(format_query_status(), "$STS");
        assert_eq!(parse_chan_status(b"$CHS1,1,0,2OK").unwrap(), [1, 1, 0, 2]);
        assert_eq!(parse_status(b"$STS1,1,0,2OK").unwrap(), [1, 1, 0, 2]);
        // Same reply shape, wrong literal prefix must not cross-parse.
        assert!(parse_chan_status(b"$STS1,1,0,2OK").is_err());
    }

    #[test]
    fn chan_info_parses_every_field() {
        assert_eq!(format_query_chan_info(1), "$CHI1");
        let info =
            parse_chan_info(b"$CHI1:ANO5,NAMDisplacement,SNO12345,OFS0,RNG2,UNTmmOK", 1).unwrap();
        assert_eq!(
            info,
            ChanInfo {
                article_number: 5,
                name: "Displacement".to_string(),
                serial_number: 12345,
                offset: 0,
                range: 2,
                unit: "mm".to_string(),
            }
        );
    }

    #[test]
    fn chan_info_wrong_channel_number_is_rejected() {
        assert!(parse_chan_info(b"$CHI1:ANO5,NAMx,SNO1,OFS0,RNG2,UNTmmOK", 2).is_err());
    }

    #[test]
    fn chan_info_name_embedded_space_breaks_the_wire_format_upstream_too() {
        // `.proto`: `"NAM%(...)[^,\s],"` -- the capture stops at the FIRST
        // comma or whitespace, and the format then demands a literal ","
        // immediately after. A name containing an embedded space (e.g.
        // "foo bar") is captured only up to "foo"; the very next wire byte
        // is the space, not the required ",", so the format itself fails
        // to match. This is an upstream protocol limitation (any real
        // device name with a space breaks its own StreamDevice format),
        // faithfully reproduced here as a parse error rather than silently
        // accepting "foo" and discarding " bar".
        assert!(parse_chan_info(b"$CHI2:ANO1,NAMfoo bar,SNO1,OFS0,RNG1,UNTmmOK", 2).is_err());
    }

    #[test]
    fn chan_info_unit_stops_at_first_o_or_k_character_not_literal_ok() {
        // Preserved `.proto` oddity: `[^OK]` is a character-class exclusion.
        // A unit string starting with 'O' (like a hypothetical "Ohm")
        // truncates to empty immediately -- reproduced here with a fixture
        // that pins the class-based stop, not a literal "OK" strip.
        let info = parse_chan_info(b"$CHI3:ANO1,NAMx,SNO1,OFS0,RNG1,UNTOhmOK", 3).unwrap();
        assert_eq!(info.unit, "");
    }

    #[test]
    fn lin_mode_query_and_set_roundtrip() {
        assert_eq!(format_query_lin_mode(), "$LIN?");
        assert_eq!(parse_lin_mode(b"$LIN?0,1,2,3OK").unwrap(), [0, 1, 2, 3]);
        assert_eq!(format_set_linear_mode(1, 2), "$LIN1:2");
        assert_eq!(parse_set_linear_mode_ack(b"$LIN1:2OK", 1).unwrap(), 2);
        assert_eq!(format_set_linear_mode(4, 0), "$LIN4:0");
        assert_eq!(parse_set_linear_mode_ack(b"$LIN4:0OK", 4).unwrap(), 0);
    }

    #[test]
    fn clear_math_func_channel_carried_in_write_value() {
        assert_eq!(format_clear_math_func(3), "$CMF3");
        assert_eq!(parse_clear_math_func_ack(b"$CMF3OK").unwrap(), 3);
    }

    #[test]
    fn data_port_query_and_set_roundtrip() {
        assert_eq!(format_query_data_port(), "$GDP");
        assert_eq!(parse_query_data_port(b"$GDP7001OK").unwrap(), 7001);
        assert_eq!(format_set_data_port(7001), "$SDP7001");
        assert_eq!(parse_set_data_port_ack(b"$SDP7001OK").unwrap(), 7001);
    }

    #[test]
    fn meas_data_ack_literal_precedes_value_unlike_every_other_command() {
        assert_eq!(format_query_meas_data(), "$GMD");
        assert_eq!(parse_query_meas_data(b"$GMDOK42").unwrap(), 42);
        // The "OK-then-value" shape must not be confused with the ordinary
        // "value-then-OK" shape used everywhere else.
        assert!(parse_query_meas_data(b"$GMD42OK").is_err());
    }

    #[test]
    fn linear_point_set_roundtrip() {
        assert_eq!(format_set_linear_point(2, 5), "$SLP2:5");
        assert_eq!(parse_set_linear_point_ack(b"$SLP2:5OK", 2).unwrap(), 5);
    }

    #[test]
    fn analog_filter_query_and_set_roundtrip() {
        assert_eq!(format_query_analog_filter(), "$ALP?");
        assert_eq!(parse_query_analog_filter(b"$ALP?1OK").unwrap(), 1);
        assert_eq!(format_set_analog_filter(1), "$ALP1");
        assert_eq!(parse_set_analog_filter_ack(b"$ALP1OK").unwrap(), 1);
    }

    #[test]
    fn malformed_replies_are_rejected_not_panicking() {
        assert!(parse_query_sample_time(b"garbage").is_err());
        assert!(parse_chan_status(b"$CHS1,2,3OK").is_err()); // only 3 ints
        assert!(parse_four_ints(b"$CHS1,2,3,4,5OK", "$CHS").is_err()); // 5 ints
    }
}
