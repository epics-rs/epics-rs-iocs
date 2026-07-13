//! Teledyne ISCO H-series wire format/parse helpers, transcribed byte-for-
//! byte from `teled_h.proto` (`SPApp/Db/teled_h.proto`). `Terminator = CR;`
//! plus `ExtraInput = Ignore;` (StreamDevice: extra bytes before the
//! terminator, after a successful `in` match, are ignored rather than
//! treated as a framing error) -- the latter has no analogue needed here
//! since every parser below only ever reads up to the terminator-stripped
//! reply the octet EOS layer already delivers (see
//! [`crate::connect::connect_octet`]'s module doc); there is no "extra"
//! trailing data left to ignore once EOS framing has already isolated one
//! reply.
//!
//! H shares roughly half its command set with D byte-for-byte (`ping`,
//! `getModel`, `setRem`, `setLoc`, `setStop`, `getStatus`) -- duplicated
//! here rather than imported from [`crate::wire_d`], mirroring upstream's
//! choice to ship two fully independent `.proto` files rather than one
//! shared include. Where H's reply shape genuinely diverges from D's (every
//! `get{Vol,Press,SetPress,Flow,MFlow}` returns a **raw string**, not a
//! pre-parsed float -- matching the two-stage `stringin`+`aSub` parse chain
//! `teledynePumpH.template` uses downstream), the divergence is preserved;
//! see each function's doc comment for the exact `.proto` line it
//! transcribes.
//!
//! `\?` wildcard and `\$2`-echo positions are skipped, not round-trip
//! validated -- same scoped simplification as `wire_d`'s module doc
//! explains.

use crate::checksum;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError {
    TooShort,
    UnexpectedPrefix,
    ChecksumMismatch,
    ParseFloat,
    /// `setDigital`'s enumerated choice selector was out of range (not an
    /// upstream case at all -- StreamDevice's `%{...}` converter has no
    /// "invalid choice" reply path since it's a fire-and-forget write with
    /// no `in` clause; this is purely this port's own bounds check on the
    /// incoming `Int32` selector value).
    InvalidChoice,
}

fn check_lit(payload: &[u8], offset: usize, lit: &[u8]) -> Result<usize, WireError> {
    let end = offset + lit.len();
    if payload.len() < end || &payload[offset..end] != lit {
        return Err(WireError::UnexpectedPrefix);
    }
    Ok(end)
}

fn skip(payload: &[u8], offset: usize, n: usize) -> Result<usize, WireError> {
    let end = offset + n;
    if payload.len() < end {
        return Err(WireError::TooShort);
    }
    Ok(end)
}

fn finish_checksum(payload: &[u8], body_end: usize) -> Result<(), WireError> {
    if payload.len() != body_end + 2 {
        return Err(WireError::TooShort);
    }
    if !checksum::checksum_valid(&payload[..body_end], &payload[body_end..]) {
        return Err(WireError::ChecksumMismatch);
    }
    Ok(())
}

// ---- commands byte-identical to teled_d.proto ----

/// `ping { out "\r\$1R %0<nsum>"; in "R %0<nsum>"; }`
pub fn format_ping(unit: u8) -> String {
    checksum::append_checksum(&format!("\r{unit}R "))
}

/// `getModel { out "\$1R008IDENTIFY%0<nsum>"; in "R0\?\?%#s"; }`
pub fn format_get_model(unit: u8) -> String {
    checksum::append_checksum(&format!("{unit}R008IDENTIFY"))
}

/// `setRem { out "\$1R006REMOTE%0<nsum>"; ... }`
pub fn format_set_rem(unit: u8) -> String {
    checksum::append_checksum(&format!("{unit}R006REMOTE"))
}

/// `setLoc { out "\$1R005LOCAL%0<nsum>"; ... }`
pub fn format_set_loc(unit: u8) -> String {
    checksum::append_checksum(&format!("{unit}R005LOCAL"))
}

/// `setStop { out "\$1R005STOP\$2%0<nsum>"; ... }`
pub fn format_set_stop(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R005STOP{pump}"))
}

/// `getStatus { out "\$1R007STATUS\$2%0<nsum>"; in "R0\?\?STATUS=%#s"; }`
pub fn format_get_status(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R007STATUS{pump}"))
}

/// `setPress { out "\$1R00FPRESS\$2=%08.4f%0<nsum>"; @init { getSetPress; }; }`
pub fn format_set_press(unit: u8, pump: &str, value: f64) -> String {
    checksum::append_checksum(&format!("{unit}R00FPRESS{pump}={value:08.4}"))
}

/// `setMFlow { out "\$1R00FMFLOW\$2=%08.4f%0<nsum>"; @init { getMFlow; }; }`
pub fn format_set_mflow(unit: u8, pump: &str, value: f64) -> String {
    checksum::append_checksum(&format!("{unit}R00FMFLOW{pump}={value:08.4}"))
}

pub fn parse_ack(payload: &[u8]) -> Result<(), WireError> {
    if payload.len() != 4 {
        return Err(WireError::TooShort);
    }
    check_lit(payload, 0, b"R ")?;
    finish_checksum(payload, 2)
}

/// Shared `in "R0\?\?%#s"` shape: `getModel`/`getUnit`/`getMode`/`getID`.
pub fn parse_captured_string(payload: &[u8]) -> Result<String, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    Ok(String::from_utf8_lossy(&payload[off..]).into_owned())
}

pub fn parse_get_status(payload: &[u8]) -> Result<String, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    let off = check_lit(payload, off, b"STATUS=")?;
    Ok(String::from_utf8_lossy(&payload[off..]).into_owned())
}

// ---- H-only "Controller communication" additions ----

/// `getUnit { out "\$1R006UNITS%0<nsum>"; in "R0\?\?%#s"; }`
pub fn format_get_unit(unit: u8) -> String {
    checksum::append_checksum(&format!("{unit}R006UNITS"))
}

/// `getMode { out "\$1R004MODE%0<nsum>"; in "R0\?\?%#s"; }`
pub fn format_get_mode(unit: u8) -> String {
    checksum::append_checksum(&format!("{unit}R004MODE"))
}

/// `getID { out "\$1R008IDENTIFY%0<nsum>"; in "R0\?\?%#s"; }` -- wire-
/// identical to `getModel` (confirmed by direct comparison of both
/// commands' `out`/`in` text in `teled_h.proto`).
pub fn format_get_id(unit: u8) -> String {
    format_get_model(unit)
}

// ---- H-diverging pump commands ----

/// `setRun { out "6R003RUNF0"; in "R %0<nsum>"; }` -- **preserved upstream
/// bug**: unlike every other command (and unlike D's own `setRun`), H's
/// `setRun` is a fully literal string with no unit/pump substitution and no
/// checksum appended at all. `unit`/`pump` are accepted (to keep this
/// function's signature uniform with every other command) but intentionally
/// unused, matching the `.proto`'s own hardcoding. See
/// `iocs/syringepump-ioc/db/teledynePumpH.template`'s module doc.
pub fn format_set_run(_unit: u8, _pump: &str) -> String {
    "6R003RUNF0".to_string()
}

/// `getVol { out "\$1R004VOL\$2%0<nsum>"; in "R0\?\?VOL\?=%s"; }` -- H
/// returns a **raw string**, not a pre-parsed float (unlike D's `getVol`);
/// `teledynePumpH.template` feeds this through a `stringin` DTYP plus a
/// downstream `aSub` to parse it -- ported as one flattened `asynOctetRead`
/// record, see the template's module doc.
pub fn format_get_vol(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R004VOL{pump}"))
}
pub fn parse_get_vol(payload: &[u8]) -> Result<String, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    let off = check_lit(payload, off, b"VOL")?;
    let off = skip(payload, off, 1)?;
    let off = check_lit(payload, off, b"=")?;
    Ok(String::from_utf8_lossy(&payload[off..]).into_owned())
}

/// `getSetPress { out "\$1R009SETPRESS\$2%0<nsum>"; in "R0\?\?PRESS\?=%s"; }`
pub fn format_get_set_press(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R009SETPRESS{pump}"))
}
pub fn parse_get_set_press(payload: &[u8]) -> Result<String, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    let off = check_lit(payload, off, b"PRESS")?;
    let off = skip(payload, off, 1)?;
    let off = check_lit(payload, off, b"=")?;
    Ok(String::from_utf8_lossy(&payload[off..]).into_owned())
}

/// `getPress { out "\$1R006PRESS\$2%0<nsum>"; in "R0\?\?PRESS\?=%s"; }` --
/// byte-identical `in` text to `getSetPress`.
pub fn format_get_press(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R006PRESS{pump}"))
}
pub fn parse_get_press(payload: &[u8]) -> Result<String, WireError> {
    parse_get_set_press(payload)
}

/// `getFlow { out "\$1R005FLOW\$2%0<nsum>"; in "R0\?\?FLOW\?=%s"; }` -- no
/// literal space before `%s` (unlike D's `getFlow`).
pub fn format_get_flow(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R005FLOW{pump}"))
}
pub fn parse_get_flow(payload: &[u8]) -> Result<String, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    let off = check_lit(payload, off, b"FLOW")?;
    let off = skip(payload, off, 1)?;
    let off = check_lit(payload, off, b"=")?;
    Ok(String::from_utf8_lossy(&payload[off..]).into_owned())
}

/// `getMFlow { out "\$1R006MFLOW\$2%0<nsum>"; in "R0\?\?FLOW\$2=%s"; }` --
/// uses `\$2` rather than `\?` at the skip position (both a 1-byte skip
/// here), otherwise identical wire shape to `getFlow`.
pub fn format_get_mflow(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R006MFLOW{pump}"))
}
pub fn parse_get_mflow(payload: &[u8]) -> Result<String, WireError> {
    parse_get_flow(payload)
}

/// `sendCmd { out "\$1R0%s%0<nsum>"; in "\?\?\?\?%#s"; }` -- H's `in` text
/// differs from D's: 4 skipped bytes with **no** leading "R0" literal check
/// (D checks "R0" then skips 2; H skips all 4 unchecked).
pub fn format_send_cmd(unit: u8, caller_body: &str) -> String {
    checksum::append_checksum(&format!("{unit}R0{caller_body}"))
}
pub fn parse_send_cmd_reply(payload: &[u8]) -> Result<String, WireError> {
    let off = skip(payload, 0, 4)?;
    Ok(String::from_utf8_lossy(&payload[off..]).into_owned())
}

// ---- H-only "New readbacks" section ----

/// `setRefill { out "\$1R006REFILL%0<nsum>"; in "R %0<nsum>"; }`
pub fn format_set_refill(unit: u8) -> String {
    checksum::append_checksum(&format!("{unit}R006REFILL"))
}

/// `getRefillLimit { out "\$1R007RLIMIT\$2%0<nsum>"; in "R0\?\?%*s\_%f"; }`
/// -- `%*s` skips a whitespace-delimited token (discarded), `\_` skips the
/// whitespace run after it (StreamDevice's "any amount of whitespace"
/// escape), then a float runs to the end of the reply. No checksum
/// converter in this `in` format -- unlike every float-returning D command,
/// this reply is not checksum-validated.
pub fn format_get_refill_limit(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R007RLIMIT{pump}"))
}
pub fn parse_get_refill_limit(payload: &[u8]) -> Result<f64, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    let rest = &payload[off..];
    let token_end = rest
        .iter()
        .position(|&b| b == b' ' || b == b'\t')
        .ok_or(WireError::TooShort)?;
    let mut ws_end = off + token_end;
    let ws_start = ws_end;
    while ws_end < payload.len() && (payload[ws_end] == b' ' || payload[ws_end] == b'\t') {
        ws_end += 1;
    }
    if ws_end == ws_start {
        return Err(WireError::TooShort);
    }
    let text = std::str::from_utf8(&payload[ws_end..]).map_err(|_| WireError::ParseFloat)?;
    text.trim().parse().map_err(|_| WireError::ParseFloat)
}

/// `setRefillRate { out "\$1R009REFILL=%08.4f%0<nsum>"; @init { getRefillLimit; }; }`
/// -- note: no `\$2` pump letter in the message at all (unlike `setPress`/
/// `setMFlow`, which do include it). The declared length field ("009") does
/// not arithmetically match "REFILL=" + an 8-char float (15 bytes, would be
/// "00F" following the convention every other command uses) -- preserved
/// verbatim since this port transcribes the literal `.proto` text rather
/// than computing the length field; see the crate-level UNFIXED note.
pub fn format_set_refill_rate(unit: u8, value: f64) -> String {
    checksum::append_checksum(&format!("{unit}R009REFILL={value:08.4}"))
}

/// `setDigital { out "\$1R008DIGITAL=%{HHHHHHHH|LHHHHHHH|HLHHHHHH|LLHHHHHH}%0<nsum>"; }`
/// -- StreamDevice's `%{...}` enum converter maps an integer selector
/// (0-3, e.g. from an `mbbo` record) to one of these 4 literal 8-character
/// choices, in order. No `in` clause upstream -- fire-and-forget write, no
/// reply expected. The declared length field ("008") likewise does not
/// match "DIGITAL=" (8 chars) plus an 8-char choice value (16 total);
/// preserved verbatim, same rationale as `setRefillRate`.
pub fn format_set_digital(unit: u8, choice: u8) -> Result<String, WireError> {
    const CHOICES: [&str; 4] = ["HHHHHHHH", "LHHHHHHH", "HLHHHHHH", "LLHHHHHH"];
    let value = *CHOICES
        .get(choice as usize)
        .ok_or(WireError::InvalidChoice)?;
    Ok(checksum::append_checksum(&format!(
        "{unit}R008DIGITAL={value}"
    )))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::checksum::nsum_checksum;

    fn cksum_str(body: &str) -> String {
        let cs = nsum_checksum(body.as_bytes());
        format!("{}{}", cs[0] as char, cs[1] as char)
    }

    #[test]
    fn shared_commands_match_d_series_literal_text() {
        assert_eq!(
            format_get_model(6),
            format!("6R008IDENTIFY{}", cksum_str("6R008IDENTIFY"))
        );
        assert_eq!(
            format_set_rem(6),
            format!("6R006REMOTE{}", cksum_str("6R006REMOTE"))
        );
        assert_eq!(
            format_set_loc(6),
            format!("6R005LOCAL{}", cksum_str("6R005LOCAL"))
        );
        assert_eq!(
            format_set_stop(6, "A"),
            format!("6R005STOPA{}", cksum_str("6R005STOPA"))
        );
        assert_eq!(
            format_get_status(6, "A"),
            format!("6R007STATUSA{}", cksum_str("6R007STATUSA"))
        );
    }

    #[test]
    fn get_id_is_wire_identical_to_get_model() {
        assert_eq!(format_get_id(6), format_get_model(6));
    }

    #[test]
    fn get_unit_and_get_mode_match_proto_literals() {
        assert_eq!(
            format_get_unit(6),
            format!("6R006UNITS{}", cksum_str("6R006UNITS"))
        );
        assert_eq!(
            format_get_mode(6),
            format!("6R004MODE{}", cksum_str("6R004MODE"))
        );
    }

    #[test]
    fn set_run_is_fully_literal_regardless_of_unit_or_pump() {
        assert_eq!(format_set_run(6, "A"), "6R003RUNF0");
        assert_eq!(format_set_run(9, "B"), "6R003RUNF0");
    }

    #[test]
    fn get_vol_get_press_get_set_press_return_raw_strings() {
        assert_eq!(parse_get_vol(b"R0??VOL?=12.34"), Ok("12.34".to_string()));
        assert_eq!(
            parse_get_set_press(b"R0??PRESS?=100.0"),
            Ok("100.0".to_string())
        );
        assert_eq!(
            parse_get_press(b"R0??PRESS?=100.0"),
            Ok("100.0".to_string())
        );
    }

    #[test]
    fn get_flow_has_no_leading_space_before_captured_text() {
        assert_eq!(parse_get_flow(b"R0??FLOW?=3.50"), Ok("3.50".to_string()));
        assert_eq!(parse_get_mflow(b"R0??FLOW?=3.50"), Ok("3.50".to_string()));
    }

    #[test]
    fn send_cmd_reply_skips_four_bytes_unconditionally() {
        assert_eq!(parse_send_cmd_reply(b"XXXXhello"), Ok("hello".to_string()));
    }

    #[test]
    fn set_refill_and_get_refill_limit_match_proto_literals() {
        assert_eq!(
            format_set_refill(6),
            format!("6R006REFILL{}", cksum_str("6R006REFILL"))
        );
        assert_eq!(
            format_get_refill_limit(6, "A"),
            format!("6R007RLIMITA{}", cksum_str("6R007RLIMITA"))
        );
    }

    #[test]
    fn get_refill_limit_skips_token_then_whitespace_then_parses_float() {
        assert_eq!(parse_get_refill_limit(b"R0??LIMIT 12.5000"), Ok(12.5));
        // Multiple whitespace bytes between token and float.
        assert_eq!(parse_get_refill_limit(b"R0??LIMIT   3.0"), Ok(3.0));
    }

    #[test]
    fn set_refill_rate_has_no_pump_letter_in_message() {
        let frame = format_set_refill_rate(6, 12.5);
        assert_eq!(
            frame,
            format!("6R009REFILL=012.5000{}", cksum_str("6R009REFILL=012.5000"))
        );
    }

    #[test]
    fn set_digital_maps_selector_to_enumerated_choice() {
        assert_eq!(
            format_set_digital(6, 0).unwrap(),
            format!(
                "6R008DIGITAL=HHHHHHHH{}",
                cksum_str("6R008DIGITAL=HHHHHHHH")
            )
        );
        assert_eq!(
            format_set_digital(6, 3).unwrap(),
            format!(
                "6R008DIGITAL=LLHHHHHH{}",
                cksum_str("6R008DIGITAL=LLHHHHHH")
            )
        );
        assert_eq!(format_set_digital(6, 4), Err(WireError::InvalidChoice));
    }

    #[test]
    fn parse_ack_and_captured_string_behave_like_d_series() {
        let body = "R ";
        let frame = format!("{body}{}", cksum_str(body));
        assert_eq!(parse_ack(frame.as_bytes()), Ok(()));
        assert_eq!(parse_captured_string(b"R0??hello"), Ok("hello".to_string()));
    }
}
