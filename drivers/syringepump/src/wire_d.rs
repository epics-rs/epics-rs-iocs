//! Teledyne ISCO D-series wire format/parse helpers, transcribed byte-for-
//! byte from `teled_d.proto` (`SPApp/Db/teled_d.proto`, upstream
//! `epics-modules/SyringePump`). `Terminator = CR;` is the file's only
//! framing directive -- both directions use CR, applied by
//! [`crate::connect::connect_octet`] as the underlying octet port's EOS
//! (driver-owned; no Teledyne `st.cmd` ships upstream to have set it
//! instead -- see [`crate::connect`]'s module doc).
//!
//! Every `out` format ends in StreamDevice's `%0<nsum>` checksum converter,
//! which -- per StreamDevice's documented semantics -- checksums the entire
//! rendered message up to that point (see [`crate::checksum`]'s module doc).
//! [`checksum::append_checksum`] reproduces this by taking the fully
//! rendered pre-checksum body and appending its checksum.
//!
//! `\?` wildcard positions and `\$2` echo-validation positions in `in`
//! formats are both treated as a fixed-width **skip** here (accepted
//! without content validation) rather than round-tripped against the value
//! sent -- these positions carry no information this port consumes (the
//! device's own length-field echo and pump-letter echo), and skipping them
//! keeps parsing simple without weakening wire-parity for anything
//! observable at an EPICS record. This is a scoped simplification, not a
//! fabrication: every literal byte and every checksum IS validated.
//!
//! `\$1` (unit, single digit) and `\$2` (pump letter, "A"/"B"/"C"/"D"/"AB"/
//! "CD") are the two protocol.rs-provided parameters common to nearly every
//! command.

use crate::checksum;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError {
    /// Reply shorter than the fixed-shape prefix/suffix this parser expects.
    TooShort,
    /// A literal byte range didn't match the expected text.
    UnexpectedPrefix,
    /// The trailing 2-byte `%0<nsum>` checksum didn't match.
    ChecksumMismatch,
    /// The captured numeric field wasn't a valid float.
    ParseFloat,
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

/// Parses a float running from `offset` up to (not including) the next
/// literal space, returning the value and the index of that space.
fn parse_float_after(payload: &[u8], offset: usize) -> Result<(f64, usize), WireError> {
    let rest = &payload[offset..];
    let space_pos = rest
        .iter()
        .position(|&b| b == b' ')
        .ok_or(WireError::TooShort)?;
    let text = std::str::from_utf8(&rest[..space_pos]).map_err(|_| WireError::ParseFloat)?;
    let value: f64 = text.trim().parse().map_err(|_| WireError::ParseFloat)?;
    Ok((value, offset + space_pos))
}

/// Validates the trailing `%0<nsum>` checksum, which covers
/// `payload[..space_offset+1]` (through the literal space right before it).
fn finish_checksum(payload: &[u8], space_offset: usize) -> Result<(), WireError> {
    let checksum_start = space_offset + 1;
    if payload.len() != checksum_start + 2 {
        return Err(WireError::TooShort);
    }
    if !checksum::checksum_valid(&payload[..checksum_start], &payload[checksum_start..]) {
        return Err(WireError::ChecksumMismatch);
    }
    Ok(())
}

/// `ping { out "\r\$1R %0<nsum>"; in "R %0<nsum>"; }` -- note the literal
/// leading CR *inside* the message body, distinct from the terminator the
/// octet port appends after every write.
pub fn format_ping(unit: u8) -> String {
    checksum::append_checksum(&format!("\r{unit}R "))
}

/// `getModel { out "\$1R008IDENTIFY%0<nsum>"; ... }`
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

/// `setRun { out "\$1R004RUN\$2%0<nsum>"; ... }` -- the only Teledyne
/// command in either `.proto` file whose db template link properly
/// parameterizes the pump letter by instantiation (see
/// `iocs/syringepump-ioc/db/teledynePumpD.template`'s module doc for the
/// preserved upstream bug where every *other* command hardcodes pump "A").
pub fn format_set_run(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R004RUN{pump}"))
}

/// `setStop { out "\$1R005STOP\$2%0<nsum>"; ... }`
pub fn format_set_stop(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R005STOP{pump}"))
}

/// `getVol { out "\$1R004VOL\$2%0<nsum>"; ... }`
pub fn format_get_vol(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R004VOL{pump}"))
}

/// `getSetPress { out "\$1R009SETPRESS\$2%0<nsum>"; ... }`
pub fn format_get_set_press(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R009SETPRESS{pump}"))
}

/// `setPress { out "\$1R00FPRESS\$2=%08.4f%0<nsum>"; @init { getSetPress; }; }`
/// -- the `@init` auto-readback is the IOC's concern (issue a `getSetPress`
/// after every `setPress` write), not this format function's.
pub fn format_set_press(unit: u8, pump: &str, value: f64) -> String {
    checksum::append_checksum(&format!("{unit}R00FPRESS{pump}={value:08.4}"))
}

/// `getPress { out "\$1R006PRESS\$2%0<nsum>"; ... }`
pub fn format_get_press(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R006PRESS{pump}"))
}

/// `getFlow { out "\$1R005FLOW\$2%0<nsum>"; ... }`
pub fn format_get_flow(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R005FLOW{pump}"))
}

/// `getMFlow { out "\$1R006MFLOW\$2%0<nsum>"; ... }`
pub fn format_get_mflow(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R006MFLOW{pump}"))
}

/// `setMFlow { out "\$1R00FMFLOW\$2=%08.4f%0<nsum>"; @init { getMFlow; }; }`
pub fn format_set_mflow(unit: u8, pump: &str, value: f64) -> String {
    checksum::append_checksum(&format!("{unit}R00FMFLOW{pump}={value:08.4}"))
}

/// `getStatus { out "\$1R007STATUS\$2%0<nsum>"; ... }`
pub fn format_get_status(unit: u8, pump: &str) -> String {
    checksum::append_checksum(&format!("{unit}R007STATUS{pump}"))
}

/// `sendCmd { out "\$1R0%s%0<nsum>"; ... }` -- upstream's own comment: "it
/// requires the first two chars of the send string to be the length, so
/// external formatting is required" -- `caller_body` must already carry
/// that 2-digit length prefix as its own first 2 characters.
pub fn format_send_cmd(unit: u8, caller_body: &str) -> String {
    checksum::append_checksum(&format!("{unit}R0{caller_body}"))
}

/// `in "R %0<nsum>"` -- the generic acknowledgement reply shared by every
/// `set*`/`ping` command with no data payload.
pub fn parse_ack(payload: &[u8]) -> Result<(), WireError> {
    if payload.len() != 4 {
        return Err(WireError::TooShort);
    }
    check_lit(payload, 0, b"R ")?;
    finish_checksum(payload, 1)
}

/// `getModel`/`sendCmd`'s shared `in "R0\?\?%#s"` shape: 2 literal bytes, 2
/// skipped (length-field echo), then free-form captured text (no checksum).
pub fn parse_captured_string(payload: &[u8]) -> Result<String, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    Ok(String::from_utf8_lossy(&payload[off..]).into_owned())
}

/// `getStatus { ...; in "R0\?\?STATUS=%#s"; }`
pub fn parse_get_status(payload: &[u8]) -> Result<String, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    let off = check_lit(payload, off, b"STATUS=")?;
    Ok(String::from_utf8_lossy(&payload[off..]).into_owned())
}

/// `getVol { ...; in "R0\?\?VOL\$2=%f %0<nsum>"; }`
pub fn parse_get_vol(payload: &[u8]) -> Result<f64, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    let off = check_lit(payload, off, b"VOL")?;
    let off = skip(payload, off, 1)?;
    let off = check_lit(payload, off, b"=")?;
    let (value, space_off) = parse_float_after(payload, off)?;
    finish_checksum(payload, space_off)?;
    Ok(value)
}

/// `getSetPress { ...; in "R0\?\?PRESS\$2=%f %0<nsum>"; }`
pub fn parse_get_set_press(payload: &[u8]) -> Result<f64, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    let off = check_lit(payload, off, b"PRESS")?;
    let off = skip(payload, off, 1)?;
    let off = check_lit(payload, off, b"=")?;
    let (value, space_off) = parse_float_after(payload, off)?;
    finish_checksum(payload, space_off)?;
    Ok(value)
}

/// `getPress { ...; in "R0\?\?PRESS\$2=%f %0<nsum>"; }` -- byte-identical
/// `in` text to `getSetPress` in `teled_d.proto`; the two are only
/// distinguished by which command was just issued, not by reply shape.
pub fn parse_get_press(payload: &[u8]) -> Result<f64, WireError> {
    parse_get_set_press(payload)
}

/// `getFlow { ...; in "R0\?\?FLOW\$2= %f %0<nsum>"; }` -- note the literal
/// space between `=` and `%f`, absent from `getVol`/`getSetPress`/
/// `getPress`.
pub fn parse_get_flow(payload: &[u8]) -> Result<f64, WireError> {
    let off = check_lit(payload, 0, b"R0")?;
    let off = skip(payload, off, 2)?;
    let off = check_lit(payload, off, b"FLOW")?;
    let off = skip(payload, off, 1)?;
    let off = check_lit(payload, off, b"= ")?;
    let (value, space_off) = parse_float_after(payload, off)?;
    finish_checksum(payload, space_off)?;
    Ok(value)
}

/// `getMFlow { ...; in "R0\?\?FLOW\$2= %f %0<nsum>"; }` -- byte-identical
/// `in` text to `getFlow` (both key off the device's "FLOW" reply tag).
pub fn parse_get_mflow(payload: &[u8]) -> Result<f64, WireError> {
    parse_get_flow(payload)
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
    fn format_ping_has_leading_cr_and_trailing_space() {
        let frame = format_ping(6);
        assert!(frame.starts_with("\r6R "));
        assert_eq!(frame, format!("\r6R {}", cksum_str("\r6R ")));
    }

    #[test]
    fn format_get_model_matches_proto_literal() {
        assert_eq!(
            format_get_model(6),
            format!("6R008IDENTIFY{}", cksum_str("6R008IDENTIFY"))
        );
    }

    #[test]
    fn format_set_rem_and_set_loc_match_proto_literals() {
        assert_eq!(
            format_set_rem(6),
            format!("6R006REMOTE{}", cksum_str("6R006REMOTE"))
        );
        assert_eq!(
            format_set_loc(6),
            format!("6R005LOCAL{}", cksum_str("6R005LOCAL"))
        );
    }

    #[test]
    fn format_set_run_and_set_stop_carry_pump_letter() {
        assert_eq!(
            format_set_run(6, "A"),
            format!("6R004RUNA{}", cksum_str("6R004RUNA"))
        );
        assert_eq!(
            format_set_run(6, "B"),
            format!("6R004RUNB{}", cksum_str("6R004RUNB"))
        );
        assert_eq!(
            format_set_stop(6, "A"),
            format!("6R005STOPA{}", cksum_str("6R005STOPA"))
        );
    }

    #[test]
    fn format_get_vol_get_set_press_match_proto_literals() {
        assert_eq!(
            format_get_vol(6, "A"),
            format!("6R004VOLA{}", cksum_str("6R004VOLA"))
        );
        assert_eq!(
            format_get_set_press(6, "A"),
            format!("6R009SETPRESSA{}", cksum_str("6R009SETPRESSA"))
        );
    }

    #[test]
    fn format_set_press_zero_pads_to_eight_with_four_decimals() {
        let frame = format_set_press(6, "A", 950.0);
        // "PRESS" + "A" + "=" + "950.0000" (8 chars) == 15 == 0x0F, matching
        // the proto's literal "00F" length field (not separately validated
        // here -- see the module doc: this port transcribes the literal
        // length text rather than recomputing it).
        assert_eq!(
            frame,
            format!("6R00FPRESSA=950.0000{}", cksum_str("6R00FPRESSA=950.0000"))
        );
    }

    #[test]
    fn format_set_press_pads_short_values() {
        let frame = format_set_press(6, "A", 1.5);
        assert!(frame.starts_with("6R00FPRESSA=001.5000"));
    }

    #[test]
    fn format_get_press_get_flow_get_mflow_match_proto_literals() {
        assert_eq!(
            format_get_press(6, "A"),
            format!("6R006PRESSA{}", cksum_str("6R006PRESSA"))
        );
        assert_eq!(
            format_get_flow(6, "A"),
            format!("6R005FLOWA{}", cksum_str("6R005FLOWA"))
        );
        assert_eq!(
            format_get_mflow(6, "A"),
            format!("6R006MFLOWA{}", cksum_str("6R006MFLOWA"))
        );
    }

    #[test]
    fn format_set_mflow_and_get_status_match_proto_literals() {
        let frame = format_set_mflow(6, "A", 5.0);
        assert_eq!(
            frame,
            format!("6R00FMFLOWA=005.0000{}", cksum_str("6R00FMFLOWA=005.0000"))
        );
        assert_eq!(
            format_get_status(6, "A"),
            format!("6R007STATUSA{}", cksum_str("6R007STATUSA"))
        );
    }

    #[test]
    fn format_send_cmd_prepends_r0_before_caller_supplied_body() {
        let frame = format_send_cmd(6, "08IDENTIFY");
        assert_eq!(
            frame,
            format!("6R008IDENTIFY{}", cksum_str("6R008IDENTIFY"))
        );
    }

    #[test]
    fn parse_ack_accepts_valid_reply() {
        let body = "R ";
        let frame = format!("{body}{}", cksum_str(body));
        assert_eq!(parse_ack(frame.as_bytes()), Ok(()));
    }

    #[test]
    fn parse_ack_rejects_bad_checksum_and_wrong_prefix() {
        assert_eq!(parse_ack(b"R xx"), Err(WireError::ChecksumMismatch));
        let body = "R ";
        // Same length as a valid ack frame (4 bytes) but wrong leading text.
        let frame = format!("XY{}", cksum_str(body));
        assert_eq!(
            parse_ack(frame.as_bytes()),
            Err(WireError::UnexpectedPrefix)
        );
    }

    #[test]
    fn parse_captured_string_extracts_free_text_after_skip() {
        // "R0" + 2 skipped length-field bytes + captured text, no checksum.
        assert_eq!(
            parse_captured_string(b"R0??Teledyne D-Series v1.0"),
            Ok("Teledyne D-Series v1.0".to_string())
        );
    }

    #[test]
    fn parse_get_status_requires_status_equals_literal() {
        assert_eq!(
            parse_get_status(b"R0??STATUS=RUNNING"),
            Ok("RUNNING".to_string())
        );
        assert_eq!(
            parse_get_status(b"R0??WRONG=RUNNING"),
            Err(WireError::UnexpectedPrefix)
        );
    }

    #[test]
    fn parse_get_vol_extracts_float_and_validates_checksum() {
        let body = "R0??VOLA=12.3400 ";
        let frame = format!("{body}{}", cksum_str(body));
        assert_eq!(parse_get_vol(frame.as_bytes()), Ok(12.34));
    }

    #[test]
    fn parse_get_vol_rejects_checksum_mismatch() {
        let body = "R0??VOLA=12.3400 ";
        let mut frame = format!("{body}{}", cksum_str(body));
        // Corrupt the last checksum digit.
        unsafe {
            let bytes = frame.as_bytes_mut();
            let last = bytes.len() - 1;
            bytes[last] = if bytes[last] == b'0' { b'1' } else { b'0' };
        }
        assert_eq!(
            parse_get_vol(frame.as_bytes()),
            Err(WireError::ChecksumMismatch)
        );
    }

    #[test]
    fn parse_get_flow_requires_extra_literal_space_before_float() {
        let body = "R0??FLOWA= 3.5000 ";
        let frame = format!("{body}{}", cksum_str(body));
        assert_eq!(parse_get_flow(frame.as_bytes()), Ok(3.5));
        // getVol's shape (no leading space before the digits) must NOT
        // parse as a valid getFlow reply.
        let vol_shaped = "R0??FLOWA=3.5000 ";
        let frame2 = format!("{vol_shaped}{}", cksum_str(vol_shaped));
        assert!(parse_get_flow(frame2.as_bytes()).is_err());
    }

    #[test]
    fn parse_get_press_and_get_set_press_share_identical_wire_shape() {
        let body = "R0??PRESSA=100.0000 ";
        let frame = format!("{body}{}", cksum_str(body));
        assert_eq!(parse_get_press(frame.as_bytes()), Ok(100.0));
        assert_eq!(parse_get_set_press(frame.as_bytes()), Ok(100.0));
    }

    #[test]
    fn parse_get_mflow_shares_get_flow_shape() {
        let body = "R0??FLOWA= 7.0000 ";
        let frame = format!("{body}{}", cksum_str(body));
        assert_eq!(parse_get_mflow(frame.as_bytes()), Ok(7.0));
    }

    #[test]
    fn parse_rejects_truncated_replies() {
        assert_eq!(parse_ack(b"R"), Err(WireError::TooShort));
        assert_eq!(parse_get_vol(b"R0??VOLA=1"), Err(WireError::TooShort));
    }
}
