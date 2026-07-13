//! Pure MPX protocol codec: command encoding, response decoding, and
//! data-frame header parsing. No I/O — everything here is a total function
//! over byte slices, which is what the unit tests exercise.
//!
//! Ported from `merlinApp/src/mpxConnection.cpp`.

use epics_rs::ad_core::attributes::NDAttrValue;

use crate::types::*;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MpxError {
    /// The encoded command would exceed `MPX_MAXLINE`.
    TooLong,
    /// Frame header was not `MPX,<10 digits>,`.
    BadHeader,
    /// Body length field named a frame we cannot hold.
    BadBodySize(i64),
    /// A response body did not have the fields its type requires.
    Malformed,
    /// The device answered with a non-zero MPX error code.
    Device(i32),
    /// Response echoed a different command type or variable name.
    Unexpected,
}

impl std::fmt::Display for MpxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooLong => write!(f, "MPX message exceeds {MPX_MAXLINE} bytes"),
            Self::BadHeader => write!(f, "malformed MPX frame header"),
            Self::BadBodySize(n) => write!(f, "MPX frame size {n} not supported"),
            Self::Malformed => write!(f, "malformed MPX response body"),
            Self::Device(code) => write!(f, "MPX error code {code} from device"),
            Self::Unexpected => write!(f, "unexpected MPX response"),
        }
    }
}

impl std::error::Error for MpxError {}

/// Encode one command-channel message.
///
/// The wire form is `MPX,<10-digit length>,<KIND>,<name>[,<value>]`, where the
/// length counts every byte after the length field *including* the comma that
/// follows it (C `mpxSet`/`mpxGet`/`mpxCommand`).
pub fn encode(kind: MpxKind, name: &str, value: Option<&str>) -> Result<String, MpxError> {
    let body = match value {
        Some(v) => format!("{},{},{}", kind.as_str(), name, v),
        None => format!("{},{}", kind.as_str(), name),
    };
    let msg = format!("MPX,{:010},{}", body.len() + 1, body);
    if msg.len() > MPX_MAXLINE {
        return Err(MpxError::TooLong);
    }
    Ok(msg)
}

/// Parse the fixed 15-byte frame header, returning the number of body bytes
/// still to be read.
///
/// The length field counts the comma that terminates the header, which has
/// already been consumed, so the body is one byte shorter (C `mpxRead`).
pub fn parse_frame_header(header: &[u8], max_body: usize) -> Result<usize, MpxError> {
    if header.len() != MPX_FRAME_HEADER_LEN || !header.starts_with(MPX_HEADER) {
        return Err(MpxError::BadHeader);
    }
    if header[3] != b',' || header[MPX_FRAME_HEADER_LEN - 1] != b',' {
        return Err(MpxError::BadHeader);
    }
    let digits =
        std::str::from_utf8(&header[4..4 + MPX_MSG_LEN_DIGITS]).map_err(|_| MpxError::BadHeader)?;
    let declared: i64 = digits.trim().parse().map_err(|_| MpxError::BadHeader)?;
    let body = declared - 1;
    if body <= 0 || body as usize >= max_body {
        return Err(MpxError::BadBodySize(body));
    }
    Ok(body as usize)
}

/// Split a response body on commas, as text.
fn fields(body: &[u8]) -> Vec<&str> {
    // A body may carry binary payload after the text header; from_utf8_lossy
    // over the caller-bounded slice keeps this total.
    std::str::from_utf8(body)
        .unwrap_or("")
        .split(',')
        .collect::<Vec<_>>()
}

/// Check that a command response echoes the type and name we sent
/// (C `mpxReadCmd`).
pub fn response_echoes(body: &[u8], kind: MpxKind, name: &str) -> bool {
    let f = fields(body);
    f.len() >= 2 && f[0] == kind.as_str() && f[1] == name
}

/// Decode a `SET` or `CMD` response body: `<KIND>,<name>,<error>`.
pub fn decode_ack(body: &[u8]) -> Result<(), MpxError> {
    let f = fields(body);
    let code: i32 = f
        .get(2)
        .ok_or(MpxError::Malformed)?
        .trim()
        .parse()
        .unwrap_or(MPX_ERR_UNEXPECTED);
    if code != MPX_OK {
        return Err(MpxError::Device(code));
    }
    Ok(())
}

/// Decode a `GET` response body: `GET,<name>,<value>,<error>`.
pub fn decode_get(body: &[u8]) -> Result<String, MpxError> {
    let f = fields(body);
    let value = f.get(2).ok_or(MpxError::Malformed)?;
    let code: i32 = f
        .get(3)
        .ok_or(MpxError::Malformed)?
        .trim()
        .parse()
        .unwrap_or(MPX_ERR_UNEXPECTED);
    if code != MPX_OK {
        return Err(MpxError::Device(code));
    }
    Ok((*value).to_string())
}

/// Classify a data-channel frame body by its leading type field.
pub fn data_header(body: &[u8]) -> DataHeader {
    if body.len() < MPX_MSG_DATATYPE_LEN {
        return DataHeader::Unknown;
    }
    match &body[..MPX_MSG_DATATYPE_LEN] {
        b"HDR" => DataHeader::Acquisition,
        b"MQ1" => DataHeader::QuadData,
        b"PR1" => DataHeader::Profile,
        _ => DataHeader::Unknown,
    }
}

/// Names of the six optional fields between `OPT1` and `END1`.
const OPT_FIELDS: [&str; 6] = [
    "ROI X",
    "ROI Y",
    "ROI Width",
    "ROI Height",
    "Profile Select",
    "DACs Present",
];
const PROFILE_SELECT_POS: usize = 4;
const DACS_PRESENT_POS: usize = 5;

/// Names of the 19 per-chip DAC values.
const DAC_INFO: [&str; 19] = [
    "Preamp",
    "Ikrum",
    "Shaper",
    "Disc",
    "Disc LS",
    "Shaper Test",
    "DAC Disc L",
    "DAC Test",
    "DAC DISC H",
    "Delay",
    "TP Buff In",
    "TP Buff Out",
    "RPZ",
    "GND",
    "TP Ref",
    "FBK",
    "Cas",
    "TP Ref A",
    "TP Ref B",
];

/// Everything the driver needs out of an `MQ1` / `PR1` frame header.
#[derive(Debug, Clone, PartialEq)]
pub struct MqHeader {
    pub frame_number: i32,
    /// Byte offset, from the start of the body, at which the binary payload
    /// begins.
    pub offset: usize,
    pub chip_count: i32,
    pub x_size: usize,
    pub y_size: usize,
    /// Bits per pixel on the wire (8, 16 or 32).
    pub pixel_depth: i32,
    /// Bit mask from the `Profile Select` optional field; 0 when absent.
    pub profile_select: i32,
    /// Attributes to hang off the NDArray, in wire order.
    pub attrs: Vec<(String, NDAttrValue)>,
}

fn parse_i32(tok: &str) -> i32 {
    // C uses atoi(): leading spaces skipped, trailing garbage ignored,
    // unparseable -> 0.
    let t = tok.trim();
    let end = t
        .char_indices()
        .position(|(i, c)| !(c.is_ascii_digit() || (i == 0 && (c == '-' || c == '+'))))
        .unwrap_or(t.len());
    t[..end].parse().unwrap_or(0)
}

fn parse_f64(tok: &str) -> f64 {
    tok.trim().parse().unwrap_or(0.0)
}

/// Parse the comma-delimited header of an `MQ1` (or `PR1`) data frame.
///
/// The binary payload starts at `offset`, so the text region is bounded by it
/// — unlike C, which ran `strtok` over a fixed 2304-byte window and could walk
/// into pixel data.
pub fn parse_mq_header(body: &[u8]) -> Result<MqHeader, MpxError> {
    // First pass: read the offset field from the leading text so the header
    // region can be bounded exactly.
    let probe_len = body.len().min(64);
    let probe = fields(&body[..probe_len]);
    let declared_offset = probe.get(2).map(|t| parse_i32(t)).unwrap_or(0);
    let text_end = if declared_offset > 0 && (declared_offset as usize) <= body.len() {
        declared_offset as usize
    } else {
        body.len()
    };

    let f = fields(&body[..text_end]);
    // type, frame number, offset, chip count, x, y, depth
    if f.len() < 7 {
        return Err(MpxError::Malformed);
    }

    let mut attrs: Vec<(String, NDAttrValue)> = Vec::new();

    let frame_number = parse_i32(f[1]);
    attrs.push(("Frame Number".into(), NDAttrValue::Int32(frame_number)));

    let offset = declared_offset;
    if offset <= 0 || offset as usize > body.len() {
        return Err(MpxError::Malformed);
    }

    let chip_count = parse_i32(f[3]);
    attrs.push(("Chip Count".into(), NDAttrValue::Int8(chip_count as i8)));

    let x_size = parse_i32(f[4]);
    attrs.push(("X Size".into(), NDAttrValue::Int32(x_size)));
    let y_size = parse_i32(f[5]);
    attrs.push(("Y Size".into(), NDAttrValue::Int32(y_size)));
    if x_size <= 0 || y_size <= 0 {
        return Err(MpxError::Malformed);
    }

    // Depth arrives as e.g. "U16" — strip the type letter.
    let depth_tok = f[6].trim_start_matches(|c: char| !c.is_ascii_digit());
    let pixel_depth = parse_i32(depth_tok);
    attrs.push(("Pixel Depth".into(), NDAttrValue::Int32(pixel_depth)));

    if let Some(t) = f.get(7) {
        attrs.push((
            "Sensor Layout".into(),
            NDAttrValue::String((*t).to_string()),
        ));
    }
    if let Some(t) = f.get(8) {
        // Chip select is a hex mask, one bit per chip. C stored it in an
        // NDAttrInt8, which truncates for detectors above 8 chips.
        let mask = u32::from_str_radix(t.trim(), 16).unwrap_or(0);
        attrs.push(("Chip Select".into(), NDAttrValue::UInt16(mask as u16)));
    }
    if let Some(t) = f.get(9) {
        // C passed a NULL value pointer here, so the attribute never carried
        // the timestamp. Publish the device's text timestamp as a string.
        attrs.push(("Time stamp".into(), NDAttrValue::String((*t).to_string())));
    }
    if let Some(t) = f.get(10) {
        attrs.push(("Shutter Time".into(), NDAttrValue::Float64(parse_f64(t))));
    }
    if let Some(t) = f.get(11) {
        attrs.push(("Counter".into(), NDAttrValue::Int8(parse_i32(t) as i8)));
    }
    if let Some(t) = f.get(12) {
        attrs.push(("Colour Mode".into(), NDAttrValue::Int8(parse_i32(t) as i8)));
    }
    if let Some(t) = f.get(13) {
        attrs.push(("Gain Mode".into(), NDAttrValue::Int8(parse_i32(t) as i8)));
    }

    let mut idx = 14;
    for i in 0..8 {
        if let Some(t) = f.get(idx) {
            attrs.push((format!("Threshold {i}"), NDAttrValue::Float64(parse_f64(t))));
            idx += 1;
        }
    }

    // Optional-field section: OPT1,<6 values>,END1
    let mut profile_select = 0;
    let mut dacs_present = 1;
    if f.get(idx).map(|t| t.starts_with("OPT1")).unwrap_or(false) {
        idx += 1;
        let mut count = 0;
        while count < OPT_FIELDS.len() {
            let Some(tok) = f.get(idx) else { break };
            if tok.starts_with("END1") {
                break;
            }
            let value = parse_i32(tok);
            attrs.push((
                OPT_FIELDS[count].to_string(),
                NDAttrValue::Int16(value as i16),
            ));
            match count {
                PROFILE_SELECT_POS => profile_select = value,
                DACS_PRESENT_POS => dacs_present = value,
                _ => {}
            }
            idx += 1;
            count += 1;
        }
        // Step over END1 when present.
        if f.get(idx).map(|t| t.starts_with("END1")).unwrap_or(false) {
            idx += 1;
        }
    }

    // Per-chip DAC blocks.
    if dacs_present != 0 {
        for chip in 0..chip_count.max(0) {
            let Some(fmt) = f.get(idx) else { break };
            attrs.push((
                format!("DAC {chip} Format"),
                NDAttrValue::String((*fmt).to_string()),
            ));
            idx += 1;
            for i in 0..8 {
                let Some(t) = f.get(idx) else { break };
                attrs.push((
                    format!("Chip {chip} Threshold bits {i}"),
                    NDAttrValue::UInt16(parse_i32(t) as u16),
                ));
                idx += 1;
            }
            for name in DAC_INFO {
                let Some(t) = f.get(idx) else { break };
                attrs.push((
                    format!("Chip {chip} {name}"),
                    NDAttrValue::Int16(parse_i32(t) as i16),
                ));
                idx += 1;
            }
        }
    }

    Ok(MqHeader {
        frame_number,
        offset: offset as usize,
        chip_count,
        x_size: x_size as usize,
        y_size: y_size as usize,
        pixel_depth,
        profile_select,
        attrs,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- command codec ---------------------------------------------------

    #[test]
    fn encode_set_matches_c_length_field() {
        // C: buff_len = 3 + 10 + 3 + len(value) + len(id) + 4 commas
        //    msg_len  = buff_len - 10 - 3 - 1
        let msg = encode(MpxKind::Set, "THRESHOLD0", Some("10.000000")).unwrap();
        assert_eq!(msg, "MPX,0000000025,SET,THRESHOLD0,10.000000");
        // The length field counts everything after it, comma included.
        assert_eq!(msg.len() - MPX_FRAME_HEADER_LEN + 1, 25);
    }

    #[test]
    fn encode_get_matches_c_length_field() {
        let msg = encode(MpxKind::Get, "DETECTORSTATUS", None).unwrap();
        assert_eq!(msg, "MPX,0000000019,GET,DETECTORSTATUS");
        assert_eq!(msg.len() - MPX_FRAME_HEADER_LEN + 1, 19);
    }

    #[test]
    fn encode_cmd_matches_c_length_field() {
        let msg = encode(MpxKind::Cmd, "STARTACQUISITION", None).unwrap();
        assert_eq!(msg, "MPX,0000000021,CMD,STARTACQUISITION");
        assert_eq!(msg.len() - MPX_FRAME_HEADER_LEN + 1, 21);
    }

    #[test]
    fn encode_rejects_oversized_message() {
        let long = "X".repeat(MPX_MAXLINE);
        assert_eq!(
            encode(MpxKind::Set, "THRESHOLD0", Some(&long)),
            Err(MpxError::TooLong)
        );
    }

    #[test]
    fn encode_accepts_message_exactly_at_the_limit() {
        // 15 header bytes + "SET,A," (6) => value may be 235 bytes.
        let value = "V".repeat(MPX_MAXLINE - 15 - 6);
        let msg = encode(MpxKind::Set, "A", Some(&value)).unwrap();
        assert_eq!(msg.len(), MPX_MAXLINE);
    }

    // --- frame header ----------------------------------------------------

    #[test]
    fn parse_frame_header_returns_body_minus_leading_comma() {
        assert_eq!(parse_frame_header(b"MPX,0000000026,", 256).unwrap(), 25);
    }

    #[test]
    fn parse_frame_header_rejects_bad_magic() {
        assert_eq!(
            parse_frame_header(b"XPM,0000000026,", 256),
            Err(MpxError::BadHeader)
        );
    }

    #[test]
    fn parse_frame_header_rejects_short_header() {
        assert_eq!(
            parse_frame_header(b"MPX,000026,", 256),
            Err(MpxError::BadHeader)
        );
    }

    #[test]
    fn parse_frame_header_rejects_body_larger_than_buffer() {
        assert_eq!(
            parse_frame_header(b"MPX,0000001000,", 256),
            Err(MpxError::BadBodySize(999))
        );
    }

    #[test]
    fn parse_frame_header_rejects_empty_body() {
        assert_eq!(
            parse_frame_header(b"MPX,0000000001,", 256),
            Err(MpxError::BadBodySize(0))
        );
    }

    // --- response decoding ------------------------------------------------

    #[test]
    fn decode_get_extracts_value() {
        assert_eq!(decode_get(b"GET,SOFTWAREVERSION,2.2,0").unwrap(), "2.2");
    }

    #[test]
    fn decode_get_reports_device_error() {
        assert_eq!(decode_get(b"GET,THSTART,0,3"), Err(MpxError::Device(3)));
    }

    #[test]
    fn decode_get_rejects_missing_error_field() {
        assert_eq!(decode_get(b"GET,THSTART,0"), Err(MpxError::Malformed));
    }

    #[test]
    fn decode_ack_accepts_zero_error() {
        assert!(decode_ack(b"SET,COUNTERDEPTH,0").is_ok());
        assert!(decode_ack(b"CMD,STARTACQUISITION,0").is_ok());
    }

    #[test]
    fn decode_ack_reports_device_error() {
        assert_eq!(decode_ack(b"CMD,THSCAN,4"), Err(MpxError::Device(4)));
    }

    #[test]
    fn decode_ack_rejects_missing_error_field() {
        assert_eq!(decode_ack(b"SET,COUNTERDEPTH"), Err(MpxError::Malformed));
    }

    #[test]
    fn response_echo_check_matches_type_and_name() {
        assert!(response_echoes(
            b"GET,THSTART,2.0,0",
            MpxKind::Get,
            "THSTART"
        ));
        assert!(!response_echoes(
            b"GET,THSTOP,2.0,0",
            MpxKind::Get,
            "THSTART"
        ));
        assert!(!response_echoes(b"SET,THSTART,0", MpxKind::Get, "THSTART"));
    }

    // --- data frame headers ----------------------------------------------

    #[test]
    fn data_header_classifies_frame_types() {
        assert_eq!(data_header(b"HDR,anything"), DataHeader::Acquisition);
        assert_eq!(data_header(b"MQ1,1,768,4"), DataHeader::QuadData);
        assert_eq!(data_header(b"PR1,1,768,4"), DataHeader::Profile);
        assert_eq!(data_header(b"12B,1,2"), DataHeader::Unknown);
        assert_eq!(data_header(b"MQ"), DataHeader::Unknown);
    }

    /// A Merlin Quad MQ1 header: 1 chip, 256x256, U16, with the optional
    /// field block and one DAC block. Padded to `offset` bytes so the binary
    /// payload starts exactly where the header says it does.
    fn quad_header(offset: usize, dacs_present: i32) -> Vec<u8> {
        let mut text = String::new();
        text.push_str("MQ1,");
        text.push_str("000001,"); // frame number
        text.push_str(&format!("{offset:06},")); // offset
        text.push_str("1,"); // chip count
        text.push_str("256,256,"); // x, y
        text.push_str("U16,"); // pixel depth
        text.push_str("R64,"); // sensor layout
        text.push_str("1,"); // chip select (hex)
        text.push_str("2024-01-02 03:04:05.678,"); // time stamp
        text.push_str("1.5E-2,"); // shutter time
        text.push_str("0,"); // counter
        text.push_str("0,"); // colour mode
        text.push_str("1,"); // gain mode
        for i in 0..8 {
            text.push_str(&format!("{}.5,", i + 1)); // thresholds 0..7
        }
        text.push_str("OPT1,");
        text.push_str(&format!("10,20,30,40,14,{dacs_present},"));
        text.push_str("END1,");
        text.push_str("3RX,"); // DAC format
        for i in 0..8 {
            text.push_str(&format!("{},", 100 + i)); // threshold bits
        }
        for i in 0..19 {
            text.push_str(&format!("{},", 200 + i)); // DAC values
        }
        let mut body = text.into_bytes();
        assert!(body.len() <= offset, "test header longer than offset");
        body.resize(offset, b' ');
        body
    }

    #[test]
    fn parse_mq_header_extracts_geometry() {
        let body = quad_header(768, 1);
        let h = parse_mq_header(&body).unwrap();
        assert_eq!(h.frame_number, 1);
        assert_eq!(h.offset, 768);
        assert_eq!(h.chip_count, 1);
        assert_eq!(h.x_size, 256);
        assert_eq!(h.y_size, 256);
        assert_eq!(h.pixel_depth, 16);
    }

    #[test]
    fn parse_mq_header_reads_optional_fields() {
        // C compared each token against END1 instead of stopping at it, so the
        // optional fields were never parsed and Profile Select stayed 0.
        let body = quad_header(768, 1);
        let h = parse_mq_header(&body).unwrap();
        assert_eq!(h.profile_select, 14);
        let get = |n: &str| h.attrs.iter().find(|(k, _)| k == n).map(|(_, v)| v.clone());
        assert_eq!(get("ROI X"), Some(NDAttrValue::Int16(10)));
        assert_eq!(get("ROI Y"), Some(NDAttrValue::Int16(20)));
        assert_eq!(get("ROI Width"), Some(NDAttrValue::Int16(30)));
        assert_eq!(get("ROI Height"), Some(NDAttrValue::Int16(40)));
        assert_eq!(get("Profile Select"), Some(NDAttrValue::Int16(14)));
        assert_eq!(get("DACs Present"), Some(NDAttrValue::Int16(1)));
    }

    #[test]
    fn parse_mq_header_reads_dac_block_after_optional_fields() {
        let body = quad_header(768, 1);
        let h = parse_mq_header(&body).unwrap();
        let get = |n: &str| h.attrs.iter().find(|(k, _)| k == n).map(|(_, v)| v.clone());
        assert_eq!(get("DAC 0 Format"), Some(NDAttrValue::String("3RX".into())));
        assert_eq!(
            get("Chip 0 Threshold bits 0"),
            Some(NDAttrValue::UInt16(100))
        );
        assert_eq!(
            get("Chip 0 Threshold bits 7"),
            Some(NDAttrValue::UInt16(107))
        );
        assert_eq!(get("Chip 0 Preamp"), Some(NDAttrValue::Int16(200)));
        assert_eq!(get("Chip 0 TP Ref B"), Some(NDAttrValue::Int16(218)));
    }

    #[test]
    fn parse_mq_header_skips_dac_block_when_dacs_absent() {
        let body = quad_header(768, 0);
        let h = parse_mq_header(&body).unwrap();
        assert!(h.attrs.iter().all(|(k, _)| !k.starts_with("Chip 0 ")));
        assert!(h.attrs.iter().all(|(k, _)| !k.starts_with("DAC ")));
    }

    #[test]
    fn parse_mq_header_publishes_scalar_attributes() {
        let body = quad_header(768, 1);
        let h = parse_mq_header(&body).unwrap();
        let get = |n: &str| h.attrs.iter().find(|(k, _)| k == n).map(|(_, v)| v.clone());
        assert_eq!(get("Frame Number"), Some(NDAttrValue::Int32(1)));
        assert_eq!(get("Chip Count"), Some(NDAttrValue::Int8(1)));
        assert_eq!(get("X Size"), Some(NDAttrValue::Int32(256)));
        assert_eq!(get("Y Size"), Some(NDAttrValue::Int32(256)));
        assert_eq!(get("Pixel Depth"), Some(NDAttrValue::Int32(16)));
        assert_eq!(
            get("Sensor Layout"),
            Some(NDAttrValue::String("R64".into()))
        );
        assert_eq!(get("Chip Select"), Some(NDAttrValue::UInt16(1)));
        assert_eq!(
            get("Time stamp"),
            Some(NDAttrValue::String("2024-01-02 03:04:05.678".into()))
        );
        assert_eq!(get("Shutter Time"), Some(NDAttrValue::Float64(0.015)));
        assert_eq!(get("Threshold 0"), Some(NDAttrValue::Float64(1.5)));
        assert_eq!(get("Threshold 7"), Some(NDAttrValue::Float64(8.5)));
    }

    #[test]
    fn parse_mq_header_stops_at_the_binary_payload() {
        // Bytes past `offset` are pixel data and must never be tokenized,
        // even when they happen to contain commas.
        let mut body = quad_header(768, 1);
        body.extend_from_slice(&[b',', 0xff, b',', 0x00]);
        let h = parse_mq_header(&body).unwrap();
        assert_eq!(h.offset, 768);
        assert_eq!(h.chip_count, 1);
        // The trailing junk must not have appended DAC attributes.
        assert!(h.attrs.iter().all(|(k, _)| k != "DAC 1 Format"));
    }

    #[test]
    fn parse_mq_header_rejects_truncated_header() {
        assert_eq!(parse_mq_header(b"MQ1,1,768"), Err(MpxError::Malformed));
    }

    #[test]
    fn parse_mq_header_rejects_offset_past_the_frame() {
        // offset larger than the frame would make the payload slice invalid.
        let body = b"MQ1,1,99999,1,256,256,U16,R64";
        assert_eq!(parse_mq_header(body), Err(MpxError::Malformed));
    }

    #[test]
    fn parse_mq_header_rejects_zero_geometry() {
        let body = b"MQ1,1,20,1,0,256,U16,R64,1,x";
        assert_eq!(parse_mq_header(body), Err(MpxError::Malformed));
    }
}
