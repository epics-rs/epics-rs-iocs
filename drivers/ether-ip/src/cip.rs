//! CIP ("Common Industrial Protocol") encoding/decoding for Logix5000 data
//! access, ported from `ether_ipApp/src/ether_ip.c`.
//!
//! ControlNet/CIP is little-endian on the wire, independent of host byte order.
//! Everything in this module is a pure function over byte slices so it can be
//! unit-tested against fixtures captured from the C encoders.
//!
//! Docs referenced by the C, and by these comments:
//!   "Spec" = ControlNet Spec. version 2.0, Errata 1
//!   "ENET" = AB Publication 1756-RM005A-EN-E

use std::fmt;

/// A tag value is transported as an abbreviated type code followed by raw data.
/// This is the size of that leading type code.
pub const TYPECODE_SIZE: usize = 2;

/// Longest tag string we accept; the AB PLC limit is 82 chars.
pub const MAX_TAG_LENGTH: usize = 100;

// ---------------------------------------------------------------------------
// CIP data types (ENET p. 11)
// ---------------------------------------------------------------------------

/// Abbreviated CIP type codes. `Struct` is the two-byte 0x02A0 code, which is
/// followed on the wire by a further "struct handle" u16.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CipType {
    Bool,
    Sint,
    Int,
    Uint,
    Dint,
    Lint,
    Ulint,
    Real,
    Lreal,
    /// Plain CIP STRING (0x00D0), as opposed to the Logix STRING struct.
    String,
    /// 32-bit bit-array ("BITS"): how Logix packs `BOOL[]`.
    Bits,
    Struct,
    Unknown(u16),
}

impl CipType {
    pub fn from_code(code: u16) -> Self {
        match code {
            0x00C1 => CipType::Bool,
            0x00C2 => CipType::Sint,
            0x00C3 => CipType::Int,
            0x00C7 => CipType::Uint,
            0x00C4 => CipType::Dint,
            0x00C5 => CipType::Lint,
            0x00C9 => CipType::Ulint,
            0x00CA => CipType::Real,
            0x00CB => CipType::Lreal,
            0x00D0 => CipType::String,
            0x00D3 => CipType::Bits,
            0x02A0 => CipType::Struct,
            other => CipType::Unknown(other),
        }
    }

    pub fn code(self) -> u16 {
        match self {
            CipType::Bool => 0x00C1,
            CipType::Sint => 0x00C2,
            CipType::Int => 0x00C3,
            CipType::Uint => 0x00C7,
            CipType::Dint => 0x00C4,
            CipType::Lint => 0x00C5,
            CipType::Ulint => 0x00C9,
            CipType::Real => 0x00CA,
            CipType::Lreal => 0x00CB,
            CipType::String => 0x00D0,
            CipType::Bits => 0x00D3,
            CipType::Struct => 0x02A0,
            CipType::Unknown(c) => c,
        }
    }

    /// Byte size of one element. Structs and strings return 0: they are not
    /// fixed-size and are handled by the string accessors.
    pub fn size(self) -> usize {
        match self {
            CipType::Bool | CipType::Sint => 1,
            CipType::Int | CipType::Uint => 2,
            CipType::Dint | CipType::Real | CipType::Bits => 4,
            CipType::Lint | CipType::Ulint | CipType::Lreal => 8,
            CipType::String | CipType::Struct | CipType::Unknown(_) => 0,
        }
    }
}

/// The Logix STRING struct: `A0 02 CE 0F <len u32> <82 chars> <pad>`.
pub const STRUCT_STRING_HANDLE: u16 = 0x0FCE;
/// Bytes holding the text length (only the first 2 are actually used).
pub const STRUCT_STRING_LEN_BYTES: usize = 4;
/// Maximum number of useful characters in a Logix STRING.
pub const STRUCT_STRING_MAX: usize = 82;
/// Overall size of the character buffer inside the struct.
pub const STRUCT_STRING_BUF: usize = 84;
/// Stride from one STRING element to the next.
pub const STRUCT_STRING_STRIDE: usize = STRUCT_STRING_LEN_BYTES + STRUCT_STRING_BUF;

// ---------------------------------------------------------------------------
// Little-endian primitives
// ---------------------------------------------------------------------------

fn rd_u16(b: &[u8], at: usize) -> Option<u16> {
    let s = b.get(at..at + 2)?;
    Some(u16::from_le_bytes([s[0], s[1]]))
}

fn rd_u32(b: &[u8], at: usize) -> Option<u32> {
    let s = b.get(at..at + 4)?;
    Some(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}

fn rd_u64(b: &[u8], at: usize) -> Option<u64> {
    let s = b.get(at..at + 8)?;
    Some(u64::from_le_bytes([
        s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7],
    ]))
}

fn wr_slice(b: &mut [u8], at: usize, src: &[u8]) -> bool {
    match b.get_mut(at..at + src.len()) {
        Some(dst) => {
            dst.copy_from_slice(src);
            true
        }
        None => false,
    }
}

// ---------------------------------------------------------------------------
// Tag parsing (name.name[element].name)
// ---------------------------------------------------------------------------

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum TagElement {
    Name(String),
    Index(u32),
}

/// A tag string compiled into CIP path elements.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct ParsedTag {
    pub elements: Vec<TagElement>,
}

impl ParsedTag {
    /// Parse `name.name[element].name` into path elements.
    ///
    /// Returns `None` for input that the C parser also rejects: an empty
    /// leading name, or an unterminated `[`.
    pub fn parse(tag: &str) -> Option<ParsedTag> {
        let b = tag.as_bytes();
        let mut out = Vec::new();
        let mut i = 0usize;

        while i < b.len() {
            // Name runs up to the next '.' or '['.
            let start = i;
            while i < b.len() && b[i] != b'.' && b[i] != b'[' {
                i += 1;
            }
            if i == start {
                // Zero-length name: the C's `strcspn` gives len 0 and it breaks
                // out of the loop, yielding whatever it had so far.
                break;
            }
            out.push(TagElement::Name(tag[start..i].to_string()));

            if i >= b.len() {
                break;
            }
            match b[i] {
                b'.' => i += 1,
                b'[' => {
                    let num_start = i + 1;
                    let close = tag[num_start..].find(']')? + num_start;
                    // The C uses atol(), which stops at the first non-digit and
                    // yields 0 for garbage.
                    let digits: String = tag[num_start..close]
                        .chars()
                        .take_while(|c| c.is_ascii_digit())
                        .collect();
                    let idx: u32 = digits.parse().unwrap_or(0);
                    out.push(TagElement::Index(idx));
                    i = close + 1;
                    // Handle the '.' in "xxxx[3].subelement".
                    if i < b.len() && b[i] == b'.' {
                        i += 1;
                    }
                }
                _ => unreachable!(),
            }
        }

        if out.is_empty() {
            None
        } else {
            Some(ParsedTag { elements: out })
        }
    }

    /// Word-size of the encoded path (CIP path sizes are counted in 16-bit words).
    pub fn path_words(&self) -> usize {
        let mut bytes = 0usize;
        for e in &self.elements {
            bytes += match e {
                // 0x91, len, string, optional pad byte
                TagElement::Name(n) => 2 + n.len() + n.len() % 2,
                TagElement::Index(i) if *i <= 0xFF => 2,
                TagElement::Index(i) if *i <= 0xFFFF => 4,
                TagElement::Index(_) => 6,
            };
        }
        bytes / 2
    }

    /// Append the encoded path. Spec 4 p.21: 0x91 is the "ANSI extended symbol
    /// segment".
    pub fn encode_path(&self, out: &mut Vec<u8>) {
        for e in &self.elements {
            match e {
                TagElement::Name(n) => {
                    out.push(0x91);
                    out.push(n.len() as u8);
                    out.extend_from_slice(n.as_bytes());
                    if n.len() % 2 != 0 {
                        out.push(0); // pad to 16-bit boundary
                    }
                }
                TagElement::Index(i) if *i <= 0xFF => {
                    out.push(0x28);
                    out.push(*i as u8);
                }
                TagElement::Index(i) if *i <= 0xFFFF => {
                    out.push(0x29);
                    out.push(0x00);
                    out.extend_from_slice(&(*i as u16).to_le_bytes());
                }
                TagElement::Index(i) => {
                    out.push(0x2A);
                    out.push(0x00);
                    out.extend_from_slice(&i.to_le_bytes());
                }
            }
        }
    }
}

impl fmt::Display for ParsedTag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut first = true;
        for e in &self.elements {
            match e {
                TagElement::Name(n) => {
                    if !first {
                        write!(f, ".")?;
                    }
                    write!(f, "{n}")?;
                }
                TagElement::Index(i) => write!(f, "[{i}]")?,
            }
            first = false;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Message Router paths
// ---------------------------------------------------------------------------

/// CIP object classes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum CipClass {
    Identity = 0x01,
    MessageRouter = 0x02,
    ConnectionManager = 0x06,
    Symbol = 0x6B,
    Template = 0x6C,
}

/// Word-size of a Class/Instance/Attribute path. `attr == 0` means "no attribute".
pub fn cia_path_words(instance: u32, attr: u8) -> usize {
    let mut words = 1; // class
    words += if instance > 0xFF { 2 } else { 1 };
    if attr != 0 {
        words += 1;
    }
    words
}

/// Encode a Class/Instance/Attribute path (Spec 4, p. 17).
pub fn encode_cia_path(out: &mut Vec<u8>, class: CipClass, instance: u32, attr: u8) {
    out.push(0x20);
    out.push(class as u8);
    if instance > 0xFF {
        out.push(0x25);
        out.push(0x00);
        out.extend_from_slice(&(instance as u16).to_le_bytes());
    } else {
        out.push(0x24);
        out.push(instance as u8);
    }
    if attr != 0 {
        out.push(0x30);
        out.push(attr);
    }
}

// ---------------------------------------------------------------------------
// Message Router request/response
// ---------------------------------------------------------------------------

/// CIP service codes (Spec 4, p.36).
pub mod service {
    pub const GET_ATTRIBUTE_ALL: u8 = 0x01;
    pub const GET_ATTRIBUTE_LIST: u8 = 0x03;
    pub const GET_ATTRIBUTE_SINGLE: u8 = 0x0E;
    pub const CIP_MULTI_REQUEST: u8 = 0x0A;
    pub const CIP_READ_DATA: u8 = 0x4C;
    pub const CIP_WRITE_DATA: u8 = 0x4D;
    pub const CM_UNCONNECTED_SEND: u8 = 0x52;
    pub const GET_INSTANCE_ATTR_LIST: u8 = 0x55;
    /// Responses echo the request service with the high bit set.
    pub const REPLY_FLAG: u8 = 0x80;
}

/// Byte size of an MR_Request header for a path of `path_words` words.
pub fn mr_request_size(path_words: usize) -> usize {
    2 + path_words * 2
}

/// Human-readable CIP general status (Spec 4, p.46 and 1756-RM005A-EN-E).
pub fn cip_error_text(status: u8) -> &'static str {
    match status {
        0x00 => "Ok",
        0x01 => "Extended error",
        0x04 => "Unknown tag or Path error",
        0x05 => "Instance not found",
        0x06 => "Buffer too small, partial data only",
        0x08 => "Service not supported",
        0x09 => "Invalid Attribute",
        0x13 => "Not enough data",
        0x14 => "Attribute not supported, ext. shows attribute",
        0x15 => "Too much data",
        0x1E => "One of the MultiRequests failed",
        _ => "<unknown>",
    }
}

/// A raw MR_Response: `service, reserved, general_status, ext_status_words`,
/// then the (optional) extended status words, then the (optional) data.
#[derive(Clone, Copy, Debug)]
pub struct MrResponse<'a> {
    pub service: u8,
    pub general_status: u8,
    pub raw: &'a [u8],
}

impl<'a> MrResponse<'a> {
    pub fn parse(raw: &'a [u8]) -> Option<MrResponse<'a>> {
        if raw.len() < 4 {
            return None;
        }
        Some(MrResponse {
            service: raw[0],
            general_status: raw[2],
            raw,
        })
    }

    /// Offset of the data section: after the 4-byte header plus the extended
    /// status words.
    pub fn data_offset(&self) -> usize {
        4 + 2 * self.raw[3] as usize
    }

    /// The data section, given the true length of this response.
    pub fn data(&self, response_size: usize) -> &'a [u8] {
        let off = self.data_offset();
        let end = response_size.min(self.raw.len());
        if off >= end { &[] } else { &self.raw[off..end] }
    }

    pub fn is_ok(&self) -> bool {
        self.general_status == 0
    }
}

// ---------------------------------------------------------------------------
// CM_Unconnected_Send (Spec 4, p 41)
// ---------------------------------------------------------------------------

/// Split a millisecond timeout into a time-per-tick and a tick count
/// (Spec 4 p. 30,31).
pub fn calc_tick_time(millisec: u32) -> Option<(u8, u8)> {
    if millisec > 8_355_840 {
        return None;
    }
    let mut ms = millisec;
    let mut tick_time: u8 = 0;
    while ms > 0xFF {
        tick_time += 1;
        ms >>= 1;
    }
    Some((tick_time, ms as u8))
}

/// Total byte size of a CM_Unconnected_Send carrying `message_size` bytes.
pub fn unconnected_send_size(message_size: usize) -> usize {
    mr_request_size(cia_path_words(1, 0))
        + 1  // priority_and_tick
        + 1  // connection_timeout_ticks
        + 2  // message_size
        + message_size + message_size % 2 // padded to a 16-bit boundary
        + 4 // path to the PLC
}

/// Wrap `message` (a complete, different MR_Request) in a CM_Unconnected_Send
/// routed over the backplane to `slot`.
///
/// The C's `make_CM_Unconnected_Send` uses a fixed 245760 ms tick time taken
/// from an example; we keep that on the wire.
pub fn encode_unconnected_send(out: &mut Vec<u8>, message: &[u8], slot: u8) {
    let (tick_time, ticks) = calc_tick_time(245_760).expect("245760 ms is within range");

    out.push(service::CM_UNCONNECTED_SEND);
    out.push(cia_path_words(1, 0) as u8);
    encode_cia_path(out, CipClass::ConnectionManager, 1, 0);

    out.push(tick_time);
    out.push(ticks);
    out.extend_from_slice(&(message.len() as u16).to_le_bytes());
    out.extend_from_slice(message);
    if !message.len().is_multiple_of(2) {
        out.push(0); // pad
    }
    // Route: port 1 = backplane, link = slot.
    out.push(1); // path_size, in words
    out.push(0); // reserved
    out.push(1); // port 1 = backplane
    out.push(slot);
}

// ---------------------------------------------------------------------------
// CIP Read/Write Data
// ---------------------------------------------------------------------------

/// Byte size of a CIP_ReadData request for `tag`.
pub fn read_data_size(tag: &ParsedTag) -> usize {
    2 + 2 * tag.path_words() + 2
}

/// Encode a CIP_ReadData request: service, path, element count.
pub fn encode_read_data(out: &mut Vec<u8>, tag: &ParsedTag, elements: u16) {
    out.push(service::CIP_READ_DATA);
    out.push(tag.path_words() as u8);
    tag.encode_path(out);
    out.extend_from_slice(&elements.to_le_bytes());
}

/// Byte size of a CIP_WriteData request carrying `data_size` bytes of payload.
pub fn write_data_size(tag: &ParsedTag, data_size: usize) -> usize {
    2 + 2 * tag.path_words() + 4 + data_size
}

/// Encode a CIP_WriteData request for an atomic type.
///
/// `raw_data` must already be in wire (little-endian) format and must NOT carry
/// the leading type code -- that matches the C, whose callers pass
/// `info->data + CIP_Typecode_size`.
pub fn encode_write_data(
    out: &mut Vec<u8>,
    tag: &ParsedTag,
    cip_type: CipType,
    elements: u16,
    raw_data: &[u8],
) {
    out.push(service::CIP_WRITE_DATA);
    out.push(tag.path_words() as u8);
    tag.encode_path(out);
    out.extend_from_slice(&cip_type.code().to_le_bytes());
    out.extend_from_slice(&elements.to_le_bytes());
    let want = cip_type.size() * elements as usize;
    let n = want.min(raw_data.len());
    out.extend_from_slice(&raw_data[..n]);
    // The C memcpy's `CIP_Type_size(type) * elements` bytes regardless of what
    // the caller's buffer holds; if the caller under-supplies we zero-fill
    // rather than read past the end.
    for _ in n..want {
        out.push(0);
    }
}

/// Encode a CIP_WriteData request for a Logix STRING struct.
///
/// On the wire this is: struct type, struct handle, element count, then the
/// struct body (length u16, pad u16, characters, NUL).
///
/// `raw_data` is the tag buffer WITHOUT the leading 2-byte type code, i.e.
/// `[handle u16][len u16][pad u16][chars..]`, matching the C's
/// `info->data + CIP_Typecode_size`.
pub fn encode_write_string(
    out: &mut Vec<u8>,
    tag: &ParsedTag,
    elements: u16,
    raw_data: &[u8],
    buf_limit: usize,
) {
    // Text begins after handle(2) + len(2) + pad(2) inside raw_data.
    const TEXT_OFF: usize = 6;
    let text: &[u8] = raw_data.get(TEXT_OFF..).unwrap_or(&[]);
    let mut len = text.iter().position(|&c| c == 0).unwrap_or(text.len());

    let head = 2 + 2 * tag.path_words() + 2 + 2 + 2 + 2 + 2;
    if head + len + 1 > buf_limit {
        len = buf_limit.saturating_sub(head + 1);
        log::warn!("EIP: string too long for request package, truncated to {len} bytes");
    }

    out.push(service::CIP_WRITE_DATA);
    out.push(tag.path_words() as u8);
    tag.encode_path(out);
    out.extend_from_slice(&CipType::Struct.code().to_le_bytes());
    out.extend_from_slice(&STRUCT_STRING_HANDLE.to_le_bytes());
    out.extend_from_slice(&elements.to_le_bytes());
    out.extend_from_slice(&(len as u16).to_le_bytes()); // string length
    out.extend_from_slice(&0u16.to_le_bytes()); // reserved word
    out.extend_from_slice(&text[..len]);
    out.push(0); // the C appends a NUL and counts it in data_size
}

/// Check a CIP_ReadData response and return its data section.
pub fn check_read_data_response(response: &[u8], response_size: usize) -> Option<&[u8]> {
    let r = MrResponse::parse(response)?;
    if r.service & 0x7F != service::CIP_READ_DATA || !r.is_ok() {
        return None;
    }
    Some(r.data(response_size))
}

/// Check a CIP_WriteData response.
pub fn check_write_data_response(response: &[u8]) -> bool {
    match MrResponse::parse(response) {
        Some(r) => r.service & 0x7F == service::CIP_WRITE_DATA && r.is_ok(),
        None => false,
    }
}

// ---------------------------------------------------------------------------
// CIP MultiRequest
// ---------------------------------------------------------------------------

/// Byte size of a MultiRequest wrapping `count` requests totalling
/// `requests_size` bytes.
pub fn multi_request_size(count: usize, requests_size: usize) -> usize {
    2                                    // service, path_size
        + cia_path_words(1, 0) * 2       // MessageRouter path
        + 2 + 2 * count                  // count + offset fields
        + requests_size
}

/// Estimated byte size of the matching MultiRequest response.
pub fn multi_response_size(count: usize, responses_size: usize) -> usize {
    4                       // service, reserved, status, ext_size
        + 2 + 2 * count     // count + offset fields
        + responses_size
}

/// Assemble a CIP MultiRequest from already-encoded sub-requests.
///
/// Layout: MR_Request header, `count`, then a `count`-long table of byte
/// offsets measured from the `count` field itself, then the sub-requests.
pub fn encode_multi_request(out: &mut Vec<u8>, requests: &[Vec<u8>]) {
    let count = requests.len();
    out.push(service::CIP_MULTI_REQUEST);
    out.push(cia_path_words(1, 0) as u8);
    encode_cia_path(out, CipClass::MessageRouter, 1, 0);

    // Offsets are relative to this `count` field.
    out.extend_from_slice(&(count as u16).to_le_bytes());
    let mut offset = (count + 1) * 2;
    for r in requests {
        out.extend_from_slice(&(offset as u16).to_le_bytes());
        offset += r.len();
    }
    for r in requests {
        out.extend_from_slice(r);
    }
}

/// Is this a valid, successful response to a MultiRequest?
pub fn check_multi_request_response(response: &[u8]) -> bool {
    match MrResponse::parse(response) {
        Some(r) => {
            r.service == (service::CIP_MULTI_REQUEST | service::REPLY_FLAG) && r.general_status == 0
        }
        None => false,
    }
}

/// Pick sub-reply `reply_no` out of a MultiRequest response.
pub fn get_multi_request_response(
    response: &[u8],
    response_size: usize,
    reply_no: usize,
) -> Option<&[u8]> {
    let r = MrResponse::parse(response)?;
    let base = r.data_offset(); // the `count` field
    let count = rd_u16(response, base)? as usize;
    if reply_no >= count {
        return None;
    }
    let offsets = base + 2;
    let offset = rd_u16(response, offsets + 2 * reply_no)? as usize;
    let start = base.checked_add(offset)?;

    let end = if reply_no + 1 < count {
        let next = rd_u16(response, offsets + 2 * (reply_no + 1))? as usize;
        base.checked_add(next)?
    } else {
        response_size
    };
    if start > end || end > response.len() {
        return None;
    }
    Some(&response[start..end])
}

// ---------------------------------------------------------------------------
// Value accessors over raw [type][data]
// ---------------------------------------------------------------------------

/// Type code at the head of a raw tag data buffer.
pub fn typecode(raw: &[u8]) -> Option<CipType> {
    rd_u16(raw, 0).map(CipType::from_code)
}

/// Byte offset of `element` within a raw `[type][data]` buffer.
fn elem_off(t: CipType, element: usize) -> usize {
    TYPECODE_SIZE + element * t.size()
}

/// Read `element` as a float.
///
/// UPSTREAM FIX: the C's `get_CIP_double` decodes SINT/INT/DINT through the
/// *unsigned* CN_USINT/CN_UINT/CN_UDINT unpackers, so a negative DINT reads as
/// ~4.29e9 and a negative SINT as 254. CIP SINT/INT/DINT are signed
/// (`ether_ip.h` declares CN_SINT/CN_INT/CN_DINT as signed), so we sign-extend.
/// BOOL and BITS stay unsigned: they are truth values and bit patterns, not
/// numbers.
pub fn get_double(raw: &[u8], element: usize) -> Option<f64> {
    let t = typecode(raw)?;
    let at = elem_off(t, element);
    match t {
        CipType::Bool => raw.get(at).map(|&v| v as f64),
        CipType::Sint => raw.get(at).map(|&v| v as i8 as f64),
        CipType::Int => rd_u16(raw, at).map(|v| v as i16 as f64),
        CipType::Uint => rd_u16(raw, at).map(|v| v as f64),
        CipType::Dint => rd_u32(raw, at).map(|v| v as i32 as f64),
        CipType::Bits => rd_u32(raw, at).map(|v| v as f64),
        CipType::Lint => rd_u64(raw, at).map(|v| v as i64 as f64),
        CipType::Ulint => rd_u64(raw, at).map(|v| v as f64),
        CipType::Real => rd_u32(raw, at).map(|v| f32::from_bits(v) as f64),
        CipType::Lreal => rd_u64(raw, at).map(f64::from_bits),
        _ => None,
    }
}

/// Read `element` as a signed 32-bit integer.
///
/// UPSTREAM FIX: the C's `get_CIP_DINT` reads SINT through an unsigned char, so
/// a SINT of -2 arrives at an `ai`/`longin` record as 254. SINT is signed; we
/// sign-extend it. BOOL keeps its unsigned 0/non-zero meaning.
pub fn get_dint(raw: &[u8], element: usize) -> Option<i32> {
    let t = typecode(raw)?;
    let at = elem_off(t, element);
    match t {
        CipType::Bool => raw.get(at).map(|&v| v as i32),
        CipType::Sint => raw.get(at).map(|&v| v as i8 as i32),
        CipType::Int => rd_u16(raw, at).map(|v| v as i16 as i32),
        CipType::Uint => rd_u16(raw, at).map(|v| v as i32),
        CipType::Dint | CipType::Bits => rd_u32(raw, at).map(|v| v as i32),
        CipType::Lint | CipType::Ulint => rd_u64(raw, at).map(|v| v as i64 as i32),
        CipType::Real => rd_u32(raw, at).map(|v| f32::from_bits(v) as i32),
        CipType::Lreal => rd_u64(raw, at).map(|v| f64::from_bits(v) as i32),
        _ => None,
    }
}

/// Read `element` as a raw 32-bit *bit pattern*.
///
/// This is the accessor the (multi-)bit records use, so it deliberately keeps
/// the C's unsigned semantics: it is a bit pattern, not a number.
pub fn get_udint(raw: &[u8], element: usize) -> Option<u32> {
    let t = typecode(raw)?;
    let at = elem_off(t, element);
    match t {
        CipType::Bool | CipType::Sint => raw.get(at).map(|&v| v as u32),
        CipType::Int | CipType::Uint => rd_u16(raw, at).map(|v| v as u32),
        CipType::Dint | CipType::Bits => rd_u32(raw, at),
        CipType::Real => rd_u32(raw, at).map(|v| f32::from_bits(v) as u32),
        CipType::Lreal => rd_u64(raw, at).map(|v| f64::from_bits(v) as u32),
        _ => None,
    }
}

/// Read `element` as a signed 64-bit integer.
pub fn get_lint(raw: &[u8], element: usize) -> Option<i64> {
    let t = typecode(raw)?;
    let at = elem_off(t, element);
    match t {
        CipType::Lint | CipType::Ulint => rd_u64(raw, at).map(|v| v as i64),
        _ => None,
    }
}

/// Read `element` as a raw byte (used by waveform FTVL=CHAR over SINT).
pub fn get_usint(raw: &[u8], element: usize) -> Option<u8> {
    let t = typecode(raw)?;
    match t {
        CipType::Bool | CipType::Sint => raw.get(elem_off(t, element)).copied(),
        _ => None,
    }
}

/// Read `element` as a string, at most `max` bytes including the terminator.
///
/// UPSTREAM FIX: the C's `T_CIP_STRING` branch does
/// `memcpy(buffer, buf, size); *(buffer+size) = '\0';` -- it copies `size`
/// bytes and then writes a terminator at `buffer[size]`, one byte past the
/// caller's buffer (`stringin` passes MAX_STRING_SIZE with a MAX_STRING_SIZE
/// array), and it ignores the actual data length. We bound the copy by both the
/// available data and `max - 1`.
pub fn get_string(raw: &[u8], element: usize, max: usize) -> Option<String> {
    if max == 0 {
        return None;
    }
    let t = typecode(raw)?;
    let cap = max - 1; // reserve room for the terminator

    let bytes: &[u8] = match t {
        CipType::String => {
            // Plain CIP STRING: type, then a subtype word, then the characters.
            let data = raw.get(TYPECODE_SIZE + 2..)?;
            let n = data.len().min(cap);
            &data[..n]
        }
        CipType::Struct => {
            let handle = rd_u16(raw, TYPECODE_SIZE)?;
            if handle != STRUCT_STRING_HANDLE {
                log::warn!("EIP get_string: unknown struct handle 0x{handle:04X}");
                return None;
            }
            let base = TYPECODE_SIZE + 2 + element * STRUCT_STRING_STRIDE;
            let len = rd_u16(raw, base)? as usize;
            // base+2 is the "no idea what this is" pad word.
            let text = raw.get(base + STRUCT_STRING_LEN_BYTES..)?;
            let n = len.min(cap).min(text.len()).min(STRUCT_STRING_BUF);
            &text[..n]
        }
        other => {
            log::warn!("EIP get_string: unhandled type {other:?}");
            return None;
        }
    };

    // Text may carry an embedded NUL; stop there like the C's consumers do.
    let end = bytes.iter().position(|&c| c == 0).unwrap_or(bytes.len());
    Some(String::from_utf8_lossy(&bytes[..end]).into_owned())
}

/// Write `element` from a float, converting to the tag's CIP type.
pub fn put_double(raw: &mut [u8], element: usize, value: f64) -> bool {
    let Some(t) = typecode(raw) else { return false };
    let at = elem_off(t, element);
    match t {
        CipType::Bool | CipType::Sint => wr_slice(raw, at, &[value as i64 as u8]),
        CipType::Int | CipType::Uint => wr_slice(raw, at, &(value as i64 as u16).to_le_bytes()),
        CipType::Dint | CipType::Bits => wr_slice(raw, at, &(value as i64 as u32).to_le_bytes()),
        CipType::Lint | CipType::Ulint => wr_slice(raw, at, &(value as i64).to_le_bytes()),
        CipType::Real => wr_slice(raw, at, &(value as f32).to_bits().to_le_bytes()),
        CipType::Lreal => wr_slice(raw, at, &value.to_bits().to_le_bytes()),
        _ => false,
    }
}

/// Write `element` from a signed 32-bit integer.
pub fn put_dint(raw: &mut [u8], element: usize, value: i32) -> bool {
    let Some(t) = typecode(raw) else { return false };
    let at = elem_off(t, element);
    match t {
        CipType::Bool | CipType::Sint => wr_slice(raw, at, &[value as u8]),
        CipType::Int | CipType::Uint => wr_slice(raw, at, &(value as u16).to_le_bytes()),
        CipType::Dint | CipType::Bits => wr_slice(raw, at, &value.to_le_bytes()),
        CipType::Lint | CipType::Ulint => wr_slice(raw, at, &(value as i64).to_le_bytes()),
        CipType::Real => wr_slice(raw, at, &(value as f32).to_bits().to_le_bytes()),
        CipType::Lreal => wr_slice(raw, at, &(value as f64).to_bits().to_le_bytes()),
        _ => false,
    }
}

/// Write `element` from a raw 32-bit bit pattern (the bit records' path).
pub fn put_udint(raw: &mut [u8], element: usize, value: u32) -> bool {
    let Some(t) = typecode(raw) else { return false };
    let at = elem_off(t, element);
    match t {
        CipType::Bool | CipType::Sint => wr_slice(raw, at, &[value as u8]),
        CipType::Int | CipType::Uint => wr_slice(raw, at, &(value as u16).to_le_bytes()),
        CipType::Dint | CipType::Bits => wr_slice(raw, at, &value.to_le_bytes()),
        CipType::Real => wr_slice(raw, at, &(value as f32).to_bits().to_le_bytes()),
        CipType::Lreal => wr_slice(raw, at, &(value as f64).to_bits().to_le_bytes()),
        _ => false,
    }
}

/// Write `element` from a signed 64-bit integer.
pub fn put_lint(raw: &mut [u8], element: usize, value: i64) -> bool {
    let Some(t) = typecode(raw) else { return false };
    let at = elem_off(t, element);
    match t {
        CipType::Lint | CipType::Ulint => wr_slice(raw, at, &value.to_le_bytes()),
        _ => false,
    }
}

/// Set the text of a Logix STRING struct in place, leaving the rest of the
/// buffer alone. Returns false for a non-STRING buffer.
pub fn put_string(raw: &mut [u8], value: &str) -> bool {
    let Some(CipType::Struct) = typecode(raw) else {
        return false;
    };
    let Some(handle) = rd_u16(raw, TYPECODE_SIZE) else {
        return false;
    };
    if handle != STRUCT_STRING_HANDLE {
        return false;
    }
    let len_at = TYPECODE_SIZE + 2;
    let text_at = len_at + STRUCT_STRING_LEN_BYTES;
    if text_at >= raw.len() {
        return false;
    }

    let bytes = value.as_bytes();
    // Leave room for the NUL the C also writes, and never exceed the struct's
    // own character buffer.
    let room = (raw.len() - text_at).saturating_sub(1);
    let len = bytes.len().min(room).min(STRUCT_STRING_MAX);

    if !wr_slice(raw, len_at, &(len as u16).to_le_bytes()) {
        return false;
    }
    if !wr_slice(raw, text_at, &bytes[..len]) {
        return false;
    }
    raw[text_at + len] = 0;
    true
}
