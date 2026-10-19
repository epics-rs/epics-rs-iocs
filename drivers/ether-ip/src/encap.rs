//! EtherNet/IP encapsulation layer.
//!
//! Port of the encapsulation half of `ether_ip.c`: the 24-byte
//! `EncapsulationHeader`, the `ListServices` / `RegisterSession` /
//! `UnRegisterSession` / `SendRRData` commands, and the CPF (Common Packet
//! Format) item list that carries a CIP message router PDU.
//!
//! Reference: ODVA "Volume 2: EtherNet/IP Adaptation of CIP", and
//! `ether_ip.c:2286-2534` / `:2951-3013`.

use std::sync::atomic::{AtomicU32, Ordering};

/// TCP port for EtherNet/IP ("0xAF12", `ether_ip.h:ETHERIP_PORT`).
pub const ETHERIP_PORT: u16 = 0xAF12;

/// `EIP_BUFFER_SIZE` (`ether_ip.h`).
pub const BUFFER_SIZE: usize = 580;

/// `EIP_DEFAULT_BUFFER_LIMIT` (`ether_ip.h`) -- how much of the buffer a
/// request may occupy. Tunable at runtime, as in the C.
pub const DEFAULT_BUFFER_LIMIT: usize = 480;

/// `sizeof_EncapsulationHeader`.
pub const HEADER_SIZE: usize = 24;

/// The CPF item list that follows the header in a `SendRRData`.
pub const RRDATA_PREAMBLE_SIZE: usize = 16;

/// `sizeof_EncapsulationRRData` = header + CPF preamble.
pub const RRDATA_SIZE: usize = HEADER_SIZE + RRDATA_PREAMBLE_SIZE;

/// Length of the transaction-id "sender context" field.
pub const TRANS_ID_LEN: usize = 8;

// Encapsulation commands (`ether_ip.h: EncapsulationCommand`).
pub const EC_LIST_SERVICES: u16 = 0x0004;
pub const EC_REGISTER_SESSION: u16 = 0x0065;
pub const EC_UNREGISTER_SESSION: u16 = 0x0066;
pub const EC_SEND_RR_DATA: u16 = 0x006F;

/// CPF item id: null address (UCMM).
const CPF_NULL_ADDRESS: u16 = 0x0000;
/// CPF item id: unconnected message data.
const CPF_UNCONNECTED_DATA: u16 = 0x00B2;

/// The "sender context" the C fills with a process-wide counter in eight
/// uppercase hex ASCII digits (`ether_ip.c:generateTransactionId`). The PLC
/// echoes it verbatim; we compare request and reply to detect a
/// desynchronized stream.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct TransactionId(pub [u8; TRANS_ID_LEN]);

/// `EIP_TRANS_ID_MIN_NUM` / `EIP_TRANS_ID_MAX_NUM`.
const TRANS_ID_MIN: u32 = 1;
const TRANS_ID_MAX: u32 = 0x7FFF_FFFF;

static NEXT_TRANS_ID: AtomicU32 = AtomicU32::new(TRANS_ID_MIN);

impl TransactionId {
    /// Next id in the sequence; back to 1 once the counter reaches
    /// `TRANS_ID_MAX`, so it runs 1..=0x7FFFFFFE as in the C.
    pub fn generate() -> TransactionId {
        let n = NEXT_TRANS_ID.update(Ordering::Relaxed, Ordering::Relaxed, |n| {
            if n + 1 >= TRANS_ID_MAX {
                TRANS_ID_MIN
            } else {
                n + 1
            }
        });
        TransactionId::from_counter(n)
    }

    /// The pure function behind [`generate`](Self::generate), so tests can pin
    /// the encoding without touching the global counter.
    pub fn from_counter(n: u32) -> TransactionId {
        let s = format!("{n:08X}");
        let mut id = [0u8; TRANS_ID_LEN];
        id.copy_from_slice(s.as_bytes());
        TransactionId(id)
    }
}

impl std::fmt::Display for TransactionId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&String::from_utf8_lossy(&self.0))
    }
}

/// A decoded `EncapsulationHeader`.
#[derive(Clone, Copy, Debug)]
pub struct EncapHeader {
    pub command: u16,
    pub length: u16,
    pub session: u32,
    pub status: u32,
    pub trans_id: TransactionId,
    pub options: u32,
}

impl EncapHeader {
    pub fn decode(buf: &[u8]) -> Option<EncapHeader> {
        if buf.len() < HEADER_SIZE {
            return None;
        }
        let mut trans_id = [0u8; TRANS_ID_LEN];
        trans_id.copy_from_slice(&buf[12..20]);
        Some(EncapHeader {
            command: u16::from_le_bytes([buf[0], buf[1]]),
            length: u16::from_le_bytes([buf[2], buf[3]]),
            session: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            status: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            trans_id: TransactionId(trans_id),
            options: u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
        })
    }
}

/// Human-readable encapsulation status (`ether_ip.c:2254`).
pub fn encap_status_text(status: u32) -> &'static str {
    match status {
        0x0000 => "OK",
        0x0001 => "invalid/unsupported command",
        0x0002 => "no memory on target",
        0x0003 => "malformed data in request",
        0x0064 => "invalid session ID",
        0x0065 => "invalid data length",
        0x0069 => "unsupported protocol revision",
        _ => "<unknown>",
    }
}

/// Emit an `EncapsulationHeader`. `length` is the byte count of the data that
/// follows.
pub fn encode_header(
    out: &mut Vec<u8>,
    command: u16,
    length: u16,
    session: u32,
    trans_id: TransactionId,
    options: u32,
) {
    out.extend_from_slice(&command.to_le_bytes());
    out.extend_from_slice(&length.to_le_bytes());
    out.extend_from_slice(&session.to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes()); // status: always 0 in a request
    out.extend_from_slice(&trans_id.0);
    out.extend_from_slice(&options.to_le_bytes());
}

/// `ListServices` request: header only, session 0.
///
/// C defect not copied: `make_EncapsulationHeader` packs `c->session`, which
/// drvEtherIP's reconnect reuses uncleared, so C's ListServices and
/// RegisterSession after a reconnect carry the previous session handle.
pub fn encode_list_services(trans_id: TransactionId) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_SIZE);
    encode_header(&mut out, EC_LIST_SERVICES, 0, 0, trans_id, 0);
    out
}

/// A single `ListServices` reply item.
#[derive(Clone, Debug)]
pub struct ServiceItem {
    pub item_type: u16,
    pub length: u16,
    pub version: u16,
    pub flags: u16,
    pub name: String,
}

impl ServiceItem {
    /// Bit 5 of `flags`: the target speaks CIP-over-encapsulation
    /// (`ether_ip.c:2426`).
    pub fn supports_cip(&self) -> bool {
        self.flags & (1 << 5) != 0
    }
}

/// Decode the item list of a `ListServices` reply (the body after the header).
pub fn decode_list_services(body: &[u8]) -> Option<Vec<ServiceItem>> {
    let count = rd_u16(body, 0)? as usize;
    let mut items = Vec::with_capacity(count);
    let mut at = 2;
    for _ in 0..count {
        let item_type = rd_u16(body, at)?;
        let length = rd_u16(body, at + 2)?;
        let version = rd_u16(body, at + 4)?;
        let flags = rd_u16(body, at + 6)?;
        let name_raw = body.get(at + 8..at + 24)?;
        let end = name_raw
            .iter()
            .position(|&c| c == 0)
            .unwrap_or(name_raw.len());
        items.push(ServiceItem {
            item_type,
            length,
            version,
            flags,
            name: String::from_utf8_lossy(&name_raw[..end]).into_owned(),
        });
        // `length` counts the bytes after type+length; the C decodes a fixed
        // 20-byte body (version, flags, name[16]) but honouring `length` keeps
        // us in step with a target that appends vendor-specific trailer bytes.
        at += 4 + usize::from(length).max(20);
    }
    Some(items)
}

/// `RegisterSession` request: header (session 0, see
/// [`encode_list_services`]) + protocol_version(1) + options(0).
pub fn encode_register_session(trans_id: TransactionId) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_SIZE + 4);
    encode_header(&mut out, EC_REGISTER_SESSION, 4, 0, trans_id, 0);
    out.extend_from_slice(&1u16.to_le_bytes()); // protocol version
    out.extend_from_slice(&0u16.to_le_bytes()); // options
    out
}

/// `UnRegisterSession` request: header only, carrying the session id.
pub fn encode_unregister_session(session: u32, trans_id: TransactionId) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_SIZE);
    encode_header(&mut out, EC_UNREGISTER_SESSION, 0, session, trans_id, 0);
    out
}

/// `SendRRData`: header, CPF item list, then `payload` (a complete MR_Request).
pub fn encode_send_rr_data(session: u32, trans_id: TransactionId, payload: &[u8]) -> Vec<u8> {
    let len = payload.len();
    let mut out = Vec::with_capacity(RRDATA_SIZE + len);
    encode_header(
        &mut out,
        EC_SEND_RR_DATA,
        (RRDATA_PREAMBLE_SIZE + len) as u16,
        session,
        trans_id,
        0,
    );
    out.extend_from_slice(&0u32.to_le_bytes()); // interface handle
    out.extend_from_slice(&0u16.to_le_bytes()); // timeout
    out.extend_from_slice(&2u16.to_le_bytes()); // item count: address + data
    out.extend_from_slice(&CPF_NULL_ADDRESS.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes()); // address length
    out.extend_from_slice(&CPF_UNCONNECTED_DATA.to_le_bytes());
    out.extend_from_slice(&(len as u16).to_le_bytes());
    out.extend_from_slice(payload);
    out
}

/// A decoded `SendRRData` reply.
#[derive(Clone, Copy, Debug)]
pub struct RrDataReply<'a> {
    pub header: EncapHeader,
    /// The CPF data item's declared length.
    pub data_length: usize,
    /// The enclosed MR_Response, clipped to `data_length`.
    pub response: &'a [u8],
}

/// Unpack a `SendRRData` reply into its header and the MR_Response it carries.
pub fn decode_rr_data(buf: &[u8]) -> Option<RrDataReply<'_>> {
    let header = EncapHeader::decode(buf)?;
    let data_length = rd_u16(buf, HEADER_SIZE + 14)? as usize;
    let start = RRDATA_SIZE;
    let end = start.checked_add(data_length)?;
    // A truncated frame must not silently yield a short response: the C reads
    // `data_length` bytes out of a fixed 580-byte buffer and trusts the target.
    if end > buf.len() {
        return None;
    }
    Some(RrDataReply {
        header,
        data_length,
        response: &buf[start..end],
    })
}

fn rd_u16(buf: &[u8], at: usize) -> Option<u16> {
    let b = buf.get(at..at + 2)?;
    Some(u16::from_le_bytes([b[0], b[1]]))
}
