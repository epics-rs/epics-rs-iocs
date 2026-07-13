//! AMS/TCP framing and AoE (ADS-over-EtherCAT) header codec.
//!
//! Wire layout taken from Beckhoff `AdsLib/AmsHeader.h` (MIT). Everything is
//! little-endian and packed; there is no alignment padding anywhere on the wire.
//!
//! ```text
//! +----------------------+---------------------+-----------------+
//! | AmsTcpHeader (6 B)   | AoEHeader (32 B)    | payload (n B)   |
//! | reserved u16 = 0     | targetNetId  [6]    |                 |
//! | length    u32 = 32+n | targetPort   u16    |                 |
//! |                      | sourceNetId  [6]    |                 |
//! |                      | sourcePort   u16    |                 |
//! |                      | cmdId        u16    |                 |
//! |                      | stateFlags   u16    |                 |
//! |                      | length       u32 = n|                 |
//! |                      | errorCode    u32    |                 |
//! |                      | invokeId     u32    |                 |
//! +----------------------+---------------------+-----------------+
//! ```

use super::defs::{AmsAddr, AmsNetId, STATE_FLAG_REQUEST};
use super::error::AdsError;

/// Size of the AMS/TCP header that prefixes every frame on the socket.
pub const AMS_TCP_HEADER_LEN: usize = 6;
/// Size of the AoE header.
pub const AOE_HEADER_LEN: usize = 32;

/// A decoded AoE header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AoeHeader {
    pub target: AmsAddr,
    pub source: AmsAddr,
    pub cmd_id: u16,
    pub state_flags: u16,
    pub length: u32,
    pub error_code: u32,
    pub invoke_id: u32,
}

impl AoeHeader {
    /// Build a request header (`stateFlags = AMS_REQUEST`).
    pub fn request(
        target: AmsAddr,
        source: AmsAddr,
        cmd_id: u16,
        payload_len: u32,
        invoke_id: u32,
    ) -> Self {
        Self {
            target,
            source,
            cmd_id,
            state_flags: STATE_FLAG_REQUEST,
            length: payload_len,
            error_code: 0,
            invoke_id,
        }
    }

    /// Append the AoE header bytes to `out`.
    pub fn encode_into(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.target.net_id.0);
        out.extend_from_slice(&self.target.port.to_le_bytes());
        out.extend_from_slice(&self.source.net_id.0);
        out.extend_from_slice(&self.source.port.to_le_bytes());
        out.extend_from_slice(&self.cmd_id.to_le_bytes());
        out.extend_from_slice(&self.state_flags.to_le_bytes());
        out.extend_from_slice(&self.length.to_le_bytes());
        out.extend_from_slice(&self.error_code.to_le_bytes());
        out.extend_from_slice(&self.invoke_id.to_le_bytes());
    }

    /// Decode exactly [`AOE_HEADER_LEN`] bytes.
    pub fn decode(buf: &[u8]) -> Result<Self, AdsError> {
        if buf.len() < AOE_HEADER_LEN {
            return Err(AdsError::ShortFrame {
                need: AOE_HEADER_LEN,
                got: buf.len(),
            });
        }
        let mut r = Reader::new(buf);
        let target_net = AmsNetId(r.array6()?);
        let target_port = r.u16()?;
        let source_net = AmsNetId(r.array6()?);
        let source_port = r.u16()?;
        Ok(Self {
            target: AmsAddr {
                net_id: target_net,
                port: target_port,
            },
            source: AmsAddr {
                net_id: source_net,
                port: source_port,
            },
            cmd_id: r.u16()?,
            state_flags: r.u16()?,
            length: r.u32()?,
            error_code: r.u32()?,
            invoke_id: r.u32()?,
        })
    }
}

/// Serialize one complete AMS/TCP frame: TCP header + AoE header + payload.
pub fn encode_frame(header: &AoeHeader, payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(AMS_TCP_HEADER_LEN + AOE_HEADER_LEN + payload.len());
    out.extend_from_slice(&0u16.to_le_bytes()); // reserved
    let len = (AOE_HEADER_LEN + payload.len()) as u32;
    out.extend_from_slice(&len.to_le_bytes());
    header.encode_into(&mut out);
    out.extend_from_slice(payload);
    out
}

/// Decode the AMS/TCP header, returning the byte count that follows it.
pub fn decode_tcp_header(buf: &[u8]) -> Result<u32, AdsError> {
    if buf.len() < AMS_TCP_HEADER_LEN {
        return Err(AdsError::ShortFrame {
            need: AMS_TCP_HEADER_LEN,
            got: buf.len(),
        });
    }
    // `reserved` (buf[0..2]) is ignored on receive, exactly as AmsConnection does.
    Ok(u32::from_le_bytes([buf[2], buf[3], buf[4], buf[5]]))
}

/// Little-endian cursor over a received payload.
///
/// Every accessor bounds-checks and yields [`AdsError::ShortFrame`] instead of
/// panicking — the bytes come off a socket and a malformed or truncated frame
/// must never take the IOC down.
pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], AdsError> {
        if self.remaining() < n {
            return Err(AdsError::ShortFrame {
                need: n,
                got: self.remaining(),
            });
        }
        let s = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }

    pub fn u16(&mut self) -> Result<u16, AdsError> {
        let s = self.take(2)?;
        Ok(u16::from_le_bytes([s[0], s[1]]))
    }

    pub fn u32(&mut self) -> Result<u32, AdsError> {
        let s = self.take(4)?;
        Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }

    pub fn u64(&mut self) -> Result<u64, AdsError> {
        let s = self.take(8)?;
        Ok(u64::from_le_bytes([
            s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7],
        ]))
    }

    pub fn u8(&mut self) -> Result<u8, AdsError> {
        Ok(self.take(1)?[0])
    }

    pub fn array6(&mut self) -> Result<[u8; 6], AdsError> {
        let s = self.take(6)?;
        let mut a = [0u8; 6];
        a.copy_from_slice(s);
        Ok(a)
    }

    pub fn bytes(&mut self, n: usize) -> Result<&'a [u8], AdsError> {
        self.take(n)
    }

    /// Read a NUL-terminated string. The NUL is consumed but not returned.
    pub fn cstr(&mut self) -> Result<String, AdsError> {
        let start = self.pos;
        while self.pos < self.buf.len() {
            if self.buf[self.pos] == 0 {
                let s = String::from_utf8_lossy(&self.buf[start..self.pos]).into_owned();
                self.pos += 1;
                return Ok(s);
            }
            self.pos += 1;
        }
        Err(AdsError::ShortFrame {
            need: self.pos - start + 1,
            got: self.pos - start,
        })
    }

    /// Skip forward, e.g. over reserved fields.
    pub fn skip(&mut self, n: usize) -> Result<(), AdsError> {
        self.take(n).map(|_| ())
    }
}

/// The `AoERequestHeader` triple shared by READ/WRITE (`AmsHeader.h:41`).
pub fn push_request_header(out: &mut Vec<u8>, index_group: u32, index_offset: u32, length: u32) {
    out.extend_from_slice(&index_group.to_le_bytes());
    out.extend_from_slice(&index_offset.to_le_bytes());
    out.extend_from_slice(&length.to_le_bytes());
}

/// `AoEReadWriteReqHeader` (`AmsHeader.h:66`).
pub fn push_read_write_header(
    out: &mut Vec<u8>,
    index_group: u32,
    index_offset: u32,
    read_length: u32,
    write_length: u32,
) {
    push_request_header(out, index_group, index_offset, read_length);
    out.extend_from_slice(&write_length.to_le_bytes());
}

/// `AdsAddDeviceNotificationRequest` (`AmsHeader.h:92`) — 40 bytes including
/// the 16-byte reserved tail.
#[allow(clippy::too_many_arguments)]
pub fn push_add_notification_request(
    out: &mut Vec<u8>,
    index_group: u32,
    index_offset: u32,
    length: u32,
    mode: u32,
    max_delay_100ns: u32,
    cycle_time_100ns: u32,
) {
    out.extend_from_slice(&index_group.to_le_bytes());
    out.extend_from_slice(&index_offset.to_le_bytes());
    out.extend_from_slice(&length.to_le_bytes());
    out.extend_from_slice(&mode.to_le_bytes());
    out.extend_from_slice(&max_delay_100ns.to_le_bytes());
    out.extend_from_slice(&cycle_time_100ns.to_le_bytes());
    out.extend_from_slice(&[0u8; 16]);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ads::defs::{ADSIGRP_SYM_HNDBYNAME, CMD_READ_WRITE, STATE_FLAG_RESPONSE};

    fn addr(net: [u8; 6], port: u16) -> AmsAddr {
        AmsAddr {
            net_id: AmsNetId(net),
            port,
        }
    }

    /// Golden bytes for a SYM_HNDBYNAME READ_WRITE, byte-for-byte what the C
    /// AdsLib puts on the wire for the same call (AmsHeader.h layout, packed LE).
    #[test]
    fn read_write_frame_golden_bytes() {
        let target = addr([192, 168, 88, 44, 1, 1], 851);
        let source = addr([10, 0, 0, 5, 1, 1], 30000);
        let name = b"MAIN.fTest";

        let mut payload = Vec::new();
        push_read_write_header(&mut payload, ADSIGRP_SYM_HNDBYNAME, 0, 4, name.len() as u32);
        payload.extend_from_slice(name);

        let header = AoeHeader::request(
            target,
            source,
            CMD_READ_WRITE,
            payload.len() as u32,
            0x1234_5678,
        );
        let frame = encode_frame(&header, &payload);

        #[rustfmt::skip]
        let expect: Vec<u8> = vec![
            // AmsTcpHeader: reserved=0, length = 32 + 16 + 10 = 58
            0x00, 0x00,
            0x3A, 0x00, 0x00, 0x00,
            // AoEHeader
            192, 168, 88, 44, 1, 1,      // targetNetId
            0x53, 0x03,                  // targetPort  = 851
            10, 0, 0, 5, 1, 1,           // sourceNetId
            0x30, 0x75,                  // sourcePort  = 30000
            0x09, 0x00,                  // cmdId       = READ_WRITE
            0x04, 0x00,                  // stateFlags  = AMS_REQUEST
            0x1A, 0x00, 0x00, 0x00,      // length      = 26
            0x00, 0x00, 0x00, 0x00,      // errorCode
            0x78, 0x56, 0x34, 0x12,      // invokeId
            // AoEReadWriteReqHeader
            0x03, 0xF0, 0x00, 0x00,      // indexGroup  = 0xF003
            0x00, 0x00, 0x00, 0x00,      // indexOffset
            0x04, 0x00, 0x00, 0x00,      // readLength  = 4
            0x0A, 0x00, 0x00, 0x00,      // writeLength = 10
            // payload
            b'M', b'A', b'I', b'N', b'.', b'f', b'T', b'e', b's', b't',
        ];
        assert_eq!(frame, expect);
        assert_eq!(frame.len(), AMS_TCP_HEADER_LEN + AOE_HEADER_LEN + 26);
    }

    #[test]
    fn tcp_header_length_counts_aoe_plus_payload() {
        let h = AoeHeader::request(addr([0; 6], 851), addr([0; 6], 30000), 2, 12, 1);
        let frame = encode_frame(&h, &[0u8; 12]);
        assert_eq!(
            decode_tcp_header(&frame).unwrap(),
            (AOE_HEADER_LEN + 12) as u32
        );
        assert_eq!(frame.len(), AMS_TCP_HEADER_LEN + AOE_HEADER_LEN + 12);
    }

    #[test]
    fn aoe_header_roundtrips() {
        let h = AoeHeader {
            target: addr([1, 2, 3, 4, 5, 6], 851),
            source: addr([7, 8, 9, 10, 11, 12], 30001),
            cmd_id: CMD_READ_WRITE,
            state_flags: STATE_FLAG_RESPONSE,
            length: 42,
            error_code: 0x0710,
            invoke_id: 0xCAFE_BABE,
        };
        let mut buf = Vec::new();
        h.encode_into(&mut buf);
        assert_eq!(buf.len(), AOE_HEADER_LEN);
        assert_eq!(AoeHeader::decode(&buf).unwrap(), h);
    }

    #[test]
    fn add_notification_request_is_40_bytes() {
        let mut out = Vec::new();
        push_add_notification_request(&mut out, 0xF005, 7, 4, 4, 1_000_000, 500_000);
        assert_eq!(out.len(), 40);
        assert_eq!(&out[0..4], &0xF005u32.to_le_bytes());
        assert_eq!(&out[4..8], &7u32.to_le_bytes());
        assert_eq!(&out[8..12], &4u32.to_le_bytes());
        assert_eq!(&out[12..16], &4u32.to_le_bytes());
        assert_eq!(&out[16..20], &1_000_000u32.to_le_bytes());
        assert_eq!(&out[20..24], &500_000u32.to_le_bytes());
        assert_eq!(&out[24..40], &[0u8; 16]);
    }

    #[test]
    fn truncated_frames_error_instead_of_panicking() {
        assert!(matches!(
            decode_tcp_header(&[0, 0, 1]),
            Err(AdsError::ShortFrame { .. })
        ));
        assert!(matches!(
            AoeHeader::decode(&[0u8; 31]),
            Err(AdsError::ShortFrame { .. })
        ));

        let mut r = Reader::new(&[1, 2, 3]);
        assert!(r.u16().is_ok());
        assert!(matches!(r.u32(), Err(AdsError::ShortFrame { .. })));
    }

    #[test]
    fn reader_cstr_stops_at_nul_and_errors_when_unterminated() {
        let mut r = Reader::new(b"MAIN.fTest\0rest");
        assert_eq!(r.cstr().unwrap(), "MAIN.fTest");
        assert_eq!(r.remaining(), 4);

        let mut r = Reader::new(b"no-nul-here");
        assert!(matches!(r.cstr(), Err(AdsError::ShortFrame { .. })));
    }
}
