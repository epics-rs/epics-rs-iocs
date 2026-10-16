//! Amptek/Silicon Labs NetFinder discovery, ported from `NetFinder.cpp`
//! and the NetFinder-specific pieces of `DppSocket.cpp`/`ConsoleHelper.cpp`,
//! restricted to what `drvAmptek.cpp` reaches through
//! `DppSocket_Connect_Direct_DPP` (unicast, `directMode=1`) and
//! `DppSocket_Connect_Default_DPP` (broadcast discovery, `directMode=0`).
//!
//! `CNetFinder::EntryToStr`/`EntryToStrRS232`/`EntryToStrUSB` (display
//! only) and every `NETDISPLAY_ENTRY` field but the discovered IP are not
//! ported -- `drvAmptek.cpp` only ever needs the responding device's
//! address to confirm/complete a connection, never the MAC, subnet,
//! event schedule, or description text `CNetFinder::AddEntry` also
//! parses.

use std::net::Ipv4Addr;

use crate::protocol;

/// `CDppSocket::CreateRand` (`DppSocket.cpp:417-426`): a 15-bit nonzero
/// correlation nonce identifying this discovery round's broadcast, drawn
/// from OS-backed entropy (`std::collections::hash_map::RandomState`'s
/// per-instance random keys) rather than reproducing C's
/// `srand(time(NULL)); rand()` -- a fixed-seed PRNG would defeat the
/// nonce's purpose of not colliding with a previous round's stray
/// replies still in flight.
pub fn create_rand() -> u16 {
    use std::collections::hash_map::RandomState;
    use std::hash::{BuildHasher, Hasher};
    loop {
        let v = (RandomState::new().build_hasher().finish() & 0x7FFF) as u16;
        if v != 0 {
            return v;
        }
    }
}

/// `CDppSocket::SendNetFinderBroadCast`'s request buffer
/// (`DppSocket.cpp:116-143`): `[0x00, 0x00, rand_hi, rand_lo, 0xF4,
/// 0xFA]`, sent to each broadcast interface's address on UDP port 3040.
pub fn build_broadcast_request(rand: u16) -> [u8; 6] {
    [
        0x00,
        0x00,
        (rand >> 8) as u8,
        (rand & 0xFF) as u8,
        0xF4,
        0xFA,
    ]
}

/// `CDppSocket::HaveNetFinderPacket` (`DppSocket.cpp:428-450`). `buf` is
/// the whole receive buffer and `num_bytes` the size of the last datagram:
/// C reads bytes 0..4 whatever was received, and accepts every packet that
/// is neither a NetFinder reply nor an echo of its own request.
pub fn have_netfinder_packet(buf: &[u8], rand: u16, num_bytes: usize) -> bool {
    let rand_hi = (rand >> 8) as u8;
    let rand_lo = (rand & 0xFF) as u8;
    let nonce = buf[2] == rand_hi && buf[3] == rand_lo;
    if num_bytes >= 32 && buf[0] == 0x01 && nonce {
        true
    } else {
        !(buf[0] == 0x00 && nonce)
    }
}

/// `doAmptekNetFinderPacket`'s outbound unicast request
/// (`ConsoleHelper.cpp:199-261`), sent to `target:10001`.
pub fn build_direct_request() -> Vec<u8> {
    protocol::build_netfinder_direct_request()
}

/// `doAmptekNetFinderPacket`'s reply check (`ConsoleHelper.cpp:232-252`)
/// and `CNetFinder::AddEntry`'s `SockAddr = buffer[20..24]`
/// (`NetFinder.cpp:340`). `buf` is the zeroed receive buffer and `n` the
/// datagram size; like C, the checksum is not verified and a `LEN` past
/// the received bytes reads the zero padding.
pub fn parse_direct_response(buf: &mut [u8; 1024], n: usize) -> Option<Ipv4Addr> {
    let header = [
        protocol::SYNC1,
        protocol::SYNC2,
        protocol::pid1::RCV_SCOPE_MISC,
        protocol::pid2_netfinder::READBACK_RESPONSE,
    ];
    if buf[..4] != header {
        return None;
    }
    let len = usize::from(buf[4]) * 256 + usize::from(buf[5]);
    if len >= n {
        return None;
    }
    // C shifts and zero-fills up to `len + 6`, past the buffer when
    // `len > 1018` (UB); the copy stops at the buffer end here.
    let end = (len + 6).min(buf.len());
    buf.copy_within(6..end, 0);
    buf[len.min(end)..end].fill(0);
    Some(Ipv4Addr::new(buf[20], buf[21], buf[22], buf[23]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_rand_is_always_in_1_to_0x7fff() {
        for _ in 0..64 {
            let r = create_rand();
            assert!((1..=0x7FFF).contains(&r), "rand {r:#x} out of range");
        }
    }

    #[test]
    fn build_broadcast_request_packs_the_nonce_into_bytes_2_and_3() {
        assert_eq!(
            build_broadcast_request(0x1234),
            [0x00, 0x00, 0x12, 0x34, 0xF4, 0xFA]
        );
        assert_eq!(
            build_broadcast_request(0x0001),
            [0x00, 0x00, 0x00, 0x01, 0xF4, 0xFA]
        );
    }

    fn valid_response(rand: u16) -> Vec<u8> {
        let mut buf = vec![0x01, 0x00, (rand >> 8) as u8, (rand & 0xFF) as u8];
        buf.extend(std::iter::repeat_n(0u8, 32 - buf.len()));
        buf
    }

    #[test]
    fn have_netfinder_packet_accepts_matching_nonce() {
        assert!(have_netfinder_packet(&valid_response(0x55AA), 0x55AA, 32));
    }

    /// Only an echo of the request shape is refused; a short reply, a
    /// stale nonce and unrelated traffic all pass (`DppSocket.cpp:432-447`).
    #[test]
    fn have_netfinder_packet_accepts_everything_but_a_request_echo() {
        assert!(have_netfinder_packet(&valid_response(0x55AA), 0x1234, 32));
        assert!(have_netfinder_packet(&valid_response(0x55AA), 0x55AA, 31));
        assert!(have_netfinder_packet(&[0x42u8; 40], 0x55AA, 40));
        let echo = build_broadcast_request(0x55AA);
        assert!(!have_netfinder_packet(&echo, 0x55AA, echo.len()));
        assert!(have_netfinder_packet(&echo, 0x1234, echo.len()));
    }

    #[test]
    fn build_direct_request_is_eight_bytes() {
        assert_eq!(build_direct_request().len(), 8);
    }

    fn received(raw: &[u8]) -> [u8; 1024] {
        let mut buf = [0u8; 1024];
        buf[..raw.len()].copy_from_slice(raw);
        buf
    }

    #[test]
    fn parse_direct_response_extracts_ip() {
        let mut raw = vec![protocol::SYNC1, protocol::SYNC2, 0x82, 0x08, 0x00, 24];
        raw.extend_from_slice(&[0u8; 20]);
        raw.extend_from_slice(&[192, 168, 0, 42, 0, 0]);
        assert_eq!(
            parse_direct_response(&mut received(&raw), raw.len()),
            Some(Ipv4Addr::new(192, 168, 0, 42))
        );
    }

    /// C requires only `LEN < iSize` (`ConsoleHelper.cpp:237`): a short
    /// reply is accepted and the address comes from the zero padding.
    #[test]
    fn parse_direct_response_accepts_short_data_as_c() {
        let raw = [
            protocol::SYNC1,
            protocol::SYNC2,
            0x82,
            0x08,
            0x00,
            0x04,
            1,
            2,
        ];
        assert_eq!(
            parse_direct_response(&mut received(&raw), raw.len()),
            Some(Ipv4Addr::UNSPECIFIED)
        );
        let raw = [
            protocol::SYNC1,
            protocol::SYNC2,
            0x82,
            0x08,
            0x00,
            0x08,
            1,
            2,
        ];
        assert_eq!(parse_direct_response(&mut received(&raw), raw.len()), None);
        let raw = [
            protocol::SYNC1,
            protocol::SYNC2,
            0x80,
            0x01,
            0x00,
            0x00,
            0,
            0,
        ];
        assert_eq!(parse_direct_response(&mut received(&raw), raw.len()), None);
    }
}
