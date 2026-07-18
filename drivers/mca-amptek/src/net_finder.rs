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
//!
//! # Fixed (not reproduced) upstream defect
//! `CDppSocket::HaveNetFinderPacket` (`DppSocket.cpp:428-450`) validates
//! a broadcast response's `(buffer[0], buffer[2..4])` against the
//! expected "Silicon Labs NetFinder packet" shape and the correlation
//! nonce, but its final `else` branch -- reached by any buffer that
//! matches *neither* the valid shape *nor* the specific
//! `buffer[0]==0x00`-with-matching-nonce "unknown packet type" shape --
//! returns `true` (with a `"no packet test"` log line) instead of
//! `false`. That default-accept means arbitrary UDP traffic landing on
//! the discovery socket (any other broadcast chatter on the LAN, or a
//! spoofed packet, since this is unauthenticated broadcast UDP) gets
//! treated as a confirmed device response and its bytes fed to
//! `CNetFinder::AddEntry`. [`have_netfinder_packet`] here defaults to
//! rejecting (`false`) unless the buffer positively matches the valid
//! shape, closing that gap.

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

/// `CDppSocket::HaveNetFinderPacket` (`DppSocket.cpp:428-450`), with the
/// default-accept fallthrough fixed to default-reject -- see the module
/// doc's "Fixed (not reproduced) upstream defect" note.
pub fn have_netfinder_packet(buf: &[u8], rand: u16) -> bool {
    if buf.len() < 4 {
        return false;
    }
    let rand_hi = (rand >> 8) as u8;
    let rand_lo = (rand & 0xFF) as u8;
    buf.len() >= 32 && buf[0] == 0x01 && buf[2] == rand_hi && buf[3] == rand_lo
}

/// `doAmptekNetFinderPacket`'s outbound unicast request
/// (`ConsoleHelper.cpp:199-261`), sent to `target:10001`.
pub fn build_direct_request() -> Vec<u8> {
    protocol::build_netfinder_direct_request()
}

/// `CNetFinder::AddEntry`'s `SockAddr` extraction
/// (`NetFinder.cpp:276-342`) applied to a
/// [`protocol::parse_netfinder_direct_response_header`]-validated
/// direct-query response: `SockAddr = SockAddr_ByteToULong(&buffer[20])`,
/// i.e. the 4 bytes at DATA offset 20..24 (wire offset 26..30, since
/// `AddEntry` is called on the header-trimmed buffer in C).
///
/// # Restructuring vs. C
/// C composes those 4 bytes into an `in_addr` via
/// `ByteToInaddr(b0,b1,b2,b3) = htonl((b0<<24)|(b1<<16)|(b2<<8)|b3)`
/// specifically so the result's in-memory byte order matches what
/// `inet_addr("b0.b1.b2.b3")` produces, purely so the two can be compared
/// as raw `unsigned long`s. [`std::net::Ipv4Addr`] already models a
/// 4-octet address with no such byte-order indirection, so this returns
/// one directly and the caller compares it to the target `Ipv4Addr` with
/// plain `==` -- equivalent to C's `lTestAddr == lDppAddr` comparison
/// without needing to reproduce the `htonl`/`inet_addr` round trip.
pub fn parse_direct_response(raw: &[u8]) -> Option<Ipv4Addr> {
    let data = protocol::parse_netfinder_direct_response_header(raw)?;
    if data.len() < 24 {
        return None;
    }
    Some(Ipv4Addr::new(data[20], data[21], data[22], data[23]))
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
        assert!(have_netfinder_packet(&valid_response(0x55AA), 0x55AA));
    }

    #[test]
    fn have_netfinder_packet_rejects_mismatched_nonce() {
        assert!(!have_netfinder_packet(&valid_response(0x55AA), 0x1234));
    }

    #[test]
    fn have_netfinder_packet_rejects_short_buffer() {
        let short = &valid_response(0x55AA)[..31];
        assert!(!have_netfinder_packet(short, 0x55AA));
    }

    /// The fixed defect: unrecognized-shape traffic must be rejected,
    /// not default-accepted.
    #[test]
    fn have_netfinder_packet_rejects_unrecognized_shape() {
        let garbage = vec![0x42u8; 40];
        assert!(!have_netfinder_packet(&garbage, 0x55AA));
    }

    #[test]
    fn build_direct_request_is_eight_bytes() {
        assert_eq!(build_direct_request().len(), 8);
    }

    #[test]
    fn parse_direct_response_extracts_ip() {
        let mut data = vec![0u8; 24];
        data[20..24].copy_from_slice(&[192, 168, 0, 42]);
        let mut raw = vec![
            protocol::SYNC1,
            protocol::SYNC2,
            0x82,
            0x08,
            0x00,
            data.len() as u8,
        ];
        raw.extend_from_slice(&data);
        raw.extend_from_slice(&[0, 0]); // unverified trailer
        assert_eq!(
            parse_direct_response(&raw),
            Some(Ipv4Addr::new(192, 168, 0, 42))
        );
    }

    #[test]
    fn parse_direct_response_rejects_short_data() {
        let mut raw = vec![protocol::SYNC1, protocol::SYNC2, 0x82, 0x08, 0x00, 0x04];
        raw.extend_from_slice(&[1, 2, 3, 4]);
        assert_eq!(parse_direct_response(&raw), None);
    }
}
