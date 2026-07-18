//! Raw UDP transport for the DP5 Ethernet interface, ported from
//! `CDppSocket`'s send/receive primitives (`DppSocket.cpp`) and the
//! connect sequences `CConsoleHelper::DppSocket_Connect_Direct_DPP`/
//! `DppSocket_Connect_Default_DPP` drive (`ConsoleHelper.cpp:300-374`).
//! `DppLibUsb`/USB and `DppInterfaceSerial` are out of scope -- see the
//! crate-level feasibility note in `lib.rs`.
//!
//! # Restructuring vs. C
//! C's `CDppSocket::UDPRecvFrom` sets the socket non-blocking, attempts a
//! `recvfrom`, and on failure sleeps for `timeout` (`CDppSocket::SetTimeOut`,
//! 10 ms for Ethernet -- `drvAmptek.cpp:49,303`) before trying exactly
//! once more. A single blocking `recv_from` with `set_read_timeout` set
//! to that same duration is behaviorally equivalent for every caller
//! here (all of which only care "did a response arrive within the
//! timeout budget, yes or no") without reproducing the
//! non-blocking-poll-then-sleep-then-retry-once dance. [`READ_TIMEOUT`]
//! is that 10 ms value (`drvAmptek.cpp:49`'s `#define TIMEOUT 0.01`).
//!
//! Neither this port nor C validates that a response's source address
//! matches the target it was sent to -- `UDPRecvFrom`/`UDPRecvFromNfAddr`
//! accept from any peer on the bound socket, relying on
//! [`crate::protocol::parse_packet`]'s checksum for integrity instead of
//! source-address filtering. [`AmptekUdpTransport`] uses `send_to`/
//! `recv_from` on an unconnected socket rather than `UdpSocket::connect`
//! to preserve that (the OS would otherwise filter by peer for a
//! connected socket, a behavior change C does not have).

use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::time::Duration;

use crate::net_finder;

/// `drvAmptek.cpp:49`: `#define TIMEOUT 0.01` (10 ms), applied via
/// `CH_.DppSocket.SetTimeOut` in `connectDevice` (`drvAmptek.cpp:303`)
/// for `DppInterfaceEthernet`. See the module doc's "Restructuring vs.
/// C" note for why this collapses C's poll/sleep/retry into one
/// bounded-wait `recv_from`.
pub const READ_TIMEOUT: Duration = Duration::from_millis(10);

/// The fixed DP5 command port every request in `drvAmptek.cpp` targets
/// (`SendPacketInet`/`doAmptekNetFinderPacket`, both hardcode `10001`).
pub const DP5_COMMAND_PORT: u16 = 10001;

/// The NetFinder broadcast-discovery port (`SendNetFinderBroadCast`,
/// `DppSocket.cpp:153`).
pub const NETFINDER_BROADCAST_PORT: u16 = 3040;

/// The largest single response `SendPacketInet` (`DppSocket.cpp:452-506`)
/// will accumulate before treating it as an overflow error.
const MAX_RESPONSE_SIZE: usize = 24648;

/// A single receive chunk size (`UDPRecvFrom(..., 1024, ...)`,
/// `SendPacketInet`'s call site, `DppSocket.cpp:470`).
const RECV_CHUNK_SIZE: usize = 1024;

/// `doNetFinderBroadcast`'s per-interface receive cap (`MAX_ENTRIES=10`,
/// `NetFinder.h`) -- reused here as the scoped-down single-broadcast poll
/// count. See the module doc and [`AmptekUdpTransport::discover_broadcast`].
const MAX_ENTRIES: usize = 10;

/// `SendPacketInet`'s iteration cap ("there can be up to 48 packets",
/// `DppSocket.cpp:493`).
const MAX_RECV_ITERATIONS: usize = 50;

pub struct AmptekUdpTransport {
    socket: UdpSocket,
    /// The DP5 command port every send targets. Always
    /// [`DP5_COMMAND_PORT`] in production ([`Self::bind`]); tests point
    /// this at a loopback responder's ephemeral port instead of binding
    /// the real well-known port, which would conflict across parallel
    /// test runs.
    command_port: u16,
}

impl AmptekUdpTransport {
    /// Bind an ephemeral local UDP socket with broadcast sends enabled
    /// (`CDppSocket`'s ctor binds `INADDR_ANY:0`, `DppSocket.cpp:36-64`;
    /// `SO_BROADCAST` is set per-send in C's `BroadCastSendTo`,
    /// `DppSocket.cpp:128-136` -- enabled once here instead, since this
    /// transport only ever broadcasts on the NetFinder discovery path).
    pub fn bind() -> io::Result<Self> {
        Self::bind_on_command_port(DP5_COMMAND_PORT)
    }

    fn bind_on_command_port(command_port: u16) -> io::Result<Self> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
        socket.set_broadcast(true)?;
        Ok(AmptekUdpTransport {
            socket,
            command_port,
        })
    }

    /// `CDppSocket::UDPSendTo` + `UDPRecvFrom` as used by
    /// `doAmptekNetFinderPacket` (`ConsoleHelper.cpp:199-261`): send the
    /// fixed NetFinder direct-query request to `target:10001`, sleep
    /// 50 ms (`Sleep(50)`, `ConsoleHelper.cpp:222`), then wait up to
    /// [`READ_TIMEOUT`] for exactly one response. Returns the address the
    /// device reported (which the caller compares against `target`) or
    /// `None` if nothing valid arrived -- matches
    /// `doAmptekNetFinderPacket`'s "one attempt" shape; the 3x retry loop
    /// lives in [`Self::connect_direct`].
    fn connect_direct_attempt(&self, target: Ipv4Addr) -> io::Result<Option<Ipv4Addr>> {
        let request = net_finder::build_direct_request();
        self.socket.send_to(&request, (target, self.command_port))?;
        std::thread::sleep(Duration::from_millis(50));
        self.socket.set_read_timeout(Some(READ_TIMEOUT))?;
        let mut buf = [0u8; RECV_CHUNK_SIZE];
        match self.socket.recv_from(&mut buf) {
            Ok((n, _from)) => Ok(net_finder::parse_direct_response(&buf[..n])),
            Err(e)
                if e.kind() == io::ErrorKind::WouldBlock || e.kind() == io::ErrorKind::TimedOut =>
            {
                Ok(None)
            }
            Err(e) => Err(e),
        }
    }

    /// `CConsoleHelper::DppSocket_Connect_Direct_DPP`
    /// (`ConsoleHelper.cpp:347-374`): up to 3 attempts, 1 s apart, confirmed
    /// when the device's own reported address equals `target`.
    pub fn connect_direct(&self, target: Ipv4Addr) -> io::Result<bool> {
        for attempt in 0..3 {
            if attempt > 0 {
                std::thread::sleep(Duration::from_secs(1));
            }
            if self.connect_direct_attempt(target)? == Some(target) {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// `CConsoleHelper::doNetFinderBroadcast` (`ConsoleHelper.cpp:127-178`),
    /// scoped down to a single global broadcast rather than C's
    /// per-interface enumeration (`osiSockDiscoverBroadcastAddresses`) --
    /// see the crate-level feasibility note in `lib.rs` for why: no
    /// broadcast-interface-enumeration crate is in this workspace, and
    /// the underlying NetFinder wire protocol (nonce, packet shape) is
    /// ported faithfully regardless of how many interfaces it's sent on.
    /// `broadcast_addr` is `255.255.255.255:3040` in production; tests
    /// pass a loopback unicast address to exercise the same
    /// send/collect logic without requiring broadcast permissions.
    pub fn discover_broadcast(&self, broadcast_addr: SocketAddrV4) -> io::Result<Vec<Ipv4Addr>> {
        let rand = net_finder::create_rand();
        let request = net_finder::build_broadcast_request(rand);
        self.socket.send_to(&request, broadcast_addr)?;
        std::thread::sleep(Duration::from_millis(100));

        self.socket.set_read_timeout(Some(READ_TIMEOUT))?;
        let mut found = Vec::new();
        for _ in 0..MAX_ENTRIES {
            let mut buf = [0u8; RECV_CHUNK_SIZE];
            match self.socket.recv_from(&mut buf) {
                Ok((n, SocketAddr::V4(from)))
                    if net_finder::have_netfinder_packet(&buf[..n], rand) =>
                {
                    found.push(*from.ip());
                }
                Ok(_) => continue,
                Err(e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    break;
                }
                Err(e) => return Err(e),
            }
        }
        Ok(found)
    }

    /// `CDppSocket::SendPacketInet` (`DppSocket.cpp:452-506`): send
    /// `packet` to `target:10001`, sleep 50 ms, then accumulate up to
    /// [`MAX_RESPONSE_SIZE`] bytes across up to [`MAX_RECV_ITERATIONS`]
    /// receives, stopping early once exactly `expected_size` bytes have
    /// arrived (`iTotal == iRequestedSize`) or a receive times out
    /// (`iSize <= 0`). Errors (not "no response") only for actual socket
    /// I/O failures; "device never replied" is `Ok(Vec::new())`, mirrored
    /// from C's `success = iTotal; return iTotal > 0`.
    pub fn send_packet_inet(
        &self,
        target: Ipv4Addr,
        packet: &[u8],
        expected_size: usize,
    ) -> io::Result<Vec<u8>> {
        self.socket.send_to(packet, (target, self.command_port))?;
        std::thread::sleep(Duration::from_millis(50));
        self.socket.set_read_timeout(Some(READ_TIMEOUT))?;

        let mut total = Vec::new();
        for _ in 0..MAX_RECV_ITERATIONS {
            let mut chunk = [0u8; RECV_CHUNK_SIZE];
            match self.socket.recv_from(&mut chunk) {
                Ok((n, _from)) if n > 0 => {
                    if total.len() + n > MAX_RESPONSE_SIZE {
                        break;
                    }
                    total.extend_from_slice(&chunk[..n]);
                    if total.len() == expected_size {
                        break;
                    }
                }
                Ok(_) => break,
                Err(e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    break;
                }
                Err(e) => return Err(e),
            }
        }
        Ok(total)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol;

    const LOOPBACK: Ipv4Addr = Ipv4Addr::LOCALHOST;

    /// Spawn a UDP responder bound to an ephemeral port and hand back
    /// both a transport wired to send to that port (standing in for
    /// [`DP5_COMMAND_PORT`], which parallel test runs can't safely bind)
    /// and a join handle for the responder thread.
    fn transport_with_responder<F>(respond: F) -> (AmptekUdpTransport, std::thread::JoinHandle<()>)
    where
        F: FnOnce(&[u8], SocketAddr) -> Vec<u8> + Send + 'static,
    {
        let responder = UdpSocket::bind((LOOPBACK, 0)).unwrap();
        let port = responder.local_addr().unwrap().port();
        let handle = std::thread::spawn(move || {
            let mut buf = [0u8; 1024];
            let (n, from) = responder.recv_from(&mut buf).unwrap();
            let reply = respond(&buf[..n], from);
            responder.send_to(&reply, from).unwrap();
        });
        (
            AmptekUdpTransport::bind_on_command_port(port).unwrap(),
            handle,
        )
    }

    #[test]
    fn connect_direct_confirms_on_matching_response() {
        let mut data = vec![0u8; 24];
        data[20..24].copy_from_slice(&[127, 0, 0, 1]);
        let (transport, handle) = transport_with_responder(move |_req, _from| {
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
            raw
        });

        assert!(transport.connect_direct(LOOPBACK).unwrap());
        handle.join().unwrap();
    }

    #[test]
    fn connect_direct_rejects_mismatched_response() {
        let mut data = vec![0u8; 24];
        data[20..24].copy_from_slice(&[10, 0, 0, 99]); // a different device
        let (transport, handle) = transport_with_responder(move |_req, _from| {
            let mut raw = vec![
                protocol::SYNC1,
                protocol::SYNC2,
                0x82,
                0x08,
                0x00,
                data.len() as u8,
            ];
            raw.extend_from_slice(&data);
            raw.extend_from_slice(&[0, 0]);
            raw
        });

        assert_eq!(
            transport.connect_direct_attempt(LOOPBACK).unwrap(),
            Some(Ipv4Addr::new(10, 0, 0, 99))
        );
        handle.join().unwrap();
    }

    #[test]
    fn connect_direct_attempt_times_out_with_no_responder() {
        let unused = UdpSocket::bind((LOOPBACK, 0)).unwrap();
        let dead_port = unused.local_addr().unwrap().port();
        drop(unused);
        let transport = AmptekUdpTransport::bind_on_command_port(dead_port).unwrap();
        assert_eq!(transport.connect_direct_attempt(LOOPBACK).unwrap(), None);
    }

    #[test]
    fn discover_broadcast_collects_valid_responses() {
        let responder = UdpSocket::bind((LOOPBACK, 0)).unwrap();
        let responder_port = responder.local_addr().unwrap().port();
        let handle = std::thread::spawn(move || {
            let mut buf = [0u8; 1024];
            let (n, from) = responder.recv_from(&mut buf).unwrap();
            assert_eq!(n, 6); // [0x00, 0x00, rand_hi, rand_lo, 0xF4, 0xFA]
            let mut reply = vec![0x01, 0x00, buf[2], buf[3]];
            reply.extend(std::iter::repeat_n(0u8, 28));
            responder.send_to(&reply, from).unwrap();
        });

        let transport = AmptekUdpTransport::bind().unwrap();
        let broadcast_addr = SocketAddrV4::new(LOOPBACK, responder_port);
        let found = transport.discover_broadcast(broadcast_addr).unwrap();
        handle.join().unwrap();
        assert_eq!(found, vec![LOOPBACK]);
    }

    #[test]
    fn discover_broadcast_ignores_unrecognized_traffic() {
        let responder = UdpSocket::bind((LOOPBACK, 0)).unwrap();
        let responder_port = responder.local_addr().unwrap().port();
        let handle = std::thread::spawn(move || {
            let mut buf = [0u8; 1024];
            let (_, from) = responder.recv_from(&mut buf).unwrap();
            responder.send_to(&[0x42u8; 40], from).unwrap(); // garbage, not a NetFinder reply
        });

        let transport = AmptekUdpTransport::bind().unwrap();
        let broadcast_addr = SocketAddrV4::new(LOOPBACK, responder_port);
        let found = transport.discover_broadcast(broadcast_addr).unwrap();
        handle.join().unwrap();
        assert!(found.is_empty());
    }

    #[test]
    fn send_packet_inet_accumulates_until_expected_size() {
        let (transport, handle) = transport_with_responder(|_req, _from| vec![0xABu8; 72]);
        let packet = protocol::build_command(protocol::TransmitPacketType::SendStatus);
        let result = transport.send_packet_inet(LOOPBACK, &packet, 72).unwrap();
        assert_eq!(result, vec![0xABu8; 72]);
        handle.join().unwrap();
    }

    #[test]
    fn send_packet_inet_returns_empty_when_unreachable() {
        let unused = UdpSocket::bind((LOOPBACK, 0)).unwrap();
        let dead_port = unused.local_addr().unwrap().port();
        drop(unused);
        let transport = AmptekUdpTransport::bind_on_command_port(dead_port).unwrap();
        let packet = protocol::build_command(protocol::TransmitPacketType::SendStatus);
        let result = transport.send_packet_inet(LOOPBACK, &packet, 72).unwrap();
        assert!(result.is_empty());
    }
}
