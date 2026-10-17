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
//! to that same duration is behaviorally equivalent for every `UDPRecvFrom` caller
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

use epics_rs::base::net::iface_v4;
use epics_rs::ca::hostname::ip_addr_to_a;

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

/// `doNetFinderBroadcast`'s receive budget (`MAX_ENTRIES=10`, `NetFinder.h`).
const MAX_ENTRIES: usize = 10;

/// `SendPacketInet`'s iteration cap ("there can be up to 48 packets",
/// `DppSocket.cpp:493`).
const MAX_RECV_ITERATIONS: usize = 50;

/// Whether a UDP discovery/probe `recv` error means "nothing valid
/// replied", i.e. the same outcome as a timeout.
///
/// On Unix an unanswered `recv_from` simply times out (`WouldBlock` /
/// `TimedOut`). On Windows the OS defers the ICMP error from an
/// unanswered datagram to the *next* `recv` rather than timing out, and
/// which ICMP shape it is depends on why nothing replied:
/// - a live host with no listener on the port → port-unreachable →
///   `WSAECONNRESET` (`ConnectionReset`; `ConnectionRefused` on a
///   connected socket);
/// - no host / no route at the target address at all (the common case
///   when probing an IP where no DP5 is currently plugged in) →
///   host/network-unreachable or network-down → `HostUnreachable` /
///   `NetworkUnreachable` / `NetworkDown`.
///
/// For NetFinder discovery and the direct-connect probe every one of
/// these means the same thing — no device answered — so they are treated
/// identically to a timeout.
fn is_no_response(kind: io::ErrorKind) -> bool {
    matches!(
        kind,
        io::ErrorKind::WouldBlock
            | io::ErrorKind::TimedOut
            | io::ErrorKind::ConnectionReset
            | io::ErrorKind::ConnectionRefused
            | io::ErrorKind::HostUnreachable
            | io::ErrorKind::NetworkUnreachable
            | io::ErrorKind::NetworkDown
    )
}

/// `osiSockDiscoverBroadcastAddresses` with a wildcard match address, as
/// the `CDppSocket` ctor calls it (`DppSocket.cpp:25-27`): one entry per
/// eligible interface, not deduplicated, at the NetFinder port.
fn interface_broadcast_list() -> Vec<SocketAddrV4> {
    iface_v4::enumerate()
        .unwrap_or_default()
        .iter()
        .filter_map(|iface| iface.search_destination())
        .map(|ip| SocketAddrV4::new(ip, NETFINDER_BROADCAST_PORT))
        .collect()
}

/// `ipAddrToA` of a broadcast list node, whose port is still 0
/// (`DppSocket.cpp:34,148`).
fn ip_addr_to_a_port0(ip: Ipv4Addr) -> String {
    let addr = SocketAddr::V4(SocketAddrV4::new(ip, 0));
    String::from_utf8_lossy(&ip_addr_to_a(addr)).into_owned()
}

/// A send and one `UDPRecvFrom` on C's invalid descriptor: the failed send
/// is followed by the 50 ms settle, and the failed `recvfrom` by the
/// timeout sleep before its failed retry (`DppSocket.cpp:186-200`).
fn no_socket_exchange() {
    std::thread::sleep(Duration::from_millis(50) + READ_TIMEOUT);
}

pub struct AmptekUdpTransport {
    /// `CDppSocket::m_hDppSocket`; `None` when `epicsSocketCreate` failed,
    /// after which C keeps the instance and every send/receive on the
    /// invalid descriptor fails, so nothing ever replies.
    socket: Option<UdpSocket>,
    /// The DP5 command port every send targets. Always
    /// [`DP5_COMMAND_PORT`] in production ([`Self::bind`]); tests point
    /// this at a loopback responder's ephemeral port instead of binding
    /// the real well-known port, which would conflict across parallel
    /// test runs.
    command_port: u16,
    /// `CDppSocket::broadcastList`: one NetFinder destination per
    /// interface, as `osiSockDiscoverBroadcastAddresses` lists them.
    broadcast_list: Vec<SocketAddrV4>,
}

impl AmptekUdpTransport {
    /// The `CDppSocket` ctor (`DppSocket.cpp:10-64`): an ephemeral
    /// `INADDR_ANY:0` UDP socket with broadcast sends enabled (C sets
    /// `SO_BROADCAST` per send in `SendNetFinderBroadCast`; enabled once
    /// here, since only the NetFinder path broadcasts). A socket failure is
    /// not fatal, as in C: the line is printed and the transport is built
    /// without a socket.
    pub fn bind() -> Self {
        Self::open(UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)))
    }

    pub(crate) fn open(socket: io::Result<UdpSocket>) -> Self {
        let Ok(socket) = socket else {
            println!("epicsSocketCreate failed");
            return Self::with_socket(None, DP5_COMMAND_PORT);
        };
        let mut transport = Self::with_socket(Some(socket), DP5_COMMAND_PORT);
        let list = interface_broadcast_list();
        println!("Found {} broadcast interfaces ...", list.len());
        println!("Broadcast list ...");
        for dest in &list {
            println!("{}", ip_addr_to_a_port0(*dest.ip()));
        }
        transport.broadcast_list = list;
        transport
    }

    #[cfg(test)]
    pub(crate) fn bind_on_command_port(command_port: u16) -> io::Result<Self> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
        Ok(Self::with_socket(Some(socket), command_port))
    }

    fn with_socket(socket: Option<UdpSocket>, command_port: u16) -> Self {
        // A refused SO_BROADCAST fails each broadcast send, whose result C
        // ignores.
        if let Some(socket) = &socket {
            let _ = socket.set_broadcast(true);
        }
        AmptekUdpTransport {
            socket,
            command_port,
            broadcast_list: Vec::new(),
        }
    }

    /// `CConsoleHelper::doAmptekNetFinderPacket` (`ConsoleHelper.cpp:199-261`):
    /// send the NetFinder direct query to `target:10001`, sleep 50 ms, read
    /// one reply. Returns the address the reply carries, `None` when no
    /// acceptable reply arrived (C's `iNumDevices == 0`).
    fn amptek_net_finder_packet(&self, target: Ipv4Addr) -> io::Result<Option<Ipv4Addr>> {
        let request = net_finder::build_direct_request();
        let Some(socket) = &self.socket else {
            no_socket_exchange();
            return Ok(None);
        };
        socket.send_to(&request, (target, self.command_port))?;
        std::thread::sleep(Duration::from_millis(50));
        socket.set_read_timeout(Some(READ_TIMEOUT))?;
        let mut buf = [0u8; RECV_CHUNK_SIZE];
        let n = match socket.recv_from(&mut buf) {
            Ok((n, _from)) => n,
            Err(e) if is_no_response(e.kind()) => 0,
            Err(e) => return Err(e),
        };
        Ok(net_finder::parse_direct_response(&mut buf, n))
    }

    /// `CConsoleHelper::DppSocket_Connect_Direct_DPP`
    /// (`ConsoleHelper.cpp:347-374`): up to 3 queries, 1 s apart, while no
    /// reply is accepted, then one address comparison against the last
    /// accepted reply (`ulNetFinderAddr`, reset to 0 first). Returns
    /// `(connected, NumDevices)`.
    pub fn connect_direct(&self, target: Ipv4Addr) -> io::Result<(bool, usize)> {
        let mut reported = None;
        for attempt in 0..3 {
            if attempt > 0 {
                std::thread::sleep(Duration::from_secs(1));
            }
            reported = self.amptek_net_finder_packet(target)?;
            if reported.is_some() {
                break;
            }
        }
        let num_devices = usize::from(reported.is_some());
        Ok((
            reported.unwrap_or(Ipv4Addr::UNSPECIFIED) == target,
            num_devices,
        ))
    }

    /// `CDppSocket::UDPRecvFromNfAddr` (`DppSocket.cpp:217-243`): one
    /// non-blocking receive into `buf`, 0 on any error. The address is
    /// the one the reply carries at bytes 20..24 when more than 67 bytes
    /// arrived, otherwise `None` (C's empty `szDPP`).
    fn recv_from_nf_addr(&self, buf: &mut [u8; RECV_CHUNK_SIZE]) -> (usize, Option<Ipv4Addr>) {
        let Some(socket) = &self.socket else {
            return (0, None);
        };
        if socket.set_nonblocking(true).is_err() {
            return (0, None);
        }
        let n = socket.recv_from(buf).map_or(0, |(n, _from)| n);
        let _ = socket.set_nonblocking(false);
        let addr = (n > 67).then(|| Ipv4Addr::new(buf[20], buf[21], buf[22], buf[23]));
        (n, addr)
    }

    /// `CConsoleHelper::doNetFinderBroadcast` (`ConsoleHelper.cpp:127-178`):
    /// one broadcast per interface in [`Self::broadcast_list`], each with a
    /// new nonce, collecting every accepted reply. As in C the receive
    /// buffer and the `iNetFinderLoops` budget are shared by all
    /// interfaces, so after the first interface times out each further one
    /// reads a single reply. C's `szNetAddress[10]` overflows past 10
    /// entries; the list is unbounded here.
    fn net_finder_broadcast(&self) -> Vec<Option<Ipv4Addr>> {
        let mut buf = [0u8; RECV_CHUNK_SIZE];
        let mut found = Vec::new();
        let mut loops = 0;
        for dest in &self.broadcast_list {
            let rand = net_finder::create_rand();
            println!(
                "interface broadcast address {}",
                ip_addr_to_a_port0(*dest.ip())
            );
            // C ignores the SendNetFinderBroadCast result.
            if let Some(socket) = &self.socket {
                let _ = socket.send_to(&net_finder::build_broadcast_request(rand), dest);
            }
            std::thread::sleep(Duration::from_millis(100));
            println!("Searching for Amptek modules on network:");
            loop {
                let (n, addr) = self.recv_from_nf_addr(&mut buf);
                if n > 0 {
                    if net_finder::have_netfinder_packet(&buf, rand, n) {
                        let shown = addr.map(|a| a.to_string()).unwrap_or_default();
                        println!(
                            "  Address: {shown}  Port: {NETFINDER_BROADCAST_PORT}   bytes:{n}"
                        );
                        found.push(addr);
                    }
                } else {
                    loops += MAX_ENTRIES;
                }
                loops += 1;
                if n == 0 || loops >= MAX_ENTRIES {
                    break;
                }
            }
        }
        println!("Found {} Amptek units", found.len());
        found
    }

    /// `CConsoleHelper::DppSocket_Connect_Default_DPP`
    /// (`ConsoleHelper.cpp:300-342`): up to 3 searches, 1 s apart, while
    /// none finds a device, then match `target` against the addresses
    /// found (an empty address compares as `inet_addr("")`, i.e.
    /// `INADDR_NONE`). On a match C sleeps 1 s and sends one direct query,
    /// whose result it ignores. Returns `(connected, NumDevices)`.
    pub fn connect_default(&self, target: Ipv4Addr) -> io::Result<(bool, usize)> {
        let mut found = self.net_finder_broadcast();
        for _ in 0..2 {
            if found.is_empty() {
                std::thread::sleep(Duration::from_secs(1));
                found = self.net_finder_broadcast();
            }
        }
        let connected = found
            .iter()
            .any(|addr| addr.unwrap_or(Ipv4Addr::BROADCAST) == target);
        if connected {
            std::thread::sleep(Duration::from_secs(1));
            self.amptek_net_finder_packet(target)?;
        }
        Ok((connected, found.len()))
    }

    /// `CDppSocket::SendPacketInet` (`DppSocket.cpp:452-506`): send
    /// `packet` to `target:10001`, sleep 50 ms, then accumulate up to
    /// `MAX_RESPONSE_SIZE` bytes across up to `MAX_RECV_ITERATIONS`
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
        let Some(socket) = &self.socket else {
            no_socket_exchange();
            return Ok(Vec::new());
        };
        socket.send_to(packet, (target, self.command_port))?;
        std::thread::sleep(Duration::from_millis(50));
        socket.set_read_timeout(Some(READ_TIMEOUT))?;

        let mut total = Vec::new();
        for _ in 0..MAX_RECV_ITERATIONS {
            let mut chunk = [0u8; RECV_CHUNK_SIZE];
            match socket.recv_from(&mut chunk) {
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
                Err(e) if is_no_response(e.kind()) => break,
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
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};

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

    fn direct_reply(ip: [u8; 4]) -> Vec<u8> {
        let mut raw = vec![protocol::SYNC1, protocol::SYNC2, 0x82, 0x08, 0x00, 24];
        raw.extend_from_slice(&[0u8; 20]);
        raw.extend_from_slice(&ip);
        raw.extend_from_slice(&[0, 0]); // unverified trailer
        raw
    }

    #[test]
    fn connect_direct_confirms_on_matching_response() {
        let (transport, handle) =
            transport_with_responder(|_req, _from| direct_reply([127, 0, 0, 1]));
        assert_eq!(transport.connect_direct(LOOPBACK).unwrap(), (true, 1));
        handle.join().unwrap();
    }

    /// A mismatched reply ends the retry loop: only silence retries
    /// (`ConsoleHelper.cpp:356-364`).
    #[test]
    fn connect_direct_does_not_retry_past_a_mismatched_reply() {
        let (transport, handle) =
            transport_with_responder(|_req, _from| direct_reply([10, 0, 0, 99]));
        assert_eq!(transport.connect_direct(LOOPBACK).unwrap(), (false, 1));
        handle.join().unwrap();
    }

    #[test]
    fn amptek_net_finder_packet_times_out_with_no_responder() {
        let unused = UdpSocket::bind((LOOPBACK, 0)).unwrap();
        let dead_port = unused.local_addr().unwrap().port();
        drop(unused);
        let transport = AmptekUdpTransport::bind_on_command_port(dead_port).unwrap();
        assert_eq!(transport.amptek_net_finder_packet(LOOPBACK).unwrap(), None);
    }

    /// A NetFinder broadcast reply of `len` bytes for the request `req`,
    /// carrying `ip` at bytes 20..24.
    fn broadcast_reply(req: &[u8], len: usize, ip: [u8; 4]) -> Vec<u8> {
        let mut reply = vec![0u8; len];
        reply[..4].copy_from_slice(&[0x01, 0x00, req[2], req[3]]);
        reply[20..24].copy_from_slice(&ip);
        reply
    }

    /// A transport whose broadcast list is `n` copies of a loopback
    /// responder that answers each request with `replies(req)`.
    fn transport_with_netfinder<F>(n: usize, replies: F) -> (AmptekUdpTransport, Arc<AtomicUsize>)
    where
        F: Fn(&[u8]) -> Vec<Vec<u8>> + Send + 'static,
    {
        let responder = UdpSocket::bind((LOOPBACK, 0)).unwrap();
        responder
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let port = responder.local_addr().unwrap().port();
        let requests = Arc::new(AtomicUsize::new(0));
        let seen = requests.clone();
        std::thread::spawn(move || {
            let mut buf = [0u8; 1024];
            while let Ok((n, from)) = responder.recv_from(&mut buf) {
                seen.fetch_add(1, Ordering::Relaxed);
                for reply in replies(&buf[..n]) {
                    responder.send_to(&reply, from).unwrap();
                }
            }
        });
        let mut transport = AmptekUdpTransport::bind_on_command_port(port).unwrap();
        transport.broadcast_list = vec![SocketAddrV4::new(LOOPBACK, port); n];
        (transport, requests)
    }

    /// The address comes from bytes 20..24 only past 67 bytes, otherwise it
    /// is empty (`DppSocket.cpp:228-235`).
    #[test]
    fn net_finder_broadcast_takes_the_address_from_the_payload() {
        let (transport, _) = transport_with_netfinder(1, |req| {
            vec![
                broadcast_reply(req, 68, [10, 1, 2, 3]),
                broadcast_reply(req, 67, [10, 1, 2, 4]),
            ]
        });
        assert_eq!(
            transport.net_finder_broadcast(),
            vec![Some(Ipv4Addr::new(10, 1, 2, 3)), None]
        );
    }

    /// One broadcast per interface; the loop budget is shared, so the
    /// second interface reads one reply (`ConsoleHelper.cpp:134-176`).
    #[test]
    fn net_finder_broadcast_sends_per_interface_with_a_shared_budget() {
        let (transport, requests) = transport_with_netfinder(2, |req| {
            vec![
                broadcast_reply(req, 68, [10, 1, 2, 3]),
                broadcast_reply(req, 68, [10, 1, 2, 4]),
            ]
        });
        let found = transport.net_finder_broadcast();
        assert_eq!(requests.load(Ordering::Relaxed), 2);
        assert_eq!(found.len(), 3);
    }

    #[test]
    fn connect_default_matches_and_sends_the_direct_query() {
        let (transport, requests) = transport_with_netfinder(1, |req| {
            if req.len() == 6 {
                vec![broadcast_reply(req, 68, [127, 0, 0, 1])]
            } else {
                vec![]
            }
        });
        assert_eq!(transport.connect_default(LOOPBACK).unwrap(), (true, 1));
        assert_eq!(requests.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn connect_default_rejects_when_target_is_not_among_responses() {
        let (transport, _) =
            transport_with_netfinder(1, |req| vec![broadcast_reply(req, 68, [10, 0, 0, 99])]);
        assert_eq!(transport.connect_default(LOOPBACK).unwrap(), (false, 1));
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
