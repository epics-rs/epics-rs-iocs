//! Test-only stub DP5 device, for boot-testing `mca-amptek-ioc` without
//! real Amptek hardware (none is available in this sandbox, and no
//! vendor-provided simulator/loopback exists for the DP5 wire protocol).
//! **Not part of the port** -- this is a hand-written test double playing
//! the wire-protocol role the real device fills, built entirely from the
//! already-ported-and-unit-tested [`mca_amptek::protocol`] framing so it
//! never invents any packet shape not already verified against
//! `DP5Protocol.h`/`ParsePacket.cpp`.
//!
//! Answers, on UDP port 10001 (`AmptekUdpTransport::DP5_COMMAND_PORT`),
//! exactly the request shapes `drvAmptek.cpp` sends (see `driver.rs`'s
//! module doc): the NetFinder direct-query connect handshake, `SendStatus`,
//! `SendSpectrumStatus`, `SendClearSpectrumStatus`, `EnableMcaMcs`,
//! `DisableMcaMcs`, the ASCII config-set packet, and the full
//! config-readback query. Every other request is ignored (dropped, no
//! reply) -- `AmptekDriver` treats a non-response as a plain send/receive
//! failure, already exercised by this crate's own transport tests.
//!
//! Usage: `cargo run -p mca-amptek --example stub_dp5`

use std::net::{SocketAddr, UdpSocket};

use mca_amptek::config::{ConfigFields, format_configuration};
use mca_amptek::protocol::{SYNC1, SYNC2, pack_out};

const DP5_COMMAND_PORT: u16 = 10001;

// PID1/PID2 byte values below are copied verbatim from the private
// `pid1`/`pid2_rcv`/`pid2_netfinder` submodules in
// `mca-amptek/src/protocol.rs` (not reachable from here since those
// modules aren't `pub`) -- each is cited against the exact protocol.rs
// constant it mirrors, not independently guessed.
mod pid {
    // protocol.rs `mod pid1`
    pub const REQ_STATUS: u8 = 0x01;
    pub const REQ_SPECTRUM: u8 = 0x02;
    pub const REQ_SCOPE_MISC: u8 = 0x03;
    pub const REQ_CONFIG: u8 = 0x20;
    pub const VENDOR_REQ: u8 = 0xF0;
    pub const RCV_STATUS: u8 = 0x80;
    pub const RCV_SPECTRUM: u8 = 0x81;
    pub const RCV_SCOPE_MISC: u8 = 0x82;
    pub const ACK: u8 = 0xFF;

    // protocol.rs `mod pid2_netfinder`
    pub const NETFINDER_READBACK_REQUEST: u8 = 0x07;
    pub const NETFINDER_READBACK_RESPONSE: u8 = 0x08;

    // protocol.rs `mod pid2_rcv`
    pub const DP4_STYLE_STATUS: u8 = 0x01;
    pub const CONFIG_READBACK: u8 = 0x07;

    // protocol.rs `build_command`'s request-side PID2 values.
    pub const REQ_SPECTRUM_STATUS_PID2: u8 = 0x03;
    pub const REQ_CLEAR_SPECTRUM_PID2: u8 = 0x04;
    pub const REQ_ENABLE_MCA_MCS_PID2: u8 = 0x02;
    pub const REQ_DISABLE_MCA_MCS_PID2: u8 = 0x03;
    pub const REQ_CONFIG_SET_PID2: u8 = 0x02;
    pub const REQ_CONFIG_READ_PID2: u8 = 0x03;

    pub const ACK_OK: u8 = 0x00;
}

/// A canonical 64-byte DP4-format status block, hand-encoded as the exact
/// byte-for-byte inverse of `mca_amptek::status::process_status`'s decode
/// formulas (`DP5Status.cpp`) -- device_id=0 (DP5), firmware/fpga 6.07,
/// MCA_EN set, everything else zeroed/quiet.
fn canned_status_block() -> [u8; 64] {
    let mut raw = [0u8; 64];
    raw[24] = 0x67; // firmware 6.07 (BYTEVersionToString: major<<4|minor)
    raw[25] = 0x67; // fpga 6.07
    raw[29] = 0x00; // serial number valid (RAW[29] < 128)
    raw[26..30].copy_from_slice(&12345u32.to_le_bytes());
    raw[30] = 0x00; // HV sign byte: positive branch
    raw[31] = 0x00; // HV magnitude
    raw[32] = 0x00;
    raw[33] = 200; // det_temp = (200 + 0*256) * 0.1 = 20.0 C
    raw[34] = 25; // dp5_temp = 25 C
    raw[35] = 0x20; // bit5 = MCA_EN
    raw[39] = 0; // device_id = DP5
    raw[49] = 0; // dpp_eco
    raw
}

fn status_response() -> Vec<u8> {
    pack_out(
        pid::RCV_STATUS,
        pid::DP4_STYLE_STATUS,
        &canned_status_block(),
    )
}

/// A 256-channel spectrum (`RCV_SPECTRUM` pid2=0x01, odd -> no trailing
/// status per `ReceivedPacket::Spectrum`'s doc), channel `i` = `i` as a
/// 3-byte little-endian value -- deterministic, easy to eyeball over CA.
fn spectrum_response() -> Vec<u8> {
    let mut data = Vec::with_capacity(256 * 3);
    for i in 0u32..256 {
        data.push((i & 0xFF) as u8);
        data.push(((i >> 8) & 0xFF) as u8);
        data.push(((i >> 16) & 0xFF) as u8);
    }
    pack_out(pid::RCV_SPECTRUM, 0x01, &data)
}

fn ack_ok() -> Vec<u8> {
    pack_out(pid::ACK, pid::ACK_OK, &[])
}

/// Plausible defaults matching `Amptek.db`'s own `PINI` field values, so
/// the readback round trip after boot's PINI writes settles on the same
/// numbers the records themselves just sent.
fn config_readback_response() -> Vec<u8> {
    let fields = ConfigFields {
        clock: 0,
        input_polarity: 0,
        peaking_time: 10.0,
        fast_peaking_time: 0,
        flat_top_time: 0.0,
        gain: 1.0,
        slow_threshold: 0.0,
        fast_threshold: 0.0,
        num_channels: 8192,
        gate: 0,
        preset_real_time: 0.0,
        preset_live_time: 0.0,
        preset_counts: 0.0,
        preset_low_channel: 0,
        preset_high_channel: 8191,
        mca_source: 0,
        pur_enable: 1,
        set_high_voltage: 0,
        set_det_temp: 0.0,
        mcs_low_channel: 0,
        mcs_high_channel: 100,
        dwell_time: 0.0,
        aux_out1: 6,
        aux_out2: 0,
        aux_out34: 1,
        connect1: 0,
        connect2: 0,
        sca_output_width: 0,
    };
    let cfg = format_configuration(&fields);
    pack_out(pid::RCV_SCOPE_MISC, pid::CONFIG_READBACK, cfg.as_bytes())
}

/// `doAmptekNetFinderPacket`'s direct-query response
/// (`CNetFinder::AddEntry`, `NetFinder.cpp:276-342`): DATA[20..24] carries
/// the responding device's own IP, which the driver compares against the
/// address it queried -- echo back the peer's source address.
fn netfinder_direct_response(from: SocketAddr) -> Option<Vec<u8>> {
    let SocketAddr::V4(from_v4) = from else {
        return None;
    };
    let mut data = vec![0u8; 24];
    data[20..24].copy_from_slice(&from_v4.ip().octets());
    Some(pack_out(
        pid::RCV_SCOPE_MISC,
        pid::NETFINDER_READBACK_RESPONSE,
        &data,
    ))
}

fn handle_request(buf: &[u8], from: SocketAddr) -> Option<Vec<u8>> {
    if buf.len() < 4 || buf[0] != SYNC1 || buf[1] != SYNC2 {
        return None;
    }
    let (pid1, pid2) = (buf[2], buf[3]);
    match (pid1, pid2) {
        (pid::REQ_SCOPE_MISC, pid::NETFINDER_READBACK_REQUEST) => netfinder_direct_response(from),
        (pid::REQ_STATUS, pid::DP4_STYLE_STATUS) => Some(status_response()),
        (pid::REQ_SPECTRUM, pid::REQ_SPECTRUM_STATUS_PID2) => Some(spectrum_response()),
        (pid::REQ_SPECTRUM, pid::REQ_CLEAR_SPECTRUM_PID2) => Some(ack_ok()),
        (pid::VENDOR_REQ, pid::REQ_ENABLE_MCA_MCS_PID2) => Some(ack_ok()),
        (pid::VENDOR_REQ, pid::REQ_DISABLE_MCA_MCS_PID2) => Some(ack_ok()),
        (pid::REQ_CONFIG, pid::REQ_CONFIG_SET_PID2) => Some(ack_ok()),
        (pid::REQ_CONFIG, pid::REQ_CONFIG_READ_PID2) => Some(config_readback_response()),
        _ => {
            eprintln!("stub_dp5: unhandled request pid1={pid1:#04x} pid2={pid2:#04x}");
            None
        }
    }
}

fn main() -> std::io::Result<()> {
    let socket = UdpSocket::bind(("127.0.0.1", DP5_COMMAND_PORT))?;
    eprintln!("stub_dp5: listening on 127.0.0.1:{DP5_COMMAND_PORT}");
    let mut buf = [0u8; 2048];
    loop {
        let (n, from) = socket.recv_from(&mut buf)?;
        if let Some(reply) = handle_request(&buf[..n], from) {
            socket.send_to(&reply, from)?;
        }
    }
}
