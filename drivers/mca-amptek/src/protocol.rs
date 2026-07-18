//! Amptek DP5 binary wire protocol: packet framing, checksum, and the
//! request-packet builders `drvAmptek.cpp` actually issues, ported from
//! `DP5Protocol.h`, `ParsePacket.cpp` and `SendCommand.cpp`
//! (`mcaApp/AmptekSrc`).
//!
//! # Restructuring vs. C
//! C's `TRANSMIT_PACKET_TYPE` has ~60 variants and `CSendCommand::DP5_CMD`/
//! `DP5_CMD_Config` switch on all of them; `drvAmptek.cpp` issues exactly
//! six (`XMTPT_SEND_STATUS`, `XMTPT_SEND_SPECTRUM_STATUS`,
//! `XMTPT_SEND_CLEAR_SPECTRUM_STATUS`, `XMTPT_ENABLE_MCA_MCS`,
//! `XMTPT_DISABLE_MCA_MCS`, `XMTPT_SEND_CONFIG_PACKET_EX`,
//! `XMTPT_FULL_READ_CONFIG_PACKET`) -- seven, counting both config
//! variants. [`TransmitPacketType`] carries only those (feasibility gate:
//! "port only what the driver uses").

/// `DP5Protocol.h`: `#define SYNC1_ 0xF5`.
pub const SYNC1: u8 = 0xF5;
/// `DP5Protocol.h`: `#define SYNC2_ 0xFA`.
pub const SYNC2: u8 = 0xFA;

/// `DP5Protocol.h enum PID1_TYPE` -- only the values the packets below use.
mod pid1 {
    pub const REQ_STATUS: u8 = 0x01;
    pub const REQ_SPECTRUM: u8 = 0x02;
    /// `PID1_REQ_SCOPE_MISC` (`DP5Protocol.h:101`) -- used for the
    /// NetFinder direct-query request ([`build_netfinder_direct_request`]).
    pub const REQ_SCOPE_MISC: u8 = 0x03;
    pub const REQ_CONFIG: u8 = 0x20;
    pub const VENDOR_REQ: u8 = 0xF0;
    pub const RCV_STATUS: u8 = 0x80;
    pub const RCV_SPECTRUM: u8 = 0x81;
    pub const RCV_SCOPE_MISC: u8 = 0x82;
    pub const ACK: u8 = 0xFF;
}

/// NetFinder-specific PID2 values, kept separate from [`pid2_rcv`] since
/// they live under a different PID1 namespace
/// (`PID1_REQ_SCOPE_MISC`/`PID1_RCV_SCOPE_MISC`) and are used only by
/// [`build_netfinder_direct_request`]/[`parse_netfinder_direct_response_header`]
/// (`doAmptekNetFinderPacket`, `ConsoleHelper.cpp:199-261`).
mod pid2_netfinder {
    /// `PID2_SEND_NETFINDER_READBACK` (`DP5Protocol.h:137`): the outbound
    /// direct-query request's PID2.
    pub const READBACK_REQUEST: u8 = 0x07;
    /// `RCVPT_NETFINDER_READBACK` (`DP5Protocol.h:226`): the inbound
    /// response's PID2.
    pub const READBACK_RESPONSE: u8 = 0x08;
}

/// `DP5Protocol.h`: the receive-side PID2 values [`ReceivedPacket::classify`]
/// routes on, restricted to what `CConsoleHelper::ReceiveData` (and
/// therefore `drvAmptek.cpp`) actually handles.
mod pid2_rcv {
    pub const DP4_STYLE_STATUS: u8 = 0x01;
    pub const SPECTRUM_256_LO: u8 = 0x01;
    pub const SPECTRUM_8192_HI: u8 = 0x0C;
    pub const CONFIG_READBACK: u8 = 0x07;
}

/// `DP5Protocol.h enum PID2_ACK_TYPE` -- the status byte
/// [`parse_packet_status`] can produce. `Ok` is C's `PID2_ACK_OK` (0x00);
/// C's ad-hoc "packet invalid, will be overwritten" sentinel `0xFF`
/// (`ConsoleHelper.cpp:408,457`) has no wire meaning here, so it is not a
/// variant -- callers represent "no response received" as `Err`/`None` at
/// the transport layer instead of a fake status byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AckStatus {
    Ok,
    SyncError,
    PidError,
    LenError,
    ChecksumError,
    BadParam,
    BadHexRec,
    Unrecognized,
    FpgaError,
    Cp2201NotFound,
    ScopeDataNotAvail,
    Pc5NotPresent,
    OkEthernetShareReq,
    EthernetBusy,
    I2cError,
    OkFpgaUploadAddr,
    FeatureNotFpgaSupported,
    CalDataNotPresent,
    /// Any PID2 value C's `PID2_TextToString` default-arms as "Unrecognized
    /// Error" (`ParsePacket.cpp:112-114`).
    Other(u8),
}

impl AckStatus {
    fn from_byte(b: u8) -> Self {
        match b {
            0x00 => AckStatus::Ok,
            0x01 => AckStatus::SyncError,
            0x02 => AckStatus::PidError,
            0x03 => AckStatus::LenError,
            0x04 => AckStatus::ChecksumError,
            0x05 => AckStatus::BadParam,
            0x06 => AckStatus::BadHexRec,
            0x07 => AckStatus::Unrecognized,
            0x08 => AckStatus::FpgaError,
            0x09 => AckStatus::Cp2201NotFound,
            0x0A => AckStatus::ScopeDataNotAvail,
            0x0B => AckStatus::Pc5NotPresent,
            0x0C => AckStatus::OkEthernetShareReq,
            0x0D => AckStatus::EthernetBusy,
            0x0E => AckStatus::I2cError,
            0x0F => AckStatus::OkFpgaUploadAddr,
            0x10 => AckStatus::FeatureNotFpgaSupported,
            0x11 => AckStatus::CalDataNotPresent,
            other => AckStatus::Other(other),
        }
    }
}

/// The DP5 request packets `drvAmptek.cpp` sends (`sendCommand`,
/// `sendCommandString` via `sendConfigurationFile`/`sendSCAs`/
/// `sendConfiguration`, and `readConfigurationFromHardware`). See the
/// module doc's "Restructuring vs. C" note on why this is a small subset
/// of C's `TRANSMIT_PACKET_TYPE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransmitPacketType {
    /// `XMTPT_SEND_STATUS` (`drvAmptek.cpp:331,874`).
    SendStatus,
    /// `XMTPT_SEND_SPECTRUM_STATUS` (`drvAmptek.cpp:1006`).
    SendSpectrumStatus,
    /// `XMTPT_SEND_CLEAR_SPECTRUM_STATUS` (`drvAmptek.cpp:870`).
    SendClearSpectrumStatus,
    /// `XMTPT_ENABLE_MCA_MCS` (`drvAmptek.cpp:860`).
    EnableMcaMcs,
    /// `XMTPT_DISABLE_MCA_MCS` (`drvAmptek.cpp:867,941`).
    DisableMcaMcs,
}

/// Build the fixed-shape (no ASCII payload) request packets
/// (`CSendCommand::DP5_CMD`, `SendCommand.cpp:42-265`, restricted to the
/// variants [`TransmitPacketType`] carries).
pub fn build_command(cmd: TransmitPacketType) -> Vec<u8> {
    let (pid1, pid2) = match cmd {
        TransmitPacketType::SendStatus => (pid1::REQ_STATUS, pid2_rcv::DP4_STYLE_STATUS),
        TransmitPacketType::SendSpectrumStatus => (pid1::REQ_SPECTRUM, 0x03),
        TransmitPacketType::SendClearSpectrumStatus => (pid1::REQ_SPECTRUM, 0x04),
        TransmitPacketType::EnableMcaMcs => (pid1::VENDOR_REQ, 0x02),
        TransmitPacketType::DisableMcaMcs => (pid1::VENDOR_REQ, 0x03),
    };
    pack_out(pid1, pid2, &[])
}

/// `XMTPT_SEND_CONFIG_PACKET_EX` (`SendCommand.cpp:309-322`): the ASCII
/// config string, uppercased, as the packet's DATA. `drvAmptek.cpp`'s
/// `sendCommandString` (`drvAmptek.cpp:437-477`) is the only caller.
pub fn build_config_packet_ex(config: &str) -> Vec<u8> {
    pack_out(
        pid1::REQ_CONFIG,
        0x02,
        config.to_ascii_uppercase().as_bytes(),
    )
}

/// `XMTPT_FULL_READ_CONFIG_PACKET` (`SendCommand.cpp:323-337`): a readback
/// query string (built by [`crate::ascii_cmd::create_full_readback_cmd`]),
/// uppercased, as the packet's DATA.
/// `drvAmptek.cpp::readConfigurationFromHardware` is the only caller.
pub fn build_full_read_config_packet(query: &str) -> Vec<u8> {
    pack_out(
        pid1::REQ_CONFIG,
        0x03,
        query.to_ascii_uppercase().as_bytes(),
    )
}

/// `CSendCommand::POUT_Buffer` (`SendCommand.cpp:446-470`): frame
/// `SYNC1 SYNC2 PID1 PID2 LEN_MSB LEN_LSB [DATA] CKSUM_MSB CKSUM_LSB`, with
/// the checksum `CS = (sum(header + data) ^ 0xFFFF) + 1` stored
/// big-endian. `data.len()` must fit in `u16` (the wire `LEN` field); the
/// longest caller payload is `CreateFullReadBackCmd`'s query string, which
/// is well under 512 bytes even for every field ported
/// ([`crate::ascii_cmd`]'s doc), so this asserts rather than erroring.
pub fn pack_out(pid1: u8, pid2: u8, data: &[u8]) -> Vec<u8> {
    let len = u16::try_from(data.len()).expect("DP5 command payload exceeds u16::MAX");
    let mut buf = Vec::with_capacity(8 + data.len());
    buf.push(SYNC1);
    buf.push(SYNC2);
    buf.push(pid1);
    buf.push(pid2);
    buf.push((len >> 8) as u8);
    buf.push((len & 0xFF) as u8);
    buf.extend_from_slice(data);

    let mut cs: u32 = buf.iter().map(|&b| u32::from(b)).sum();
    cs = (cs ^ 0xFFFF).wrapping_add(1);
    buf.push(((cs >> 8) & 0xFF) as u8);
    buf.push((cs & 0xFF) as u8);
    buf
}

/// `doAmptekNetFinderPacket`'s outbound request (`ConsoleHelper.cpp:203`):
/// the fixed 8-byte literal `{0xF5,0xFA,0x03,0x07,0x00,0x00,0xFE,0x07}` is
/// exactly `pack_out(PID1_REQ_SCOPE_MISC, PID2_SEND_NETFINDER_READBACK,
/// &[])` -- built through [`pack_out`] here rather than hardcoded, with a
/// test asserting the two produce identical bytes.
pub fn build_netfinder_direct_request() -> Vec<u8> {
    pack_out(pid1::REQ_SCOPE_MISC, pid2_netfinder::READBACK_REQUEST, &[])
}

/// `doAmptekNetFinderPacket`'s inbound-response validation
/// (`ConsoleHelper.cpp:232-252`): check `SYNC1 SYNC2
/// PID1_RCV_SCOPE_MISC RCVPT_NETFINDER_READBACK`, then return the `LEN`
/// bytes of DATA following the 6-byte header.
///
/// # Preserved upstream quirk
/// Unlike [`parse_packet`], this does **not** verify the trailing
/// checksum -- C's `doAmptekNetFinderPacket` never calls
/// `ParsePacketStatus`/`TestPacketCkSumOK` on this response, it only
/// checks the 4-byte header and the `LEN < iSize` bound
/// (`ConsoleHelper.cpp:236-237`) before trusting the payload. Reproduced
/// as-is rather than silently adding a check the vendor's own reference
/// client skips, since a firmware quirk that fails this checksum but
/// still carries a valid discovery reply cannot be ruled out without
/// hardware to test against.
///
/// # Restructuring vs. C
/// Requires `6 + LEN <= raw.len()` before slicing out the DATA region.
/// C's equivalent check is only `LEN < iSize` (`ConsoleHelper.cpp:237`)
/// against a fixed, zero-initialized 1024-byte buffer, so a claimed LEN
/// that runs past the bytes actually received there silently reads
/// zero-padding rather than crashing; a Rust slice has no such padding,
/// so this returns `None` instead -- for every real NetFinder readback
/// (always well over 24 bytes) the two conditions coincide.
pub fn parse_netfinder_direct_response_header(raw: &[u8]) -> Option<&[u8]> {
    if raw.len() < 6 {
        return None;
    }
    if raw[0] != SYNC1
        || raw[1] != SYNC2
        || raw[2] != pid1::RCV_SCOPE_MISC
        || raw[3] != pid2_netfinder::READBACK_RESPONSE
    {
        return None;
    }
    let len = usize::from(raw[4]) * 256 + usize::from(raw[5]);
    if 6 + len > raw.len() {
        return None;
    }
    Some(&raw[6..6 + len])
}

/// A parsed response packet: `ParsePacketStatus` + `ParsePacket`'s outcome
/// (`ParsePacket.cpp:11-49,119-168`) collapsed into one type -- C keeps the
/// status byte and the routed `ReqProcess` bitmask in two separate
/// out-params (`Packet_In::STATUS`, `DppStateType::ReqProcess`); a
/// malformed/errored packet can never carry spectrum or status data in
/// this port, so [`ReceivedPacket`] makes that unrepresentable instead of
/// leaving `data`/`pid1`/`pid2` populated-but-meaningless on an error path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReceivedPacket {
    /// `preqProcessStatus`: a DP4-style status packet, `data` is the raw
    /// 64-byte status block (`ConsoleHelper.cpp:698-705`).
    Status { data: Vec<u8> },
    /// `preqProcessSpectrum`: a spectrum (+ optional trailing status)
    /// packet. `pid2` selects channel count and whether status follows
    /// (`ConsoleHelper::ProcessSpectrumEx`, `ConsoleHelper.cpp:736-754`).
    Spectrum { pid2: u8, data: Vec<u8> },
    /// `preqProcessCfgRead`: an ASCII configuration readback packet
    /// (`ConsoleHelper::ProcessCfgReadEx`'s `CfgReadBack` branch,
    /// `ConsoleHelper.cpp:769-823`).
    ConfigReadback { data: Vec<u8> },
    /// `preqProcessAck`: **any** well-framed `PID1_ACK` packet
    /// (`ParsePacket.cpp:154-155`). C's routing `if`/`else if` chain only
    /// gates on the packet's frame-level `STATUS` (sync/length/checksum,
    /// checked once up front); once that passes, `PID1 == PID1_ACK` alone
    /// selects this route regardless of the ack code in `PID2` -- a
    /// `PID2_ACK_BAD_PARAM` response is routed here exactly like a bare
    /// `PID2_ACK_OK`. `ReceiveData()` therefore reports success
    /// (`bDataReceived = true`) for an error ack too; distinguishing a real
    /// failure from a bare success is `ParseCmd`'s job
    /// (`ParsePacket.cpp:179-199`), which only `sendCommandString` bothers
    /// to call -- see [`offending_command_text`]. `data` is `PIN->DATA`,
    /// needed by that same check.
    Ack { status: AckStatus, data: Vec<u8> },
    /// A packet whose framing/checksum itself failed (`ParsePacketStatus`,
    /// `ParsePacket.cpp:17-46`), or any other well-framed, checksum-valid
    /// packet C's `ParsePacket` falls through to
    /// `PID2_ACK_PID_ERROR`/`preqProcessError` for
    /// (`ParsePacket.cpp:156-159`). Never produced for an ack packet --
    /// see [`ReceivedPacket::Ack`].
    Error { status: AckStatus },
}

/// `CConsoleHelper::ProcessSpectrumEx` (`ConsoleHelper.cpp:736-754`):
/// derive the channel count from a [`ReceivedPacket::Spectrum`]'s `pid2`
/// and decode each channel as a 3-byte little-endian value. Returns
/// `(channels, values)`; the caller slices out any trailing 64-byte DP4
/// status block (present when `pid2` is even, `data[channels*3..]`) and
/// decodes it separately via [`crate::status::process_status`] -- this
/// function only does the spectrum half, matching `ProcessSpectrumEx`
/// itself (its own trailing-status branch calls out to
/// `DP5Stat.Process_Status` rather than inlining it).
///
/// `(pid2 - 1) & 14` is always in `0..=14` for any `u8` (C computes the
/// same mask after promoting `PID2` to `int`, so the identical
/// arithmetic-vs-bitwise identity holds there too), so `channels` is
/// always in `256..=32768` regardless of `pid2` -- no overflow risk from
/// an out-of-range value.
///
/// # Restructuring vs. C
/// `Packet_In::DATA` is a fixed 32768-byte array, so a short response
/// still leaves `CHANNELS` fully computed from `pid2` and
/// `ProcessSpectrumEx` reads whatever stale/zeroed bytes happen to sit
/// past what the socket actually wrote -- garbage data, not a
/// memory-safety bug, but not reproducible against a right-sized
/// `Vec<u8>` here. This returns however many complete 3-byte groups
/// `data` actually contains (`channels.min(data.len() / 3)`) instead.
pub fn decode_spectrum(pid2: u8, data: &[u8]) -> (usize, Vec<i32>) {
    let shift = ((i32::from(pid2) - 1) & 14) / 2;
    let channels = (256i32 << shift) as usize;
    let n = channels.min(data.len() / 3);
    let values = (0..n)
        .map(|i| {
            let base = i * 3;
            i32::from(data[base])
                + i32::from(data[base + 1]) * 256
                + i32::from(data[base + 2]) * 65536
        })
        .collect();
    (channels, values)
}

/// `CParsePacket::ParsePacketStatus` (`ParsePacket.cpp:11-49`): validate
/// `SYNC1`/`SYNC2`/checksum and extract `(pid1, pid2, data)`. Returns
/// `Err(status)` for anything but `PID2_ACK_OK` -- C instead threads the
/// status byte through `Packet_In::STATUS` for the caller to re-check; see
/// [`ReceivedPacket`]'s doc for why this port collapses that into `Result`.
///
/// `raw` is the packet as received (starting at `SYNC1`, including the
/// trailing 2-byte checksum) -- C reads directly out of a fixed 24648-byte
/// buffer at absolute offsets; this takes exactly the bytes that arrived
/// instead, and returns a length error if `raw` is too short to contain
/// the `LEN` the header claims (C indexes straight past the buffer end in
/// that case, `ParsePacket.cpp:23-26` -- an upstream defect, not
/// reproduced; see the crate-level defect list).
fn parse_packet_status(raw: &[u8]) -> Result<(u8, u8, Vec<u8>), AckStatus> {
    if raw.len() < 2 || raw[0] != SYNC1 {
        return Err(AckStatus::SyncError);
    }
    if raw[1] != SYNC2 {
        return Err(AckStatus::SyncError);
    }
    if raw.len() < 6 {
        return Err(AckStatus::LenError);
    }
    if raw[4] >= 128 {
        return Err(AckStatus::LenError);
    }
    let len = usize::from(raw[4]) * 256 + usize::from(raw[5]);
    let pid1 = raw[2];
    let pid2 = raw[3];
    // header(6) + data(len) + checksum(2)
    if raw.len() < 6 + len + 2 {
        return Err(AckStatus::LenError);
    }
    let header_and_data = &raw[0..6 + len];
    let sum: u32 = header_and_data.iter().map(|&b| u32::from(b)).sum();
    let wire_checksum = u32::from(raw[6 + len]) * 256 + u32::from(raw[6 + len + 1]);
    if (sum.wrapping_add(wire_checksum)) & 0xFFFF != 0 {
        return Err(AckStatus::ChecksumError);
    }
    Ok((pid1, pid2, raw[6..6 + len].to_vec()))
}

/// `CParsePacket::ParseCmd` (`ParsePacket.cpp:179-199`): extract the
/// offending command text C's firmware echoes back in `DATA` for
/// `BadParam`/`Pc5NotPresent`/`Unrecognized` responses. Callable on any
/// [`ReceivedPacket::Ack`], matching `sendCommandString`
/// (`drvAmptek.cpp:437-477`), the only caller that checks this -- it is
/// the sole way to tell a real error ack from a bare success, since
/// [`ReceivedPacket::Ack`] itself makes no such distinction.
pub fn offending_command_text(status: AckStatus, data: &[u8]) -> Option<String> {
    if !data.is_empty()
        && matches!(
            status,
            AckStatus::BadParam | AckStatus::Pc5NotPresent | AckStatus::Unrecognized
        )
    {
        Some(String::from_utf8_lossy(data).into_owned())
    } else {
        None
    }
}

/// `CParsePacket::ParsePacket` (`ParsePacket.cpp:119-168`): validate the
/// packet, then route it by `(pid1, pid2)`. Restricted to the routes
/// `CConsoleHelper::ReceiveData` acts on
/// (`preqProcessStatus`/`preqProcessSpectrum`/`preqProcessCfgRead`/
/// `preqProcessAck`; `ConsoleHelper.cpp:696-732`) -- scope data, misc data,
/// diagnostics and PA-cal routes exist in C but `drvAmptek.cpp` never
/// triggers them, so a packet on one of those routes here classifies as
/// [`ReceivedPacket::Error`] with `AckStatus::PidError`, matching C's own
/// unmatched-PID fallthrough (`ParsePacket.cpp:156-158`).
pub fn parse_packet(raw: &[u8]) -> ReceivedPacket {
    let (pid1, pid2, data) = match parse_packet_status(raw) {
        Ok(parsed) => parsed,
        Err(status) => {
            return ReceivedPacket::Error { status };
        }
    };

    if pid1 == pid1::RCV_STATUS && pid2 == pid2_rcv::DP4_STYLE_STATUS {
        ReceivedPacket::Status { data }
    } else if pid1 == pid1::RCV_SPECTRUM
        && (pid2_rcv::SPECTRUM_256_LO..=pid2_rcv::SPECTRUM_8192_HI).contains(&pid2)
    {
        ReceivedPacket::Spectrum { pid2, data }
    } else if pid1 == pid1::RCV_SCOPE_MISC && pid2 == pid2_rcv::CONFIG_READBACK {
        ReceivedPacket::ConfigReadback { data }
    } else if pid1 == pid1::ACK {
        ReceivedPacket::Ack {
            status: AckStatus::from_byte(pid2),
            data,
        }
    } else {
        ReceivedPacket::Error {
            status: AckStatus::PidError,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `CSendCommand::POUT_Buffer` (`SendCommand.cpp:446-470`) for a
    /// zero-length packet (e.g. `XMTPT_ENABLE_MCA_MCS`): the checksum of
    /// `SYNC1+SYNC2+PID1+PID2+0+0` two's-complemented into the trailer.
    #[test]
    fn pack_out_zero_length_checksum() {
        let buf = pack_out(0xF0, 0x02, &[]);
        assert_eq!(buf[0], SYNC1);
        assert_eq!(buf[1], SYNC2);
        assert_eq!(buf.len(), 8);
        let sum: u32 = buf[0..6].iter().map(|&b| u32::from(b)).sum();
        let cs = (sum ^ 0xFFFF).wrapping_add(1);
        assert_eq!(buf[6], ((cs >> 8) & 0xFF) as u8);
        assert_eq!(buf[7], (cs & 0xFF) as u8);
    }

    /// A round trip through [`pack_out`] then [`parse_packet_status`] must
    /// recover the exact `(pid1, pid2, data)` -- the checksum algorithm is
    /// symmetric (`CS = (sum ^ 0xFFFF) + 1`, verified by re-summing
    /// including the trailer and requiring `& 0xFFFF == 0`,
    /// `ParsePacket.cpp:23-28`).
    #[test]
    fn pack_and_parse_round_trip() {
        let payload = b"CLCK=AUTO;TPEA=12.500000;";
        let buf = pack_out(0x20, 0x02, payload);
        let (pid1, pid2, data) = parse_packet_status(&buf).unwrap();
        assert_eq!(pid1, 0x20);
        assert_eq!(pid2, 0x02);
        assert_eq!(data, payload);
    }

    #[test]
    fn parse_packet_status_rejects_bad_sync() {
        let mut buf = pack_out(0x01, 0x01, &[]);
        buf[0] = 0x00;
        assert_eq!(parse_packet_status(&buf), Err(AckStatus::SyncError));
    }

    #[test]
    fn parse_packet_status_rejects_corrupted_checksum() {
        let mut buf = pack_out(0x01, 0x01, &[]);
        let last = buf.len() - 1;
        buf[last] ^= 0xFF;
        assert_eq!(parse_packet_status(&buf), Err(AckStatus::ChecksumError));
    }

    #[test]
    fn parse_packet_status_rejects_truncated_buffer() {
        let buf = pack_out(0x20, 0x02, b"CLCK=AUTO;");
        assert_eq!(
            parse_packet_status(&buf[..buf.len() - 3]),
            Err(AckStatus::LenError)
        );
    }

    /// `ParsePacket`'s status route: `PID1_RCV_STATUS` +
    /// `PID2_SEND_DP4_STYLE_STATUS` (`ParsePacket.cpp:125-126`).
    #[test]
    fn parse_packet_routes_status() {
        let raw_status = vec![0u8; 64];
        let buf = pack_out(0x80, 0x01, &raw_status);
        match parse_packet(&buf) {
            ReceivedPacket::Status { data } => assert_eq!(data, raw_status),
            other => panic!("expected Status, got {other:?}"),
        }
    }

    /// `ParsePacket`'s spectrum route, spanning both the plain and
    /// plus-status PID2 sub-ranges (`ParsePacket.cpp:127-128`).
    #[test]
    fn parse_packet_routes_spectrum_across_the_full_pid2_range() {
        for pid2 in 0x01..=0x0Cu8 {
            let buf = pack_out(0x81, pid2, &[1, 2, 3]);
            match parse_packet(&buf) {
                ReceivedPacket::Spectrum { pid2: got, data } => {
                    assert_eq!(got, pid2);
                    assert_eq!(data, vec![1, 2, 3]);
                }
                other => panic!("pid2={pid2:#x}: expected Spectrum, got {other:?}"),
            }
        }
    }

    /// `ProcessSpectrumEx`'s channel-count formula
    /// (`256 * 2^(((pid2-1)&14)/2)`), spot-checked at the boundaries of
    /// each doubling step across the full `SPECTRUM_256_LO..=
    /// SPECTRUM_8192_HI` range.
    #[test]
    fn decode_spectrum_channel_count_matches_pid2() {
        let expected = [
            (0x01u8, 256usize),
            (0x02, 256),
            (0x03, 512),
            (0x04, 512),
            (0x05, 1024),
            (0x06, 1024),
            (0x07, 2048),
            (0x08, 2048),
            (0x09, 4096),
            (0x0A, 4096),
            (0x0B, 8192),
            (0x0C, 8192),
        ];
        for (pid2, channels) in expected {
            let (got, _) = decode_spectrum(pid2, &[]);
            assert_eq!(got, channels, "pid2={pid2:#x}");
        }
    }

    /// Each channel is a 3-byte little-endian value
    /// (`DATA[i*3] + DATA[i*3+1]*256 + DATA[i*3+2]*65536`).
    #[test]
    fn decode_spectrum_decodes_3_byte_little_endian_channels() {
        // pid2=0x01 -> 256 channels; only supply 3 real channels' worth
        // of data, the rest is implicitly absent (short-response case).
        let data = [
            0x01, 0x00, 0x00, // channel 0 = 1
            0xFF, 0x00, 0x00, // channel 1 = 255
            0x00, 0x01, 0x00, // channel 2 = 256
        ];
        let (channels, values) = decode_spectrum(0x01, &data);
        assert_eq!(channels, 256);
        assert_eq!(values, vec![1, 255, 256]);
    }

    /// A response shorter than `channels * 3` bytes yields only the
    /// complete 3-byte groups actually present, not a panic or
    /// out-of-bounds read -- see the function doc's "Restructuring vs.
    /// C" note.
    #[test]
    fn decode_spectrum_truncates_to_available_data() {
        let data = [0x2A, 0x00, 0x00, 0x01]; // 1 full channel + 1 stray byte
        let (channels, values) = decode_spectrum(0x01, &data);
        assert_eq!(channels, 256);
        assert_eq!(values, vec![42]);
    }

    /// `ParsePacket`'s config-readback route (`RCVPT_CONFIG_READBACK`,
    /// `ParsePacket.cpp:148-149`).
    #[test]
    fn parse_packet_routes_config_readback() {
        let buf = pack_out(0x82, 0x07, b"CLCK=AUTO;");
        match parse_packet(&buf) {
            ReceivedPacket::ConfigReadback { data } => assert_eq!(data, b"CLCK=AUTO;"),
            other => panic!("expected ConfigReadback, got {other:?}"),
        }
    }

    /// A bare ACK (`PID1_ACK`, `PID2_ACK_OK`) -- `ParsePacket.cpp:154-155`.
    #[test]
    fn parse_packet_routes_bare_ack() {
        let buf = pack_out(0xFF, 0x00, &[]);
        assert_eq!(
            parse_packet(&buf),
            ReceivedPacket::Ack {
                status: AckStatus::Ok,
                data: vec![],
            }
        );
    }

    /// An ACK carrying an error status still routes to `Ack`, not `Error`
    /// -- `ParsePacket` doesn't look at the ack code, only `PID1_ACK`
    /// (`ParsePacket.cpp:154-155`); `offending_command_text` is the
    /// separate, caller-invoked check that actually distinguishes it
    /// (`CParsePacket::ParseCmd`, `ParsePacket.cpp:179-199`).
    #[test]
    fn parse_packet_routes_error_ack_to_ack_not_error() {
        let buf = pack_out(0xFF, 0x05, b"BADCMD=1;");
        match parse_packet(&buf) {
            ReceivedPacket::Ack {
                status: AckStatus::BadParam,
                data,
            } => {
                assert_eq!(
                    offending_command_text(AckStatus::BadParam, &data),
                    Some("BADCMD=1;".to_string())
                );
            }
            other => panic!("expected BadParam Ack, got {other:?}"),
        }
    }

    /// An unmatched `(pid1, pid2)` combination C's `ParsePacket` falls
    /// through to `PID2_ACK_PID_ERROR` for (`ParsePacket.cpp:156-159`).
    #[test]
    fn parse_packet_unmatched_pid_is_pid_error() {
        let buf = pack_out(0x03, 0x01, &[]);
        assert_eq!(
            parse_packet(&buf),
            ReceivedPacket::Error {
                status: AckStatus::PidError,
            }
        );
    }

    /// `doAmptekNetFinderPacket`'s hardcoded 8-byte literal
    /// (`ConsoleHelper.cpp:203`) must equal what [`pack_out`] produces
    /// for the same `(pid1, pid2)`.
    #[test]
    fn netfinder_direct_request_matches_the_c_literal() {
        const C_LITERAL: [u8; 8] = [0xF5, 0xFA, 0x03, 0x07, 0x00, 0x00, 0xFE, 0x07];
        assert_eq!(build_netfinder_direct_request(), C_LITERAL);
    }

    #[test]
    fn netfinder_direct_response_header_extracts_data() {
        let mut raw = vec![SYNC1, SYNC2, 0x82, 0x08, 0x00, 0x04];
        raw.extend_from_slice(&[1, 2, 3, 4]);
        raw.extend_from_slice(&[0xAA, 0xBB]); // unverified trailer
        assert_eq!(
            parse_netfinder_direct_response_header(&raw),
            Some(&[1, 2, 3, 4][..])
        );
    }

    #[test]
    fn netfinder_direct_response_header_rejects_wrong_pid() {
        let mut raw = vec![SYNC1, SYNC2, 0x80, 0x01, 0x00, 0x00];
        raw.extend_from_slice(&[0, 0]);
        assert_eq!(parse_netfinder_direct_response_header(&raw), None);
    }

    #[test]
    fn netfinder_direct_response_header_rejects_len_exceeding_received_bytes() {
        // Header claims 4 bytes of data but none were actually received.
        let raw = vec![SYNC1, SYNC2, 0x82, 0x08, 0x00, 0x04];
        assert_eq!(parse_netfinder_direct_response_header(&raw), None);
    }
}
