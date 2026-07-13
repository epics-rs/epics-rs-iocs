//! The two UDP sockets: the data broadcast (C `udpDataListenerTask`) and the
//! status broadcast (C `statusTask`'s socket half).
//!
//! Reassembly is a pure state machine, [`FrameAssembler`], so that packet loss
//! can be tested without a detector.

use std::io;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};

use socket2::{Domain, Protocol, Socket, Type};

use crate::types::{
    AUTOCAL_DATA, DAQ_PACKET_FRAGMENT, MAX_UDP_DATA_BUFFER, MAX_UDP_PACKET_LEN, PACKET_ID_OFFSET,
    PACKET_SENSOR_DATA_BYTES, PACKET_SENSOR_DATA_OFFSET, Sensor,
};

/// A complete frame: the sensor payload of every packet, in packet order.
pub struct Frame {
    pub is_autocal: bool,
    /// At least one packet arrived out of order or not at all
    /// (C's `FRAME_HAS_ALIGN_ERRORS` tag bit).
    pub align_errors: bool,
    pub payload: Vec<u8>,
}

/// Puts the packets of one frame back in order.
///
/// Each packet's 16-bit identifier counts within a group of
/// [`DAQ_PACKET_FRAGMENT`] packets, so a packet's place in the frame is
/// `group_start + id`.
///
/// C tracked the *expected* index and the *expected* position in the group
/// separately, and wrote each packet at the expected index before advancing:
/// when packets were lost, the packet that did arrive went into the missing
/// packet's slot and the whole tail of the frame was shifted by the size of the
/// gap. It also skipped a full group of 45 on a wrapped identifier, whatever
/// its position in the group, which left the next group misaligned. Here a
/// packet is written where its identifier says it belongs, and nothing else
/// moves.
pub struct FrameAssembler {
    sensor: Sensor,
    payload: Vec<u8>,
    /// Where the current group of 45 starts, in packets.
    group_start: usize,
    /// The packet expected next within the group.
    next_in_group: usize,
    expected_packets: usize,
    is_autocal: bool,
    align_errors: bool,
    started: bool,
}

impl FrameAssembler {
    pub fn new(sensor: Sensor) -> Self {
        Self {
            payload: vec![0u8; sensor.num_udp_packets * PACKET_SENSOR_DATA_BYTES],
            expected_packets: sensor.num_udp_packets,
            sensor,
            group_start: 0,
            next_in_group: 0,
            is_autocal: false,
            align_errors: false,
            started: false,
        }
    }

    fn reset(&mut self) {
        // A packet that never arrives must leave zeros behind, not the bytes
        // the previous frame put in that slot.
        self.payload.fill(0);
        self.group_start = 0;
        self.next_in_group = 0;
        self.is_autocal = false;
        self.align_errors = false;
        self.started = false;
        self.expected_packets = self.sensor.num_udp_packets;
    }

    /// Feed one packet. A frame comes back when its last packet has arrived.
    pub fn accept(&mut self, packet: &[u8]) -> Option<Frame> {
        if packet.len() != MAX_UDP_PACKET_LEN {
            // C ignored every packet whose length was not exactly one packet,
            // and so does this.
            return None;
        }

        if !self.started {
            // The first packet of a frame says what kind of frame it is, and
            // therefore how many packets it has.
            self.is_autocal = packet[0] & AUTOCAL_DATA != 0;
            self.expected_packets = self.sensor.packets(self.is_autocal);
            self.started = true;
        }

        let id = (u16::from_be_bytes([packet[PACKET_ID_OFFSET], packet[PACKET_ID_OFFSET + 1]])
            as usize)
            % DAQ_PACKET_FRAGMENT;

        if id < self.next_in_group {
            // The identifier wrapped: this packet opens the next group, and the
            // tail of the group we were in never arrived.
            self.align_errors = true;
            self.group_start += DAQ_PACKET_FRAGMENT;
            self.next_in_group = 0;
        } else if id > self.next_in_group {
            self.align_errors = true;
        }

        let slot = self.group_start + id;
        if slot >= self.expected_packets {
            // More packets than the frame can hold: the detector is sending a
            // frame we cannot describe, so start over on the next one.
            log::error!(
                "pixirad: packet {slot} does not fit a frame of {} packets",
                self.expected_packets
            );
            self.reset();
            return None;
        }

        let from = PACKET_SENSOR_DATA_OFFSET;
        self.payload[slot * PACKET_SENSOR_DATA_BYTES..(slot + 1) * PACKET_SENSOR_DATA_BYTES]
            .copy_from_slice(&packet[from..from + PACKET_SENSOR_DATA_BYTES]);

        self.next_in_group = id + 1;
        if self.next_in_group == DAQ_PACKET_FRAGMENT {
            self.group_start += DAQ_PACKET_FRAGMENT;
            self.next_in_group = 0;
        }

        if self.group_start + self.next_in_group < self.expected_packets {
            return None;
        }

        let frame = Frame {
            is_autocal: self.is_autocal,
            align_errors: self.align_errors,
            payload: self.payload[..self.expected_packets * PACKET_SENSOR_DATA_BYTES].to_vec(),
        };
        self.reset();
        Some(frame)
    }
}

/// Bind a UDP socket that listens on every interface, with the receive buffer
/// the detector's data rate needs (C's `SO_RCVBUF` call).
pub fn bind_data_socket(port: u16) -> io::Result<UdpSocket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_recv_buffer_size(MAX_UDP_DATA_BUFFER)?;
    socket.bind(&SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port).into())?;
    let actual = socket.recv_buffer_size()?;
    if actual < MAX_UDP_DATA_BUFFER {
        log::warn!(
            "pixirad: the kernel gave the data socket a {actual}-byte receive buffer, \
             {MAX_UDP_DATA_BUFFER} were asked for"
        );
    }
    Ok(socket.into())
}

/// Bind the status broadcast socket.
pub fn bind_status_socket(port: u16) -> io::Result<UdpSocket> {
    UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Asic, Build};

    fn sensor(packets: usize) -> Sensor {
        Sensor {
            asic: Asic::PIII,
            build: Build::PX1,
            modules: 1,
            rows: 2,
            cols: 4,
            dout: 2,
            cols_per_dout: 2,
            matrix_size_pxls: 8,
            bit_per_cnt_std: 15,
            autocal_bit_cnt: 3,
            num_udp_packets: packets,
            num_autocal_udp_packets: packets,
        }
    }

    /// A packet whose payload is `fill` repeated, carrying identifier `id`.
    fn packet(id: u16, fill: u8, autocal: bool) -> Vec<u8> {
        let mut p = vec![0u8; MAX_UDP_PACKET_LEN];
        p[0] = if autocal { AUTOCAL_DATA } else { 0 };
        p[PACKET_ID_OFFSET..PACKET_ID_OFFSET + 2].copy_from_slice(&id.to_be_bytes());
        for b in p[PACKET_SENSOR_DATA_OFFSET..PACKET_SENSOR_DATA_OFFSET + PACKET_SENSOR_DATA_BYTES]
            .iter_mut()
        {
            *b = fill;
        }
        p
    }

    fn payload_of(frame: &Frame, slot: usize) -> u8 {
        frame.payload[slot * PACKET_SENSOR_DATA_BYTES]
    }

    #[test]
    fn a_frame_completes_when_its_last_packet_arrives() {
        let mut asm = FrameAssembler::new(sensor(3));
        assert!(asm.accept(&packet(0, 10, false)).is_none());
        assert!(asm.accept(&packet(1, 11, false)).is_none());
        let frame = asm.accept(&packet(2, 12, false)).expect("frame");
        assert!(!frame.align_errors);
        assert!(!frame.is_autocal);
        assert_eq!(frame.payload.len(), 3 * PACKET_SENSOR_DATA_BYTES);
        assert_eq!(payload_of(&frame, 0), 10);
        assert_eq!(payload_of(&frame, 2), 12);
    }

    #[test]
    fn a_lost_packet_does_not_shift_the_ones_after_it() {
        // The defect C had: packet 2 arrives after packet 0 is lost, and C put
        // it in slot 1.
        let mut asm = FrameAssembler::new(sensor(3));
        assert!(asm.accept(&packet(1, 11, false)).is_none());
        let frame = asm.accept(&packet(2, 12, false)).expect("frame");
        assert!(frame.align_errors);
        assert_eq!(
            payload_of(&frame, 0),
            0,
            "the lost packet's slot stays zero"
        );
        assert_eq!(payload_of(&frame, 1), 11);
        assert_eq!(payload_of(&frame, 2), 12);
    }

    #[test]
    fn a_wrapped_identifier_opens_the_next_group_at_its_boundary() {
        // 50 packets: two groups, the second one 5 long. The last five packets
        // of group 0 are lost, so the identifier wraps to 0 while the group
        // position is 40. C jumped a full 45 from there and lost the alignment
        // of every later group; the packet belongs at slot 45.
        let mut asm = FrameAssembler::new(sensor(50));
        for id in 0..40u16 {
            assert!(asm.accept(&packet(id, 1, false)).is_none());
        }
        for id in 0..4u16 {
            assert!(asm.accept(&packet(id, 2, false)).is_none());
        }
        let frame = asm.accept(&packet(4, 3, false)).expect("frame");
        assert!(frame.align_errors);
        assert_eq!(payload_of(&frame, 39), 1);
        assert_eq!(
            payload_of(&frame, 44),
            0,
            "the lost packets' slots stay zero"
        );
        assert_eq!(payload_of(&frame, 45), 2);
        assert_eq!(payload_of(&frame, 49), 3);
    }

    #[test]
    fn an_autocal_frame_is_shorter_and_says_so() {
        let mut s = sensor(4);
        s.num_autocal_udp_packets = 2;
        let mut asm = FrameAssembler::new(s);
        assert!(asm.accept(&packet(0, 7, true)).is_none());
        let frame = asm.accept(&packet(1, 8, true)).expect("frame");
        assert!(frame.is_autocal);
        assert_eq!(frame.payload.len(), 2 * PACKET_SENSOR_DATA_BYTES);
    }

    #[test]
    fn a_runt_packet_is_ignored() {
        let mut asm = FrameAssembler::new(sensor(1));
        assert!(asm.accept(&[0u8; 32]).is_none());
        assert!(asm.accept(&packet(0, 5, false)).is_some());
    }
}
