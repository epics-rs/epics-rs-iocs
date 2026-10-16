//! SPiiPlusBinComm.cpp: the frames of the ACS binary array protocol.
//!
//! A command is `[D3][id][len lo][len hi] body [D6]`; a reply is
//! `[E3][id][len lo][len hi] data [E6]`, bit 7 of `len hi` set while
//! another slice is waiting. Arrays over [`MAX_PACKET_DATA`] bytes move in
//! slices: a read asks for slice `n` with `%n` before the query, a write
//! numbers its own.

/// `MAX_MESSAGE_LEN`.
pub const MAX_MESSAGE_LEN: usize = 256;
/// `MAX_PACKET_SIZE`: the data of a full slice plus its header and suffix.
pub const MAX_PACKET_SIZE: usize = 1405;
/// `MAX_PACKET_DATA`.
pub const MAX_PACKET_DATA: usize = 1400;

pub const FRAME_START: u8 = 0xd3;
pub const FRAME_END: u8 = 0xd6;
pub const REPLY_START: u8 = 0xe3;
pub const REPLY_END: u8 = 0xe6;
/// `SLICE_AVAILABLE`, in a reply's `len hi`.
pub const SLICE_AVAILABLE: u8 = 0x80;
/// `ACKNOWLEDGE`: the first byte of a write's two-byte reply.
pub const ACKNOWLEDGE: u8 = 0xe9;

const WRITE_D_ARRAY_CMD: u8 = 0xf2;
const WRITE_LD_ARRAY_CMD: u8 = 0x37;

/// The element type of an array read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Element {
    /// REAL, 8 bytes: `readFloat64ArrayCmd` / `readFloat64SliceCmd`.
    Float64,
    /// INT, 4 bytes: `readInt32ArrayCmd` / `readInt32SliceCmd`.
    Int32,
}

impl Element {
    /// `DOUBLE_DATA_SIZE` / `INT_DATA_SIZE`.
    pub fn size(self) -> usize {
        match self {
            Element::Float64 => 8,
            Element::Int32 => 4,
        }
    }

    /// `READ_D_ARRAY_CMD` / `READ_I_ARRAY_CMD`: an array in one packet.
    fn read_cmd(self) -> u8 {
        match self {
            Element::Float64 => 0xf0,
            Element::Int32 => 0xf1,
        }
    }

    /// `READ_LD_ARRAY_CMD` / `READ_LI_ARRAY_CMD`: the first of several.
    fn read_long_cmd(self) -> u8 {
        match self {
            Element::Float64 => 0x41,
            Element::Int32 => 0x44,
        }
    }

    /// `READ_LD_SLICE_CMD` / `READ_LI_SLICE_CMD`: each slice after it.
    fn read_slice_cmd(self) -> u8 {
        match self {
            Element::Float64 => 0x42,
            Element::Int32 => 0x45,
        }
    }
}

/// The inclusive index ranges of a variable: `var(i1,j1)(i2,j2)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Span {
    pub idx1: (i32, i32),
    pub idx2: (i32, i32),
}

impl Span {
    /// A one-dimensional span, `(start,end)(0,0)`.
    pub fn one(start: i32, end: i32) -> Self {
        Self {
            idx1: (start, end),
            idx2: (0, 0),
        }
    }

    fn elements(&self) -> usize {
        ((self.idx1.1 - self.idx1.0 + 1) * (self.idx2.1 - self.idx2.0 + 1)).max(0) as usize
    }

    fn name(&self, var: &str) -> String {
        format!(
            "{var}({},{})({},{})",
            self.idx1.0, self.idx1.1, self.idx2.0, self.idx2.1
        )
    }
}

/// A read command and the reply it expects.
#[derive(Debug, PartialEq, Eq)]
pub struct ReadCmd {
    pub frame: Vec<u8>,
    /// `*inBytes`: the bytes to read for this packet's reply.
    pub in_bytes: usize,
    /// `*dataBytes`: the data bytes of the whole array.
    pub data_bytes: usize,
}

fn frame(id: u8, body: &[u8]) -> Vec<u8> {
    let len = body.len();
    let mut frame = Vec::with_capacity(len + 5);
    frame.extend_from_slice(&[FRAME_START, id, len as u8, (len >> 8) as u8]);
    frame.extend_from_slice(body);
    frame.push(FRAME_END);
    frame
}

/// `readFloat64ArrayCmd` / `readInt32ArrayCmd`: `%??[size]var(..)(..)`.
pub fn read_array_cmd(element: Element, var: &str, span: Span) -> ReadCmd {
    let data_bytes = span.elements() * element.size();
    let (id, in_bytes) = if data_bytes > MAX_PACKET_DATA {
        (element.read_long_cmd(), MAX_PACKET_SIZE)
    } else {
        (element.read_cmd(), data_bytes + 5)
    };
    let mut body = b"%??".to_vec();
    body.push(element.size() as u8);
    body.extend_from_slice(span.name(var).as_bytes());
    ReadCmd {
        frame: frame(id, &body),
        in_bytes,
        data_bytes,
    }
}

/// `readFloat64SliceCmd` / `readInt32SliceCmd`: `%slice%??[size]var(..)(..)`.
pub fn read_slice_cmd(element: Element, slice: usize, var: &str, span: Span) -> ReadCmd {
    let data_bytes = span.elements() * element.size();
    // C's `ceil` is of an integer quotient, so this is the count of full
    // slices, and the slice past them is the remainder.
    let full = data_bytes / MAX_PACKET_DATA;
    let in_bytes = if slice < full {
        MAX_PACKET_SIZE
    } else {
        data_bytes - full * MAX_PACKET_DATA + 5
    };
    let mut body = format!("%{slice}%??").into_bytes();
    body.push(element.size() as u8);
    body.extend_from_slice(span.name(var).as_bytes());
    ReadCmd {
        frame: frame(element.read_slice_cmd(), &body),
        in_bytes,
        data_bytes,
    }
}

/// One packet of an array write.
#[derive(Debug, PartialEq, Eq)]
pub struct WriteCmd {
    pub frame: Vec<u8>,
    /// `*remainingSlices`: packets still to send after this one.
    pub remaining_slices: usize,
    /// `*packetDoubles`: the values this packet carries.
    pub packet_doubles: usize,
}

/// `writeFloat64ArrayCmd`: packet `slice` of `data` written to `var` over
/// `span`, `[%slice]%>>[08]var(..)(..)/%values`. The reply is two bytes
/// (`*inBytes`), [`ACKNOWLEDGE`] and the command id, or an error.
pub fn write_float64_array_cmd(var: &str, span: Span, data: &[f64], slice: usize) -> WriteCmd {
    let name = span.name(var);
    let num_doubles = span.elements();
    let total_bytes = num_doubles * 8;
    let single = name.len() + 6 + total_bytes <= MAX_PACKET_DATA;
    let (remaining_slices, packet_doubles, first) = if single {
        (0, num_doubles, 0)
    } else {
        let per_packet = (MAX_PACKET_DATA - name.len() - 8) / 8;
        let packets = num_doubles.div_ceil(per_packet);
        let packet_doubles = if slice == packets - 1 {
            num_doubles - slice * per_packet
        } else {
            per_packet
        };
        (packets - slice - 1, packet_doubles, slice * per_packet)
    };
    let mut body = Vec::with_capacity(name.len() + 8 + packet_doubles * 8);
    let id = if single {
        WRITE_D_ARRAY_CMD
    } else {
        body.extend_from_slice(format!("%{slice}").as_bytes());
        WRITE_LD_ARRAY_CMD
    };
    body.extend_from_slice(b"%>>\x08");
    body.extend_from_slice(name.as_bytes());
    body.extend_from_slice(b"/%");
    for value in &data[first..first + packet_doubles] {
        body.extend_from_slice(&value.to_le_bytes());
    }
    WriteCmd {
        frame: frame(id, &body),
        remaining_slices,
        packet_doubles,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    // The expected frames are what the C functions build for the same call.

    #[test]
    fn array_read_in_one_packet() {
        let cmd = read_array_cmd(Element::Float64, "APOS", Span::one(0, 7));
        assert_eq!(
            hex(&cmd.frame),
            "d3f01200253f3f0841504f5328302c372928302c3029d6"
        );
        assert_eq!((cmd.in_bytes, cmd.data_bytes), (69, 64));
        let cmd = read_array_cmd(Element::Int32, "APOS", Span::one(0, 7));
        assert_eq!(
            hex(&cmd.frame),
            "d3f11200253f3f0441504f5328302c372928302c3029d6"
        );
        assert_eq!((cmd.in_bytes, cmd.data_bytes), (37, 32));
    }

    #[test]
    fn array_of_exactly_one_packet_is_not_sliced() {
        let cmd = read_array_cmd(Element::Float64, "X", Span::one(0, 174));
        assert_eq!(cmd.frame[1], 0xf0);
        assert_eq!((cmd.in_bytes, cmd.data_bytes), (1405, 1400));
    }

    #[test]
    fn sliced_read_asks_full_slices_then_the_remainder() {
        let cmd = read_array_cmd(Element::Float64, "AIN", Span::one(0, 256));
        assert_eq!(
            hex(&cmd.frame),
            "d3411300253f3f0841494e28302c3235362928302c3029d6"
        );
        assert_eq!((cmd.in_bytes, cmd.data_bytes), (1405, 2056));
        let cmd = read_slice_cmd(Element::Float64, 1, "AIN", Span::one(0, 256));
        assert_eq!(
            hex(&cmd.frame),
            "d34215002531253f3f0841494e28302c3235362928302c3029d6"
        );
        assert_eq!((cmd.in_bytes, cmd.data_bytes), (661, 2056));
        let cmd = read_slice_cmd(Element::Float64, 1, "X", Span::one(0, 349));
        assert_eq!(cmd.in_bytes, 1405);
        let cmd = read_slice_cmd(Element::Float64, 2, "X", Span::one(0, 349));
        assert_eq!(cmd.in_bytes, 5);
    }

    #[test]
    fn write_fits_one_packet_without_a_slice_number() {
        let data: Vec<f64> = (0..4).map(|k| f64::from(k) * 0.5 - 3.0).collect();
        let cmd = write_float64_array_cmd("G", Span::one(0, 3), &data, 0);
        assert_eq!(
            hex(&cmd.frame),
            "d3f23100253e3e084728302c332928302c30292f25\
             00000000000008c000000000000004c000000000000000c000000000000\
             0f8bfd6"
        );
        assert_eq!((cmd.remaining_slices, cmd.packet_doubles), (0, 4));
    }

    #[test]
    fn write_splits_into_numbered_packets_of_whole_doubles() {
        let data: Vec<f64> = (0..1000).map(|k| f64::from(k) * 0.5 - 3.0).collect();
        let span = Span::one(0, 999);
        let first = write_float64_array_cmd("GV", span, &data, 0);
        assert_eq!(&first.frame[..12], b"\xd3\x37\x76\x05%0%>>\x08GV");
        assert_eq!((first.remaining_slices, first.packet_doubles), (5, 172));
        let last = write_float64_array_cmd("GV", span, &data, 5);
        assert_eq!((last.remaining_slices, last.packet_doubles), (0, 140));
        assert_eq!(
            &last.frame[last.frame.len() - 9..last.frame.len() - 1],
            &data[999].to_le_bytes()
        );
    }
}
