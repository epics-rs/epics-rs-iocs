//! The `tcp://` preview stream: a JSON header line, then the frame's binary
//! payload (port of `serval_stream.cpp` and `histogram_io.cpp`'s framing).
//!
//! Serval's `jsonimage` / `jsonhisto` channels are a *raw TCP byte stream* (not
//! HTTP): for every frame it writes one line of ASCII JSON terminated by `\n`,
//! immediately followed by `width*height*bytesPerPixel` (or `binSize*4`) bytes
//! of **big-endian** payload. There is no trailer and no length prefix outside
//! the header, so a decoder that miscounts consumed bytes by even one desyncs
//! the stream permanently.
//!
//! UPSTREAM DEFECT (serval_stream.cpp:524, :1389, histogram_io.cpp:311): C
//! computes the payload bytes it already holds as
//! `remaining = total_read - (newline_pos - line_buffer + 1)`, where
//! `total_read` counts from the *read buffer's* start but `line_buffer` is the
//! caller's `json_start` — a different base whenever the frame did not begin at
//! offset 0. The difference under-counts (re-reading payload bytes as a header
//! → permanent desync) or over-counts (`memcpy` from past the end of the read
//! buffer → heap over-read). This decoder owns the buffer and consumes exactly
//! `header_len + 1 + payload_len` bytes per frame, so the class of bug cannot
//! be expressed.

use serde_json::Value;

/// `histogram_io.h:20` — the TDC clock period, in seconds.
pub const TDC_CLOCK_PERIOD_SEC: f64 = (1.5625 / 6.0) * 1e-9;

/// Serval's `pixelFormat` (serval_stream.cpp:483).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    UInt16,
    UInt32,
}

impl PixelFormat {
    fn parse(s: &str) -> Self {
        if s.eq_ignore_ascii_case("uint32") {
            Self::UInt32
        } else {
            Self::UInt16
        }
    }

    pub const fn bytes(self) -> usize {
        match self {
            Self::UInt16 => 2,
            Self::UInt32 => 4,
        }
    }
}

/// A decoded `jsonimage` frame. Pixels are widened to `u32` regardless of the
/// wire format so the accumulators have one input type.
#[derive(Debug, Clone, PartialEq)]
pub struct ImageFrame {
    pub width: usize,
    pub height: usize,
    pub format: PixelFormat,
    pub frame_number: i32,
    pub time_at_frame: f64,
    pub pixels: Vec<u32>,
}

/// A decoded `jsonhisto` frame.
#[derive(Debug, Clone, PartialEq)]
pub struct HistogramFrame {
    pub bin_width: i32,
    pub bin_offset: i32,
    pub frame_number: i32,
    pub time_at_frame: f64,
    pub counts: Vec<u32>,
}

impl HistogramFrame {
    /// The bin's left edge, in milliseconds (C `calculate_bin_edges` scaled by
    /// 1e3, histogram_io.cpp:567).
    pub fn time_axis_ms(&self) -> Vec<f64> {
        (0..self.counts.len())
            .map(|i| {
                (f64::from(self.bin_offset) + i as f64 * f64::from(self.bin_width))
                    * TDC_CLOCK_PERIOD_SEC
                    * 1e3
            })
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Frame {
    Image(ImageFrame),
    Histogram(HistogramFrame),
}

#[derive(Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// The header line was not JSON, or its fields are out of range. The frame
    /// is unrecoverable: the payload length is unknown, so the stream has to be
    /// torn down rather than guessed past.
    BadHeader(String),
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let Self::BadHeader(m) = self;
        write!(f, "stream header: {m}")
    }
}

impl std::error::Error for DecodeError {}

/// C's dimension guards (serval_stream.cpp:490, histogram_io.cpp:245).
const MAX_DIM: usize = 100_000;
const MAX_BINS: usize = 1_000_000;
/// A header line longer than this is not a header; the connection is desynced.
const MAX_HEADER: usize = 64 * 1024;

/// Which channel a decoder is reading — the two share the framing but not the
/// header schema.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Image,
    Histogram,
}

/// An incremental frame decoder: push bytes, pull frames.
pub struct FrameDecoder {
    channel: Channel,
    buf: Vec<u8>,
}

impl FrameDecoder {
    pub fn new(channel: Channel) -> Self {
        Self {
            channel,
            buf: Vec::new(),
        }
    }

    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    pub fn buffered(&self) -> usize {
        self.buf.len()
    }

    /// The next complete frame, or `None` when more bytes are needed.
    pub fn next_frame(&mut self) -> Result<Option<Frame>, DecodeError> {
        let Some(newline) = self.buf.iter().position(|&b| b == b'\n') else {
            if self.buf.len() > MAX_HEADER {
                return Err(DecodeError::BadHeader(format!(
                    "no newline in {} bytes",
                    self.buf.len()
                )));
            }
            return Ok(None);
        };

        // C skips leading bytes before the '{' (serval_stream.cpp:452); a line
        // with no '{' at all is a keep-alive/blank line, not an error.
        let line = &self.buf[..newline];
        let Some(brace) = line.iter().position(|&b| b == b'{') else {
            self.buf.drain(..=newline);
            return self.next_frame();
        };
        let header: Value = serde_json::from_slice(&line[brace..])
            .map_err(|e| DecodeError::BadHeader(e.to_string()))?;

        let (payload_len, frame) = match self.channel {
            Channel::Image => image_header(&header)?,
            Channel::Histogram => histogram_header(&header)?,
        };

        if self.buf.len() < newline + 1 + payload_len {
            return Ok(None); // Header stays buffered; retry when more arrives.
        }
        let payload: Vec<u8> = self
            .buf
            .drain(..newline + 1 + payload_len)
            .skip(newline + 1)
            .collect();
        Ok(Some(frame.finish(&payload)))
    }
}

/// A frame whose header is decoded and whose payload is still to come.
enum Pending {
    Image {
        width: usize,
        height: usize,
        format: PixelFormat,
        frame_number: i32,
        time_at_frame: f64,
    },
    Histogram {
        bin_size: usize,
        bin_width: i32,
        bin_offset: i32,
        frame_number: i32,
        time_at_frame: f64,
    },
}

impl Pending {
    fn finish(self, payload: &[u8]) -> Frame {
        match self {
            Self::Image {
                width,
                height,
                format,
                frame_number,
                time_at_frame,
            } => Frame::Image(ImageFrame {
                width,
                height,
                format,
                frame_number,
                time_at_frame,
                pixels: decode_be(payload, format),
            }),
            Self::Histogram {
                bin_size,
                bin_width,
                bin_offset,
                frame_number,
                time_at_frame,
            } => {
                debug_assert_eq!(payload.len(), bin_size * 4);
                Frame::Histogram(HistogramFrame {
                    bin_width,
                    bin_offset,
                    frame_number,
                    time_at_frame,
                    counts: decode_be(payload, PixelFormat::UInt32),
                })
            }
        }
    }
}

/// The payload is big-endian on the wire.
///
/// UPSTREAM DEFECT (serval_stream.cpp:568, :578, :1433, :1443,
/// histogram_io.cpp:352): C calls `__builtin_bswap16/32` unconditionally, which
/// is only a network-to-host conversion on a little-endian host — on a
/// big-endian host it corrupts every pixel. `from_be_bytes` is correct on both.
fn decode_be(payload: &[u8], format: PixelFormat) -> Vec<u32> {
    match format {
        PixelFormat::UInt16 => payload
            .chunks_exact(2)
            .map(|c| u32::from(u16::from_be_bytes([c[0], c[1]])))
            .collect(),
        PixelFormat::UInt32 => payload
            .chunks_exact(4)
            .map(|c| u32::from_be_bytes([c[0], c[1], c[2], c[3]]))
            .collect(),
    }
}

fn header_i64(header: &Value, key: &str, default: i64) -> i64 {
    match header.get(key) {
        Some(Value::Number(n)) => n
            .as_i64()
            .unwrap_or_else(|| n.as_f64().unwrap_or(0.0) as i64),
        _ => default,
    }
}

fn image_header(header: &Value) -> Result<(usize, Pending), DecodeError> {
    let width = header_i64(header, "width", -1);
    let height = header_i64(header, "height", -1);
    let format = PixelFormat::parse(
        header
            .get("pixelFormat")
            .and_then(Value::as_str)
            .unwrap_or("uint16"),
    );
    let (Ok(width), Ok(height)) = (usize::try_from(width), usize::try_from(height)) else {
        return Err(DecodeError::BadHeader(format!(
            "dimensions {width}x{height}"
        )));
    };
    if width == 0 || height == 0 || width > MAX_DIM || height > MAX_DIM {
        return Err(DecodeError::BadHeader(format!(
            "dimensions {width}x{height}"
        )));
    }
    // UPSTREAM DEFECT (serval_stream.cpp:487): C computes `pixel_count =
    // width * height` in `int` *before* the range check, so a hostile or
    // corrupt header overflows the multiplication and the check passes on the
    // wrapped value. Checked here, in usize, before anything is allocated.
    let payload = width
        .checked_mul(height)
        .and_then(|n| n.checked_mul(format.bytes()))
        .ok_or_else(|| DecodeError::BadHeader(format!("payload of {width}x{height} overflows")))?;
    Ok((
        payload,
        Pending::Image {
            width,
            height,
            format,
            frame_number: header_i64(header, "frameNumber", 0) as i32,
            time_at_frame: header
                .get("timeAtFrame")
                .and_then(Value::as_f64)
                .unwrap_or(0.0),
        },
    ))
}

fn histogram_header(header: &Value) -> Result<(usize, Pending), DecodeError> {
    let bin_size = header_i64(header, "binSize", -1);
    let Ok(bin_size) = usize::try_from(bin_size) else {
        return Err(DecodeError::BadHeader(format!("binSize {bin_size}")));
    };
    if bin_size == 0 || bin_size > MAX_BINS {
        return Err(DecodeError::BadHeader(format!("binSize {bin_size}")));
    }
    Ok((
        bin_size * 4,
        Pending::Histogram {
            bin_size,
            bin_width: header_i64(header, "binWidth", 0) as i32,
            bin_offset: header_i64(header, "binOffset", 0) as i32,
            frame_number: header_i64(header, "frameNumber", 0) as i32,
            time_at_frame: header
                .get("timeAtFrame")
                .and_then(Value::as_f64)
                .unwrap_or(0.0),
        },
    ))
}

/// Split Serval's `tcp://[listen@]host:port` into a host and a port (C
/// `parseTcpPath`, serval_stream.cpp:30).
///
/// `listen@` means *Serval* listens; the IOC is the client either way, so the
/// marker is dropped.
pub fn parse_tcp_path(path: &str) -> Option<(String, u16)> {
    let rest = path.strip_prefix("tcp://")?;
    let rest = rest.rsplit_once('@').map_or(rest, |(_, after)| after);
    let (host, port) = rest.rsplit_once(':')?;
    if host.is_empty() {
        return None;
    }
    let port: u16 = port.parse().ok()?;
    if port == 0 {
        return None;
    }
    Some((host.to_string(), port))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image_bytes(width: usize, height: usize, frame: i32, pixels: &[u16]) -> Vec<u8> {
        let mut out = format!(
            r#"{{"width":{width},"height":{height},"pixelFormat":"uint16","frameNumber":{frame},"timeAtFrame":1.5}}"#
        )
        .into_bytes();
        out.push(b'\n');
        for p in pixels {
            out.extend_from_slice(&p.to_be_bytes());
        }
        out
    }

    #[test]
    fn a_whole_image_frame_decodes() {
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&image_bytes(2, 2, 7, &[1, 2, 3, 0xbeef]));
        let Some(Frame::Image(f)) = d.next_frame().unwrap() else {
            panic!("no frame")
        };
        assert_eq!(f.width, 2);
        assert_eq!(f.height, 2);
        assert_eq!(f.frame_number, 7);
        assert_eq!(f.time_at_frame, 1.5);
        assert_eq!(f.format, PixelFormat::UInt16);
        assert_eq!(f.pixels, vec![1, 2, 3, 0xbeef]);
        assert_eq!(d.buffered(), 0);
        assert_eq!(d.next_frame().unwrap(), None);
    }

    #[test]
    fn a_frame_split_across_reads_decodes_once_complete() {
        let bytes = image_bytes(2, 1, 1, &[0x0102, 0x0304]);
        let mut d = FrameDecoder::new(Channel::Image);
        for split in 1..bytes.len() {
            let mut d2 = FrameDecoder::new(Channel::Image);
            d2.push(&bytes[..split]);
            assert_eq!(d2.next_frame().unwrap(), None, "split at {split}");
            d2.push(&bytes[split..]);
            let Some(Frame::Image(f)) = d2.next_frame().unwrap() else {
                panic!("split at {split}: no frame")
            };
            assert_eq!(f.pixels, vec![0x0102, 0x0304], "split at {split}");
        }
        // A byte at a time, too.
        for b in &bytes {
            d.push(std::slice::from_ref(b));
            let _ = d.next_frame().unwrap();
        }
        assert!(d.buffered() <= bytes.len());
    }

    #[test]
    fn back_to_back_frames_do_not_desync() {
        // The C bug: the second frame's header is looked for at the wrong
        // offset, so payload bytes get re-scanned as a header.
        let mut bytes = image_bytes(2, 1, 1, &[0xaaaa, 0xbbbb]);
        bytes.extend(image_bytes(2, 1, 2, &[0xcccc, 0xdddd]));
        bytes.extend(image_bytes(1, 1, 3, &[0x0a0a]));
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes);

        let mut seen = Vec::new();
        while let Some(Frame::Image(f)) = d.next_frame().unwrap() {
            seen.push((f.frame_number, f.pixels));
        }
        assert_eq!(
            seen,
            vec![
                (1, vec![0xaaaa, 0xbbbb]),
                (2, vec![0xcccc, 0xdddd]),
                (3, vec![0x0a0a]),
            ]
        );
        assert_eq!(d.buffered(), 0);
    }

    #[test]
    fn a_payload_byte_that_looks_like_a_newline_is_not_a_frame_boundary() {
        // 0x0a0a contains two '\n' bytes: a decoder that re-scans consumed
        // payload for the next header splits here.
        let mut bytes = image_bytes(1, 1, 1, &[0x0a0a]);
        bytes.extend(image_bytes(1, 1, 2, &[0x0001]));
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes);
        let Some(Frame::Image(a)) = d.next_frame().unwrap() else {
            panic!()
        };
        let Some(Frame::Image(b)) = d.next_frame().unwrap() else {
            panic!()
        };
        assert_eq!(a.pixels, vec![0x0a0a]);
        assert_eq!(b.frame_number, 2);
        assert_eq!(b.pixels, vec![0x0001]);
    }

    #[test]
    fn uint32_pixels_decode_big_endian() {
        let mut bytes =
            br#"{"width":1,"height":2,"pixelFormat":"uint32","frameNumber":4}"#.to_vec();
        bytes.push(b'\n');
        bytes.extend_from_slice(&0x0102_0304u32.to_be_bytes());
        bytes.extend_from_slice(&0xdead_beefu32.to_be_bytes());
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes);
        let Some(Frame::Image(f)) = d.next_frame().unwrap() else {
            panic!()
        };
        assert_eq!(f.format, PixelFormat::UInt32);
        assert_eq!(f.pixels, vec![0x0102_0304, 0xdead_beef]);
        // timeAtFrame absent → 0.0, as in C's j.value(...) default.
        assert_eq!(f.time_at_frame, 0.0);
    }

    #[test]
    fn a_histogram_frame_decodes_with_its_time_axis() {
        let mut bytes =
            br#"{"binSize":3,"binWidth":10,"binOffset":5,"frameNumber":2,"timeAtFrame":0.25}"#
                .to_vec();
        bytes.push(b'\n');
        for v in [1u32, 2, 0xffff_ffff] {
            bytes.extend_from_slice(&v.to_be_bytes());
        }
        let mut d = FrameDecoder::new(Channel::Histogram);
        d.push(&bytes);
        let Some(Frame::Histogram(h)) = d.next_frame().unwrap() else {
            panic!()
        };
        assert_eq!(h.counts, vec![1, 2, 0xffff_ffff]);
        assert_eq!(h.bin_width, 10);
        assert_eq!(h.bin_offset, 5);
        assert_eq!(h.frame_number, 2);
        let axis = h.time_axis_ms();
        assert_eq!(axis.len(), 3);
        assert!((axis[0] - 5.0 * TDC_CLOCK_PERIOD_SEC * 1e3).abs() < 1e-15);
        assert!((axis[2] - 25.0 * TDC_CLOCK_PERIOD_SEC * 1e3).abs() < 1e-15);
    }

    #[test]
    fn a_bad_header_is_an_error_not_a_guess() {
        for header in [
            &br#"{"width":0,"height":4}"#[..],
            &br#"{"width":-1,"height":4}"#[..],
            &br#"{"width":200000,"height":4}"#[..],
            &br#"{"height":4}"#[..],
            &br#"{"width":1,"#[..], // truncated json, newline present
        ] {
            let mut bytes = header.to_vec();
            bytes.push(b'\n');
            let mut d = FrameDecoder::new(Channel::Image);
            d.push(&bytes);
            assert!(
                d.next_frame().is_err(),
                "{}",
                String::from_utf8_lossy(header)
            );
        }
        let mut d = FrameDecoder::new(Channel::Histogram);
        d.push(b"{\"binSize\":0}\n");
        assert!(d.next_frame().is_err());
    }

    #[test]
    fn a_line_without_a_brace_is_skipped() {
        let mut bytes = b"\r\n".to_vec();
        bytes.extend(image_bytes(1, 1, 9, &[7]));
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes);
        let Some(Frame::Image(f)) = d.next_frame().unwrap() else {
            panic!()
        };
        assert_eq!(f.frame_number, 9);
    }

    #[test]
    fn leading_junk_before_the_brace_is_skipped() {
        let mut bytes = b"garbage".to_vec();
        bytes.extend(image_bytes(1, 1, 3, &[42]));
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes);
        let Some(Frame::Image(f)) = d.next_frame().unwrap() else {
            panic!()
        };
        assert_eq!(f.pixels, vec![42]);
    }

    #[test]
    fn tcp_paths_parse_with_and_without_listen() {
        assert_eq!(
            parse_tcp_path("tcp://listen@localhost:8089"),
            Some(("localhost".to_string(), 8089))
        );
        assert_eq!(
            parse_tcp_path("tcp://127.0.0.1:8451"),
            Some(("127.0.0.1".to_string(), 8451))
        );
        assert_eq!(parse_tcp_path("http://localhost:8089"), None);
        assert_eq!(parse_tcp_path("tcp://localhost"), None);
        assert_eq!(parse_tcp_path("tcp://localhost:0"), None);
        assert_eq!(parse_tcp_path("tcp://localhost:70000"), None);
        assert_eq!(parse_tcp_path("tcp://:8089"), None);
        assert_eq!(parse_tcp_path("file:///data"), None);
    }
}
