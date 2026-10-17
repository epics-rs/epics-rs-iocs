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

/// A `jsonimage` pixel's storage (serval_stream_validation.h:18-22).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    UInt8,
    UInt16,
    UInt32,
}

impl PixelFormat {
    const fn from_bytes(bytes: usize) -> Self {
        match bytes {
            1 => Self::UInt8,
            4 => Self::UInt32,
            _ => Self::UInt16,
        }
    }

    pub const fn bytes(self) -> usize {
        match self {
            Self::UInt8 => 1,
            Self::UInt16 => 2,
            Self::UInt32 => 4,
        }
    }
}

/// The largest `jsonimage` frame accepted (C `ImageFrameLimits`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImageLimits {
    pub max_dimension: usize,
    pub max_pixels: usize,
    pub max_payload_bytes: usize,
}

/// Eight 256x256 chips at four bytes a pixel (serval_stream_validation.cpp:22-26).
const SUPPORTED_PIXELS: usize = 8 * 256 * 256;
const SUPPORTED_DIMENSION: usize = 8 * 256;

impl ImageLimits {
    /// C `detectorImageFrameLimits` (serval_stream_validation.cpp:39-61): the
    /// detector's ADMaxSizeX/Y and PixCount may tighten the supported limits,
    /// never widen them; any of them unset leaves the supported limits.
    pub fn detector(max_size_x: i32, max_size_y: i32, pix_count: i32) -> Self {
        let mut limits = Self {
            max_dimension: SUPPORTED_DIMENSION,
            max_pixels: SUPPORTED_PIXELS,
            max_payload_bytes: SUPPORTED_PIXELS * 4,
        };
        if max_size_x > 0 && max_size_y > 0 && pix_count > 0 {
            limits.max_dimension = (max_size_x.max(max_size_y) as usize).min(SUPPORTED_DIMENSION);
            limits.max_pixels = (pix_count as usize).min(SUPPORTED_PIXELS);
            limits.max_payload_bytes = limits.max_pixels * 4;
        }
        limits
    }
}

/// C `ImageHeaderError`; `Display` is `imageHeaderErrorMessage`
/// (serval_stream_validation.cpp:196-217).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageHeaderError {
    MissingField,
    InvalidFieldType,
    InvalidDimension,
    UnsupportedPixelFormat,
    InconsistentPixelMetadata,
    PixelLimitExceeded,
    PayloadLimitExceeded,
}

impl std::fmt::Display for ImageHeaderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::MissingField => "missing width or height",
            Self::InvalidFieldType => "image layout metadata has the wrong JSON type",
            Self::InvalidDimension => "image dimensions are non-positive or exceed detector limits",
            Self::UnsupportedPixelFormat => "unsupported jsonimage pixel storage format",
            Self::InconsistentPixelMetadata => "pixelFormat, bitDepth, and dataSize disagree",
            Self::PixelLimitExceeded => "image pixel count exceeds detector limits",
            Self::PayloadLimitExceeded => "image payload size exceeds detector limits",
        })
    }
}

/// A validated `jsonimage` header's geometry (C `ImageFrameLayout`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImageLayout {
    pub width: usize,
    pub height: usize,
    pub format: PixelFormat,
    pub payload_bytes: usize,
}

/// C `validateJsonImageHeader` (serval_stream_validation.cpp:63-194).
pub fn validate_image_header(
    header: &Value,
    limits: &ImageLimits,
) -> Result<ImageLayout, ImageHeaderError> {
    use ImageHeaderError::*;
    // nlohmann's get<int64_t> casts an unsigned value past i64::MAX.
    let integer = |key: &str| match header.get(key) {
        Some(Value::Number(n)) => n.as_i64().or_else(|| n.as_u64().map(|u| u as i64)),
        _ => None,
    };
    let (Some(_), Some(_)) = (header.get("width"), header.get("height")) else {
        return Err(MissingField);
    };
    let (Some(width), Some(height)) = (integer("width"), integer("height")) else {
        return Err(InvalidFieldType);
    };
    let pixel_format = match header.get("pixelFormat") {
        None => None,
        Some(Value::String(f)) => Some(f.as_str()),
        Some(_) => return Err(InvalidFieldType),
    };
    let [data_size, bit_depth] = ["dataSize", "bitDepth"].map(|key| match header.get(key) {
        None => Ok(None),
        Some(_) => integer(key).map(Some).ok_or(InvalidFieldType),
    });
    let (data_size, bit_depth) = (data_size?, bit_depth?);

    let dimension_ok =
        |d: i64| d > 0 && d as u64 <= limits.max_dimension as u64 && d <= i64::from(i32::MAX);
    if !dimension_ok(width) || !dimension_ok(height) {
        return Err(InvalidDimension);
    }
    let (width, height) = (width as usize, height as usize);
    let pixels = width
        .checked_mul(height)
        .filter(|&n| n <= limits.max_pixels)
        .ok_or(PixelLimitExceeded)?;

    let mut bytes = 0;
    let mut merge = |candidate: usize| {
        if bytes != 0 && bytes != candidate {
            return Err(InconsistentPixelMetadata);
        }
        bytes = candidate;
        Ok(())
    };
    if let Some(f) = pixel_format.filter(|f| !f.is_empty()) {
        merge(match f {
            "uint8" | "UINT8" => 1,
            "uint16" | "UINT16" => 2,
            "uint32" | "UINT32" => 4,
            _ => return Err(UnsupportedPixelFormat),
        })?;
    }
    if let Some(depth) = bit_depth {
        if !matches!(depth, 8 | 16 | 32) {
            return Err(UnsupportedPixelFormat);
        }
        merge(depth as usize / 8)?;
    }
    if let Some(size) = data_size {
        if size <= 0
            || size as u64 > limits.max_payload_bytes as u64
            || !(size as u64).is_multiple_of(pixels as u64)
        {
            return Err(PayloadLimitExceeded);
        }
        let declared = size as usize / pixels;
        if !matches!(declared, 1 | 2 | 4) {
            return Err(UnsupportedPixelFormat);
        }
        merge(declared)?;
    }
    // Serval 4.1.6 always sends dataSize and bitDepth; uint16 is C's legacy
    // fallback for a header with neither.
    if bytes == 0 {
        bytes = 2;
    }
    let payload_bytes = pixels
        .checked_mul(bytes)
        .filter(|&n| n <= limits.max_payload_bytes)
        .ok_or(PayloadLimitExceeded)?;
    Ok(ImageLayout {
        width,
        height,
        format: PixelFormat::from_bytes(bytes),
        payload_bytes,
    })
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
    pub threshold_id: i32,
    pub integration_size: i32,
    pub pixels: Vec<u32>,
    /// The parsed header line, for C's PrvImgLogHeaders log.
    pub header: Value,
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
        time_axis_ms(self.bin_offset, self.bin_width, self.counts.len())
    }
}

/// `bins` left edges in milliseconds from a histogram's offset and width.
pub fn time_axis_ms(bin_offset: i32, bin_width: i32, bins: usize) -> Vec<f64> {
    (0..bins)
        .map(|i| {
            (f64::from(bin_offset) + i as f64 * f64::from(bin_width)) * TDC_CLOCK_PERIOD_SEC * 1e3
        })
        .collect()
}

#[derive(Debug, Clone, PartialEq)]
pub enum Frame {
    Image(ImageFrame),
    Histogram(HistogramFrame),
}

/// A header the decoder cannot get past. The frame is unrecoverable: the
/// payload length is unknown, so the stream has to be torn down rather than
/// guessed past. Each variant is one of C's error prints.
#[derive(Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// No newline within `MAX_HEADER` bytes.
    NoNewline,
    /// The header line is not JSON.
    Json(String),
    /// C `validateJsonImageHeader` refused the image header.
    Image(ImageHeaderError),
    /// Histogram bin count out of range.
    BinSize(i64),
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoNewline => write!(f, "no newline in the header"),
            Self::Json(e) => write!(f, "header is not JSON: {e}"),
            Self::Image(e) => write!(f, "{e}"),
            Self::BinSize(n) => write!(f, "binSize {n}"),
        }
    }
}

impl std::error::Error for DecodeError {}

/// C's bin-count guard (histogram_io.cpp:245).
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
    limits: ImageLimits,
}

impl FrameDecoder {
    pub fn new(channel: Channel) -> Self {
        Self {
            channel,
            buf: Vec::new(),
            limits: ImageLimits::detector(0, 0, 0),
        }
    }

    /// The limits the next image header is validated against; C reads them
    /// from the detector's params for every header (serval_stream.cpp:520-527).
    pub fn set_image_limits(&mut self, limits: ImageLimits) {
        self.limits = limits;
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
                return Err(DecodeError::NoNewline);
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
        let header: Value =
            serde_json::from_slice(&line[brace..]).map_err(|e| DecodeError::Json(e.to_string()))?;

        let (payload_len, frame) = match self.channel {
            Channel::Image => image_header(&header, &self.limits)?,
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
        Ok(Some(frame.finish(&payload, header)))
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
        threshold_id: i32,
        integration_size: i32,
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
    fn finish(self, payload: &[u8], header: Value) -> Frame {
        match self {
            Self::Image {
                width,
                height,
                format,
                frame_number,
                time_at_frame,
                threshold_id,
                integration_size,
            } => Frame::Image(ImageFrame {
                width,
                height,
                format,
                frame_number,
                time_at_frame,
                threshold_id,
                integration_size,
                pixels: decode_be(payload, format),
                header,
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
        PixelFormat::UInt8 => payload.iter().map(|&b| u32::from(b)).collect(),
        PixelFormat::UInt16 => payload
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| u32::from(u16::from_be_bytes([c[0], c[1]])))
            .collect(),
        PixelFormat::UInt32 => payload
            .as_chunks::<4>()
            .0
            .iter()
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

/// C `jsonImageThresholdId` (serval_stream.cpp:35-47): the first integer of
/// `thresholdID`, `thresholdId`, `thresholdIndex`, else 0.
fn threshold_id(header: &Value) -> i32 {
    ["thresholdID", "thresholdId", "thresholdIndex"]
        .iter()
        .find_map(|key| match header.get(*key) {
            Some(Value::Number(n)) if !n.is_f64() => {
                n.as_i64().or_else(|| n.as_u64().map(|v| v as i64))
            }
            _ => None,
        })
        .map_or(0, |v| v as i32)
}

fn image_header(header: &Value, limits: &ImageLimits) -> Result<(usize, Pending), DecodeError> {
    let ImageLayout {
        width,
        height,
        format,
        payload_bytes: payload,
    } = validate_image_header(header, limits).map_err(DecodeError::Image)?;
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
            threshold_id: threshold_id(header),
            integration_size: header_i64(header, "integrationSize", 0) as i32,
        },
    ))
}

fn histogram_header(header: &Value) -> Result<(usize, Pending), DecodeError> {
    let bin_size = header_i64(header, "binSize", -1);
    let Ok(bins) = usize::try_from(bin_size) else {
        return Err(DecodeError::BinSize(bin_size));
    };
    if bins == 0 || bins > MAX_BINS {
        return Err(DecodeError::BinSize(bin_size));
    }
    let bin_size = bins;
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

    /// C `jsonImageThresholdId` (serval_stream.cpp:35-47) and
    /// `j.value("integrationSize", 0)`.
    #[test]
    fn an_image_header_carries_its_threshold_and_integration_size() {
        for (fields, threshold, integration) in [
            (r#""thresholdID":1,"integrationSize":5"#, 1, 5),
            (r#""thresholdId":2"#, 2, 0),
            (r#""thresholdIndex":3"#, 3, 0),
            (r#""thresholdID":1.0,"thresholdIndex":4"#, 4, 0),
            (r#""thresholdID":"1""#, 0, 0),
            ("", 0, 0),
        ] {
            let sep = if fields.is_empty() { "" } else { "," };
            let mut bytes =
                format!(r#"{{"width":1,"height":1,"pixelFormat":"uint16"{sep}{fields}}}"#)
                    .into_bytes();
            bytes.push(b'\n');
            bytes.extend_from_slice(&[0, 7]);
            let mut d = FrameDecoder::new(Channel::Image);
            d.push(&bytes);
            let Some(Frame::Image(f)) = d.next_frame().unwrap() else {
                panic!("{fields}")
            };
            assert_eq!(
                (f.threshold_id, f.integration_size),
                (threshold, integration),
                "{fields}"
            );
        }
    }

    /// C `validateJsonImageHeader` (serval_stream_validation.cpp:63-194),
    /// one case per refusal and per storage source.
    #[test]
    fn an_image_header_is_validated_as_c_does() {
        use ImageHeaderError::*;
        let supported = ImageLimits::detector(0, 0, 0);
        let check = |fields: &str, limits: &ImageLimits| {
            let header: Value = serde_json::from_str(&format!("{{{fields}}}")).unwrap();
            validate_image_header(&header, limits).map(|l| (l.format, l.payload_bytes))
        };
        let wh = r#""width":4,"height":2"#;
        for (fields, want) in [
            (wh.to_string(), Ok((PixelFormat::UInt16, 16))),
            (
                format!(r#"{wh},"pixelFormat":"uint8""#),
                Ok((PixelFormat::UInt8, 8)),
            ),
            (
                format!(r#"{wh},"pixelFormat":"UINT32""#),
                Ok((PixelFormat::UInt32, 32)),
            ),
            (
                format!(r#"{wh},"pixelFormat":"""#),
                Ok((PixelFormat::UInt16, 16)),
            ),
            (format!(r#"{wh},"bitDepth":8"#), Ok((PixelFormat::UInt8, 8))),
            (
                format!(r#"{wh},"dataSize":32"#),
                Ok((PixelFormat::UInt32, 32)),
            ),
            (
                format!(r#"{wh},"bitDepth":16,"dataSize":16"#),
                Ok((PixelFormat::UInt16, 16)),
            ),
            (r#""width":4"#.to_string(), Err(MissingField)),
            (
                r#""width":4.0,"height":2"#.to_string(),
                Err(InvalidFieldType),
            ),
            (format!(r#"{wh},"pixelFormat":16"#), Err(InvalidFieldType)),
            (format!(r#"{wh},"dataSize":"16""#), Err(InvalidFieldType)),
            (format!(r#"{wh},"bitDepth":16.0"#), Err(InvalidFieldType)),
            (r#""width":0,"height":2"#.to_string(), Err(InvalidDimension)),
            (
                r#""width":2049,"height":1"#.to_string(),
                Err(InvalidDimension),
            ),
            (
                r#""width":2048,"height":512"#.to_string(),
                Err(PixelLimitExceeded),
            ),
            (
                format!(r#"{wh},"pixelFormat":"Uint16""#),
                Err(UnsupportedPixelFormat),
            ),
            (
                format!(r#"{wh},"bitDepth":12"#),
                Err(UnsupportedPixelFormat),
            ),
            (
                format!(r#"{wh},"dataSize":24"#),
                Err(UnsupportedPixelFormat),
            ),
            (format!(r#"{wh},"dataSize":12"#), Err(PayloadLimitExceeded)),
            (format!(r#"{wh},"dataSize":0"#), Err(PayloadLimitExceeded)),
            (
                format!(r#"{wh},"pixelFormat":"uint16","bitDepth":32"#),
                Err(InconsistentPixelMetadata),
            ),
            (
                format!(r#"{wh},"bitDepth":8,"dataSize":16"#),
                Err(InconsistentPixelMetadata),
            ),
        ] {
            assert_eq!(check(&fields, &supported), want, "{fields}");
        }
        // A detector's params tighten the limits; unset ones leave them.
        let quad = ImageLimits::detector(512, 512, 512 * 512);
        assert_eq!(
            check(r#""width":513,"height":1"#, &quad),
            Err(InvalidDimension)
        );
        let small = ImageLimits::detector(4, 2, 4);
        assert_eq!(check(wh, &small), Err(PixelLimitExceeded));
        assert_eq!(ImageLimits::detector(4096, 0, 1), supported);
    }

    #[test]
    fn uint8_pixels_decode_and_a_refused_header_is_an_error() {
        let mut bytes = br#"{"width":2,"height":1,"bitDepth":8}"#.to_vec();
        bytes.extend_from_slice(b"\n\x07\xff");
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes);
        let Some(Frame::Image(f)) = d.next_frame().unwrap() else {
            panic!()
        };
        assert_eq!((f.format, f.pixels), (PixelFormat::UInt8, vec![7, 255]));

        let mut d = FrameDecoder::new(Channel::Image);
        d.set_image_limits(ImageLimits::detector(1, 1, 1));
        d.push(br#"{"width":2,"height":1}"#);
        d.push(b"\n");
        assert_eq!(
            d.next_frame(),
            Err(DecodeError::Image(ImageHeaderError::InvalidDimension))
        );
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
