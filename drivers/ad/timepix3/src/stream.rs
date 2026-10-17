//! The `tcp://` preview stream: a JSON header line, then the frame's binary
//! payload (port of `serval_stream.cpp` and `histogram_io.cpp`'s framing).
//!
//! Serval's `jsonimage` / `jsonhisto` channels are a *raw TCP byte stream* (not
//! HTTP): for every frame it writes one line of ASCII JSON terminated by `\n`,
//! immediately followed by `width*height*bytesPerPixel` (or `binSize*4`) bytes
//! of **big-endian** payload, then a `\n` terminator. There is no length
//! prefix outside the header, so a decoder that miscounts consumed bytes by
//! even one desyncs the stream permanently.

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

/// C `FrameResult` (serval_stream_framing.h): the ways `append` / `next`
/// refuse the stream. Each ends the worker's run.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameError {
    /// The payload-size resolver refused the header; its message.
    InvalidHeader(String),
    InvalidTrailer,
    HeaderTooLarge,
    PayloadTooLarge,
    BufferLimitExceeded,
}

impl std::fmt::Display for FrameError {
    /// C `frameResultMessage`, then `": "` and the resolver's message when
    /// there is one (serval_stream.cpp:607-610).
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidHeader(e) if e.is_empty() => f.write_str("invalid JSON frame header"),
            Self::InvalidHeader(e) => write!(f, "invalid JSON frame header: {e}"),
            Self::InvalidTrailer => f.write_str("invalid or missing frame payload terminator"),
            Self::HeaderTooLarge => f.write_str("frame header exceeds the configured limit"),
            Self::PayloadTooLarge => f.write_str("frame payload exceeds the configured limit"),
            Self::BufferLimitExceeded => f.write_str("stream buffer exceeds the configured limit"),
        }
    }
}

/// C `EndOfStreamStatus`: where the bytes buffered at EOF stop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndOfStream {
    Clean,
    TruncatedHeader,
    TruncatedPayload,
    TruncatedTrailer,
}

impl EndOfStream {
    /// C `endOfStreamStatusMessage`.
    pub fn message(self) -> &'static str {
        match self {
            Self::Clean => "clean frame boundary",
            Self::TruncatedHeader => "truncated frame header",
            Self::TruncatedPayload => "truncated frame payload",
            Self::TruncatedTrailer => "truncated frame payload terminator",
        }
    }
}

/// C `MAX_BUFFER_SIZE` (ADTimePix.h:415): the header and trailing-byte
/// limits of every stream's `FrameBuffer`, and the size of each socket read
/// (serval_stream.cpp:505, histogram_io.cpp:971) — a larger read can carry more
/// trailing bytes than the limit admits.
pub const MAX_BUFFER_SIZE: usize = 32768;
/// C `maxHistogramBins` (histogram_io.cpp:973).
const MAX_BINS: usize = 1_000_000;

/// A header the resolver accepted and the payload it declared.
struct Resolved<T> {
    header_bytes: usize,
    payload_bytes: usize,
    header: T,
}

/// C `ADTimePix3Stream::FrameBuffer` (serval_stream_framing.cpp): a header
/// line, its declared payload and a `\n` terminator, consumed together. The
/// resolver runs once per header and no payload byte is searched for a
/// delimiter.
pub struct FrameBuffer<T> {
    max_header: usize,
    max_payload: usize,
    max_buffered: usize,
    bytes: Vec<u8>,
    current: Option<Resolved<T>>,
}

impl<T> FrameBuffer<T> {
    pub fn new(max_header: usize, max_payload: usize, max_trailing: usize) -> Self {
        Self {
            max_header,
            max_payload,
            max_buffered: max_header
                .saturating_add(max_payload)
                .saturating_add(max_trailing)
                .saturating_add(2),
            bytes: Vec::new(),
            current: None,
        }
    }

    pub fn append(&mut self, data: &[u8]) -> Result<(), FrameError> {
        if self.bytes.len() > self.max_buffered || data.len() > self.max_buffered - self.bytes.len()
        {
            return Err(FrameError::BufferLimitExceeded);
        }
        self.bytes.extend_from_slice(data);
        Ok(())
    }

    /// The next frame's resolved header and payload, `None` until it is
    /// complete. `resolve` sees the header line without its `\r\n` and
    /// returns the payload size.
    pub fn next(
        &mut self,
        resolve: impl FnOnce(&[u8]) -> Result<(usize, T), String>,
    ) -> Result<Option<(T, Vec<u8>)>, FrameError> {
        if self.current.is_none() {
            let Some(newline) = self.bytes.iter().position(|&b| b == b'\n') else {
                return if self.bytes.len() > self.max_header {
                    Err(FrameError::HeaderTooLarge)
                } else {
                    Ok(None)
                };
            };
            if newline > self.max_header {
                return Err(FrameError::HeaderTooLarge);
            }
            let text = match self.bytes[..newline] {
                [ref head @ .., b'\r'] => head,
                ref line => line,
            };
            let (payload_bytes, header) = resolve(text).map_err(FrameError::InvalidHeader)?;
            if payload_bytes > self.max_payload {
                return Err(FrameError::PayloadTooLarge);
            }
            self.current = Some(Resolved {
                header_bytes: newline + 1,
                payload_bytes,
                header,
            });
        }
        let Some(current) = &self.current else {
            unreachable!("resolved above");
        };
        let body = current.header_bytes + current.payload_bytes;
        if self.bytes.len() <= body {
            return Ok(None);
        }
        if self.bytes[body] != b'\n' {
            return Err(FrameError::InvalidTrailer);
        }
        let Some(current) = self.current.take() else {
            unreachable!("checked above");
        };
        let payload = self.bytes[current.header_bytes..body].to_vec();
        self.bytes.drain(..=body);
        Ok(Some((current.header, payload)))
    }

    pub fn end_of_stream(&self) -> EndOfStream {
        match &self.current {
            _ if self.bytes.is_empty() => EndOfStream::Clean,
            None => EndOfStream::TruncatedHeader,
            Some(c) if self.bytes.len() < c.header_bytes + c.payload_bytes => {
                EndOfStream::TruncatedPayload
            }
            Some(_) => EndOfStream::TruncatedTrailer,
        }
    }

    pub fn buffered(&self) -> usize {
        self.bytes.len()
    }
}

/// Which channel a decoder is reading — the two share the framing but not the
/// header schema.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Image,
    Histogram,
}

/// A channel's `FrameBuffer` with its payload-size resolver: C
/// `runJsonImageTcpWorker` (serval_stream.cpp:507-538) and
/// `prvHstWorkerThread` (histogram_io.cpp:973-996).
pub struct FrameDecoder {
    channel: Channel,
    buffer: FrameBuffer<(Pending, Value, bool)>,
    limits: ImageLimits,
}

impl FrameDecoder {
    pub fn new(channel: Channel) -> Self {
        let max_payload = match channel {
            Channel::Image => ImageLimits::detector(0, 0, 0).max_payload_bytes,
            Channel::Histogram => MAX_BINS * 4,
        };
        Self {
            channel,
            buffer: FrameBuffer::new(MAX_BUFFER_SIZE, max_payload, MAX_BUFFER_SIZE),
            limits: ImageLimits::detector(0, 0, 0),
        }
    }

    /// The limits the next image header is validated against; C reads them
    /// from the detector's params for every header (serval_stream.cpp:520-527).
    pub fn set_image_limits(&mut self, limits: ImageLimits) {
        self.limits = limits;
    }

    pub fn push(&mut self, bytes: &[u8]) -> Result<(), FrameError> {
        self.buffer.append(bytes)
    }

    pub fn buffered(&self) -> usize {
        self.buffer.buffered()
    }

    pub fn end_of_stream(&self) -> EndOfStream {
        self.buffer.end_of_stream()
    }

    /// The next complete frame, or `None` when more bytes are needed.
    pub fn next_frame(&mut self) -> Result<Option<Frame>, FrameError> {
        let (channel, limits) = (self.channel, &self.limits);
        loop {
            let resolved = self.buffer.next(|line| {
                let header = nlohmann_parse(line).filter(Value::is_object);
                let (payload, pending) = match channel {
                    Channel::Image => {
                        let header = header
                            .as_ref()
                            .ok_or("malformed or non-object jsonimage header")?;
                        image_header(header, limits).map_err(|e| e.to_string())?
                    }
                    Channel::Histogram => histogram_header(header.as_ref())?,
                };
                Ok((
                    payload,
                    (pending, header.unwrap_or_default(), line_has_json(line)),
                ))
            })?;
            match resolved {
                None => return Ok(None),
                Some(((pending, header, true), payload)) => {
                    return Ok(Some(pending.finish(&payload, header)));
                }
                // C's line handler returns true without touching the frame.
                Some(((_, _, false), _)) => continue,
            }
        }
    }
}

/// `json::parse`: nlohmann skips a UTF-8 BOM at the very start, which
/// serde_json refuses.
fn nlohmann_parse(line: &[u8]) -> Option<Value> {
    let line = line.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(line);
    serde_json::from_slice(line).ok()
}

/// The line handlers' rescan of an accepted header (serval_stream.cpp:94-101,
/// :732-741, histogram_io.cpp:185-192): skip non-printable bytes (a byte past
/// 0x7f is a negative `char`, so below 32), and find a `{` where it stops. A
/// leading space is printable, so such a header passes the resolver and the
/// frame is dropped here.
fn line_has_json(line: &[u8]) -> bool {
    line.iter()
        .find(|&&b| b == b'{' || (32..=126).contains(&b))
        .is_some_and(|&b| b == b'{')
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
/// UPSTREAM DEFECT (serval_stream.cpp:210, :218, :848, :860,
/// histogram_io.cpp:331): C calls `__builtin_bswap16/32` unconditionally, which
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

fn image_header(
    header: &Value,
    limits: &ImageLimits,
) -> Result<(usize, Pending), ImageHeaderError> {
    let ImageLayout {
        width,
        height,
        format,
        payload_bytes: payload,
    } = validate_image_header(header, limits)?;
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

/// C `prvHstWorkerThread`'s payload-size resolver (histogram_io.cpp:978-995)
/// over a header that parsed as an object, or `None`.
fn histogram_header(header: Option<&Value>) -> Result<(usize, Pending), String> {
    let Some((header, Value::Number(n))) = header.and_then(|h| Some((h, h.get("binSize")?))) else {
        return Err("malformed jsonhisto header or invalid binSize".into());
    };
    if n.is_f64() {
        return Err("malformed jsonhisto header or invalid binSize".into());
    }
    // A u64 past i64::MAX is negative as C's `get<long long>`.
    let bin_size = n
        .as_i64()
        .and_then(|v| usize::try_from(v).ok())
        .filter(|&v| v > 0 && v <= MAX_BINS)
        .ok_or("binSize is outside the supported range")?;
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
/// `parseTcpPath`, serval_stream.cpp:647-684): everything up to the first
/// `@` is dropped, the host ends at the first `:` and may be empty, and the
/// port is read as `std::stoi`. `Err(Some(line))` is the ERR text C prints;
/// `Err(None)` a path C refuses silently.
///
/// `listen@` means *Serval* listens; the IOC is the client either way.
pub fn parse_tcp_path(path: &str) -> Result<(String, u16), Option<String>> {
    let rest = path.strip_prefix("tcp://").ok_or(None)?;
    let rest = rest.split_once('@').map_or(rest, |(_, after)| after);
    let (host, port) = rest.split_once(':').ok_or(None)?;
    let Some(n) = stoi(port) else {
        return Err(Some(format!("Failed to parse port: {port}")));
    };
    match u16::try_from(n) {
        Ok(port) if port != 0 => Ok((host.to_string(), port)),
        _ => Err(Some(format!("Invalid port number: {n} (must be 1-65535)"))),
    }
}

/// `std::stoi`: leading C whitespace, an optional sign, at least one digit,
/// and anything after the digits ignored; `None` where it throws.
fn stoi(s: &str) -> Option<i32> {
    let s = s.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let (negative, s) = match s.as_bytes().first() {
        Some(b'-') => (true, &s[1..]),
        Some(b'+') => (false, &s[1..]),
        _ => (false, s),
    };
    let digits = s.bytes().take_while(u8::is_ascii_digit).count();
    if digits == 0 {
        return None;
    }
    let mut n: i64 = 0;
    for d in s[..digits].bytes() {
        n = n.checked_mul(10)?.checked_add(i64::from(d - b'0'))?;
    }
    i32::try_from(if negative { -n } else { n }).ok()
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
        out.push(b'\n');
        out
    }

    #[test]
    fn a_whole_image_frame_decodes() {
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&image_bytes(2, 2, 7, &[1, 2, 3, 0xbeef])).unwrap();
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
            d2.push(&bytes[..split]).unwrap();
            assert_eq!(d2.next_frame().unwrap(), None, "split at {split}");
            d2.push(&bytes[split..]).unwrap();
            let Some(Frame::Image(f)) = d2.next_frame().unwrap() else {
                panic!("split at {split}: no frame")
            };
            assert_eq!(f.pixels, vec![0x0102, 0x0304], "split at {split}");
        }
        // A byte at a time, too.
        for b in &bytes {
            d.push(std::slice::from_ref(b)).unwrap();
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
        d.push(&bytes).unwrap();

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

    /// The resolver parses as nlohmann (a leading BOM accepted); the line
    /// handler then drops a frame whose header starts with a printable
    /// non-`{`, such as a space (serval_stream.cpp:94-101, :732-741,
    /// histogram_io.cpp:185-192).
    #[test]
    fn a_header_prefix_is_handled_as_c_line_handlers() {
        let prefixed = |prefix: &[u8], frame: i32| {
            let mut b = prefix.to_vec();
            b.extend(image_bytes(1, 1, frame, &[frame as u16]));
            b
        };
        let mut bytes = prefixed(b" ", 1);
        bytes.extend(prefixed(b"\xEF\xBB\xBF", 2));
        bytes.extend(prefixed(b"\t", 3));
        bytes.extend(prefixed(b"\xEF\xBB\xBF ", 4));
        bytes.extend(prefixed(b"", 5));
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes).unwrap();
        let mut seen = Vec::new();
        while let Some(Frame::Image(f)) = d.next_frame().unwrap() {
            seen.push(f.frame_number);
        }
        assert_eq!(seen, vec![2, 3, 5]);
        assert_eq!(d.buffered(), 0);

        let histogram = |prefix: &str, frame: i32| {
            let mut b = format!(r#"{prefix}{{"binSize":1,"frameNumber":{frame}}}"#).into_bytes();
            b.push(b'\n');
            b.extend_from_slice(&1u32.to_be_bytes());
            b.push(b'\n');
            b
        };
        let mut d = FrameDecoder::new(Channel::Histogram);
        d.push(&[histogram(" ", 1), histogram("\u{feff}", 2)].concat())
            .unwrap();
        let Some(Frame::Histogram(h)) = d.next_frame().unwrap() else {
            panic!("no histogram frame")
        };
        assert_eq!(h.frame_number, 2);
        assert_eq!(d.next_frame().unwrap(), None);
    }

    #[test]
    fn a_payload_byte_that_looks_like_a_newline_is_not_a_frame_boundary() {
        // 0x0a0a contains two '\n' bytes: a decoder that re-scans consumed
        // payload for the next header splits here.
        let mut bytes = image_bytes(1, 1, 1, &[0x0a0a]);
        bytes.extend(image_bytes(1, 1, 2, &[0x0001]));
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes).unwrap();
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
        bytes.push(b'\n');
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes).unwrap();
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
            bytes.extend_from_slice(&[0, 7, b'\n']);
            let mut d = FrameDecoder::new(Channel::Image);
            d.push(&bytes).unwrap();
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
        bytes.extend_from_slice(b"\n\x07\xff\n");
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(&bytes).unwrap();
        let Some(Frame::Image(f)) = d.next_frame().unwrap() else {
            panic!()
        };
        assert_eq!((f.format, f.pixels), (PixelFormat::UInt8, vec![7, 255]));

        let mut d = FrameDecoder::new(Channel::Image);
        d.set_image_limits(ImageLimits::detector(1, 1, 1));
        d.push(br#"{"width":2,"height":1}"#).unwrap();
        d.push(b"\n").unwrap();
        assert_eq!(
            d.next_frame(),
            Err(FrameError::InvalidHeader(
                ImageHeaderError::InvalidDimension.to_string()
            ))
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
        bytes.push(b'\n');
        let mut d = FrameDecoder::new(Channel::Histogram);
        d.push(&bytes).unwrap();
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
            d.push(&bytes).unwrap();
            assert!(
                d.next_frame().is_err(),
                "{}",
                String::from_utf8_lossy(header)
            );
        }
        let mut d = FrameDecoder::new(Channel::Histogram);
        d.push(b"{\"binSize\":0}\n").unwrap();
        assert!(d.next_frame().is_err());
    }

    /// C `FrameBuffer` (serval_stream_framing.cpp:55-127): the header line
    /// must be the JSON object, a `\r` before its newline is dropped, and the
    /// payload must be followed by `\n`.
    #[test]
    fn a_frame_is_framed_as_c_frame_buffer() {
        let mut d = FrameDecoder::new(Channel::Image);
        d.push(b"garbage{\"width\":1,\"height\":1}\n").unwrap();
        assert_eq!(
            d.next_frame().unwrap_err().to_string(),
            "invalid JSON frame header: malformed or non-object jsonimage header"
        );

        let mut d = FrameDecoder::new(Channel::Image);
        d.push(b"{\"width\":1,\"height\":1}\r\n\x00\x07\n").unwrap();
        let Some(Frame::Image(f)) = d.next_frame().unwrap() else {
            panic!()
        };
        assert_eq!(f.pixels, vec![7]);

        let mut d = FrameDecoder::new(Channel::Image);
        d.push(b"{\"width\":1,\"height\":1}\n\x00\x07").unwrap();
        assert_eq!(d.next_frame(), Ok(None));
        assert_eq!(d.end_of_stream(), EndOfStream::TruncatedTrailer);
        d.push(b"{").unwrap();
        assert_eq!(
            d.next_frame().unwrap_err().to_string(),
            "invalid or missing frame payload terminator"
        );
    }

    /// C `endOfStreamStatus` at each point of a frame.
    #[test]
    fn the_end_of_stream_status_names_where_the_bytes_stop() {
        let mut d = FrameDecoder::new(Channel::Image);
        assert_eq!(d.end_of_stream(), EndOfStream::Clean);
        d.push(b"{\"width\":1").unwrap();
        assert_eq!(d.next_frame(), Ok(None));
        assert_eq!(d.end_of_stream(), EndOfStream::TruncatedHeader);
        d.push(b",\"height\":1}\n\x00").unwrap();
        assert_eq!(d.next_frame(), Ok(None));
        assert_eq!(d.end_of_stream(), EndOfStream::TruncatedPayload);
        d.push(b"\x07\n").unwrap();
        assert!(d.next_frame().unwrap().is_some());
        assert_eq!(d.end_of_stream(), EndOfStream::Clean);
        assert_eq!(
            EndOfStream::TruncatedPayload.message(),
            "truncated frame payload"
        );
    }

    /// The header and buffer limits (serval_stream_framing.cpp:26-53, :62-73).
    #[test]
    fn a_stream_past_the_limits_is_refused() {
        let mut d = FrameDecoder::new(Channel::Histogram);
        d.push(&vec![b'x'; MAX_BUFFER_SIZE + 1]).unwrap();
        assert_eq!(d.next_frame(), Err(FrameError::HeaderTooLarge));

        let mut b = FrameBuffer::<()>::new(4, 4, 4);
        b.append(&[0; 14]).unwrap();
        assert_eq!(b.append(&[0]), Err(FrameError::BufferLimitExceeded));
        let mut b = FrameBuffer::<()>::new(16, 4, 16);
        b.append(b"h\n").unwrap();
        assert_eq!(b.next(|_| Ok((5, ()))), Err(FrameError::PayloadTooLarge));
    }

    /// C `prvHstWorkerThread`'s resolver messages (histogram_io.cpp:981-992).
    #[test]
    fn a_histogram_header_is_refused_with_c_messages() {
        for (header, message) in [
            (&b"[1]"[..], "malformed jsonhisto header or invalid binSize"),
            (b"{}", "malformed jsonhisto header or invalid binSize"),
            (
                b"{\"binSize\":2.0}",
                "malformed jsonhisto header or invalid binSize",
            ),
            (b"{\"binSize\":0}", "binSize is outside the supported range"),
            (
                b"{\"binSize\":1000001}",
                "binSize is outside the supported range",
            ),
            (
                b"{\"binSize\":18446744073709551615}",
                "binSize is outside the supported range",
            ),
        ] {
            let mut d = FrameDecoder::new(Channel::Histogram);
            d.push(header).unwrap();
            d.push(b"\n").unwrap();
            assert_eq!(
                d.next_frame(),
                Err(FrameError::InvalidHeader(message.into())),
                "{}",
                String::from_utf8_lossy(header)
            );
        }
    }

    #[test]
    fn tcp_paths_parse_with_and_without_listen() {
        let ok = |h: &str, p| Ok((h.to_string(), p));
        assert_eq!(
            parse_tcp_path("tcp://listen@localhost:8089"),
            ok("localhost", 8089)
        );
        assert_eq!(
            parse_tcp_path("tcp://127.0.0.1:8451"),
            ok("127.0.0.1", 8451)
        );
        assert_eq!(parse_tcp_path("http://localhost:8089"), Err(None));
        assert_eq!(parse_tcp_path("tcp://localhost"), Err(None));
        assert_eq!(parse_tcp_path("file:///data"), Err(None));
    }

    /// C `parseTcpPath` (serval_stream.cpp:658-683): the first `@` and the
    /// first `:` split, the host may be empty, and `std::stoi` reads the port.
    #[test]
    fn a_tcp_path_splits_and_parses_as_c() {
        let ok = |h: &str, p| Ok((h.to_string(), p));
        assert_eq!(parse_tcp_path("tcp://a@b@host:1"), ok("b@host", 1));
        assert_eq!(
            parse_tcp_path("tcp://[::1]:8089"),
            Err(Some("Failed to parse port: :1]:8089".into()))
        );
        assert_eq!(parse_tcp_path("tcp://host:1:2"), ok("host", 1));
        assert_eq!(parse_tcp_path("tcp://:8089"), ok("", 8089));
        assert_eq!(parse_tcp_path("tcp://h: +8089x"), ok("h", 8089));
        assert_eq!(
            parse_tcp_path("tcp://h:0"),
            Err(Some("Invalid port number: 0 (must be 1-65535)".into()))
        );
        assert_eq!(
            parse_tcp_path("tcp://h:-5"),
            Err(Some("Invalid port number: -5 (must be 1-65535)".into()))
        );
        assert_eq!(
            parse_tcp_path("tcp://h:70000"),
            Err(Some("Invalid port number: 70000 (must be 1-65535)".into()))
        );
        assert_eq!(
            parse_tcp_path("tcp://h:x1"),
            Err(Some("Failed to parse port: x1".into()))
        );
        assert_eq!(
            parse_tcp_path("tcp://h:2147483648"),
            Err(Some("Failed to parse port: 2147483648".into()))
        );
    }
}
