//! SIMPLON ZeroMQ stream interface, version 1 (port of `streamApi.cpp`).
//!
//! The detector PUSHes multipart messages on `tcp://HOST:9999`. The C driver
//! links libzmq and pulls the parts one `zmq_msg_recv` at a time; this port uses
//! `zeromq` (zmq.rs, pure Rust), whose `recv` hands back the whole multipart
//! message, so the part-by-part state machine collapses into a match on the
//! first part's `htype`:
//!
//! * `dheader-1.0`  — series start. 1, 2 or 8 parts depending on `header_detail`.
//! * `dimage-1.0`   — 4 parts: header, shape/type/encoding, blob, timestamps.
//! * `dseries_end-1.0` — 1 part: the series is over.
//!
//! The decoders work over byte slices and a pool, so a frame can be decoded
//! without a detector or a socket.

use epics_rs::ad_core::codec::{Codec, CodecName};
use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool};
use serde_json::Value;

use crate::bslz4;
use crate::pixels::{frame_type, le_to_host};

const ZMQ_PORT: u16 = 9999;

#[derive(Debug, PartialEq)]
pub enum StreamError {
    /// A message arrived whose `htype` is none of the three we know.
    WrongHtype(String),
    Decode(String),
    Transport(String),
    /// The pool refused the frame's array (streamApi.cpp:370).
    NoArray(u64),
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WrongHtype(h) => write!(f, "wrong header type, htype={h}"),
            Self::Decode(m) => write!(f, "{m}"),
            Self::Transport(m) => write!(f, "zmq: {m}"),
            Self::NoArray(frame) => write!(f, "failed to allocate NDArray for frame {frame}"),
        }
    }
}

impl std::error::Error for StreamError {}

type Result<T> = std::result::Result<T, StreamError>;

/// A decoded image frame.
pub struct StreamFrame {
    pub series: u64,
    pub frame: u64,
    /// `[width, height]`, in the order the detector reports the shape, and a
    /// codec when the frame is passed through still compressed.
    pub array: DriverArray,
}

impl std::fmt::Debug for StreamFrame {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StreamFrame")
            .field("series", &self.series)
            .field("frame", &self.frame)
            .field("array", &*self.array)
            .finish()
    }
}

/// What one multipart message turned out to be.
#[derive(Debug)]
pub enum StreamMessage {
    /// Series start; carries the series id.
    Header(u64),
    Image(Box<StreamFrame>),
    /// End of series (C `dseries_end`).
    End,
}

/// The `htype` of a message, without its version suffix.
///
/// C compares with a prefix (`htype.compare(0, len, "dheader")`,
/// streamApi.cpp:227), so `dheader-1.0` and any later minor version match.
fn htype_of(part: &[u8]) -> Result<String> {
    let v: Value = serde_json::from_slice(part)
        .map_err(|e| StreamError::Decode(format!("failed to parse JSON header: {e}")))?;
    v.get("htype")
        .and_then(Value::as_str)
        .map(str::to_string)
        .ok_or_else(|| StreamError::Decode("no 'htype' in stream message".into()))
}

fn json_u64(v: &Value, key: &str) -> Result<u64> {
    v.get(key)
        .and_then(Value::as_u64)
        .ok_or_else(|| StreamError::Decode(format!("unable to read '{key}' token")))
}

/// Element type of a stream frame (`"uint32"` / `"uint16"` / `"uint8"`).
fn data_type_of(name: &str) -> Result<NDDataType> {
    // C matches with strncmp against the prefix (streamApi.cpp:345-359).
    if name.starts_with("uint32") {
        Ok(NDDataType::UInt32)
    } else if name.starts_with("uint16") {
        Ok(NDDataType::UInt16)
    } else if name.starts_with("uint8") {
        Ok(NDDataType::UInt8)
    } else {
        Err(StreamError::Decode(format!("unknown dataType {name}")))
    }
}

/// The codec an `encoding` string names, or `None` for uncompressed data
/// (C `uncompress`, streamApi.cpp:112, and the pass-through branch at :394).
fn codec_of(encoding: &str) -> Result<Option<CodecName>> {
    match encoding {
        "<" => Ok(None),
        "lz4<" => Ok(Some(CodecName::LZ4)),
        "bs32-lz4<" | "bs16-lz4<" | "bs8-lz4<" => Ok(Some(CodecName::BSLZ4)),
        other => Err(StreamError::Decode(format!("unknown encoding {other}"))),
    }
}

/// Decode the `dimage_d` part and its blob into an array from `pool`
/// (C `StreamAPI::getFrame`, streamApi.cpp:302), the pixels written straight
/// into it as C writes `pData` (:378-391). `frame` names a refused alloc.
///
/// With `decompress` set the blob is decoded here; otherwise it is passed
/// through with a [`Codec`] attached for a downstream codec plugin to handle,
/// exactly as the C driver does.
///
/// UPSTREAM DEFECT (streamApi.cpp:389): C ignores `uncompress()`'s return value,
/// so a failed decode silently publishes a frame of garbage. Here the error
/// propagates and the frame is dropped.
fn decode_image(
    shape_part: &[u8],
    blob: &[u8],
    decompress: bool,
    signed: bool,
    pool: &DriverPool,
    frame: u64,
) -> Result<DriverArray> {
    let v: Value = serde_json::from_slice(shape_part)
        .map_err(|e| StreamError::Decode(format!("failed to parse image shape JSON: {e}")))?;

    let shape = v
        .get("shape")
        .and_then(Value::as_array)
        .ok_or_else(|| StreamError::Decode("unable to read 'shape' token".into()))?;
    if shape.len() < 2 {
        return Err(StreamError::Decode(format!(
            "'shape' has {} entries, need 2",
            shape.len()
        )));
    }
    let dims = [
        shape[0]
            .as_u64()
            .ok_or_else(|| StreamError::Decode("shape[0] is not a number".into()))?
            as usize,
        shape[1]
            .as_u64()
            .ok_or_else(|| StreamError::Decode("shape[1] is not a number".into()))?
            as usize,
    ];

    let type_name = v
        .get("type")
        .and_then(Value::as_str)
        .ok_or_else(|| StreamError::Decode("unable to read 'type' token".into()))?;
    let data_type = data_type_of(type_name)?;

    let encoding = v
        .get("encoding")
        .and_then(Value::as_str)
        .ok_or_else(|| StreamError::Decode("unable to read 'encoding' token".into()))?;
    let codec = codec_of(encoding)?;

    // C: uncompressedSize = shape[0]*shape[1]*elemSize.
    let n_elems = dims[0]
        .checked_mul(dims[1])
        .ok_or_else(|| StreamError::Decode("frame dimensions overflow".into()))?;
    let uncompressed = n_elems * data_type.element_size();
    let nd_dims = vec![NDDimension::new(dims[0]), NDDimension::new(dims[1])];
    let pixels = |pool: &DriverPool| {
        pool.alloc(nd_dims.clone(), frame_type(data_type, signed))
            .map_err(|_| StreamError::NoArray(frame))
    };

    let Some(codec_name) = codec else {
        // Uncompressed: the blob is the pixels.
        if blob.len() < uncompressed {
            return Err(StreamError::Decode(format!(
                "uncompressed frame is {} bytes, expected {uncompressed}",
                blob.len()
            )));
        }
        let mut array = pixels(pool)?;
        let mut fill = array.fill();
        fill.data_mut()
            .as_u8_slice_mut()
            .copy_from_slice(&blob[..uncompressed]);
        le_to_host(fill.data_mut().view_mut());
        return Ok(array);
    };

    if decompress {
        if !matches!(codec_name, CodecName::LZ4 | CodecName::BSLZ4) {
            return Err(StreamError::Decode(format!(
                "unhandled codec {codec_name:?}"
            )));
        }
        let mut array = pixels(pool)?;
        let mut fill = array.fill();
        let out = fill.data_mut().as_u8_slice_mut();
        if codec_name == CodecName::LZ4 {
            let n = lz4_flex::block::decompress_into(blob, out)
                .map_err(|e| StreamError::Decode(format!("LZ4 decompress failed: {e}")))?;
            if n != uncompressed {
                return Err(StreamError::Decode(format!(
                    "decompressed to {n} bytes, expected {uncompressed}"
                )));
            }
        } else {
            bslz4::decode_with_header_into(blob, out, data_type.element_size())
                .map_err(|e| StreamError::Decode(e.to_string()))?;
        }
        le_to_host(fill.data_mut().view_mut());
        return Ok(array);
    }

    // Pass-through: hand the compressed bytes downstream with a codec attached.
    // BSLZ4 payloads shed their 12-byte header first — that is the framing the
    // NDCodec plugin expects (C strips it at streamApi.cpp:401).
    let payload: &[u8] = match codec_name {
        CodecName::BSLZ4 => blob
            .get(bslz4::HEADER_LEN..)
            .ok_or_else(|| StreamError::Decode("bslz4 payload shorter than its header".into()))?,
        _ => blob,
    };
    let codec = Codec {
        name: codec_name,
        compressed_size: payload.len(),
        level: 0,
        shuffle: 0,
        compressor: 0,
        // SignedData relabels the type the codec plugin decompresses to.
        original_data_type: frame_type(data_type, signed),
    };
    // A compressed frame holds its payload bytes, which its dims do not size.
    let mut array = pool
        .alloc_sized(nd_dims, NDDataType::UInt8, payload.len())
        .map_err(|_| StreamError::NoArray(frame))?;
    let mut fill = array.fill();
    fill.data_mut().as_u8_slice_mut().copy_from_slice(payload);
    fill.set_codec(Some(codec));
    Ok(array)
}

/// Classify and decode one multipart stream message.
///
/// `parts` are the frames of a single ZeroMQ multipart message.
/// The extra parts (config, flatfield, pixelmask, countrate) are the
/// detector's calibration dump; C reads and discards them.
fn dheader_series(first: &[u8]) -> Result<u64> {
    let v: Value = serde_json::from_slice(first)
        .map_err(|e| StreamError::Decode(format!("failed to parse dheader: {e}")))?;
    json_u64(&v, "series")
}

pub fn decode_message(
    parts: &[&[u8]],
    decompress: bool,
    signed: bool,
    pool: &DriverPool,
) -> Result<StreamMessage> {
    let first = parts
        .first()
        .ok_or_else(|| StreamError::Decode("empty zmq message".into()))?;
    let htype = htype_of(first)?;

    if htype.starts_with("dseries_end") {
        return Ok(StreamMessage::End);
    }

    if htype.starts_with("dheader") {
        return Ok(StreamMessage::Header(dheader_series(first)?));
    }

    if htype.starts_with("dimage") {
        if parts.len() < 3 {
            return Err(StreamError::Decode(format!(
                "dimage message has {} parts, need at least 3",
                parts.len()
            )));
        }
        let v: Value = serde_json::from_slice(first)
            .map_err(|e| StreamError::Decode(format!("failed to parse dimage header: {e}")))?;
        let series = json_u64(&v, "series")?;
        let frame = json_u64(&v, "frame")?;
        let array = decode_image(parts[1], parts[2], decompress, signed, pool, frame)?;
        return Ok(StreamMessage::Image(Box::new(StreamFrame {
            series,
            frame,
            array,
        })));
    }

    Err(StreamError::WrongHtype(htype))
}

/// ZeroMQ PULL socket on the detector's stream port (C `StreamAPI`).
pub struct StreamApi {
    socket: zeromq::PullSocket,
}

impl StreamApi {
    pub async fn connect(hostname: &str) -> Result<Self> {
        Self::connect_to(&format!("tcp://{hostname}:{ZMQ_PORT}")).await
    }

    pub(crate) async fn connect_to(addr: &str) -> Result<Self> {
        use zeromq::Socket;
        let mut socket = zeromq::PullSocket::new();
        socket
            .connect(addr)
            .await
            .map_err(|e| StreamError::Transport(format!("connect {addr}: {e}")))?;
        Ok(Self { socket })
    }

    /// Receive the next message, or `Ok(None)` on timeout (C polls the socket
    /// with a timeout in seconds). It is decoded apart, so the caller can read
    /// StreamDecompress after the frame arrives, where C's getFrame does.
    pub async fn recv(&mut self, timeout: std::time::Duration) -> Result<Option<Received>> {
        use zeromq::SocketRecv;

        match tokio::time::timeout(timeout, self.socket.recv()).await {
            Err(_) => Ok(None),
            Ok(Ok(m)) => Ok(Some(Received(m))),
            Ok(Err(e)) => Err(StreamError::Transport(e.to_string())),
        }
    }
}

/// One multipart message off the stream socket, not yet decoded.
pub struct Received(zeromq::ZmqMessage);

impl Received {
    /// C `StreamAPI::getHeader`: anything but a `dheader`, a series end
    /// included, is a wrong header type.
    pub fn header(&self) -> Result<u64> {
        let first = self
            .0
            .iter()
            .next()
            .ok_or_else(|| StreamError::Decode("empty zmq message".into()))?;
        let htype = htype_of(first)?;
        if !htype.starts_with("dheader") {
            return Err(StreamError::WrongHtype(htype));
        }
        dheader_series(first)
    }

    pub fn decode(
        &self,
        decompress: bool,
        signed: bool,
        pool: &DriverPool,
    ) -> Result<StreamMessage> {
        let parts: Vec<&[u8]> = self.0.iter().map(|f| f.as_ref()).collect();
        decode_message(&parts, decompress, signed, pool)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDataView};

    fn decode(parts: &[&[u8]], decompress: bool) -> Result<StreamMessage> {
        decode_message(parts, decompress, false, &DriverPool::standalone(0))
    }

    fn sizes(array: &DriverArray) -> Vec<usize> {
        array.dims.iter().map(|d| d.size).collect()
    }

    fn dimage_header(series: u64, frame: u64) -> Vec<u8> {
        format!(r#"{{"htype":"dimage-1.0","series":{series},"frame":{frame},"hash":"x"}}"#)
            .into_bytes()
    }

    fn shape_part(w: usize, h: usize, ty: &str, enc: &str, size: usize) -> Vec<u8> {
        format!(
            r#"{{"htype":"dimage_d-1.0","shape":[{w},{h}],"type":"{ty}","encoding":"{enc}","size":{size}}}"#
        )
        .into_bytes()
    }

    #[test]
    fn decodes_a_series_header() {
        let part = br#"{"htype":"dheader-1.0","series":17,"header_detail":"basic"}"#;
        match decode(&[part.as_slice()], true).unwrap() {
            StreamMessage::Header(series) => assert_eq!(series, 17),
            _ => panic!("expected a header"),
        }
    }

    #[test]
    fn decodes_a_series_end() {
        let part = br#"{"htype":"dseries_end-1.0","series":17}"#;
        assert!(matches!(
            decode(&[part.as_slice()], true).unwrap(),
            StreamMessage::End
        ));
    }

    #[test]
    fn an_unknown_htype_is_a_stray_packet() {
        let part = br#"{"htype":"dconfig-1.0"}"#;
        assert_eq!(
            decode(&[part.as_slice()], true).unwrap_err(),
            StreamError::WrongHtype("dconfig-1.0".into())
        );
    }

    #[test]
    fn decodes_an_uncompressed_image() {
        let pixels: Vec<u16> = (0..12u16).collect();
        let blob: Vec<u8> = pixels.iter().flat_map(|p| p.to_le_bytes()).collect();
        let shape = shape_part(4, 3, "uint16", "<", blob.len());
        let header = dimage_header(5, 2);
        let times = br#"{"htype":"dconfig-1.0","start_time":0,"stop_time":1}"#;

        let msg = decode(&[&header, &shape, &blob, times.as_slice()], true).unwrap();
        let StreamMessage::Image(f) = msg else {
            panic!("expected an image");
        };
        assert_eq!(f.series, 5);
        assert_eq!(f.frame, 2);
        assert_eq!(sizes(&f.array), [4, 3]);
        assert!(f.array.codec.is_none());
        let NDDataView::U16(got) = f.array.data().view() else {
            panic!("expected U16 pixels");
        };
        assert_eq!(got, pixels);
    }

    #[test]
    fn decodes_an_lz4_image() {
        let pixels: Vec<u32> = (0..64u32).map(|i| i * 11).collect();
        let raw: Vec<u8> = pixels.iter().flat_map(|p| p.to_le_bytes()).collect();
        let blob = lz4_flex::block::compress(&raw);
        let shape = shape_part(8, 8, "uint32", "lz4<", blob.len());
        let header = dimage_header(1, 0);

        let StreamMessage::Image(f) = decode(&[&header, &shape, &blob], true).unwrap() else {
            panic!("expected an image");
        };
        assert!(
            f.array.codec.is_none(),
            "decompressed frames carry no codec"
        );
        let NDDataView::U32(got) = f.array.data().view() else {
            panic!("expected U32 pixels");
        };
        assert_eq!(got, pixels);
    }

    /// Build a bslz4 stream blob the way the detector does: the 12-byte header
    /// followed by the bitshuffle/LZ4 block stream.
    fn bslz4_blob(pixels: &[u32]) -> Vec<u8> {
        let src = NDArray::with_data(
            vec![NDDimension::new(pixels.len())],
            NDDataBuffer::from_slice(pixels),
        );
        let compressed = ad_plugins_rs::codec::compress_bslz4(&src).unwrap();
        let mut blob = Vec::new();
        blob.extend_from_slice(&((pixels.len() * 4) as u64).to_be_bytes());
        blob.extend_from_slice(&((bslz4::default_block_size(4) * 4) as u32).to_be_bytes());
        blob.extend_from_slice(compressed.data().as_u8_slice());
        blob
    }

    #[test]
    fn decodes_a_bslz4_image() {
        let pixels: Vec<u32> = (0..4096u32).map(|i| i % 977).collect();
        let blob = bslz4_blob(&pixels);
        let shape = shape_part(64, 64, "uint32", "bs32-lz4<", blob.len());
        let header = dimage_header(3, 9);

        let StreamMessage::Image(f) = decode(&[&header, &shape, &blob], true).unwrap() else {
            panic!("expected an image");
        };
        assert_eq!(sizes(&f.array), [64, 64]);
        let NDDataView::U32(got) = f.array.data().view() else {
            panic!("expected U32 pixels");
        };
        assert_eq!(got, pixels);
    }

    #[test]
    fn passthrough_strips_the_bslz4_header_and_attaches_a_codec() {
        let pixels: Vec<u32> = (0..4096u32).map(|i| i % 31).collect();
        let blob = bslz4_blob(&pixels);
        let shape = shape_part(64, 64, "uint32", "bs32-lz4<", blob.len());
        let header = dimage_header(3, 9);

        let StreamMessage::Image(f) = decode(&[&header, &shape, &blob], false).unwrap() else {
            panic!("expected an image");
        };
        let codec = f
            .array
            .codec
            .clone()
            .expect("pass-through must carry a codec");
        assert_eq!(codec.name, CodecName::BSLZ4);
        assert_eq!(codec.original_data_type, NDDataType::UInt32);
        // The 12-byte header is gone; the rest is the block stream verbatim.
        let NDDataView::U8(payload) = f.array.data().view() else {
            panic!("expected raw bytes");
        };
        assert_eq!(payload.len(), blob.len() - bslz4::HEADER_LEN);
        assert_eq!(payload, &blob[bslz4::HEADER_LEN..]);
        assert_eq!(codec.compressed_size, payload.len());
    }

    #[test]
    fn passthrough_of_lz4_keeps_the_whole_blob() {
        let pixels: Vec<u16> = (0..256u16).collect();
        let raw: Vec<u8> = pixels.iter().flat_map(|p| p.to_le_bytes()).collect();
        let blob = lz4_flex::block::compress(&raw);
        let shape = shape_part(16, 16, "uint16", "lz4<", blob.len());
        let header = dimage_header(1, 1);

        let StreamMessage::Image(f) = decode(&[&header, &shape, &blob], false).unwrap() else {
            panic!("expected an image");
        };
        let codec = f
            .array
            .codec
            .clone()
            .expect("pass-through must carry a codec");
        assert_eq!(codec.name, CodecName::LZ4);
        let NDDataView::U8(payload) = f.array.data().view() else {
            panic!("expected raw bytes");
        };
        assert_eq!(payload, blob.as_slice());
    }

    #[test]
    fn a_corrupt_compressed_frame_is_an_error_not_garbage_pixels() {
        // The upstream defect: C ignores uncompress()'s failure and publishes
        // whatever happened to be in the buffer.
        let blob = vec![0xFFu8; 64];
        let shape = shape_part(8, 8, "uint32", "lz4<", blob.len());
        let header = dimage_header(1, 1);
        assert!(decode(&[&header, &shape, &blob], true).is_err());
    }

    #[test]
    fn unknown_encodings_and_types_are_rejected() {
        let blob = vec![0u8; 16];
        let header = dimage_header(1, 1);

        let shape = shape_part(4, 4, "uint32", "zstd<", blob.len());
        assert!(decode(&[&header, &shape, &blob], true).is_err());

        let shape = shape_part(4, 4, "float32", "<", blob.len());
        assert!(decode(&[&header, &shape, &blob], true).is_err());
    }

    /// C relabels the array after getFrame (eigerDetector.cpp:1519-1533).
    #[test]
    fn signed_data_allocates_the_signed_type_over_the_same_bits() {
        let raw: Vec<u8> = [1u16, 0xffff]
            .iter()
            .flat_map(|p| p.to_le_bytes())
            .collect();
        let header = dimage_header(1, 1);
        let pool = DriverPool::standalone(0);

        let shape = shape_part(2, 1, "uint16", "<", raw.len());
        let msg = decode_message(&[&header, &shape, &raw], true, true, &pool).unwrap();
        let StreamMessage::Image(f) = msg else {
            panic!("expected an image");
        };
        assert!(matches!(f.array.data().view(), NDDataView::I16(p) if p == [1, -1]));

        let blob = lz4_flex::block::compress(&raw);
        let shape = shape_part(2, 1, "uint16", "lz4<", blob.len());
        let msg = decode_message(&[&header, &shape, &blob], false, true, &pool).unwrap();
        let StreamMessage::Image(f) = msg else {
            panic!("expected an image");
        };
        let codec = f.array.codec.as_ref().expect("passed through");
        assert_eq!(codec.original_data_type, NDDataType::Int16);
    }

    #[test]
    fn a_refused_array_names_its_frame() {
        let raw = vec![0u8; 16];
        let shape = shape_part(4, 2, "uint16", "<", raw.len());
        let header = dimage_header(1, 9);
        let pool = DriverPool::standalone(1);
        assert_eq!(
            decode_message(&[&header, &shape, &raw], true, false, &pool).unwrap_err(),
            StreamError::NoArray(9)
        );
    }

    #[test]
    fn a_short_uncompressed_blob_is_rejected() {
        // 4x4 uint32 needs 64 bytes; give it 16.
        let blob = vec![0u8; 16];
        let shape = shape_part(4, 4, "uint32", "<", blob.len());
        let header = dimage_header(1, 1);
        assert!(decode(&[&header, &shape, &blob], true).is_err());
    }
}
