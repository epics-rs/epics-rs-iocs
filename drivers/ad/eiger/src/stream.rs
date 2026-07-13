//! SIMPLON ZeroMQ stream interface, version 1 (port of `streamApi.cpp`).
//!
//! The detector PUSHes multipart messages on tcp://<host>:9999. The C driver
//! links libzmq and pulls the parts one `zmq_msg_recv` at a time; this port uses
//! `zeromq` (zmq.rs, pure Rust), whose `recv` hands back the whole multipart
//! message, so the part-by-part state machine collapses into a match on the
//! first part's `htype`:
//!
//! * `dheader-1.0`  — series start. 1, 2 or 8 parts depending on `header_detail`.
//! * `dimage-1.0`   — 4 parts: header, shape/type/encoding, blob, timestamps.
//! * `dseries_end-1.0` — 1 part: the series is over.
//!
//! The decoders are pure functions over byte slices so a frame can be decoded
//! without a detector or a socket.

use epics_rs::ad_core::codec::{Codec, CodecName};
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType};
use serde_json::Value;

use crate::bslz4;

const ZMQ_PORT: u16 = 9999;

#[derive(Debug, PartialEq)]
pub enum StreamError {
    /// A message arrived whose `htype` is none of the three we know.
    WrongHtype(String),
    Decode(String),
    Transport(String),
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WrongHtype(h) => write!(f, "wrong header type, htype={h}"),
            Self::Decode(m) => write!(f, "{m}"),
            Self::Transport(m) => write!(f, "zmq: {m}"),
        }
    }
}

impl std::error::Error for StreamError {}

type Result<T> = std::result::Result<T, StreamError>;

/// The pixels of one `dimage` message, before the series/frame ids are attached.
#[derive(Debug)]
pub struct DecodedImage {
    /// `[width, height]`, in the order the detector reports the shape.
    pub dims: [usize; 2],
    pub data: NDDataBuffer,
    /// Set when the frame is passed through still compressed.
    pub codec: Option<Codec>,
}

/// A decoded image frame.
#[derive(Debug)]
pub struct StreamFrame {
    pub series: u64,
    pub frame: u64,
    /// `[width, height]`, in the order the detector reports the shape.
    pub dims: [usize; 2],
    pub data: NDDataBuffer,
    /// Set when the frame is passed through still compressed.
    pub codec: Option<Codec>,
}

/// What one multipart message turned out to be.
#[derive(Debug)]
pub enum StreamMessage {
    /// Series start; carries the series id.
    Header(u64),
    Image(StreamFrame),
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

/// Reinterpret decoded bytes as the frame's element type.
///
/// The detector is little-endian and so is every host this runs on; the C driver
/// memcpy's straight into the NDArray, which assumes the same.
fn buffer_from_bytes(bytes: &[u8], data_type: NDDataType) -> Result<NDDataBuffer> {
    let elem = data_type.element_size();
    if !bytes.len().is_multiple_of(elem) {
        return Err(StreamError::Decode(format!(
            "frame is {} bytes, not a multiple of the {elem}-byte element",
            bytes.len()
        )));
    }
    Ok(match data_type {
        NDDataType::UInt8 => NDDataBuffer::U8(bytes.to_vec()),
        NDDataType::UInt16 => NDDataBuffer::U16(
            bytes
                .chunks_exact(2)
                .map(|c| u16::from_le_bytes([c[0], c[1]]))
                .collect(),
        ),
        NDDataType::UInt32 => NDDataBuffer::U32(
            bytes
                .chunks_exact(4)
                .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect(),
        ),
        other => {
            return Err(StreamError::Decode(format!(
                "unsupported stream data type {other:?}"
            )));
        }
    })
}

/// Decode the `dimage_d` part and its blob into pixels
/// (C `StreamAPI::getFrame`, streamApi.cpp:302).
///
/// With `decompress` set the blob is decoded here; otherwise it is passed
/// through with a [`Codec`] attached for a downstream codec plugin to handle,
/// exactly as the C driver does.
///
/// UPSTREAM DEFECT (streamApi.cpp:389): C ignores `uncompress()`'s return value,
/// so a failed decode silently publishes a frame of garbage. Here the error
/// propagates and the frame is dropped.
pub fn decode_image(shape_part: &[u8], blob: &[u8], decompress: bool) -> Result<DecodedImage> {
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

    let Some(codec_name) = codec else {
        // Uncompressed: the blob is the pixels.
        if blob.len() < uncompressed {
            return Err(StreamError::Decode(format!(
                "uncompressed frame is {} bytes, expected {uncompressed}",
                blob.len()
            )));
        }
        return Ok(DecodedImage {
            dims,
            data: buffer_from_bytes(&blob[..uncompressed], data_type)?,
            codec: None,
        });
    };

    if decompress {
        let bytes = match codec_name {
            CodecName::LZ4 => lz4_flex::block::decompress(blob, uncompressed)
                .map_err(|e| StreamError::Decode(format!("LZ4 decompress failed: {e}")))?,
            CodecName::BSLZ4 => {
                bslz4::decode_with_header(blob, data_type.element_size(), Some(uncompressed))
                    .map_err(|e| StreamError::Decode(e.to_string()))?
            }
            other => {
                return Err(StreamError::Decode(format!("unhandled codec {other:?}")));
            }
        };
        if bytes.len() != uncompressed {
            return Err(StreamError::Decode(format!(
                "decompressed to {} bytes, expected {uncompressed}",
                bytes.len()
            )));
        }
        return Ok(DecodedImage {
            dims,
            data: buffer_from_bytes(&bytes, data_type)?,
            codec: None,
        });
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
        original_data_type: data_type,
    };
    Ok(DecodedImage {
        dims,
        data: NDDataBuffer::U8(payload.to_vec()),
        codec: Some(codec),
    })
}

/// Classify and decode one multipart stream message.
///
/// `parts` are the frames of a single ZeroMQ multipart message.
pub fn decode_message(parts: &[&[u8]], decompress: bool) -> Result<StreamMessage> {
    let first = parts
        .first()
        .ok_or_else(|| StreamError::Decode("empty zmq message".into()))?;
    let htype = htype_of(first)?;

    if htype.starts_with("dseries_end") {
        return Ok(StreamMessage::End);
    }

    if htype.starts_with("dheader") {
        let v: Value = serde_json::from_slice(first)
            .map_err(|e| StreamError::Decode(format!("failed to parse dheader: {e}")))?;
        // The extra parts (config, flatfield, pixelmask, countrate) are the
        // detector's calibration dump; C reads and discards them.
        return Ok(StreamMessage::Header(json_u64(&v, "series")?));
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
        let image = decode_image(parts[1], parts[2], decompress)?;
        return Ok(StreamMessage::Image(StreamFrame {
            series,
            frame,
            dims: image.dims,
            data: image.data,
            codec: image.codec,
        }));
    }

    Err(StreamError::WrongHtype(htype))
}

/// ZeroMQ PULL socket on the detector's stream port (C `StreamAPI`).
pub struct StreamApi {
    socket: zeromq::PullSocket,
}

impl StreamApi {
    pub async fn connect(hostname: &str) -> Result<Self> {
        use zeromq::Socket;
        let mut socket = zeromq::PullSocket::new();
        let addr = format!("tcp://{hostname}:{ZMQ_PORT}");
        socket
            .connect(&addr)
            .await
            .map_err(|e| StreamError::Transport(format!("connect {addr}: {e}")))?;
        Ok(Self { socket })
    }

    /// Receive and decode the next message, or `Ok(None)` on timeout
    /// (C polls the socket with a timeout in seconds).
    pub async fn recv(
        &mut self,
        timeout: std::time::Duration,
        decompress: bool,
    ) -> Result<Option<StreamMessage>> {
        use zeromq::SocketRecv;

        let msg = match tokio::time::timeout(timeout, self.socket.recv()).await {
            Err(_) => return Ok(None),
            Ok(Ok(m)) => m,
            Ok(Err(e)) => return Err(StreamError::Transport(e.to_string())),
        };
        let parts: Vec<&[u8]> = msg.iter().map(|f| f.as_ref()).collect();
        decode_message(&parts, decompress).map(Some)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::{NDArray, NDDimension};

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
        match decode_message(&[part.as_slice()], true).unwrap() {
            StreamMessage::Header(series) => assert_eq!(series, 17),
            _ => panic!("expected a header"),
        }
    }

    #[test]
    fn decodes_a_series_end() {
        let part = br#"{"htype":"dseries_end-1.0","series":17}"#;
        assert!(matches!(
            decode_message(&[part.as_slice()], true).unwrap(),
            StreamMessage::End
        ));
    }

    #[test]
    fn an_unknown_htype_is_a_stray_packet() {
        let part = br#"{"htype":"dconfig-1.0"}"#;
        assert_eq!(
            decode_message(&[part.as_slice()], true).unwrap_err(),
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

        let msg = decode_message(&[&header, &shape, &blob, times.as_slice()], true).unwrap();
        let StreamMessage::Image(f) = msg else {
            panic!("expected an image");
        };
        assert_eq!(f.series, 5);
        assert_eq!(f.frame, 2);
        assert_eq!(f.dims, [4, 3]);
        assert!(f.codec.is_none());
        let NDDataBuffer::U16(got) = f.data else {
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

        let StreamMessage::Image(f) = decode_message(&[&header, &shape, &blob], true).unwrap()
        else {
            panic!("expected an image");
        };
        assert!(f.codec.is_none(), "decompressed frames carry no codec");
        let NDDataBuffer::U32(got) = f.data else {
            panic!("expected U32 pixels");
        };
        assert_eq!(got, pixels);
    }

    /// Build a bslz4 stream blob the way the detector does: the 12-byte header
    /// followed by the bitshuffle/LZ4 block stream.
    fn bslz4_blob(pixels: &[u32]) -> Vec<u8> {
        let src = NDArray::with_data(
            vec![NDDimension::new(pixels.len())],
            NDDataBuffer::U32(pixels.to_vec()),
        );
        let compressed = ad_plugins_rs::codec::compress_bslz4(&src);
        let mut blob = Vec::new();
        blob.extend_from_slice(&((pixels.len() * 4) as u64).to_be_bytes());
        blob.extend_from_slice(&((bslz4::default_block_size(4) * 4) as u32).to_be_bytes());
        blob.extend_from_slice(compressed.data.as_u8_slice());
        blob
    }

    #[test]
    fn decodes_a_bslz4_image() {
        let pixels: Vec<u32> = (0..4096u32).map(|i| i % 977).collect();
        let blob = bslz4_blob(&pixels);
        let shape = shape_part(64, 64, "uint32", "bs32-lz4<", blob.len());
        let header = dimage_header(3, 9);

        let StreamMessage::Image(f) = decode_message(&[&header, &shape, &blob], true).unwrap()
        else {
            panic!("expected an image");
        };
        assert_eq!(f.dims, [64, 64]);
        let NDDataBuffer::U32(got) = f.data else {
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

        let StreamMessage::Image(f) = decode_message(&[&header, &shape, &blob], false).unwrap()
        else {
            panic!("expected an image");
        };
        let codec = f.codec.expect("pass-through must carry a codec");
        assert_eq!(codec.name, CodecName::BSLZ4);
        assert_eq!(codec.original_data_type, NDDataType::UInt32);
        // The 12-byte header is gone; the rest is the block stream verbatim.
        let NDDataBuffer::U8(payload) = &f.data else {
            panic!("expected raw bytes");
        };
        assert_eq!(payload.len(), blob.len() - bslz4::HEADER_LEN);
        assert_eq!(payload.as_slice(), &blob[bslz4::HEADER_LEN..]);
        assert_eq!(codec.compressed_size, payload.len());
    }

    #[test]
    fn passthrough_of_lz4_keeps_the_whole_blob() {
        let pixels: Vec<u16> = (0..256u16).collect();
        let raw: Vec<u8> = pixels.iter().flat_map(|p| p.to_le_bytes()).collect();
        let blob = lz4_flex::block::compress(&raw);
        let shape = shape_part(16, 16, "uint16", "lz4<", blob.len());
        let header = dimage_header(1, 1);

        let StreamMessage::Image(f) = decode_message(&[&header, &shape, &blob], false).unwrap()
        else {
            panic!("expected an image");
        };
        let codec = f.codec.expect("pass-through must carry a codec");
        assert_eq!(codec.name, CodecName::LZ4);
        let NDDataBuffer::U8(payload) = &f.data else {
            panic!("expected raw bytes");
        };
        assert_eq!(payload.as_slice(), blob.as_slice());
    }

    #[test]
    fn a_corrupt_compressed_frame_is_an_error_not_garbage_pixels() {
        // The upstream defect: C ignores uncompress()'s failure and publishes
        // whatever happened to be in the buffer.
        let blob = vec![0xFFu8; 64];
        let shape = shape_part(8, 8, "uint32", "lz4<", blob.len());
        let header = dimage_header(1, 1);
        assert!(decode_message(&[&header, &shape, &blob], true).is_err());
    }

    #[test]
    fn unknown_encodings_and_types_are_rejected() {
        let blob = vec![0u8; 16];
        let header = dimage_header(1, 1);

        let shape = shape_part(4, 4, "uint32", "zstd<", blob.len());
        assert!(decode_message(&[&header, &shape, &blob], true).is_err());

        let shape = shape_part(4, 4, "float32", "<", blob.len());
        assert!(decode_message(&[&header, &shape, &blob], true).is_err());
    }

    #[test]
    fn a_short_uncompressed_blob_is_rejected() {
        // 4x4 uint32 needs 64 bytes; give it 16.
        let blob = vec![0u8; 16];
        let shape = shape_part(4, 4, "uint32", "<", blob.len());
        let header = dimage_header(1, 1);
        assert!(decode_message(&[&header, &shape, &blob], true).is_err());
    }
}
