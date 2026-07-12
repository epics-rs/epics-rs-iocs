//! Turning a `GetImage` payload into an NDArray buffer.
//!
//! The server sends the pixels in its native byte order — little-endian on the
//! Windows PC PSLViewer runs on, which is also what C's `memcpy` into the
//! NDArray assumed.

use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType};

use crate::protocol::{ImageHeader, ProtocolError};

/// Decode the payload of a frame whose header has already been parsed.
///
/// [`crate::protocol::parse_image_header`] has already checked that the
/// announced byte count matches the announced geometry, so a payload of the
/// announced length is exactly one frame.
pub fn decode_payload(header: &ImageHeader, payload: &[u8]) -> Result<NDDataBuffer, ProtocolError> {
    if payload.len() != header.data_len {
        return Err(ProtocolError::ImageSizeMismatch {
            announced: header.data_len,
            expected: payload.len(),
        });
    }
    Ok(match header.data_type {
        NDDataType::UInt8 => NDDataBuffer::U8(payload.to_vec()),
        NDDataType::UInt16 => NDDataBuffer::U16(
            payload
                .chunks_exact(2)
                .map(|c| u16::from_le_bytes([c[0], c[1]]))
                .collect(),
        ),
        NDDataType::UInt32 => NDDataBuffer::U32(
            payload
                .chunks_exact(4)
                .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect(),
        ),
        NDDataType::Float32 => NDDataBuffer::F32(
            payload
                .chunks_exact(4)
                .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect(),
        ),
        // parse_image_header only ever produces the four types above.
        other => {
            return Err(ProtocolError::UnknownImageMode(format!("{other:?}")));
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::parse_image_header;

    #[test]
    fn decode_payload_reads_16_bit_pixels_little_endian() {
        let header = parse_image_header(b"I;16;2;1;4;").unwrap();
        let buf = decode_payload(&header, &[0x01, 0x00, 0xff, 0x0f]).unwrap();
        let NDDataBuffer::U16(pixels) = buf else {
            panic!("a 16-bit frame must decode to a U16 buffer");
        };
        assert_eq!(pixels, vec![1, 0x0fff]);
    }

    #[test]
    fn decode_payload_reads_8_bit_pixels() {
        let header = parse_image_header(b"L;2;1;2;").unwrap();
        let buf = decode_payload(&header, &[7, 9]).unwrap();
        assert!(matches!(buf, NDDataBuffer::U8(p) if p == vec![7, 9]));
    }

    #[test]
    fn decode_payload_reads_32_bit_pixels() {
        let header = parse_image_header(b"I;1;1;4;").unwrap();
        let buf = decode_payload(&header, &[0x04, 0x03, 0x02, 0x01]).unwrap();
        assert!(matches!(buf, NDDataBuffer::U32(p) if p == vec![0x01020304]));
    }

    #[test]
    fn decode_payload_reads_float_pixels() {
        let header = parse_image_header(b"F;1;1;4;").unwrap();
        let buf = decode_payload(&header, &1.5f32.to_le_bytes()).unwrap();
        assert!(matches!(buf, NDDataBuffer::F32(p) if p == vec![1.5]));
    }

    #[test]
    fn decode_payload_reads_rgb_pixels_interleaved() {
        let header = parse_image_header(b"RGB;2;1;6;").unwrap();
        let buf = decode_payload(&header, &[1, 2, 3, 4, 5, 6]).unwrap();
        assert!(matches!(buf, NDDataBuffer::U8(p) if p == vec![1, 2, 3, 4, 5, 6]));
    }

    #[test]
    fn decode_payload_rejects_a_short_payload() {
        let header = parse_image_header(b"I;16;2;1;4;").unwrap();
        assert!(matches!(
            decode_payload(&header, &[0, 0]),
            Err(ProtocolError::ImageSizeMismatch { .. })
        ));
    }
}
