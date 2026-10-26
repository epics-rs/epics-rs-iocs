//! Turning a `GetImage` payload into an NDArray buffer.
//!
//! The server sends the pixels in its native byte order — little-endian on the
//! Windows PC PSLViewer runs on, which is also what C's `memcpy` into the
//! NDArray assumed.

use epics_rs::ad_core::ndarray::NDDataViewMut;

use crate::protocol::{ImageHeader, ProtocolError};

/// Check the payload of a frame whose header has already been parsed, before
/// a buffer is allocated for it.
///
/// [`crate::protocol::parse_image_header`] has already checked that the
/// announced byte count matches the announced geometry, so a payload of the
/// announced length is exactly one frame.
pub fn check_payload(header: &ImageHeader, payload: &[u8]) -> Result<(), ProtocolError> {
    if payload.len() != header.data_len {
        return Err(ProtocolError::ImageSizeMismatch {
            announced: header.data_len,
            expected: payload.len(),
        });
    }
    Ok(())
}

/// Copy a checked payload into `out`, a buffer of its header's geometry and
/// data type: C's `memcpy` into `pData` (PSL.cpp:573-596).
pub fn decode_into(payload: &[u8], out: NDDataViewMut<'_>) {
    match out {
        NDDataViewMut::U8(d) => d.copy_from_slice(payload),
        NDDataViewMut::U16(d) => le_into(payload, d, u16::from_le_bytes),
        NDDataViewMut::U32(d) => le_into(payload, d, u32::from_le_bytes),
        NDDataViewMut::F32(d) => le_into(payload, d, f32::from_le_bytes),
        // parse_image_header only ever produces the four types above.
        other => unreachable!("a PSL frame is never {:?}", other.data_type()),
    }
}

fn le_into<const N: usize, T>(src: &[u8], dst: &mut [T], pixel: fn([u8; N]) -> T) {
    for (d, c) in dst.iter_mut().zip(src.as_chunks::<N>().0) {
        *d = pixel(*c);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::parse_image_header;
    use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataView};

    fn decode_payload(header: &ImageHeader, payload: &[u8]) -> Result<NDDataBuffer, ProtocolError> {
        check_payload(header, payload)?;
        let count = header.dims().iter().product();
        // Stale bytes, as a reused pool buffer holds.
        let mut buf = NDDataBuffer::zeros(header.data_type, count);
        buf.as_u8_slice_mut().fill(0xA5);
        decode_into(payload, buf.view_mut());
        Ok(buf)
    }

    #[test]
    fn decode_payload_reads_16_bit_pixels_little_endian() {
        let header = parse_image_header(b"I;16;2;1;4;").unwrap();
        let buf = decode_payload(&header, &[0x01, 0x00, 0xff, 0x0f]).unwrap();
        let NDDataView::U16(pixels) = buf.view() else {
            panic!("a 16-bit frame must decode to a U16 buffer");
        };
        assert_eq!(pixels, vec![1, 0x0fff]);
    }

    #[test]
    fn decode_payload_reads_8_bit_pixels() {
        let header = parse_image_header(b"L;2;1;2;").unwrap();
        let buf = decode_payload(&header, &[7, 9]).unwrap();
        assert!(matches!(buf.view(), NDDataView::U8(p) if p == vec![7, 9]));
    }

    #[test]
    fn decode_payload_reads_32_bit_pixels() {
        let header = parse_image_header(b"I;1;1;4;").unwrap();
        let buf = decode_payload(&header, &[0x04, 0x03, 0x02, 0x01]).unwrap();
        assert!(matches!(buf.view(), NDDataView::U32(p) if p == vec![0x01020304]));
    }

    #[test]
    fn decode_payload_reads_float_pixels() {
        let header = parse_image_header(b"F;1;1;4;").unwrap();
        let buf = decode_payload(&header, &1.5f32.to_le_bytes()).unwrap();
        assert!(matches!(buf.view(), NDDataView::F32(p) if p == vec![1.5]));
    }

    #[test]
    fn decode_payload_reads_rgb_pixels_interleaved() {
        let header = parse_image_header(b"RGB;2;1;6;").unwrap();
        let buf = decode_payload(&header, &[1, 2, 3, 4, 5, 6]).unwrap();
        assert!(matches!(buf.view(), NDDataView::U8(p) if p == vec![1, 2, 3, 4, 5, 6]));
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
