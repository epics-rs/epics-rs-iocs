//! Pixel-payload decoding: byte order and the Y flip.
//!
//! The Merlin FPGA writes pixels big-endian with the origin at the bottom
//! left; areaDetector wants host order with the origin at the top left. C did
//! this in `copyToNDArray8/16/32` and `copyProfileToNDArray32`.

use epics_rs::ad_core::ndarray::NDDataBuffer;

use crate::protocol::MpxError;

/// Copy the binary payload of a data frame into an NDArray buffer, flipping
/// the Y axis and swapping byte order when the detector needs it.
///
/// `offset` is the payload start, from the beginning of the frame body.
///
/// C indexed the source row as `(dims[1] - y)`, which read one row past the
/// payload on `y == 0` and never read source row 0. The correct flip is
/// `(height - 1 - y)`.
pub fn decode_image(
    body: &[u8],
    offset: usize,
    width: usize,
    height: usize,
    pixel_depth: i32,
    swap: bool,
) -> Result<NDDataBuffer, MpxError> {
    let bpp = match pixel_depth {
        8 => 1usize,
        16 => 2,
        32 => 4,
        _ => return Err(MpxError::Malformed),
    };
    let needed = width
        .checked_mul(height)
        .and_then(|n| n.checked_mul(bpp))
        .ok_or(MpxError::Malformed)?;
    // C trusted the header's geometry and copied unconditionally; a short or
    // lying frame walked off the end of the read buffer.
    if offset > body.len() || body.len() - offset < needed {
        return Err(MpxError::BadBodySize((offset + needed) as i64));
    }
    let src = &body[offset..offset + needed];

    let row_bytes = width * bpp;
    let mut out: Vec<u8> = Vec::with_capacity(needed);
    for y in 0..height {
        let sy = height - 1 - y;
        out.extend_from_slice(&src[sy * row_bytes..(sy + 1) * row_bytes]);
    }

    Ok(match bpp {
        1 => NDDataBuffer::U8(out),
        2 => NDDataBuffer::U16(
            out.chunks_exact(2)
                .map(|c| {
                    let b = [c[0], c[1]];
                    if swap {
                        u16::from_be_bytes(b)
                    } else {
                        u16::from_le_bytes(b)
                    }
                })
                .collect(),
        ),
        _ => NDDataBuffer::U32(
            out.chunks_exact(4)
                .map(|c| {
                    let b = [c[0], c[1], c[2], c[3]];
                    if swap {
                        u32::from_be_bytes(b)
                    } else {
                        u32::from_le_bytes(b)
                    }
                })
                .collect(),
        ),
    })
}

/// The X and Y profiles carried by a `PR1` frame.
pub struct Profiles {
    pub x: Vec<i32>,
    pub y: Vec<i32>,
    /// The two profiles packed into one `max(x,y) * 2` NDArray, as C did.
    pub image: NDDataBuffer,
}

/// Decode a profile frame payload: `width` then `height` 64-bit counts,
/// truncated to 32 bits (C `copyProfileToNDArray32`).
///
/// The Y profile is reversed so that it shares the image's top-left origin —
/// C's loop counted `y` down but advanced its write pointer up, so despite the
/// "Invert the Y profile" comment it never inverted anything.
pub fn decode_profiles(
    body: &[u8],
    offset: usize,
    width: usize,
    height: usize,
    swap: bool,
) -> Result<Profiles, MpxError> {
    let needed = width
        .checked_add(height)
        .and_then(|n| n.checked_mul(8))
        .ok_or(MpxError::Malformed)?;
    if offset > body.len() || body.len() - offset < needed {
        return Err(MpxError::BadBodySize((offset + needed) as i64));
    }
    let words: Vec<u32> = body[offset..offset + needed]
        .chunks_exact(8)
        .map(|c| {
            let b: [u8; 8] = c.try_into().expect("chunks_exact(8)");
            let v = if swap {
                u64::from_be_bytes(b)
            } else {
                u64::from_le_bytes(b)
            };
            v as u32
        })
        .collect();

    let x: Vec<i32> = words[..width].iter().map(|v| *v as i32).collect();
    let y: Vec<i32> = words[width..].iter().rev().map(|v| *v as i32).collect();

    // C allocated a max(width, height) x 2 NDArray and filled it row-major.
    let stride = width.max(height);
    let mut image = vec![0u32; stride * 2];
    for (i, v) in words[..width].iter().enumerate() {
        image[i] = *v;
    }
    for (i, v) in words[width..].iter().rev().enumerate() {
        image[stride + i] = *v;
    }

    Ok(Profiles {
        x,
        y,
        image: NDDataBuffer::U32(image),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 2x3 (w x h) 8-bit image, rows tagged by their source row index.
    fn image8() -> Vec<u8> {
        let mut v = b"HDR,".to_vec(); // 4-byte stand-in header
        v.extend_from_slice(&[0, 1, 10, 11, 20, 21]);
        v
    }

    #[test]
    fn decode_image8_flips_y_without_reading_past_the_payload() {
        let buf = decode_image(&image8(), 4, 2, 3, 8, true).unwrap();
        let NDDataBuffer::U8(px) = buf else {
            panic!("expected U8")
        };
        // Source rows are (0,1) (10,11) (20,21); output must start with the
        // last source row and end with the first.
        assert_eq!(px, vec![20, 21, 10, 11, 0, 1]);
    }

    #[test]
    fn decode_image16_swaps_big_endian_pixels() {
        // 1x2 image, pixels 0x0102 and 0x0304 big-endian.
        let body = [b'X', 0x01, 0x02, 0x03, 0x04];
        let buf = decode_image(&body, 1, 1, 2, 16, true).unwrap();
        let NDDataBuffer::U16(px) = buf else {
            panic!("expected U16")
        };
        // Y-flipped: bottom row (0x0304) first.
        assert_eq!(px, vec![0x0304, 0x0102]);
    }

    #[test]
    fn decode_image16_keeps_host_order_when_detector_does_not_swap() {
        let body = [b'X', 0x01, 0x02, 0x03, 0x04];
        let buf = decode_image(&body, 1, 1, 2, 16, false).unwrap();
        let NDDataBuffer::U16(px) = buf else {
            panic!("expected U16")
        };
        assert_eq!(px, vec![0x0403, 0x0201]);
    }

    #[test]
    fn decode_image32_swaps_big_endian_pixels() {
        let body = [b'X', 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x03, 0x04];
        let buf = decode_image(&body, 1, 1, 2, 32, true).unwrap();
        let NDDataBuffer::U32(px) = buf else {
            panic!("expected U32")
        };
        assert_eq!(px, vec![0x0304, 0x0102]);
    }

    #[test]
    fn decode_image_rejects_short_payload() {
        // 2x3 16-bit needs 12 payload bytes; only 4 are present.
        let body = [b'X', 1, 2, 3, 4];
        assert!(matches!(
            decode_image(&body, 1, 2, 3, 16, true),
            Err(MpxError::BadBodySize(_))
        ));
    }

    #[test]
    fn decode_image_rejects_offset_past_the_body() {
        let body = [b'X', 1, 2];
        assert!(matches!(
            decode_image(&body, 99, 1, 1, 8, true),
            Err(MpxError::BadBodySize(_))
        ));
    }

    #[test]
    fn decode_image_rejects_unsupported_depth() {
        let body = vec![0u8; 64];
        assert!(matches!(
            decode_image(&body, 0, 2, 2, 12, true),
            Err(MpxError::Malformed)
        ));
    }

    #[test]
    fn decode_profiles_splits_x_and_reverses_y() {
        // width 2, height 3: five 64-bit big-endian counts.
        let mut body = vec![b'P'];
        for v in [1u64, 2, 10, 20, 30] {
            body.extend_from_slice(&v.to_be_bytes());
        }
        let p = decode_profiles(&body, 1, 2, 3, true).unwrap();
        assert_eq!(p.x, vec![1, 2]);
        assert_eq!(p.y, vec![30, 20, 10]);
        let NDDataBuffer::U32(img) = p.image else {
            panic!("expected U32")
        };
        // stride = max(2,3) = 3: row 0 is X (padded), row 1 is the flipped Y.
        assert_eq!(img, vec![1, 2, 0, 30, 20, 10]);
    }

    #[test]
    fn decode_profiles_rejects_short_payload() {
        let body = vec![0u8; 9];
        assert!(matches!(
            decode_profiles(&body, 1, 2, 3, true),
            Err(MpxError::BadBodySize(_))
        ));
    }
}
