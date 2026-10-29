//! Pixel-payload decoding: byte order and the Y flip.
//!
//! The Merlin FPGA writes pixels big-endian with the origin at the bottom
//! left; areaDetector wants host order with the origin at the top left. C did
//! this in `copyToNDArray8/16/32` and `copyProfileToNDArray32`.

use epics_rs::ad_core::ndarray::{NDDataType, NDDataViewMut};

use crate::protocol::MpxError;

/// The binary payload of a data frame, checked against its header's geometry.
pub struct ImagePayload<'a> {
    src: &'a [u8],
    width: usize,
    height: usize,
    data_type: NDDataType,
    swap: bool,
}

/// Check that `body` holds a `width x height` frame of `pixel_depth` bits
/// from `offset`, the payload start.
pub fn image_payload(
    body: &[u8],
    offset: usize,
    width: usize,
    height: usize,
    pixel_depth: i32,
    swap: bool,
) -> Result<ImagePayload<'_>, MpxError> {
    let (bpp, data_type) = match pixel_depth {
        8 => (1usize, NDDataType::UInt8),
        16 => (2, NDDataType::UInt16),
        32 => (4, NDDataType::UInt32),
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
    Ok(ImagePayload {
        src: &body[offset..offset + needed],
        width,
        height,
        data_type,
        swap,
    })
}

impl ImagePayload<'_> {
    pub fn data_type(&self) -> NDDataType {
        self.data_type
    }

    /// Copy the payload into `out`, a `width x height` buffer of
    /// [`Self::data_type`], flipping the Y axis and swapping byte order when
    /// the detector needs it.
    ///
    /// C indexed the source row as `(dims[1] - y)`, which read one row past
    /// the payload on `y == 0` and never read source row 0. The correct flip
    /// is `(height - 1 - y)`.
    pub fn decode_into(&self, out: NDDataViewMut<'_>) {
        let (w, h, swap) = (self.width, self.height, self.swap);
        match out {
            NDDataViewMut::U8(d) => flip_into(self.src, d, w, h, |[b]: [u8; 1]| b),
            NDDataViewMut::U16(d) => flip_into(
                self.src,
                d,
                w,
                h,
                if swap {
                    u16::from_be_bytes
                } else {
                    u16::from_le_bytes
                },
            ),
            NDDataViewMut::U32(d) => flip_into(
                self.src,
                d,
                w,
                h,
                if swap {
                    u32::from_be_bytes
                } else {
                    u32::from_le_bytes
                },
            ),
            _ => unreachable!("decode_into is handed a {:?} buffer", self.data_type),
        }
    }
}

/// Row `y` of `dst` takes source row `height - 1 - y`, pixel by pixel.
fn flip_into<const N: usize, T>(
    src: &[u8],
    dst: &mut [T],
    width: usize,
    height: usize,
    pixel: impl Fn([u8; N]) -> T,
) {
    for y in 0..height {
        let sy = height - 1 - y;
        let row = &src[sy * width * N..(sy + 1) * width * N];
        for (d, c) in dst[y * width..(y + 1) * width]
            .iter_mut()
            .zip(row.as_chunks::<N>().0)
        {
            *d = pixel(*c);
        }
    }
}

/// The X and Y profiles carried by a `PR1` frame.
pub struct Profiles {
    pub x: Vec<i32>,
    pub y: Vec<i32>,
}

impl Profiles {
    /// Pack both profiles into one `max(x,y) x 2` NDUInt32 buffer, X in row
    /// 0 and Y in row 1, zero padded.
    pub fn pack_into(&self, image: &mut [u32]) {
        let (row_x, row_y) = image.split_at_mut(image.len() / 2);
        for (row, profile) in [(row_x, &self.x), (row_y, &self.y)] {
            let (head, pad) = row.split_at_mut(profile.len());
            for (d, v) in head.iter_mut().zip(profile) {
                *d = *v as u32;
            }
            pad.fill(0);
        }
    }
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
    let mut counts = body[offset..offset + needed]
        .as_chunks::<8>()
        .0
        .iter()
        .map(|c| {
            let b: [u8; 8] = *c;
            let v = if swap {
                u64::from_be_bytes(b)
            } else {
                u64::from_le_bytes(b)
            };
            v as u32 as i32
        });
    let x: Vec<i32> = counts.by_ref().take(width).collect();
    let mut y: Vec<i32> = counts.collect();
    y.reverse();
    Ok(Profiles { x, y })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataView};

    fn decode_image(
        body: &[u8],
        offset: usize,
        width: usize,
        height: usize,
        pixel_depth: i32,
        swap: bool,
    ) -> Result<NDDataBuffer, MpxError> {
        let payload = image_payload(body, offset, width, height, pixel_depth, swap)?;
        // Stale bytes, as a reused pool buffer holds.
        let mut buf = NDDataBuffer::zeros(payload.data_type(), width * height);
        buf.as_u8_slice_mut().fill(0xA5);
        payload.decode_into(buf.view_mut());
        Ok(buf)
    }

    /// 2x3 (w x h) 8-bit image, rows tagged by their source row index.
    fn image8() -> Vec<u8> {
        let mut v = b"HDR,".to_vec(); // 4-byte stand-in header
        v.extend_from_slice(&[0, 1, 10, 11, 20, 21]);
        v
    }

    #[test]
    fn decode_image8_flips_y_without_reading_past_the_payload() {
        let buf = decode_image(&image8(), 4, 2, 3, 8, true).unwrap();
        let NDDataView::U8(px) = buf.view() else {
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
        let NDDataView::U16(px) = buf.view() else {
            panic!("expected U16")
        };
        // Y-flipped: bottom row (0x0304) first.
        assert_eq!(px, vec![0x0304, 0x0102]);
    }

    #[test]
    fn decode_image16_keeps_host_order_when_detector_does_not_swap() {
        let body = [b'X', 0x01, 0x02, 0x03, 0x04];
        let buf = decode_image(&body, 1, 1, 2, 16, false).unwrap();
        let NDDataView::U16(px) = buf.view() else {
            panic!("expected U16")
        };
        assert_eq!(px, vec![0x0403, 0x0201]);
    }

    #[test]
    fn decode_image32_swaps_big_endian_pixels() {
        let body = [b'X', 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x03, 0x04];
        let buf = decode_image(&body, 1, 1, 2, 32, true).unwrap();
        let NDDataView::U32(px) = buf.view() else {
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
        // A reused pool buffer's stale words.
        let mut img = vec![0xA5A5_A5A5u32; 6];
        p.pack_into(&mut img);
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
