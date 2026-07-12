//! Monitor-image TIFF decode (port of `eigerDetector::parseTiffFile`,
//! eigerDetector.cpp:1926).
//!
//! The monitor interface serves a single-strip, little-endian, uncompressed
//! greyscale TIFF. The C driver hand-parses the five tags it needs rather than
//! linking libtiff; this port does the same, but bounds-checks every read.

use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType};

/// `II` + 42, little-endian — the classic TIFF magic (C compares the first
/// `uint32` against `0x002A4949`).
const TIFF_MAGIC_LE: u32 = 0x002A_4949;

const TAG_IMAGE_WIDTH: u16 = 256;
const TAG_IMAGE_LENGTH: u16 = 257;
const TAG_BITS_PER_SAMPLE: u16 = 258;
const TAG_STRIP_OFFSETS: u16 = 273;
const TAG_STRIP_BYTE_COUNTS: u16 = 279;

#[derive(Debug, PartialEq)]
pub struct TiffError(pub String);

impl std::fmt::Display for TiffError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

fn err<T>(msg: impl Into<String>) -> Result<T, TiffError> {
    Err(TiffError(msg.into()))
}

fn u16_at(buf: &[u8], at: usize) -> Result<u16, TiffError> {
    buf.get(at..at + 2)
        .map(|b| u16::from_le_bytes([b[0], b[1]]))
        .ok_or_else(|| TiffError(format!("truncated at byte {at}")))
}

fn u32_at(buf: &[u8], at: usize) -> Result<u32, TiffError> {
    buf.get(at..at + 4)
        .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        .ok_or_else(|| TiffError(format!("truncated at byte {at}")))
}

/// A decoded monitor image: `[width, height]` and its pixels.
#[derive(Debug)]
pub struct TiffImage {
    pub dims: [usize; 2],
    pub data: NDDataBuffer,
}

/// Decode a monitor TIFF blob.
///
/// UPSTREAM DEFECT (eigerDetector.cpp:1937-1948, 1994): C reads the IFD offset,
/// the entry count and every tag without checking them against `len`, so a
/// truncated or hostile blob reads past the end of the buffer; and it allocates
/// the NDArray *before* the `stripOffset`/`dataLen` sanity checks, leaking it on
/// both early-return paths (:1994, :1995).
///
/// UPSTREAM DEFECT (eigerDetector.cpp:1996): `memcpy(pImage->pData, buf +
/// stripOffset, dataLen)` copies `dataLen` bytes — the value of the
/// StripByteCounts tag — into a buffer sized `width*height*depth/8`, with no
/// check that the two agree. A detector (or a man-in-the-middle) reporting a
/// StripByteCounts larger than the image dimensions overflows the NDArray. Here
/// the strip must be exactly the size the dimensions imply.
pub fn decode(buf: &[u8]) -> Result<TiffImage, TiffError> {
    if u32_at(buf, 0)? != TIFF_MAGIC_LE {
        return err("wrong tiff header");
    }

    let ifd = u32_at(buf, 4)? as usize;
    let num_entries = u16_at(buf, ifd)? as usize;

    let (mut width, mut height, mut depth, mut data_len, mut strip_offset) = (0usize, 0, 0, 0, 0);
    for i in 0..num_entries {
        // Each IFD entry is 12 bytes: id(2) type(2) count(4) value(4).
        let entry = ifd + 2 + i * 12;
        let id = u16_at(buf, entry)?;
        // C reads the value field as a uint32 regardless of the tag's declared
        // type. That is correct on a little-endian host for the SHORT and LONG
        // counts of 1 that the monitor interface emits, and it is all this
        // hand-parser claims to handle.
        let value = u32_at(buf, entry + 8)? as usize;
        match id {
            TAG_IMAGE_WIDTH => width = value,
            TAG_IMAGE_LENGTH => height = value,
            TAG_BITS_PER_SAMPLE => depth = value,
            TAG_STRIP_OFFSETS => strip_offset = value,
            TAG_STRIP_BYTE_COUNTS => data_len = value,
            _ => {}
        }
    }

    if width == 0 || height == 0 || depth == 0 || data_len == 0 {
        return err("missing tags");
    }
    if strip_offset == 0 {
        return err("missing StripOffsets tag");
    }

    let data_type = match depth {
        8 => NDDataType::UInt8,
        16 => NDDataType::UInt16,
        32 => NDDataType::UInt32,
        other => return err(format!("unexpected bit depth={other}")),
    };

    let expected = width
        .checked_mul(height)
        .and_then(|n| n.checked_mul(data_type.element_size()))
        .ok_or_else(|| TiffError("image dimensions overflow".into()))?;
    if data_len != expected {
        return err(format!(
            "StripByteCounts is {data_len} bytes but {width}x{height}x{depth}b needs {expected}"
        ));
    }

    let strip = buf
        .get(strip_offset..strip_offset + data_len)
        .ok_or_else(|| TiffError("pixel data out of range".into()))?;

    let data = match data_type {
        NDDataType::UInt8 => NDDataBuffer::U8(strip.to_vec()),
        NDDataType::UInt16 => NDDataBuffer::U16(
            strip
                .chunks_exact(2)
                .map(|c| u16::from_le_bytes([c[0], c[1]]))
                .collect(),
        ),
        _ => NDDataBuffer::U32(
            strip
                .chunks_exact(4)
                .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect(),
        ),
    };

    Ok(TiffImage {
        dims: [width, height],
        data,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build the same single-strip TIFF the monitor interface serves.
    fn build(width: u32, height: u32, depth: u32, pixels: &[u8]) -> Vec<u8> {
        let tags: [(u16, u32); 5] = [
            (TAG_IMAGE_WIDTH, width),
            (TAG_IMAGE_LENGTH, height),
            (TAG_BITS_PER_SAMPLE, depth),
            (TAG_STRIP_OFFSETS, 0), // patched below
            (TAG_STRIP_BYTE_COUNTS, pixels.len() as u32),
        ];
        let ifd = 8u32;
        let strip_offset = ifd + 2 + 12 * tags.len() as u32 + 4;

        let mut buf = Vec::new();
        buf.extend_from_slice(&TIFF_MAGIC_LE.to_le_bytes());
        buf.extend_from_slice(&ifd.to_le_bytes());
        buf.extend_from_slice(&(tags.len() as u16).to_le_bytes());
        for (id, value) in tags {
            let value = if id == TAG_STRIP_OFFSETS {
                strip_offset
            } else {
                value
            };
            buf.extend_from_slice(&id.to_le_bytes());
            buf.extend_from_slice(&3u16.to_le_bytes()); // type SHORT
            buf.extend_from_slice(&1u32.to_le_bytes()); // count
            buf.extend_from_slice(&value.to_le_bytes());
        }
        buf.extend_from_slice(&0u32.to_le_bytes()); // next IFD = none
        assert_eq!(buf.len() as u32, strip_offset);
        buf.extend_from_slice(pixels);
        buf
    }

    #[test]
    fn decodes_a_16_bit_monitor_image() {
        let pixels: Vec<u16> = (0..12u16).map(|i| i * 100).collect();
        let raw: Vec<u8> = pixels.iter().flat_map(|p| p.to_le_bytes()).collect();
        let img = decode(&build(4, 3, 16, &raw)).unwrap();
        assert_eq!(img.dims, [4, 3]);
        let NDDataBuffer::U16(got) = img.data else {
            panic!("expected U16 pixels");
        };
        assert_eq!(got, pixels);
    }

    #[test]
    fn decodes_a_32_bit_monitor_image() {
        let pixels: Vec<u32> = (0..6u32).map(|i| i * 70_000).collect();
        let raw: Vec<u8> = pixels.iter().flat_map(|p| p.to_le_bytes()).collect();
        let img = decode(&build(3, 2, 32, &raw)).unwrap();
        assert_eq!(img.dims, [3, 2]);
        let NDDataBuffer::U32(got) = img.data else {
            panic!("expected U32 pixels");
        };
        assert_eq!(got, pixels);
    }

    #[test]
    fn decodes_an_8_bit_monitor_image() {
        let pixels: Vec<u8> = (0..9u8).collect();
        let img = decode(&build(3, 3, 8, &pixels)).unwrap();
        let NDDataBuffer::U8(got) = img.data else {
            panic!("expected U8 pixels");
        };
        assert_eq!(got, pixels);
    }

    #[test]
    fn a_wrong_magic_is_rejected() {
        let mut buf = build(2, 2, 8, &[0; 4]);
        buf[0] = 0x4D; // 'M' — big-endian TIFF, which the monitor never sends
        assert_eq!(
            decode(&buf).unwrap_err(),
            TiffError("wrong tiff header".into())
        );
    }

    #[test]
    fn a_missing_tag_is_rejected() {
        // Zero the BitsPerSample value, leaving depth == 0.
        let mut buf = build(2, 2, 8, &[0; 4]);
        let ifd = 8usize;
        let entry = ifd + 2 + 2 * 12; // third tag
        buf[entry + 8..entry + 12].copy_from_slice(&0u32.to_le_bytes());
        assert_eq!(decode(&buf).unwrap_err(), TiffError("missing tags".into()));
    }

    #[test]
    fn an_unexpected_bit_depth_is_rejected() {
        let buf = build(2, 2, 12, &[0; 4]);
        assert!(decode(&buf).is_err());
    }

    /// The overflow C would have memcpy'd: StripByteCounts claims more bytes
    /// than the width/height/depth imply.
    #[test]
    fn a_strip_longer_than_the_dimensions_is_rejected() {
        let mut buf = build(2, 2, 8, &[0; 64]);
        let ifd = 8usize;
        let entry = ifd + 2 + 4 * 12; // StripByteCounts
        buf[entry + 8..entry + 12].copy_from_slice(&64u32.to_le_bytes());
        let e = decode(&buf).unwrap_err();
        assert!(e.0.contains("StripByteCounts is 64 bytes"), "{e}");
    }

    /// The out-of-bounds reads C would have done on a truncated blob.
    #[test]
    fn a_truncated_blob_is_rejected_not_read_past_the_end() {
        let full = build(4, 3, 16, &[0; 24]);
        for len in 0..full.len() {
            // Must not panic; every short prefix is either an error or, for a
            // prefix that happens to hold a complete IFD, a decode failure.
            let _ = decode(&full[..len]);
        }
        assert!(decode(&full[..full.len() - 1]).is_err());
    }

    #[test]
    fn a_strip_offset_past_the_end_is_rejected() {
        let mut buf = build(2, 2, 8, &[0; 4]);
        let ifd = 8usize;
        let entry = ifd + 2 + 3 * 12; // StripOffsets
        buf[entry + 8..entry + 12].copy_from_slice(&10_000u32.to_le_bytes());
        assert_eq!(
            decode(&buf).unwrap_err(),
            TiffError("pixel data out of range".into())
        );
    }
}
