//! TIFF decoding for the 16-bit images `marccd_server` writes.
//!
//! Everything here is pure and file-driven so it can be exercised with fixtures.

use std::io::BufReader;
use std::path::Path;

use tiff::decoder::{Decoder, DecodingResult};

/// A decoded single-channel 16-bit TIFF.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TiffImage {
    pub width: u32,
    pub height: u32,
    /// One `epicsUInt16` per pixel.
    pub data: Vec<u16>,
}

/// Decode a TIFF the way C's `readTiff` consumes one.
///
/// C copies the raw strip bytes straight into an `NDUInt16` buffer and only
/// validates `TIFFTAG_IMAGEWIDTH`, `TIFFTAG_IMAGELENGTH` and the total byte
/// count. The equivalent here is: accept a 16-bit single-sample image and
/// return its samples as `epicsUInt16`; reject anything whose decoded element
/// count is not `width * height` (C's `totalSize` mismatch retry path) or whose
/// sample width is not 16 bits (same path).
///
/// Deviation from C, documented in the crate docs: the `tiff` crate decodes
/// every strip, whereas C's loop passes strip index `0` on each iteration. The
/// two agree for the single-strip files `marccd_server` writes; for a
/// multi-strip file C repeats strip 0 and this function does not.
pub fn decode_tiff(path: &Path) -> Result<TiffImage, String> {
    let file = std::fs::File::open(path).map_err(|e| format!("cannot open {path:?}: {e}"))?;
    let mut decoder =
        Decoder::new(BufReader::new(file)).map_err(|e| format!("not a TIFF file: {e}"))?;
    let (width, height) = decoder
        .dimensions()
        .map_err(|e| format!("no image dimensions: {e}"))?;

    let image = decoder
        .read_image()
        .map_err(|e| format!("cannot decode image: {e}"))?;

    let data: Vec<u16> = match image {
        DecodingResult::U16(v) => v,
        // C reinterprets the raw strip bytes as epicsUInt16; a signed 16-bit
        // sample carries the same bit pattern.
        DecodingResult::I16(v) => v.into_iter().map(|x| x as u16).collect(),
        _ => return Err("image is not 16 bits per sample".to_string()),
    };

    let expected = (width as usize)
        .checked_mul(height as usize)
        .ok_or_else(|| "image dimensions overflow".to_string())?;
    if data.len() != expected {
        return Err(format!(
            "file size incorrect = {} samples, should be {expected}",
            data.len()
        ));
    }

    Ok(TiffImage {
        width,
        height,
        data,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_tiff(path: &Path, w: u32, h: u32, data: &[u16], rows: Option<u32>) {
        use tiff::encoder::{TiffEncoder, colortype};
        let mut file = std::fs::File::create(path).unwrap();
        let mut enc = TiffEncoder::new(&mut file).unwrap();
        if let Some(r) = rows {
            let mut img = enc.new_image::<colortype::Gray16>(w, h).unwrap();
            img.rows_per_strip(r).unwrap();
            img.write_data(data).unwrap();
        } else {
            enc.write_image::<colortype::Gray16>(w, h, data).unwrap();
        }
    }

    #[test]
    fn tiff_roundtrip_single_strip() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("img_001.tif");
        let data: Vec<u16> = (0..6 * 4).map(|i| (i * 7 + 3) as u16).collect();
        write_tiff(&path, 6, 4, &data, None);

        let img = decode_tiff(&path).unwrap();
        assert_eq!(img.width, 6);
        assert_eq!(img.height, 4);
        assert_eq!(img.data, data);
    }

    #[test]
    fn tiff_full_16bit_range() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("range.tif");
        let data = vec![0u16, 1, 32767, 32768, 65535];
        write_tiff(&path, 5, 1, &data, None);
        let img = decode_tiff(&path).unwrap();
        assert_eq!(img.data, data);
    }

    #[test]
    fn tiff_multi_strip_is_decoded_fully() {
        // Deviation from C, which passes strip index 0 on every iteration.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("multi.tif");
        let data: Vec<u16> = (0..4 * 6).map(|i| i as u16).collect();
        write_tiff(&path, 4, 6, &data, Some(2));
        let img = decode_tiff(&path).unwrap();
        assert_eq!(img.data, data);
    }

    #[test]
    fn tiff_missing_file_is_an_error() {
        assert!(decode_tiff(Path::new("/nonexistent/none.tif")).is_err());
    }

    #[test]
    fn tiff_truncated_file_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bad.tif");
        std::fs::write(&path, b"II*\x00garbage").unwrap();
        assert!(decode_tiff(&path).is_err());
    }

    #[test]
    fn tiff_32_bit_is_rejected() {
        use tiff::encoder::{TiffEncoder, colortype};
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("i32.tif");
        {
            let mut file = std::fs::File::create(&path).unwrap();
            let mut enc = TiffEncoder::new(&mut file).unwrap();
            enc.write_image::<colortype::GrayI32>(2, 2, &[1i32, 2, 3, 4])
                .unwrap();
        }
        let err = decode_tiff(&path).unwrap_err();
        assert!(err.contains("16 bits per sample"), "{err}");
    }
}
