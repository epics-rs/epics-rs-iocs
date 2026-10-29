//! TIFF decoding for the 16-bit images `marccd_server` writes.
//!
//! Everything here is pure and file-driven so it can be exercised with fixtures.

use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use tiff::decoder::{Decoder, DecodingBuffer, DecodingSampleType};

/// A TIFF opened as far as its dimensions, which C's `readTiff` checks
/// before it reads a strip.
pub struct TiffFile {
    decoder: Decoder<BufReader<File>>,
    pub width: u32,
    pub height: u32,
}

impl TiffFile {
    pub fn open(path: &Path) -> Result<Self, String> {
        let file = File::open(path).map_err(|e| format!("cannot open {path:?}: {e}"))?;
        let mut decoder =
            Decoder::new(BufReader::new(file)).map_err(|e| format!("not a TIFF file: {e}"))?;
        let (width, height) = decoder
            .dimensions()
            .map_err(|e| format!("no image dimensions: {e}"))?;
        Ok(Self {
            decoder,
            width,
            height,
        })
    }

    /// Decode the pixels into `out`.
    ///
    /// C copies the raw strip bytes straight into the `NDUInt16` array and only
    /// checks the total byte count. The equivalent here: a 16-bit sample,
    /// signed or not, is decoded bit for bit, and anything whose sample width
    /// is not 16 bits or whose first plane is not `out.len()` samples fails, as
    /// C's `totalSize` mismatch retry path does.
    ///
    /// Deviation from C, documented in the crate docs: the `tiff` crate decodes
    /// every strip, whereas C's loop passes strip index `0` on each iteration.
    /// The two agree for the single-strip files `marccd_server` writes; for a
    /// multi-strip file C repeats strip 0 and this function does not.
    pub fn read_into(mut self, out: &mut [u16]) -> Result<(), String> {
        let layout = self
            .decoder
            .image_buffer_layout()
            .map_err(|e| format!("cannot decode image: {e}"))?;
        if !matches!(
            layout.sample_type,
            Some(DecodingSampleType::U16 | DecodingSampleType::I16)
        ) {
            return Err("image is not 16 bits per sample".to_string());
        }
        let mut buffer = DecodingBuffer::U16(out);
        let bytes = buffer.as_bytes_mut();
        if layout.len != bytes.len() {
            return Err(format!(
                "file size incorrect = {} bytes, should be {}",
                layout.len,
                bytes.len()
            ));
        }
        self.decoder
            .read_image_bytes(bytes)
            .map_err(|e| format!("cannot decode image: {e}"))
    }
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

    /// Open `path` and decode it into a stale buffer sized from its header.
    fn decode(path: &Path) -> Result<(u32, u32, Vec<u16>), String> {
        let tiff = TiffFile::open(path)?;
        let (width, height) = (tiff.width, tiff.height);
        let mut pixels = vec![0x5a5a; width as usize * height as usize];
        tiff.read_into(&mut pixels)?;
        Ok((width, height, pixels))
    }

    #[test]
    fn tiff_roundtrip_single_strip() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("img_001.tif");
        let data: Vec<u16> = (0..6 * 4).map(|i| (i * 7 + 3) as u16).collect();
        write_tiff(&path, 6, 4, &data, None);

        let (width, height, pixels) = decode(&path).unwrap();
        assert_eq!((width, height), (6, 4));
        assert_eq!(pixels, data);
    }

    #[test]
    fn tiff_full_16bit_range() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("range.tif");
        let data = vec![0u16, 1, 32767, 32768, 65535];
        write_tiff(&path, 5, 1, &data, None);
        let (_, _, pixels) = decode(&path).unwrap();
        assert_eq!(pixels, data);
    }

    #[test]
    fn tiff_multi_strip_is_decoded_fully() {
        // Deviation from C, which passes strip index 0 on every iteration.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("multi.tif");
        let data: Vec<u16> = (0..4 * 6).map(|i| i as u16).collect();
        write_tiff(&path, 4, 6, &data, Some(2));
        let (_, _, pixels) = decode(&path).unwrap();
        assert_eq!(pixels, data);
    }

    #[test]
    fn a_buffer_other_than_the_image_size_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("img.tif");
        write_tiff(&path, 2, 2, &[1, 2, 3, 4], None);
        for len in [3, 5] {
            let mut pixels = vec![0; len];
            let err = TiffFile::open(&path)
                .unwrap()
                .read_into(&mut pixels)
                .unwrap_err();
            assert!(err.contains("file size incorrect = 8 bytes"), "{err}");
        }
    }

    #[test]
    fn tiff_missing_file_is_an_error() {
        assert!(TiffFile::open(Path::new("/nonexistent/none.tif")).is_err());
    }

    #[test]
    fn tiff_truncated_file_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bad.tif");
        std::fs::write(&path, b"II*\x00garbage").unwrap();
        assert!(decode(&path).is_err());
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
        let err = decode(&path).unwrap_err();
        assert!(err.contains("16 bits per sample"), "{err}");
    }
}
