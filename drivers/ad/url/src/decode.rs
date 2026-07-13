//! Decode an image byte stream into the dtype/color-mode shape the driver
//! publishes as an `NDArray`.
//!
//! Mirrors C++ `URLDriver::readImage()`'s `switch(imageType)` /
//! `switch(depth)` pair, which maps GraphicsMagick's `GrayscaleType`/
//! `TrueColorType` and 8/16/32-bit depth to `NDColorMode`/`NDDataType`. This
//! driver decodes with the `image` crate instead of GraphicsMagick:
//!
//! - `GrayscaleType` (any depth) -> `image` crate `L8`/`La8`/`L16`/`La16` ->
//!   `NDColorMode::Mono`, dims `[x, y]`.
//! - `TrueColorType` -> `image` crate `Rgb8`/`Rgba8`/`Rgb16`/`Rgba16` ->
//!   `NDColorMode::RGB1`, dims `[3, x, y]` (color-interleaved, matching
//!   GraphicsMagick's `"RGB"` channel-map extraction).
//! - An alpha channel present in the source (`La8`/`La16`/`Rgba8`/`Rgba16`) is
//!   dropped, matching GraphicsMagick's explicit `"R"`/`"RGB"` channel maps
//!   which never extract alpha.
//!
//! Deviation from C++ (documented, not a bug): GraphicsMagick's `depth == 32`
//! case is a 32-bit *integer* pixel (`NDUInt32`/`IntegerPixel`). The `image`
//! crate's only 32-bit variants (`Rgb32F`/`Rgba32F`) are 32-bit *float* HDR
//! pixels, not integer — there is no like-for-like Rust substitute, so this
//! driver rejects them as unsupported rather than fabricating a lossy
//! float-to-int mapping.

use image::ColorType;

use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType, NDDimension};

#[derive(Debug)]
pub enum DecodeError {
    Image(image::ImageError),
    Unsupported(String),
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DecodeError::Image(e) => write!(f, "image decode error: {e}"),
            DecodeError::Unsupported(s) => write!(f, "unsupported image: {s}"),
        }
    }
}

impl std::error::Error for DecodeError {}

pub struct DecodedImage {
    /// C++ `dims[]`: `[x, y]` for Mono, `[3, x, y]` for RGB1.
    pub dims: Vec<NDDimension>,
    pub color_mode: NDColorMode,
    pub data_type: NDDataType,
    pub data: NDDataBuffer,
}

/// Decode `bytes` (an already-fetched image file) into driver-shaped data.
pub fn decode_image(bytes: &[u8]) -> Result<DecodedImage, DecodeError> {
    let img = image::load_from_memory(bytes).map_err(DecodeError::Image)?;

    match img.color() {
        ColorType::L8 | ColorType::La8 => {
            let buf = img.into_luma8();
            let (w, h) = buf.dimensions();
            Ok(DecodedImage {
                dims: vec![NDDimension::new(w as usize), NDDimension::new(h as usize)],
                color_mode: NDColorMode::Mono,
                data_type: NDDataType::UInt8,
                data: NDDataBuffer::U8(buf.into_raw()),
            })
        }
        ColorType::L16 | ColorType::La16 => {
            let buf = img.into_luma16();
            let (w, h) = buf.dimensions();
            Ok(DecodedImage {
                dims: vec![NDDimension::new(w as usize), NDDimension::new(h as usize)],
                color_mode: NDColorMode::Mono,
                data_type: NDDataType::UInt16,
                data: NDDataBuffer::U16(buf.into_raw()),
            })
        }
        ColorType::Rgb8 | ColorType::Rgba8 => {
            let buf = img.into_rgb8();
            let (w, h) = buf.dimensions();
            Ok(DecodedImage {
                dims: vec![
                    NDDimension::new(3),
                    NDDimension::new(w as usize),
                    NDDimension::new(h as usize),
                ],
                color_mode: NDColorMode::RGB1,
                data_type: NDDataType::UInt8,
                data: NDDataBuffer::U8(buf.into_raw()),
            })
        }
        ColorType::Rgb16 | ColorType::Rgba16 => {
            let buf = img.into_rgb16();
            let (w, h) = buf.dimensions();
            Ok(DecodedImage {
                dims: vec![
                    NDDimension::new(3),
                    NDDimension::new(w as usize),
                    NDDimension::new(h as usize),
                ],
                color_mode: NDColorMode::RGB1,
                data_type: NDDataType::UInt16,
                data: NDDataBuffer::U16(buf.into_raw()),
            })
        }
        other => Err(DecodeError::Unsupported(format!("{other:?}"))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode_png(img: &image::DynamicImage) -> Vec<u8> {
        let mut buf = Vec::new();
        img.write_to(&mut std::io::Cursor::new(&mut buf), image::ImageFormat::Png)
            .unwrap();
        buf
    }

    fn encode_tiff(img: &image::DynamicImage) -> Vec<u8> {
        let mut buf = Vec::new();
        img.write_to(
            &mut std::io::Cursor::new(&mut buf),
            image::ImageFormat::Tiff,
        )
        .unwrap();
        buf
    }

    #[test]
    fn decodes_8bit_gray_png_as_mono_u8() {
        let img = image::DynamicImage::ImageLuma8(image::GrayImage::from_fn(4, 3, |x, y| {
            image::Luma([(x + y) as u8])
        }));
        let bytes = encode_png(&img);

        let decoded = decode_image(&bytes).unwrap();

        assert_eq!(decoded.color_mode, NDColorMode::Mono);
        assert_eq!(decoded.data_type, NDDataType::UInt8);
        assert_eq!(decoded.dims.len(), 2);
        assert_eq!(decoded.dims[0].size, 4);
        assert_eq!(decoded.dims[1].size, 3);
        match decoded.data {
            NDDataBuffer::U8(v) => assert_eq!(v.len(), 4 * 3),
            _ => panic!("expected U8 buffer"),
        }
    }

    #[test]
    fn decodes_16bit_gray_tiff_as_mono_u16() {
        let img = image::DynamicImage::ImageLuma16(image::ImageBuffer::from_fn(5, 2, |x, y| {
            image::Luma([((x + y) as u16) * 1000])
        }));
        let bytes = encode_tiff(&img);

        let decoded = decode_image(&bytes).unwrap();

        assert_eq!(decoded.color_mode, NDColorMode::Mono);
        assert_eq!(decoded.data_type, NDDataType::UInt16);
        assert_eq!(decoded.dims.len(), 2);
        assert_eq!(decoded.dims[0].size, 5);
        assert_eq!(decoded.dims[1].size, 2);
        match decoded.data {
            NDDataBuffer::U16(v) => assert_eq!(v.len(), 5 * 2),
            _ => panic!("expected U16 buffer"),
        }
    }

    #[test]
    fn decodes_8bit_rgb_png_as_rgb1_u8() {
        let img = image::DynamicImage::ImageRgb8(image::RgbImage::from_fn(6, 4, |x, y| {
            image::Rgb([x as u8, y as u8, (x + y) as u8])
        }));
        let bytes = encode_png(&img);

        let decoded = decode_image(&bytes).unwrap();

        assert_eq!(decoded.color_mode, NDColorMode::RGB1);
        assert_eq!(decoded.data_type, NDDataType::UInt8);
        assert_eq!(decoded.dims.len(), 3);
        assert_eq!(decoded.dims[0].size, 3);
        assert_eq!(decoded.dims[1].size, 6);
        assert_eq!(decoded.dims[2].size, 4);
        match decoded.data {
            NDDataBuffer::U8(v) => assert_eq!(v.len(), 3 * 6 * 4),
            _ => panic!("expected U8 buffer"),
        }
    }

    #[test]
    fn decodes_16bit_rgb_png_as_rgb1_u16() {
        let img = image::DynamicImage::ImageRgb16(image::ImageBuffer::from_fn(3, 3, |x, y| {
            image::Rgb([x as u16 * 100, y as u16 * 100, 0])
        }));
        let bytes = encode_png(&img);

        let decoded = decode_image(&bytes).unwrap();

        assert_eq!(decoded.color_mode, NDColorMode::RGB1);
        assert_eq!(decoded.data_type, NDDataType::UInt16);
        assert_eq!(decoded.dims.len(), 3);
        assert_eq!(decoded.dims[0].size, 3);
        assert_eq!(decoded.dims[1].size, 3);
        assert_eq!(decoded.dims[2].size, 3);
        match decoded.data {
            NDDataBuffer::U16(v) => assert_eq!(v.len(), 3 * 3 * 3),
            _ => panic!("expected U16 buffer"),
        }
    }

    #[test]
    fn drops_alpha_channel_on_rgba_source() {
        let img = image::DynamicImage::ImageRgba8(image::RgbaImage::from_fn(2, 2, |x, y| {
            image::Rgba([x as u8, y as u8, 0, 128])
        }));
        let bytes = encode_png(&img);

        let decoded = decode_image(&bytes).unwrap();

        assert_eq!(decoded.color_mode, NDColorMode::RGB1);
        assert_eq!(decoded.data_type, NDDataType::UInt8);
        match decoded.data {
            // 2x2 RGB1 (alpha dropped) = 3 * 2 * 2 = 12 bytes, not 16.
            NDDataBuffer::U8(v) => assert_eq!(v.len(), 3 * 2 * 2),
            _ => panic!("expected U8 buffer"),
        }
    }

    #[test]
    fn rejects_32bit_float_hdr_as_unsupported() {
        let pixels = vec![image::Rgb([0.5f32, 0.25, 0.125]); 4];
        let mut bytes = Vec::new();
        image::codecs::hdr::HdrEncoder::new(&mut bytes)
            .encode(&pixels, 2, 2)
            .unwrap();

        let result = decode_image(&bytes);

        assert!(matches!(result, Err(DecodeError::Unsupported(_))));
    }

    #[test]
    fn rejects_garbage_bytes_as_image_error() {
        let result = decode_image(b"not an image");
        assert!(matches!(result, Err(DecodeError::Image(_))));
    }
}
