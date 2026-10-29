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

use image::{ColorType, DynamicImage};

use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::ndarray::{NDDataType, NDDataViewMut, NDDimension};

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
    /// The decoder's pixels, as C keeps the Magick `Image` until it writes
    /// them into `pData`.
    image: DynamicImage,
}

impl DecodedImage {
    /// C `NDArrayInfo::totalBytes` of the frame these pixels fill.
    pub fn total_bytes(&self) -> usize {
        self.dims.iter().map(|d| d.size).product::<usize>() * self.data_type.element_size()
    }

    /// C `image.write(0, 0, ncols, nrows, map, storageType, pData)`
    /// (URLDriver.cpp:139): the pixels into the frame allocated with
    /// `dims` and `data_type`, alpha dropped.
    pub fn write_into(&self, out: NDDataViewMut<'_>) {
        fn drop_alpha<const N: usize, T: Copy>(out: &mut [T], src: &[T]) {
            let (out, _) = out.as_chunks_mut::<N>();
            for (dst, px) in out.iter_mut().zip(src.chunks_exact(N + 1)) {
                dst.copy_from_slice(&px[..N]);
            }
        }
        match (&self.image, out) {
            (DynamicImage::ImageLuma8(b), NDDataViewMut::U8(out)) => out.copy_from_slice(b),
            (DynamicImage::ImageLumaA8(b), NDDataViewMut::U8(out)) => drop_alpha::<1, _>(out, b),
            (DynamicImage::ImageLuma16(b), NDDataViewMut::U16(out)) => out.copy_from_slice(b),
            (DynamicImage::ImageLumaA16(b), NDDataViewMut::U16(out)) => drop_alpha::<1, _>(out, b),
            (DynamicImage::ImageRgb8(b), NDDataViewMut::U8(out)) => out.copy_from_slice(b),
            (DynamicImage::ImageRgba8(b), NDDataViewMut::U8(out)) => drop_alpha::<3, _>(out, b),
            (DynamicImage::ImageRgb16(b), NDDataViewMut::U16(out)) => out.copy_from_slice(b),
            (DynamicImage::ImageRgba16(b), NDDataViewMut::U16(out)) => drop_alpha::<3, _>(out, b),
            _ => unreachable!("the frame is allocated with the image's type"),
        }
    }
}

/// Decode `bytes` (an already-fetched image file) into driver-shaped data.
pub fn decode_image(bytes: &[u8]) -> Result<DecodedImage, DecodeError> {
    let image = image::load_from_memory(bytes).map_err(DecodeError::Image)?;
    let (w, h) = (image.width() as usize, image.height() as usize);
    let (color_mode, data_type) = match image.color() {
        ColorType::L8 | ColorType::La8 => (NDColorMode::Mono, NDDataType::UInt8),
        ColorType::L16 | ColorType::La16 => (NDColorMode::Mono, NDDataType::UInt16),
        ColorType::Rgb8 | ColorType::Rgba8 => (NDColorMode::RGB1, NDDataType::UInt8),
        ColorType::Rgb16 | ColorType::Rgba16 => (NDColorMode::RGB1, NDDataType::UInt16),
        other => return Err(DecodeError::Unsupported(format!("{other:?}"))),
    };
    let dims = match color_mode {
        NDColorMode::Mono => vec![NDDimension::new(w), NDDimension::new(h)],
        _ => vec![
            NDDimension::new(3),
            NDDimension::new(w),
            NDDimension::new(h),
        ],
    };
    Ok(DecodedImage {
        dims,
        color_mode,
        data_type,
        image,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataView};

    /// `write_into` a stale buffer of the decoded frame's type.
    fn pixels(decoded: &DecodedImage) -> NDDataBuffer {
        let n = decoded.total_bytes() / decoded.data_type.element_size();
        let mut buf = match decoded.data_type {
            NDDataType::UInt8 => NDDataBuffer::from_vec(vec![0xa5u8; n]),
            _ => NDDataBuffer::from_vec(vec![0xa5a5u16; n]),
        };
        decoded.write_into(buf.view_mut());
        buf
    }

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
        let expected: Vec<u8> = (0..3).flat_map(|y| (0..4).map(move |x| x + y)).collect();
        assert!(matches!(pixels(&decoded).view(), NDDataView::U8(v) if v == expected));
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
        let expected: Vec<u16> = (0..2)
            .flat_map(|y| (0..5).map(move |x| (x + y) * 1000))
            .collect();
        assert!(matches!(pixels(&decoded).view(), NDDataView::U16(v) if v == expected));
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
        let expected: Vec<u8> = (0..4)
            .flat_map(|y| (0..6).flat_map(move |x| [x, y, x + y]))
            .collect();
        assert!(matches!(pixels(&decoded).view(), NDDataView::U8(v) if v == expected));
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
        let expected: Vec<u16> = (0..3)
            .flat_map(|y| (0..3).flat_map(move |x| [x * 100, y * 100, 0]))
            .collect();
        assert!(matches!(pixels(&decoded).view(), NDDataView::U16(v) if v == expected));
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
        // 2x2 RGB1 (alpha dropped) = 3 * 2 * 2 = 12 bytes, not 16.
        let expected = [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0];
        assert!(matches!(pixels(&decoded).view(), NDDataView::U8(v) if v == expected));
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
