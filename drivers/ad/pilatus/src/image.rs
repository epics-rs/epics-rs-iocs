//! TIFF decoding, bad-pixel maps and flat-field arithmetic.
//!
//! Everything here is pure and file-driven so it can be exercised with fixtures.

use std::io::BufReader;
use std::path::Path;

use tiff::decoder::{Decoder, DecodingResult};
use tiff::tags::Tag;

use crate::protocol::{expect_lit, scan_i32, skip_ws};
use crate::types::{BadPixel, MAX_BAD_PIXELS};

/// C `readTiff` copies at most `sizeof(tempBuffer) - 1` bytes of
/// `TIFFTAG_IMAGEDESCRIPTION` into the `TIFFImageDescription` attribute.
const MAX_IMAGE_DESCRIPTION: usize = 2047;

/// A decoded single-channel TIFF.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TiffImage {
    pub width: u32,
    pub height: u32,
    /// One `epicsInt32` per pixel.
    pub data: Vec<i32>,
    /// `TIFFTAG_IMAGEDESCRIPTION`, if present.
    pub description: Option<String>,
}

/// Decode a TIFF the way C's `readTiff` consumes one.
///
/// C copies the raw strip bytes straight into an `epicsInt32` buffer and only
/// validates the width, the height and the total byte count. The equivalent
/// here is: accept any 32-bit single-sample image and reinterpret its bits as
/// `epicsInt32`; reject anything whose decoded element count is not
/// `width * height` (C's `totalSize != arrayInfo.totalBytes` retry path) or
/// whose sample width is not 32 bits (same path).
///
/// Deviation from C, documented in the crate docs: the `tiff` crate decodes
/// every strip, whereas C's loop passes strip index `0` on each iteration.
/// The two agree for the single-strip files camserver writes; for a
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

    let data: Vec<i32> = match image {
        DecodingResult::I32(v) => v,
        // Bit-identical to C's raw byte copy into an epicsInt32 buffer.
        DecodingResult::U32(v) => v.into_iter().map(|x| x as i32).collect(),
        DecodingResult::F32(v) => v.into_iter().map(|x| x.to_bits() as i32).collect(),
        _ => return Err("image is not 32 bits per sample".to_string()),
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

    let description = decoder
        .get_tag_ascii_string(Tag::ImageDescription)
        .ok()
        .map(|mut s| {
            if s.len() > MAX_IMAGE_DESCRIPTION {
                let cut = (0..=MAX_IMAGE_DESCRIPTION)
                    .rev()
                    .find(|&i| s.is_char_boundary(i))
                    .unwrap_or(0);
                s.truncate(cut);
            }
            s
        });

    Ok(TiffImage {
        width,
        height,
        data,
        description,
    })
}

// ---------------------------------------------------------------------------
// Bad pixels
// ---------------------------------------------------------------------------

/// C `readBadPixelFile` inner loop: `fscanf(file, " %d,%d %d,%d", ...)` up to
/// `MAX_BAD_PIXELS` times, stopping at EOF and failing on a short conversion.
///
/// `nx` is `NDArraySizeX` and `ny` is `NDArraySizeY` at the time of the call.
///
/// Upstream defect preserved: the replacement index is `ygood * ny + xgood`,
/// which should be `ygood * nx + xgood`. It only agrees with the bad index's
/// stride on a square detector.
pub fn parse_bad_pixel_file(text: &str, nx: i32, ny: i32) -> Result<Vec<BadPixel>, String> {
    let mut rest = text;
    let mut out = Vec::new();

    for _ in 0..MAX_BAD_PIXELS {
        // fscanf returns EOF when the first conversion finds no input.
        if skip_ws(rest).is_empty() {
            break;
        }
        let mut scan = || -> Option<(i32, i32, i32, i32)> {
            let (xbad, s) = scan_i32(rest)?;
            let s = expect_lit(s, ",")?;
            let (ybad, s) = scan_i32(s)?;
            let (xgood, s) = scan_i32(s)?;
            let s = expect_lit(s, ",")?;
            let (ygood, s) = scan_i32(s)?;
            rest = s;
            Some((xbad, ybad, xgood, ygood))
        };
        let Some((xbad, ybad, xgood, ygood)) = scan() else {
            return Err("too few items, should be 4".to_string());
        };
        out.push(BadPixel {
            bad_index: ybad as i64 * nx as i64 + xbad as i64,
            replace_index: ygood as i64 * ny as i64 + xgood as i64,
        });
    }

    Ok(out)
}

/// C `correctBadPixels`.
///
/// Rust-only change: C indexes `pImage->pData` unchecked, so a bad-pixel file
/// written for a different detector size corrupts memory. Entries whose bad or
/// replacement index falls outside the image are skipped here.
pub fn correct_bad_pixels(data: &mut [i32], map: &[BadPixel]) {
    let len = data.len() as i64;
    for bp in map {
        if bp.bad_index < 0
            || bp.bad_index >= len
            || bp.replace_index < 0
            || bp.replace_index >= len
        {
            continue;
        }
        data[bp.bad_index as usize] = data[bp.replace_index as usize];
    }
}

// ---------------------------------------------------------------------------
// Flat field
// ---------------------------------------------------------------------------

/// C `readFlatFieldFile` averaging step: the mean of every element `>=
/// min_flat_field`.
///
/// Upstream behaviour preserved: with no element at or above the threshold C
/// evaluates `0.0 / 0` and stores NaN, which this returns too.
pub fn flat_field_average(data: &[i32], min_flat_field: i32) -> f64 {
    let mut sum = 0.0f64;
    let mut ngood = 0i64;
    for &v in data {
        if v < min_flat_field {
            continue;
        }
        ngood += 1;
        sum += v as f64;
    }
    sum / ngood as f64
}

/// C `readFlatFieldFile` second pass: every element below the threshold is
/// replaced by the average.
///
/// Rust-only change: C's `(epicsInt32)averageFlatField` is undefined when the
/// value does not fit (in particular for the NaN above); `as i32` saturates.
pub fn apply_flat_field_floor(data: &mut [i32], min_flat_field: i32, average: f64) {
    let fill = average as i32;
    for v in data.iter_mut() {
        if *v < min_flat_field {
            *v = fill;
        }
    }
}

/// C `pilatusTask`: `*pData = (epicsInt32)((averageFlatField * *pData) / *pFlat)`.
///
/// Rust-only change: `as i32` saturates instead of invoking C's undefined
/// float-to-int conversion, and a zero flat-field element yields 0 rather than
/// an infinite intermediate.
pub fn apply_flat_field(data: &mut [i32], flat: &[i32], average: f64) {
    for (v, &f) in data.iter_mut().zip(flat.iter()) {
        let corrected = (average * *v as f64) / f as f64;
        *v = if corrected.is_finite() {
            corrected as i32
        } else {
            0
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_tiff(
        path: &Path,
        w: u32,
        h: u32,
        data: &[i32],
        desc: Option<&str>,
        rows: Option<u32>,
    ) {
        use tiff::encoder::{TiffEncoder, colortype};
        let mut file = std::fs::File::create(path).unwrap();
        let mut enc = TiffEncoder::new(&mut file).unwrap();
        let mut img = enc.new_image::<colortype::GrayI32>(w, h).unwrap();
        if let Some(r) = rows {
            img.rows_per_strip(r).unwrap();
        }
        if let Some(d) = desc {
            img.encoder().write_tag(Tag::ImageDescription, d).unwrap();
        }
        img.write_data(data).unwrap();
    }

    #[test]
    fn tiff_roundtrip_single_strip() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("img_001.tif");
        let data: Vec<i32> = (0..6 * 4).map(|i| i * 7 - 10).collect();
        write_tiff(
            &path,
            6,
            4,
            &data,
            Some("Pilatus 100K, exp 1.000000 s"),
            None,
        );

        let img = decode_tiff(&path).unwrap();
        assert_eq!(img.width, 6);
        assert_eq!(img.height, 4);
        assert_eq!(img.data, data);
        assert_eq!(
            img.description.as_deref(),
            Some("Pilatus 100K, exp 1.000000 s")
        );
    }

    #[test]
    fn tiff_negative_samples_are_signed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("neg.tif");
        let data = vec![-1i32, -2147483648, 2147483647, 0];
        write_tiff(&path, 2, 2, &data, None, None);
        let img = decode_tiff(&path).unwrap();
        assert_eq!(img.data, data);
        assert_eq!(img.description, None);
    }

    #[test]
    fn tiff_multi_strip_is_decoded_fully() {
        // Deviation from C, which passes strip index 0 on every iteration.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("multi.tif");
        let data: Vec<i32> = (0..4 * 6).collect();
        write_tiff(&path, 4, 6, &data, None, Some(2));
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
    fn tiff_16_bit_is_rejected() {
        use tiff::encoder::{TiffEncoder, colortype};
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("u16.tif");
        {
            let mut file = std::fs::File::create(&path).unwrap();
            let mut enc = TiffEncoder::new(&mut file).unwrap();
            enc.write_image::<colortype::Gray16>(2, 2, &[1u16, 2, 3, 4])
                .unwrap();
        }
        let err = decode_tiff(&path).unwrap_err();
        assert!(err.contains("32 bits per sample"), "{err}");
    }

    #[test]
    fn tiff_description_is_truncated_to_c_buffer() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("long.tif");
        let desc = "x".repeat(3000);
        write_tiff(&path, 1, 1, &[0], Some(&desc), None);
        let img = decode_tiff(&path).unwrap();
        assert_eq!(img.description.unwrap().len(), MAX_IMAGE_DESCRIPTION);
    }

    #[test]
    fn bad_pixel_file_parses_c_format() {
        // nx = 487, ny = 195 (Pilatus 100K)
        let map = parse_bad_pixel_file("10,20 11,20\n 30,40   31,41 \n", 487, 195).unwrap();
        assert_eq!(
            map,
            vec![
                BadPixel {
                    bad_index: 20 * 487 + 10,
                    // Upstream defect: ny, not nx.
                    replace_index: 20 * 195 + 11,
                },
                BadPixel {
                    bad_index: 40 * 487 + 30,
                    replace_index: 41 * 195 + 31,
                },
            ]
        );
    }

    #[test]
    fn bad_pixel_file_empty_yields_no_entries() {
        assert_eq!(parse_bad_pixel_file("", 10, 10).unwrap(), vec![]);
        assert_eq!(parse_bad_pixel_file("  \n\t ", 10, 10).unwrap(), vec![]);
    }

    #[test]
    fn bad_pixel_file_short_record_is_an_error() {
        assert!(parse_bad_pixel_file("10,20 11", 10, 10).is_err());
        assert!(parse_bad_pixel_file("10,20 11,", 10, 10).is_err());
        // The literal ',' does not skip whitespace, matching fscanf.
        assert!(parse_bad_pixel_file("10 ,20 11,20", 10, 10).is_err());
    }

    #[test]
    fn bad_pixel_file_stops_at_max_bad_pixels() {
        let text = "1,1 2,2\n".repeat(MAX_BAD_PIXELS + 5);
        let map = parse_bad_pixel_file(&text, 10, 10).unwrap();
        assert_eq!(map.len(), MAX_BAD_PIXELS);
    }

    #[test]
    fn correct_bad_pixels_copies_replacement() {
        let mut data = vec![0, 1, 2, 3];
        correct_bad_pixels(
            &mut data,
            &[BadPixel {
                bad_index: 0,
                replace_index: 3,
            }],
        );
        assert_eq!(data, vec![3, 1, 2, 3]);
    }

    #[test]
    fn correct_bad_pixels_skips_out_of_range() {
        let mut data = vec![0, 1];
        correct_bad_pixels(
            &mut data,
            &[
                BadPixel {
                    bad_index: 5,
                    replace_index: 0,
                },
                BadPixel {
                    bad_index: 0,
                    replace_index: -1,
                },
            ],
        );
        assert_eq!(data, vec![0, 1]);
    }

    #[test]
    fn flat_field_average_ignores_values_below_threshold() {
        let data = vec![50, 100, 200, 300];
        assert_eq!(flat_field_average(&data, 100), 200.0);
    }

    #[test]
    fn flat_field_average_is_nan_when_nothing_qualifies() {
        assert!(flat_field_average(&[1, 2, 3], 100).is_nan());
    }

    #[test]
    fn flat_field_floor_replaces_low_values() {
        let mut data = vec![50, 100, 200, 300];
        let avg = flat_field_average(&data, 100);
        apply_flat_field_floor(&mut data, 100, avg);
        assert_eq!(data, vec![200, 100, 200, 300]);
    }

    #[test]
    fn flat_field_correction_scales_by_average() {
        // (average * raw) / flat
        let mut data = vec![100, 200];
        apply_flat_field(&mut data, &[100, 400], 200.0);
        assert_eq!(data, vec![200, 100]);
    }

    #[test]
    fn flat_field_correction_zero_divisor_yields_zero() {
        let mut data = vec![100];
        apply_flat_field(&mut data, &[0], 200.0);
        assert_eq!(data, vec![0]);
    }
}
