//! The BPC (bad-pixel-config) mask: detector geometry, the image↔file index map,
//! and the mask drawing operations (port of `mask_io.cpp`,
//! `detector_geometry.cpp` and `tpx3_dual_quad_mapping.cpp`).
//!
//! A BPC file holds one byte per detector pixel in *chip* order; the mask PV
//! holds one element per pixel in *image* order. `pel_index` is the map between
//! them, and it depends on the chip count and the detector orientation.

use crate::family::DetectorFamily;

/// C `rowsCols`'s outputs (mask_io.cpp:251-313); all zero where C leaves
/// them so.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Geometry {
    pub rows: i32,
    pub cols: i32,
    pub x_chips: i32,
    pub y_chips: i32,
    pub pel_width: i32,
}

impl Geometry {
    /// The mask buffer's element count.
    pub fn pixel_count(&self) -> usize {
        usize::try_from(self.rows).unwrap_or(0) * usize::try_from(self.cols).unwrap_or(0)
    }
}

/// C `ADTimePix3DetectorGeometry::Status` (detector_geometry.h:23-29).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryError {
    InvalidArgument,
    PixelCountMismatch,
    NonSquareChip,
    NonIntegralChipGrid,
}

impl GeometryError {
    /// C `statusMessage` (detector_geometry.cpp:66-81).
    pub fn message(self) -> &'static str {
        match self {
            Self::InvalidArgument => "invalid detector geometry argument",
            Self::PixelCountMismatch => "detector raster and pixel count disagree",
            Self::NonSquareChip => "per-chip pixel count is not a square",
            Self::NonIntegralChipGrid => "detector raster does not form an integral chip grid",
        }
    }
}

/// C `ADTimePix3DetectorGeometry::derive` (detector_geometry.cpp:15-55).
pub fn derive(
    pixel_count: i32,
    row_length: i32,
    chips: i32,
    rows: i32,
) -> Result<Geometry, GeometryError> {
    if pixel_count <= 0 || row_length <= 0 || chips <= 0 || rows <= 0 {
        return Err(GeometryError::InvalidArgument);
    }
    if chips % row_length != 0 || pixel_count % chips != 0 {
        return Err(GeometryError::PixelCountMismatch);
    }
    let per_chip = pixel_count / chips;
    let width = f64::from(per_chip).sqrt() as i32;
    if width <= 0 || i64::from(width) * i64::from(width) != i64::from(per_chip) {
        return Err(GeometryError::NonSquareChip);
    }
    let x_chips = row_length;
    let y_chips = chips / x_chips;
    let raster_rows = i64::from(y_chips) * i64::from(width);
    let raster_cols = i64::from(x_chips) * i64::from(width);
    let (Ok(r), Ok(c)) = (i32::try_from(raster_rows), i32::try_from(raster_cols)) else {
        return Err(GeometryError::NonIntegralChipGrid);
    };
    if y_chips <= 0 || r != rows || raster_rows * raster_cols != i64::from(pixel_count) {
        return Err(GeometryError::NonIntegralChipGrid);
    }
    Ok(Geometry {
        rows: r,
        cols: c,
        x_chips,
        y_chips,
        pel_width: width,
    })
}

/// C `isTwoQuadLayout` (detector_geometry.cpp:57-64): two distinct TPX3 quad
/// boards.
pub fn is_two_quad_layout(first: &str, second: &str) -> bool {
    first.len() >= 2
        && second.len() >= 2
        && first.starts_with("41")
        && second.starts_with("41")
        && first != second
}

/// The parameters C `rowsCols` reads.
pub struct Layout<'a> {
    pub pix_count: i32,
    pub row_len: i32,
    pub num_chips: i32,
    pub num_rows: i32,
    pub tpx3: bool,
    pub boards: [&'a str; 2],
}

/// Why C `rowsCols` returns asynError.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RowsColsError {
    /// Eight chips that are not two TPX3 quads (mask_io.cpp:265-279).
    EightChipLayout,
    Geometry(GeometryError),
    /// `rowLength <= 0` on the one- and four-chip path, which C reports with
    /// no message (mask_io.cpp:306).
    NoRowLength,
}

/// C `rowsCols` (mask_io.cpp:251-313).
pub fn rows_cols(l: &Layout) -> Result<Geometry, RowsColsError> {
    if l.num_chips == 8 {
        if !l.tpx3 || !is_two_quad_layout(l.boards[0], l.boards[1]) {
            return Err(RowsColsError::EightChipLayout);
        }
        return derive(l.pix_count, l.row_len, l.num_chips, l.num_rows)
            .map_err(RowsColsError::Geometry);
    }
    if l.row_len <= 0 {
        return Err(RowsColsError::NoRowLength);
    }
    let y_chips = l.num_chips / l.row_len;
    let pel_width = l.num_rows / l.row_len;
    Ok(Geometry {
        rows: l.num_rows,
        cols: y_chips.wrapping_mul(pel_width),
        x_chips: l.row_len,
        y_chips,
        pel_width,
    })
}

/// C `pelIndex` (mask_io.cpp:989-1174) on the geometry `rowsCols` gave:
/// image pixel `(i, j)` → its logical BPC pixel, and the WARN line C prints
/// on the way. C returns 0, not -1, for an orientation or a chip count it
/// does not map.
pub fn pel_index(
    g: &Geometry,
    num_chips: i32,
    orientation: i32,
    i: i32,
    j: i32,
) -> (i32, Option<String>) {
    // C `findChip` (mask_io.cpp:734-748).
    let w = g.pel_width;
    if w <= 0 || i < 0 || i >= g.cols || j < 0 || j >= g.rows {
        return (
            -1,
            Some("pelIndex: image coordinate outside detector geometry".into()),
        );
    }
    let (xc, yc) = (i / w, j / w);
    let sq = w * w;
    let unsupported = |m: &str| (0, Some(m.to_string()));
    match num_chips {
        1 => {
            let index = match orientation {
                0 => i + (w - 1 - j) * w,
                1 => j + i * w,
                2 => (w - 1 - i) + j * w,
                3 => (w - 1 - j) + (w - 1 - i) * w,
                4 => (w - 1 - i) + (w - 1 - j) * w,
                5 => j + (w - 1 - i) * w,
                6 => i + j * w,
                7 => (w - 1 - j) + i * w,
                _ => return unsupported("pelIndex: single-chip orientation not supported"),
            };
            (index, None)
        }
        4 => {
            const NAMES: [&str; 8] = [
                "UP",
                "RIGHT",
                "DOWN",
                "LEFT",
                "UP MIRRORED",
                "RIGHT MIRRORED",
                "DOWN MIRRORED",
                "LEFT MIRRORED",
            ];
            let Some(name) = usize::try_from(orientation).ok().and_then(|o| NAMES.get(o)) else {
                return unsupported("pelIndex: quad orientation not supported for this layout");
            };
            // The tile-local coordinates C calls ii/jj.
            let (ii, jj) = (i - w, j - w);
            let index = match (orientation, xc, yc) {
                (0, 1, 1) => ii + (w - 1 - jj) * w,
                (0, 1, 0) => sq + (w - 1 - ii) + j * w,
                (0, 0, 0) => 2 * sq + (w - 1 - i) + j * w,
                (0, 0, 1) => 3 * sq + i + (w - 1 - jj) * w,

                (1, 1, 1) => sq + (w - 1 - jj) + (w - 1 - ii) * w,
                (1, 1, 0) => 2 * sq + (w - 1 - j) + (w - 1 - ii) * w,
                (1, 0, 0) => 3 * sq + j + i * w,
                (1, 0, 1) => jj + i * w,

                (2, 1, 1) => 2 * sq + ii + (w - 1 - jj) * w,
                (2, 1, 0) => 3 * sq + (w - 1 - ii) + j * w,
                (2, 0, 0) => (w - 1 - i) + j * w,
                (2, 0, 1) => sq + i + (w - 1 - jj) * w,

                (3, 1, 1) => 3 * sq + (w - 1 - jj) + (w - 1 - ii) * w,
                (3, 1, 0) => (w - 1 - j) + (w - 1 - ii) * w,
                (3, 0, 0) => sq + j + i * w,
                (3, 0, 1) => 2 * sq + jj + i * w,

                (4, 1, 1) => 3 * sq + (w - 1 - ii) + (w - 1 - jj) * w,
                (4, 1, 0) => 2 * sq + ii + j * w,
                (4, 0, 0) => sq + i + j * w,
                (4, 0, 1) => (w - 1 - i) + (w - 1 - jj) * w,

                (5, 1, 1) => jj + (w - 1 - ii) * w,
                (5, 1, 0) => 3 * sq + j + (w - 1 - ii) * w,
                (5, 0, 0) => 2 * sq + (w - 1 - j) + i * w,
                (5, 0, 1) => sq + (w - 1 - jj) + i * w,

                (6, 1, 1) => sq + (w - 1 - ii) + (w - 1 - jj) * w,
                (6, 1, 0) => ii + j * w,
                (6, 0, 0) => 3 * sq + i + j * w,
                (6, 0, 1) => 2 * sq + (w - 1 - i) + (w - 1 - jj) * w,

                (7, 1, 1) => 2 * sq + jj + (w - 1 - ii) * w,
                (7, 1, 0) => sq + j + (w - 1 - ii) * w,
                (7, 0, 0) => (w - 1 - j) + i * w,
                (7, 0, 1) => 3 * sq + (w - 1 - jj) + i * w,

                _ => {
                    return (
                        -1,
                        Some(format!(
                            "pelIndex: {name} layout chip index out of range for 2x2"
                        )),
                    );
                }
            };
            (index, None)
        }
        8 => {
            if orientation != 0 {
                return (
                    -1,
                    Some("pelIndex: 8-chip mapping only for DetectorOrientation UP (0)".into()),
                );
            }
            if g.x_chips != 4 || g.y_chips != 2 || xc >= g.x_chips || yc >= g.y_chips {
                return (
                    -1,
                    Some("pelIndex: two-quad mapping requires a 4x2 chip grid".into()),
                );
            }
            match dual_quad_image_to_bpc(i, j, w) {
                Some(index) => (index, None),
                None => (
                    -1,
                    Some("pelIndex: invalid two-quad image coordinate".into()),
                ),
            }
        }
        n => (
            0,
            Some(format!(
                "pelIndex: chip count {n} not supported (use 1, 4, or 8)"
            )),
        ),
    }
}

/// C `ADTimePix3Tpx3DualQuad::imageToBpc` (tpx3_dual_quad_mapping.cpp:91-109):
/// two TPX3 quads side by side, the right one turned 180 degrees.
///
/// ```text
///   2 1 | 4 7
///   3 0 | 5 6
/// ```
pub fn dual_quad_image_to_bpc(x: i32, y: i32, w: i32) -> Option<i32> {
    if w <= 0 || x < 0 || x >= 4 * w || y < 0 || y >= 2 * w {
        return None;
    }
    let (mut qx, mut qy, mut chip_offset) = (x, y, 0);
    if x >= 2 * w {
        qx = 4 * w - 1 - x;
        qy = 2 * w - 1 - y;
        chip_offset = 4 * w * w;
    }
    // C `quadImageToBpc` (:42-71).
    let (chip, lx, ly) = if qx >= w && qy >= w {
        (0, qx - w, 2 * w - 1 - qy)
    } else if qx >= w {
        (1, 2 * w - 1 - qx, qy)
    } else if qy < w {
        (2, w - 1 - qx, qy)
    } else {
        (3, qx, 2 * w - 1 - qy)
    };
    Some(chip_offset + chip * w * w + lx + ly * w)
}

/// C `ADTimePix3MaskGeometry::Status` failures; `message` is its
/// `statusMessage` (mask_geometry.cpp:123-135).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrawError {
    InvalidArgument,
    BufferTooSmall,
}

impl DrawError {
    pub fn message(self) -> &'static str {
        match self {
            Self::InvalidArgument => "invalid mask geometry or operation",
            Self::BufferTooSmall => "mask waveform is smaller than detector geometry",
        }
    }
}

/// C `validate` (mask_geometry.cpp:16-30): the `width * height` elements an
/// operation may touch. The image is C's (width, height): rowsCols' (COLS, ROWS).
fn validate(buf: &[i32], (width, height): (i32, i32)) -> Result<usize, DrawError> {
    if width <= 0 || height <= 0 {
        return Err(DrawError::InvalidArgument);
    }
    let required = (width as usize)
        .checked_mul(height as usize)
        .ok_or(DrawError::InvalidArgument)?;
    if buf.len() < required {
        return Err(DrawError::BufferTooSmall);
    }
    Ok(required)
}

fn set_mask_bit(cell: &mut i32, masked: bool) {
    if masked {
        *cell |= 1;
    } else {
        *cell &= !1;
    }
}

/// C `ADTimePix3MaskGeometry::reset`: assigns `value` to every element of the
/// image.
pub fn reset(buf: &mut [i32], image: (i32, i32), value: i32) -> Result<(), DrawError> {
    let required = validate(buf, image)?;
    buf[..required].fill(value);
    Ok(())
}

/// C `ADTimePix3MaskGeometry::rectangle`, clipped to the image.
pub fn rectangle(
    buf: &mut [i32],
    (width, height): (i32, i32),
    x: i32,
    size_x: i32,
    y: i32,
    size_y: i32,
    masked: bool,
) -> Result<(), DrawError> {
    validate(buf, (width, height))?;
    if size_x < 0 || size_y < 0 {
        return Err(DrawError::InvalidArgument);
    }
    let (x0, y0) = (i64::from(x).max(0), i64::from(y).max(0));
    let x1 = i64::from(width).min(i64::from(x) + i64::from(size_x));
    let y1 = i64::from(height).min(i64::from(y) + i64::from(size_y));
    for iy in y0..y1 {
        for ix in x0..x1 {
            set_mask_bit(&mut buf[(iy * i64::from(width) + ix) as usize], masked);
        }
    }
    Ok(())
}

/// C `ADTimePix3MaskGeometry::circle`, clipped to the image; C compares in
/// `long double`, exact here in `i128`.
pub fn circle(
    buf: &mut [i32],
    (width, height): (i32, i32),
    cx: i32,
    cy: i32,
    radius: i32,
    masked: bool,
) -> Result<(), DrawError> {
    validate(buf, (width, height))?;
    if radius < 0 {
        return Err(DrawError::InvalidArgument);
    }
    let (cx, cy, r) = (i64::from(cx), i64::from(cy), i64::from(radius));
    let r2 = i128::from(r) * i128::from(r);
    for iy in (cy - r).max(0)..=(cy + r).min(i64::from(height) - 1) {
        for ix in (cx - r).max(0)..=(cx + r).min(i64::from(width) - 1) {
            let (dx, dy) = (i128::from(ix - cx), i128::from(iy - cy));
            if dx * dx + dy * dy <= r2 {
                set_mask_bit(&mut buf[(iy * i64::from(width) + ix) as usize], masked);
            }
        }
    }
    Ok(())
}

/// C `ADTimePix3BpcMask::operatorMaskSupported`: the families whose
/// per-pixel mask bit is documented.
pub fn operator_mask_supported(family: DetectorFamily) -> bool {
    matches!(family, DetectorFamily::Tpx3 | DetectorFamily::Mpx3)
}

/// C `ADTimePix3BpcMask::bytesPerPixel`; zero for an unsupported family.
pub fn bytes_per_pixel(family: DetectorFamily) -> usize {
    match family {
        DetectorFamily::Tpx3 => 1,
        DetectorFamily::Mpx3 => 2,
        DetectorFamily::Unknown => 0,
    }
}

/// C `ADTimePix3BpcMask::pixelValue`: the packed pixel at `offset`, MPX3 words
/// big-endian.
pub fn pixel_value(family: DetectorFamily, bytes: &[u8], offset: usize) -> Option<u16> {
    let width = bytes_per_pixel(family);
    let pixel = bytes.get(offset..offset.checked_add(width)?)?;
    match *pixel {
        [b] => Some(u16::from(b)),
        [hi, lo] => Some(u16::from_be_bytes([hi, lo])),
        _ => None,
    }
}

/// C `ADTimePix3BpcMask::isMasked`: bit 0 of the packed pixel.
pub fn is_masked(family: DetectorFamily, bytes: &[u8], offset: usize) -> bool {
    pixel_value(family, bytes, offset).is_some_and(|v| v & 1 != 0)
}

/// C `ADTimePix3BpcMask::countMasked`.
pub fn count_masked(family: DetectorFamily, bytes: &[u8]) -> usize {
    match bytes_per_pixel(family) {
        0 => 0,
        width => (0..bytes.len() / width)
            .filter(|&k| is_masked(family, bytes, k * width))
            .count(),
    }
}

/// C `ADTimePix3BpcMask::setMasked`: bit 0 of the pixel's last byte, every
/// other bit kept.
pub fn set_masked(family: DetectorFamily, bytes: &mut [u8], offset: usize, masked: bool) -> bool {
    let width = bytes_per_pixel(family);
    let Some(last) = offset
        .checked_add(width)
        .filter(|&end| width != 0 && end <= bytes.len())
        .map(|end| end - 1)
    else {
        return false;
    };
    if masked {
        bytes[last] |= 1;
    } else {
        bytes[last] &= !1;
    }
    true
}

/// C `ADTimePix3ServalPixelConfig::bytesPerChip`; zero for an invalid or
/// overflowing geometry.
pub fn bytes_per_chip(pixels_per_chip: usize, bytes_per_pixel: i32, slices: i32) -> usize {
    if pixels_per_chip == 0 || bytes_per_pixel <= 0 || slices <= 0 {
        return 0;
    }
    (bytes_per_pixel as usize)
        .checked_mul(slices as usize)
        .and_then(|m| pixels_per_chip.checked_mul(m))
        .unwrap_or(0)
}

/// C `ADTimePix3ServalPixelConfig::selectedSliceIndex`: the first byte of
/// logical pixel `logical` in threshold slice `selected`.
pub fn selected_slice_index(
    logical: usize,
    pixels_per_chip: usize,
    bytes_per_pixel: i32,
    slices: i32,
    selected: i32,
) -> Option<usize> {
    let chip_bytes = bytes_per_chip(pixels_per_chip, bytes_per_pixel, slices);
    if chip_bytes == 0 || selected < 0 || selected >= slices {
        return None;
    }
    let bpp = bytes_per_pixel as usize;
    let (chip, local) = (logical / pixels_per_chip, logical % pixels_per_chip);
    chip.checked_mul(chip_bytes)?
        .checked_add(selected as usize * pixels_per_chip * bpp)?
        .checked_add(local * bpp)
}

/// C `ADTimePix3ServalPixelConfig::mpx3LayoutCoordinates`: a chip-local
/// pixel through one Serval Layout chip entry, whose origin is bottom-left,
/// to top-left image coordinates; an unknown orientation is refused.
pub fn mpx3_layout_coordinates(
    (local_x, local_y): (i32, i32),
    width: i32,
    (origin_x, origin_y): (i32, i32),
    image_height: i32,
    orientation: &str,
) -> Option<(i32, i32)> {
    if width <= 0
        || !(0..width).contains(&local_x)
        || !(0..width).contains(&local_y)
        || origin_x < 0
        || origin_y < 0
        || image_height < width
        || origin_y > image_height - width
    {
        return None;
    }
    let (fx, fy) = (width - 1 - local_x, width - 1 - local_y);
    let (x, y) = match orientation {
        "RtLBtT" => (fx, fy),
        "LtRTtB" => (local_x, local_y),
        "BtTLtR" => (local_y, fx),
        "TtBRtL" => (fy, local_x),
        "LtRBtT" => (local_x, fy),
        "RtLTtB" => (fx, local_y),
        "TtBLtR" => (local_y, local_x),
        "BtTRtL" => (fy, fx),
        _ => return None,
    };
    Some((origin_x + x, image_height - width - origin_y + y))
}

/// C `readInt32Array`'s MaskPel loop (mask_io.cpp:139-151): bit 1 wherever
/// the BPC pixel is masked; an empty BPC leaves the waveform untouched.
pub fn mask_from_bpc(
    family: DetectorFamily,
    offsets: &[Option<usize>],
    bpc: &[u8],
    out: &mut [i32],
) {
    if bpc.is_empty() {
        return;
    }
    out.fill(0);
    for (dst, offset) in out.iter_mut().zip(offsets) {
        if offset.is_some_and(|k| is_masked(family, bpc, k)) {
            *dst |= 1 << 1;
        }
    }
}

/// C `readInt32Array`'s MaskWrite loop (mask_io.cpp:174-182): masks the BPC
/// pixel of every waveform element with bit 0; it never unmasks.
pub fn apply_mask_to_bpc(
    family: DetectorFamily,
    offsets: &[Option<usize>],
    mask: &[i32],
    bpc: &mut [u8],
) {
    for (&v, offset) in mask.iter().zip(offsets) {
        if let Some(k) = *offset
            && v & 1 != 0
        {
            set_masked(family, bpc, k, true);
        }
    }
}

/// C `ADTimePix3ServalPixelConfig::ResponseError`; `None` is the `Ok` arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelConfigError {
    HttpFailure,
    EmptyBody,
    MalformedJson,
    InvalidRoot,
    InvalidBase64,
    LengthMismatch,
}

impl PixelConfigError {
    /// C `responseErrorMessage`.
    pub fn message(self) -> &'static str {
        match self {
            Self::HttpFailure => "HTTP request failed",
            Self::EmptyBody => "Empty response body",
            Self::MalformedJson => "Malformed JSON response",
            Self::InvalidRoot => "JSON response is not a string",
            Self::InvalidBase64 => "Invalid base64 payload",
            Self::LengthMismatch => "Decoded PixelConfig length mismatch",
        }
    }
}

/// C `isspace` in the C locale, which `is_ascii_whitespace` lacks `\v` of.
fn c_isspace(c: u8) -> bool {
    c.is_ascii_whitespace() || c == 0x0b
}

/// C `decodeBase64Strict`: whitespace dropped, whole groups of four, and
/// padding only in the last group with its unused bits zero.
pub fn decode_base64_strict(encoded: &str) -> Option<Vec<u8>> {
    let compact: Vec<u8> = encoded.bytes().filter(|&c| !c_isspace(c)).collect();
    if compact.is_empty() || !compact.len().is_multiple_of(4) {
        return None;
    }
    let value = |c: u8| -> Option<u8> {
        Some(match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return None,
        })
    };
    let mut decoded = Vec::with_capacity(compact.len() / 4 * 3);
    let groups = compact.len() / 4;
    for (n, group) in compact.as_chunks::<4>().0.iter().enumerate() {
        let last = n + 1 == groups;
        let (a, b) = (value(group[0])?, value(group[1])?);
        if group[2] == b'=' {
            if !last || group[3] != b'=' || b & 0x0f != 0 {
                return None;
            }
            decoded.push((a << 2) | (b >> 4));
            continue;
        }
        let c = value(group[2])?;
        decoded.push((a << 2) | (b >> 4));
        if group[3] == b'=' {
            if !last || c & 0x03 != 0 {
                return None;
            }
            decoded.push((b << 4) | (c >> 2));
            continue;
        }
        let d = value(group[3])?;
        decoded.push((b << 4) | (c >> 2));
        decoded.push((c << 6) | d);
    }
    Some(decoded)
}

/// C `ADTimePix3ServalPixelConfig::parseResponse`: a 200 reply whose body is
/// a JSON string of exactly `expected` bytes in strict base64.
pub fn parse_pixel_config_response(
    code: i32,
    body: &str,
    expected: usize,
) -> Result<Vec<u8>, PixelConfigError> {
    if code != 200 {
        return Err(PixelConfigError::HttpFailure);
    }
    if body.is_empty() {
        return Err(PixelConfigError::EmptyBody);
    }
    let parsed: serde_json::Value =
        serde_json::from_str(body).map_err(|_| PixelConfigError::MalformedJson)?;
    let serde_json::Value::String(encoded) = parsed else {
        return Err(PixelConfigError::InvalidRoot);
    };
    if expected == 0 || expected > usize::MAX - 2 {
        return Err(PixelConfigError::LengthMismatch);
    }
    let characters = encoded.bytes().filter(|&c| !c_isspace(c)).count();
    let expected_characters = expected.div_ceil(3);
    if expected_characters.checked_mul(4) != Some(characters) {
        return Err(PixelConfigError::LengthMismatch);
    }
    let decoded = decode_base64_strict(&encoded).ok_or(PixelConfigError::InvalidBase64)?;
    if decoded.len() != expected {
        return Err(PixelConfigError::LengthMismatch);
    }
    Ok(decoded)
}

/// C `ADTimePix3ServalPixelConfig::absolutePackedDifference`: `|a − b|` of
/// the big-endian pixel at `offset`, one or two bytes wide.
pub fn absolute_packed_difference(
    first: &[u8],
    second: &[u8],
    offset: usize,
    bytes_per_pixel: i32,
) -> Option<u32> {
    if !(1..=2).contains(&bytes_per_pixel) {
        return None;
    }
    let range = offset..offset.checked_add(bytes_per_pixel as usize)?;
    let pack = |v: &[u8]| -> Option<u32> {
        Some(
            v.get(range.clone())?
                .iter()
                .fold(0, |acc, &b| (acc << 8) | u32::from(b)),
        )
    };
    Some(pack(first)?.abs_diff(pack(second)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layout(row_len: i32, num_chips: i32, num_rows: i32) -> Layout<'static> {
        Layout {
            pix_count: num_chips * 65536,
            row_len,
            num_chips,
            num_rows,
            tpx3: true,
            boards: ["4100", "4101"],
        }
    }

    /// The single-chip TimePix3: 256x256, one chip.
    fn single() -> Geometry {
        rows_cols(&layout(1, 1, 256)).unwrap()
    }

    /// The quad 2x2: 512x512, four 256x256 chips, two per row.
    fn quad() -> Geometry {
        rows_cols(&layout(2, 4, 512)).unwrap()
    }

    /// `pel_index` over the whole image, as the driver tabulates it.
    fn table(g: &Geometry, chips: i32, o: i32) -> Vec<i32> {
        (0..g.rows)
            .flat_map(|j| (0..g.cols).map(move |i| (i, j)))
            .map(|(i, j)| pel_index(g, chips, o, i, j).0)
            .collect()
    }

    #[test]
    fn the_geometry_matches_c_for_the_shipped_layouts() {
        let g = single();
        assert_eq!((g.rows, g.cols), (256, 256));
        assert_eq!((g.x_chips, g.y_chips, g.pel_width), (1, 1, 256));

        let g = quad();
        assert_eq!((g.rows, g.cols), (512, 512));
        assert_eq!((g.x_chips, g.y_chips, g.pel_width), (2, 2, 256));
        assert_eq!(g.pixel_count(), 512 * 512);
    }

    /// C `rowsCols` (mask_io.cpp:251-313): no row length is a silent error,
    /// and the one/four-chip path keeps C's arithmetic unchecked.
    #[test]
    fn rows_cols_follows_c() {
        assert_eq!(
            rows_cols(&layout(0, 4, 512)),
            Err(RowsColsError::NoRowLength)
        );
        assert_eq!(
            rows_cols(&layout(4, 2, 512)),
            Ok(Geometry {
                rows: 512,
                cols: 0,
                x_chips: 4,
                y_chips: 0,
                pel_width: 128
            })
        );
    }

    /// The eight-chip path needs two distinct TPX3 quad boards and a raster
    /// `derive` accepts (mask_io.cpp:258-301, detector_geometry.cpp:15-64).
    #[test]
    fn the_eight_chip_layout_is_two_tpx3_quads() {
        assert_eq!(
            rows_cols(&layout(4, 8, 512)),
            Ok(Geometry {
                rows: 512,
                cols: 1024,
                x_chips: 4,
                y_chips: 2,
                pel_width: 256
            })
        );
        let mut l = layout(4, 8, 512);
        l.boards = ["4100", "4100"];
        assert_eq!(rows_cols(&l), Err(RowsColsError::EightChipLayout));
        l.boards = ["4100", "4101"];
        l.tpx3 = false;
        assert_eq!(rows_cols(&l), Err(RowsColsError::EightChipLayout));
        assert_eq!(
            rows_cols(&layout(4, 8, 256)),
            Err(RowsColsError::Geometry(GeometryError::NonIntegralChipGrid))
        );
        let mut l = layout(4, 8, 512);
        l.pix_count = 8 * 1000;
        assert_eq!(
            rows_cols(&l),
            Err(RowsColsError::Geometry(GeometryError::NonSquareChip))
        );
        l.pix_count = 8 * 65536 + 1;
        assert_eq!(
            rows_cols(&l),
            Err(RowsColsError::Geometry(GeometryError::PixelCountMismatch))
        );
        assert_eq!(
            GeometryError::NonSquareChip.message(),
            "per-chip pixel count is not a square"
        );
    }

    #[test]
    fn pel_index_is_a_bijection_for_every_supported_layout() {
        for (g, chips, orientations) in [(single(), 1, 0..8), (quad(), 4, 0..8)] {
            for o in orientations {
                let mut seen = vec![false; g.pixel_count()];
                for k in table(&g, chips, o) {
                    let k = usize::try_from(k).unwrap();
                    assert!(!seen[k], "chips {chips} o {o}: {k} hit twice");
                    seen[k] = true;
                }
                assert!(seen.iter().all(|&b| b), "chips {chips} o {o}: not onto");
            }
        }
        let g = rows_cols(&layout(4, 8, 512)).unwrap();
        let mut seen = vec![false; g.pixel_count()];
        for k in table(&g, 8, 0) {
            let k = usize::try_from(k).unwrap();
            assert!(!seen[k]);
            seen[k] = true;
        }
    }

    /// C's two-quad table (tpx3_dual_quad_mapping.h:14-19): the right quad is
    /// the left one turned 180 degrees.
    #[test]
    fn the_two_quad_map_matches_c() {
        let w = 256;
        let chip = |x, y| dual_quad_image_to_bpc(x, y, w).unwrap() / (w * w);
        assert_eq!(
            [chip(0, 0), chip(300, 0), chip(600, 0), chip(900, 0)],
            [2, 1, 4, 7]
        );
        assert_eq!(
            [chip(0, 300), chip(300, 300), chip(600, 300), chip(900, 300)],
            [3, 0, 5, 6]
        );
        assert_eq!(dual_quad_image_to_bpc(4 * w, 0, w), None);
    }

    /// C's WARN lines and its 0 / -1 returns (mask_io.cpp:996-1171).
    #[test]
    fn pel_index_warns_as_c() {
        let g = quad();
        assert_eq!(
            pel_index(&g, 4, 9, 0, 0),
            (
                0,
                Some("pelIndex: quad orientation not supported for this layout".into())
            )
        );
        assert_eq!(
            pel_index(&g, 4, 0, 512, 0),
            (
                -1,
                Some("pelIndex: image coordinate outside detector geometry".into())
            )
        );
        assert_eq!(
            pel_index(&g, 2, 0, 0, 0),
            (
                0,
                Some("pelIndex: chip count 2 not supported (use 1, 4, or 8)".into())
            )
        );
        let wide = Geometry { cols: 768, ..g };
        assert_eq!(
            pel_index(&wide, 4, 3, 600, 0),
            (
                -1,
                Some("pelIndex: LEFT layout chip index out of range for 2x2".into())
            )
        );
        let g8 = rows_cols(&layout(4, 8, 512)).unwrap();
        assert_eq!(
            pel_index(&g8, 8, 1, 0, 0),
            (
                -1,
                Some("pelIndex: 8-chip mapping only for DetectorOrientation UP (0)".into())
            )
        );
    }

    #[test]
    fn a_single_chip_up_mask_maps_to_the_c_index() {
        // C: index = i + ((W-1) - j)*W.
        let g = single();
        assert_eq!(pel_index(&g, 1, 0, 0, 0), (255 * 256, None));
        assert_eq!(pel_index(&g, 1, 0, 5, 255), (5, None));
    }

    fn image(g: &Geometry) -> (i32, i32) {
        (g.cols, g.rows)
    }

    fn masked(buf: &[i32]) -> usize {
        buf.iter().filter(|&&v| v & 1 != 0).count()
    }

    #[test]
    fn reset_assigns_the_value_as_c() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count() + 3];
        buf[7] = 1 << 1;
        buf[g.pixel_count()] = 9;
        reset(&mut buf, image(&g), 5).unwrap();
        assert_eq!(buf[7], 5);
        assert!(buf[..g.pixel_count()].iter().all(|&v| v == 5));
        // Past the image the waveform is left alone.
        assert_eq!(buf[g.pixel_count()], 9);
    }

    #[test]
    fn a_draw_is_refused_with_c_status() {
        let g = single();
        let mut small = vec![0i32; g.pixel_count() - 1];
        for r in [
            reset(&mut small, image(&g), 1),
            rectangle(&mut small, image(&g), 0, 1, 0, 1, true),
            circle(&mut small, image(&g), 0, 0, 1, true),
        ] {
            assert_eq!(r, Err(DrawError::BufferTooSmall));
        }
        let mut buf = vec![0i32; g.pixel_count()];
        for r in [
            reset(&mut buf, (0, 0), 1),
            rectangle(&mut buf, image(&g), 0, -1, 0, 1, true),
            rectangle(&mut buf, image(&g), 0, 1, 0, -1, true),
            circle(&mut buf, image(&g), 3, 4, -2, true),
        ] {
            assert_eq!(r, Err(DrawError::InvalidArgument));
        }
        assert_eq!(rectangle(&mut buf, image(&g), 0, 0, 0, 5, true), Ok(()));
        assert_eq!(masked(&buf), 0);
        assert_eq!(
            DrawError::BufferTooSmall.message(),
            "mask waveform is smaller than detector geometry"
        );
        assert_eq!(
            DrawError::InvalidArgument.message(),
            "invalid mask geometry or operation"
        );
    }

    #[test]
    fn a_rectangle_sets_exactly_its_pixels() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count()];
        rectangle(&mut buf, image(&g), 2, 3, 4, 2, true).unwrap();
        let on: Vec<usize> = buf
            .iter()
            .enumerate()
            .filter(|&(_, &v)| v & 1 != 0)
            .map(|(k, _)| k)
            .collect();
        let want: Vec<usize> = (4..6)
            .flat_map(|j| (2..5).map(move |i| j * 256 + i))
            .collect();
        assert_eq!(on, want);

        rectangle(&mut buf, image(&g), 2, 3, 4, 2, false).unwrap();
        assert!(buf.iter().all(|&v| v == 0));
    }

    #[test]
    fn a_rectangle_is_clipped_to_the_detector() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count()];
        rectangle(&mut buf, image(&g), 254, 10, 254, 10, true).unwrap();
        assert_eq!(masked(&buf), 4);
        assert_eq!(buf[255 * 256 + 255] & 1, 1);
        let mut buf = vec![0i32; g.pixel_count()];
        rectangle(&mut buf, image(&g), -2, 3, -1, 2, true).unwrap();
        assert_eq!(masked(&buf), 1);
        assert_eq!(buf[0] & 1, 1);
    }

    #[test]
    fn a_circle_sets_the_disc() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count()];
        circle(&mut buf, image(&g), 10, 10, 1, true).unwrap();
        assert_eq!(masked(&buf), 5);
        for (i, j) in [(10, 10), (9, 10), (11, 10), (10, 9), (10, 11)] {
            assert_eq!(buf[j * 256 + i] & 1, 1, "({i},{j})");
        }
        let mut buf = vec![0i32; g.pixel_count()];
        circle(&mut buf, image(&g), 3, 4, 0, true).unwrap();
        assert_eq!(masked(&buf), 1);
    }

    #[test]
    fn a_circle_at_the_edge_does_not_wrap_or_overflow() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count()];
        circle(&mut buf, image(&g), 0, 0, 2, true).unwrap();
        // A quarter disc: (0,0),(1,0),(2,0),(0,1),(1,1),(0,2).
        assert_eq!(masked(&buf), 6);
        assert_eq!(buf[2 * 256] & 1, 1);
        let mut buf = vec![0i32; g.pixel_count()];
        circle(&mut buf, image(&g), i32::MAX, i32::MIN, i32::MAX, true).unwrap();
    }

    fn offsets(map: &[i32], width: usize) -> Vec<Option<usize>> {
        map.iter()
            .map(|&k| usize::try_from(k).ok().map(|k| k * width))
            .collect()
    }

    #[test]
    fn the_mask_round_trips_through_a_bpc_buffer() {
        let g = quad();
        let mut mask = vec![0i32; g.pixel_count()];
        rectangle(&mut mask, image(&g), 300, 2, 100, 2, true).unwrap();
        for (family, width) in [(DetectorFamily::Tpx3, 1), (DetectorFamily::Mpx3, 2)] {
            let mut bpc = vec![0u8; 4 * CHIP_PIXELS * width];
            let at = offsets(&table(&g, 4, 0), width);
            apply_mask_to_bpc(family, &at, &mask, &mut bpc);
            // MPX3 masks the low byte of the big-endian word.
            assert_eq!(
                bpc.iter()
                    .enumerate()
                    .filter(|&(k, &b)| b != 0 && k % width == width - 1)
                    .count(),
                4
            );
            assert_eq!(count_masked(family, &bpc), 4);

            let mut back = vec![7i32; g.pixel_count()];
            mask_from_bpc(family, &at, &bpc, &mut back);
            for j in 100..102 {
                for i in 300..302 {
                    assert_eq!(back[j * 512 + i], 1 << 1, "({i},{j})");
                }
            }
            assert_eq!(back.iter().filter(|&&v| v != 0).count(), 4);
        }
    }

    #[test]
    fn the_bpc_mask_bit_is_read_and_set_per_family_as_c() {
        let (t, m) = (DetectorFamily::Tpx3, DetectorFamily::Mpx3);
        let bytes = [0x81u8, 0x01, 0x00];
        assert_eq!(pixel_value(t, &bytes, 0), Some(0x81));
        assert_eq!(pixel_value(m, &bytes, 0), Some(0x8101));
        assert_eq!(pixel_value(m, &bytes, 2), None);
        assert_eq!(pixel_value(DetectorFamily::Unknown, &bytes, 0), None);
        assert!(is_masked(m, &bytes, 0) && !is_masked(m, &bytes, 1));
        assert_eq!(count_masked(t, &bytes), 2);
        assert_eq!(count_masked(m, &bytes), 1);
        let mut b = [0xfeu8, 0xfe];
        assert!(set_masked(m, &mut b, 0, true));
        assert_eq!(b, [0xfe, 0xff]);
        assert!(set_masked(m, &mut b, 0, false));
        assert_eq!(b, [0xfe, 0xfe]);
        assert!(!set_masked(m, &mut b, 1, true));
        assert!(!set_masked(DetectorFamily::Unknown, &mut b, 0, true));
        // An empty BPC leaves the waveform as it was.
        let mut out = [5i32; 2];
        mask_from_bpc(t, &[Some(0), Some(1)], &[], &mut out);
        assert_eq!(out, [5, 5]);
        assert!(operator_mask_supported(t) && !operator_mask_supported(DetectorFamily::Unknown));
    }

    #[test]
    fn a_slice_index_and_a_layout_coordinate_are_c_serval_pixel_config() {
        assert_eq!(bytes_per_chip(65536, 2, 2), 262144);
        assert_eq!(bytes_per_chip(65536, 0, 1), 0);
        assert_eq!(
            selected_slice_index(65536 + 3, 65536, 2, 2, 1),
            Some(262144 + 131072 + 6)
        );
        assert_eq!(selected_slice_index(3, 65536, 2, 2, 2), None);
        assert_eq!(selected_slice_index(3, 65536, 1, 1, 0), Some(3));
        let at = |o| mpx3_layout_coordinates((1, 2), 4, (4, 0), 8, o);
        assert_eq!(at("LtRTtB"), Some((5, 6)));
        assert_eq!(at("RtLBtT"), Some((6, 5)));
        assert_eq!(at("BtTLtR"), Some((6, 6)));
        assert_eq!(at("TtBRtL"), Some((5, 5)));
        assert_eq!(at("LtRBtT"), Some((5, 5)));
        assert_eq!(at("RtLTtB"), Some((6, 6)));
        assert_eq!(at("TtBLtR"), Some((6, 5)));
        assert_eq!(at("BtTRtL"), Some((5, 6)));
        assert_eq!(at("Up"), None);
        assert_eq!(
            mpx3_layout_coordinates((0, 0), 4, (0, 5), 8, "LtRTtB"),
            None
        );
        assert_eq!(
            mpx3_layout_coordinates((4, 0), 4, (0, 0), 8, "LtRTtB"),
            None
        );
    }

    /// One 256 x 256 chip.
    const CHIP_PIXELS: usize = 65536;

    #[test]
    fn a_pixel_config_reply_is_a_json_string_of_exactly_the_chip_bytes() {
        use PixelConfigError::*;
        let parse = |code, body: &str, n| parse_pixel_config_response(code, body, n);
        assert_eq!(parse(200, r#""Zm9v""#, 3), Ok(b"foo".to_vec()));
        assert_eq!(parse(200, "\"Zm9v\\nYmFy\"", 6), Ok(b"foobar".to_vec()));
        assert_eq!(parse(200, r#""Zm8=""#, 2), Ok(b"fo".to_vec()));
        assert_eq!(parse(500, r#""Zm9v""#, 3), Err(HttpFailure));
        assert_eq!(parse(200, "", 3), Err(EmptyBody));
        // A bare payload is not JSON.
        assert_eq!(parse(200, "Zm9v", 3), Err(MalformedJson));
        assert_eq!(parse(200, "[1]", 3), Err(InvalidRoot));
        assert_eq!(parse(200, r#""Zm9v""#, 0), Err(LengthMismatch));
        assert_eq!(parse(200, r#""Zm9vYmFy""#, 3), Err(LengthMismatch));
        assert_eq!(parse(200, r#""Zm9!""#, 3), Err(InvalidBase64));
        // Padding with non-zero unused bits, and padding before the end.
        assert_eq!(parse(200, r#""Zm9=""#, 2), Err(InvalidBase64));
        assert_eq!(parse(200, r#""Zg==Zm9v""#, 5), Err(InvalidBase64));
        // C isspace counts \v; a NUL is not base64.
        assert_eq!(decode_base64_strict("Zm9v\u{b}"), Some(b"foo".to_vec()));
        assert_eq!(decode_base64_strict("AA\0A"), None);
        assert_eq!(decode_base64_strict("!!!!"), None);
        assert_eq!(decode_base64_strict("AAAA"), Some(vec![0, 0, 0]));
        assert_eq!(decode_base64_strict("/w=="), Some(vec![0xff]));
        assert_eq!(decode_base64_strict(""), None);
    }

    #[test]
    fn the_packed_difference_is_absolute_and_big_endian() {
        assert_eq!(absolute_packed_difference(&[200], &[5], 0, 1), Some(195));
        assert_eq!(absolute_packed_difference(&[1], &[5], 0, 1), Some(4));
        assert_eq!(
            absolute_packed_difference(&[0, 1, 0], &[0, 0, 255], 1, 2),
            Some(0x0100 - 0x00ff)
        );
        assert_eq!(absolute_packed_difference(&[1, 2], &[1, 2], 1, 2), None);
        assert_eq!(absolute_packed_difference(&[1], &[1], 0, 3), None);
    }
}
