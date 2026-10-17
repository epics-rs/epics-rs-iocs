//! The BPC (bad-pixel-config) mask: detector geometry, the image↔file index map,
//! and the mask drawing operations (port of `mask_io.cpp`,
//! `detector_geometry.cpp` and `tpx3_dual_quad_mapping.cpp`).
//!
//! A BPC file holds one byte per detector pixel in *chip* order; the mask PV
//! holds one element per pixel in *image* order. `pel_index` is the map between
//! them, and it depends on the chip count and the detector orientation.

/// One chip's pixel-config bytes (C `kPixelConfigBytes`, serval_http.cpp:1011).
pub const PIXEL_CONFIG_BYTES: usize = 65536;

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

    /// The row-major offset of image pixel `(i, j)`.
    fn offset(&self, i: usize, j: usize) -> Option<usize> {
        let cols = usize::try_from(self.cols).ok()?;
        if i >= cols || j >= usize::try_from(self.rows).ok()? {
            return None;
        }
        Some(j * cols + i)
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

/// The mask bit the driver draws with (C `1 << 0`).
const MASK_BIT: i32 = 1 << 0;

/// Set or clear the mask bit on every pixel (C `maskReset`, mask_io.cpp:229).
///
/// UPSTREAM DEFECT (mask_io.cpp:238): C *assigns* `OnOff` to the whole element,
/// wiping the other bits the waveform carries (bit 1, the "in the BPC file"
/// flag the mask read at mask_io.cpp:141 sets, and bit 8 from the BPC PV) —
/// while the rectangle and circle operations right below it set and clear bit 0
/// alone. Reset now touches the same one bit.
pub fn mask_reset(geom: &Geometry, buf: &mut [i32], on: bool) {
    for k in 0..geom.pixel_count().min(buf.len()) {
        set_bit(&mut buf[k], on);
    }
}

/// C `maskRectangle` (mask_io.cpp:244).
pub fn mask_rectangle(
    geom: &Geometry,
    buf: &mut [i32],
    x: i32,
    x_size: i32,
    y: i32,
    y_size: i32,
    on: bool,
) {
    for j in y..y.saturating_add(y_size) {
        for i in x..x.saturating_add(x_size) {
            let (Ok(i), Ok(j)) = (usize::try_from(i), usize::try_from(j)) else {
                continue;
            };
            if let Some(k) = geom.offset(i, j)
                && k < buf.len()
            {
                set_bit(&mut buf[k], on);
            }
        }
    }
}

/// C `maskCircle` (mask_io.cpp:267).
pub fn mask_circle(geom: &Geometry, buf: &mut [i32], x: i32, y: i32, radius: i32, on: bool) {
    if radius < 0 {
        return;
    }
    for j in y - radius..=y + radius {
        for i in x - radius..=x + radius {
            if (i - x) * (i - x) + (j - y) * (j - y) > radius * radius {
                continue;
            }
            let (Ok(iu), Ok(ju)) = (usize::try_from(i), usize::try_from(j)) else {
                continue;
            };
            if let Some(k) = geom.offset(iu, ju)
                && k < buf.len()
            {
                set_bit(&mut buf[k], on);
            }
        }
    }
}

fn set_bit(cell: &mut i32, on: bool) {
    if on {
        *cell |= MASK_BIT;
    } else {
        *cell &= !MASK_BIT;
    }
}

/// The mask waveform Serval's BPC file implies: bit 1 set wherever the file has
/// its mask bit set (C mask_io.cpp:131-146). `map` is `pel_index` per image
/// pixel, negative where C has none.
pub fn mask_from_bpc(map: &[i32], bpc: &[u8], out: &mut [i32]) {
    out.fill(0);
    for (dst, &k) in map.iter().enumerate() {
        let Ok(k) = usize::try_from(k) else { continue };
        if dst < out.len() && bpc.get(k).is_some_and(|b| b & 1 != 0) {
            out[dst] |= 1 << 1;
        }
    }
}

/// Apply the mask waveform to a BPC buffer, ready to be written back (C
/// mask_io.cpp:153-161). Out-of-range indices are skipped and counted.
pub fn apply_mask_to_bpc(map: &[i32], mask: &[i32], bpc: &mut [u8]) -> usize {
    let mut dropped = 0;
    for (src, &k) in map.iter().enumerate() {
        if src >= mask.len() || mask[src] & MASK_BIT == 0 {
            continue;
        }
        match usize::try_from(k) {
            Ok(k) if k < bpc.len() => bpc[k] |= 1,
            _ => dropped += 1,
        }
    }
    dropped
}

/// `|SERVAL − BPC|` per image pixel (C serval_http.cpp:1120-1142).
///
/// `serval` is the chip pixel-config bytes concatenated in chip order, `bpc`
/// the file on disk. A pixel whose chip did not decode contributes 0.
pub fn pixel_config_diff(map: &[i32], serval: &[u8], bpc: &[u8], out: &mut [i32]) {
    out.fill(0);
    for (dst, &k) in map.iter().enumerate() {
        let Ok(k) = usize::try_from(k) else { continue };
        if dst >= out.len() {
            continue;
        }
        let (Some(&a), Some(&b)) = (serval.get(k), bpc.get(k)) else {
            continue;
        };
        out[dst] = i32::from(a.abs_diff(b));
    }
}

/// The masked pixels of a BPC file, as image coordinates (C
/// `exportMaskedPelsJsonFromBpcBuffer`, serval_http.cpp:920-940).
pub fn masked_pixels(geom: &Geometry, map: &[i32], bpc: &[u8]) -> Vec<(usize, usize)> {
    let cols = usize::try_from(geom.cols).unwrap_or(0).max(1);
    map.iter()
        .enumerate()
        .filter(|&(_, &k)| usize::try_from(k).is_ok_and(|k| bpc.get(k).is_some_and(|b| b & 1 != 0)))
        .map(|(n, _)| (n % cols, n / cols))
        .collect()
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

    #[test]
    fn reset_touches_only_the_mask_bit() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count()];
        // Bit 1 is the "present in the BPC file" flag the mask read sets.
        buf[7] = 1 << 1;
        mask_reset(&g, &mut buf, true);
        assert_eq!(buf[7], (1 << 1) | 1, "C's reset assigns and wipes bit 1");
        assert_eq!(buf[0], 1);
        mask_reset(&g, &mut buf, false);
        assert_eq!(buf[7], 1 << 1);
        assert_eq!(buf[0], 0);
    }

    #[test]
    fn a_rectangle_sets_exactly_its_pixels() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count()];
        mask_rectangle(&g, &mut buf, 2, 3, 4, 2, true);
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

        mask_rectangle(&g, &mut buf, 2, 3, 4, 2, false);
        assert!(buf.iter().all(|&v| v == 0));
    }

    #[test]
    fn a_rectangle_is_clipped_to_the_detector() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count()];
        mask_rectangle(&g, &mut buf, 254, 10, 254, 10, true);
        // Only the 2x2 corner survives.
        assert_eq!(buf.iter().filter(|&&v| v & 1 != 0).count(), 4);
        assert_eq!(buf[255 * 256 + 255] & 1, 1);
        // A negative origin clips too, rather than indexing backwards.
        let mut buf = vec![0i32; g.pixel_count()];
        mask_rectangle(&g, &mut buf, -2, 3, -1, 2, true);
        assert_eq!(buf.iter().filter(|&&v| v & 1 != 0).count(), 1);
        assert_eq!(buf[0] & 1, 1);
    }

    #[test]
    fn a_circle_sets_the_disc() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count()];
        mask_circle(&g, &mut buf, 10, 10, 1, true);
        // Radius 1 -> the centre and its four neighbours.
        assert_eq!(buf.iter().filter(|&&v| v & 1 != 0).count(), 5);
        for (i, j) in [(10, 10), (9, 10), (11, 10), (10, 9), (10, 11)] {
            assert_eq!(buf[j * 256 + i] & 1, 1, "({i},{j})");
        }
        // Radius 0 -> a single pixel; a negative radius -> nothing.
        let mut buf = vec![0i32; g.pixel_count()];
        mask_circle(&g, &mut buf, 3, 4, 0, true);
        assert_eq!(buf.iter().filter(|&&v| v & 1 != 0).count(), 1);
        mask_circle(&g, &mut buf, 3, 4, -2, true);
        assert_eq!(buf.iter().filter(|&&v| v & 1 != 0).count(), 1);
    }

    #[test]
    fn a_circle_at_the_edge_does_not_wrap_or_overflow() {
        let g = single();
        let mut buf = vec![0i32; g.pixel_count()];
        mask_circle(&g, &mut buf, 0, 0, 2, true);
        // A quarter disc: (0,0),(1,0),(2,0),(0,1),(1,1),(0,2).
        assert_eq!(buf.iter().filter(|&&v| v & 1 != 0).count(), 6);
        assert_eq!(buf[0] & 1, 1);
        assert_eq!(buf[2] & 1, 1);
        assert_eq!(buf[2 * 256] & 1, 1);
    }

    #[test]
    fn the_mask_round_trips_through_a_bpc_buffer() {
        let g = quad();
        let mut mask = vec![0i32; g.pixel_count()];
        mask_rectangle(&g, &mut mask, 300, 2, 100, 2, true);
        let mut bpc = vec![0u8; 4 * PIXEL_CONFIG_BYTES];
        let map = table(&g, 4, 0);
        assert_eq!(apply_mask_to_bpc(&map, &mask, &mut bpc), 0);
        assert_eq!(bpc.iter().filter(|&&b| b & 1 != 0).count(), 4);

        let mut back = vec![0i32; g.pixel_count()];
        mask_from_bpc(&map, &bpc, &mut back);
        for j in 100..102 {
            for i in 300..302 {
                assert_eq!(back[j * 512 + i], 1 << 1, "({i},{j})");
            }
        }
        assert_eq!(back.iter().filter(|&&v| v != 0).count(), 4);
    }

    #[test]
    fn a_bpc_buffer_too_small_for_the_mask_drops_pixels_instead_of_overflowing() {
        let g = quad();
        let mut mask = vec![0i32; g.pixel_count()];
        mask_reset(&g, &mut mask, true);
        // Only one chip's worth of file for a four-chip mask.
        let mut bpc = vec![0u8; PIXEL_CONFIG_BYTES];
        let dropped = apply_mask_to_bpc(&table(&g, 4, 0), &mask, &mut bpc);
        assert_eq!(dropped, 3 * PIXEL_CONFIG_BYTES);
        assert!(bpc.iter().all(|&b| b & 1 != 0));
    }

    #[test]
    fn the_pixel_config_diff_is_the_absolute_difference_at_the_mapped_byte() {
        let g = single();
        let mut serval = vec![0u8; PIXEL_CONFIG_BYTES];
        let mut bpc = vec![0u8; PIXEL_CONFIG_BYTES];
        let map = table(&g, 1, 0);
        let k = usize::try_from(pel_index(&g, 1, 0, 3, 4).0).unwrap();
        serval[k] = 200;
        bpc[k] = 5;
        let mut out = vec![0i32; g.pixel_count()];
        pixel_config_diff(&map, &serval, &bpc, &mut out);
        assert_eq!(out[4 * 256 + 3], 195);
        assert_eq!(out.iter().filter(|&&v| v != 0).count(), 1);
        // No underflow when the file byte is the larger one.
        serval[k] = 1;
        pixel_config_diff(&map, &serval, &bpc, &mut out);
        assert_eq!(out[4 * 256 + 3], 4);
    }

    #[test]
    fn masked_pixels_land_where_the_mask_editor_put_them() {
        for o in 0..8 {
            let g = quad();
            let map = table(&g, 4, o);
            let mut mask = vec![0i32; g.pixel_count()];
            mask_rectangle(&g, &mut mask, 10, 1, 400, 1, true);
            let mut bpc = vec![0u8; 4 * PIXEL_CONFIG_BYTES];
            apply_mask_to_bpc(&map, &mask, &mut bpc);
            assert_eq!(
                masked_pixels(&g, &map, &bpc),
                vec![(10, 400)],
                "orientation {o}"
            );
        }
    }
}
