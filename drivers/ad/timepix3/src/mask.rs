//! The BPC (bad-pixel-config) mask: detector geometry, the image↔file index map,
//! and the mask drawing operations (port of `mask_io.cpp`).
//!
//! A BPC file holds one byte per detector pixel in *chip* order; the mask PV
//! holds one element per pixel in *image* order. `pel_index` is the map between
//! them, and it depends on the chip count and the detector orientation.

/// One chip's pixel-config bytes (C `kPixelConfigBytes`, serval_http.cpp:1011).
pub const PIXEL_CONFIG_BYTES: usize = 65536;

/// The detector layout, derived from what Serval reported (C `rowsCols`,
/// mask_io.cpp:210).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Geometry {
    pub rows: usize,
    pub cols: usize,
    pub x_chips: usize,
    pub y_chips: usize,
    pub pel_width: usize,
    pub num_chips: usize,
    pub orientation: i32,
}

impl Geometry {
    /// UPSTREAM DEFECT (mask_io.cpp:210-226): C divides by `rowLength` with no
    /// guard, so a detector that has not reported its layout yet (`RowLen` = 0,
    /// its initial value) raises SIGFPE and takes the IOC down. A layout that
    /// does not divide is not a layout.
    pub fn new(row_len: i32, num_chips: i32, num_rows: i32, orientation: i32) -> Option<Self> {
        let row_len = usize::try_from(row_len).ok()?;
        let num_chips = usize::try_from(num_chips).ok()?;
        let num_rows = usize::try_from(num_rows).ok()?;
        if row_len == 0 || num_chips == 0 || num_rows == 0 {
            return None;
        }
        let x_chips = row_len;
        let y_chips = num_chips / row_len;
        let pel_width = num_rows / row_len;
        if y_chips == 0 || pel_width == 0 {
            return None;
        }
        Some(Self {
            rows: num_rows,
            cols: y_chips * pel_width,
            x_chips,
            y_chips,
            pel_width,
            num_chips,
            orientation,
        })
    }

    /// The mask buffer's element count.
    pub fn pixel_count(&self) -> usize {
        self.rows * self.cols
    }

    /// The row-major offset of image pixel `(i, j)`.
    ///
    /// UPSTREAM DEFECT (mask_io.cpp:237, 250, 275): `maskReset`,
    /// `maskRectangle` and `maskCircle` index the buffer as `buf[j*ROWS + i]` —
    /// the row stride is the number of *rows*, not of columns. Every other site
    /// (mask read at :140, mask write at :159, `PixelConfigDiff` at
    /// serval_http.cpp:1125) uses `j*COLS + i`. The two agree only while the
    /// detector is square, which every currently shipped layout happens to be;
    /// on a non-square layout the drawing operations write to the wrong pixels
    /// and, when `COLS > ROWS`, past the end of the waveform. One stride,
    /// `cols`, everywhere.
    fn offset(&self, i: usize, j: usize) -> Option<usize> {
        if i >= self.cols || j >= self.rows {
            return None;
        }
        Some(j * self.cols + i)
    }

    /// Image pixel `(i, j)` → its byte offset in the BPC file (C `pelIndex`,
    /// mask_io.cpp:764).
    pub fn pel_index(&self, i: usize, j: usize) -> Option<usize> {
        if i >= self.cols || j >= self.rows {
            return None;
        }
        let w = self.pel_width as i64;
        let (i, j) = (i as i64, j as i64);
        let sq = w * w;
        let o = self.orientation;

        let index: i64 = match self.num_chips {
            1 => match o {
                0 => i + (w - 1 - j) * w,
                1 => j + i * w,
                2 => (w - 1 - i) + j * w,
                3 => (w - 1 - j) + (w - 1 - i) * w,
                4 => (w - 1 - i) + (w - 1 - j) * w,
                5 => j + (w - 1 - i) * w,
                6 => i + j * w,
                7 => (w - 1 - j) + i * w,
                _ => return None,
            },
            4 => {
                let xc = i / w;
                let yc = j / w;
                if xc > 1 || yc > 1 {
                    return None;
                }
                // The tile-local coordinates C calls ii/jj.
                let ii = i - w;
                let jj = j - w;
                match (o, xc, yc) {
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

                    _ => return None,
                }
            }
            8 => {
                // C maps the 8-chip mosaic for the UP orientation only
                // (mask_io.cpp:925); the other seven need per-layout tables the
                // C driver does not have, and they cannot be derived from any
                // source available here.
                if o != 0 || self.x_chips * self.y_chips != 8 {
                    return None;
                }
                let xc = i / w;
                let yc = j / w;
                let lx = i - xc * w;
                let ly = j - yc * w;
                let chip = yc * self.x_chips as i64 + xc;
                chip * sq + lx + (w - 1 - ly) * w
            }
            _ => return None,
        };

        usize::try_from(index).ok()
    }

    /// BPC byte offset → image pixel `(i, j)`, the exact inverse of
    /// `pel_index`.
    ///
    /// UPSTREAM DEFECT (serval_http.cpp:1005, mask_io.cpp:529): C hand-writes a
    /// *second* table, `bpc2ImgIndex`, for this direction — and C's own comment
    /// records that "bpc2ImgIndex() is not the inverse of pelIndex for all quad
    /// orientations (e.g. LEFT)". The two tables disagree, so the masked-pixel
    /// export (serval_http.cpp:931) places pixels somewhere other than where
    /// the mask editor put them. There is only one map here, and this direction
    /// is derived from it, so they cannot disagree.
    pub fn bpc_to_image(&self, bpc_index: usize) -> Option<(usize, usize)> {
        for j in 0..self.rows {
            for i in 0..self.cols {
                if self.pel_index(i, j) == Some(bpc_index) {
                    return Some((i, j));
                }
            }
        }
        None
    }
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
    for j in 0..geom.rows {
        for i in 0..geom.cols {
            if let Some(k) = geom.offset(i, j)
                && k < buf.len()
            {
                set_bit(&mut buf[k], on);
            }
        }
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
/// its mask bit set (C mask_io.cpp:131-146).
pub fn mask_from_bpc(geom: &Geometry, bpc: &[u8], out: &mut [i32]) {
    out.fill(0);
    for j in 0..geom.rows {
        for i in 0..geom.cols {
            let (Some(dst), Some(k)) = (geom.offset(i, j), geom.pel_index(i, j)) else {
                continue;
            };
            if dst < out.len() && bpc.get(k).is_some_and(|b| b & 1 != 0) {
                out[dst] |= 1 << 1;
            }
        }
    }
}

/// Apply the mask waveform to a BPC buffer, ready to be written back (C
/// mask_io.cpp:153-161).
///
/// UPSTREAM DEFECT (mask_io.cpp:159): C writes `bufBPC[pelIndex(i, j)] |= 1`
/// with the index unchecked — a mask whose geometry is larger than the BPC file
/// on disk (a stale or truncated file, or a layout Serval changed under the
/// IOC) writes past the end of the malloc'd buffer. Out-of-range indices are
/// skipped, and the count of skipped pixels is returned so the caller can say
/// so.
pub fn apply_mask_to_bpc(geom: &Geometry, mask: &[i32], bpc: &mut [u8]) -> usize {
    let mut dropped = 0;
    for j in 0..geom.rows {
        for i in 0..geom.cols {
            let Some(src) = geom.offset(i, j) else {
                continue;
            };
            if src >= mask.len() || mask[src] & MASK_BIT == 0 {
                continue;
            }
            match geom.pel_index(i, j) {
                Some(k) if k < bpc.len() => bpc[k] |= 1,
                _ => dropped += 1,
            }
        }
    }
    dropped
}

/// `|SERVAL − BPC|` per image pixel (C serval_http.cpp:1120-1142).
///
/// `serval` is the chip pixel-config bytes concatenated in chip order, `bpc`
/// the file on disk. A pixel whose chip did not decode contributes 0.
pub fn pixel_config_diff(geom: &Geometry, serval: &[u8], bpc: &[u8], out: &mut [i32]) {
    out.fill(0);
    for j in 0..geom.rows {
        for i in 0..geom.cols {
            let (Some(dst), Some(k)) = (geom.offset(i, j), geom.pel_index(i, j)) else {
                continue;
            };
            if dst >= out.len() {
                continue;
            }
            let (Some(&a), Some(&b)) = (serval.get(k), bpc.get(k)) else {
                continue;
            };
            out[dst] = i32::from(a.abs_diff(b));
        }
    }
}

/// The masked pixels of a BPC file, as image coordinates (C
/// `exportMaskedPelsJsonFromBpcBuffer`, serval_http.cpp:920-940).
pub fn masked_pixels(geom: &Geometry, bpc: &[u8]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    // Walk the image (the forward map), not the file, so this stays linear.
    for j in 0..geom.rows {
        for i in 0..geom.cols {
            if let Some(k) = geom.pel_index(i, j)
                && bpc.get(k).is_some_and(|b| b & 1 != 0)
            {
                out.push((i, j));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The single-chip TimePix3: 256x256, one chip.
    fn single(orientation: i32) -> Geometry {
        Geometry::new(1, 1, 256, orientation).unwrap()
    }

    /// The quad 2x2: 512x512, four 256x256 chips, two per row.
    fn quad(orientation: i32) -> Geometry {
        Geometry::new(2, 4, 512, orientation).unwrap()
    }

    #[test]
    fn the_geometry_matches_c_for_the_shipped_layouts() {
        let g = single(0);
        assert_eq!((g.rows, g.cols), (256, 256));
        assert_eq!((g.x_chips, g.y_chips, g.pel_width), (1, 1, 256));

        let g = quad(0);
        assert_eq!((g.rows, g.cols), (512, 512));
        assert_eq!((g.x_chips, g.y_chips, g.pel_width), (2, 2, 256));
        assert_eq!(g.pixel_count(), 512 * 512);
    }

    #[test]
    fn a_layout_that_would_divide_by_zero_is_rejected() {
        // C's rowsCols does `numChips / rowLength` with rowLength = 0 at init.
        assert_eq!(Geometry::new(0, 4, 512, 0), None);
        assert_eq!(Geometry::new(-1, 4, 512, 0), None);
        assert_eq!(Geometry::new(2, 0, 512, 0), None);
        assert_eq!(Geometry::new(2, 4, 0, 0), None);
        // numChips < rowLength → zero chip rows.
        assert_eq!(Geometry::new(4, 2, 512, 0), None);
    }

    #[test]
    fn pel_index_is_a_bijection_for_every_supported_layout() {
        for chips in [1, 4] {
            for o in 0..8 {
                let g = if chips == 1 { single(o) } else { quad(o) };
                let mut seen = vec![false; g.pixel_count()];
                for j in 0..g.rows {
                    for i in 0..g.cols {
                        let k = g
                            .pel_index(i, j)
                            .unwrap_or_else(|| panic!("chips {chips} o {o} ({i},{j})"));
                        assert!(k < seen.len(), "chips {chips} o {o} ({i},{j}) -> {k}");
                        assert!(!seen[k], "chips {chips} o {o}: {k} hit twice");
                        seen[k] = true;
                    }
                }
                assert!(seen.iter().all(|&b| b), "chips {chips} o {o}: not onto");
            }
        }
    }

    #[test]
    fn the_eight_chip_layout_maps_only_the_up_orientation() {
        // 2x4: rowLength 2, 8 chips, 512 rows → 256-wide chips, cols = 4*256.
        let g = Geometry::new(2, 8, 512, 0).unwrap();
        assert_eq!((g.rows, g.cols, g.x_chips, g.y_chips), (512, 1024, 2, 4));
        assert!(g.pel_index(0, 0).is_some());
        let g = Geometry::new(2, 8, 512, 1).unwrap();
        // C warns and returns -1 for every non-UP 8-chip orientation.
        assert_eq!(g.pel_index(0, 0), None);
    }

    #[test]
    fn bpc_to_image_inverts_pel_index_on_every_orientation() {
        for o in 0..8 {
            let g = quad(o);
            for (i, j) in [(0usize, 0usize), (1, 2), (255, 255), (256, 256), (511, 300)] {
                let k = g.pel_index(i, j).unwrap();
                assert_eq!(g.bpc_to_image(k), Some((i, j)), "orientation {o}");
            }
        }
    }

    #[test]
    fn a_single_chip_up_mask_maps_to_the_c_index() {
        // C: index = i + ((W-1) - j)*W.
        let g = single(0);
        assert_eq!(g.pel_index(0, 0), Some(255 * 256));
        assert_eq!(g.pel_index(5, 255), Some(5));
        assert_eq!(g.pel_index(256, 0), None);
        assert_eq!(g.pel_index(0, 256), None);
    }

    #[test]
    fn reset_touches_only_the_mask_bit() {
        let g = single(0);
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
        let g = single(0);
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
        let g = single(0);
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
        let g = single(0);
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
        let g = single(0);
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
        let g = quad(0);
        let mut mask = vec![0i32; g.pixel_count()];
        mask_rectangle(&g, &mut mask, 300, 2, 100, 2, true);
        let mut bpc = vec![0u8; 4 * PIXEL_CONFIG_BYTES];
        assert_eq!(apply_mask_to_bpc(&g, &mask, &mut bpc), 0);
        assert_eq!(bpc.iter().filter(|&&b| b & 1 != 0).count(), 4);

        let mut back = vec![0i32; g.pixel_count()];
        mask_from_bpc(&g, &bpc, &mut back);
        for j in 100..102 {
            for i in 300..302 {
                assert_eq!(back[j * g.cols + i], 1 << 1, "({i},{j})");
            }
        }
        assert_eq!(back.iter().filter(|&&v| v != 0).count(), 4);
    }

    #[test]
    fn a_bpc_buffer_too_small_for_the_mask_drops_pixels_instead_of_overflowing() {
        let g = quad(0);
        let mut mask = vec![0i32; g.pixel_count()];
        mask_reset(&g, &mut mask, true);
        // Only one chip's worth of file for a four-chip mask.
        let mut bpc = vec![0u8; PIXEL_CONFIG_BYTES];
        let dropped = apply_mask_to_bpc(&g, &mask, &mut bpc);
        assert_eq!(dropped, 3 * PIXEL_CONFIG_BYTES);
        assert!(bpc.iter().all(|&b| b & 1 != 0));
    }

    #[test]
    fn the_pixel_config_diff_is_the_absolute_difference_at_the_mapped_byte() {
        let g = single(0);
        let mut serval = vec![0u8; PIXEL_CONFIG_BYTES];
        let mut bpc = vec![0u8; PIXEL_CONFIG_BYTES];
        let k = g.pel_index(3, 4).unwrap();
        serval[k] = 200;
        bpc[k] = 5;
        let mut out = vec![0i32; g.pixel_count()];
        pixel_config_diff(&g, &serval, &bpc, &mut out);
        assert_eq!(out[4 * 256 + 3], 195);
        assert_eq!(out.iter().filter(|&&v| v != 0).count(), 1);
        // No underflow when the file byte is the larger one.
        serval[k] = 1;
        pixel_config_diff(&g, &serval, &bpc, &mut out);
        assert_eq!(out[4 * 256 + 3], 4);
    }

    #[test]
    fn masked_pixels_land_where_the_mask_editor_put_them() {
        // The C round trip is broken for LEFT: pelIndex writes the file, but
        // bpc2ImgIndex reads it back at a different pixel.
        for o in 0..8 {
            let g = quad(o);
            let mut mask = vec![0i32; g.pixel_count()];
            mask_rectangle(&g, &mut mask, 10, 1, 400, 1, true);
            let mut bpc = vec![0u8; 4 * PIXEL_CONFIG_BYTES];
            apply_mask_to_bpc(&g, &mask, &mut bpc);
            assert_eq!(masked_pixels(&g, &bpc), vec![(10, 400)], "orientation {o}");
        }
    }
}
