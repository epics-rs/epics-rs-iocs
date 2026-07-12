//! The Bruker SFRM frame file (C `BISDetector::readSFRM`).
//!
//! The header is a run of 80-byte lines, `KEY:` in the first 8 bytes and the
//! value in the other 72; the driver reads the lines it needs by index, exactly
//! as the C did. After `HDRBLKS` 512-byte blocks come the pixels, and after
//! them the underflow and overflow tables that hold the values that did not fit
//! the pixel width.
//!
//! No SFRM specification is available on this machine, so the line indices are
//! kept as the C had them and no new one is invented. One consequence is
//! documented on [`Header::word_order`].

use std::fmt;

/// One header line (C `lineLen`).
const LINE_LEN: usize = 80;
/// Where a line's value starts (C `dataOffset`).
const DATA_OFFSET: usize = 8;
/// The header is counted in blocks of this size (C `blockLen`).
const BLOCK_LEN: usize = 512;
/// The highest header line the driver reads, plus one.
const LINES_READ: usize = 80;

#[derive(Debug, PartialEq, Eq)]
pub struct SfrmError(String);

impl fmt::Display for SfrmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

fn err<T>(message: String) -> Result<T, SfrmError> {
    Err(SfrmError(message))
}

/// The header fields the driver reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub format: i32,
    pub version: i32,
    pub header_blocks: usize,
    /// `-1` means the frame was written without an underflow table.
    pub num_underflows: i32,
    pub num_overflows1: i32,
    pub num_overflows2: i32,
    pub bytes_per_pixel: usize,
    pub underflow_bytes_per_pixel: usize,
    pub rows: usize,
    pub cols: usize,
    /// Byte order within a 16-bit word; only 0 (little-endian) can be read.
    ///
    /// The C read *long* order from this same line as well — line 42 twice —
    /// and then rejected the file unless both were 0. Which header line really
    /// carries `LONGORD` cannot be derived from the driver, from the frames on
    /// this machine, or from any specification available here, so the duplicate
    /// read is dropped rather than guessed at: long order is not validated.
    pub word_order: i32,
    pub num_exposures: i32,
    pub bias: i32,
    pub baseline_offset: i32,
    pub orientation: i32,
    pub overscan: i32,
}

/// A decoded frame. `data` is row-major, `cols` values per row.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Image {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<u32>,
}

/// The numbers on header line `line` (C's `sscanf(buffer + line*80 + 8, ...)`).
///
/// C let `sscanf` run past the end of the line into the next key, and left the
/// caller's variables uninitialised when it matched nothing. Here a line that
/// does not hold `count` numbers is an error.
fn numbers(header: &[u8], line: usize, count: usize) -> Result<Vec<i64>, SfrmError> {
    let from = line * LINE_LEN + DATA_OFFSET;
    let to = (line + 1) * LINE_LEN;
    if header.len() < to {
        return err(format!("the header stops before line {line}"));
    }
    let text = String::from_utf8_lossy(&header[from..to]);
    let values: Vec<i64> = text
        .split_whitespace()
        .map_while(|t| t.parse::<i64>().ok())
        .collect();
    if values.len() < count {
        return err(format!(
            "header line {line} holds {} numbers, {count} were expected: '{}'",
            values.len(),
            text.trim()
        ));
    }
    Ok(values)
}

pub fn parse_header(bytes: &[u8]) -> Result<Header, SfrmError> {
    let format = numbers(bytes, 0, 1)?[0] as i32;
    let version = numbers(bytes, 1, 1)?[0] as i32;
    let header_blocks = numbers(bytes, 2, 1)?[0];

    if format != 100 || version < 11 {
        return err(format!("unsupported format {format} or version {version}"));
    }
    if header_blocks <= 0 {
        return err(format!("a header of {header_blocks} blocks cannot be read"));
    }
    let header_blocks = header_blocks as usize;

    // The driver reads up to line 79, so the header must be at least that long
    // whatever HDRBLKS says.
    let header_bytes = header_blocks * BLOCK_LEN;
    if bytes.len() < header_bytes || header_bytes < LINES_READ * LINE_LEN {
        return err(format!(
            "a header of {header_blocks} blocks in a file of {} bytes is too short",
            bytes.len()
        ));
    }

    let counts = numbers(bytes, 20, 3)?;
    let widths = numbers(bytes, 39, 2)?;
    let rows = numbers(bytes, 40, 1)?[0];
    let cols = numbers(bytes, 41, 1)?[0];
    let word_order = numbers(bytes, 42, 1)?[0] as i32;
    let frame = numbers(bytes, 79, 5)?;

    if word_order != 0 {
        return err(format!("unsupported word order {word_order}"));
    }

    // C's switch on bytesPerPixel had no default: any other width left the
    // pixel buffer unwritten and the byte count uninitialised, and the frame
    // was published anyway.
    let bytes_per_pixel = widths[0];
    if !matches!(bytes_per_pixel, 1 | 2 | 4) {
        return err(format!("unsupported pixel width {bytes_per_pixel}"));
    }
    let underflow_bytes_per_pixel = widths[1];
    if !matches!(underflow_bytes_per_pixel, 1 | 2 | 4) {
        return err(format!(
            "unsupported underflow width {underflow_bytes_per_pixel}"
        ));
    }
    if rows <= 0 || cols <= 0 {
        return err(format!("a frame of {rows} x {cols} pixels cannot be read"));
    }

    Ok(Header {
        format,
        version,
        header_blocks,
        num_underflows: counts[0] as i32,
        num_overflows1: counts[1] as i32,
        num_overflows2: counts[2] as i32,
        bytes_per_pixel: bytes_per_pixel as usize,
        underflow_bytes_per_pixel: underflow_bytes_per_pixel as usize,
        rows: rows as usize,
        cols: cols as usize,
        word_order,
        num_exposures: frame[0] as i32,
        bias: frame[1] as i32,
        baseline_offset: frame[2] as i32,
        orientation: frame[3] as i32,
        overscan: frame[4] as i32,
    })
}

/// Read `count` little-endian values of `width` bytes from `at`, and step over
/// the padding that takes the table up to a 16-byte boundary.
fn table(
    bytes: &[u8],
    at: &mut usize,
    count: i32,
    width: usize,
    what: &str,
) -> Result<Vec<u32>, SfrmError> {
    if count <= 0 {
        return Ok(Vec::new());
    }
    let count = count as usize;
    let padded = (count * width).div_ceil(16) * 16;
    if bytes.len() < *at + padded {
        return err(format!(
            "the file stops {} bytes into a {what} table of {padded} bytes",
            bytes.len().saturating_sub(*at)
        ));
    }
    let mut values = Vec::with_capacity(count);
    for i in 0..count {
        let from = *at + i * width;
        values.push(read_le(&bytes[from..from + width]));
    }
    *at += padded;
    Ok(values)
}

fn read_le(bytes: &[u8]) -> u32 {
    let mut value = 0u32;
    for (i, b) in bytes.iter().enumerate() {
        value |= (*b as u32) << (8 * i);
    }
    value
}

/// Decode a whole SFRM file.
pub fn decode(bytes: &[u8]) -> Result<Image, SfrmError> {
    let header = parse_header(bytes)?;

    let n_pixels = header.rows * header.cols;
    let mut at = header.header_blocks * BLOCK_LEN;
    let data_bytes = n_pixels * header.bytes_per_pixel;
    if bytes.len() < at + data_bytes {
        return err(format!(
            "the file holds {} of the {data_bytes} pixel bytes the header promises",
            bytes.len().saturating_sub(at)
        ));
    }

    let mut data: Vec<u32> = (0..n_pixels)
        .map(|i| {
            let from = at + i * header.bytes_per_pixel;
            read_le(&bytes[from..from + header.bytes_per_pixel])
        })
        .collect();
    at += data_bytes;

    // The tables follow the pixels in this order, each padded to 16 bytes.
    let underflows = table(
        bytes,
        &mut at,
        header.num_underflows,
        header.underflow_bytes_per_pixel,
        "underflow",
    )?;
    let overflows1 = table(bytes, &mut at, header.num_overflows1, 2, "1-byte overflow")?;
    let overflows2 = table(bytes, &mut at, header.num_overflows2, 4, "2-byte overflow")?;

    // A frame written without an underflow table carries no baseline either.
    let baseline_offset = if header.num_underflows == -1 {
        0
    } else {
        header.baseline_offset
    };

    // Substitute the saturated pixels from the tables (C's correction loop).
    // C indexed the tables without ever checking that they had another entry —
    // and read through a null pointer when the count was zero but a pixel was
    // saturated anyway — so a frame whose tables were shorter than its
    // saturated pixels ran off the end of the heap. A table that runs out is an
    // error here.
    let (mut n1, mut n2, mut nu) = (0usize, 0usize, 0usize);
    for value in data.iter_mut() {
        if header.bytes_per_pixel == 1 && *value == 255 {
            let Some(v) = overflows1.get(n1) else {
                return err(format!(
                    "the 1-byte overflow table holds {} entries, the frame needs more",
                    overflows1.len()
                ));
            };
            *value = *v;
            n1 += 1;
        }
        if header.bytes_per_pixel != 4 && *value == 65535 {
            let Some(v) = overflows2.get(n2) else {
                return err(format!(
                    "the 2-byte overflow table holds {} entries, the frame needs more",
                    overflows2.len()
                ));
            };
            *value = *v;
            n2 += 1;
        }
        if *value == 0 {
            if !underflows.is_empty() {
                let Some(v) = underflows.get(nu) else {
                    return err(format!(
                        "the underflow table holds {} entries, the frame needs more",
                        underflows.len()
                    ));
                };
                *value = *v;
                nu += 1;
            }
        } else if baseline_offset != 0 {
            *value = value.wrapping_add(baseline_offset as u32);
        }
    }

    Ok(Image {
        rows: header.rows,
        cols: header.cols,
        data,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The header lines the driver reads, and nothing else.
    struct HeaderBuilder {
        lines: Vec<(usize, String)>,
        blocks: usize,
    }

    impl HeaderBuilder {
        /// A one-block-per-16-lines header that passes every check.
        fn new(bytes_per_pixel: usize, rows: usize, cols: usize) -> Self {
            Self {
                // 80 lines of 80 bytes = 6400 bytes = 12.5 blocks, so 13.
                blocks: 13,
                lines: vec![
                    (0, "FORMAT :100".into()),
                    (1, "VERSION:11".into()),
                    (2, "HDRBLKS:13".into()),
                    (20, "NOVERFL:0 0 0".into()),
                    (39, format!("NPIXELB:{bytes_per_pixel} 2")),
                    (40, format!("NROWS  :{rows}")),
                    (41, format!("NCOLS  :{cols}")),
                    (42, "WORDORD:0".into()),
                    (79, "LINEAR :1 0 0 0 0".into()),
                ],
            }
        }

        fn set(mut self, line: usize, text: &str) -> Self {
            self.lines.retain(|(n, _)| *n != line);
            self.lines.push((line, text.into()));
            self
        }

        fn build(&self) -> Vec<u8> {
            let mut header = vec![b' '; self.blocks * BLOCK_LEN];
            for (line, text) in &self.lines {
                let at = line * LINE_LEN;
                header[at..at + text.len()].copy_from_slice(text.as_bytes());
            }
            header
        }
    }

    fn pad16(table: &mut Vec<u8>) {
        while !table.len().is_multiple_of(16) {
            table.push(0);
        }
    }

    #[test]
    fn a_two_byte_frame_decodes_to_its_pixels() {
        let mut file = HeaderBuilder::new(2, 2, 3).build();
        for pixel in [1u16, 2, 3, 4, 5, 6] {
            file.extend_from_slice(&pixel.to_le_bytes());
        }
        let image = decode(&file).expect("image");
        assert_eq!(image.rows, 2);
        assert_eq!(image.cols, 3);
        assert_eq!(image.data, vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn the_baseline_offset_is_added_to_every_non_zero_pixel() {
        let mut file = HeaderBuilder::new(2, 1, 3)
            .set(79, "LINEAR :1 0 32 0 0")
            .build();
        for pixel in [0u16, 5, 7] {
            file.extend_from_slice(&pixel.to_le_bytes());
        }
        let image = decode(&file).expect("image");
        assert_eq!(image.data, vec![0, 37, 39]);
    }

    #[test]
    fn a_frame_with_no_underflow_table_has_no_baseline_offset() {
        // NOVERFL = -1 in C means "no tables"; the baseline is then meaningless.
        let mut file = HeaderBuilder::new(2, 1, 2)
            .set(20, "NOVERFL:-1 0 0")
            .set(79, "LINEAR :1 0 32 0 0")
            .build();
        for pixel in [4u16, 9] {
            file.extend_from_slice(&pixel.to_le_bytes());
        }
        let image = decode(&file).expect("image");
        assert_eq!(image.data, vec![4, 9]);
    }

    #[test]
    fn saturated_and_underflowed_pixels_come_from_the_tables() {
        // One byte per pixel: 255 is the 1-byte overflow marker, and a 1-byte
        // overflow value of 65535 escalates to the 2-byte table.
        let mut file = HeaderBuilder::new(1, 1, 4).set(20, "NOVERFL:1 2 1").build();
        file.extend_from_slice(&[0u8, 255, 255, 7]);

        let mut underflows = vec![];
        underflows.extend_from_slice(&12u16.to_le_bytes());
        pad16(&mut underflows);
        file.extend_from_slice(&underflows);

        let mut overflows1 = vec![];
        overflows1.extend_from_slice(&1000u16.to_le_bytes());
        overflows1.extend_from_slice(&65535u16.to_le_bytes());
        pad16(&mut overflows1);
        file.extend_from_slice(&overflows1);

        let mut overflows2 = vec![];
        overflows2.extend_from_slice(&70000u32.to_le_bytes());
        pad16(&mut overflows2);
        file.extend_from_slice(&overflows2);

        let image = decode(&file).expect("image");
        assert_eq!(image.data, vec![12, 1000, 70000, 7]);
    }

    #[test]
    fn a_table_shorter_than_the_frame_needs_is_an_error() {
        // C read past the end of the table here.
        let mut file = HeaderBuilder::new(1, 1, 2).set(20, "NOVERFL:0 1 0").build();
        file.extend_from_slice(&[255u8, 255]);
        let mut overflows1 = vec![];
        overflows1.extend_from_slice(&1000u16.to_le_bytes());
        pad16(&mut overflows1);
        file.extend_from_slice(&overflows1);

        let e = decode(&file).expect_err("the second 255 has no entry");
        assert!(e.to_string().contains("1-byte overflow table"), "{e}");
    }

    #[test]
    fn a_saturated_pixel_with_no_table_at_all_is_an_error() {
        // C dereferenced a null pointer here.
        let mut file = HeaderBuilder::new(1, 1, 1).build();
        file.push(255);
        assert!(decode(&file).is_err());
    }

    #[test]
    fn an_unsupported_pixel_width_is_an_error() {
        // C's switch fell through and published an unwritten buffer.
        let file = HeaderBuilder::new(2, 1, 1).set(39, "NPIXELB:3 2").build();
        let e = decode(&file).expect_err("width 3");
        assert!(e.to_string().contains("pixel width 3"), "{e}");
    }

    #[test]
    fn a_wrong_format_or_version_is_an_error() {
        let file = HeaderBuilder::new(2, 1, 1).set(0, "FORMAT :86").build();
        assert!(decode(&file).is_err());
        let file = HeaderBuilder::new(2, 1, 1).set(1, "VERSION:10").build();
        assert!(decode(&file).is_err());
    }

    #[test]
    fn a_big_endian_word_order_is_an_error() {
        let file = HeaderBuilder::new(2, 1, 1).set(42, "WORDORD:1").build();
        let e = decode(&file).expect_err("word order 1");
        assert!(e.to_string().contains("word order 1"), "{e}");
    }

    #[test]
    fn a_header_that_stops_short_is_an_error() {
        // C ran sscanf over uninitialised malloc'd memory here.
        let file = HeaderBuilder::new(2, 1, 1).build();
        assert!(decode(&file[..200]).is_err());
        assert!(decode(&[]).is_err());
    }

    #[test]
    fn a_truncated_pixel_block_is_an_error() {
        let mut file = HeaderBuilder::new(2, 2, 2).build();
        file.extend_from_slice(&[0u8; 4]); // 2 of the 4 pixels
        let e = decode(&file).expect_err("half a frame");
        assert!(e.to_string().contains("pixel bytes"), "{e}");
    }
}
