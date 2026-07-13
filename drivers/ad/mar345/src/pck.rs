//! Decoder for the CCP4 "pck" packed image format `mar345dtb` writes, ported
//! from `mar345App/src/mar3xx_pck.c` (`get_pck` / `unpack_word`).
//!
//! The file is a text header line `\nCCP4 packed image, X: %04d, Y: %04d\n`
//! embedded somewhere near the start, followed by a bit-packed stream. The
//! stream is a sequence of chunks; each chunk starts with a 6-bit descriptor
//! (`log2(pixel count)` in the low 3 bits, a `bitdecode[]` index in the next 3),
//! then that many fixed-width, sign-extended deltas. Each delta reconstructs a
//! pixel from a predictor: the first pixel is stored raw, pixels in the first
//! row use the left neighbour, and the rest use a 4-neighbour average — the
//! inverse of the `diff_words` encoder.
//!
//! Only decoding (`get_pck`) is ported; the driver never encodes. The bit
//! arithmetic (`shift_left` / `shift_right` / `setbits`) is reproduced exactly,
//! including the masking that makes the signed shifts behave as logical shifts.

/// C stdio `BUFSIZ` (glibc). The header scan reads at most this many bytes per
/// line, exactly as `get_pck`'s `char header[BUFSIZ]` loop does.
const BUFSIZ: usize = 8192;

/// C `getc` end-of-file sentinel.
const EOF: i32 = -1;

/// Byte cursor over the in-memory file contents, standing in for `FILE *` +
/// `getc`.
struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl Reader<'_> {
    /// C `getc(fp)`: the next byte as an `int`, or `EOF` at end of stream.
    fn getc(&mut self) -> i32 {
        if self.pos < self.data.len() {
            let b = self.data[self.pos] as i32;
            self.pos += 1;
            b
        } else {
            EOF
        }
    }
}

/// C `setbits[n]` — the low `n` bits set (`setbits[32] == 0xFFFFFFFF`).
fn setbits(n: i32) -> u32 {
    if n <= 0 {
        0
    } else if n >= 32 {
        0xFFFF_FFFF
    } else {
        (1u32 << n) - 1
    }
}

/// C `shift_left(x, n) = ((x) & setbits[32 - n]) << n` — mask to the low
/// `32 - n` bits, then shift up (so nothing overflows past bit 31).
fn shift_left(x: i32, n: i32) -> i32 {
    if n <= 0 {
        x
    } else if n >= 32 {
        0
    } else {
        (((x as u32) & setbits(32 - n)) << n) as i32
    }
}

/// C `shift_right(x, n) = ((x) >> n) & setbits[32 - n]` — a logical right shift
/// (the mask clears the top `n` bits that an arithmetic shift would fill).
fn shift_right(x: i32, n: i32) -> i32 {
    if n <= 0 {
        x
    } else if n >= 32 {
        0
    } else {
        (((x as u32) >> n) & setbits(32 - n)) as i32
    }
}

/// Decode a pck file's contents into a row-major `nx * ny` buffer of pixels.
///
/// The dimensions come from the caller (C uses `NDArraySizeX`/`NDArraySizeY`,
/// set by the last mode change); the packed stream's own header dimensions drive
/// the predictor offsets and total pixel count, matching `unpack_word`. The two
/// are equal in normal operation. Writes are clamped to the buffer length, so a
/// header larger than `nx * ny` truncates rather than reading/writing out of
/// bounds (C would overflow the allocation).
pub fn get_pck(data: &[u8], nx: usize, ny: usize) -> Vec<i16> {
    let mut img = vec![0i16; nx.saturating_mul(ny)];
    let mut r = Reader { data, pos: 0 };
    let (x, y) = scan_header(&mut r);
    if x > 0 && y > 0 {
        unpack_word(&mut r, x, y, &mut img);
    }
    img
}

/// C `get_pck` header scan: read the stream line by line until one matches
/// `"\nCCP4 packed image, X: %04d, Y: %04d\n"`, and return its `(X, Y)`.
fn scan_header(r: &mut Reader) -> (i32, i32) {
    let mut header = [0u8; BUFSIZ];
    header[0] = b'\n';
    let mut c: i32 = 0;
    let mut x: i32 = 0;
    let mut y: i32 = 0;

    // C: `while ((c != EOF) && ((x == 0) || (y == 0)))`.
    while c != EOF && (x == 0 || y == 0) {
        c = 0;
        let mut i: usize = 0;
        x = 0;
        y = 0;
        // C: `while ((++i < BUFSIZ) && (c != EOF) && (c != '\n') && (x==0) && (y==0))`.
        loop {
            i += 1;
            if !(i < BUFSIZ && c != EOF && c != b'\n' as i32 && x == 0 && y == 0) {
                break;
            }
            c = r.getc();
            header[i] = c as u8;
            if c == b'\n' as i32
                && let Some((px, py)) = scanf_pack(&header[..=i])
            {
                x = px;
                y = py;
            }
        }
    }
    (x, y)
}

/// Emulate `sscanf(header, "\nCCP4 packed image, X: %04d, Y: %04d\n", &x, &y)`.
///
/// Whitespace in the format matches zero or more input whitespace bytes; `%04d`
/// skips leading whitespace then reads an optional sign and up to four digits
/// (the width cap is faithful to `scanf`, harmless here because the values are
/// always four digits). Returns `None` unless both integers convert — matching a
/// `sscanf` return value of 2.
fn scanf_pack(input: &[u8]) -> Option<(i32, i32)> {
    const FMT: &[u8] = b"\nCCP4 packed image, X: %04d, Y: %04d\n";
    let mut ints = [0i32; 2];
    let mut nint = 0usize;
    let mut fi = 0usize;
    let mut ii = 0usize;

    while fi < FMT.len() {
        let fc = FMT[fi];
        if fc.is_ascii_whitespace() {
            fi += 1;
            while ii < input.len() && input[ii].is_ascii_whitespace() {
                ii += 1;
            }
        } else if fc == b'%' {
            fi += 1;
            let mut width = 0usize;
            let mut has_width = false;
            while fi < FMT.len() && FMT[fi].is_ascii_digit() {
                width = width * 10 + (FMT[fi] - b'0') as usize;
                has_width = true;
                fi += 1;
            }
            // Only `%d` occurs in this format.
            if FMT.get(fi).copied()? != b'd' {
                return None;
            }
            fi += 1;
            while ii < input.len() && input[ii].is_ascii_whitespace() {
                ii += 1;
            }
            let maxw = if has_width && width > 0 {
                width
            } else {
                usize::MAX
            };
            let start = ii;
            let mut consumed = 0usize;
            if ii < input.len() && (input[ii] == b'+' || input[ii] == b'-') {
                ii += 1;
                consumed += 1;
            }
            let digit_start = ii;
            while ii < input.len() && consumed < maxw && input[ii].is_ascii_digit() {
                ii += 1;
                consumed += 1;
            }
            if ii == digit_start {
                return None;
            }
            let v: i32 = std::str::from_utf8(&input[start..ii]).ok()?.parse().ok()?;
            if nint >= ints.len() {
                return None;
            }
            ints[nint] = v;
            nint += 1;
        } else {
            if ii >= input.len() || input[ii] != fc {
                return None;
            }
            ii += 1;
            fi += 1;
        }
    }

    if nint == 2 {
        Some((ints[0], ints[1]))
    } else {
        None
    }
}

/// C `unpack_word` — decode the bit-packed stream into `img`, applying the delta
/// predictor. Pixel writes are clamped to `img.len()`.
fn unpack_word(r: &mut Reader, x: i32, y: i32, img: &mut [i16]) {
    const BITDECODE: [i32; 8] = [0, 4, 5, 6, 7, 8, 16, 32];

    let xw = x as i64;
    let total = ((x as i64) * (y as i64)).min(img.len() as i64).max(0);

    let mut valids: i32 = 0;
    let mut spillbits: i32 = 0;
    let mut window: i32 = 0;
    let mut spill: i32 = 0;
    let mut pixel: i64 = 0;

    while pixel < total {
        if valids < 6 {
            if spillbits > 0 {
                window |= shift_left(spill, valids);
                valids += spillbits;
                spillbits = 0;
            } else {
                spill = r.getc();
                spillbits = 8;
            }
        } else {
            let mut pixnum = 1i32 << (window & setbits(3) as i32);
            window = shift_right(window, 3);
            let bitnum = BITDECODE[(window & setbits(3) as i32) as usize];
            window = shift_right(window, 3);
            valids -= 6;

            while pixnum > 0 && pixel < total {
                if valids < bitnum {
                    if spillbits > 0 {
                        window |= shift_left(spill, valids);
                        if (32 - valids) > spillbits {
                            valids += spillbits;
                            spillbits = 0;
                        } else {
                            let usedbits = 32 - valids;
                            spill = shift_right(spill, usedbits);
                            spillbits -= usedbits;
                            valids = 32;
                        }
                    } else {
                        spill = r.getc();
                        spillbits = 8;
                    }
                } else {
                    pixnum -= 1;
                    let nextint = if bitnum == 0 {
                        0
                    } else {
                        let mut ni = window & setbits(bitnum) as i32;
                        valids -= bitnum;
                        window = shift_right(window, bitnum);
                        if (ni & (1i32 << (bitnum - 1))) != 0 {
                            ni |= !(setbits(bitnum) as i32);
                        }
                        ni
                    };

                    let p = pixel as usize;
                    if pixel > xw {
                        let pred = (i32::from(img[p - 1])
                            + i32::from(img[(pixel - xw + 1) as usize])
                            + i32::from(img[(pixel - xw) as usize])
                            + i32::from(img[(pixel - xw - 1) as usize])
                            + 2)
                            / 4;
                        img[p] = nextint.wrapping_add(pred) as i16;
                        pixel += 1;
                    } else if pixel != 0 {
                        img[p] = i32::from(img[p - 1]).wrapping_add(nextint) as i16;
                        pixel += 1;
                    } else {
                        img[p] = nextint as i16;
                        pixel += 1;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build the full file image from an ASCII header and packed body, then
    /// decode and compare against the original — the C encoder wrote both.
    fn check(bytes: &[u8], nx: usize, ny: usize, expected: &[i16]) {
        let out = get_pck(bytes, nx, ny);
        assert_eq!(out, expected);
    }

    // fixture A: 8x8 varied, includes negatives and large jumps (5000, -5000,
    // ±12345) so the descriptor bit widths span 4..16-bit chunks and both the
    // left-neighbour and 4-neighbour predictors run. Bytes and image emitted by
    // the C `put_pck` encoder.
    const FA_BYTES: &[u8] = &[
        67, 67, 80, 52, 32, 112, 97, 99, 107, 101, 100, 32, 105, 109, 97, 103, 101, 44, 32, 88, 58,
        32, 48, 48, 48, 56, 44, 32, 89, 58, 32, 48, 48, 48, 56, 10, 33, 244, 160, 220, 221, 37,
        119, 228, 97, 181, 90, 173, 86, 75, 177, 120, 60, 30, 143, 247, 147, 149, 184, 90, 173, 88,
        225, 82, 92, 43, 78, 40, 158, 251, 18, 140, 255, 143, 255, 239, 255, 79, 0, 8, 237, 119, 0,
        116, 0, 28, 20, 176, 0, 44, 255, 139, 255, 139, 255, 11, 63, 211, 240, 64, 207, 143, 255,
        239, 255, 79, 0, 172, 0, 204, 48, 132, 0, 132, 0, 100, 208, 47, 255, 139, 255, 139, 255,
        143, 255, 143, 255, 143, 255, 99, 35, 178, 0,
    ];
    const FA_IMG: &[i16] = &[
        -48, -41, -34, -27, -20, -13, -6, 1, 8, 15, 22, 29, 36, 43, -47, -40, -33, -26, -19, -12,
        -5, 2, 9, 16, 23, 30, 37, 44, -46, -39, 5000, -5000, -18, -11, -4, 3, 10, 17, 24, 31, 38,
        45, -45, -38, -31, -12345, 12345, -10, -3, 4, 11, 18, 25, 32, 39, 46, -44, -37, -30, -23,
        -16, -9, -2, 5,
    ];

    // fixture B: 4x4 constant 1000 — every delta is 0, exercising the bitnum==0
    // chunk path.
    const FB_BYTES: &[u8] = &[
        67, 67, 80, 52, 32, 112, 97, 99, 107, 101, 100, 32, 105, 109, 97, 103, 101, 44, 32, 88, 58,
        32, 48, 48, 48, 52, 44, 32, 89, 58, 32, 48, 48, 48, 52, 10, 48, 250, 192, 32, 0, 0, 0,
    ];
    const FB_IMG: &[i16] = &[
        1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
        1000,
    ];

    // fixture C: 16x1 single row — pixel is never > x, so only the raw and
    // left-neighbour predictors run.
    const FC_BYTES: &[u8] = &[
        67, 67, 80, 52, 32, 112, 97, 99, 107, 101, 100, 32, 105, 109, 97, 103, 101, 44, 32, 88, 58,
        32, 48, 48, 49, 54, 44, 32, 89, 58, 32, 48, 48, 48, 49, 10, 155, 24, 12, 197, 145, 44, 141,
        246, 68, 83, 149, 93, 25, 182, 97, 29, 220, 240, 15,
    ];
    const FC_IMG: &[i16] = &[
        -30, -29, -26, -21, -14, -5, 6, 19, 34, 51, 70, 91, 114, 139, 166, 195,
    ];

    #[test]
    fn decode_varied_8x8() {
        check(FA_BYTES, 8, 8, FA_IMG);
    }

    #[test]
    fn decode_constant_4x4() {
        check(FB_BYTES, 4, 4, FB_IMG);
    }

    #[test]
    fn decode_single_row_16x1() {
        check(FC_BYTES, 16, 1, FC_IMG);
    }

    #[test]
    fn header_scan_skips_leading_lines() {
        // A real .mar file has a binary/text header before the CCP4 line; the
        // scan must find the identifier line wherever it appears.
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MAR 345 image header\n");
        bytes.extend_from_slice(b"some junk: 12, 34\n");
        bytes.extend_from_slice(FB_BYTES);
        let out = get_pck(&bytes, 4, 4);
        assert_eq!(out, FB_IMG);
    }

    #[test]
    fn scanf_pack_matches_identifier() {
        assert_eq!(
            scanf_pack(b"\nCCP4 packed image, X: 3450, Y: 3450\n"),
            Some((3450, 3450))
        );
        assert_eq!(
            scanf_pack(b"\nCCP4 packed image, X: 0008, Y: 0008\n"),
            Some((8, 8))
        );
    }

    #[test]
    fn scanf_pack_rejects_non_identifier() {
        assert_eq!(scanf_pack(b"\nsomething else\n"), None);
        // Missing the Y field: sscanf would return 1, not 2.
        assert_eq!(scanf_pack(b"\nCCP4 packed image, X: 3450\n"), None);
    }

    #[test]
    fn no_header_yields_zero_buffer() {
        // No identifier line: get_pck leaves the allocation untouched. C returns
        // the (uninitialised) allocation; this port zero-initialises it.
        let out = get_pck(b"no header here at all\n", 2, 2);
        assert_eq!(out, vec![0i16; 4]);
    }

    #[test]
    fn oversized_header_is_clamped_not_out_of_bounds() {
        // The stream header is 8x8 but the caller buffer is only 2x2; decoding
        // must not read or write past the buffer.
        let out = get_pck(FA_BYTES, 2, 2);
        assert_eq!(out.len(), 4);
        // First four pixels decode as in the full image.
        assert_eq!(out, &FA_IMG[..4]);
    }

    #[test]
    fn setbits_matches_c_table() {
        assert_eq!(setbits(0), 0x0000_0000);
        assert_eq!(setbits(3), 0x0000_0007);
        assert_eq!(setbits(16), 0x0000_FFFF);
        assert_eq!(setbits(32), 0xFFFF_FFFF);
    }

    #[test]
    fn shifts_mask_like_c_macros() {
        // shift_right is logical even for a value with the top bit set.
        assert_eq!(shift_right(-1, 4) as u32, 0x0FFF_FFFF);
        assert_eq!(shift_right(0x80, 3), 0x10);
        // shift_left masks to the low 32-n bits before shifting.
        assert_eq!(shift_left(0xFF, 4), 0xFF0);
        assert_eq!(shift_right(123, 32), 0);
        assert_eq!(shift_left(123, 32), 0);
    }
}
