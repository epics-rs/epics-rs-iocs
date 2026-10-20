//! 12-bit packed pixel formats to `u16`.
//!
//! USB3 Vision (`Mono12p`): byte 0 holds the 8 lsb of pixel 0, the low nibble
//! of byte 1 its 4 msb, the high nibble of byte 1 the 4 lsb of pixel 1, and
//! byte 2 the 8 msb of pixel 1.
//!
//! GigE Vision (`Mono12Packed`): byte 0 holds the 8 msb of pixel 0, the low
//! nibble of byte 1 its 4 lsb, the high nibble of byte 1 the 4 lsb of pixel 1,
//! and byte 2 the 8 msb of pixel 1.
//!
//! With `left_shift` the 12 bits are moved to the top of the word, leaving
//! bits 0-3 zero.

pub fn decompress_mono12p(left_shift: bool, input: &[u8], output: &mut [u16]) {
    let (input, _) = input.as_chunks::<3>();
    let (output, _) = output.as_chunks_mut::<2>();
    for (src, dst) in input.iter().zip(output) {
        let (b0, b1, b2) = (src[0] as u16, src[1] as u16, src[2] as u16);
        let p0 = b0 | ((b1 & 0x0f) << 8);
        let p1 = ((b1 & 0xf0) >> 4) | (b2 << 4);
        let shift = if left_shift { 4 } else { 0 };
        dst[0] = p0 << shift;
        dst[1] = p1 << shift;
    }
}

pub fn decompress_mono12_packed(left_shift: bool, input: &[u8], output: &mut [u16]) {
    let (input, _) = input.as_chunks::<3>();
    let (output, _) = output.as_chunks_mut::<2>();
    for (src, dst) in input.iter().zip(output) {
        let (b0, b1, b2) = (src[0] as u16, src[1] as u16, src[2] as u16);
        let p0 = (b0 << 4) | (b1 & 0x0f);
        let p1 = ((b1 & 0xf0) >> 4) | (b2 << 4);
        let shift = if left_shift { 4 } else { 0 };
        dst[0] = p0 << shift;
        dst[1] = p1 << shift;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mono12p_unpacks_both_pixels() {
        // pixel0 = 0xABC, pixel1 = 0x123
        let input = [0xBC, 0x3A, 0x12];
        let mut out = [0u16; 2];
        decompress_mono12p(false, &input, &mut out);
        assert_eq!(out, [0xABC, 0x123]);
        decompress_mono12p(true, &input, &mut out);
        assert_eq!(out, [0xABC0, 0x1230]);
    }

    #[test]
    fn mono12_packed_unpacks_both_pixels() {
        // pixel0 = 0xABC, pixel1 = 0x123
        let input = [0xAB, 0x3C, 0x12];
        let mut out = [0u16; 2];
        decompress_mono12_packed(false, &input, &mut out);
        assert_eq!(out, [0xABC, 0x123]);
        decompress_mono12_packed(true, &input, &mut out);
        assert_eq!(out, [0xABC0, 0x1230]);
    }
}
