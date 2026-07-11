//! Wire protocol for the vac module's two device families.
//!
//! Everything here is pure: command formatting, reply validation/stripping,
//! and fixed-offset response decoding. No I/O, no records — so every format
//! and parse path is unit-testable against fixture strings.

pub mod scan;
pub mod vac_sen;

/// A fixed-size C character buffer with `strcpy`-at-offset semantics.
///
/// Both C device supports assemble each device reply into a fixed byte buffer
/// at a per-command offset (`strcpy(&responseBuffer[10*i], pstartdata)`), then
/// decode the whole buffer by absolute offset. The NUL that `strcpy` writes is
/// load-bearing: it terminates the previous field. Reproducing that byte layout
/// is what keeps the decode offsets meaningful.
#[derive(Clone)]
pub struct CBuf<const N: usize>(pub [u8; N]);

impl<const N: usize> Default for CBuf<N> {
    fn default() -> Self {
        Self([0u8; N])
    }
}

impl<const N: usize> CBuf<N> {
    /// C `strcpy(&buf[off], src)` — copy `src` then a NUL terminator, both
    /// clamped to the buffer. C would overrun; we stop at the end.
    pub fn strcpy_at(&mut self, off: usize, src: &[u8]) {
        if off >= N {
            return;
        }
        let n = src.len().min(N - off);
        self.0[off..off + n].copy_from_slice(&src[..n]);
        if off + n < N {
            self.0[off + n] = 0;
        }
    }

    /// The `len` bytes at `off`, clamped to the buffer (C `strncpy` source).
    pub fn slice(&self, off: usize, len: usize) -> &[u8] {
        if off >= N {
            return &[];
        }
        &self.0[off..(off + len).min(N)]
    }

    /// The byte at `off`, or NUL past the end.
    pub fn at(&self, off: usize) -> u8 {
        self.0.get(off).copied().unwrap_or(0)
    }
}

/// The NUL-terminated prefix of `s`, as C's string functions would see it.
pub fn cstr(s: &[u8]) -> &[u8] {
    match s.iter().position(|&b| b == 0) {
        Some(i) => &s[..i],
        None => s,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strcpy_at_writes_terminator() {
        let mut b: CBuf<16> = CBuf::default();
        b.strcpy_at(0, b"abc");
        assert_eq!(&b.0[..4], b"abc\0");
    }

    #[test]
    fn strcpy_at_clamps_at_buffer_end() {
        let mut b: CBuf<4> = CBuf::default();
        b.strcpy_at(2, b"xyz");
        assert_eq!(&b.0, b"\0\0xy");
    }

    #[test]
    fn strcpy_at_past_end_is_a_noop() {
        let mut b: CBuf<4> = CBuf::default();
        b.strcpy_at(9, b"x");
        assert_eq!(&b.0, b"\0\0\0\0");
    }

    #[test]
    fn cstr_stops_at_nul() {
        assert_eq!(cstr(b"ab\0cd"), b"ab");
        assert_eq!(cstr(b"abc"), b"abc");
    }
}
