//! C `sscanf` over a device reply, one conversion at a time, for the formats
//! `devTPG261.c` and `devTelevac.c` scan.

use std::cell::Cell;
use std::ffi::c_long;

use epics_libcom_rs::runtime::scanf::{ScanNum, sscanf};
use epics_libcom_rs::runtime::stdlib::scanf_long;

/// A failed conversion stops the scan and leaves the rest unassigned.
pub struct Scan<'a> {
    rest: &'a [u8],
    failed: bool,
}

impl<'a> Scan<'a> {
    pub fn new(text: &'a [u8]) -> Self {
        Self {
            rest: text,
            failed: false,
        }
    }

    /// One `sscanf` conversion, `fmt` ending in `%n`.
    fn conv<T: ScanNum + Default>(&mut self, fmt: &str) -> Option<T> {
        let (v, n) = (Cell::new(T::default()), Cell::new(0i32));
        self.take(
            (!self.failed && sscanf(self.rest, fmt, &[&v, &n]) == 1)
                .then(|| (v.get(), n.get() as usize)),
        )
    }

    fn take<T>(&mut self, read: Option<(T, usize)>) -> Option<T> {
        let Some((v, n)) = read else {
            self.failed = true;
            return None;
        };
        self.rest = &self.rest[n..];
        Some(v)
    }

    /// `%d`, read at `long` and saturated to `int` where glibc keeps the low
    /// 32 bits.
    pub fn int(&mut self) -> Option<i32> {
        let read = (!self.failed).then(|| scanf_long(self.rest, 10)).flatten();
        self.take(read.map(|(v, n)| {
            let v = v.clamp(c_long::from(i32::MIN), c_long::from(i32::MAX));
            (v as i32, n)
        }))
    }

    /// `%x` into an `int`.
    pub fn hex(&mut self) -> Option<i32> {
        self.conv("%x%n")
    }

    /// `%E` / `%f` into a `float`.
    pub fn float(&mut self) -> Option<f32> {
        self.conv("%E%n")
    }

    /// `%c`: the next byte, whitespace included.
    pub fn char(&mut self) -> Option<u8> {
        self.conv("%c%n")
    }

    /// A literal byte in the format.
    pub fn literal(&mut self, byte: u8) -> &mut Self {
        if !self.failed {
            match self.rest.split_first() {
                Some((&b, rest)) if b == byte => self.rest = rest,
                _ => self.failed = true,
            }
        }
        self
    }

    /// A literal `,` in the format.
    pub fn comma(&mut self) -> &mut Self {
        self.literal(b',')
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `sscanf(pvalue, "%d,%E,%E", ...)`: each conversion lands until the
    /// first that fails.
    #[test]
    fn scan_stops_at_the_first_failed_conversion() {
        let mut s = Scan::new(b"0,1.2300E-06,4.5000E-06");
        assert_eq!(s.int(), Some(0));
        assert_eq!(s.comma().float(), Some(1.23e-6));
        assert_eq!(s.comma().float(), Some(4.5e-6));
        let mut s = Scan::new(b"4");
        assert_eq!(s.int(), Some(4));
        assert_eq!(s.comma().float(), None);
        let mut s = Scan::new(b"");
        assert_eq!(s.int(), None);
        assert_eq!(s.comma().int(), None);
    }

    /// `"%d=%f%c%x%c"` on a Televac reply.
    #[test]
    fn chars_and_hex_read_as_c_does() {
        let mut s = Scan::new(b"1=7.60-2T");
        assert_eq!(s.int(), Some(1));
        assert_eq!(s.literal(b'=').float(), Some(7.6));
        assert_eq!(s.char(), Some(b'-'));
        assert_eq!(s.hex(), Some(2));
        assert_eq!(s.char(), Some(b'T'));
        assert_eq!(s.char(), None);
        assert_eq!(Scan::new(b"0F").hex(), Some(15));
        assert_eq!(Scan::new(b"0x1f").hex(), Some(31));
        assert_eq!(Scan::new(b"0n").hex(), Some(0));
        assert_eq!(Scan::new(b"n0").hex(), None);
    }

    /// glibc's `sscanf("%E")` fails a float it read only part of, where
    /// `strtod` would take its longest prefix; gcc's output on this host.
    #[test]
    fn a_float_cut_short_fails_the_scan() {
        for text in [&b"infi,5"[..], b"0x,5", b"0xg,5"] {
            let mut s = Scan::new(text);
            let text = String::from_utf8_lossy(text);
            assert_eq!(s.float(), None, "{text}");
            assert_eq!(s.comma().int(), None, "{text}");
        }
        let mut s = Scan::new(b"inf,5");
        assert_eq!(s.float(), Some(f32::INFINITY));
        assert_eq!(s.comma().int(), Some(5));
        let mut s = Scan::new(b" 0x1p3,5");
        assert_eq!(s.float(), Some(8.0));
        assert_eq!(s.comma().int(), Some(5));
    }

    /// glibc's blanks include `\v`.
    #[test]
    fn conversions_skip_a_vertical_tab() {
        assert_eq!(Scan::new(b"\x0b7").int(), Some(7));
        assert_eq!(Scan::new(b"\x0b1f").hex(), Some(31));
    }

    /// `%E` into a `float` rounds the decimal once; through a `double` this
    /// one lands on a float tie and rounds to 1.0.
    #[test]
    fn a_float_rounds_once() {
        assert_eq!(
            Scan::new(b"1.0000000596046448").float(),
            Some(1.0 + f32::EPSILON)
        );
    }
}
