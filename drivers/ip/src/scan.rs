//! C `sscanf` over a device reply, one conversion at a time, for the formats
//! `devTPG261.c` and `devTelevac.c` scan.

use epics_rs::base::runtime::stdlib::strtod;

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

    /// `%d`.
    pub fn int(&mut self) -> Option<i32> {
        if self.failed {
            return None;
        }
        let s = self.rest.trim_ascii_start();
        let sign = usize::from(matches!(s.first(), Some(b'-' | b'+')));
        let digits = s[sign..].iter().take_while(|b| b.is_ascii_digit()).count();
        if digits == 0 {
            self.failed = true;
            return None;
        }
        let end = sign + digits;
        self.rest = &s[end..];
        // Out-of-range input is undefined in C; saturate.
        let text = std::str::from_utf8(&s[..end]).unwrap_or("0");
        Some(
            text.parse::<i64>()
                .map_or(0, |v| v.clamp(i32::MIN.into(), i32::MAX.into()) as i32),
        )
    }

    /// `%x` into an `int`: an optional sign and `0x`, then hex digits.
    pub fn hex(&mut self) -> Option<i32> {
        if self.failed {
            return None;
        }
        let s = self.rest.trim_ascii_start();
        let negative = s.first() == Some(&b'-');
        let mut at = usize::from(matches!(s.first(), Some(b'-' | b'+')));
        if s[at..].len() > 2
            && s[at] == b'0'
            && matches!(s[at + 1], b'x' | b'X')
            && s[at + 2].is_ascii_hexdigit()
        {
            at += 2;
        }
        let digits = s[at..].iter().take_while(|b| b.is_ascii_hexdigit()).count();
        if digits == 0 {
            self.failed = true;
            return None;
        }
        self.rest = &s[at + digits..];
        let text = std::str::from_utf8(&s[at..at + digits]).unwrap_or("0");
        // C stores the conversion's low 32 bits.
        let value = u64::from_str_radix(text, 16).unwrap_or(u64::MAX) as u32 as i32;
        Some(if negative {
            value.wrapping_neg()
        } else {
            value
        })
    }

    /// `%E` / `%f` into a `float`.
    pub fn float(&mut self) -> Option<f32> {
        if self.failed {
            return None;
        }
        let parsed = strtod(self.rest);
        if parsed.len == 0 {
            self.failed = true;
            return None;
        }
        self.rest = &self.rest[parsed.len..];
        Some(parsed.value as f32)
    }

    /// `%c`: the next byte, whitespace included.
    pub fn char(&mut self) -> Option<u8> {
        if self.failed {
            return None;
        }
        match self.rest.split_first() {
            Some((&c, rest)) => {
                self.rest = rest;
                Some(c)
            }
            None => {
                self.failed = true;
                None
            }
        }
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
}
