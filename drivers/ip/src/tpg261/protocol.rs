//! What `devTPG261.c` puts on the wire and takes off it.
//!
//! Every exchange is two round trips (`devTPG261.c:276-286`): the command with
//! a `<CR>`, answered `<ACK>` or `<NAK>`, then `<ENQ>`, answered with the data.
//! Both replies end in `<CR><LF>`, the input EOS the callback sets
//! (`devTPG261.c:657`).

use epics_rs::base::calc::engine::strtod::strtod;

/// `<ENQ>`, which asks for the data (`devTPG261.c:653`).
pub const ENQ: u8 = 0x05;

/// `<NAK>`. C compares against `"\21"` (`devTPG261.c:654`), octal for DC1, so a
/// refused command was never seen; this is the real NAK.
pub const NAK: u8 = 0x15;

/// `devTPG261.h`'s command numbers, by `tpg261Commands[]`'s names
/// (`devTPG261.c:69-82`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    GetId = 0,
    GetPressure = 1,
    GetUnit = 2,
    SetUnit = 3,
    GetSensor = 4,
    SetSensor = 5,
    GetSpVal1 = 6,
    GetSpS1 = 7,
    SetSp1 = 8,
    GetSpVal2 = 9,
    GetSpS2 = 10,
    SetSp2 = 11,
}

const COMMANDS: [(Command, &str); 12] = [
    (Command::GetId, "GET_ID"),
    (Command::GetPressure, "GET_PRESSURE"),
    (Command::GetUnit, "GET_UNIT"),
    (Command::SetUnit, "SET_UNIT"),
    (Command::GetSensor, "GET_SENSOR"),
    (Command::SetSensor, "SET_SENSOR"),
    (Command::GetSpVal1, "GET_SPVAL1"),
    (Command::GetSpS1, "GET_SPS1"),
    (Command::SetSp1, "SET_SP1"),
    (Command::GetSpVal2, "GET_SPVAL2"),
    (Command::GetSpS2, "GET_SPS2"),
    (Command::SetSp2, "SET_SP2"),
];

impl Command {
    /// `epicsStrCaseCmp` against each name.
    pub fn from_name(name: &str) -> Option<Self> {
        COMMANDS
            .iter()
            .find(|(_, n)| n.eq_ignore_ascii_case(name))
            .map(|(c, _)| *c)
    }

    pub fn number(self) -> i32 {
        self as i32
    }
}

/// C `sscanf` over a reply, one conversion at a time; a failed conversion
/// stops the scan and leaves the rest unassigned.
pub struct Scan<'a> {
    rest: &'a [u8],
    failed: bool,
}

impl<'a> Scan<'a> {
    pub fn new(text: &'a str) -> Self {
        Self {
            rest: text.as_bytes(),
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

    /// `%E` into a `float`.
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

    /// A literal `,` in the format.
    pub fn comma(&mut self) -> &mut Self {
        if !self.failed {
            match self.rest.split_first() {
                Some((b',', rest)) => self.rest = rest,
                _ => self.failed = true,
            }
        }
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_names_match_case_insensitively() {
        assert_eq!(
            Command::from_name("get_pressure"),
            Some(Command::GetPressure)
        );
        assert_eq!(Command::from_name("SET_SP2"), Some(Command::SetSp2));
        assert_eq!(Command::from_name("GET_FOO"), None);
    }

    /// `sscanf(pvalue, "%d,%E,%E", ...)`: each conversion lands until the
    /// first that fails.
    #[test]
    fn scan_stops_at_the_first_failed_conversion() {
        let mut s = Scan::new("0,1.2300E-06,4.5000E-06");
        assert_eq!(s.int(), Some(0));
        assert_eq!(s.comma().float(), Some(1.23e-6));
        assert_eq!(s.comma().float(), Some(4.5e-6));
        let mut s = Scan::new("4");
        assert_eq!(s.int(), Some(4));
        assert_eq!(s.comma().float(), None);
        let mut s = Scan::new("");
        assert_eq!(s.int(), None);
        assert_eq!(s.comma().int(), None);
    }
}
