//! What `devTPG261.c` puts on the wire and takes off it.
//!
//! Every exchange is two round trips (`devTPG261.c:276-286`): the command with
//! a `<CR>`, answered `<ACK>` or `<NAK>`, then `<ENQ>`, answered with the data.
//! Both replies end in `<CR><LF>`, the input EOS the callback sets
//! (`devTPG261.c:657`).

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
}
