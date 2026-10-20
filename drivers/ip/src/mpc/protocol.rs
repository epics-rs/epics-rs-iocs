//! What `devMPC.c` puts on the wire and takes off it.
//!
//! A command is `~ AA XX d 00` (`buildCommand`, `devMPC.c:313-346`): the
//! controller address in two hex digits, the command in two, the data, and the
//! constant `00` checksum C sends ("the device is happy with just 00"). The
//! `<CR>` on both directions is the octet port's EOS.

use epics_rs::base::runtime::stdlib::strtod;

/// `devMPC.h`'s command numbers, by `mpcCommands[]`'s names (`devMPC.c:90-126`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    GetStatus = 0,
    GetPres = 1,
    GetCur = 2,
    GetVolt = 3,
    GetSize = 4,
    GetSpVal12 = 5,
    GetSpS12 = 6,
    GetSpVal34 = 7,
    GetSpS34 = 8,
    GetSpVal56 = 9,
    GetSpS56 = 10,
    GetSpVal78 = 11,
    GetSpS78 = 12,
    GetAutoRestart = 13,
    GetTspStat = 14,
    SetUnit = 20,
    SetDis = 21,
    SetSize = 22,
    SetSp12 = 23,
    SetSp34 = 24,
    SetSp56 = 25,
    SetSp78 = 26,
    SetStart = 27,
    SetStop = 28,
    SetLock = 29,
    SetUnlock = 30,
    SetAutoRestart = 31,
    SetTspTimed = 32,
    SetTspOff = 33,
    SetTspFilament = 34,
    SetTspClear = 35,
    SetTspAutoAdv = 36,
    SetTspContinuous = 37,
    SetTspSublimation = 38,
    SetTspDegas = 39,
}

const COMMANDS: [(Command, &str); 35] = [
    (Command::GetStatus, "GET_STATUS"),
    (Command::GetPres, "GET_PRESSURE"),
    (Command::GetCur, "GET_CURRENT"),
    (Command::GetVolt, "GET_VOLT"),
    (Command::GetSize, "GET_SIZE"),
    (Command::GetSpVal12, "GET_SPVAL12"),
    (Command::GetSpS12, "GET_SPS12"),
    (Command::GetSpVal34, "GET_SPVAL34"),
    (Command::GetSpS34, "GET_SPS34"),
    (Command::GetSpVal56, "GET_SPVAL56"),
    (Command::GetSpS56, "GET_SPS56"),
    (Command::GetSpVal78, "GET_SPVAL78"),
    (Command::GetSpS78, "GET_SPS78"),
    (Command::GetAutoRestart, "GET_AUTO_RESTART"),
    (Command::GetTspStat, "GET_TSP_STATUS"),
    (Command::SetUnit, "SET_UNIT"),
    (Command::SetDis, "SET_DISPLAY"),
    (Command::SetSize, "SET_SIZE"),
    (Command::SetSp12, "SET_SP12"),
    (Command::SetSp34, "SET_SP34"),
    (Command::SetSp56, "SET_SP56"),
    (Command::SetSp78, "SET_SP78"),
    (Command::SetStart, "SET_START"),
    (Command::SetStop, "SET_STOP"),
    (Command::SetLock, "SET_LOCK"),
    (Command::SetUnlock, "SET_UNLOCK"),
    (Command::SetAutoRestart, "SET_AUTO_RESTART"),
    (Command::SetTspTimed, "SET_TSP_TIMED"),
    (Command::SetTspOff, "SET_TSP_OFF"),
    (Command::SetTspFilament, "SET_TSP_FILAMENT"),
    (Command::SetTspClear, "SET_TSP_CLEAR"),
    (Command::SetTspAutoAdv, "SET_TSP_AUTO_ADVANCE"),
    (Command::SetTspContinuous, "SET_TSP_CONTINUOUS"),
    (Command::SetTspSublimation, "SET_TSP_SUBLIMATION"),
    (Command::SetTspDegas, "SET_TSP_DEGAS"),
];

impl Command {
    /// `epicsStrCaseCmp` over `mpcCommands[]` (`devMPC.c:243-249`).
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

/// `DisplayStr[]` (`devMPC.h:50`), indexed by the mbbo's RVAL.
pub const DISPLAY_STRINGS: [&str; 3] = [",PRES", ",CUR", ",VOLT"];

/// `buildCommand`'s `sendBuf`: `address` is already `%02X`.
pub fn build_command(address: &str, hex: u8, data: &str) -> String {
    format!("~ {address} {hex:02X} {data} 00")
}

/// `initCommon`'s `sscanf(userParam, "%s %d", command, &i)`: the command word
/// and the supply. A missing supply leaves C's `i` uninitialised; it is 0 here.
pub fn split_user_param(user_param: &str) -> (&str, i32) {
    let mut words = user_param.split_ascii_whitespace();
    let command = words.next().unwrap_or("");
    let supply = words.next().map_or(0, |w| c_atoi(w.as_bytes()));
    (command, supply)
}

/// C `atoi`, and `sscanf("%d")` where it converts: glibc reads both as
/// `(int) strtol`. 0 when there are no digits.
pub use epics_rs::base::types::c_parse::atoi as c_atoi;

/// C `strtod(s, NULL)`.
pub fn c_strtod(s: &[u8]) -> f64 {
    strtod(s).value
}

/// `devMPCCallback`'s reply stripping (`devMPC.c:890-898`), on a read of at
/// least 4 bytes: an `OK` at offset 3 keeps the data between the 9-byte header
/// and the trailing space and checksum, or `OK` for a reply shorter than 12;
/// anything else leaves `recBuf` empty.
pub fn strip_reply(read: &[u8]) -> Vec<u8> {
    if read.get(3) != Some(&b'O') || read.get(4) != Some(&b'K') {
        return Vec::new();
    }
    if read.len() < 12 {
        return b"OK".to_vec();
    }
    let data = &read[9..9 + read.len() - 12];
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    data[..end].to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_frame_matches_build_command() {
        assert_eq!(build_command("05", 0x0b, "1"), "~ 05 0B 1 00");
        assert_eq!(build_command("05", 0x2a, ""), "~ 05 2A  00");
    }

    #[test]
    fn command_names_match_case_insensitively() {
        assert_eq!(Command::from_name("get_pressure"), Some(Command::GetPres));
        assert_eq!(
            Command::from_name("SET_TSP_DEGAS"),
            Some(Command::SetTspDegas)
        );
        assert_eq!(Command::from_name("GET_PRES"), None);
        assert_eq!(Command::SetTspDegas.number(), 39);
    }

    #[test]
    fn user_param_is_the_command_and_the_supply() {
        assert_eq!(split_user_param("GET_STATUS 2"), ("GET_STATUS", 2));
        assert_eq!(split_user_param("GET_STATUS"), ("GET_STATUS", 0));
        assert_eq!(split_user_param("GET_STATUS x"), ("GET_STATUS", 0));
    }

    /// One case per `devMPCCallback` arm after the size checks.
    #[test]
    fn replies_strip_as_dev_mpc_callback_does() {
        assert_eq!(strip_reply(b"05 OK 00 1.2E-09 MBAR 3B"), b"1.2E-09 MBAR");
        assert_eq!(strip_reply(b"05 OK 00 Y 3B"), b"Y");
        assert_eq!(strip_reply(b"05 OK 00 3B"), b"OK");
        assert_eq!(strip_reply(b"05 ER 00 SYNTAX 3B"), b"");
        assert_eq!(strip_reply(b"05 O"), b"");
    }

    #[test]
    fn numbers_parse_as_c_does() {
        assert_eq!(c_strtod(b"1.2E-09"), 1.2e-9);
        assert_eq!(c_strtod(b" 30L/S"), 30.0);
        assert_eq!(c_strtod(b"junk"), 0.0);
        assert_eq!(c_atoi(b"1"), 1);
        assert_eq!(c_atoi(b"x"), 0);
        // glibc: (int) strtol, the same for sscanf("%d").
        assert_eq!(c_atoi(b"4294967297"), 1);
        assert_eq!(c_atoi(b"99999999999999999999"), -1);
    }
}
