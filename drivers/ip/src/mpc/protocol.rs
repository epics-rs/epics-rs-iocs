//! MPC / Digitel ion-pump controller ASCII protocol (`devMPC.c`).
//!
//! Wire format, quoted from `devMPC.c:317-330`:
//!
//! ```text
//! ~ AA XX d cc<CR>
//! ```
//!
//! `AA` = controller address (2 hex digits), `XX` = 2-hex-digit command,
//! `d` = comma-separated data, `cc` = 2-hex-digit checksum. The C sends a
//! constant `00` checksum with the note "the device is happy with just 00";
//! that is a byte the device accepts, so it is kept verbatim here.
//!
//! The reply is `AA OK 00 <data> cc`: `OK` sits at offset 3, the payload at
//! offset 9, and the trailing three bytes are the space plus checksum.
//! The `<CR>` terminator on both directions comes from the octet port's EOS
//! (set in `st.cmd`), as it did in C.

/// Supplies (pumps) per controller: the MPC drives two.
pub const NUM_SUPPLIES: usize = 2;

/// Setpoint pairs: setpoints 1..8, two per pair (odd = supply 1, even = 2).
pub const NUM_SETPOINT_PAIRS: usize = 4;

/// Command codes, from the `hexCmd` values in `devMPC.c`.
pub mod cmd {
    pub const READ_STATUS: u8 = 0x0d;
    pub const READ_PRESSURE: u8 = 0x0b;
    pub const READ_CURRENT: u8 = 0x0a;
    pub const READ_VOLTAGE: u8 = 0x0c;
    pub const READ_SIZE: u8 = 0x11;
    pub const READ_SETPOINT: u8 = 0x3c;
    pub const READ_AUTO_RESTART: u8 = 0x34;
    pub const READ_TSP_STATUS: u8 = 0x2a;

    pub const SET_UNIT: u8 = 0x0e;
    pub const SET_DISPLAY: u8 = 0x25;
    pub const SET_SIZE: u8 = 0x12;
    pub const SET_SETPOINT: u8 = 0x3d;
    /// `devMPC.c:663` sends 0x37 for VAL 0 and 0x38 otherwise; `MPC.db`'s STOP
    /// record names VAL 0 `START` and VAL 1 `STOP`, so 0x37 starts the pump.
    pub const PUMP_START: u8 = 0x37;
    pub const PUMP_STOP: u8 = 0x38;
    pub const KEYBOARD_LOCK: u8 = 0x44;
    pub const KEYBOARD_UNLOCK: u8 = 0x45;
    pub const SET_AUTO_RESTART: u8 = 0x33;
    pub const TSP_TIMED: u8 = 0x27;
    pub const TSP_OFF: u8 = 0x28;
    pub const TSP_FILAMENT: u8 = 0x29;
    pub const TSP_CLEAR: u8 = 0x2b;
    pub const TSP_AUTO_ADVANCE: u8 = 0x2c;
    pub const TSP_CONTINUOUS: u8 = 0x2d;
    pub const TSP_SUBLIMATION: u8 = 0x2e;
    pub const TSP_DEGAS: u8 = 0x2f;
}

/// Display/units selection sent with `SET_UNIT` and `SET_DISPLAY`
/// (`DisplayStr[]`, `devMPC.h:50`).
pub const DISPLAY_STRINGS: [&str; 3] = [",PRES", ",CUR", ",VOLT"];

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum MpcError {
    #[error("reply is too short ({0} bytes): {1:?}")]
    TooShort(usize, String),

    /// The C (`devMPC.c:890`) only copies the payload when the reply says `OK`,
    /// but leaves the asyn status at success, so a rejected command silently
    /// published 0 / "" to the record. Here it is an error.
    #[error("controller rejected the command: {0:?}")]
    Rejected(String),

    #[error("cannot parse {field} from {raw:?}")]
    Parse { field: &'static str, raw: String },
}

/// Encode `~ AA XX d 00` (without the CR, which the octet port's output EOS
/// appends). `devMPC.c::buildCommand`.
pub fn build_command(address: u8, command: u8, data: &str) -> String {
    format!("~ {address:02X} {command:02X} {data} 00")
}

/// Setpoint number 1..8 for `pair` (0-based) and `supply` (1 or 2).
///
/// `devMPC.c:383` (`readAi`), `:502` (`readBi`) and `:621` (`writeAo`) each
/// spell this out; all three reduce to `pair * 2 + supply`.
pub fn setpoint_number(pair: usize, supply: u8) -> u8 {
    (pair as u8) * 2 + supply
}

/// Strip the MPC reply frame and return the payload (`devMPC.c:878-898`).
pub fn parse_reply(raw: &str) -> Result<String, MpcError> {
    let raw = raw.trim_end_matches(['\r', '\n']);
    let len = raw.len();
    if len < 5 {
        return Err(MpcError::TooShort(len, raw.to_string()));
    }
    if &raw[3..5] != "OK" {
        return Err(MpcError::Rejected(raw.to_string()));
    }
    if len < 12 {
        return Ok("OK".to_string());
    }
    Ok(raw[9..len - 3].to_string())
}

/// A reading plus the engineering units the controller reported for it.
#[derive(Debug, Clone, PartialEq)]
pub struct Reading {
    pub value: f64,
    pub egu: String,
}

fn number(field: &'static str, text: &str) -> Result<f64, MpcError> {
    text.trim().parse::<f64>().map_err(|_| MpcError::Parse {
        field,
        raw: text.to_string(),
    })
}

/// `<value 7 chars> <units>` — e.g. `1.2E-09 MBAR` (`devMPC.c:423-431`).
///
/// The C copies the units with `strncpy(pvalue, ploc, rtnSize-8)` into a
/// 10-byte buffer, which overruns for a longer reply; here the units are just
/// the rest of the payload.
pub fn parse_pressure(payload: &str) -> Result<Reading, MpcError> {
    if payload.len() < 9 {
        return Err(MpcError::TooShort(payload.len(), payload.to_string()));
    }
    Ok(Reading {
        value: number("pressure", &payload[..7])?,
        egu: payload[8..].trim().to_string(),
    })
}

/// `<value 7 chars>[ AMPS]` — units are forced to `AMPS` (`devMPC.c:432-438`).
pub fn parse_current(payload: &str) -> Result<Reading, MpcError> {
    if payload.len() < 7 {
        return Err(MpcError::TooShort(payload.len(), payload.to_string()));
    }
    Ok(Reading {
        value: number("current", &payload[..7])?,
        egu: "AMPS".to_string(),
    })
}

/// The whole payload is the voltage (`devMPC.c:439-445`).
pub fn parse_voltage(payload: &str) -> Result<Reading, MpcError> {
    Ok(Reading {
        value: number("voltage", payload)?,
        egu: "VOLTS".to_string(),
    })
}

/// `<size>L/S` — the digits before the `L` (`devMPC.c:446-458`).
pub fn parse_size(payload: &str) -> Result<Reading, MpcError> {
    let digits = payload.split('L').next().unwrap_or("");
    if digits.is_empty() {
        return Err(MpcError::Parse {
            field: "pump size",
            raw: payload.to_string(),
        });
    }
    Ok(Reading {
        value: number("pump size", digits)?,
        egu: "L/S".to_string(),
    })
}

/// `n,s,<value 7 chars>...` — the setpoint pressure starts at offset 4
/// (`devMPC.c:459-469`).
pub fn parse_setpoint_value(payload: &str) -> Result<Reading, MpcError> {
    if payload.len() < 11 {
        return Err(MpcError::TooShort(payload.len(), payload.to_string()));
    }
    Ok(Reading {
        value: number("setpoint", &payload[4..11])?,
        egu: "TORR".to_string(),
    })
}

/// The setpoint relay state is the last character of the payload
/// (`devMPC.c:527-533`).
pub fn parse_setpoint_state(payload: &str) -> Result<i32, MpcError> {
    let last = payload.trim_end().chars().last().ok_or(MpcError::Parse {
        field: "setpoint state",
        raw: payload.to_string(),
    })?;
    last.to_digit(10).map(|d| d as i32).ok_or(MpcError::Parse {
        field: "setpoint state",
        raw: payload.to_string(),
    })
}

/// `YES` / `NO` (`devMPC.c:534-536`).
pub fn parse_auto_restart(payload: &str) -> i32 {
    i32::from(payload.trim() == "YES")
}

/// Data field for `SET_SETPOINT`: `n,s,<on>,<off>`, both pressures set to
/// `value` (`devMPC.c:616-630`).
pub fn setpoint_data(pair: usize, supply: u8, value: f64) -> String {
    let number = setpoint_number(pair, supply);
    let pressure = crate::fmt::exp_c(value, 1);
    format!("{number},{supply},{pressure},{pressure}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_frame_matches_the_c_builder() {
        // devMPC.c: "~ " + "%02X" address + " %2.2X " command + data + " 00"
        assert_eq!(build_command(5, cmd::READ_PRESSURE, "1"), "~ 05 0B 1 00");
        assert_eq!(build_command(0, cmd::READ_STATUS, "2"), "~ 00 0D 2 00");
        assert_eq!(build_command(0xff, cmd::TSP_OFF, ""), "~ FF 28  00");
    }

    #[test]
    fn setpoint_numbers_are_one_to_eight() {
        assert_eq!(setpoint_number(0, 1), 1);
        assert_eq!(setpoint_number(0, 2), 2);
        assert_eq!(setpoint_number(3, 1), 7);
        assert_eq!(setpoint_number(3, 2), 8);
    }

    #[test]
    fn reply_payload_strips_header_and_checksum() {
        // "05 OK 00 " header (9 bytes), payload, " cc" trailer (3 bytes).
        assert_eq!(
            parse_reply("05 OK 00 1.2E-09 MBAR 3B").unwrap(),
            "1.2E-09 MBAR"
        );
        assert_eq!(parse_reply("05 OK 00\r").unwrap(), "OK");
    }

    #[test]
    fn rejected_reply_is_an_error_not_an_empty_payload() {
        // The C leaves recBuf empty and the status at success here.
        assert_eq!(
            parse_reply("05 ER 01 3F"),
            Err(MpcError::Rejected("05 ER 01 3F".to_string()))
        );
        assert!(matches!(parse_reply("05"), Err(MpcError::TooShort(2, _))));
    }

    #[test]
    fn pressure_carries_the_units_the_controller_reported() {
        let r = parse_pressure("1.2E-09 MBAR").unwrap();
        assert_eq!(r.value, 1.2e-9);
        assert_eq!(r.egu, "MBAR");
        // Longer units string: the C would have overrun its 10-byte buffer.
        assert_eq!(parse_pressure("3.4E-06 PASCAL").unwrap().egu, "PASCAL");
        assert!(parse_pressure("1.2E-09").is_err());
    }

    #[test]
    fn current_and_voltage_use_fixed_units() {
        let c = parse_current("2.1E-08 AMPS").unwrap();
        assert_eq!(c.value, 2.1e-8);
        assert_eq!(c.egu, "AMPS");
        // Recent controllers omit AMPS from the reply (devMPC.c:5-7).
        assert_eq!(parse_current("2.1E-08").unwrap().value, 2.1e-8);

        let v = parse_voltage("3200").unwrap();
        assert_eq!(v.value, 3200.0);
        assert_eq!(v.egu, "VOLTS");
    }

    #[test]
    fn pump_size_reads_the_digits_before_the_l() {
        let s = parse_size("60L/S").unwrap();
        assert_eq!(s.value, 60.0);
        assert_eq!(s.egu, "L/S");
        assert!(parse_size("L/S").is_err());
    }

    #[test]
    fn setpoint_readback_splits_value_and_state() {
        // "n,s,<7-char pressure>,<7-char pressure>,<state>"
        let payload = "1,1,1.0E-06,1.0E-06,0";
        assert_eq!(parse_setpoint_value(payload).unwrap().value, 1.0e-6);
        assert_eq!(parse_setpoint_state(payload).unwrap(), 0);
        assert_eq!(parse_setpoint_state("1,1,1.0E-06,1.0E-06,1").unwrap(), 1);
        assert!(parse_setpoint_value("1,1,1.0E").is_err());
    }

    #[test]
    fn auto_restart_is_yes_or_no() {
        assert_eq!(parse_auto_restart("YES"), 1);
        assert_eq!(parse_auto_restart("NO"), 0);
    }

    #[test]
    fn setpoint_write_repeats_the_pressure_for_on_and_off() {
        assert_eq!(setpoint_data(0, 1, 1.0e-6), "1,1,1.0E-06,1.0E-06");
        assert_eq!(setpoint_data(3, 2, 5.5e-8), "8,2,5.5E-08,5.5E-08");
    }
}
