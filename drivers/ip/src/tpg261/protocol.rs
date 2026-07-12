//! Pfeiffer TPG261 / TPG262 gauge-controller protocol (`devTPG261.c`).
//!
//! Every transaction is two exchanges (`devTPG261.c:276-286`):
//!
//! ```text
//! -> "PR1<CR>"      the command
//! <- <ACK><CR><LF>  or <NAK><CR><LF>
//! -> <ENQ>          ask for the data
//! <- "0,1.234E-6<CR><LF>"
//! ```
//!
//! The `<CR><LF>` framing comes from the octet port's input EOS; the C set it
//! per transaction, here `st.cmd` sets it once on the port.

/// Gauges per controller: TPG262 has two, TPG261 one.
pub const NUM_GAUGES: usize = 2;

/// Setpoints per gauge (the controller has four, two per gauge).
pub const SETPOINTS_PER_GAUGE: usize = 2;

/// `<ENQ>` — "send me the data" (`devTPG261.c:653`).
pub const ENQ: u8 = 0x05;

/// `<ACK>` (`devTPG261.c:655`).
pub const ACK: u8 = 0x06;

/// `<NAK>`.
///
/// The C wrote `strcpy(nak, "\21")` (`devTPG261.c:654`), but `\21` is a C octal
/// escape: 0o21 = 0x11 = DC1, not NAK. ACK and ENQ in the same block are right
/// (`"\06"`, `"\05"`), so the author meant decimal 21 = 0x15 and the comparison
/// against a real NAK never matched — a rejected command was read as if it had
/// been accepted. This is the ASCII value.
pub const NAK: u8 = 0x15;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum TpgError {
    #[error("the controller rejected the command (NAK)")]
    Nak,

    #[error("empty reply")]
    Empty,

    #[error("cannot parse {field} from {raw:?}")]
    Parse { field: &'static str, raw: String },
}

/// Gauge measurement status, first field of a `PRn` reply (`devTPG261.c:337`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GaugeStatus {
    /// 0 — measurement ok.
    Ok,
    /// 4 — no sensor / sensor off: the C raises a MINOR alarm.
    NoSensor,
    /// Anything else: the C raises an INVALID alarm.
    Error(i32),
}

impl GaugeStatus {
    pub fn from_code(code: i32) -> Self {
        match code {
            0 => Self::Ok,
            4 => Self::NoSensor,
            other => Self::Error(other),
        }
    }
}

/// `PRn` — read the pressure of gauge `gauge` (1 or 2).
pub fn read_pressure(gauge: u8) -> String {
    format!("PR{gauge}")
}

/// `SPn` — read setpoint `n` (1..4). Setpoints 1-2 belong to gauge 1, 3-4 to
/// gauge 2 (`devTPG261.c:310-323`).
pub fn read_setpoint(gauge: u8, index: usize) -> String {
    format!("SP{}", setpoint_number(gauge, index))
}

/// Setpoint number 1..4 for `gauge` (1 or 2) and `index` (0 or 1).
pub fn setpoint_number(gauge: u8, index: usize) -> u8 {
    2 * (gauge - 1) + index as u8 + 1
}

/// `SPS` — read all four setpoint relay states.
pub fn read_setpoint_states() -> String {
    "SPS".to_string()
}

/// `SEN` — read both gauges' on/off state.
pub fn read_sensor_states() -> String {
    "SEN".to_string()
}

/// `UNI` — read the pressure units.
pub fn read_units() -> String {
    "UNI".to_string()
}

/// `TID` — read both gauge identifications.
pub fn read_gauge_id() -> String {
    "TID".to_string()
}

/// `SPn,s,<lower>,<upper>` — the C sets both thresholds to the requested value
/// (`devTPG261.c:513-529`).
pub fn write_setpoint(gauge: u8, index: usize, value: f64) -> String {
    let number = setpoint_number(gauge, index);
    let pressure = crate::fmt::exp_c(value, 4);
    format!("SP{number},{},{pressure},{pressure}", gauge - 1)
}

/// `SEN,<g1>,<g2>` — 0 leaves a gauge alone, 1 switches it off, 2 on. VAL 0
/// therefore sends 1 and VAL 1 sends 2 (`devTPG261.c:559-565`).
pub fn write_sensor(gauge: u8, on: bool) -> String {
    let code = i32::from(on) + 1;
    if gauge == 1 {
        format!("SEN,{code},0")
    } else {
        format!("SEN,0,{code}")
    }
}

/// `UNI,<n>` (`devTPG261.c:596-597`).
pub fn write_units(units: i32) -> String {
    format!("UNI,{units}")
}

/// Classify the controller's answer to a command: ACK or NAK.
pub fn check_ack(reply: &str) -> Result<(), TpgError> {
    let reply = reply.trim_end_matches(['\r', '\n']);
    match reply.as_bytes().first() {
        None => Err(TpgError::Empty),
        Some(&b) if b == NAK => Err(TpgError::Nak),
        Some(_) => Ok(()),
    }
}

fn fields<'a>(raw: &'a str, field: &'static str, want: usize) -> Result<Vec<&'a str>, TpgError> {
    let parts: Vec<&str> = raw.trim().split(',').map(str::trim).collect();
    if parts.len() < want {
        return Err(TpgError::Parse {
            field,
            raw: raw.to_string(),
        });
    }
    Ok(parts)
}

fn int(field: &'static str, text: &str) -> Result<i32, TpgError> {
    text.parse().map_err(|_| TpgError::Parse {
        field,
        raw: text.to_string(),
    })
}

fn float(field: &'static str, text: &str) -> Result<f64, TpgError> {
    text.parse().map_err(|_| TpgError::Parse {
        field,
        raw: text.to_string(),
    })
}

/// `<status>,<pressure>` (`devTPG261.c:336`).
pub fn parse_pressure(raw: &str) -> Result<(GaugeStatus, f64), TpgError> {
    let parts = fields(raw, "pressure", 2)?;
    Ok((
        GaugeStatus::from_code(int("gauge status", parts[0])?),
        float("pressure", parts[1])?,
    ))
}

/// `<status>,<lower>,<upper>` — the C keeps the lower threshold
/// (`devTPG261.c:349`).
pub fn parse_setpoint(raw: &str) -> Result<f64, TpgError> {
    let parts = fields(raw, "setpoint", 2)?;
    float("setpoint", parts[1])
}

/// `<s1>,<s2>,<s3>,<s4>` — the relay state of every setpoint.
///
/// The C read these into `int vals[3]` with a four-conversion `sscanf`
/// (`devTPG261.c:396`), writing one element past the array.
pub fn parse_setpoint_states(raw: &str) -> Result<[i32; 4], TpgError> {
    let parts = fields(raw, "setpoint states", 4)?;
    Ok([
        int("setpoint state", parts[0])?,
        int("setpoint state", parts[1])?,
        int("setpoint state", parts[2])?,
        int("setpoint state", parts[3])?,
    ])
}

/// `<s1>,<s2>` — both gauges' on/off state (`devTPG261.c:405`).
pub fn parse_sensor_states(raw: &str) -> Result<[i32; NUM_GAUGES], TpgError> {
    let parts = fields(raw, "sensor states", 2)?;
    Ok([
        int("sensor state", parts[0])?,
        int("sensor state", parts[1])?,
    ])
}

/// A bare integer (`devTPG261.c:442`).
pub fn parse_units(raw: &str) -> Result<i32, TpgError> {
    int("units", raw.trim())
}

/// `<id1>,<id2>` — one identification per gauge (`devTPG261.c:480-490`).
///
/// The C indexed the reply with `strchr(recBuf, ',')` and dereferenced the
/// result without a NULL check, so a controller answering without a comma
/// crashed the IOC; it also copied the id into a 10-byte buffer.
pub fn parse_gauge_id(raw: &str, gauge: u8) -> Result<String, TpgError> {
    let parts = fields(raw, "gauge id", NUM_GAUGES)?;
    Ok(parts[usize::from(gauge - 1)].to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn setpoint_numbers_map_gauges_to_relays() {
        assert_eq!(setpoint_number(1, 0), 1);
        assert_eq!(setpoint_number(1, 1), 2);
        assert_eq!(setpoint_number(2, 0), 3);
        assert_eq!(setpoint_number(2, 1), 4);
    }

    #[test]
    fn read_commands_match_the_c_strings() {
        assert_eq!(read_pressure(2), "PR2");
        assert_eq!(read_setpoint(2, 1), "SP4");
        assert_eq!(read_setpoint_states(), "SPS");
        assert_eq!(read_sensor_states(), "SEN");
        assert_eq!(read_units(), "UNI");
        assert_eq!(read_gauge_id(), "TID");
    }

    #[test]
    fn write_commands_match_the_c_strings() {
        // sprintf("SP%d,%d,", stptNo, address-1) + "%8.4E" twice.
        assert_eq!(write_setpoint(1, 0, 1.0e-6), "SP1,0,1.0000E-06,1.0000E-06");
        assert_eq!(write_setpoint(2, 1, 2.5e-3), "SP4,1,2.5000E-03,2.5000E-03");
        // sprintf("SEN,%d,0", val+1)
        assert_eq!(write_sensor(1, false), "SEN,1,0");
        assert_eq!(write_sensor(1, true), "SEN,2,0");
        assert_eq!(write_sensor(2, true), "SEN,0,2");
        assert_eq!(write_units(1), "UNI,1");
    }

    #[test]
    fn nak_is_rejected_and_ack_accepted() {
        assert_eq!(check_ack("\u{6}\r\n"), Ok(()));
        assert_eq!(check_ack("\u{15}\r\n"), Err(TpgError::Nak));
        // The C compared against 0x11 (its "\21" is octal), so this NAK
        // was read as an ACK and the ENQ went out anyway.
        assert_eq!(check_ack(""), Err(TpgError::Empty));
    }

    #[test]
    fn pressure_carries_the_gauge_status() {
        assert_eq!(
            parse_pressure("0,1.2340E-06").unwrap(),
            (GaugeStatus::Ok, 1.234e-6)
        );
        assert_eq!(
            parse_pressure("4,0.0000E+00").unwrap().0,
            GaugeStatus::NoSensor
        );
        assert_eq!(
            parse_pressure("5,0.0000E+00").unwrap().0,
            GaugeStatus::Error(5)
        );
        assert!(parse_pressure("0").is_err());
    }

    #[test]
    fn setpoint_readback_takes_the_lower_threshold() {
        assert_eq!(parse_setpoint("0,1.0000E-06,2.0000E-06").unwrap(), 1.0e-6);
    }

    #[test]
    fn all_four_setpoint_states_are_parsed() {
        assert_eq!(parse_setpoint_states("0,1,0,1").unwrap(), [0, 1, 0, 1]);
        // Three fields is what the C's vals[3] could actually hold.
        assert!(parse_setpoint_states("0,1,0").is_err());
    }

    #[test]
    fn sensor_states_units_and_ids() {
        assert_eq!(parse_sensor_states("2,1").unwrap(), [2, 1]);
        assert_eq!(parse_units("1").unwrap(), 1);
        assert_eq!(parse_gauge_id("TPR,IKR", 1).unwrap(), "TPR");
        assert_eq!(parse_gauge_id("TPR,IKR", 2).unwrap(), "IKR");
        // A reply with no comma crashed the C.
        assert!(parse_gauge_id("TPR", 1).is_err());
    }
}
