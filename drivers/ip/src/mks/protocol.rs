//! MKS 937 vacuum-gauge-controller protocol (`devAiMKS.c`).
//!
//! Three commands, all answered with a fixed 7-character reply:
//!
//! ```text
//! SU   pressure units, e.g. "Torr   "
//! SG   gauge type per slot, 2 characters each: "CcPrPr "
//! Rn   pressure of gauge n (1..5), or a status word such as "H " / "NO"
//! ```

/// Gauges an MKS 937 can carry (`devAiMKS.c:136`).
pub const MAX_GAUGES: usize = 5;

/// Read the pressure units.
pub const READ_UNITS: &str = "SU";

/// Read the gauge types.
pub const READ_GAUGE_TYPES: &str = "SG";

/// `Rn` — read gauge `gauge` (1..5).
pub fn read_pressure(gauge: u8) -> String {
    format!("R{gauge}")
}

/// The gauge boards: slot 1 holds gauge 1, slot 2 gauges 2-3, slot 3 gauges 4-5
/// (`devAiMKS.c:205-217`).
pub fn gauge_type_field(reply: &str, gauge: u8) -> Option<&str> {
    let offset = match gauge {
        1 => 0,
        2 | 3 => 2,
        4 | 5 => 4,
        _ => return None,
    };
    reply.get(offset..offset + 2)
}

/// Gauge types the C knows, with the Torr limits it assigns
/// (`devAiMKS.c:218-242`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GaugeType {
    ColdCathode,
    Pirani,
    CapacitanceManometer,
    Thermocouple,
    Convection,
}

impl GaugeType {
    pub fn from_code(code: &str) -> Option<Self> {
        match code {
            "Cc" => Some(Self::ColdCathode),
            "Pr" => Some(Self::Pirani),
            "Cm" => Some(Self::CapacitanceManometer),
            "Tc" => Some(Self::Thermocouple),
            "Cv" => Some(Self::Convection),
            _ => None,
        }
    }

    /// `(low, high)` in Torr.
    pub fn limits(self) -> (f64, f64) {
        match self {
            Self::ColdCathode => (1.0e-11, 1.0e-2),
            Self::Pirani => (5.0e-4, 760.0),
            // These depend on the head model (devAiMKS.c:228).
            Self::CapacitanceManometer => (1.0e-3, 1.0e0),
            Self::Thermocouple => (1.0e-3, 1.0e0),
            Self::Convection => (1.0e-3, 1.0e3),
        }
    }
}

/// Convert the Torr gauge limits to the units the controller is reporting in
/// (`devAiMKS.c:247-250`).
pub fn unit_multiplier(units: &str) -> f64 {
    if units.starts_with("mbar") {
        1.3
    } else if units.starts_with("Pascal") {
        130.0
    } else if units.starts_with("micron") {
        1000.0
    } else {
        1.0
    }
}

/// What an `Rn` reply says (`devAiMKS.c:317-354`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Reading {
    /// A number: the pressure.
    Pressure(f64),
    /// `H ` / `A `: above the gauge's range. The C publishes the high limit and
    /// raises a MINOR alarm.
    AboveRange,
    /// `L `: below the gauge's range — low limit, MINOR alarm.
    BelowRange,
    /// `MI` (misconnected), `NO` (no gauge), `HV` (high voltage off) — MAJOR.
    NoGauge,
    /// `SYNTAX!` / `NotCMD!`: the 937 emits these spuriously, and the C
    /// deliberately neither alarms nor logs (`devAiMKS.c:338-347`).
    Spurious,
    /// Anything else — MAJOR alarm.
    Unknown,
}

pub fn parse_reading(reply: &str) -> Reading {
    let text = reply.trim_end_matches(['\r', '\n']);
    let first = text.chars().next();
    if first.is_some_and(|c| c.is_ascii_digit() || c == ' ') {
        // The C uses atof(), which takes the leading number and ignores the rest.
        return match text.trim().parse::<f64>() {
            Ok(value) => Reading::Pressure(value),
            Err(_) => Reading::Unknown,
        };
    }
    if text.starts_with("H ") || text.starts_with("A ") {
        Reading::AboveRange
    } else if text.starts_with("L ") {
        Reading::BelowRange
    } else if text.starts_with("MI") || text.starts_with("NO") || text.starts_with("HV") {
        Reading::NoGauge
    } else if text.starts_with("SYNTAX!") || text.starts_with("NotCMD!") {
        Reading::Spurious
    } else {
        Reading::Unknown
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commands_match_the_c_strings() {
        assert_eq!(read_pressure(3), "R3");
        assert_eq!(READ_UNITS, "SU");
        assert_eq!(READ_GAUGE_TYPES, "SG");
    }

    #[test]
    fn gauge_types_come_from_the_board_slots() {
        let reply = "CcPrPr ";
        assert_eq!(gauge_type_field(reply, 1), Some("Cc"));
        assert_eq!(gauge_type_field(reply, 2), Some("Pr"));
        assert_eq!(gauge_type_field(reply, 3), Some("Pr"));
        assert_eq!(gauge_type_field(reply, 4), Some("Pr"));
        assert_eq!(gauge_type_field(reply, 5), Some("Pr"));
        assert_eq!(gauge_type_field(reply, 6), None);

        assert_eq!(GaugeType::from_code("Cc"), Some(GaugeType::ColdCathode));
        assert_eq!(GaugeType::from_code("Cv"), Some(GaugeType::Convection));
        assert_eq!(GaugeType::from_code("??"), None);
        assert_eq!(GaugeType::ColdCathode.limits(), (1.0e-11, 1.0e-2));
        assert_eq!(GaugeType::Pirani.limits(), (5.0e-4, 760.0));
    }

    #[test]
    fn unit_multipliers_convert_the_torr_limits() {
        assert_eq!(unit_multiplier("Torr   "), 1.0);
        assert_eq!(unit_multiplier("mbar   "), 1.3);
        assert_eq!(unit_multiplier("Pascal "), 130.0);
        assert_eq!(unit_multiplier("micron "), 1000.0);
    }

    #[test]
    fn readings_are_classified_as_the_c_does() {
        assert_eq!(parse_reading("1.2E-06"), Reading::Pressure(1.2e-6));
        assert_eq!(parse_reading(" 7.6E+02"), Reading::Pressure(760.0));
        assert_eq!(parse_reading("H 1.0E0"), Reading::AboveRange);
        assert_eq!(parse_reading("A 1.0E0"), Reading::AboveRange);
        assert_eq!(parse_reading("L 1.0E0"), Reading::BelowRange);
        assert_eq!(parse_reading("NO GAUG"), Reading::NoGauge);
        assert_eq!(parse_reading("MISCONN"), Reading::NoGauge);
        assert_eq!(parse_reading("HV OFF "), Reading::NoGauge);
        assert_eq!(parse_reading("SYNTAX!"), Reading::Spurious);
        assert_eq!(parse_reading("NotCMD!"), Reading::Spurious);
        assert_eq!(parse_reading("garbage"), Reading::Unknown);
    }
}
