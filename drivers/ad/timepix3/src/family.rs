//! Timepix3 vs Medipix3 identification (C `detector_family.cpp`).

/// C `DetectorFamily`; the discriminant is what `TPX3_DETECTOR_FAMILY` holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DetectorFamily {
    #[default]
    Unknown = 0,
    Tpx3 = 1,
    Mpx3 = 2,
}

/// C `DetectorCapabilities`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Capabilities {
    pub supports_tdc: bool,
    pub supports_tof_histogram: bool,
    pub supports_dual_preview: bool,
    pub supports_image_thresholds: bool,
    pub preview_layer_count: i32,
    pub bpc_bytes_per_pel: i32,
    pub bpc_threshold_slices: i32,
}

impl DetectorFamily {
    /// C `detectDetectorFamily`: the chip type wins, then `MpxType`, then the
    /// first chipboard ID's leading digit.
    pub fn detect(mpx_type: i32, chip_type: &str, chipboard_id: &str) -> Self {
        match chip_type.to_ascii_uppercase().as_str() {
            "MPX3" => return Self::Mpx3,
            "TPX3" => return Self::Tpx3,
            _ => {}
        }
        match mpx_type {
            5 => return Self::Mpx3,
            6 => return Self::Tpx3,
            _ => {}
        }
        match chipboard_id.as_bytes().first() {
            Some(b'5') => Self::Mpx3,
            Some(b'4') => Self::Tpx3,
            _ => Self::Unknown,
        }
    }

    /// C `capabilitiesForFamily`.
    pub fn capabilities(self) -> Capabilities {
        let none = Capabilities {
            supports_tdc: false,
            supports_tof_histogram: false,
            supports_dual_preview: false,
            supports_image_thresholds: false,
            preview_layer_count: 0,
            bpc_bytes_per_pel: 1,
            bpc_threshold_slices: 1,
        };
        match self {
            Self::Mpx3 => Capabilities {
                supports_dual_preview: true,
                supports_image_thresholds: true,
                preview_layer_count: 2,
                bpc_bytes_per_pel: 2,
                ..none
            },
            Self::Tpx3 => Capabilities {
                supports_tdc: true,
                supports_tof_histogram: true,
                supports_dual_preview: true,
                preview_layer_count: 2,
                ..none
            },
            Self::Unknown => none,
        }
    }

    /// C `detectorFamilyName`.
    pub fn name(self) -> &'static str {
        match self {
            Self::Tpx3 => "TPX3",
            Self::Mpx3 => "MPX3",
            Self::Unknown => "UNKNOWN",
        }
    }

    pub fn from_i32(v: i32) -> Self {
        match v {
            1 => Self::Tpx3,
            2 => Self::Mpx3,
            _ => Self::Unknown,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_chip_type_wins_over_the_mpx_type_and_the_chipboard() {
        assert_eq!(DetectorFamily::detect(6, "mpx3", "4"), DetectorFamily::Mpx3);
        assert_eq!(DetectorFamily::detect(5, "Tpx3", "5"), DetectorFamily::Tpx3);
    }

    #[test]
    fn the_mpx_type_wins_over_the_chipboard() {
        assert_eq!(DetectorFamily::detect(5, "", "4"), DetectorFamily::Mpx3);
        assert_eq!(DetectorFamily::detect(6, "", "5"), DetectorFamily::Tpx3);
    }

    #[test]
    fn the_chipboard_digit_decides_last() {
        assert_eq!(DetectorFamily::detect(0, "", "5x"), DetectorFamily::Mpx3);
        assert_eq!(DetectorFamily::detect(0, "", "41"), DetectorFamily::Tpx3);
        assert_eq!(DetectorFamily::detect(0, "", ""), DetectorFamily::Unknown);
        assert_eq!(DetectorFamily::detect(0, "X", "9"), DetectorFamily::Unknown);
    }

    #[test]
    fn only_mpx3_has_image_thresholds_and_two_byte_pels() {
        let m = DetectorFamily::Mpx3.capabilities();
        assert!(m.supports_image_thresholds && !m.supports_tdc && m.bpc_bytes_per_pel == 2);
        let t = DetectorFamily::Tpx3.capabilities();
        assert!(t.supports_tdc && t.supports_tof_histogram && t.bpc_bytes_per_pel == 1);
        let u = DetectorFamily::Unknown.capabilities();
        assert!(!u.supports_dual_preview && u.preview_layer_count == 0);
    }
}
