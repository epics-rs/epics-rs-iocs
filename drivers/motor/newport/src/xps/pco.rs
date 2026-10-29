//! XPS position compare output: the `XPS_POSITION_COMPARE_*` parameters a
//! port write applies to the positioner and reads back
//! (XPSController.cpp:262-266, :316-320; XPSAxis.cpp:939-1101).

/// `XPSPositionCompareMode_t` (XPSController.h).
pub const MODE_DISABLE: i32 = 0;
pub const MODE_PULSE: i32 = 1;
pub const MODE_AQUADB_WINDOWED: i32 = 2;
pub const MODE_AQUADB_ALWAYS: i32 = 3;

/// `positionComparePulseWidths` and `positionCompareSettlingTimes`
/// (XPSController.h:28-29), in µs, which the parameters index.
pub const PULSE_WIDTHS: [f64; 4] = [0.2, 1.0, 2.5, 10.0];
pub const SETTLING_TIMES: [f64; 4] = [0.075, 1.0, 4.0, 12.0];

/// The `XPS_POSITION_COMPARE_*` parameters (XPSController.cpp:161-166).
#[derive(Clone, Copy, Debug)]
pub struct PcoParams {
    pub mode: usize,
    pub min_position: usize,
    pub max_position: usize,
    pub step_size: usize,
    pub pulse_width: usize,
    pub settling_time: usize,
}

/// The motor record's `MRES`, `OFF` and `DIR` at the axis.
#[derive(Clone, Copy, Debug, Default)]
pub struct RecordUnits {
    pub resolution: f64,
    pub offset: f64,
    pub direction: i32,
}

impl RecordUnits {
    fn signed_resolution(&self) -> f64 {
        if self.direction != 0 {
            -self.resolution
        } else {
            self.resolution
        }
    }

    /// C's record-to-XPS divisor, 1 for a zero resolution.
    fn xps_divisor(&self) -> f64 {
        let r = self.signed_resolution();
        if r == 0.0 { 1.0 } else { r }
    }

    /// C `motorRecPositionToXPSPosition` (XPSAxis.cpp:882-895).
    pub fn position_to_xps(&self, position: f64, step_size: f64) -> f64 {
        (position - self.offset) / self.xps_divisor() * step_size
    }

    /// C `motorRecStepToXPSStep` (XPSAxis.cpp:911-923).
    pub fn step_to_xps(&self, step: f64, step_size: f64) -> f64 {
        step / self.xps_divisor() * step_size
    }

    /// C `XPSStepToMotorRecStep` (XPSAxis.cpp:925-937). C also writes the 1
    /// for a zero `stepSize_` back into the axis, rescaling its later
    /// moves; the axis's step size is left as it is.
    pub fn step_from_xps(&self, step: f64, step_size: f64) -> f64 {
        let step_size = if step_size == 0.0 { 1.0 } else { step_size };
        step * self.signed_resolution() / step_size
    }
}

/// What C `setPositionCompare` reads from the parameters at the axis.
#[derive(Clone, Copy, Debug, Default)]
pub struct PcoSettings {
    pub mode: i32,
    /// Motor record units.
    pub min_position: f64,
    pub max_position: f64,
    pub step_size: f64,
    /// Indices into [`PULSE_WIDTHS`] and [`SETTLING_TIMES`].
    pub pulse_width: i32,
    pub settling_time: i32,
    pub units: RecordUnits,
}

/// `table[index]`, None past either end.
pub fn entry(table: &[f64; 4], index: i32) -> Option<f64> {
    usize::try_from(index)
        .ok()
        .and_then(|i| table.get(i))
        .copied()
}

/// C `getPositionCompare`'s index search: the entry within 0.001 of
/// `value`, else the table's length.
pub fn index_of(table: &[f64; 4], value: f64) -> i32 {
    table
        .iter()
        .position(|t| (value - t).abs() < 0.001)
        .unwrap_or(table.len()) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_units_convert_as_c() {
        let units = RecordUnits {
            resolution: 0.5,
            offset: 1.0,
            direction: 0,
        };
        assert_eq!(units.position_to_xps(3.0, 0.25), 1.0);
        assert_eq!(units.step_to_xps(0.5, 0.25), 0.25);
        assert_eq!(units.step_from_xps(0.25, 0.25), 0.5);
        let reversed = RecordUnits {
            direction: 1,
            ..units
        };
        assert_eq!(reversed.position_to_xps(3.0, 0.25), -1.0);
        assert_eq!(reversed.step_from_xps(0.25, 0.25), -0.5);
        let unset = RecordUnits::default();
        assert_eq!(unset.position_to_xps(2.0, 0.25), 0.5);
        assert_eq!(unset.step_from_xps(0.25, 0.0), 0.0);
    }

    #[test]
    fn table_lookups_stop_at_the_end() {
        assert_eq!(entry(&PULSE_WIDTHS, 3), Some(10.0));
        assert_eq!(entry(&PULSE_WIDTHS, 4), None);
        assert_eq!(entry(&SETTLING_TIMES, -1), None);
        assert_eq!(index_of(&SETTLING_TIMES, 4.0005), 2);
        assert_eq!(index_of(&PULSE_WIDTHS, 5.0), 4);
    }
}
