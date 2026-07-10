//! Enumerations from `simDetector.h`.

/// `SimModes_t` (simDetector.h:83-88).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum SimMode {
    LinearRamp = 0,
    Peaks = 1,
    Sine = 2,
    OffsetNoise = 3,
}

impl SimMode {
    /// Map the `SIM_MODE` parameter value. C `switch(simMode)` silently does
    /// nothing for out-of-range values, which leaves the raw buffer as the
    /// background/zero fill — the same result as `OffsetNoise`.
    pub fn from_i32(v: i32) -> Self {
        match v {
            0 => Self::LinearRamp,
            1 => Self::Peaks,
            2 => Self::Sine,
            _ => Self::OffsetNoise,
        }
    }
}

/// `SimSineOperation_t` (simDetector.h:90-93).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum SineOperation {
    Add = 0,
    Multiply = 1,
}

impl SineOperation {
    pub fn from_i32(v: i32) -> Self {
        match v {
            0 => Self::Add,
            _ => Self::Multiply,
        }
    }
}

/// Commands sent to the simulation task, one per C `epicsEvent`.
///
/// `simDetector` uses two independent binary semaphores (`startEventId_`,
/// `stopEventId_`); the task waits on start when idle and does a timed wait on
/// stop during the exposure and acquire-period delays. Each is modelled by its
/// own capacity-1 channel so a signal on one never satisfies a wait on the
/// other.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Signal;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_mode_from_i32() {
        assert_eq!(SimMode::from_i32(0), SimMode::LinearRamp);
        assert_eq!(SimMode::from_i32(1), SimMode::Peaks);
        assert_eq!(SimMode::from_i32(2), SimMode::Sine);
        assert_eq!(SimMode::from_i32(3), SimMode::OffsetNoise);
        assert_eq!(SimMode::from_i32(99), SimMode::OffsetNoise);
    }

    #[test]
    fn sine_operation_from_i32() {
        assert_eq!(SineOperation::from_i32(0), SineOperation::Add);
        assert_eq!(SineOperation::from_i32(1), SineOperation::Multiply);
    }
}
