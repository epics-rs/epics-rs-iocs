//! `simDetector::setShutter` (simDetector.cpp:490-502).

use epics_rs::ad_core::driver::ShutterMode;

/// What `setShutter(open)` must actually do for a given `ADShutterMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShutterOp {
    /// `ADShutterModeNone`: `ADDriver::setShutter` does nothing.
    Nothing,
    /// `ADShutterModeDetector`: the simulation "simulates a shutter by just
    /// changing the status readback" — write `ADShutterStatus`.
    DetectorStatus(i32),
    /// `ADShutterModeEPICS`: delegate to `ADDriver::setShutter`, which writes
    /// `ADShutterControlEPICS` and then sleeps `openDelay - closeDelay`.
    EpicsControl(i32),
}

/// Decide the shutter action. `open` is C's `ADShutterOpen`/`ADShutterClosed`.
pub fn shutter_op(mode: i32, open: bool) -> ShutterOp {
    let value = i32::from(open);
    match ShutterMode::from_i32(mode) {
        Some(ShutterMode::DetectorOnly) => ShutterOp::DetectorStatus(value),
        Some(ShutterMode::EpicsOnly) => ShutterOp::EpicsControl(value),
        // C's `else` branch covers ADShutterModeNone and any out-of-range value:
        // both reach `ADDriver::setShutter`, whose switch default is a no-op.
        Some(ShutterMode::None) | None => ShutterOp::Nothing,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detector_mode_writes_shutter_status() {
        assert_eq!(
            shutter_op(ShutterMode::DetectorOnly as i32, true),
            ShutterOp::DetectorStatus(1)
        );
        assert_eq!(
            shutter_op(ShutterMode::DetectorOnly as i32, false),
            ShutterOp::DetectorStatus(0)
        );
    }

    #[test]
    fn epics_mode_delegates_to_the_base_class() {
        assert_eq!(
            shutter_op(ShutterMode::EpicsOnly as i32, true),
            ShutterOp::EpicsControl(1)
        );
        assert_eq!(
            shutter_op(ShutterMode::EpicsOnly as i32, false),
            ShutterOp::EpicsControl(0)
        );
    }

    #[test]
    fn none_and_out_of_range_modes_do_nothing() {
        assert_eq!(
            shutter_op(ShutterMode::None as i32, true),
            ShutterOp::Nothing
        );
        assert_eq!(shutter_op(-1, true), ShutterOp::Nothing);
        assert_eq!(shutter_op(7, false), ShutterOp::Nothing);
    }
}
