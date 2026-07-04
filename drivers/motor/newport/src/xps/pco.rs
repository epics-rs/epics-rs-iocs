//! XPS position-compare output (PCO), driver-private.
//!
//! Port of C `XPSAxis::setPositionCompare` (XPSAxis.cpp). Deliberately NOT
//! wired through the motor framework: the C change adding a base-class PCO API
//! (`05b25c1d`, motor PR #248) is an open, unmerged PR, so there is no upstream
//! base interface to mirror — the feature is exposed by the
//! `XPSPositionCompare` iocsh command instead (see `ioc.rs`).

use super::rpc::{XpsResult, XpsSocket};

/// XPS position-compare modes (C `XPSPositionCompareMode_t`).
pub const XPS_PCO_MODE_DISABLE: i32 = 0;
pub const XPS_PCO_MODE_PULSE: i32 = 1;
pub const XPS_PCO_MODE_AQUADB_WINDOWED: i32 = 2;
pub const XPS_PCO_MODE_AQUADB_ALWAYS: i32 = 3;

/// Default pulse width (µs) when the iocsh arg is omitted: the smallest entry
/// of C's pulse-width table `{0.2, 1.0, 2.5, 10.0}`, the XPS factory default.
pub const XPS_PCO_DEFAULT_PULSE_WIDTH: f64 = 0.2;

/// Default encoder settling time (µs) when the iocsh arg is omitted: the
/// smallest entry of C's settling table `{0.075, 1.0, 4.0, 12.0}`.
pub const XPS_PCO_DEFAULT_SETTLING_TIME: f64 = 0.075;

/// Position-compare parameters, in device (positioner) units.
#[derive(Clone, Copy, Debug, Default)]
pub struct PcoParams {
    /// Mode selector (see `XPS_PCO_MODE_*`).
    pub mode: i32,
    pub min_position: f64,
    pub max_position: f64,
    /// Compare step for Pulse mode (ignored by the AquadB modes).
    pub position_step: f64,
    /// Pulse width (µs). Passed straight through — DEVIATION from C, which
    /// selects one of `{0.2, 1.0, 2.5, 10.0}` via a table index.
    pub pulse_width_us: f64,
    /// Encoder settling time (µs). Same raw-µs-vs-index DEVIATION as above.
    pub settling_time_us: f64,
}

/// Apply a position-compare configuration to `positioner` (C
/// `setPositionCompare`): always disable first, stage the pulse parameters,
/// then dispatch on the mode. Mode Disable — and any unrecognised mode,
/// matching C's silent switch default — leaves the output off with the pulse
/// parameters staged.
pub fn set_position_compare(
    sock: &XpsSocket,
    positioner: &str,
    params: &PcoParams,
) -> XpsResult<()> {
    sock.positioner_position_compare_disable(positioner)?;
    sock.positioner_position_compare_pulse_parameters_set(
        positioner,
        params.pulse_width_us,
        params.settling_time_us,
    )?;
    match params.mode {
        XPS_PCO_MODE_PULSE => {
            sock.positioner_position_compare_set(
                positioner,
                params.min_position,
                params.max_position,
                params.position_step,
            )?;
            sock.positioner_position_compare_enable(positioner)?;
        }
        XPS_PCO_MODE_AQUADB_WINDOWED => {
            sock.positioner_position_compare_aquadb_windowed_set(
                positioner,
                params.min_position,
                params.max_position,
            )?;
            sock.positioner_position_compare_enable(positioner)?;
        }
        XPS_PCO_MODE_AQUADB_ALWAYS => {
            sock.positioner_position_compare_aquadb_always_enable(positioner)?;
        }
        XPS_PCO_MODE_DISABLE => {}
        _ => {}
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pco_mode_constants_match_c_enum() {
        // C XPSPositionCompareMode_t ordering (XPSController.h).
        assert_eq!(XPS_PCO_MODE_DISABLE, 0);
        assert_eq!(XPS_PCO_MODE_PULSE, 1);
        assert_eq!(XPS_PCO_MODE_AQUADB_WINDOWED, 2);
        assert_eq!(XPS_PCO_MODE_AQUADB_ALWAYS, 3);
    }

    #[test]
    fn pco_defaults_are_smallest_c_table_entries() {
        assert_eq!(XPS_PCO_DEFAULT_PULSE_WIDTH, 0.2);
        assert_eq!(XPS_PCO_DEFAULT_SETTLING_TIME, 0.075);
    }
}
