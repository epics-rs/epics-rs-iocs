//! What the SMC hydra, corvus and Taurus axes share: the C members their
//! `report` prints.

use std::sync::{Mutex, MutexGuard};

use motor_common::AxisReport;

/// Switch-config bit that disables (masks) a limit switch.
const SWITCH_IGNORE: i32 = 0x2;

/// The C members an SMC axis's `report` prints, which it reads while a
/// poller may hold the driver: their one home, behind their own lock. A
/// member the driver's C class does not have is `None`, and its line is not
/// printed.
pub struct SmcMembers {
    /// 0-based axis index; commands use `axis_no + 1`.
    pub(crate) axis_no: usize,
    /// Hydra's `motorForm_` (`getmotor`): 0 stepper, 1 linear/torque.
    pub(crate) motor_form: Option<i32>,
    pub(crate) pitch: Option<f64>,
    pub(crate) pole_pairs: Option<i32>,
    pub(crate) cl_period: f64,
    pub(crate) state: Mutex<SmcState>,
}

/// The members that change after construction.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmcState {
    /// C `axisRes_`: controller units per record step.
    pub(crate) axis_res: f64,
    /// C `lowLimitConfig_`/`highLimitConfig_`, the last `getsw` the poll
    /// read.
    pub(crate) limit_config: (i32, i32),
    /// C `negTravelLimit_`/`posTravelLimit_`.
    pub(crate) travel_limits: (f64, f64),
}

impl SmcMembers {
    pub(crate) fn state(&self) -> MutexGuard<'_, SmcState> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn axis_res(&self) -> f64 {
        self.state().axis_res
    }

    /// The poll's `getsw` reply into C's members, and the
    /// `(ignoreLowLimit, ignoreHighLimit)` C derives from them.
    pub(crate) fn read_limit_config(&self, reply: &str) -> (bool, bool) {
        let mut state = self.state();
        state.limit_config = limit_config(reply, state.limit_config);
        let (low, high) = state.limit_config;
        (low & SWITCH_IGNORE != 0, high & SWITCH_IGNORE != 0)
    }
}

impl AxisReport for SmcMembers {
    /// C `SMChydraAxis::report` (SMChydraAxis.cpp:76-94), and corvus's and
    /// Taurus's, which print the same lines less the members they lack.
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level <= 0 {
            return;
        }
        let SmcState {
            axis_res,
            limit_config: (low, high),
            travel_limits: (neg, pos),
        } = *self.state();
        let _ = writeln!(out, "  axis {}", self.axis_no);
        if let Some(motor_form) = self.motor_form {
            let _ = writeln!(out, "    motorForm {motor_form}");
        }
        if let Some(pitch) = self.pitch {
            let _ = writeln!(out, "    pitch {pitch:.6}");
        }
        if let Some(pole_pairs) = self.pole_pairs {
            let _ = writeln!(out, "    polePairs {pole_pairs}");
        }
        let _ = write!(
            out,
            "    clPeriod {:.6}\n    axisRes {axis_res:.6}\n    lowLimitConfig {low}\n    \
             highLimitConfig {high}\n    posTravelLimit {pos:.6}\n    \
             negTravelLimit {neg:.6}\n",
            self.cl_period
        );
    }
}

/// `sscanf(reply, "%lf %lf", &negTravelLimit_, &posTravelLimit_)`. A value
/// sscanf does not convert is 0 here; C leaves the member uninitialized, a
/// defect not copied.
pub(crate) fn travel_limits(reply: &str) -> (f64, f64) {
    let mut values = reply
        .split_whitespace()
        .map_while(|tok| tok.parse::<f64>().ok());
    let neg = values.next().unwrap_or(0.0);
    (neg, values.next().unwrap_or(0.0))
}

/// `sscanf(reply, "%i %i", &lowLimitConfig_, &highLimitConfig_)`: a value
/// sscanf does not convert keeps the last.
fn limit_config(reply: &str, (low, high): (i32, i32)) -> (i32, i32) {
    let mut values = reply
        .split_whitespace()
        .map_while(|tok| tok.parse::<i32>().ok());
    let low = values.next().unwrap_or(low);
    (low, values.next().unwrap_or(high))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn travel_limits_scan_as_c_sscanf() {
        assert_eq!(travel_limits("-100.0 100.0"), (-100.0, 100.0));
        assert_eq!(travel_limits("-5 x"), (-5.0, 0.0));
        assert_eq!(travel_limits(""), (0.0, 0.0));
    }

    #[test]
    fn limit_config_scans_as_c_sscanf() {
        assert_eq!(limit_config("2 3", (0, 0)), (2, 3));
        assert_eq!(limit_config("2 x", (0, 1)), (2, 1));
        assert_eq!(limit_config("", (4, 5)), (4, 5));
    }

    #[test]
    fn the_switch_ignore_mask_is_bit_1() {
        let members = SmcMembers {
            axis_no: 0,
            motor_form: None,
            pitch: None,
            pole_pairs: None,
            cl_period: 0.0,
            state: Mutex::default(),
        };
        assert_eq!(members.read_limit_config("1 0"), (false, false));
        assert_eq!(members.read_limit_config("2 3"), (true, true));
        assert_eq!(members.read_limit_config("0 x"), (false, true));
    }
}
