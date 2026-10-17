//! The `motorAxis` members both drivers' `motorAxisReportAxis` print
//! (drvEnsembleAsyn.cc:218-228, drvA3200Asyn.cc:199-209).

use std::sync::atomic::{AtomicU32, Ordering};

use epics_libcom_rs::cfmt::CFloat;
use motor_common::AxisReport;

/// C reads these without the axis lock; only `axis_status` changes after
/// `Config`.
pub struct AxisMembers {
    /// The Ensemble's axis number, the A3200's axis name.
    name: String,
    /// C `stepSize`, signed.
    step_size: f64,
    max_digits: usize,
    /// C `homePreset`: the `HomeOffset` parameter, 0 when refused.
    home_preset: f64,
    /// C `axisStatus`: the poller's last axis status word.
    axis_status: AtomicU32,
}

impl AxisMembers {
    pub fn new(name: String, step_size: f64, max_digits: usize, home_preset: f64) -> Self {
        Self {
            name,
            step_size,
            max_digits,
            home_preset,
            axis_status: AtomicU32::new(0),
        }
    }

    pub fn set_axis_status(&self, axis_status: u32) {
        self.axis_status.store(axis_status, Ordering::Relaxed);
    }
}

impl AxisReport for AxisMembers {
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level > 0 {
            let _ = write!(
                out,
                "Axis {}\n   axisStatus:  0x{:x}\n   home preset: {:.6}\n   \
                 step size:   {:.6}\n   max digits:  {}\n",
                self.name,
                self.axis_status.load(Ordering::Relaxed),
                CFloat(self.home_preset),
                CFloat(self.step_size),
                self.max_digits
            );
        }
    }
}
