use uldaq_sys::*;

use crate::device::DaqDevice;
use crate::error::{self, Result};

/// In/out pulse timing for [`DaqDevice::pulse_out_start`]: the driver may
/// adjust all three to the nearest achievable values (C `ulTmrPulseOutStart`
/// double pointers).
#[derive(Clone, Copy, Debug)]
pub struct PulseTiming {
    pub frequency: f64,
    pub duty_cycle: f64,
    pub initial_delay: f64,
}

impl DaqDevice {
    /// Start a pulse output on the given timer.
    ///
    /// `timing` is in/out: the driver may adjust it to the nearest achievable
    /// values.
    pub fn pulse_out_start(
        &self,
        timer: i32,
        timing: &mut PulseTiming,
        pulse_count: u64,
        idle_state: i32,
        options: i32,
    ) -> Result<()> {
        error::check(unsafe {
            ulTmrPulseOutStart(
                self.handle(),
                timer,
                &mut timing.frequency,
                &mut timing.duty_cycle,
                pulse_count,
                &mut timing.initial_delay,
                idle_state,
                options,
            )
        })
    }

    /// Stop a pulse output on the given timer.
    pub fn pulse_out_stop(&self, timer: i32) -> Result<()> {
        error::check(unsafe { ulTmrPulseOutStop(self.handle(), timer) })
    }
}
