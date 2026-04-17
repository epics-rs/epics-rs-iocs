use uldaq_sys::*;

use crate::device::DaqDevice;
use crate::error::{self, Result};

impl DaqDevice {
    /// Start a pulse output on the given timer.
    ///
    /// `frequency`, `duty_cycle`, and `initial_delay` are in/out: the driver
    /// may adjust them to the nearest achievable values.
    pub fn pulse_out_start(
        &self,
        timer: i32,
        frequency: &mut f64,
        duty_cycle: &mut f64,
        pulse_count: u64,
        initial_delay: &mut f64,
        idle_state: i32,
        options: i32,
    ) -> Result<()> {
        error::check(unsafe {
            ulTmrPulseOutStart(
                self.handle(),
                timer,
                frequency,
                duty_cycle,
                pulse_count,
                initial_delay,
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
