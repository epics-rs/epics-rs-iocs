use meascomp::device::DaqDevice;
use meascomp::error::Result;
use meascomp::timer::PulseTiming;
use uldaq_sys::*;

/// The clock libuldaq runs the USB-CTR timers from (UsbCtrx.cpp:21).
const CLOCK_FREQUENCY: f64 = 96e6;

/// Frequency, duty-cycle and delay bounds C `startPulseGenerator` clamps to
/// (drvUSBCTR.cpp:87-90), except the delay: C's 67.11 s is 2^32 ticks of a
/// 64 MHz clock, but the delay goes to the timer as a u32 count of this one
/// (TmrUsb1208hs.cpp:71,95) and libuldaq rejects anything longer
/// (TmrDevice.cpp:72-75).
pub const MIN_FREQUENCY: f64 = 0.023;
pub const MAX_FREQUENCY: f64 = 48e6;
pub const MIN_DELAY: f64 = 0.0;
pub const MAX_DELAY: f64 = u32::MAX as f64 / CLOCK_FREQUENCY;

/// The timing C hands to `ulTmrPulseOutStart`: the requested period, duty
/// cycle and delay pulled into what the timer can do (drvUSBCTR.cpp:465-472),
/// so an out-of-range request runs at the nearest limit instead of being
/// rejected by libuldaq. A zero period is an infinite frequency, i.e. the
/// maximum; a negative one the minimum.
pub fn clamp_timing(period: f64, duty_cycle: f64, delay: f64) -> PulseTiming {
    let frequency = (1.0 / period).clamp(MIN_FREQUENCY, MAX_FREQUENCY);
    let mut duty_cycle = duty_cycle;
    if duty_cycle <= 0.0 {
        duty_cycle = 0.0001;
    }
    if duty_cycle >= 1.0 {
        duty_cycle = 0.9999;
    }
    let initial_delay = delay.clamp(MIN_DELAY, MAX_DELAY);
    PulseTiming {
        frequency,
        duty_cycle,
        initial_delay,
    }
}

/// Start a pulse generator on the given timer channel.
/// Returns (actual_period, actual_duty_cycle, actual_delay).
pub fn start(
    device: &DaqDevice,
    timer: i32,
    period: f64,
    duty_cycle: f64,
    delay: f64,
    count: u64,
    idle_state: i32,
) -> std::result::Result<(f64, f64, f64), meascomp::error::MeasCompError> {
    let mut timing = clamp_timing(period, duty_cycle, delay);
    let idle = if idle_state != 0 {
        TMRIS_HIGH
    } else {
        TMRIS_LOW
    };

    device.pulse_out_start(timer, &mut timing, count, idle, PO_DEFAULT)?;

    // C: period = 1./frequency from what the device actually runs.
    let actual_period = 1.0 / timing.frequency;

    Ok((actual_period, timing.duty_cycle, timing.initial_delay))
}

/// Stop a pulse generator.
pub fn stop(device: &DaqDevice, timer: i32) -> Result<()> {
    device.pulse_out_stop(timer)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_period_in_range_passes_through() {
        let t = clamp_timing(0.001, 0.25, 0.5);
        assert_eq!(t.frequency, 1000.0);
        assert_eq!(t.duty_cycle, 0.25);
        assert_eq!(t.initial_delay, 0.5);
    }

    #[test]
    fn the_frequency_is_pulled_into_the_timer_range() {
        assert_eq!(clamp_timing(100.0, 0.5, 0.0).frequency, MIN_FREQUENCY);
        assert_eq!(clamp_timing(1e-9, 0.5, 0.0).frequency, MAX_FREQUENCY);
        // 1/0 is +inf and 1/-x is negative, as in C.
        assert_eq!(clamp_timing(0.0, 0.5, 0.0).frequency, MAX_FREQUENCY);
        assert_eq!(clamp_timing(-1.0, 0.5, 0.0).frequency, MIN_FREQUENCY);
    }

    #[test]
    fn the_duty_cycle_stays_strictly_inside_zero_and_one() {
        assert_eq!(clamp_timing(0.001, 0.0, 0.0).duty_cycle, 0.0001);
        assert_eq!(clamp_timing(0.001, -1.0, 0.0).duty_cycle, 0.0001);
        assert_eq!(clamp_timing(0.001, 1.0, 0.0).duty_cycle, 0.9999);
        assert_eq!(clamp_timing(0.001, 1.5, 0.0).duty_cycle, 0.9999);
    }

    #[test]
    fn the_delay_is_pulled_into_its_range() {
        assert_eq!(clamp_timing(0.001, 0.5, -5.0).initial_delay, MIN_DELAY);
        assert_eq!(clamp_timing(0.001, 0.5, 100.0).initial_delay, MAX_DELAY);
    }

    #[test]
    fn the_longest_delay_fits_the_timer_count() {
        // libuldaq truncates delay * clock to an integer tick count and
        // refuses one above UINT_MAX (TmrDevice.cpp:72-75).
        assert!((MAX_DELAY * CLOCK_FREQUENCY) as u64 <= u64::from(u32::MAX));
        assert!(((MAX_DELAY + 1e-6) * CLOCK_FREQUENCY) as u64 > u64::from(u32::MAX));
    }
}
