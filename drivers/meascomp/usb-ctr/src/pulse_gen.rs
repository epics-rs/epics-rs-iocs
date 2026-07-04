use meascomp::device::DaqDevice;
use meascomp::error::Result;
use meascomp::timer::PulseTiming;
use uldaq_sys::*;

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
    let mut timing = PulseTiming {
        frequency: if period > 0.0 { 1.0 / period } else { 1000.0 },
        duty_cycle,
        initial_delay: delay,
    };
    let idle = if idle_state != 0 {
        TMRIS_HIGH
    } else {
        TMRIS_LOW
    };

    device.pulse_out_start(timer, &mut timing, count, idle, PO_DEFAULT)?;

    let actual_period = if timing.frequency > 0.0 {
        1.0 / timing.frequency
    } else {
        period
    };

    log::info!(
        "PulseGen {timer}: freq={:.1} Hz, duty={:.3}, delay={:.6} s, count={count}",
        timing.frequency,
        timing.duty_cycle,
        timing.initial_delay,
    );
    Ok((actual_period, timing.duty_cycle, timing.initial_delay))
}

/// Stop a pulse generator.
pub fn stop(device: &DaqDevice, timer: i32) -> Result<()> {
    device.pulse_out_stop(timer)
}
