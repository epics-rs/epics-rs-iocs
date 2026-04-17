use meascomp::device::DaqDevice;
use meascomp::error::Result;
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
    let mut freq = if period > 0.0 { 1.0 / period } else { 1000.0 };
    let mut duty = duty_cycle;
    let mut initial_delay = delay;
    let idle = if idle_state != 0 { TMRIS_HIGH } else { TMRIS_LOW };

    device.pulse_out_start(
        timer,
        &mut freq,
        &mut duty,
        count,
        &mut initial_delay,
        idle,
        PO_DEFAULT,
    )?;

    let actual_period = if freq > 0.0 { 1.0 / freq } else { period };

    log::info!(
        "PulseGen {timer}: freq={freq:.1} Hz, duty={duty:.3}, delay={initial_delay:.6} s, count={count}"
    );
    Ok((actual_period, duty, initial_delay))
}

/// Stop a pulse generator.
pub fn stop(device: &DaqDevice, timer: i32) -> Result<()> {
    device.pulse_out_stop(timer)
}
