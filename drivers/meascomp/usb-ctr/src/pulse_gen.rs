use epics_libcom_rs::cfmt::CFloat;
use meascomp::device::DaqDevice;
use meascomp::error::Result;
use meascomp::timer::PulseTiming;
use uldaq_sys::*;

use epics_rs::asyn::trace::TraceMask;

use crate::trace::SelfTrace;

/// C drvUSBCTR's timer limits.
const MIN_FREQUENCY: f64 = 0.023;
const MAX_FREQUENCY: f64 = 48e6;
const MIN_DELAY: f64 = 0.0;
const MAX_DELAY: f64 = 67.11;

/// One timer's `PULSE_GEN_*` parameters.
pub struct PulseRequest {
    pub period: f64,
    pub duty_cycle: f64,
    pub delay: f64,
    pub count: u64,
    pub idle_state: i32,
}

/// Start a pulse generator on the given timer channel; C
/// `USBCTR::startPulseGenerator` (drvUSBCTR.cpp:451).
/// Returns (actual_period, actual_duty_cycle, actual_delay).
pub fn start(
    trace: &SelfTrace,
    device: &DaqDevice,
    timer: i32,
    request: &PulseRequest,
) -> std::result::Result<(f64, f64, f64), meascomp::error::MeasCompError> {
    let PulseRequest {
        period,
        duty_cycle,
        delay,
        count,
        idle_state,
    } = *request;
    // C clamps every request into the timer's range before starting it.
    let mut timing = PulseTiming {
        frequency: (1.0 / period).clamp(MIN_FREQUENCY, MAX_FREQUENCY),
        duty_cycle: if duty_cycle <= 0.0 {
            0.0001
        } else if duty_cycle >= 1.0 {
            0.9999
        } else {
            duty_cycle
        },
        initial_delay: delay.clamp(MIN_DELAY, MAX_DELAY),
    };
    let idle = if idle_state != 0 {
        TMRIS_HIGH
    } else {
        TMRIS_LOW
    };

    if let Err(e) = device.pulse_out_start(timer, &mut timing, count, idle, PO_DEFAULT) {
        trace.print(
            TraceMask::ERROR,
            format_args!(
                "USBCTR::startPulseGenerator error calling cbPulseOutStart timerNum={timer} frequency={:.6}, dutyCycle={:.6}, count={count}, delay={:.6}, idleState={idle_state}, status={}, error={}\n",
                CFloat(timing.frequency),
                CFloat(timing.duty_cycle),
                CFloat(timing.initial_delay),
                e.code,
                e.message()
            ),
        );
        return Err(e);
    }

    let actual_period = 1.0 / timing.frequency;

    trace.print(
        TraceMask::FLOW,
        format_args!(
            "USBCTR:startPulseGenerator: started pulse generator {timer} actual frequency={:.6}, actual period={actual_period:.6}, actual duty cycle={:.6}, actual delay={:.6}\n",
            CFloat(timing.frequency),
            CFloat(timing.duty_cycle),
            CFloat(timing.initial_delay),
            actual_period = CFloat(actual_period)
        ),
    );
    Ok((actual_period, timing.duty_cycle, timing.initial_delay))
}

/// Stop a pulse generator.
/// C `USBCTR::stopPulseGenerator` (drvUSBCTR.cpp:501).
pub fn stop(trace: &SelfTrace, device: &DaqDevice, timer: i32) -> Result<()> {
    device.pulse_out_stop(timer).inspect_err(|e| {
        trace.print(
            TraceMask::ERROR,
            format_args!(
                "USBCTR::stopPulseGenerator error calling cbPulseOutStop timerNum={timer}, status={}, error={}\n",
                e.code, e.message()
            ),
        );
    })
}
