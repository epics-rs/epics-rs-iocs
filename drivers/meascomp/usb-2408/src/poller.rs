use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::trace::TraceMask;

use meascomp::device::DaqDevice;
use meascomp::error::MeasCompError;

use crate::params::*;
use crate::trace::{self, DRIVER};
use crate::wave_dig::{self, WaveDigState};
use crate::wave_gen::{self, WaveGenState};

/// Shared state between driver and poller.
pub struct PollerState {
    pub wave_dig: WaveDigState,
    pub wave_gen: WaveGenState,
}

/// Start the polling thread for USB-2408-2AO.
pub fn start_poller(
    handle: PortHandle,
    params: MultiFunctionParams,
    device: Arc<Mutex<DaqDevice>>,
    state: Arc<Mutex<PollerState>>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("usb-2408-poller".into())
        .spawn(move || poller_loop(handle, params, device, state))
        .expect("failed to spawn USB-2408 poller thread")
}

/// Per-cycle snapshot collected while the device mutex is held.
/// All `handle.*_blocking` calls must happen OUTSIDE the device lock to
/// avoid deadlock with the asyn actor (which takes the device mutex for
/// incoming writes).
#[derive(Default)]
struct PollSnapshot {
    digital_input: Option<u64>,
    counters: [Option<i64>; MAX_COUNTERS],
    // wave_gen / wave_dig status
    wave_gen_running: bool,
    wave_gen_current_point: usize,
    /// The generation of a generator scan that went idle this cycle.
    wave_gen_ended: Option<u64>,
    wave_dig_running: bool,
    wave_dig_current_point: usize,
    /// The generation of a digitizer scan that went idle this cycle.
    wave_dig_ended: Option<u64>,
    // Analog inputs (only populated when wave_dig is not running)
    ai_raw: [Option<i32>; MAX_ANALOG_IN],
    ai_temp: [Option<f64>; MAX_ANALOG_IN],
    /// The C lines of the first call that failed this cycle. C prints them
    /// only when the previous cycle did not fail, and only that call's,
    /// since it abandons the cycle there (`goto error`).
    failure: Vec<String>,
    /// The first ulTIn failure's line. C prints every one as it happens,
    /// every cycle, so it is printed on the spot; it fails the cycle all
    /// the same.
    tin_failure: Option<String>,
}

impl PollSnapshot {
    /// Keep a failed call's C lines, if it is the cycle's first failure.
    fn failed(&mut self, lines: Vec<String>) {
        if self.failure.is_empty() {
            self.failure = lines;
        }
    }
}

/// C `reportError(err, "pollerThread", message)` for a failed call
/// (drvMultiFunction.cpp:1315-1318).
fn error_line(message: &str, e: &MeasCompError) -> String {
    format!(
        "{DRIVER}::pollerThread Error: {message}, err={} {}",
        e.code, e.message
    )
}

fn poller_loop(
    handle: PortHandle,
    params: MultiFunctionParams,
    device: Arc<Mutex<DaqDevice>>,
    state: Arc<Mutex<PollerState>>,
) {
    let mut prev_digital_input: u64 = 0;
    let mut force_callback = true;
    let mut cycle_start = Instant::now();
    // C prevStatus: whether the previous cycle failed. A failure is
    // reported once when it starts and once when it clears, not every poll.
    let mut prev_failed = false;

    loop {
        // C publishes the time from the previous loop top to this one, so
        // POLL_TIME_MS is the real cycle time, sleep included.
        let now = Instant::now();
        publish(
            &handle,
            0,
            vec![ParamSetValue::new(
                params.poll_time_ms,
                0,
                ParamValue::Float64(now.duration_since(cycle_start).as_secs_f64() * 1000.0),
            )],
        );
        cycle_start = now;

        // ---- Phase 1: read config params (no device lock) ----
        let input_mode = handle
            .read_int32_blocking(params.analog_in_mode, 0)
            .unwrap_or(uldaq_sys::AI_DIFFERENTIAL);
        let mut in_types = [0i32; MAX_ANALOG_IN];
        let mut in_ranges = [uldaq_sys::BIP10VOLTS; MAX_ANALOG_IN];
        let mut tc_scales = [uldaq_sys::TS_CELSIUS; MAX_ANALOG_IN];
        for ch in 0..MAX_ANALOG_IN {
            in_types[ch] = handle
                .read_int32_blocking(params.analog_in_type, ch as i32)
                .unwrap_or(0);
            in_ranges[ch] = handle
                .read_int32_blocking(params.analog_in_range, ch as i32)
                .unwrap_or(uldaq_sys::BIP10VOLTS);
            tc_scales[ch] = handle
                .read_int32_blocking(params.temperature_scale, ch as i32)
                .unwrap_or(uldaq_sys::TS_CELSIUS);
        }

        // ---- Phase 2: uldaq reads (device lock held) ----
        let snapshot = {
            let mut snap = PollSnapshot::default();
            if let Ok(dev) = device.lock() {
                match dev.digital_in(uldaq_sys::AUXPORT) {
                    Ok(data) => snap.digital_input = Some(data),
                    // C adds the index of the port that failed
                    // (drvMultiFunction.cpp:2620-2623).
                    Err(e) => snap.failed(vec![
                        error_line("Calling DIn", &e),
                        "portNumber=0".to_string(),
                    ]),
                }

                for counter in 0..MAX_COUNTERS {
                    match dev.counter_in(counter as i32) {
                        Ok(value) => snap.counters[counter] = Some(value as i64),
                        Err(e) => snap.failed(vec![error_line("Calling CIn", &e)]),
                    }
                }

                if let Ok(mut st) = state.lock() {
                    if st.wave_gen.running {
                        let (p, ended) = wave_gen::read_wave_gen(&dev, &mut st.wave_gen);
                        if p.code == uldaq_sys::ERR_NO_ERROR {
                            trace::print(
                                &handle,
                                TraceMask::IO_DRIVER,
                                format_args!(
                                    "{DRIVER}::pollerThread waveform generator status, \
                                     aoStatus={}, aoCount={}, aoIndex={}",
                                    p.status, p.total_count, p.index
                                ),
                            );
                        } else {
                            snap.failed(vec![error_line(
                                "Calling AOutScanStatus",
                                &MeasCompError::from_code(p.code),
                            )]);
                        }
                        if ended {
                            snap.wave_gen_ended = Some(st.wave_gen.generation);
                        }
                        snap.wave_gen_current_point = st.wave_gen.current_point;
                    }
                    snap.wave_gen_running = st.wave_gen.running;

                    if st.wave_dig.running {
                        let (p, ended) = wave_dig::read_wave_dig(&dev, &mut st.wave_dig);
                        if p.code == uldaq_sys::ERR_NO_ERROR {
                            trace::print(
                                &handle,
                                TraceMask::IO_DRIVER,
                                format_args!(
                                    "{DRIVER}::pollerThread waveform digitizer status, \
                                     aiStatus={}, aiCount={}, aiIndex={}",
                                    p.status, p.total_count, p.index
                                ),
                            );
                        } else {
                            snap.failed(vec![error_line(
                                "Calling AInScanStatus",
                                &MeasCompError::from_code(p.code),
                            )]);
                        }
                        if ended {
                            snap.wave_dig_ended = Some(st.wave_dig.generation);
                        }
                        snap.wave_dig_current_point = st.wave_dig.current_point;
                    }
                    snap.wave_dig_running = st.wave_dig.running;

                    if !st.wave_dig.running {
                        for ch in 0..MAX_ANALOG_IN {
                            let ch_i = ch as i32;
                            // One call per channel, chosen by its configured
                            // type: ulAIn rejects a thermocouple channel with
                            // ERR_BAD_RANGE and ulTIn rejects a voltage one.
                            // C drvMultiFunction skips each the same way.
                            if in_types[ch] == 0 {
                                // C reads raw counts once; the record's LINR
                                // conversion over the driver's ADC bounds
                                // turns them into volts.
                                match dev.analog_in(
                                    ch_i,
                                    input_mode,
                                    in_ranges[ch],
                                    uldaq_sys::AIN_FF_NOSCALEDATA,
                                ) {
                                    Ok(raw) => snap.ai_raw[ch] = Some(raw as i32),
                                    // C's USB-2408 branch fails the cycle on
                                    // it without a line of its own; the one C
                                    // prints for the USB-TEMP-AI's ulAIn
                                    // (drvMultiFunction.cpp:2754) says which.
                                    Err(e) => snap.failed(vec![error_line("Calling AIn", &e)]),
                                }
                            } else {
                                // An open or broken thermocouple is expected,
                                // not an error: ulTIn reports it as -9999.
                                let temp = match dev.temperature_in(
                                    ch_i,
                                    tc_scales[ch],
                                    uldaq_sys::TIN_FF_DEFAULT,
                                ) {
                                    Ok(temp) => Some(temp),
                                    Err(e) if e.code == uldaq_sys::ERR_OPEN_CONNECTION => {
                                        Some(-9999.0)
                                    }
                                    Err(e) => {
                                        let line = error_line("Calling TIn", &e);
                                        trace::print(
                                            &handle,
                                            TraceMask::ERROR,
                                            format_args!("{line}"),
                                        );
                                        snap.tin_failure.get_or_insert(line);
                                        None
                                    }
                                };
                                // C reports every TIn it counts as a success,
                                // an open thermocouple included
                                // (drvMultiFunction.cpp:2822-2834).
                                if temp.is_some() {
                                    trace::print(
                                        &handle,
                                        TraceMask::IO_DRIVER,
                                        format_args!("{DRIVER}::pollerThread Info: Calling TIn"),
                                    );
                                }
                                snap.ai_temp[ch] = temp;
                            }
                        }
                    }
                }
            }
            snap
        }; // device lock released here

        // ---- Phase 3: report + write results (no device lock) ----
        let failed = !snapshot.failure.is_empty() || snapshot.tin_failure.is_some();
        if failed && !prev_failed {
            for line in &snapshot.failure {
                trace::print(&handle, TraceMask::ERROR, format_args!("{line}"));
            }
            if let Some(line) = snapshot.failure.first().or(snapshot.tin_failure.as_ref()) {
                let _ = handle.set_params_and_notify_blocking(
                    0,
                    vec![ParamSetValue::new(
                        params.last_error_message,
                        0,
                        ParamValue::Octet(line.clone().into_bytes()),
                    )],
                );
            }
        } else if !failed && prev_failed {
            trace::print(
                &handle,
                TraceMask::ERROR,
                format_args!("{DRIVER}::pollerThread Error: Device returned to normal status"),
            );
        }
        prev_failed = failed;

        if let Some(data) = snapshot.digital_input {
            let changed = data ^ prev_digital_input;
            if force_callback || changed != 0 {
                prev_digital_input = data;
                force_callback = false;
                let _ = handle.set_params_and_notify_blocking(
                    0,
                    vec![ParamSetValue::uint32_digital(
                        params.digital_input,
                        0,
                        data as u32,
                        0xFFFF_FFFF,
                        0,
                    )],
                );
            }
        }
        for (counter, value) in snapshot.counters.iter().enumerate() {
            if let Some(v) = value {
                let addr = counter as i32;
                publish(
                    &handle,
                    addr,
                    vec![ParamSetValue::new(
                        params.counter_value,
                        addr,
                        ParamValue::Int32(*v as i32),
                    )],
                );
            }
        }
        if snapshot.wave_gen_running {
            publish(
                &handle,
                0,
                vec![ParamSetValue::new(
                    params.wave_gen_current_point,
                    0,
                    ParamValue::Int32(snapshot.wave_gen_current_point as i32),
                )],
            );
            // C stopWaveGen from the poll: the driver ends the scan -- Run,
            // the outputs put back -- if it is still the one that went idle.
            if let Some(generation) = snapshot.wave_gen_ended {
                let _ = handle.write_int32_blocking(params.wave_gen_scan_end, 0, generation as i32);
            }
        }
        if snapshot.wave_dig_running {
            publish(
                &handle,
                0,
                vec![ParamSetValue::new(
                    params.wave_dig_current_point,
                    0,
                    ParamValue::Int32(snapshot.wave_dig_current_point as i32),
                )],
            );
            // C stopWaveDig from the poll: the driver ends the scan -- data,
            // Run, auto-restart -- if it is still the one that went idle.
            if let Some(generation) = snapshot.wave_dig_ended {
                let _ = handle.write_int32_blocking(params.wave_dig_scan_end, 0, generation as i32);
            }
        } else {
            // Every sample is a callback, changed or not, as C forces with
            // its set(v+1)/set(v) pair: the averaging records must count each
            // poll, and an unchanging reading must keep its timestamp moving.
            for ch in 0..MAX_ANALOG_IN {
                let addr = ch as i32;
                let mut updates = Vec::new();
                if let Some(raw) = snapshot.ai_raw[ch] {
                    let reason = params.analog_in_value;
                    updates.push(ParamSetValue::new(
                        reason,
                        addr,
                        ParamValue::Int32(raw.wrapping_add(1)),
                    ));
                    updates.push(ParamSetValue::new(reason, addr, ParamValue::Int32(raw)));
                }
                if let Some(t) = snapshot.ai_temp[ch] {
                    let reason = params.temperature_in_value;
                    updates.push(ParamSetValue::new(
                        reason,
                        addr,
                        ParamValue::Float64(t + 1.0),
                    ));
                    updates.push(ParamSetValue::new(reason, addr, ParamValue::Float64(t)));
                }
                if !updates.is_empty() {
                    let _ = handle.set_params_and_notify_blocking(addr, updates);
                }
            }
        }

        for addr in 0..MAX_SIGNALS as i32 {
            let _ = handle.call_param_callbacks_blocking(addr);
        }

        let poll_ms = handle
            .read_float64_blocking(params.poll_sleep_ms, 0)
            .unwrap_or(50.0);
        std::thread::sleep(poll_sleep(poll_ms));
    }
}

/// Store readbacks and run their callbacks, without going through the
/// driver's write handlers: a readback is not a command.
fn publish(handle: &PortHandle, addr: i32, updates: Vec<ParamSetValue>) {
    let _ = handle.set_params_and_notify_blocking(addr, updates);
}

/// C `epicsThreadSleep(pollTime/1000.)`: the fraction of a millisecond is
/// kept, and a negative or non-finite POLL_SLEEP_MS sleeps not at all.
fn poll_sleep(poll_ms: f64) -> Duration {
    Duration::try_from_secs_f64(poll_ms / 1000.0).unwrap_or(Duration::ZERO)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fractional_poll_sleep_is_kept() {
        assert_eq!(poll_sleep(0.5), Duration::from_micros(500));
        assert_eq!(poll_sleep(50.0), Duration::from_millis(50));
    }

    #[test]
    fn a_cycle_keeps_only_its_first_failure() {
        let mut snap = PollSnapshot::default();
        snap.failed(vec!["DIn".to_string(), "portNumber=0".to_string()]);
        snap.failed(vec!["CIn".to_string()]);
        assert_eq!(snap.failure, ["DIn", "portNumber=0"]);
    }

    #[test]
    fn a_failed_call_is_reported_in_c_words() {
        let e = MeasCompError {
            code: uldaq_sys::ERR_DEV_NOT_CONNECTED,
            message: "Device not connected".to_string(),
        };
        assert_eq!(
            error_line("Calling CIn", &e),
            format!(
                "MultiFunction::pollerThread Error: Calling CIn, err={} Device not connected",
                uldaq_sys::ERR_DEV_NOT_CONNECTED
            )
        );
    }

    #[test]
    fn an_unusable_poll_sleep_does_not_sleep() {
        assert_eq!(poll_sleep(-1.0), Duration::ZERO);
        assert_eq!(poll_sleep(f64::NAN), Duration::ZERO);
    }
}
