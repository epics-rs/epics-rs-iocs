use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

use meascomp::device::DaqDevice;

use crate::params::*;
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
    wave_gen_just_stopped: bool,
    wave_dig_running: bool,
    wave_dig_current_point: usize,
    wave_dig_just_stopped: bool,
    /// Array callbacks for the digitized data, built while the state lock is
    /// held and pushed after it is released.
    wave_dig_arrays: Vec<ParamSetValue>,
    // Analog inputs (only populated when wave_dig is not running)
    ai_raw: [Option<i32>; MAX_ANALOG_IN],
    ai_temp: [Option<f64>; MAX_ANALOG_IN],
    // Errors to log after releasing the lock.
    errors: Vec<String>,
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
        let _ = handle.write_float64_blocking(
            params.poll_time_ms,
            0,
            now.duration_since(cycle_start).as_secs_f64() * 1000.0,
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
                    Err(e) => snap.errors.push(format!("DIn: {e}")),
                }

                for counter in 0..MAX_COUNTERS {
                    match dev.counter_in(counter as i32) {
                        Ok(value) => snap.counters[counter] = Some(value as i64),
                        Err(e) => snap.errors.push(format!("CIn({counter}): {e}")),
                    }
                }

                if let Ok(mut st) = state.lock() {
                    let wg_was_running = st.wave_gen.running;
                    if st.wave_gen.running {
                        wave_gen::read_wave_gen(&dev, &mut st.wave_gen);
                        snap.wave_gen_current_point = st.wave_gen.current_point;
                    }
                    snap.wave_gen_running = st.wave_gen.running;
                    snap.wave_gen_just_stopped = wg_was_running && !st.wave_gen.running;

                    let wd_was_running = st.wave_dig.running;
                    if st.wave_dig.running {
                        wave_dig::read_wave_dig(&dev, &mut st.wave_dig);
                        snap.wave_dig_current_point = st.wave_dig.current_point;
                    }
                    snap.wave_dig_running = st.wave_dig.running;
                    snap.wave_dig_just_stopped = wd_was_running && !st.wave_dig.running;
                    if snap.wave_dig_just_stopped {
                        snap.wave_dig_arrays = wave_dig::waveform_updates(&params, &st.wave_dig);
                    }

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
                                    Err(e) => snap.errors.push(format!("AIn({ch}): {e}")),
                                }
                            } else {
                                // An open or broken thermocouple is expected,
                                // not an error: ulTIn reports it as -9999.
                                match dev.temperature_in(
                                    ch_i,
                                    tc_scales[ch],
                                    uldaq_sys::TIN_FF_DEFAULT,
                                ) {
                                    Ok(temp) => snap.ai_temp[ch] = Some(temp),
                                    Err(e) => {
                                        if e.code == uldaq_sys::ERR_OPEN_CONNECTION {
                                            snap.ai_temp[ch] = Some(-9999.0);
                                        } else {
                                            snap.errors.push(format!("TIn({ch}): {e}"));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            snap
        }; // device lock released here

        // ---- Phase 3: log + write results (no device lock) ----
        let failed = !snapshot.errors.is_empty();
        if failed && !prev_failed {
            for msg in &snapshot.errors {
                log::warn!("USB-2408 poller {msg}");
            }
            if let Some(msg) = snapshot.errors.last() {
                let _ = handle.set_params_and_notify_blocking(
                    0,
                    vec![ParamSetValue::new(
                        params.last_error_message,
                        0,
                        ParamValue::Octet(msg.clone().into_bytes()),
                    )],
                );
            }
        } else if !failed && prev_failed {
            log::warn!("USB-2408 poller: device returned to normal status");
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
                let _ =
                    handle.write_int32_blocking(params.counter_value, counter as i32, *v as i32);
            }
        }
        if snapshot.wave_gen_running || snapshot.wave_gen_just_stopped {
            let _ = handle.write_int32_blocking(
                params.wave_gen_current_point,
                0,
                snapshot.wave_gen_current_point as i32,
            );
            if snapshot.wave_gen_just_stopped {
                let _ = handle.write_int32_blocking(params.wave_gen_run, 0, 0);
            }
        }
        if snapshot.wave_dig_running || snapshot.wave_dig_just_stopped {
            let _ = handle.write_int32_blocking(
                params.wave_dig_current_point,
                0,
                snapshot.wave_dig_current_point as i32,
            );
            if snapshot.wave_dig_just_stopped {
                let _ = handle.write_int32_blocking(params.wave_dig_run, 0, 0);
                // The scan is complete: hand the acquired points to the
                // waveform records, as C stopWaveDig does through readWaveDig.
                let _ = handle.set_params_and_notify_blocking(0, snapshot.wave_dig_arrays.clone());
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
    fn an_unusable_poll_sleep_does_not_sleep() {
        assert_eq!(poll_sleep(-1.0), Duration::ZERO);
        assert_eq!(poll_sleep(f64::NAN), Duration::ZERO);
    }
}
