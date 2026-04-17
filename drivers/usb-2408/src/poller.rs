use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::port_handle::PortHandle;

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

fn poller_loop(
    handle: PortHandle,
    params: MultiFunctionParams,
    device: Arc<Mutex<DaqDevice>>,
    state: Arc<Mutex<PollerState>>,
) {
    let mut prev_digital_input: u64 = 0;
    let mut force_callback = true;

    loop {
        let start = Instant::now();

        if let Ok(dev) = device.lock() {
            // Read digital inputs
            match dev.digital_in(uldaq_sys::AUXPORT) {
                Ok(data) => {
                    let changed = data ^ prev_digital_input;
                    if force_callback || changed != 0 {
                        prev_digital_input = data;
                        force_callback = false;
                        let _ = handle.write_int32_blocking(
                            params.digital_input,
                            0,
                            data as i32,
                        );
                    }
                }
                Err(e) => log::warn!("USB-2408 poller DIn error: {e}"),
            }

            // Read counters
            for counter in 0..MAX_COUNTERS as i32 {
                match dev.counter_in(counter) {
                    Ok(value) => {
                        let _ = handle.write_int32_blocking(
                            params.counter_value,
                            counter,
                            value as i32,
                        );
                    }
                    Err(e) => log::warn!("USB-2408 poller CIn({counter}) error: {e}"),
                }
            }

            if let Ok(mut st) = state.lock() {
                // Waveform generator status
                if st.wave_gen.running {
                    wave_gen::read_wave_gen(&dev, &mut st.wave_gen);
                    let _ = handle.write_int32_blocking(
                        params.wave_gen_current_point,
                        0,
                        st.wave_gen.current_point as i32,
                    );
                    if !st.wave_gen.running {
                        let _ = handle.write_int32_blocking(params.wave_gen_run, 0, 0);
                    }
                }

                // Waveform digitizer status
                if st.wave_dig.running {
                    wave_dig::read_wave_dig(&dev, &mut st.wave_dig);
                    let _ = handle.write_int32_blocking(
                        params.wave_dig_current_point,
                        0,
                        st.wave_dig.current_point as i32,
                    );
                    if !st.wave_dig.running {
                        let _ = handle.write_int32_blocking(params.wave_dig_run, 0, 0);
                    }
                } else {
                    // Only read analog inputs when digitizer is not running
                    let input_mode = handle
                        .read_int32_blocking(params.analog_in_mode, 0)
                        .unwrap_or(uldaq_sys::AI_DIFFERENTIAL);

                    for ch in 0..MAX_ANALOG_IN as i32 {
                        let in_type = handle
                            .read_int32_blocking(params.analog_in_type, ch)
                            .unwrap_or(0);

                        let range = handle
                            .read_int32_blocking(params.analog_in_range, ch)
                            .unwrap_or(uldaq_sys::BIP10VOLTS);

                        // Read raw value
                        match dev.analog_in(ch, input_mode, range, uldaq_sys::AIN_FF_NOSCALEDATA) {
                            Ok(raw) => {
                                let _ = handle.write_int32_blocking(params.analog_in_value, ch, raw as i32);
                            }
                            Err(e) => log::warn!("USB-2408 poller AIn({ch}) error: {e}"),
                        }

                        // Read scaled voltage
                        match dev.analog_in(ch, input_mode, range, uldaq_sys::AIN_FF_DEFAULT) {
                            Ok(volts) => {
                                let _ = handle.write_float64_blocking(params.voltage_in_value, ch, volts);
                            }
                            Err(e) => log::warn!("USB-2408 poller AIn scaled({ch}) error: {e}"),
                        }

                        // Read temperature if configured as thermocouple
                        if in_type != 0 {
                            let scale = handle
                                .read_int32_blocking(params.temperature_scale, ch)
                                .unwrap_or(uldaq_sys::TS_CELSIUS);

                            match dev.temperature_in(ch, scale, uldaq_sys::TIN_FF_DEFAULT) {
                                Ok(temp) => {
                                    let _ = handle.write_float64_blocking(
                                        params.temperature_in_value,
                                        ch,
                                        temp,
                                    );
                                }
                                Err(e) => {
                                    if e.code == uldaq_sys::ERR_TEMP_OUT_OF_RANGE {
                                        let _ = handle.write_float64_blocking(
                                            params.temperature_in_value,
                                            ch,
                                            -9999.0,
                                        );
                                    } else {
                                        log::warn!("USB-2408 poller TIn({ch}) error: {e}");
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Callbacks
        for addr in 0..MAX_SIGNALS as i32 {
            let _ = handle.call_param_callbacks_blocking(addr);
        }

        let elapsed = start.elapsed();
        let _ = handle.write_float64_blocking(params.poll_time_ms, 0, elapsed.as_secs_f64() * 1000.0);

        let poll_ms = handle
            .read_float64_blocking(params.poll_sleep_ms, 0)
            .unwrap_or(50.0);
        std::thread::sleep(Duration::from_millis(poll_ms as u64));
    }
}
