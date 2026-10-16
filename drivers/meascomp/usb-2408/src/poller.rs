use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::trace::TraceMask;

use meascomp::device::DaqDevice;

use crate::driver::MultiFunctionDriver;
use crate::params::*;
use crate::trace::{SelfTrace, status_of};
use crate::wave_dig::{self, WaveDigScan, WaveDigState};
use crate::wave_gen::{self, WaveGenState};

/// Shared state between driver and poller.
pub struct PollerState {
    pub wave_dig: WaveDigState,
    pub wave_gen: WaveGenState,
    pub trace: SelfTrace,
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
    /// Running at the start of the cycle.
    wave_dig_running: bool,
    wave_dig_current_point: usize,
    /// Array callbacks of a digitizer scan that ended this cycle, built
    /// while the state lock is held and pushed after it is released.
    wave_dig_arrays: Option<Vec<ParamSetValue>>,
    // Analog inputs (only populated when wave_dig is not running)
    ai_raw: [Option<i32>; MAX_ANALOG_IN],
    ai_volts: [Option<f64>; MAX_ANALOG_IN],
    ai_temp: [Option<f64>; MAX_ANALOG_IN],
}

fn poller_loop(
    handle: PortHandle,
    params: MultiFunctionParams,
    device: Arc<Mutex<DaqDevice>>,
    state: Arc<Mutex<PollerState>>,
) {
    const FUNCTION: &str = "pollerThread";
    let mut prev_digital_input: u64 = 0;
    let mut force_callback = true;
    let trace = state.lock().unwrap().trace.clone();
    // C `prevStatus`: a failing call is reported on the first cycle of an
    // outage only, and its end once.
    let mut prev_status = 0;
    let mut start = Instant::now();

    loop {
        // C's POLL_TIME_MS is the period from one pass's start to the next.
        let poll_time = start.elapsed();
        start = Instant::now();

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

        // What a digitizer scan that ends this cycle restarts with.
        let restart = if state.lock().is_ok_and(|st| st.wave_dig.running) {
            WaveDigScan::read_restart(
                &params,
                |r, a| handle.read_int32_blocking(r, a).unwrap_or(0),
                |r, a| handle.read_float64_blocking(r, a).unwrap_or(0.0),
            )
        } else {
            None
        };

        // What a generator that ends this cycle puts back.
        let wave_gen_restore = if state.lock().is_ok_and(|st| st.wave_gen.running) {
            wave_gen::restore_outputs(&params, |r, a| {
                handle.read_int32_blocking(r, a).unwrap_or(0)
            })
        } else {
            Vec::new()
        };

        // ---- Phase 2: uldaq reads (device lock held) ----
        // C `status`: the last uldaq call's result. A failed DIn, CIn or
        // AOutScanStatus ends the cycle's reads (C `goto error`).
        let mut status = 0;
        let mut goto_error = false;
        let snapshot = {
            let mut snap = PollSnapshot::default();
            if let Ok(dev) = device.lock() {
                'error: {
                    match dev.digital_in(uldaq_sys::AUXPORT) {
                        Ok(data) => snap.digital_input = Some(data),
                        Err(e) => {
                            status = e.code;
                            if prev_status == 0 {
                                trace.report_error(status, FUNCTION, "Calling DIn");
                                trace.print(TraceMask::ERROR, format_args!("portNumber=0"));
                            }
                            goto_error = true;
                            break 'error;
                        }
                    }

                    for counter in 0..MAX_COUNTERS {
                        match dev.counter_in(counter as i32) {
                            Ok(value) => snap.counters[counter] = Some(value as i64),
                            Err(e) => {
                                status = e.code;
                                if prev_status == 0 {
                                    trace.report_error(status, FUNCTION, "Calling CIn");
                                }
                                goto_error = true;
                                break 'error;
                            }
                        }
                    }

                    let Ok(mut st) = state.lock() else {
                        break 'error;
                    };
                    let wg_was_running = st.wave_gen.running;
                    if st.wave_gen.running {
                        if let Err(code) = wave_gen::read_wave_gen(
                            &dev,
                            &mut st.wave_gen,
                            &wave_gen_restore,
                            &trace,
                        ) {
                            status = code;
                            if prev_status == 0 {
                                trace.report_error(status, FUNCTION, "Calling AOutScanStatus");
                            }
                            goto_error = true;
                            break 'error;
                        }
                        snap.wave_gen_current_point = st.wave_gen.current_point;
                    }
                    snap.wave_gen_running = st.wave_gen.running;
                    snap.wave_gen_just_stopped = wg_was_running && !st.wave_gen.running;

                    if st.wave_dig.running {
                        match wave_dig::read_wave_dig(
                            &dev,
                            &mut st.wave_dig,
                            &params,
                            restart.as_ref(),
                            &trace,
                        ) {
                            Ok(arrays) => snap.wave_dig_arrays = arrays,
                            // Known deviation: Linux C does not `goto error`
                            // here and goes on with the scanStatus/xferStatus
                            // ulAInScanStatus left unset (UB); rs skips the
                            // buffer copy and the stop for this pass.
                            Err(code) => {
                                status = code;
                                if prev_status == 0 {
                                    trace.report_error(status, FUNCTION, "Calling AInScanStatus");
                                }
                            }
                        }
                        snap.wave_dig_current_point = st.wave_dig.current_point;
                        snap.wave_dig_running = true;
                    } else {
                        for ch in 0..MAX_ANALOG_IN {
                            let ch_i = ch as i32;
                            // One call per channel, chosen by its configured
                            // type: ulAIn rejects a thermocouple channel with
                            // ERR_BAD_RANGE and ulTIn rejects a voltage one.
                            // C drvMultiFunction skips each the same way.
                            if in_types[ch] != 0 {
                                continue;
                            }
                            // C does not report a failed ulAIn; its status is
                            // the raw read's (C has no scaled read).
                            let raw = dev.analog_in(
                                ch_i,
                                input_mode,
                                in_ranges[ch],
                                uldaq_sys::AIN_FF_NOSCALEDATA,
                            );
                            status = status_of(&raw);
                            snap.ai_raw[ch] = raw.ok().map(|v| v as i32);
                            let volts = dev.analog_in(
                                ch_i,
                                input_mode,
                                in_ranges[ch],
                                uldaq_sys::AIN_FF_DEFAULT,
                            );
                            snap.ai_volts[ch] = volts.ok();
                        }
                        for ch in 0..MAX_ANALOG_IN {
                            if in_types[ch] == 0 {
                                continue;
                            }
                            // An open or broken thermocouple is expected,
                            // not an error: ulTIn reports it as -9999.
                            status = match dev.temperature_in(
                                ch as i32,
                                tc_scales[ch],
                                uldaq_sys::TIN_FF_DEFAULT,
                            ) {
                                Ok(temp) => {
                                    snap.ai_temp[ch] = Some(temp);
                                    0
                                }
                                Err(e) if e.code == uldaq_sys::ERR_OPEN_CONNECTION => {
                                    snap.ai_temp[ch] = Some(-9999.0);
                                    0
                                }
                                Err(e) => e.code,
                            };
                            trace.report_error(status, FUNCTION, "Calling TIn");
                        }
                    }
                }
            }
            snap
        }; // device lock released here

        if prev_status != 0 && status == 0 {
            trace.report_error(-1, FUNCTION, "Device returned to normal status");
        }
        prev_status = status;

        // ---- Phase 3: set the results under the port, as C's setXxxParam
        // calls do; a `goto error` cycle skips the callbacks.
        let int = |reason, addr: usize, v: i32| {
            ParamSetValue::new(reason, addr as i32, ParamValue::Int32(v))
        };
        let float = |reason, addr: usize, v: f64| {
            ParamSetValue::new(reason, addr as i32, ParamValue::Float64(v))
        };
        let mut updates = vec![float(
            params.poll_time_ms,
            0,
            poll_time.as_secs_f64() * 1000.0,
        )];
        if let Some(data) = snapshot.digital_input {
            let changed = data ^ prev_digital_input;
            if force_callback || changed != 0 {
                prev_digital_input = data;
                force_callback = false;
                updates.push(ParamSetValue::uint32_digital(
                    params.digital_input,
                    0,
                    data as u32,
                    0xFFFF_FFFF,
                    0,
                ));
            }
        }
        for (counter, value) in snapshot.counters.iter().enumerate() {
            if let Some(v) = value {
                updates.push(int(params.counter_value, counter, *v as i32));
            }
        }
        if snapshot.wave_gen_running || snapshot.wave_gen_just_stopped {
            updates.push(int(
                params.wave_gen_current_point,
                0,
                snapshot.wave_gen_current_point as i32,
            ));
            if snapshot.wave_gen_just_stopped {
                updates.push(int(params.wave_gen_run, 0, 0));
            }
        }
        if snapshot.wave_dig_running {
            updates.push(int(
                params.wave_dig_current_point,
                0,
                snapshot.wave_dig_current_point as i32,
            ));
            // The scan ended: C stopWaveDig's RUN, readWaveDig callbacks
            // and, under AUTO_RESTART, startWaveDig's parameters.
            updates.extend(snapshot.wave_dig_arrays.into_iter().flatten());
        } else {
            for ch in 0..MAX_ANALOG_IN {
                if let Some(raw) = snapshot.ai_raw[ch] {
                    updates.push(int(params.analog_in_value, ch, raw));
                }
                if let Some(v) = snapshot.ai_volts[ch] {
                    updates.push(float(params.voltage_in_value, ch, v));
                }
                if let Some(t) = snapshot.ai_temp[ch] {
                    updates.push(float(params.temperature_in_value, ch, t));
                }
            }
        }
        let _ = handle.with_driver_blocking::<MultiFunctionDriver, _>(move |driver| {
            let base = driver.base_mut();
            for update in updates {
                let _ = apply(base, update);
            }
            if !goto_error {
                for addr in 0..MAX_SIGNALS as i32 {
                    let _ = base.call_param_callbacks(addr);
                }
            }
        });

        let poll_ms = handle
            .read_float64_blocking(params.poll_sleep_ms, 0)
            .unwrap_or(50.0);
        std::thread::sleep(Duration::from_secs_f64(poll_ms.max(0.0) / 1000.0));
    }
}

/// One `setXxxParam` of the poll pass.
fn apply(base: &mut PortDriverBase, update: ParamSetValue) -> AsynResult<()> {
    match update {
        ParamSetValue::Value {
            reason,
            addr,
            value,
        } => base.set_param_value(reason, addr, value),
        ParamSetValue::UInt32Digital {
            reason,
            addr,
            value,
            mask,
            interrupt_mask,
        } => base.set_uint32_param(reason, addr, value, mask, interrupt_mask),
        ParamSetValue::Status {
            reason,
            addr,
            status,
            alarm_status,
            alarm_severity,
        } => base.set_param_status(reason, addr, status, alarm_status, alarm_severity),
    }
}
