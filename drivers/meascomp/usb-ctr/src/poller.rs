use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::trace::TraceMask;

use meascomp::device::DaqDevice;

use crate::mcs::{self, McsState};
use crate::params::*;
use crate::scaler::{self, ScalerState};
use crate::trace::{self, DRIVER, GetStatus};

/// Shared state between driver and poller.
pub struct PollerState {
    /// C `numCounters_`, fixed by the board model at construction.
    pub num_counters: usize,
    pub scaler: ScalerState,
    pub mcs: McsState,
}

/// Start the polling thread that reads DIO, counters, scaler, and MCS.
pub fn start_poller(
    handle: PortHandle,
    params: CtrParams,
    device: Arc<Mutex<DaqDevice>>,
    state: Arc<Mutex<PollerState>>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("usb-ctr-poller".into())
        .spawn(move || poller_loop(handle, params, device, state))
        .expect("failed to spawn CTR poller thread")
}

/// Per-cycle snapshot. Collected while the device mutex is held; `handle.*`
/// calls happen OUTSIDE the lock to avoid deadlocking with the asyn actor.
#[derive(Default)]
struct PollSnapshot {
    digital_input: Option<u64>,
    // Normal counter polling (scaler/MCS not running)
    counters: [Option<i64>; MAX_COUNTERS],
    // Scaler state
    scaler_done_snapshot: Option<[u64; MAX_COUNTERS]>,
    // MCS state
    mcs_running: bool,
    mcs_current_point: usize,
    mcs_just_stopped: bool,
    /// Seconds since the MCS scan started; C readMCS keeps
    /// mcaElapsedRealTime/LiveTime on every counter address.
    mcs_elapsed: f64,
    num_counters: usize,
    errors: Vec<String>,
}

fn poller_loop(
    handle: PortHandle,
    params: CtrParams,
    device: Arc<Mutex<DaqDevice>>,
    state: Arc<Mutex<PollerState>>,
) {
    let mut prev_digital_input: u64 = 0;
    let mut force_callback = true;
    let mut cycle_start = Instant::now();

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

        // C readMCS re-reads PresetReal on every poll, so a change mid-scan
        // takes effect.
        let preset_real = handle
            .read_float64_blocking(params.mca_preset_real, 0)
            .unwrap_or(0.0);

        // ---- Phase 1: uldaq reads (device lock held) ----
        let snapshot = {
            let mut snap = PollSnapshot::default();
            if let Ok(dev) = device.lock() {
                match dev.digital_in(uldaq_sys::AUXPORT) {
                    Ok(data) => snap.digital_input = Some(data),
                    Err(e) => snap.errors.push(format!(
                        "{DRIVER}:pollerThread: ERROR calling cbDIn, status={}",
                        e.code
                    )),
                }

                if let Ok(mut st) = state.lock() {
                    let num_counters = st.num_counters;
                    snap.num_counters = num_counters;
                    if st.scaler.running {
                        let read = scaler::read_scaler(&dev, &mut st.scaler, num_counters);
                        trace::print(
                            &handle,
                            None,
                            TraceMask::FLOW,
                            format_args!("{}", GetStatus("readScaler", read.position)),
                        );
                        snap.errors.extend(read.stop_error);
                        if let Some(last_index) = read.last_index {
                            trace::print(
                                &handle,
                                None,
                                TraceMask::FLOW,
                                format_args!(
                                    "{DRIVER}::readScaler lastIndex={last_index}, \
                                     scalerCounts_[0]={}, scalerPresetCounts_[0]={}",
                                    st.scaler.counts[0] as i32, st.scaler.presets[0]
                                ),
                            );
                        }
                        if st.scaler.done {
                            snap.scaler_done_snapshot = Some(st.scaler.counts);
                        }
                    } else if st.mcs.running {
                        let readout = mcs::read_mcs(&dev, &mut st.mcs, preset_real);
                        trace::print(
                            &handle,
                            None,
                            TraceMask::FLOW,
                            format_args!("{}", GetStatus("readMCS", readout.position)),
                        );
                        snap.errors.extend(readout.stop_error);
                        snap.mcs_running = true;
                        snap.mcs_current_point = readout.current_point;
                        snap.mcs_just_stopped = readout.finished;
                        snap.mcs_elapsed = readout.elapsed;
                    } else {
                        for counter in 0..num_counters {
                            match dev.counter_in(counter as i32) {
                                Ok(value) => snap.counters[counter] = Some(value as i64),
                                // C does not check ulCIn; reported in the
                                // form of its DIn line.
                                Err(e) => snap.errors.push(format!(
                                    "{DRIVER}:pollerThread: ERROR calling ulCIn, counter={counter}, \
                                     status={}",
                                    e.code
                                )),
                            }
                        }
                    }
                }
            }
            snap
        }; // device lock released here

        // ---- Phase 2: log + write results (no device lock) ----
        // C prints its DIn failure on every cycle it happens.
        for msg in &snapshot.errors {
            trace::print(&handle, None, TraceMask::ERROR, format_args!("{msg}"));
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
        if let Some(counts) = snapshot.scaler_done_snapshot {
            for (i, c) in counts.iter().enumerate() {
                let addr = i as i32;
                publish(
                    &handle,
                    addr,
                    vec![ParamSetValue::new(
                        params.counter_value,
                        addr,
                        ParamValue::Int32(*c as i32),
                    )],
                );
            }
        } else if snapshot.mcs_running {
            publish(
                &handle,
                0,
                vec![ParamSetValue::new(
                    params.mcs_current_point,
                    0,
                    ParamValue::Int32(snapshot.mcs_current_point as i32),
                )],
            );
            // C readMCS sets the elapsed times on every counter address, so a
            // per-counter mca record sees them too, and clears MCA_ACQUIRING
            // on every counter address once the scan has ended.
            for addr in 0..MAX_MCS_COUNTERS as i32 {
                let mut updates = vec![
                    ParamSetValue::new(
                        params.mca_elapsed_real,
                        addr,
                        ParamValue::Float64(snapshot.mcs_elapsed),
                    ),
                    ParamSetValue::new(
                        params.mca_elapsed_live,
                        addr,
                        ParamValue::Float64(snapshot.mcs_elapsed),
                    ),
                ];
                if snapshot.mcs_just_stopped && (addr as usize) < snapshot.num_counters {
                    updates.push(ParamSetValue::new(
                        params.mca_acquiring,
                        addr,
                        ParamValue::Int32(0),
                    ));
                }
                publish(&handle, addr, updates);
            }
        } else {
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
        }

        // Callbacks for all addresses
        for addr in 0..MAX_MCS_COUNTERS as i32 {
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
    fn an_unusable_poll_sleep_does_not_sleep() {
        assert_eq!(poll_sleep(-1.0), Duration::ZERO);
        assert_eq!(poll_sleep(f64::NAN), Duration::ZERO);
    }
}
