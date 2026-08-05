use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

use meascomp::device::DaqDevice;

use crate::mcs::{self, McsState};
use crate::params::*;
use crate::scaler::{self, ScalerState};

/// Shared state between driver and poller.
pub struct PollerState {
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

    loop {
        let start = Instant::now();

        // ---- Phase 1: uldaq reads (device lock held) ----
        let snapshot = {
            let mut snap = PollSnapshot::default();
            if let Ok(dev) = device.lock() {
                match dev.digital_in(uldaq_sys::AUXPORT) {
                    Ok(data) => snap.digital_input = Some(data),
                    Err(e) => snap.errors.push(format!("DIn: {e}")),
                }

                if let Ok(mut st) = state.lock() {
                    if st.scaler.running {
                        scaler::read_scaler(&dev, &mut st.scaler);
                        if st.scaler.done {
                            snap.scaler_done_snapshot = Some(st.scaler.counts);
                        }
                    } else if st.mcs.running {
                        let mcs_was_acquiring = st.mcs.acquiring;
                        mcs::read_mcs(&dev, &mut st.mcs);
                        snap.mcs_running = true;
                        snap.mcs_current_point = st.mcs.current_point;
                        snap.mcs_just_stopped = mcs_was_acquiring && !st.mcs.acquiring;
                        snap.mcs_elapsed = st.mcs.elapsed_secs();
                    } else {
                        for counter in 0..MAX_COUNTERS {
                            match dev.counter_in(counter as i32) {
                                Ok(value) => snap.counters[counter] = Some(value as i64),
                                Err(e) => snap.errors.push(format!("CIn({counter}): {e}")),
                            }
                        }
                    }
                }
            }
            snap
        }; // device lock released here

        // ---- Phase 2: log + write results (no device lock) ----
        for msg in &snapshot.errors {
            log::warn!("CTR poller {msg}");
        }
        if let Some(msg) = snapshot.errors.last() {
            let _ = handle.set_params_and_notify_blocking(
                0,
                vec![ParamSetValue::new(
                    params.last_error_message,
                    0,
                    ParamValue::Octet(msg.clone()),
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
                let _ = handle.write_int32_blocking(params.counter_value, i as i32, *c as i32);
            }
        } else if snapshot.mcs_running {
            let _ = handle.write_int32_blocking(
                params.mcs_current_point,
                0,
                snapshot.mcs_current_point as i32,
            );
            // C readMCS sets the elapsed times on every counter address, so a
            // per-counter mca record sees them too.
            for addr in 0..MAX_MCS_COUNTERS as i32 {
                let _ = handle.write_float64_blocking(
                    params.mca_elapsed_real,
                    addr,
                    snapshot.mcs_elapsed,
                );
                let _ = handle.write_float64_blocking(
                    params.mca_elapsed_live,
                    addr,
                    snapshot.mcs_elapsed,
                );
            }
            if snapshot.mcs_just_stopped {
                let _ = handle.write_int32_blocking(params.mca_acquiring, 0, 0);
            }
        } else {
            for (counter, value) in snapshot.counters.iter().enumerate() {
                if let Some(v) = value {
                    let _ = handle.write_int32_blocking(
                        params.counter_value,
                        counter as i32,
                        *v as i32,
                    );
                }
            }
        }

        // Callbacks for all addresses
        for addr in 0..MAX_MCS_COUNTERS as i32 {
            let _ = handle.call_param_callbacks_blocking(addr);
        }

        let elapsed = start.elapsed();
        let _ =
            handle.write_float64_blocking(params.poll_time_ms, 0, elapsed.as_secs_f64() * 1000.0);

        let poll_ms = handle
            .read_float64_blocking(params.poll_sleep_ms, 0)
            .unwrap_or(50.0);
        std::thread::sleep(Duration::from_millis(poll_ms as u64));
    }
}
