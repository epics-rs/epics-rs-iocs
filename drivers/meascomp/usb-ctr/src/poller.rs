use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::trace::TraceMask;

use meascomp::device::DaqDevice;

use crate::mcs::{self, McsPass, McsState};
use crate::params::*;
use crate::scaler::{self, ScalerState};
use crate::trace::SelfTrace;

/// Shared state between driver and poller.
pub struct PollerState {
    /// C `numCounters_`: 8 on a USB-CTR08, 4 on a USB-CTR04.
    pub num_counters: usize,
    pub scaler: ScalerState,
    pub mcs: McsState,
    pub trace: SelfTrace,
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
    mcs: Option<McsPass>,
    num_counters: usize,
}

/// C `USBCTR::pollerThread`: DIn, then the running scaler or MCS, then every
/// address's callbacks.
fn poller_loop(
    handle: PortHandle,
    params: CtrParams,
    device: Arc<Mutex<DaqDevice>>,
    state: Arc<Mutex<PollerState>>,
) {
    let mut prev_digital_input: u64 = 0;
    let mut force_callback = true;
    let trace = state.lock().unwrap().trace.clone();
    let mut start = Instant::now();

    loop {
        // C's POLL_TIME_MS is the period from one pass's start to the next.
        let poll_time = start.elapsed();
        start = Instant::now();

        // ---- Phase 1: uldaq reads (device lock held) ----
        let snapshot = {
            let mut snap = PollSnapshot::default();
            if let Ok(dev) = device.lock() {
                match dev.digital_in(uldaq_sys::AUXPORT) {
                    Ok(data) => snap.digital_input = Some(data),
                    Err(e) => trace.print(
                        TraceMask::ERROR,
                        format_args!(
                            "USBCTR:pollerThread: ERROR calling cbDIn, status={}",
                            e.code
                        ),
                    ),
                }

                if let Ok(mut st) = state.lock() {
                    if st.scaler.running {
                        let num_counters = st.num_counters;
                        scaler::read_scaler(&dev, &mut st.scaler, num_counters, &trace);
                    } else if st.mcs.running {
                        snap.mcs = Some(mcs::read_mcs(&dev, &mut st.mcs, &trace));
                        snap.num_counters = st.num_counters;
                    }
                }
            }
            snap
        }; // device lock released here

        // ---- Phase 2: set params (no device lock), as setXxxParam does ----
        let mut updates = vec![ParamSetValue::new(
            params.poll_time_ms,
            0,
            ParamValue::Float64(poll_time.as_secs_f64() * 1000.0),
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
        if let Some(pass) = snapshot.mcs {
            updates.extend(
                pass.updates(&params, snapshot.num_counters)
                    .into_iter()
                    .map(|(reason, addr, value)| ParamSetValue::new(reason, addr, value)),
            );
        }
        let _ = handle.set_params_and_notify_blocking(0, updates);

        // Callbacks for all addresses
        for addr in 0..MAX_MCS_COUNTERS as i32 {
            let _ = handle.call_param_callbacks_blocking(addr);
        }
        if let Ok(st) = state.lock() {
            st.scaler.done.call_callbacks();
        }

        let poll_ms = handle
            .read_float64_blocking(params.poll_sleep_ms, 0)
            .unwrap_or(50.0);
        std::thread::sleep(Duration::from_secs_f64(poll_ms.max(0.0) / 1000.0));
    }
}
