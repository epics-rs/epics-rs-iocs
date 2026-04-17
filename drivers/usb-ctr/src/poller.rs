use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::port_handle::PortHandle;

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
                Err(e) => log::warn!("CTR poller DIn error: {e}"),
            }

            // Read counter values (only when scaler/MCS not running)
            if let Ok(mut st) = state.lock() {
                if st.scaler.running {
                    scaler::read_scaler(&dev, &mut st.scaler);
                    if st.scaler.done {
                        // Update counter PVs with final scaler counts
                        for i in 0..MAX_COUNTERS {
                            let _ = handle.write_int32_blocking(
                                params.counter_value,
                                i as i32,
                                st.scaler.counts[i] as i32,
                            );
                        }
                        let _ = handle.write_int32_blocking(params.scaler_done, 0, 1);
                    }
                } else if st.mcs.running {
                    mcs::read_mcs(&dev, &mut st.mcs);
                    let _ = handle.write_int32_blocking(
                        params.mcs_current_point,
                        0,
                        st.mcs.current_point as i32,
                    );
                    if !st.mcs.acquiring {
                        let _ = handle.write_int32_blocking(params.mca_acquiring, 0, 0);
                    }
                } else {
                    // Normal counter polling
                    for counter in 0..MAX_COUNTERS as i32 {
                        match dev.counter_in(counter) {
                            Ok(value) => {
                                let _ = handle.write_int32_blocking(
                                    params.counter_value,
                                    counter,
                                    value as i32,
                                );
                            }
                            Err(e) => log::warn!("CTR poller CIn({counter}) error: {e}"),
                        }
                    }
                }
            }
        }

        // Callbacks for all addresses
        for addr in 0..MAX_MCS_COUNTERS as i32 {
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
