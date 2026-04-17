use meascomp::device::DaqDevice;
use uldaq_sys::*;

use crate::params::*;

/// Shared scaler state between the driver (arm/reset) and poller (read/done).
pub struct ScalerState {
    pub running: bool,
    pub done: bool,
    pub counts: [u64; MAX_COUNTERS],
    pub presets: [u64; MAX_COUNTERS],
    /// Buffer for ulCInScan continuous mode (20 samples per counter).
    pub scan_buffer: Vec<u64>,
}

impl ScalerState {
    pub fn new() -> Self {
        Self {
            running: false,
            done: false,
            counts: [0; MAX_COUNTERS],
            presets: [0; MAX_COUNTERS],
            scan_buffer: vec![0u64; MAX_COUNTERS * 20],
        }
    }
}

/// Configure counters and start the continuous counter scan for scaler mode.
pub fn start_scaler(
    device: &DaqDevice,
    state: &mut ScalerState,
) -> Result<(), String> {
    let num_counters = MAX_COUNTERS as i32;

    // Configure each counter for counting mode
    for i in 0..num_counters {
        let mode = if i == 0 {
            // Counter 0: preset control with gating
            CMM_OUTPUT_ON | CMM_OUTPUT_INITIAL_STATE_HIGH
                | CMM_GATING_ON | CMM_INVERT_GATE
                | CMM_RANGE_LIMIT_ON | CMM_NO_RECYCLE
        } else {
            CMM_OUTPUT_ON | CMM_OUTPUT_INITIAL_STATE_HIGH
                | CMM_GATING_ON | CMM_INVERT_GATE
        };

        device.counter_config_scan(
            i,
            CMT_COUNT,
            mode,
            CED_RISING_EDGE,
            CTS_TICK_20PT83ns,
            CDM_NONE,
            CDT_DEBOUNCE_0ns,
            CF_DEFAULT,
        ).map_err(|e| format!("counter_config_scan({i}) error: {e}"))?;
    }

    // Set presets as max limits
    for i in 0..MAX_COUNTERS {
        if state.presets[i] > 0 {
            device.counter_load(i as i32, CRT_MAX_LIMIT, state.presets[i])
                .map_err(|e| format!("counter_load MAX_LIMIT({i}) error: {e}"))?;
        }
    }

    // Start continuous counter scan
    let mut rate = 10000.0; // Will be adjusted by driver
    let options = SO_CONTINUOUS | SO_SINGLEIO;
    let flags = CINSCAN_FF_CTR64_BIT;

    device.counter_in_scan(
        0,
        num_counters - 1,
        20, // 20 samples per counter
        &mut rate,
        options,
        flags,
        &mut state.scan_buffer,
    ).map_err(|e| format!("counter_in_scan error: {e}"))?;

    state.running = true;
    state.done = false;
    log::info!("Scaler started, rate={rate:.0} Hz");
    Ok(())
}

/// Read latest counter values from the scan buffer. Check for preset completion.
pub fn read_scaler(
    device: &DaqDevice,
    state: &mut ScalerState,
) {
    let (status, xfer) = match device.counter_in_scan_status() {
        Ok(v) => v,
        Err(e) => {
            log::warn!("scaler scan status error: {e}");
            return;
        }
    };

    if xfer.current_total_count == 0 {
        return;
    }

    let num_counters = MAX_COUNTERS;
    let buf_len = state.scan_buffer.len();
    if buf_len == 0 || xfer.current_index < 0 {
        return;
    }

    // current_index is the last written position in the circular buffer.
    // Find the start of the last complete sample set.
    let cur_idx = xfer.current_index as usize % buf_len;
    let num_in_buf = cur_idx + 1;
    if num_in_buf < num_counters {
        return;
    }
    let last_index = (num_in_buf / num_counters - 1) * num_counters;

    // Read counts from buffer
    for j in 0..num_counters {
        state.counts[j] = state.scan_buffer[last_index + j];
    }

    // Check presets
    let mut preset_reached = false;
    for j in 0..num_counters {
        if state.presets[j] > 0 && state.counts[j] >= state.presets[j] {
            preset_reached = true;
            break;
        }
    }

    if preset_reached || status == SS_IDLE {
        stop_scaler(device, state);
        state.done = true;
    }
}

/// Stop the counter scan.
pub fn stop_scaler(device: &DaqDevice, state: &mut ScalerState) {
    if state.running {
        if let Err(e) = device.counter_in_scan_stop() {
            log::warn!("scaler scan stop error: {e}");
        }
        state.running = false;
    }
}

/// Reset all counters to zero.
pub fn reset_scaler(device: &DaqDevice, state: &mut ScalerState) {
    stop_scaler(device, state);
    for i in 0..MAX_COUNTERS {
        state.counts[i] = 0;
        if let Err(e) = device.counter_clear(i as i32) {
            log::warn!("counter_clear({i}) error: {e}");
        }
    }
    state.done = false;
}
