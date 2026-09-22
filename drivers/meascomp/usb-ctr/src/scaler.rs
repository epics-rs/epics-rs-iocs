use meascomp::counter::{CInScanConfig, CounterScanConfig};
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

impl Default for ScalerState {
    fn default() -> Self {
        Self::new()
    }
}

impl ScalerState {
    pub fn new() -> Self {
        Self {
            running: false,
            done: false,
            counts: [0; MAX_COUNTERS],
            presets: [0; MAX_COUNTERS],
            scan_buffer: vec![0u64; MAX_COUNTERS * SAMPLES_PER_COUNTER],
        }
    }
}

/// C `setScalerPresets` (drvUSBCTR.cpp:1040-1083), run on every arm before
/// the scan starts: each non-zero preset becomes its counter's MAX_LIMIT,
/// and counter 0's output-compare pair switches its output at PR1. That
/// output gates counters 1-7, so the time preset stops them in hardware,
/// not a poll later.
fn load_presets(
    device: &DaqDevice,
    state: &ScalerState,
    num_counters: usize,
) -> Result<(), String> {
    for i in 0..num_counters {
        if state.presets[i] > 0 {
            device
                .counter_load(i as i32, CRT_MAX_LIMIT, state.presets[i])
                .map_err(|e| format!("counter_load MAX_LIMIT({i}) error: {e}"))?;
        }
    }
    device
        .counter_load(0, CRT_OUTPUT_VAL0, 0)
        .map_err(|e| format!("counter_load OUTPUT_VAL0 error: {e}"))?;
    device
        .counter_load(0, CRT_OUTPUT_VAL1, state.presets[0])
        .map_err(|e| format!("counter_load OUTPUT_VAL1 error: {e}"))?;
    Ok(())
}

/// Configure counters and start the continuous counter scan for scaler mode.
pub fn start_scaler(
    device: &DaqDevice,
    state: &mut ScalerState,
    num_counters: usize,
) -> Result<(), String> {
    load_presets(device, state, num_counters)?;
    let num_counters = num_counters as i32;

    // Configure each counter for counting mode
    for i in 0..num_counters {
        let mode = if i == 0 {
            // Counter 0: preset control with gating
            CMM_OUTPUT_ON
                | CMM_OUTPUT_INITIAL_STATE_HIGH
                | CMM_GATING_ON
                | CMM_INVERT_GATE
                | CMM_RANGE_LIMIT_ON
                | CMM_NO_RECYCLE
        } else {
            CMM_OUTPUT_ON | CMM_OUTPUT_INITIAL_STATE_HIGH | CMM_GATING_ON | CMM_INVERT_GATE
        };

        device
            .counter_config_scan(
                i,
                &CounterScanConfig {
                    measurement_type: CMT_COUNT,
                    measurement_mode: mode,
                    edge_detection: CED_RISING_EDGE,
                    tick_size: CTS_TICK_20PT83ns,
                    debounce_mode: CDM_NONE,
                    debounce_time: CDT_DEBOUNCE_0ns,
                    flags: CF_DEFAULT,
                },
            )
            .map_err(|e| format!("counter_config_scan({i}) error: {e}"))?;
    }

    // Start continuous counter scan
    // C startScaler: 20 samples per counter at 100 Hz, continuous.
    let mut rate = 100.0;
    let samples = num_counters as usize * SAMPLES_PER_COUNTER;
    if state.scan_buffer.len() < samples {
        state.scan_buffer.resize(samples, 0);
    }

    device
        .counter_in_scan(
            &CInScanConfig {
                low_counter: 0,
                high_counter: num_counters - 1,
                samples_per_counter: SAMPLES_PER_COUNTER as i32,
                options: SO_CONTINUOUS | SO_SINGLEIO,
                flags: CINSCAN_FF_CTR64_BIT,
            },
            &mut rate,
            &mut state.scan_buffer[..samples],
        )
        .map_err(|e| format!("counter_in_scan error: {e}"))?;

    state.running = true;
    state.done = false;
    log::info!("Scaler started, rate={rate:.0} Hz");
    Ok(())
}

/// Read latest counter values from the scan buffer. Check for preset completion.
pub fn read_scaler(device: &DaqDevice, state: &mut ScalerState, num_counters: usize) {
    // C readScaler polls the scan only for its position; the scan status
    // itself never ends a count -- only a preset does.
    let xfer = match device.counter_in_scan_status() {
        Ok((_, xfer)) => xfer,
        Err(e) => {
            log::warn!("scaler scan status error: {e}");
            return;
        }
    };
    let buf_len = (num_counters * SAMPLES_PER_COUNTER).min(state.scan_buffer.len());
    let Some((counts, done)) = scan_sets(
        &state.scan_buffer[..buf_len],
        xfer.current_index,
        num_counters,
        &state.presets[..num_counters],
    ) else {
        return;
    };
    state.counts[..num_counters].copy_from_slice(&counts[..num_counters]);
    if done {
        stop_scaler(device, state);
        state.done = true;
    }
}

/// Samples per counter in the continuous ring, C `samplesPerCounter`.
const SAMPLES_PER_COUNTER: usize = 20;

/// C readScaler's walk over the ring (drvUSBCTR.cpp:970-987): from the
/// start of the buffer up to the last complete sample set at
/// `current_index`, the counts of the first set in which a preset is
/// reached -- or, if none is, of the last complete set. `None` until one
/// complete set has arrived.
fn scan_sets(
    buffer: &[u64],
    current_index: i64,
    num_counters: usize,
    presets: &[u64],
) -> Option<([u64; MAX_COUNTERS], bool)> {
    if current_index < 0 || num_counters == 0 || buffer.is_empty() {
        return None;
    }
    let num_values = current_index as usize % buffer.len() + 1;
    if num_values < num_counters {
        return None;
    }
    let last_index = (num_values / num_counters - 1) * num_counters;
    let mut counts = [0u64; MAX_COUNTERS];
    for start in (0..=last_index).step_by(num_counters) {
        counts[..num_counters].copy_from_slice(&buffer[start..start + num_counters]);
        let done = counts[..num_counters]
            .iter()
            .zip(presets)
            .any(|(count, preset)| *preset > 0 && count >= preset);
        if done {
            return Some((counts, true));
        }
    }
    Some((counts, false))
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
pub fn reset_scaler(device: &DaqDevice, state: &mut ScalerState, num_counters: usize) {
    stop_scaler(device, state);
    for i in 0..num_counters {
        state.counts[i] = 0;
        if let Err(e) = device.counter_clear(i as i32) {
            log::warn!("counter_clear({i}) error: {e}");
        }
    }
    state.done = false;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_is_read_before_one_complete_set() {
        assert_eq!(scan_sets(&[0; 8], -1, 2, &[0, 0]), None);
        assert_eq!(scan_sets(&[0; 8], 0, 2, &[0, 0]), None);
    }

    #[test]
    fn without_a_preset_the_last_complete_set_is_read() {
        let buf = [1, 10, 2, 20, 3, 30, 0, 0];
        let (counts, done) = scan_sets(&buf, 4, 2, &[0, 0]).unwrap();
        assert_eq!(&counts[..2], &[2, 20]);
        assert!(!done);
    }

    #[test]
    fn the_first_set_reaching_a_preset_ends_the_count() {
        let buf = [1, 10, 5, 20, 9, 30, 0, 0];
        let (counts, done) = scan_sets(&buf, 5, 2, &[5, 0]).unwrap();
        assert_eq!(&counts[..2], &[5, 20]);
        assert!(done);
    }
}
