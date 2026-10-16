use meascomp::counter::{CInScanConfig, CounterScanConfig};
use meascomp::device::DaqDevice;
use uldaq_sys::*;

use epics_rs::asyn::trace::TraceMask;

use epics_rs::scaler::device_support::scaler_asyn::DoneParam;

use crate::params::*;
use crate::trace::SelfTrace;

/// Shared scaler state between the driver (arm/reset) and poller (read/done).
pub struct ScalerState {
    pub running: bool,
    /// C `scalerDone_`.
    pub done: DoneParam,
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
            // C `setIntegerParam(scalerDone_, 1)` (drvUSBCTR.cpp:417).
            done: DoneParam::new(1),
            counts: [0; MAX_COUNTERS],
            presets: [0; MAX_COUNTERS],
            scan_buffer: vec![0u64; MAX_COUNTERS * 20],
        }
    }
}

/// Configure counters and start the continuous counter scan for scaler mode.
/// C `USBCTR::startScaler` (drvUSBCTR.cpp:887), which reports each failure
/// and marks the scaler running regardless.
pub fn start_scaler(
    device: &DaqDevice,
    state: &mut ScalerState,
    num_counters: usize,
    trace: &SelfTrace,
) {
    // C writeInt32 calls setScalerPresets ahead of startScaler.
    set_scaler_presets(device, state, num_counters, trace);
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

        if let Err(e) = device.counter_config_scan(
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
        ) {
            trace.print(
                TraceMask::ERROR,
                format_args!(
                    "USBCTR::startScaler error calling ulCConfigScan, counter={i}, mode=0x{mode:x}, status={}, error={}",
                    e.code, e.message
                ),
            );
        }
    }

    // C's `long rate = 100`.
    let requested_rate = 100.0;
    let mut rate = requested_rate;
    let config = CInScanConfig {
        low_counter: 0,
        high_counter: num_counters - 1,
        samples_per_counter: 20,
        options: SO_CONTINUOUS | SO_SINGLEIO,
        flags: CINSCAN_FF_CTR64_BIT,
    };
    if let Err(e) = device.counter_in_scan(&config, &mut rate, &mut state.scan_buffer) {
        // C prints its requested `long rate`, not the one ulCInScan returned.
        trace.print(
            TraceMask::ERROR,
            format_args!(
                "USBCTR::startScaler error calling ulCInScan, firstCounter={}, lastCounter={}, samplesPerCounter={}, rate={}, options=0x{:x}, flags=0x{:x}, status={}, error={}",
                config.low_counter,
                config.high_counter,
                config.samples_per_counter,
                requested_rate as i32,
                config.options,
                config.flags,
                e.code,
                e.message
            ),
        );
    }
    state.running = true;
}

/// Load each non-zero preset as its counter's limit, and counter 0's output
/// registers so its output drops at preset 0. C `USBCTR::setScalerPresets`,
/// whose failures are reported and passed over.
fn set_scaler_presets(
    device: &DaqDevice,
    state: &ScalerState,
    num_counters: usize,
    trace: &SelfTrace,
) {
    for i in 0..num_counters {
        let preset = state.presets[i];
        if preset == 0 {
            continue;
        }
        if let Err(e) = device.counter_load(i as i32, CRT_MAX_LIMIT, preset) {
            trace.print(
                TraceMask::ERROR,
                format_args!(
                    "USBCTR::setScalerPresets error calling cbCLoad32, counter={i}, presetCounts={}, status={}, error={}",
                    preset as i32, e.code, e.message
                ),
            );
        }
    }
    if let Err(e) = device.counter_load(0, CRT_OUTPUT_VAL0, 0) {
        trace.print(
            TraceMask::ERROR,
            format_args!(
                "USBCTR::setScalerPresets error calling cbCLoad32, reg=OUTPUTVAL0REG0, value=0, status={}, error={}",
                e.code, e.message
            ),
        );
    }
    let preset0 = state.presets[0];
    if let Err(e) = device.counter_load(0, CRT_OUTPUT_VAL1, preset0) {
        trace.print(
            TraceMask::ERROR,
            format_args!(
                "USBCTR::setScalerPresets error calling cbCLoad32, reg=OUTPUTVAL1REG0, value={}, status={}, error={}",
                preset0 as i32, e.code, e.message
            ),
        );
    }
}

/// Read latest counter values from the scan buffer. Check for preset completion.
pub fn read_scaler(
    device: &DaqDevice,
    state: &mut ScalerState,
    num_counters: usize,
    trace: &SelfTrace,
) {
    let polled = device.counter_in_scan_status();
    print_get_status(trace, "readScaler", &polled);
    let Ok((status, xfer)) = polled else {
        return;
    };

    if xfer.current_total_count == 0 {
        return;
    }

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
        stop_scaler(device, state, trace);
    }
}

/// Stop the counter scan. C `stopScaler` sets `scalerDone_` whether or not
/// the scan was running.
pub fn stop_scaler(device: &DaqDevice, state: &mut ScalerState, trace: &SelfTrace) {
    if state.running {
        if let Err(e) = device.counter_in_scan_stop() {
            trace.print(
                TraceMask::ERROR,
                format_args!(
                    "USBCTR::stopScaler ERROR calling cbStopBackground(CTRFUNCTION), status={}, error={}",
                    e.code, e.message
                ),
            );
        }
        state.running = false;
    }
    state.done.set(1);
}

/// C `resetScaler`: stop a running scan and zero the counts; the hardware
/// counters are left alone.
pub fn reset_scaler(
    device: &DaqDevice,
    state: &mut ScalerState,
    num_counters: usize,
    trace: &SelfTrace,
) {
    if state.running {
        stop_scaler(device, state, trace);
    }
    state.counts[..num_counters].fill(0);
}

/// C `readScaler`/`readMCS`'s `getStatus returned` line, printed whatever the
/// status call returned.
pub fn print_get_status(
    trace: &SelfTrace,
    function_name: &str,
    polled: &meascomp::error::Result<(i32, TransferStatus)>,
) {
    let (status, scan, count, index) = match polled {
        Ok((scan, xfer)) => (
            0,
            *scan,
            xfer.current_total_count as i64,
            xfer.current_index,
        ),
        Err(e) => (e.code, 0, 0, 0),
    };
    trace.print(
        TraceMask::FLOW,
        format_args!(
            "USBCTR::{function_name} getStatus returned status={status}, ctrStatus={scan}, ctrCount={count}, ctrIndex={index}"
        ),
    );
}
