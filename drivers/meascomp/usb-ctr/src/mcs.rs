use std::time::SystemTime;

use meascomp::counter::CounterScanConfig;
use meascomp::device::DaqDevice;
use uldaq_sys::*;

use crate::params::*;

/// MCS (Multi-Channel Scaler) acquisition state.
pub struct McsState {
    pub running: bool,
    pub acquiring: bool,
    pub num_counters_enabled: usize,
    pub max_points: usize,
    pub current_point: usize,
    pub start_time: f64,
    pub preset_real_time: f64,
    pub dwell_time: f64,

    /// Scan buffer for ulDaqInScan (f64 values).
    pub scan_buffer: Vec<f64>,
    /// Per-counter MCS data [counter][point].
    pub mcs_buffers: Vec<Vec<i32>>,
    /// Absolute time per point.
    pub abs_time_buffer: Vec<f64>,
    /// Time waveform per point.
    pub time_buffer: Vec<f32>,
    /// Which counters are enabled (bitmask).
    pub counter_enable: u32,
    /// Mapping from scan channel index to counter number.
    pub chan_map: Vec<usize>,
}

impl McsState {
    pub fn new(max_points: usize) -> Self {
        let mut mcs_buffers = Vec::with_capacity(MAX_MCS_COUNTERS);
        for _ in 0..MAX_MCS_COUNTERS {
            mcs_buffers.push(vec![0i32; max_points]);
        }
        Self {
            running: false,
            acquiring: false,
            num_counters_enabled: 0,
            max_points,
            current_point: 0,
            start_time: 0.0,
            preset_real_time: 0.0,
            dwell_time: 0.001,
            scan_buffer: Vec::new(),
            mcs_buffers,
            abs_time_buffer: vec![0.0; max_points],
            time_buffer: vec![0.0; max_points],
            counter_enable: 0x1FF, // all 9 enabled by default
            chan_map: Vec::new(),
        }
    }
}

/// Erase all MCS buffers.
pub fn erase_mcs(state: &mut McsState) {
    for buf in &mut state.mcs_buffers {
        buf.iter_mut().for_each(|v| *v = 0);
    }
    state.abs_time_buffer.iter_mut().for_each(|v| *v = 0.0);
    state.time_buffer.iter_mut().for_each(|v| *v = 0.0);
    state.current_point = 0;
}

/// Compute the time waveform based on dwell time.
pub fn compute_times(state: &mut McsState) {
    for i in 0..state.max_points {
        state.time_buffer[i] = (i as f64 * state.dwell_time) as f32;
    }
}

/// Acquisition settings for [`start_mcs`], read from the MCA/MCS records.
#[derive(Clone, Copy, Debug)]
pub struct McsScan {
    pub num_points: usize,
    pub dwell_time: f64,
    pub counter_enable: u32,
    pub ch_advance_source: i32,
    /// Accepted but not implemented (matches C++ drvUSBCTR, which ignores
    /// prescale for MCS).
    pub prescale: i32,
    pub ext_trigger: bool,
    pub point0_no_clear: bool,
}

/// Start MCS acquisition using DaqInScan.
pub fn start_mcs(device: &DaqDevice, state: &mut McsState, scan: &McsScan) -> Result<(), String> {
    let McsScan {
        num_points,
        dwell_time,
        counter_enable,
        ch_advance_source,
        prescale: _,
        ext_trigger,
        point0_no_clear,
    } = *scan;
    state.dwell_time = dwell_time;
    state.counter_enable = counter_enable;
    let max_pts = state.max_points.min(num_points);

    // Build channel descriptor list from enabled counters
    let mut chan_descs = Vec::new();
    let mut chan_map = Vec::new();

    for i in 0..MAX_MCS_COUNTERS {
        if counter_enable & (1 << i) != 0 {
            // Configure counter for MCS (matches C++ drvUSBCTR startMCS)
            let mode = CMM_OUTPUT_ON
                | CMM_OUTPUT_INITIAL_STATE_HIGH
                | CMM_CLEAR_ON_READ
                | CMM_GATING_ON
                | CMM_INVERT_GATE;
            if i < MAX_COUNTERS {
                device
                    .counter_config_scan(
                        i as i32,
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

            let (channel, chan_type) = if i == DIGITAL_IO_COUNTER {
                (AUXPORT, DAQI_DIGITAL)
            } else {
                (i as i32, DAQI_CTR32)
            };

            chan_descs.push(DaqInChanDescriptor {
                channel,
                chan_type,
                range: 0,
                ..DaqInChanDescriptor::default()
            });
            chan_map.push(i);
        }
    }

    state.num_counters_enabled = chan_descs.len();
    state.chan_map = chan_map;

    // Allocate scan buffer
    let total_samples = state.num_counters_enabled * max_pts;
    state.scan_buffer.resize(total_samples, 0.0);

    // Rate = 1/dwell
    let mut rate = if dwell_time > 0.0 {
        1.0 / dwell_time
    } else {
        1000.0
    };

    let mut options = SO_SINGLEIO;
    if ch_advance_source != 0 {
        options |= SO_EXTCLOCK;
    }
    if ext_trigger {
        options |= SO_EXTTRIGGER;
    }

    let mut flags = DAQINSCAN_FF_DEFAULT;
    if point0_no_clear {
        flags |= DAQINSCAN_FF_NOCLEAR;
    }

    // Clear counter 0 output registers to prevent scaler presets from
    // interfering with MCS acquisition (matches C++ drvUSBCTR.cpp).
    let _ = device.counter_load(0, CRT_OUTPUT_VAL0, 0);
    let _ = device.counter_load(0, CRT_OUTPUT_VAL1, 0xFFFFFFFF);

    device
        .daq_in_scan(
            &chan_descs,
            max_pts as i32,
            &mut rate,
            options,
            flags,
            &mut state.scan_buffer,
        )
        .map_err(|e| format!("daq_in_scan error: {e}"))?;

    state.running = true;
    state.acquiring = true;
    state.current_point = 0;
    state.start_time = current_time_secs();

    compute_times(state);
    log::info!(
        "MCS started: {} counters, {} points, dwell={dwell_time:.6}s, rate={rate:.0}",
        state.num_counters_enabled,
        max_pts
    );
    Ok(())
}

/// Read MCS data from the scan buffer. Called from poller when running.
pub fn read_mcs(device: &DaqDevice, state: &mut McsState) {
    let (status, xfer) = match device.daq_in_scan_status() {
        Ok(v) => v,
        Err(e) => {
            log::warn!("MCS scan status error: {e}");
            return;
        }
    };

    if xfer.current_total_count == 0 {
        return;
    }

    let n_chans = state.num_counters_enabled;
    if n_chans == 0 || xfer.current_index < 0 {
        return;
    }

    let last_point = (xfer.current_index as usize / n_chans + 1).min(state.max_points);
    let now = current_time_secs();

    // Copy new data points
    while state.current_point < last_point {
        let buf_offset = state.current_point * n_chans;
        for (scan_idx, &ctr_idx) in state.chan_map.iter().enumerate() {
            state.mcs_buffers[ctr_idx][state.current_point] =
                state.scan_buffer[buf_offset + scan_idx] as i32;
        }
        state.abs_time_buffer[state.current_point] = now;
        state.current_point += 1;
    }

    // Check if done
    let elapsed = now - state.start_time;
    let done = status == SS_IDLE
        || state.current_point >= state.max_points
        || (state.preset_real_time > 0.0 && elapsed >= state.preset_real_time);

    if done {
        stop_mcs(device, state);
        state.acquiring = false;
    }
}

/// Stop MCS acquisition.
pub fn stop_mcs(device: &DaqDevice, state: &mut McsState) {
    if state.running {
        if let Err(e) = device.daq_in_scan_stop() {
            log::warn!("MCS daq_in_scan_stop error: {e}");
        }
        state.running = false;
    }
}

fn current_time_secs() -> f64 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}
