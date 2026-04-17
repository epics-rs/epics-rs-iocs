use std::time::SystemTime;

use meascomp::device::DaqDevice;
use uldaq_sys::*;

use crate::params::*;

/// Waveform digitizer state.
pub struct WaveDigState {
    pub running: bool,
    pub num_chans: usize,
    pub first_chan: usize,
    pub num_points: usize,
    pub current_point: usize,
    pub auto_restart: bool,
    /// Scan buffer (f64, allocated by ulAInScan).
    pub scan_buffer: Vec<f64>,
    /// Per-channel waveform data [channel][point].
    pub channel_buffers: Vec<Vec<f32>>,
    /// Absolute time per point.
    pub abs_time_buffer: Vec<f64>,
    /// Time waveform per point.
    pub time_buffer: Vec<f32>,
    pub dwell_actual: f64,
    // Saved parameters for auto-restart
    pub input_mode: i32,
    pub range: i32,
    pub options: i32,
}

impl WaveDigState {
    pub fn new(max_points: usize) -> Self {
        let mut channel_buffers = Vec::with_capacity(MAX_ANALOG_IN);
        for _ in 0..MAX_ANALOG_IN {
            channel_buffers.push(vec![0.0f32; max_points]);
        }
        Self {
            running: false,
            num_chans: MAX_ANALOG_IN,
            first_chan: 0,
            num_points: max_points,
            current_point: 0,
            auto_restart: false,
            scan_buffer: Vec::new(),
            channel_buffers,
            abs_time_buffer: vec![0.0; max_points],
            time_buffer: vec![0.0; max_points],
            dwell_actual: 0.001,
            input_mode: AI_DIFFERENTIAL,
            range: BIP10VOLTS,
            options: SO_DEFAULTIO,
        }
    }
}

/// Start the waveform digitizer (analog input scan).
pub fn start_wave_dig(
    device: &DaqDevice,
    state: &mut WaveDigState,
    first_chan: usize,
    num_chans: usize,
    num_points: usize,
    dwell: f64,
    input_mode: i32,
    range: i32,
    ext_trigger: bool,
    ext_clock: bool,
    continuous: bool,
    retrigger: bool,
    burst_mode: bool,
) -> Result<(), String> {
    state.first_chan = first_chan;
    state.num_chans = num_chans;
    state.num_points = num_points;
    state.current_point = 0;

    let total_samples = num_chans * num_points;
    state.scan_buffer.resize(total_samples, 0.0);

    // Load input queue for multi-channel scanning (required by uldaq)
    let mut queue = Vec::with_capacity(num_chans);
    for i in 0..num_chans {
        queue.push(AiQueueElement {
            channel: (first_chan + i) as i32,
            input_mode,
            range,
            ..AiQueueElement::default()
        });
    }
    device.analog_in_load_queue(&queue)
        .map_err(|e| format!("analog_in_load_queue error: {e}"))?;

    let mut rate = if dwell > 0.0 { 1.0 / dwell } else { 1000.0 };

    let mut options = SO_DEFAULTIO;
    if ext_trigger { options |= SO_EXTTRIGGER; }
    if ext_clock { options |= SO_EXTCLOCK; }
    if continuous { options |= SO_CONTINUOUS; }
    if retrigger { options |= SO_RETRIGGER; }
    if burst_mode { options |= SO_BURSTMODE; }

    // Save for auto-restart
    state.input_mode = input_mode;
    state.range = range;
    state.options = options;

    device.analog_in_scan(
        first_chan as i32,
        (first_chan + num_chans - 1) as i32,
        input_mode,
        range,
        num_points as i32,
        &mut rate,
        options,
        AINSCAN_FF_DEFAULT,
        &mut state.scan_buffer,
    ).map_err(|e| format!("analog_in_scan error: {e}"))?;

    state.dwell_actual = 1.0 / rate;
    state.running = true;

    // Compute time waveform
    for i in 0..num_points {
        state.time_buffer[i] = (i as f64 * state.dwell_actual) as f32;
    }

    log::info!(
        "WaveDig started: ch{first_chan}-{}, {num_points} pts, rate={rate:.0} Hz",
        first_chan + num_chans - 1
    );
    Ok(())
}

/// Read waveform digitizer data from scan buffer. Called from poller.
pub fn read_wave_dig(device: &DaqDevice, state: &mut WaveDigState) {
    let (status, xfer) = match device.analog_in_scan_status() {
        Ok(v) => v,
        Err(e) => {
            log::warn!("WaveDig scan status error: {e}");
            return;
        }
    };

    if xfer.current_total_count == 0 {
        return;
    }

    let n_chans = state.num_chans;
    if n_chans == 0 || xfer.current_index < 0 { return; }

    let last_point = ((xfer.current_index as usize + 1) / n_chans).min(state.num_points);
    let now = current_time_secs();

    // Copy new data
    while state.current_point < last_point {
        let buf_offset = state.current_point * n_chans;
        for j in 0..n_chans {
            let ch = state.first_chan + j;
            if ch < MAX_ANALOG_IN {
                state.channel_buffers[ch][state.current_point] =
                    state.scan_buffer[buf_offset + j] as f32;
            }
        }
        state.abs_time_buffer[state.current_point] = now;
        state.current_point += 1;
    }

    if status == SS_IDLE {
        state.running = false;
        if state.auto_restart {
            state.current_point = 0;
            // Reload queue with saved parameters
            let mut queue = Vec::with_capacity(state.num_chans);
            for i in 0..state.num_chans {
                queue.push(AiQueueElement {
                    channel: (state.first_chan + i) as i32,
                    input_mode: state.input_mode,
                    range: state.range,
                    ..AiQueueElement::default()
                });
            }
            if let Err(e) = device.analog_in_load_queue(&queue) {
                log::warn!("WaveDig auto-restart: queue reload failed: {e}");
            } else {
                // Restart scan with saved options
                let mut rate = if state.dwell_actual > 0.0 { 1.0 / state.dwell_actual } else { 1000.0 };
                match device.analog_in_scan(
                    state.first_chan as i32,
                    (state.first_chan + state.num_chans - 1) as i32,
                    state.input_mode,
                    state.range,
                    state.num_points as i32,
                    &mut rate,
                    state.options,
                    AINSCAN_FF_DEFAULT,
                    &mut state.scan_buffer,
                ) {
                    Ok(()) => { state.running = true; }
                    Err(e) => log::warn!("WaveDig auto-restart: scan failed: {e}"),
                }
            }
        }
    }
}

/// Stop the waveform digitizer.
pub fn stop_wave_dig(device: &DaqDevice, state: &mut WaveDigState) {
    if state.running {
        if let Err(e) = device.analog_in_scan_stop() {
            log::warn!("WaveDig scan stop error: {e}");
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
