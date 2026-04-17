use meascomp::device::DaqDevice;
use uldaq_sys::*;

use crate::params::*;

/// Waveform generator state.
pub struct WaveGenState {
    pub running: bool,
    pub num_chans: usize,
    pub num_points: usize,
    pub current_point: usize,
    /// Output buffer for ulAOutScan.
    pub scan_buffer: Vec<f64>,
    /// Saved output values to restore after stop.
    pub saved_outputs: [f64; MAX_ANALOG_OUT],
    pub dwell_actual: f64,
}

impl WaveGenState {
    pub fn new(max_points: usize) -> Self {
        Self {
            running: false,
            num_chans: MAX_ANALOG_OUT,
            num_points: max_points,
            current_point: 0,
            scan_buffer: Vec::new(),
            saved_outputs: [0.0; MAX_ANALOG_OUT],
            dwell_actual: 0.001,
        }
    }
}

/// Convert voltage waveform to raw 16-bit DAC units for ±10V range.
/// Matches C++ drvMultiFunction: offset=10.0, scale=65535/20.0
pub fn volts_to_dac(data: &mut [f64]) {
    const DAC_OFFSET: f64 = 10.0;       // mid-scale for ±10V
    const DAC_SCALE: f64 = 65535.0 / 20.0; // 16-bit DAC units per volt
    for v in data.iter_mut() {
        *v = ((*v + DAC_OFFSET) * DAC_SCALE + 0.5).clamp(0.0, 65535.0);
    }
}

/// Waveform type selection.
pub const WAVE_TYPE_USER: i32 = 0;
pub const WAVE_TYPE_SIN: i32 = 1;
pub const WAVE_TYPE_SQUARE: i32 = 2;
pub const WAVE_TYPE_SAWTOOTH: i32 = 3;
pub const WAVE_TYPE_PULSE: i32 = 4;
pub const WAVE_TYPE_RANDOM: i32 = 5;

/// Generate an internal waveform of the given type.
pub fn generate_waveform(
    wave_type: i32,
    num_points: usize,
    amplitude: f64,
    offset: f64,
    pulse_width: f64,
) -> Vec<f64> {
    let mut data = vec![0.0f64; num_points];
    let n = num_points as f64;

    match wave_type {
        WAVE_TYPE_SIN => {
            for i in 0..num_points {
                data[i] = offset + amplitude * (2.0 * std::f64::consts::PI * i as f64 / n).sin();
            }
        }
        WAVE_TYPE_SQUARE => {
            for i in 0..num_points {
                data[i] = if i < num_points / 2 {
                    offset + amplitude
                } else {
                    offset - amplitude
                };
            }
        }
        WAVE_TYPE_SAWTOOTH => {
            for i in 0..num_points {
                data[i] = offset + amplitude * (2.0 * i as f64 / n - 1.0);
            }
        }
        WAVE_TYPE_PULSE => {
            let pulse_samples = ((pulse_width * n) as usize).max(1).min(num_points);
            for i in 0..num_points {
                data[i] = if i < pulse_samples {
                    offset + amplitude
                } else {
                    offset
                };
            }
        }
        WAVE_TYPE_RANDOM => {
            // Simple pseudo-random using a basic LCG
            let mut seed: u64 = 12345;
            for i in 0..num_points {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let frac = (seed >> 33) as f64 / (1u64 << 31) as f64; // 0..1
                data[i] = offset + amplitude * (2.0 * frac - 1.0);
            }
        }
        _ => {
            // WAVE_TYPE_USER: return zeros, caller should provide user data
        }
    }
    data
}

/// Start the waveform generator (analog output scan).
pub fn start_wave_gen(
    device: &DaqDevice,
    state: &mut WaveGenState,
    first_chan: i32,
    last_chan: i32,
    num_points: usize,
    freq: f64,
    range: i32,
    ext_trigger: bool,
    ext_clock: bool,
    continuous: bool,
    retrigger: bool,
    waveform_data: &[f64],
    saved_outputs: &[f64],
) -> Result<(), String> {
    let num_chans = (last_chan - first_chan + 1) as usize;
    state.num_chans = num_chans;
    state.num_points = num_points;
    state.current_point = 0;

    // Save current outputs for restore on stop
    for ch in first_chan..=last_chan {
        let idx = ch as usize;
        if idx < MAX_ANALOG_OUT && idx < saved_outputs.len() {
            state.saved_outputs[idx] = saved_outputs[idx];
        }
    }

    let total = num_chans * num_points;
    state.scan_buffer = if waveform_data.len() >= total {
        waveform_data[..total].to_vec()
    } else {
        let mut buf = waveform_data.to_vec();
        buf.resize(total, 0.0);
        buf
    };

    let mut rate = freq * num_points as f64;

    let mut options = SO_DEFAULTIO;
    if ext_trigger { options |= SO_EXTTRIGGER; }
    if ext_clock { options |= SO_EXTCLOCK; }
    if continuous { options |= SO_CONTINUOUS; }
    if retrigger { options |= SO_RETRIGGER; }

    device.analog_out_scan(
        first_chan,
        last_chan,
        range,
        num_points as i32,
        &mut rate,
        options,
        AOUTSCAN_FF_NOSCALEDATA,
        &mut state.scan_buffer,
    ).map_err(|e| format!("analog_out_scan error: {e}"))?;

    state.dwell_actual = if rate > 0.0 { 1.0 / rate } else { 0.001 };
    state.running = true;

    log::info!(
        "WaveGen started: ch{first_chan}-{last_chan}, {num_points} pts, rate={rate:.0} Hz"
    );
    Ok(())
}

/// Read waveform generator status. Called from poller.
pub fn read_wave_gen(device: &DaqDevice, state: &mut WaveGenState) {
    let (status, xfer) = match device.analog_out_scan_status() {
        Ok(v) => v,
        Err(e) => {
            log::warn!("WaveGen scan status error: {e}");
            return;
        }
    };

    if state.num_chans > 0 && xfer.current_index >= 0 {
        state.current_point = (xfer.current_index as usize / state.num_chans) + 1;
    }

    if status == SS_IDLE {
        stop_wave_gen(device, state);
    }
}

/// Stop the waveform generator and restore saved output values.
pub fn stop_wave_gen(device: &DaqDevice, state: &mut WaveGenState) {
    if state.running {
        if let Err(e) = device.analog_out_scan_stop() {
            log::warn!("WaveGen scan stop error: {e}");
        }
        // Restore saved outputs
        for ch in 0..MAX_ANALOG_OUT {
            let _ = device.analog_out(ch as i32, BIP10VOLTS, AOUT_FF_NOSCALEDATA, state.saved_outputs[ch]);
        }
        state.running = false;
    }
}
