use meascomp::analog_out::AOutScanConfig;
use meascomp::device::DaqDevice;
use uldaq_sys::*;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::trace::TraceMask;

use crate::params::*;
use crate::trace::{DRIVER_NAME, SelfTrace};

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
    /// C `waveGenUserBuffer_`: per channel, `max_points` volts; a
    /// WAVEGEN_USER_WF write overwrites its first elements and keeps the rest.
    pub user_waveforms: Vec<Vec<f32>>,
    /// C `waveGenIntBuffer_`: per channel, the waveform `define_waveform`
    /// last built.
    pub int_waveforms: Vec<Vec<f32>>,
    /// Capacity the port was configured with (C `maxOutputPoints_`).
    pub max_points: usize,
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
            user_waveforms: vec![vec![0.0; max_points]; MAX_ANALOG_OUT],
            int_waveforms: vec![vec![0.0; max_points]; MAX_ANALOG_OUT],
            max_points,
        }
    }
}

/// C `startWaveGen`'s volts-to-D/A conversion: 16-bit DAC, ±10 V, with a
/// user waveform's amplitude and offset applied first.
pub fn volts_to_dac(volts: f32, amplitude: f64, offset: f64) -> f64 {
    const DAC_OFFSET: f64 = 10.0; // mid-scale for ±10V
    const DAC_SCALE: f64 = 65535.0 / 20.0; // 16-bit DAC units per volt
    // C casts to epicsUInt16 after the +0.5: the nearest count, which the
    // clamp keeps in range where the C cast would be undefined.
    ((f64::from(volts) * amplitude + offset + DAC_OFFSET) * DAC_SCALE + 0.5)
        .floor()
        .clamp(0.0, 65535.0)
}

/// Waveform type selection.
pub const WAVE_TYPE_USER: i32 = 0;
pub const WAVE_TYPE_SIN: i32 = 1;
pub const WAVE_TYPE_SQUARE: i32 = 2;
pub const WAVE_TYPE_SAWTOOTH: i32 = 3;
pub const WAVE_TYPE_PULSE: i32 = 4;
pub const WAVE_TYPE_RANDOM: i32 = 5;

/// One channel's internal waveform settings (C `defineWaveform`'s reads).
#[derive(Clone, Copy, Debug)]
pub struct InternalWave {
    pub wave_type: i32,
    pub num_points: usize,
    pub dwell: f64,
    pub offset: f64,
    pub amplitude: f64,
    pub pulse_width: f64,
    pub pulse_delay: f64,
}

/// C `defineWaveform`'s internal shapes, written over the first
/// `num_points` of `out` (which holds at least that many).
pub fn define_internal(out: &mut [f32], wave: &InternalWave) {
    let InternalWave {
        wave_type,
        num_points,
        dwell,
        offset,
        amplitude,
        pulse_width,
        pulse_delay,
    } = *wave;
    let n = num_points as i64;
    let out = &mut out[..num_points];
    let base = offset - amplitude / 2.;
    match wave_type {
        WAVE_TYPE_SIN => {
            let scale = 2. * std::f64::consts::PI / (n - 1) as f64;
            for (i, v) in out.iter_mut().enumerate() {
                *v = (offset + amplitude / 2. * (i as f64 * scale).sin()) as f32;
            }
        }
        WAVE_TYPE_SQUARE => {
            for (i, v) in out.iter_mut().enumerate() {
                let high = (i as i64) < n / 2;
                *v = (if high { base + amplitude } else { base }) as f32;
            }
        }
        WAVE_TYPE_SAWTOOTH => {
            let scale = 1. / (n - 1) as f64;
            for (i, v) in out.iter_mut().enumerate() {
                *v = (base + amplitude * i as f64 * scale) as f32;
            }
        }
        WAVE_TYPE_PULSE => {
            let mut n_pulse = ((pulse_width / dwell) + 0.5) as i64;
            let mut n_delay = ((pulse_delay / dwell) + 0.5) as i64;
            if n_pulse < 1 {
                n_pulse = 1;
            }
            if n_pulse >= n - 1 {
                n_pulse = n - 1;
            }
            if n_delay + n_pulse >= n - 1 {
                n_delay = n - n_pulse - 1;
            }
            if n_delay < 0 {
                n_delay = 0;
            }
            for (i, v) in out.iter_mut().enumerate() {
                let i = i as i64;
                let high = i >= n_delay && i < n_delay + n_pulse;
                *v = (if high { offset + amplitude } else { offset }) as f32;
            }
        }
        WAVE_TYPE_RANDOM => {
            let scale = amplitude / f64::from(GLIBC_RAND_MAX);
            let mut rand = GlibcRand::seeded(1);
            for v in out.iter_mut() {
                *v = (base + f64::from(rand.next()) * scale) as f32;
            }
        }
        // C leaves the buffer as it was for any other type.
        _ => {}
    }
}

const GLIBC_RAND_MAX: i32 = 2147483647;

/// glibc `srand(seed)` / `rand()` (the TYPE_3 additive feedback
/// generator), which C's random waveform draws from after `srand(1)`.
struct GlibcRand {
    r: Vec<i32>,
}

impl GlibcRand {
    fn seeded(seed: u32) -> Self {
        let mut r = Vec::with_capacity(344 + 64);
        let mut word = if seed == 0 { 1 } else { seed as i32 };
        r.push(word);
        for _ in 1..31 {
            let hi = word / 127773;
            let lo = word % 127773;
            word = 16807 * lo - 2836 * hi;
            if word < 0 {
                word += GLIBC_RAND_MAX;
            }
            r.push(word);
        }
        for i in 31..34 {
            r.push(r[i - 31]);
        }
        let mut rand = Self { r };
        for _ in 34..344 {
            rand.step();
        }
        rand
    }

    fn step(&mut self) -> i32 {
        let i = self.r.len();
        let v = self.r[i - 31].wrapping_add(self.r[i - 3]);
        self.r.push(v);
        v
    }

    fn next(&mut self) -> i32 {
        ((self.step() as u32) >> 1) as i32
    }
}

/// Generation settings for [`start_wave_gen`], read from the WaveGen records.
#[derive(Clone, Copy, Debug)]
pub struct WaveGenScan {
    pub first_chan: i32,
    pub last_chan: i32,
    pub num_points: usize,
    pub freq: f64,
    pub range: i32,
    pub ext_trigger: bool,
    pub ext_clock: bool,
    pub continuous: bool,
    pub retrigger: bool,
}

/// Start the waveform generator (analog output scan). C `startWaveGen`
/// from `ulAOutScan` on; the error is the uldaq status it reported.
pub fn start_wave_gen(
    device: &DaqDevice,
    state: &mut WaveGenState,
    scan: &WaveGenScan,
    waveform_data: &[f64],
    saved_outputs: &[f64],
    trace: &SelfTrace,
) -> Result<(), UlError> {
    const FUNCTION: &str = "startWaveGen";
    let WaveGenScan {
        first_chan,
        last_chan,
        num_points,
        freq,
        range,
        ext_trigger,
        ext_clock,
        continuous,
        retrigger,
    } = *scan;
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
    if ext_trigger {
        options |= SO_EXTTRIGGER;
    }
    if ext_clock {
        options |= SO_EXTCLOCK;
    }
    if continuous {
        options |= SO_CONTINUOUS;
    }
    if retrigger {
        options |= SO_RETRIGGER;
    }

    let result = device.analog_out_scan(
        &AOutScanConfig {
            low_chan: first_chan,
            high_chan: last_chan,
            range,
            samples_per_chan: num_points as i32,
            options,
            flags: AOUTSCAN_FF_NOSCALEDATA,
        },
        &mut rate,
        &mut state.scan_buffer,
    );
    let status = trace.report(&result, FUNCTION, "Calling AOutScan");
    if status != 0 {
        return Err(status);
    }

    state.dwell_actual = if rate > 0.0 { 1.0 / rate } else { 0.001 };
    state.running = true;
    trace.print(
        TraceMask::FLOW,
        format_args!(
            "{DRIVER_NAME}:{FUNCTION}: called cbAOutScan, firstChan={first_chan}, lastChan={last_chan}, numPoints*numWaveGenChans_={}, dwell={:.6}, options=0x{options:x}",
            num_chans * num_points,
            state.dwell_actual
        ),
    );
    Ok(())
}

/// The generator half of C `pollerThread`. `Err` is the failed
/// `ulAOutScanStatus` status, for the poller's once-per-outage report.
pub fn read_wave_gen(
    device: &DaqDevice,
    state: &mut WaveGenState,
    restore: &[(i32, i32)],
    trace: &SelfTrace,
) -> Result<(), UlError> {
    let (status, xfer) = device.analog_out_scan_status().map_err(|e| e.code)?;
    trace.print(
        TraceMask::IO_DRIVER,
        format_args!(
            "{DRIVER_NAME}::pollerThread waveform generator status, aoStatus={status}, aoCount={}, aoIndex={}",
            xfer.current_total_count, xfer.current_index
        ),
    );

    if state.num_chans > 0 && xfer.current_index >= 0 {
        state.current_point = (xfer.current_index as usize / state.num_chans) + 1;
    }

    if status == SS_IDLE {
        stop_wave_gen(device, state, restore, trace);
    }
    Ok(())
}

/// Stop the waveform generator and restore saved output values. C
/// `stopWaveGen`; returns the `ulAOutScanStop` status, which C does not
/// print.
pub fn stop_wave_gen(
    device: &DaqDevice,
    state: &mut WaveGenState,
    restore: &[(i32, i32)],
    trace: &SelfTrace,
) -> UlError {
    state.running = false;
    let err = device.analog_out_scan_stop().err().map_or(0, |e| e.code);
    for &(ch, range) in restore {
        trace.report(
            &device.analog_out(
                ch,
                range,
                AOUT_FF_NOSCALEDATA,
                state.saved_outputs[ch as usize],
            ),
            "stopWaveGen",
            "calling AOut",
        );
    }
    err
}

/// The outputs C `stopWaveGen` puts back: each enabled channel, with its
/// ANALOG_OUT_RANGE.
pub fn restore_outputs(
    params: &MultiFunctionParams,
    int: impl Fn(usize, i32) -> AsynResult<i32>,
) -> AsynResult<Vec<(i32, i32)>> {
    let mut restore = Vec::new();
    for ch in 0..MAX_ANALOG_OUT as i32 {
        if int(params.wave_gen_enable, ch)? != 0 {
            restore.push((ch, int(params.analog_out_range, ch)?));
        }
    }
    Ok(restore)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wave(wave_type: i32, num_points: usize) -> InternalWave {
        InternalWave {
            wave_type,
            num_points,
            dwell: 1.0,
            offset: 1.0,
            amplitude: 2.0,
            pulse_width: 2.0,
            pulse_delay: 1.0,
        }
    }

    fn defined(wave: &InternalWave) -> Vec<f32> {
        let mut out = vec![-7.0; wave.num_points + 1];
        define_internal(&mut out, wave);
        out
    }

    #[test]
    fn dac_conversion_spans_the_bipolar_range() {
        assert_eq!(volts_to_dac(-10.0, 1.0, 0.0), 0.0);
        assert_eq!(volts_to_dac(0.0, 1.0, 0.0), 32768.0);
        assert_eq!(volts_to_dac(10.0, 1.0, 0.0), 65535.0);
        // A user waveform's amplitude and offset come first.
        assert_eq!(volts_to_dac(1.0, 2.0, -2.0), 32768.0);
    }

    #[test]
    fn dac_conversion_clamps_beyond_the_range() {
        assert_eq!(volts_to_dac(-25.0, 1.0, 0.0), 0.0);
        assert_eq!(volts_to_dac(25.0, 1.0, 0.0), 65535.0);
    }

    #[test]
    fn a_square_wave_is_base_plus_amplitude_then_base() {
        // base = offset - amplitude/2 = 0.
        assert_eq!(
            defined(&wave(WAVE_TYPE_SQUARE, 4)),
            vec![2.0, 2.0, 0.0, 0.0, -7.0]
        );
    }

    #[test]
    fn a_sawtooth_climbs_from_base_to_base_plus_amplitude() {
        assert_eq!(
            defined(&wave(WAVE_TYPE_SAWTOOTH, 3)),
            vec![0.0, 1.0, 2.0, -7.0]
        );
    }

    #[test]
    fn a_sine_is_half_amplitude_about_the_offset_over_n_minus_one() {
        let out = defined(&wave(WAVE_TYPE_SIN, 5));
        assert_eq!(out[0], 1.0);
        assert!((out[1] - 2.0).abs() < 1e-6);
        assert!((out[3] - 0.0).abs() < 1e-6);
        assert!((out[4] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn a_pulse_is_delayed_and_counted_in_dwells() {
        // width 2 dwells, delay 1 dwell, over 6 points.
        assert_eq!(
            defined(&wave(WAVE_TYPE_PULSE, 6)),
            vec![1.0, 3.0, 3.0, 1.0, 1.0, 1.0, -7.0]
        );
        // A pulse that would run off the end is pulled back to end one short.
        let mut late = wave(WAVE_TYPE_PULSE, 4);
        late.pulse_delay = 5.0;
        assert_eq!(defined(&late), vec![1.0, 3.0, 3.0, 1.0, -7.0]);
    }

    #[test]
    fn the_random_wave_is_glibcs_srand_1_sequence() {
        let mut rand = GlibcRand::seeded(1);
        assert_eq!(
            [rand.next(), rand.next(), rand.next()],
            [1804289383, 846930886, 1681692777]
        );
    }

    #[test]
    fn an_unknown_type_leaves_the_buffer_alone() {
        assert_eq!(defined(&wave(WAVE_TYPE_USER, 2)), vec![-7.0; 3]);
    }
}
