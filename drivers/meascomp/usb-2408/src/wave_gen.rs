use meascomp::analog_out::AOutScanConfig;
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
    /// DAC value of each channel the running scan drives, saved at start and
    /// written back at stop. `None` for a channel not in the scan, which the
    /// stop therefore never touches.
    pub saved_outputs: [Option<f64>; MAX_ANALOG_OUT],
    pub dwell_actual: f64,
    /// Per-channel user-defined waveform in volts, C `waveGenUserBuffer_`:
    /// `max_points` long and zero until written; a write replaces only the
    /// points it carries. Used when a channel's WAVEGEN_WAVE_TYPE is `User`.
    pub user_buffers: Vec<Vec<f32>>,
    /// Capacity the port was configured with; a user waveform is truncated to
    /// it so it can never outrun the scan buffer.
    pub max_points: usize,
    /// Per-channel internal waveform in volts, C `waveGenIntBuffer_`, as the
    /// last `define_waveform` computed it.
    pub int_buffers: Vec<Vec<f32>>,
    /// Relative time bases for the user-defined and internal timing pairs,
    /// C `waveGenUserTimeBuffer_` / `waveGenIntTimeBuffer_`.
    pub user_time_buffer: Vec<f32>,
    pub int_time_buffer: Vec<f32>,
    /// Bumped by every start, so a report that an earlier scan ended can
    /// never end a later one.
    pub generation: u64,
}

impl WaveGenState {
    pub fn new(max_points: usize) -> Self {
        Self {
            running: false,
            num_chans: MAX_ANALOG_OUT,
            num_points: max_points,
            current_point: 0,
            scan_buffer: Vec::new(),
            saved_outputs: [None; MAX_ANALOG_OUT],
            dwell_actual: 0.001,
            user_buffers: vec![vec![0.0; max_points]; MAX_ANALOG_OUT],
            max_points,
            int_buffers: vec![vec![0.0; max_points]; MAX_ANALOG_OUT],
            user_time_buffer: vec![0.0; max_points],
            int_time_buffer: vec![0.0; max_points],
            generation: 0,
        }
    }
}

/// Convert voltage waveform to raw 16-bit DAC units for ±10V range.
/// Matches C++ drvMultiFunction: offset=10.0, scale=65535/20.0
pub fn volts_to_dac(data: &mut [f64]) {
    const DAC_OFFSET: f64 = 10.0; // mid-scale for ±10V
    const DAC_SCALE: f64 = 65535.0 / 20.0; // 16-bit DAC units per volt
    for v in data.iter_mut() {
        // C casts to epicsUInt16 after the +0.5, i.e. rounds to the nearest
        // DAC count. Without the truncation every sample kept a half-count
        // bias and the clamp was applied to the un-rounded value.
        *v = ((*v + DAC_OFFSET) * DAC_SCALE + 0.5)
            .floor()
            .clamp(0.0, 65535.0);
    }
}

/// Waveform type selection.
pub const WAVE_TYPE_USER: i32 = 0;
pub const WAVE_TYPE_SIN: i32 = 1;
pub const WAVE_TYPE_SQUARE: i32 = 2;
pub const WAVE_TYPE_SAWTOOTH: i32 = 3;
pub const WAVE_TYPE_PULSE: i32 = 4;
pub const WAVE_TYPE_RANDOM: i32 = 5;

/// One channel's internal-waveform settings, read from its WaveGen records.
#[derive(Clone, Copy, Debug)]
pub struct WaveShape {
    pub wave_type: i32,
    /// Peak-to-peak volts.
    pub amplitude: f64,
    pub offset: f64,
    /// Pulse width and delay, in seconds.
    pub pulse_width: f64,
    pub pulse_delay: f64,
    /// WAVEGEN_INT_DWELL: the pulse times are counted in samples of it.
    pub dwell: f64,
}

/// Generate an internal waveform of the given type.
///
/// `amplitude` is peak-to-peak, as C `defineWaveform` takes it: sin, square,
/// sawtooth and random span `offset +/- amplitude/2`; the pulse goes from
/// `offset` to `offset + amplitude`. Samples are `f32`, C's
/// `waveGenIntBuffer_` precision, so the DAC codes they convert to match.
pub fn generate_waveform(shape: &WaveShape, num_points: usize) -> Vec<f32> {
    let WaveShape {
        wave_type,
        amplitude,
        offset,
        pulse_width,
        pulse_delay,
        dwell,
    } = *shape;
    let mut data = vec![0.0f32; num_points];
    let base = offset - amplitude / 2.0;
    // C divides by numPoints-1 so the sine's last point closes the period and
    // the sawtooth ends exactly at base + amplitude; a 1-point waveform keeps
    // a well-defined 0 phase instead of C's 0/0.
    let span = num_points.saturating_sub(1).max(1) as f64;

    match wave_type {
        WAVE_TYPE_SIN => {
            let scale = 2.0 * std::f64::consts::PI / span;
            for (i, d) in data.iter_mut().enumerate() {
                *d = (offset + amplitude / 2.0 * (i as f64 * scale).sin()) as f32;
            }
        }
        WAVE_TYPE_SQUARE => {
            for (i, d) in data.iter_mut().enumerate() {
                *d = if i < num_points / 2 {
                    (base + amplitude) as f32
                } else {
                    base as f32
                };
            }
        }
        WAVE_TYPE_SAWTOOTH => {
            let scale = 1.0 / span;
            for (i, d) in data.iter_mut().enumerate() {
                *d = (base + amplitude * i as f64 * scale) as f32;
            }
        }
        WAVE_TYPE_PULSE => {
            // C: width and delay are times, rounded to whole samples of the
            // internal dwell, and at least one sample is left low
            // (drvMultiFunction.cpp:1556-1566).
            let n = num_points as i64;
            let mut n_pulse = (pulse_width / dwell + 0.5) as i64;
            let mut n_delay = (pulse_delay / dwell + 0.5) as i64;
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
            for (i, d) in data.iter_mut().enumerate() {
                let i = i as i64;
                *d = if i >= n_delay && i < n_delay + n_pulse {
                    (offset + amplitude) as f32
                } else {
                    offset as f32
                };
            }
        }
        WAVE_TYPE_RANDOM => {
            // C: srand(1) then rand() per point, so every run -- and every
            // C IOC -- emits the same sequence.
            let mut rng = GlibcRand::new(1);
            let scale = amplitude / GlibcRand::RAND_MAX as f64;
            for d in &mut data {
                *d = (base + rng.next() as f64 * scale) as f32;
            }
        }
        _ => {
            // WAVE_TYPE_USER: the caller fills this in from the channel's
            // WAVEGEN_USER_WF; zeros only if nothing was ever written.
        }
    }
    data
}

/// glibc's `rand()` (the TYPE_3 additive-feedback generator `srand` seeds),
/// the one C `defineWaveform` draws its random waveform from on Linux.
struct GlibcRand {
    r: [u32; 34],
    i: usize,
}

impl GlibcRand {
    const RAND_MAX: u32 = 0x7FFF_FFFF;

    fn new(seed: u32) -> Self {
        let mut r = [0u32; 34];
        r[0] = seed;
        for i in 1..31 {
            // r[i] = (16807 * r[i-1]) % 2147483647, as a signed 32-bit word.
            let v = (16807 * r[i - 1] as i32 as i64) % 2_147_483_647;
            r[i] = if v < 0 { v + 2_147_483_647 } else { v } as u32;
        }
        for i in 31..34 {
            r[i] = r[i - 31];
        }
        let mut rng = Self { r, i: 0 };
        // glibc discards the first 310 outputs of a fresh seed.
        for _ in 0..310 {
            rng.step();
        }
        rng
    }

    /// r[k] = r[k-31] + r[k-3] over a 34-word ring.
    fn step(&mut self) -> u32 {
        let k = self.i;
        let v = self.r[(k + 34 - 31) % 34].wrapping_add(self.r[(k + 34 - 3) % 34]);
        self.r[k % 34] = v;
        self.i = (k + 1) % 34;
        v
    }

    fn next(&mut self) -> u32 {
        self.step() >> 1
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

/// Start the waveform generator (analog output scan).
pub fn start_wave_gen(
    device: &DaqDevice,
    state: &mut WaveGenState,
    scan: &WaveGenScan,
    waveform_data: &[f64],
    saved_outputs: [Option<f64>; MAX_ANALOG_OUT],
) -> Result<(), String> {
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

    // The outputs to put back when the scan ends.
    state.saved_outputs = saved_outputs;

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

    device
        .analog_out_scan(
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
        )
        .map_err(|e| format!("analog_out_scan error: {e}"))?;

    state.dwell_actual = if rate > 0.0 { 1.0 / rate } else { 0.001 };
    state.running = true;
    state.generation = state.generation.wrapping_add(1);

    log::info!("WaveGen started: ch{first_chan}-{last_chan}, {num_points} pts, rate={rate:.0} Hz");
    Ok(())
}

/// C pollerThread's generator block: the current point, and whether the scan
/// has gone idle. Ending it (Run back to 0, the outputs put back) is the
/// driver's transition, not this read's.
pub fn read_wave_gen(device: &DaqDevice, state: &mut WaveGenState) -> bool {
    // C skips the rest of the cycle on a status error (goto error).
    let report = device.analog_out_scan_status();
    if let Some(e) = report.error {
        log::warn!("WaveGen scan status error: {e}");
        return false;
    }
    let (status, xfer) = (report.status, report.xfer);

    if state.num_chans > 0 && xfer.current_index >= 0 {
        state.current_point = (xfer.current_index as usize / state.num_chans) + 1;
    }

    status == SS_IDLE
}

/// Stop the waveform generator and restore saved output values.
pub fn stop_wave_gen(device: &DaqDevice, state: &mut WaveGenState) {
    if state.running {
        if let Err(e) = device.analog_out_scan_stop() {
            log::warn!("WaveGen scan stop error: {e}");
        }
        // C stopWaveGen puts back only the channels the scan drove
        // (drvMultiFunction.cpp:1721-1733); any other output keeps its value.
        for (ch, saved) in state.saved_outputs.iter_mut().enumerate() {
            if let Some(value) = saved.take()
                && let Err(e) = device.analog_out(ch as i32, BIP10VOLTS, AOUT_FF_NOSCALEDATA, value)
            {
                log::warn!("WaveGen restore AO{ch} error: {e}");
            }
        }
        state.running = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shape(
        wave_type: i32,
        amplitude: f64,
        offset: f64,
        pulse_width: f64,
        pulse_delay: f64,
        dwell: f64,
    ) -> WaveShape {
        WaveShape {
            wave_type,
            amplitude,
            offset,
            pulse_width,
            pulse_delay,
            dwell,
        }
    }

    #[test]
    fn a_pulse_is_timed_in_samples_of_the_dwell_after_its_delay() {
        // 0.25 s wide after 0.2 s at 0.1 s per sample: 2 low, 3 high, rest low.
        let data = generate_waveform(&shape(WAVE_TYPE_PULSE, 1.0, 0.0, 0.25, 0.2, 0.1), 10);
        assert_eq!(data, vec![0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn a_delay_never_pushes_the_pulse_off_the_end() {
        let data = generate_waveform(&shape(WAVE_TYPE_PULSE, 1.0, 0.0, 2.0, 10.0, 1.0), 6);
        assert_eq!(data, vec![0.0, 0.0, 0.0, 1.0, 1.0, 0.0]);
    }

    #[test]
    fn dac_conversion_spans_the_bipolar_range() {
        let mut data = [-10.0, 0.0, 10.0];
        volts_to_dac(&mut data);
        assert_eq!(data[0], 0.0);
        assert_eq!(data[1], 32768.0);
        assert_eq!(data[2], 65535.0);
    }

    #[test]
    fn dac_conversion_clamps_beyond_the_range() {
        let mut data = [-25.0, 25.0];
        volts_to_dac(&mut data);
        assert_eq!(data[0], 0.0);
        assert_eq!(data[1], 65535.0);
    }

    #[test]
    fn an_unset_wave_type_is_all_zeros() {
        // WAVE_TYPE_USER has no internal shape: the driver fills it from
        // WAVEGEN_USER_WF, and an unwritten one must stay at 0 V.
        let data = generate_waveform(&shape(WAVE_TYPE_USER, 5.0, 1.0, 0.5, 0.0, 1.0), 8);
        assert_eq!(data, vec![0.0; 8]);
    }

    #[test]
    fn the_random_sequence_is_glibc_rand_after_srand_1() {
        let mut rng = GlibcRand::new(1);
        let first: Vec<u32> = (0..5).map(|_| rng.next()).collect();
        assert_eq!(
            first,
            vec![1804289383, 846930886, 1681692777, 1714636915, 1957747793]
        );
    }

    #[test]
    fn a_sawtooth_ends_at_the_top_of_its_span() {
        let data = generate_waveform(&shape(WAVE_TYPE_SAWTOOTH, 2.0, 0.0, 0.5, 0.0, 1.0), 4);
        assert_eq!(data[0], -1.0);
        assert_eq!(data[3], 1.0);
    }

    #[test]
    fn a_sine_period_closes_on_its_last_point() {
        let data = generate_waveform(&shape(WAVE_TYPE_SIN, 2.0, 0.0, 0.5, 0.0, 1.0), 5);
        assert!(data[4].abs() < 1e-6, "last point {}", data[4]);
        assert!((data[1] - 1.0).abs() < 1e-6, "quarter period {}", data[1]);
    }

    #[test]
    fn a_square_wave_is_half_high_half_low_around_the_offset() {
        // 2 V peak-to-peak about 1 V: 2 V then 0 V (C base + amplitude, base).
        let data = generate_waveform(&shape(WAVE_TYPE_SQUARE, 2.0, 1.0, 0.5, 0.0, 1.0), 4);
        assert_eq!(data, vec![2.0, 2.0, 0.0, 0.0]);
    }

    #[test]
    fn a_sine_swings_half_the_amplitude_about_the_offset() {
        let data = generate_waveform(&shape(WAVE_TYPE_SIN, 2.0, 0.0, 0.5, 0.0, 1.0), 5);
        let peak = data.iter().cloned().fold(f32::MIN, f32::max);
        assert!((peak - 1.0).abs() < 1e-6, "peak {peak}");
    }

    #[test]
    fn a_sawtooth_starts_half_the_amplitude_below_the_offset() {
        let data = generate_waveform(&shape(WAVE_TYPE_SAWTOOTH, 2.0, 0.0, 0.5, 0.0, 1.0), 4);
        assert!(data.iter().all(|v| (-1.0..=1.0).contains(v)));
    }

    #[test]
    fn a_pulse_is_at_least_one_sample_wide() {
        // A pulse width that rounds to zero samples must still produce a
        // pulse, not a flat line at the offset.
        let data = generate_waveform(&shape(WAVE_TYPE_PULSE, 1.0, 0.0, 0.0, 0.0, 1.0), 10);
        assert_eq!(data[0], 1.0);
        assert_eq!(&data[1..], &[0.0; 9]);
    }

    #[test]
    fn a_pulse_wider_than_the_waveform_leaves_one_sample_low() {
        let data = generate_waveform(&shape(WAVE_TYPE_PULSE, 1.0, 0.0, 10.0, 0.0, 1.0), 4);
        assert_eq!(data, vec![1.0, 1.0, 1.0, 0.0]);
    }

    #[test]
    fn a_single_point_waveform_is_well_defined() {
        for wave_type in [
            WAVE_TYPE_SIN,
            WAVE_TYPE_SQUARE,
            WAVE_TYPE_SAWTOOTH,
            WAVE_TYPE_PULSE,
            WAVE_TYPE_RANDOM,
        ] {
            let data = generate_waveform(&shape(wave_type, 1.0, 0.0, 0.5, 0.0, 1.0), 1);
            assert_eq!(data.len(), 1);
            assert!(data[0].is_finite());
        }
    }
}
