//! The eight waveform generators, ported from `ADCSimDetector::computeArraysT`
//! (ADCSimDetector.cpp:125-213) and `computeArrays` (216-253).
//!
//! Everything here is pure apart from the injected RNG, so each generator is
//! unit-testable without an IOC.
//!
//! ## C integer-conversion semantics
//!
//! The C template is instantiated for all ten `NDDataType_t`s and relies on the
//! platform's `double -> integer` conversion, which truncates toward zero and
//! then wraps modulo 2^N. Rust's `as` saturates instead. [`SimPixel::from_f64_c`]
//! restores the C behaviour by going through `i128` (truncate) before the
//! narrowing bit-truncation.
//!
//! Two consequences of the C source that this port reproduces exactly:
//!
//! * Signals 5 and 6 are built from the **already stored** (hence already
//!   truncated) values of signals 0 and 1, not from their `double` originals.
//! * All eight signals of one time point share the *same* `rndm` draw.

use std::f64::consts::PI;

use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType};

use crate::rng::Rng;
use crate::types::MAX_SIGNALS;

// ============================================================================
// Sample arithmetic with C conversion semantics
// ============================================================================

trait SimPixel: Copy {
    const ZERO: Self;

    /// `(epicsType)v` — truncate toward zero, then wrap modulo 2^N for integers.
    fn from_f64_c(v: f64) -> Self;
    /// Integer/float promotion to `double` in a C expression.
    fn to_f64(self) -> f64;
}

macro_rules! impl_sim_pixel_int {
    ($($t:ty),*) => {$(
        impl SimPixel for $t {
            const ZERO: Self = 0;

            fn from_f64_c(v: f64) -> Self {
                // `v as i128` truncates toward zero and saturates only at the
                // i128 bounds (~1.7e38); `as $t` then truncates the low bits,
                // reproducing the C wrap.
                (v as i128) as Self
            }
            fn to_f64(self) -> f64 { self as f64 }
        }
    )*};
}

macro_rules! impl_sim_pixel_float {
    ($($t:ty),*) => {$(
        impl SimPixel for $t {
            const ZERO: Self = 0.0;

            fn from_f64_c(v: f64) -> Self { v as Self }
            fn to_f64(self) -> f64 { self as f64 }
        }
    )*};
}

impl_sim_pixel_int!(i8, u8, i16, u16, i32, u32, i64, u64);
impl_sim_pixel_float!(f32, f64);

// ============================================================================
// Configuration
// ============================================================================

/// Per-signal parameters, read from asyn address `j` for signal `j`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChannelParams {
    /// `SIM_AMPLITUDE`.
    pub amplitude: f64,
    /// `SIM_OFFSET`.
    pub offset: f64,
    /// `SIM_PERIOD`. `frequency = 1 / period`; a zero period yields an infinite
    /// frequency exactly as the C division does.
    pub period: f64,
    /// `SIM_PHASE`, in degrees. Divided by 360 before use.
    pub phase: f64,
    /// `SIM_NOISE`.
    pub noise: f64,
}

/// Everything `computeArraysT` reads before filling the array.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimConfig {
    /// `SIM_NUM_TIME_POINTS` — `dims[1]`.
    pub num_time_points: usize,
    /// `SIM_TIME_STEP`, seconds per time point.
    pub time_step: f64,
    /// `SIM_ACQUIRE_TIME`. `<= 0` means "acquire forever".
    pub acquire_time: f64,
    /// `NDDataType`.
    pub data_type: NDDataType,
    /// Signal 0..7 parameters.
    pub channels: [ChannelParams; MAX_SIGNALS],
}

/// What one call to [`compute_arrays`] produced.
pub struct ComputeResult {
    /// `MAX_SIGNALS * num_time_points` samples, signal index fastest-varying.
    /// Time points past an early stop stay at the `memset` zero.
    pub data: NDDataBuffer,
    /// `frequency[j] = 1 / period[j]`, to be written back to `SIM_FREQUENCY`.
    pub frequencies: [f64; MAX_SIGNALS],
    /// `elapsedTime_` after the loop.
    pub elapsed_time: f64,
    /// The `elapsedTime_ > acquireTime` break fired: stop acquiring.
    pub acquire_finished: bool,
    /// Number of time points actually written (the rest are zero).
    pub points_computed: usize,
}

// ============================================================================
// The generator
// ============================================================================

/// `computeArraysT<T>` for one concrete sample type.
///
/// `elapsed_time` is C's `elapsedTime_` member: read and advanced in place.
fn compute_typed<T: SimPixel>(
    cfg: &SimConfig,
    elapsed_time: &mut f64,
    rng: &mut Rng,
) -> (Vec<T>, bool, usize) {
    // `memset(pData, 0, MAX_SIGNALS*numTimePoints*sizeof(T))`.
    let mut data = vec![T::ZERO; MAX_SIGNALS * cfg.num_time_points];

    let amplitude: [f64; MAX_SIGNALS] = std::array::from_fn(|j| cfg.channels[j].amplitude);
    let offset: [f64; MAX_SIGNALS] = std::array::from_fn(|j| cfg.channels[j].offset);
    let frequency: [f64; MAX_SIGNALS] = std::array::from_fn(|j| 1.0 / cfg.channels[j].period);
    let noise: [f64; MAX_SIGNALS] = std::array::from_fn(|j| cfg.channels[j].noise);
    // `phase[j] = phase[j]/360.0` — degrees to fractions of a cycle.
    let phase: [f64; MAX_SIGNALS] = std::array::from_fn(|j| cfg.channels[j].phase / 360.0);

    let mut acquire_finished = false;
    let mut points_computed = 0usize;

    for i in 0..cfg.num_time_points {
        let base = i * MAX_SIGNALS;
        // One draw shared by all eight signals of this time point.
        let rndm = rng.next_rndm();
        let t = *elapsed_time;
        let arg = |j: usize| (t * frequency[j] + phase[j]) * 2.0 * PI;

        // Sine wave.
        data[base] = T::from_f64_c(offset[0] + noise[0] * rndm + amplitude[0] * arg(0).sin());
        // Cosine wave.
        data[base + 1] = T::from_f64_c(offset[1] + noise[1] * rndm + amplitude[1] * arg(1).cos());
        // Square wave.
        let square = if arg(2).sin() > 0.0 { 1.0 } else { -1.0 };
        data[base + 2] = T::from_f64_c(offset[2] + noise[2] * rndm + amplitude[2] * square);
        // Sawtooth. Note `*M_PI`, not `*2.*M_PI`. When `tan(...)` is 0 the C code
        // divides by zero, giving `atan(inf) = pi/2`; Rust's `1.0/0.0` is `inf`
        // too, so the result matches.
        let saw = (1.0 / ((t * frequency[3] + phase[3]) * PI).tan()).atan();
        data[base + 3] =
            T::from_f64_c(offset[3] + noise[3] * rndm + amplitude[3] * -2.0 / PI * saw);
        // Random noise.
        data[base + 4] = T::from_f64_c(offset[4] + noise[4] * rndm + amplitude[4] * rndm);
        // Sine + cosine. C precedence: `amplitude[5]*pData[0] + pData[1]`, i.e.
        // only signal 0 is scaled by the amplitude, and both operands are the
        // *stored* (already truncated) samples.
        let s0 = data[base].to_f64();
        let s1 = data[base + 1].to_f64();
        data[base + 5] = T::from_f64_c(offset[5] + noise[5] * rndm + amplitude[5] * s0 + s1);
        // Sine * cosine, again from the stored samples.
        data[base + 6] = T::from_f64_c(offset[6] + noise[6] * rndm + amplitude[6] * s0 * s1);
        // Sum of the first four harmonics. The phase is *not* scaled by the
        // harmonic index in the C source.
        let harmonic = |n: f64| ((t * n * frequency[7] + phase[7]) * 2.0 * PI).sin();
        let sums = harmonic(1.0) + harmonic(2.0) + harmonic(3.0) + harmonic(4.0);
        data[base + 7] = T::from_f64_c(offset[7] + noise[7] * rndm + amplitude[7] * sums);

        points_computed = i + 1;
        *elapsed_time += cfg.time_step;
        if cfg.acquire_time > 0.0 && *elapsed_time > cfg.acquire_time {
            acquire_finished = true;
            break;
        }
    }

    (data, acquire_finished, points_computed)
}

macro_rules! compute_dispatch {
    ($($dt:ident => $buf:ident : $t:ty),* $(,)?) => {
        /// `computeArrays` (ADCSimDetector.cpp:216-253) — the switch over
        /// `NDDataType_t` that picks the template instantiation.
        pub fn compute_arrays(cfg: &SimConfig, elapsed_time: f64, rng: &mut Rng) -> ComputeResult {
            let mut elapsed = elapsed_time;
            let (data, acquire_finished, points_computed) = match cfg.data_type {
                $(
                    NDDataType::$dt => {
                        let (v, f, n) = compute_typed::<$t>(cfg, &mut elapsed, rng);
                        (NDDataBuffer::$buf(v), f, n)
                    }
                )*
            };
            ComputeResult {
                data,
                frequencies: std::array::from_fn(|j| 1.0 / cfg.channels[j].period),
                elapsed_time: elapsed,
                acquire_finished,
                points_computed,
            }
        }
    };
}

compute_dispatch!(
    Int8 => I8: i8,
    UInt8 => U8: u8,
    Int16 => I16: i16,
    UInt16 => U16: u16,
    Int32 => I32: i32,
    UInt32 => U32: u32,
    Int64 => I64: i64,
    UInt64 => U64: u64,
    Float32 => F32: f32,
    Float64 => F64: f64,
);

#[cfg(test)]
mod tests {
    use super::*;

    /// An RNG substitute is not available (the field is a concrete `Rng`), so
    /// tests that need a known `rndm` set `noise = 0` and `amplitude[4] = 0`
    /// where the draw would otherwise leak into the result.
    fn cfg(channels: [ChannelParams; MAX_SIGNALS], n: usize) -> SimConfig {
        SimConfig {
            num_time_points: n,
            time_step: 0.001,
            acquire_time: 0.0,
            data_type: NDDataType::Float64,
            channels,
        }
    }

    fn ch(amplitude: f64, offset: f64, period: f64, phase: f64) -> ChannelParams {
        ChannelParams {
            amplitude,
            offset,
            period,
            phase,
            noise: 0.0,
        }
    }

    fn quiet(c: ChannelParams, j: usize) -> [ChannelParams; MAX_SIGNALS] {
        let mut channels = [ChannelParams {
            period: 1.0,
            ..Default::default()
        }; MAX_SIGNALS];
        channels[j] = c;
        channels
    }

    fn f64s(result: &ComputeResult) -> &[f64] {
        match &result.data {
            NDDataBuffer::F64(v) => v,
            other => panic!("expected F64, got {:?}", other.data_type()),
        }
    }

    fn sample(result: &ComputeResult, i: usize, j: usize) -> f64 {
        f64s(result)[i * MAX_SIGNALS + j]
    }

    fn approx(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-12, "{a} != {b}");
    }

    #[test]
    fn buffer_is_signals_fastest_and_zero_filled_before_use() {
        let mut rng = Rng::new(1);
        let r = compute_arrays(
            &cfg([ChannelParams::default(); MAX_SIGNALS], 5),
            0.0,
            &mut rng,
        );
        assert_eq!(f64s(&r).len(), MAX_SIGNALS * 5);
        // period 0 -> frequency inf -> sin(inf) is NaN; only check the shape.
        assert_eq!(r.points_computed, 5);
    }

    #[test]
    fn signal_0_is_a_sine_of_elapsed_time_times_frequency_plus_phase() {
        let mut rng = Rng::new(1);
        // period 4 s, time step 1 s: elapsed = 0, 1, 2, 3 -> sin(0, pi/2, pi, 3pi/2).
        let mut c = cfg(quiet(ch(2.0, 0.5, 4.0, 0.0), 0), 4);
        c.time_step = 1.0;
        let r = compute_arrays(&c, 0.0, &mut rng);
        approx(sample(&r, 0, 0), 0.5);
        approx(sample(&r, 1, 0), 2.5);
        approx(sample(&r, 2, 0), 0.5);
        approx(sample(&r, 3, 0), -1.5);
    }

    #[test]
    fn signal_1_is_a_cosine() {
        let mut rng = Rng::new(1);
        let mut c = cfg(quiet(ch(3.0, 0.0, 4.0, 0.0), 1), 3);
        c.time_step = 1.0;
        let r = compute_arrays(&c, 0.0, &mut rng);
        approx(sample(&r, 0, 1), 3.0);
        approx(sample(&r, 1, 1), 0.0);
        approx(sample(&r, 2, 1), -3.0);
    }

    #[test]
    fn phase_is_degrees_divided_by_360() {
        let mut rng = Rng::new(1);
        // 90 degrees of phase turns sin into cos.
        let mut c = cfg(quiet(ch(1.0, 0.0, 4.0, 90.0), 0), 1);
        c.time_step = 1.0;
        let r = compute_arrays(&c, 0.0, &mut rng);
        approx(sample(&r, 0, 0), 1.0);
    }

    #[test]
    fn signal_2_is_a_square_wave_with_a_strictly_positive_sine_test() {
        let mut rng = Rng::new(1);
        let mut c = cfg(quiet(ch(1.0, 0.0, 4.0, 0.0), 2), 4);
        c.time_step = 1.0;
        let r = compute_arrays(&c, 0.0, &mut rng);
        // sin(0) == 0 exactly, and 0 is not > 0, so the first sample is -1.
        approx(sample(&r, 0, 2), -1.0);
        // sin(pi/2) == 1.
        approx(sample(&r, 1, 2), 1.0);
        // sin(pi) is not 0 in binary floating point but +1.2246e-16, which is
        // > 0 — so the half-way sample is +1, not -1. C behaves identically.
        assert!(PI.sin() > 0.0);
        approx(sample(&r, 2, 2), 1.0);
        // sin(3pi/2) == -1.
        approx(sample(&r, 3, 2), -1.0);
    }

    #[test]
    fn signal_3_is_a_sawtooth_over_a_half_period_argument() {
        let mut rng = Rng::new(1);
        let mut c = cfg(quiet(ch(1.0, 0.0, 4.0, 0.0), 3), 4);
        c.time_step = 1.0;
        let r = compute_arrays(&c, 0.0, &mut rng);
        // t=0: tan(0)=0 -> 1/0 = inf -> atan(inf) = pi/2 -> -2/pi * pi/2 = -1.
        approx(sample(&r, 0, 3), -1.0);
        // t=1: tan(pi/4)=1 -> atan(1)=pi/4 -> -0.5.
        approx(sample(&r, 1, 3), -0.5);
        // t=2: tan(pi/2)=huge -> 1/tan ~ 0 -> ~0.
        assert!(sample(&r, 2, 3).abs() < 1e-15);
        // t=3: tan(3pi/4) = -1 -> atan(-1) = -pi/4 -> +0.5.
        approx(sample(&r, 3, 3), 0.5);
    }

    #[test]
    fn signal_4_is_offset_plus_amplitude_times_the_shared_draw() {
        let mut channels = [ChannelParams {
            period: 1.0,
            ..Default::default()
        }; MAX_SIGNALS];
        channels[4] = ch(1.0, 10.0, 1.0, 0.0);
        let mut rng = Rng::new(99);
        let r = compute_arrays(&cfg(channels, 3), 0.0, &mut rng);

        let mut expect = Rng::new(99);
        for i in 0..3 {
            approx(sample(&r, i, 4), 10.0 + expect.next_rndm());
        }
    }

    #[test]
    fn all_eight_signals_of_one_time_point_share_the_same_draw() {
        // Give every signal noise 1 and nothing else; the value of each sample
        // is then exactly `rndm`, identical across the eight signals.
        let channels = [ChannelParams {
            amplitude: 0.0,
            offset: 0.0,
            period: 1.0,
            phase: 0.0,
            noise: 1.0,
        }; MAX_SIGNALS];
        let mut rng = Rng::new(5);
        let r = compute_arrays(&cfg(channels, 4), 0.0, &mut rng);

        let mut expect = Rng::new(5);
        for i in 0..4 {
            let rndm = expect.next_rndm();
            for j in 0..MAX_SIGNALS {
                // Signals 5 and 6 are built from the stored 0/1 samples.
                let want = match j {
                    5 => rndm + rndm, // noise*rndm + 0*s0 + s1, s1 == rndm
                    6 => rndm,        // noise*rndm + 0*s0*s1
                    _ => rndm,
                };
                approx(sample(&r, i, j), want);
            }
        }
    }

    #[test]
    fn signal_5_scales_only_the_sine_by_its_amplitude() {
        let mut channels = [ChannelParams {
            period: 1.0,
            ..Default::default()
        }; MAX_SIGNALS];
        channels[0] = ch(2.0, 0.0, 4.0, 90.0); // sin with 90 deg phase -> cos
        channels[1] = ch(5.0, 0.0, 4.0, 0.0); // cos
        channels[5] = ch(3.0, 1.0, 1.0, 0.0);
        let mut rng = Rng::new(1);
        let mut c = cfg(channels, 1);
        c.time_step = 1.0;
        let r = compute_arrays(&c, 0.0, &mut rng);
        let s0 = sample(&r, 0, 0);
        let s1 = sample(&r, 0, 1);
        approx(s0, 2.0);
        approx(s1, 5.0);
        // offset + amplitude*s0 + s1 = 1 + 3*2 + 5 = 12. Not 3*(s0+s1)=21.
        approx(sample(&r, 0, 5), 12.0);
    }

    #[test]
    fn signal_6_multiplies_the_two_stored_samples() {
        let mut channels = [ChannelParams {
            period: 1.0,
            ..Default::default()
        }; MAX_SIGNALS];
        channels[0] = ch(2.0, 0.0, 4.0, 90.0);
        channels[1] = ch(5.0, 0.0, 4.0, 0.0);
        channels[6] = ch(3.0, 1.0, 1.0, 0.0);
        let mut rng = Rng::new(1);
        let mut c = cfg(channels, 1);
        c.time_step = 1.0;
        let r = compute_arrays(&c, 0.0, &mut rng);
        // 1 + 3*2*5 = 31.
        approx(sample(&r, 0, 6), 31.0);
    }

    #[test]
    fn signals_5_and_6_use_the_truncated_integer_samples_not_the_doubles() {
        let mut channels = [ChannelParams {
            period: 1.0,
            ..Default::default()
        }; MAX_SIGNALS];
        // sin at t=0 with 90 deg phase = 1.0 -> s0 = 2.7 -> stored as 2 (Int32).
        channels[0] = ch(2.7, 0.0, 4.0, 90.0);
        channels[1] = ch(5.9, 0.0, 4.0, 0.0); // cos(0) = 1 -> 5.9 -> stored as 5
        channels[5] = ch(1.0, 0.0, 1.0, 0.0);
        channels[6] = ch(1.0, 0.0, 1.0, 0.0);
        let mut rng = Rng::new(1);
        let mut c = cfg(channels, 1);
        c.time_step = 1.0;
        c.data_type = NDDataType::Int32;
        let r = compute_arrays(&c, 0.0, &mut rng);
        let v = match &r.data {
            NDDataBuffer::I32(v) => v.clone(),
            other => panic!("expected I32, got {:?}", other.data_type()),
        };
        assert_eq!(v[0], 2);
        assert_eq!(v[1], 5);
        assert_eq!(v[5], 7); // 2 + 5, not 2.7 + 5.9 = 8.6 -> 8
        assert_eq!(v[6], 10); // 2 * 5, not 2.7 * 5.9 = 15.9 -> 15
    }

    #[test]
    fn signal_7_sums_four_harmonics_with_an_unscaled_phase() {
        let mut rng = Rng::new(1);
        let mut c = cfg(quiet(ch(1.0, 0.0, 8.0, 45.0), 7), 1);
        c.time_step = 1.0;
        let r = compute_arrays(&c, 1.0, &mut rng);
        // t=1, f=1/8, phase=0.125 cycles. arg_n = (n/8 + 1/8)*2pi.
        let phase = 45.0 / 360.0;
        let want: f64 = (1..=4)
            .map(|n| ((n as f64 / 8.0 + phase) * 2.0 * PI).sin())
            .sum();
        approx(sample(&r, 0, 7), want);
    }

    #[test]
    fn frequency_is_the_reciprocal_of_the_period() {
        let mut channels = [ChannelParams::default(); MAX_SIGNALS];
        for (j, c) in channels.iter_mut().enumerate() {
            c.period = (j + 1) as f64;
        }
        let mut rng = Rng::new(1);
        let r = compute_arrays(&cfg(channels, 1), 0.0, &mut rng);
        for j in 0..MAX_SIGNALS {
            approx(r.frequencies[j], 1.0 / (j + 1) as f64);
        }
    }

    #[test]
    fn a_zero_period_yields_an_infinite_frequency_as_in_c() {
        let mut rng = Rng::new(1);
        let r = compute_arrays(
            &cfg([ChannelParams::default(); MAX_SIGNALS], 1),
            0.0,
            &mut rng,
        );
        assert!(r.frequencies[0].is_infinite());
    }

    #[test]
    fn elapsed_time_advances_by_one_time_step_per_point() {
        let mut rng = Rng::new(1);
        let mut c = cfg(quiet(ch(0.0, 0.0, 1.0, 0.0), 0), 10);
        c.time_step = 0.25;
        let r = compute_arrays(&c, 2.0, &mut rng);
        approx(r.elapsed_time, 2.0 + 10.0 * 0.25);
        assert!(!r.acquire_finished);
    }

    #[test]
    fn acquire_time_stops_the_loop_and_leaves_the_tail_zeroed() {
        let mut rng = Rng::new(1);
        let mut c = cfg(quiet(ch(1.0, 5.0, 4.0, 0.0), 0), 10);
        c.time_step = 1.0;
        c.acquire_time = 3.0; // stop once elapsed > 3, i.e. after the 4th point
        let r = compute_arrays(&c, 0.0, &mut rng);
        assert!(r.acquire_finished);
        assert_eq!(r.points_computed, 4);
        approx(r.elapsed_time, 4.0);
        // The 4 computed points carry the offset; the tail is untouched.
        for i in 0..4 {
            assert_ne!(sample(&r, i, 0), 0.0);
        }
        for i in 4..10 {
            for j in 0..MAX_SIGNALS {
                assert_eq!(sample(&r, i, j), 0.0);
            }
        }
        assert_eq!(f64s(&r).len(), MAX_SIGNALS * 10);
    }

    #[test]
    fn a_non_positive_acquire_time_never_stops_the_loop() {
        for acquire_time in [0.0, -1.0] {
            let mut rng = Rng::new(1);
            let mut c = cfg(quiet(ch(1.0, 0.0, 1.0, 0.0), 0), 6);
            c.time_step = 1.0;
            c.acquire_time = acquire_time;
            let r = compute_arrays(&c, 0.0, &mut rng);
            assert!(!r.acquire_finished);
            assert_eq!(r.points_computed, 6);
        }
    }

    #[test]
    fn every_nd_data_type_is_dispatched_with_the_right_buffer() {
        let types = [
            NDDataType::Int8,
            NDDataType::UInt8,
            NDDataType::Int16,
            NDDataType::UInt16,
            NDDataType::Int32,
            NDDataType::UInt32,
            NDDataType::Int64,
            NDDataType::UInt64,
            NDDataType::Float32,
            NDDataType::Float64,
        ];
        for dt in types {
            let mut rng = Rng::new(1);
            let mut c = cfg(quiet(ch(0.0, 3.0, 1.0, 0.0), 0), 2);
            c.data_type = dt;
            let r = compute_arrays(&c, 0.0, &mut rng);
            assert_eq!(r.data.data_type(), dt);
            assert_eq!(r.data.len(), MAX_SIGNALS * 2);
        }
    }

    #[test]
    fn integer_conversion_truncates_toward_zero_and_wraps_like_c() {
        // offset 300.9 into an Int8: C truncates to 300 then wraps to 44.
        let mut rng = Rng::new(1);
        let mut c = cfg(quiet(ch(0.0, 300.9, 1.0, 0.0), 0), 1);
        c.data_type = NDDataType::Int8;
        let r = compute_arrays(&c, 0.0, &mut rng);
        match &r.data {
            NDDataBuffer::I8(v) => assert_eq!(v[0], 44),
            other => panic!("expected I8, got {:?}", other.data_type()),
        }

        // offset -1.9 into a UInt8: truncate to -1, wrap to 255.
        let mut rng = Rng::new(1);
        let mut c = cfg(quiet(ch(0.0, -1.9, 1.0, 0.0), 0), 1);
        c.data_type = NDDataType::UInt8;
        let r = compute_arrays(&c, 0.0, &mut rng);
        match &r.data {
            NDDataBuffer::U8(v) => assert_eq!(v[0], 255),
            other => panic!("expected U8, got {:?}", other.data_type()),
        }
    }
}
