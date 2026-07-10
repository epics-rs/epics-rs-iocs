//! Image generation, ported from `simDetector.cpp`.
//!
//! The four pattern generators (`computeLinearRampArray`, `computePeaksArray`,
//! `computeSineArray`, and the `SimModeOffsetNoise` no-op) and their shared
//! preamble (`computeArray`) are reproduced element for element. Everything in
//! this module is pure apart from the RNG, so each generator is unit-testable
//! without an IOC.
//!
//! ## C integer-conversion semantics
//!
//! The C templates are instantiated for all ten `NDDataType_t`s and rely on the
//! platform's `double -> integer` conversion, which truncates toward zero and
//! then wraps modulo 2^N. Rust's `as` saturates instead, so a `gain * j` ramp
//! that overruns `u8` would clamp at 255 rather than wrap — visibly a different
//! image. [`SimPixel::from_f64_c`] restores the C behaviour by going through
//! `i128` (truncate) before the narrowing bit-truncation.

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute};
use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::error::ADResult;
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;

use crate::rng::Rng;
use crate::types::{SimMode, SineOperation};

/// `MAX_PEAK_SIGMA` (simDetector.cpp:37).
const MAX_PEAK_SIGMA: i32 = 4;

// ============================================================================
// Pixel arithmetic with C conversion semantics
// ============================================================================

trait SimPixel: Copy + PartialEq {
    const ZERO: Self;

    /// `(epicsType)v` — truncate toward zero, then wrap modulo 2^N for integers.
    fn from_f64_c(v: f64) -> Self;
    /// Integer/float promotion to `double` in a C expression.
    fn to_f64(self) -> f64;
    /// `a += b` for two values of the pixel type (wraps for integers).
    fn add_c(self, rhs: Self) -> Self;
    /// `a * b` for two values of the pixel type (wraps for integers).
    fn mul_c(self, rhs: Self) -> Self;

    fn is_zero(self) -> bool {
        self == Self::ZERO
    }
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
            fn add_c(self, rhs: Self) -> Self { self.wrapping_add(rhs) }
            fn mul_c(self, rhs: Self) -> Self { self.wrapping_mul(rhs) }
        }
    )*};
}

macro_rules! impl_sim_pixel_float {
    ($($t:ty),*) => {$(
        impl SimPixel for $t {
            const ZERO: Self = 0.0;

            fn from_f64_c(v: f64) -> Self { v as Self }
            fn to_f64(self) -> f64 { self as f64 }
            fn add_c(self, rhs: Self) -> Self { self + rhs }
            fn mul_c(self, rhs: Self) -> Self { self * rhs }
        }
    )*};
}

impl_sim_pixel_int!(i8, u8, i16, u16, i32, u32, i64, u64);
impl_sim_pixel_float!(f32, f64);

// ============================================================================
// Configuration
// ============================================================================

/// Sine-wave parameters for one axis (`SIM_XSINE*` / `SIM_YSINE*`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SineParams {
    pub operation: SineOperation,
    pub amplitude1: f64,
    pub frequency1: f64,
    pub phase1: f64,
    pub amplitude2: f64,
    pub frequency2: f64,
    pub phase2: f64,
}

impl Default for SineParams {
    /// The record defaults from `simDetector.template` (amplitude 1, sine-2 at
    /// double frequency and 90 degree phase).
    fn default() -> Self {
        Self {
            operation: SineOperation::Add,
            amplitude1: 1.0,
            frequency1: 1.0,
            phase1: 0.0,
            amplitude2: 1.0,
            frequency2: 2.0,
            phase2: 90.0,
        }
    }
}

/// Detector geometry (`ADMaxSizeX` .. `ADReverseY`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Geometry {
    pub max_size_x: i32,
    pub max_size_y: i32,
    pub min_x: i32,
    pub min_y: i32,
    pub size_x: i32,
    pub size_y: i32,
    pub bin_x: i32,
    pub bin_y: i32,
    pub reverse_x: bool,
    pub reverse_y: bool,
}

impl Geometry {
    /// "Make sure parameters are consistent, fix them if they are not"
    /// (simDetector.cpp:541-597). The order matters: `sizeX` is clamped against
    /// `maxSizeX - minX` only after `minX` has itself been clamped, and the
    /// binning divisibility trim runs last.
    pub fn clamp(&mut self) {
        if self.min_x < 0 {
            self.min_x = 0;
        }
        if self.min_y < 0 {
            self.min_y = 0;
        }
        if self.min_x > self.max_size_x - 1 {
            self.min_x = self.max_size_x - 1;
        }
        if self.min_y > self.max_size_y - 1 {
            self.min_y = self.max_size_y - 1;
        }
        if self.size_x < 1 {
            self.size_x = 1;
        }
        if self.size_y < 1 {
            self.size_y = 1;
        }
        if self.size_x > self.max_size_x - self.min_x {
            self.size_x = self.max_size_x - self.min_x;
        }
        if self.size_y > self.max_size_y - self.min_y {
            self.size_y = self.max_size_y - self.min_y;
        }
        if self.bin_x < 1 {
            self.bin_x = 1;
        }
        if self.bin_y < 1 {
            self.bin_y = 1;
        }
        if self.bin_x > self.size_x {
            self.bin_x = self.size_x;
        }
        if self.bin_y > self.size_y {
            self.bin_y = self.size_y;
        }
        if self.size_x % self.bin_x != 0 {
            self.size_x = (self.size_x / self.bin_x) * self.bin_x;
        }
        if self.size_y % self.bin_y != 0 {
            self.size_y = (self.size_y / self.bin_y) * self.bin_y;
        }
    }
}

/// Everything `computeImage` reads out of the parameter library.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimConfig {
    pub geometry: Geometry,
    pub color_mode: NDColorMode,
    pub data_type: NDDataType,
    pub sim_mode: SimMode,

    pub gain: f64,
    pub gain_x: f64,
    pub gain_y: f64,
    pub gain_red: f64,
    pub gain_green: f64,
    pub gain_blue: f64,

    pub offset: f64,
    pub noise: f64,

    pub peak_start_x: i32,
    pub peak_start_y: i32,
    pub peak_width_x: i32,
    pub peak_width_y: i32,
    pub peak_num_x: i32,
    pub peak_num_y: i32,
    pub peak_step_x: i32,
    pub peak_step_y: i32,
    pub peak_height_variation: f64,

    pub x_sine: SineParams,
    pub y_sine: SineParams,
}

impl SimConfig {
    /// The `simDetector` constructor defaults (simDetector.cpp:1102-1134),
    /// for a `max_size_x` x `max_size_y` UInt8 mono detector.
    pub fn defaults(max_size_x: i32, max_size_y: i32) -> Self {
        Self {
            geometry: Geometry {
                max_size_x,
                max_size_y,
                min_x: 0,
                min_y: 0,
                size_x: max_size_x,
                size_y: max_size_y,
                bin_x: 1,
                bin_y: 1,
                reverse_x: false,
                reverse_y: false,
            },
            color_mode: NDColorMode::Mono,
            data_type: NDDataType::UInt8,
            sim_mode: SimMode::LinearRamp,
            gain: 1.0,
            gain_x: 1.0,
            gain_y: 1.0,
            gain_red: 1.0,
            gain_green: 1.0,
            gain_blue: 1.0,
            offset: 0.0,
            noise: 0.0,
            peak_start_x: 1,
            peak_start_y: 1,
            peak_width_x: 10,
            peak_width_y: 20,
            peak_num_x: 1,
            peak_num_y: 1,
            peak_step_x: 1,
            peak_step_y: 1,
            peak_height_variation: 0.0,
            x_sine: SineParams::default(),
            y_sine: SineParams::default(),
        }
    }
}

/// Which NDArray dimension carries X, Y and colour, per `NDColorMode`
/// (simDetector.cpp:599-623).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Layout {
    pub ndims: usize,
    pub x_dim: usize,
    pub y_dim: usize,
    pub color_dim: Option<usize>,
}

impl Layout {
    /// DEVIATION: C leaves `ndims = 0` and `colorDim = -1` for colour modes the
    /// driver does not support (Bayer, YUV*), then indexes `dims[]` with them.
    /// `simDetector.template` restricts `ColorMode` to Mono/RGB1/RGB2/RGB3, so
    /// that path is unreachable from a correctly loaded IOC; here anything else
    /// is treated as Mono rather than left undefined.
    pub fn for_color_mode(mode: NDColorMode) -> Self {
        match mode {
            NDColorMode::RGB1 => Layout {
                ndims: 3,
                x_dim: 1,
                y_dim: 2,
                color_dim: Some(0),
            },
            NDColorMode::RGB2 => Layout {
                ndims: 3,
                x_dim: 0,
                y_dim: 2,
                color_dim: Some(1),
            },
            NDColorMode::RGB3 => Layout {
                ndims: 3,
                x_dim: 0,
                y_dim: 1,
                color_dim: Some(2),
            },
            _ => Layout {
                ndims: 2,
                x_dim: 0,
                y_dim: 1,
                color_dim: None,
            },
        }
    }

    pub fn is_color(&self) -> bool {
        self.color_dim.is_some()
    }
}

// ============================================================================
// Typed working buffers (C `pRaw_`, `pBackground_`, `pRamp_`, `pPeak_`)
// ============================================================================

struct Bufs<T> {
    raw: Vec<T>,
    background: Vec<T>,
    ramp: Vec<T>,
    peak: Vec<T>,
}

impl<T: SimPixel> Bufs<T> {
    fn new(n: usize) -> Self {
        Self {
            raw: vec![T::ZERO; n],
            background: vec![T::ZERO; n],
            ramp: vec![T::ZERO; n],
            peak: vec![T::ZERO; n],
        }
    }
}

macro_rules! sim_buffers {
    ($($dt:ident / $buf:ident => $t:ty),* $(,)?) => {
        enum SimBuffers { $($dt(Bufs<$t>)),* }

        impl SimBuffers {
            fn new(data_type: NDDataType, n: usize) -> Self {
                match data_type { $(NDDataType::$dt => Self::$dt(Bufs::new(n))),* }
            }

            fn data_type(&self) -> NDDataType {
                match self { $(Self::$dt(_) => NDDataType::$dt),* }
            }

            fn len(&self) -> usize {
                match self { $(Self::$dt(b) => b.raw.len()),* }
            }

            fn compute(&mut self, cfg: &SimConfig, st: &mut DynState, reset: bool, rng: &mut Rng) {
                match self { $(Self::$dt(b) => compute_array(b, cfg, st, reset, rng)),* }
            }

            /// Copy of the accumulating raw buffer, as an `NDDataBuffer`.
            fn raw_buffer(&self) -> NDDataBuffer {
                match self { $(Self::$dt(b) => NDDataBuffer::$buf(b.raw.clone())),* }
            }
        }
    };
}

sim_buffers! {
    Int8 / I8 => i8,
    UInt8 / U8 => u8,
    Int16 / I16 => i16,
    UInt16 / U16 => u16,
    Int32 / I32 => i32,
    UInt32 / U32 => u32,
    Int64 / I64 => i64,
    UInt64 / U64 => u64,
    Float32 / F32 => f32,
    Float64 / F64 => f64,
}

/// State that survives between frames (C member variables).
#[derive(Default)]
struct DynState {
    /// C `useBackground_`.
    use_background: bool,
    x_sine1: Vec<f64>,
    x_sine2: Vec<f64>,
    y_sine1: Vec<f64>,
    y_sine2: Vec<f64>,
    /// C `xSineCounter_` / `ySineCounter_`; free-running across frames.
    x_sine_counter: f64,
    y_sine_counter: f64,
}

// ============================================================================
// computeArray
// ============================================================================

/// `simDetector::computeArray` (simDetector.cpp:45-107).
fn compute_array<T: SimPixel>(
    b: &mut Bufs<T>,
    cfg: &SimConfig,
    st: &mut DynState,
    reset: bool,
    rng: &mut Rng,
) {
    let n = b.raw.len();
    let offset = T::from_f64_c(cfg.offset);

    if reset {
        st.use_background = false;
        if cfg.noise != 0.0 || !offset.is_zero() {
            st.use_background = true;
            if cfg.noise == 0.0 {
                b.background.fill(offset);
            } else {
                let off = offset.to_f64();
                for v in b.background.iter_mut() {
                    *v = T::from_f64_c(cfg.noise * rng.next_f64() + off);
                }
            }
        }
    }

    if st.use_background {
        // Copy the pre-computed background starting at a random location, i.e.
        // rotate it left by `background_start` elements (memcpy pair in C).
        let background_start = (n as f64 * rng.next_f64()) as usize;
        let split = n - background_start;
        b.raw[..split].copy_from_slice(&b.background[background_start..]);
        b.raw[split..].copy_from_slice(&b.background[..background_start]);
    } else if cfg.sim_mode != SimMode::LinearRamp {
        // LinearRamp without a background accumulates in place across frames,
        // so it is deliberately not zeroed here.
        b.raw.fill(T::ZERO);
    }

    match cfg.sim_mode {
        SimMode::LinearRamp => compute_linear_ramp(b, cfg, st, reset),
        SimMode::Peaks => compute_peaks(b, cfg, reset, rng),
        SimMode::Sine => compute_sine(b, cfg, st, reset),
        SimMode::OffsetNoise => {}
    }
}

// ============================================================================
// computeLinearRampArray
// ============================================================================

/// Per-row element index of the red/green/blue (or mono) channel start, and the
/// element step between successive columns, for the C pointer walk.
fn rgb_row_bases(layout: &Layout, row: usize, size_x: usize, size_y: usize) -> ([usize; 3], usize) {
    match layout.color_dim {
        // RGB1: interleaved; row stride 3*sizeX, column step 3.
        Some(0) => (
            [3 * size_x * row, 3 * size_x * row + 1, 3 * size_x * row + 2],
            3,
        ),
        // RGB2: row-interleaved planes; row stride 3*sizeX, column step 1.
        Some(1) => {
            let base = 3 * size_x * row;
            ([base, base + size_x, base + 2 * size_x], 1)
        }
        // RGB3: separate planes; row stride sizeX, column step 1.
        Some(2) => {
            let base = size_x * row;
            (
                [base, base + size_x * size_y, base + 2 * size_x * size_y],
                1,
            )
        }
        _ => ([size_x * row, 0, 0], 1),
    }
}

/// `simDetector::computeLinearRampArray` (simDetector.cpp:110-230).
fn compute_linear_ramp<T: SimPixel>(b: &mut Bufs<T>, cfg: &SimConfig, st: &DynState, reset: bool) {
    if st.use_background {
        fill_linear_ramp(&mut b.ramp, cfg, reset);
        for (r, m) in b.raw.iter_mut().zip(b.ramp.iter()) {
            *r = r.add_c(*m);
        }
    } else {
        fill_linear_ramp(&mut b.raw, cfg, reset);
    }
}

fn fill_linear_ramp<T: SimPixel>(data: &mut [T], cfg: &SimConfig, reset: bool) {
    let layout = Layout::for_color_mode(cfg.color_mode);
    let sx = cfg.geometry.max_size_x as usize;
    let sy = cfg.geometry.max_size_y as usize;

    // incMono = (epicsType)gain; incRed = (epicsType)gainRed * incMono.
    // The gain truncation to the pixel type before the multiply is C's, not a
    // rounding accident: for integer types `gainRed = 0.5` yields zero.
    let inc_mono = T::from_f64_c(cfg.gain);
    let inc = [
        T::from_f64_c(cfg.gain_red).mul_c(inc_mono),
        T::from_f64_c(cfg.gain_green).mul_c(inc_mono),
        T::from_f64_c(cfg.gain_blue).mul_c(inc_mono),
    ];

    for i in 0..sy {
        let (bases, column_step) = rgb_row_bases(&layout, i, sx, sy);
        for j in 0..sx {
            // The intensity at each pixel[i,j] is `inc * (gainX*j + gainY*i)`
            // on a reset frame, and `pixel += inc` on every later frame.
            let ramp = cfg.gain_x * j as f64 + cfg.gain_y * i as f64;
            if layout.is_color() {
                for (base, inc_c) in bases.iter().zip(inc.iter()) {
                    let idx = base + j * column_step;
                    data[idx] = if reset {
                        T::from_f64_c(inc_c.to_f64() * ramp)
                    } else {
                        data[idx].add_c(*inc_c)
                    };
                }
            } else {
                let idx = bases[0] + j;
                data[idx] = if reset {
                    T::from_f64_c(inc_mono.to_f64() * ramp)
                } else {
                    data[idx].add_c(inc_mono)
                };
            }
        }
    }
}

// ============================================================================
// computePeaksArray
// ============================================================================

/// `peakFullWidth{X,Y}` (simDetector.cpp:267-268).
fn peak_full_width(width: i32, size: i32) -> i32 {
    let full = 2 * MAX_PEAK_SIGMA * width + 1;
    if full < size { full } else { size - 1 }
}

/// `simDetector::computePeaksArray` (simDetector.cpp:233-346).
fn compute_peaks<T: SimPixel>(b: &mut Bufs<T>, cfg: &SimConfig, reset: bool, rng: &mut Rng) {
    let sx = cfg.geometry.max_size_x;
    let sy = cfg.geometry.max_size_y;
    let pfw_x = peak_full_width(cfg.peak_width_x, sx);
    let pfw_y = peak_full_width(cfg.peak_width_y, sy);

    if reset {
        fill_peak_gaussian(&mut b.peak, cfg, pfw_x, pfw_y);
    }
    add_peaks(&b.peak, &mut b.raw, cfg, pfw_x, pfw_y, rng);
}

/// The 2-D Gaussian kernel, laid out with row stride `maxSizeX`.
fn fill_peak_gaussian<T: SimPixel>(peak: &mut [T], cfg: &SimConfig, pfw_x: i32, pfw_y: i32) {
    let sx = cfg.geometry.max_size_x as usize;
    for i in 0..pfw_y {
        for j in 0..pfw_x {
            let gauss_y =
                (-((i - pfw_y / 2) as f64 / cfg.peak_width_y as f64).powf(2.0) / 2.0).exp();
            let gauss_x =
                (-((j - pfw_x / 2) as f64 / cfg.peak_width_x as f64).powf(2.0) / 2.0).exp();
            peak[i as usize * sx + j as usize] = T::from_f64_c(cfg.gain * gauss_x * gauss_y);
        }
    }
}

fn add_peaks<T: SimPixel>(
    peak: &[T],
    raw: &mut [T],
    cfg: &SimConfig,
    pfw_x: i32,
    pfw_y: i32,
    rng: &mut Rng,
) {
    let layout = Layout::for_color_mode(cfg.color_mode);
    let sx = cfg.geometry.max_size_x;
    let sy = cfg.geometry.max_size_y;
    let sxu = sx as usize;
    let syu = sy as usize;

    for i in 0..cfg.peak_num_y {
        for j in 0..cfg.peak_num_x {
            let gain_variation = if cfg.peak_height_variation != 0.0 {
                1.0 + (cfg.peak_height_variation / 100.0) * (rng.next_f64() - 0.5)
            } else {
                1.0
            };
            let offset_y = i * cfg.peak_step_y + cfg.peak_start_y;
            let offset_x = j * cfg.peak_step_x + cfg.peak_start_x;

            for k in 0..pfw_y {
                let y_out = offset_y + k - pfw_y / 2;
                if y_out < 0 || y_out >= sy {
                    continue;
                }
                let (bases, column_step) = rgb_row_bases(&layout, y_out as usize, sxu, syu);

                for l in 0..pfw_x {
                    let pin = peak[k as usize * sxu + l as usize];
                    let x_out = offset_x + l - pfw_x / 2;
                    if x_out < 0 || x_out >= sx {
                        continue;
                    }
                    if layout.is_color() {
                        let xo = x_out as usize * column_step;
                        let gains = [cfg.gain_red, cfg.gain_green, cfg.gain_blue];
                        for (base, gain_c) in bases.iter().zip(gains.iter()) {
                            let idx = base + xo;
                            // C: `pRed[xOut] += (epicsType)(gainRed * gainVariation * *pIn)`
                            // — the product is truncated to the pixel type first,
                            // then added.
                            raw[idx] = raw[idx]
                                .add_c(T::from_f64_c(gain_c * gain_variation * pin.to_f64()));
                        }
                    } else {
                        let idx = bases[0] + x_out as usize;
                        // C: `pOut[xOut] += gainVariation * *pIn` — the mono path
                        // has no cast, so the *sum* is computed in double and
                        // truncated on assignment.
                        raw[idx] = T::from_f64_c(raw[idx].to_f64() + gain_variation * pin.to_f64());
                    }
                }
            }
        }
    }
}

// ============================================================================
// computeSineArray
// ============================================================================

/// `simDetector::computeSineArray` (simDetector.cpp:349-487).
fn compute_sine<T: SimPixel>(b: &mut Bufs<T>, cfg: &SimConfig, st: &mut DynState, reset: bool) {
    let layout = Layout::for_color_mode(cfg.color_mode);
    let sx = cfg.geometry.max_size_x as usize;
    let sy = cfg.geometry.max_size_y as usize;

    if reset {
        st.x_sine1 = vec![0.0; sx];
        st.x_sine2 = vec![0.0; sx];
        st.y_sine1 = vec![0.0; sy];
        st.y_sine2 = vec![0.0; sy];
        st.x_sine_counter = 0.0;
        st.y_sine_counter = 0.0;
    }

    fill_sine_axis(
        &mut st.x_sine1,
        &mut st.x_sine2,
        &mut st.x_sine_counter,
        cfg.gain_x,
        sx,
        &cfg.x_sine,
    );
    fill_sine_axis(
        &mut st.y_sine1,
        &mut st.y_sine2,
        &mut st.y_sine_counter,
        cfg.gain_y,
        sy,
        &cfg.y_sine,
    );

    if !layout.is_color() {
        // Mono combines sine-1 and sine-2 per axis; the colour path leaves them
        // separate and uses sine-2 only for blue.
        combine_sines(&mut st.x_sine1, &st.x_sine2, cfg.x_sine.operation);
        combine_sines(&mut st.y_sine1, &st.y_sine2, cfg.y_sine.operation);
    }

    for i in 0..sy {
        let (bases, column_step) = rgb_row_bases(&layout, i, sx, sy);
        for j in 0..sx {
            if layout.is_color() {
                let values = [
                    cfg.gain * cfg.gain_red * st.x_sine1[j],
                    cfg.gain * cfg.gain_green * st.y_sine1[i],
                    cfg.gain * cfg.gain_blue * (st.x_sine2[j] + st.y_sine2[i]) / 2.0,
                ];
                for (base, value) in bases.iter().zip(values.iter()) {
                    raw_add(&mut b.raw, base + j * column_step, *value);
                }
            } else {
                let idx = bases[0] + j;
                raw_add(&mut b.raw, idx, cfg.gain * (st.y_sine1[i] + st.x_sine1[j]));
            }
        }
    }
}

fn raw_add<T: SimPixel>(raw: &mut [T], idx: usize, value: f64) {
    raw[idx] = raw[idx].add_c(T::from_f64_c(value));
}

fn fill_sine_axis(
    sine1: &mut [f64],
    sine2: &mut [f64],
    counter: &mut f64,
    gain: f64,
    size: usize,
    p: &SineParams,
) {
    let tau = 2.0 * std::f64::consts::PI;
    for (s1, s2) in sine1.iter_mut().zip(sine2.iter_mut()) {
        // C: `time = counter++ * gain / size` — post-increment, so the sample
        // uses the pre-increment counter, and the counter free-runs across frames.
        let t = *counter * gain / size as f64;
        *counter += 1.0;
        *s1 = p.amplitude1 * ((t * p.frequency1 + p.phase1 / 360.0) * tau).sin();
        *s2 = p.amplitude2 * ((t * p.frequency2 + p.phase2 / 360.0) * tau).sin();
    }
}

fn combine_sines(sine1: &mut [f64], sine2: &[f64], op: SineOperation) {
    for (a, b) in sine1.iter_mut().zip(sine2.iter()) {
        *a = match op {
            SineOperation::Add => *a + *b,
            SineOperation::Multiply => *a * *b,
        };
    }
}

// ============================================================================
// computeImage
// ============================================================================

/// Frame generator: owns the working buffers and the cross-frame sine state.
pub struct SimEngine {
    buffers: Option<SimBuffers>,
    state: DynState,
    rng: Rng,
}

impl SimEngine {
    pub fn new(rng: Rng) -> Self {
        Self {
            buffers: None,
            state: DynState::default(),
            rng,
        }
    }

    /// `simDetector::computeImage` (simDetector.cpp:505-718), minus the
    /// geometry clamping — the caller does that via [`Geometry::clamp`] because
    /// it must also write the corrected values back to the parameter library.
    ///
    /// `reset_requested` is the `RESET_IMAGE` parameter. The buffers are also
    /// reallocated (and therefore reset) whenever the element count or data type
    /// changes, so `reset` is exactly "the working buffers are fresh" — the
    /// invariant the absolute-vs-incremental branches in the generators depend on.
    pub fn compute_image(
        &mut self,
        cfg: &SimConfig,
        pool: &NDArrayPool,
        reset_requested: bool,
    ) -> ADResult<NDArray> {
        let layout = Layout::for_color_mode(cfg.color_mode);
        let sx = cfg.geometry.max_size_x as usize;
        let sy = cfg.geometry.max_size_y as usize;
        let n_elements = sx * sy * if layout.is_color() { 3 } else { 1 };

        let stale = self
            .buffers
            .as_ref()
            .is_none_or(|b| b.len() != n_elements || b.data_type() != cfg.data_type);
        let reset = reset_requested || stale;
        if reset {
            self.buffers = Some(SimBuffers::new(cfg.data_type, n_elements));
            self.state.x_sine1 = Vec::new();
            self.state.x_sine2 = Vec::new();
            self.state.y_sine1 = Vec::new();
            self.state.y_sine2 = Vec::new();
        }

        let bufs = self
            .buffers
            .as_mut()
            .expect("buffers allocated on the reset path above");
        bufs.compute(cfg, &mut self.state, reset, &mut self.rng);

        // The raw array is always the full detector: maxSizeX x maxSizeY.
        let mut dims = vec![NDDimension::new(0); layout.ndims];
        dims[layout.x_dim] = NDDimension::new(sx);
        dims[layout.y_dim] = NDDimension::new(sy);
        if let Some(c) = layout.color_dim {
            dims[c] = NDDimension::new(3);
        }
        let mut raw = NDArray::with_data(dims, bufs.raw_buffer());
        if cfg.sim_mode != SimMode::OffsetNoise {
            // C adds the ColorMode attribute inside the three pattern
            // generators; SimModeOffsetNoise is an empty case, so its arrays
            // carry no ColorMode attribute.
            raw.attributes.add(NDAttribute {
                name: "ColorMode".into(),
                description: "Color mode".into(),
                source: NDAttrSource::Driver,
                value: NDAttrValue::Int32(cfg.color_mode as i32),
                source_impl: None,
            });
        }

        // Extract the region of interest with binning / reversal.
        let g = &cfg.geometry;
        let mut dims_out = vec![NDDimension::new(1); layout.ndims];
        dims_out[layout.x_dim] = NDDimension {
            size: g.size_x as usize,
            offset: g.min_x as usize,
            binning: g.bin_x as usize,
            reverse: g.reverse_x,
        };
        dims_out[layout.y_dim] = NDDimension {
            size: g.size_y as usize,
            offset: g.min_y as usize,
            binning: g.bin_y as usize,
            reverse: g.reverse_y,
        };
        if let Some(c) = layout.color_dim {
            dims_out[c] = NDDimension::new(3);
        }

        pool.convert(&raw, &dims_out, cfg.data_type)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pool() -> NDArrayPool {
        NDArrayPool::new(0)
    }

    fn engine() -> SimEngine {
        SimEngine::new(Rng::new(0xC0FFEE))
    }

    fn pixels_u8(a: &NDArray) -> &Vec<u8> {
        match &a.data {
            NDDataBuffer::U8(v) => v,
            other => panic!("expected U8 buffer, got {:?}", other.data_type()),
        }
    }

    fn pixels_f64(a: &NDArray) -> &Vec<f64> {
        match &a.data {
            NDDataBuffer::F64(v) => v,
            other => panic!("expected F64 buffer, got {:?}", other.data_type()),
        }
    }

    // ---------------------------------------------------------------- casts

    #[test]
    fn from_f64_c_wraps_like_c_not_saturating() {
        // Rust `300.0 as u8` saturates to 255; C `(unsigned char)300.0` gives 44.
        assert_eq!(<u8 as SimPixel>::from_f64_c(300.0), 44);
        assert_eq!(<u8 as SimPixel>::from_f64_c(255.9), 255);
        assert_eq!(<u8 as SimPixel>::from_f64_c(256.0), 0);
        assert_eq!(<i8 as SimPixel>::from_f64_c(-129.0), 127);
        assert_eq!(<u16 as SimPixel>::from_f64_c(65_537.5), 1);
        assert_eq!(<i32 as SimPixel>::from_f64_c(-3.9), -3);
        assert_eq!(<f32 as SimPixel>::from_f64_c(1.5), 1.5f32);
    }

    #[test]
    fn add_and_mul_wrap_for_integers() {
        assert_eq!(<u8 as SimPixel>::add_c(250, 10), 4);
        assert_eq!(<u8 as SimPixel>::mul_c(200, 2), 144);
        assert_eq!(<f64 as SimPixel>::add_c(1.5, 2.25), 3.75);
    }

    // ------------------------------------------------------------- geometry

    #[test]
    fn clamp_negative_min_and_oversize() {
        let mut g = Geometry {
            max_size_x: 100,
            max_size_y: 80,
            min_x: -5,
            min_y: -1,
            size_x: 500,
            size_y: 500,
            bin_x: 1,
            bin_y: 1,
            reverse_x: false,
            reverse_y: false,
        };
        g.clamp();
        assert_eq!((g.min_x, g.min_y), (0, 0));
        assert_eq!((g.size_x, g.size_y), (100, 80));
    }

    #[test]
    fn clamp_min_beyond_max_then_size_collapses_to_one() {
        let mut g = Geometry {
            max_size_x: 10,
            max_size_y: 10,
            min_x: 50,
            min_y: 50,
            size_x: 4,
            size_y: 4,
            bin_x: 1,
            bin_y: 1,
            reverse_x: false,
            reverse_y: false,
        };
        g.clamp();
        // minX clamps to maxSizeX-1 = 9, so sizeX clamps to 10-9 = 1.
        assert_eq!((g.min_x, g.size_x), (9, 1));
        assert_eq!((g.min_y, g.size_y), (9, 1));
    }

    #[test]
    fn clamp_binning_trims_size_to_multiple() {
        let mut g = Geometry {
            max_size_x: 100,
            max_size_y: 100,
            min_x: 0,
            min_y: 0,
            size_x: 10,
            size_y: 10,
            bin_x: 3,
            bin_y: 4,
            reverse_x: false,
            reverse_y: false,
        };
        g.clamp();
        assert_eq!((g.size_x, g.bin_x), (9, 3));
        assert_eq!((g.size_y, g.bin_y), (8, 4));
    }

    #[test]
    fn clamp_binning_larger_than_size_is_reduced() {
        let mut g = Geometry {
            max_size_x: 16,
            max_size_y: 16,
            min_x: 0,
            min_y: 0,
            size_x: 4,
            size_y: 4,
            bin_x: 9,
            bin_y: 0,
            reverse_x: false,
            reverse_y: false,
        };
        g.clamp();
        assert_eq!(g.bin_x, 4);
        assert_eq!(g.bin_y, 1);
    }

    // ---------------------------------------------------------------- layout

    #[test]
    fn layouts_match_c_dimension_assignment() {
        assert_eq!(
            Layout::for_color_mode(NDColorMode::Mono),
            Layout {
                ndims: 2,
                x_dim: 0,
                y_dim: 1,
                color_dim: None
            }
        );
        assert_eq!(
            Layout::for_color_mode(NDColorMode::RGB1),
            Layout {
                ndims: 3,
                x_dim: 1,
                y_dim: 2,
                color_dim: Some(0)
            }
        );
        assert_eq!(
            Layout::for_color_mode(NDColorMode::RGB2),
            Layout {
                ndims: 3,
                x_dim: 0,
                y_dim: 2,
                color_dim: Some(1)
            }
        );
        assert_eq!(
            Layout::for_color_mode(NDColorMode::RGB3),
            Layout {
                ndims: 3,
                x_dim: 0,
                y_dim: 1,
                color_dim: Some(2)
            }
        );
    }

    // ----------------------------------------------------------- linear ramp

    #[test]
    fn linear_ramp_mono_reset_frame_is_absolute() {
        // reset: pixel[i][j] = (T)(incMono * (gainX*j + gainY*i)), incMono = (T)gain
        let mut cfg = SimConfig::defaults(4, 3);
        cfg.gain = 2.0;
        cfg.gain_x = 1.0;
        cfg.gain_y = 10.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_u8(&img);
        let expect: Vec<u8> = (0..3)
            .flat_map(|i| (0..4).map(move |j| (2.0 * (j as f64 + 10.0 * i as f64)) as u8))
            .collect();
        assert_eq!(*p, expect);
    }

    #[test]
    fn linear_ramp_mono_second_frame_increments_by_gain() {
        let mut cfg = SimConfig::defaults(4, 2);
        cfg.gain = 3.0;
        let mut eng = engine();
        let first = eng.compute_image(&cfg, &pool(), true).unwrap();
        let first: Vec<u8> = pixels_u8(&first).clone();
        let second = eng.compute_image(&cfg, &pool(), false).unwrap();
        let second = pixels_u8(&second);
        for (a, b) in first.iter().zip(second.iter()) {
            assert_eq!(*b, a.wrapping_add(3));
        }
    }

    #[test]
    fn linear_ramp_u8_wraps_like_c() {
        // gainX=1, gain=1 over a 300-wide detector: column 256 must wrap to 0.
        let mut cfg = SimConfig::defaults(300, 1);
        cfg.gain_y = 0.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_u8(&img);
        assert_eq!(p[255], 255);
        assert_eq!(p[256], 0);
        assert_eq!(p[299], 43);
    }

    #[test]
    fn linear_ramp_integer_gain_red_truncates_before_multiply() {
        // incRed = (u8)0.5 * incMono = 0 → the red plane stays flat at 0.
        let mut cfg = SimConfig::defaults(2, 2);
        cfg.color_mode = NDColorMode::RGB1;
        cfg.gain = 4.0;
        cfg.gain_red = 0.5;
        cfg.gain_green = 1.0;
        cfg.gain_blue = 2.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_u8(&img);
        // pixel (j=1, i=1): gainX*1 + gainY*1 = 2
        // red: (u8)((u8)0.5*(u8)4 = 0 → 0*2) = 0
        // green: (u8)(4*2) = 8 ; blue: (u8)((u8)2*(u8)4 = 8 → 8*2) = 16
        let idx = 3 * 2 + 3; // row 1, col 1, RGB1
        assert_eq!(p[idx], 0);
        assert_eq!(p[idx + 1], 8);
        assert_eq!(p[idx + 2], 16);
    }

    #[test]
    fn linear_ramp_rgb2_row_and_plane_strides() {
        let mut cfg = SimConfig::defaults(3, 2);
        cfg.color_mode = NDColorMode::RGB2;
        cfg.data_type = NDDataType::Float64;
        cfg.gain = 1.0;
        cfg.gain_x = 1.0;
        cfg.gain_y = 100.0;
        cfg.gain_green = 2.0;
        cfg.gain_blue = 3.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_f64(&img);
        // RGB2 row stride is 3*sizeX, planes are sizeX apart within the row.
        for i in 0..2usize {
            for j in 0..3usize {
                let base = 3 * 3 * i;
                let v = j as f64 + 100.0 * i as f64;
                assert_eq!(p[base + j], v, "red i={i} j={j}");
                assert_eq!(p[base + 3 + j], 2.0 * v, "green i={i} j={j}");
                assert_eq!(p[base + 6 + j], 3.0 * v, "blue i={i} j={j}");
            }
        }
    }

    #[test]
    fn linear_ramp_rgb3_planes_are_contiguous() {
        let mut cfg = SimConfig::defaults(3, 2);
        cfg.color_mode = NDColorMode::RGB3;
        cfg.data_type = NDDataType::Float64;
        cfg.gain_y = 100.0;
        cfg.gain_green = 2.0;
        cfg.gain_blue = 3.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_f64(&img);
        let plane = 3 * 2;
        for i in 0..2usize {
            for j in 0..3usize {
                let v = j as f64 + 100.0 * i as f64;
                assert_eq!(p[i * 3 + j], v);
                assert_eq!(p[plane + i * 3 + j], 2.0 * v);
                assert_eq!(p[2 * plane + i * 3 + j], 3.0 * v);
            }
        }
    }

    // ------------------------------------------------------------ background

    #[test]
    fn offset_only_background_is_constant_and_rotation_invariant() {
        let mut cfg = SimConfig::defaults(4, 4);
        cfg.sim_mode = SimMode::OffsetNoise;
        cfg.offset = 17.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert!(pixels_u8(&img).iter().all(|&v| v == 17));
    }

    #[test]
    fn offset_noise_background_stays_within_offset_plus_noise() {
        let mut cfg = SimConfig::defaults(8, 8);
        cfg.sim_mode = SimMode::OffsetNoise;
        cfg.data_type = NDDataType::Float64;
        cfg.offset = 10.0;
        cfg.noise = 5.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        for &v in pixels_f64(&img) {
            assert!((10.0..15.0).contains(&v), "{v}");
        }
    }

    #[test]
    fn background_frames_are_rotations_of_one_another() {
        let mut cfg = SimConfig::defaults(8, 1);
        cfg.sim_mode = SimMode::OffsetNoise;
        cfg.data_type = NDDataType::Float64;
        cfg.noise = 100.0;
        let mut eng = engine();
        let a = eng.compute_image(&cfg, &pool(), true).unwrap();
        let a: Vec<f64> = pixels_f64(&a).clone();
        let b = eng.compute_image(&cfg, &pool(), false).unwrap();
        let b = pixels_f64(&b);
        let is_rotation = (0..8).any(|k| (0..8).all(|i| b[i] == a[(i + k) % 8]));
        assert!(is_rotation, "second frame must be a rotation of the first");
    }

    #[test]
    fn zero_offset_and_noise_yields_no_background() {
        // OffsetNoise with nothing set: the raw buffer is memset to 0 each frame.
        let mut cfg = SimConfig::defaults(4, 4);
        cfg.sim_mode = SimMode::OffsetNoise;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert!(pixels_u8(&img).iter().all(|&v| v == 0));
    }

    #[test]
    fn offset_truncated_to_pixel_type_can_disable_background() {
        // (u8)0.5 == 0 and noise == 0 → useBackground_ stays false.
        let mut cfg = SimConfig::defaults(2, 2);
        cfg.sim_mode = SimMode::OffsetNoise;
        cfg.offset = 0.5;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert!(pixels_u8(&img).iter().all(|&v| v == 0));

        // ...but the same offset on a float detector does enable it.
        cfg.data_type = NDDataType::Float64;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert!(pixels_f64(&img).iter().all(|&v| v == 0.5));
    }

    #[test]
    fn linear_ramp_over_background_adds_ramp_to_offset() {
        let mut cfg = SimConfig::defaults(4, 1);
        cfg.data_type = NDDataType::Float64;
        cfg.offset = 1000.0;
        cfg.gain = 1.0;
        cfg.gain_y = 0.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_f64(&img);
        assert_eq!(*p, vec![1000.0, 1001.0, 1002.0, 1003.0]);
    }

    // ----------------------------------------------------------------- peaks

    #[test]
    fn peak_full_width_follows_c_ternary() {
        // 2*4*w+1 when it fits, else size-1.
        assert_eq!(peak_full_width(2, 100), 17);
        assert_eq!(peak_full_width(20, 100), 99);
        assert_eq!(peak_full_width(0, 100), 1);
    }

    #[test]
    fn peaks_single_peak_is_centred_on_start_with_gain_amplitude() {
        let mut cfg = SimConfig::defaults(41, 41);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Peaks;
        cfg.gain = 100.0;
        cfg.peak_width_x = 5;
        cfg.peak_width_y = 5;
        cfg.peak_start_x = 20;
        cfg.peak_start_y = 20;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_f64(&img);
        // peakFullWidth = min(2*4*5+1, 41-1) = 40, so the peak spans the whole
        // 41x41 frame and its centre lands on (peakStartX, peakStartY).
        assert_eq!(peak_full_width(5, 41), 40);
        // The Gaussian peaks at gain (exp(0) * exp(0)).
        assert_eq!(p[20 * 41 + 20], 100.0);
        // One sigma out in x: gain * exp(-0.5).
        let expect = 100.0f64 * (-0.5f64).exp();
        assert!(
            (p[20 * 41 + 25] - expect).abs() < 1e-9,
            "{}",
            p[20 * 41 + 25]
        );
        // Corner (0, 0) is four sigma out on both axes: gain * exp(-8) * exp(-8).
        let corner = 100.0f64 * (-8.0f64).exp() * (-8.0f64).exp();
        assert!((p[0] - corner).abs() < 1e-15, "{}", p[0]);
    }

    #[test]
    fn peaks_are_clipped_at_the_image_border() {
        let mut cfg = SimConfig::defaults(16, 16);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Peaks;
        cfg.gain = 50.0;
        cfg.peak_width_x = 1;
        cfg.peak_width_y = 1;
        cfg.peak_start_x = 0;
        cfg.peak_start_y = 0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_f64(&img);
        // Centred at (0,0): the peak's left/top half falls outside and is dropped.
        assert_eq!(p[0], 50.0);
        assert!(p[15] == 0.0 && p[15 * 16] == 0.0);
    }

    #[test]
    fn peaks_grid_places_num_x_by_num_y_peaks_at_step_spacing() {
        let mut cfg = SimConfig::defaults(64, 64);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Peaks;
        cfg.gain = 10.0;
        cfg.peak_width_x = 1;
        cfg.peak_width_y = 1;
        cfg.peak_num_x = 3;
        cfg.peak_num_y = 2;
        cfg.peak_step_x = 20;
        cfg.peak_step_y = 25;
        cfg.peak_start_x = 5;
        cfg.peak_start_y = 6;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_f64(&img);
        for i in 0..2 {
            for j in 0..3 {
                let y = 6 + i * 25;
                let x = 5 + j * 20;
                assert_eq!(p[y * 64 + x], 10.0, "peak at ({x},{y})");
            }
        }
    }

    #[test]
    fn peaks_height_variation_scales_within_the_requested_percentage() {
        let mut cfg = SimConfig::defaults(32, 32);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Peaks;
        cfg.gain = 1000.0;
        cfg.peak_width_x = 1;
        cfg.peak_width_y = 1;
        cfg.peak_start_x = 16;
        cfg.peak_start_y = 16;
        cfg.peak_height_variation = 20.0; // +/- 10 %
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let peak = pixels_f64(&img)[16 * 32 + 16];
        assert!((900.0..=1100.0).contains(&peak), "{peak}");
        assert_ne!(peak, 1000.0, "variation must actually perturb the peak");
    }

    #[test]
    fn peaks_mono_truncates_the_sum_but_rgb_truncates_the_product() {
        // pIn is (u8)gain = 1. gainVariation is 1.0 (no variation), so mono adds
        // 1.0 to 0 → 1. For RGB, gainRed = 0.4 → (u8)(0.4*1.0*1) = 0.
        let mut cfg = SimConfig::defaults(9, 9);
        cfg.sim_mode = SimMode::Peaks;
        cfg.gain = 1.0;
        cfg.peak_width_x = 1;
        cfg.peak_width_y = 1;
        cfg.peak_start_x = 4;
        cfg.peak_start_y = 4;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert_eq!(pixels_u8(&img)[4 * 9 + 4], 1);

        cfg.color_mode = NDColorMode::RGB1;
        cfg.gain_red = 0.4;
        cfg.gain_green = 1.0;
        cfg.gain_blue = 1.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_u8(&img);
        let idx = 3 * 9 * 4 + 3 * 4;
        assert_eq!(p[idx], 0, "red product truncated to 0");
        assert_eq!(p[idx + 1], 1);
        assert_eq!(p[idx + 2], 1);
    }

    #[test]
    fn peaks_accumulate_onto_the_background() {
        let mut cfg = SimConfig::defaults(9, 9);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Peaks;
        cfg.gain = 5.0;
        cfg.offset = 100.0;
        cfg.peak_width_x = 1;
        cfg.peak_width_y = 1;
        cfg.peak_start_x = 4;
        cfg.peak_start_y = 4;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert_eq!(pixels_f64(&img)[4 * 9 + 4], 105.0);
    }

    // ------------------------------------------------------------------ sine

    #[test]
    fn sine_mono_add_matches_the_closed_form() {
        let mut cfg = SimConfig::defaults(4, 2);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Sine;
        cfg.gain = 10.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_f64(&img);

        // Independent restatement of the C formula with the template defaults:
        // sine1 = sin(2*pi*t), sine2 = sin(2*pi*(2t + 0.25)), combined by Add.
        let tau = 2.0 * std::f64::consts::PI;
        let axis = |size: usize| -> Vec<f64> {
            (0..size)
                .map(|k| {
                    let t = k as f64 / size as f64; // gain_x = gain_y = 1, counter starts at 0
                    (t * tau).sin() + ((2.0 * t + 90.0 / 360.0) * tau).sin()
                })
                .collect()
        };
        let xs = axis(4);
        let ys = axis(2);
        for i in 0..2 {
            for j in 0..4 {
                let expect = 10.0 * (ys[i] + xs[j]);
                assert!(
                    (p[i * 4 + j] - expect).abs() < 1e-9,
                    "i={i} j={j}: {} vs {expect}",
                    p[i * 4 + j]
                );
            }
        }
    }

    #[test]
    fn sine_mono_multiply_differs_from_add() {
        let mut cfg = SimConfig::defaults(8, 8);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Sine;
        cfg.gain = 100.0;
        let add = engine().compute_image(&cfg, &pool(), true).unwrap();
        let add: Vec<f64> = pixels_f64(&add).clone();

        cfg.x_sine.operation = SineOperation::Multiply;
        cfg.y_sine.operation = SineOperation::Multiply;
        let mul = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert_ne!(add, *pixels_f64(&mul));
    }

    #[test]
    fn sine_counter_free_runs_across_frames() {
        // Frame 2 continues the phase: xTime for column 0 is sizeX*gainX/sizeX = 1.
        let mut cfg = SimConfig::defaults(4, 1);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Sine;
        cfg.gain = 1.0;
        cfg.y_sine.amplitude1 = 0.0;
        cfg.y_sine.amplitude2 = 0.0;
        let mut eng = engine();
        let f1 = eng.compute_image(&cfg, &pool(), true).unwrap();
        let f1: Vec<f64> = pixels_f64(&f1).clone();
        let f2 = eng.compute_image(&cfg, &pool(), false).unwrap();
        let f2 = pixels_f64(&f2);
        // Sine period in x is exactly one frame width, so frame 2's samples repeat
        // frame 1's — but the raw buffer is zeroed first, so the values match.
        for (a, b) in f1.iter().zip(f2.iter()) {
            assert!((a - b).abs() < 1e-9, "{a} vs {b}");
        }

        // A reset restarts the counter; without one, a non-integer period would
        // advance. Verify the counter actually advanced by using gain_x = 0.25.
        cfg.gain_x = 0.25;
        let mut eng = engine();
        let g1 = eng.compute_image(&cfg, &pool(), true).unwrap();
        let g1: Vec<f64> = pixels_f64(&g1).clone();
        let g2 = eng.compute_image(&cfg, &pool(), false).unwrap();
        assert_ne!(g1, *pixels_f64(&g2));
    }

    #[test]
    fn sine_rgb_uses_sine1_for_red_green_and_sine2_for_blue() {
        let mut cfg = SimConfig::defaults(4, 4);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Sine;
        cfg.color_mode = NDColorMode::RGB1;
        cfg.gain = 100.0;
        // Kill sine-2 → blue must be flat zero while red/green stay non-trivial.
        cfg.x_sine.amplitude2 = 0.0;
        cfg.y_sine.amplitude2 = 0.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        let p = pixels_f64(&img);
        let blue: Vec<f64> = (0..16).map(|k| p[3 * k + 2]).collect();
        assert!(blue.iter().all(|&v| v == 0.0), "{blue:?}");
        let red: Vec<f64> = (0..16).map(|k| p[3 * k]).collect();
        assert!(red.iter().any(|&v| v != 0.0));
    }

    #[test]
    fn sine_accumulates_onto_the_background() {
        let mut cfg = SimConfig::defaults(4, 1);
        cfg.data_type = NDDataType::Float64;
        cfg.sim_mode = SimMode::Sine;
        cfg.gain = 1.0;
        cfg.offset = 500.0;
        cfg.x_sine.amplitude1 = 0.0;
        cfg.x_sine.amplitude2 = 0.0;
        cfg.y_sine.amplitude1 = 0.0;
        cfg.y_sine.amplitude2 = 0.0;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert!(pixels_f64(&img).iter().all(|&v| v == 500.0));
    }

    // ------------------------------------------------------------ ROI / bin

    #[test]
    fn roi_offset_and_size_select_a_sub_region() {
        let mut cfg = SimConfig::defaults(8, 8);
        cfg.data_type = NDDataType::Float64;
        cfg.gain_y = 100.0;
        cfg.geometry.min_x = 2;
        cfg.geometry.min_y = 3;
        cfg.geometry.size_x = 2;
        cfg.geometry.size_y = 2;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert_eq!(img.dims[0].size, 2);
        assert_eq!(img.dims[1].size, 2);
        assert_eq!(*pixels_f64(&img), vec![302.0, 303.0, 402.0, 403.0]);
    }

    #[test]
    fn binning_sums_source_pixels() {
        let mut cfg = SimConfig::defaults(4, 1);
        cfg.data_type = NDDataType::Float64;
        cfg.gain_y = 0.0;
        cfg.geometry.bin_x = 2;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        // Source row is 0,1,2,3 → binned pairs sum to 1 and 5.
        assert_eq!(*pixels_f64(&img), vec![1.0, 5.0]);
    }

    #[test]
    fn reverse_x_flips_the_output_row() {
        let mut cfg = SimConfig::defaults(4, 1);
        cfg.data_type = NDDataType::Float64;
        cfg.gain_y = 0.0;
        cfg.geometry.reverse_x = true;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert_eq!(*pixels_f64(&img), vec![3.0, 2.0, 1.0, 0.0]);
    }

    // -------------------------------------------------------------- metadata

    #[test]
    fn color_mode_attribute_is_present_except_for_offset_noise() {
        let mut cfg = SimConfig::defaults(4, 4);
        cfg.color_mode = NDColorMode::RGB1;
        for mode in [SimMode::LinearRamp, SimMode::Peaks, SimMode::Sine] {
            cfg.sim_mode = mode;
            let img = engine().compute_image(&cfg, &pool(), true).unwrap();
            let attr = img
                .attributes
                .get("ColorMode")
                .expect("ColorMode attribute");
            assert_eq!(attr.value.as_i64(), Some(NDColorMode::RGB1 as i64));
        }
        cfg.sim_mode = SimMode::OffsetNoise;
        let img = engine().compute_image(&cfg, &pool(), true).unwrap();
        assert!(img.attributes.get("ColorMode").is_none());
    }

    #[test]
    fn rgb_dimension_order_follows_the_color_mode() {
        let mut cfg = SimConfig::defaults(8, 4);
        for (mode, dims) in [
            (NDColorMode::RGB1, [3, 8, 4]),
            (NDColorMode::RGB2, [8, 3, 4]),
            (NDColorMode::RGB3, [8, 4, 3]),
        ] {
            cfg.color_mode = mode;
            let img = engine().compute_image(&cfg, &pool(), true).unwrap();
            let got: Vec<usize> = img.dims.iter().map(|d| d.size).collect();
            assert_eq!(got, dims.to_vec(), "{mode:?}");
        }
    }

    #[test]
    fn every_nd_data_type_produces_a_matching_buffer() {
        for ord in 0..10u8 {
            let dt = NDDataType::from_ordinal(ord).unwrap();
            let mut cfg = SimConfig::defaults(4, 4);
            cfg.data_type = dt;
            for mode in [
                SimMode::LinearRamp,
                SimMode::Peaks,
                SimMode::Sine,
                SimMode::OffsetNoise,
            ] {
                cfg.sim_mode = mode;
                let img = engine().compute_image(&cfg, &pool(), true).unwrap();
                assert_eq!(img.data.data_type(), dt, "{dt:?} / {mode:?}");
                assert_eq!(img.data.len(), 16);
            }
        }
    }

    #[test]
    fn changing_data_type_forces_a_buffer_reset() {
        let mut cfg = SimConfig::defaults(4, 4);
        cfg.gain = 1.0;
        let mut eng = engine();
        eng.compute_image(&cfg, &pool(), true).unwrap();
        eng.compute_image(&cfg, &pool(), false).unwrap(); // ramp is now 2*base

        cfg.data_type = NDDataType::Float64;
        let img = eng.compute_image(&cfg, &pool(), false).unwrap();
        // Reallocation implies a reset: the absolute ramp, not the accumulated one.
        assert_eq!(pixels_f64(&img)[1], 1.0);
    }
}
