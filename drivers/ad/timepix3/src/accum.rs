//! Frame accumulation: the running sum and the sum of the last N frames
//! (port of `img_accumulation.cpp` + the accumulation half of
//! `histogram_io.cpp`).
//!
//! The Img channel and the PrvHst channel accumulate identically — one is a 2-D
//! pixel array, the other a 1-D bin array — so both use this one accumulator
//! and the geometry lives in the caller.

use std::collections::VecDeque;

use crate::window::{Configuration, RollingSumStatus, RollingWindowSum};

/// C clamps `imgFramesToSum_` / `prvHstFramesToSum_` into this range
/// (ADTimePix.cpp:604).
pub const MIN_FRAMES_TO_SUM: usize = 1;
pub const MAX_FRAMES_TO_SUM: usize = 100_000;
/// C `DEFAULT_RETENTION_LIMIT_MB` (ADTimePix.h:987).
pub const DEFAULT_RETENTION_LIMIT_MB: usize = 512;
/// C `IMG_MAX_PROCESSING_TIME_SAMPLES` (ADTimePix.h:985).
const PROCESSING_TIME_SAMPLES: usize = 10;

/// What one frame produced.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Update {
    /// The running sum after this frame — always fresh.
    pub running: Vec<u64>,
    /// The window sum, present only on the frames where the update interval
    /// elapses and the window took the frame (C `should_update_sum`,
    /// serval_stream.cpp:1180-1182).
    pub sum_n: Option<Vec<u64>>,
    /// The frames the window holds (C `FramesSummed`).
    pub frames_summed: usize,
    /// The window's new configuration, when this frame's geometry, the
    /// requested frames or the budget changed it.
    pub window_config: Option<Configuration>,
    /// The window's verdict on this frame; `None` when it is unconfigured.
    pub window_push: Option<RollingSumStatus>,
    /// Counts in *this* frame.
    pub frame_counts: u64,
    /// Set when this frame's geometry differed from the previous one and the
    /// accumulation was restarted (C's "size mismatch", serval_stream.cpp:735).
    pub reset: bool,
}

/// C `ADTimePix3Numeric::RangeState` (numeric_range.h:17), ordered as C
/// promotes it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum RangeState {
    #[default]
    Ok,
    OutputClamped,
    AccumulatorSaturated,
}

impl RangeState {
    /// C `rangeStateName` (numeric_range.cpp:78).
    pub fn name(self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::OutputClamped => "Published output clamped",
            Self::AccumulatorSaturated => "Accumulator saturated",
        }
    }
}

/// C `clampAverageToUInt32` (numeric_range.cpp:38): `sum / divisor`, capped at
/// `u32::MAX`, and whether it was capped.
pub fn clamp_average_to_u32(sum: u64, divisor: u64) -> (u32, bool) {
    if divisor == 0 {
        return (0, false);
    }
    match u32::try_from(sum / divisor) {
        Ok(v) => (v, false),
        Err(_) => (u32::MAX, true),
    }
}

#[derive(Debug)]
pub struct Accumulator {
    len: usize,
    running: Vec<u64>,
    /// The last frame, held apart from the window, which may retain none.
    current: Vec<u32>,
    window: RollingWindowSum,
    frames_to_sum: usize,
    retention_limit_mb: usize,
    update_interval: usize,
    since_update: usize,
    frame_count: u64,
    total_counts: u64,
    range: RangeState,
    /// The caller's geometry of the last frame (C `get_width`/`get_height`).
    shape: Vec<usize>,
    value_bytes: usize,
    /// The last processing times, in ms (C `imgProcessingTimeSamples_`).
    processing_ms: VecDeque<f64>,
    /// When the processing time and memory usage were last posted.
    processing_posted: f64,
    memory_posted: f64,
    /// Img also posts both on a full sample set or window; PrvHst only on
    /// its clock (serval_stream.cpp:1245-1246, :1362-1364 vs
    /// histogram_io.cpp:869, :876).
    posts_when_full: bool,
    /// C `prvHstTimeMsBuffer_.size()`: grown to bins + 1, never shrunk but
    /// by a reset (histogram_io.cpp:640-642, ADTimePix.cpp:1379).
    time_axis_len: usize,
}

impl Default for Accumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl Accumulator {
    pub fn new() -> Self {
        Self {
            len: 0,
            running: Vec::new(),
            current: Vec::new(),
            window: RollingWindowSum::default(),
            // C's defaults (ADTimePix.cpp:1495, :1978).
            frames_to_sum: 10,
            retention_limit_mb: DEFAULT_RETENTION_LIMIT_MB,
            update_interval: 1,
            since_update: 0,
            frame_count: 0,
            total_counts: 0,
            range: RangeState::Ok,
            shape: Vec::new(),
            value_bytes: 4,
            processing_ms: VecDeque::new(),
            processing_posted: 0.0,
            memory_posted: 0.0,
            posts_when_full: true,
            time_axis_len: 0,
        }
    }

    /// The PrvHst channel's accumulator.
    pub fn prv_hst() -> Self {
        Self {
            posts_when_full: false,
            ..Self::new()
        }
    }

    /// serval_stream.cpp:1222-1252, histogram_io.cpp:848-873: the mean of
    /// the last ten processing times, returned when C posts it (a second
    /// since the last post, or for Img a full sample set).
    pub fn record_processing_time(&mut self, ms: f64, now: f64) -> Option<f64> {
        self.processing_ms.push_back(ms);
        if self.processing_ms.len() > PROCESSING_TIME_SAMPLES {
            self.processing_ms.pop_front();
        }
        let mean = self.processing_ms.iter().sum::<f64>() / self.processing_ms.len() as f64;
        let full = self.posts_when_full && self.processing_ms.len() >= PROCESSING_TIME_SAMPLES;
        (now - self.processing_posted >= 1.0 || full).then(|| {
            self.processing_posted = now;
            mean
        })
    }

    /// serval_stream.cpp:1360-1371, histogram_io.cpp:875-881: memory usage
    /// is posted every `interval` seconds, and for Img on every frame while
    /// the window is full.
    pub fn memory_due(&mut self, now: f64, interval: f64) -> bool {
        let full = self.posts_when_full
            && self.window.effective_frames() > 0
            && self.window.frame_count() >= self.window.effective_frames();
        let due = now - self.memory_posted >= interval || full;
        if due {
            self.memory_posted = now;
        }
        due
    }

    pub fn processing_samples(&self) -> usize {
        self.processing_ms.len()
    }

    /// Grow the time axis to `len` values, as the PrvHst frame path does.
    pub fn grow_time_axis(&mut self, len: usize) {
        self.time_axis_len = self.time_axis_len.max(len);
    }

    pub fn time_axis_len(&self) -> usize {
        self.time_axis_len
    }

    /// Elements per frame; 0 before the first frame and after a reset.
    pub fn elements(&self) -> usize {
        self.len
    }

    /// Bytes per value of the last frame `add_shaped` recorded (C's
    /// `ImageData` holds a uint8 frame as UINT16).
    pub fn value_bytes(&self) -> usize {
        self.value_bytes
    }

    /// The geometry `add_shaped` last recorded; empty after a reset.
    pub fn shape(&self) -> &[usize] {
        &self.shape
    }

    /// C `imgRangeState_` / `prvHstRangeState_`.
    pub fn range(&self) -> RangeState {
        self.range
    }

    /// C `updateImgRangeState` / `updatePrvHstRangeState` (ADTimePix.cpp:1322,
    /// :1337): promote the latch, and return the state when C would post its
    /// RangeSaturated/RangeStatus pair.
    pub fn report_range(&mut self, state: RangeState) -> Option<RangeState> {
        let promoted = if state == RangeState::Ok {
            RangeState::Ok
        } else {
            state.max(self.range)
        };
        if promoted == self.range && state != RangeState::Ok {
            return None;
        }
        self.range = promoted;
        Some(promoted)
    }

    /// The running sum, one element per pixel or bin.
    pub fn running(&self) -> &[u64] {
        &self.running
    }

    /// The window sum (C `imgWindowSum_.sum()` / `prvHstWindowSum_.sum()`).
    pub fn window_sum(&self) -> Vec<u64> {
        self.window.sum().to_vec()
    }

    pub fn window(&self) -> &RollingWindowSum {
        &self.window
    }

    pub fn frames_to_sum(&self) -> usize {
        self.frames_to_sum
    }

    /// C reconfigures a window that has a geometry at once
    /// (ADTimePix.cpp:755-858); the configuration is returned for posting.
    pub fn set_frames_to_sum(&mut self, n: i32) -> Option<Configuration> {
        self.frames_to_sum = usize::try_from(n)
            .unwrap_or(MIN_FRAMES_TO_SUM)
            .clamp(MIN_FRAMES_TO_SUM, MAX_FRAMES_TO_SUM);
        self.reconfigure()
    }

    pub fn set_retention_limit_mb(&mut self, mb: i32) -> Option<Configuration> {
        self.retention_limit_mb = usize::try_from(mb).unwrap_or(1).max(1);
        self.reconfigure()
    }

    pub fn retention_limit_mb(&self) -> usize {
        self.retention_limit_mb
    }

    fn budget_bytes(&self) -> usize {
        self.retention_limit_mb * 1024 * 1024
    }

    fn reconfigure(&mut self) -> Option<Configuration> {
        let elements = self.window.element_count();
        (elements > 0).then(|| {
            self.window
                .configure(elements, self.frames_to_sum, self.budget_bytes())
        })
    }

    /// An interval below 1 would make `since_update >= interval` true on every
    /// frame *and* leave the counter meaningless; C never validates it.
    pub fn set_update_interval(&mut self, n: i32) {
        self.update_interval = usize::try_from(n).unwrap_or(1).max(1);
    }

    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }

    pub fn total_counts(&self) -> u64 {
        self.total_counts
    }

    pub fn buffered_frames(&self) -> usize {
        self.window.frame_count()
    }

    /// The last frame, as delivered.
    pub fn current_frame(&self) -> Option<&[u32]> {
        (!self.current.is_empty()).then_some(self.current.as_slice())
    }

    /// C `resetImgAccumulation` / `resetPrvHstAccumulation`: the last frame
    /// and the window's configuration survive.
    pub fn reset(&mut self) {
        self.running.clear();
        self.window.reset();
        self.len = 0;
        self.since_update = 0;
        self.frame_count = 0;
        self.total_counts = 0;
        self.shape.clear();
        self.processing_ms.clear();
        self.time_axis_len = 0;
    }

    /// `add`, recording the frame's geometry for the arrays built later from
    /// the accumulation and its storage for the memory estimate.
    pub fn add_shaped(&mut self, frame: &[u32], shape: &[usize], value_bytes: usize) -> Update {
        let update = self.add(frame);
        self.shape = shape.to_vec();
        self.value_bytes = value_bytes;
        update
    }

    pub fn add(&mut self, frame: &[u32]) -> Update {
        let mut reset = false;
        if frame.len() != self.len {
            // A geometry change invalidates every accumulated frame.
            self.running = vec![0; frame.len()];
            self.window.clear();
            self.len = frame.len();
            self.frame_count = 0;
            self.total_counts = 0;
            self.since_update = 0;
            reset = true;
        }

        let mut frame_counts: u64 = 0;
        for (acc, &v) in self.running.iter_mut().zip(frame) {
            // C caps the running sum at UINT64_MAX (img_accumulation.cpp:151).
            *acc = acc.saturating_add(u64::from(v));
            frame_counts = frame_counts.saturating_add(u64::from(v));
        }
        self.total_counts = self.total_counts.saturating_add(frame_counts);
        self.frame_count += 1;
        self.current.clear();
        self.current.extend_from_slice(frame);

        // serval_stream.cpp:1132-1164, histogram_io.cpp:607-632.
        let budget = self.budget_bytes();
        let window_config = (self.window.element_count() != self.len
            || self.window.requested_frames() != self.frames_to_sum
            || self.window.budget_bytes() != budget)
            .then(|| self.window.configure(self.len, self.frames_to_sum, budget));
        let window_push = window_config
            .is_none_or(|c| c.status == RollingSumStatus::Ok)
            .then(|| self.window.push(frame));

        self.since_update += 1;
        let sum_n = (window_push == Some(RollingSumStatus::Ok)
            && self.since_update >= self.update_interval
            && self.window.frame_count() > 0)
            .then(|| {
                self.since_update = 0;
                self.window_sum()
            });

        Update {
            running: self.running.clone(),
            sum_n,
            frames_summed: self.window.frame_count(),
            frame_counts,
            reset,
            window_config,
            window_push,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_running_sum_accumulates_every_frame() {
        let mut a = Accumulator::new();
        let u = a.add(&[1, 2, 3]);
        assert_eq!(u.running, vec![1, 2, 3]);
        assert_eq!(u.frame_counts, 6);
        assert!(u.reset);
        let u = a.add(&[10, 20, 30]);
        assert_eq!(u.running, vec![11, 22, 33]);
        assert!(!u.reset);
        assert_eq!(a.frame_count(), 2);
        assert_eq!(a.total_counts(), 66);
    }

    #[test]
    fn the_running_sum_saturates_instead_of_wrapping() {
        let mut a = Accumulator::new();
        for _ in 0..8 {
            a.add(&[u32::MAX]);
        }
        // 8 * (2^32-1) fits; nothing has saturated yet.
        assert_eq!(a.total_counts(), 8 * u64::from(u32::MAX));
    }

    #[test]
    fn the_sum_of_n_keeps_only_the_last_n_frames() {
        let mut a = Accumulator::new();
        a.set_frames_to_sum(2);
        a.add(&[1, 1]);
        a.add(&[2, 2]);
        let u = a.add(&[4, 4]);
        // Frames 2 and 3 only.
        assert_eq!(u.sum_n, Some(vec![6, 6]));
        assert_eq!(u.frames_summed, 2);
        // The running sum still holds all three.
        assert_eq!(u.running, vec![7, 7]);
    }

    #[test]
    fn the_sum_of_n_only_recomputes_on_the_interval() {
        let mut a = Accumulator::new();
        a.set_frames_to_sum(10);
        a.set_update_interval(3);
        assert!(a.add(&[1]).sum_n.is_none());
        assert!(a.add(&[1]).sum_n.is_none());
        let u = a.add(&[1]);
        assert_eq!(u.sum_n, Some(vec![3]));
        assert_eq!(u.frames_summed, 3);
        assert!(a.add(&[1]).sum_n.is_none());
    }

    #[test]
    fn frames_to_sum_is_clamped_to_the_c_range() {
        let mut a = Accumulator::new();
        a.set_frames_to_sum(0);
        assert_eq!(a.frames_to_sum(), MIN_FRAMES_TO_SUM);
        a.set_frames_to_sum(-5);
        assert_eq!(a.frames_to_sum(), MIN_FRAMES_TO_SUM);
        a.set_frames_to_sum(1_000_000);
        assert_eq!(a.frames_to_sum(), MAX_FRAMES_TO_SUM);
        a.set_frames_to_sum(7);
        assert_eq!(a.frames_to_sum(), 7);
    }

    #[test]
    fn shrinking_frames_to_sum_trims_the_buffer_immediately() {
        let mut a = Accumulator::new();
        a.set_frames_to_sum(5);
        for i in 1..=5 {
            a.add(&[i]);
        }
        assert_eq!(a.buffered_frames(), 5);
        a.set_frames_to_sum(2);
        assert_eq!(a.buffered_frames(), 2);
        let u = a.add(&[10]);
        // Frames 5 and 10 (4 was trimmed).
        assert_eq!(u.sum_n, Some(vec![15]));
    }

    /// The window holds what the retention budget allows beside its sum
    /// (rolling_window_sum.cpp:44-121); the last frame is kept regardless.
    #[test]
    fn the_retention_budget_caps_the_sum_of_n() {
        let mut a = Accumulator::new();
        // 64 Ki elements in 1 MiB: a 512 KiB sum and two 256 KiB frames.
        a.set_retention_limit_mb(1);
        let frame = |v| vec![v; 65536];
        a.add(&frame(1));
        a.add(&frame(2));
        let u = a.add(&frame(4));
        let c = a.window();
        assert_eq!((c.effective_frames(), u.frames_summed), (2, 2));
        assert_eq!(u.sum_n.as_deref().map(|s| s[0]), Some(6));
        assert_eq!(u.running[0], 7);

        // A budget below one frame leaves no window and no sum of N.
        let mut a = Accumulator::new();
        a.set_retention_limit_mb(1);
        let u = a.add(&vec![1; 131072]);
        assert_eq!(
            u.window_config.map(|c| c.status),
            Some(RollingSumStatus::BudgetTooSmall)
        );
        assert_eq!((u.window_push, u.sum_n, u.frames_summed), (None, None, 0));
        assert_eq!(a.current_frame().map(<[u32]>::len), Some(131072));
    }

    #[test]
    fn a_zero_update_interval_cannot_stall_the_sum() {
        let mut a = Accumulator::new();
        a.set_update_interval(0);
        assert!(a.add(&[1]).sum_n.is_some());
        a.set_update_interval(-3);
        assert!(a.add(&[1]).sum_n.is_some());
    }

    #[test]
    fn a_geometry_change_restarts_the_accumulation() {
        let mut a = Accumulator::new();
        a.add(&[1, 2, 3]);
        a.add(&[1, 2, 3]);
        let u = a.add(&[5, 5]);
        assert!(u.reset);
        assert_eq!(u.running, vec![5, 5]);
        assert_eq!(u.sum_n, Some(vec![5, 5]));
        assert_eq!(a.frame_count(), 1);
        assert_eq!(a.total_counts(), 10);
    }

    #[test]
    fn reset_clears_everything() {
        let mut a = Accumulator::new();
        a.add(&[1, 2]);
        a.reset();
        assert_eq!(a.frame_count(), 0);
        assert_eq!(a.total_counts(), 0);
        assert_eq!(a.buffered_frames(), 0);
        assert_eq!(a.current_frame(), Some(&[1, 2][..]));
        let u = a.add(&[9, 9]);
        assert_eq!(u.running, vec![9, 9]);
    }
}
