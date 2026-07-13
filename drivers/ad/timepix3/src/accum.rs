//! Frame accumulation: the running sum and the sum of the last N frames
//! (port of `img_accumulation.cpp` + the accumulation half of
//! `histogram_io.cpp`).
//!
//! The Img channel and the PrvHst channel accumulate identically — one is a 2-D
//! pixel array, the other a 1-D bin array — so both use this one accumulator
//! and the geometry lives in the caller.

use std::collections::VecDeque;

/// C clamps `imgFramesToSum_` / `prvHstFramesToSum_` into this range
/// (ADTimePix.cpp:604).
pub const MIN_FRAMES_TO_SUM: usize = 1;
pub const MAX_FRAMES_TO_SUM: usize = 100_000;

/// What one frame produced.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Update {
    /// The running sum after this frame — always fresh.
    pub running: Vec<u64>,
    /// The sum of the buffered frames, present only on the frames where the
    /// update interval elapses (C `should_update_sum`, serval_stream.cpp:805).
    pub sum_n: Option<Vec<u64>>,
    /// How many frames the `sum_n` covers.
    pub frames_summed: usize,
    /// Counts in *this* frame.
    pub frame_counts: u64,
    /// Set when this frame's geometry differed from the previous one and the
    /// accumulation was restarted (C's "size mismatch", serval_stream.cpp:735).
    pub reset: bool,
}

#[derive(Debug)]
pub struct Accumulator {
    len: usize,
    running: Vec<u64>,
    buffer: VecDeque<Vec<u32>>,
    frames_to_sum: usize,
    update_interval: usize,
    since_update: usize,
    frame_count: u64,
    total_counts: u64,
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
            buffer: VecDeque::new(),
            // C's defaults (ADTimePix.cpp:1495).
            frames_to_sum: 10,
            update_interval: 1,
            since_update: 0,
            frame_count: 0,
            total_counts: 0,
        }
    }

    pub fn frames_to_sum(&self) -> usize {
        self.frames_to_sum
    }

    pub fn set_frames_to_sum(&mut self, n: i32) {
        self.frames_to_sum = usize::try_from(n)
            .unwrap_or(MIN_FRAMES_TO_SUM)
            .clamp(MIN_FRAMES_TO_SUM, MAX_FRAMES_TO_SUM);
        while self.buffer.len() > self.frames_to_sum {
            self.buffer.pop_front();
        }
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
        self.buffer.len()
    }

    /// The last frame, as delivered.
    pub fn current_frame(&self) -> Option<&[u32]> {
        self.buffer.back().map(Vec::as_slice)
    }

    pub fn reset(&mut self) {
        self.running.clear();
        self.buffer.clear();
        self.len = 0;
        self.since_update = 0;
        self.frame_count = 0;
        self.total_counts = 0;
    }

    pub fn add(&mut self, frame: &[u32]) -> Update {
        let mut reset = false;
        if frame.len() != self.len {
            // A geometry change invalidates every accumulated frame.
            self.running = vec![0; frame.len()];
            self.buffer.clear();
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

        self.buffer.push_back(frame.to_vec());
        while self.buffer.len() > self.frames_to_sum {
            self.buffer.pop_front();
        }

        self.since_update += 1;
        let (sum_n, frames_summed) = if self.since_update >= self.update_interval {
            self.since_update = 0;
            let mut sum = vec![0u64; self.len];
            for f in &self.buffer {
                for (s, &v) in sum.iter_mut().zip(f) {
                    *s = s.saturating_add(u64::from(v));
                }
            }
            let n = self.buffer.len();
            (Some(sum), n)
        } else {
            (None, 0)
        };

        Update {
            running: self.running.clone(),
            sum_n,
            frames_summed,
            frame_counts,
            reset,
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
        assert!(a.current_frame().is_none());
        let u = a.add(&[9, 9]);
        assert_eq!(u.running, vec![9, 9]);
    }
}
