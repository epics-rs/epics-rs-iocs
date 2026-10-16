//! C `RollingWindowSum` (rolling_window_sum.cpp): the exact sum of the last
//! N frames, with the retained frames and the sum held to a byte budget.

use std::collections::VecDeque;

/// C `RollingSumStatus`; `name` is `rollingSumStatusName`
/// (rolling_window_sum.cpp:30-42).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum RollingSumStatus {
    Ok,
    #[default]
    InvalidArgument,
    SizeOverflow,
    BudgetTooSmall,
    AllocationFailure,
    FrameSizeMismatch,
    NumericOverflow,
}

impl RollingSumStatus {
    pub fn name(self) -> &'static str {
        match self {
            Self::Ok => "ok",
            Self::InvalidArgument => "invalid argument",
            Self::SizeOverflow => "size overflow",
            Self::BudgetTooSmall => "budget too small",
            Self::AllocationFailure => "allocation failure",
            Self::FrameSizeMismatch => "frame size mismatch",
            Self::NumericOverflow => "numeric overflow",
        }
    }
}

/// C `RollingSumConfiguration`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Configuration {
    pub status: RollingSumStatus,
    pub requested_frames: usize,
    pub effective_frames: usize,
    pub element_count: usize,
    pub budget_bytes: usize,
    pub required_bytes: usize,
    pub limited: bool,
}

#[derive(Debug, Default)]
pub struct RollingWindowSum {
    element_count: usize,
    requested_frames: usize,
    effective_frames: usize,
    budget_bytes: usize,
    sum: Vec<u64>,
    frames: VecDeque<Vec<u32>>,
}

impl RollingWindowSum {
    /// C `configure` (rolling_window_sum.cpp:44-121): as many of the
    /// requested frames as the budget holds beside the u64 sum; a new
    /// geometry drops the retained frames.
    pub fn configure(
        &mut self,
        element_count: usize,
        requested_frames: usize,
        budget_bytes: usize,
    ) -> Configuration {
        let mut result = Configuration {
            requested_frames,
            element_count,
            budget_bytes,
            ..Configuration::default()
        };
        let base = result;
        let fail = move |this: &mut Self, status, required_bytes| {
            this.clear();
            Configuration {
                status,
                required_bytes,
                ..base
            }
        };
        if element_count == 0 || requested_frames == 0 || budget_bytes == 0 {
            return fail(self, RollingSumStatus::InvalidArgument, 0);
        }
        let (Some(sum_bytes), Some(frame_bytes)) =
            (element_count.checked_mul(8), element_count.checked_mul(4))
        else {
            return fail(self, RollingSumStatus::SizeOverflow, 0);
        };
        let Some(minimum) = sum_bytes.checked_add(frame_bytes) else {
            return fail(self, RollingSumStatus::SizeOverflow, 0);
        };
        if budget_bytes < minimum {
            return fail(self, RollingSumStatus::BudgetTooSmall, minimum);
        }
        let effective_frames = requested_frames.min((budget_bytes - sum_bytes) / frame_bytes);
        let Some(required) = effective_frames
            .checked_mul(frame_bytes)
            .and_then(|retained| retained.checked_add(sum_bytes))
        else {
            return fail(self, RollingSumStatus::SizeOverflow, 0);
        };
        result.required_bytes = required;

        if self.element_count != element_count || self.sum.len() != element_count {
            let mut replacement = Vec::new();
            if replacement.try_reserve_exact(element_count).is_err() {
                return fail(self, RollingSumStatus::AllocationFailure, required);
            }
            replacement.resize(element_count, 0);
            self.frames.clear();
            self.sum = replacement;
            self.element_count = element_count;
        }
        self.requested_frames = requested_frames;
        self.effective_frames = effective_frames;
        self.budget_bytes = budget_bytes;
        while self.frames.len() > self.effective_frames {
            self.evict_oldest();
        }

        result.status = RollingSumStatus::Ok;
        result.effective_frames = effective_frames;
        result.limited = effective_frames < requested_frames;
        result
    }

    /// C `push` (rolling_window_sum.cpp:123-167): a frame that would carry
    /// any element past `u64::MAX` is refused and leaves the window as it was.
    pub fn push(&mut self, values: &[u32]) -> RollingSumStatus {
        if values.len() != self.element_count || !self.configured() {
            return RollingSumStatus::FrameSizeMismatch;
        }
        let mut next = Vec::new();
        if next.try_reserve_exact(values.len()).is_err() {
            return RollingSumStatus::AllocationFailure;
        }
        next.extend_from_slice(values);

        let evict = self.frames.len() == self.effective_frames;
        let oldest = evict.then(|| &self.frames[0]);
        let overflows = self.sum.iter().zip(&next).enumerate().any(|(i, (&s, &v))| {
            let base = s - oldest.map_or(0, |o| u64::from(o[i]));
            base.checked_add(u64::from(v)).is_none()
        });
        if overflows {
            return RollingSumStatus::NumericOverflow;
        }
        if evict {
            self.evict_oldest();
        }
        for (s, &v) in self.sum.iter_mut().zip(&next) {
            *s += u64::from(v);
        }
        self.frames.push_back(next);
        RollingSumStatus::Ok
    }

    fn evict_oldest(&mut self) {
        if let Some(oldest) = self.frames.pop_front() {
            for (s, &v) in self.sum.iter_mut().zip(&oldest) {
                *s -= u64::from(v);
            }
        }
    }

    /// C `reset`: drop the frames and zero the sum, keeping the configuration.
    pub fn reset(&mut self) {
        self.frames.clear();
        self.sum.iter_mut().for_each(|s| *s = 0);
    }

    /// C `clear`: drop the frames, the sum and the geometry.
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    pub fn sum(&self) -> &[u64] {
        &self.sum
    }

    pub fn element_count(&self) -> usize {
        self.element_count
    }

    pub fn frame_count(&self) -> usize {
        self.frames.len()
    }

    pub fn requested_frames(&self) -> usize {
        self.requested_frames
    }

    pub fn effective_frames(&self) -> usize {
        self.effective_frames
    }

    pub fn budget_bytes(&self) -> usize {
        self.budget_bytes
    }

    /// C `memoryBytes` (rolling_window_sum.cpp:191-198).
    pub fn memory_bytes(&self) -> usize {
        self.sum.capacity() * 8 + self.frames.iter().map(|f| f.capacity() * 4).sum::<usize>()
    }

    pub fn configured(&self) -> bool {
        self.effective_frames > 0 && !self.sum.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIB: usize = 1024 * 1024;

    #[test]
    fn the_budget_caps_the_retained_frames() {
        let mut w = RollingWindowSum::default();
        // 64 Ki elements: a 512 KiB sum and 256 KiB per frame.
        let c = w.configure(65536, 10, 2 * MIB);
        assert_eq!(
            (c.status, c.effective_frames, c.limited, c.required_bytes),
            (RollingSumStatus::Ok, 6, true, 512 * 1024 + 6 * 256 * 1024)
        );
        let c = w.configure(65536, 10, 512 * 1024);
        assert_eq!(
            (c.status, c.required_bytes),
            (RollingSumStatus::BudgetTooSmall, 768 * 1024)
        );
        assert!(!w.configured());
        assert_eq!(
            w.configure(0, 10, MIB).status,
            RollingSumStatus::InvalidArgument
        );
    }

    #[test]
    fn the_sum_covers_the_last_effective_frames() {
        let mut w = RollingWindowSum::default();
        w.configure(2, 2, MIB);
        for f in [[1, 1], [2, 2], [4, 4]] {
            assert_eq!(w.push(&f), RollingSumStatus::Ok);
        }
        assert_eq!((w.sum(), w.frame_count()), (&[6, 6][..], 2));
        // Shrinking the window evicts at once; a new geometry starts over.
        w.configure(2, 1, MIB);
        assert_eq!(w.sum(), &[4, 4]);
        w.configure(3, 1, MIB);
        assert_eq!((w.sum(), w.frame_count()), (&[0, 0, 0][..], 0));
        assert_eq!(w.push(&[1, 2]), RollingSumStatus::FrameSizeMismatch);
    }

    #[test]
    fn a_frame_that_would_overflow_the_sum_is_refused_whole() {
        let mut w = RollingWindowSum::default();
        w.configure(2, 3, MIB);
        w.sum = vec![u64::MAX - 1, 0];
        assert_eq!(w.push(&[2, 7]), RollingSumStatus::NumericOverflow);
        assert_eq!((w.sum(), w.frame_count()), (&[u64::MAX - 1, 0][..], 0));
    }

    #[test]
    fn reset_keeps_the_configuration_and_clear_does_not() {
        let mut w = RollingWindowSum::default();
        w.configure(1, 4, MIB);
        w.push(&[5]);
        w.reset();
        assert_eq!(
            (w.sum(), w.frame_count(), w.effective_frames()),
            (&[0][..], 0, 4)
        );
        w.clear();
        assert!(!w.configured());
        assert_eq!(w.element_count(), 0);
    }
}
