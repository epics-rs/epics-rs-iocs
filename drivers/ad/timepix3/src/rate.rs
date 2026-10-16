//! The per-stream acquisition rate C computes from the frame numbers it
//! receives (serval_stream.cpp:248-283, :899-936, histogram_io.cpp:251-292).

use std::collections::VecDeque;

/// When C moves its "previous frame" forward.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Advance {
    /// On every frame after the first (PrvImg, PrvHst).
    Always,
    /// Only on a frame that produced a rate sample (Img).
    OnSample,
}

/// One frame's outcome.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    /// The moving-average rate, posted when `due`.
    pub rate: f64,
    /// A rate sample was taken and a second has passed since the last post.
    pub due: bool,
    /// `(expected, got)` when the frame number skipped ahead.
    pub lost: Option<(i32, i32)>,
}

#[derive(Debug, Clone)]
pub struct RateTracker {
    max_samples: usize,
    advance: Advance,
    /// The previous frame number and its arrival time; `None` is C's
    /// `firstFrameReceived == false`.
    previous: Option<(i32, f64)>,
    samples: VecDeque<f64>,
    rate: f64,
    /// Survives a reset, as C's `lastRateUpdateTime` does.
    last_post: f64,
}

impl RateTracker {
    pub fn new(max_samples: usize, advance: Advance) -> Self {
        Self {
            max_samples,
            advance,
            previous: None,
            samples: VecDeque::new(),
            rate: 0.0,
            last_post: 0.0,
        }
    }

    /// Account for `frame` arriving at `now` (seconds).
    pub fn frame(&mut self, frame: i32, now: f64) -> Observation {
        let Some((prev_frame, prev_time)) = self.previous else {
            self.previous = Some((frame, now));
            self.rate = 0.0;
            return Observation {
                rate: 0.0,
                due: false,
                lost: None,
            };
        };
        let frame_diff = frame.wrapping_sub(prev_frame);
        let time_diff = now - prev_time;
        let lost = (frame_diff > 1).then(|| (prev_frame.wrapping_add(1), frame));
        let mut due = false;
        let sampled = frame_diff > 0 && time_diff > 0.0;
        if sampled {
            self.samples.push_back(f64::from(frame_diff) / time_diff);
            if self.samples.len() > self.max_samples {
                self.samples.pop_front();
            }
            self.rate = self.samples.iter().sum::<f64>() / self.samples.len() as f64;
            if now - self.last_post >= 1.0 {
                self.last_post = now;
                due = true;
            }
        }
        if sampled || self.advance == Advance::Always {
            self.previous = Some((frame, now));
        }
        Observation {
            rate: self.rate,
            due,
            lost,
        }
    }

    pub fn sample_count(&self) -> usize {
        self.samples.len()
    }

    /// C `resetAcquisitionRuntimeState` (acquisition_runtime.cpp:108-153).
    pub fn reset(&mut self) {
        self.previous = None;
        self.samples.clear();
        self.rate = 0.0;
    }

    /// C `acquireStart` clears only `prvImgFirstFrameReceived_`
    /// (acquire.cpp:416).
    pub fn restart(&mut self) {
        self.previous = None;
    }
}

/// The three streams' trackers, sized as C sizes them (ADTimePix.h:903, :965;
/// histogram_io.cpp:275 shadows ADTimePix.h:1013's 10 with 100).
#[derive(Debug, Clone)]
pub struct Rates {
    pub prv_img: RateTracker,
    pub img: RateTracker,
    pub prv_hst: RateTracker,
}

impl Default for Rates {
    fn default() -> Self {
        Self {
            prv_img: RateTracker::new(10, Advance::Always),
            img: RateTracker::new(10, Advance::OnSample),
            prv_hst: RateTracker::new(100, Advance::Always),
        }
    }
}

impl Rates {
    pub fn reset(&mut self) {
        self.prv_img.reset();
        self.img.reset();
        self.prv_hst.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_first_frame_only_starts_the_clock() {
        let mut r = RateTracker::new(10, Advance::Always);
        let o = r.frame(5, 100.0);
        assert_eq!((o.rate, o.due, o.lost), (0.0, false, None));
        let o = r.frame(6, 100.5);
        assert_eq!((o.rate, o.due), (2.0, true));
    }

    #[test]
    fn the_rate_is_a_moving_average_posted_once_a_second() {
        let mut r = RateTracker::new(2, Advance::Always);
        r.frame(0, 10.0);
        assert!(r.frame(1, 11.0).due); // 1 Hz
        let o = r.frame(3, 11.5); // 4 Hz, frame 2 lost
        assert_eq!((o.rate, o.due, o.lost), (2.5, false, Some((2, 3))));
        let o = r.frame(4, 12.5); // 1 Hz; the 1 Hz first sample fell out
        assert_eq!((o.rate, o.due), (2.5, true));
    }

    #[test]
    fn img_advances_only_on_a_sample_and_prv_img_always() {
        // BothCounters repeats a frame number: Img keeps measuring from the
        // last sample, PrvImg restarts from the repeat.
        let mut img = RateTracker::new(10, Advance::OnSample);
        let mut prv = RateTracker::new(10, Advance::Always);
        for r in [&mut img, &mut prv] {
            r.frame(1, 0.0);
            r.frame(1, 1.0);
        }
        assert_eq!(img.frame(2, 2.0).rate, 0.5);
        assert_eq!(prv.frame(2, 2.0).rate, 1.0);
    }

    #[test]
    fn a_reset_clears_the_samples_but_not_the_post_clock() {
        let mut r = RateTracker::new(10, Advance::Always);
        r.frame(0, 0.0);
        assert!(r.frame(1, 1.0).due);
        r.reset();
        r.frame(0, 1.2);
        let o = r.frame(1, 1.7);
        assert_eq!((o.rate, o.due), (2.0, false));
    }
}
