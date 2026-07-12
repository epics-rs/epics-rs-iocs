//! The state the port driver and the background threads share.
//!
//! In C this is a pile of `ADTimePix` members guarded by three `epicsMutex`es
//! — and the stream workers reach straight into the asyn parameter library
//! without holding the port lock (UPSTREAM DEFECT, serval_stream.cpp:582-700,
//! :1447-1533, histogram_io.cpp:430-700). Here the workers own only this
//! struct; every parameter update goes through the port handle, which is the
//! single owner of the parameter library.

use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use parking_lot::Mutex;

use crate::accum::Accumulator;

/// What the port driver asks the background threads to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    AcquisitionStarted,
    AcquisitionStopped,
    /// C `checkConnection` on demand (`TPX3_REFRESH_CONNECTION`).
    RefreshConnection,
    /// C's `TPX3_HEALTH` write: re-read the dashboard, the detector and the
    /// measurement config.
    RefreshStatus,
}

/// Where the three preview channels stream from, as Serval's `Base` PVs give
/// it (`tcp://[listen@]host:port`, or a `file://` path when the channel writes
/// to disk instead).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct StreamPaths {
    pub prv_img: Option<String>,
    pub img: Option<String>,
    pub prv_hst: Option<String>,
}

pub struct Shared {
    /// The Img channel's accumulation (asyn addresses 1-3).
    pub img: Mutex<Accumulator>,
    /// The PrvHst channel's accumulation (asyn addresses 4-7).
    pub hst: Mutex<Accumulator>,
    /// The mask waveform the driver draws into (C mutates the *record's*
    /// buffer; the driver owns it here).
    pub mask: Mutex<Vec<i32>>,
    pub streams: Mutex<StreamPaths>,
    orientation: AtomicI32,
    acquiring: AtomicBool,
}

impl Default for Shared {
    fn default() -> Self {
        Self::new()
    }
}

impl Shared {
    pub fn new() -> Self {
        Self {
            img: Mutex::new(Accumulator::new()),
            hst: Mutex::new(Accumulator::new()),
            mask: Mutex::new(Vec::new()),
            streams: Mutex::new(StreamPaths::default()),
            orientation: AtomicI32::new(0),
            acquiring: AtomicBool::new(false),
        }
    }

    pub fn set_stream_paths(
        &self,
        prv_img: Option<String>,
        img: Option<String>,
        prv_hst: Option<String>,
    ) {
        *self.streams.lock() = StreamPaths {
            prv_img,
            img,
            prv_hst,
        };
    }

    pub fn stream_paths(&self) -> StreamPaths {
        self.streams.lock().clone()
    }

    pub fn set_orientation(&self, orientation: i32) {
        self.orientation.store(orientation, Ordering::Release);
    }

    pub fn orientation(&self) -> i32 {
        self.orientation.load(Ordering::Acquire)
    }

    pub fn set_acquiring(&self, on: bool) {
        self.acquiring.store(on, Ordering::Release);
    }

    pub fn acquiring(&self) -> bool {
        self.acquiring.load(Ordering::Acquire)
    }

    /// Take the mask out for editing, growing it to the geometry first.
    pub fn take_mask(&self, len: usize) -> Vec<i32> {
        let mut mask = self.mask.lock();
        if mask.len() != len {
            mask.clear();
            mask.resize(len, 0);
        }
        std::mem::take(&mut *mask)
    }

    pub fn put_mask(&self, mask: Vec<i32>) {
        *self.mask.lock() = mask;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_mask_grows_to_the_geometry_and_survives_a_round_trip() {
        let s = Shared::new();
        let mut m = s.take_mask(4);
        assert_eq!(m, vec![0, 0, 0, 0]);
        m[2] = 1;
        s.put_mask(m);
        assert_eq!(s.take_mask(4), vec![0, 0, 1, 0]);

        // A geometry change discards the old mask rather than reinterpreting it.
        s.put_mask(vec![0, 0, 1, 0]);
        assert_eq!(s.take_mask(2), vec![0, 0]);
    }

    #[test]
    fn the_stream_paths_round_trip() {
        let s = Shared::new();
        s.set_stream_paths(Some("tcp://h:1".into()), None, Some("tcp://h:2".into()));
        let p = s.stream_paths();
        assert_eq!(p.prv_img.as_deref(), Some("tcp://h:1"));
        assert_eq!(p.img, None);
        assert_eq!(p.prv_hst.as_deref(), Some("tcp://h:2"));
    }
}
