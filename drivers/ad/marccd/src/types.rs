//! Constants, the marccd task-state bit helpers, mode enums and the
//! `epicsEvent` equivalent.
//!
//! Values mirror `marCCD.cpp` verbatim.

use std::time::Duration;

use parking_lot::{Condvar, Mutex};

/// C `MAX_MESSAGE_SIZE` — server request/reply buffer size.
pub const MAX_MESSAGE_SIZE: usize = 256;
/// C `MAX_FILENAME_LEN`.
pub const MAX_FILENAME_LEN: usize = 256;
/// C `MARCCD_SERVER_TIMEOUT` — every `writeServer` / `readServer` uses this.
pub const MARCCD_SERVER_TIMEOUT: f64 = 1.0;
/// C `FILE_READ_DELAY` — poll interval while waiting for the TIFF file.
pub const FILE_READ_DELAY: f64 = 0.01;
/// C `MARCCD_POLL_DELAY` — poll interval between `get_state` calls in the
/// task-status wait loops.
pub const MARCCD_POLL_DELAY: f64 = 0.01;

// --- C task numbers --------------------------------------------------------
pub const TASK_ACQUIRE: i32 = 0;
pub const TASK_READ: i32 = 1;
pub const TASK_CORRECT: i32 = 2;
pub const TASK_WRITE: i32 = 3;
pub const TASK_DEZINGER: i32 = 4;
pub const TASK_SERIES: i32 = 5;

// --- C task-status bits ----------------------------------------------------
pub const TASK_STATUS_QUEUED: i32 = 0x1;
pub const TASK_STATUS_EXECUTING: i32 = 0x2;
pub const TASK_STATUS_ERROR: i32 = 0x4;
pub const TASK_STATUS_RESERVED: i32 = 0x8;

// --- C task states ---------------------------------------------------------
pub const TASK_STATE_IDLE: i32 = 0;
pub const TASK_STATE_ERROR: i32 = 7;
/// C `TASK_STATE_BUSY` — "busy interpreting command" (also used as the
/// `>= 8` threshold in the wait loops).
pub const TASK_STATE_BUSY: i32 = 8;

const STATE_MASK: i32 = 0xf;
const STATUS_MASK: i32 = 0xf;

/// C `TASK_STATUS_MASK(task)`.
#[inline]
fn task_status_mask(task: i32) -> i32 {
    STATUS_MASK << (4 * (task + 1))
}

/// C `TASK_STATE(current_status)` — the low nibble.
#[inline]
pub fn task_state(current_status: i32) -> i32 {
    current_status & STATE_MASK
}

/// C `TASK_STATUS(current_status, task)` — the nibble for `task`.
#[inline]
pub fn task_status(current_status: i32, task: i32) -> i32 {
    (current_status & task_status_mask(task)) >> (4 * (task + 1))
}

/// C `TEST_TASK_STATUS(current_status, task, status)`.
#[inline]
pub fn test_task_status(current_status: i32, task: i32, status: i32) -> i32 {
    task_status(current_status, task) & status
}

/// C `marCCDFrameType_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum FrameType {
    Normal = 0,
    Background = 1,
    Raw = 2,
    DoubleCorrelation = 3,
}

impl FrameType {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Normal),
            1 => Some(Self::Background),
            2 => Some(Self::Raw),
            3 => Some(Self::DoubleCorrelation),
            _ => None,
        }
    }
}

/// C `marCCDImageMode_t`. The first three share the base `ADImageMode_t`
/// ordinals; series modes extend them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ImageMode {
    Single = 0,
    Multiple = 1,
    Continuous = 2,
    SeriesTriggered = 3,
    SeriesTimed = 4,
}

impl ImageMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Single),
            1 => Some(Self::Multiple),
            2 => Some(Self::Continuous),
            3 => Some(Self::SeriesTriggered),
            4 => Some(Self::SeriesTimed),
            _ => None,
        }
    }
}

/// C `imageModeStrings[]`.
pub const IMAGE_MODE_STRINGS: [&str; 5] = [
    "Single",
    "Multiple",
    "Continuous",
    "Series triggered",
    "Series timed",
];
/// C `numImageModes[]`, indexed by `serverMode` (1 or 2; index 0 unused).
pub const NUM_IMAGE_MODES: [usize; 3] = [3, 3, 5];

/// C `marCCDTriggerMode_t`. `Internal`/`Frame` map to the base
/// `ADTriggerInternal`/`ADTriggerExternal` ordinals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum TriggerMode {
    Internal = 0,
    Frame = 1,
    Bulb = 2,
    Timed = 3,
}

impl TriggerMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Internal),
            1 => Some(Self::Frame),
            2 => Some(Self::Bulb),
            3 => Some(Self::Timed),
            _ => None,
        }
    }
}

/// C `triggerModeStrings[]`.
pub const TRIGGER_MODE_STRINGS: [&str; 4] = ["Internal", "Frame", "Bulb", "Timed"];
/// C `numTriggerModes[]`.
pub const NUM_TRIGGER_MODES: [usize; 3] = [2, 2, 4];

/// C `gateModeStrings[]`.
pub const GATE_MODE_STRINGS: [&str; 2] = ["None", "Gated"];
/// C `numGateModes[]`.
pub const NUM_GATE_MODES: [usize; 3] = [0, 0, 2];

/// C `readoutModeStrings[]`.
pub const READOUT_MODE_STRINGS: [&str; 6] = [
    "Standard",
    "High gain",
    "Low noise",
    "HDR",
    "Turbo",
    "HDR16",
];
/// C `numReadoutModes[]`.
pub const NUM_READOUT_MODES: [usize; 3] = [0, 0, 6];

/// Subset of `asynStatus` the server helpers return, carrying C's numeric codes
/// so the `if (status)` tests port directly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CamStatus {
    /// `asynSuccess` (0)
    Success,
    /// `asynTimeout` (1)
    Timeout,
    /// `asynError` (3)
    Error,
}

impl CamStatus {
    /// C's `if (status)` — anything but `asynSuccess`.
    pub fn is_err(self) -> bool {
        self != Self::Success
    }
}

/// Binary event with the semantics of `epicsEvent` created `epicsEventEmpty`:
/// `signal` sets the flag, a wait consumes it.
#[derive(Default)]
pub struct Event {
    signaled: Mutex<bool>,
    cv: Condvar,
}

impl Event {
    pub fn new() -> Self {
        Self::default()
    }

    /// `epicsEventSignal`.
    pub fn signal(&self) {
        *self.signaled.lock() = true;
        self.cv.notify_all();
    }

    /// `epicsEventWait` — block until signaled, then consume the signal.
    pub fn wait(&self) {
        let mut guard = self.signaled.lock();
        while !*guard {
            self.cv.wait(&mut guard);
        }
        *guard = false;
    }

    /// `epicsEventWaitWithTimeout` — returns `true` on `epicsEventWaitOK`
    /// (signal received and consumed), `false` on timeout.
    pub fn wait_timeout(&self, timeout: Duration) -> bool {
        let mut guard = self.signaled.lock();
        if *guard {
            *guard = false;
            return true;
        }
        let result = self.cv.wait_for(&mut guard, timeout);
        if *guard {
            *guard = false;
            true
        } else {
            let _ = result;
            false
        }
    }

    /// `epicsEventTryWait` — consume a pending signal without blocking; returns
    /// `true` if one was consumed.
    pub fn try_wait(&self) -> bool {
        let mut guard = self.signaled.lock();
        if *guard {
            *guard = false;
            true
        } else {
            false
        }
    }
}

/// Convert a C `double` seconds timeout into a `Duration`, clamping any
/// non-positive encoding to zero.
pub fn secs(v: f64) -> Duration {
    if v.is_finite() && v > 0.0 {
        Duration::from_secs_f64(v)
    } else {
        Duration::ZERO
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn task_state_is_low_nibble() {
        assert_eq!(task_state(0x8), 8);
        assert_eq!(task_state(0x27), 7);
        assert_eq!(task_state(0x0), 0);
    }

    #[test]
    fn task_status_extracts_correct_nibble() {
        // Pack: state=0, acquire(task 0) nibble at bits 4..8 = 0x2 (executing).
        let s = 0x2 << 4;
        assert_eq!(task_status(s, TASK_ACQUIRE), 0x2);
        assert_eq!(task_status(s, TASK_READ), 0x0);

        // readout (task 1) nibble at bits 8..12.
        let s = 0x1 << 8;
        assert_eq!(task_status(s, TASK_READ), 0x1);

        // series (task 5) nibble at bits 24..28.
        let s = 0x4 << 24;
        assert_eq!(task_status(s, TASK_SERIES), 0x4);
    }

    #[test]
    fn test_task_status_masks_bits() {
        let s = (TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) << 4; // acquire
        assert_ne!(
            test_task_status(s, TASK_ACQUIRE, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED),
            0
        );
        assert_eq!(test_task_status(s, TASK_ACQUIRE, TASK_STATUS_ERROR), 0);
        assert_eq!(test_task_status(s, TASK_READ, TASK_STATUS_EXECUTING), 0);
    }

    #[test]
    fn event_signal_is_consumed_by_one_wait() {
        let ev = Event::new();
        ev.signal();
        assert!(ev.wait_timeout(Duration::from_millis(1)));
        assert!(!ev.wait_timeout(Duration::from_millis(1)));
    }

    #[test]
    fn event_try_wait_consumes() {
        let ev = Event::new();
        assert!(!ev.try_wait());
        ev.signal();
        assert!(ev.try_wait());
        assert!(!ev.try_wait());
    }
}
