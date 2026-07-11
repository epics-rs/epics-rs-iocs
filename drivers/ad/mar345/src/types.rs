//! Constants, the mar345 mode/status enums, the `epicsEvent` equivalent and
//! small time helpers. Values mirror `mar345.cpp` verbatim.

use std::time::Duration;

use parking_lot::{Condvar, Mutex};

/// C `MAX_MESSAGE_SIZE` — server request/reply buffer size.
pub const MAX_MESSAGE_SIZE: usize = 256;
/// C `MAX_FILENAME_LEN`.
pub const MAX_FILENAME_LEN: usize = 256;
/// C `MAR345_SOCKET_TIMEOUT` — the write / single-read socket timeout.
pub const MAR345_SOCKET_TIMEOUT: f64 = 1.0;
/// C `MAR345_COMMAND_TIMEOUT` — total wait for a slow command's "Ended o.k.".
pub const MAR345_COMMAND_TIMEOUT: f64 = 180.0;
/// C `MAR345_POLL_DELAY` — the poll interval / per-read timeout in the wait
/// loops.
pub const MAR345_POLL_DELAY: f64 = 0.01;

/// C `imageSizes[2][4]`, indexed `[res][size]`: the square pixel dimension of a
/// scan. `res` is [`Resolution`], `size` is [`ScanSize`].
pub const IMAGE_SIZES: [[i32; 4]; 2] = [[1800, 2400, 3000, 3450], [1200, 1600, 2000, 2300]];

/// C `mar345Mode_t` — the slow-task the `mar345Task` worker should run next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Mode {
    Idle = 0,
    Erase = 1,
    Acquire = 2,
    Change = 3,
}

impl Mode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Idle),
            1 => Some(Self::Erase),
            2 => Some(Self::Acquire),
            3 => Some(Self::Change),
            _ => None,
        }
    }
}

/// C `mar345Status_t` — written to the standard `ADStatus` parameter; the
/// `DetectorState_RBV` mbbi in `mar345.template` maps these ordinals to labels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Status {
    Idle = 0,
    Expose = 1,
    Scan = 2,
    Erase = 3,
    ChangeMode = 4,
    Aborting = 5,
    Error = 6,
    Waiting = 7,
}

/// C `mar345TriggerMode_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum TriggerMode {
    Internal = 0,
    External = 1,
    Alignment = 2,
}

/// C `mar345EraseMode_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum EraseMode {
    None = 0,
    Before = 1,
    After = 2,
}

impl EraseMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::None),
            1 => Some(Self::Before),
            2 => Some(Self::After),
            _ => None,
        }
    }
}

/// C `mar345Size_t` — the readout diameter (180 / 240 / 300 / 345 mm).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ScanSize {
    S180 = 0,
    S240 = 1,
    S300 = 2,
    S345 = 3,
}

/// C `mar345Res_t` — the readout resolution (0.10 / 0.15 mm).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Resolution {
    R100 = 0,
    R150 = 1,
}

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
        let _ = self.cv.wait_for(&mut guard, timeout);
        if *guard {
            *guard = false;
            true
        } else {
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
    fn image_sizes_match_c() {
        assert_eq!(
            IMAGE_SIZES[Resolution::R100 as usize][ScanSize::S345 as usize],
            3450
        );
        assert_eq!(
            IMAGE_SIZES[Resolution::R150 as usize][ScanSize::S180 as usize],
            1200
        );
        assert_eq!(
            IMAGE_SIZES[Resolution::R100 as usize][ScanSize::S180 as usize],
            1800
        );
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

    #[test]
    fn mode_roundtrip() {
        for v in 0..4 {
            assert_eq!(Mode::from_i32(v).unwrap() as i32, v);
        }
        assert_eq!(Mode::from_i32(4), None);
    }
}
