//! Constants, enums and the epicsEvent equivalent used by the Pilatus port.
//!
//! Values mirror `pilatusDetector.cpp` verbatim.

use std::time::Duration;

use parking_lot::{Condvar, Mutex};

/// C `MAX_MESSAGE_SIZE` — camserver request/reply buffer size.
pub const MAX_MESSAGE_SIZE: usize = 256;
/// C `MAX_FILENAME_LEN`.
pub const MAX_FILENAME_LEN: usize = 256;
/// C `MAX_HEADER_STRING_LEN` — `getStringParam` copies at most this many bytes
/// (including the NUL) out of `HEADERSTRING`.
pub const MAX_HEADER_STRING_LEN: usize = 68;
/// C `MAX_BAD_PIXELS`.
pub const MAX_BAD_PIXELS: usize = 100;
/// C `ASYN_POLL_TIME` — per-iteration socket read timeout inside `readCamserver`.
pub const ASYN_POLL_TIME: f64 = 0.01;
/// C `CAMSERVER_DEFAULT_TIMEOUT`.
pub const CAMSERVER_DEFAULT_TIMEOUT: f64 = 1.0;
/// C `CAMSERVER_ACQUIRE_TIMEOUT` — slack added on top of the exposure time when
/// waiting for the asynchronous `7 OK` completion reply.
pub const CAMSERVER_ACQUIRE_TIMEOUT: f64 = 10.0;
/// C `CAMSERVER_RESET_POWER_TIMEOUT`.
pub const CAMSERVER_RESET_POWER_TIMEOUT: f64 = 30.0;
/// C `FILE_READ_DELAY` — poll interval while waiting for the image file.
pub const FILE_READ_DELAY: f64 = 0.01;

/// C `gainStrings[]`, indexed by `(int)(ADGain + 0.5)` clamped to `0..=3`.
pub const GAIN_STRINGS: [&str; 4] = ["lowG", "midG", "highG", "uhighG"];

/// C `PilatusTriggerMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum TriggerMode {
    Internal = 0,
    ExternalEnable = 1,
    ExternalTrigger = 2,
    MultipleExternalTrigger = 3,
    Alignment = 4,
}

impl TriggerMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Internal),
            1 => Some(Self::ExternalEnable),
            2 => Some(Self::ExternalTrigger),
            3 => Some(Self::MultipleExternalTrigger),
            4 => Some(Self::Alignment),
            _ => None,
        }
    }

    /// The camserver acquisition verb for this mode (C `pilatusTask` switch).
    pub fn command(self) -> &'static str {
        match self {
            Self::Internal | Self::Alignment => "Exposure",
            Self::ExternalEnable => "ExtEnable",
            Self::ExternalTrigger => "ExtTrigger",
            Self::MultipleExternalTrigger => "ExtMTrigger",
        }
    }
}

/// Subset of `asynStatus` the camserver helpers return, carrying C's numeric
/// codes so the `status > 1` / `!status` tests in `pilatusTask` port directly.
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
    /// The `asynStatus` ordinal, so C's `if (status > 1)` ports verbatim.
    pub fn code(self) -> i32 {
        match self {
            Self::Success => 0,
            Self::Timeout => 1,
            Self::Error => 3,
        }
    }

    /// C's `if (status)` — anything but `asynSuccess`.
    pub fn is_err(self) -> bool {
        self != Self::Success
    }
}

/// One entry of C's `badPixelMap[]`. Indices are computed at bad-pixel-file
/// read time from the then-current `NDArraySizeX` / `NDArraySizeY`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BadPixel {
    pub bad_index: i64,
    pub replace_index: i64,
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
            // `wait_for` may return spuriously; the flag is authoritative.
            let _ = result;
            false
        }
    }
}

/// Convert a C `double` seconds timeout into a `Duration`, clamping the
/// negative "wait forever" encoding (never used by this driver) to zero.
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
    fn cam_status_codes_match_asyn_status() {
        assert_eq!(CamStatus::Success.code(), 0);
        assert_eq!(CamStatus::Timeout.code(), 1);
        assert_eq!(CamStatus::Error.code(), 3);
        // C: `if (status > 1) acquire = 0;` must fire for Error, not Timeout.
        assert!(CamStatus::Error.code() > 1);
        assert!(CamStatus::Timeout.code() <= 1);
    }

    #[test]
    fn trigger_mode_commands() {
        assert_eq!(TriggerMode::Internal.command(), "Exposure");
        assert_eq!(TriggerMode::ExternalEnable.command(), "ExtEnable");
        assert_eq!(TriggerMode::ExternalTrigger.command(), "ExtTrigger");
        assert_eq!(
            TriggerMode::MultipleExternalTrigger.command(),
            "ExtMTrigger"
        );
        assert_eq!(TriggerMode::Alignment.command(), "Exposure");
    }

    #[test]
    fn event_signal_is_consumed_by_one_wait() {
        let ev = Event::new();
        ev.signal();
        assert!(ev.wait_timeout(Duration::from_millis(1)));
        assert!(!ev.wait_timeout(Duration::from_millis(1)));
    }
}
