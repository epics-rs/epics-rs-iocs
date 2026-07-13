//! Constants, the SPECS run-mode/value-type enums, the `epicsEvent`
//! equivalent, and a small time helper. Values mirror `specsAnalyser.h`/
//! `specsAnalyser.cpp` verbatim.

use std::time::Duration;

use parking_lot::{Condvar, Mutex};

/// C `SPECS_TIMEOUT` (`specsAnalyser.h:33`) — the write/read socket timeout
/// for every wire exchange.
pub const SOCKET_TIMEOUT: f64 = 10.0;
/// C `SPECS_UPDATE_RATE` (`specsAnalyser.h:35`) — the acquisition
/// status-poll interval.
pub const UPDATE_RATE: f64 = 0.1;
/// C `SPECS_MAX_STRING` (`specsAnalyser.h:31`) — the raw reply buffer size
/// for a single read (including any continuation read).
pub const MAX_MESSAGE_SIZE: usize = 4096;
/// C `const int maxValues=1000000` (`specsAnalyser.cpp:536`) — caps how many
/// data points a single `GetAcquisitionData` request may span.
pub const MAX_VALUES: i32 = 1_000_000;

/// C `runMode` values (`specsAnalyser.h:41-44`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum RunMode {
    Fat = 0,
    Sfat = 1,
    Frr = 2,
    Fe = 3,
}

impl RunMode {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Fat),
            1 => Some(Self::Sfat),
            2 => Some(Self::Frr),
            3 => Some(Self::Fe),
            _ => None,
        }
    }
}

/// C `SPECSValueType_t` (`GetAnalyzerParameterInfo`'s `ValueType` field,
/// `specsAnalyser.h:47-50`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpecsValueType {
    Double,
    Integer,
    String,
    Bool,
}

impl SpecsValueType {
    /// `SpecsAnalyser::getAnalyserParameterType`'s `data["ValueType"]` match
    /// (`specsAnalyser.cpp:1506-1513`). Upstream leaves `type` unset (whatever
    /// the caller's stack garbage was) when the wire string matches none of
    /// the four; `None` here is the safe equivalent.
    pub fn from_wire(s: &str) -> Option<Self> {
        match s {
            "double" => Some(Self::Double),
            "integer" => Some(Self::Integer),
            "string" => Some(Self::String),
            "bool" => Some(Self::Bool),
            _ => None,
        }
    }
}

/// Binary event with the semantics of `epicsEvent` created `epicsEventEmpty`:
/// `signal` sets the flag, a wait consumes it. C's `startEventId_` /
/// `stopEventId_`.
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
}

/// Convert a C `double` seconds timeout/delay into a `Duration`, clamping any
/// non-positive encoding to zero (`specsAnalyser.cpp:692-693`'s `delay >= 0.0`
/// guard, generalised the way the mar345 port already does it).
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
    fn run_mode_roundtrip() {
        for v in 0..4 {
            assert_eq!(RunMode::from_i32(v).unwrap() as i32, v);
        }
        assert_eq!(RunMode::from_i32(4), None);
    }

    #[test]
    fn value_type_from_wire() {
        assert_eq!(
            SpecsValueType::from_wire("double"),
            Some(SpecsValueType::Double)
        );
        assert_eq!(
            SpecsValueType::from_wire("integer"),
            Some(SpecsValueType::Integer)
        );
        assert_eq!(
            SpecsValueType::from_wire("string"),
            Some(SpecsValueType::String)
        );
        assert_eq!(
            SpecsValueType::from_wire("bool"),
            Some(SpecsValueType::Bool)
        );
        assert_eq!(SpecsValueType::from_wire("nonsense"), None);
    }

    #[test]
    fn event_signal_is_consumed_by_one_wait() {
        let ev = Event::new();
        ev.signal();
        assert!(ev.wait_timeout(Duration::from_millis(1)));
        assert!(!ev.wait_timeout(Duration::from_millis(1)));
    }

    #[test]
    fn secs_clamps_non_positive_to_zero() {
        assert_eq!(secs(-1.0), Duration::ZERO);
        assert_eq!(secs(0.0), Duration::ZERO);
        assert_eq!(secs(f64::NAN), Duration::ZERO);
        assert_eq!(secs(1.5), Duration::from_secs_f64(1.5));
    }
}
