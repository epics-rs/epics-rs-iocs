//! Global configuration defaults (`iocshVariables.h`, `iocshIntegration.cpp:44-57`).
//!
//! In C these are `variable(...)` entries in the .dbd that an st.cmd may assign
//! between `dbLoadRecords` calls, so a later database picks up a different
//! default than an earlier one. The same mutability is needed here: link parsing
//! happens during `dbLoadRecords`, so the value read is whatever the script set
//! most recently.

use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};

/// Store an `f64` in an `AtomicU64` by bit pattern; the values are plain
/// configuration scalars, so bit-exact round-tripping is all that is needed.
#[derive(Debug)]
pub struct AtomicF64(AtomicU64);

impl AtomicF64 {
    const fn new(v: f64) -> Self {
        Self(AtomicU64::new(v.to_bits()))
    }

    pub fn get(&self) -> f64 {
        f64::from_bits(self.0.load(Ordering::Relaxed))
    }

    pub fn set(&self, v: f64) {
        self.0.store(v.to_bits(), Ordering::Relaxed);
    }
}

/// Connect timeout / reconnect attempt interval [s].
pub static CONNECT_TIMEOUT: AtomicF64 = AtomicF64::new(5.0);
/// Batch size for operations (0 = no limit, don't batch).
pub static MAX_OPERATIONS_PER_SERVICE_CALL: AtomicI64 = AtomicI64::new(0);
/// Subscription publishing interval [ms].
pub static DEFAULT_PUBLISH_INTERVAL: AtomicF64 = AtomicF64::new(100.0);
/// Monitored item sampling interval [ms] (-1 = use publishing interval).
pub static DEFAULT_SAMPLING_INTERVAL: AtomicF64 = AtomicF64::new(-1.0);
/// Server side queue size (1 = no queuing).
pub static DEFAULT_SERVER_QUEUE_SIZE: AtomicI64 = AtomicI64::new(1);
/// Discard policy on queue overrun (1 = discard oldest; 0 = newest).
pub static DEFAULT_DISCARD_OLDEST: AtomicI64 = AtomicI64::new(1);
/// Timestamp selection (1 = use server time; 0 = use source time).
pub static DEFAULT_USE_SERVER_TIME: AtomicI64 = AtomicI64::new(1);
/// Output record handling (1 = bidirectional).
pub static DEFAULT_OUTPUT_READBACK: AtomicI64 = AtomicI64::new(1);
/// Client queue size factor (multiplied with the server side size).
pub static CLIENT_QUEUE_SIZE_FACTOR: AtomicF64 = AtomicF64::new(1.5);
/// Minimum client queue size.
pub static MINIMUM_CLIENT_QUEUE_SIZE: AtomicI64 = AtomicI64::new(3);

/// Set one of the `opcua_*` iocsh variables by its C name.
///
/// Returns `false` for an unknown name, which the `var` iocsh command reports.
pub fn set_variable(name: &str, value: &str) -> bool {
    fn as_f64(v: &str) -> Option<f64> {
        v.trim().parse().ok()
    }
    fn as_i64(v: &str) -> Option<i64> {
        v.trim().parse().ok()
    }

    match name {
        "opcua_ConnectTimeout" => as_f64(value).map(|v| CONNECT_TIMEOUT.set(v)).is_some(),
        "opcua_MaxOperationsPerServiceCall" => as_i64(value)
            .map(|v| MAX_OPERATIONS_PER_SERVICE_CALL.store(v, Ordering::Relaxed))
            .is_some(),
        "opcua_DefaultPublishInterval" => as_f64(value)
            .map(|v| DEFAULT_PUBLISH_INTERVAL.set(v))
            .is_some(),
        "opcua_DefaultSamplingInterval" => as_f64(value)
            .map(|v| DEFAULT_SAMPLING_INTERVAL.set(v))
            .is_some(),
        "opcua_DefaultServerQueueSize" => as_i64(value)
            .map(|v| DEFAULT_SERVER_QUEUE_SIZE.store(v, Ordering::Relaxed))
            .is_some(),
        "opcua_DefaultDiscardOldest" => as_i64(value)
            .map(|v| DEFAULT_DISCARD_OLDEST.store(v, Ordering::Relaxed))
            .is_some(),
        "opcua_DefaultUseServerTime" => as_i64(value)
            .map(|v| DEFAULT_USE_SERVER_TIME.store(v, Ordering::Relaxed))
            .is_some(),
        "opcua_DefaultOutputReadback" => as_i64(value)
            .map(|v| DEFAULT_OUTPUT_READBACK.store(v, Ordering::Relaxed))
            .is_some(),
        "opcua_ClientQueueSizeFactor" => as_f64(value)
            .map(|v| CLIENT_QUEUE_SIZE_FACTOR.set(v))
            .is_some(),
        "opcua_MinimumClientQueueSize" => as_i64(value)
            .map(|v| MINIMUM_CLIENT_QUEUE_SIZE.store(v, Ordering::Relaxed))
            .is_some(),
        _ => false,
    }
}
