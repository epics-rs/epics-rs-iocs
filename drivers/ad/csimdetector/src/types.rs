//! Constants and marker types from `ADCSimDetector.h`.

/// `MAX_SIGNALS` (ADCSimDetector.h:26). The 2-D array is
/// `[MAX_SIGNALS, numTimePoints]` and the port serves `MAX_SIGNALS + 1`
/// addresses: 0 for the 2-D array, `1..=MAX_SIGNALS` for the 1-D signals.
pub const MAX_SIGNALS: usize = 8;

/// Commands sent to the simulation task, one per C `epicsEvent`.
///
/// `startEventId_` and `stopEventId_` are independent binary semaphores; each
/// is modelled by its own capacity-1 channel, so a signal on one never
/// satisfies a wait on the other and a second signal while one is pending is a
/// no-op (exactly `epicsEventSignal` on a full `epicsEventFull` event).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Signal;
