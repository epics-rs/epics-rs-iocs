//! The acquisition lifecycle (C `acquisition_coordinator.cpp`): the one owner
//! of the phase and of the resources a start attempt acquired.

use std::time::Duration;

use parking_lot::{Condvar, Mutex};

/// C `Phase`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Phase {
    #[default]
    Idle,
    Starting,
    Running,
    Stopping,
    Fault,
}

impl Phase {
    /// C `phaseName`.
    pub fn name(self) -> &'static str {
        match self {
            Self::Idle => "Idle",
            Self::Starting => "Starting",
            Self::Running => "Running",
            Self::Stopping => "Stopping",
            Self::Fault => "Fault",
        }
    }
}

/// C `Resource`, as its bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Resource {
    RemoteMeasurement = 1 << 0,
    MonitorThread = 1 << 1,
    PreviewWorker = 1 << 2,
    IntegratedPreviewWorker = 1 << 3,
    ImageWorker = 1 << 4,
    HistogramWorker = 1 << 5,
}

/// C `Snapshot`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Snapshot {
    pub phase: Phase,
    pub resources: u32,
}

impl Snapshot {
    pub fn has(&self, resource: Resource) -> bool {
        self.resources & resource as u32 != 0
    }

    /// C `connectedStatusMessage`.
    pub fn connected_status_message(&self) -> &'static str {
        match self.phase {
            Phase::Idle => "OK",
            Phase::Starting => "Starting acquisition...",
            Phase::Running => "Acquisition running",
            Phase::Stopping => "Stopping acquisition...",
            Phase::Fault if self.resources != 0 => {
                "Acquisition fault; write Acquire=0 to retry cleanup"
            }
            Phase::Fault => "Acquisition fault; retry acquisition",
        }
    }
}

/// C `CleanupClaim`: only the owner tears resources down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CleanupClaim {
    pub owner: bool,
    pub fault: bool,
    pub snapshot: Snapshot,
}

#[derive(Debug, Default)]
struct State {
    phase: Phase,
    resources: u32,
    fault_pending: bool,
}

/// C `Coordinator`.
#[derive(Debug, Default)]
pub struct Coordinator {
    state: Mutex<State>,
    changed: Condvar,
}

impl Coordinator {
    pub fn begin_start(&self) -> bool {
        let mut s = self.state.lock();
        if !matches!(s.phase, Phase::Idle | Phase::Fault) || s.resources != 0 {
            return false;
        }
        s.phase = Phase::Starting;
        s.fault_pending = false;
        self.changed.notify_all();
        true
    }

    pub fn note_resource(&self, resource: Resource) -> bool {
        let mut s = self.state.lock();
        if s.phase != Phase::Starting {
            return false;
        }
        s.resources |= resource as u32;
        true
    }

    /// Running needs the remote measurement and the monitor.
    pub fn commit_start(&self) -> bool {
        let mut s = self.state.lock();
        let required = Resource::RemoteMeasurement as u32 | Resource::MonitorThread as u32;
        if s.phase != Phase::Starting || s.resources & required != required {
            return false;
        }
        s.phase = Phase::Running;
        self.changed.notify_all();
        true
    }

    pub fn begin_stop(&self, fault: bool) -> CleanupClaim {
        let mut s = self.state.lock();
        s.fault_pending |= fault;
        let mut claim = CleanupClaim {
            owner: false,
            fault: s.fault_pending,
            snapshot: Snapshot {
                phase: s.phase,
                resources: s.resources,
            },
        };
        if s.phase == Phase::Stopping {
            return claim;
        }
        if matches!(s.phase, Phase::Idle | Phase::Fault) && s.resources == 0 {
            if s.fault_pending {
                s.phase = Phase::Fault;
                self.changed.notify_all();
            }
            return claim;
        }
        s.phase = Phase::Stopping;
        claim.owner = true;
        claim.snapshot.phase = Phase::Stopping;
        self.changed.notify_all();
        claim
    }

    /// Every resource but the remote measurement is gone once the owner has
    /// torn down; the remote one only when Serval confirmed it stopped.
    pub fn finish_stop(&self, remote_stopped: bool, fault: bool) -> Phase {
        let mut s = self.state.lock();
        s.resources &= Resource::RemoteMeasurement as u32;
        if remote_stopped {
            s.resources = 0;
        }
        let final_fault = fault || s.fault_pending || s.resources != 0;
        s.phase = if final_fault {
            Phase::Fault
        } else {
            Phase::Idle
        };
        s.fault_pending = false;
        self.changed.notify_all();
        s.phase
    }

    pub fn snapshot(&self) -> Snapshot {
        let s = self.state.lock();
        Snapshot {
            phase: s.phase,
            resources: s.resources,
        }
    }

    /// C `monitoring` and `requestedAcquire`, which are the same test.
    pub fn monitoring(&self) -> bool {
        matches!(self.state.lock().phase, Phase::Starting | Phase::Running)
    }

    /// C `waitForRunningFor`: false on timeout or when the start settled
    /// anywhere but Running.
    pub fn wait_for_running_for(&self, timeout: Duration) -> bool {
        let mut s = self.state.lock();
        let timed_out = self
            .changed
            .wait_while_for(&mut s, |s| s.phase == Phase::Starting, timeout)
            .timed_out();
        !timed_out && s.phase == Phase::Running
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn running() -> Coordinator {
        let c = Coordinator::default();
        assert!(c.begin_start());
        assert!(c.note_resource(Resource::RemoteMeasurement));
        assert!(c.note_resource(Resource::MonitorThread));
        assert!(c.commit_start());
        c
    }

    #[test]
    fn a_start_begins_only_from_idle_or_fault_with_nothing_held() {
        let c = Coordinator::default();
        assert!(c.begin_start());
        assert!(!c.begin_start());
        let c = running();
        assert!(!c.begin_start());
        // A fault that kept the remote measurement blocks a new start.
        assert!(c.begin_stop(false).owner);
        assert_eq!(c.finish_stop(false, false), Phase::Fault);
        assert!(c.snapshot().has(Resource::RemoteMeasurement));
        assert!(!c.begin_start());
        assert!(c.begin_stop(false).owner);
        assert_eq!(c.finish_stop(true, false), Phase::Idle);
        assert!(c.begin_start());
    }

    #[test]
    fn a_commit_needs_the_measurement_and_the_monitor() {
        let c = Coordinator::default();
        assert!(!c.note_resource(Resource::RemoteMeasurement));
        assert!(c.begin_start());
        assert!(c.note_resource(Resource::RemoteMeasurement));
        assert!(!c.commit_start());
        assert!(c.note_resource(Resource::MonitorThread));
        assert!(c.commit_start());
        assert_eq!(c.snapshot().phase, Phase::Running);
    }

    #[test]
    fn only_the_first_stop_owns_the_cleanup() {
        let c = running();
        let first = c.begin_stop(false);
        assert!(first.owner && first.snapshot.phase == Phase::Stopping);
        let second = c.begin_stop(true);
        assert!(!second.owner && second.fault);
        // The pending fault from the second stop survives into the result.
        assert_eq!(c.finish_stop(true, false), Phase::Fault);
    }

    #[test]
    fn a_stop_with_nothing_held_owns_nothing() {
        let c = Coordinator::default();
        let claim = c.begin_stop(false);
        assert!(!claim.owner);
        assert_eq!(c.snapshot().phase, Phase::Idle);
        let claim = c.begin_stop(true);
        assert!(!claim.owner && claim.fault);
        assert_eq!(c.snapshot().phase, Phase::Fault);
    }

    #[test]
    fn finish_keeps_the_measurement_until_serval_confirms() {
        let c = Coordinator::default();
        assert!(c.begin_start());
        assert!(c.note_resource(Resource::RemoteMeasurement));
        assert!(c.note_resource(Resource::PreviewWorker));
        assert!(c.begin_stop(true).owner);
        assert_eq!(c.finish_stop(false, true), Phase::Fault);
        assert_eq!(c.snapshot().resources, Resource::RemoteMeasurement as u32);
    }

    #[test]
    fn the_status_message_follows_the_phase() {
        let mut s = Snapshot::default();
        assert_eq!(s.connected_status_message(), "OK");
        s.phase = Phase::Fault;
        assert_eq!(
            s.connected_status_message(),
            "Acquisition fault; retry acquisition"
        );
        s.resources = Resource::RemoteMeasurement as u32;
        assert_eq!(
            s.connected_status_message(),
            "Acquisition fault; write Acquire=0 to retry cleanup"
        );
    }

    #[test]
    fn a_waiter_sees_the_start_settle() {
        let c = std::sync::Arc::new(Coordinator::default());
        assert!(c.begin_start());
        assert!(!c.wait_for_running_for(Duration::from_millis(10)));
        let w = c.clone();
        let waiter = std::thread::spawn(move || w.wait_for_running_for(Duration::from_secs(5)));
        assert!(c.note_resource(Resource::RemoteMeasurement));
        assert!(c.note_resource(Resource::MonitorThread));
        assert!(c.commit_start());
        assert!(waiter.join().unwrap());
        let c = Coordinator::default();
        assert!(!c.wait_for_running_for(Duration::from_millis(10)));
    }
}
