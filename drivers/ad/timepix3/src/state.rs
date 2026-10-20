//! The state the port driver and the background threads share.
//!
//! In C this is a pile of `ADTimePix` members guarded by three `epicsMutex`es
//! — and the stream workers reach straight into the asyn parameter library
//! without holding the port lock (UPSTREAM DEFECT: `processPrvImgDataLine`,
//! `processImgDataLine`, serval_stream.cpp:92-230, :729-870;
//! `processPrvHstDataLine`, histogram_io.cpp:162-400). Here the workers read the few
//! parameters they need from [`ParamMirror`] and hand their writes to
//! [`PortJobs`], so neither waits for the port: a stop that holds the port
//! through the workers' drain, as C's does, still sees them drain.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::time::Duration;

use epics_rs::asyn::port::PortDriverBase;
use parking_lot::{Condvar, Mutex};

use crate::accum::Accumulator;
use crate::coordinator::Coordinator;
use crate::family::DetectorFamily;
use crate::rate::Rates;
use crate::worker::WorkerState;

/// What the port driver asks the background threads to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    /// Wake the measurement monitor (C creates `timePixCallback`).
    AcquisitionStarted,
    /// C `pushProcessedHstToPlugins` on a `TPX3_HST_WRITE_PROCESSED` write of 1.
    PushProcessedHst,
    /// C `pushProcessedImgToPlugins` on a `TPX3_IMG_WRITE_PROCESSED` write of 1.
    PushProcessedImg,
}

/// Where the four streamed channels stream from, as Serval's `Base` PVs give
/// it (`tcp://[listen@]host:port`, or a `file://` path when the channel writes
/// to disk instead).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct StreamPaths {
    pub prv_img: Option<String>,
    pub prv_img1: Option<String>,
    pub img: Option<String>,
    pub prv_hst: Option<String>,
}

/// C `prvImgHost_`/`prvImgPort_` .. `prvHstHost_`/`prvHstPort_`: the last
/// `tcp://` target that parsed, which a worker connects to. A Base that does
/// not parse leaves the previous one (ADTimePix.cpp:298-387, acquire.cpp:498-501).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct StreamEndpoints {
    pub prv_img: Option<(String, u16)>,
    pub prv_img1: Option<(String, u16)>,
    pub img: Option<(String, u16)>,
    pub prv_hst: Option<(String, u16)>,
}

/// C `PreviewJsonimageStream`'s state (serval_stream.cpp:59-85); the frames
/// its band-pass reads are the pool owner's `pArrays` slots. Index 0 is
/// PrvImg, 1 PrvImg1.
pub struct PreviewState {
    /// C `jsonHeadersRemaining`.
    pub json_headers: i32,
    /// C `t1ReadyForDiff`.
    pub t1_ready: bool,
    /// C `t0OrphanForDiff`.
    pub t0_orphan: bool,
    /// C `lastSeenFrameForPair`, -1 for none.
    pub last_seen_frame: i32,
    /// C `lastDiffT0Frame`, -1 for none.
    pub last_diff_t0: i32,
}

impl Default for PreviewState {
    fn default() -> Self {
        Self {
            json_headers: 0,
            t1_ready: false,
            t0_orphan: false,
            last_seen_frame: -1,
            last_diff_t0: -1,
        }
    }
}

impl PreviewState {
    /// The T0/T1 pairing reset of an acquisition start (acquire.cpp:411-427)
    /// and `resetAcquisitionRuntimeState` (acquisition_runtime.cpp:108-134).
    pub fn reset_pairing(&mut self) {
        self.t1_ready = false;
        self.t0_orphan = false;
        self.last_seen_frame = -1;
        self.last_diff_t0 = -1;
    }
}

/// A parameter the stream workers read with C's unlocked `getIntegerParam`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mirrored {
    ArrayCallbacks,
    ArrayCounter,
    MaxSizeX,
    MaxSizeY,
    PixCount,
    ImgAccumulationEnable,
    PrvHstAccumulationEnable,
}

impl Mirrored {
    pub const ALL: [Self; 7] = [
        Self::ArrayCallbacks,
        Self::ArrayCounter,
        Self::MaxSizeX,
        Self::MaxSizeY,
        Self::PixCount,
        Self::ImgAccumulationEnable,
        Self::PrvHstAccumulationEnable,
    ];
}

/// The address-0 values of the [`Mirrored`] parameters, stored by an Int32
/// store rule on each, so every store of the parameter lands here too.
#[derive(Debug, Default)]
pub struct ParamMirror([AtomicI32; Mirrored::ALL.len()]);

impl ParamMirror {
    pub fn get(&self, which: Mirrored) -> i32 {
        self.0[which as usize].load(Ordering::Acquire)
    }

    pub fn store(&self, which: Mirrored, value: i32) {
        self.0[which as usize].store(value, Ordering::Release);
    }
}

type PortJob = Box<dyn FnOnce(&mut PortDriverBase) + Send>;

/// What the stream workers do to the parameter library, in their order. Runs
/// on the port's actor, from whichever comes first: the flush each push
/// submits, or a stop that holds the port while it waits for the workers.
#[derive(Default)]
pub struct PortJobs {
    queue: Mutex<VecDeque<PortJob>>,
    pushed: Condvar,
}

impl PortJobs {
    pub fn push(&self, job: impl FnOnce(&mut PortDriverBase) + Send + 'static) {
        self.queue.lock().push_back(Box::new(job));
        self.pushed.notify_all();
    }

    /// Run every queued job, oldest first. Only the actor calls it, so jobs
    /// run one at a time.
    pub fn run(&self, base: &mut PortDriverBase) {
        while let Some(job) = self.queue.lock().pop_front() {
            job(base);
        }
    }

    /// Run the jobs pushed until `timeout`, or until a push after the first
    /// wait: the stop's wait loop.
    pub fn run_for(&self, base: &mut PortDriverBase, timeout: Duration) {
        {
            let mut q = self.queue.lock();
            if q.is_empty() {
                self.pushed.wait_for(&mut q, timeout);
            }
        }
        self.run(base);
    }
}

/// How often a stop waiting on the workers looks at their state; a job
/// pushed meanwhile wakes it at once.
const WORKER_STATE_POLL: Duration = Duration::from_millis(5);

pub struct Shared {
    /// The Img channel's accumulation (asyn addresses 1-3).
    pub img: Mutex<Accumulator>,
    /// The PrvHst channel's accumulation (asyn addresses 4-7).
    pub hst: Mutex<Accumulator>,
    /// The three streams' acquisition rates.
    pub rates: Mutex<Rates>,
    pub streams: Mutex<StreamPaths>,
    pub endpoints: Mutex<StreamEndpoints>,
    /// C `prvImgWorkerState_`, `prvImg1WorkerState_`, `imgWorkerState_`,
    /// `prvHstWorkerState_`.
    pub prv_img_worker: WorkerState,
    pub prv_img1_worker: WorkerState,
    pub img_worker: WorkerState,
    pub prv_hst_worker: WorkerState,
    /// The PrvImg and PrvImg1 preview streams' state.
    pub preview: [Mutex<PreviewState>; 2],
    orientation: AtomicI32,
    /// C `acquisitionCoordinator_`: the one owner of the acquisition phase.
    pub acquisition: Coordinator,
    /// C `detectorFamily_`, as its `DetectorFamily` discriminant.
    family: AtomicI32,
    /// C `detectorFamilyApplied_`.
    family_applied: AtomicBool,
    pub mirror: ParamMirror,
    pub port_jobs: PortJobs,
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
            hst: Mutex::new(Accumulator::prv_hst()),
            rates: Mutex::new(Rates::default()),
            streams: Mutex::new(StreamPaths::default()),
            endpoints: Mutex::new(StreamEndpoints::default()),
            prv_img_worker: WorkerState::default(),
            prv_img1_worker: WorkerState::default(),
            img_worker: WorkerState::default(),
            prv_hst_worker: WorkerState::default(),
            preview: Default::default(),
            orientation: AtomicI32::new(0),
            acquisition: Coordinator::default(),
            family: AtomicI32::new(DetectorFamily::Unknown as i32),
            family_applied: AtomicBool::new(false),
            mirror: ParamMirror::default(),
            port_jobs: PortJobs::default(),
        }
    }

    pub fn set_stream_paths(&self, paths: StreamPaths) {
        *self.streams.lock() = paths;
    }

    pub fn stream_paths(&self) -> StreamPaths {
        self.streams.lock().clone()
    }

    pub fn stream_workers(&self) -> [&WorkerState; 4] {
        [
            &self.prv_img_worker,
            &self.prv_img1_worker,
            &self.img_worker,
            &self.prv_hst_worker,
        ]
    }

    /// C `waitForStreamWorkersToStop` (acquisition_runtime.cpp:84-104): wait
    /// up to `timeout` for every worker to end its run, running the workers'
    /// jobs meanwhile. The caller holds the port.
    pub fn wait_for_stream_workers_to_stop(&self, base: &mut PortDriverBase, timeout: Duration) {
        let deadline = std::time::Instant::now() + timeout;
        while self.stream_workers().iter().any(|w| w.any_running()) {
            let now = std::time::Instant::now();
            if now >= deadline {
                break;
            }
            self.port_jobs
                .run_for(base, (deadline - now).min(WORKER_STATE_POLL));
        }
        self.port_jobs.run(base);
    }

    /// C `stopAndJoinStreamWorkers` (acquire.cpp:159-199): stop every worker
    /// and wait, as C's join does without a bound, until each has left its
    /// run, running the workers' jobs meanwhile. The caller holds the port.
    pub fn stop_and_join_stream_workers(&self, base: &mut PortDriverBase) {
        for worker in self.stream_workers() {
            worker.request_stop();
        }
        while !self.stream_workers().iter().all(|w| w.parked()) {
            self.port_jobs.run_for(base, WORKER_STATE_POLL);
        }
        self.port_jobs.run(base);
    }

    pub fn set_orientation(&self, orientation: i32) {
        self.orientation.store(orientation, Ordering::Release);
    }

    pub fn orientation(&self) -> i32 {
        self.orientation.load(Ordering::Acquire)
    }

    pub fn family(&self) -> DetectorFamily {
        DetectorFamily::from_i32(self.family.load(Ordering::Acquire))
    }

    /// Record `family`; true when C's `familyChanged || !detectorFamilyApplied_`
    /// asks for the family defaults (ADTimePix.cpp:2110-2122).
    pub fn note_family(&self, family: DetectorFamily) -> bool {
        let old = self.family.swap(family as i32, Ordering::AcqRel);
        let applied = self.family_applied.swap(true, Ordering::AcqRel);
        old != family as i32 || !applied
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_stream_paths_round_trip() {
        let s = Shared::new();
        s.set_stream_paths(StreamPaths {
            prv_img: Some("tcp://h:1".into()),
            prv_hst: Some("tcp://h:2".into()),
            ..Default::default()
        });
        let p = s.stream_paths();
        assert_eq!(p.prv_img.as_deref(), Some("tcp://h:1"));
        assert_eq!(p.img, None);
        assert_eq!(p.prv_hst.as_deref(), Some("tcp://h:2"));
    }
}
