//! One stream worker's lifecycle (C `stream_worker_state.cpp`).
//!
//! C creates a worker thread per acquisition and joins the old one before the
//! next start. The worker here outlives acquisitions and parks between them,
//! so every transition it makes names the [`Generation`] `start` handed out:
//! a worker still finishing an old run cannot fail or connect the next one.

use std::net::{Shutdown, TcpStream};
use std::time::Duration;

use parking_lot::{Condvar, Mutex};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    #[default]
    Stopped,
    Running,
    Connected,
}

#[derive(Debug, Default)]
struct Inner {
    phase: Phase,
    generation: u64,
    /// The connected socket, so a stop can interrupt a blocked receive (C
    /// `stopAndJoinStreamWorkers` calls `client->interrupt()`).
    socket: Option<TcpStream>,
    /// The worker is serving a run rather than parked in `wait_for_start`:
    /// what C's `epicsThreadMustJoin` waits out.
    serving: bool,
}

/// The run of a worker one `start` admitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Generation(u64);

#[derive(Debug, Default)]
pub struct WorkerState {
    inner: Mutex<Inner>,
    changed: Condvar,
}

impl WorkerState {
    /// C `start`: Stopped → Running.
    pub fn start(&self) -> bool {
        let mut s = self.inner.lock();
        if s.phase != Phase::Stopped {
            return false;
        }
        s.phase = Phase::Running;
        s.generation += 1;
        self.changed.notify_all();
        true
    }

    /// C `requestStop`, plus the receive interrupt of
    /// `stopAndJoinStreamWorkers` (acquire.cpp:159-181).
    pub fn request_stop(&self) {
        let mut s = self.inner.lock();
        s.phase = Phase::Stopped;
        if let Some(socket) = s.socket.take() {
            let _ = socket.shutdown(Shutdown::Both);
        }
        self.changed.notify_all();
    }

    /// Park the worker until a start newer than `served`.
    pub fn wait_for_start(&self, served: Option<Generation>) -> Generation {
        let mut s = self.inner.lock();
        s.serving = false;
        self.changed.notify_all();
        let served = served.map_or(0, |g| g.0);
        self.changed.wait_while(&mut s, |s| {
            s.phase == Phase::Stopped || s.generation == served
        });
        s.serving = true;
        Generation(s.generation)
    }

    /// C `fail`: this run is over.
    pub fn fail(&self, g: Generation) {
        let mut s = self.inner.lock();
        if s.generation == g.0 {
            s.phase = Phase::Stopped;
            s.socket = None;
        }
        self.changed.notify_all();
    }

    /// C `markConnected`: Running → Connected. False when the run was
    /// stopped meanwhile, and the socket is then the caller's to drop.
    pub fn mark_connected(&self, g: Generation, socket: &TcpStream) -> bool {
        let mut s = self.inner.lock();
        if s.generation != g.0 || s.phase != Phase::Running {
            return false;
        }
        s.phase = Phase::Connected;
        s.socket = socket.try_clone().ok();
        self.changed.notify_all();
        true
    }

    /// C `markDisconnected`: Connected → Running.
    pub fn mark_disconnected(&self, g: Generation) {
        let mut s = self.inner.lock();
        if s.generation == g.0 && s.phase == Phase::Connected {
            s.phase = Phase::Running;
        }
        if s.generation == g.0 {
            s.socket = None;
        }
        self.changed.notify_all();
    }

    /// C `running`, for the worker's own run.
    pub fn running(&self, g: Generation) -> bool {
        let s = self.inner.lock();
        s.generation == g.0 && s.phase != Phase::Stopped
    }

    /// C `running`, for whichever run the worker is on.
    pub fn any_running(&self) -> bool {
        self.inner.lock().phase != Phase::Stopped
    }

    /// The worker has left its last run and is parked: C's join has returned.
    pub fn parked(&self) -> bool {
        !self.inner.lock().serving
    }

    /// C `connected`.
    pub fn connected(&self) -> bool {
        self.inner.lock().phase == Phase::Connected
    }

    /// C `waitForStopFor`: the worker's interruptible sleep; true once the
    /// run is over.
    pub fn wait_for_stop_for(&self, g: Generation, timeout: Duration) -> bool {
        let mut s = self.inner.lock();
        self.changed.wait_while_for(
            &mut s,
            |s| s.generation == g.0 && s.phase != Phase::Stopped,
            timeout,
        );
        !(s.generation == g.0 && s.phase != Phase::Stopped)
    }

    /// C `waitForConnectedFor`: the start's readiness wait.
    pub fn wait_for_connected_for(&self, timeout: Duration) -> bool {
        let mut s = self.inner.lock();
        self.changed
            .wait_while_for(&mut s, |s| s.phase == Phase::Running, timeout);
        s.phase == Phase::Connected
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;
    use std::sync::Arc;

    fn pair() -> (TcpStream, TcpStream) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        (client, listener.accept().unwrap().0)
    }

    #[test]
    fn a_start_admits_one_run_until_it_stops() {
        let w = WorkerState::default();
        assert!(w.start());
        assert!(!w.start());
        let g = w.wait_for_start(None);
        assert!(w.running(g) && !w.connected());
        w.request_stop();
        assert!(!w.running(g));
        assert!(w.start());
    }

    #[test]
    fn readiness_is_the_connected_phase_or_a_stop() {
        let w = Arc::new(WorkerState::default());
        assert!(w.start());
        assert!(!w.wait_for_connected_for(Duration::from_millis(20)));

        let (client, _server) = pair();
        let worker = w.clone();
        let t = std::thread::spawn(move || {
            let g = worker.wait_for_start(None);
            assert!(worker.mark_connected(g, &client));
        });
        assert!(w.wait_for_connected_for(Duration::from_secs(5)));
        t.join().unwrap();

        // A failed run ends the wait at once rather than at the deadline.
        let g = w.wait_for_start(None);
        w.mark_disconnected(g);
        w.fail(g);
        let t0 = std::time::Instant::now();
        assert!(!w.wait_for_connected_for(Duration::from_secs(5)));
        assert!(t0.elapsed() < Duration::from_secs(1));
    }

    #[test]
    fn a_worker_is_parked_until_it_takes_a_run_and_again_after_it() {
        let w = Arc::new(WorkerState::default());
        assert!(w.parked());
        assert!(w.start());
        let worker = w.clone();
        let t = std::thread::spawn(move || {
            let g = worker.wait_for_start(None);
            worker.wait_for_stop_for(g, Duration::from_secs(5));
            worker.wait_for_start(Some(g));
        });
        while w.parked() {
            std::thread::yield_now();
        }
        w.request_stop();
        while !w.parked() {
            std::thread::yield_now();
        }
        assert!(w.start());
        t.join().unwrap();
    }

    #[test]
    fn a_stale_run_cannot_touch_the_next_one() {
        let w = WorkerState::default();
        assert!(w.start());
        let old = w.wait_for_start(None);
        w.request_stop();
        assert!(w.start());
        let new = w.wait_for_start(Some(old));
        assert_ne!(old, new);

        let (client, _server) = pair();
        assert!(!w.mark_connected(old, &client));
        w.fail(old);
        assert!(w.running(new) && !w.running(old));
        assert!(w.mark_connected(new, &client));
        w.mark_disconnected(old);
        assert!(w.connected());
    }

    #[test]
    fn a_stop_interrupts_a_blocked_receive_and_the_sleep() {
        use std::io::Read;
        let w = Arc::new(WorkerState::default());
        assert!(w.start());
        let g = w.wait_for_start(None);
        let (mut client, _server) = pair();
        assert!(w.mark_connected(g, &client));
        // Winsock's shutdown does not wake a recv blocked in another thread;
        // there the worker's read timeout (C `kReceivePollTimeoutMs`) ends it.
        #[cfg(windows)]
        client
            .set_read_timeout(Some(Duration::from_millis(250)))
            .unwrap();

        let stopper = w.clone();
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            stopper.request_stop();
        });
        let t0 = std::time::Instant::now();
        assert_eq!(client.read(&mut [0u8; 8]).unwrap_or(0), 0);
        assert!(w.wait_for_stop_for(g, Duration::from_secs(5)));
        assert!(t0.elapsed() < Duration::from_secs(1));
        t.join().unwrap();
    }
}
