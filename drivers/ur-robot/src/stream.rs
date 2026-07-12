//! The RTDE output stream: one reader thread, one shared snapshot.
//!
//! Both `RTDEReceiveInterface` and `RTDEControlInterface` run a `receiveCallback`
//! thread that drains data packages into a shared `RobotState`. That machinery is
//! this module.
//!
//! **Invariant:** once a [`StateStream`] is spawned, the reader thread is the
//! only owner of the [`Session`]'s read half. Commands go out through a
//! [`SessionWriter`] on the same socket (a socket is full-duplex), never through
//! the session the reader owns.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use parking_lot::Mutex;

use crate::error::{UrError, UrResult};
use crate::session::Session;
use crate::state::Value;

/// The newest robot state received from the controller.
#[derive(Debug, Clone, Default)]
pub struct Snapshot {
    values: HashMap<String, Value>,
}

impl Snapshot {
    pub fn new(values: HashMap<String, Value>) -> Self {
        Self { values }
    }

    pub fn double(&self, name: &str) -> Option<f64> {
        self.values.get(name).and_then(Value::as_f64)
    }

    pub fn int(&self, name: &str) -> Option<i32> {
        self.values.get(name).and_then(Value::as_i32)
    }

    pub fn uint(&self, name: &str) -> Option<u32> {
        self.values.get(name).and_then(Value::as_u32)
    }

    pub fn doubles(&self, name: &str) -> Option<&[f64]> {
        self.values.get(name).and_then(Value::as_f64s)
    }

    pub fn ints(&self, name: &str) -> Option<&[i32]> {
        self.values.get(name).and_then(Value::as_i32s)
    }

    /// Output int register `n`. The register block is absent on controllers too
    /// old for it, hence the `Option`.
    pub fn output_int_register(&self, n: i32) -> Option<i32> {
        self.int(&format!("output_int_register_{n}"))
    }

    pub fn output_double_register(&self, n: i32) -> Option<f64> {
        self.double(&format!("output_double_register_{n}"))
    }

    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }
}

#[derive(Default)]
struct Shared {
    snapshot: Snapshot,
    connected: bool,
}

/// A running RTDE output stream.
pub struct StateStream {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    reader: Option<JoinHandle<()>>,
}

impl StateStream {
    /// Take ownership of a session on which synchronisation has already been
    /// started, and drain it on a background thread.
    pub fn spawn(mut session: Session) -> Self {
        let shared = Arc::new(Mutex::new(Shared {
            snapshot: Snapshot::default(),
            connected: true,
        }));
        let stop = Arc::new(AtomicBool::new(false));

        let thread_shared = Arc::clone(&shared);
        let thread_stop = Arc::clone(&stop);
        let reader = std::thread::spawn(move || {
            while !thread_stop.load(Ordering::SeqCst) {
                match session.receive_data() {
                    Ok(Some(values)) => {
                        let mut s = thread_shared.lock();
                        s.snapshot = Snapshot::new(values);
                        s.connected = true;
                    }
                    // Timed out with nothing to read: quiet, not gone.
                    Ok(None) => {}
                    Err(e) => {
                        log::error!("ur-robot: RTDE stream lost: {e}");
                        thread_shared.lock().connected = false;
                        break;
                    }
                }
            }
            session.disconnect(true);
        });

        Self {
            shared,
            stop,
            reader: Some(reader),
        }
    }

    /// Block until the first robot state arrives (the C++ constructors do this).
    pub fn wait_first_state(&self, timeout: Duration) -> UrResult<()> {
        let deadline = Instant::now() + timeout;
        loop {
            {
                let s = self.shared.lock();
                if !s.connected {
                    return Err(UrError::Io(
                        "RTDE stream died before the first state".into(),
                    ));
                }
                if !s.snapshot.is_empty() {
                    return Ok(());
                }
            }
            if Instant::now() >= deadline {
                return Err(UrError::Timeout(timeout, "first RTDE robot state".into()));
            }
            std::thread::sleep(Duration::from_micros(100));
        }
    }

    pub fn is_connected(&self) -> bool {
        self.shared.lock().connected
    }

    /// The newest robot state. Empty until the first package arrives.
    pub fn snapshot(&self) -> Snapshot {
        self.shared.lock().snapshot.clone()
    }

    /// Stop the reader thread and close the socket.
    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(h) = self.reader.take() {
            let _ = h.join();
        }
        self.shared.lock().connected = false;
    }
}

impl Drop for StateStream {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshot_reads_by_name_and_type() {
        let mut values = HashMap::new();
        values.insert("timestamp".to_string(), Value::Double(12.5));
        values.insert("robot_mode".to_string(), Value::Int32(7));
        values.insert("safety_status_bits".to_string(), Value::Uint32(0x7ff));
        values.insert(
            "actual_q".to_string(),
            Value::Doubles(vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0]),
        );
        values.insert(
            "joint_mode".to_string(),
            Value::Int32s(vec![253, 253, 253, 253, 253, 253]),
        );
        values.insert("output_int_register_12".to_string(), Value::Int32(2));
        let snap = Snapshot::new(values);

        assert_eq!(snap.double("timestamp"), Some(12.5));
        assert_eq!(snap.int("robot_mode"), Some(7));
        assert_eq!(snap.uint("safety_status_bits"), Some(0x7ff));
        assert_eq!(snap.doubles("actual_q").unwrap().len(), 6);
        assert_eq!(snap.ints("joint_mode").unwrap()[0], 253);
        assert_eq!(snap.output_int_register(12), Some(2));
        assert_eq!(snap.output_int_register(13), None);
        assert_eq!(snap.double("no_such_variable"), None);
        assert!(!snap.is_empty());
        assert!(Snapshot::default().is_empty());
    }
}
