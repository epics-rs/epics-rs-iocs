//! Cross-driver lookup by port name.
//!
//! urRobot's drivers reach into each other: `RTDEControl` and `URGripper` call
//! `findDerivedAsynPortDriver<URDashboard>(name)` and then read that driver's
//! asyn parameters (its IP, robot mode, PolyScope version), and `RTDEControl`
//! also reads `RTDEReceive`'s SAFETY_STATUS_BITS and OUTPUT_INTEGER_REG12.
//!
//! asyn-rs has no downcast from a port name to a concrete driver, so each
//! driver publishes exactly the state its dependents read into this name-keyed
//! registry. That makes the dependency explicit — a dependent reads a typed
//! struct, not another driver's parameter table by index.

use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

use parking_lot::Mutex;

use crate::dashboard::PolyScopeVersion;

/// What the dashboard driver publishes for its dependents.
#[derive(Debug, Clone, Default)]
pub struct DashboardState {
    pub connected: bool,
    /// Verbatim reply to `robotmode`, e.g. `"Robotmode: RUNNING"`.
    pub robot_mode: String,
    pub polyscope: PolyScopeVersion,
}

impl DashboardState {
    /// `robot_ready()` in gripper_driver.cpp:9 — powered on and reachable.
    pub fn robot_on(&self) -> bool {
        self.connected
            && (self.robot_mode == "Robotmode: IDLE" || self.robot_mode == "Robotmode: RUNNING")
    }

    /// The control driver only connects when the robot is actually running.
    pub fn robot_running(&self) -> bool {
        self.connected && self.robot_mode == "Robotmode: RUNNING"
    }
}

/// A dashboard driver's shared state, plus the robot IP it was configured with.
#[derive(Clone)]
pub struct DashboardHandle {
    pub ip: String,
    state: Arc<Mutex<DashboardState>>,
}

impl DashboardHandle {
    pub fn new(ip: &str) -> Self {
        Self {
            ip: ip.to_string(),
            state: Arc::new(Mutex::new(DashboardState::default())),
        }
    }

    pub fn get(&self) -> DashboardState {
        self.state.lock().clone()
    }

    pub fn set(&self, state: DashboardState) {
        *self.state.lock() = state;
    }
}

/// What the receive driver publishes for the control driver.
#[derive(Debug, Clone, Default)]
pub struct ReceiveState {
    pub connected: bool,
    /// `safety_status_bits & 0x7ff`. 1 (NORMAL) is the only value the control
    /// driver will act in.
    pub safety_status_bits: i32,
    /// `output_int_register_12`, bumped by a finished custom URScript.
    pub output_int_register_12: i32,
}

#[derive(Clone, Default)]
pub struct ReceiveHandle {
    state: Arc<Mutex<ReceiveState>>,
}

impl ReceiveHandle {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn get(&self) -> ReceiveState {
        self.state.lock().clone()
    }

    pub fn set(&self, state: ReceiveState) {
        *self.state.lock() = state;
    }
}

type Registry<T> = OnceLock<Mutex<HashMap<String, T>>>;

static DASHBOARDS: Registry<DashboardHandle> = OnceLock::new();
static RECEIVERS: Registry<ReceiveHandle> = OnceLock::new();

fn map<T>(reg: &'static Registry<T>) -> &'static Mutex<HashMap<String, T>> {
    reg.get_or_init(|| Mutex::new(HashMap::new()))
}

pub fn register_dashboard(port: &str, handle: DashboardHandle) {
    map(&DASHBOARDS).lock().insert(port.to_string(), handle);
}

pub fn dashboard(port: &str) -> Option<DashboardHandle> {
    map(&DASHBOARDS).lock().get(port).cloned()
}

pub fn register_receive(port: &str, handle: ReceiveHandle) {
    map(&RECEIVERS).lock().insert(port.to_string(), handle);
}

pub fn receive(port: &str) -> Option<ReceiveHandle> {
    map(&RECEIVERS).lock().get(port).cloned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_dashboard_is_found_by_its_port_name() {
        let h = DashboardHandle::new("192.168.1.10");
        h.set(DashboardState {
            connected: true,
            robot_mode: "Robotmode: RUNNING".into(),
            polyscope: PolyScopeVersion {
                major: 5,
                minor: 11,
                patch: 3,
                build: 108355,
            },
        });
        register_dashboard("UR_DASH_TEST", h);

        let found = dashboard("UR_DASH_TEST").expect("registered dashboard");
        assert_eq!(found.ip, "192.168.1.10");
        assert!(found.get().robot_on());
        assert!(found.get().robot_running());
        assert_eq!(found.get().polyscope.major, 5);
        assert!(dashboard("NO_SUCH_PORT").is_none());
    }

    #[test]
    fn robot_on_requires_a_connected_dashboard_and_a_powered_mode() {
        let mut s = DashboardState {
            connected: true,
            robot_mode: "Robotmode: IDLE".into(),
            polyscope: PolyScopeVersion::default(),
        };
        assert!(s.robot_on());
        assert!(!s.robot_running());

        s.robot_mode = "Robotmode: POWER_OFF".into();
        assert!(!s.robot_on());

        s.robot_mode = "Robotmode: RUNNING".into();
        assert!(s.robot_on());
        s.connected = false;
        assert!(!s.robot_on());
    }

    #[test]
    fn a_receiver_publishes_safety_bits_and_the_script_counter() {
        let h = ReceiveHandle::new();
        h.set(ReceiveState {
            connected: true,
            safety_status_bits: 1,
            output_int_register_12: 7,
        });
        register_receive("UR_RECV_TEST", h);

        let found = receive("UR_RECV_TEST").expect("registered receiver");
        let s = found.get();
        assert_eq!(s.safety_status_bits, 1);
        assert_eq!(s.output_int_register_12, 7);
        assert!(receive("NO_SUCH_PORT").is_none());
    }
}
