//! RTDE receive interface — the robot-state output stream.
//!
//! Ported from `ur_rtde/src/rtde_receive_interface.cpp`. The controller pushes a
//! data package every RTDE cycle (125 Hz on CB3, 500 Hz on e-Series); the
//! [`StateStream`] reader thread drains them and republishes the newest state.
//! Callers read snapshots, never the socket.

use std::time::Duration;

use crate::error::UrResult;
use crate::rtde::{ControllerVersion, default_frequency};
use crate::session::{DEFAULT_TIMEOUT, Session};
use crate::stream::{Snapshot, StateStream};

/// Wait for the first robot state, as the C++ constructor does.
const FIRST_STATE_TIMEOUT: Duration = Duration::from_secs(5);

/// The variables `setupRecipes()` asks for on every controller
/// (rtde_receive_interface.cpp:128).
const BASE_VARIABLES: &[&str] = &[
    "timestamp",
    "target_q",
    "target_qd",
    "target_qdd",
    "target_current",
    "target_moment",
    "actual_q",
    "actual_qd",
    "actual_current",
    "joint_control_output",
    "actual_TCP_pose",
    "actual_TCP_speed",
    "actual_TCP_force",
    "target_TCP_pose",
    "target_TCP_speed",
    "actual_digital_input_bits",
    "joint_temperatures",
    "actual_execution_time",
    "robot_mode",
    "joint_mode",
    "safety_mode",
    "actual_tool_accelerometer",
    "speed_scaling",
    "target_speed_fraction",
    "actual_momentum",
    "actual_main_voltage",
    "actual_robot_voltage",
    "actual_robot_current",
    "actual_joint_voltage",
    "actual_digital_output_bits",
    "runtime_state",
    "standard_analog_input0",
    "standard_analog_input1",
    "standard_analog_output0",
    "standard_analog_output1",
    "robot_status_bits",
    "safety_status_bits",
];

/// True when the controller is at least `major.minor`.
fn at_least(v: ControllerVersion, major: u32, minor: u32) -> bool {
    (v.major, v.minor) >= (major, minor)
}

/// Build the output variable list for a controller version.
///
/// The output-register block is gated on "PolyScope > 3.4" (lower range) or
/// "> 3.9 / > 5.3" (upper range). Upstream writes the lower-range gate as
/// `major_version >= 3 && minor_version >= 4` (rtde_receive_interface.cpp:203),
/// which is **false on PolyScope 5.0-5.3** because it compares the minor number
/// across major generations — so `output_int_register_12`, which urRobot's
/// OUTPUT_INTEGER_REG12 PV reads to detect custom-script completion, is silently
/// dropped there. The gate here is a version comparison, so 5.0 counts as newer
/// than 3.4.
pub fn output_variables(
    version: ControllerVersion,
    use_upper_range_registers: bool,
) -> Vec<String> {
    let mut vars: Vec<String> = BASE_VARIABLES.iter().map(|s| s.to_string()).collect();

    if version.major == 5 && version.minor >= 23 {
        vars.push("actual_current_as_torque".into());
    }
    if version.major == 5 && version.minor >= 9 {
        vars.push("ft_raw_wrench".into());
    }
    if (version.major == 3 && version.minor >= 11)
        || (version.major == 5 && version.minor >= 5 && version.bugfix >= 1)
        || (version.major == 5 && version.minor >= 6)
    {
        vars.push("payload".into());
        vars.push("payload_cog".into());
    }
    if (version.major == 3 && version.minor >= 15) || (version.major == 5 && version.minor >= 11) {
        vars.push("payload_inertia".into());
    }

    let (offset, supported) = if use_upper_range_registers {
        // "> 3.9 or > 5.3", i.e. per controller generation.
        let ok = match version.major {
            3 => version.minor >= 9,
            m if m >= 5 => version.minor >= 3,
            _ => false,
        };
        (24, ok)
    } else {
        // "> 3.4" — a plain version comparison, so every 5.x qualifies.
        (0, at_least(version, 3, 4))
    };

    if supported {
        vars.push(format!("output_int_register_{}", offset + 2));
        for i in 12..=19 {
            vars.push(format!("output_int_register_{}", offset + i));
        }
        for i in 12..=19 {
            vars.push(format!("output_double_register_{}", offset + i));
        }
    } else {
        log::warn!(
            "ur-robot: PolyScope {}.{} is too old for the {} range of output registers; \
             they will not be published",
            version.major,
            version.minor,
            if use_upper_range_registers {
                "upper"
            } else {
                "lower"
            }
        );
    }

    vars
}

/// RTDE receive interface.
pub struct ReceiveInterface {
    hostname: String,
    use_upper_range_registers: bool,
    stream: Option<StateStream>,
    version: ControllerVersion,
}

impl ReceiveInterface {
    /// Connect, register the output recipe, start synchronisation and spawn the
    /// reader thread. Returns once the first robot state has arrived.
    pub fn connect(hostname: &str, use_upper_range_registers: bool) -> UrResult<Self> {
        let mut me = Self {
            hostname: hostname.to_string(),
            use_upper_range_registers,
            stream: None,
            version: ControllerVersion::default(),
        };
        me.start()?;
        Ok(me)
    }

    fn start(&mut self) -> UrResult<()> {
        let mut session = Session::new(&self.hostname, DEFAULT_TIMEOUT);
        session.connect()?;
        session.negotiate_protocol_version()?;
        let version = session.controller_version()?;
        self.version = version;

        let variables = output_variables(version, self.use_upper_range_registers);
        session.send_output_setup(&variables, default_frequency(version))?;
        session.send_start()?;

        let stream = StateStream::spawn(session);
        stream.wait_first_state(FIRST_STATE_TIMEOUT)?;
        self.stream = Some(stream);
        Ok(())
    }

    /// `reconnect()`.
    pub fn reconnect(&mut self) -> UrResult<()> {
        self.disconnect();
        self.start()
    }

    /// `disconnect()` — stop the reader thread and drop the socket.
    pub fn disconnect(&mut self) {
        self.stream = None;
    }

    /// `isConnected()` — false once the reader thread has seen the stream die.
    pub fn is_connected(&self) -> bool {
        self.stream.as_ref().is_some_and(StateStream::is_connected)
    }

    pub fn controller_version(&self) -> ControllerVersion {
        self.version
    }

    /// The newest robot state, or an empty snapshot when disconnected.
    pub fn snapshot(&self) -> Snapshot {
        self.stream
            .as_ref()
            .map(StateStream::snapshot)
            .unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(major: u32, minor: u32, bugfix: u32) -> ControllerVersion {
        ControllerVersion {
            major,
            minor,
            bugfix,
            build: 0,
        }
    }

    #[test]
    fn base_variables_are_always_requested() {
        let vars = output_variables(v(3, 3, 0), false);
        assert_eq!(&vars[..BASE_VARIABLES.len()], BASE_VARIABLES);
        // PolyScope 3.3 is older than 3.4: no register block.
        assert!(!vars.iter().any(|s| s.starts_with("output_int_register_")));
    }

    #[test]
    fn lower_register_block_is_present_on_polyscope_5_0() {
        // The upstream gate (major >= 3 && minor >= 4) is false here, dropping
        // output_int_register_12.
        let vars = output_variables(v(5, 0, 0), false);
        assert!(vars.contains(&"output_int_register_12".to_string()));
        assert!(vars.contains(&"output_int_register_2".to_string()));
        assert!(vars.contains(&"output_int_register_19".to_string()));
        assert!(vars.contains(&"output_double_register_12".to_string()));
        assert!(vars.contains(&"output_double_register_19".to_string()));
        assert!(!vars.contains(&"output_int_register_20".to_string()));
    }

    #[test]
    fn lower_register_block_present_from_3_4() {
        assert!(
            output_variables(v(3, 4, 0), false).contains(&"output_int_register_12".to_string())
        );
        assert!(
            !output_variables(v(3, 3, 9), false).contains(&"output_int_register_12".to_string())
        );
    }

    #[test]
    fn upper_register_block_needs_3_9_or_5_3() {
        let vars = output_variables(v(5, 11, 0), true);
        assert!(vars.contains(&"output_int_register_26".to_string())); // 24 + 2
        assert!(vars.contains(&"output_int_register_36".to_string())); // 24 + 12
        assert!(vars.contains(&"output_double_register_43".to_string())); // 24 + 19

        assert!(
            !output_variables(v(5, 2, 0), true).contains(&"output_int_register_36".to_string())
        );
        assert!(
            !output_variables(v(3, 8, 0), true).contains(&"output_int_register_36".to_string())
        );
        assert!(output_variables(v(3, 9, 0), true).contains(&"output_int_register_36".to_string()));
    }

    #[test]
    fn version_gated_variables() {
        assert!(!output_variables(v(5, 8, 0), false).contains(&"ft_raw_wrench".to_string()));
        assert!(output_variables(v(5, 9, 0), false).contains(&"ft_raw_wrench".to_string()));

        assert!(
            !output_variables(v(5, 22, 0), false).contains(&"actual_current_as_torque".to_string())
        );
        assert!(
            output_variables(v(5, 23, 0), false).contains(&"actual_current_as_torque".to_string())
        );

        // payload: 3.11+, or 5.5.1+, or 5.6+
        assert!(output_variables(v(3, 11, 0), false).contains(&"payload".to_string()));
        assert!(!output_variables(v(3, 10, 0), false).contains(&"payload".to_string()));
        assert!(output_variables(v(5, 5, 1), false).contains(&"payload".to_string()));
        assert!(!output_variables(v(5, 5, 0), false).contains(&"payload".to_string()));
        assert!(output_variables(v(5, 6, 0), false).contains(&"payload".to_string()));

        assert!(output_variables(v(5, 11, 0), false).contains(&"payload_inertia".to_string()));
        assert!(!output_variables(v(5, 10, 0), false).contains(&"payload_inertia".to_string()));
    }
}
