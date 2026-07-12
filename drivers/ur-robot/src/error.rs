//! Errors raised by the UR transports.

/// Failure from any of the UR interfaces (RTDE, dashboard, script, gripper).
#[derive(Debug, thiserror::Error)]
pub enum UrError {
    #[error("connect failed: {0}")]
    Connect(String),

    #[error("{0} is not connected")]
    NotConnected(String),

    #[error("i/o error: {0}")]
    Io(String),

    #[error("protocol error: {0}")]
    Protocol(String),

    #[error("control script error: {0}")]
    Script(String),

    #[error("RTDE data package: {0}")]
    Decode(#[from] crate::state::DecodeError),

    /// The robot answered a dashboard command with something other than the
    /// success line the command requires.
    #[error("dashboard command '{command}' failed: {reply}")]
    Dashboard { command: String, reply: String },

    /// A requested RTDE output variable is not known to this controller.
    #[error("controller does not provide RTDE variable(s): {0}")]
    VariablesNotFound(String),

    /// Another fieldbus already owns an RTDE input register we need.
    #[error(
        "an RTDE input register is already in use; disable the EtherNet/IP adapter, \
         PROFINET or any MODBUS unit configured on the robot"
    )]
    InputRegistersInUse,

    #[error("timed out after {0:?}: {1}")]
    Timeout(std::time::Duration, String),

    #[error("value {value} is not within [{min}; {max}]")]
    OutOfRange { value: f64, min: f64, max: f64 },
}

pub type UrResult<T> = Result<T, UrError>;

/// `verifyValueIsWithin` (rtde_control_interface.cpp:22 and
/// rtde_io_interface.cpp:167).
pub fn verify_within(value: f64, min: f64, max: f64) -> UrResult<()> {
    if value.is_nan() || !(value >= min && value <= max) {
        return Err(UrError::OutOfRange { value, min, max });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn range_check_rejects_nan_and_out_of_band() {
        // The joint-velocity band, 0..3.14 rad/s.
        const HIGH: f64 = 314.0 / 100.0;
        assert!(verify_within(1.0, 0.0, HIGH).is_ok());
        assert!(verify_within(0.0, 0.0, HIGH).is_ok());
        assert!(verify_within(HIGH, 0.0, HIGH).is_ok());
        assert!(verify_within(-0.1, 0.0, HIGH).is_err());
        assert!(verify_within(3.15, 0.0, HIGH).is_err());
        assert!(verify_within(f64::NAN, 0.0, HIGH).is_err());
    }
}
