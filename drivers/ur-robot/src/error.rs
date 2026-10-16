//! Errors raised by the UR transports.

/// Failure from any of the UR interfaces (RTDE, dashboard, script, gripper).
#[derive(Debug, thiserror::Error)]
pub enum UrError {
    /// The transport's own connect failure text (its C++ `what()`).
    #[error("{0}")]
    Connect(String),

    #[error("{0} is not connected")]
    NotConnected(String),

    #[error("i/o error: {0}")]
    Io(String),

    /// The RTDE peer closed the socket (asio's `error::eof`).
    #[error("End of file")]
    PeerClosed,

    #[error("protocol error: {0}")]
    Protocol(String),

    #[error("control script error: {0}")]
    Script(String),

    #[error("RTDE data package: {0}")]
    Decode(#[from] crate::state::DecodeError),

    /// A dashboard call failed; carries ur_rtde's `what()` — for most
    /// commands the server's reply itself (dashboard_client.cpp:95-135).
    #[error("{0}")]
    Dashboard(String),

    /// Requested RTDE output variables the controller does not know, comma
    /// separated (rtde.cpp:414-434).
    #[error(
        "The following variables was not found by the controller: [{0}],\n see which variables \
         are supported by your PolyScope version here: \n\
         https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/"
    )]
    VariablesNotFound(String),

    /// Another fieldbus already owns an RTDE input register we need
    /// (rtde.cpp:397).
    #[error(
        "One of the RTDE input registers are already in use! Currently you must disable the \
         EtherNet/IP adapter, PROFINET or any MODBUS unit configured on the robot. This might \
         change in the future."
    )]
    InputRegistersInUse,

    /// `RTDEControlInterface` saw no start reply before
    /// `RTDE_START_SYNCHRONIZATION_TIMEOUT`.
    #[error("Failed to start RTDE data synchronization, before timeout")]
    SyncStartTimeout,

    #[error("timed out after {0:?}: {1}")]
    Timeout(std::time::Duration, String),

    #[error("value {value} is not within [{min}; {max}]")]
    OutOfRange { value: f64, min: f64, max: f64 },

    /// The control script refused a command (C++ returns a bare `false`,
    /// indistinguishable from a query answering "no"; here the refusal is
    /// its own variant and carries why).
    #[error("command refused: {reason}")]
    CommandRefused { reason: RefusalReason },
}

/// Why the control script refused a command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefusalReason {
    /// The program is stopped or the control script is not running.
    NotRunning,
    /// A protective or emergency stop is active.
    SafetyStop,
    /// The script did not become ready, or did not finish, in time.
    Timeout,
}

impl std::fmt::Display for RefusalReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::NotRunning => "the control script is not running",
            Self::SafetyStop => "a protective or emergency stop is active",
            Self::Timeout => "the control script did not respond in time",
        })
    }
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
