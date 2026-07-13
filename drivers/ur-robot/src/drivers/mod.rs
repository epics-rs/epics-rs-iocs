//! The five asyn port drivers, one per `asynPortDriver` subclass in urRobot.
//!
//! | driver | urRobot class | iocsh command |
//! |---|---|---|
//! | [`dashboard::DashboardDriver`] | `URDashboard` | `URDashboardConfig` |
//! | [`receive::ReceiveDriver`] | `RTDEReceive` | `RTDEReceiveConfig` |
//! | [`control::ControlDriver`] | `RTDEControl` | `RTDEControlConfig` |
//! | [`io::IoDriver`] | `RTDEInOut` | `RTDEInOutConfig` |
//! | [`gripper::GripperDriver`] | `URGripper` | `URGripperConfig` |

pub mod control;
pub mod dashboard;
pub mod gripper;
pub mod io;
pub mod receive;
pub mod runtime;

use epics_rs::asyn::error::{AsynError, AsynStatus};

/// The `asynError` a C driver returns from `writeInt32` / `writeFloat64` /
/// `writeOctet` when the device call failed.
pub(crate) fn asyn_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}
