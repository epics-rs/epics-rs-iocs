//! The five asyn port drivers, one per `asynPortDriver` subclass in urRobot.
//!
//! | driver | urRobot class | iocsh command |
//! |---|---|---|
//! | [`dashboard::DashboardDriver`] | `URDashboard` | `URDashboardConfig` |
//! | [`receive::ReceiveDriver`] | `RTDEReceive` | `RTDEReceiveConfig` |
//! | [`control::ControlDriver`] | `RTDEControl` | `RTDEControlConfig` |
//! | [`io::IoDriver`] | `RTDEInOut` | `RTDEInOutConfig` |
//! | [`gripper::GripperDriver`] | `URGripper` | `URGripperConfig` |
//!
//! # Momentary commands
//!
//! Most parameters carry a value: a setpoint, a level, a mode. A minority are
//! momentary commands — `MOVEJ`, `JOG_STOP`, `POWER_OFF`, `ACTIVATE` — where
//! the request *is* the write and the value says nothing. Each driver names
//! its own set in `is_command`, and every `write_int32` that can reach one
//! drops a zero write before acting on it.
//!
//! The gate exists because a command param is reachable from more than a
//! deliberate `caput <cmd> 1`. Records process each other: the jog watchdog
//! fires `Control:JogStop.PROC` when the client stops kicking it, the safety
//! fanout `Control:Stop` fires `Control:stopL.PROC`, and
//! `RobotiqGripper:Activate` carries `PINI="$(AUTO_ACTIVATE=YES)"` so iocInit
//! activates the gripper. Processing a record sends whatever `VAL` holds, so
//! every command record in `db/` declares `field(VAL, 1)` — that is the half
//! of this rule that lives in the database, and it is what keeps those paths
//! working once the driver stops acting on zero.
//!
//! What the gate buys: `caput Receive:Disconnect 0` no longer drops the RTDE
//! stream, and a bo left at zero cannot fire a command by being processed.
//! What it does not buy: `PINI` on a command record still runs that command
//! during iocInit, because `VAL` is 1 by then. That is deliberate for
//! `Activate` and an authoring error anywhere else.

pub mod control;
pub mod dashboard;
pub mod gripper;
pub mod io;
pub mod ioc_ready;
pub mod receive;
pub mod runtime;

use epics_rs::asyn::error::{AsynError, AsynStatus};

/// An RTDE stream with no data package for this long is stale: the socket is
/// open but the robot state it serves is no longer current. The default RTDE
/// output frequency is 125 Hz (500 Hz e-Series), so one second is >100 missed
/// packages — far past jitter, deliberately not configurable. Shared by the
/// receive driver (staleness → reconnect) and the control driver (staleness →
/// `IS_CONNECTED` only; control never reconnects on its own, as in C, because
/// a reconnect re-uploads the control script).
pub(crate) const STALE_AFTER: std::time::Duration = std::time::Duration::from_secs(1);

/// The `asynError` a C driver returns from `writeInt32` / `writeFloat64` /
/// `writeOctet` when the device call failed.
pub(crate) fn asyn_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}
