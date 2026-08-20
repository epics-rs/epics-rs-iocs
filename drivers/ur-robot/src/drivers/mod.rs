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

/// The end-of-write flush every C handler performs: the default
/// `asynPortDriver::write*` end with `callParamCallbacks(addr)`
/// (asynPortDriver.cpp:2031), and every urRobot C override reaches its own
/// call through the `skip:` label — on error exits too. Without it, a value
/// cached by a write handler reaches I/O Intr records only at the next poll
/// cycle, and never on the poll-less io port. The write's own error outranks
/// a flush error.
pub(crate) fn flush_after<T>(
    base: &mut epics_rs::asyn::port::PortDriverBase,
    addr: i32,
    result: Result<T, AsynError>,
) -> Result<T, AsynError> {
    let flushed = base.call_param_callbacks(addr);
    result.and_then(|v| flushed.map(|()| v))
}

/// The alarm half of a poll cycle: on a health transition, the status
/// updates that raise (link lost) or clear (link recovered) the COMM alarm
/// on every readback `(reason, addr)` in `targets`; on a steady state,
/// nothing. Each poll thread is the single alarm owner for its port — the
/// C pattern is `setParamStatus(asynDisconnected)` + `callParamCallbacks()`
/// from the poll loop, and the record-side fill-in maps `Disconnected` to
/// COMM/INVALID (asynEpicsUtils.c:238-265), so the alarm pair is left 0
/// here rather than duplicating that mapping.
pub(crate) fn health_transition(
    targets: &[(usize, i32)],
    healthy_now: bool,
    was_healthy: &mut bool,
) -> Vec<epics_rs::asyn::request::ParamSetValue> {
    if healthy_now == *was_healthy {
        return Vec::new();
    }
    *was_healthy = healthy_now;
    let status = if healthy_now {
        AsynStatus::Success
    } else {
        AsynStatus::Disconnected
    };
    targets
        .iter()
        .map(|&(reason, addr)| {
            epics_rs::asyn::request::ParamSetValue::status(reason, addr, status, 0, 0)
        })
        .collect()
}
