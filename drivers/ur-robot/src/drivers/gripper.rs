//! `URGripper` — asyn port driver for a Robotiq gripper on the tool port.
//!
//! Port of `urRobotApp/src/gripper_driver.cpp`. MAX_ADDR = 1. The gripper's IP
//! and its power state come from the dashboard driver, looked up by port name
//! through [`crate::registry`].

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;
use parking_lot::Mutex;

use crate::drivers::asyn_error;
use crate::error::{UrError, UrResult};
use crate::gripper::{MoveMode, MoveParameter, ObjectStatus, RobotiqGripper, Unit};
use crate::registry::{self, DashboardHandle};

/// The C++ driver hard-codes `RobotiqGripper(ip)`, whose socket timeout is
/// 2 s in ur_rtde.
const GRIPPER_TIMEOUT: Duration = Duration::from_millis(2000);

#[derive(Clone, Copy)]
pub struct GripperParams {
    pub connect: usize,
    pub is_connected: usize,
    pub is_open: usize,
    pub is_closed: usize,
    pub is_stopped_inner: usize,
    pub is_stopped_outer: usize,
    pub is_active: usize,
    pub activate: usize,
    pub open: usize,
    pub close: usize,
    pub set_speed: usize,
    pub set_force: usize,
    pub auto_calibrate: usize,
    pub open_position: usize,
    pub closed_position: usize,
    pub current_position: usize,
    pub move_status: usize,
    pub set_position_range: usize,
    pub min_position: usize,
    pub max_position: usize,
    pub position_unit: usize,
    pub is_calibrated: usize,
}

impl GripperParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            connect: base.create_param("CONNECT", ParamType::Int32)?,
            is_connected: base.create_param("IS_CONNECTED", ParamType::Int32)?,
            is_open: base.create_param("IS_OPEN", ParamType::Int32)?,
            is_closed: base.create_param("IS_CLOSED", ParamType::Int32)?,
            is_stopped_inner: base.create_param("IS_STOPPED_INNER", ParamType::Int32)?,
            is_stopped_outer: base.create_param("IS_STOPPED_OUTER", ParamType::Int32)?,
            is_active: base.create_param("IS_ACTIVE", ParamType::Int32)?,
            activate: base.create_param("ACTIVATE", ParamType::Int32)?,
            open: base.create_param("OPEN", ParamType::Int32)?,
            close: base.create_param("CLOSE", ParamType::Int32)?,
            set_speed: base.create_param("SET_SPEED", ParamType::Float64)?,
            set_force: base.create_param("SET_FORCE", ParamType::Float64)?,
            auto_calibrate: base.create_param("AUTO_CALIBRATE", ParamType::Int32)?,
            open_position: base.create_param("OPEN_POSITION", ParamType::Float64)?,
            closed_position: base.create_param("CLOSED_POSITION", ParamType::Float64)?,
            current_position: base.create_param("CURRENT_POSITION", ParamType::Float64)?,
            move_status: base.create_param("MOVE_STATUS", ParamType::Int32)?,
            set_position_range: base.create_param("SET_POSITION_RANGE", ParamType::Int32)?,
            min_position: base.create_param("MIN_POSITION", ParamType::Int32)?,
            max_position: base.create_param("MAX_POSITION", ParamType::Int32)?,
            position_unit: base.create_param("POSITION_UNIT", ParamType::Int32)?,
            is_calibrated: base.create_param("IS_CALIBRATED", ParamType::Int32)?,
        })
    }
}

/// The four `POSITION_UNIT` mbbo choices (gripper_driver.cpp:359-380).
fn position_unit(value: i32) -> Option<Unit> {
    match value {
        0 => Some(Unit::Device),
        1 => Some(Unit::Normalized),
        2 => Some(Unit::Percent),
        3 => Some(Unit::Mm),
        _ => None,
    }
}

pub struct GripperDriver {
    base: PortDriverBase,
    params: GripperParams,
    gripper: Arc<Mutex<RobotiqGripper>>,
    dashboard: DashboardHandle,
}

impl GripperDriver {
    /// `dashboard_port` is the asyn port name of the [`super::dashboard::DashboardDriver`]
    /// that owns the robot — it supplies the IP and the power state.
    pub fn new(port_name: &str, dashboard_port: &str) -> AsynResult<Self> {
        let dashboard = registry::dashboard(dashboard_port).ok_or_else(|| {
            asyn_error(format!(
                "no URDashboard port named {dashboard_port}; configure it before the gripper"
            ))
        })?;

        let mut base = PortDriverBase::new(
            port_name,
            1,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let params = GripperParams::create(&mut base)?;

        let gripper = Arc::new(Mutex::new(RobotiqGripper::new(
            &dashboard.ip,
            GRIPPER_TIMEOUT,
        )));
        let me = Self {
            base,
            params,
            gripper,
            dashboard,
        };
        me.try_connect();
        Ok(me)
    }

    pub fn params(&self) -> GripperParams {
        self.params
    }

    pub fn gripper(&self) -> Arc<Mutex<RobotiqGripper>> {
        Arc::clone(&self.gripper)
    }

    pub fn dashboard(&self) -> DashboardHandle {
        self.dashboard.clone()
    }

    fn robot_ready(&self) -> bool {
        self.dashboard.get().robot_on()
    }

    fn try_connect(&self) -> bool {
        if !self.robot_ready() {
            log::error!("ur-robot: cannot connect to the gripper; the robot must be powered on");
            return false;
        }
        let mut gripper = self.gripper.lock();
        match gripper.connect().and_then(|()| gripper.get_var("STA")) {
            Ok(_) => {
                log::info!("ur-robot: connected to the gripper");
                true
            }
            Err(e) => {
                log::error!("ur-robot: gripper connect failed: {e}");
                false
            }
        }
    }
}

impl PortDriver for GripperDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let p = self.params;
        self.base.params.set_int32(reason, user.addr, value)?;

        if !self.robot_ready() {
            return Err(asyn_error(
                "the robot must be powered on and the dashboard connected to use the gripper",
            ));
        }

        if reason == p.connect {
            return if self.try_connect() {
                Ok(())
            } else {
                Err(asyn_error("could not connect to the gripper"))
            };
        }

        // MIN_POSITION / MAX_POSITION only stage the range; SET_POSITION_RANGE
        // applies it.
        if reason == p.min_position || reason == p.max_position {
            return Ok(());
        }

        let min = self.base.params.get_int32(p.min_position, 0).unwrap_or(0);
        let max = self.base.params.get_int32(p.max_position, 0).unwrap_or(0);

        let mut gripper = self.gripper.lock();
        if !gripper.is_connected() {
            return Err(asyn_error("the Robotiq gripper is not connected"));
        }

        let mut calibrated = None;
        let result: UrResult<()> = if reason == p.activate {
            gripper.activate(false)
        } else if reason == p.open {
            gripper.open(MoveMode::StartMove).map(|_| ())
        } else if reason == p.close {
            gripper.close(MoveMode::StartMove).map(|_| ())
        } else if reason == p.set_position_range {
            gripper.set_native_position_range(min, max);
            Ok(())
        } else if reason == p.position_unit {
            match position_unit(value) {
                Some(unit) => {
                    gripper.set_unit(MoveParameter::Position, unit);
                    Ok(())
                }
                None => {
                    log::warn!("ur-robot: position unit {value} is undefined; no action taken");
                    Ok(())
                }
            }
        } else if reason == p.auto_calibrate {
            match gripper.is_active() {
                Ok(true) => gripper.auto_calibrate(None).inspect(|()| {
                    calibrated = Some(1);
                }),
                Ok(false) => Err(UrError::Protocol(
                    "activate the gripper before auto-calibrating".into(),
                )),
                Err(e) => Err(e),
            }
        } else {
            Ok(())
        };
        drop(gripper);

        if let Some(v) = calibrated {
            self.base.params.set_int32(p.is_calibrated, 0, v)?;
        }
        result.map_err(|e| asyn_error(format!("gripper command failed: {e}")))
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let p = self.params;
        self.base.params.set_float64(reason, user.addr, value)?;

        if !self.robot_ready() {
            return Err(asyn_error(
                "the robot must be powered on and the dashboard connected to use the gripper",
            ));
        }

        let mut gripper = self.gripper.lock();
        if !gripper.is_connected() {
            return Err(asyn_error("the Robotiq gripper is not connected"));
        }

        if reason == p.set_speed {
            gripper.set_speed(value);
        } else if reason == p.set_force {
            gripper.set_force(value);
        }
        Ok(())
    }
}

/// The gripper poll thread (`URGripper::poll`).
pub fn start_poller(
    handle: PortHandle,
    params: GripperParams,
    gripper: Arc<Mutex<RobotiqGripper>>,
    dashboard: DashboardHandle,
    poll_period: Duration,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("ur-gripper-poll".into())
        .spawn(move || {
            loop {
                let updates = {
                    let mut g = gripper.lock();
                    if dashboard.get().robot_on() && g.is_connected() {
                        match poll_once(params, &mut g) {
                            Ok(updates) => updates,
                            Err(e) => {
                                log::error!("ur-robot: gripper poll error: {e}");
                                g.disconnect();
                                vec![ParamSetValue::Int32 {
                                    reason: params.is_connected,
                                    addr: 0,
                                    value: 0,
                                }]
                            }
                        }
                    } else {
                        g.disconnect();
                        vec![ParamSetValue::Int32 {
                            reason: params.is_connected,
                            addr: 0,
                            value: 0,
                        }]
                    }
                };
                let _ = handle.set_params_and_notify_blocking(0, updates);
                std::thread::sleep(poll_period);
            }
        })
        .expect("failed to spawn the gripper poll thread")
}

/// One pass of the gripper poll loop.
pub fn poll_once(p: GripperParams, g: &mut RobotiqGripper) -> UrResult<Vec<ParamSetValue>> {
    let is_active = g.is_active()?;
    let is_open = g.is_open()?;
    let is_closed = g.is_closed()?;
    let current = g.current_position()?;
    let move_status = g.object_detection_status()?;
    let (inner, outer) = stopped_flags(move_status);

    Ok(vec![
        ParamSetValue::Int32 {
            reason: p.is_connected,
            addr: 0,
            value: 1,
        },
        ParamSetValue::Int32 {
            reason: p.is_active,
            addr: 0,
            value: i32::from(is_active),
        },
        ParamSetValue::Int32 {
            reason: p.is_open,
            addr: 0,
            value: i32::from(is_open),
        },
        ParamSetValue::Int32 {
            reason: p.is_closed,
            addr: 0,
            value: i32::from(is_closed),
        },
        ParamSetValue::Float64 {
            reason: p.current_position,
            addr: 0,
            value: current,
        },
        ParamSetValue::Float64 {
            reason: p.open_position,
            addr: 0,
            value: g.open_position(),
        },
        ParamSetValue::Float64 {
            reason: p.closed_position,
            addr: 0,
            value: g.closed_position(),
        },
        ParamSetValue::Int32 {
            reason: p.move_status,
            addr: 0,
            value: move_status.raw(),
        },
        ParamSetValue::Int32 {
            reason: p.is_stopped_inner,
            addr: 0,
            value: i32::from(inner),
        },
        ParamSetValue::Int32 {
            reason: p.is_stopped_outer,
            addr: 0,
            value: i32::from(outer),
        },
    ])
}

fn stopped_flags(status: ObjectStatus) -> (bool, bool) {
    match status {
        ObjectStatus::StoppedInnerObject => (true, false),
        ObjectStatus::StoppedOuterObject => (false, true),
        _ => (false, false),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_position_unit_choices_map_to_the_four_units() {
        assert_eq!(position_unit(0), Some(Unit::Device));
        assert_eq!(position_unit(1), Some(Unit::Normalized));
        assert_eq!(position_unit(2), Some(Unit::Percent));
        assert_eq!(position_unit(3), Some(Unit::Mm));
        assert_eq!(position_unit(4), None);
        assert_eq!(position_unit(-1), None);
    }

    #[test]
    fn only_the_two_stopped_states_raise_a_stopped_flag() {
        assert_eq!(
            stopped_flags(ObjectStatus::StoppedInnerObject),
            (true, false)
        );
        assert_eq!(
            stopped_flags(ObjectStatus::StoppedOuterObject),
            (false, true)
        );
        assert_eq!(stopped_flags(ObjectStatus::Moving), (false, false));
        assert_eq!(stopped_flags(ObjectStatus::AtDest), (false, false));
    }
}
