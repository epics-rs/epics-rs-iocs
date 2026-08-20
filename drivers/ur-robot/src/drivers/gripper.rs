//! `URGripper` — asyn port driver for a Robotiq gripper on the tool port.
//!
//! Port of `urRobotApp/src/gripper_driver.cpp`. MAX_ADDR = 1. The gripper's IP
//! and its power state come from the dashboard driver, looked up by port name
//! through [`crate::registry`].

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;
use parking_lot::Mutex;

use crate::drivers::asyn_error;
use crate::drivers::ioc_ready::IocReady;
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

    /// The momentary commands of this port — see the
    /// [module docs](crate::drivers#momentary-commands).
    ///
    /// `SET_POSITION_RANGE` is here because it applies the range that
    /// `MIN_POSITION` / `MAX_POSITION` staged; its own value is unread.
    fn is_command(&self, reason: usize) -> bool {
        reason == self.connect
            || reason == self.activate
            || reason == self.open
            || reason == self.close
            || reason == self.set_position_range
            || reason == self.auto_calibrate
    }

    /// The device-state readbacks that carry the COMM alarm while the
    /// gripper link is down. Excluded beyond the commands: `IS_CONNECTED`
    /// (the health readback must stay valid at 0 — it is the one PV that
    /// says why) and the five operator settings
    /// (`SET_SPEED`/`SET_FORCE`/`MIN_POSITION`/`MAX_POSITION`/
    /// `POSITION_UNIT`), which hold setpoints, not device state.
    /// `IS_CALIBRATED` is a target although the write path publishes it:
    /// it is device state and goes stale with the link like the rest.
    /// `alarm_set_is_every_device_readback` enforces that a new parameter
    /// lands in exactly one group.
    fn alarm_targets(&self) -> Vec<(usize, i32)> {
        [
            self.is_open,
            self.is_closed,
            self.is_stopped_inner,
            self.is_stopped_outer,
            self.is_active,
            self.open_position,
            self.closed_position,
            self.current_position,
            self.move_status,
            self.is_calibrated,
        ]
        .into_iter()
        .map(|r| (r, 0))
        .collect()
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
                self.apply_pending_activation(&mut gripper);
                true
            }
            Err(e) => {
                log::error!("ur-robot: gripper connect failed: {e}");
                false
            }
        }
    }

    /// Deliver a boot-time `ACTIVATE` once the link is up (B3). The PINI
    /// `$(AUTO_ACTIVATE=YES)` write lands before any connect can have
    /// succeeded, so the cached value is the desired state and the
    /// connection owner delivers it here. `activate` skips itself when the
    /// gripper is already active, so a reconnect never re-runs the
    /// activation cycle.
    fn apply_pending_activation(&self, gripper: &mut RobotiqGripper) {
        let p = self.params;
        if self.base.params.get_int32(p.activate, 0).unwrap_or(0) == 0 {
            return;
        }
        if let Err(e) = gripper.activate(false) {
            log::error!("ur-robot: deferred gripper activation failed: {e}");
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

        if p.is_command(reason) && value == 0 {
            return Ok(());
        }

        // The position range and unit configure the RobotiqGripper object,
        // which outlives every (re)connect — nothing goes on the wire. They
        // apply regardless of the link state, so the PINI writes at boot
        // (robot off, gripper unreachable) cannot be lost. C gates them on
        // the connection and the boot values are lost.
        // MIN_POSITION / MAX_POSITION only stage the range; SET_POSITION_RANGE
        // applies it.
        if reason == p.min_position || reason == p.max_position {
            return Ok(());
        }
        if reason == p.set_position_range {
            let min = self.base.params.get_int32(p.min_position, 0).unwrap_or(0);
            let max = self.base.params.get_int32(p.max_position, 0).unwrap_or(0);
            self.gripper.lock().set_native_position_range(min, max);
            return Ok(());
        }
        if reason == p.position_unit {
            match position_unit(value) {
                Some(unit) => self.gripper.lock().set_unit(MoveParameter::Position, unit),
                None => {
                    log::warn!("ur-robot: position unit {value} is undefined; no action taken");
                }
            }
            return Ok(());
        }

        // An ACTIVATE that arrives while the gripper is unreachable is the
        // desired state, not an error: try_connect activates from the cached
        // value once the link is up (the PINI `$(AUTO_ACTIVATE=YES)` write
        // always lands before any connect can have succeeded). C errors
        // and the boot-time activation is lost.
        if reason == p.activate && !(self.robot_ready() && self.gripper.lock().is_connected()) {
            log::info!("ur-robot: gripper activation deferred until the gripper connects");
            return Ok(());
        }

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

        let mut gripper = self.gripper.lock();
        if !gripper.is_connected() {
            return Err(asyn_error("the Robotiq gripper is not connected"));
        }

        let mut calibrated = None;
        // Pre-checked open/close (upstream 558b98f, gripper_driver.cpp:218-245):
        // a request whose end state already holds — open when already open or
        // stopped at an outer object, close when already closed or stopped at
        // an inner object — starts no motion, and the OPEN/CLOSE busy param is
        // staged back to 0 so the record releases instead of waiting for a
        // MoveStatus transition that will never come.
        let mut refused = None;
        let result: UrResult<()> = if reason == p.activate {
            gripper.activate(false)
        } else if reason == p.open {
            (|| {
                if gripper.is_open()?
                    || gripper.object_detection_status()? == ObjectStatus::StoppedOuterObject
                {
                    log::warn!("ur-robot: gripper already open or stopped at an outer object");
                    refused = Some(p.open);
                    return Ok(());
                }
                gripper.open(MoveMode::StartMove).map(|_| ())
            })()
        } else if reason == p.close {
            (|| {
                if gripper.is_closed()?
                    || gripper.object_detection_status()? == ObjectStatus::StoppedInnerObject
                {
                    log::warn!("ur-robot: gripper already closed or stopped at an inner object");
                    refused = Some(p.close);
                    return Ok(());
                }
                gripper.close(MoveMode::StartMove).map(|_| ())
            })()
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
        if let Some(param) = refused {
            self.base.params.set_int32(param, 0, 0)?;
        }
        result.map_err(|e| asyn_error(format!("gripper command failed: {e}")))
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let p = self.params;
        self.base.params.set_float64(reason, user.addr, value)?;

        // SET_SPEED / SET_FORCE configure the RobotiqGripper object, which
        // outlives every (re)connect — nothing goes on the wire until the
        // next move. They apply regardless of the link state, so the PINI
        // writes at boot (robot off, gripper unreachable) cannot be lost.
        // C gates them on the connection and the boot values are lost.
        let mut gripper = self.gripper.lock();
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
    ready: Arc<IocReady>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("ur-gripper-poll".into())
        .spawn(move || {
            ready.wait();
            let alarm_targets = params.alarm_targets();
            // Starts true so an IOC that boots with the gripper down raises
            // the COMM alarm on its first cycle.
            let mut was_healthy = true;
            loop {
                let (mut updates, healthy_now) = {
                    let mut g = gripper.lock();
                    if dashboard.get().robot_on() && g.is_connected() {
                        match poll_once(params, &mut g) {
                            Ok(updates) => (updates, true),
                            Err(e) => {
                                log::error!("ur-robot: gripper poll error: {e}");
                                g.disconnect();
                                (
                                    vec![ParamSetValue::new(
                                        params.is_connected,
                                        0,
                                        ParamValue::Int32(0),
                                    )],
                                    false,
                                )
                            }
                        }
                    } else {
                        g.disconnect();
                        (
                            vec![ParamSetValue::new(
                                params.is_connected,
                                0,
                                ParamValue::Int32(0),
                            )],
                            false,
                        )
                    }
                };
                updates.extend(crate::drivers::health_transition(
                    &alarm_targets,
                    healthy_now,
                    &mut was_healthy,
                ));
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
        ParamSetValue::new(p.is_connected, 0, ParamValue::Int32(1)),
        ParamSetValue::new(p.is_active, 0, ParamValue::Int32(i32::from(is_active))),
        ParamSetValue::new(p.is_open, 0, ParamValue::Int32(i32::from(is_open))),
        ParamSetValue::new(p.is_closed, 0, ParamValue::Int32(i32::from(is_closed))),
        ParamSetValue::new(p.current_position, 0, ParamValue::Float64(current)),
        ParamSetValue::new(p.open_position, 0, ParamValue::Float64(g.open_position())),
        ParamSetValue::new(
            p.closed_position,
            0,
            ParamValue::Float64(g.closed_position()),
        ),
        ParamSetValue::new(p.move_status, 0, ParamValue::Int32(move_status.raw())),
        ParamSetValue::new(p.is_stopped_inner, 0, ParamValue::Int32(i32::from(inner))),
        ParamSetValue::new(p.is_stopped_outer, 0, ParamValue::Int32(i32::from(outer))),
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
    use crate::registry::DashboardState;

    #[test]
    fn the_position_unit_choices_map_to_the_four_units() {
        assert_eq!(position_unit(0), Some(Unit::Device));
        assert_eq!(position_unit(1), Some(Unit::Normalized));
        assert_eq!(position_unit(2), Some(Unit::Percent));
        assert_eq!(position_unit(3), Some(Unit::Mm));
        assert_eq!(position_unit(4), None);
        assert_eq!(position_unit(-1), None);
    }

    /// `MIN_POSITION` / `MAX_POSITION` stage the range and `POSITION_UNIT`
    /// selects a unit by number, so zero is a legitimate write for all three;
    /// only the `SET_POSITION_RANGE` that applies them is a command.
    #[test]
    fn the_six_commands_are_commands_and_the_staged_range_is_not() {
        let mut base = PortDriverBase::new("grip_is_command", 1, PortFlags::default());
        let p = GripperParams::create(&mut base).expect("params create");

        for reason in [
            p.connect,
            p.activate,
            p.open,
            p.close,
            p.set_position_range,
            p.auto_calibrate,
        ] {
            assert!(p.is_command(reason));
        }
        for reason in [
            p.min_position,
            p.max_position,
            p.position_unit,
            p.set_speed,
            p.set_force,
        ] {
            assert!(!p.is_command(reason));
        }
    }

    /// A gripper fixture that answers `GET <VAR>` from a fixed map, acks
    /// every `SET`, and records each request line it saw.
    fn spawn_recording_gripper(
        vars: Vec<(&'static str, i32)>,
    ) -> (u16, std::thread::JoinHandle<Vec<String>>) {
        use std::io::{BufRead, BufReader, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let jh = std::thread::spawn(move || {
            let state: std::collections::HashMap<String, i32> =
                vars.into_iter().map(|(k, v)| (k.to_string(), v)).collect();
            let (sock, _) = listener.accept().unwrap();
            let mut w = sock.try_clone().unwrap();
            let mut r = BufReader::new(sock);
            let mut lines = Vec::new();
            loop {
                let mut line = String::new();
                if r.read_line(&mut line).unwrap_or(0) == 0 {
                    break;
                }
                let line = line.trim_end().to_string();
                let f: Vec<&str> = line.split_whitespace().collect();
                lines.push(line.clone());
                match f.first().copied() {
                    Some("GET") => {
                        let v = state.get(f[1]).copied().unwrap_or(0);
                        w.write_all(format!("{} {}\n", f[1], v).as_bytes()).unwrap();
                    }
                    Some("SET") => w.write_all(b"ack\n").unwrap(),
                    _ => break,
                }
            }
            lines
        });
        (port, jh)
    }

    /// The boot race B3 closes: at iocsh time the dashboard has not
    /// connected, so every PINI write lands while the gripper is
    /// unreachable. The settings configure the persistent gripper object
    /// immediately, ACTIVATE is held as the desired state, and the first
    /// successful connect delivers it.
    #[test]
    fn boot_writes_survive_to_the_first_connect() {
        let dash = DashboardHandle::new("127.0.0.1");
        registry::register_dashboard("GRIP_B3_DASH", dash.clone());
        let mut drv = GripperDriver::new("GRIP_B3", "GRIP_B3_DASH").expect("driver");
        let p = drv.params();

        // The PINI writes: all must succeed unreachable (C errors and the
        // values are lost).
        drv.write_int32(&mut AsynUser::new(p.min_position), 10)
            .unwrap();
        drv.write_int32(&mut AsynUser::new(p.max_position), 200)
            .unwrap();
        drv.write_int32(&mut AsynUser::new(p.set_position_range), 1)
            .unwrap();
        drv.write_int32(&mut AsynUser::new(p.position_unit), 3)
            .unwrap();
        drv.write_float64(&mut AsynUser::new(p.set_speed), 0.5)
            .unwrap();
        drv.write_float64(&mut AsynUser::new(p.set_force), 0.25)
            .unwrap();
        drv.write_int32(&mut AsynUser::new(p.activate), 1).unwrap();

        // The numeric settings reached the gripper object at write time.
        assert_eq!(drv.gripper().lock().native_position_range(), (10, 200));

        // The robot comes up; the fixture reports an already-active gripper
        // (STA 3), so the delivered activation is its is_active probe.
        let (port, server) = spawn_recording_gripper(vec![("STA", 3)]);
        drv.gripper().lock().set_port(port);
        dash.set(DashboardState {
            connected: true,
            robot_mode: "Robotmode: IDLE".into(),
            ..Default::default()
        });

        drv.write_int32(&mut AsynUser::new(p.connect), 1).unwrap();
        drop(drv);

        // One STA probe from try_connect, one from the pending activation.
        let seen = server.join().unwrap();
        assert_eq!(
            seen.iter().filter(|l| l.as_str() == "GET STA").count(),
            2,
            "expected the connect probe plus the deferred activation, saw: {seen:?}"
        );
    }

    /// Without a cached ACTIVATE the connect must not touch the activation
    /// state — only the try_connect STA probe goes on the wire.
    #[test]
    fn a_connect_without_a_cached_activate_leaves_the_gripper_alone() {
        let dash = DashboardHandle::new("127.0.0.1");
        registry::register_dashboard("GRIP_B3_NOACT_DASH", dash.clone());
        let mut drv = GripperDriver::new("GRIP_B3_NOACT", "GRIP_B3_NOACT_DASH").expect("driver");
        let p = drv.params();

        let (port, server) = spawn_recording_gripper(vec![("STA", 3)]);
        drv.gripper().lock().set_port(port);
        dash.set(DashboardState {
            connected: true,
            robot_mode: "Robotmode: IDLE".into(),
            ..Default::default()
        });

        drv.write_int32(&mut AsynUser::new(p.connect), 1).unwrap();
        drop(drv);

        let seen = server.join().unwrap();
        assert_eq!(
            seen.iter().filter(|l| l.as_str() == "GET STA").count(),
            1,
            "expected only the connect probe, saw: {seen:?}"
        );
    }

    /// Completeness of the alarm set: every parameter this driver creates is
    /// a command, an alarm target, or one of the deliberate exclusions — a
    /// parameter added without classification fails here instead of silently
    /// staying NO_ALARM through an outage.
    #[test]
    fn alarm_set_is_every_device_readback() {
        let mut base = PortDriverBase::new("grip_alarm_set", 1, PortFlags::default());
        let p = GripperParams::create(&mut base).expect("params create");

        let targets = p.alarm_targets();
        let excluded = [
            p.is_connected,
            p.set_speed,
            p.set_force,
            p.min_position,
            p.max_position,
            p.position_unit,
        ];
        for reason in 0..base.params.len() {
            let targeted = targets.iter().any(|(r, _)| *r == reason);
            let exempt = excluded.contains(&reason) || p.is_command(reason);
            assert!(
                targeted != exempt,
                "param {reason} must be exactly one of: alarm target, command/exclusion"
            );
        }
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
