//! `RTDEControl` — asyn port driver for robot motion.
//!
//! Port of `urRobotApp/src/rtde_control_driver.cpp`. MAX_ADDR = 6 (one address
//! per joint / pose element). The poll thread drives the asynchronous-motion
//! state machine and watches for a custom URScript finishing.

use std::sync::Arc;
use std::time::{Duration, Instant};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;
use parking_lot::Mutex;

use crate::control::ControlInterface;
use crate::drivers::asyn_error;
use crate::error::{UrError, UrResult};
use crate::registry::{self, DashboardHandle, ReceiveHandle};

pub const NUM_JOINTS: usize = 6;

/// The safety word the control interface will act in: NORMAL only.
const SAFETY_NORMAL: i32 = 1;
/// `poll_custom_script()` blocks until the control script is running again,
/// with this bound (rtde_control_driver.cpp:133).
const REUPLOAD_TIMEOUT: Duration = Duration::from_secs(1);

fn deg_to_rad(deg: f64) -> f64 {
    deg * std::f64::consts::PI / 180.0
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionType {
    Joint,
    Cartesian,
}

/// `Done -> WaitingMotion -> [WaitingAction ->] Done`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionStatus {
    Done,
    WaitingMotion,
    WaitingAction,
}

#[derive(Debug, Clone, Copy)]
pub struct MotionTask {
    pub motion: MotionType,
    /// Run the waypoint action after the move finishes.
    pub action: bool,
}

/// A custom URScript running on the controller in place of the control script.
#[derive(Debug, Clone, Copy)]
struct CustomScript {
    started: Instant,
    /// `output_int_register_12` as it read when the script was launched; the
    /// wrapped script bumps it on completion.
    start_count: i32,
    timeout: Duration,
}

#[derive(Clone, Copy)]
pub struct ControlParams {
    pub disconnect: usize,
    pub reconnect: usize,
    pub is_connected: usize,
    pub is_steady: usize,
    pub move_j: usize,
    pub stop_j: usize,
    pub actual_q: usize,
    pub joint_cmd: usize,
    pub move_l: usize,
    pub stop_l: usize,
    pub actual_tcp_pose: usize,
    pub pose_cmd: usize,
    pub tcp_offset: usize,
    pub reupload_control_script: usize,
    pub stop_control_script: usize,
    pub joint_speed: usize,
    pub joint_acceleration: usize,
    pub joint_blend: usize,
    pub linear_speed: usize,
    pub linear_acceleration: usize,
    pub linear_blend: usize,
    pub async_move_done: usize,
    pub waypoint_move: usize,
    pub run_waypoint_action: usize,
    pub waypoint_action_done: usize,
    pub teach_mode: usize,
    pub trigger_prot_stop: usize,
    pub motion_done_count: usize,
    pub custom_script_file: usize,
    pub custom_inline_script: usize,
    pub run_custom_script_file: usize,
    pub custom_script_running: usize,
    pub custom_script_error: usize,
    pub custom_script_timeout: usize,
    pub jog_start: usize,
    pub jog_stop: usize,
    pub jog_speed: usize,
    pub jog_acceleration: usize,
    pub jogging: usize,
}

impl ControlParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            disconnect: base.create_param("DISCONNECT", ParamType::Int32)?,
            reconnect: base.create_param("RECONNECT", ParamType::Int32)?,
            is_connected: base.create_param("IS_CONNECTED", ParamType::Int32)?,
            is_steady: base.create_param("IS_STEADY", ParamType::Int32)?,
            move_j: base.create_param("MOVEJ", ParamType::Int32)?,
            stop_j: base.create_param("STOPJ", ParamType::Int32)?,
            actual_q: base.create_param("ACTUAL_Q", ParamType::Float64Array)?,
            joint_cmd: base.create_param("JOINT_CMD", ParamType::Float64)?,
            move_l: base.create_param("MOVEL", ParamType::Int32)?,
            stop_l: base.create_param("STOPL", ParamType::Int32)?,
            actual_tcp_pose: base.create_param("ACTUAL_TCP_POSE", ParamType::Float64Array)?,
            pose_cmd: base.create_param("POSE_CMD", ParamType::Float64)?,
            tcp_offset: base.create_param("TCP_OFFSET", ParamType::Float64)?,
            reupload_control_script: base
                .create_param("REUPLOAD_CONTROL_SCRIPT", ParamType::Int32)?,
            stop_control_script: base.create_param("STOP_CONTROL_SCRIPT", ParamType::Int32)?,
            joint_speed: base.create_param("JOINT_SPEED", ParamType::Float64)?,
            joint_acceleration: base.create_param("JOINT_ACCELERATION", ParamType::Float64)?,
            joint_blend: base.create_param("JOINT_BLEND", ParamType::Float64)?,
            linear_speed: base.create_param("LINEAR_SPEED", ParamType::Float64)?,
            linear_acceleration: base.create_param("LINEAR_ACCELERATION", ParamType::Float64)?,
            linear_blend: base.create_param("LINEAR_BLEND", ParamType::Float64)?,
            async_move_done: base.create_param("ASYNC_MOVE_DONE", ParamType::Int32)?,
            waypoint_move: base.create_param("WAYPOINT_MOVE", ParamType::Int32)?,
            run_waypoint_action: base.create_param("RUN_WAYPOINT_ACTION", ParamType::Int32)?,
            waypoint_action_done: base.create_param("WAYPOINT_ACTION_DONE", ParamType::Int32)?,
            teach_mode: base.create_param("TEACH_MODE", ParamType::Int32)?,
            trigger_prot_stop: base.create_param("TRIGGER_PROT_STOP", ParamType::Int32)?,
            motion_done_count: base.create_param("MOTION_DONE_COUNT", ParamType::Int32)?,
            custom_script_file: base.create_param("CUSTOM_SCRIPT_FILE", ParamType::Octet)?,
            custom_inline_script: base.create_param("CUSTOM_INLINE_SCRIPT", ParamType::Octet)?,
            run_custom_script_file: base
                .create_param("RUN_CUSTOM_SCRIPT_FILE", ParamType::Int32)?,
            custom_script_running: base.create_param("CUSTOM_SCRIPT_RUNNING", ParamType::Int32)?,
            custom_script_error: base.create_param("CUSTOM_SCRIPT_ERROR", ParamType::Int32)?,
            custom_script_timeout: base
                .create_param("CUSTOM_SCRIPT_TIMEOUT", ParamType::Float64)?,
            jog_start: base.create_param("JOG_START", ParamType::Int32)?,
            jog_stop: base.create_param("JOG_STOP", ParamType::Int32)?,
            jog_speed: base.create_param("JOG_SPEED", ParamType::Float64)?,
            jog_acceleration: base.create_param("JOG_ACCELERATION", ParamType::Float64)?,
            jogging: base.create_param("JOGGING", ParamType::Int32)?,
        })
    }
}

/// Everything the write handlers and the poll thread share.
///
/// The interface and the motion state live under one lock: the poll thread has
/// to read the state, issue a motion command and update the state as one step,
/// and every write handler mutates both. Two locks would make that ordering a
/// standing invariant to enforce by hand; one lock makes it hold by
/// construction.
pub struct ControlInner {
    pub iface: Option<ControlInterface>,
    /// Commanded joint angles, radians.
    pub cmd_joints: [f64; NUM_JOINTS],
    /// Commanded TCP pose: x,y,z metres, rx,ry,rz radians.
    pub cmd_pose: [f64; NUM_JOINTS],
    /// TCP offset: x,y,z metres, rx,ry,rz radians.
    pub tcp_offset: [f64; NUM_JOINTS],
    /// Jog speeds: m/s and rad/s.
    pub jog_speeds: [f64; NUM_JOINTS],
    pub jog_acceleration: f64,
    pub new_jog: bool,
    /// moveJ dynamics, rad/s and rad/s².
    pub joint_speed: f64,
    pub joint_accel: f64,
    pub joint_blend: f64,
    /// moveL dynamics, m/s and m/s².
    pub linear_speed: f64,
    pub linear_accel: f64,
    pub linear_blend: f64,

    pub waypoint_move: bool,
    pub waypoint_action_done: bool,
    pub pending_motion: Option<MotionTask>,
    pub motion_status: MotionStatus,
    pub motion_done_count: i32,
    run_action_val: i32,

    pub custom_script_path: String,
    pub custom_script_timeout: Duration,
    custom_script: Option<CustomScript>,
}

impl Default for ControlInner {
    fn default() -> Self {
        Self {
            iface: None,
            cmd_joints: [0.0; NUM_JOINTS],
            cmd_pose: [0.0; NUM_JOINTS],
            tcp_offset: [0.0; NUM_JOINTS],
            jog_speeds: [0.0; NUM_JOINTS],
            jog_acceleration: 0.0,
            new_jog: false,
            joint_speed: 0.5,
            joint_accel: 1.4,
            joint_blend: 0.0,
            linear_speed: 0.05,
            linear_accel: 0.5,
            linear_blend: 0.0,
            waypoint_move: false,
            waypoint_action_done: false,
            pending_motion: None,
            motion_status: MotionStatus::Done,
            motion_done_count: 0,
            run_action_val: 0,
            custom_script_path: String::new(),
            custom_script_timeout: Duration::ZERO,
            custom_script: None,
        }
    }
}

impl ControlInner {
    fn connected(&self) -> bool {
        self.iface
            .as_ref()
            .is_some_and(ControlInterface::is_connected)
    }

    fn iface_mut(&mut self) -> UrResult<&mut ControlInterface> {
        match self.iface.as_mut() {
            Some(i) if i.is_connected() => Ok(i),
            Some(_) => Err(UrError::NotConnected("RTDE control".into())),
            None => Err(UrError::NotConnected(
                "RTDE control (not initialised)".into(),
            )),
        }
    }

    /// `set_motion_task_done()`.
    fn motion_task_done(&mut self, p: ControlParams, out: &mut Vec<ParamSetValue>) {
        self.motion_status = MotionStatus::Done;
        self.pending_motion = None;
        self.motion_done_count += 1;
        out.push(ParamSetValue::new(
            p.async_move_done,
            0,
            ParamValue::Int32(1),
        ));
        out.push(ParamSetValue::new(
            p.motion_done_count,
            0,
            ParamValue::Int32(self.motion_done_count),
        ));
    }
}

pub struct ControlDriver {
    base: PortDriverBase,
    params: ControlParams,
    inner: Arc<Mutex<ControlInner>>,
    robot_ip: String,
    dashboard: DashboardHandle,
    receive: ReceiveHandle,
}

impl ControlDriver {
    /// `dashboard_port` supplies the robot IP and the RUNNING check;
    /// `receive_port` supplies the safety word and the custom-script counter.
    pub fn new(port_name: &str, dashboard_port: &str, receive_port: &str) -> AsynResult<Self> {
        let dashboard = registry::dashboard(dashboard_port).ok_or_else(|| {
            asyn_error(format!(
                "no URDashboard port named {dashboard_port}; configure it before the control port"
            ))
        })?;
        let receive = registry::receive(receive_port).ok_or_else(|| {
            asyn_error(format!(
                "no RTDEReceive port named {receive_port}; configure it before the control port"
            ))
        })?;

        let mut base = PortDriverBase::new(
            port_name,
            NUM_JOINTS,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let params = ControlParams::create(&mut base)?;

        let me = Self {
            base,
            params,
            inner: Arc::new(Mutex::new(ControlInner::default())),
            robot_ip: dashboard.ip.clone(),
            dashboard,
            receive,
        };
        me.try_connect();
        Ok(me)
    }

    pub fn params(&self) -> ControlParams {
        self.params
    }

    pub fn inner(&self) -> Arc<Mutex<ControlInner>> {
        Arc::clone(&self.inner)
    }

    pub fn receive(&self) -> ReceiveHandle {
        self.receive.clone()
    }

    /// `try_connect()`.
    ///
    /// Refuses to connect unless the safety word reads NORMAL and the dashboard
    /// reports `Robotmode: RUNNING`, exactly as upstream. Unlike upstream, an
    /// already-connected interface answers true instead of false
    /// (rtde_control_driver.cpp:66-77 only reports success when it had to
    /// reconnect, so RECONNECT on a healthy link returns asynError).
    fn try_connect(&self) -> bool {
        let receive = self.receive.get();
        if !receive.connected || receive.safety_status_bits != SAFETY_NORMAL {
            log::error!(
                "ur-robot: cannot connect the control interface in the current safety state; \
                 clear the safeguard stop / E-stop and try again"
            );
            return false;
        }
        if !self.dashboard.get().robot_running() {
            log::error!(
                "ur-robot: cannot connect the control interface in the current robot mode; \
                 power the robot on, release the brakes and try again"
            );
            return false;
        }

        let mut inner = self.inner.lock();
        match inner.iface.as_mut() {
            Some(iface) => {
                if iface.is_connected() {
                    return true;
                }
                match iface.reconnect() {
                    Ok(()) => {
                        log::info!("ur-robot: reconnected to the RTDE control interface");
                        true
                    }
                    Err(e) => {
                        log::error!("ur-robot: RTDE control reconnect failed: {e}");
                        false
                    }
                }
            }
            None => match ControlInterface::connect(&self.robot_ip, false) {
                Ok(iface) => {
                    log::info!("ur-robot: connected to the RTDE control interface");
                    inner.iface = Some(iface);
                    true
                }
                Err(e) => {
                    log::error!("ur-robot: RTDE control connect failed: {e}");
                    false
                }
            },
        }
    }

    /// Launch a custom URScript (from a file or inline) on the controller.
    fn run_custom_script(&mut self, body: &str, source: &str) -> AsynResult<()> {
        let p = self.params;
        let mut inner = self.inner.lock();
        if inner.pending_motion.is_some() && inner.motion_status != MotionStatus::WaitingAction {
            return Err(asyn_error(
                "a motion task is in progress; the URScript was not run",
            ));
        }
        let start_count = self.receive.get().output_int_register_12;
        let timeout = inner.custom_script_timeout;

        let iface = inner
            .iface_mut()
            .map_err(|e| asyn_error(format!("cannot run the URScript: {e}")))?;
        iface
            .send_custom_script(body)
            .map_err(|e| asyn_error(format!("failed to run the URScript {source}: {e}")))?;

        inner.custom_script = Some(CustomScript {
            started: Instant::now(),
            start_count,
            timeout,
        });
        drop(inner);

        self.base.params.set_int32(p.custom_script_error, 0, 0)?;
        self.base.params.set_int32(p.custom_script_running, 0, 1)?;
        Ok(())
    }
}

impl PortDriver for ControlDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        let p = self.params;
        self.base.params.set_float64(reason, addr, value)?;

        let index = usize::try_from(addr)
            .ok()
            .filter(|i| *i < NUM_JOINTS)
            .ok_or_else(|| asyn_error(format!("address {addr} is out of range")))?;

        let mut inner = self.inner.lock();
        if reason == p.custom_script_timeout {
            inner.custom_script_timeout = Duration::from_secs_f64(value.max(0.0));
            return Ok(());
        }
        if reason == p.jog_acceleration {
            inner.jog_acceleration = value;
            inner.new_jog = true;
            return Ok(());
        }

        if !inner.connected() {
            return Err(asyn_error("the RTDE control interface is not connected"));
        }

        if reason == p.joint_cmd {
            inner.cmd_joints[index] = deg_to_rad(value);
        } else if reason == p.pose_cmd {
            // x,y,z arrive in mm; rx,ry,rz in degrees.
            inner.cmd_pose[index] = if index >= 3 {
                deg_to_rad(value)
            } else {
                value / 1000.0
            };
        } else if reason == p.jog_speed {
            inner.jog_speeds[index] = if index >= 3 {
                deg_to_rad(value)
            } else {
                value / 1000.0
            };
            inner.new_jog = true;
        } else if reason == p.tcp_offset {
            // x,y,z arrive in mm; rx,ry,rz are already radians.
            inner.tcp_offset[index] = if index >= 3 { value } else { value / 1000.0 };
            let offset = inner.tcp_offset;
            let iface = inner
                .iface_mut()
                .map_err(|e| asyn_error(format!("setTcp failed: {e}")))?;
            iface
                .set_tcp(&offset)
                .map_err(|e| asyn_error(format!("setTcp failed: {e}")))?;
        } else if reason == p.joint_speed {
            inner.joint_speed = deg_to_rad(value);
        } else if reason == p.joint_acceleration {
            inner.joint_accel = deg_to_rad(value);
        } else if reason == p.joint_blend {
            inner.joint_blend = value / 1000.0;
        } else if reason == p.linear_speed {
            inner.linear_speed = value / 1000.0;
        } else if reason == p.linear_acceleration {
            inner.linear_accel = value / 1000.0;
        } else if reason == p.linear_blend {
            inner.linear_blend = value / 1000.0;
        }
        Ok(())
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let p = self.params;
        self.base.params.set_int32(reason, user.addr, value)?;

        if reason == p.reconnect {
            return if self.try_connect() {
                Ok(())
            } else {
                Err(asyn_error(
                    "could not connect to the RTDE control interface",
                ))
            };
        }

        if reason == p.run_custom_script_file {
            let path = self.inner.lock().custom_script_path.clone();
            let body = std::fs::read_to_string(&path)
                .map_err(|e| asyn_error(format!("could not read the URScript file {path}: {e}")));
            let body = match body {
                Ok(body) => body,
                Err(e) => {
                    self.base.params.set_int32(p.custom_script_error, 0, 1)?;
                    return Err(e);
                }
            };
            return self.run_custom_script(&body, &path);
        }

        let mut inner = self.inner.lock();

        if reason == p.disconnect {
            match inner.iface.as_mut() {
                Some(iface) => {
                    iface.disconnect();
                    return Ok(());
                }
                None => {
                    return Err(asyn_error("the RTDE control interface is not initialised"));
                }
            }
        }

        // These only stage state for a later command; they do not need the link.
        if reason == p.waypoint_move {
            inner.waypoint_move = value != 0;
            return Ok(());
        }
        if reason == p.waypoint_action_done {
            inner.waypoint_action_done = value != 0;
            return Ok(());
        }

        if !inner.connected() {
            return Err(asyn_error("the RTDE control interface is not connected"));
        }

        let mut updates: Vec<ParamSetValue> = Vec::new();
        let result: UrResult<()> = (|| {
            if reason == p.move_j || reason == p.move_l {
                let motion = if reason == p.move_j {
                    MotionType::Joint
                } else {
                    MotionType::Cartesian
                };
                if inner.pending_motion.is_some() {
                    log::warn!("ur-robot: a motion is already in progress; please wait");
                    return Ok(());
                }
                let target = match motion {
                    MotionType::Joint => inner.cmd_joints,
                    MotionType::Cartesian => inner.cmd_pose,
                };
                let iface = inner.iface_mut()?;
                let within = match motion {
                    MotionType::Joint => iface.is_joints_within_safety_limits(&target)?,
                    MotionType::Cartesian => iface.is_pose_within_safety_limits(&target)?,
                };
                if !within {
                    log::warn!(
                        "ur-robot: the requested target is not within the safety limits; \
                         no action taken"
                    );
                    return Ok(());
                }
                let action = inner.waypoint_move;
                inner.pending_motion = Some(MotionTask { motion, action });
                inner.waypoint_move = false;
                updates.push(ParamSetValue::new(
                    p.async_move_done,
                    0,
                    ParamValue::Int32(0),
                ));
                return Ok(());
            }

            if reason == p.stop_j {
                inner.motion_task_done(p, &mut updates);
                inner.iface_mut()?.stop_j(2.0, false)?;
            } else if reason == p.stop_l {
                inner.motion_task_done(p, &mut updates);
                inner.iface_mut()?.stop_l(10.0, false)?;
            } else if reason == p.reupload_control_script {
                inner.iface_mut()?.reupload_script()?;
            } else if reason == p.stop_control_script {
                inner.iface_mut()?.stop_script()?;
            } else if reason == p.trigger_prot_stop {
                inner.iface_mut()?.trigger_protective_stop()?;
            } else if reason == p.teach_mode {
                if value != 0 {
                    inner.iface_mut()?.teach_mode()?;
                } else {
                    inner.iface_mut()?.end_teach_mode()?;
                }
            } else if reason == p.jog_start {
                if inner.new_jog {
                    let speeds = inner.jog_speeds;
                    let accel = inner.jog_acceleration;
                    inner.iface_mut()?.speed_l(&speeds, accel, 0.01)?;
                    inner.new_jog = false;
                }
                updates.push(ParamSetValue::new(p.jogging, 0, ParamValue::Int32(1)));
            } else if reason == p.jog_stop {
                inner.iface_mut()?.speed_stop(10.0)?;
                updates.push(ParamSetValue::new(p.jogging, 0, ParamValue::Int32(0)));
            }
            Ok(())
        })();
        drop(inner);

        for u in updates {
            if let ParamSetValue::Value {
                reason,
                addr,
                value: ParamValue::Int32(value),
            } = u
            {
                self.base.params.set_int32(reason, addr, value)?;
            }
        }
        result.map_err(|e| asyn_error(format!("RTDE control command failed: {e}")))
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let p = self.params;
        let text = String::from_utf8_lossy(data)
            .trim_end_matches('\0')
            .to_string();
        self.base
            .params
            .set_string(reason, user.addr, text.clone())?;

        if reason == p.custom_script_file {
            return if std::path::Path::new(&text).is_file() {
                self.inner.lock().custom_script_path = text;
                self.base.params.set_int32(p.custom_script_error, 0, 0)?;
                Ok(data.len())
            } else {
                self.base.params.set_int32(p.custom_script_error, 0, 1)?;
                Err(asyn_error(format!("no such URScript file: {text}")))
            };
        }

        if reason == p.custom_inline_script {
            self.run_custom_script(&text, "(inline)")?;
            return Ok(data.len());
        }

        Ok(data.len())
    }
}

/// The control poll thread (`RTDEControl::poll`).
pub fn start_poller(
    handle: PortHandle,
    params: ControlParams,
    inner: Arc<Mutex<ControlInner>>,
    receive: ReceiveHandle,
    poll_period: Duration,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("ur-control-poll".into())
        .spawn(move || {
            loop {
                let updates = {
                    let mut guard = inner.lock();
                    poll_once(params, &mut guard, &receive)
                };
                let _ = handle.set_params_and_notify_blocking(0, updates);
                std::thread::sleep(poll_period);
            }
        })
        .expect("failed to spawn the RTDE control poll thread")
}

/// One pass of the control poll loop.
pub fn poll_once(
    p: ControlParams,
    inner: &mut ControlInner,
    receive: &ReceiveHandle,
) -> Vec<ParamSetValue> {
    let mut updates = Vec::new();

    if !inner.connected() {
        updates.push(ParamSetValue::new(p.is_connected, 0, ParamValue::Int32(0)));
        return updates;
    }
    updates.push(ParamSetValue::new(p.is_connected, 0, ParamValue::Int32(1)));

    let is_steady = if inner.custom_script.is_some() {
        false
    } else {
        match inner.iface_mut().and_then(|i| i.is_steady()) {
            Ok(v) => v,
            Err(e) => {
                log::error!("ur-robot: isSteady failed: {e}");
                false
            }
        }
    };
    updates.push(ParamSetValue::new(
        p.is_steady,
        0,
        ParamValue::Int32(i32::from(is_steady)),
    ));

    // A safety event aborts whatever motion is in flight.
    if receive.get().safety_status_bits != SAFETY_NORMAL {
        if inner.pending_motion.is_some() {
            log::info!("ur-robot: motion stopped by a safety event");
            inner.motion_task_done(p, &mut updates);
        }
        return updates;
    }

    if inner.pending_motion.is_some() {
        drive_motion(p, inner, receive, &mut updates);
    } else if inner.custom_script.is_some() {
        poll_custom_script(p, inner, receive, &mut updates);
    }
    updates
}

/// The asynchronous-motion state machine.
fn drive_motion(
    p: ControlParams,
    inner: &mut ControlInner,
    receive: &ReceiveHandle,
    updates: &mut Vec<ParamSetValue>,
) {
    let Some(task) = inner.pending_motion else {
        return;
    };

    match inner.motion_status {
        MotionStatus::Done => {
            // Start the motion the write handler queued.
            let (target, speed, accel) = match task.motion {
                MotionType::Joint => (inner.cmd_joints, inner.joint_speed, inner.joint_accel),
                MotionType::Cartesian => (inner.cmd_pose, inner.linear_speed, inner.linear_accel),
            };
            let started = inner.iface_mut().and_then(|i| match task.motion {
                MotionType::Joint => i.move_j(&target, speed, accel, true),
                MotionType::Cartesian => i.move_l(&target, speed, accel, true),
            });
            match started {
                Ok(_) => inner.motion_status = MotionStatus::WaitingMotion,
                Err(e) => {
                    log::error!("ur-robot: could not start the motion: {e}");
                    inner.motion_task_done(p, updates);
                }
            }
        }
        MotionStatus::WaitingMotion => {
            let progress = inner.iface_mut().and_then(|i| i.async_operation_progress());
            let status = match progress {
                Ok(s) => s,
                Err(e) => {
                    log::error!("ur-robot: could not read the async operation progress: {e}");
                    return;
                }
            };
            if status.is_running() {
                return;
            }
            if task.action {
                // Toggle so the action record always processes.
                inner.run_action_val ^= 1;
                inner.waypoint_action_done = false;
                updates.push(ParamSetValue::new(
                    p.waypoint_action_done,
                    0,
                    ParamValue::Int32(0),
                ));
                updates.push(ParamSetValue::new(
                    p.run_waypoint_action,
                    0,
                    ParamValue::Int32(inner.run_action_val),
                ));
                inner.motion_status = MotionStatus::WaitingAction;
            } else {
                inner.motion_task_done(p, updates);
            }
        }
        MotionStatus::WaitingAction => {
            if inner.custom_script.is_some() {
                poll_custom_script(p, inner, receive, updates);
            }
            if inner.waypoint_action_done {
                inner.motion_task_done(p, updates);
            }
        }
    }
}

/// `poll_custom_script()` — has the wrapped script bumped the counter, or has it
/// run out of time?
fn poll_custom_script(
    p: ControlParams,
    inner: &mut ControlInner,
    receive: &ReceiveHandle,
    updates: &mut Vec<ParamSetValue>,
) {
    let Some(script) = inner.custom_script else {
        return;
    };
    let count = receive.get().output_int_register_12;
    let finished = count != script.start_count;
    let timed_out = !finished && script.started.elapsed() >= script.timeout;

    if finished {
        inner.custom_script = None;
        updates.push(ParamSetValue::new(
            p.custom_script_running,
            0,
            ParamValue::Int32(0),
        ));
        // The custom script replaced the control script on the controller; put
        // the control script back.
        if let Err(e) = inner
            .iface_mut()
            .and_then(|i| i.reupload_script().map(|()| i))
            .and_then(|i| wait_for_program(i, REUPLOAD_TIMEOUT))
        {
            log::error!("ur-robot: could not reupload the control script: {e}");
        }
    } else if timed_out {
        inner.custom_script = None;
        log::error!(
            "ur-robot: the URScript timed out after {:.1} s",
            script.timeout.as_secs_f64()
        );
        updates.push(ParamSetValue::new(
            p.custom_script_running,
            0,
            ParamValue::Int32(0),
        ));
        updates.push(ParamSetValue::new(
            p.custom_script_error,
            0,
            ParamValue::Int32(1),
        ));
    }
}

fn wait_for_program(iface: &mut ControlInterface, timeout: Duration) -> UrResult<()> {
    let deadline = Instant::now() + timeout;
    loop {
        if iface.is_program_running()? {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(UrError::Timeout(
                timeout,
                "the control script did not restart".into(),
            ));
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> ControlParams {
        let mut n = 0usize;
        let mut next = || {
            n += 1;
            n
        };
        ControlParams {
            disconnect: next(),
            reconnect: next(),
            is_connected: next(),
            is_steady: next(),
            move_j: next(),
            stop_j: next(),
            actual_q: next(),
            joint_cmd: next(),
            move_l: next(),
            stop_l: next(),
            actual_tcp_pose: next(),
            pose_cmd: next(),
            tcp_offset: next(),
            reupload_control_script: next(),
            stop_control_script: next(),
            joint_speed: next(),
            joint_acceleration: next(),
            joint_blend: next(),
            linear_speed: next(),
            linear_acceleration: next(),
            linear_blend: next(),
            async_move_done: next(),
            waypoint_move: next(),
            run_waypoint_action: next(),
            waypoint_action_done: next(),
            teach_mode: next(),
            trigger_prot_stop: next(),
            motion_done_count: next(),
            custom_script_file: next(),
            custom_inline_script: next(),
            run_custom_script_file: next(),
            custom_script_running: next(),
            custom_script_error: next(),
            custom_script_timeout: next(),
            jog_start: next(),
            jog_stop: next(),
            jog_speed: next(),
            jog_acceleration: next(),
            jogging: next(),
        }
    }

    fn int_update(updates: &[ParamSetValue], want: usize) -> Option<i32> {
        updates.iter().find_map(|u| match u {
            ParamSetValue::Value {
                reason,
                value: ParamValue::Int32(value),
                ..
            } if *reason == want => Some(*value),
            _ => None,
        })
    }

    #[test]
    fn a_disconnected_interface_only_publishes_is_connected_zero() {
        let p = params();
        let mut inner = ControlInner::default();
        let receive = ReceiveHandle::new();
        let updates = poll_once(p, &mut inner, &receive);
        assert_eq!(updates.len(), 1);
        assert_eq!(int_update(&updates, p.is_connected), Some(0));
    }

    /// An `inner` whose only non-default state is a custom script in flight.
    fn inner_running_script(started: Instant, start_count: i32, timeout: Duration) -> ControlInner {
        ControlInner {
            custom_script: Some(CustomScript {
                started,
                start_count,
                timeout,
            }),
            ..Default::default()
        }
    }

    /// A receive handle reporting NORMAL safety and a script counter.
    fn receive_with_counter(count: i32) -> ReceiveHandle {
        let receive = ReceiveHandle::new();
        receive.set(registry::ReceiveState {
            connected: true,
            safety_status_bits: 1,
            output_int_register_12: count,
        });
        receive
    }

    #[test]
    fn motion_task_done_signals_the_move_and_bumps_the_counter() {
        let p = params();
        let mut inner = ControlInner {
            pending_motion: Some(MotionTask {
                motion: MotionType::Joint,
                action: false,
            }),
            motion_status: MotionStatus::WaitingMotion,
            ..Default::default()
        };

        let mut updates = Vec::new();
        inner.motion_task_done(p, &mut updates);

        assert!(inner.pending_motion.is_none());
        assert_eq!(inner.motion_status, MotionStatus::Done);
        assert_eq!(int_update(&updates, p.async_move_done), Some(1));
        assert_eq!(int_update(&updates, p.motion_done_count), Some(1));

        let mut updates = Vec::new();
        inner.motion_task_done(p, &mut updates);
        assert_eq!(int_update(&updates, p.motion_done_count), Some(2));
    }

    #[test]
    fn a_finished_custom_script_clears_running_when_the_counter_moves() {
        let p = params();
        let mut inner = inner_running_script(Instant::now(), 3, Duration::from_secs(60));
        let receive = receive_with_counter(4);

        let mut updates = Vec::new();
        poll_custom_script(p, &mut inner, &receive, &mut updates);

        assert!(inner.custom_script.is_none());
        assert_eq!(int_update(&updates, p.custom_script_running), Some(0));
        // No CUSTOM_SCRIPT_ERROR on a clean finish.
        assert_eq!(int_update(&updates, p.custom_script_error), None);
    }

    #[test]
    fn a_custom_script_that_never_finishes_times_out_and_raises_the_error_flag() {
        let p = params();
        let mut inner = inner_running_script(
            Instant::now() - Duration::from_secs(5),
            3,
            Duration::from_secs(1),
        );
        let receive = receive_with_counter(3);

        let mut updates = Vec::new();
        poll_custom_script(p, &mut inner, &receive, &mut updates);

        assert!(inner.custom_script.is_none());
        assert_eq!(int_update(&updates, p.custom_script_running), Some(0));
        assert_eq!(int_update(&updates, p.custom_script_error), Some(1));
    }

    #[test]
    fn a_running_custom_script_within_its_timeout_stays_running() {
        let p = params();
        let mut inner = inner_running_script(Instant::now(), 3, Duration::from_secs(60));
        let receive = receive_with_counter(3);

        let mut updates = Vec::new();
        poll_custom_script(p, &mut inner, &receive, &mut updates);
        assert!(inner.custom_script.is_some());
        assert!(updates.is_empty());
    }

    #[test]
    fn degrees_convert_to_radians() {
        assert!((deg_to_rad(180.0) - std::f64::consts::PI).abs() < 1e-12);
        assert!((deg_to_rad(-90.0) + std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }
}
