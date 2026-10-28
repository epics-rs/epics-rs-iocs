//! `RTDEControl` — asyn port driver for robot motion.
//!
//! Port of `urRobotApp/src/rtde_control_driver.cpp`. MAX_ADDR = 6 (one address
//! per joint / pose element). The poll thread drives the asynchronous-motion
//! state machine and watches for a custom URScript finishing.

use std::sync::Arc;
use std::time::{Duration, Instant};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;
use parking_lot::Mutex;

use crate::control::ControlInterface;
use crate::drivers::ioc_ready::IocReady;
use crate::drivers::{asyn_error, refused};
use crate::error::{UrError, UrResult};
use crate::registry::{self, DashboardHandle, ReceiveHandle};
use crate::script::{SCRIPT_PORT, ScriptClient, wrap_custom_script};
use crate::session::DEFAULT_TIMEOUT;
use crate::spdlog::{self, FmtDouble};

pub const NUM_JOINTS: usize = 6;

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
    pub(crate) fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
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

    /// The momentary commands of this port — see the
    /// [module docs](crate::drivers#momentary-commands).
    ///
    /// `TEACH_MODE`, `WAYPOINT_MOVE` and `WAYPOINT_ACTION_DONE` are absent:
    /// each reads its own value, so zero means "leave teach mode" / "no
    /// action" rather than "no request".
    fn is_command(&self, reason: usize) -> bool {
        reason == self.reconnect
            || reason == self.disconnect
            || reason == self.run_custom_script_file
            || reason == self.move_j
            || reason == self.move_l
            || reason == self.stop_j
            || reason == self.stop_l
            || reason == self.reupload_control_script
            || reason == self.stop_control_script
            || reason == self.trigger_prot_stop
            || reason == self.jog_start
            || reason == self.jog_stop
    }

    /// The device readbacks that carry the COMM alarm while the control link
    /// is down — on this port that is `IS_STEADY` alone. Everything else is
    /// a command, an operator setpoint/mode, IOC-side state-machine output
    /// that stays valid through an outage (`ASYNC_MOVE_DONE`,
    /// `MOTION_DONE_COUNT`, the waypoint/custom-script/jog state), the
    /// health readback `IS_CONNECTED` (must stay valid at 0 — it is the one
    /// PV that says why), or the two unpublished upstream-parity params
    /// (`ACTUAL_Q`/`ACTUAL_TCP_POSE`, which no record binds).
    /// `alarm_set_is_every_device_readback` enforces that a new parameter
    /// lands in exactly one group.
    fn alarm_targets(&self) -> Vec<(usize, i32)> {
        vec![(self.is_steady, 0)]
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
    /// TCP offset: x,y,z metres, rx,ry,rz radians. `None` until a
    /// `TCP_OFFSET` write stages one — only then may a (re)connect deliver
    /// it, so an IOC that never wrote an offset leaves the robot's own
    /// setting alone.
    pub tcp_offset: Option<[f64; NUM_JOINTS]>,
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
            tcp_offset: None,
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

    /// Deliver the staged TCP offset once the link is up (B3). PINI
    /// processes the six `TCPOffset_*` records at boot, before any connect
    /// can have succeeded, so the offset is carried here until the
    /// connection owner can apply it; a failure leaves it staged for the
    /// next reconnect.
    fn apply_cached_settings(&mut self) {
        let Some(offset) = self.tcp_offset else {
            return;
        };
        let _ = self.iface_mut().and_then(|i| i.set_tcp(&offset));
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
        // Release the moveJ/moveL busy records (upstream 558b98f,
        // rtde_control_driver.hpp set_motion_task_done). Both are cleared —
        // only one can be in flight, and the other is already 0.
        out.push(ParamSetValue::new(p.move_j, 0, ParamValue::Int32(0)));
        out.push(ParamSetValue::new(p.move_l, 0, ParamValue::Int32(0)));
    }
}

pub struct ControlDriver {
    base: PortDriverBase,
    params: ControlParams,
    inner: Arc<Mutex<ControlInner>>,
    robot_ip: String,
    dashboard: DashboardHandle,
    receive: ReceiveHandle,
    /// C's `script_client_`, connected by the constructor and used only for
    /// custom scripts (rtde_control_driver.cpp:226-227,536-552).
    script: ScriptClient,
}

impl ControlDriver {
    /// `dashboard_port` supplies the robot IP and the RUNNING check;
    /// `receive_port` supplies the safety word and the custom-script counter.
    ///
    /// Fails, as C's constructor throws, when the dashboard never published a
    /// PolyScope version or the script server refuses the connection.
    pub fn new(port_name: &str, dashboard_port: &str, receive_port: &str) -> AsynResult<Self> {
        Self::with_script_port(port_name, dashboard_port, receive_port, SCRIPT_PORT)
    }

    fn with_script_port(
        port_name: &str,
        dashboard_port: &str,
        receive_port: &str,
        script_port: u16,
    ) -> AsynResult<Self> {
        let dashboard = registry::dashboard(dashboard_port).ok_or_else(|| {
            asyn_error(format!(
                "no URDashboard port named {dashboard_port}; configure it before the control port"
            ))
        })?;
        if dashboard.get().polyscope.is_none() {
            return Err(asyn_error("Failed to parse Polyscope version"));
        }
        let mut script = ScriptClient::new(&dashboard.ip, script_port, DEFAULT_TIMEOUT);
        script.connect().map_err(|e| asyn_error(e.to_string()))?;
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
            InterfaceMask::INT32
                | InterfaceMask::FLOAT64
                | InterfaceMask::OCTET
                | InterfaceMask::FLOAT64_ARRAY
                | InterfaceMask::DRV_USER,
            InterfaceMask::INT32
                | InterfaceMask::FLOAT64
                | InterfaceMask::OCTET
                | InterfaceMask::FLOAT64_ARRAY,
        );
        let params = ControlParams::create(&mut base)?;

        let me = Self {
            base,
            params,
            inner: Arc::new(Mutex::new(ControlInner::default())),
            robot_ip: dashboard.ip.clone(),
            dashboard,
            receive,
            script,
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
        if !receive.safety_normal() {
            spdlog::error(format_args!(
                "Cannot connect to control interface in current safety state. \
                 Clear safeguard stop, E-stop, etc. then try again.\n"
            ));
            return false;
        }
        // An undefined mode fails C's readOnce, which returns silently.
        let Some(running) = self.dashboard.get().robot_running() else {
            return false;
        };
        if !running {
            spdlog::error(format_args!(
                "Unable to connect to control interface in current mode. \
                 Ensure robot is on, in normal mode, and brakes released, then try again.\n"
            ));
            return false;
        }

        let mut inner = self.inner.lock();
        match inner.iface.as_mut() {
            Some(iface) => {
                if iface.is_connected() {
                    return true;
                }
                spdlog::debug(format_args!("Reconnecting to UR RTDE Control interface"));
                match iface.reconnect() {
                    Ok(()) => {
                        inner.apply_cached_settings();
                        true
                    }
                    Err(e) => {
                        spdlog::error(format_args!("Failed to reconnect: {e}"));
                        false
                    }
                }
            }
            None => match ControlInterface::connect(&self.robot_ip, false) {
                Ok(iface) => {
                    spdlog::info(format_args!("Connected to UR RTDE Control interface"));
                    inner.iface = Some(iface);
                    inner.apply_cached_settings();
                    true
                }
                Err(e) => {
                    spdlog::error(format_args!(
                        "Failed to connected to UR RTDE Control interface\n{e}"
                    ));
                    false
                }
            },
        }
    }

    /// Launch a custom URScript (from a file or inline) on the controller.
    ///
    /// `body` is read only after the motion and script-client checks, and a
    /// `None` (an unopenable file) is skipped silently; every skip succeeds,
    /// as C's `goto skip` with `comm_ok` still true
    /// (rtde_control_driver.cpp:531-545).
    fn run_custom_script(
        &mut self,
        source: &str,
        body: impl FnOnce() -> Option<String>,
    ) -> AsynResult<()> {
        let p = self.params;
        let mut inner = self.inner.lock();
        if inner.pending_motion.is_some() && inner.motion_status != MotionStatus::WaitingAction {
            spdlog::warn(format_args!("Motion task in progress. Cannot run script."));
            return Ok(());
        }
        if !self.script.is_connected() {
            spdlog::error(format_args!("ScriptClient not connected!"));
            return Ok(());
        }
        let Some(body) = body() else {
            return Ok(());
        };
        let body = body.as_str();
        let start_count = self.receive.get().output_int_register_12;
        let timeout = inner.custom_script_timeout;

        match source {
            "(inline)" => spdlog::debug(format_args!("Running inline URScript: {body}")),
            path => spdlog::debug(format_args!("Running custom URScript: {path}")),
        }
        let iface = inner.iface_mut().map_err(|_| refused())?;
        iface
            .stop_script()
            .and_then(|()| self.script.send(&wrap_custom_script(body)))
            .map_err(|e| {
                spdlog::error(format_args!("{e}"));
                refused()
            })?;

        inner.custom_script = Some(CustomScript {
            started: Instant::now(),
            start_count,
            timeout,
        });
        drop(inner);

        self.base.set_int32_param(p.custom_script_error, 0, 0)?;
        self.base.set_int32_param(p.custom_script_running, 0, 1)?;
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
        // C resolves the joint index via getAddress (rtde_control_driver.cpp:330).
        let addr = crate::drivers::device_addr(user.addr);
        let result: AsynResult<()> = (|| {
            let reason = user.reason;
            let p = self.params;
            self.base.set_float64_param(reason, addr, value)?;

            // Only the four per-axis parameters take one address per joint; a
            // scalar parameter written at addr 1..5 is a miswired record, not a
            // joint index.
            let per_axis = reason == p.joint_cmd
                || reason == p.pose_cmd
                || reason == p.jog_speed
                || reason == p.tcp_offset;
            let limit = if per_axis { NUM_JOINTS } else { 1 };
            let index = usize::try_from(addr)
                .ok()
                .filter(|i| *i < limit)
                .ok_or_else(|| {
                    spdlog::error(format_args!("address {addr} is out of range"));
                    refused()
                })?;

            // Every arm below stages IOC-local state, so none of them needs the
            // link up — PINI processes these records at boot, before any connect
            // can have succeeded. TCP_OFFSET is the one that also reaches the
            // device: connected, it applies now; disconnected, the staged offset
            // waits for try_connect (C refuses the write and the offset is lost).
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

            if reason == p.joint_cmd {
                inner.cmd_joints[index] = deg_to_rad(value);
            } else if reason == p.pose_cmd {
                // x,y,z arrive in mm; rx,ry,rz are a rotation vector already in
                // radians (upstream c02490e) — no angular scaling applies.
                inner.cmd_pose[index] = if index >= 3 { value } else { value / 1000.0 };
            } else if reason == p.jog_speed {
                // x,y,z arrive in mm/s; rx,ry,rz in rad/s.
                inner.jog_speeds[index] = if index >= 3 { value } else { value / 1000.0 };
                spdlog::debug(format_args!(
                    "Setting jog speed[{index}] to {:.4}",
                    CFloat(inner.jog_speeds[index])
                ));
                inner.new_jog = true;
            } else if reason == p.tcp_offset {
                // x,y,z arrive in mm; rx,ry,rz are already radians.
                let offset = inner.tcp_offset.get_or_insert([0.0; NUM_JOINTS]);
                offset[index] = if index >= 3 { value } else { value / 1000.0 };
                let offset = *offset;
                spdlog::debug(format_args!(
                    "Setting TCP offset to [{}] m,rad",
                    offset.map(|v| format!("{v:.4}", v = CFloat(v))).join(",")
                ));
                if inner.connected() {
                    inner
                        .iface_mut()
                        .and_then(|i| i.set_tcp(&offset))
                        .map_err(|e| {
                            spdlog::error(format_args!("{e}"));
                            refused()
                        })?;
                }
            } else if reason == p.joint_speed {
                inner.joint_speed = deg_to_rad(value);
                spdlog::debug(format_args!(
                    "Setting joint speed to {:.4}",
                    CFloat(inner.joint_speed)
                ));
            } else if reason == p.joint_acceleration {
                inner.joint_accel = deg_to_rad(value);
                spdlog::debug(format_args!(
                    "Setting joint acceleration to {:.4}",
                    CFloat(inner.joint_accel)
                ));
            } else if reason == p.joint_blend {
                inner.joint_blend = value / 1000.0;
                spdlog::debug(format_args!(
                    "Setting joint blend to {:.4}",
                    CFloat(inner.joint_blend)
                ));
            } else if reason == p.linear_speed {
                inner.linear_speed = value / 1000.0;
                spdlog::debug(format_args!(
                    "Setting linear speed to {:.4}",
                    CFloat(inner.linear_speed)
                ));
            } else if reason == p.linear_acceleration {
                inner.linear_accel = value / 1000.0;
                spdlog::debug(format_args!(
                    "Setting linear acceleration to {:.4}",
                    CFloat(inner.linear_accel)
                ));
            } else if reason == p.linear_blend {
                inner.linear_blend = value / 1000.0;
                spdlog::debug(format_args!(
                    "Setting linear blend to {:.4}",
                    CFloat(inner.linear_blend)
                ));
            }
            Ok(())
        })();
        crate::drivers::flush_after(&mut self.base, addr, result)
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let result: AsynResult<()> = (|| {
            let reason = user.reason;
            let p = self.params;
            self.base.set_int32_param(reason, user.addr, value)?;

            if p.is_command(reason) && value == 0 {
                return Ok(());
            }

            if reason == p.reconnect {
                return if self.try_connect() {
                    Ok(())
                } else {
                    Err(refused())
                };
            }

            if reason == p.run_custom_script_file {
                let path = self.inner.lock().custom_script_path.clone();
                return self
                    .run_custom_script(&path.clone(), || std::fs::read_to_string(&path).ok());
            }

            let mut inner = self.inner.lock();

            if reason == p.disconnect {
                match inner.iface.as_mut() {
                    Some(iface) => {
                        spdlog::debug(format_args!("Disconnecting from RTDE control interface"));
                        iface.disconnect();
                        return Ok(());
                    }
                    None => {
                        spdlog::error(format_args!("RTDE Control interface not initialized"));
                        return Err(refused());
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
                print_unusable(&inner);
                return Err(refused());
            }

            let mut updates: Vec<ParamSetValue> = Vec::new();
            let result: UrResult<()> = (|| {
                if reason == p.move_j || reason == p.move_l {
                    let motion = if reason == p.move_j {
                        MotionType::Joint
                    } else {
                        MotionType::Cartesian
                    };
                    // The refused/accepted writes below feed the moveJ/moveL busy
                    // records: 1 on an accepted request, 0 on a refusal so the
                    // record releases immediately (upstream 558b98f).
                    if inner.pending_motion.is_some() {
                        spdlog::warn(format_args!("Motion already in progress...please wait"));
                        updates.push(ParamSetValue::new(reason, 0, ParamValue::Int32(0)));
                        return Ok(());
                    }
                    let target = match motion {
                        MotionType::Joint => inner.cmd_joints,
                        MotionType::Cartesian => inner.cmd_pose,
                    };
                    let joined = target.map(|v| format!("{v:.4}", v = CFloat(v))).join(",");
                    match motion {
                        MotionType::Joint => spdlog::debug(format_args!("moveJ({joined}) rad")),
                        MotionType::Cartesian => {
                            spdlog::debug(format_args!("moveL({joined}) m,rad"))
                        }
                    }
                    let iface = inner.iface_mut()?;
                    let within = match motion {
                        MotionType::Joint => iface.is_joints_within_safety_limits(&target),
                        MotionType::Cartesian => iface.is_pose_within_safety_limits(&target),
                    };
                    let within = match within {
                        // A refused or failed query must release the busy record
                        // before the error propagates, or the record stays busy
                        // for a motion that was never queued.
                        Err(e) => {
                            updates.push(ParamSetValue::new(reason, 0, ParamValue::Int32(0)));
                            return Err(e);
                        }
                        Ok(w) => w,
                    };
                    if !within {
                        spdlog::warn(format_args!(
                            "{}",
                            match motion {
                                MotionType::Joint => {
                                    "Requested joint angles not within safety limits. No action taken."
                                }
                                MotionType::Cartesian => {
                                    "Requested TCP pose not within safety limits. No action taken."
                                }
                            }
                        ));
                        updates.push(ParamSetValue::new(reason, 0, ParamValue::Int32(0)));
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
                    updates.push(ParamSetValue::new(reason, 0, ParamValue::Int32(1)));
                    return Ok(());
                }

                if reason == p.stop_j {
                    inner.motion_task_done(p, &mut updates);
                    spdlog::debug(format_args!("Stopping (linear in joint space)"));
                    inner.iface_mut()?.stop_j(2.0, false)?;
                } else if reason == p.stop_l {
                    inner.motion_task_done(p, &mut updates);
                    spdlog::debug(format_args!("Stopping (linear in tool space)"));
                    inner.iface_mut()?.stop_l(10.0, false)?;
                } else if reason == p.reupload_control_script {
                    // C catches, prints and succeeds (rtde_control_driver.cpp:497-504).
                    spdlog::debug(format_args!("Reuploading control script"));
                    if let Err(e) = inner.iface_mut()?.reupload_script() {
                        spdlog::error(format_args!("Failed to reupload control script: {e}"));
                    }
                } else if reason == p.stop_control_script {
                    spdlog::debug(format_args!("Stopping control script"));
                    if let Err(e) = inner.iface_mut()?.stop_script() {
                        spdlog::error(format_args!("Failed to stop control script: {e}"));
                    }
                } else if reason == p.trigger_prot_stop {
                    spdlog::debug(format_args!("Triggering protective stop"));
                    inner.iface_mut()?.trigger_protective_stop()?;
                } else if reason == p.teach_mode {
                    if value != 0 {
                        spdlog::debug(format_args!("Enabling teach mode"));
                        inner.iface_mut()?.teach_mode()?;
                    } else {
                        spdlog::debug(format_args!("Disabling teach mode"));
                        inner.iface_mut()?.end_teach_mode()?;
                    }
                } else if reason == p.jog_start {
                    if inner.new_jog {
                        let speeds = inner.jog_speeds;
                        let accel = inner.jog_acceleration;
                        inner.iface_mut()?.speed_l(&speeds, accel, 0.01)?;
                        spdlog::debug(format_args!(
                            "Starting jog: accel={}, speeds=[{}]",
                            FmtDouble(accel),
                            speeds.map(|v| format!("{v:.2}", v = CFloat(v))).join(",")
                        ));
                        inner.new_jog = false;
                    }
                    updates.push(ParamSetValue::new(p.jogging, 0, ParamValue::Int32(1)));
                } else if reason == p.jog_stop {
                    spdlog::debug(format_args!("Stopping jog"));
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
                    self.base.set_int32_param(reason, addr, value)?;
                }
            }
            result.map_err(|e| {
                spdlog::error(format_args!("{e}"));
                refused()
            })
        })();
        if result.is_err() {
            spdlog::debug(format_args!(
                "RTDE communication error in RTDEControl::writeInt32"
            ));
        }
        crate::drivers::flush_after(&mut self.base, user.addr, result)
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let result: AsynResult<usize> = (|| {
            let reason = user.reason;
            let p = self.params;
            let text = String::from_utf8_lossy(data)
                .trim_end_matches('\0')
                .to_string();
            self.base
                .set_string_param(reason, user.addr, text.clone())?;

            // A missing file prints and succeeds, as C's writeOctet does
            // (rtde_control_driver.cpp:611-619).
            if reason == p.custom_script_file {
                if std::path::Path::new(&text).exists() {
                    spdlog::debug(format_args!("Successfully read URScript file: {text}"));
                    self.inner.lock().custom_script_path = text;
                    self.base.set_int32_param(p.custom_script_error, 0, 0)?;
                } else {
                    spdlog::error(format_args!("Failed to read URScript file: {text}"));
                    self.base.set_int32_param(p.custom_script_error, 0, 1)?;
                }
                return Ok(data.len());
            }

            if reason == p.custom_inline_script {
                self.run_custom_script("(inline)", || Some(text.clone()))?;
                return Ok(data.len());
            }

            Ok(data.len())
        })();
        if result.is_err() {
            spdlog::debug(format_args!(
                "RTDE communication error in RTDEControl::writeOctet"
            ));
        }
        crate::drivers::flush_after(&mut self.base, user.addr, result)
    }
}

/// The control poll thread (`RTDEControl::poll`).
pub fn start_poller(
    handle: PortHandle,
    params: ControlParams,
    inner: Arc<Mutex<ControlInner>>,
    receive: ReceiveHandle,
    poll_period: Duration,
    ready: Arc<IocReady>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("ur-control-poll".into())
        .spawn(move || {
            let alarm_targets = params.alarm_targets();
            // Starts true so an IOC that boots with the robot down raises
            // the COMM alarm on its first cycle.
            let mut was_healthy = true;
            loop {
                let (mut updates, healthy_now) = {
                    let mut guard = inner.lock();
                    poll_once(params, &mut guard, &receive)
                };
                if ready.is_open() {
                    updates.extend(crate::drivers::health_transition(
                        &alarm_targets,
                        healthy_now,
                        &mut was_healthy,
                    ));
                    let _ = handle.set_params_and_notify_blocking(0, updates);
                }
                std::thread::sleep(poll_period);
            }
        })
        .expect("failed to spawn the RTDE control poll thread")
}

/// One pass of the control poll loop: the parameter updates, and whether the
/// link is healthy for the COMM alarm on `ControlParams::alarm_targets`.
pub fn poll_once(
    p: ControlParams,
    inner: &mut ControlInner,
    receive: &ReceiveHandle,
) -> (Vec<ParamSetValue>, bool) {
    let mut updates = Vec::new();

    if !inner.connected() {
        updates.push(ParamSetValue::new(p.is_connected, 0, ParamValue::Int32(0)));
        return (updates, false);
    }
    // IS_CONNECTED reflects aliveness, not just the socket: a stale stream
    // (open socket, packages stopped) reads 0. Unlike receive, control never
    // reconnects on its own — a reconnect re-uploads the control script — so
    // staleness here is reflection only; the write gate stays socket-based.
    let alive = inner
        .iface
        .as_ref()
        .is_some_and(|i| i.is_alive(crate::drivers::STALE_AFTER));
    updates.push(ParamSetValue::new(
        p.is_connected,
        0,
        ParamValue::Int32(i32::from(alive)),
    ));

    let is_steady = if inner.custom_script.is_some() {
        false
    } else {
        inner
            .iface_mut()
            .and_then(|i| i.is_steady())
            .unwrap_or(false)
    };
    updates.push(ParamSetValue::new(
        p.is_steady,
        0,
        ParamValue::Int32(i32::from(is_steady)),
    ));

    // A safety event aborts whatever motion is in flight.
    if !receive.get().safety_normal() {
        if inner.pending_motion.is_some() {
            spdlog::debug(format_args!("Motion stopped due to safety."));
            inner.motion_task_done(p, &mut updates);
        }
        return (updates, alive);
    }

    if inner.pending_motion.is_some() {
        drive_motion(p, inner, receive, &mut updates);
    } else if inner.custom_script.is_some() {
        poll_custom_script(p, inner, receive, &mut updates);
    }
    (updates, alive)
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
                Err(_) => inner.motion_task_done(p, updates),
            }
        }
        MotionStatus::WaitingMotion => {
            let progress = inner.iface_mut().and_then(|i| i.async_operation_progress());
            let Ok(status) = progress else {
                return;
            };
            if status.is_running() {
                return;
            }
            if task.action {
                spdlog::debug(format_args!("Waypoint reached. Starting action..."));
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
                spdlog::debug(format_args!("Motion complete."));
                inner.motion_task_done(p, updates);
            }
        }
        MotionStatus::WaitingAction => {
            if inner.custom_script.is_some() {
                poll_custom_script(p, inner, receive, updates);
            }
            if inner.waypoint_action_done {
                spdlog::debug(format_args!("Waypoint action complete."));
                inner.motion_task_done(p, updates);
            }
        }
    }
}

/// C's two refusals for a write with no usable control interface
/// (rtde_control_driver.cpp:422-439).
fn print_unusable(inner: &ControlInner) {
    if inner.iface.is_none() {
        spdlog::error(format_args!("RTDE Control interface not initialized"));
    } else {
        spdlog::error(format_args!("RTDE Control interface not connected"));
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
        spdlog::debug(format_args!("URScript done"));
        spdlog::debug(format_args!("Reuploading RTDE control script"));
        if let Err(UrError::Timeout(..)) = inner
            .iface_mut()
            .and_then(|i| i.reupload_script().map(|()| i))
            .and_then(|i| wait_for_program(i, REUPLOAD_TIMEOUT))
        {
            spdlog::error(format_args!("Timed out trying to reupload control script"));
        }
    } else if timed_out {
        inner.custom_script = None;
        spdlog::error(format_args!(
            "URScript timed out: {}",
            inner.custom_script_path
        ));
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
    use crate::dashboard::PolyScopeVersion;
    use crate::registry::DashboardState;

    /// Privileged port 1: no test can bind it, so a connect is always refused.
    /// A freed ephemeral port can be taken by a concurrent test's listener.
    const DEAD_PORT: u16 = 1;

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

        let (updates, healthy) = poll_once(p, &mut inner, &receive);
        assert!(!healthy);
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
            safety_status_bits: Some(1),
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
        // The moveJ/moveL busy records release on completion (558b98f).
        assert_eq!(int_update(&updates, p.move_j), Some(0));
        assert_eq!(int_update(&updates, p.move_l), Some(0));

        let mut updates = Vec::new();
        inner.motion_task_done(p, &mut updates);
        assert_eq!(int_update(&updates, p.motion_done_count), Some(2));
    }

    /// The boot race B3 closes for the control port: PINI writes the six
    /// `TCPOffset_*` records (and every dynamics setting) at iocsh time,
    /// before any connect can have succeeded. The writes stage IOC-local
    /// state and succeed; the staged offset survives a delivery attempt
    /// without a link, so the next reconnect can retry it.
    #[test]
    fn a_boot_time_tcp_offset_stages_until_a_connect_can_deliver_it() {
        let dash = DashboardHandle::new("127.0.0.1");
        dash.set(DashboardState {
            polyscope: Some(PolyScopeVersion::default()),
            ..Default::default()
        });
        registry::register_dashboard("CTRL_B3_DASH", dash).expect("fresh port name");
        registry::register_receive("CTRL_B3_RECV", ReceiveHandle::new()).expect("fresh port name");
        let script_server = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        let script_port = script_server.local_addr().expect("addr").port();
        let mut drv =
            ControlDriver::with_script_port("CTRL_B3", "CTRL_B3_DASH", "CTRL_B3_RECV", script_port)
                .expect("driver");
        let p = drv.params();

        // Never written: nothing staged, and a connect must leave the
        // robot's own offset alone.
        assert_eq!(drv.inner().lock().tcp_offset, None);

        for (addr, value) in [
            (0, 1000.0),
            (1, 2000.0),
            (2, 3000.0),
            (3, 0.1),
            (4, 0.2),
            (5, 0.3),
        ] {
            drv.write_float64(&mut AsynUser::new(p.tcp_offset).with_addr(addr), value)
                .unwrap();
        }
        drv.write_float64(&mut AsynUser::new(p.joint_speed), 30.0)
            .unwrap();

        let inner = drv.inner();
        assert_eq!(
            inner.lock().tcp_offset,
            Some([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        );
        assert!((inner.lock().joint_speed - deg_to_rad(30.0)).abs() < 1e-12);

        // A delivery attempt without a link keeps the offset staged.
        inner.lock().apply_cached_settings();
        assert_eq!(
            inner.lock().tcp_offset,
            Some([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        );
    }

    /// Scalar float parameters take addr 0 only; the per-axis parameters
    /// take one address per joint (#27: a scalar write at addr 1..5 must
    /// fail, not silently pass because 1 < NUM_JOINTS).
    #[test]
    fn a_scalar_float_write_takes_address_zero_only() {
        let dash = DashboardHandle::new("127.0.0.1");
        dash.set(DashboardState {
            polyscope: Some(PolyScopeVersion::default()),
            ..Default::default()
        });
        registry::register_dashboard("CTRL_B3_ADDR_DASH", dash).expect("fresh port name");
        registry::register_receive("CTRL_B3_ADDR_RECV", ReceiveHandle::new())
            .expect("fresh port name");
        let script_server = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        let script_port = script_server.local_addr().expect("addr").port();
        let mut drv = ControlDriver::with_script_port(
            "CTRL_B3_ADDR",
            "CTRL_B3_ADDR_DASH",
            "CTRL_B3_ADDR_RECV",
            script_port,
        )
        .expect("driver");
        let p = drv.params();

        assert!(
            drv.write_float64(&mut AsynUser::new(p.joint_speed).with_addr(1), 30.0)
                .is_err()
        );
        assert!(
            drv.write_float64(&mut AsynUser::new(p.tcp_offset).with_addr(5), 0.1)
                .is_ok()
        );
        assert!(
            drv.write_float64(&mut AsynUser::new(p.tcp_offset).with_addr(6), 0.1)
                .is_err()
        );
    }

    /// C's constructor throws before the port is usable when the dashboard
    /// never published a version or the script server refuses
    /// (rtde_control_driver.cpp:96,227); the Config command prints `what()`.
    #[test]
    fn construction_fails_as_the_c_constructor_throws() {
        let message = |r: AsynResult<ControlDriver>| match r {
            Err(epics_rs::asyn::error::AsynError::Status { message, .. }) => message,
            Err(e) => panic!("unexpected error {e}"),
            Ok(_) => panic!("construction must fail"),
        };
        registry::register_receive("CTRL_THROW_RECV", ReceiveHandle::new())
            .expect("fresh port name");

        let unversioned = DashboardHandle::new("127.0.0.1");
        registry::register_dashboard("CTRL_THROW_DASH0", unversioned).expect("fresh port name");
        assert_eq!(
            message(ControlDriver::new(
                "CTRL_THROW0",
                "CTRL_THROW_DASH0",
                "CTRL_THROW_RECV"
            )),
            "Failed to parse Polyscope version"
        );

        let versioned = DashboardHandle::new("127.0.0.1");
        versioned.set(DashboardState {
            polyscope: Some(PolyScopeVersion::default()),
            ..Default::default()
        });
        registry::register_dashboard("CTRL_THROW_DASH1", versioned).expect("fresh port name");
        let refused = message(ControlDriver::with_script_port(
            "CTRL_THROW1",
            "CTRL_THROW_DASH1",
            "CTRL_THROW_RECV",
            DEAD_PORT,
        ));
        assert!(refused.starts_with("connect: "), "{refused}");
    }

    /// A move whose start returns `Err` — a `CommandRefused` and a transport
    /// failure take the same arm — must finish the task, not enter
    /// `WaitingMotion`. Before the typed refusal, a refused start was
    /// `Ok(false)` and the state machine waited on a motion the script never
    /// ran.
    #[test]
    fn a_move_that_cannot_start_never_enters_waiting_motion() {
        let p = params();
        let mut inner = ControlInner {
            pending_motion: Some(MotionTask {
                motion: MotionType::Joint,
                action: false,
            }),
            ..Default::default()
        };
        let receive = ReceiveHandle::new();

        let mut updates = Vec::new();
        drive_motion(p, &mut inner, &receive, &mut updates);

        assert!(inner.pending_motion.is_none());
        assert_eq!(inner.motion_status, MotionStatus::Done);
        assert_eq!(int_update(&updates, p.move_j), Some(0));
        assert_eq!(int_update(&updates, p.async_move_done), Some(1));
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

    /// `TEACH_MODE` is the one that would be tempting to lump in with the
    /// commands: it is a bo like the rest. It reads its value — zero ends
    /// teach mode — so classifying it as a command would silently drop
    /// `caput Control:TeachMode 0` and strand the robot in teach mode.
    #[test]
    fn the_twelve_commands_are_commands_and_the_valued_writes_are_not() {
        let mut base = PortDriverBase::new(
            "ctrl_is_command",
            NUM_JOINTS,
            PortFlags::default(),
            InterfaceMask::INT32
                | InterfaceMask::FLOAT64
                | InterfaceMask::OCTET
                | InterfaceMask::FLOAT64_ARRAY
                | InterfaceMask::DRV_USER,
            InterfaceMask::INT32
                | InterfaceMask::FLOAT64
                | InterfaceMask::OCTET
                | InterfaceMask::FLOAT64_ARRAY,
        );
        let p = ControlParams::create(&mut base).expect("params create");

        for reason in [
            p.reconnect,
            p.disconnect,
            p.run_custom_script_file,
            p.move_j,
            p.move_l,
            p.stop_j,
            p.stop_l,
            p.reupload_control_script,
            p.stop_control_script,
            p.trigger_prot_stop,
            p.jog_start,
            p.jog_stop,
        ] {
            assert!(p.is_command(reason));
        }
        for reason in [
            p.teach_mode,
            p.waypoint_move,
            p.waypoint_action_done,
            p.joint_cmd,
            p.pose_cmd,
            p.jog_speed,
            p.jog_acceleration,
            p.tcp_offset,
        ] {
            assert!(!p.is_command(reason));
        }
    }

    /// Completeness of the alarm set: every parameter this driver creates is
    /// a command, an alarm target, or one of the deliberate exclusions — a
    /// parameter added without classification fails here instead of silently
    /// staying NO_ALARM through an outage.
    #[test]
    fn alarm_set_is_every_device_readback() {
        let mut base = PortDriverBase::new(
            "ctrl_alarm_set",
            NUM_JOINTS,
            PortFlags::default(),
            InterfaceMask::INT32
                | InterfaceMask::FLOAT64
                | InterfaceMask::OCTET
                | InterfaceMask::FLOAT64_ARRAY
                | InterfaceMask::DRV_USER,
            InterfaceMask::INT32
                | InterfaceMask::FLOAT64
                | InterfaceMask::OCTET
                | InterfaceMask::FLOAT64_ARRAY,
        );
        let p = ControlParams::create(&mut base).expect("params create");

        let targets = p.alarm_targets();
        let excluded = [
            // Health readback.
            p.is_connected,
            // Operator setpoints and modes.
            p.joint_cmd,
            p.pose_cmd,
            p.tcp_offset,
            p.joint_speed,
            p.joint_acceleration,
            p.joint_blend,
            p.linear_speed,
            p.linear_acceleration,
            p.linear_blend,
            p.teach_mode,
            p.waypoint_move,
            p.custom_script_file,
            p.custom_inline_script,
            p.custom_script_timeout,
            p.jog_speed,
            p.jog_acceleration,
            // IOC-side state-machine outputs: valid through an outage.
            p.async_move_done,
            p.run_waypoint_action,
            p.waypoint_action_done,
            p.motion_done_count,
            p.custom_script_running,
            p.custom_script_error,
            p.jogging,
            // Unpublished upstream-parity params; no record binds them.
            p.actual_q,
            p.actual_tcp_pose,
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
}
