//! `RTDEReceive` — asyn port driver for the RTDE output stream.
//!
//! Port of `urRobotApp/src/rtde_receive_driver.cpp`. MAX_ADDR = 6 (one per
//! joint / pose element).

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
use crate::receive::ReceiveInterface;
use crate::registry::{self, ReceiveHandle, ReceiveState};
use crate::stream::Snapshot;

pub const NUM_JOINTS: usize = 6;
/// Only the lower 11 bits of the safety status word are used
/// (rtde_receive_driver.cpp:40).
const SAFETY_STATUS_BITS_MASK: u32 = 0x7ff;

use crate::drivers::STALE_AFTER;

/// Reconnect pacing for the poll thread: the first attempt fires on the
/// unhealthy transition, later ones back off 1 s → 2 s → 5 s (cap). Each
/// attempt itself is bounded by the connect/`FIRST_STATE_TIMEOUT` limits
/// inside [`ReceiveInterface::connect`]/`reconnect`.
const RECONNECT_BACKOFF: [Duration; 3] = [
    Duration::from_secs(1),
    Duration::from_secs(2),
    Duration::from_secs(5),
];

/// Backoff state for the poll-thread reconnect.
struct Backoff {
    idx: usize,
    next: Option<std::time::Instant>,
}

impl Backoff {
    fn new() -> Self {
        Self { idx: 0, next: None }
    }

    /// Whether an attempt is due — always true before the first failure.
    fn due(&self, now: std::time::Instant) -> bool {
        self.next.is_none_or(|t| now >= t)
    }

    /// Record a failed attempt: schedule the next one and escalate the delay.
    fn failed(&mut self, now: std::time::Instant) {
        self.next = Some(now + RECONNECT_BACKOFF[self.idx]);
        self.idx = (self.idx + 1).min(RECONNECT_BACKOFF.len() - 1);
    }

    /// Back to healthy: the next unhealthy transition retries immediately.
    fn reset(&mut self) {
        self.idx = 0;
        self.next = None;
    }
}

#[derive(Clone, Copy)]
pub struct ReceiveParams {
    pub disconnect: usize,
    pub reconnect: usize,
    pub is_connected: usize,
    pub runtime_state: usize,
    pub robot_mode: usize,
    pub safety_status_bits: usize,
    pub controller_timestamp: usize,
    pub std_analog_input0: usize,
    pub std_analog_input1: usize,
    pub std_analog_output0: usize,
    pub std_analog_output1: usize,
    pub actual_joint_pos_arr: usize,
    pub actual_joint_pos: usize,
    pub actual_tcp_pose_arr: usize,
    pub actual_tcp_pose: usize,
    pub digital_input_bits: usize,
    pub digital_output_bits: usize,
    pub actual_joint_velocities: usize,
    pub actual_joint_currents: usize,
    pub joint_control_currents: usize,
    pub actual_tcp_speed: usize,
    pub actual_tcp_force: usize,
    pub safety_mode: usize,
    pub joint_modes: usize,
    pub actual_tool_accel: usize,
    pub target_joint_positions: usize,
    pub target_joint_velocities: usize,
    pub target_joint_accelerations: usize,
    pub target_joint_currents: usize,
    pub target_joint_moments: usize,
    pub target_tcp_pose: usize,
    pub target_tcp_speed: usize,
    pub joint_temperatures: usize,
    pub speed_scaling: usize,
    pub target_speed_fraction: usize,
    pub actual_momentum: usize,
    pub actual_main_voltage: usize,
    pub actual_robot_voltage: usize,
    pub actual_robot_current: usize,
    pub actual_joint_voltages: usize,
    pub output_integer_reg12: usize,
}

impl ReceiveParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            disconnect: base.create_param("DISCONNECT", ParamType::Int32)?,
            reconnect: base.create_param("RECONNECT", ParamType::Int32)?,
            is_connected: base.create_param("IS_CONNECTED", ParamType::Int32)?,
            runtime_state: base.create_param("RUNTIME_STATE", ParamType::Int32)?,
            robot_mode: base.create_param("ROBOT_MODE", ParamType::Int32)?,
            safety_status_bits: base.create_param("SAFETY_STATUS_BITS", ParamType::Int32)?,
            controller_timestamp: base.create_param("CONTROLLER_TIMESTAMP", ParamType::Float64)?,
            std_analog_input0: base.create_param("STD_ANALOG_INPUT0", ParamType::Float64)?,
            std_analog_input1: base.create_param("STD_ANALOG_INPUT1", ParamType::Float64)?,
            std_analog_output0: base.create_param("STD_ANALOG_OUTPUT0", ParamType::Float64)?,
            std_analog_output1: base.create_param("STD_ANALOG_OUTPUT1", ParamType::Float64)?,
            actual_joint_pos_arr: base
                .create_param("ACTUAL_JOINT_POS_ARR", ParamType::Float64Array)?,
            actual_joint_pos: base.create_param("ACTUAL_JOINT_POS", ParamType::Float64)?,
            actual_tcp_pose_arr: base
                .create_param("ACTUAL_TCP_POSE_ARR", ParamType::Float64Array)?,
            actual_tcp_pose: base.create_param("ACTUAL_TCP_POSE", ParamType::Float64)?,
            digital_input_bits: base.create_param("DIGITAL_INPUT_BITS", ParamType::Int32)?,
            digital_output_bits: base.create_param("DIGITAL_OUTPUT_BITS", ParamType::Int32)?,
            actual_joint_velocities: base
                .create_param("ACTUAL_JOINT_VELOCITIES", ParamType::Float64Array)?,
            actual_joint_currents: base
                .create_param("ACTUAL_JOINT_CURRENTS", ParamType::Float64Array)?,
            joint_control_currents: base
                .create_param("JOINT_CONTROL_CURRENTS", ParamType::Float64Array)?,
            actual_tcp_speed: base.create_param("ACTUAL_TCP_SPEED", ParamType::Float64Array)?,
            actual_tcp_force: base.create_param("ACTUAL_TCP_FORCE", ParamType::Float64Array)?,
            safety_mode: base.create_param("SAFETY_MODE", ParamType::Int32)?,
            joint_modes: base.create_param("JOINT_MODES", ParamType::Int32Array)?,
            actual_tool_accel: base.create_param("ACTUAL_TOOL_ACCEL", ParamType::Float64Array)?,
            target_joint_positions: base
                .create_param("TARGET_JOINT_POSITIONS", ParamType::Float64Array)?,
            target_joint_velocities: base
                .create_param("TARGET_JOINT_VELOCITIES", ParamType::Float64Array)?,
            target_joint_accelerations: base
                .create_param("TARGET_JOINT_ACCELERATIONS", ParamType::Float64Array)?,
            target_joint_currents: base
                .create_param("TARGET_JOINT_CURRENTS", ParamType::Float64Array)?,
            target_joint_moments: base
                .create_param("TARGET_JOINT_MOMENTS", ParamType::Float64Array)?,
            target_tcp_pose: base.create_param("TARGET_TCP_POSE", ParamType::Float64Array)?,
            target_tcp_speed: base.create_param("TARGET_TCP_SPEED", ParamType::Float64Array)?,
            joint_temperatures: base.create_param("JOINT_TEMPERATURES", ParamType::Float64Array)?,
            speed_scaling: base.create_param("SPEED_SCALING", ParamType::Float64)?,
            target_speed_fraction: base
                .create_param("TARGET_SPEED_FRACTION", ParamType::Float64)?,
            actual_momentum: base.create_param("ACTUAL_MOMENTUM", ParamType::Float64)?,
            actual_main_voltage: base.create_param("ACTUAL_MAIN_VOLTAGE", ParamType::Float64)?,
            actual_robot_voltage: base.create_param("ACTUAL_ROBOT_VOLTAGE", ParamType::Float64)?,
            actual_robot_current: base.create_param("ACTUAL_ROBOT_CURRENT", ParamType::Float64)?,
            actual_joint_voltages: base
                .create_param("ACTUAL_JOINT_VOLTAGES", ParamType::Float64Array)?,
            output_integer_reg12: base.create_param("OUTPUT_INTEGER_REG12", ParamType::Int32)?,
        })
    }

    /// The momentary commands of this port — see the
    /// [module docs](crate::drivers#momentary-commands).
    fn is_command(&self, reason: usize) -> bool {
        reason == self.disconnect || reason == self.reconnect
    }

    /// Every readback `(reason, addr)` the poll thread publishes — the set
    /// that carries the COMM alarm while the stream is down. Excluded: the
    /// two link commands (nothing to alarm on a command), and
    /// `IS_CONNECTED`, which is the health readback itself and must stay
    /// valid at 0 while everything else goes COMM/INVALID — it is the one
    /// PV that says why. `alarm_set_is_every_readback` enforces that a new
    /// parameter lands in exactly one of the two groups.
    fn alarm_targets(&self) -> Vec<(usize, i32)> {
        let scalars = [
            self.runtime_state,
            self.robot_mode,
            self.safety_status_bits,
            self.controller_timestamp,
            self.std_analog_input0,
            self.std_analog_input1,
            self.std_analog_output0,
            self.std_analog_output1,
            self.actual_joint_pos_arr,
            self.actual_tcp_pose_arr,
            self.digital_input_bits,
            self.digital_output_bits,
            self.actual_joint_velocities,
            self.actual_joint_currents,
            self.joint_control_currents,
            self.actual_tcp_speed,
            self.actual_tcp_force,
            self.safety_mode,
            self.joint_modes,
            self.actual_tool_accel,
            self.target_joint_positions,
            self.target_joint_velocities,
            self.target_joint_accelerations,
            self.target_joint_currents,
            self.target_joint_moments,
            self.target_tcp_pose,
            self.target_tcp_speed,
            self.joint_temperatures,
            self.speed_scaling,
            self.target_speed_fraction,
            self.actual_momentum,
            self.actual_main_voltage,
            self.actual_robot_voltage,
            self.actual_robot_current,
            self.actual_joint_voltages,
            self.output_integer_reg12,
        ];
        let mut targets: Vec<(usize, i32)> = scalars.into_iter().map(|r| (r, 0)).collect();
        // The per-joint scalars are published at every address.
        for addr in 0..NUM_JOINTS as i32 {
            targets.push((self.actual_joint_pos, addr));
            targets.push((self.actual_tcp_pose, addr));
        }
        targets
    }
}

/// The RTDE receive driver.
pub struct ReceiveDriver {
    base: PortDriverBase,
    params: ReceiveParams,
    iface: Arc<Mutex<Option<ReceiveInterface>>>,
    robot_ip: String,
    shared: ReceiveHandle,
}

impl ReceiveDriver {
    pub fn new(port_name: &str, robot_ip: &str) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            NUM_JOINTS,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let params = ReceiveParams::create(&mut base)?;

        let shared = ReceiveHandle::new();
        registry::register_receive(port_name, shared.clone()).map_err(asyn_error)?;

        let mut me = Self {
            base,
            params,
            iface: Arc::new(Mutex::new(None)),
            robot_ip: robot_ip.to_string(),
            shared,
        };
        me.try_connect();
        Ok(me)
    }

    pub fn params(&self) -> ReceiveParams {
        self.params
    }

    pub fn iface(&self) -> Arc<Mutex<Option<ReceiveInterface>>> {
        Arc::clone(&self.iface)
    }

    pub fn shared(&self) -> ReceiveHandle {
        self.shared.clone()
    }

    /// `try_connect()`.
    ///
    /// The C++ version returns false when the interface exists and is *already*
    /// connected (rtde_receive_driver.cpp:30-36): the `else` branch only sets
    /// `connected` inside `if (!isConnected())`, so processing RECONNECT on a
    /// healthy link answers `asynError`. Here a live connection is a success.
    fn try_connect(&mut self) -> bool {
        connect_or_reconnect(&mut self.iface.lock(), &self.robot_ip)
    }
}

/// Connect (no interface yet) or reconnect (interface present but down) the
/// receive interface in `slot`. A live connection is a success. Shared by the
/// RECONNECT command ([`ReceiveDriver::try_connect`]) and the poll thread's
/// automatic recovery.
fn connect_or_reconnect(slot: &mut Option<ReceiveInterface>, robot_ip: &str) -> bool {
    match slot.as_mut() {
        Some(iface) => {
            // The healthy short-circuit gates on aliveness, not the socket: a
            // stale link (socket open, packages stopped) still answers
            // `is_connected() == true` and must be cycled, or both the manual
            // RECONNECT and the poll-thread recovery would no-op forever.
            if iface.is_alive(STALE_AFTER) {
                return true;
            }
            log::info!("ur-robot: reconnecting to the RTDE receive interface");
            match iface.reconnect() {
                Ok(()) => true,
                Err(e) => {
                    log::error!("ur-robot: RTDE receive reconnect failed: {e}");
                    false
                }
            }
        }
        None => match ReceiveInterface::connect(robot_ip, false) {
            Ok(iface) => {
                log::info!("ur-robot: connected to the RTDE receive interface");
                *slot = Some(iface);
                true
            }
            Err(e) => {
                log::error!("ur-robot: RTDE receive connect failed: {e}");
                false
            }
        },
    }
}

impl PortDriver for ReceiveDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let result: AsynResult<()> = (|| {
            let reason = user.reason;
            let p = self.params;
            self.base.params.set_int32(reason, user.addr, value)?;

            if p.is_command(reason) && value == 0 {
                return Ok(());
            }

            if reason == p.reconnect {
                return if self.try_connect() {
                    Ok(())
                } else {
                    Err(asyn_error(
                        "could not connect to the RTDE receive interface",
                    ))
                };
            }

            let mut slot = self.iface.lock();
            let Some(iface) = slot.as_mut() else {
                return Err(asyn_error("the RTDE receive interface is not initialised"));
            };

            if reason == p.disconnect {
                iface.disconnect();
                return Ok(());
            }

            if !iface.is_connected() {
                return Err(asyn_error("the RTDE receive interface is not connected"));
            }
            Ok(())
        })();
        crate::drivers::flush_after(&mut self.base, user.addr, result)
    }
}

/// The receive poll thread (`RTDEReceive::poll`).
///
/// Deviation from C (which never reconnects on its own): the poll thread owns
/// automatic recovery. An unhealthy stream — socket down OR stale per
/// [`STALE_AFTER`] — is retried on the [`RECONNECT_BACKOFF`] schedule, so a
/// monitoring IOC comes back by itself when the robot does.
pub fn start_poller(
    handle: PortHandle,
    params: ReceiveParams,
    iface: Arc<Mutex<Option<ReceiveInterface>>>,
    shared: ReceiveHandle,
    robot_ip: String,
    poll_period: Duration,
    ready: Arc<IocReady>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("ur-receive-poll".into())
        .spawn(move || {
            ready.wait();
            let mut backoff = Backoff::new();
            let alarm_targets = params.alarm_targets();
            // Starts true so an IOC that boots with the robot down raises
            // the COMM alarm on its first cycle instead of waiting for a
            // connection that may never come.
            let mut was_healthy = true;
            loop {
                let healthy = {
                    let slot = iface.lock();
                    slot.as_ref().is_some_and(|i| i.is_alive(STALE_AFTER))
                };
                if healthy {
                    backoff.reset();
                } else {
                    let now = std::time::Instant::now();
                    if backoff.due(now) {
                        if connect_or_reconnect(&mut iface.lock(), &robot_ip) {
                            backoff.reset();
                        } else {
                            backoff.failed(now);
                        }
                    }
                }

                let snapshot = {
                    let slot = iface.lock();
                    slot.as_ref()
                        .filter(|i| i.is_alive(STALE_AFTER))
                        .map(|i| i.snapshot())
                };

                let mut updates = match &snapshot {
                    Some(snap) if !snap.is_empty() => {
                        let state = ReceiveState {
                            connected: true,
                            safety_status_bits: snap
                                .uint("safety_status_bits")
                                .map(|b| (b & SAFETY_STATUS_BITS_MASK) as i32)
                                .unwrap_or_default(),
                            output_int_register_12: snap
                                .output_int_register(12)
                                .unwrap_or_default(),
                        };
                        shared.set(state);
                        publish(params, snap)
                    }
                    _ => {
                        let mut state = shared.get();
                        state.connected = false;
                        shared.set(state);
                        vec![ParamSetValue::new(
                            params.is_connected,
                            0,
                            ParamValue::Int32(0),
                        )]
                    }
                };

                // Same predicate as the publish arm above, so the alarm
                // rides the flush cycle that first sees the transition —
                // recovery clears the alarm in the very batch that carries
                // the fresh values.
                let healthy_now = matches!(&snapshot, Some(s) if !s.is_empty());
                updates.extend(crate::drivers::health_transition(
                    &alarm_targets,
                    healthy_now,
                    &mut was_healthy,
                ));

                for (addr, batch) in by_addr(updates) {
                    let _ = handle.set_params_and_notify_blocking(addr, batch);
                }
                std::thread::sleep(poll_period);
            }
        })
        .expect("failed to spawn the RTDE receive poll thread")
}

/// Split a poll batch into one flush per address list.
///
/// `callParamCallbacks` flushes a single address list (C
/// asynPortDriver.cpp:820), and [`publish`] writes the joint and TCP-pose
/// elements at addresses `0..NUM_JOINTS`. Flushing address 0 alone stores the
/// other five but consumes no changed flag for them, so `Joint2`-`Joint6` and
/// `PoseY`-`PoseRz` never see an `I/O Intr` and stay UDF for the life of the
/// IOC.
fn by_addr(updates: Vec<ParamSetValue>) -> Vec<(i32, Vec<ParamSetValue>)> {
    let mut batches: Vec<(i32, Vec<ParamSetValue>)> = Vec::new();
    for u in updates {
        let addr = u.addr();
        match batches.iter_mut().find(|(a, _)| *a == addr) {
            Some((_, batch)) => batch.push(u),
            None => batches.push((addr, vec![u])),
        }
    }
    batches
}

fn deg(rad: f64) -> f64 {
    rad * 180.0 / std::f64::consts::PI
}

/// Read one snapshot into the parameter updates the C++ poll loop performs.
fn publish(p: ReceiveParams, snap: &Snapshot) -> Vec<ParamSetValue> {
    let mut updates = Vec::new();
    let mut int32 = |reason: usize, value: i32| {
        updates.push(ParamSetValue::new(reason, 0, ParamValue::Int32(value)))
    };

    int32(p.is_connected, 1);
    if let Some(bits) = snap.uint("safety_status_bits") {
        int32(
            p.safety_status_bits,
            (bits & SAFETY_STATUS_BITS_MASK) as i32,
        );
    }
    if let Some(v) = snap.uint("runtime_state") {
        int32(p.runtime_state, v as i32);
    }
    if let Some(v) = snap.int("robot_mode") {
        int32(p.robot_mode, v);
    }
    if let Some(v) = snap.int("safety_mode") {
        int32(p.safety_mode, v);
    }
    if let Some(v) = snap.uint("actual_digital_input_bits") {
        int32(p.digital_input_bits, v as i32);
    }
    if let Some(v) = snap.uint("actual_digital_output_bits") {
        int32(p.digital_output_bits, v as i32);
    }
    if let Some(v) = snap.output_int_register(12) {
        int32(p.output_integer_reg12, v);
    }

    let mut float64 = |reason: usize, value: f64| {
        updates.push(ParamSetValue::new(reason, 0, ParamValue::Float64(value)))
    };
    for (reason, name) in [
        (p.controller_timestamp, "timestamp"),
        (p.std_analog_input0, "standard_analog_input0"),
        (p.std_analog_input1, "standard_analog_input1"),
        (p.std_analog_output0, "standard_analog_output0"),
        (p.std_analog_output1, "standard_analog_output1"),
        (p.speed_scaling, "speed_scaling"),
        (p.target_speed_fraction, "target_speed_fraction"),
        (p.actual_momentum, "actual_momentum"),
        (p.actual_main_voltage, "actual_main_voltage"),
        (p.actual_robot_voltage, "actual_robot_voltage"),
        (p.actual_robot_current, "actual_robot_current"),
    ] {
        if let Some(v) = snap.double(name) {
            float64(reason, v);
        }
    }

    // Arrays that go out unconverted.
    for (reason, name) in [
        (p.actual_joint_velocities, "actual_qd"),
        (p.actual_joint_currents, "actual_current"),
        (p.joint_control_currents, "joint_control_output"),
        (p.actual_tcp_speed, "actual_TCP_speed"),
        (p.actual_tcp_force, "actual_TCP_force"),
        (p.target_joint_positions, "target_q"),
        (p.target_joint_velocities, "target_qd"),
        (p.target_joint_accelerations, "target_qdd"),
        (p.target_joint_currents, "target_current"),
        (p.target_joint_moments, "target_moment"),
        (p.target_tcp_pose, "target_TCP_pose"),
        (p.target_tcp_speed, "target_TCP_speed"),
        (p.joint_temperatures, "joint_temperatures"),
        (p.actual_joint_voltages, "actual_joint_voltage"),
        (p.actual_tool_accel, "actual_tool_accelerometer"),
    ] {
        if let Some(v) = snap.doubles(name) {
            updates.push(ParamSetValue::new(
                reason,
                0,
                ParamValue::Float64Array(v.to_vec().into()),
            ));
        }
    }

    if let Some(v) = snap.ints("joint_mode") {
        updates.push(ParamSetValue::new(
            p.joint_modes,
            0,
            ParamValue::Int32Array(v.to_vec().into()),
        ));
    }

    // Joint positions: radians on the wire, degrees on the PVs.
    if let Some(q) = snap.doubles("actual_q") {
        let degrees: Vec<f64> = q.iter().copied().map(deg).collect();
        for (i, v) in degrees.iter().enumerate() {
            updates.push(ParamSetValue::new(
                p.actual_joint_pos,
                i as i32,
                ParamValue::Float64(*v),
            ));
        }
        updates.push(ParamSetValue::new(
            p.actual_joint_pos_arr,
            0,
            ParamValue::Float64Array(degrees.into()),
        ));
    }

    // TCP pose: x,y,z in metres -> mm; rx,ry,rz stay radians — they are a
    // rotation vector (axis-angle scaled by the angle), not Euler angles, so
    // a degree scaling has no meaning (upstream c02490e).
    if let Some(pose) = snap.doubles("actual_TCP_pose") {
        let converted: Vec<f64> = pose
            .iter()
            .enumerate()
            .map(|(i, v)| if i < 3 { v * 1000.0 } else { *v })
            .collect();
        for (i, v) in converted.iter().enumerate() {
            updates.push(ParamSetValue::new(
                p.actual_tcp_pose,
                i as i32,
                ParamValue::Float64(*v),
            ));
        }
        updates.push(ParamSetValue::new(
            p.actual_tcp_pose_arr,
            0,
            ParamValue::Float64Array(converted.into()),
        ));
    }

    updates
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::Value;
    use epics_rs::asyn::error::AsynStatus;
    use std::collections::HashMap;
    use std::time::Instant;

    /// The pacing boundaries: an attempt is due immediately on the unhealthy
    /// transition, then 1 s → 2 s → 5 s, capped at 5 s, and reset restores
    /// the immediate first attempt.
    #[test]
    fn backoff_escalates_to_the_cap_and_resets() {
        let mut b = Backoff::new();
        let t0 = Instant::now();
        assert!(b.due(t0), "first attempt is immediate");

        b.failed(t0);
        assert!(!b.due(t0 + Duration::from_millis(999)));
        assert!(b.due(t0 + Duration::from_secs(1)));

        b.failed(t0);
        assert!(!b.due(t0 + Duration::from_millis(1999)));
        assert!(b.due(t0 + Duration::from_secs(2)));

        b.failed(t0);
        assert!(b.due(t0 + Duration::from_secs(5)));
        // Capped: further failures stay at 5 s.
        b.failed(t0);
        assert!(!b.due(t0 + Duration::from_millis(4999)));
        assert!(b.due(t0 + Duration::from_secs(5)));

        b.reset();
        assert!(b.due(t0), "reset restores the immediate first attempt");
    }

    fn params() -> ReceiveParams {
        // Distinct indices; only their identity matters to `publish`.
        let mut n = 0usize;
        let mut next = || {
            n += 1;
            n
        };
        ReceiveParams {
            disconnect: next(),
            reconnect: next(),
            is_connected: next(),
            runtime_state: next(),
            robot_mode: next(),
            safety_status_bits: next(),
            controller_timestamp: next(),
            std_analog_input0: next(),
            std_analog_input1: next(),
            std_analog_output0: next(),
            std_analog_output1: next(),
            actual_joint_pos_arr: next(),
            actual_joint_pos: next(),
            actual_tcp_pose_arr: next(),
            actual_tcp_pose: next(),
            digital_input_bits: next(),
            digital_output_bits: next(),
            actual_joint_velocities: next(),
            actual_joint_currents: next(),
            joint_control_currents: next(),
            actual_tcp_speed: next(),
            actual_tcp_force: next(),
            safety_mode: next(),
            joint_modes: next(),
            actual_tool_accel: next(),
            target_joint_positions: next(),
            target_joint_velocities: next(),
            target_joint_accelerations: next(),
            target_joint_currents: next(),
            target_joint_moments: next(),
            target_tcp_pose: next(),
            target_tcp_speed: next(),
            joint_temperatures: next(),
            speed_scaling: next(),
            target_speed_fraction: next(),
            actual_momentum: next(),
            actual_main_voltage: next(),
            actual_robot_voltage: next(),
            actual_robot_current: next(),
            actual_joint_voltages: next(),
            output_integer_reg12: next(),
        }
    }

    #[test]
    fn joint_positions_are_published_in_degrees_per_addr_and_as_an_array() {
        let p = params();
        let mut values = HashMap::new();
        values.insert(
            "actual_q".to_string(),
            Value::Doubles(vec![
                0.0,
                std::f64::consts::FRAC_PI_2,
                -std::f64::consts::PI,
                0.0,
                0.0,
                0.0,
            ]),
        );
        let updates = publish(p, &Snapshot::new(values));

        let per_addr: Vec<(i32, f64)> = updates
            .iter()
            .filter_map(|u| match u {
                ParamSetValue::Value {
                    reason,
                    addr,
                    value: ParamValue::Float64(value),
                } if *reason == p.actual_joint_pos => Some((*addr, *value)),
                _ => None,
            })
            .collect();
        assert_eq!(per_addr.len(), 6);
        assert!((per_addr[1].1 - 90.0).abs() < 1e-9);
        assert!((per_addr[2].1 + 180.0).abs() < 1e-9);

        let arr = updates
            .iter()
            .find_map(|u| match u {
                ParamSetValue::Value {
                    reason,
                    value: ParamValue::Float64Array(value),
                    ..
                } if *reason == p.actual_joint_pos_arr => Some(value.clone()),
                _ => None,
            })
            .expect("joint position array");
        assert_eq!(arr.len(), 6);
        assert!((arr[1] - 90.0).abs() < 1e-9);
    }

    #[test]
    fn tcp_pose_converts_metres_to_mm_and_passes_rotation_vector_raw() {
        let p = params();
        let mut values = HashMap::new();
        values.insert(
            "actual_TCP_pose".to_string(),
            Value::Doubles(vec![0.1, -0.2, 0.35, std::f64::consts::PI, 0.0, 0.0]),
        );
        let updates = publish(p, &Snapshot::new(values));
        let arr = updates
            .iter()
            .find_map(|u| match u {
                ParamSetValue::Value {
                    reason,
                    value: ParamValue::Float64Array(value),
                    ..
                } if *reason == p.actual_tcp_pose_arr => Some(value.clone()),
                _ => None,
            })
            .expect("TCP pose array");
        assert!((arr[0] - 100.0).abs() < 1e-9);
        assert!((arr[1] + 200.0).abs() < 1e-9);
        assert!((arr[2] - 350.0).abs() < 1e-9);
        assert!((arr[3] - std::f64::consts::PI).abs() < 1e-9);
    }

    #[test]
    fn safety_status_bits_are_masked_to_11_bits() {
        let p = params();
        let mut values = HashMap::new();
        values.insert(
            "safety_status_bits".to_string(),
            Value::Uint32(0xffff_f801), // high bits set, plus NORMAL
        );
        let updates = publish(p, &Snapshot::new(values));
        let bits = updates
            .iter()
            .find_map(|u| match u {
                ParamSetValue::Value {
                    reason,
                    value: ParamValue::Int32(value),
                    ..
                } if *reason == p.safety_status_bits => Some(*value),
                _ => None,
            })
            .expect("safety status bits");
        assert_eq!(bits, 0x001);
    }

    #[test]
    fn a_snapshot_without_the_register_block_still_publishes_the_rest() {
        let p = params();
        let mut values = HashMap::new();
        values.insert("timestamp".to_string(), Value::Double(3.5));
        let updates = publish(p, &Snapshot::new(values));
        assert!(updates.iter().any(|u| matches!(
            u,
            ParamSetValue::Value { reason, value: ParamValue::Float64(value), .. }
                if *reason == p.controller_timestamp && (*value - 3.5).abs() < 1e-9
        )));
        // OUTPUT_INTEGER_REG12 is simply absent, not published as 0.
        assert!(!updates.iter().any(|u| matches!(
            u,
            ParamSetValue::Value { reason, value: ParamValue::Int32(_), .. } if *reason == p.output_integer_reg12
        )));
    }

    fn addr_of(u: &ParamSetValue) -> i32 {
        u.addr()
    }

    #[test]
    fn every_update_is_flushed_under_its_own_address() {
        let p = params();
        let mut values = HashMap::new();
        values.insert("timestamp".to_string(), Value::Double(3.5));
        values.insert(
            "actual_q".to_string(),
            Value::Doubles(vec![0.0, 0.1, 0.2, 0.3, 0.4, 0.5]),
        );
        values.insert(
            "actual_TCP_pose".to_string(),
            Value::Doubles(vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
        );
        let updates = publish(p, &Snapshot::new(values));
        let total = updates.len();

        let batches = by_addr(updates);
        assert_eq!(
            batches.iter().map(|(a, _)| *a).collect::<Vec<_>>(),
            (0..NUM_JOINTS as i32).collect::<Vec<_>>(),
            "one flush per address, address 0 first"
        );
        assert_eq!(batches.iter().map(|(_, b)| b.len()).sum::<usize>(), total);
        for (addr, batch) in &batches {
            assert!(batch.iter().all(|u| addr_of(u) == *addr));
        }
    }

    #[test]
    fn a_batch_that_touches_only_address_0_is_one_flush() {
        let p = params();
        let mut values = HashMap::new();
        values.insert("timestamp".to_string(), Value::Double(3.5));
        let batches = by_addr(publish(p, &Snapshot::new(values)));
        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].0, 0);
    }

    /// The gate's own boundary, on the one command whose action is reachable
    /// without a robot: with no interface, `DISCONNECT` fails loudly, so the
    /// zero write returning `Ok` is proof it never got that far. Drop the
    /// `is_command` guard from `write_int32` and the first assertion fails.
    #[test]
    fn a_zero_write_to_a_command_is_a_no_op_and_a_one_still_acts() {
        let mut base = PortDriverBase::new("recv_gate", NUM_JOINTS, PortFlags::default());
        let params = ReceiveParams::create(&mut base).expect("params create");
        let mut driver = ReceiveDriver {
            base,
            params,
            iface: Arc::new(Mutex::new(None)),
            robot_ip: String::new(),
            shared: ReceiveHandle::new(),
        };

        let mut user = AsynUser::new(params.disconnect);
        assert!(driver.write_int32(&mut user, 0).is_ok());
        assert!(driver.write_int32(&mut user, 1).is_err());

        // The store happens either way: the readback follows the record.
        assert_eq!(
            driver.base.params.get_int32(params.disconnect, 0).ok(),
            Some(1)
        );
    }

    /// Completeness of the alarm set: every parameter this driver creates is
    /// either an alarm target or one of the three deliberate exclusions
    /// (the two link commands and the health readback). A parameter added to
    /// `ReceiveParams` without classifying it here fails this test instead of
    /// silently staying NO_ALARM through an outage.
    #[test]
    fn alarm_set_is_every_readback() {
        let mut base = PortDriverBase::new("recv_alarm_set", NUM_JOINTS, PortFlags::default());
        let p = ReceiveParams::create(&mut base).expect("params create");

        let targets = p.alarm_targets();
        let excluded = [p.disconnect, p.reconnect, p.is_connected];
        for reason in 0..base.params.len() {
            let targeted = targets.iter().any(|(r, _)| *r == reason);
            let is_excluded = excluded.contains(&reason);
            assert!(
                targeted != is_excluded,
                "param {reason} must be exactly one of: alarm target, exclusion"
            );
        }

        // The per-joint scalars are covered at every address they publish to.
        for per_addr in [p.actual_joint_pos, p.actual_tcp_pose] {
            for addr in 0..NUM_JOINTS as i32 {
                assert!(targets.contains(&(per_addr, addr)));
            }
        }
    }

    /// The alarm batch fires on the two health transitions and only there:
    /// link lost raises Disconnected on every target (the record-side
    /// fill-in maps it to COMM/INVALID), recovery clears to Success, and
    /// both steady states stay silent.
    #[test]
    fn the_comm_alarm_rides_the_health_transitions_only() {
        use crate::drivers::health_transition;
        let p = params();
        let targets = p.alarm_targets();
        let mut was = true;

        assert!(health_transition(&targets, true, &mut was).is_empty());

        let raise = health_transition(&targets, false, &mut was);
        assert_eq!(raise.len(), targets.len());
        assert!(raise.iter().all(|u| matches!(
            u,
            ParamSetValue::Status {
                status: AsynStatus::Disconnected,
                alarm_status: 0,
                alarm_severity: 0,
                ..
            }
        )));
        assert!(!was, "the edge is consumed");

        assert!(health_transition(&targets, false, &mut was).is_empty());

        let clear = health_transition(&targets, true, &mut was);
        assert_eq!(clear.len(), targets.len());
        assert!(clear.iter().all(|u| matches!(
            u,
            ParamSetValue::Status {
                status: AsynStatus::Success,
                ..
            }
        )));
        assert!(was);
    }

    #[test]
    fn the_two_link_commands_are_commands_and_the_readbacks_are_not() {
        let mut base = PortDriverBase::new("recv_is_command", NUM_JOINTS, PortFlags::default());
        let p = ReceiveParams::create(&mut base).expect("params create");

        assert!(p.is_command(p.disconnect));
        assert!(p.is_command(p.reconnect));
        for reason in [
            p.is_connected,
            p.runtime_state,
            p.controller_timestamp,
            p.actual_joint_pos,
            p.output_integer_reg12,
        ] {
            assert!(!p.is_command(reason));
        }
    }
}
