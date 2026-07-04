//! `motorSimCreateController` / `motorSimConfigAxis` iocsh commands for the
//! simulated motor driver.
//!
//! `motorSimConfigAxis` reconfigures an axis *after* its controller was
//! created, so this module keeps a registry of the concrete
//! [`MotorSimAxis`] handles ([`MotorSimHolder`]) alongside the generic
//! [`MotorHolder`] that owns the record device supports — both share the same
//! `Arc<Mutex<MotorSimAxis>>` allocation, so a config command mutates the axis
//! the poll loop drives.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::MotorHolder;
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};

use crate::motorsim::{
    DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_LOW_LIMIT, DEFAULT_START, MotorSimAxis,
};

/// Owns the record device supports (via the generic [`MotorHolder`]) and a
/// registry of the concrete axes keyed by `"{motorPort}:{axis}"` for
/// `motorSimConfigAxis`.
pub struct MotorSimHolder {
    inner: Arc<MotorHolder>,
    axes: Mutex<HashMap<String, Arc<Mutex<MotorSimAxis>>>>,
}

impl MotorSimHolder {
    /// Create an empty holder.
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            inner: MotorHolder::new(),
            axes: Mutex::new(HashMap::new()),
        })
    }

    /// The generic holder, for `register_dynamic_device_support`.
    pub fn inner(&self) -> &Arc<MotorHolder> {
        &self.inner
    }

    fn axis_key(motor_port: &str, axis: i64) -> String {
        format!("{motor_port}:{axis}")
    }

    /// Build the `motorSimCreateController(motorPort, numAxes, [movingPollMs],
    /// [idlePollMs])` command. (C's `priority`/`stackSize` args have no analogue
    /// in the asyn-rs runtime; the optional poll intervals take their place.)
    pub fn create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "motorSimCreateController",
            vec![
                arg_str_req("motorPort"),
                arg_int_req("numAxes"),
                arg_int_opt("movingPollMs"),
                arg_int_opt("idlePollMs"),
            ],
            "motorSimCreateController(motorPort, numAxes, [movingPollMs], [idlePollMs]) - \
             Create a simulated motor controller (DTYP motorSim_{motorPort}_{0..numAxes-1})",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let mut num_axes = req_int(args, 1, "numAxes")?;
                if num_axes < 1 {
                    num_axes = 1; // C: `if (numAxes < 1) numAxes = 1;`
                }
                let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 2, 3)?;

                for index in 0..num_axes {
                    let axis = Arc::new(Mutex::new(MotorSimAxis::new(
                        DEFAULT_LOW_LIMIT,
                        DEFAULT_HI_LIMIT,
                        DEFAULT_HOME,
                        DEFAULT_START,
                    )));
                    let motor: Arc<Mutex<dyn AsynMotor>> = axis.clone();
                    let dtyp_key = format!("motorSim_{motor_port}_{index}");
                    holder
                        .inner
                        .install(ctx, dtyp_key, motor, moving_poll_ms, idle_poll_ms);
                    holder
                        .axes
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .insert(Self::axis_key(&motor_port, index), axis);
                }
                println!(
                    "motorSimCreateController: motorPort={motor_port} axes={num_axes} \
                     poll=[{moving_poll_ms}/{idle_poll_ms}]ms \
                     (DTYP=motorSim_{motor_port}_{{0..{}}})",
                    num_axes - 1
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Build the `motorSimConfigAxis(motorPort, axis, hiHardLimit, lowHardLimit,
    /// home, start)` command (C `motorSimConfigAxis`).
    pub fn config_axis_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "motorSimConfigAxis",
            vec![
                arg_str_req("motorPort"),
                arg_int_req("axis"),
                arg_int_req("hiHardLimit"),
                arg_int_req("lowHardLimit"),
                arg_int_req("home"),
                arg_int_req("start"),
            ],
            "motorSimConfigAxis(motorPort, axis, hiHardLimit, lowHardLimit, home, start) - \
             Reconfigure a simulated axis's hard limits, home, and start offset",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let motor_port = req_string(args, 0, "motorPort")?;
                let axis = req_int(args, 1, "axis")?;
                let hi = req_int(args, 2, "hiHardLimit")?;
                let low = req_int(args, 3, "lowHardLimit")?;
                let home = req_int(args, 4, "home")?;
                let start = req_int(args, 5, "start")?;

                let axes = holder.axes.lock().unwrap_or_else(|e| e.into_inner());
                let handle = axes
                    .get(&Self::axis_key(&motor_port, axis))
                    .ok_or_else(|| {
                        format!("motorSimConfigAxis: no axis {axis} on controller {motor_port}")
                    })?;
                handle.lock().unwrap_or_else(|e| e.into_inner()).config(
                    hi as f64,
                    low as f64,
                    home as f64,
                    start as f64,
                );
                println!(
                    "motorSimConfigAxis: {motor_port} axis {axis} \
                     limits=[{low},{hi}] home={home} start={start}"
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }
}
