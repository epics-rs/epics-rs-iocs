//! `motorSimCreateController` / `motorSimConfigAxis` iocsh commands for the
//! simulated motor driver.
//!
//! `motorSimConfigAxis` reconfigures an axis *after* its controller was
//! created, so [`MotorSimHolder`] keeps the concrete [`MotorSimAxis`]
//! handles, the same allocations the pollers drive, keyed by port and axis.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::iocsh::{arg_int_opt, arg_int_req, arg_str_req, req_int, req_string};
use motor_common::{PortDriver, asyn_user, install, register_port};

use crate::motorsim::{
    DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_LOW_LIMIT, DEFAULT_START, MotorSimAxis,
};

/// C's `motorSimTask` updates every axis each `DELTA` (motorSimDriver.cpp:244),
/// moving or not.
const POLL_MS: u64 = 100;

/// The concrete axes keyed by `"{portName}:{axis}"` for `motorSimConfigAxis`.
pub struct MotorSimHolder {
    axes: Mutex<HashMap<String, Arc<Mutex<MotorSimAxis>>>>,
}

impl MotorSimHolder {
    /// Create an empty holder.
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            axes: Mutex::new(HashMap::new()),
        })
    }

    fn axis_key(port_name: &str, axis: i64) -> String {
        format!("{port_name}:{axis}")
    }

    /// `motorSimCreateController(portName, numAxes, priority, stackSize)`.
    /// The axes are records' `@asyn(portName,axis)`; `priority` and
    /// `stackSize` size C's port thread, which has no counterpart here.
    pub fn create_controller_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "motorSimCreateController",
            vec![
                arg_str_req("Port name"),
                arg_int_req("Number of axes"),
                arg_int_opt("priority"),
                arg_int_opt("stackSize"),
            ],
            "motorSimCreateController(portName, numAxes, priority, stackSize)",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let port_name = req_string(args, 0, "Port name")?;
                let mut num_axes = req_int(args, 1, "Number of axes")?;
                if num_axes < 1 {
                    num_axes = 1; // C: `if (numAxes < 1) numAxes = 1;`
                }
                register_port(&port_name, PortDriver::Controller, num_axes as i32)?;
                for index in 0..num_axes {
                    let axis = Arc::new(Mutex::new(MotorSimAxis::new(
                        DEFAULT_LOW_LIMIT,
                        DEFAULT_HI_LIMIT,
                        DEFAULT_HOME,
                        DEFAULT_START,
                    )));
                    let motor: Arc<Mutex<dyn AsynMotor>> = axis.clone();
                    install(
                        ctx,
                        asyn_user(&port_name, index as i32),
                        motor,
                        POLL_MS,
                        POLL_MS,
                    )?;
                    holder
                        .axes
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .insert(Self::axis_key(&port_name, index), axis);
                }
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// `motorSimConfigAxis(portName, axis, hiHardLimit, lowHardLimit, home,
    /// start)`, printing what C's does.
    pub fn config_axis_command(self: &Arc<Self>) -> CommandDef {
        let holder = self.clone();
        CommandDef::new(
            "motorSimConfigAxis",
            vec![
                arg_str_req("Port name"),
                arg_int_req("Axis #"),
                arg_int_req("High limit"),
                arg_int_req("Low limit"),
                arg_int_req("Home position"),
                arg_int_req("Start posn"),
            ],
            "motorSimConfigAxis(portName, axis, hiHardLimit, lowHardLimit, home, start)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = req_string(args, 0, "Port name")?;
                let axis = req_int(args, 1, "Axis #")?;
                let hi = req_int(args, 2, "High limit")?;
                let low = req_int(args, 3, "Low limit")?;
                let home = req_int(args, 4, "Home position")?;
                let start = req_int(args, 5, "Start posn")?;

                let axes = holder.axes.lock().unwrap_or_else(|e| e.into_inner());
                let Some(handle) = axes.get(&Self::axis_key(&port_name, axis)) else {
                    println!("Controller not found");
                    return Ok(CommandOutcome::Continue);
                };
                println!(
                    "motorSimDriver:motorSimConfigAxis: configuring controller {port_name} axis {axis}"
                );
                handle.lock().unwrap_or_else(|e| e.into_inner()).config(
                    hi as f64,
                    low as f64,
                    home as f64,
                    start as f64,
                );
                Ok(CommandOutcome::Continue)
            },
        )
    }
}
