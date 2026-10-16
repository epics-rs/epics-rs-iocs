//! `motorSimCreateController` / `motorSimConfigAxis` (motorSimDriver.cpp)
//! and `motorSimCreate` (drvMotorSim.c) iocsh commands for the simulated
//! motor driver.
//!
//! `motorSimConfigAxis` reconfigures an axis *after* its controller was
//! created, so [`MotorSimHolder`] keeps the concrete [`MotorSimAxis`]
//! handles, the same allocations the pollers drive, keyed by port and axis.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::iocsh::{arg_int_opt, arg_int_req, arg_str_req, req_int, req_string};
use motor_common::model1::{Model1, c_int};
use motor_common::{asyn_user, install, model1_device, register_port};

use crate::motorsim::{
    DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_LOW_LIMIT, DEFAULT_START, DrvMotorSimAxis, MotorSimAxis,
};

/// C's `motorSimTask` updates every axis each `DELTA` (motorSimDriver.cpp:244),
/// moving or not.
const POLL_MS: u64 = 100;

/// `device(motor,VME_IO,devMotorSim,"Motor Simulation")`
/// (motorSimSupport.dbd:1).
static DEV_MOTOR_SIM: DevSup = model1_device("devMotorSim", "Motor Simulation");

/// drvMotorSim.c has no `Setup`; its task calls back every `DELTA`.
static MOTOR_SIM: Model1 = Model1::new(&DEV_MOTOR_SIM, i32::MAX, (1000 / POLL_MS) as i32);

/// `motorSimCreate(card, signal, lowLimit, hiLimit, home, nCards, nSignals,
/// startPosn)`: axes `signal..signal+nSignals` of cards `card..card+nCards`
/// for `DTYP "Motor Simulation"`, `OUT "#C card S signal @motorSim"`.
pub fn motor_sim_create_command() -> CommandDef {
    MOTOR_SIM.register_row();
    let created: Mutex<HashSet<(i32, i32)>> = Mutex::new(HashSet::new());
    CommandDef::new(
        "motorSimCreate",
        vec![
            arg_int_req("Card"),
            arg_int_req("Signal"),
            // C labels these "High limit", "Low limit" but passes them to
            // motorSimCreate(card, axis, lowLimit, hiLimit, ...), and every
            // startup file gives low first.
            arg_int_req("Low limit"),
            arg_int_req("High limit"),
            arg_int_req("Home position"),
            arg_int_req("Num cards"),
            arg_int_req("Num signals"),
            arg_int_req("Start posn"),
        ],
        "motorSimCreate(card, signal, lowLimit, hiLimit, home, nCards, nSignals, startPosn)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            let signal = c_int(args, 1);
            let low = c_int(args, 2);
            let hi = c_int(args, 3);
            let home = c_int(args, 4);
            let num_cards = c_int(args, 5).max(1);
            let num_signals = c_int(args, 6).max(1);
            println!(
                "Creating motor simulator: card: {card}, axis: {signal}, hi: {hi}, low {low}, home: {home}, ncards: {num_cards}, naxis: {num_signals}"
            );
            let mut created = created.lock().unwrap_or_else(|e| e.into_inner());
            for c in card..card.saturating_add(num_cards) {
                for s in signal..signal.saturating_add(num_signals) {
                    if !created.insert((c, s)) {
                        println!("Motor for card {c}, signal {s} already exists");
                        continue;
                    }
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(
                        DrvMotorSimAxis::new(low as f64, hi as f64, home as f64),
                    ));
                    MOTOR_SIM.install(ctx, c, s, motor)?;
                    println!("Created motor for card {c}, signal {s} OK");
                }
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

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
                register_port(ctx, &port_name, num_axes as i32)?;
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
