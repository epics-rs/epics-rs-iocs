//! `motorSimCreateController` / `motorSimConfigAxis` (motorSimDriver.cpp)
//! and `motorSimCreate` (drvMotorSim.c) iocsh commands for the simulated
//! motor driver.
//!
//! `motorSimConfigAxis` reconfigures an axis *after* its controller was
//! created, so [`MotorSimHolder`] keeps the concrete [`MotorSimAxis`]
//! handles, the same allocations the pollers drive, keyed by port and axis.

use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use epics_rs::motor::devices::{
    motor_interface_device, register_motor_interface_device, register_motor_interface_driver,
};
use motor_common::iocsh::{arg_int_opt, arg_int_req, arg_str_req, req_int, req_string};
use motor_common::model1::c_int;
use motor_common::model2::Model2;
pub use motor_common::model2::drv_asyn_motor_configure_command;
use motor_common::{Controller, TraceUser, asyn_user, register_port};

use crate::motorsim::{
    DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_LOW_LIMIT, DEFAULT_START, DrvMotorSimAxis, MotorSimAxis,
};

/// C's `motorSimTask` updates every axis each `DELTA` (motorSimDriver.cpp:244),
/// moving or not.
const POLL_MS: u64 = 100;

/// `device(motor,VME_IO,devMotorSim,"Motor Simulation")`
/// (motorSimSupport.dbd:1).
static DEV_MOTOR_SIM: DevSup = motor_interface_device("devMotorSim", "Motor Simulation");

/// `driver(motorSim)`: the drvet a record's `OUT "#C card S signal
/// @motorSim"` names.
const DRIVER: &str = "motorSim";

/// The same drvMotorSim axes as a `motorAxisDrvSET_t`, for
/// `drvAsynMotorConfigure(port, "motorSim", card, nAxes)` (drvMotorAsyn.c).
/// motorProcTask has no forced polls.
pub static MOTOR_SIM: Model2 = Model2::new(DRIVER, 0);

/// The axes motorSimCreate built, by card and signal.
type SimAxes = BTreeMap<(i32, i32), Arc<Mutex<dyn AsynMotor>>>;

/// `motorSimCreate(card, signal, lowLimit, hiLimit, home, nCards, nSignals,
/// startPosn)`: axes `signal..signal+nSignals` of cards `card..card+nCards`
/// for `DTYP "Motor Simulation"`, `OUT "#C card S signal @motorSim"`.
pub fn motor_sim_create_command() -> CommandDef {
    register_motor_interface_device(&DEV_MOTOR_SIM);
    register_motor_interface_driver(DRIVER);
    let created: Mutex<SimAxes> = Mutex::new(BTreeMap::new());
    let poller = Controller::new();
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
            // The first motorSimCreate starts drvMotorSim's one
            // motorSimThread, whose motorProcTask runs every axis each DELTA,
            // moving or idle (drvMotorSim.c:657-690, 788-800).
            poller.start(ctx, POLL_MS, POLL_MS, 0);
            let mut created = created.lock().unwrap_or_else(|e| e.into_inner());
            for c in card..card.saturating_add(num_cards) {
                for s in signal..signal.saturating_add(num_signals) {
                    if created.contains_key(&(c, s)) {
                        println!("Motor for card {c}, signal {s} already exists");
                        continue;
                    }
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(
                        DrvMotorSimAxis::new(low as f64, hi as f64, home as f64),
                    ));
                    created.insert((c, s), motor.clone());
                    let user = TraceUser::MotorInterface {
                        device: DEV_MOTOR_SIM.choice,
                        driver: DRIVER,
                        card: c,
                        signal: s,
                    };
                    // C devMotorSim's callback scanOnce's from init_record, before scanInit, and
                    // segfaults (a C defect); here a callback before the
                    // gate only sets needUpdate.
                    poller.install(user, motor)?;
                    println!("Created motor for card {c}, signal {s} OK");
                }
                // Model2::open indexes by axis, so a card serves the axes
                // from 0 up to its first gap.
                let axes = (0..).map_while(|s| created.get(&(c, s)).cloned()).collect();
                MOTOR_SIM.set_card(c, axes, POLL_MS, POLL_MS);
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
                let poller = Controller::new();
                for index in 0..num_axes {
                    let axis = Arc::new(Mutex::new(MotorSimAxis::new(
                        DEFAULT_LOW_LIMIT,
                        DEFAULT_HI_LIMIT,
                        DEFAULT_HOME,
                        DEFAULT_START,
                    )));
                    let motor: Arc<Mutex<dyn AsynMotor>> = axis.clone();
                    poller.install(asyn_user(&port_name, index as i32), motor)?;
                    holder
                        .axes
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .insert(Self::axis_key(&port_name, index), axis);
                }
                // motorSimController's one motorSimTask polls every axis
                // each DELTA (motorSimDriver.cpp:244), forcing none.
                poller.start(ctx, POLL_MS, POLL_MS, 0);
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

#[cfg(all(test, target_os = "linux"))]
mod one_poller {
    use std::time::{Duration, Instant};

    use epics_rs::base::runtime::task::BlockingBridge;
    use epics_rs::base::server::database::PvDatabase;
    use motor_common::testing::{pollers, pollers_after};

    use super::*;

    /// motorSimController's one motorSimTask polls every axis.
    #[epics_rs::base::epics_test]
    async fn create_controller_polls_its_axes_on_one_poller() {
        let args = vec![
            ArgValue::String("simOnePoller".into()),
            ArgValue::Int(4),
            ArgValue::Int(0),
            ArgValue::Int(0),
        ];
        let cmd = MotorSimHolder::new().create_controller_command();
        assert_eq!(pollers_after(cmd, args).await, 1);
    }

    /// drvMotorSim's one motorSimThread runs every motorSimCreate's axes.
    /// The second call's axes join a running poller, which needs a runtime
    /// that keeps polling while the shell thread waits: not `pollers_after`'s.
    #[test]
    fn every_motor_sim_create_s_axes_share_one_poller() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let _entered = rt.enter();
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), BlockingBridge::capture());
        let cmd = motor_sim_create_command();
        for card in 0..2 {
            let args = [card, 0, -100, 100, 0, 1, 3, 0].map(ArgValue::Int);
            cmd.handler.call(&args, &ctx).unwrap();
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        while pollers() == 0 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        std::thread::sleep(Duration::from_millis(200));
        assert_eq!(pollers(), 1);
    }
}
