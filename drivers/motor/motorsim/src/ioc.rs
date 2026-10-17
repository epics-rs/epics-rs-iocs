//! `motorSimCreateController` / `motorSimConfigAxis` (motorSimDriver.cpp)
//! and `motorSimCreate` (drvMotorSim.c) iocsh commands for the simulated
//! motor driver.
//!
//! `motorSimConfigAxis` reconfigures an axis *after* its controller was
//! created, so [`MotorSimHolder`] keeps the concrete [`MotorSimAxis`]
//! handles, the same allocations the pollers drive, keyed by port and axis.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use epics_rs::asyn::trace::TraceMask;
use epics_rs::motor::controller_port::index;
use epics_rs::motor::devices::{
    motor_interface_device, register_motor_interface_device, register_motor_interface_driver,
};
use motor_common::drv_motor_asyn::{DrvGet, DrvSet};
use motor_common::iocsh::{arg_int_opt, arg_int_req, arg_str_req, req_int, req_string};
use motor_common::model1::c_int;
pub use motor_common::model2::drv_asyn_motor_configure_command;
use motor_common::model2::{AxisLog, Model2, Model2Axis};
use motor_common::{
    Controller, ControllerParams, ControllerReport, TraceUser, asyn_user, register_port_with,
};

use crate::motorsim::{
    DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_LOW_LIMIT, DEFAULT_START, Deferral, DrvMotorSimAxis,
    MotorSimAxis, sim_log,
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

/// An axis motorSimCreate built, as its device support, drvMotorAsyn's
/// writes and its log reach it.
struct SimAxis<A> {
    card: i32,
    signal: i32,
    motor: Arc<Mutex<A>>,
    setter: Arc<Mutex<dyn DrvSet>>,
    getter: Arc<dyn DrvGet>,
    log: Arc<AxisLog>,
}

impl SimAxis<DrvMotorSimAxis> {
    fn new(axis: DrvMotorSimAxis, card: i32, signal: i32) -> Self {
        let log = axis.log();
        let getter = axis.getter();
        let motor = Arc::new(Mutex::new(axis));
        Self {
            card,
            signal,
            motor: motor.clone(),
            setter: motor,
            getter,
            log,
        }
    }
}

/// drvMotorSim's `drv`: the axes in its `pFirst` list's creation order
/// (drvMotorSim.c:719-735), the one motorSimThread that runs them
/// (drvMotorSim.c:121, 790), which processes every axis from its creation
/// (drvMotorSim.c:670-678), opened or not, the driver's own print and its
/// deferral.
struct SimDriver<A = DrvMotorSimAxis> {
    created: Mutex<Vec<SimAxis<A>>>,
    poller: Arc<Controller>,
    log: Arc<AxisLog>,
    deferral: Arc<Deferral>,
}

fn find<A>(created: &[SimAxis<A>], card: i32, signal: i32) -> Option<&SimAxis<A>> {
    created
        .iter()
        .find(|a| a.card == card && a.signal == signal)
}

impl<A: AsynMotor + 'static> SimAxis<A> {
    /// The axis as drvAsynMotorConfigure serves it.
    fn model2(&self) -> Model2Axis {
        Model2Axis {
            motor: self.motor.clone(),
            setter: self.setter.clone(),
            getter: self.getter.clone(),
            log: self.log.clone(),
        }
    }
}

impl<A: AsynMotor + 'static> SimDriver<A> {
    fn new() -> Self {
        Self {
            created: Mutex::new(Vec::new()),
            poller: Arc::new(Controller::new()),
            log: Arc::new(AxisLog::with_own(sim_log)),
            deferral: Arc::default(),
        }
    }

    /// motorSimCreateAxis (drvMotorSim.c:693-760): the axis joins `pFirst`,
    /// which motorProcTask walks each pass.
    fn create(&self, axis: SimAxis<A>) -> Result<(), String> {
        let (card, signal, motor) = (axis.card, axis.signal, axis.motor.clone());
        self.created
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(axis);
        let user = TraceUser::MotorInterface {
            device: DEV_MOTOR_SIM.choice,
            driver: DRIVER,
            card,
            signal,
        };
        self.poller.install(user, motor).map(drop)
    }

    /// devMotorSim's open of the axis (drvMotorSim.c:205-214), from a record's
    /// init_record. C's callback then scanOnce's before scanInit and
    /// segfaults (a C defect); here a post before the gate only sets
    /// needUpdate.
    fn open(&self, card: i32, signal: i32) -> bool {
        let created = self.created.lock().unwrap_or_else(|e| e.into_inner());
        find(&created, card, signal).is_some()
    }
}

impl SimDriver {
    /// C `motorAxisReport` (drvMotorSim.c:132-166).
    fn report(&self, level: i32) -> String {
        let mut out = String::new();
        let created = self.created.lock().unwrap_or_else(|e| e.into_inner());
        for SimAxis {
            card,
            signal,
            motor,
            ..
        } in created.iter()
        {
            let _ = writeln!(
                out,
                "Found driver for motorSim card {card}, axis {signal}, mutex {:p}",
                Arc::as_ptr(motor)
            );
            if level > 0 {
                motor
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .report(&mut out);
            }
        }
        out
    }
}

/// `motorSimCreate(card, signal, lowLimit, hiLimit, home, nCards, nSignals,
/// startPosn)`: axes `signal..signal+nSignals` of cards `card..card+nCards`
/// for `DTYP "Motor Simulation"`, `OUT "#C card S signal @motorSim"`.
pub fn motor_sim_create_command() -> CommandDef {
    let driver = Arc::new(SimDriver::new());
    MOTOR_SIM.set_driver_log(driver.log.clone());
    register_motor_interface_device(&DEV_MOTOR_SIM);
    let opener = driver.clone();
    register_motor_interface_driver(DRIVER, move |card, signal| opener.open(card, signal));
    let reporter = driver.clone();
    MOTOR_SIM.register_report(move |level| reporter.report(level));
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
            driver.log.print(
                TraceMask::FLOW,
                format_args!(
                    "Creating motor simulator: card: {card}, axis: {signal}, hi: {hi}, low {low}, home: {home}, ncards: {num_cards}, naxis: {num_signals}"
                ),
            );
            // The first motorSimCreate starts drvMotorSim's one
            // motorSimThread, whose motorProcTask runs every axis each DELTA,
            // moving or idle (drvMotorSim.c:657-690, 788-800).
            driver.poller.start(ctx, POLL_MS, POLL_MS, 0);
            for c in card..card.saturating_add(num_cards) {
                for s in signal..signal.saturating_add(num_signals) {
                    let exists = {
                        let created = driver.created.lock().unwrap_or_else(|e| e.into_inner());
                        find(&created, c, s).map(|axis| axis.log.clone())
                    };
                    if let Some(log) = exists {
                        log.print(
                            TraceMask::ERROR,
                            format_args!("Motor for card {c}, signal {s} already exists"),
                        );
                        continue;
                    }
                    let axis = SimAxis::new(
                        DrvMotorSimAxis::new(
                            c,
                            s,
                            low as f64,
                            hi as f64,
                            home as f64,
                            driver.deferral.clone(),
                        ),
                        c,
                        s,
                    );
                    let log = axis.log.clone();
                    driver.create(axis)?;
                    log.print(
                        TraceMask::FLOW,
                        format_args!("Created motor for card {c}, signal {s} OK"),
                    );
                }
                let created = driver.created.lock().unwrap_or_else(|e| e.into_inner());
                // Model2::open indexes by axis, so a card serves the axes
                // from 0 up to its first gap; drvAsynMotorConfigure installs
                // them on the driver's poller.
                let axes: Vec<_> = (0..)
                    .map_while(|s| find(&created, c, s).map(SimAxis::model2))
                    .collect();
                MOTOR_SIM.set_card_on(c, axes, driver.poller.clone());
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C `motorSimController::report` (motorSimDriver.cpp:111-150), over the
/// axes [`MotorSimHolder`] keeps.
struct MotorSimReport {
    holder: Arc<MotorSimHolder>,
}

impl ControllerParams for MotorSimReport {
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let port = ctx.port_name();
        let _ = writeln!(
            out,
            "Simulation motor driver {port}, numAxes={}",
            ctx.num_axes()
        );
        let axes = self.holder.axes.lock().unwrap_or_else(|e| e.into_inner());
        for addr in 0..ctx.num_axes() as i32 {
            let _ = writeln!(out, "  axis {addr}");
            let Some(axis) = axes.get(&MotorSimHolder::axis_key(port, addr.into())) else {
                continue;
            };
            if level > 0 {
                // C's locals stay 0.0 when getDoubleParam finds no value.
                let soft = |reason| ctx.float64_param(addr, reason).unwrap_or(0.0);
                let soft_limits = (soft(index::LOW_LIMIT), soft(index::HIGH_LIMIT));
                let axis = axis.lock().unwrap_or_else(|e| e.into_inner());
                axis.report(out, soft_limits);
            }
        }
        drop(axes);
        ctx.report(out, level);
    }
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
                let report = MotorSimReport {
                    holder: holder.clone(),
                };
                register_port_with(ctx, &port_name, num_axes as i32, report)?;
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

    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::interfaces::motor::{MotorStatus, PollFailure};
    use epics_rs::asyn::user::AsynUser;
    use epics_rs::base::runtime::task::{BlockingBridge, sleep};
    use epics_rs::base::server::database::PvDatabase;
    use epics_rs::base::server::ioc_app::DeviceSupportContext;
    use epics_rs::motor::devices::device_support_factory;
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

    /// C `motorSimController::report` (motorSimDriver.cpp:111-150): the
    /// driver line and each axis, at level 1 its trajectory, limits, home,
    /// encoder offset and the port's soft-limit params, before the base.
    #[epics_rs::base::epics_test]
    async fn the_port_report_prints_the_motorsim_lines() {
        use epics_rs::motor::controller_port::index;
        use motor_common::testing::{port_report, run_command};

        let holder = MotorSimHolder::new();
        let args = vec![
            ArgValue::String("simReport".into()),
            ArgValue::Int(2),
            ArgValue::Int(0),
            ArgValue::Int(0),
        ];
        run_command(holder.create_controller_command(), args).await;
        let args = ["simReport"]
            .map(|p| ArgValue::String(p.into()))
            .into_iter()
            .chain([0, 100, -100, 5, 3].map(ArgValue::Int))
            .collect();
        run_command(holder.config_axis_command(), args).await;
        let port = epics_rs::asyn::manager::PortManager::global()
            .find_port_handle("simReport")
            .unwrap();
        port.write_float64_blocking(index::HIGH_LIMIT, 0, 50.0)
            .unwrap();
        port.write_float64_blocking(index::LOW_LIMIT, 0, -20.0)
            .unwrap();

        let text = port_report("simReport", 0);
        assert!(
            text.contains("Simulation motor driver simReport, numAxes=2\n  axis 0\n  axis 1\n"),
            "{text}"
        );
        let text = port_report("simReport", 1);
        let lines: Vec<&str> = text.lines().collect();
        let at = lines
            .iter()
            .position(|l| *l == "Simulation motor driver simReport, numAxes=2")
            .unwrap_or_else(|| panic!("{text}"));
        let axis0 = &lines[at + 1..at + 9];
        assert_eq!(axis0[0], "  axis 0");
        assert!(
            axis0[1].starts_with(
                "  Current position = 0.000000, velocity = 0.000000 at current time: "
            ),
            "{text}"
        );
        assert!(
            axis0[2].starts_with(
                "  Destination posn = 0.000000, velocity = 0.000000 at desination time:  "
            ),
            "{text}"
        );
        assert_eq!(
            axis0[3..],
            [
                "    Hard limits: -100.000000, 100.000000",
                "           Home: 5.000000",
                "    Enc. offset: 3.000000",
                "    Soft limits: -20.000000, 50.000000",
                "  axis 1",
            ],
            "{text}"
        );
    }

    /// drvMotorSim's one motorSimThread (drvMotorSim.c:790) polls the axes
    /// of every motorSimCreate, whether drvAsynMotorConfigure or a
    /// devMotorSim record opens them.
    #[epics_rs::base::epics_test]
    async fn create_configure_and_record_init_share_one_poller() {
        let create = motor_sim_create_command();
        for card in 0..2 {
            let args = [card, 0, -100, 100, 0, 1, 3, 0].map(ArgValue::Int).to_vec();
            assert_eq!(pollers_after(create.clone(), args).await, 1);
        }
        let configure = drv_asyn_motor_configure_command(vec![&MOTOR_SIM]);
        let args = vec![
            ArgValue::String("simShared".into()),
            ArgValue::String(DRIVER.into()),
            ArgValue::Int(0),
            ArgValue::Int(3),
        ];
        assert_eq!(pollers_after(configure, args).await, 1);
        // devMotorSim init_record on an axis drvAsynMotorConfigure opened
        // and on one only the record opens.
        for out in ["#C0 S1 @motorSim", "#C1 S2 @motorSim"] {
            let init = std::thread::spawn(move || {
                let ctx = DeviceSupportContext {
                    record_type: "motor",
                    dtyp: DEV_MOTOR_SIM.choice,
                    inp: "",
                    out,
                };
                device_support_factory()(&ctx).is_some()
            });
            while !init.is_finished() {
                sleep(Duration::from_millis(5)).await;
            }
            assert!(init.join().unwrap(), "{out}");
        }
        assert_eq!(pollers(), 1);
    }

    /// C `motorAxisReport` (drvMotorSim.c:132-166), the drvet `dbior`
    /// runs: each axis in creation order, at level 1 its trajectory,
    /// limits and paramLib, whichever of drvAsynMotorConfigure or a
    /// devMotorSim record opened it.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_the_drv_motor_sim_axes_in_creation_order() {
        use motor_common::testing::{driver_report, run_command};

        let create = motor_sim_create_command();
        for card in [1, 0] {
            let args = [card, 0, -100, 100, 0, 1, 1, 0].map(ArgValue::Int).to_vec();
            run_command(create.clone(), args).await;
        }
        let args = vec![
            ArgValue::String("simDrvReport".into()),
            ArgValue::String(DRIVER.into()),
            ArgValue::Int(1),
            ArgValue::Int(1),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&MOTOR_SIM]), args).await;
        let init = std::thread::spawn(|| {
            let ctx = DeviceSupportContext {
                record_type: "motor",
                dtyp: DEV_MOTOR_SIM.choice,
                inp: "",
                out: "#C0 S0 @motorSim",
            };
            device_support_factory()(&ctx).is_some()
        });
        while !init.is_finished() {
            sleep(Duration::from_millis(5)).await;
        }
        assert!(init.join().unwrap());
        sleep(Duration::from_millis(350)).await;

        let text = driver_report(DRIVER, 0);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 2, "{text}");
        for (line, card) in lines.iter().zip([1, 0]) {
            let head = format!("Found driver for motorSim card {card}, axis 0, mutex 0x");
            assert!(line.starts_with(&head), "{text}");
        }
        assert_ne!(lines[0][46..], lines[1][46..], "{text}");

        let text = driver_report(DRIVER, 1);
        let blocks: Vec<&str> = text.split("Found driver").skip(1).collect();
        assert_eq!(blocks.len(), 2, "{text}");
        for block in blocks {
            let lines: Vec<&str> = block.lines().collect();
            assert!(
                lines[1].starts_with(
                    "Current position = 0.000000, velocity = 0.000000 at current time: "
                ),
                "{text}"
            );
            assert!(
                lines[2].starts_with(
                    "Destination posn = 0.000000, velocity = 0.000000 at desination time:  "
                ),
                "{text}"
            );
            assert_eq!(
                lines[3..9],
                [
                    "Hard limits: -100.000000, 100.000000",
                    "Soft limits: 0.000000, 0.000000",
                    "Number of parameters is: 28",
                    "Parameter 0 is a double, value 0.000000",
                    "Parameter 1 is undefined",
                    "Parameter 2 is undefined",
                ],
                "{text}"
            );
            assert_eq!(
                lines[3 + 3 + 12],
                "Parameter 12 is an integer, value 1",
                "{text}"
            );
            assert_eq!(lines.len(), 3 + 3 + 28, "{text}");
        }
    }

    /// motorProcTask processes every created axis (drvMotorSim.c:670-678),
    /// so one that neither a record nor drvAsynMotorConfigure opened dumps
    /// the parameters motorSimProcess sets (:646-653) too.
    #[epics_rs::base::epics_test]
    async fn dbior_dumps_an_axis_nothing_opened() {
        use motor_common::testing::{driver_report, run_command};

        let args = [7, 0, -100, 100, 0, 1, 1, 0].map(ArgValue::Int).to_vec();
        run_command(motor_sim_create_command(), args).await;
        sleep(Duration::from_millis(350)).await;

        let text = driver_report(DRIVER, 1);
        let block = text
            .split("Found driver")
            .find(|b| b.starts_with(" for motorSim card 7, axis 0,"))
            .unwrap_or_else(|| panic!("{text}"));
        let lines: Vec<&str> = block.lines().collect();
        assert_eq!(
            lines[6], "Parameter 0 is a double, value 0.000000",
            "{text}"
        );
        assert_eq!(
            lines[6 + 12],
            "Parameter 12 is an integer, value 1",
            "{text}"
        );
    }

    /// drvMotorSim.c:306-351, 389-409, 467, 514, 535, 562: once
    /// drvAsynMotorConfigure sets an axis' log, its sets and motions print
    /// their FLOW lines on the axis' user, without a newline as C's.
    #[epics_rs::base::epics_test]
    async fn configured_axes_print_the_c_flow_lines() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};
        use motor_common::testing::run_command;

        let args = [0, 0, -100, 100, 0, 1, 2, 0].map(ArgValue::Int).to_vec();
        run_command(motor_sim_create_command(), args).await;
        let args = vec![
            ArgValue::String("simFlow".into()),
            ArgValue::String("motorSim".into()),
            ArgValue::Int(0),
            ArgValue::Int(2),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&MOTOR_SIM]), args).await;
        let path = std::env::temp_dir().join(format!("simFlow-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some("simFlow"), TraceMask::FLOW);
        trace.set_device_trace_mask("simFlow", 0, TraceMask::FLOW);
        trace.set_device_trace_info_mask("simFlow", 0, TraceInfoMask::empty());
        trace.set_device_trace_file("simFlow", 0, TraceFile::File(file));
        let port = manager.find_port_handle("simFlow").unwrap();
        std::thread::spawn(move || {
            let float = |reason, value| port.write_float64_blocking(reason, 0, value).unwrap();
            for (reason, value) in [(100, 2.0), (101, 0.5), (102, 4.0), (104, 5.0), (105, 1.5)] {
                float(reason, value);
            }
            port.write_int32_blocking(107, 0, 1).unwrap();
            port.write_int32_blocking(106, 0, 1).unwrap();
            for (reason, value) in [
                (0, 3.0),
                (1, 2.0),
                (2, 2.0),
                (3, -50.0),
                (4, 50.0),
                (5, 0.1),
                (6, 0.2),
                (7, 0.3),
                (8, 1.0),
                (26, 1.0),
            ] {
                float(reason, value);
            }
            for (reason, value) in [(0, 4), (3, -40), (4, 40), (8, 0), (26, 0)] {
                port.write_int32_blocking(reason, 0, value).unwrap();
            }
        })
        .join()
        .unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let mut rest = text.as_str();
        for line in [
            "Set card 0, axis 0 move to 5.000000, min vel=0.500000, max_vel=2.000000, accel=4.000000",
            "Set card 0, axis 0 move with velocity of 1.500000, accel=4.000000",
            "Set card 0, axis 0 to stop with accel=4.000000",
            "Set card 0, axis 0 to home FORWARDS, min vel=0.500000, max_vel=2.000000, accel=4.000000",
            "Set card 0, axis 0 to position 3.000000",
            "Set card 0, axis 0 resolution to 2.000000",
            "Set card 0, axis 0 to enc. ratio to 2.000000",
            "Set card 0, axis 0 low limit to -50.000000",
            "Set card 0, axis 0 high limit to 50.000000",
            "Set card 0, axis 0 pgain to 0.100000",
            "Set card 0, axis 0 igain to 0.200000",
            "Set card 0, axis 0 dgain to 0.300000",
            "Set card 0, axis 0 closed loop to ON",
            "Setting Deferred Move flag on PMAC card 0\n",
            "Set card 0, axis 0 to position 4",
            "Set card 0, axis 0 low limit to -40",
            "Set card 0, axis 0 high limit to 40",
            "Set card 0, axis 0 closed loop to OFF",
            "Clearing Deferred Move flag on PMAC card 0\n",
        ] {
            let at = rest
                .find(line)
                .unwrap_or_else(|| panic!("{line:?} not in order in {text}"));
            rest = &rest[at + line.len()..];
        }
    }

    /// Polls `motorProcTask` would process; never moves.
    struct Counted(Arc<Mutex<Vec<Instant>>>);

    impl AsynMotor for Counted {
        fn move_absolute(
            &mut self,
            _: &AsynUser,
            _: f64,
            _: f64,
            _: f64,
            _: f64,
        ) -> AsynResult<()> {
            Ok(())
        }
        fn home(&mut self, _: &AsynUser, _: f64, _: f64, _: f64, _: bool) -> AsynResult<()> {
            Ok(())
        }
        fn stop(&mut self, _: &AsynUser, _: f64) -> AsynResult<()> {
            Ok(())
        }
        fn set_position(&mut self, _: &AsynUser, _: f64) -> AsynResult<()> {
            Ok(())
        }
        fn poll_failure(&self) -> PollFailure {
            PollFailure::NONE
        }
        fn poll(&mut self, _: &AsynUser) -> AsynResult<MotorStatus> {
            self.0.lock().unwrap().push(Instant::now());
            Ok(MotorStatus {
                done: true,
                ..MotorStatus::default()
            })
        }
    }

    /// motorSimCreate, drvMotorAsyn and devMotorSim all reach one
    /// drvMotorSim axis, which motorProcTask still processes once a pass
    /// (drvMotorSim.c:657-680).
    #[epics_rs::base::epics_test]
    async fn an_axis_opened_by_both_paths_is_installed_once() {
        static ONCE: Model2 = Model2::new("onceSim", 0);
        let polls = Arc::new(Mutex::new(Vec::new()));
        let counted = Arc::new(Mutex::new(Counted(polls.clone())));
        let driver = Arc::new(SimDriver::<Counted>::new());
        // devMotorSim's open reaches only the axis' motor.
        let unused = DrvMotorSimAxis::new(0, 0, 0.0, 0.0, 0.0, Arc::default());
        let axis = SimAxis {
            card: 0,
            signal: 0,
            motor: counted,
            getter: unused.getter(),
            setter: Arc::new(Mutex::new(unused)),
            log: Arc::default(),
        };
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), BlockingBridge::capture());
        driver.poller.start(&ctx, POLL_MS, POLL_MS, 0);
        let model2 = axis.model2();
        let creator = driver.clone();
        let create = std::thread::spawn(move || creator.create(axis));
        while !create.is_finished() {
            sleep(Duration::from_millis(5)).await;
        }
        create.join().unwrap().unwrap();
        ONCE.set_card_on(0, vec![model2], driver.poller.clone());
        let args = vec![
            ArgValue::String("onceP".into()),
            ArgValue::String("onceSim".into()),
            ArgValue::Int(0),
            ArgValue::Int(1),
        ];
        assert_eq!(
            pollers_after(drv_asyn_motor_configure_command(vec![&ONCE]), args).await,
            1
        );
        let opener = driver.clone();
        let open = std::thread::spawn(move || opener.open(0, 0));
        while !open.is_finished() {
            sleep(Duration::from_millis(5)).await;
        }
        assert!(open.join().unwrap());
        polls.lock().unwrap().clear();
        sleep(Duration::from_millis(650)).await;
        let polls = polls.lock().unwrap().clone();
        assert!(polls.len() >= 4, "{polls:?}");
        let gaps: Vec<_> = polls.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(
            gaps.iter().all(|gap| *gap > Duration::from_millis(50)),
            "one poll a pass: {gaps:?}"
        );
    }
}
