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

/// drvMotorSim's `drv`: the axes and the one motorSimThread that runs them
/// (drvMotorSim.c:121, 790). An axis is polled once something opens it.
struct SimDriver {
    created: Mutex<SimAxes>,
    poller: Arc<Controller>,
}

impl SimDriver {
    /// devMotorSim's open of the axis (drvMotorSim.c:205-214), from a record's
    /// init_record. C's callback then scanOnce's before scanInit and
    /// segfaults (a C defect); here a post before the gate only sets
    /// needUpdate.
    fn open(&self, card: i32, signal: i32) -> bool {
        let motor = self
            .created
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&(card, signal))
            .cloned();
        let Some(motor) = motor else {
            return false;
        };
        let user = TraceUser::MotorInterface {
            device: DEV_MOTOR_SIM.choice,
            driver: DRIVER,
            card,
            signal,
        };
        self.poller.install(user, motor).is_ok()
    }
}

/// `motorSimCreate(card, signal, lowLimit, hiLimit, home, nCards, nSignals,
/// startPosn)`: axes `signal..signal+nSignals` of cards `card..card+nCards`
/// for `DTYP "Motor Simulation"`, `OUT "#C card S signal @motorSim"`.
pub fn motor_sim_create_command() -> CommandDef {
    let driver = Arc::new(SimDriver {
        created: Mutex::new(BTreeMap::new()),
        poller: Arc::new(Controller::new()),
    });
    register_motor_interface_device(&DEV_MOTOR_SIM);
    let opener = driver.clone();
    register_motor_interface_driver(DRIVER, move |card, signal| opener.open(card, signal));
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
            driver.poller.start(ctx, POLL_MS, POLL_MS, 0);
            let mut created = driver.created.lock().unwrap_or_else(|e| e.into_inner());
            for c in card..card.saturating_add(num_cards) {
                for s in signal..signal.saturating_add(num_signals) {
                    if created.contains_key(&(c, s)) {
                        println!("Motor for card {c}, signal {s} already exists");
                        continue;
                    }
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(
                        DrvMotorSimAxis::new(low as f64, hi as f64, home as f64),
                    ));
                    created.insert((c, s), motor);
                    println!("Created motor for card {c}, signal {s} OK");
                }
                // Model2::open indexes by axis, so a card serves the axes
                // from 0 up to its first gap; drvAsynMotorConfigure installs
                // them on the driver's poller.
                let axes = (0..).map_while(|s| created.get(&(c, s)).cloned()).collect();
                MOTOR_SIM.set_card_on(c, axes, driver.poller.clone());
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

    /// drvMotorAsyn and devMotorSim open one drvMotorSim axis, which
    /// motorProcTask still processes once a pass (drvMotorSim.c:657-680).
    #[epics_rs::base::epics_test]
    async fn an_axis_opened_by_both_paths_is_installed_once() {
        static ONCE: Model2 = Model2::new("onceSim", 0);
        let polls = Arc::new(Mutex::new(Vec::new()));
        let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(Counted(polls.clone())));
        let driver = Arc::new(SimDriver {
            created: Mutex::new(BTreeMap::from([((0, 0), motor.clone())])),
            poller: Arc::new(Controller::new()),
        });
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), BlockingBridge::capture());
        driver.poller.start(&ctx, POLL_MS, POLL_MS, 0);
        ONCE.set_card_on(0, vec![motor], driver.poller.clone());
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
