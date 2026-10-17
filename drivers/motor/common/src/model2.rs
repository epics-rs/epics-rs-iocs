//! The startup interface of a C model-2 (`drvMotorAsyn`) driver: the driver's
//! own `Config` command builds a card's axes, and
//! `drvAsynMotorConfigure(portName, driverName, card, numAxes)` opens them
//! into asyn port `portName`, which records reach as `@asyn(portName,axis)`.

use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::motor::device_state::Model2Params;
use epics_rs::motor::devices::{model2_params, motor_interface_params};

use crate::drv_motor_asyn::{Axis, DrvMotorAsyn, DrvSet};
use crate::iocsh::{arg_int_req, arg_str_req};
use crate::model1::c_int;
use crate::{Controller, TraceUser, register_drv_motor_asyn_port};

/// The axes a card's `Config` built, the poller they go on, and the log
/// users `drvAsynMotorConfigure` connects them.
struct Card {
    card: i32,
    axes: Vec<Arc<Mutex<dyn AsynMotor>>>,
    poller: Poller,
    logs: Vec<Arc<AxisLog>>,
    /// The drvAsynMotorConfigure ports that opened the card.
    ports: Vec<String>,
    setters: Vec<Arc<Mutex<dyn DrvSet>>>,
}

/// The `print`/`logParam` pair a driver keeps for an axis, or for itself,
/// which `motorAxisSetLog` replaces: `drvAsynMotorConfigure` gives an
/// opened axis `logFunc` on the user at `(port, addr)` (drvMotorAsyn.c:413)
/// and the driver `logFunc` on the unconnected `defaultAsynUser` (:421).
#[derive(Debug, Default)]
pub struct AxisLog {
    user: Mutex<LogUser>,
    /// The driver's own print until then; without one, `logFunc` on
    /// `defaultAsynUser`.
    own: Option<fn(TraceMask, std::fmt::Arguments<'_>)>,
}

#[derive(Debug, Default, Clone, PartialEq)]
enum LogUser {
    #[default]
    Own,
    Default,
    At(String, i32),
}

impl AxisLog {
    /// A log that prints with `own` until `drvAsynMotorConfigure` sets it.
    pub fn with_own(own: fn(TraceMask, std::fmt::Arguments<'_>)) -> Self {
        Self {
            user: Mutex::default(),
            own: Some(own),
        }
    }

    fn set(&self, user: LogUser) {
        *self.user.lock().unwrap_or_else(|e| e.into_inner()) = user;
    }

    fn attach(&self, port: &str, addr: i32) {
        self.set(LogUser::At(port.to_owned(), addr));
    }

    fn user(&self) -> LogUser {
        self.user.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }

    /// The driver's `PRINT(param, MOTOR_ERROR, ...)`, which `logFunc` turns
    /// into `asynPrint` on the user (drvMotorAsyn.c:793).
    pub fn print(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        let user = self.user();
        let (port, addr) = match (&user, self.own) {
            (LogUser::Own, Some(own)) => return own(mask, args),
            (LogUser::At(port, addr), _) => (Some(port.as_str()), Some(*addr)),
            _ => (None, None),
        };
        PortManager::global()
            .trace_manager()
            .print(port, addr, 0, mask, args);
    }
}

/// drvMotorAsyn has no task of its own; each driver's task polls its axes.
#[derive(Clone)]
enum Poller {
    /// `drvAsynMotorConfigure` starts one over the card's axes at these
    /// periods, standing in for the driver's task.
    Configured {
        moving_poll_ms: u64,
        idle_poll_ms: u64,
    },
    /// The driver's own, started by its `Config` (motorSim's
    /// `motorSimThread`, drvMotorSim.c:790); the axes join it.
    Driver(Arc<Controller>),
}

impl Poller {
    /// The controller the card's axes are installed into.
    fn controller(&self) -> Arc<Controller> {
        match self {
            Self::Configured { .. } => Arc::new(Controller::new()),
            Self::Driver(poller) => poller.clone(),
        }
    }
}

/// One model-2 driver: the `motorAxisDrvSET_t` name `drvAsynMotorConfigure`
/// finds it by, the fast polls its poller forces after a wakeup, and the
/// axes each card's `Config` built.
pub struct Model2 {
    name: &'static str,
    forced_fast_polls: u32,
    cards: Mutex<Vec<Card>>,
    /// The driver's own log, `motorAxisSetLog(NULL, ...)`'s.
    log: Mutex<Option<Arc<AxisLog>>>,
}

impl Model2 {
    pub const fn new(name: &'static str, forced_fast_polls: u32) -> Self {
        Self {
            name,
            forced_fast_polls,
            cards: Mutex::new(Vec::new()),
            log: Mutex::new(None),
        }
    }

    /// The log the driver prints on for itself.
    pub fn set_driver_log(&self, log: Arc<AxisLog>) {
        *self.log.lock().unwrap_or_else(|e| e.into_inner()) = Some(log);
    }

    /// The driver's `motorAxisReport`, the drvet `report` `dbior` runs: C
    /// exports the `motorAxisDrvSET_t` under its name as a drvet
    /// (drvMotorSim.c:71).
    pub fn register_report(&'static self, report: impl Fn(i32) -> String + Send + Sync + 'static) {
        crate::register_driver_report(self.name, report);
    }

    /// What `Config` built for `card`, polled at `moving_poll_ms` and
    /// `idle_poll_ms`, replacing an earlier `Config` of it.
    pub fn set_card(
        &self,
        card: i32,
        axes: Vec<Arc<Mutex<dyn AsynMotor>>>,
        moving_poll_ms: u64,
        idle_poll_ms: u64,
    ) {
        self.put_card(Card {
            card,
            axes,
            poller: Poller::Configured {
                moving_poll_ms,
                idle_poll_ms,
            },
            logs: Vec::new(),
            ports: Vec::new(),
            setters: Vec::new(),
        });
    }

    /// What `Config` built for `card`, polled by the driver's own `poller`,
    /// replacing an earlier `Config` of it.
    pub fn set_card_on(
        &self,
        card: i32,
        axes: Vec<Arc<Mutex<dyn AsynMotor>>>,
        poller: Arc<Controller>,
    ) {
        self.put_card(Card {
            card,
            axes,
            poller: Poller::Driver(poller),
            logs: Vec::new(),
            ports: Vec::new(),
            setters: Vec::new(),
        });
    }

    /// The log user of each of `card`'s axes, by axis; set after
    /// `set_card`/`set_card_on`.
    pub fn set_axis_logs(&self, card: i32, logs: Vec<Arc<AxisLog>>) {
        let mut cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(c) = cards.iter_mut().find(|c| c.card == card) {
            c.logs = logs;
        }
    }

    /// The `motorAxisSetInteger`/`motorAxisSetDouble` of each of `card`'s
    /// axes, by axis, for the port's writes; set after
    /// `set_card`/`set_card_on`.
    pub fn set_axis_setters(&self, card: i32, setters: Vec<Arc<Mutex<dyn DrvSet>>>) {
        let mut cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(c) = cards.iter_mut().find(|c| c.card == card) {
            c.setters = setters;
        }
    }

    fn axis_setter(&self, card: i32, axis: i32) -> Option<Arc<Mutex<dyn DrvSet>>> {
        let cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        let card = cards.iter().find(|c| c.card == card)?;
        card.setters.get(usize::try_from(axis).ok()?).cloned()
    }

    fn axis_log(&self, card: i32, axis: i32) -> Option<Arc<AxisLog>> {
        let cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        let card = cards.iter().find(|c| c.card == card)?;
        card.logs.get(usize::try_from(axis).ok()?).cloned()
    }

    fn put_card(&self, card: Card) {
        let mut cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        cards.retain(|c| c.card != card.card);
        cards.push(card);
    }

    /// Change the poll periods `card`'s axes will be installed with; false
    /// for a card no `Config` built or one on the driver's own poller.
    pub fn set_poll_periods(
        &self,
        card: i32,
        moving_ms: Option<u64>,
        idle_ms: Option<u64>,
    ) -> bool {
        let mut cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        let Some(Poller::Configured {
            moving_poll_ms,
            idle_poll_ms,
        }) = cards
            .iter_mut()
            .find(|c| c.card == card)
            .map(|c| &mut c.poller)
        else {
            return false;
        };
        *moving_poll_ms = moving_ms.unwrap_or(*moving_poll_ms);
        *idle_poll_ms = idle_ms.unwrap_or(*idle_poll_ms);
        true
    }

    /// The poller of `card`, or `None` for an unconfigured card.
    fn poller(&self, card: i32) -> Option<Poller> {
        let cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        Some(cards.iter().find(|c| c.card == card)?.poller.clone())
    }

    /// The paramLib of `card`'s axis `axis`, which the driver's report dumps:
    /// the one table whether a devMotorSim-style record or
    /// drvAsynMotorConfigure opened the axis, `None` until one did.
    pub fn params(&self, card: i32, axis: i32) -> Option<Model2Params> {
        motor_interface_params(self.name, card, axis).or_else(|| {
            let cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
            let card = cards.iter().find(|c| c.card == card)?;
            card.ports.iter().find_map(|port| model2_params(port, axis))
        })
    }

    fn add_port(&self, card: i32, port: &str) {
        let mut cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(card) = cards.iter_mut().find(|c| c.card == card) {
            card.ports.push(port.to_owned());
        }
    }

    /// The driver's `open(card, axis)`: `None` for an unconfigured card or an
    /// axis it does not have.
    fn open(&self, card: i32, axis: i32) -> Option<Arc<Mutex<dyn AsynMotor>>> {
        let cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        let card = cards.iter().find(|c| c.card == card)?;
        card.axes.get(usize::try_from(axis).ok()?).cloned()
    }
}

/// `drvAsynMotorConfigure(portName, driverName, cardNum, numAxes)` over the
/// model-2 drivers this IOC links, printing what C's does (drvMotorAsyn.c).
pub fn drv_asyn_motor_configure_command(drivers: Vec<&'static Model2>) -> CommandDef {
    CommandDef::new(
        "drvAsynMotorConfigure",
        vec![
            arg_str_req("portName"),
            arg_str_req("driverName"),
            arg_int_req("cardNum"),
            arg_int_req("numAxes"),
        ],
        "drvAsynMotorConfigure(portName, driverName, cardNum, numAxes)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let text = |i: usize| match args.get(i) {
                Some(ArgValue::String(s)) => s.to_str().map(str::to_owned),
                _ => Ok(String::new()),
            };
            let port_name = text(0)?;
            let driver_name = text(1)?;
            let card = c_int(args, 2);
            let num_axes = c_int(args, 3);
            let Some(driver) = drivers.iter().find(|d| d.name == driver_name) else {
                println!("drvAsynMotorConfigure ERROR: Can't find driver: {driver_name}");
                return Ok(CommandOutcome::Continue);
            };
            let Ok(claim) = claim_port(&port_name) else {
                println!("drvAsynMotorConfigure ERROR: Can't register port");
                return Ok(CommandOutcome::Continue);
            };
            register_drv_motor_asyn_port(&port_name, num_axes)?;
            // The driver's one task polls the card's axes in turn.
            let poller = driver.poller(card);
            let controller = poller.as_ref().map(Poller::controller);
            let mut axes = Vec::new();
            for addr in 0..num_axes {
                let (Some(controller), Some(motor)) = (&controller, driver.open(card, addr)) else {
                    println!("drvAsynMotorConfigure: Failed to open axis {addr}");
                    axes.push(None);
                    continue;
                };
                let user = TraceUser::Model2 {
                    port: port_name.clone(),
                    addr,
                };
                let wakeup = controller.install(user, motor.clone())?;
                axes.push(Some(Axis::new(
                    motor,
                    driver.axis_setter(card, addr),
                    wakeup,
                )));
                if let Some(log) = driver.axis_log(card, addr) {
                    log.attach(&port_name, addr);
                }
            }
            driver.add_port(card, &port_name);
            if let Some(log) = &*driver.log.lock().unwrap_or_else(|e| e.into_inner()) {
                log.set(LogUser::Default);
            }
            let port = DrvMotorAsyn::new(&port_name, axes);
            let (rt, _thread) = create_port_runtime(&claim, port, RuntimeConfig::default())
                .map_err(|e| e.to_string())?;
            claim
                .publish(
                    rt.port_handle().clone(),
                    ctx.db().scan_control().interrupt_gate(),
                )
                .map_err(|e| e.to_string())?;
            if let (
                Some(controller),
                Some(Poller::Configured {
                    moving_poll_ms,
                    idle_poll_ms,
                }),
            ) = (controller, poller)
            {
                controller.start(ctx, moving_poll_ms, idle_poll_ms, driver.forced_fast_polls);
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::motor::sim_motor::SimMotor;

    static DRIVER: Model2 = Model2::new("test", 0);

    static REPORTING: Model2 = Model2::new("motorTestReport", 0);

    /// C `dbior` runs the `motorAxisDrvSET_t` exported as a drvet.
    #[test]
    fn dbior_reaches_the_model2_axis_report() {
        use epics_rs::base::server::driver_support::find_driver_support;
        REPORTING.register_report(|level| format!("level {level}\n"));
        let drvet = find_driver_support("motorTestReport").unwrap();
        assert_eq!(drvet.report(1).unwrap(), "level 1\n");
    }

    fn axis() -> Arc<Mutex<dyn AsynMotor>> {
        Arc::new(Mutex::new(SimMotor::default()))
    }

    #[test]
    fn open_finds_only_built_axes_and_takes_later_poll_periods() {
        DRIVER.set_card(3, vec![axis(), axis()], 100, 500);
        assert!(DRIVER.open(3, 1).is_some());
        assert!(DRIVER.open(3, 2).is_none());
        assert!(DRIVER.open(3, -1).is_none());
        assert!(DRIVER.open(0, 0).is_none());
        assert!(DRIVER.poller(0).is_none());
        assert!(DRIVER.set_poll_periods(3, Some(50), None));
        assert!(!DRIVER.set_poll_periods(0, Some(50), None));
        assert!(matches!(
            DRIVER.poller(3),
            Some(Poller::Configured {
                moving_poll_ms: 50,
                idle_poll_ms: 500
            })
        ));
    }
}

#[cfg(all(test, feature = "testing", target_os = "linux"))]
mod one_task {
    use super::*;
    use crate::testing::pollers_after;
    use epics_rs::motor::sim_motor::SimMotor;

    static DRIVER: Model2 = Model2::new("oneTask", 0);

    /// drvMotorAsyn's driver task polls every axis of the card in turn.
    #[epics_rs::base::epics_test]
    async fn drv_asyn_motor_configure_polls_the_card_s_axes_on_one_poller() {
        let axes = (0..3)
            .map(|_| Arc::new(Mutex::new(SimMotor::default())) as Arc<Mutex<dyn AsynMotor>>)
            .collect();
        DRIVER.set_card(0, axes, 100, 1000);
        let args = vec![
            ArgValue::String("oneTaskPort".into()),
            ArgValue::String("oneTask".into()),
            ArgValue::Int(0),
            ArgValue::Int(3),
        ];
        let cmd = drv_asyn_motor_configure_command(vec![&DRIVER]);
        assert_eq!(pollers_after(cmd, args).await, 1);
    }

    static OWN: Model2 = Model2::new("ownTask", 0);

    /// motorSimCreate starts the driver's one task and builds the card;
    /// drvAsynMotorConfigure only opens the axes onto it.
    #[epics_rs::base::epics_test]
    async fn a_driver_s_own_poller_is_the_only_one_after_configure() {
        let create = CommandDef::new(
            "ownCreate",
            vec![],
            "ownCreate()",
            |_: &[ArgValue], ctx: &CommandContext| {
                let poller = Arc::new(Controller::new());
                poller.start(ctx, 100, 1000, 0);
                let axes = (0..2)
                    .map(|_| Arc::new(Mutex::new(SimMotor::default())) as Arc<Mutex<dyn AsynMotor>>)
                    .collect();
                OWN.set_card_on(0, axes, poller);
                Ok(CommandOutcome::Continue)
            },
        );
        assert_eq!(pollers_after(create, vec![]).await, 1);
        let args = vec![
            ArgValue::String("ownTaskPort".into()),
            ArgValue::String("ownTask".into()),
            ArgValue::Int(0),
            ArgValue::Int(2),
        ];
        let cmd = drv_asyn_motor_configure_command(vec![&OWN]);
        assert_eq!(pollers_after(cmd, args).await, 1);
        assert!(!OWN.set_poll_periods(0, Some(50), None));
    }
}

#[cfg(all(test, feature = "testing", target_os = "linux"))]
mod axis_log {
    use super::*;
    use crate::testing::run_command;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::motor::sim_motor::SimMotor;

    static LOGGED: Model2 = Model2::new("logged", 0);

    /// drvMotorAsyn.c:395-413: each opened axis logs through the user at its
    /// address on the configured port; one that did not open keeps none.
    #[epics_rs::base::epics_test]
    async fn configure_hands_each_opened_axis_its_log_user() {
        let logs: Vec<Arc<AxisLog>> = (0..3).map(|_| Arc::default()).collect();
        let axis = || -> Arc<Mutex<dyn AsynMotor>> { Arc::new(Mutex::new(SimMotor::default())) };
        LOGGED.set_card(0, vec![axis(), axis()], 100, 1000);
        LOGGED.set_axis_logs(0, logs.clone());
        let args = vec![
            ArgValue::String("loggedPort".into()),
            ArgValue::String("logged".into()),
            ArgValue::Int(0),
            ArgValue::Int(3),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&LOGGED]), args).await;
        assert_eq!(logs[2].user(), LogUser::Own);

        let path = std::env::temp_dir().join(format!("model2-log-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some("loggedPort"), TraceMask::ERROR);
        trace.set_trace_file(Some("loggedPort"), TraceFile::File(file.clone()));
        for addr in 0..3 {
            trace.set_device_trace_file("loggedPort", addr, TraceFile::File(file.clone()));
        }
        logs[1].print(TraceMask::ERROR, format_args!("axis one\n"));
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(text.contains("axis one\n"), "{text}");
    }

    static OWNED: Model2 = Model2::new("owned", 0);
    static OWN_LINES: Mutex<Vec<String>> = Mutex::new(Vec::new());

    fn own(_: TraceMask, args: std::fmt::Arguments<'_>) {
        OWN_LINES.lock().unwrap().push(args.to_string());
    }

    /// drvMotorAsyn.c:413, :421: setLog replaces the driver's own print, for
    /// each opened axis and for the driver, and only those.
    #[epics_rs::base::epics_test]
    async fn configure_replaces_the_driver_s_own_print() {
        let logs: Vec<Arc<AxisLog>> = (0..2).map(|_| Arc::new(AxisLog::with_own(own))).collect();
        let driver_log = Arc::new(AxisLog::with_own(own));
        let axis = || -> Arc<Mutex<dyn AsynMotor>> { Arc::new(Mutex::new(SimMotor::default())) };
        OWNED.set_card(0, vec![axis()], 100, 1000);
        OWNED.set_axis_logs(0, logs.clone());
        OWNED.set_driver_log(driver_log.clone());
        for log in [&logs[0], &logs[1], &driver_log] {
            log.print(TraceMask::FLOW, format_args!("before"));
        }
        let args = vec![
            ArgValue::String("ownedPort".into()),
            ArgValue::String("owned".into()),
            ArgValue::Int(0),
            ArgValue::Int(2),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&OWNED]), args).await;
        for (log, line) in [
            (&logs[0], "axis"),
            (&logs[1], "unopened"),
            (&driver_log, "driver"),
        ] {
            log.print(TraceMask::FLOW, format_args!("{line}"));
        }
        assert_eq!(
            *OWN_LINES.lock().unwrap(),
            ["before", "before", "before", "unopened"]
        );
        assert_eq!(logs[0].user(), LogUser::At("ownedPort".into(), 0));
        assert_eq!(driver_log.user(), LogUser::Default);
    }
}

#[cfg(all(test, feature = "testing", target_os = "linux"))]
mod asyn_writes {
    use super::*;
    use crate::testing::run_command;
    use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
    use epics_rs::asyn::interfaces::motor::{MotorStatus, PollFailure};
    use epics_rs::asyn::port::DrvUserRequest;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::asyn::user::AsynUser;

    /// What the port called on the axis.
    #[derive(Default)]
    struct Recorder(Vec<String>);

    impl AsynMotor for Recorder {
        fn move_absolute(
            &mut self,
            _: &AsynUser,
            position: f64,
            min_velocity: f64,
            velocity: f64,
            acceleration: f64,
        ) -> AsynResult<()> {
            self.0.push(format!(
                "move {position} {min_velocity} {velocity} {acceleration}"
            ));
            Ok(())
        }
        fn home(&mut self, _: &AsynUser, _: f64, _: f64, _: f64, forward: bool) -> AsynResult<()> {
            self.0.push(format!("home {forward}"));
            Ok(())
        }
        fn stop(&mut self, _: &AsynUser, acceleration: f64) -> AsynResult<()> {
            self.0.push(format!("stop {acceleration}"));
            Ok(())
        }
        fn set_position(&mut self, _: &AsynUser, _: f64) -> AsynResult<()> {
            Ok(())
        }
        fn poll(&mut self, _: &AsynUser) -> AsynResult<MotorStatus> {
            Ok(MotorStatus::default())
        }
        fn poll_failure(&self) -> PollFailure {
            PollFailure::NONE
        }
    }

    /// A driver whose setInteger/setDouble know no function, as MM4000's
    /// `default:` (drvMM4000Asyn.c:384, :443).
    struct Unknown(Arc<AxisLog>);

    impl DrvSet for Unknown {
        fn set_integer(&mut self, _: &AsynUser, function: i32, _: i32) -> AsynResult<()> {
            self.0.print(
                TraceMask::ERROR,
                format_args!("motorAxisSetInteger: unknown function {function}\n"),
            );
            Err(AsynError::Status {
                status: AsynStatus::Error,
                message: String::new(),
            })
        }
        fn set_double(&mut self, _: &AsynUser, function: i32, _: f64) -> AsynResult<()> {
            self.0.print(
                TraceMask::ERROR,
                format_args!("motorAxisSetDouble: unknown function {function}\n"),
            );
            Err(AsynError::Status {
                status: AsynStatus::Error,
                message: String::new(),
            })
        }
    }

    static WRITTEN: Model2 = Model2::new("written", 0);

    /// drvMotorAsyn.c:576-752: the port's writes reach the driver's motion
    /// calls with the velocities written before, and every other function
    /// its setInteger/setDouble.
    #[epics_rs::base::epics_test]
    async fn the_port_s_writes_reach_the_driver() {
        let recorder = Arc::new(Mutex::new(Recorder::default()));
        let log: Arc<AxisLog> = Arc::default();
        WRITTEN.set_card(0, vec![recorder.clone()], 100, 1000);
        WRITTEN.set_axis_logs(0, vec![log.clone()]);
        WRITTEN.set_axis_setters(0, vec![Arc::new(Mutex::new(Unknown(log)))]);
        let args = vec![
            ArgValue::String("writtenPort".into()),
            ArgValue::String("written".into()),
            ArgValue::Int(0),
            ArgValue::Int(2),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&WRITTEN]), args).await;

        let path = std::env::temp_dir().join(format!("model2-writes-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some("writtenPort"), TraceMask::ERROR | TraceMask::IO_DRIVER);
        trace.set_device_trace_file("writtenPort", 0, TraceFile::File(file.clone()));
        let port = manager.find_port_handle("writtenPort").unwrap();
        let errors = std::thread::spawn(move || {
            let reason = |info: &str| {
                port.drv_user_create_blocking(&DrvUserRequest::new(info, 0))
                    .map(|i| i.reason)
                    .map_err(|e| e.message().to_string())
            };
            let defer = reason("motor_defer_moves").unwrap();
            let home = reason("MOTOR_MOVE_HOME").unwrap();
            let mut errors = vec![reason("MOTOR_SPEED").unwrap_err()];
            port.write_float64_blocking(reason("MOTOR_VELOCITY").unwrap(), 0, 2.0)
                .unwrap();
            port.write_float64_blocking(reason("MOTOR_VEL_BASE").unwrap(), 0, 0.5)
                .unwrap();
            port.write_float64_blocking(reason("MOTOR_ACCEL").unwrap(), 0, 4.0)
                .unwrap();
            port.write_float64_blocking(reason("MOTOR_MOVE_ABS").unwrap(), 0, 7.0)
                .unwrap();
            port.write_int32_blocking(reason("MOTOR_HOME").unwrap(), 0, 1)
                .unwrap();
            port.write_int32_blocking(reason("MOTOR_STOP_AXIS").unwrap(), 0, 1)
                .unwrap();
            assert!(port.write_int32_blocking(defer, 0, 1).is_err());
            assert!(port.write_float64_blocking(home, 0, 3.0).is_err());
            for addr in [1, 2] {
                errors.push(
                    port.write_int32_blocking(defer, addr, 1)
                        .unwrap_err()
                        .message()
                        .to_string(),
                );
            }
            errors
        })
        .join()
        .unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);

        assert_eq!(
            errors,
            [
                "drvmotorAsyn::drvUserCreate, unknown command=MOTOR_SPEED",
                "drvMotorAsyn::writeInt32 Uninitialised axis 1",
                "drvMotorAsyn::writeInt32 Invalid axis 2",
            ]
        );
        assert_eq!(
            recorder.lock().unwrap().0,
            ["move 7 0.5 2 4", "home true", "stop 4"]
        );
        for line in [
            "drvMotorAsyn::writeFloat64, reason=104, value=7.000000\n",
            "drvMotorAsyn::writeInt32, reason=106, value=1\n",
            "motorAxisSetInteger: unknown function 26\n",
            "drvMotorAsyn::writeInt32, reason=26, value=1\n",
            "motorAxisSetDouble: unknown function 27\n",
            "drvMotorAsyn::writeFloat64, reason=27, value=3.000000\n",
        ] {
            assert!(text.contains(line), "{line:?} not in {text}");
        }
    }
}
