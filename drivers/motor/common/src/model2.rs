//! The startup interface of a C model-2 (`drvMotorAsyn`) driver: the driver's
//! own `Config` command builds a card's axes, and
//! `drvAsynMotorConfigure(portName, driverName, card, numAxes)` opens them
//! into asyn port `portName`, which records reach as `@asyn(portName,axis)`.

use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use crate::iocsh::{arg_int_req, arg_str_req};
use crate::model1::c_int;
use crate::{Controller, TraceUser, register_drv_motor_asyn_port};

/// The axes a card's `Config` built, and the poller they go on.
struct Card {
    card: i32,
    axes: Vec<Arc<Mutex<dyn AsynMotor>>>,
    poller: Poller,
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
}

impl Model2 {
    pub const fn new(name: &'static str, forced_fast_polls: u32) -> Self {
        Self {
            name,
            forced_fast_polls,
            cards: Mutex::new(Vec::new()),
        }
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
        });
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
            register_drv_motor_asyn_port(&port_name, num_axes)?;
            // The driver's one task polls the card's axes in turn.
            let poller = driver.poller(card);
            let controller = poller.as_ref().map(Poller::controller);
            for addr in 0..num_axes {
                let (Some(controller), Some(motor)) = (&controller, driver.open(card, addr)) else {
                    println!("drvAsynMotorConfigure: Failed to open axis {addr}");
                    continue;
                };
                let user = TraceUser::Model2 {
                    port: port_name.clone(),
                    addr,
                };
                controller.install(user, motor)?;
            }
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
