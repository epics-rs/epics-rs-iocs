//! The startup interface of a C model-1 driver: `<Drv>Setup(num_cards,
//! scan_rate)` sizes its card table and sets its polling rate, and
//! `<Drv>Config(card, …)` fills one slot. The driver's `motor_init`, at
//! drvSup init, connects to each card and finds its axes. Records reach the
//! axes through `DTYP "<choice>"` and `OUT "#C card S signal"`.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::{AsynMotor, PostMessage};
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use crate::iocsh::arg_int_req;

/// An `iocshArgInt` as C reads it: 0 when absent or not an integer.
pub fn c_int(args: &[ArgValue], i: usize) -> i32 {
    match args.get(i) {
        Some(ArgValue::Int(v)) => *v as i32,
        _ => 0,
    }
}
use std::time::Duration;

use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::server::ioc_app::{InitHookState, init_hook_register};
use epics_rs::motor::builder::{ControllerAxis, MotorTaskBuilder};
use epics_rs::motor::device_support::BuildTrans;

use crate::{TraceUser, register_model1_device};

/// `SCAN_RATE`, the default polling rate in Hz (motor.h:127).
pub const SCAN_RATE: i32 = 6;

/// One model-1 driver: its `device(motor,VME_IO,…)` row, its card-table
/// limit and its default scan rate, and what its `Setup` set.
pub struct Model1 {
    row: &'static DevSup,
    max_cards: i32,
    default_scan_rate: i32,
    /// `(num_cards, scan_rate)` once `Setup` ran.
    setup: Mutex<Option<(i32, i32)>>,
    /// The cards `Config` filled, until drvSup init runs their
    /// `motor_init` and starts the driver's one `motor_task`.
    task: Mutex<Task>,
    /// What the driver's report prints for each card `Config` filled.
    described: Mutex<Vec<(i32, String)>>,
}

/// What a model-1 drvet `report` reads: C's `<Drv>_num_cards` and the cards
/// in `motor_state`.
pub struct Model1Cards<'a> {
    num_cards: i32,
    described: &'a [(i32, String)],
}

impl Model1Cards<'_> {
    /// C `<Drv>_num_cards`: 0 before `Setup`.
    pub fn num_cards(&self) -> i32 {
        self.num_cards
    }

    /// What [`Model1::describe`] gave for `card`; `None` is C's NULL
    /// `motor_state[card]`, a card whose `Config` failed or never ran.
    pub fn card(&self, card: i32) -> Option<&str> {
        self.described
            .iter()
            .find(|(c, _)| *c == card)
            .map(|(_, line)| line.as_str())
    }

    /// The report most model-1 drivers print (e.g. drvIM483PL.cc:147-171):
    /// `line(card, description)` for each card, or the lines for no
    /// `Setup` and a failed card naming the `driver`.
    pub fn report(&self, driver: &str, line: impl Fn(i32, &str) -> String) -> String {
        if self.num_cards <= 0 {
            return format!("    No {driver} controllers configured.\n");
        }
        (0..self.num_cards)
            .map(|card| match self.card(card) {
                Some(description) => line(card, description),
                None => format!("    {driver} controller {card} connection failed.\n"),
            })
            .collect()
    }
}

/// One axis a card's `motor_init` found: signal, motor, and the device's own
/// `build_trans` when it has one.
type FoundAxis = (i32, Arc<Mutex<dyn AsynMotor>>, Option<Arc<dyn BuildTrans>>);

/// What one card's `motor_init` pass found on its controller: the axes it
/// gives the records, and its report line. A card left with neither is the
/// NULL `motor_state[card]` of a controller that did not answer, which C
/// drops without a word; each of its records then fails init with
/// "card does not exist!" (motordevCom.cc:231-235).
pub struct CardInit {
    card: i32,
    axes: Vec<FoundAxis>,
    description: Option<String>,
}

impl CardInit {
    /// The card being initialized.
    pub fn card(&self) -> i32 {
        self.card
    }

    /// Make `motor` signal `signal` of the card.
    pub fn axis(&mut self, signal: i32, motor: Arc<Mutex<dyn AsynMotor>>) {
        self.axes.push((signal, motor, None));
    }

    /// [`Self::axis`] for a device whose `build_trans` decides from its own
    /// state, as devEMC18011's does.
    pub fn axis_with_build_trans(
        &mut self,
        signal: i32,
        motor: Arc<Mutex<dyn AsynMotor>>,
        build_trans: Arc<dyn BuildTrans>,
    ) {
        self.axes.push((signal, motor, Some(build_trans)));
    }

    /// What the driver's report prints for the card: C's `motor_state[card]`,
    /// read for `ident` and the controller's port.
    pub fn describe(&mut self, line: impl Into<String>) {
        self.description = Some(line.into());
    }
}

/// A card's `motor_init` pass, which its `Config` deferred to drvSup init.
type Init = Box<dyn FnOnce(&mut CardInit) + Send>;

enum Task {
    /// No card yet.
    Empty,
    /// Each configured card's init, and where to spawn the task.
    Filling(BTreeMap<i32, Init>),
    /// drvSup init ran.
    Started,
}

impl Model1 {
    /// `max_cards` and `default_scan_rate` are the driver's `<DRV>_NUM_CARDS`
    /// and `SCAN_RATE`.
    pub const fn new(row: &'static DevSup, max_cards: i32, default_scan_rate: i32) -> Self {
        Self {
            row,
            max_cards,
            default_scan_rate,
            setup: Mutex::new(None),
            task: Mutex::new(Task::Empty),
            described: Mutex::new(Vec::new()),
        }
    }

    /// The driver's `driver(drvet)` line and its `report(level)`, which
    /// `dbior` runs over the cards.
    pub fn register_report(
        &'static self,
        drvet: &'static str,
        report: fn(&Model1Cards<'_>) -> String,
    ) {
        crate::register_driver_report(drvet, move |_level| {
            let num_cards = self
                .setup
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .map_or(0, |(num_cards, _)| num_cards);
            let described = self.described.lock().unwrap_or_else(|e| e.into_inner());
            report(&Model1Cards {
                num_cards,
                described: &described,
            })
        });
    }

    fn describe(&self, card: i32, line: String) {
        let mut described = self.described.lock().unwrap_or_else(|e| e.into_inner());
        described.retain(|(c, _)| *c != card);
        described.push((card, line));
    }

    /// `<name>(num_cards, scan_rate)`. Building it adds the driver's row to
    /// the motor DTYP menu, as its dbd line would.
    pub fn setup_command(&'static self, name: &'static str) -> CommandDef {
        self.register_row();
        CommandDef::new(
            name,
            vec![
                arg_int_req("Max. controller count"),
                arg_int_req("Polling rate"),
            ],
            format!("{name}(num_cards, scan_rate)"),
            move |args: &[ArgValue], _ctx: &CommandContext| {
                self.setup(c_int(args, 0), c_int(args, 1));
                Ok(CommandOutcome::Continue)
            },
        )
    }

    /// Add the driver's row to the motor DTYP menu, for a driver whose
    /// `Setup` takes more than [`Model1::setup_command`] parses.
    pub fn register_row(&self) {
        register_model1_device(self.row);
    }

    /// What every model-1 `Setup` does with its card count and scan rate,
    /// clamping both (e.g. drvSC800.cc:449-458).
    pub fn setup(&self, num_cards: i32, scan_rate: i32) {
        let num_cards = if num_cards < 1 || num_cards > self.max_cards {
            self.max_cards
        } else {
            num_cards
        };
        let scan_rate = if (1..=60).contains(&scan_rate) {
            scan_rate
        } else {
            self.default_scan_rate
        };
        *self.setup.lock().unwrap_or_else(|e| e.into_inner()) = Some((num_cards, scan_rate));
    }

    /// Whether `Config` may fill `card`: C returns ERROR, silently, for a
    /// card outside the table `Setup` sized, or with no `Setup` at all.
    pub fn has_card(&self, card: i32) -> bool {
        match *self.setup.lock().unwrap_or_else(|e| e.into_inner()) {
            Some((num_cards, _)) => (0..num_cards).contains(&card),
            None => false,
        }
    }

    /// What `Config` does with `card`: keep `init`, its `motor_init` pass,
    /// which drvSup init runs for each card in card order, as C `motor_init`
    /// loops over `motor_state` (e.g. drvSC800.cc:520-623). A second `Config`
    /// of the card replaces the first. The axes the passes find are queried
    /// while moving at the `Setup` scan rate on the driver's one task, which
    /// drvSup init then starts as C `motor_init` starts `motor_task`.
    pub fn config(
        &'static self,
        card: i32,
        init: impl FnOnce(&mut CardInit) + Send + 'static,
    ) -> Result<(), String> {
        let mut task = self.task.lock().unwrap_or_else(|e| e.into_inner());
        if matches!(*task, Task::Empty) {
            *task = Task::Filling(BTreeMap::new());
            init_hook_register(Arc::new(move |state| {
                if state == InitHookState::AfterInitDrvSup {
                    self.start();
                }
            }));
        }
        let Task::Filling(cards) = &mut *task else {
            // Deliberate deviation from a C defect: C's Config fills the slot
            // after drvSup init without a word, and that card is never
            // initialized or polled; refuse it instead.
            return Err(format!(
                "{}: card {card} configured after iocInit",
                self.row.choice
            ));
        };
        cards.insert(card, Box::new(init));
        Ok(())
    }

    /// drvSup init: each card's `motor_init` pass, then the task over the
    /// axes they found.
    fn start(&self) {
        let cards = {
            let mut task = self.task.lock().unwrap_or_else(|e| e.into_inner());
            match std::mem::replace(&mut *task, Task::Started) {
                Task::Filling(cards) => cards,
                _ => return,
            }
        };
        let scan_rate = self
            .setup
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .map_or(self.default_scan_rate, |(_, rate)| rate);
        let mut builder = MotorTaskBuilder::new(Duration::from_millis(1000 / scan_rate as u64));
        let mut any_axis = false;
        for (card, init) in cards {
            let mut found = CardInit {
                card,
                axes: Vec::new(),
                description: None,
            };
            init(&mut found);
            if let Some(line) = found.description {
                self.describe(card, line);
            }
            for (signal, motor, build_trans) in found.axes {
                let user = TraceUser::Model1 {
                    device: self.row.choice,
                    card,
                    signal,
                };
                let ControllerAxis { axis, .. } = builder.axis(motor, user);
                let axis = match build_trans {
                    Some(build_trans) => axis.with_build_trans(build_trans),
                    None => axis,
                };
                match epics_rs::motor::devices::register_axis(axis) {
                    Ok(()) => any_axis = true,
                    Err(e) => errlog_vprintf(format_args!("{}: {e}\n", self.row.choice)),
                }
            }
        }
        if any_axis {
            builder.build().start();
        }
    }
}

/// The record's PREM and POST as a model-1 device holds them: `build_trans`
/// reads PREM into the move it builds, and points `postmsgptr` at the POST
/// field when POST is set, for `set_status` to send once that move ends
/// (devIM483PL.cc:191-205, drvIM483PL.cc:357-366).
#[derive(Debug, Default)]
pub struct MoveStrings {
    /// What `set_move_strings` gave for the move built next.
    next: Option<(Vec<u8>, Option<PostMessage>)>,
    /// `nodeptr->postmsgptr` of the move built last.
    post: Option<PostMessage>,
}

impl MoveStrings {
    /// `AsynMotor::set_move_strings`.
    pub fn set(&mut self, prem: &[u8], post: Option<PostMessage>) {
        self.next = Some((prem.to_vec(), post));
    }

    /// Building a move: its PREM, empty when there is none, and its POST
    /// becomes the one [`MoveStrings::post_due`] gives.
    pub fn arm(&mut self) -> Vec<u8> {
        let (prem, post) = self.next.take().unwrap_or_default();
        self.post = post;
        prem
    }

    /// `set_status` seeing the move done or stopped by a limit: the POST as
    /// the field holds it now, once. An emptied field gives nothing, as
    /// `send_mess` returns on an empty message.
    pub fn post_due(&mut self) -> Option<Vec<u8>> {
        self.post.take().map(|p| p.get()).filter(|p| !p.is_empty())
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::interfaces::motor::{MotorStatus, PollFailure};
    use epics_rs::asyn::user::AsynUser;
    use epics_rs::base::server::ioc_app::{InitHookState, init_hook_announce};

    use super::*;
    use crate::model1_device;

    static DEV_TASK: DevSup = model1_device("devTestTask", "TestTask");
    static TASK: Model1 = Model1::new(&DEV_TASK, 4, SCAN_RATE);

    type Log = Arc<Mutex<Vec<i32>>>;

    struct Card {
        card: i32,
        log: Log,
    }

    impl AsynMotor for Card {
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
            PollFailure::PROBLEM_AND_COMMS_ERROR
        }
        fn poll(&mut self, _: &AsynUser) -> AsynResult<MotorStatus> {
            self.log.lock().unwrap().push(self.card);
            Ok(MotorStatus::default())
        }
    }

    #[test]
    fn the_cards_wait_for_drvsup_init_and_start_on_one_task() {
        let rt = tokio::runtime::Builder::new_current_thread()
            .build()
            .unwrap();
        let _entered = rt.enter();
        let log = Log::default();
        let inits = Log::default();
        TASK.setup(2, 10);
        for card in [1, 0] {
            let motor = Arc::new(Mutex::new(Card {
                card,
                log: log.clone(),
            }));
            let inits = inits.clone();
            TASK.config(card, move |init| {
                inits.lock().unwrap().push(init.card());
                init.axis(0, motor);
            })
            .unwrap();
        }
        assert!(inits.lock().unwrap().is_empty(), "Config talks to no card");
        init_hook_announce(InitHookState::AfterInitDrvSup);
        assert_eq!(*inits.lock().unwrap(), [0, 1], "motor_init, in card order");
        assert!(
            log.lock().unwrap().is_empty(),
            "motor_task has no startup pass; each driver's motor_init reads"
        );
        assert!(TASK.config(2, |_| {}).is_err());
    }

    static DEV_SILENT: DevSup = model1_device("devTestSilent", "TestSilent");
    static SILENT: Model1 = Model1::new(&DEV_SILENT, 4, SCAN_RATE);

    /// A card whose controller does not answer: C's `motor_init` drops it
    /// and the report then calls it failed.
    #[test]
    fn a_card_motor_init_finds_nothing_on_is_dropped() {
        let rt = tokio::runtime::Builder::new_current_thread()
            .build()
            .unwrap();
        let _entered = rt.enter();
        SILENT.register_report("drvTestSilent", |cards| {
            cards.report("TestSilent", |card, line| format!("{card} {line}\n"))
        });
        SILENT.setup(2, 10);
        SILENT.config(0, |_| {}).unwrap();
        SILENT.config(1, |init| init.describe("id: B")).unwrap();
        init_hook_announce(InitHookState::AfterInitDrvSup);
        let drvet =
            epics_rs::base::server::driver_support::find_driver_support("drvTestSilent").unwrap();
        assert_eq!(
            drvet.report(0).unwrap(),
            b"    TestSilent controller 0 connection failed.\n1 id: B\n"
        );
    }

    static DEV_REPORT: DevSup = model1_device("devTestReport", "TestReport");
    static REPORT: Model1 = Model1::new(&DEV_REPORT, 4, SCAN_RATE);

    fn report(cards: &Model1Cards<'_>) -> String {
        cards.report("TestReport", |card, line| {
            format!("    TestReport controller {card}, {line}\n")
        })
    }

    /// C `dbior`'s `pdrvet->report(level)` over `motor_state`.
    #[test]
    fn dbior_reaches_the_model1_report_over_the_cards() {
        use epics_rs::base::server::driver_support::find_driver_support;
        REPORT.register_report("drvTestReport", report);
        let drvet = find_driver_support("drvTestReport").unwrap();
        assert_eq!(
            drvet.report(0).unwrap(),
            b"    No TestReport controllers configured.\n"
        );
        REPORT.setup(2, 10);
        REPORT.describe(1, "id: B".into());
        assert_eq!(
            drvet.report(0).unwrap(),
            b"    TestReport controller 0 connection failed.\n    TestReport controller 1, id: B\n"
        );
    }
}
