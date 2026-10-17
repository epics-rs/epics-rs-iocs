//! The startup interface of a C model-1 driver: `<Drv>Setup(num_cards,
//! scan_rate)` sizes its card table and sets its polling rate, and
//! `<Drv>Config(card, …)` fills one slot. Records reach the axes through
//! `DTYP "<choice>"` and `OUT "#C card S signal"`.

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

use epics_rs::base::runtime::task::BlockingBridge;
use epics_rs::base::server::ioc_app::{InitHookState, init_hook_register};
use epics_rs::motor::builder::{ControllerAxis, MotorTaskBuilder};

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
    /// The driver's one `motor_task`, gathering the cards `Config` fills
    /// until drvSup init starts it.
    task: Mutex<Task>,
}

enum Task {
    /// No card yet.
    Empty,
    /// The cards so far, and where to spawn the task.
    Filling(MotorTaskBuilder, BlockingBridge),
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
        }
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

    /// Make `motor` signal `signal` of `card`, queried while moving at the
    /// `Setup` scan rate on the driver's one task, which starts at drvSup
    /// init as C `motor_init` starts `motor_task` (motordrvCom.cc:177).
    pub fn install(
        &'static self,
        ctx: &CommandContext,
        card: i32,
        signal: i32,
        motor: Arc<Mutex<dyn AsynMotor>>,
    ) -> Result<(), String> {
        let user = TraceUser::Model1 {
            device: self.row.choice,
            card,
            signal,
        };
        let mut task = self.task.lock().unwrap_or_else(|e| e.into_inner());
        if matches!(*task, Task::Empty) {
            let scan_rate = self
                .setup
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .map_or(self.default_scan_rate, |(_, rate)| rate);
            let scan = Duration::from_millis(1000 / scan_rate as u64);
            *task = Task::Filling(MotorTaskBuilder::new(scan), ctx.bridge().clone());
            init_hook_register(Arc::new(move |state| {
                if state == InitHookState::AfterInitDrvSup {
                    self.start();
                }
            }));
        }
        let Task::Filling(builder, _) = &mut *task else {
            // Deliberate deviation from a C defect: C's Config fills the slot
            // after drvSup init without a word, and that card is never
            // initialized or polled; refuse it instead.
            return Err(format!(
                "{}: card {card} configured after iocInit",
                self.row.choice
            ));
        };
        let ControllerAxis { axis, .. } = builder.axis(motor, user);
        epics_rs::motor::devices::register_axis(axis)
    }

    /// The task, after its startup pass over the cards.
    fn start(&self) {
        let mut task = self.task.lock().unwrap_or_else(|e| e.into_inner());
        if let Task::Filling(builder, bridge) = std::mem::replace(&mut *task, Task::Started) {
            bridge.spawn(builder.build().run());
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
    use epics_rs::base::runtime::task::BlockingBridge;
    use epics_rs::base::server::database::PvDatabase;
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
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), BlockingBridge::capture());
        let log = Log::default();
        TASK.setup(2, 10);
        for card in [1, 0] {
            let motor = Arc::new(Mutex::new(Card {
                card,
                log: log.clone(),
            }));
            TASK.install(&ctx, card, 0, motor).unwrap();
        }
        assert!(log.lock().unwrap().is_empty(), "Config polls no card");
        init_hook_announce(InitHookState::AfterInitDrvSup);
        assert_eq!(
            *log.lock().unwrap(),
            [1, 0],
            "drvSup init's startup pass, in Config order"
        );
        let late = Arc::new(Mutex::new(Card {
            card: 2,
            log: log.clone(),
        }));
        assert!(TASK.install(&ctx, 2, 0, late).is_err());
    }
}
