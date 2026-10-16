//! The startup interface of a C model-1 driver: `<Drv>Setup(num_cards,
//! scan_rate)` sizes its card table and sets its polling rate, and
//! `<Drv>Config(card, …)` fills one slot. Records reach the axes through
//! `DTYP "<choice>"` and `OUT "#C card S signal"`.

use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use crate::iocsh::{DEFAULT_IDLE_POLL_MS, arg_int_req};

/// An `iocshArgInt` as C reads it: 0 when absent or not an integer.
pub fn c_int(args: &[ArgValue], i: usize) -> i32 {
    match args.get(i) {
        Some(ArgValue::Int(v)) => *v as i32,
        _ => 0,
    }
}
use crate::{TraceUser, install, register_model1_device};

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

    /// Make `motor` signal `signal` of `card`, polled while moving at the
    /// `Setup` scan rate.
    pub fn install(
        &self,
        ctx: &CommandContext,
        card: i32,
        signal: i32,
        motor: Arc<Mutex<dyn AsynMotor>>,
    ) -> Result<(), String> {
        let scan_rate = self
            .setup
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .map_or(self.default_scan_rate, |(_, rate)| rate);
        let user = TraceUser::Model1 {
            device: self.row.choice,
            card,
            signal,
        };
        install(
            ctx,
            user,
            motor,
            1000 / scan_rate as u64,
            DEFAULT_IDLE_POLL_MS,
        )
    }
}
