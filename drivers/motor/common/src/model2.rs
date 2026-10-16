//! The startup interface of a C model-2 (`drvMotorAsyn`) driver: the driver's
//! own `Config` command builds a card's axes, and
//! `drvAsynMotorConfigure(portName, driverName, card, numAxes)` opens them
//! into asyn port `portName`, which records reach as `@asyn(portName,axis)`.

use std::sync::{Arc, Mutex};

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;

use crate::iocsh::{arg_int_req, arg_str_req};
use crate::model1::c_int;
use crate::{TraceUser, install, register_drv_motor_asyn_port};

/// One axis a `Config` built, with the poll periods of its driver's poller.
#[derive(Clone)]
pub struct Model2Axis {
    pub motor: Arc<Mutex<dyn AsynMotor>>,
    pub moving_poll_ms: u64,
    pub idle_poll_ms: u64,
}

/// One model-2 driver: the `motorAxisDrvSET_t` name `drvAsynMotorConfigure`
/// finds it by, and the axes each card's `Config` built.
pub struct Model2 {
    name: &'static str,
    cards: Mutex<Vec<(i32, Vec<Model2Axis>)>>,
}

impl Model2 {
    pub const fn new(name: &'static str) -> Self {
        Self {
            name,
            cards: Mutex::new(Vec::new()),
        }
    }

    /// What `Config` built for `card`, replacing an earlier `Config` of it.
    pub fn set_card(&self, card: i32, axes: Vec<Model2Axis>) {
        let mut cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        cards.retain(|(c, _)| *c != card);
        cards.push((card, axes));
    }

    /// Change the poll periods `card`'s axes will be installed with; false
    /// for a card no `Config` built.
    pub fn set_poll_periods(
        &self,
        card: i32,
        moving_ms: Option<u64>,
        idle_ms: Option<u64>,
    ) -> bool {
        let mut cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        let Some((_, axes)) = cards.iter_mut().find(|(c, _)| *c == card) else {
            return false;
        };
        for axis in axes {
            axis.moving_poll_ms = moving_ms.unwrap_or(axis.moving_poll_ms);
            axis.idle_poll_ms = idle_ms.unwrap_or(axis.idle_poll_ms);
        }
        true
    }

    /// The driver's `open(card, axis)`: `None` for an unconfigured card or an
    /// axis it does not have.
    fn open(&self, card: i32, axis: i32) -> Option<Model2Axis> {
        let cards = self.cards.lock().unwrap_or_else(|e| e.into_inner());
        let (_, axes) = cards.iter().find(|(c, _)| *c == card)?;
        axes.get(usize::try_from(axis).ok()?).cloned()
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
            for addr in 0..num_axes {
                let Some(axis) = driver.open(card, addr) else {
                    println!("drvAsynMotorConfigure: Failed to open axis {addr}");
                    continue;
                };
                let user = TraceUser::Model2 {
                    port: port_name.clone(),
                    addr,
                };
                install(
                    ctx,
                    user,
                    axis.motor,
                    axis.moving_poll_ms,
                    axis.idle_poll_ms,
                )?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::motor::sim_motor::SimMotor;

    static DRIVER: Model2 = Model2::new("test");

    fn axis() -> Model2Axis {
        Model2Axis {
            motor: Arc::new(Mutex::new(SimMotor::default())),
            moving_poll_ms: 100,
            idle_poll_ms: 500,
        }
    }

    #[test]
    fn open_finds_only_built_axes_and_takes_later_poll_periods() {
        DRIVER.set_card(3, vec![axis(), axis()]);
        assert!(DRIVER.open(3, 1).is_some());
        assert!(DRIVER.open(3, 2).is_none());
        assert!(DRIVER.open(3, -1).is_none());
        assert!(DRIVER.open(0, 0).is_none());
        assert!(DRIVER.set_poll_periods(3, Some(50), None));
        assert!(!DRIVER.set_poll_periods(0, Some(50), None));
        let opened = DRIVER.open(3, 0).unwrap();
        assert_eq!((opened.moving_poll_ms, opened.idle_poll_ms), (50, 500));
    }
}
