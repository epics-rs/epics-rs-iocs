//! iocsh commands for the `motor-pi` driver's ported controllers.

use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::c630::{PIC630_NUM_AXIS, PIC630Axis, PIC630Controller};
use crate::c662::{PIC662Axis, PIC662Controller};
use crate::c663::{PIC663Axis, PIC663Controller};
use crate::c844::{PIC844_NUM_AXES, PIC844Axis, PIC844Controller};
use crate::c848::{PIC848Axis, PIC848Controller};
use crate::c862::{PIC862Axis, PIC862Controller};
use crate::e516::{PIE516Axis, PIE516Controller};
use crate::e517::{PIE517Axis, PIE517Controller};
use crate::e710::{PIE710Axis, PIE710Controller};
use crate::e816::{PIE816Axis, PIE816Controller};

/// Command timeout (C `drvPIC630.h` `COMM_TIMEOUT` 2 s).
const PIC630_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIC662.h` `COMM_TIMEOUT` 2 s).
const PIC662_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPI.h` `COMM_TIMEOUT` 2 s).
const PIC844_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIC848.h` `COMM_TIMEOUT` 2 s).
const PIC848_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIC862.h` `COMM_TIMEOUT` 2.0 s).
const PIC862_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIE516.h` `COMM_TIMEOUT` 2.0 s).
const PIE516_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIE517.h` `COMM_TIMEOUT` 2.0 s).
const PIE517_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIE710.h` `COMM_TIMEOUT` 2.0 s).
const PIE710_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIE816.h` `COMM_TIMEOUT` 2.0 s).
const PIE816_TIMEOUT: Duration = Duration::from_secs(2);

/// Command timeout (C `drvPIC663.h` `COMM_TIMEOUT` 2.0 s — C-663 is a C-862
/// clone and shares the same 2.0 s value).
const PIC663_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devPIC862,"PIC862")` (devPIMotor.dbd:26).
static DEV_PIC862: DevSup = model1_device("devPIC862", "PIC862");

/// `PIC862_NUM_CARDS` (drvPIC862.cc:43).
static PIC862: Model1 = Model1::new(&DEV_PIC862, 8, SCAN_RATE);

/// `PIC862Setup(num_cards, scan_rate)`.
pub fn pic862_setup_command() -> CommandDef {
    PIC862.setup_command("PIC862Setup")
}

/// `PIC862Config(card, name, address)`.
/// `address` is the controller's multi-drop bus address (0-F).
pub fn pic862_config_command() -> CommandDef {
    CommandDef::new(
        "PIC862Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "PIC862Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIC862.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let addr = c_int(args, 2);
            if !(0..=15).contains(&addr) {
                return Err("PIC862Config: addr must be 0-15".into());
            }

            let handle = connect_with_eos(
                &asyn_port,
                PIC862_TIMEOUT,
                b"\n\x03",
                b"\r",
                EosRefusal::Ignored,
            )?;
            let controller = PIC862Controller::new(handle, addr as u8)
                .map_err(|e| format!("PIC862Config: {e}"))?;
            let controller = Arc::new(Mutex::new(controller));

            let axis = PIC862Axis::new(controller).map_err(|e| format!("PIC862Config: {e}"))?;
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            PIC862.install(ctx, card, 0, motor)?;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPIC663,"PIC663")` (devPIMotor.dbd:32).
static DEV_PIC663: DevSup = model1_device("devPIC663", "PIC663");

/// `PIC663_NUM_CARDS` (drvPIC663.cc:34).
static PIC663: Model1 = Model1::new(&DEV_PIC663, 8, SCAN_RATE);

/// `PIC663Setup(num_cards, scan_rate)`.
pub fn pic663_setup_command() -> CommandDef {
    PIC663.register_report("drvPIC663", |cards| {
        cards.report("PIC663", |card, line| {
            format!("    PIC663 controller #{card}, {line} \n")
        })
    });
    PIC663.setup_command("PIC663Setup")
}

/// `PIC663Config(card, name, address)`.
/// `address` is the controller's multi-drop bus address (0-F).
pub fn pic663_config_command() -> CommandDef {
    CommandDef::new(
        "PIC663Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "PIC663Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIC663.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let addr = c_int(args, 2);
            if !(0..=15).contains(&addr) {
                return Err("PIC663Config: addr must be 0-15".into());
            }

            let handle = connect_with_eos(
                &asyn_port,
                PIC663_TIMEOUT,
                b"\n\x03",
                b"\r",
                EosRefusal::Ignored,
            )?;
            let controller = PIC663Controller::new(handle, addr as u8)
                .map_err(|e| format!("PIC663Config: {e}"))?;
            let line = format!("port={asyn_port}, id: {}", controller.ident());
            let controller = Arc::new(Mutex::new(controller));

            let axis = PIC663Axis::new(controller).map_err(|e| format!("PIC663Config: {e}"))?;
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            PIC663.install(ctx, card, 0, motor)?;
            PIC663.describe(card, line);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPIC630,"PI C630")` (devPIMotor.dbd:8).
static DEV_PIC630: DevSup = model1_device("devPIC630", "PI C630");

/// `PIC630_NUM_CARDS` (drvPIC630.h:24).
static PIC630: Model1 = Model1::new(&DEV_PIC630, 8, SCAN_RATE);

/// `PIC630_num_axis`: the axes on each chain, as `PIC630Setup` set them.
static PIC630_AXES: AtomicU8 = AtomicU8::new(PIC630_NUM_AXIS);

/// `PIC630Setup(num_cards, num_channels, scan_rate)`.
pub fn pic630_setup_command() -> CommandDef {
    PIC630.register_row();
    PIC630.register_report("drvPIC630", |cards| {
        cards.report("PIC630", |card, line| {
            format!("    PIC630 controller #{card}, {line} \n")
        })
    });
    CommandDef::new(
        "PIC630Setup",
        vec![
            arg_int_req("Max. controller groups"),
            arg_int_req("Max. axes per group"),
            arg_int_req("Polling rate"),
        ],
        "PIC630Setup(num_cards, num_channels, scan_rate)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            PIC630.setup(c_int(args, 0), c_int(args, 2));
            let channels = c_int(args, 1);
            let channels = if (1..=PIC630_NUM_AXIS as i32).contains(&channels) {
                channels as u8
            } else {
                PIC630_NUM_AXIS
            };
            PIC630_AXES.store(channels, Ordering::Relaxed);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `PIC630Config(card, name, cur0, …, cur8)`: the drive current of each
/// axis on the chain (0 = OFF, 1 = 100 mA … 8 = 800 mA), sent at connect.
pub fn pic630_config_command() -> CommandDef {
    const CURRENTS: [&str; PIC630_NUM_AXIS as usize] = [
        "Ch 1 current setting",
        "Ch 2 current setting",
        "Ch 3 current setting",
        "Ch 4 current setting",
        "Ch 5 current setting",
        "Ch 6 current setting",
        "Ch 7 current setting",
        "Ch 8 current setting",
        "Ch 9 current setting",
    ];
    let mut arg_defs = vec![
        arg_int_req("Card being configured"),
        arg_str_req("asyn port name"),
    ];
    arg_defs.extend(CURRENTS.map(arg_int_req));
    CommandDef::new(
        "PIC630Config",
        arg_defs,
        "PIC630Config(card, name, cur0, ..., cur8)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIC630.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let num_axes = PIC630_AXES.load(Ordering::Relaxed);

            let handle = connect_with_eos(
                &asyn_port,
                PIC630_TIMEOUT,
                b"\n",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller = PIC630Controller::new(handle, num_axes)
                .map_err(|e| format!("PIC630Config: {e}"))?;
            let controller = Arc::new(Mutex::new(controller));

            for signal in 0..num_axes {
                let current = c_int(args, 2 + signal as usize);
                let axis = PIC630Axis::new(controller.clone(), signal, current)
                    .map_err(|e| format!("PIC630Config: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                PIC630.install(ctx, card, signal as i32, motor)?;
            }
            // C's ident is the constant motor_init copies (drvPIC630.cc:546).
            PIC630.describe(card, format!("port={asyn_port}, id: PIC630"));
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPIC662,"PIC662")` (devPIMotor.dbd:20).
static DEV_PIC662: DevSup = model1_device("devPIC662", "PIC662");

/// `PIC662_NUM_CARDS` (drvPIC662.cc:61).
static PIC662: Model1 = Model1::new(&DEV_PIC662, 8, SCAN_RATE);

/// `PIC662Setup(num_cards, scan_rate)`.
pub fn pic662_setup_command() -> CommandDef {
    PIC662.register_report("drvPIC662", |cards| {
        cards.report("PIC662", |card, line| {
            format!("    PIC662 controller #{card}, {line} \n")
        })
    });
    PIC662.setup_command("PIC662Setup")
}

/// `PIC662Config(card, name)`.
pub fn pic662_config_command() -> CommandDef {
    CommandDef::new(
        "PIC662Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
        ],
        "PIC662Config(card, name)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIC662.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                PIC662_TIMEOUT,
                b"\n",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller =
                PIC662Controller::new(handle).map_err(|e| format!("PIC662Config: {e}"))?;
            let line = format!("port={asyn_port}, id: {}", controller.ident());
            let controller = Arc::new(Mutex::new(controller));

            let axis = PIC662Axis::new(controller).map_err(|e| format!("PIC662Config: {e}"))?;
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
            PIC662.install(ctx, card, 0, motor)?;
            PIC662.describe(card, line);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPIC844,"PIC844")` (devPIMotor.dbd:2).
static DEV_PIC844: DevSup = model1_device("devPIC844", "PIC844");

/// `PIC844_NUM_CARDS` (drvPIC844.cc:69).
static PIC844: Model1 = Model1::new(&DEV_PIC844, 8, SCAN_RATE);

/// `PIC844Setup(num_cards, scan_rate)`.
pub fn pic844_setup_command() -> CommandDef {
    PIC844.setup_command("PIC844Setup")
}

/// `PIC844Config(card, name, address)`.
pub fn pic844_config_command() -> CommandDef {
    CommandDef::new(
        "PIC844Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "PIC844Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIC844.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                PIC844_TIMEOUT,
                b"\n",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller =
                PIC844Controller::new(handle).map_err(|e| format!("PIC844Config: {e}"))?;
            let controller = Arc::new(Mutex::new(controller));

            for signal in 0..PIC844_NUM_AXES {
                let axis = PIC844Axis::new(controller.clone(), signal)
                    .map_err(|e| format!("PIC844Config: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                PIC844.install(ctx, card, signal as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPIC848,"PIC848")` (devPIMotor.dbd:14).
static DEV_PIC848: DevSup = model1_device("devPIC848", "PIC848");

/// `PIC848_NUM_CARDS` (drvPIC848.cc:58).
static PIC848: Model1 = Model1::new(&DEV_PIC848, 8, SCAN_RATE);

/// `PIC848Setup(num_cards, scan_rate)`.
pub fn pic848_setup_command() -> CommandDef {
    PIC848.setup_command("PIC848Setup")
}

/// `PIC848Config(card, name, address)`.
pub fn pic848_config_command() -> CommandDef {
    CommandDef::new(
        "PIC848Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "PIC848Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIC848.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                PIC848_TIMEOUT,
                b"\n",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller =
                PIC848Controller::new(handle).map_err(|e| format!("PIC848Config: {e}"))?;
            let num_axes = controller.num_axes();
            if num_axes == 0 {
                return Err(format!(
                    "PIC848Config: card={card} asynPort={asyn_port} reported no stages \
                     (CST? => NOSTAGE on axis A)"
                ));
            }
            let controller = Arc::new(Mutex::new(controller));

            for signal in 0..num_axes {
                let axis = PIC848Axis::new(controller.clone(), signal)
                    .map_err(|e| format!("PIC848Config: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                PIC848.install(ctx, card, signal as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPIE516,"PIE516")` (devPIMotor.dbd:44).
static DEV_PIE516: DevSup = model1_device("devPIE516", "PIE516");

/// `PIE516_NUM_CARDS` (drvPIE516.cc:65).
static PIE516: Model1 = Model1::new(&DEV_PIE516, 10, SCAN_RATE);

/// `PIE516Setup(num_cards, scan_rate)`.
pub fn pie516_setup_command() -> CommandDef {
    PIE516.setup_command("PIE516Setup")
}

/// `PIE516Config(card, name, address)`.
pub fn pie516_config_command() -> CommandDef {
    CommandDef::new(
        "PIE516Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "PIE516Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIE516.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                PIE516_TIMEOUT,
                b"\n",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller =
                PIE516Controller::new(handle).map_err(|e| format!("PIE516Config: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                let axis_obj = PIE516Axis::new(controller.clone(), axis)
                    .map_err(|e| format!("PIE516Config: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_obj));
                PIE516.install(ctx, card, axis as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPIE517,"PIE517")` (devPIMotor.dbd:50).
static DEV_PIE517: DevSup = model1_device("devPIE517", "PIE517");

/// `PIE517_NUM_CARDS` (drvPIE517.cc:73).
static PIE517: Model1 = Model1::new(&DEV_PIE517, 10, SCAN_RATE);

/// `PIE517Setup(num_cards, scan_rate)`.
pub fn pie517_setup_command() -> CommandDef {
    PIE517.setup_command("PIE517Setup")
}

/// `PIE517Config(card, name, address)`.
pub fn pie517_config_command() -> CommandDef {
    CommandDef::new(
        "PIE517Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "PIE517Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIE517.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                PIE517_TIMEOUT,
                b"\n",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller =
                PIE517Controller::new(handle).map_err(|e| format!("PIE517Config: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                let axis_obj = PIE517Axis::new(controller.clone(), axis)
                    .map_err(|e| format!("PIE517Config: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_obj));
                PIE517.install(ctx, card, axis as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPIE710,"PIE710")` (devPIMotor.dbd:38).
static DEV_PIE710: DevSup = model1_device("devPIE710", "PIE710");

/// `PIE710_NUM_CARDS` (drvPIE710.cc:57).
static PIE710: Model1 = Model1::new(&DEV_PIE710, 8, SCAN_RATE);

/// `PIE710Setup(num_cards, scan_rate)`.
pub fn pie710_setup_command() -> CommandDef {
    PIE710.setup_command("PIE710Setup")
}

/// `PIE710Config(card, name, address)`.
pub fn pie710_config_command() -> CommandDef {
    CommandDef::new(
        "PIE710Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "PIE710Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIE710.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                PIE710_TIMEOUT,
                b"\n",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller =
                PIE710Controller::new(handle).map_err(|e| format!("PIE710Config: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                let axis_obj = PIE710Axis::new(controller.clone(), axis)
                    .map_err(|e| format!("PIE710Config: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_obj));
                PIE710.install(ctx, card, axis as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `device(motor,VME_IO,devPIE816,"PIE816")` (devPIMotor.dbd:56).
static DEV_PIE816: DevSup = model1_device("devPIE816", "PIE816");

/// `PIE816_NUM_CARDS` (drvPIE816.cc:73).
static PIE816: Model1 = Model1::new(&DEV_PIE816, 10, SCAN_RATE);

/// `PIE816Setup(num_cards, scan_rate)`.
pub fn pie816_setup_command() -> CommandDef {
    PIE816.setup_command("PIE816Setup")
}

/// `PIE816Config(card, name, address)`.
pub fn pie816_config_command() -> CommandDef {
    CommandDef::new(
        "PIE816Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "PIE816Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PIE816.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;

            let handle = connect_with_eos(
                &asyn_port,
                PIE816_TIMEOUT,
                b"\n",
                b"\n",
                EosRefusal::Ignored,
            )?;
            let controller =
                PIE816Controller::new(handle).map_err(|e| format!("PIE816Config: {e}"))?;
            let num_axes = controller.num_axes();
            let controller = Arc::new(Mutex::new(controller));

            for axis in 0..num_axes {
                let axis_obj = PIE816Axis::new(controller.clone(), axis)
                    .map_err(|e| format!("PIE816Config: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_obj));
                PIE816.install(ctx, card, axis as i32, motor)?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod report {
    use super::*;
    use motor_common::testing::{LineRig, driver_report, run_command};

    fn int(v: i64) -> ArgValue {
        ArgValue::Int(v)
    }

    fn port(name: &str) -> ArgValue {
        ArgValue::String(name.into())
    }

    /// The drvet `report` `dbior` runs, before `setup` and after `config`
    /// brought up card 1 of the two `setup` made.
    async fn reports(
        drvet: &str,
        setup: (CommandDef, Vec<ArgValue>),
        config: (CommandDef, Vec<ArgValue>),
    ) -> (String, String) {
        let before = driver_report(drvet, 0);
        run_command(setup.0, setup.1).await;
        run_command(config.0, config.1).await;
        (before, driver_report(drvet, 0))
    }

    /// drvPIC630.cc:114-136, which `dbior` runs; the ident is C's constant "PIC630".
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pic630_chain() {
        let setup = pic630_setup_command();
        let (_line, _io) = LineRig::new("c630RepLine", |last| match last {
            "1TS" => "1TS:0".into(),
            "1TP" => "1TP:0".into(),
            c => c.into(),
        });
        let mut config = vec![int(1), port("c630RepLine")];
        config.extend((0..9).map(|_| int(1)));
        let (before, after) = reports(
            "drvPIC630",
            (setup, vec![int(2), int(1), int(10)]),
            (pic630_config_command(), config),
        )
        .await;
        assert_eq!(before, "    No PIC630 controllers configured.\n");
        assert_eq!(
            after,
            "    PIC630 controller 0 connection failed.\n    PIC630 controller #1, \
             port=c630RepLine, id: PIC630 \n"
        );
    }

    /// drvPIC662.cc:135-160, which `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pic662_controller() {
        let setup = pic662_setup_command();
        let (_line, _io) = LineRig::new("c662RepLine", |last| match last {
            "*IDN?" => "E-662".into(),
            "*ESR?" => "1".into(),
            _ => "0.000".into(),
        });
        let (before, after) = reports(
            "drvPIC662",
            (setup, vec![int(2), int(10)]),
            (pic662_config_command(), vec![int(1), port("c662RepLine")]),
        )
        .await;
        assert_eq!(before, "    No PIC662 controllers configured.\n");
        assert_eq!(
            after,
            "    PIC662 controller 0 connection failed.\n    PIC662 controller #1, \
             port=c662RepLine, id: E-662 \n"
        );
    }

    /// drvPIC663.cc:108-133, which `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pic663_controller() {
        let setup = pic663_setup_command();
        let (_line, _io) = LineRig::new("c663RepLine", |last| match last {
            "TP" => "P:0\r".into(),
            _ => "C-663\r".into(),
        });
        let (before, after) = reports(
            "drvPIC663",
            (setup, vec![int(2), int(10)]),
            (
                pic663_config_command(),
                vec![int(1), port("c663RepLine"), int(1)],
            ),
        )
        .await;
        assert_eq!(before, "    No PIC663 controllers configured.\n");
        assert_eq!(
            after,
            "    PIC663 controller 0 connection failed.\n    PIC663 controller #1, \
             port=c663RepLine, id: C-663 \n"
        );
    }
}
