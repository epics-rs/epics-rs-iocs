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
    PIC862.register_report("drvPIC862", |cards| {
        cards.report("PIC862", |card, line| {
            format!("    PIC862 controller #{card}, {line} \n")
        })
    });
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

            PIC862.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIC862_TIMEOUT,
                    b"\n\x03",
                    b"\r",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIC862Controller::new(handle, addr as u8) else {
                    return;
                };
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));

                let Ok(axis) = PIC862Axis::new(controller) else {
                    return;
                };
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                init.axis(0, motor);
                init.describe(line);
            })?;
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

            PIC663.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIC663_TIMEOUT,
                    b"\n\x03",
                    b"\r",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIC663Controller::new(handle, addr as u8) else {
                    return;
                };
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));

                let Ok(axis) = PIC663Axis::new(controller) else {
                    return;
                };
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                init.axis(0, motor);
                init.describe(line);
            })?;
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
            let currents: Vec<i32> = (0..num_axes).map(|s| c_int(args, 2 + s as usize)).collect();

            PIC630.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIC630_TIMEOUT,
                    b"\n",
                    b"\n",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIC630Controller::new(handle, num_axes) else {
                    return;
                };
                let controller = Arc::new(Mutex::new(controller));

                for (signal, current) in (0..num_axes).zip(currents) {
                    let axis = PIC630Axis::new(controller.clone(), signal, current);
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                    init.axis(signal as i32, motor);
                }
                // C's ident is the constant motor_init copies (drvPIC630.cc:546).
                init.describe(format!("port={asyn_port}, id: PIC630"));
            })?;
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

            PIC662.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIC662_TIMEOUT,
                    b"\n",
                    b"\n",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIC662Controller::new(handle) else {
                    return;
                };
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));

                let axis = PIC662Axis::new(controller);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                init.axis(0, motor);
                init.describe(line);
            })?;
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
    PIC844.register_report("drvPIC844", |cards| {
        cards.report("PIC844", |card, line| {
            format!("    PIC844 controller #{card}, {line} \n")
        })
    });
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

            PIC844.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIC844_TIMEOUT,
                    b"\n",
                    b"\n",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIC844Controller::new(handle) else {
                    return;
                };
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));

                for signal in 0..PIC844_NUM_AXES {
                    let Ok(axis) = PIC844Axis::new(controller.clone(), signal) else {
                        return;
                    };
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                    init.axis(signal as i32, motor);
                }
                init.describe(line);
            })?;
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
    PIC848.register_report("drvPIC848", |cards| {
        cards.report("PIC848", |card, line| {
            format!("    PIC848 controller #{card}, {line} \n")
        })
    });
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

            PIC848.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIC848_TIMEOUT,
                    b"\n",
                    b"\n",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIC848Controller::new(handle) else {
                    return;
                };
                let num_axes = controller.num_axes();
                if num_axes == 0 {
                    return;
                }
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));

                for signal in 0..num_axes {
                    let Ok(axis) = PIC848Axis::new(controller.clone(), card, signal) else {
                        return;
                    };
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                    init.axis(signal as i32, motor);
                }
                init.describe(line);
            })?;
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
    PIE516.register_report("drvPIE516", |cards| {
        cards.report("PIE516", |card, line| {
            format!("    PIE516 controller #{card}, {line} \n")
        })
    });
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

            PIE516.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIE516_TIMEOUT,
                    b"\n",
                    b"\n",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIE516Controller::new(handle) else {
                    return;
                };
                let num_axes = controller.num_axes();
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));

                for axis in 0..num_axes {
                    let axis_obj = PIE516Axis::new(controller.clone(), axis);
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_obj));
                    init.axis(axis as i32, motor);
                }
                init.describe(line);
            })?;
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
    PIE517.register_report("drvPIE517", |cards| {
        cards.report("PIE517", |card, line| {
            format!("    PIE517 controller #{card}, {line} \n")
        })
    });
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

            PIE517.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIE517_TIMEOUT,
                    b"\n",
                    b"\n",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIE517Controller::new(handle) else {
                    return;
                };
                let num_axes = controller.num_axes();
                // C copies an uninitialised buffer into the ident (see crate::e517).
                let line = format!("port={asyn_port}, id: ");
                let controller = Arc::new(Mutex::new(controller));

                for axis in 0..num_axes {
                    let Ok(axis_obj) = PIE517Axis::new(controller.clone(), axis) else {
                        return;
                    };
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_obj));
                    init.axis(axis as i32, motor);
                }
                init.describe(line);
            })?;
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
    PIE710.register_report("drvPIE710", |cards| {
        cards.report("PIE710", |card, line| {
            format!("    PIE710 controller #{card}, {line} \n")
        })
    });
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

            PIE710.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIE710_TIMEOUT,
                    b"\n",
                    b"\n",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIE710Controller::new(handle) else {
                    return;
                };
                let num_axes = controller.num_axes();
                // drvPIE710.cc:580 passes `buff[0]`, a char, as strcpy's
                // source; a C defect not copied: the GI reply is the ident.
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));

                for axis in 0..num_axes {
                    let Ok(axis_obj) = PIE710Axis::new(controller.clone(), axis) else {
                        return;
                    };
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_obj));
                    init.axis(axis as i32, motor);
                }
                init.describe(line);
            })?;
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
    PIE816.register_report("drvPIE816", |cards| {
        cards.report("PIE816", |card, line| {
            format!("    PIE816 controller #{card}, {line} \n")
        })
    });
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

            PIE816.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    0,
                    PIE816_TIMEOUT,
                    b"\n",
                    b"\n",
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = PIE816Controller::new(handle) else {
                    return;
                };
                let num_axes = controller.num_axes();
                let line = format!("port={asyn_port}, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));

                for axis in 0..num_axes {
                    let Ok(axis_obj) = PIE816Axis::new(controller.clone(), axis) else {
                        return;
                    };
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_obj));
                    init.axis(axis as i32, motor);
                }
                init.describe(line);
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod report {
    use super::*;
    use motor_common::testing::{LineRig, driver_report, drvsup_init, run_command};

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
        drvsup_init().await;
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

    /// drvPIC662.cc:558-559: `REMOTE_MODE` is a bare send_mess once `*IDN?`
    /// answered, so a refused one keeps the card.
    #[epics_rs::base::epics_test]
    async fn a_pic662_answering_idn_survives_a_refused_remote_mode() {
        let (_line, _io) = LineRig::failing(
            "c662NoRem",
            |last| match last {
                "*IDN?" => "E-662".into(),
                _ => "0.000".into(),
            },
            |cmd| cmd.starts_with("DEV:CONT"),
        );
        let (_, after) = reports(
            "drvPIC662",
            (pic662_setup_command(), vec![int(1), int(10)]),
            (pic662_config_command(), vec![int(0), port("c662NoRem")]),
        )
        .await;
        assert_eq!(
            after,
            "    PIC662 controller #0, port=c662NoRem, id: E-662 \n"
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

    /// drvPIC844.cc:143-168, which `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pic844_controller() {
        let setup = pic844_setup_command();
        let (_line, _io) = LineRig::new("c844RepLine", |last| match last {
            "*IDN?" => "C-844".into(),
            c if c.ends_with("AXIS:STAT?") => "ON".into(),
            _ => "0".into(),
        });
        let (before, after) = reports(
            "drvPIC844",
            (setup, vec![int(2), int(10)]),
            (
                pic844_config_command(),
                vec![int(1), port("c844RepLine"), int(0)],
            ),
        )
        .await;
        assert_eq!(before, "    No PIC844 controllers configured.\n");
        assert_eq!(
            after,
            "    PIC844 controller 0 connection failed.\n    PIC844 controller #1, \
             port=c844RepLine, id: C-844 \n"
        );
    }

    /// drvPIC848.cc:134-159, which `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pic848_controller() {
        let setup = pic848_setup_command();
        let (_line, _io) = LineRig::new("c848RepLine", |last| match last {
            "*IDN?" => "C-848".into(),
            "CST? A" => "A=M-110".into(),
            "CST? B" => "B=NOSTAGE".into(),
            _ => "A=0".into(),
        });
        let (before, after) = reports(
            "drvPIC848",
            (setup, vec![int(2), int(10)]),
            (
                pic848_config_command(),
                vec![int(1), port("c848RepLine"), int(0)],
            ),
        )
        .await;
        assert_eq!(before, "    No PIC848 controllers configured.\n");
        assert_eq!(
            after,
            "    PIC848 controller 0 connection failed.\n    PIC848 controller #1, \
             port=c848RepLine, id: C-848 \n"
        );
    }

    /// drvPIC862.cc:117-142, which `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pic862_controller() {
        let setup = pic862_setup_command();
        let (_line, _io) = LineRig::new("c862RepLine", |last| match last {
            "TP" => "P:0\r".into(),
            _ => "C-862\r".into(),
        });
        let (before, after) = reports(
            "drvPIC862",
            (setup, vec![int(2), int(10)]),
            (
                pic862_config_command(),
                vec![int(1), port("c862RepLine"), int(1)],
            ),
        )
        .await;
        assert_eq!(before, "    No PIC862 controllers configured.\n");
        assert_eq!(
            after,
            "    PIC862 controller 0 connection failed.\n    PIC862 controller #1, \
             port=c862RepLine, id: C-862 \n"
        );
    }

    /// drvPIE516.cc:140-165, which `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pie516_controller() {
        let setup = pie516_setup_command();
        let (_line, _io) = LineRig::new("e516RepLine", |last| match last {
            "ONL?" | "SVO? A" => "1".into(),
            "VER?" => "E-516".into(),
            "POS? A" => "0.000".into(),
            _ => String::new(),
        });
        let (before, after) = reports(
            "drvPIE516",
            (setup, vec![int(2), int(10)]),
            (
                pie516_config_command(),
                vec![int(1), port("e516RepLine"), int(0)],
            ),
        )
        .await;
        assert_eq!(before, "    No PIE516 controllers configured.\n");
        assert_eq!(
            after,
            "    PIE516 controller 0 connection failed.\n    PIE516 controller #1, \
             port=e516RepLine, id: E-516 \n"
        );
    }

    /// drvPIE516.cc:584-628: `ONL 1` and `VCO A1 B1 C1` are bare send_mess,
    /// so refused ones keep the card that reported online.
    #[epics_rs::base::epics_test]
    async fn a_pie516_reporting_online_survives_refused_sets() {
        let (_line, _io) = LineRig::failing(
            "e516NoSet",
            |last| match last {
                "ONL?" | "SVO? A" => "1".into(),
                "VER?" => "E-516".into(),
                "POS? A" => "0.000".into(),
                _ => String::new(),
            },
            |cmd| cmd.starts_with("ONL 1") || cmd.starts_with("VCO"),
        );
        let (_, after) = reports(
            "drvPIE516",
            (pie516_setup_command(), vec![int(1), int(10)]),
            (
                pie516_config_command(),
                vec![int(0), port("e516NoSet"), int(0)],
            ),
        )
        .await;
        assert_eq!(
            after,
            "    PIE516 controller #0, port=e516NoSet, id: E-516 \n"
        );
    }

    /// drvPIE517.cc:148-173, which `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pie517_controller() {
        let setup = pie517_setup_command();
        let (_line, _io) = LineRig::new("e517RepLine", |last| match last {
            "SVO? 1" => "1=1".into(),
            "POS? 1" => "1=0.000".into(),
            _ => String::new(),
        });
        let (before, after) = reports(
            "drvPIE517",
            (setup, vec![int(2), int(10)]),
            (
                pie517_config_command(),
                vec![int(1), port("e517RepLine"), int(0)],
            ),
        )
        .await;
        assert_eq!(before, "    No PIE517 controllers configured.\n");
        assert_eq!(
            after,
            "    PIE517 controller 0 connection failed.\n    PIE517 controller #1, \
             port=e517RepLine, id:  \n"
        );
    }

    /// drvPIE710.cc:132-157, which `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pie710_controller() {
        let setup = pie710_setup_command();
        let (_line, _io) = LineRig::new("e710RepLine", |last| match last {
            "GI" => "PI E-710 V5.000".into(),
            "1TP" => "0.0".into(),
            _ => String::new(),
        });
        let (before, after) = reports(
            "drvPIE710",
            (setup, vec![int(2), int(10)]),
            (
                pie710_config_command(),
                vec![int(1), port("e710RepLine"), int(0)],
            ),
        )
        .await;
        assert_eq!(before, "    No PIE710 controllers configured.\n");
        assert_eq!(
            after,
            "    PIE710 controller 0 connection failed.\n    PIE710 controller #1, \
             port=e710RepLine, id: PI E-710 V5.000 \n"
        );
    }

    /// drvPIE816.cc:149-174, which `dbior` runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pie816_controller() {
        let setup = pie816_setup_command();
        let (_line, _io) = LineRig::new("e816RepLine", |last| match last {
            "*IDN?" => "E-816".into(),
            "SVO? A" => "1".into(),
            "POS? A" => "0.000".into(),
            _ => String::new(),
        });
        let (before, after) = reports(
            "drvPIE816",
            (setup, vec![int(2), int(10)]),
            (
                pie816_config_command(),
                vec![int(1), port("e816RepLine"), int(0)],
            ),
        )
        .await;
        assert_eq!(before, "    No PIE816 controllers configured.\n");
        assert_eq!(
            after,
            "    PIE816 controller 0 connection failed.\n    PIE816 controller #1, \
             port=e816RepLine, id: E-816 \n"
        );
    }
}
