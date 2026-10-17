//! `SC800Setup` / `SC800Config` iocsh commands for the Kohzu SC-200/400/800,
//! C's model-1 interface: records reach axis `signal` of `card` through
//! `DTYP "SC-800"` and `OUT "#C card S signal"`.
//!
//! Config keeps the port and address; `motor_init` at iocInit connects and
//! identifies the controller, and the axis count comes from the model
//! (SC-800 → 8, SC-400 → 4, SC-200 → 2).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::kohzu::{INPUT_EOS, KohzuAxis, KohzuController, OUTPUT_EOS};

/// Serial command timeout (C `TIMEOUT` 3 s).
const SC800_TIMEOUT: Duration = Duration::from_secs(3);

/// `device(motor,VME_IO, devSC800, "SC-800")` (devKohzuMotor.dbd:4).
static DEV_SC800: DevSup = model1_device("devSC800", "SC-800");

/// `SC800_NUM_CARDS` (drvSC800.cc:59).
static SC800: Model1 = Model1::new(&DEV_SC800, 16, SCAN_RATE);

/// `SC800Setup(num_cards, scan_rate)`.
pub fn sc800_setup_command() -> CommandDef {
    SC800.register_report("drvSC800", |cards| {
        cards.report("SC800", |card, line| {
            format!("    SC800 controller {card}, {line} \n")
        })
    });
    SC800.setup_command("SC800Setup")
}

/// `SC800Config(card, name, address)`; `address` is the asyn address C
/// connects at.
pub fn sc800_config_command() -> CommandDef {
    CommandDef::new(
        "SC800Config",
        vec![
            arg_int_req("Card# being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("asyn address (GPIB)"),
        ],
        "SC800Config(card, name, address)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !SC800.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let address = c_int(args, 2);
            SC800.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    address,
                    SC800_TIMEOUT,
                    INPUT_EOS,
                    OUTPUT_EOS,
                    EosRefusal::Ignored,
                ) else {
                    errlog_vprintf(format_args!(
                        "\ndrvSC800.cc:motor_init() *** asyn connection error on port = \
                         {asyn_port}, address = {address} ***\n\n"
                    ));
                    std::thread::sleep(Duration::from_secs(5));
                    return;
                };
                let Ok(controller) = KohzuController::new(handle) else {
                    return;
                };
                let num_axes = controller.num_axes();
                let line = format!(
                    "port={asyn_port}, address={address}, id: {}",
                    controller.ident()
                );
                let controller = Arc::new(Mutex::new(controller));
                for index in 0..num_axes {
                    let motor: Arc<Mutex<dyn AsynMotor>> =
                        Arc::new(Mutex::new(KohzuAxis::new(controller.clone(), index)));
                    init.axis(index as i32, motor);
                }
                init.describe(line);
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::{
        LineRig, driver_report, drvsup_init, ioc_init_errlog, run_command,
    };

    /// drvSC800.cc:537-538 connects at the Config's `address`.
    #[epics_rs::base::epics_test]
    async fn sc800_talks_at_the_config_address() {
        let int = ArgValue::Int;
        run_command(sc800_setup_command(), vec![int(1), int(10)]).await;
        let (line, _io) = LineRig::multi_device("sc800Gpib", 8, |cmd| {
            match cmd.trim_start_matches('\u{2}').trim_end() {
                "IDN" => "C\tIDN0\t200\t1".into(),
                c => format!("C\t{c}"),
            }
        });
        let config = vec![int(0), ArgValue::String("sc800Gpib".into()), int(5)];
        run_command(sc800_config_command(), config).await;
        drvsup_init().await;
        let addrs = line.addresses();
        assert!(!addrs.is_empty());
        assert!(addrs.iter().all(|&a| a == 5), "{addrs:?}");
    }

    /// drvSC800.cc:541-545: a connect refused at motor_init is the one card
    /// failure C prints, then it sleeps 5 s.
    #[epics_rs::base::epics_test]
    async fn sc800_prints_a_connect_error_at_ioc_init() {
        let int = ArgValue::Int;
        run_command(sc800_setup_command(), vec![int(1), int(10)]).await;
        let config = vec![int(0), ArgValue::String("sc800Nowhere".into()), int(3)];
        run_command(sc800_config_command(), config).await;
        let start = std::time::Instant::now();
        let lines = ioc_init_errlog("").await;
        assert!(start.elapsed() >= Duration::from_secs(5));
        assert_eq!(
            lines,
            [
                "\ndrvSC800.cc:motor_init() *** asyn connection error on port = sc800Nowhere, \
              address = 3 ***"
            ]
        );
    }

    /// drvSC800.cc:557-578: IDN goes out up to 3 times, until a reply names
    /// an SC-800, 400 or 200.
    #[epics_rs::base::epics_test]
    async fn sc800_sends_idn_up_to_three_times() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let int = ArgValue::Int;
        run_command(sc800_setup_command(), vec![int(2), int(10)]).await;
        let idn = |name: &str, good_on: usize| {
            let seen = AtomicUsize::new(0);
            LineRig::new(name, move |cmd| {
                match cmd.trim_start_matches('\u{2}').trim_end() {
                    "IDN" if seen.fetch_add(1, Ordering::SeqCst) + 1 == good_on => {
                        "C\tIDN0\t200\t1".into()
                    }
                    "IDN" => "C\tIDN0\t?".into(),
                    c => format!("C\t{c}"),
                }
            })
        };
        let (third, _io3) = idn("sc800Third", 3);
        let (never, _io0) = idn("sc800Never", 0);
        for (card, port) in [(0, "sc800Third"), (1, "sc800Never")] {
            let config = vec![int(card), ArgValue::String(port.into()), int(0)];
            run_command(sc800_config_command(), config).await;
        }
        let db = r##"record(motor, "sc800:m0") {
            field(DTYP, "SC-800")
            field(OUT, "#C0 S0 @")
        }
        record(motor, "sc800:m1") {
            field(DTYP, "SC-800")
            field(OUT, "#C1 S0 @")
        }"##;
        assert_eq!(
            ioc_init_errlog(db).await,
            [
                "recGblRecordError: motor_init_record_com(): card does not exist! \
              Illegal field value PV: sc800:m1"
            ]
        );
        let idns = |sent: Vec<String>| sent.iter().filter(|c| c.contains("IDN")).count();
        assert_eq!(idns(third.sent()), 3);
        assert_eq!(idns(never.sent()), 3);
    }

    /// drvSC800.cc's drvet `report` (drvSC800.cc:144-170), which `dbior` runs
    /// at every level.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_sc800_card() {
        let int = ArgValue::Int;
        let setup = sc800_setup_command();
        assert_eq!(
            driver_report("drvSC800", 0),
            "    No SC800 controllers configured.\n"
        );
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::multi_device("sc800RepGpib", 8, |cmd| {
            match cmd.trim_start_matches('\u{2}').trim_end() {
                "IDN" => "C\tIDN0\t400\t7".into(),
                c => format!("C\t{c}"),
            }
        });
        let config = vec![int(1), ArgValue::String("sc800RepGpib".into()), int(4)];
        run_command(sc800_config_command(), config).await;
        drvsup_init().await;
        for level in [0, 1, 2] {
            assert_eq!(
                driver_report("drvSC800", level),
                "    SC800 controller 0 connection failed.\n    SC800 controller 1, \
                 port=sc800RepGpib, address=4, id: SC-400 Ver7 \n"
            );
        }
    }
}
