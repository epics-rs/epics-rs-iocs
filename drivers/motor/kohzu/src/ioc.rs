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
                    return;
                };
                let Ok(controller) = KohzuController::new(handle) else {
                    return;
                };
                let num_axes = controller.num_axes();
                let controller = Arc::new(Mutex::new(controller));
                for index in 0..num_axes {
                    let motor: Arc<Mutex<dyn AsynMotor>> =
                        Arc::new(Mutex::new(KohzuAxis::new(controller.clone(), index)));
                    init.axis(index as i32, motor);
                }
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::{LineRig, drvsup_init, run_command};

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
}
