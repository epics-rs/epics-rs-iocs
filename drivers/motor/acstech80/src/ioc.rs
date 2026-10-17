//! `SPiiPlusSetup` / `SPiiPlusConfig` iocsh commands for the ACS Tech80
//! SPiiPlus, C's model-1 interface: records reach axis `signal` of `card`
//! through `DTYP "SPiiPlus"` and `OUT "#C card S signal"`. Config keeps
//! the port; `motor_init` at iocInit connects and identifies the controller.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, opt_string, req_string};
use motor_common::model1::{Model1, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::spiiplus::{CommandMode, INPUT_EOS, OUTPUT_EOS, SpiiPlusAxis, SpiiPlusController};

/// Serial command timeout (C `TIMEOUT` 2 s).
const SPIIPLUS_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devSPiiPlus,"SPiiPlus")` (devSPiiPlus.dbd:2).
static DEV_SPIIPLUS: DevSup = model1_device("devSPiiPlus", "SPiiPlus");

/// `SPiiPlus_NUM_CARDS` (drvSPiiPlus.cc:67).
static SPIIPLUS: Model1 = Model1::new(&DEV_SPIIPLUS, 8, SCAN_RATE);

/// `SPiiPlusSetup(num_cards, scan_rate)`.
pub fn spiiplus_setup_command() -> CommandDef {
    SPIIPLUS.register_report("drvSPiiPlus", |cards| {
        cards.report("SPiiPlus", |card, line| {
            format!("    SPiiPlus controller {card}, {line} \n")
        })
    });
    SPIIPLUS.setup_command("SPiiPlusSetup")
}

/// `SPiiPlusConfig(card, name, mode)`.
pub fn spiiplus_config_command() -> CommandDef {
    CommandDef::new(
        "SPiiPlusConfig",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_str_req("Command Mode [BUFfer/CONnect/DIRect]"),
        ],
        "SPiiPlusConfig(card, name, mode)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !SPIIPLUS.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let mode = CommandMode::parse(&opt_string(args, 2)?.unwrap_or_default());
            ctx.println(&format!("SPiiPlus config mode = {}", mode as i32));

            SPIIPLUS.config(ctx, card, move |init| {
                let Ok(handle) = connect_with_eos(
                    &asyn_port,
                    // C connects at an asyn_address SPiiPlusConfig never sets (drvSPiiPlus.cc:682).
                    0,
                    SPIIPLUS_TIMEOUT,
                    INPUT_EOS,
                    OUTPUT_EOS,
                    EosRefusal::Ignored,
                ) else {
                    return;
                };
                let Ok(controller) = SpiiPlusController::new(handle, mode) else {
                    return;
                };
                let num_axes = controller.num_axes();
                let line = format!("port={asyn_port}, address=0, id: {}", controller.ident());
                let controller = Arc::new(Mutex::new(controller));
                for index in 0..num_axes {
                    let ax = SpiiPlusAxis::new(controller.clone(), index);
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(ax));
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
    use epics_rs::base::runtime::task::BlockingBridge;
    use epics_rs::base::server::database::PvDatabase;
    use epics_rs::base::server::iocsh::IocShell;
    use motor_common::testing::{LineRig, driver_report, drvsup_init, run_command};

    /// drvSPiiPlus.cc:638 prints the parsed mode at Config, BUFFER 0,
    /// CONNECT 1, DIRECT 2 (drvSPiiPlus.h:79).
    #[epics_rs::base::epics_test]
    async fn config_prints_the_command_mode() {
        let int = ArgValue::Int;
        run_command(spiiplus_setup_command(), vec![int(3), int(10)]).await;
        let out = std::env::temp_dir().join(format!("spiiplus-mode-{}", std::process::id()));
        let shell = IocShell::new(Arc::new(PvDatabase::new()), BlockingBridge::capture());
        shell.register(spiiplus_config_command());
        for (card, mode) in [(0, "buffer"), (1, "con"), (2, "Dir")] {
            let line = format!(
                "SPiiPlusConfig({card}, \"spRig\", \"{mode}\") >> {}",
                out.display()
            );
            shell.execute_line(line).unwrap();
        }
        let printed = std::fs::read_to_string(&out).unwrap();
        let _ = std::fs::remove_file(&out);
        assert_eq!(
            printed,
            "SPiiPlus config mode = 0\nSPiiPlus config mode = 1\nSPiiPlus config mode = 2\n"
        );
    }

    /// drvSPiiPlus.cc's drvet `report` (drvSPiiPlus.cc:159-185), which
    /// `dbior` runs at every level. C prints an `asyn_address` SPiiPlusConfig
    /// never sets; 0 is the address the line is opened at.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_spiiplus_card() {
        let int = ArgValue::Int;
        let setup = spiiplus_setup_command();
        assert_eq!(
            driver_report("drvSPiiPlus", 0),
            "    No SPiiPlus controllers configured.\n"
        );
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("spRepLine", |cmd| match cmd.trim_end() {
            "?VR" => "SPiiPlus 6.50".into(),
            "?APOS(0)" => "0".into(),
            "?APOS(1)" => "?1033".into(),
            _ => ":".into(),
        });
        let config = vec![
            int(1),
            ArgValue::String("spRepLine".into()),
            ArgValue::String("BUF".into()),
        ];
        run_command(spiiplus_config_command(), config).await;
        drvsup_init().await;
        for level in [0, 1, 2] {
            assert_eq!(
                driver_report("drvSPiiPlus", level),
                "    SPiiPlus controller 0 connection failed.\n    SPiiPlus controller 1, \
                 port=spRepLine, address=0, id: SPiiPlus 6.50 \n"
            );
        }
    }
}
