//! `PM304Setup` / `PM304Config` iocsh commands for the Mclennan PM304/PM600,
//! C's model-1 interface: records reach axis `signal` of `card` through
//! `DTYP "Mclennan PM304"` and `OUT "#C card S signal"`. Config keeps the
//! port; `motor_init` at iocInit connects and identifies the controller.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::Model1Motor;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::DevSup;

use motor_common::connect::connect_serial;
use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::{Model1, Model1Cards, SCAN_RATE, c_int};
use motor_common::model1_device;

use crate::pm304::{Pm304Axis, Pm304Controller};

/// Serial command timeout (C `TIMEOUT` 2 s).
const PM304_TIMEOUT: Duration = Duration::from_secs(2);

/// `device(motor,VME_IO,devPM304,"Mclennan PM304")` (devMclennanMotor.dbd:2).
static DEV_PM304: DevSup = model1_device("devPM304", "Mclennan PM304");

/// `PM304_NUM_CARDS` (drvPM304.h:27).
static PM304: Model1 = Model1::new(&DEV_PM304, 4, SCAN_RATE);

/// drvPM304's drvet `report` (drvPM304.cc:149-162): a card whose
/// `motor_init` failed prints nothing.
fn pm304_report(cards: &Model1Cards<'_>) -> String {
    if cards.num_cards() <= 0 {
        return "    NO PM304 controllers found\n".into();
    }
    (0..cards.num_cards())
        .filter_map(|card| {
            let ident = cards.card(card)?;
            Some(format!("    PM304 controller {card}, id: {ident} \n"))
        })
        .collect()
}

/// `PM304Setup(num_cards, scan_rate)`.
pub fn pm304_setup_command() -> CommandDef {
    PM304.register_report("drvPM304", pm304_report);
    PM304.setup_command("PM304Setup")
}

/// `PM304Config(card, name, n_axes)`.
pub fn pm304_config_command() -> CommandDef {
    CommandDef::new(
        "PM304Config",
        vec![
            arg_int_req("Card being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("Number of axes"),
        ],
        "PM304Config(card, name, n_axes)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            if !PM304.has_card(card) {
                return Ok(CommandOutcome::Continue);
            }
            let asyn_port = req_string(args, 1, "asyn port name")?;
            let mut n_axes = c_int(args, 2);
            if n_axes == 0 {
                n_axes = 1; // C: older startup files omit it.
            }
            let n_axes = usize::try_from(n_axes).map_err(|_| "PM304Config: n_axes < 0")?;

            PM304.config(card, move |init| {
                let Ok(handle) = connect_serial(&asyn_port, 0, PM304_TIMEOUT) else {
                    return;
                };
                let Ok(controller) = Pm304Controller::new(handle, n_axes) else {
                    return;
                };
                let num_axes = controller.num_axes();
                let ident = controller.ident().to_string();
                let controller = Arc::new(Mutex::new(controller));
                for index in 0..num_axes {
                    let ax = Pm304Axis::new(controller.clone(), index);
                    let motor: Arc<Mutex<dyn Model1Motor>> = Arc::new(Mutex::new(ax));
                    init.axis(index as i32, motor);
                }
                init.describe(ident);
            })?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod report {
    use super::*;
    use motor_common::testing::{LineRig, driver_report, drvsup_init, run_command};

    /// drvPM304.cc's drvet `report` (drvPM304.cc:149-162), which `dbior`
    /// runs.
    #[epics_rs::base::epics_test]
    async fn dbior_prints_each_pm304_card() {
        let setup = pm304_setup_command();
        assert_eq!(
            driver_report("drvPM304", 0),
            "    NO PM304 controllers found\n"
        );
        let int = ArgValue::Int;
        run_command(setup, vec![int(2), int(10)]).await;
        let (_line, _io) = LineRig::new("pm304RepLine", |cmd| match cmd {
            "1ID" => "Mclennan PM304 v1".into(),
            _ => "OK".into(),
        });
        let config = vec![int(1), ArgValue::String("pm304RepLine".into()), int(1)];
        run_command(pm304_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvPM304", 0),
            "    PM304 controller 1, id: Mclennan PM304 v1 \n"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::{
        LineRig, call_command, driver_report, drvsup_init, ioc_init_errlog, run_command,
    };

    /// C's PM304Config only stores the port (drvPM304.cc:497-509); motor_init
    /// drops a card that does not answer without a word, and its record
    /// says so (motordevCom.cc:231-235).
    #[epics_rs::base::epics_test]
    async fn a_silent_card_fails_its_record_not_its_config() {
        let (_line, _io) = LineRig::new("pm304Silent", |_| String::new());
        run_command(
            pm304_setup_command(),
            vec![ArgValue::Int(1), ArgValue::Int(10)],
        )
        .await;
        let config = vec![
            ArgValue::Int(0),
            ArgValue::String("pm304Silent".into()),
            ArgValue::Int(1),
        ];
        assert!(call_command(pm304_config_command(), config).await.is_ok());
        let db = r##"record(motor, "pm304:m1") {
            field(DTYP, "Mclennan PM304")
            field(OUT, "#C0 S0 @")
        }"##;
        assert_eq!(
            ioc_init_errlog(db).await,
            vec![
                "recGblRecordError: motor_init_record_com(): card does not exist! \
                 Illegal field value PV: pm304:m1"
            ]
        );
    }

    /// drvPM304.cc:643-648: `1OA` probes the card up to 3 times before
    /// `1ST` and `1ID`; a card silent to all three gets nothing more.
    #[epics_rs::base::epics_test]
    async fn motor_init_probes_1oa_before_identifying() {
        let int = ArgValue::Int;
        run_command(pm304_setup_command(), vec![int(2), int(10)]).await;
        let (live, _io1) = LineRig::new("pm304Live", |cmd| match cmd.trim_end() {
            "1ID" => "Mclennan PM304 V6.11".into(),
            _ => "OK".into(),
        });
        let (mute, _io2) = LineRig::new("pm304Mute", |_| String::new());
        for (card, port) in [(0, "pm304Live"), (1, "pm304Mute")] {
            let config = vec![int(card), ArgValue::String(port.into()), int(1)];
            run_command(pm304_config_command(), config).await;
        }
        drvsup_init().await;
        let sent = |line: &LineRig| -> Vec<String> {
            line.sent()
                .iter()
                .map(|c| c.trim_end().to_owned())
                .collect()
        };
        assert_eq!(sent(&live)[..3], ["1OA", "1ST", "1ID"]);
        assert_eq!(sent(&mute), ["1OA", "1OA", "1OA"]);
    }

    /// drvPM304.cc:653-700: only the `1OA` probe decides the card; a silent
    /// `1ID` leaves an empty ident and a PM600 guess, and the card stays.
    #[epics_rs::base::epics_test]
    async fn r43_card_answering_1oa_survives_silent_1id() {
        let int = ArgValue::Int;
        run_command(pm304_setup_command(), vec![int(1), int(10)]).await;
        let (_line, _io) = LineRig::new("pm304NoId", |cmd| match cmd.trim_end() {
            "1OA" => "0".into(),
            _ => String::new(),
        });
        let config = vec![int(0), ArgValue::String("pm304NoId".into()), int(1)];
        run_command(pm304_config_command(), config).await;
        drvsup_init().await;
        assert_eq!(
            driver_report("drvPM304", 0),
            "    PM304 controller 0, id:  \n"
        );
    }
}

#[cfg(test)]
mod init_status {
    use super::*;
    use epics_rs::base::types::EpicsValue;
    use motor_common::testing::{LineRig, drvsup_init, record_init, run_command};

    /// drvPM304.cc:674-700: `motor_init` reads each axis's `ID` and then its
    /// status once, before the next axis's `ID`; the record starts from that
    /// read.
    #[epics_rs::base::epics_test]
    async fn motor_init_reads_each_status_after_its_own_id() {
        let int = ArgValue::Int;
        run_command(pm304_setup_command(), vec![int(1), int(10)]).await;
        let (line, _io) = LineRig::new("pm304InitLine", |cmd| match cmd.trim_end() {
            c if c.ends_with("ID") => "Mclennan PM304".into(),
            c if c.ends_with("OS") => "00010000".into(),
            "1OA" => "AP=14".into(),
            "2OA" => "AP=3".into(),
            _ => "OK".into(),
        });
        let config = vec![int(0), ArgValue::String("pm304InitLine".into()), int(2)];
        run_command(pm304_config_command(), config).await;
        drvsup_init().await;
        let sent: Vec<String> = line
            .sent()
            .iter()
            .map(|m| m.trim_end().to_owned())
            .collect();
        assert_eq!(
            sent,
            [
                "1OA", "1ST", "1ID", "1ID", "1OS", "1OA", "2ID", "2OS", "2OA"
            ]
        );
        let db = record_init(
            r##"
record(motor, "pm304:m1") { field(DTYP, "Mclennan PM304") field(OUT, "#C0 S0 @") }
record(motor, "pm304:m2") { field(DTYP, "Mclennan PM304") field(OUT, "#C0 S1 @") }
"##,
        )
        .await;
        assert_eq!(db.get_pv("pm304:m1.RMP").unwrap(), EpicsValue::Long(14));
        assert_eq!(db.get_pv("pm304:m2.RMP").unwrap(), EpicsValue::Long(3));
    }
}
