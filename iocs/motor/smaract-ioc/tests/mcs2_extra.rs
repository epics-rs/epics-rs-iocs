//! `db/MCS2_Extra.db` on an MCS2 controller: `Ref-Sts` follows
//! IS_REFERENCED (0x80) of the channel state the poll posts to
//! `positionerStatusRb` (smarActMCS2MotorDriver.cpp's `pstatrb_`).

#![cfg(target_os = "linux")]

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicI64, Ordering};
use std::time::Duration;

use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use epics_rs::base::types::EpicsValue;
use motor_common::testing::LineRig;

/// What `:CHAN0:STAT?` answers.
static STATE: AtomicI64 = AtomicI64::new(0);

const IS_REFERENCED: i64 = 0x80;
/// SENSOR_PRESENT | IS_CALIBRATED: set beside IS_REFERENCED, so `MASK` has
/// to keep them out.
const CALIBRATED: i64 = 0x60;

fn mcs2_reply(cmd: &str) -> String {
    let reply = match cmd.trim_end() {
        ":SYST:ERR:COUN?" => "0".to_owned(),
        ":DEV:SNUM?" => "MCS2-00001".to_owned(),
        ":CHAN0:STAT?" => STATE.load(Ordering::SeqCst).to_string(),
        ":CHAN0:POS?" | ":CHAN0:POS:TARG?" => "0".to_owned(),
        ":CHAN0:AMPL?" => "1".to_owned(),
        ":CHAN0:PTYP?" => "300".to_owned(),
        ":CHAN0:MCLF?" => "5000".to_owned(),
        ":CHAN0:HOLD?" => "60".to_owned(),
        _ => String::new(),
    };
    format!("{reply}\r\n")
}

/// Wait for `pv` to read `want`.
async fn until(db: &PvDatabase, pv: &str, want: EpicsValue) {
    for _ in 0..1000 {
        if db.get_pv(pv).unwrap() == want {
            return;
        }
        sleep(Duration::from_millis(10)).await;
    }
    panic!("{pv} is {:?}, not {want:?}", db.get_pv(pv).unwrap());
}

#[epics_rs::base::epics_test]
async fn ref_sts_follows_is_referenced() {
    STATE.store(CALIBRATED, Ordering::SeqCst);
    let (_line, _io) = LineRig::new("REF_IO", mcs2_reply);
    let db = Arc::new(PvDatabase::new());

    // st.cmd's MCS2CreateController, before iocInit, on the IOC's database.
    let (shell_db, bridge) = (db.clone(), BlockingBridge::capture());
    let shell = std::thread::spawn(move || {
        let mut args = vec![
            ArgValue::String("REF".into()),
            ArgValue::String("REF_IO".into()),
        ];
        args.extend([1, 10, 10, 0].map(ArgValue::Int));
        let cmd = motor_smaract::ioc::mcs2_create_controller_command();
        cmd.handler
            .call(&args, &CommandContext::new(shell_db, bridge))
            .unwrap();
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap();

    let macros: HashMap<String, String> = [
        ("P", "REF:"),
        ("M", "m1:"),
        ("PORT", "REF"),
        ("ADDR", "0"),
        ("TIMEOUT", "1.0"),
    ]
    .map(|(k, v)| (k.to_owned(), v.to_owned()))
    .into();
    let extra = concat!(env!("CARGO_MANIFEST_DIR"), "/db/MCS2_Extra.db");
    let ioc = epics_rs::asyn::adapter::register_asyn_device_support_for_builder(IocBuilder::new())
        .with_database(db.clone())
        .db_file(extra, &macros)
        .unwrap()
        .build_ioc()
        .await
        .unwrap();
    assert!(ioc.run().await);

    for state in [CALIBRATED, CALIBRATED | IS_REFERENCED, CALIBRATED] {
        STATE.store(state, Ordering::SeqCst);
        until(
            &db,
            "REF:m1:positionerStatusRb",
            EpicsValue::Long(state as i32),
        )
        .await;
        let referenced = (state & IS_REFERENCED != 0) as u16;
        until(&db, "REF:m1:Ref-Sts", EpicsValue::Enum(referenced)).await;
        assert_eq!(
            db.get_pv("REF:m1:Ref-Sts.RVAL").unwrap(),
            EpicsValue::ULong((state & IS_REFERENCED) as u32)
        );
    }
}
