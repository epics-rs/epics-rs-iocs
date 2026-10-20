//! UDF and the alarm an output ends with once its session connects, as the
//! C IOC leaves them against the open62541 test server. The write dsets clear
//! UDF only in their read, after the record's `checkAlarms`: a scalar on every
//! pop (`DataElementOpen62541Leaf.h:718`), a string only when a value lands
//! (`DataElementOpen62541Leaf.cpp:436`). So a stringout still raises
//! UDF_ALARM on the pass its first value arrives (`stringoutRecord.c:146`),
//! while an ao defines itself from VAL before the check (`aoRecord.c`
//! `prec->udf = isnan(prec->val)`), and a longout is defined by the
//! connectionLoss pass every item takes before its session connects
//! (`SessionOpen62541.cpp:2701`).

mod common;

use std::sync::Arc;
use std::time::Duration;

use async_opcua::types::{NodeId, Variant};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::recgbl::alarm_status::{NO_ALARM, READ_ALARM, UDF_ALARM};
use epics_rs::base::server::record::{AlarmSeverity, DbLinkType, DevSup, Dset};
use epics_rs::base::types::EpicsValue;
use opcua::device_support;
use opcua::registry::Registry;

use common::{Server, registry};

const fn row(name: &'static str) -> DevSup {
    DevSup {
        name,
        choice: device_support::DTYP,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(6, &[3, 4], true).with_init(device_support::opcua_init),
    }
}

static DEVICES: &[(&str, &[DevSup])] = &[
    ("stringout", &[row("devSoOpcua")]),
    ("ao", &[row("devAoOpcua")]),
    ("longout", &[row("devLoOpcua")]),
];

/// `Sim.Nope` is not on the server: its reads fail `BadNodeIdUnknown`.
const RECORDS: &str = r#"
record(stringout, "SO") { field(DTYP, "OPCUA") field(OUT, "@S1 ns=2;s=Sim.String monitor=n") }
record(stringout, "SO_BAD") { field(DTYP, "OPCUA") field(OUT, "@S1 ns=2;s=Sim.Nope monitor=n") }
record(ao, "AO") { field(DTYP, "OPCUA") field(OUT, "@S1 ns=2;s=Sim.Double monitor=n") }
record(ao, "AO_BAD") { field(DTYP, "OPCUA") field(OUT, "@S1 ns=2;s=Sim.Nope monitor=n") }
record(longout, "LO") { field(DTYP, "OPCUA") field(OUT, "@S1 ns=2;s=Sim.Int32 monitor=n") }
record(longout, "LO_BAD") { field(DTYP, "OPCUA") field(OUT, "@S1 ns=2;s=Sim.Nope monitor=n") }
"#;

async fn connected() -> (Arc<Registry>, Arc<PvDatabase>) {
    let server = Arc::new(Server {
        nodes: vec![
            (NodeId::new(2, "Sim.String"), Variant::from("x")),
            (NodeId::new(2, "Sim.Double"), Variant::Double(0.002)),
            (NodeId::new(2, "Sim.Int32"), Variant::Int32(i32::MIN)),
        ],
        ..Default::default()
    });
    let registry = registry(&server, "autoconnect=y");
    let (db, _) = IocBuilder::new()
        .register_dynamic_device_support(DEVICES, device_support::factory(registry.clone()))
        .db_string(RECORDS, &Default::default())
        .unwrap()
        .build()
        .await
        .unwrap();
    db.scan_control().run();
    registry.start();
    for (name, value) in [
        ("SO", EpicsValue::String("x".into())),
        ("AO", EpicsValue::Double(0.002)),
        ("LO", EpicsValue::Long(i32::MIN)),
    ] {
        for _ in 0..2000 {
            if db.get_pv(name).ok() == Some(value.clone()) {
                break;
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        assert_eq!(db.get_pv(name).unwrap(), value, "{name}");
    }
    // The failed reads' passes, many times over.
    tokio::time::sleep(Duration::from_millis(300)).await;
    (registry, db)
}

fn alarm(db: &PvDatabase, name: &str) -> (u16, AlarmSeverity, u8) {
    let rec = db.get_record(name).unwrap();
    let inst = rec.read();
    (inst.common.stat, inst.common.sevr, inst.common.udf)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_stringout_keeps_its_udf_alarm_on_the_pass_its_value_lands() {
    let (_registry, db) = connected().await;
    assert_eq!(alarm(&db, "SO"), (UDF_ALARM, AlarmSeverity::Invalid, 0));
    assert_eq!(alarm(&db, "SO_BAD"), (UDF_ALARM, AlarmSeverity::Invalid, 1));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_ao_is_defined_from_its_val_before_the_alarm_check() {
    let (_registry, db) = connected().await;
    assert_eq!(alarm(&db, "AO"), (NO_ALARM, AlarmSeverity::NoAlarm, 0));
    assert_eq!(
        alarm(&db, "AO_BAD"),
        (READ_ALARM, AlarmSeverity::Invalid, 0)
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_longout_is_defined_by_the_pass_before_its_session_connects() {
    let (_registry, db) = connected().await;
    assert_eq!(alarm(&db, "LO"), (NO_ALARM, AlarmSeverity::NoAlarm, 0));
    assert_eq!(
        alarm(&db, "LO_BAD"),
        (READ_ALARM, AlarmSeverity::Invalid, 0)
    );
}
