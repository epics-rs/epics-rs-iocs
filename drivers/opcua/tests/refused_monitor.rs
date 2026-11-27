//! A monitored item the server refuses is the server's answer about that node,
//! not a lost session: the item stays up, and a `.PROC` of its record still
//! reads the node and raises the read's alarm.

mod common;

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use async_opcua::types::NodeId;
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::recgbl::alarm_status::READ_ALARM;
use epics_rs::base::server::record::{AlarmSeverity, DbLinkType, DevSup, Dset};
use opcua::{device_support, ioc};

use common::{Server, registry};

static DEVICES: &[(&str, &[DevSup])] = &[(
    "longin",
    &[DevSup {
        name: "devLiOpcua",
        choice: device_support::DTYP,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(6, &[3, 4], true).with_init(device_support::opcua_init),
    }],
)];

fn alarm(db: &PvDatabase, name: &str) -> (u16, AlarmSeverity) {
    let rec = db.get_record(name).unwrap();
    let inst = rec.read();
    (inst.common.stat, inst.common.sevr)
}

async fn until(what: &str, f: impl Fn() -> bool) {
    let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
    while !f() {
        assert!(tokio::time::Instant::now() < deadline, "timed out: {what}");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_record_whose_monitor_was_refused_still_reads_on_proc() {
    // The server holds no node, so it refuses the monitor `BadNodeIdUnknown`.
    let server = Arc::new(Server::default());
    let registry = registry(&server, "autoconnect=y");
    ioc::subscription_command(&registry, Some("SUB"), Some("S1"), 100.0, &[]);
    let (db, _) = IocBuilder::new()
        .register_dynamic_device_support(DEVICES, device_support::factory(registry.clone()))
        .db_string(
            "record(longin, \"LI\") { field(DTYP, \"OPCUA\") \
             field(INP, \"@SUB ns=2;s=Sim.Nope\") }\n",
            &Default::default(),
        )
        .unwrap()
        .build()
        .await
        .unwrap();
    db.scan_control().run();
    registry.start();
    until("the monitor request", || {
        server.monitors.load(Ordering::SeqCst) == 1
    })
    .await;
    // The refusal's pass, many times over.
    tokio::time::sleep(Duration::from_millis(300)).await;

    let item = registry.session("S1").unwrap().items.lock()[0].clone();
    let line = item.lock().show_line();
    assert!(
        line.contains(" state=up status=BadNodeIdUnknown "),
        "{line}"
    );
    assert_eq!(alarm(&db, "LI"), (READ_ALARM, AlarmSeverity::Invalid));

    let reads = || {
        let nope = NodeId::new(2, "Sim.Nope");
        server.reads.lock().iter().filter(|id| **id == nope).count()
    };
    let before = reads();
    db.process_record("LI").await.unwrap();
    until("the .PROC's read", || reads() > before).await;
    until("the read's alarm", || {
        alarm(&db, "LI") == (READ_ALARM, AlarmSeverity::Invalid)
    })
    .await;
}
