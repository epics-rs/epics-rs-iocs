//! Every update queued for a record gets a pass of its own. The C pops one
//! update per pass and, when another is queued behind it, asks for the next
//! pass (`if (nextReason) pcon->requestRecordProcessing(nextReason)`,
//! `devOpcua.cpp:408` and its 17 twins). At start every item queues a
//! connectionLoss before its session connects (`SessionOpen62541.cpp:2701`),
//! so the initial read's update is always the second one.

mod common;

use std::sync::Arc;
use std::time::Duration;

use async_opcua::types::{NodeId, Variant};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
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

static DEVICES: &[(&str, &[DevSup])] = &[("longout", &[row("devLoOpcua")])];

async fn ioc(records: usize) -> (Arc<Registry>, Arc<PvDatabase>) {
    let server = Arc::new(Server {
        nodes: vec![(NodeId::new(2, "Sim.Int32"), Variant::Int32(42))],
        ..Default::default()
    });
    let registry = registry(&server, "autoconnect=y");
    let db_text: String = (0..records)
        .map(|i| {
            format!(
                "record(longout, \"LO{i}\") {{ field(DTYP, \"OPCUA\") \
                 field(OUT, \"@S1 ns=2;s=Sim.Int32 monitor=n\") }}\n"
            )
        })
        .collect();
    let (db, _) = IocBuilder::new()
        .register_dynamic_device_support(DEVICES, device_support::factory(registry.clone()))
        .db_string(&db_text, &Default::default())
        .unwrap()
        .build()
        .await
        .unwrap();
    db.scan_control().run();
    (registry, db)
}

/// How many of the records have not taken the initial read after `wait`.
async fn not_read(db: &PvDatabase, records: usize, wait: Duration) -> usize {
    let missing = || {
        (0..records)
            .filter(|i| db.get_pv(format!("LO{i}")).ok() != Some(EpicsValue::Long(42)))
            .count()
    };
    let deadline = tokio::time::Instant::now() + wait;
    while missing() != 0 && tokio::time::Instant::now() < deadline {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    missing()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_read_queued_behind_the_connection_loss_still_reaches_the_record() {
    let (registry, db) = ioc(1).await;
    // Hold the record through start, so the connectionLoss and the initial
    // read are both queued before its first pass.
    let record = db.get_record("LO0").unwrap();
    let (held_tx, held_rx) = std::sync::mpsc::channel();
    let holder = std::thread::spawn(move || {
        let _guard = record.write();
        held_tx.send(()).unwrap();
        std::thread::sleep(Duration::from_millis(500));
    });
    held_rx.recv().unwrap();
    registry.start();
    holder.join().unwrap();
    assert_eq!(not_read(&db, 1, Duration::from_secs(2)).await, 0);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn every_record_of_a_large_database_takes_its_initial_read() {
    const RECORDS: usize = 3000;
    let (registry, db) = ioc(RECORDS).await;
    registry.start();
    assert_eq!(
        not_read(&db, RECORDS, Duration::from_secs(10)).await,
        0,
        "of {RECORDS}"
    );
}
