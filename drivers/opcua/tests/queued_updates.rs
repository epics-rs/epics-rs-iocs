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
use epics_rs::base::server::database::{ProcStack, PvDatabase};
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::base::types::EpicsValue;
use opcua::device_support;
use opcua::item::Leaf;
use opcua::queue::ProcessReason;
use opcua::registry::Registry;
use parking_lot::Mutex;

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
    ("longout", &[row("devLoOpcua")]),
    ("longin", &[row("devLiOpcua")]),
];

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

/// Whether `leaf`'s queue empties within `wait`.
async fn drained(leaf: &Mutex<Leaf>, wait: Duration) -> bool {
    let deadline = tokio::time::Instant::now() + wait;
    while !leaf.lock().queue().is_empty() && tokio::time::Instant::now() < deadline {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    leaf.lock().queue().is_empty()
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

/// The pulse is C's `callbackRequest` (`RecordConnector.cpp:200-201`), which
/// `iocPause` does not hold back the way it holds `scanIoRequest`, so an
/// update that lands while the IOC is paused is taken then, and so is the one
/// after it.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_update_that_lands_while_the_ioc_is_paused_still_reaches_the_record() {
    let (registry, db) = ioc(1).await;
    registry.start();
    assert_eq!(not_read(&db, 1, Duration::from_secs(2)).await, 0);
    let leaf = {
        let session = registry.session("S1").unwrap();
        let items = session.items.lock();
        items[0].lock().leaves[0].clone()
    };
    assert!(drained(&leaf, Duration::from_secs(2)).await);
    db.scan_control().pause();
    for reason in [ProcessReason::ReadFailure, ProcessReason::WriteFailure] {
        leaf.lock().request_processing(reason);
        assert!(drained(&leaf, Duration::from_secs(2)).await, "{reason:?}");
    }
}

/// A pass the update's pulse did not bring (a `.PROC`, a scan, a FLNK) is the
/// C's `reason == none`: `processCallback` sets `reason` around its own
/// `dbProcess` only (RecordConnector.cpp:98-110). An input record then reads
/// the node (`devOpcua.cpp:394`) and leaves the update to its pulse's pass.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_pass_no_pulse_brought_reads_and_leaves_the_queued_update() {
    let server = Arc::new(Server {
        nodes: vec![(NodeId::new(2, "Sim.Int32"), Variant::Int32(42))],
        ..Default::default()
    });
    let registry = registry(&server, "autoconnect=y");
    let (db, _) = IocBuilder::new()
        .register_dynamic_device_support(DEVICES, device_support::factory(registry.clone()))
        .db_string(
            "record(longin, \"LI\") { field(DTYP, \"OPCUA\") \
             field(INP, \"@S1 ns=2;s=Sim.Int32 monitor=n\") }\n",
            &Default::default(),
        )
        .unwrap()
        .build()
        .await
        .unwrap();
    db.scan_control().run();
    registry.start();
    let leaf = {
        let session = registry.session("S1").unwrap();
        let items = session.items.lock();
        items[0].lock().leaves[0].clone()
    };
    let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
    while db.get_pv("LI").ok() != Some(EpicsValue::Long(42))
        && tokio::time::Instant::now() < deadline
    {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert!(drained(&leaf, Duration::from_secs(2)).await);

    // The pass runs under the record, so the pulse's pass waits behind it.
    let (held_tx, held_rx) = std::sync::mpsc::channel();
    let (go_tx, go_rx) = std::sync::mpsc::channel();
    let proc_pass = {
        let (db, leaf) = (db.clone(), leaf.clone());
        std::thread::spawn(move || {
            let _gate = db.lock_record("LI");
            held_tx.send(()).unwrap();
            go_rx.recv().unwrap();
            db.process_record_with_links_already_locked("LI", &mut ProcStack::new())
                .unwrap();
            leaf.lock().queue().len()
        })
    };
    held_rx.recv().unwrap();
    leaf.lock().request_processing(ProcessReason::ReadFailure);
    go_tx.send(()).unwrap();
    assert_eq!(proc_pass.join().unwrap(), 1);
    assert!(drained(&leaf, Duration::from_secs(2)).await);
}
