//! When an output record writes to the server, through the real database: on
//! its own process (a put), never on the pass an update from the server
//! processes it — the C's write dsets take that update into the record instead
//! (`devOpcua.cpp:418-435`) — and never for an element of an `opcuaItem`
//! record, whose item record decides when the node goes out
//! (`devOpcua.cpp:428`, `linkedToItem`).

mod common;

use std::sync::Arc;
use std::time::Duration;

use async_opcua::types::{NodeId, Variant};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::base::types::EpicsValue;
use opcua::device_support;
use opcua::ioc;
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
    ("ao", &[row("devAoOpcua")]),
    (
        "opcuaItem",
        &[DevSup {
            name: "devItemOpcua",
            choice: device_support::DTYP,
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[3, 4], false),
        }],
    ),
];

/// The value `Sim.Double` holds on the server double.
const ON_SERVER: f64 = 33.25;

/// An IOC on `db` whose session `S1` is up, with each record holding the
/// value its initial read brought.
async fn ioc(db: &str, settled: &[&str]) -> (Arc<Server>, Arc<Registry>, Arc<PvDatabase>) {
    let server = Arc::new(Server {
        nodes: vec![(NodeId::new(2, "Sim.Double"), Variant::Double(ON_SERVER))],
        ..Default::default()
    });
    let registry = registry(&server, "autoconnect=y");
    let (name, factory) = ioc::item_record_factory();
    let (db, _) = IocBuilder::new()
        .register_record_type(name, factory)
        .register_dynamic_device_support(DEVICES, device_support::factory(registry.clone()))
        .db_string(db, &Default::default())
        .unwrap()
        .build()
        .await
        .unwrap();
    // `iocRun`: the I/O Intr pulses the updates bring may now process.
    db.scan_control().run();
    registry.start();
    for record in settled {
        until(|| db.get_pv(record).ok() == Some(EpicsValue::Double(ON_SERVER))).await;
    }
    (server, registry, db)
}

async fn until(f: impl Fn() -> bool) {
    for _ in 0..2000 {
        if f() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    panic!("the condition did not hold within two seconds");
}

/// Long enough for a write the session would send, and for the pass its
/// completion brings, many times over.
async fn quiet() {
    tokio::time::sleep(Duration::from_millis(300)).await;
}

fn written(server: &Server) -> Vec<Option<Variant>> {
    server
        .writes
        .lock()
        .iter()
        .flatten()
        .map(|w| w.value.value.clone())
        .collect()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_output_writes_once_per_put_and_not_for_the_updates_it_takes() {
    let (server, _registry, db) = ioc(
        r#"
record(ao, "AO") {
  field(DTYP, "OPCUA")
  field(OUT, "@S1 ns=2;s=Sim.Double monitor=n")
}
"#,
        &["AO"],
    )
    .await;
    // The initial read processed the record; it wrote nothing back.
    quiet().await;
    assert_eq!(written(&server), []);

    db.put_record_field_from_ca("AO", "VAL", EpicsValue::Double(7.5))
        .await
        .unwrap();
    until(|| !server.writes.lock().is_empty()).await;
    // The write's completion processes the record again, and that pass takes
    // the completion: it does not send the value a second time.
    quiet().await;
    assert_eq!(written(&server), [Some(Variant::Double(7.5))]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_element_leaves_the_write_to_its_item_record() {
    let (server, _registry, db) = ioc(
        r#"
record(opcuaItem, "IT1") {
  field(DTYP, "OPCUA")
  field(INP, "@S1 ns=2;s=Sim.Double monitor=n")
}
record(ao, "AO_E") {
  field(DTYP, "OPCUA")
  field(OUT, "@IT1 monitor=n")
}
"#,
        &["AO_E"],
    )
    .await;

    db.put_record_field_from_ca("AO_E", "VAL", EpicsValue::Double(7.5))
        .await
        .unwrap();
    quiet().await;
    assert_eq!(written(&server), [], "the element waits for its item");

    db.put_record_field_from_ca("IT1", "WRITE", EpicsValue::Char(1))
        .await
        .unwrap();
    until(|| !server.writes.lock().is_empty()).await;
    quiet().await;
    assert_eq!(written(&server), [Some(Variant::Double(7.5))]);
}
