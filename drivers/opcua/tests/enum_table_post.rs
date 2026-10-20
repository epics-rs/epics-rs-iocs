//! The server's enumeration lands in an mbbi's state table inside the pass
//! that reads it, and `db_post_events(prec, &prec->val, DBE_PROPERTY)` goes
//! out from there (`devOpcua.cpp:710-732`), under the record's lock and ahead
//! of the value the record's `monitor()` posts on the same pass.

use std::sync::Arc;
use std::time::{Duration, SystemTime};

use async_opcua::types::{StatusCode, Variant};
use async_trait::async_trait;
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::event_queue::EventReader;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::recgbl::EventMask;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::base::types::DbFieldType;
use opcua::client::{SetupError, UaClient, UaConnector};
use opcua::device_support;
use opcua::ioc;
use opcua::queue::{ConnectionStatus, ProcessReason, Update};
use opcua::registry::Registry;
use opcua::session::SessionConfig;
use opcua::value::EnumChoices;

struct Offline;

#[async_trait]
impl UaConnector for Offline {
    async fn setup(&self, _config: &SessionConfig) -> Result<Box<dyn UaClient>, SetupError> {
        Err(SetupError::new("no server"))
    }
}

static DEVICES: &[(&str, &[DevSup])] = &[(
    "mbbi",
    &[DevSup {
        name: "devMbbiOpcua",
        choice: device_support::DTYP,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(6, &[3, 4], true).with_init(device_support::opcua_init),
    }],
)];

const DB: &str = r#"
record(mbbi, "E:MBBI") {
  field(DTYP, "OPCUA")
  field(INP, "@S1 ns=2;s=Node monitor=n")
}
"#;

/// DBE_VALUE | DBE_PROPERTY on VAL.
const VALUE_PROPERTY: u16 = 0x09;

async fn ioc() -> (Arc<Registry>, Arc<PvDatabase>) {
    let registry = Registry::new(Arc::new(Offline));
    ioc::session_command(
        &registry,
        Some("S1"),
        Some("opc.tcp://localhost:4999"),
        &["autoconnect=n"],
    );
    let (db, _) = IocBuilder::new()
        .register_dynamic_device_support(DEVICES, device_support::factory(registry.clone()))
        .db_string(DB, &Default::default())
        .unwrap()
        .build()
        .await
        .unwrap();
    db.scan_control().run();
    tokio::time::sleep(Duration::from_millis(100)).await;
    (registry, db)
}

fn subscribe(db: &PvDatabase) -> EventReader {
    db.get_record("E:MBBI")
        .unwrap()
        .write()
        .add_subscriber("VAL", 1, DbFieldType::Enum, VALUE_PROPERTY)
        .unwrap()
}

/// The server's first value, with its enumeration {3: Idle, 7: Busy}.
fn serve(registry: &Registry, v: u32) {
    let session = registry.session("S1").unwrap();
    let leaf = session.items.lock().last().unwrap().lock().leaves[0].clone();
    let mut choices = EnumChoices::new();
    choices.insert(3, "Idle".to_string());
    choices.insert(7, "Busy".to_string());
    let mut leaf = leaf.lock();
    leaf.choices = Some(Arc::new(choices));
    leaf.state = ConnectionStatus::Up;
    leaf.incoming = Some(Variant::UInt32(v));
    leaf.push(Update::new(
        ProcessReason::IncomingData,
        Some(Variant::UInt32(v)),
        StatusCode::Good,
        SystemTime::UNIX_EPOCH + Duration::from_secs(1),
    ));
}

/// Each event's mask and the labels a client reading `DBR_GR_ENUM` on it
/// then gets, until two have landed.
async fn take_two(db: &PvDatabase, reader: &mut EventReader) -> Vec<(EventMask, Vec<String>)> {
    let cell = db.get_record("E:MBBI").unwrap();
    let mut out = Vec::new();
    for _ in 0..2000 {
        while let Ok(ev) = reader.try_recv() {
            let labels = db
                .channel_snapshot_for_field(&cell, "VAL", false)
                .and_then(|s| s.enums)
                .map(|e| e.strings.iter().map(|s| s.to_string()).collect())
                .unwrap_or_default();
            out.push((ev.mask(), labels));
        }
        if out.len() >= 2 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    out
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_enumeration_is_posted_property_inside_its_pass_before_the_value() {
    let (registry, db) = ioc().await;
    let mut reader = subscribe(&db);
    serve(&registry, 7);
    let events = take_two(&db, &mut reader).await;

    assert_eq!(events.len(), 2, "{events:?}");
    assert_eq!(events[0].0, EventMask::PROPERTY, "{events:?}");
    assert!(events[1].0.contains(EventMask::VALUE), "{events:?}");
    assert!(!events[1].0.contains(EventMask::PROPERTY), "{events:?}");
    assert_eq!(&events[0].1[..2], ["Idle", "Busy"]);
    // Applied once: the next value posts no PROPERTY.
    tokio::time::sleep(Duration::from_millis(50)).await;
    while reader.try_recv().is_ok() {}
    serve(&registry, 3);
    tokio::time::sleep(Duration::from_millis(200)).await;
    while let Ok(ev) = reader.try_recv() {
        assert!(!ev.mask().contains(EventMask::PROPERTY));
    }
}
