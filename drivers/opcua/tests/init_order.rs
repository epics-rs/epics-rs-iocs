//! The order a session's items are created in, through the real init passes:
//! it is the order of the initial read, of registerNodes and of the monitored
//! items. C creates an `opcuaItem` record's item in its `init_record` pass 0
//! (`opcuaItemRecord.cpp:57-80`) and every other record's in
//! `opcua_add_record`, after pass 0 (`devOpcua.cpp:88-110`).

use std::sync::Arc;

use async_trait::async_trait;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use opcua::client::{SetupError, UaClient, UaConnector};
use opcua::device_support;
use opcua::ioc;
use opcua::registry::Registry;
use opcua::session::SessionConfig;

struct Offline;

#[async_trait]
impl UaConnector for Offline {
    async fn setup(&self, _config: &SessionConfig) -> Result<Box<dyn UaClient>, SetupError> {
        Err(SetupError::new("no server"))
    }
}

const fn row(name: &'static str) -> DevSup {
    DevSup {
        name,
        choice: device_support::DTYP,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(6, &[3, 4], true).with_init(device_support::opcua_init),
    }
}

static DEVICES: &[(&str, &[DevSup])] = &[
    ("ai", &[row("devAiOpcua")]),
    ("longout", &[row("devLoOpcua")]),
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

// The item record names its DTYP: an empty one does not reach its device
// menu's first row in epics-base-rs yet (C does, `iocInit.c`).
const DB: &str = r#"
record(ai, "R:AI") {
  field(DTYP, "OPCUA")
  field(INP, "@SUB1 ns=2;s=Demo.Static.Scalar.Double")
}
record(longout, "R:LO") {
  field(DTYP, "OPCUA")
  field(OUT, "@S1 ns=3;i=1234 monitor=n")
}
record(opcuaItem, "R:ITEM") {
  field(DTYP, "OPCUA")
  field(INP, "@SUB1 ns=2;s=Demo.Struct")
}
record(ai, "R:EL") {
  field(DTYP, "OPCUA")
  field(INP, "@R:ITEM element=a")
}
"#;

#[test]
fn an_item_records_item_comes_before_every_device_records() {
    let registry = Registry::new(Arc::new(Offline));
    ioc::session_command(
        &registry,
        Some("S1"),
        Some("opc.tcp://localhost:4999"),
        &["autoconnect=n"],
    );
    ioc::subscription_command(&registry, Some("SUB1"), Some("S1"), 250.0, &[]);

    let built = registry.clone();
    std::thread::spawn(move || {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(async {
                let (name, factory) = ioc::item_record_factory();
                IocBuilder::new()
                    .register_record_type(name, factory)
                    .register_dynamic_device_support(DEVICES, device_support::factory(built))
                    .db_string(DB, &Default::default())
                    .unwrap()
                    .build()
                    .await
                    .map(|_| ())
                    .unwrap();
            })
    })
    .join()
    .expect("builder thread");

    let session = registry.session("S1").unwrap();
    let order: Vec<String> = session
        .items
        .lock()
        .iter()
        .map(|item| item.lock().addressing_record())
        .collect();
    assert_eq!(order, ["R:ITEM", "R:AI", "R:LO"]);
}
