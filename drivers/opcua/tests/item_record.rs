//! The `opcuaItem` record: the record type (`opcuaItemRecord.cpp`) and the
//! action its device support takes for it (`opcua_action_item`).
//!
//! The record has no value of its own. It exists so that several records can
//! bind to the elements of one structured node, and so that the read or the
//! write of the whole node can be ordered from the database: READ, WRITE,
//! DEFACTN and WOC.
//!
//! The tests that only need a reason to process drive the record's queue
//! directly; the ones that assert an OPC UA service call actually went out run
//! the session worker against the client-boundary double.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use async_opcua::types::{
    AttributeId, DataValue, NodeId, ReadValueId, StatusCode, Variant, WriteValue,
};
use async_trait::async_trait;
use parking_lot::Mutex;
use tokio::sync::mpsc;

use epics_rs::base::server::device_support::DeviceSupport;
use epics_rs::base::server::record::{Record, ScanType};
use epics_rs::base::server::records::longout::LongoutRecord;
use epics_rs::base::types::EpicsValue;

use opcua::client::{Notification, ServerInfo, UaConnection, UaConnector};
use opcua::device_support::OpcuaDevice;
use opcua::item::Leaf;
use opcua::queue::{ConnectionStatus, ProcessReason, Update};
use opcua::record::OpcuaItemRecord;
use opcua::registry::Registry;
use opcua::session::SessionConfig;
use opcua::subscription::SubscriptionConfig;

// --------------------------------------------------------------- the test double

#[derive(Default)]
struct Calls {
    reads: Vec<Vec<ReadValueId>>,
    writes: Vec<Vec<WriteValue>>,
}

#[derive(Default)]
struct Mock {
    calls: Mutex<Calls>,
    values: Mutex<HashMap<NodeId, Variant>>,
}

struct MockConnector(Arc<Mock>);

#[async_trait]
impl UaConnector for MockConnector {
    async fn connect(&self, _config: &SessionConfig) -> Result<Arc<dyn UaConnection>, String> {
        Ok(self.0.clone())
    }
}

#[async_trait]
impl UaConnection for Mock {
    async fn read(&self, nodes: &[ReadValueId]) -> Result<Vec<DataValue>, StatusCode> {
        self.calls.lock().reads.push(nodes.to_vec());
        let values = self.values.lock();
        Ok(nodes
            .iter()
            .map(|node| {
                if node.attribute_id == AttributeId::DataType as u32 {
                    // Int32 — i=6.
                    DataValue::value_only(Variant::NodeId(Box::new(NodeId::new(0, 6u32))))
                } else {
                    DataValue {
                        value: Some(
                            values
                                .get(&node.node_id)
                                .cloned()
                                .unwrap_or(Variant::Int32(42)),
                        ),
                        status: Some(StatusCode::Good),
                        ..Default::default()
                    }
                }
            })
            .collect())
    }

    async fn write(&self, values: &[WriteValue]) -> Result<Vec<StatusCode>, StatusCode> {
        self.calls.lock().writes.push(values.to_vec());
        Ok(vec![StatusCode::Good; values.len()])
    }

    async fn register_nodes(&self, nodes: &[NodeId]) -> Result<Vec<NodeId>, StatusCode> {
        Ok(nodes.to_vec())
    }

    async fn server_info(&self) -> Result<ServerInfo, StatusCode> {
        Ok(ServerInfo::default())
    }

    async fn type_tree(&self) -> Result<Arc<async_opcua::types::custom::DataTypeTree>, StatusCode> {
        Err(StatusCode::BadNotSupported)
    }

    async fn create_subscription(
        &self,
        _config: &SubscriptionConfig,
        _sink: mpsc::UnboundedSender<Notification>,
    ) -> Result<u32, StatusCode> {
        Ok(1)
    }

    async fn create_monitored_items(
        &self,
        _subscription_id: u32,
        items: Vec<async_opcua::types::MonitoredItemCreateRequest>,
    ) -> Result<Vec<StatusCode>, StatusCode> {
        Ok(vec![StatusCode::Good; items.len()])
    }

    async fn disconnect(&self) {}
}

// -------------------------------------------------------------------- test setup

/// A registry whose session never connects: what the record does is observable
/// on its own queue and in its own fields.
fn offline() -> Arc<Registry> {
    struct Offline;
    #[async_trait]
    impl UaConnector for Offline {
        async fn connect(&self, _config: &SessionConfig) -> Result<Arc<dyn UaConnection>, String> {
            Err("no server".to_string())
        }
    }
    let registry = Registry::new(Arc::new(Offline));
    registry
        .add_session(SessionConfig::new("S", "opc.tcp://server:4840"))
        .unwrap();
    registry
}

fn live(mock: &Arc<Mock>) -> Arc<Registry> {
    let registry = Registry::new(Arc::new(MockConnector(mock.clone())));
    registry
        .add_session(SessionConfig::new("S", "opc.tcp://server:4840"))
        .unwrap();
    registry
}

fn bind(registry: &Arc<Registry>, name: &str, record: &mut dyn Record, link: &str) -> OpcuaDevice {
    record.init_record(0).expect("the record initialises");
    let mut device = OpcuaDevice::new(registry.clone(), link.to_string());
    device.set_record_info(name, ScanType::IoIntr);
    device.init(record).expect("the link binds");
    device
}

/// The item record's own leaf — the one its `special()` and its device support
/// put a reason to process on.
fn leaf_of(record: &mut OpcuaItemRecord) -> Arc<Mutex<Leaf>> {
    record.binding().expect("the record is bound").leaf.clone()
}

/// One process pass of the item record, as the framework runs it: the device's
/// read stage (the item's action), then the record itself.
fn process(device: &mut OpcuaDevice, record: &mut OpcuaItemRecord) {
    let outcome = device.read(record).expect("the item acts");
    record.set_device_did_compute(outcome.did_compute);
    record.process().expect("the record processes");
}

/// A put from a client: the field, then the `special()` the framework calls.
fn put(record: &mut OpcuaItemRecord, field: &str, value: EpicsValue) {
    record.put_field(field, value).expect("the field takes it");
    record.special(field, true).expect("special runs");
}

fn up(leaf: &Arc<Mutex<Leaf>>) {
    leaf.lock().state = ConnectionStatus::Up;
}

fn queue(leaf: &Arc<Mutex<Leaf>>, reason: ProcessReason, status: StatusCode) {
    leaf.lock().queue.push(Update::new(
        reason,
        None,
        status,
        std::time::SystemTime::UNIX_EPOCH,
    ));
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

/// Wait for the session's initial read of the item to reach the record, and take
/// the update it queued — every test below is about what the record does *next*,
/// so the queue must start from the state a live IOC reaches right after
/// `iocInit`: connected, read once, nothing outstanding.
async fn settled(mock: &Arc<Mock>, leaf: &Arc<Mutex<Leaf>>) {
    until(|| !mock.calls.lock().reads.is_empty()).await;
    until(|| leaf.lock().queue.pop().is_some()).await;
}

// ------------------------------------------------------------------------- init

#[test]
fn init_shows_the_session_the_link_named() {
    let registry = offline();
    let mut record = OpcuaItemRecord::default();
    bind(&registry, "ITEM", &mut record, "S ns=2;s=Struct monitor=n");

    assert_eq!(record.sess, "S");
    assert_eq!(record.subs, "");
    assert!(record.binding().is_some());
}

#[test]
fn init_takes_the_links_bini_when_the_database_set_none() {
    let registry = offline();
    let mut record = OpcuaItemRecord::default();
    bind(
        &registry,
        "ITEM",
        &mut record,
        "S ns=2;s=Struct monitor=n bini=write",
    );

    // menuBini: read, ignore, write.
    assert_eq!(record.bini, 2);
}

#[test]
fn a_bini_the_database_set_is_not_overwritten_by_the_link() {
    let registry = offline();
    let mut record = OpcuaItemRecord::default();
    record.bini = 1; // ignore
    bind(
        &registry,
        "ITEM",
        &mut record,
        "S ns=2;s=Struct monitor=n bini=write",
    );

    assert_eq!(record.bini, 1);
}

// ---------------------------------------------------------------------- failures

#[test]
fn a_connection_loss_is_a_comm_alarm_and_leaves_the_record_undefined() {
    let registry = offline();
    let mut record = OpcuaItemRecord::default();
    let mut device = bind(&registry, "ITEM", &mut record, "S ns=2;s=Struct monitor=n");
    let leaf = leaf_of(&mut record);
    up(&leaf);
    queue(&leaf, ProcessReason::ConnectionLoss, StatusCode::Good);

    process(&mut device, &mut record);

    // COMM_ALARM, INVALID_ALARM.
    assert_eq!(device.last_alarm(), Some((9, 3)));
    assert!(record.value_is_undefined());
}

#[test]
fn a_read_failure_is_a_read_alarm() {
    let registry = offline();
    let mut record = OpcuaItemRecord::default();
    let mut device = bind(&registry, "ITEM", &mut record, "S ns=2;s=Struct monitor=n");
    let leaf = leaf_of(&mut record);
    up(&leaf);
    queue(&leaf, ProcessReason::ReadFailure, StatusCode::BadTimeout);

    process(&mut device, &mut record);

    // READ_ALARM, INVALID_ALARM.
    assert_eq!(device.last_alarm(), Some((1, 3)));
    assert!(record.value_is_undefined());
}

#[test]
fn a_write_failure_is_a_write_alarm() {
    let registry = offline();
    let mut record = OpcuaItemRecord::default();
    let mut device = bind(&registry, "ITEM", &mut record, "S ns=2;s=Struct monitor=n");
    let leaf = leaf_of(&mut record);
    up(&leaf);
    queue(&leaf, ProcessReason::WriteFailure, StatusCode::BadTimeout);

    process(&mut device, &mut record);

    // WRITE_ALARM, INVALID_ALARM.
    assert_eq!(device.last_alarm(), Some((2, 3)));
    assert!(record.value_is_undefined());
}

#[test]
fn a_session_that_is_down_is_a_comm_alarm() {
    let registry = offline();
    let mut record = OpcuaItemRecord::default();
    let mut device = bind(&registry, "ITEM", &mut record, "S ns=2;s=Struct monitor=n");

    // No reason, and the session never came up.
    process(&mut device, &mut record);

    assert_eq!(device.last_alarm(), Some((9, 3)));
}

// ------------------------------------------------------------------- the status

#[test]
fn the_nodes_status_lands_in_statcode_and_stattext() {
    let registry = offline();
    let mut record = OpcuaItemRecord::default();
    let mut device = bind(&registry, "ITEM", &mut record, "S ns=2;s=Struct monitor=n");
    let leaf = leaf_of(&mut record);
    up(&leaf);
    {
        let binding = record.binding().expect("bound").clone();
        let mut item = binding.item.lock();
        item.last_status = StatusCode::BadNodeIdUnknown;
    }
    queue(&leaf, ProcessReason::ReadComplete, StatusCode::Good);

    process(&mut device, &mut record);

    assert_eq!(record.statcode, StatusCode::BadNodeIdUnknown.bits());
    assert_eq!(record.stattext, "BadNodeIdUnknown");
    // The status is not a failure of the *action*: the C clears UDF for every
    // reason but the three failures.
    assert!(!record.value_is_undefined());
    // OSTATCODE latches the code the record has now shown, so the next process
    // that finds the same code posts nothing.
    assert_eq!(record.ostatcode, record.statcode);
}

// ------------------------------------------------------------- the binding order

#[tokio::test]
async fn an_element_record_may_bind_before_the_item_record_it_belongs_to() {
    // The framework wires device support in `HashMap` order, so this is the order
    // half the runs will take. The item exists from whichever record binds first;
    // the item record's binding is the one that gives it its node and its session.
    let mock = Arc::new(Mock::default());
    let registry = live(&mock);

    let mut element = LongoutRecord::new(0);
    let mut element_device = bind(&registry, "EL", &mut element, "ITEM");

    let mut item = OpcuaItemRecord::default();
    let mut item_device = bind(&registry, "ITEM", &mut item, "S ns=2;s=Struct monitor=n");
    let leaf = leaf_of(&mut item);
    registry.start();
    settled(&mock, &leaf).await;

    // The element is on the item the item record adopted: its value goes out in
    // the item's write, and the write reaches the server.
    element.put_field("VAL", EpicsValue::Long(5)).unwrap();
    element.process().unwrap();
    element_device
        .write(&mut element)
        .expect("the value is set");
    put(&mut item, "WRITE", EpicsValue::Char(1));
    process(&mut item_device, &mut item);

    until(|| !mock.calls.lock().writes.is_empty()).await;
    let writes = mock.calls.lock().writes.clone();
    assert_eq!(writes[0][0].value.value, Some(Variant::Int32(5)));
}

#[tokio::test]
async fn an_element_record_whose_item_record_is_not_in_the_database_is_reported() {
    // The C refuses the link at load; this port cannot tell "not bound yet" from
    // "not in the database" until every record has bound, so `start` is where it
    // is found. The element must not take the IOC down, and must not be adopted.
    let registry = offline();
    let mut element = LongoutRecord::new(0);
    let device = bind(&registry, "EL", &mut element, "MISSING");

    registry.start();

    let binding = device.binding().expect("the element bound to an item");
    assert!(
        !binding.item.lock().is_adopted(),
        "no item record ever gave the item a node or a session"
    );
}

// -------------------------------------------------------------------- the actions

#[tokio::test]
async fn a_put_to_read_reads_the_node() {
    let mock = Arc::new(Mock::default());
    let registry = live(&mock);
    let mut record = OpcuaItemRecord::default();
    let mut device = bind(&registry, "ITEM", &mut record, "S ns=2;s=Struct monitor=n");
    let leaf = leaf_of(&mut record);
    registry.start();
    settled(&mock, &leaf).await;
    let reads = mock.calls.lock().reads.len();

    put(&mut record, "READ", EpicsValue::Char(1));
    process(&mut device, &mut record);

    until(|| mock.calls.lock().reads.len() > reads).await;
}

#[tokio::test]
async fn a_put_to_write_sends_what_the_elements_wrote() {
    let mock = Arc::new(Mock::default());
    let registry = live(&mock);
    let mut item = OpcuaItemRecord::default();
    let mut item_device = bind(&registry, "ITEM", &mut item, "S ns=2;s=Struct monitor=n");
    // An element of the item: its value goes out inside the item's write, and it
    // asks for no write of its own.
    let mut element = LongoutRecord::new(0);
    let mut element_device = bind(&registry, "EL", &mut element, "ITEM");
    let leaf = leaf_of(&mut item);
    registry.start();
    settled(&mock, &leaf).await;

    element.put_field("VAL", EpicsValue::Long(7)).unwrap();
    element.process().unwrap();
    element_device
        .write(&mut element)
        .expect("the value is set");
    assert!(mock.calls.lock().writes.is_empty(), "the element waits");

    put(&mut item, "WRITE", EpicsValue::Char(1));
    process(&mut item_device, &mut item);

    until(|| !mock.calls.lock().writes.is_empty()).await;
    let writes = mock.calls.lock().writes.clone();
    assert_eq!(writes[0].len(), 1);
    assert_eq!(
        writes[0][0].value.value,
        Some(Variant::Int32(7)),
        "the element's value, in the type the node last delivered"
    );
}

#[tokio::test]
async fn defactn_write_makes_a_bare_process_a_write() {
    let mock = Arc::new(Mock::default());
    let registry = live(&mock);
    let mut item = OpcuaItemRecord::default();
    // menuDefAction: read, write.
    item.defactn = 1;
    let mut item_device = bind(&registry, "ITEM", &mut item, "S ns=2;s=Struct monitor=n");
    let mut element = LongoutRecord::new(0);
    let mut element_device = bind(&registry, "EL", &mut element, "ITEM");
    let leaf = leaf_of(&mut item);
    registry.start();
    settled(&mock, &leaf).await;

    element.put_field("VAL", EpicsValue::Long(3)).unwrap();
    element.process().unwrap();
    element_device
        .write(&mut element)
        .expect("the value is set");

    // A put to VAL — no reason of its own, so DEFACTN decides.
    put(&mut item, "VAL", EpicsValue::ULong(0));
    process(&mut item_device, &mut item);

    until(|| !mock.calls.lock().writes.is_empty()).await;
}

#[tokio::test]
async fn woc_immediate_sends_what_the_elements_have_already_written() {
    let mock = Arc::new(Mock::default());
    let registry = live(&mock);
    let mut item = OpcuaItemRecord::default();
    let mut item_device = bind(&registry, "ITEM", &mut item, "S ns=2;s=Struct monitor=n");
    let mut element = LongoutRecord::new(0);
    let mut element_device = bind(&registry, "EL", &mut element, "ITEM");
    let leaf = leaf_of(&mut item);
    registry.start();
    settled(&mock, &leaf).await;

    element.put_field("VAL", EpicsValue::Long(9)).unwrap();
    element.process().unwrap();
    element_device
        .write(&mut element)
        .expect("the value is set");
    assert!(mock.calls.lock().writes.is_empty(), "WOC is still manual");

    // Switching WOC to immediate sends the dirty element there and then — the
    // item asks its own record to process for a write (`requestWriteIfDirty`),
    // and that request is the update the record's next process pass pops.
    put(&mut item, "WOC", EpicsValue::Enum(1));
    process(&mut item_device, &mut item);

    until(|| !mock.calls.lock().writes.is_empty()).await;
}

#[tokio::test]
async fn woc_immediate_with_nothing_dirty_sends_nothing() {
    let mock = Arc::new(Mock::default());
    let registry = live(&mock);
    let mut item = OpcuaItemRecord::default();
    let mut item_device = bind(&registry, "ITEM", &mut item, "S ns=2;s=Struct monitor=n");
    let mut element = LongoutRecord::new(0);
    bind(&registry, "EL", &mut element, "ITEM");
    let leaf = leaf_of(&mut item);
    registry.start();
    settled(&mock, &leaf).await;

    put(&mut item, "WOC", EpicsValue::Enum(1));
    assert!(
        leaf.lock().queue.pop().is_none(),
        "no element has written, so the item asks for no write"
    );

    // The bare process that follows is the default action, a read — not a write.
    process(&mut item_device, &mut item);
    assert!(mock.calls.lock().writes.is_empty());
}
