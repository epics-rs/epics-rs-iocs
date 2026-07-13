//! Session worker tests, driven against a test double of the client boundary
//! (`UaConnector`/`UaConnection`) — no live OPC UA server is involved.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use async_opcua::types::{
    AttributeId, DataChangeFilter, DataValue, DeadbandType, NodeId, ReadValueId, StatusCode,
    Variant, WriteValue,
};
use async_trait::async_trait;
use parking_lot::Mutex;
use tokio::sync::mpsc;

use opcua::client::{Notification, ServerInfo, UaConnection, UaConnector};
use opcua::item::{Item, Leaf};
use opcua::link::{InfoDefaults, LinkInfo, NodeIdentifier, RecordKind, parse_link};
use opcua::queue::{ConnectionStatus, ProcessReason, Update};
use opcua::session::{self, Control, Priority, Request, SessionConfig, SessionHandle};
use opcua::subscription::SubscriptionConfig;

// -------------------------------------------------------------- the test double

#[derive(Clone, Default)]
struct Behaviour {
    /// The whole service call fails.
    read_error: Option<StatusCode>,
    write_error: Option<StatusCode>,
    /// The per-value status the write results carry.
    write_status: Option<StatusCode>,
    /// Return one result short of what was asked for — a server that under-answers.
    truncate_read: bool,
}

#[derive(Default)]
struct Calls {
    reads: Vec<Vec<ReadValueId>>,
    writes: Vec<Vec<WriteValue>>,
    registered: Vec<NodeId>,
    subscriptions: Vec<SubscriptionConfig>,
    monitored: Vec<async_opcua::types::MonitoredItemCreateRequest>,
    disconnects: usize,
}

#[derive(Default)]
struct Mock {
    server: ServerInfo,
    behaviour: Mutex<Behaviour>,
    calls: Mutex<Calls>,
    /// Value attribute per node, defaulting to Int32(42).
    values: Mutex<HashMap<NodeId, Variant>>,
    sink: Mutex<Option<mpsc::UnboundedSender<Notification>>>,
    connects: Mutex<Vec<SessionConfig>>,
}

impl Mock {
    fn arc() -> Arc<Mock> {
        Arc::new(Mock::default())
    }

    fn behave(&self, f: impl FnOnce(&mut Behaviour)) {
        f(&mut self.behaviour.lock());
    }
}

/// The connector hands out the same connection every time, so a test can look at
/// what the worker did without holding a second handle.
struct MockConnector(Arc<Mock>);

#[async_trait]
impl UaConnector for MockConnector {
    async fn connect(&self, config: &SessionConfig) -> Result<Arc<dyn UaConnection>, String> {
        self.0.connects.lock().push(config.clone());
        Ok(self.0.clone())
    }
}

#[async_trait]
impl UaConnection for Mock {
    async fn read(&self, nodes: &[ReadValueId]) -> Result<Vec<DataValue>, StatusCode> {
        self.calls.lock().reads.push(nodes.to_vec());
        if let Some(status) = self.behaviour.lock().read_error {
            return Err(status);
        }
        let values = self.values.lock();
        let mut results: Vec<DataValue> = nodes
            .iter()
            .map(|node| {
                if node.attribute_id == AttributeId::DataType as u32 {
                    // Int32 — i=6.
                    DataValue::value_only(Variant::NodeId(Box::new(NodeId::new(0, 6u32))))
                } else {
                    let value = values
                        .get(&node.node_id)
                        .cloned()
                        .unwrap_or(Variant::Int32(42));
                    DataValue {
                        value: Some(value),
                        status: Some(StatusCode::Good),
                        ..Default::default()
                    }
                }
            })
            .collect();
        if self.behaviour.lock().truncate_read {
            results.pop();
        }
        Ok(results)
    }

    async fn write(&self, values: &[WriteValue]) -> Result<Vec<StatusCode>, StatusCode> {
        self.calls.lock().writes.push(values.to_vec());
        let behaviour = self.behaviour.lock().clone();
        if let Some(status) = behaviour.write_error {
            return Err(status);
        }
        let status = behaviour.write_status.unwrap_or(StatusCode::Good);
        Ok(vec![status; values.len()])
    }

    async fn register_nodes(&self, nodes: &[NodeId]) -> Result<Vec<NodeId>, StatusCode> {
        self.calls.lock().registered.extend(nodes.iter().cloned());
        Ok(nodes
            .iter()
            .map(|node| NodeId::new(0, format!("registered:{node}")))
            .collect())
    }

    async fn server_info(&self) -> Result<ServerInfo, StatusCode> {
        Ok(self.server.clone())
    }

    async fn type_tree(&self) -> Result<Arc<async_opcua::types::custom::DataTypeTree>, StatusCode> {
        // The type dictionary is a server document; a test double has none. The
        // worker must go on without it (enumerations and structures then do not
        // resolve, which is what it logs).
        Err(StatusCode::BadNotSupported)
    }

    async fn create_subscription(
        &self,
        config: &SubscriptionConfig,
        sink: mpsc::UnboundedSender<Notification>,
    ) -> Result<u32, StatusCode> {
        self.calls.lock().subscriptions.push(config.clone());
        *self.sink.lock() = Some(sink);
        Ok(7)
    }

    async fn create_monitored_items(
        &self,
        _subscription_id: u32,
        items: Vec<async_opcua::types::MonitoredItemCreateRequest>,
    ) -> Result<Vec<StatusCode>, StatusCode> {
        let n = items.len();
        self.calls.lock().monitored.extend(items);
        Ok(vec![StatusCode::Good; n])
    }

    async fn disconnect(&self) {
        self.calls.lock().disconnects += 1;
    }
}

// ------------------------------------------------------------------- test setup

struct Names;

impl opcua::link::NameResolver for Names {
    fn subscription_session(&self, name: &str) -> Option<String> {
        (name == "subs").then(|| "sess".to_string())
    }
    fn is_session(&self, name: &str) -> bool {
        name == "sess"
    }
}

const INPUT: RecordKind = RecordKind {
    is_output: false,
    is_item_record: false,
};
const OUTPUT: RecordKind = RecordKind {
    is_output: true,
    is_item_record: false,
};

fn link(text: &str, kind: RecordKind) -> LinkInfo {
    parse_link(text, kind, &InfoDefaults::default(), &Names)
        .unwrap_or_else(|e| panic!("parsing '{text}' failed: {e}"))
}

fn node_id_of(link: &LinkInfo) -> NodeId {
    match link.identifier.clone().expect("the link addresses a node") {
        NodeIdentifier::Numeric(n) => NodeId::new(link.namespace_index, n),
        NodeIdentifier::String(s) => NodeId::new(link.namespace_index, s),
    }
}

/// One item with one record bound to the whole node.
fn item(text: &str, kind: RecordKind, handle: u32) -> (Arc<Mutex<Item>>, Arc<Mutex<Leaf>>) {
    let info = link(text, kind);
    let mut item = Item::new(info.clone(), node_id_of(&info), handle);
    // A capacity of one is all a test needs; the record end of the channel is
    // dropped, so a pulse that finds no room is simply dropped.
    let (notify, _rx) = mpsc::channel(1);
    let leaf = Arc::new(Mutex::new(Leaf::new("rec".to_string(), info, notify)));
    item.leaves.push(leaf.clone());
    (Arc::new(Mutex::new(item)), leaf)
}

fn spawn(
    config: SessionConfig,
    mock: &Arc<Mock>,
    items: Vec<Arc<Mutex<Item>>>,
) -> Arc<SessionHandle> {
    let (handle, worker) = session::create(config, Arc::new(MockConnector(mock.clone())));
    *handle.items.lock() = items;
    tokio::spawn(worker.run());
    handle
}

/// Poll until `f` holds. Every wait in these tests is for the worker task to get
/// around to something, which is microseconds away.
async fn until(f: impl Fn() -> bool) {
    for _ in 0..2000 {
        if f() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    panic!("the condition did not hold within two seconds");
}

fn pop(leaf: &Arc<Mutex<Leaf>>) -> Update {
    leaf.lock()
        .queue
        .pop()
        .map(|(update, _)| update)
        .expect("the leaf has an update")
}

fn drain(leaf: &Arc<Mutex<Leaf>>) -> Vec<Update> {
    let mut leaf = leaf.lock();
    let mut updates = Vec::new();
    while let Some((update, _)) = leaf.queue.pop() {
        updates.push(update);
    }
    updates
}

// -------------------------------------------------------------- batch size rules

#[test]
fn a_batch_is_bounded_by_the_smaller_of_the_two_limits() {
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.nodes_max = 10;
    let server = ServerInfo {
        max_nodes_per_read: 4,
        max_nodes_per_write: 20,
        ..Default::default()
    };
    assert_eq!(config.read_batch_size(&server), 4);
    assert_eq!(config.write_batch_size(&server), 10);
}

#[test]
fn a_limit_only_one_side_sets_is_the_one_that_holds() {
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.write_nodes_max = 3;
    let server = ServerInfo {
        max_nodes_per_read: 5,
        max_nodes_per_write: 0,
        ..Default::default()
    };
    assert_eq!(config.read_batch_size(&server), 5);
    assert_eq!(config.write_batch_size(&server), 3);
}

#[test]
fn the_service_specific_limit_beats_the_general_one() {
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.nodes_max = 8;
    config.read_nodes_max = 2;
    assert_eq!(config.read_batch_size(&ServerInfo::default()), 2);
    assert_eq!(config.write_batch_size(&ServerInfo::default()), 8);
}

#[test]
fn no_limit_anywhere_means_no_limit() {
    let config = SessionConfig::new("sess", "opc.tcp://host");
    assert_eq!(config.read_batch_size(&ServerInfo::default()), usize::MAX);
}

// ----------------------------------------------------------------- session options

#[test]
fn session_options_are_parsed() {
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.set_option("debug", "2").unwrap();
    config.set_option("autoconnect", "no").unwrap();
    config.set_option("nodes-max", "100").unwrap();
    config.set_option("read-timeout-min", "5.5").unwrap();
    config.set_option("sec-mode", "SignAndEncrypt").unwrap();
    config.set_option("sec-id", "user:secret").unwrap();
    config.set_option("pki-dir", "/etc/pki").unwrap();

    assert_eq!(config.debug, 2);
    assert!(!config.autoconnect);
    assert_eq!(config.nodes_max, 100);
    assert_eq!(config.read_timeout_min, 5.5);
    assert_eq!(
        config.security_mode,
        opcua::client::SecurityMode::SignAndEncrypt
    );
    assert_eq!(
        config.identity,
        opcua::client::Identity::UserName {
            user: "user".into(),
            password: "secret".into()
        }
    );
    assert_eq!(config.pki_dir, "/etc/pki");
}

#[test]
fn an_unknown_session_option_is_rejected() {
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    assert!(config.set_option("nonsense", "1").is_err());
    assert!(config.set_option("nodes-max", "x").is_err());
}

#[test]
fn an_x509_identity_names_a_certificate_and_a_key() {
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config
        .set_option("sec-id", "cert:/tmp/id.der:/tmp/id.pem")
        .unwrap();
    assert_eq!(
        config.identity,
        opcua::client::Identity::Certificate {
            certificate: "/tmp/id.der".into(),
            private_key: "/tmp/id.pem".into()
        }
    );
    assert!(config.set_option("sec-id", "cert:only-one").is_err());
}

#[test]
fn subscription_options_are_parsed() {
    let mut config = SubscriptionConfig::new("subs", "sess");
    config.set_option("priority", "5").unwrap();
    config.set_option("debug", "1").unwrap();
    assert_eq!(config.priority, 5);
    assert_eq!(config.debug, 1);
    assert!(config.set_option("nonsense", "1").is_err());
}

// -------------------------------------------------------------------- connecting

#[tokio::test]
async fn connecting_reads_every_item_once() {
    let mock = Mock::arc();
    mock.values
        .lock()
        .insert(NodeId::new(2, "Sensor"), Variant::Int32(11));
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT, 0);
    let (item1, leaf1) = item("sess ns=2;s=Other monitor=n", INPUT, 1);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0, item1],
    );
    until(|| handle.is_connected()).await;
    until(|| !mock.calls.lock().reads.is_empty()).await;

    let update = pop(&leaf0);
    assert_eq!(update.reason, ProcessReason::ReadComplete);
    assert_eq!(update.data, Some(Variant::Int32(11)));
    assert_eq!(pop(&leaf1).data, Some(Variant::Int32(42)));

    // Both attributes of both items, in one service call.
    let reads = mock.calls.lock().reads.clone();
    assert_eq!(reads.len(), 1);
    assert_eq!(reads[0].len(), 4);
    assert_eq!(reads[0][0].attribute_id, AttributeId::DataType as u32);
    assert_eq!(reads[0][1].attribute_id, AttributeId::Value as u32);
}

#[tokio::test]
async fn the_data_type_read_alongside_the_value_lands_on_the_item() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT, 0);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0.clone()],
    );
    until(|| handle.is_connected()).await;
    until(|| item0.lock().data_type.is_some()).await;
    assert_eq!(item0.lock().data_type, Some(NodeId::new(0, 6u32)));
}

#[tokio::test]
async fn a_session_that_does_not_autoconnect_waits_for_the_connect_command() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT, 0);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;

    let handle = spawn(config, &mock, vec![item0]);
    tokio::time::sleep(Duration::from_millis(20)).await;
    assert!(!handle.is_connected());
    assert!(mock.connects.lock().is_empty());

    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    assert_eq!(mock.connects.lock().len(), 1);
}

#[tokio::test]
async fn bini_write_sends_the_records_value_after_the_initial_read() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor bini=write monitor=n", OUTPUT, 0);
    {
        let mut leaf = leaf0.lock();
        leaf.outgoing = Some(Variant::Int32(7));
        leaf.dirty = true;
    }

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    until(|| !mock.calls.lock().writes.is_empty()).await;

    let writes = mock.calls.lock().writes.clone();
    assert_eq!(writes.len(), 1);
    assert_eq!(writes[0][0].node_id, NodeId::new(2, "Sensor"));
    assert_eq!(writes[0][0].value.value, Some(Variant::Int32(7)));

    // The read result, then the write result.
    let updates = drain(&leaf0);
    assert_eq!(updates[0].reason, ProcessReason::ReadComplete);
    assert_eq!(updates[1].reason, ProcessReason::WriteComplete);
}

#[tokio::test]
async fn bini_ignore_drops_the_initial_read_but_still_learns_the_type() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor bini=ignore monitor=n", OUTPUT, 0);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    until(|| !mock.calls.lock().reads.is_empty()).await;

    assert!(drain(&leaf0).is_empty());
    assert_eq!(leaf0.lock().incoming, Some(Variant::Int32(42)));
}

#[tokio::test]
async fn a_registered_node_is_the_one_the_service_calls_use() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor register=y monitor=n", INPUT, 0);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0.clone()],
    );
    until(|| handle.is_connected()).await;

    assert_eq!(mock.calls.lock().registered, vec![NodeId::new(2, "Sensor")]);
    let expected = NodeId::new(0, "registered:ns=2;s=Sensor");
    assert_eq!(item0.lock().wire_node_id(), &expected);
    assert_eq!(mock.calls.lock().reads[0][0].node_id, expected);
}

#[tokio::test]
async fn the_namespace_map_moves_an_item_onto_the_servers_index() {
    let mut mock = Mock::default();
    mock.server.namespace_array = vec![
        "http://opcfoundation.org/UA/".to_string(),
        "urn:server".to_string(),
        "urn:sensors".to_string(),
    ];
    let mock = Arc::new(mock);

    let (item0, _leaf0) = item("sess ns=4;s=Sensor monitor=n", INPUT, 0);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.namespace_map.insert(4, "urn:sensors".to_string());

    let handle = spawn(config, &mock, vec![item0.clone()]);
    until(|| handle.is_connected()).await;

    assert_eq!(item0.lock().node_id, NodeId::new(2, "Sensor"));
    assert_eq!(item0.lock().configured_node_id, NodeId::new(4, "Sensor"));
}

#[tokio::test]
async fn an_item_whose_namespace_the_server_does_not_have_keeps_its_index() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=4;s=Sensor monitor=n", INPUT, 0);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.namespace_map.insert(4, "urn:absent".to_string());

    let handle = spawn(config, &mock, vec![item0.clone()]);
    until(|| handle.is_connected()).await;
    assert_eq!(item0.lock().node_id, NodeId::new(4, "Sensor"));
}

// --------------------------------------------------------------------- requests

#[tokio::test]
async fn a_records_read_request_reaches_the_server() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT, 0);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    drain(&leaf0);

    mock.values
        .lock()
        .insert(NodeId::new(2, "Sensor"), Variant::Int32(99));
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| !leaf0.lock().queue.is_empty()).await;

    let update = pop(&leaf0);
    assert_eq!(update.reason, ProcessReason::ReadComplete);
    assert_eq!(update.data, Some(Variant::Int32(99)));
}

#[tokio::test]
async fn a_records_write_request_carries_its_value() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", OUTPUT, 0);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    drain(&leaf0);

    {
        let mut leaf = leaf0.lock();
        leaf.outgoing = Some(Variant::Double(1.5));
        leaf.dirty = true;
    }
    handle.request(Priority::Low, Request::Write { handle: 0 });
    until(|| !mock.calls.lock().writes.is_empty()).await;

    let writes = mock.calls.lock().writes.clone();
    assert_eq!(writes[0][0].value.value, Some(Variant::Double(1.5)));
    assert_eq!(pop(&leaf0).reason, ProcessReason::WriteComplete);
    assert!(!leaf0.lock().dirty);
}

#[tokio::test]
async fn a_failed_write_reaches_the_record_as_a_write_failure() {
    let mock = Mock::arc();
    mock.behave(|b| b.write_status = Some(StatusCode::BadTypeMismatch));
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", OUTPUT, 0);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    drain(&leaf0);

    {
        let mut leaf = leaf0.lock();
        leaf.outgoing = Some(Variant::Int32(1));
        leaf.dirty = true;
    }
    handle.request(Priority::Low, Request::Write { handle: 0 });
    until(|| !leaf0.lock().queue.is_empty()).await;

    let update = pop(&leaf0);
    assert_eq!(update.reason, ProcessReason::WriteFailure);
    assert_eq!(update.status, StatusCode::BadTypeMismatch);
    // The session stays up: a bad value is not a bad connection.
    assert!(handle.is_connected());
}

#[tokio::test]
async fn a_failed_read_reaches_the_record_as_a_read_failure() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT, 0);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    drain(&leaf0);

    mock.behave(|b| b.read_error = Some(StatusCode::BadNodeIdUnknown));
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| !leaf0.lock().queue.is_empty()).await;

    let update = pop(&leaf0);
    assert_eq!(update.reason, ProcessReason::ReadFailure);
    assert_eq!(update.status, StatusCode::BadNodeIdUnknown);
    assert!(handle.is_connected());
}

/// Upstream C defect fixed at source: `SessionOpen62541::readComplete` bounds-checks
/// the DataType result and then reads the Value result at `i + 1` unchecked, so a
/// server that returns fewer results than requested reads past the end of the array.
#[tokio::test]
async fn a_server_that_returns_too_few_results_does_not_read_past_the_end() {
    let mock = Mock::arc();
    mock.behave(|b| b.truncate_read = true);
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT, 0);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    until(|| !leaf0.lock().queue.is_empty()).await;

    let update = pop(&leaf0);
    assert_eq!(update.reason, ProcessReason::ReadFailure);
    assert_eq!(update.status, StatusCode::BadUnexpectedError);
}

#[tokio::test]
async fn a_batch_is_split_at_the_servers_limit() {
    let mut mock = Mock::default();
    // Two nodes per item, so one item per read.
    mock.server.max_nodes_per_read = 2;
    let mock = Arc::new(mock);

    let (item0, _l0) = item("sess ns=2;s=A monitor=n", INPUT, 0);
    let (item1, _l1) = item("sess ns=2;s=B monitor=n", INPUT, 1);
    let (item2, _l2) = item("sess ns=2;s=C monitor=n", INPUT, 2);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0, item1, item2],
    );
    until(|| handle.is_connected()).await;

    // The initial read of three items is one call per item; every call asks for
    // the two attributes of one node.
    let reads = mock.calls.lock().reads.clone();
    assert!(
        reads.len() >= 3,
        "expected one call per item, got {reads:?}"
    );
    for call in &reads {
        assert_eq!(call.len(), 2);
    }
}

#[tokio::test]
async fn a_high_priority_request_goes_out_before_a_low_priority_one() {
    let mut mock = Mock::default();
    mock.server.max_nodes_per_read = 2;
    let mock = Arc::new(mock);

    let (item0, _l0) = item("sess ns=2;s=Low monitor=n", INPUT, 0);
    let (item1, _l1) = item("sess ns=2;s=High monitor=n", INPUT, 1);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    // Hold the batch back long enough for both requests to be queued.
    config.read_timeout_min = 30.0;

    let handle = spawn(config, &mock, vec![item0, item1]);
    until(|| handle.is_connected()).await;
    mock.calls.lock().reads.clear();

    handle.request(Priority::Low, Request::Read { handle: 0 });
    handle.request(Priority::High, Request::Read { handle: 1 });
    until(|| mock.calls.lock().reads.len() >= 2).await;

    let reads = mock.calls.lock().reads.clone();
    assert_eq!(reads[0][0].node_id, NodeId::new(2, "High"));
    assert_eq!(reads[1][0].node_id, NodeId::new(2, "Low"));
}

// ------------------------------------------------------------------ connection loss

#[tokio::test]
async fn a_connection_error_takes_the_session_down_and_tells_every_record() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT, 0);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;

    let handle = spawn(config, &mock, vec![item0.clone()]);
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    drain(&leaf0);

    mock.behave(|b| b.read_error = Some(StatusCode::BadConnectionClosed));
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| !handle.is_connected()).await;

    let updates = drain(&leaf0);
    assert_eq!(updates[0].reason, ProcessReason::ReadFailure);
    assert_eq!(updates[1].reason, ProcessReason::ConnectionLoss);
    assert_eq!(updates[1].status, StatusCode::BadConnectionClosed);
    assert_eq!(item0.lock().state, ConnectionStatus::Down);
    assert_eq!(mock.calls.lock().disconnects, 1);
}

#[tokio::test]
async fn requests_queued_for_a_lost_connection_are_dropped() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT, 0);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;
    // A hold-off long enough that the queued read is still waiting when the
    // disconnect arrives.
    config.read_timeout_min = 200.0;

    let handle = spawn(config, &mock, vec![item0]);
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    mock.calls.lock().reads.clear();

    handle.request(Priority::Low, Request::Read { handle: 0 });
    handle.control(Control::Disconnect);
    until(|| !handle.is_connected()).await;
    tokio::time::sleep(Duration::from_millis(250)).await;

    // The dropped request must not be answered against a later connection.
    assert!(mock.calls.lock().reads.is_empty());
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    assert_eq!(mock.calls.lock().reads.len(), 1, "only the initial read");
}

#[tokio::test]
async fn a_disconnected_session_reconnects_when_it_autoconnects() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT, 0);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;

    mock.behave(|b| b.read_error = Some(StatusCode::BadServerNotConnected));
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| mock.calls.lock().disconnects == 1).await;
    mock.behave(|b| b.read_error = None);

    until(|| mock.connects.lock().len() >= 2).await;
    until(|| handle.is_connected()).await;
}

// ----------------------------------------------------------------- subscriptions

#[tokio::test]
async fn a_monitored_item_carries_the_links_monitoring_parameters() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item(
        "subs ns=2;s=Sensor sampling=250 qsize=8 discard=new deadband=0.5",
        INPUT,
        0,
    );
    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    handle
        .subscriptions
        .lock()
        .push(SubscriptionConfig::new("subs", "sess"));
    until(|| !mock.calls.lock().monitored.is_empty()).await;

    let monitored = mock.calls.lock().monitored.clone();
    assert_eq!(monitored.len(), 1);
    let request = &monitored[0];
    assert_eq!(request.item_to_monitor.node_id, NodeId::new(2, "Sensor"));
    assert_eq!(
        request.item_to_monitor.attribute_id,
        AttributeId::Value as u32
    );
    assert_eq!(request.requested_parameters.client_handle, 0);
    assert_eq!(request.requested_parameters.sampling_interval, 250.0);
    assert_eq!(request.requested_parameters.queue_size, 8);
    assert!(!request.requested_parameters.discard_oldest);

    let filter: DataChangeFilter = request
        .requested_parameters
        .filter
        .inner_as::<DataChangeFilter>()
        .expect("a deadband makes a DataChangeFilter")
        .clone();
    assert_eq!(filter.deadband_type, DeadbandType::Absolute as u32);
    assert_eq!(filter.deadband_value, 0.5);
}

#[tokio::test]
async fn an_unmonitored_item_gets_no_monitored_item() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("subs ns=2;s=Sensor monitor=n", INPUT, 0);
    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    handle
        .subscriptions
        .lock()
        .push(SubscriptionConfig::new("subs", "sess"));
    until(|| !mock.calls.lock().subscriptions.is_empty()).await;
    tokio::time::sleep(Duration::from_millis(20)).await;
    assert!(mock.calls.lock().monitored.is_empty());
}

#[tokio::test]
async fn a_value_the_server_pushes_reaches_the_item_its_handle_names() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("subs ns=2;s=A", INPUT, 0);
    let (item1, leaf1) = item("subs ns=2;s=B", INPUT, 1);
    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0, item1],
    );
    handle
        .subscriptions
        .lock()
        .push(SubscriptionConfig::new("subs", "sess"));
    until(|| mock.sink.lock().is_some()).await;
    drain(&leaf0);
    drain(&leaf1);

    let sink = mock.sink.lock().clone().expect("the subscription is up");
    sink.send(Notification {
        client_handle: 1,
        value: DataValue::value_only(Variant::Int32(64)),
    })
    .unwrap();
    until(|| !leaf1.lock().queue.is_empty()).await;

    let update = pop(&leaf1);
    assert_eq!(update.reason, ProcessReason::IncomingData);
    assert_eq!(update.data, Some(Variant::Int32(64)));
    assert!(leaf0.lock().queue.is_empty());
}
