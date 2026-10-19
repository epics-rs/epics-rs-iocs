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

use epics_rs::base::runtime::log::{errlog_add_listener, errlog_flush, errlog_remove_listener};
use opcua::client::{
    ConnectError, CreatedSubscription, Notification, ServerInfo, UaConnection, UaConnector,
};
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
    /// Every Value read answers with this status and no value.
    value_status: Option<StatusCode>,
    /// Return one result short of what was asked for — a server that under-answers.
    truncate_read: bool,
    /// The per-item status `CreateMonitoredItems` answers with.
    monitor_status: Option<StatusCode>,
    /// `CreateSubscription` fails with this.
    subscription_error: Option<StatusCode>,
    /// The publishing interval `CreateSubscription` revises to; the requested
    /// one when unset.
    revised_interval: Option<f64>,
}

#[derive(Default)]
struct Calls {
    reads: Vec<Vec<ReadValueId>>,
    writes: Vec<Vec<WriteValue>>,
    registered: Vec<NodeId>,
    subscriptions: Vec<SubscriptionConfig>,
    monitored: Vec<async_opcua::types::MonitoredItemCreateRequest>,
    disconnects: usize,
    /// The read and subscription service calls, in the order they were made.
    events: Vec<String>,
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
    /// Refuse this many connect attempts before accepting one.
    refuse_connects: Mutex<usize>,
    /// The server drops the connection.
    lost: tokio::sync::Notify,
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
    async fn connect(
        &self,
        config: &SessionConfig,
        _identity: &opcua::client::Identity,
    ) -> Result<Arc<dyn UaConnection>, ConnectError> {
        self.0.connects.lock().push(config.clone());
        let mut refuse = self.0.refuse_connects.lock();
        if *refuse > 0 {
            *refuse -= 1;
            return Err(ConnectError {
                status: Some(StatusCode::BadNotConnected),
                message: "connect service failed with status BadNotConnected".into(),
            });
        }
        Ok(self.0.clone())
    }
}

#[async_trait]
impl UaConnection for Mock {
    async fn read(&self, nodes: &[ReadValueId]) -> (u32, Result<Vec<DataValue>, StatusCode>) {
        let id = {
            let mut calls = self.calls.lock();
            calls.reads.push(nodes.to_vec());
            calls.events.push("read".to_string());
            calls.reads.len() as u32
        };
        if let Some(status) = self.behaviour.lock().read_error {
            return (id, Err(status));
        }
        let values = self.values.lock();
        let value_status = self.behaviour.lock().value_status;
        let mut results: Vec<DataValue> = nodes
            .iter()
            .map(|node| {
                if node.attribute_id == AttributeId::DataType as u32 {
                    // Int32 — i=6.
                    DataValue::value_only(Variant::NodeId(Box::new(NodeId::new(0, 6u32))))
                } else if let Some(status) = value_status {
                    DataValue {
                        status: Some(status),
                        ..Default::default()
                    }
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
        (id, Ok(results))
    }

    async fn write(&self, values: &[WriteValue]) -> (u32, Result<Vec<StatusCode>, StatusCode>) {
        let id = {
            let mut calls = self.calls.lock();
            calls.writes.push(values.to_vec());
            calls.writes.len() as u32
        };
        let behaviour = self.behaviour.lock().clone();
        if let Some(status) = behaviour.write_error {
            return (id, Err(status));
        }
        let status = behaviour.write_status.unwrap_or(StatusCode::Good);
        (id, Ok(vec![status; values.len()]))
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
    ) -> Result<CreatedSubscription, StatusCode> {
        let id = {
            let mut calls = self.calls.lock();
            calls.subscriptions.push(config.clone());
            calls.events.push(format!("create {}", config.name));
            calls.subscriptions.len() as u32
        };
        if let Some(status) = self.behaviour.lock().subscription_error {
            return Err(status);
        }
        *self.sink.lock() = Some(sink);
        Ok(CreatedSubscription {
            id,
            revised_publishing_interval: self
                .behaviour
                .lock()
                .revised_interval
                .unwrap_or(config.publishing_interval),
        })
    }

    async fn create_monitored_items(
        &self,
        subscription_id: u32,
        items: Vec<async_opcua::types::MonitoredItemCreateRequest>,
    ) -> Result<Vec<async_opcua::types::MonitoredItemCreateResult>, StatusCode> {
        let results = items
            .iter()
            .map(|item| async_opcua::types::MonitoredItemCreateResult {
                status_code: self
                    .behaviour
                    .lock()
                    .monitor_status
                    .unwrap_or(StatusCode::Good),
                revised_sampling_interval: item.requested_parameters.sampling_interval,
                revised_queue_size: item.requested_parameters.queue_size,
                ..Default::default()
            })
            .collect();
        let mut calls = self.calls.lock();
        calls.events.push(format!("monitor {subscription_id}"));
        calls.monitored.extend(items);
        Ok(results)
    }

    async fn disconnect(&self) {
        self.calls.lock().disconnects += 1;
    }

    async fn closed(&self) {
        self.lost.notified().await
    }
}

// ------------------------------------------------------------------- test setup

struct Names;

impl opcua::link::NameResolver for Names {
    fn subscription_session(&self, name: &str) -> Option<String> {
        matches!(name, "subs" | "early").then(|| "sess".to_string())
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
    parse_link("rec", text, kind, &InfoDefaults::default(), &Names)
        .unwrap_or_else(|e| panic!("parsing '{text}' failed: {e}"))
}

fn node_id_of(link: &LinkInfo) -> NodeId {
    match link.identifier.clone().expect("the link addresses a node") {
        NodeIdentifier::Numeric(n) => NodeId::new(link.namespace_index, n),
        NodeIdentifier::String(s) => NodeId::new(link.namespace_index, s),
    }
}

/// One item with one record bound to the whole node. The item is adopted — given
/// its node, its index and its session — when [`spawn`] puts it on a
/// session, which is where a `Registry` adopts one too.
fn item(text: &str, kind: RecordKind) -> (Arc<Mutex<Item>>, Arc<Mutex<Leaf>>) {
    let info = link(text, kind);
    let mut item = Item::pending();
    // A capacity of one is all a test needs; the record end of the channel is
    // dropped, so a pulse that finds no room is simply dropped.
    let (notify, _rx) = mpsc::channel(1);
    let leaf = Arc::new(Mutex::new(Leaf::new("rec".to_string(), "ai", info, notify)));
    item.leaves.push(leaf.clone());
    (Arc::new(Mutex::new(item)), leaf)
}

fn spawn(
    config: SessionConfig,
    mock: &Arc<Mock>,
    items: Vec<Arc<Mutex<Item>>>,
) -> Arc<SessionHandle> {
    let (handle, worker) = session::create(config, Arc::new(MockConnector(mock.clone())));
    for (index, item) in items.iter().enumerate() {
        let mut item = item.lock();
        let info = item.leaves[0].lock().link.clone();
        let node_id = node_id_of(&info);
        item.adopt(info, node_id, index as u32, handle.clone());
    }
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
    config.set_option("nodes-max", "10").unwrap();
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

/// `nodes-max` sets both limits when it is applied, so of it and a
/// service-specific limit the later option wins (`SessionOpen62541.cpp:418-429`).
#[test]
fn the_later_of_nodes_max_and_a_service_limit_wins() {
    let sizes = |options: &[(&str, &str)]| {
        let mut config = SessionConfig::new("sess", "opc.tcp://host");
        for (key, value) in options {
            config.set_option(key, value).unwrap();
        }
        (
            config.read_batch_size(&ServerInfo::default()),
            config.write_batch_size(&ServerInfo::default()),
        )
    };
    assert_eq!(
        sizes(&[("nodes-max", "8"), ("read-nodes-max", "2")]),
        (2, 8)
    );
    assert_eq!(
        sizes(&[("read-nodes-max", "2"), ("nodes-max", "8")]),
        (8, 8)
    );
    assert_eq!(
        sizes(&[("write-nodes-max", "3"), ("batch-nodes", "5")]),
        (5, 5)
    );
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
    config.set_option("read-timeout-min", "5").unwrap();
    config.set_option("sec-mode", "SignAndEncrypt").unwrap();
    config.set_option("sec-id", "/etc/opcua/creds").unwrap();
    config.set_option("pki-dir", "/etc/pki").unwrap();

    assert_eq!(config.debug, 2);
    assert!(!config.autoconnect);
    assert_eq!((config.read_nodes_max, config.write_nodes_max), (100, 100));
    assert_eq!(config.read_timeout_min, 5.0);
    assert_eq!(
        config.security_mode,
        opcua::client::SecurityMode::SignAndEncrypt
    );
    assert_eq!(config.identity_file.as_deref(), Some("/etc/opcua/creds"));
    assert_eq!(config.pki_dir.as_deref(), Some("/etc/pki"));
}

#[test]
fn a_bad_session_option_is_passed_over() {
    // C reports an unknown name or bad mode and goes on; strtoul reads "x"
    // as 0 and "0x10" as 16.
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.set_option("nonsense", "1").unwrap();
    config.set_option("sec-mode", "Bogus").unwrap();
    config.set_option("nodes-max", "0x10").unwrap();
    assert_eq!(config.read_nodes_max, 16);
    config.set_option("nodes-max", "x").unwrap();
    assert_eq!(config.read_nodes_max, 0);
    assert_eq!(config.security_mode, opcua::client::SecurityMode::Best);
}

#[test]
fn sec_policy_takes_the_short_name_and_keeps_the_uri() {
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.set_option("sec-policy", "Basic256Sha256").unwrap();
    assert_eq!(
        config.security_policy.as_deref(),
        Some("http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256")
    );
    // An unknown name is reported and leaves the policy alone.
    config.set_option("sec-policy", "Rot13").unwrap();
    assert_eq!(
        config.security_policy.as_deref(),
        Some("http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256")
    );
}

#[test]
fn an_illegal_autoconnect_is_the_one_option_error() {
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    assert_eq!(
        config.set_option("autoconnect", "maybe"),
        Err("illegal value 'm'".to_string())
    );
    assert!(config.autoconnect);
}

/// `setupIdentity` on a credentials file holding `contents`.
fn identity_from(contents: &str) -> opcua::client::Identity {
    static N: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let n = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let file = std::env::temp_dir().join(format!("opcua-creds-{}-{n}", std::process::id()));
    std::fs::write(&file, contents).unwrap();
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.identity_file = Some(file.to_string_lossy().into_owned());
    let identity = config.setup_identity();
    std::fs::remove_file(file).unwrap();
    identity
}

#[test]
fn no_credentials_file_is_the_anonymous_token() {
    let config = SessionConfig::new("sess", "opc.tcp://host");
    assert_eq!(config.setup_identity(), opcua::client::Identity::Anonymous);
}

#[test]
fn an_empty_sec_id_is_no_credentials_file() {
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.set_option("sec-id", "/etc/opcua/creds").unwrap();
    config.set_option("sec-id", "").unwrap();
    assert_eq!(config.identity_file, None);
    assert_eq!(config.setup_identity(), opcua::client::Identity::Anonymous);
}

#[test]
fn a_credentials_file_with_user_and_pass_is_a_username_token() {
    assert_eq!(
        identity_from("# operator\nuser=op\npass=se=cret\n"),
        opcua::client::Identity::UserName {
            user: "op".into(),
            password: "se=cret".into()
        }
    );
}

#[test]
fn a_credentials_file_with_cert_and_key_is_a_certificate_token() {
    let cert = std::env::temp_dir().join(format!("opcua-id-{}.der", std::process::id()));
    std::fs::write(&cert, b"der").unwrap();
    let identity = identity_from(&format!("cert={}\nkey=/tmp/id.pem\n", cert.display()));
    std::fs::remove_file(&cert).unwrap();
    assert_eq!(
        identity,
        opcua::client::Identity::Certificate {
            certificate: cert.to_string_lossy().into_owned(),
            private_key: "/tmp/id.pem".into()
        }
    );
}

#[test]
fn an_unusable_credentials_file_falls_back_to_the_anonymous_token() {
    use opcua::client::Identity::Anonymous;
    // A commented-out setting, a lone user, a certificate that cannot be loaded.
    assert_eq!(identity_from("#user=op\n#pass=secret\n"), Anonymous);
    assert_eq!(identity_from("user=op\n"), Anonymous);
    assert_eq!(
        identity_from("cert=/nonexistent/id.der\nkey=/tmp/id.pem\n"),
        Anonymous
    );
    // A directory opens for metadata but cannot be read as a certificate.
    assert_eq!(
        identity_from(&format!(
            "cert={}\nkey=/tmp/id.pem\n",
            std::env::temp_dir().display()
        )),
        Anonymous
    );
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.identity_file = Some("/nonexistent/creds".into());
    assert_eq!(config.setup_identity(), Anonymous);
}

#[test]
fn subscription_options_are_parsed() {
    let mut config = SubscriptionConfig::new("subs", "sess");
    config.set_option("priority", "5");
    config.set_option("debug", "1");
    assert_eq!(config.priority, 5);
    assert_eq!(config.debug, 1);
    // Reported and passed over, as in the C.
    config.set_option("nonsense", "1");
    config.set_option("priority", "256");
    assert_eq!(config.priority, 5);
}

#[test]
fn a_subscription_requests_its_interval_unrounded() {
    let mut config = SubscriptionConfig::new("subs", "sess");
    config.publishing_interval = 33.3;
    config.priority = 4;
    let request = config.request(Default::default());
    assert_eq!(request.requested_publishing_interval, 33.3);
    assert_eq!(request.requested_lifetime_count, 150_150);
    assert_eq!(request.requested_max_keep_alive_count, 10);
    assert_eq!(request.max_notifications_per_publish, 0);
    assert!(request.publishing_enabled);
    assert_eq!(request.priority, 4);
}

// -------------------------------------------------------------------- connecting

#[tokio::test]
async fn connecting_reads_every_item_once() {
    let mock = Mock::arc();
    mock.values
        .lock()
        .insert(NodeId::new(2, "Sensor"), Variant::Int32(11));
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
    let (item1, leaf1) = item("sess ns=2;s=Other monitor=n", INPUT);

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
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);

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
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
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

/// An `opcuaConnect` while an autoconnecting session waits out its retry
/// interval is C's `connect(true)`: it tries at once.
#[tokio::test]
async fn opcua_connect_cuts_the_autoconnect_retry_short() {
    let mock = Mock::arc();
    *mock.refuse_connects.lock() = 1;
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| mock.connects.lock().len() == 1).await;

    // The retry interval is opcua_ConnectTimeout, 5 s; `until` gives up first.
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    assert_eq!(mock.connects.lock().len(), 2);

    // Connected, opcuaConnect only reports it; no further attempt.
    handle.control(Control::Connect);
    tokio::time::sleep(Duration::from_millis(20)).await;
    assert!(handle.is_connected());
    assert_eq!(mock.connects.lock().len(), 2);
}

/// `bini=write` does not use the value just read: the item queues a write request
/// for the record, which answers it by writing its own value back
/// (`manageStateAndBiniProcessing`, `devOpcua.cpp:186-199`).
#[tokio::test]
async fn bini_write_asks_the_record_to_write_after_the_initial_read() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor bini=write monitor=n", OUTPUT);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    until(|| !leaf0.lock().queue.is_empty()).await;

    let updates = drain(&leaf0);
    assert_eq!(updates.len(), 1);
    assert_eq!(updates[0].reason, ProcessReason::WriteRequest);
    assert!(updates[0].data.is_none());
    // The value read is still what the outgoing value's type is taken from.
    assert_eq!(leaf0.lock().incoming, Some(Variant::Int32(42)));
}

/// A record that asked for a write while the session was up gets its value out
/// in the next batch.
#[tokio::test]
async fn a_dirty_leaf_is_written_when_the_record_requests_it() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", OUTPUT);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    drain(&leaf0);
    {
        let mut leaf = leaf0.lock();
        leaf.outgoing = Some(Variant::Int32(7));
        leaf.dirty = true;
    }
    handle.request(Priority::Low, Request::Write { handle: 0 });
    until(|| !mock.calls.lock().writes.is_empty()).await;

    let writes = mock.calls.lock().writes.clone();
    assert_eq!(writes[0][0].node_id, NodeId::new(2, "Sensor"));
    assert_eq!(writes[0][0].value.value, Some(Variant::Int32(7)));
    assert_eq!(pop(&leaf0).reason, ProcessReason::WriteComplete);
}

#[tokio::test]
async fn bini_ignore_drops_the_initial_read_but_still_learns_the_type() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor bini=ignore monitor=n", OUTPUT);

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
    let (item0, _leaf0) = item("sess ns=2;s=Sensor register=y monitor=n", INPUT);

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

    let (item0, _leaf0) = item("sess ns=4;s=Sensor monitor=n", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.add_namespace_mapping(4, "urn:sensors");

    let handle = spawn(config, &mock, vec![item0.clone()]);
    until(|| handle.is_connected()).await;

    assert_eq!(item0.lock().node_id, NodeId::new(2, "Sensor"));
    assert_eq!(item0.lock().configured_node_id, NodeId::new(4, "Sensor"));
}

/// The C's map is keyed by URI (`std::map<std::string, unsigned short>`), so
/// the mappings the server lacks are reported in URI order
/// (`SessionOpen62541::updateNamespaceMap`).
#[tokio::test]
async fn namespaces_the_server_lacks_are_reported_in_uri_order() {
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    let mut mock = Mock::default();
    mock.server.namespace_array = vec!["http://opcfoundation.org/UA/".to_string()];
    let mock = Arc::new(mock);
    let handle = spawn(SessionConfig::new("sess", "opc.tcp://host"), &mock, vec![]);
    for (index, uri) in (10..).zip(["urn:f", "urn:e", "urn:d", "urn:c", "urn:b", "urn:a"]) {
        handle.add_namespace_mapping(index, uri);
    }
    until(|| handle.is_connected()).await;
    errlog_flush();
    errlog_remove_listener(id);
    let reported: Vec<String> = lines
        .lock()
        .iter()
        .filter(|l| l.contains("not found on server"))
        .cloned()
        .collect();
    let expected: Vec<String> = ["urn:a", "urn:b", "urn:c", "urn:d", "urn:e", "urn:f"]
        .iter()
        .map(|uri| {
            format!("OPC UA session sess: locally mapped namespace '{uri}' not found on server\n")
        })
        .collect();
    assert_eq!(reported, expected);
}

/// `SessionOpen62541.cpp:879-883`.
#[tokio::test]
async fn a_read_namespace_array_is_reported_at_debug() {
    let mut mock = Mock::default();
    mock.server.namespace_array = vec![
        "http://opcfoundation.org/UA/".to_string(),
        "urn:server".to_string(),
        "urn:sensors".to_string(),
    ];
    let mock = Arc::new(mock);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.set_option("debug", "1").unwrap();
    config.add_namespace_mapping(4, "urn:sensors");

    let sink = tempfile::NamedTempFile::new().unwrap();
    // SAFETY: nextest runs each test in its own process, so fd 1 is this
    // test's alone.
    let saved = unsafe {
        let saved = libc::dup(1);
        libc::dup2(std::os::fd::AsRawFd::as_raw_fd(sink.as_file()), 1);
        saved
    };
    let handle = spawn(config, &mock, vec![]);
    until(|| handle.is_connected()).await;
    // SAFETY: restoring what this test replaced.
    unsafe {
        libc::dup2(saved, 1);
        libc::close(saved);
    }

    let out = std::fs::read_to_string(sink.path()).unwrap();
    let lines: Vec<&str> = out
        .lines()
        .filter(|l| l.contains("(updateNamespaceMap)"))
        .collect();
    assert_eq!(
        lines,
        [
            "Session sess: (updateNamespaceMap) namespace array with 3 elements read; \
          updating index map with 1 entries"
        ]
    );
}

#[tokio::test]
async fn an_item_whose_namespace_the_server_does_not_have_keeps_its_index() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=4;s=Sensor monitor=n", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.add_namespace_mapping(4, "urn:absent");

    let handle = spawn(config, &mock, vec![item0.clone()]);
    until(|| handle.is_connected()).await;
    assert_eq!(item0.lock().node_id, NodeId::new(4, "Sensor"));
}

// --------------------------------------------------------------------- requests

#[tokio::test]
async fn a_records_read_request_reaches_the_server() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);

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
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", OUTPUT);

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
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", OUTPUT);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0.clone()],
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
    // A write result is an event: the item keeps the status of its last read
    // (`ItemOpen62541::setIncomingEvent`).
    assert_eq!(item0.lock().last_status, StatusCode::Good);
    // The session stays up: a bad value is not a bad connection.
    assert!(handle.is_connected());
}

#[tokio::test]
async fn a_failed_read_reaches_the_record_as_a_read_failure() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0.clone()],
    );
    until(|| handle.is_connected()).await;
    drain(&leaf0);

    mock.behave(|b| b.read_error = Some(StatusCode::BadNodeIdUnknown));
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| !leaf0.lock().queue.is_empty()).await;

    let update = pop(&leaf0);
    assert_eq!(update.reason, ProcessReason::ReadFailure);
    // A failed service is an event; only a value carries a status onto the item.
    assert_eq!(item0.lock().last_status, StatusCode::Good);
    assert!(handle.is_connected());
}

/// A value the server refuses is still `setIncomingData`: its status lands on
/// the item (`ItemOpen62541.cpp:184`).
#[tokio::test]
async fn a_refused_value_reaches_the_record_and_the_item_status() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0.clone()],
    );
    until(|| handle.is_connected()).await;
    drain(&leaf0);

    mock.behave(|b| b.value_status = Some(StatusCode::BadNodeIdUnknown));
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| !leaf0.lock().queue.is_empty()).await;

    assert_eq!(pop(&leaf0).reason, ProcessReason::ReadFailure);
    assert_eq!(item0.lock().last_status, StatusCode::BadNodeIdUnknown);
}

/// Upstream C defect fixed at source: `SessionOpen62541::readComplete` bounds-checks
/// the DataType result and then reads the Value result at `i + 1` unchecked, so a
/// server that returns fewer results than requested reads past the end of the array.
#[tokio::test]
async fn a_server_that_returns_too_few_results_does_not_read_past_the_end() {
    let mock = Mock::arc();
    mock.behave(|b| b.truncate_read = true);
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);

    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    until(|| !leaf0.lock().queue.is_empty()).await;

    assert_eq!(pop(&leaf0).reason, ProcessReason::ReadFailure);
}

/// C's `readComplete` reports a short answer before handling it
/// (`SessionOpen62541.cpp:2549-2554`).
#[tokio::test]
async fn a_short_read_answer_is_reported() {
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    let mock = Mock::arc();
    mock.behave(|b| b.truncate_read = true);
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0],
    );
    until(|| handle.is_connected()).await;
    until(|| !leaf0.lock().queue.is_empty()).await;
    errlog_flush();
    errlog_remove_listener(id);
    let reported: Vec<String> = lines
        .lock()
        .iter()
        .filter(|l| l.contains("(readComplete)"))
        .cloned()
        .collect();
    assert_eq!(
        reported,
        [
            "OPC UA session sess: (readComplete) received a callback with 1 values for a request containing 1 items\n"
        ]
    );
}

/// The request and completion debug lines carry the service call's
/// transaction id (`SessionOpen62541.cpp:696-701`, `:774-779`, `:2543-2547`,
/// `:2587-2592`, `:2618-2622`, `:2641-2646`).
#[tokio::test]
async fn read_and_write_debug_lines_name_the_transaction() {
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", OUTPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.set_option("debug", "5").unwrap();

    let sink = tempfile::NamedTempFile::new().unwrap();
    // SAFETY: nextest runs each test in its own process, so fd 1 is this
    // test's alone.
    let saved = unsafe {
        let saved = libc::dup(1);
        libc::dup2(std::os::fd::AsRawFd::as_raw_fd(sink.as_file()), 1);
        saved
    };
    let handle = spawn(config, &mock, vec![item0]);
    until(|| handle.is_connected()).await;
    until(|| !leaf0.lock().queue.is_empty()).await;
    drain(&leaf0);
    let write = || {
        let mut leaf = leaf0.lock();
        leaf.outgoing = Some(Variant::Int32(7));
        leaf.dirty = true;
    };
    write();
    handle.request(Priority::Low, Request::Write { handle: 0 });
    until(|| !leaf0.lock().queue.is_empty()).await;
    drain(&leaf0);

    mock.behave(|b| {
        b.read_error = Some(StatusCode::BadNodeIdUnknown);
        b.write_error = Some(StatusCode::BadNodeIdUnknown);
    });
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| !leaf0.lock().queue.is_empty()).await;
    drain(&leaf0);
    write();
    handle.request(Priority::Low, Request::Write { handle: 0 });
    until(|| !leaf0.lock().queue.is_empty()).await;
    // SAFETY: restoring what this test replaced.
    unsafe {
        libc::dup2(saved, 1);
        libc::close(saved);
    }

    let out = std::fs::read_to_string(sink.path()).unwrap();
    let lines: Vec<&str> = out
        .lines()
        .filter(|l| l.contains("Read)") || l.contains("Write)") || l.contains("Complete)"))
        .collect();
    assert_eq!(
        lines,
        [
            "Session sess: (requestRead) beginRead service ok (transaction id 1; retrieving 1 nodes)",
            "Session sess: (readComplete) getting data for read service (transaction id 1; data for 2 items)",
            "Session sess: (requestWrite) beginWrite service ok (transaction id 1; writing 1 nodes)",
            "Session sess: (writeComplete) getting results for write service (transaction id 1; results for 1 items)",
            "Session sess: (requestRead) beginRead service ok (transaction id 2; retrieving 1 nodes)",
            "Session sess: (readComplete) for read service (transaction id 2) failed with status BadNodeIdUnknown",
            "Session sess: (requestWrite) beginWrite service ok (transaction id 2; writing 1 nodes)",
            "Session sess: (writeComplete) for write service (transaction id 2) failed with status BadNodeIdUnknown",
        ]
    );
}

#[tokio::test]
async fn a_batch_is_split_at_the_servers_limit() {
    let mut mock = Mock::default();
    // Two nodes per item, so one item per read.
    mock.server.max_nodes_per_read = 2;
    let mock = Arc::new(mock);

    let (item0, _l0) = item("sess ns=2;s=A monitor=n", INPUT);
    let (item1, _l1) = item("sess ns=2;s=B monitor=n", INPUT);
    let (item2, _l2) = item("sess ns=2;s=C monitor=n", INPUT);

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

    let (item0, _l0) = item("sess ns=2;s=Low monitor=n", INPUT);
    let (item1, _l1) = item("sess ns=2;s=High monitor=n", INPUT);
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
    let (item0, leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;

    let handle = spawn(config, &mock, vec![item0.clone()]);
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    drain(&leaf0);

    *mock.refuse_connects.lock() = 1;
    mock.behave(|b| b.read_error = Some(StatusCode::BadConnectionClosed));
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| !handle.is_connected()).await;

    let updates = drain(&leaf0);
    assert_eq!(updates[0].reason, ProcessReason::ReadFailure);
    assert_eq!(updates[1].reason, ProcessReason::ConnectionLoss);
    // `ItemOpen62541::setIncomingEvent` (`ItemOpen62541.cpp:215-216`).
    assert_eq!(item0.lock().last_status, StatusCode::BadServerNotConnected);
    assert_eq!(item0.lock().state, ConnectionStatus::Down);
    assert_eq!(mock.calls.lock().disconnects, 1);
}

/// C prints the ACTIVATED -> CREATED transition (`SessionOpen62541.cpp:2520-2523`).
#[tokio::test]
async fn a_lost_connection_prints_disconnected() {
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;
    let handle = spawn(config, &mock, vec![item0]);
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;

    *mock.refuse_connects.lock() = 1;
    mock.behave(|b| b.read_error = Some(StatusCode::BadConnectionClosed));
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| !handle.is_connected()).await;
    errlog_flush();
    errlog_remove_listener(id);
    assert!(
        lines
            .lock()
            .iter()
            .any(|l| l == "OPC UA session sess: disconnected\n")
    );
}

/// `opcuaDisconnect` takes the session from ACTIVATED to CREATED too, so C
/// prints the same line (`SessionOpen62541.cpp:2520-2523`).
#[tokio::test]
async fn a_requested_disconnect_prints_disconnected() {
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;
    let handle = spawn(config, &mock, vec![item0]);
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;

    handle.control(Control::Disconnect);
    until(|| !handle.is_connected()).await;
    errlog_flush();
    errlog_remove_listener(id);
    assert_eq!(
        lines
            .lock()
            .iter()
            .filter(|l| *l == "OPC UA session sess: disconnected\n")
            .count(),
        1
    );
}

/// The connection fields of the `opcuaShow` line, and `registered=`.
fn shown_state(handle: &SessionHandle) -> String {
    let mut out = Vec::new();
    handle.show(0, &mut out, &mut Vec::new()).unwrap();
    let line = String::from_utf8(out).unwrap();
    let field = |key: &str| {
        let start = line.find(key).expect(key) + key.len();
        line[start..].split(' ').next().unwrap().to_string()
    };
    format!(
        "{} {} {} registered={}",
        field("connect status="),
        field("sessionState="),
        field("channelState="),
        field("registered="),
    )
}

/// The states the C oracle IOC shows (`SessionOpen62541::show`): unconnected,
/// a refused connect, connected, disconnected on request, and lost.
#[tokio::test]
async fn the_shown_connection_state_follows_the_client() {
    let mock = Mock::arc();
    *mock.refuse_connects.lock() = 1;
    let (item0, _leaf0) = item("sess ns=2;s=Sensor register=y monitor=n", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;
    let handle = spawn(config, &mock, vec![item0]);
    assert_eq!(
        shown_state(&handle),
        "BadInvalidState Closed Closed registered=0"
    );

    handle.control(Control::Connect);
    until(|| mock.connects.lock().len() == 1).await;
    until(|| shown_state(&handle) == "BadNotConnected Closed Closed registered=0").await;

    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    assert_eq!(shown_state(&handle), "Good Activated Open registered=1");

    handle.control(Control::Disconnect);
    until(|| !handle.is_connected()).await;
    assert_eq!(shown_state(&handle), "Good Closed Closed registered=0");

    // The lost session's one reconnect attempt is refused, so its status is
    // the connect status (the C shows BadDisconnect for a refused TCP connect).
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    *mock.refuse_connects.lock() = 1;
    mock.behave(|b| b.read_error = Some(StatusCode::BadConnectionClosed));
    handle.request(Priority::Low, Request::Read { handle: 0 });
    until(|| mock.connects.lock().len() == 4).await;
    until(|| shown_state(&handle) == "BadNotConnected Created Fresh registered=0").await;
}

/// open62541 notices a dropped connection in `UA_Client_run_iterate` with no
/// request outstanding, takes the session to CREATED/FRESH and makes one
/// reconnect attempt at once, autoconnect or not (`connectIterate`).
#[tokio::test]
async fn a_connection_the_server_drops_is_lost_and_retried_once() {
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    let mock = Mock::arc();
    let (item0, leaf0) = item("sess ns=2;s=Sensor register=y monitor=n", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;
    let handle = spawn(config, &mock, vec![item0]);
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    drain(&leaf0);
    let reads = mock.calls.lock().reads.len();

    *mock.refuse_connects.lock() = 1;
    mock.lost.notify_one();
    until(|| shown_state(&handle) == "BadNotConnected Created Fresh registered=0").await;
    assert!(!handle.is_connected());
    assert_eq!(drain(&leaf0)[0].reason, ProcessReason::ConnectionLoss);
    assert_eq!(mock.calls.lock().reads.len(), reads);
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(mock.connects.lock().len(), 2);

    errlog_flush();
    errlog_remove_listener(id);
    let lines = lines.lock();
    assert_eq!(
        lines
            .iter()
            .filter(|l| *l == "OPC UA session sess: disconnected\n")
            .count(),
        1
    );
    assert!(!lines.iter().any(|l| l.contains("connect service failed")));
}

#[tokio::test]
async fn a_connection_the_server_drops_reconnects_at_once() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;
    let handle = spawn(config, &mock, vec![item0]);
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;

    mock.lost.notify_one();
    until(|| mock.connects.lock().len() == 2).await;
    until(|| handle.is_connected()).await;
    assert_eq!(mock.calls.lock().disconnects, 1);
    assert_eq!(shown_state(&handle), "Good Activated Open registered=0");
}

#[tokio::test]
async fn requests_queued_for_a_lost_connection_are_dropped() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
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
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);

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
    assert_eq!(request.requested_parameters.client_handle, 1);
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

/// The `ACTIVATED` arm creates every subscription, in name order, before the
/// initial read, and adds their monitored items only after it
/// (`SessionOpen62541.cpp:2493-2516`).
#[tokio::test]
async fn subscriptions_are_created_before_the_initial_read_and_monitor_after_it() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("subs ns=2;s=A", INPUT);
    let (item1, _leaf1) = item("early ns=2;s=B", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.set_option("debug", "1").unwrap();

    let sink = tempfile::NamedTempFile::new().unwrap();
    // SAFETY: nextest runs each test in its own process, so fd 1 is this
    // test's alone.
    let saved = unsafe {
        let saved = libc::dup(1);
        libc::dup2(std::os::fd::AsRawFd::as_raw_fd(sink.as_file()), 1);
        saved
    };
    let handle = spawn(config, &mock, vec![item0, item1]);
    for name in ["subs", "early"] {
        let mut subscription = SubscriptionConfig::new(name, "sess");
        subscription.debug = 1;
        handle.subscriptions.lock().push(subscription);
    }
    until(|| mock.calls.lock().monitored.len() == 2).await;
    // SAFETY: restoring what this test replaced.
    unsafe {
        libc::dup2(saved, 1);
        libc::close(saved);
    }

    assert_eq!(
        mock.calls.lock().events[..5],
        [
            "create early",
            "create subs",
            "read",
            "monitor 1",
            "monitor 2"
        ]
    );
    let out = std::fs::read_to_string(sink.path()).unwrap();
    let lines: Vec<&str> = out
        .lines()
        .filter(|l| l.contains("triggering") || l.starts_with("Subscription "))
        .collect();
    assert_eq!(
        lines,
        [
            "Session sess: triggering initial read for all 2 items",
            "Subscription early@sess: created 1 monitored items (Good)",
            "Subscription subs@sess: created 1 monitored items (Good)",
        ]
    );
}

/// `SubscriptionOpen62541::addMonitoredItems` makes one CreateMonitoredItems
/// call per item, and open62541 numbers the session's monitored items from 1 in
/// the order they are created (`++client->monitoredItemHandles`).
#[tokio::test]
async fn each_monitored_item_is_its_own_call_under_a_session_wide_handle() {
    let mock = Mock::arc();
    let (unmonitored, _l0) = item("sess ns=2;s=U monitor=n", INPUT);
    let (item_a, _la) = item("subs ns=2;s=A", INPUT);
    let (item_b, _lb) = item("early ns=2;s=B", INPUT);
    let (item_c, _lc) = item("subs ns=2;s=C", INPUT);
    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![unmonitored, item_a, item_b, item_c],
    );
    for name in ["subs", "early"] {
        handle
            .subscriptions
            .lock()
            .push(SubscriptionConfig::new(name, "sess"));
    }
    until(|| mock.calls.lock().monitored.len() == 3).await;

    let calls = mock.calls.lock();
    let monitors: Vec<&str> = calls
        .events
        .iter()
        .filter(|e| e.starts_with("monitor"))
        .map(String::as_str)
        .collect();
    assert_eq!(monitors, ["monitor 1", "monitor 2", "monitor 2"]);
    let created: Vec<(NodeId, u32)> = calls
        .monitored
        .iter()
        .map(|r| {
            (
                r.item_to_monitor.node_id.clone(),
                r.requested_parameters.client_handle,
            )
        })
        .collect();
    assert_eq!(
        created,
        [
            (NodeId::new(2, "B"), 1),
            (NodeId::new(2, "A"), 2),
            (NodeId::new(2, "C"), 3),
        ]
    );
}

/// C still runs `addMonitoredItems` for a subscription whose create failed, so
/// each of its items is told the connection is lost.
#[tokio::test]
async fn a_refused_subscription_still_fails_its_items() {
    let mock = Mock::arc();
    mock.behave(|b| b.subscription_error = Some(StatusCode::BadTooManySubscriptions));
    let (item0, leaf0) = item("subs ns=2;s=Sensor", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;
    let handle = spawn(config, &mock, vec![item0.clone()]);
    handle
        .subscriptions
        .lock()
        .push(SubscriptionConfig::new("subs", "sess"));
    handle.control(Control::Connect);
    until(|| !mock.calls.lock().subscriptions.is_empty()).await;
    until(|| item0.lock().state == ConnectionStatus::Down).await;

    assert!(mock.calls.lock().monitored.is_empty());
    let last = drain(&leaf0).pop().expect("the leaf has an update");
    assert_eq!(last.reason, ProcessReason::ConnectionLoss);
    assert_eq!(item0.lock().last_status, StatusCode::BadServerNotConnected);
}

/// `SubscriptionOpen62541::show` prints the server's revised interval beside
/// the requested one: the requested one until a create answers, and 0 once one
/// fails, as the reinitialised response leaves it (`SubscriptionOpen62541.cpp:48`,
/// `:127`).
#[tokio::test]
async fn a_subscription_shows_the_interval_the_server_revised() {
    for (error, shown) in [
        (None, "300(250)"),
        (Some(StatusCode::BadTooManySubscriptions), "0(250)"),
    ] {
        let mock = Mock::arc();
        mock.behave(|b| {
            b.subscription_error = error;
            b.revised_interval = Some(300.0);
        });
        let mut config = SessionConfig::new("sess", "opc.tcp://host");
        config.autoconnect = false;
        let handle = spawn(config, &mock, vec![]);
        let mut subscription = SubscriptionConfig::new("subs", "sess");
        subscription.publishing_interval = 250.0;
        handle.subscriptions.lock().push(subscription.clone());
        let line = |interval: &str| {
            format!(
                "subscription=subs session=sess interval={interval} prio=0 enable=?(Y) \
                 debug=0 items=0\n"
            )
        };
        assert_eq!(subscription.show(&handle, 0), line("250(250)"));

        handle.control(Control::Connect);
        until(|| subscription.show(&handle, 0) == line(shown)).await;
    }
}

/// `SubscriptionOpen62541::addMonitoredItems` files a connection loss for an
/// item the server refuses to monitor.
#[tokio::test]
async fn a_refused_monitored_item_is_told_the_connection_is_lost() {
    let mock = Mock::arc();
    mock.behave(|b| b.monitor_status = Some(StatusCode::BadNodeIdUnknown));
    let (item0, leaf0) = item("subs ns=2;s=Sensor", INPUT);
    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0.clone()],
    );
    handle
        .subscriptions
        .lock()
        .push(SubscriptionConfig::new("subs", "sess"));
    until(|| !mock.calls.lock().monitored.is_empty()).await;
    until(|| item0.lock().state == ConnectionStatus::Down).await;

    let last = drain(&leaf0).pop().expect("the leaf has an update");
    assert_eq!(last.reason, ProcessReason::ConnectionLoss);
    assert_eq!(item0.lock().last_status, StatusCode::BadServerNotConnected);
}

#[tokio::test]
async fn an_unmonitored_item_gets_no_monitored_item() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("subs ns=2;s=Sensor monitor=n", INPUT);
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
    let (item0, leaf0) = item("subs ns=2;s=A", INPUT);
    let (item1, leaf1) = item("subs ns=2;s=B", INPUT);
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
        client_handle: 2,
        value: DataValue::value_only(Variant::Int32(64)),
    })
    .unwrap();
    until(|| !leaf1.lock().queue.is_empty()).await;

    let update = pop(&leaf1);
    assert_eq!(update.reason, ProcessReason::IncomingData);
    assert_eq!(update.data, Some(Variant::Int32(64)));
    assert!(leaf0.lock().queue.is_empty());
}

#[test]
fn an_item_shows_itself_as_item_open62541_show_does() {
    let (item0, _leaf0) = item("subs ns=2;s=Sensor register=y", INPUT);
    let (handle, _worker) = session::create(
        SessionConfig::new("sess", "opc.tcp://host"),
        Arc::new(MockConnector(Mock::arc())),
    );
    let mut item = item0.lock();
    let info = item.leaves[0].lock().link.clone();
    item.adopt(info, NodeId::new(2, "Sensor"), 0, handle);

    assert_eq!(
        item.show_line(),
        "item ns=0(2);s=Sensor record=rec state=down status=BadServerNotConnected dataDirty=n context=subs@sess \
         sampling=0(-1) deadband=0 qsize=0(1) cqsize=3 discard=old timestamp=server \
         bini=read output=n monitor=y registered=-(y)"
    );
}

#[tokio::test]
async fn an_item_shows_what_the_server_revised() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item("subs ns=2;s=Sensor sampling=250 qsize=4", INPUT);
    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0.clone()],
    );
    handle
        .subscriptions
        .lock()
        .push(SubscriptionConfig::new("subs", "sess"));
    until(|| item0.lock().revised_queue_size == 4).await;

    let line = item0.lock().show_line();
    assert!(line.contains(" sampling=250(250) "), "{line}");
    assert!(line.contains(" qsize=4(4) "), "{line}");
}

/// C++ streams print a double with 6 significant digits (`%g`), in
/// `ItemOpen62541::show` and in the monitored-item debug line
/// (`SubscriptionOpen62541.cpp:186-191`).
#[tokio::test]
async fn sampling_and_deadband_print_as_a_cpp_stream_does() {
    let mock = Mock::arc();
    let (item0, _leaf0) = item(
        "subs ns=2;s=Sensor sampling=33.3333333 deadband=1e20 qsize=4",
        INPUT,
    );
    let mut subscription = SubscriptionConfig::new("subs", "sess");
    subscription.set_option("debug", "5");

    let sink = tempfile::NamedTempFile::new().unwrap();
    // SAFETY: nextest runs each test in its own process, so fd 1 is this
    // test's alone.
    let saved = unsafe {
        let saved = libc::dup(1);
        libc::dup2(std::os::fd::AsRawFd::as_raw_fd(sink.as_file()), 1);
        saved
    };
    let handle = spawn(
        SessionConfig::new("sess", "opc.tcp://host"),
        &mock,
        vec![item0.clone()],
    );
    handle.subscriptions.lock().push(subscription);
    until(|| item0.lock().revised_queue_size == 4).await;
    // SAFETY: restoring what this test replaced.
    unsafe {
        libc::dup2(saved, 1);
        libc::close(saved);
    }

    let line = item0.lock().show_line();
    assert!(
        line.contains(" sampling=33.3333(33.3333) deadband=1e+20 "),
        "{line}"
    );
    let out = std::fs::read_to_string(sink.path()).unwrap();
    assert!(
        out.contains(" revised sampling interval 33.3333 revised queue size 4"),
        "{out}"
    );
}

#[test]
fn an_item_shows_the_size_of_its_element_tree() {
    const ITEM: RecordKind = RecordKind {
        is_output: false,
        is_item_record: true,
    };
    let mut item = Item::pending();
    let (notify, _rx) = mpsc::channel(1);
    for (record, text, kind) in [
        ("ITEM", "subs ns=2;s=Struct", ITEM),
        ("A_B", "ITEM element=a.b", INPUT),
        ("A_C", "ITEM element=a.c", INPUT),
        ("D", "ITEM element=d", INPUT),
    ] {
        let leaf = Leaf::new(record.to_string(), "ai", link(text, kind), notify.clone());
        item.leaves.push(Arc::new(Mutex::new(leaf)));
    }

    let line = item.show_line();
    assert!(line.ends_with(" dataNodes=2 dataLeafs=3"), "{line}");
}

/// From level 1 an item shows its data tree below its line: nodes named by
/// the element paths, in the order the records added them, and a leaf per
/// element record (`DataElementOpen62541Node::show`,
/// `DataElementOpen62541Leaf::show`).
#[test]
fn an_item_shows_its_element_tree_from_level_one() {
    const ITEM: RecordKind = RecordKind {
        is_output: false,
        is_item_record: true,
    };
    let mut item = Item::pending();
    let (notify, _rx) = mpsc::channel(1);
    for (record, rtyp, text, kind) in [
        ("ITEM", "opcuaItem", "subs ns=2;s=Struct", ITEM),
        ("A_B", "ai", "ITEM element=a.b", INPUT),
        ("D", "stringin", "ITEM element=d monitor=n", INPUT),
        ("A_C", "longout", "ITEM element=a.c", OUTPUT),
    ] {
        let leaf = Leaf::new(record.to_string(), rtyp, link(text, kind), notify.clone());
        item.leaves.push(Arc::new(Mutex::new(leaf)));
    }
    item.leaves[2].lock().incoming = Some(Variant::from("text"));

    let line = item.show_line();
    assert_eq!(item.show(0), format!("{line}\n"));
    assert_eq!(
        item.show(1),
        format!(
            "{line}\n\
             \x20 node=[ROOT] children=2 mapped=n\n\
             \x20   node=a children=2 mapped=n\n\
             \x20     leaf=b record(ai)=A_B type=Null timestamp=server bini=read monitor=y\n\
             \x20     leaf=c record(longout)=A_C type=Null timestamp=server bini=read monitor=y\n\
             \x20   leaf=d record(stringin)=D type=String timestamp=server bini=read monitor=n\n"
        )
    );
}

/// An item with one record bound to the whole node has that record's leaf
/// as its root.
#[test]
fn a_single_record_is_the_root_leaf() {
    let (item0, leaf0) = item("subs ns=2;s=Sensor", INPUT);
    leaf0.lock().incoming = Some(Variant::Double(1.5));
    let item = item0.lock();
    assert_eq!(
        item.show(1),
        format!(
            "{}\n  leaf=[ROOT] record(ai)=rec type=Double timestamp=server bini=read monitor=y\n",
            item.show_line()
        )
    );
}

#[test]
fn the_element_map_reports_what_the_structure_cannot_supply() {
    use async_opcua::types::custom::{
        DataTypeTree, DynamicStructure, EncodingIds, ParentIds, ParsedStructureField,
        StructTypeInfo,
    };
    use async_opcua::types::{ExtensionObject, StructureType, VariantScalarTypeId};

    let field = |name: &str| ParsedStructureField {
        name: name.into(),
        type_id: async_opcua::types::DataTypeId::Int32.into(),
        value_rank: -1,
        array_dimensions: None,
        is_optional: false,
        scalar_type: VariantScalarTypeId::Int32,
    };
    let type_id = NodeId::new(2, 3000);
    let info = Arc::new(StructTypeInfo {
        structure_type: StructureType::Structure,
        fields: vec![field("x"), field("y")],
        index_by_name: [("x".to_string(), 0), ("y".to_string(), 1)].into(),
        encoding_ids: EncodingIds {
            binary_id: NodeId::new(2, 3001),
            json_id: NodeId::new(2, 3002),
            xml_id: NodeId::new(2, 3003),
        },
        is_abstract: false,
        node_id: type_id.clone(),
        name: "Point".into(),
    });
    let mut tree = DataTypeTree::new(ParentIds::new());
    tree.add_type(
        type_id,
        async_opcua::types::custom::TypeInfo::Struct(info.clone()),
    );
    let tree = Arc::new(tree);
    let point = DynamicStructure::new_struct(
        info,
        tree.clone(),
        vec![Variant::Int32(1), Variant::Int32(2)],
    )
    .unwrap();
    let value = Variant::ExtensionObject(ExtensionObject::from_message(point));

    let mut item = Item::pending();
    item.node_id = NodeId::new(2, "Pt");
    item.type_tree = Some(tree);
    let (notify, _rx) = mpsc::channel(1);
    for (record, text) in [
        ("Y", "PT element=y"),
        ("Z", "PT element=z"),
        ("XQ", "PT element=x.q"),
    ] {
        let leaf = Leaf::new(record.to_string(), "ai", link(text, INPUT), notify.clone());
        item.leaves.push(Arc::new(Mutex::new(leaf)));
    }

    assert_eq!(
        item.element_report(&value),
        vec![
            "Item ns=2;s=Pt: element z not found in Point".to_string(),
            "Error: ns=2;s=Pt element x is no structured data but a Int32".to_string(),
        ]
    );
}

/// open62541's 1 s connectivity check on an idle session
/// (`SessionOpen62541::connectionInactive`).
#[tokio::test]
async fn an_idle_session_whose_server_stops_answering_goes_inactive() {
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    let mock = Mock::arc();
    let (item0, _leaf0) = item("sess ns=2;s=Sensor monitor=n", INPUT);
    let mut config = SessionConfig::new("sess", "opc.tcp://host");
    config.autoconnect = false;
    let handle = spawn(config, &mock, vec![item0]);
    handle.control(Control::Connect);
    until(|| handle.is_connected()).await;
    mock.calls.lock().reads.clear();

    // A check that is answered keeps the session up.
    tokio::time::sleep(Duration::from_millis(1200)).await;
    assert!(handle.is_connected());
    let state = NodeId::new(0, 2259u32);
    assert!(
        mock.calls
            .lock()
            .reads
            .iter()
            .any(|r| r.len() == 1 && r[0].node_id == state)
    );

    *mock.refuse_connects.lock() = 1;
    mock.behave(|b| b.read_error = Some(StatusCode::BadTimeout));
    until(|| !handle.is_connected()).await;
    errlog_flush();
    errlog_remove_listener(id);
    assert!(
        lines
            .lock()
            .iter()
            .any(|l| l == "OPC UA Session sess: server inactive\n")
    );
}
