//! The sessions at IOC exit (`SessionOpen62541::atExit`,
//! `SessionOpen62541.cpp:2713-2723`): one console line, then every session
//! closed — a connected one with CloseSession, an attempt in flight abandoned.
//!
//! The exit callbacks are the process's, so each test here runs alone in its
//! own process under nextest.

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use async_opcua::types::{
    DataValue, MonitoredItemCreateRequest, MonitoredItemCreateResult, NodeId, ReadValueId,
    StatusCode, WriteValue,
};
use async_trait::async_trait;
use epics_rs::base::runtime::exit::call_at_exits;
use epics_rs::base::runtime::log::{errlog_add_listener, errlog_flush, errlog_remove_listener};
use parking_lot::Mutex;
use tokio::sync::mpsc;

use opcua::client::{
    ConnectError, CreatedSubscription, Identity, Notification, ServerInfo, UaConnection,
    UaConnector,
};
use opcua::ioc;
use opcua::registry::Registry;
use opcua::session::SessionConfig;
use opcua::subscription::SubscriptionConfig;

#[derive(Default)]
struct Server {
    connects: AtomicUsize,
    disconnects: AtomicUsize,
    /// Connect attempts never finish — a server that does not answer.
    hang: bool,
    /// Connect attempts fail.
    refuse: bool,
}

struct Connector(Arc<Server>);

#[async_trait]
impl UaConnector for Connector {
    async fn connect(
        &self,
        _config: &SessionConfig,
        _identity: &Identity,
    ) -> Result<Arc<dyn UaConnection>, ConnectError> {
        self.0.connects.fetch_add(1, Ordering::SeqCst);
        if self.0.hang {
            std::future::pending::<()>().await;
        }
        if self.0.refuse {
            return Err(ConnectError::setup("refused"));
        }
        Ok(Arc::new(Connection(self.0.clone())))
    }
}

struct Connection(Arc<Server>);

#[async_trait]
impl UaConnection for Connection {
    async fn read(&self, nodes: &[ReadValueId]) -> (u32, Result<Vec<DataValue>, StatusCode>) {
        (0, Ok(vec![DataValue::default(); nodes.len()]))
    }
    async fn write(&self, values: &[WriteValue]) -> (u32, Result<Vec<StatusCode>, StatusCode>) {
        (0, Ok(vec![StatusCode::Good; values.len()]))
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
        config: &SubscriptionConfig,
        _sink: mpsc::UnboundedSender<Notification>,
    ) -> Result<CreatedSubscription, StatusCode> {
        Ok(CreatedSubscription {
            id: 1,
            revised_publishing_interval: config.publishing_interval,
        })
    }
    async fn create_monitored_items(
        &self,
        _subscription_id: u32,
        items: Vec<MonitoredItemCreateRequest>,
    ) -> Result<Vec<MonitoredItemCreateResult>, StatusCode> {
        Ok(vec![MonitoredItemCreateResult::default(); items.len()])
    }
    async fn disconnect(&self) {
        self.0.disconnects.fetch_add(1, Ordering::SeqCst);
    }
}

fn registry(server: &Arc<Server>, autoconnect: &str) -> Arc<Registry> {
    let registry = Registry::new(Arc::new(Connector(server.clone())));
    ioc::session_command(
        &registry,
        Some("S1"),
        Some("opc.tcp://localhost:4999"),
        &[autoconnect],
    );
    registry
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

/// Run the exit callbacks and return what they logged and how long they took.
fn exit() -> (Vec<String>, Duration) {
    // Lines still queued from the start are not the exit's.
    errlog_flush();
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    let started = Instant::now();
    call_at_exits();
    let took = started.elapsed();
    errlog_flush();
    errlog_remove_listener(id);
    let lines = lines.lock().clone();
    (lines, took)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_connected_session_is_closed_at_exit_and_stays_closed() {
    let server = Arc::new(Server::default());
    let registry = registry(&server, "autoconnect=y");
    registry.start();
    let session = registry.session("S1").unwrap();
    until(|| session.is_connected()).await;

    let (lines, _) = exit();
    assert_eq!(lines, ["OPC UA: Disconnecting sessions\n"]);
    // The exit returns once the session has closed.
    assert_eq!(server.disconnects.load(Ordering::SeqCst), 1);
    assert!(!session.is_connected());

    // An autoconnect session closed at exit does not come back.
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(server.connects.load(Ordering::SeqCst), 1);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_connect_in_flight_is_abandoned_at_exit() {
    let server = Arc::new(Server {
        hang: true,
        ..Default::default()
    });
    let registry = registry(&server, "autoconnect=y");
    registry.start();
    until(|| server.connects.load(Ordering::SeqCst) == 1).await;

    let (lines, took) = exit();
    assert!(
        lines.contains(&"OPC UA: Disconnecting sessions\n".to_string()),
        "{lines:?}"
    );
    assert!(took < Duration::from_secs(1), "the exit waited {took:?}");
    assert_eq!(server.disconnects.load(Ordering::SeqCst), 0);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_session_never_connected_only_prints_the_exit_line() {
    let server = Arc::new(Server::default());
    let registry = registry(&server, "autoconnect=n");
    registry.start();

    let (lines, took) = exit();
    assert_eq!(lines, ["OPC UA: Disconnecting sessions\n"]);
    assert!(took < Duration::from_secs(1), "the exit waited {took:?}");
    assert_eq!(server.connects.load(Ordering::SeqCst), 0);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_session_waiting_to_retry_ends_at_exit() {
    let server = Arc::new(Server {
        refuse: true,
        ..Default::default()
    });
    let registry = registry(&server, "autoconnect=y");
    registry.start();
    until(|| server.connects.load(Ordering::SeqCst) == 1).await;

    let (lines, took) = exit();
    assert_eq!(lines, ["OPC UA: Disconnecting sessions\n"]);
    assert!(took < Duration::from_secs(1), "the exit waited {took:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_exit_callbacks_may_run_off_the_runtime() {
    let server = Arc::new(Server::default());
    let registry = registry(&server, "autoconnect=y");
    registry.start();
    let session = registry.session("S1").unwrap();
    until(|| session.is_connected()).await;

    let (lines, _) = std::thread::spawn(exit)
        .join()
        .expect("the exit callbacks ran");
    assert!(
        lines.contains(&"OPC UA: Disconnecting sessions\n".to_string()),
        "{lines:?}"
    );
    assert_eq!(server.disconnects.load(Ordering::SeqCst), 1);
}
