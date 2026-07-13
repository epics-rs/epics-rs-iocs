//! Session configuration and the session's worker
//! (`Session.cpp`, `SessionOpen62541.cpp`, `RequestQueueBatcher.h`).
//!
//! One worker task per session owns the connection. It supervises it (connect,
//! reconnect, connection loss), batches the records' read and write requests
//! into service calls, and dispatches everything the server sends back into the
//! items. Records never touch the connection.

use std::collections::{HashMap, VecDeque};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use async_opcua::types::{
    AttributeId, DataValue, EndpointDescription, MonitoredItemCreateRequest, NodeId, ReadValueId,
    StatusCode, Variant, WriteValue,
};
use parking_lot::Mutex;
use tokio::sync::mpsc;

use crate::client::{Identity, SecurityMode, ServerInfo, UaConnection, UaConnector};
use crate::defaults;
use crate::item::Item;
use crate::queue::{ConnectionStatus, ProcessReason};
use crate::subscription::SubscriptionConfig;

/// The attributes one Read asks for per item: DataType and Value
/// (`Session.h:359`, `no_of_properties_read`).
const NO_OF_PROPERTIES_READ: usize = 2;

/// `opcuaSession(NAME, URL, [options])` (`iocshIntegration.cpp:88-140`) plus the
/// session options of `opcuaOptions` (`SessionOpen62541.cpp:159-260`).
#[derive(Debug, Clone, PartialEq)]
pub struct SessionConfig {
    pub name: String,
    pub url: String,
    pub debug: u32,
    /// Reconnect on connection loss and connect at `iocInit`.
    pub autoconnect: bool,
    /// Per-service-call node limits (0 = no client-side limit).
    pub nodes_max: u32,
    pub read_nodes_max: u32,
    pub write_nodes_max: u32,
    /// Batch hold-off bounds [ms] (`read-timeout-min`/`read-timeout-max` and
    /// their write counterparts).
    pub read_timeout_min: f64,
    pub read_timeout_max: f64,
    pub write_timeout_min: f64,
    pub write_timeout_max: f64,
    pub security_mode: SecurityMode,
    pub security_policy: Option<String>,
    pub identity: Identity,
    pub pki_dir: String,
    pub certificate_path: Option<String>,
    pub private_key_path: Option<String>,
    pub trust_server_certs: bool,
    /// Namespace index → URI, from `opcuaMapNamespace`.
    pub namespace_map: HashMap<u16, String>,
}

impl SessionConfig {
    pub fn new(name: impl Into<String>, url: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            url: url.into(),
            debug: 0,
            autoconnect: true,
            nodes_max: 0,
            read_nodes_max: 0,
            write_nodes_max: 0,
            read_timeout_min: 0.0,
            read_timeout_max: 0.0,
            write_timeout_min: 0.0,
            write_timeout_max: 0.0,
            security_mode: SecurityMode::Best,
            security_policy: None,
            identity: Identity::Anonymous,
            pki_dir: "pki".to_string(),
            certificate_path: None,
            private_key_path: None,
            trust_server_certs: false,
            namespace_map: HashMap::new(),
        }
    }

    /// One `key=value` session option (`SessionOpen62541.cpp:159-260`).
    pub fn set_option(&mut self, name: &str, value: &str) -> Result<(), String> {
        let number = |v: &str| -> Result<f64, String> {
            v.parse()
                .map_err(|_| format!("invalid value '{v}' for option '{name}'"))
        };
        let count = |v: &str| -> Result<u32, String> {
            v.parse()
                .map_err(|_| format!("invalid value '{v}' for option '{name}'"))
        };
        match name {
            "debug" => self.debug = count(value)?,
            "autoconnect" => {
                self.autoconnect =
                    matches!(value.chars().next(), Some('y' | 'Y' | 't' | 'T' | '1'));
            }
            "nodes-max" | "batch-nodes" => {
                if name == "batch-nodes" {
                    log::warn!(
                        "DEPRECATION WARNING: option 'batch-nodes' is deprecated; use 'nodes-max'."
                    );
                }
                self.nodes_max = count(value)?;
            }
            "read-nodes-max" => self.read_nodes_max = count(value)?,
            "read-timeout-min" => self.read_timeout_min = number(value)?,
            "read-timeout-max" => self.read_timeout_max = number(value)?,
            "write-nodes-max" => self.write_nodes_max = count(value)?,
            "write-timeout-min" => self.write_timeout_min = number(value)?,
            "write-timeout-max" => self.write_timeout_max = number(value)?,
            "sec-mode" => {
                self.security_mode = SecurityMode::parse(value)
                    .ok_or_else(|| format!("invalid security mode '{value}'"))?;
            }
            "sec-policy" => self.security_policy = Some(value.to_string()),
            "sec-id" => self.identity = parse_identity(value)?,
            "pki-dir" => self.pki_dir = value.to_string(),
            "cert" => self.certificate_path = Some(value.to_string()),
            "key" => self.private_key_path = Some(value.to_string()),
            _ => return Err(format!("unknown session option '{name}'")),
        }
        Ok(())
    }

    /// The endpoint to ask the server for.
    pub fn endpoint_description(&self) -> EndpointDescription {
        let policy = self
            .security_policy
            .clone()
            .unwrap_or_else(|| security_policy_uri(self.security_mode).to_string());
        EndpointDescription {
            endpoint_url: self.url.as_str().into(),
            security_policy_uri: policy.as_str().into(),
            security_mode: self.security_mode.as_message_security_mode(),
            ..Default::default()
        }
    }

    /// The nodes the client will put in one Read (`SessionOpen62541.cpp:2461-2465`):
    /// the smaller of the server's limit and the session's, or whichever of the
    /// two is set. Zero means no limit.
    pub fn read_batch_size(&self, server: &ServerInfo) -> usize {
        batch_size(
            server.max_nodes_per_read,
            effective(self.read_nodes_max, self.nodes_max),
        )
    }

    pub fn write_batch_size(&self, server: &ServerInfo) -> usize {
        batch_size(
            server.max_nodes_per_write,
            effective(self.write_nodes_max, self.nodes_max),
        )
    }
}

fn effective(specific: u32, general: u32) -> u32 {
    if specific > 0 { specific } else { general }
}

fn batch_size(server_limit: u32, session_limit: u32) -> usize {
    let max = match (server_limit, session_limit) {
        (0, 0) => 0,
        (s, 0) | (0, s) => s,
        (a, b) => a.min(b),
    };
    let global = defaults::MAX_OPERATIONS_PER_SERVICE_CALL.load(Ordering::Relaxed);
    let global = u32::try_from(global.max(0)).unwrap_or(0);
    match (max, global) {
        (0, 0) => usize::MAX,
        (m, 0) | (0, m) => m as usize,
        (a, b) => a.min(b) as usize,
    }
}

/// `sec-id` names an identity: a user name and password, or an X.509 identity
/// certificate and its key.
fn parse_identity(value: &str) -> Result<Identity, String> {
    match value.split_once(':') {
        Some(("cert", rest)) => match rest.split_once(':') {
            Some((certificate, private_key)) => Ok(Identity::Certificate {
                certificate: certificate.to_string(),
                private_key: private_key.to_string(),
            }),
            None => Err(format!(
                "identity 'cert:{rest}' needs a certificate and a key, 'cert:<cert>:<key>'"
            )),
        },
        Some((user, password)) => Ok(Identity::UserName {
            user: user.to_string(),
            password: password.to_string(),
        }),
        None => Err(format!(
            "identity '{value}' must be '<user>:<password>' or 'cert:<cert>:<key>'"
        )),
    }
}

fn security_policy_uri(mode: SecurityMode) -> &'static str {
    match mode {
        SecurityMode::None => "http://opcfoundation.org/UA/SecurityPolicy#None",
        // The strongest policy the OPC UA specification currently requires a
        // server to support (Part 7 profile "Basic256Sha256").
        _ => "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256",
    }
}

/// What a record asks the session to do (`RequestQueueBatcher`'s three priority
/// queues, one per `menuPriority` value).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    Read { handle: u32 },
    Write { handle: u32 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum Priority {
    #[default]
    Low = 0,
    Medium = 1,
    High = 2,
}

impl Priority {
    /// The record's PRIO field.
    pub fn from_prio(prio: u16) -> Self {
        match prio {
            0 => Priority::Low,
            1 => Priority::Medium,
            _ => Priority::High,
        }
    }
}

/// The handle a record's device support keeps on its session.
#[derive(Debug)]
pub struct SessionHandle {
    pub config: SessionConfig,
    /// Items on this session, indexed by client handle.
    pub items: Mutex<Vec<Arc<Mutex<Item>>>>,
    pub subscriptions: Mutex<Vec<SubscriptionConfig>>,
    pub status: Mutex<ConnectionStatus>,
    commands: mpsc::UnboundedSender<(Priority, Request)>,
    control: mpsc::UnboundedSender<Control>,
}

/// Out-of-band commands from iocsh.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Control {
    Connect,
    Disconnect,
}

impl SessionHandle {
    /// Ask the session to read or write a node on a record's behalf.
    pub fn request(&self, priority: Priority, request: Request) {
        // The worker outlives every record, so a closed channel means the IOC is
        // shutting down.
        let _ = self.commands.send((priority, request));
    }

    pub fn control(&self, control: Control) {
        let _ = self.control.send(control);
    }

    pub fn is_connected(&self) -> bool {
        !matches!(*self.status.lock(), ConnectionStatus::Down)
    }
}

/// Everything the worker owns.
pub struct SessionWorker {
    handle: Arc<SessionHandle>,
    connector: Arc<dyn UaConnector>,
    commands: mpsc::UnboundedReceiver<(Priority, Request)>,
    control: mpsc::UnboundedReceiver<Control>,
    connection: Option<Arc<dyn UaConnection>>,
    server: ServerInfo,
    /// Read and write requests waiting for a service call, highest priority first.
    reads: [VecDeque<u32>; 3],
    writes: [VecDeque<u32>; 3],
}

/// Create a session and its worker. The worker must be spawned.
pub fn create(
    config: SessionConfig,
    connector: Arc<dyn UaConnector>,
) -> (Arc<SessionHandle>, SessionWorker) {
    let (commands_tx, commands) = mpsc::unbounded_channel();
    let (control_tx, control) = mpsc::unbounded_channel();
    let handle = Arc::new(SessionHandle {
        config,
        items: Mutex::new(Vec::new()),
        subscriptions: Mutex::new(Vec::new()),
        status: Mutex::new(ConnectionStatus::Down),
        commands: commands_tx,
        control: control_tx,
    });
    let worker = SessionWorker {
        handle: handle.clone(),
        connector,
        commands,
        control,
        connection: None,
        server: ServerInfo::default(),
        reads: Default::default(),
        writes: Default::default(),
    };
    (handle, worker)
}

impl SessionWorker {
    /// The worker's whole life: connect, serve, reconnect.
    pub async fn run(mut self) {
        let mut connect_now = self.handle.config.autoconnect;
        loop {
            if self.connection.is_none() {
                if !connect_now {
                    match self.control.recv().await {
                        Some(Control::Connect) => connect_now = true,
                        Some(Control::Disconnect) => continue,
                        None => return,
                    }
                    continue;
                }
                if let Err(e) = self.connect().await {
                    log::error!("session {}: {e}", self.handle.config.name);
                    // `opcua_ConnectTimeout` is both the connect timeout and the
                    // interval between attempts (`iocshVariables.h:24`).
                    let retry = defaults::CONNECT_TIMEOUT.get().max(0.1);
                    tokio::time::sleep(Duration::from_secs_f64(retry)).await;
                    connect_now = self.handle.config.autoconnect;
                    continue;
                }
            }

            match self.serve().await {
                ServeOutcome::Shutdown => return,
                ServeOutcome::Disconnected => {
                    connect_now = self.handle.config.autoconnect;
                }
                ServeOutcome::DisconnectRequested => {
                    connect_now = false;
                }
            }
        }
    }

    /// Bring the session up (`SessionOpen62541::connectionStatusChanged`, the
    /// `ACTIVATED` arm at `SessionOpen62541.cpp:2440-2530`): learn the server's
    /// limits and namespaces, rebuild the node ids, register the nodes, read
    /// every item once, run the `bini=write` writes, then start the
    /// subscriptions.
    async fn connect(&mut self) -> Result<(), String> {
        let connection = self.connector.connect(&self.handle.config).await?;
        self.server = connection
            .server_info()
            .await
            .map_err(|e| format!("reading the server's capabilities failed: {e}"))?;
        let type_tree = connection.type_tree().await.ok();
        if type_tree.is_none() {
            log::warn!(
                "session {}: the server's type dictionary could not be read; \
                 enumerations and structures will not resolve",
                self.handle.config.name
            );
        }

        self.rebuild_node_ids();
        self.register_nodes(&connection).await;

        let items = self.handle.items.lock().clone();
        for item in &items {
            let mut item = item.lock();
            item.type_tree = type_tree.clone();
            item.set_state(ConnectionStatus::InitialRead);
        }
        *self.handle.status.lock() = ConnectionStatus::InitialRead;
        self.connection = Some(connection.clone());

        // The initial read goes ahead of everything a record may already have
        // queued, and its result is what `bini` acts on.
        let handles: Vec<u32> = (0..items.len() as u32).collect();
        self.read_batch(&connection, &handles).await;
        self.write_bini(&connection, &items).await;

        for item in &items {
            item.lock().set_state(ConnectionStatus::Up);
        }
        *self.handle.status.lock() = ConnectionStatus::Up;

        self.start_subscriptions(&connection).await;
        log::info!("session {} is up", self.handle.config.name);
        Ok(())
    }

    /// Serve requests until the connection drops or the IOC shuts down.
    async fn serve(&mut self) -> ServeOutcome {
        let Some(connection) = self.connection.clone() else {
            return ServeOutcome::Disconnected;
        };
        loop {
            let flush_at = self.next_flush();
            tokio::select! {
                biased;

                control = self.control.recv() => match control {
                    Some(Control::Disconnect) => {
                        self.disconnect(&connection, ProcessReason::ConnectionLoss).await;
                        return ServeOutcome::DisconnectRequested;
                    }
                    Some(Control::Connect) => {}
                    None => return ServeOutcome::Shutdown,
                },

                request = self.commands.recv() => match request {
                    Some((priority, Request::Read { handle })) => {
                        self.reads[priority as usize].push_back(handle);
                    }
                    Some((priority, Request::Write { handle })) => {
                        self.writes[priority as usize].push_back(handle);
                    }
                    None => return ServeOutcome::Shutdown,
                },

                _ = tokio::time::sleep_until(flush_at.into()), if self.has_pending() => {
                    if !self.flush(&connection).await {
                        self.disconnect(&connection, ProcessReason::ConnectionLoss).await;
                        return ServeOutcome::Disconnected;
                    }
                }
            }
        }
    }

    fn has_pending(&self) -> bool {
        self.reads.iter().any(|q| !q.is_empty()) || self.writes.iter().any(|q| !q.is_empty())
    }

    /// `RequestQueueBatcher`'s hold-off: a batch waits `holdOffFix + holdOffVar *
    /// size` before it is sent, so a burst of record requests coalesces into one
    /// service call (`RequestQueueBatcher.h:96-116`).
    fn next_flush(&self) -> Instant {
        let pending = self
            .reads
            .iter()
            .chain(self.writes.iter())
            .map(VecDeque::len)
            .sum::<usize>();
        let min = self.handle.config.read_timeout_min;
        let max = self.handle.config.read_timeout_max;
        let batch = self.read_limit().min(pending.max(1));
        let hold_off = if max > min && batch > 0 {
            min + (max - min) * (pending as f64 / batch as f64).min(1.0)
        } else {
            min
        };
        Instant::now() + Duration::from_secs_f64(hold_off / 1000.0)
    }

    /// How many *items* one Read may carry.
    ///
    /// Upstream C defect fixed at source: the limits are node counts — the
    /// server's `MaxNodesPerRead` bounds `nodesToRead`, not the number of items —
    /// but the C hands the node limit straight to the batcher as its
    /// items-per-batch limit (`SessionOpen62541.cpp:463-467`) while every item
    /// puts *two* nodes into the request (`SessionOpen62541.cpp:660`,
    /// `no_of_properties_read`). A server that states `MaxNodesPerRead = 100`
    /// therefore gets Reads of 200 nodes and answers `BadTooManyOperations`.
    fn read_limit(&self) -> usize {
        (self.handle.config.read_batch_size(&self.server) / NO_OF_PROPERTIES_READ).max(1)
    }

    /// A write puts one node per item into the request.
    fn write_limit(&self) -> usize {
        self.handle.config.write_batch_size(&self.server).max(1)
    }

    /// Send one batch, highest priority first. Returns false on a connection error.
    async fn flush(&mut self, connection: &Arc<dyn UaConnection>) -> bool {
        let (read_limit, write_limit) = (self.read_limit(), self.write_limit());
        if let Some(handles) = take_batch(&mut self.writes, write_limit) {
            return self.write_batch(connection, &handles).await;
        }
        if let Some(handles) = take_batch(&mut self.reads, read_limit) {
            return self.read_batch(connection, &handles).await;
        }
        true
    }

    /// Read items, in as many service calls as the node limit needs. Every read
    /// goes through here, so no Read can exceed the limit.
    async fn read_batch(&mut self, connection: &Arc<dyn UaConnection>, handles: &[u32]) -> bool {
        for chunk in handles.chunks(self.read_limit()) {
            if !self.read_chunk(connection, chunk).await {
                return false;
            }
        }
        true
    }

    /// One Read. Each item takes two nodes in it: its DataType and its Value
    /// (`SessionOpen62541::processRequests(ReadRequest)`, `no_of_properties_read
    /// = 2`) — the data type is what tells an Int32 that is an enumeration from
    /// one that is not.
    async fn read_chunk(&mut self, connection: &Arc<dyn UaConnection>, handles: &[u32]) -> bool {
        let items = self.handle.items.lock().clone();
        let mut nodes = Vec::with_capacity(handles.len() * 2);
        for handle in handles {
            let Some(item) = items.get(*handle as usize) else {
                continue;
            };
            let node_id = item.lock().wire_node_id().clone();
            nodes.push(ReadValueId {
                node_id: node_id.clone(),
                attribute_id: AttributeId::DataType as u32,
                ..Default::default()
            });
            nodes.push(ReadValueId {
                node_id,
                attribute_id: AttributeId::Value as u32,
                ..Default::default()
            });
        }
        if nodes.is_empty() {
            return true;
        }

        match connection.read(&nodes).await {
            Ok(results) => {
                for (i, handle) in handles.iter().enumerate() {
                    let Some(item) = items.get(*handle as usize) else {
                        continue;
                    };
                    // The C bounds-checks only the DataType result and then
                    // reads the Value result at `i + 1` unchecked
                    // (`SessionOpen62541.cpp:2196-2230`), so a server that
                    // returns fewer results than requested walks off the end of
                    // the array. Both are checked here.
                    let (Some(data_type), Some(value)) =
                        (results.get(i * 2), results.get(i * 2 + 1))
                    else {
                        item.lock().set_incoming_event(
                            ProcessReason::ReadFailure,
                            StatusCode::BadUnexpectedError,
                        );
                        continue;
                    };
                    self.deliver_read(item, data_type, value.clone());
                }
                true
            }
            Err(status) => {
                log::error!("session {}: read failed: {status}", self.handle.config.name);
                for handle in handles {
                    if let Some(item) = items.get(*handle as usize) {
                        item.lock()
                            .set_incoming_event(ProcessReason::ReadFailure, status);
                    }
                }
                !is_connection_error(status)
            }
        }
    }

    fn deliver_read(&self, item: &Arc<Mutex<Item>>, data_type: &DataValue, value: DataValue) {
        let mut item = item.lock();
        if let Some(Variant::NodeId(id)) = &data_type.value {
            item.data_type = Some((**id).clone());
        }
        let status = value.status.unwrap_or(StatusCode::Good);
        if status.is_bad() && value.value.is_none() {
            item.set_incoming_event(ProcessReason::ReadFailure, status);
        } else {
            item.set_incoming_data(value, ProcessReason::ReadComplete);
        }
    }

    /// Write items, in as many service calls as the node limit needs.
    async fn write_batch(&mut self, connection: &Arc<dyn UaConnection>, handles: &[u32]) -> bool {
        for chunk in handles.chunks(self.write_limit()) {
            if !self.write_chunk(connection, chunk).await {
                return false;
            }
        }
        true
    }

    async fn write_chunk(&mut self, connection: &Arc<dyn UaConnection>, handles: &[u32]) -> bool {
        let items = self.handle.items.lock().clone();
        let mut values = Vec::with_capacity(handles.len());
        let mut written = Vec::with_capacity(handles.len());
        for handle in handles {
            let Some(item) = items.get(*handle as usize) else {
                continue;
            };
            let (node_id, outgoing) = {
                let mut item = item.lock();
                (item.wire_node_id().clone(), item.take_outgoing())
            };
            let Some(outgoing) = outgoing else { continue };
            values.push(WriteValue {
                node_id,
                attribute_id: AttributeId::Value as u32,
                value: DataValue::value_only(outgoing),
                ..Default::default()
            });
            written.push(item.clone());
        }
        if values.is_empty() {
            return true;
        }

        match connection.write(&values).await {
            Ok(results) => {
                for (i, item) in written.iter().enumerate() {
                    let status = results
                        .get(i)
                        .copied()
                        .unwrap_or(StatusCode::BadUnexpectedError);
                    let reason = if status.is_good() {
                        ProcessReason::WriteComplete
                    } else {
                        ProcessReason::WriteFailure
                    };
                    item.lock().set_incoming_event(reason, status);
                }
                true
            }
            Err(status) => {
                log::error!(
                    "session {}: write failed: {status}",
                    self.handle.config.name
                );
                for item in &written {
                    item.lock()
                        .set_incoming_event(ProcessReason::WriteFailure, status);
                }
                !is_connection_error(status)
            }
        }
    }

    /// `bini=write`: after the initial read, write the record's own value back
    /// (`ItemOpen62541.cpp:150-190`).
    async fn write_bini(&mut self, connection: &Arc<dyn UaConnection>, items: &[Arc<Mutex<Item>>]) {
        let handles: Vec<u32> = items
            .iter()
            .filter(|item| item.lock().has_dirty_leaf())
            .map(|item| item.lock().client_handle)
            .collect();
        if handles.is_empty() {
            return;
        }
        for item in items {
            let mut item = item.lock();
            if item.has_dirty_leaf() {
                item.set_state(ConnectionStatus::InitialWrite);
            }
        }
        self.write_batch(connection, &handles).await;
    }

    /// Map the configured namespace indices onto the server's, then rebuild every
    /// node id (`SessionOpen62541::updateNamespaceMap` / `rebuildNodeIds`).
    fn rebuild_node_ids(&self) {
        let map = &self.handle.config.namespace_map;
        let mut remap: HashMap<u16, u16> = HashMap::new();
        for (local, uri) in map {
            match self
                .server
                .namespace_array
                .iter()
                .position(|server_uri| server_uri == uri)
            {
                Some(index) => {
                    remap.insert(*local, index as u16);
                }
                None => log::error!(
                    "session {}: the server has no namespace '{uri}'; \
                     items in namespace {local} keep their configured index",
                    self.handle.config.name
                ),
            }
        }

        for item in self.handle.items.lock().iter() {
            let mut item = item.lock();
            let configured = item.configured_node_id.clone();
            let namespace = remap
                .get(&configured.namespace)
                .copied()
                .unwrap_or(configured.namespace);
            item.node_id = NodeId::new(namespace, configured.identifier.clone());
            item.registered_node_id = None;
        }
    }

    /// `RegisterNodes` for the items whose link asked for it
    /// (`SessionOpen62541::registerNodes`).
    ///
    /// The C prints every registered item to stdout unconditionally
    /// (`SessionOpen62541.cpp:1470` — `it->show(0)` outside any debug guard);
    /// this logs one line at debug level instead.
    async fn register_nodes(&self, connection: &Arc<dyn UaConnection>) {
        let items = self.handle.items.lock().clone();
        let to_register: Vec<(usize, NodeId)> = items
            .iter()
            .enumerate()
            .filter_map(|(i, item)| {
                let item = item.lock();
                item.link.register_node.then(|| (i, item.node_id.clone()))
            })
            .collect();
        if to_register.is_empty() {
            return;
        }

        let ids: Vec<NodeId> = to_register.iter().map(|(_, id)| id.clone()).collect();
        match connection.register_nodes(&ids).await {
            Ok(registered) => {
                for ((i, _), id) in to_register.iter().zip(registered) {
                    log::debug!("session {}: registered node {id}", self.handle.config.name);
                    items[*i].lock().registered_node_id = Some(id);
                }
            }
            Err(status) => log::error!(
                "session {}: registering nodes failed: {status}",
                self.handle.config.name
            ),
        }
    }

    /// Create every subscription and its monitored items
    /// (`SubscriptionOpen62541::create` / `addMonitoredItems`).
    async fn start_subscriptions(&mut self, connection: &Arc<dyn UaConnection>) {
        let subscriptions = self.handle.subscriptions.lock().clone();
        let items = self.handle.items.lock().clone();

        for config in &subscriptions {
            let (tx, rx) = mpsc::unbounded_channel();
            let id = match connection.create_subscription(config, tx).await {
                Ok(id) => id,
                Err(status) => {
                    log::error!("subscription {}: create failed: {status}", config.name);
                    continue;
                }
            };

            let monitored: Vec<Arc<Mutex<Item>>> = items
                .iter()
                .filter(|item| {
                    let item = item.lock();
                    item.is_monitored() && item.link.subscription() == Some(config.name.as_str())
                })
                .cloned()
                .collect();
            let requests: Vec<MonitoredItemCreateRequest> = monitored
                .iter()
                .map(|item| monitored_item_request(&item.lock()))
                .collect();
            if !requests.is_empty() {
                match connection.create_monitored_items(id, requests).await {
                    Ok(statuses) => {
                        for (item, status) in monitored.iter().zip(statuses) {
                            if status.is_bad() {
                                let item = item.lock();
                                log::error!(
                                    "subscription {}: monitoring {} failed: {status}",
                                    config.name,
                                    item.node_id
                                );
                            }
                        }
                    }
                    Err(status) => log::error!(
                        "subscription {}: creating monitored items failed: {status}",
                        config.name
                    ),
                }
            }

            // One dispatcher task per subscription: every value the server pushes
            // goes to the item its client handle names.
            let items = items.clone();
            let name = config.name.clone();
            tokio::spawn(async move {
                let mut rx = rx;
                while let Some(notification) = rx.recv().await {
                    match items.get(notification.client_handle as usize) {
                        Some(item) => item
                            .lock()
                            .set_incoming_data(notification.value, ProcessReason::IncomingData),
                        None => log::error!(
                            "subscription {name}: a value arrived for the unknown item handle {}",
                            notification.client_handle
                        ),
                    }
                }
            });
        }
    }

    async fn disconnect(&mut self, connection: &Arc<dyn UaConnection>, reason: ProcessReason) {
        connection.disconnect().await;
        self.connection = None;
        *self.handle.status.lock() = ConnectionStatus::Down;
        // Requests queued for a connection that no longer exists would be
        // answered against the next one; the C leaves them in `outstandingOps`
        // and later logs them as unknown transactions
        // (`SessionOpen62541::markConnectionLoss` clears only the reader and
        // writer queues).
        for queue in self.reads.iter_mut().chain(self.writes.iter_mut()) {
            queue.clear();
        }
        for item in self.handle.items.lock().iter() {
            item.lock()
                .set_incoming_event(reason, StatusCode::BadConnectionClosed);
        }
    }
}

enum ServeOutcome {
    Shutdown,
    Disconnected,
    DisconnectRequested,
}

fn take_batch(queues: &mut [VecDeque<u32>; 3], limit: usize) -> Option<Vec<u32>> {
    for priority in (0..3).rev() {
        let queue = &mut queues[priority];
        if queue.is_empty() {
            continue;
        }
        let take = queue.len().min(limit.max(1));
        return Some(queue.drain(..take).collect());
    }
    None
}

fn monitored_item_request(item: &Item) -> MonitoredItemCreateRequest {
    use async_opcua::types::{
        DataChangeFilter, DataChangeTrigger, DeadbandType, ExtensionObject, MonitoringMode,
        MonitoringParameters,
    };

    let link = &item.link;
    let filter = if link.deadband > 0.0 {
        ExtensionObject::from_message(DataChangeFilter {
            trigger: DataChangeTrigger::StatusValue,
            deadband_type: DeadbandType::Absolute as u32,
            deadband_value: link.deadband,
        })
    } else {
        ExtensionObject::null()
    };

    MonitoredItemCreateRequest {
        item_to_monitor: ReadValueId {
            node_id: item.wire_node_id().clone(),
            attribute_id: AttributeId::Value as u32,
            ..Default::default()
        },
        monitoring_mode: MonitoringMode::Reporting,
        requested_parameters: MonitoringParameters {
            client_handle: item.client_handle,
            sampling_interval: link.sampling_interval,
            filter,
            queue_size: link.queue_size,
            discard_oldest: link.discard_oldest,
        },
    }
}

/// Which failures mean the session is gone rather than the request being bad.
fn is_connection_error(status: StatusCode) -> bool {
    matches!(
        status,
        StatusCode::BadConnectionClosed
            | StatusCode::BadServerNotConnected
            | StatusCode::BadSessionClosed
            | StatusCode::BadSessionIdInvalid
            | StatusCode::BadNotConnected
            | StatusCode::BadSecureChannelClosed
            | StatusCode::BadTimeout
    )
}
