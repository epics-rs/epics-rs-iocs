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

use async_opcua::crypto::SecurityPolicy;
use async_opcua::types::{
    AttributeId, DataValue, EndpointDescription, MessageSecurityMode, MonitoredItemCreateRequest,
    MonitoredItemCreateResult, NodeId, ReadValueId, StatusCode, Variant, WriteValue,
};
use epics_rs::base::runtime::log::errlog_vprintf;
use parking_lot::Mutex;
use tokio::sync::mpsc;

use crate::client::{
    Identity, SecurityMode, ServerInfo, UaConnection, UaConnector, security_policy_string,
};
use crate::defaults;
use crate::item::{Item, stream_double};
use crate::queue::{ConnectionStatus, ProcessReason};
use crate::subscription::SubscriptionConfig;

/// The attributes one Read asks for per item: DataType and Value
/// (`Session.h:359`, `no_of_properties_read`).
const NO_OF_PROPERTIES_READ: usize = 2;

/// The `connectivityCheckInterval` C sets once the session is activated
/// (`SessionOpen62541.cpp:2427`).
const CONNECTIVITY_CHECK_INTERVAL: Duration = Duration::from_secs(1);

/// `Server_ServerStatus_State` (ns=0;i=2259), what open62541's
/// connectivity check reads.
const SERVER_STATUS_STATE: u32 = 2259;

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
    /// `sec-id`: the credentials file [`Self::setup_identity`] reads at every
    /// connect.
    pub identity_file: Option<String>,
    /// `None` — the client's PKI store (`opcuaSetupPKI`), or the default.
    pub pki_dir: Option<String>,
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
            identity_file: None,
            pki_dir: None,
            certificate_path: None,
            private_key_path: None,
            trust_server_certs: false,
            namespace_map: HashMap::new(),
        }
    }

    /// The identity token the next connect presents
    /// (`SessionOpen62541::setupIdentity`): what the `sec-id` credentials file
    /// holds, or anonymous when there is none or it is unusable.
    pub fn setup_identity(&self) -> Identity {
        let Some(file) = &self.identity_file else {
            if self.debug > 0 {
                println!(
                    "Session {}: (setupIdentity) setting Anonymous token",
                    self.name
                );
            }
            return Identity::Anonymous;
        };
        // C goes on parsing the stream that failed to open, which reads nothing.
        let text = std::fs::read_to_string(file).unwrap_or_else(|_| {
            errlog_vprintf(format_args!(
                "OPC UA session {}: cannot open credentials file {file}\n",
                self.name
            ));
            String::new()
        });
        let (mut user, mut pass, mut cert, mut key) = ("", "", "", "");
        for line in text.lines() {
            let Some(equ) = line.find('=') else { continue };
            if line.find('#').is_some_and(|hash| hash < equ) {
                continue;
            }
            let value = &line[equ + 1..];
            match &line[..equ] {
                "user" => user = value,
                "pass" => pass = value,
                "cert" => cert = value,
                "key" => key = value,
                _ => {}
            }
        }

        if !user.is_empty() && !pass.is_empty() {
            if self.debug > 0 {
                println!(
                    "Session {}: (setupIdentity) setting Username token ({user}/*****)",
                    self.name
                );
            }
            Identity::UserName {
                user: user.to_string(),
                password: pass.to_string(),
            }
        } else if !cert.is_empty() && !key.is_empty() {
            if self.debug > 0 {
                println!(
                    "Session {}: (setupIdentity) loading identity certificate {cert}",
                    self.name
                );
            }
            // `loadFile` reads the whole file; an unreadable one is the same
            // failure as a missing one.
            match std::fs::read(cert) {
                Ok(bytes) if !bytes.is_empty() => Identity::Certificate {
                    certificate: cert.to_string(),
                    private_key: key.to_string(),
                },
                result => {
                    let reason = match result {
                        Err(e) => strerror(&e),
                        // `loadFile` of an empty file leaves errno at 0.
                        Ok(_) => "Success".to_string(),
                    };
                    errlog_vprintf(format_args!(
                        "OPC UA Session {}: loading client certificate {cert} failed: {reason}\n",
                        self.name
                    ));
                    Identity::Anonymous
                }
            }
        } else {
            errlog_vprintf(format_args!(
                "OPC UA session {}: credentials file {file} does not contain settings for \
                 Username token (user + pass) or Certificate token (cert + key [+ pass])\n",
                self.name
            ));
            Identity::Anonymous
        }
    }

    /// One `key=value` session option (`SessionOpen62541::setOption`). As in
    /// the C a bad name or value is reported and passed over; `Err` is only
    /// the C's exception (an illegal `autoconnect`), which ends the option list.
    pub fn set_option(&mut self, name: &str, value: &str) -> Result<(), String> {
        if self.debug > 0 || name == "debug" {
            eprintln!("Session {}: setting option {name} to {value}", self.name);
        }
        // C reads every count with strtoul and stores it narrowed.
        let count = || strtoul(value) as u32;
        match name {
            "sec-mode" => match SecurityMode::parse(value) {
                Some(mode) => self.security_mode = mode,
                None => errlog_vprintf(format_args!(
                    "invalid security-mode (valid: best None Sign SignAndEncrypt)\n"
                )),
            },
            "sec-policy" => {
                // C matches the policy's short name and keeps its URI.
                let supported = supported_security_policies();
                match supported.iter().find(|(_, name)| *name == value) {
                    Some((uri, _)) => self.security_policy = Some(uri.to_string()),
                    None => {
                        let valid: String = supported
                            .iter()
                            .map(|(_, name)| format!(" {name}"))
                            .collect();
                        errlog_vprintf(format_args!("invalid security policy (valid:{valid})\n"));
                    }
                }
            }
            // An empty name is C's empty `securityIdentityFile`: anonymous.
            "sec-id" => self.identity_file = (!value.is_empty()).then(|| value.to_string()),
            "debug" => self.debug = count(),
            "nodes-max" | "batch-nodes" => {
                if name == "batch-nodes" {
                    errlog_vprintf(format_args!(
                        "DEPRECATED: option 'batch-nodes'; use 'nodes-max' instead\n"
                    ));
                }
                self.nodes_max = count();
            }
            "read-nodes-max" => self.read_nodes_max = count(),
            "read-timeout-min" => self.read_timeout_min = count() as f64,
            "read-timeout-max" => self.read_timeout_max = count() as f64,
            "write-nodes-max" => self.write_nodes_max = count(),
            "write-timeout-min" => self.write_timeout_min = count() as f64,
            "write-timeout-max" => self.write_timeout_max = count() as f64,
            "autoconnect" => {
                if let Some(c) = value.chars().next() {
                    self.autoconnect = get_yes_no(c)?;
                }
            }
            // Not C session options: the C sets these through opcuaSetupPKI
            // and opcuaClientCertificate.
            "pki-dir" => self.pki_dir = Some(value.to_string()),
            "cert" => self.certificate_path = Some(value.to_string()),
            "key" => self.private_key_path = Some(value.to_string()),
            _ => errlog_vprintf(format_args!("unknown option '{name}' - ignored\n")),
        }
        Ok(())
    }

    /// `SessionOpen62541::setupSecurity` (`SessionOpen62541.cpp:1060-1086`):
    /// of the server's opc.tcp endpoints whose mode and policy the session
    /// accepts and whose policy the client supports, the one with the highest
    /// security level. The session still connects to its own URL.
    pub fn select_endpoint(
        &self,
        endpoints: &[EndpointDescription],
    ) -> Option<EndpointDescription> {
        let mode = self.security_mode.as_message_security_mode();
        let mut selected: Option<&EndpointDescription> = None;
        for e in endpoints {
            let policy = e.security_policy_uri.as_ref();
            if e.endpoint_url.as_ref().starts_with("opc.tcp")
                && (self.security_mode == SecurityMode::Best || e.security_mode == mode)
                && self
                    .security_policy
                    .as_deref()
                    .is_none_or(|p| p.contains("#None") || p == policy)
                && selected.is_none_or(|s| e.security_level > s.security_level)
                && SecurityPolicy::from_uri(policy) != SecurityPolicy::Unknown
            {
                selected = Some(e);
            }
        }
        selected.map(|e| EndpointDescription {
            endpoint_url: self.url.as_str().into(),
            ..e.clone()
        })
    }

    /// The endpoint a `sec-mode=None` session connects to without discovery.
    pub fn unsecured_endpoint(&self) -> EndpointDescription {
        EndpointDescription {
            endpoint_url: self.url.as_str().into(),
            security_policy_uri: SecurityPolicy::None.to_uri().into(),
            security_mode: MessageSecurityMode::None,
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
///
/// The configuration stays mutable: `opcuaOptions` and `opcuaMapNamespace` may
/// name a session at any time before it connects, and the C applies them to the
/// live session object (`SessionOpen62541::setOption`). The worker reads it, the
/// iocsh commands write it, and neither holds the lock across an await.
#[derive(Debug)]
pub struct SessionHandle {
    pub name: String,
    pub config: Mutex<SessionConfig>,
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

    /// Apply one `key=value` option (`opcuaOptions`).
    pub fn set_option(&self, name: &str, value: &str) -> Result<(), String> {
        self.config.lock().set_option(name, value)
    }

    /// `opcuaMapNamespace`.
    pub fn add_namespace_mapping(&self, index: u16, uri: &str) {
        self.config
            .lock()
            .namespace_map
            .insert(index, uri.to_string());
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
        name: config.name.clone(),
        config: Mutex::new(config),
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
        let mut connect_now = self.autoconnect();
        // `opcuaConnect` asked for this attempt (C `connect(manual)`).
        let mut manual = false;
        loop {
            if self.connection.is_none() {
                if !connect_now {
                    match self.control.recv().await {
                        Some(Control::Connect) => {
                            connect_now = true;
                            manual = true;
                        }
                        Some(Control::Disconnect) => continue,
                        None => return,
                    }
                    continue;
                }
                let attempt = self.connect().await;
                let was_manual = std::mem::take(&mut manual);
                if let Err(e) = attempt {
                    if was_manual || self.debug() > 0 {
                        errlog_vprintf(format_args!("OPC UA session {}: {e}\n", self.handle.name));
                    }
                    // `opcua_ConnectTimeout` is both the connect timeout and the
                    // interval between attempts (`iocshVariables.h:24`); an
                    // `opcuaConnect` meanwhile is C's immediate `connect(true)`.
                    let retry = defaults::CONNECT_TIMEOUT.get().max(0.1);
                    let deadline = tokio::time::Instant::now() + Duration::from_secs_f64(retry);
                    loop {
                        tokio::select! {
                            _ = tokio::time::sleep_until(deadline) => {
                                connect_now = self.autoconnect();
                                break;
                            }
                            control = self.control.recv() => match control {
                                Some(Control::Connect) => {
                                    connect_now = true;
                                    manual = true;
                                    break;
                                }
                                Some(Control::Disconnect) => {}
                                None => return,
                            },
                        }
                    }
                    continue;
                }
            }

            match self.serve().await {
                ServeOutcome::Shutdown => return,
                ServeOutcome::Disconnected => {
                    connect_now = self.autoconnect();
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
    fn autoconnect(&self) -> bool {
        self.handle.config.lock().autoconnect
    }

    /// The `ACTIVATED` arm's first lines (`SessionOpen62541.cpp:2432-2445`).
    fn report_connected(&self, identity: &Identity, endpoint: &EndpointDescription) {
        let (user, token) = match identity {
            Identity::Anonymous => ("Anonymous", ""),
            Identity::UserName { user, .. } => (user.as_str(), " (username token)"),
            Identity::Certificate { .. } => ("Certificate user", " (certificate token)"),
        };
        errlog_vprintf(format_args!(
            "OPC UA session {}: connected as '{user}'{token} with security level {} (mode={:?}; policy={})\n",
            self.handle.name,
            endpoint.security_level,
            endpoint.security_mode,
            security_policy_string(endpoint.security_policy_uri.as_ref()),
        ));
        if endpoint.security_mode == MessageSecurityMode::None {
            errlog_vprintf(format_args!(
                "OPC UA session {}: WARNING - this session uses *** NO SECURITY ***\n",
                self.handle.name
            ));
        }
    }

    fn debug(&self) -> u32 {
        self.handle.config.lock().debug
    }

    async fn connect(&mut self) -> Result<(), String> {
        let config = self.handle.config.lock().clone();
        let identity = config.setup_identity();
        let connection = self.connector.connect(&config, &identity).await?;
        if config.debug > 0 {
            eprintln!("Session {}: connect service succeeded", self.handle.name);
        }
        if let Some(endpoint) = connection.endpoint() {
            self.report_connected(&identity, &endpoint);
        }
        // C reads each capability on its own and keeps its default when the
        // read fails, with no print (`SessionOpen62541.cpp:2447-2484`).
        self.server = connection.server_info().await.unwrap_or_default();
        let type_tree = connection.type_tree().await.ok();

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
        // queued, and its result is what `bini` acts on: a `bini=write` leaf gets
        // a write request queued for it by the item, which the record then pops
        // and answers by writing its own value back.
        let handles: Vec<u32> = (0..items.len() as u32).collect();
        self.read_batch(&connection, &handles).await;

        for item in &items {
            item.lock().set_state(ConnectionStatus::Up);
        }
        *self.handle.status.lock() = ConnectionStatus::Up;

        self.start_subscriptions(&connection).await;
        Ok(())
    }

    /// Serve requests until the connection drops or the IOC shuts down.
    async fn serve(&mut self) -> ServeOutcome {
        let Some(connection) = self.connection.clone() else {
            return ServeOutcome::Disconnected;
        };
        // open62541's background connectivity check: one Read of the server
        // state at a time, the next one an interval after the last answer.
        let mut check = None;
        let mut next_check = Instant::now() + CONNECTIVITY_CHECK_INTERVAL;
        loop {
            let flush_at = self.next_flush();
            tokio::select! {
                biased;

                control = self.control.recv() => match control {
                    Some(Control::Disconnect) => {
                        self.disconnect(&connection, ProcessReason::ConnectionLoss).await;
                        return ServeOutcome::DisconnectRequested;
                    }
                    Some(Control::Connect) => {
                        eprintln!("Session {} already connected (Activated)", self.handle.name);
                    }
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

                _ = tokio::time::sleep_until(next_check.into()), if check.is_none() => {
                    let connection = connection.clone();
                    check = Some(Box::pin(async move {
                        let node = ReadValueId {
                            node_id: NodeId::new(0, SERVER_STATUS_STATE),
                            attribute_id: AttributeId::Value as u32,
                            ..Default::default()
                        };
                        connection.read(&[node]).await.1
                    }));
                }

                result = async { check.as_mut().unwrap().await }, if check.is_some() => {
                    check = None;
                    next_check = Instant::now() + CONNECTIVITY_CHECK_INTERVAL;
                    // Only a timed-out check is inactivity
                    // (`SessionOpen62541::connectionInactive`).
                    if result.err() == Some(StatusCode::BadTimeout) {
                        errlog_vprintf(format_args!(
                            "OPC UA Session {}: server inactive\n",
                            self.handle.name
                        ));
                        self.disconnect(&connection, ProcessReason::ConnectionLoss).await;
                        return ServeOutcome::Disconnected;
                    }
                }

                _ = tokio::time::sleep_until(flush_at.into()), if self.has_pending() => {
                    if !self.flush(&connection).await {
                        // The ACTIVATED -> CREATED arm (`SessionOpen62541.cpp:2520-2523`).
                        errlog_vprintf(format_args!(
                            "OPC UA session {}: disconnected\n",
                            self.handle.name
                        ));
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
        let (min, max) = {
            let config = self.handle.config.lock();
            (config.read_timeout_min, config.read_timeout_max)
        };
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
        (self.handle.config.lock().read_batch_size(&self.server) / NO_OF_PROPERTIES_READ).max(1)
    }

    /// A write puts one node per item into the request.
    fn write_limit(&self) -> usize {
        self.handle
            .config
            .lock()
            .write_batch_size(&self.server)
            .max(1)
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

        let (id, result) = connection.read(&nodes).await;
        let debug = self.debug();
        if debug >= 5
            && !result
                .as_ref()
                .is_err_and(|status| is_connection_error(*status))
        {
            println!(
                "Session {}: (requestRead) beginRead service ok (transaction id {id}; retrieving {} nodes)",
                self.handle.name,
                nodes.len() / 2
            );
        }
        match result {
            Ok(results) => {
                if debug >= 2 {
                    println!(
                        "Session {}: (readComplete) getting data for read service (transaction id {id}; data for {} items)",
                        self.handle.name,
                        results.len()
                    );
                }
                if results.len() != nodes.len() {
                    errlog_vprintf(format_args!(
                        "OPC UA session {}: (readComplete) received a callback with {} values for a request containing {} items\n",
                        self.handle.name,
                        results.len(),
                        nodes.len() / 2
                    ));
                }
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
                if is_connection_error(status) {
                    errlog_vprintf(format_args!(
                        "OPC UA session {}: (requestRead) beginRead service failed with status {}\n",
                        self.handle.name,
                        status_name(status)
                    ));
                } else if debug > 0 {
                    println!(
                        "Session {}: (readComplete) for read service (transaction id {id}) failed with status {}",
                        self.handle.name,
                        status_name(status)
                    );
                }
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
            item.resolve_choices();
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

        let (id, result) = connection.write(&values).await;
        let debug = self.debug();
        if debug >= 5
            && !result
                .as_ref()
                .is_err_and(|status| is_connection_error(*status))
        {
            println!(
                "Session {}: (requestWrite) beginWrite service ok (transaction id {id}; writing {} nodes)",
                self.handle.name,
                values.len()
            );
        }
        match result {
            Ok(results) => {
                if debug >= 2 {
                    println!(
                        "Session {}: (writeComplete) getting results for write service (transaction id {id}; results for {} items)",
                        self.handle.name,
                        results.len()
                    );
                }
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
                if is_connection_error(status) {
                    errlog_vprintf(format_args!(
                        "OPC UA session {}: (requestWrite) beginWrite service failed with status {}\n",
                        self.handle.name,
                        status_name(status)
                    ));
                } else if debug > 0 {
                    println!(
                        "Session {}: (writeComplete) for write service (transaction id {id}) failed with status {}",
                        self.handle.name,
                        status_name(status)
                    );
                }
                for item in &written {
                    item.lock()
                        .set_incoming_event(ProcessReason::WriteFailure, status);
                }
                !is_connection_error(status)
            }
        }
    }

    /// Map the configured namespace indices onto the server's, then rebuild every
    /// node id (`SessionOpen62541::updateNamespaceMap` / `rebuildNodeIds`).
    fn rebuild_node_ids(&self) {
        let map = self.handle.config.lock().namespace_map.clone();
        let mut remap: HashMap<u16, u16> = HashMap::new();
        // An unread namespace array is no mapping to report on, as in C.
        let read = !self.server.namespace_array.is_empty();
        for (local, uri) in map.iter().filter(|_| read) {
            match self
                .server
                .namespace_array
                .iter()
                .position(|server_uri| server_uri == uri)
            {
                Some(index) => {
                    remap.insert(*local, index as u16);
                }
                None => errlog_vprintf(format_args!(
                    "OPC UA session {}: locally mapped namespace '{uri}' not found on server\n",
                    self.handle.name
                )),
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
    /// (`SessionOpen62541::registerNodes`), each shown as C does, outside any
    /// debug guard.
    async fn register_nodes(&self, connection: &Arc<dyn UaConnection>) {
        let items = self.handle.items.lock().clone();
        let to_register: Vec<(usize, NodeId)> = items
            .iter()
            .enumerate()
            .filter_map(|(i, item)| {
                let item = item.lock();
                item.link.register_node.then(|| {
                    println!("{}", item.show_line());
                    (i, item.node_id.clone())
                })
            })
            .collect();
        if to_register.is_empty() {
            return;
        }

        let ids: Vec<NodeId> = to_register.iter().map(|(_, id)| id.clone()).collect();
        match connection.register_nodes(&ids).await {
            Ok(registered) => {
                if self.debug() > 0 {
                    println!(
                        "Session {}: (registerNodes) registerNodes service ok ({} nodes registered)",
                        self.handle.name,
                        registered.len()
                    );
                }
                for ((i, _), id) in to_register.iter().zip(registered) {
                    items[*i].lock().registered_node_id = Some(id);
                }
            }
            Err(status) => errlog_vprintf(format_args!(
                "OPC UA session {}: (registerNodes) registerNodes service failed with status {}\n",
                self.handle.name,
                status_name(status)
            )),
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
                Ok(id) => {
                    if config.debug > 0 {
                        errlog_vprintf(format_args!(
                            "OPC UA subscription {} on session {} created (Good)\n",
                            config.name, self.handle.name
                        ));
                    }
                    Some(id)
                }
                Err(status) => {
                    errlog_vprintf(format_args!(
                        "OPC UA subscription {}: createSubscription on session {} failed ({})\n",
                        config.name,
                        self.handle.name,
                        status_name(status)
                    ));
                    None
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
                // C creates the items one call each, so a failed batch is every
                // item failing with its status.
                // C goes on to add the items of a subscription the server refused;
                // open62541 then fails each on the unknown subscription id.
                let refused = |status| {
                    vec![
                        MonitoredItemCreateResult {
                            status_code: status,
                            ..Default::default()
                        };
                        monitored.len()
                    ]
                };
                let results = match id {
                    Some(id) => connection
                        .create_monitored_items(id, requests)
                        .await
                        .unwrap_or_else(refused),
                    None => refused(StatusCode::BadSubscriptionIdInvalid),
                };
                for (item, result) in monitored.iter().zip(&results) {
                    let mut item = item.lock();
                    let status = result.status_code;
                    if status.is_good() {
                        item.revised_sampling_interval = result.revised_sampling_interval;
                        item.revised_queue_size = result.revised_queue_size;
                        if config.debug >= 5 {
                            println!(
                                "** OPC UA record {} monitored item {} succeeded with id {} \
                                 revised sampling interval {} revised queue size {}",
                                item.addressing_record(),
                                item.wire_node_id(),
                                result.monitored_item_id,
                                stream_double(result.revised_sampling_interval),
                                result.revised_queue_size
                            );
                        }
                    } else {
                        eprintln!(
                            "OPC UA record {} monitored item {} failed with error {}",
                            item.addressing_record(),
                            item.wire_node_id(),
                            status_name(status)
                        );
                        item.set_incoming_event(ProcessReason::ConnectionLoss, status);
                    }
                }
                if config.debug > 0 {
                    let last = results
                        .last()
                        .map_or(StatusCode::Good, |result| result.status_code);
                    println!(
                        "Subscription {}@{}: created {} monitored items ({})",
                        config.name,
                        self.handle.name,
                        monitored.len(),
                        status_name(last)
                    );
                }
            }

            if id.is_none() {
                continue;
            }
            // One dispatcher task per subscription: every value the server pushes
            // goes to the item its client handle names.
            let items = items.clone();
            tokio::spawn(async move {
                let mut rx = rx;
                while let Some(notification) = rx.recv().await {
                    if let Some(item) = items.get(notification.client_handle as usize) {
                        item.lock()
                            .set_incoming_data(notification.value, ProcessReason::IncomingData);
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
/// C `UA_StatusCode_name`: the code's name without the info bits.
pub(crate) fn status_name(status: StatusCode) -> &'static str {
    status.sub_code().name()
}

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

/// C `strerror(errno)`: the message without Rust's `(os error N)` suffix.
fn strerror(e: &std::io::Error) -> String {
    let text = e.to_string();
    match text.find(" (os error") {
        Some(end) => text[..end].to_string(),
        None => text,
    }
}

/// C `strtoul(s, nullptr, 0)`: leading whitespace, an optional sign, then the
/// longest prefix in base 16 (`0x`), 8 (`0`) or 10; nothing parsed is 0 and a
/// minus sign wraps.
pub(crate) fn strtoul(s: &str) -> u64 {
    let s = s.trim_start();
    let (negative, s) = match s.as_bytes().first() {
        Some(b'-') => (true, &s[1..]),
        Some(b'+') => (false, &s[1..]),
        _ => (false, s),
    };
    let (radix, digits) = if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X"))
        && hex.starts_with(|c: char| c.is_ascii_hexdigit())
    {
        (16, hex)
    } else if s.starts_with('0') {
        (8, s)
    } else {
        (10, s)
    };
    let mut value: u64 = 0;
    for c in digits.chars() {
        let Some(d) = c.to_digit(radix) else { break };
        value = value.saturating_mul(radix as u64).saturating_add(d as u64);
    }
    if negative {
        value.wrapping_neg()
    } else {
        value
    }
}

/// C `securitySupportedPolicies`: (URI, short name) of each policy the
/// client can use, in the client library's order.
fn supported_security_policies() -> Vec<(&'static str, &'static str)> {
    use async_opcua::crypto::SecurityPolicy;
    [
        SecurityPolicy::None,
        SecurityPolicy::Basic128Rsa15,
        SecurityPolicy::Basic256,
        SecurityPolicy::Basic256Sha256,
        SecurityPolicy::Aes128Sha256RsaOaep,
        SecurityPolicy::Aes256Sha256RsaPss,
    ]
    .into_iter()
    .map(|policy| {
        let uri = policy.to_uri();
        (uri, uri.rsplit_once('#').map_or(uri, |(_, name)| name))
    })
    .collect()
}

/// C `getYesNo` (`linkParser.cpp:44`).
pub(crate) fn get_yes_no(c: char) -> Result<bool, String> {
    if "YyTt1".contains(c) {
        Ok(true)
    } else if "NnFf0".contains(c) {
        Ok(false)
    } else {
        Err(format!("illegal value '{c}'"))
    }
}
