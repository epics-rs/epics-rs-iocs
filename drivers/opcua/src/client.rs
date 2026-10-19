//! The OPC UA client boundary.
//!
//! Everything above this file — items, elements, the record device support —
//! talks to the server through [`UaConnection`], so it can be driven by a test
//! double as well as by a real server. [`AsyncOpcuaConnector`] is the real one,
//! built on the `async-opcua` client.
//!
//! This is the seam where the C module's two client backends
//! (`devOpcuaSup/UaSdk/`, `devOpcuaSup/open62541/`) are replaced by one pure-Rust
//! client. The open62541 backend is the behavioural reference.

use std::fmt;
use std::sync::Arc;
use std::time::Duration;

use async_opcua::client::custom_types::DataTypeTreeBuilder;
use async_opcua::client::services::{CreateSubscription, Read, Write};
use async_opcua::client::{
    Client, ClientBuilder, IdentityToken, Session, SessionPollResult, Subscription,
    SubscriptionCallbacks, UARequest,
};
use async_opcua::core::ResponseMessage;
use async_opcua::crypto::SecurityPolicy;
use async_opcua::types::{
    DataValue, EndpointDescription, MessageSecurityMode, MonitoredItemCreateRequest,
    MonitoredItemCreateResult, NodeId, ReadValueId, StatusCode, TimestampsToReturn, UserTokenType,
    Variant, WriteValue, custom::DataTypeTree,
};
use async_trait::async_trait;
use epics_rs::base::runtime::log::errlog_vprintf;
use futures_util::TryStreamExt;
use parking_lot::Mutex;
use tokio::sync::{mpsc, watch};

use crate::session::{SessionConfig, status_name};
use crate::subscription::SubscriptionConfig;

/// Where the PKI file store lives when neither the client nor the session says.
pub const DEFAULT_PKI_DIR: &str = "pki";

/// `Server_NamespaceArray` (OPC UA Part 5, well-known node).
const NAMESPACE_ARRAY: u32 = 2255;
/// `Server_ServerCapabilities_OperationLimits_MaxNodesPerRead`.
const MAX_NODES_PER_READ: u32 = 11705;
/// `Server_ServerCapabilities_OperationLimits_MaxNodesPerWrite`.
const MAX_NODES_PER_WRITE: u32 = 11707;

/// What the client learns from the server once the session is up
/// (`SessionOpen62541::connectionStatusChanged`, `SessionOpen62541.cpp:2450-2490`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ServerInfo {
    /// The server's namespace URIs, indexed by namespace index.
    pub namespace_array: Vec<String>,
    /// 0 means the server states no limit.
    pub max_nodes_per_read: u32,
    pub max_nodes_per_write: u32,
}

/// One monitored item's value, as it arrives from the server.
#[derive(Debug, Clone)]
pub struct Notification {
    /// The handle the item was created with — the client's item index.
    pub client_handle: u32,
    pub value: DataValue,
}

/// The server's answer to `CreateSubscription`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CreatedSubscription {
    pub id: u32,
    pub revised_publishing_interval: f64,
}

/// A live session with a server.
#[async_trait]
pub trait UaConnection: Send + Sync {
    /// The result comes with the request's transaction id, which C's debug
    /// lines print (`requestId`, `SessionOpen62541.cpp:686-701`).
    async fn read(&self, nodes: &[ReadValueId]) -> (u32, Result<Vec<DataValue>, StatusCode>);
    /// As [`Self::read`].
    async fn write(&self, values: &[WriteValue]) -> (u32, Result<Vec<StatusCode>, StatusCode>);
    /// `RegisterNodes` — the server may return a faster handle for each node.
    async fn register_nodes(&self, nodes: &[NodeId]) -> Result<Vec<NodeId>, StatusCode>;
    async fn server_info(&self) -> Result<ServerInfo, StatusCode>;
    /// The server's data type dictionary, which supplies enumeration choices and
    /// the structure definitions the element tree walks.
    async fn type_tree(&self) -> Result<Arc<DataTypeTree>, StatusCode>;
    /// Create a subscription; every value it delivers goes to `sink`.
    async fn create_subscription(
        &self,
        config: &SubscriptionConfig,
        sink: mpsc::UnboundedSender<Notification>,
    ) -> Result<CreatedSubscription, StatusCode>;
    /// Returns one status per requested item, in order.
    async fn create_monitored_items(
        &self,
        subscription_id: u32,
        items: Vec<MonitoredItemCreateRequest>,
    ) -> Result<Vec<MonitoredItemCreateResult>, StatusCode>;
    async fn disconnect(&self);
    /// Resolves when the connection is gone from under the session — open62541's
    /// `closeSecureChannel` in `UA_Client_run_iterate`.
    async fn closed(&self) {
        std::future::pending::<()>().await
    }
    /// The endpoint the session runs on, as [`SessionConfig::connect_endpoint`]
    /// gave it.
    fn endpoint(&self) -> Option<EndpointDescription> {
        None
    }
}

/// A failed security setup: its console text, and the `connectStatus` the C's
/// attempt leaves (`SessionOpen62541.cpp:1049`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetupError {
    /// `None` when the setup failed before any service call, which leaves
    /// the status as it was.
    pub status: Option<StatusCode>,
    pub message: String,
}

impl SetupError {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            status: None,
            message: message.into(),
        }
    }

    fn service(status: StatusCode, message: impl Into<String>) -> Self {
        Self {
            status: Some(status),
            message: message.into(),
        }
    }
}

/// A failed connection as open62541 1.2's client network layer reports it:
/// any socket failure is BadDisconnect (`UA_ClientConnectionTCP_poll`), an
/// address that does not resolve BadConnectionClosed (`initConnect`).
fn network_status(status: StatusCode) -> StatusCode {
    match status {
        StatusCode::BadCommunicationError => StatusCode::BadDisconnect,
        StatusCode::BadTcpEndpointUrlInvalid => StatusCode::BadConnectionClosed,
        status => status,
    }
}

impl fmt::Display for SetupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

/// Opens sessions. The driver holds one; a test holds a double.
#[async_trait]
pub trait UaConnector: Send + Sync {
    /// `SessionOpen62541::connect` up to `UA_Client_connect`
    /// (`SessionOpen62541.cpp:478-574`): a client with the security the
    /// session asks for and the identity it presents.
    async fn setup(&self, config: &SessionConfig) -> Result<Box<dyn UaClient>, SetupError>;
}

/// The open62541 client one `SessionOpen62541::connect` sets up.
#[async_trait]
pub trait UaClient: Send + Sync {
    /// What [`SessionConfig::setup_identity`] gave the setup.
    fn identity(&self) -> &Identity;
    /// `UA_Client_connect`. Again after a lost connection, it is open62541's
    /// reconnect on the same client (`connectIterate`, open62541.c:45447-45560):
    /// no setup, and no GetEndpoints once the first connect picked the endpoint.
    async fn connect(&mut self) -> Result<Arc<dyn UaConnection>, StatusCode>;
}

/// The `async-opcua` client.
///
/// Accepted-and-documented: the C's `opcuaSaveRejected` command writes rejected
/// server certificates to a directory. `async-opcua` owns its own PKI
/// (`pki_dir`, with `rejected/` and `trusted/` beneath it), so the
/// rejected-certificate directory is fixed by the PKI layout instead of being
/// separately settable.
pub struct AsyncOpcuaConnector {
    application_name: String,
    application_uri: String,
    security: Mutex<ClientSecurity>,
}

/// The security the *client* is configured with, which every session shares —
/// the C's `Session::setClientCertificate` (`opcuaClientCertificate`) and
/// `Session::setupPKI` (`opcuaSetupPKI`) are static, one client per IOC. A
/// session's own `cert`, `key` and `pki-dir` options override it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ClientSecurity {
    pub certificate_path: Option<String>,
    pub private_key_path: Option<String>,
    /// The root of the PKI file store. `async-opcua` lays it out itself —
    /// `trusted/`, `rejected/`, `issuers/` beneath the root.
    pub pki_root: Option<String>,
}

impl AsyncOpcuaConnector {
    pub fn new() -> Self {
        Self {
            application_name: "EPICS IOC".to_string(),
            application_uri: "urn:EPICS:IOC".to_string(),
            security: Mutex::new(ClientSecurity::default()),
        }
    }

    /// `opcuaClientCertificate(cert, key)`.
    pub fn set_client_certificate(&self, certificate: &str, private_key: &str) {
        let mut security = self.security.lock();
        security.certificate_path = Some(certificate.to_string());
        security.private_key_path = Some(private_key.to_string());
    }

    /// `opcuaSetupPKI(root)`.
    ///
    /// Accepted-and-documented: the C's four-argument form names the four
    /// locations (trusted server certs, server CRLs, issuer certs, issuer CRLs)
    /// separately. `async-opcua` derives all four from the root with a layout of
    /// its own, so only the C's one-argument form — "a standard directory
    /// structure under the specified location" — has an equivalent here.
    pub fn set_pki_root(&self, root: &str) {
        self.security.lock().pki_root = Some(root.to_string());
    }

    pub fn security(&self) -> ClientSecurity {
        self.security.lock().clone()
    }
}

impl Default for AsyncOpcuaConnector {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl UaConnector for AsyncOpcuaConnector {
    async fn setup(&self, config: &SessionConfig) -> Result<Box<dyn UaClient>, SetupError> {
        // The session's own options win over the client-wide ones; what neither
        // sets falls back to the `async-opcua` default PKI location.
        let client = self.security();
        // `opcuaSetupPKI` names the PKI locations (`SessionOpen62541.cpp:516-531`).
        let client_pki_root = client.pki_root.is_some();
        let pki_dir = config
            .pki_dir
            .clone()
            .or(client.pki_root)
            .unwrap_or_else(|| DEFAULT_PKI_DIR.to_string());
        let certificate = config.certificate_path.clone().or(client.certificate_path);
        let private_key = config.private_key_path.clone().or(client.private_key_path);
        let name = &config.name;
        let debug = config.debug > 0;

        // `setupClientSecurityInfo` (`SessionOpen62541.cpp:1167-1226`).
        if debug {
            match (&certificate, &private_key) {
                (Some(certificate), Some(private_key)) => {
                    println!(
                        "Session {name}: (initClientSecurity) loading client certificate {certificate}"
                    );
                    println!(
                        "Session {name}: (initClientSecurity) loading client private key {private_key}"
                    );
                }
                _ => println!(
                    "Session {name}: (setupClientSecurityInfo) no client certificate configured"
                ),
            }
        }

        let mut builder = ClientBuilder::new()
            .application_name(&self.application_name)
            .application_uri(&self.application_uri)
            .product_uri("urn:epics-rs:opcua")
            .session_retry_limit(0)
            .session_name(&config.name)
            .pki_dir(&pki_dir)
            .trust_server_certs(config.trust_server_certs)
            .create_sample_keypair(true);
        if let Some(cert) = &certificate {
            builder = builder.certificate_path(cert);
        }
        if let Some(key) = &private_key {
            builder = builder.private_key_path(key);
        }

        let client = builder.client().map_err(|errors| {
            SetupError::new(format!(
                "security discovery and setup failed with status {}",
                errors.join(", ")
            ))
        })?;
        if debug && (config.pki_dir.is_some() || client_pki_root) {
            println!("Session {name}: (connect) setting up PKI provider");
        }

        // `setupSecurity` (`SessionOpen62541.cpp:1020-1160`).
        let identity = config.setup_identity();
        let token = identity.token().map_err(SetupError::new)?;
        let secured = if config.security_mode == SecurityMode::None {
            if debug {
                println!("Session {name}: (setupSecurity) no security configured ");
            }
            None
        } else {
            if !config.url.starts_with("opc.tcp") {
                errlog_vprintf(format_args!(
                    "OPC UA session {}: fatal - only URLs of type 'opc.tcp' supported\n",
                    config.name
                ));
                return Err(SetupError::new(
                    "security discovery and setup failed with status fatal",
                ));
            }
            let url = &config.url;
            if debug {
                println!("Session {name}: (setupSecurity) reading endpoints from {url}");
            }
            let endpoints = client
                .get_server_endpoints_from_url(url.as_str())
                .await
                .map_err(|e| {
                    let status = network_status(e.status());
                    if debug {
                        println!(
                            "Session {name}: (setupSecurity) UaDiscovery::getEndpoints from {url} \
                             failed with status {}",
                            status_name(status)
                        );
                    }
                    SetupError::service(
                        status,
                        "security discovery and setup failed with status cantConnect",
                    )
                })?;
            let Some((number, endpoint)) = config.select_endpoint(&endpoints) else {
                if debug {
                    println!(
                        "Session {name}: (setupSecurity) found no endpoint that matches the \
                         security requirements"
                    );
                }
                return Err(SetupError::service(
                    StatusCode::Good,
                    "security discovery and setup failed with status noMatchingEndpoint",
                ));
            };
            if debug {
                println!(
                    "Session {name}: (setupSecurity) found matching endpoint number {number}, \
                     using mode={:?} policy={} (level {})",
                    endpoint.security_mode,
                    security_policy_string(endpoint.security_policy_uri.as_ref()),
                    endpoint.security_level
                );
            }
            Some(endpoint)
        };
        Ok(Box::new(AsyncOpcuaClient {
            client,
            config: config.clone(),
            identity,
            token,
            secured,
            endpoint: None,
        }))
    }
}

struct AsyncOpcuaClient {
    client: Client,
    config: SessionConfig,
    identity: Identity,
    token: IdentityToken,
    /// The endpoint security discovery picked, `None` without security.
    secured: Option<EndpointDescription>,
    /// The endpoint the first connect settled on, as open62541 keeps it in
    /// `config.endpoint`.
    endpoint: Option<EndpointDescription>,
}

#[async_trait]
impl UaClient for AsyncOpcuaClient {
    fn identity(&self) -> &Identity {
        &self.identity
    }

    async fn connect(&mut self) -> Result<Arc<dyn UaConnection>, StatusCode> {
        let endpoint = match &self.endpoint {
            Some(endpoint) => endpoint.clone(),
            None => {
                // `UA_Client_connect` reads the endpoints again and picks the one
                // to connect to by itself, for `sec-mode=None` as for any other.
                let endpoints = self
                    .client
                    .get_server_endpoints_from_url(self.config.url.as_str())
                    .await
                    .map_err(|e| network_status(e.status()))?;
                let token = match self.token {
                    IdentityToken::Anonymous => UserTokenType::Anonymous,
                    IdentityToken::UserName(..) => UserTokenType::UserName,
                    IdentityToken::X509(..) => UserTokenType::Certificate,
                    IdentityToken::IssuedToken(..) => UserTokenType::IssuedToken,
                };
                let endpoint = self
                    .config
                    .connect_endpoint(&endpoints, self.secured.as_ref(), token)
                    .map_err(network_status)?;
                self.endpoint.insert(endpoint).clone()
            }
        };
        let (session, event_loop) = self
            .client
            .connect_to_endpoint_directly(endpoint, self.token.clone())
            .map_err(|e| network_status(e.status()))?;

        // The event loop makes the one attempt `session_retry_limit(0)` allows
        // and ends with its status; the session state stays `Connecting`, so
        // its end is the only failure signal. It also ends on a lost
        // connection, before it would reconnect: the session worker owns
        // reconnecting.
        let (ended_tx, ended) = watch::channel(());
        let mut handle = tokio::spawn(async move {
            let _ended = ended_tx;
            let events = event_loop.enter();
            tokio::pin!(events);
            loop {
                match events.try_next().await {
                    Ok(Some(SessionPollResult::ConnectionLost(status))) | Err(status) => {
                        return status;
                    }
                    Ok(Some(_)) => {}
                    Ok(None) => return StatusCode::Good,
                }
            }
        });
        let failed_with = tokio::select! {
            true = session.wait_for_connection() => None,
            ended = &mut handle => Some(ended.unwrap_or(StatusCode::BadInternalError)),
        };
        if let Some(status) = failed_with {
            return Err(network_status(status));
        }

        Ok(Arc::new(AsyncOpcuaConnection { session, ended }))
    }
}

struct AsyncOpcuaConnection {
    session: Arc<Session>,
    /// Never sent on; it closes when the session's event loop ends.
    ended: watch::Receiver<()>,
}

#[async_trait]
impl UaConnection for AsyncOpcuaConnection {
    async fn read(&self, nodes: &[ReadValueId]) -> (u32, Result<Vec<DataValue>, StatusCode>) {
        let read = Read::new(&self.session)
            .nodes_to_read(nodes.to_vec())
            .timestamps_to_return(TimestampsToReturn::Both)
            .max_age(0.0);
        let id = read.header().request_handle;
        let result = read.send(self.session.channel()).await;
        (
            id,
            result.map(|response| response.results.unwrap_or_default()),
        )
    }

    async fn write(&self, values: &[WriteValue]) -> (u32, Result<Vec<StatusCode>, StatusCode>) {
        let write = Write::new(&self.session).nodes_to_write(values.to_vec());
        let id = write.header().request_handle;
        let result = write.send(self.session.channel()).await;
        (
            id,
            result.map(|response| response.results.unwrap_or_default()),
        )
    }

    async fn register_nodes(&self, nodes: &[NodeId]) -> Result<Vec<NodeId>, StatusCode> {
        self.session.register_nodes(nodes).await
    }

    async fn server_info(&self) -> Result<ServerInfo, StatusCode> {
        let nodes: Vec<ReadValueId> = [NAMESPACE_ARRAY, MAX_NODES_PER_READ, MAX_NODES_PER_WRITE]
            .iter()
            .map(|id| ReadValueId::from(NodeId::new(0, *id)))
            .collect();
        let values = self
            .session
            .read(&nodes, TimestampsToReturn::Neither, 0.0)
            .await?;

        let namespace_array = match values.first().and_then(|v| v.value.as_ref()) {
            Some(Variant::Array(a)) => a
                .values
                .iter()
                .map(|v| match v {
                    Variant::String(s) => s.as_ref().to_string(),
                    other => other.to_string(),
                })
                .collect(),
            _ => Vec::new(),
        };
        // A server that does not publish an operation limit means "no limit".
        let limit = |i: usize| match values.get(i).and_then(|v| v.value.as_ref()) {
            Some(Variant::UInt32(v)) => *v,
            _ => 0,
        };
        Ok(ServerInfo {
            namespace_array,
            max_nodes_per_read: limit(1),
            max_nodes_per_write: limit(2),
        })
    }

    async fn type_tree(&self) -> Result<Arc<DataTypeTree>, StatusCode> {
        let tree = DataTypeTreeBuilder::new(|id| id.namespace != 0)
            .build(&self.session)
            .await
            .map_err(|e| e.status())?;
        Ok(Arc::new(tree))
    }

    async fn create_subscription(
        &self,
        config: &SubscriptionConfig,
        sink: mpsc::UnboundedSender<Notification>,
    ) -> Result<CreatedSubscription, StatusCode> {
        // `Session::create_subscription` sends the interval in whole
        // milliseconds; this is its body with the C's request.
        let header = CreateSubscription::new(&self.session).header().clone();
        let timeout = Duration::from_millis(header.timeout_hint.into());
        let response = match self
            .session
            .channel()
            .send(config.request(header), timeout)
            .await?
        {
            ResponseMessage::CreateSubscription(response) => response,
            ResponseMessage::ServiceFault(fault) => {
                return Err(fault.response_header.service_result);
            }
            _ => return Err(StatusCode::BadUnknownResponse),
        };
        if response.response_header.service_result.is_bad() {
            return Err(response.response_header.service_result);
        }

        let name = config.name.clone();
        let subscription = Subscription::new(
            response.subscription_id,
            Duration::from_millis(response.revised_publishing_interval.max(0.0).floor() as u64),
            response.revised_lifetime_count,
            response.revised_max_keep_alive_count,
            SubscriptionConfig::MAX_NOTIFICATIONS_PER_PUBLISH,
            config.priority,
            true,
            Box::new(SubscriptionCallbacks::new(
                // `SubscriptionOpen62541::subscriptionStatusChanged`.
                move |change| {
                    errlog_vprintf(format_args!(
                        "Subscription {name} status changed to {}\n",
                        status_name(change.status)
                    ))
                },
                move |value, item| {
                    // The channel is closed only when the driver is shutting the
                    // session down, so a failed send is not worth logging.
                    let _ = sink.send(Notification {
                        client_handle: item.client_handle(),
                        value,
                    });
                },
                |_, _| {},
            )),
        );
        self.session
            .subscription_state()
            .lock()
            .add_subscription(subscription);
        self.session.trigger_publish_now();
        Ok(CreatedSubscription {
            id: response.subscription_id,
            revised_publishing_interval: response.revised_publishing_interval,
        })
    }

    async fn create_monitored_items(
        &self,
        subscription_id: u32,
        items: Vec<MonitoredItemCreateRequest>,
    ) -> Result<Vec<MonitoredItemCreateResult>, StatusCode> {
        let created = self
            .session
            .create_monitored_items(subscription_id, TimestampsToReturn::Both, items)
            .await?;
        Ok(created.into_iter().map(|item| item.result).collect())
    }

    async fn disconnect(&self) {
        let _ = self.session.disconnect().await;
    }

    async fn closed(&self) {
        let mut ended = self.ended.clone();
        while ended.changed().await.is_ok() {}
    }

    fn endpoint(&self) -> Option<EndpointDescription> {
        Some(self.session.endpoint_info().endpoint.clone())
    }
}

/// C `Session::securityPolicyString`.
pub fn security_policy_string(uri: &str) -> String {
    if uri.is_empty() {
        return "None".to_string();
    }
    let Some((_, name)) = uri.rsplit_once('#') else {
        return "Invalid".to_string();
    };
    if SecurityPolicy::from_uri(uri) == SecurityPolicy::Unknown {
        format!("{name} (unsupported)")
    } else {
        name.to_string()
    }
}

/// How the client authenticates: what
/// [`SessionConfig::setup_identity`] read from the `sec-id` credentials file.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum Identity {
    #[default]
    Anonymous,
    UserName {
        user: String,
        password: String,
    },
    /// An X.509 identity certificate, distinct from the application certificate.
    Certificate {
        certificate: String,
        private_key: String,
    },
}

impl Identity {
    fn token(&self) -> Result<IdentityToken, String> {
        Ok(match self {
            Identity::Anonymous => IdentityToken::Anonymous,
            Identity::UserName { user, password } => {
                IdentityToken::new_user_name(user.clone(), password.clone())
            }
            Identity::Certificate {
                certificate,
                private_key,
            } => IdentityToken::new_x509_path(certificate, private_key)
                .map_err(|e| format!("loading client certificate {certificate} failed: {e}"))?,
        })
    }
}

/// The C's `sec-mode` option (`SessionOpen62541.cpp:214-234`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SecurityMode {
    /// Take the most secure mode the server offers.
    #[default]
    Best,
    None,
    Sign,
    SignAndEncrypt,
}

impl SecurityMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "best" => Some(SecurityMode::Best),
            "None" => Some(SecurityMode::None),
            "Sign" => Some(SecurityMode::Sign),
            "SignAndEncrypt" => Some(SecurityMode::SignAndEncrypt),
            _ => None,
        }
    }

    pub fn as_message_security_mode(self) -> MessageSecurityMode {
        match self {
            // `Best` is resolved by endpoint selection, which prefers the most
            // secure endpoint the server offers; until then it asks for the
            // strongest mode.
            SecurityMode::Best | SecurityMode::SignAndEncrypt => {
                MessageSecurityMode::SignAndEncrypt
            }
            SecurityMode::None => MessageSecurityMode::None,
            SecurityMode::Sign => MessageSecurityMode::Sign,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            SecurityMode::Best => "best",
            SecurityMode::None => "None",
            SecurityMode::Sign => "Sign",
            SecurityMode::SignAndEncrypt => "SignAndEncrypt",
        }
    }
}
