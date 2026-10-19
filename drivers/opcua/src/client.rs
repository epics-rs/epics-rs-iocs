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

use std::sync::Arc;
use std::time::Duration;

use async_opcua::client::custom_types::DataTypeTreeBuilder;
use async_opcua::client::services::{Read, Write};
use async_opcua::client::{
    ClientBuilder, IdentityToken, Session, SubscriptionCallbacks, UARequest,
};
use async_opcua::crypto::SecurityPolicy;
use async_opcua::types::{
    DataValue, EndpointDescription, MessageSecurityMode, MonitoredItemCreateRequest,
    MonitoredItemCreateResult, NodeId, ReadValueId, StatusCode, TimestampsToReturn, Variant,
    WriteValue, custom::DataTypeTree,
};
use async_trait::async_trait;
use epics_rs::base::runtime::log::errlog_vprintf;
use parking_lot::Mutex;
use tokio::sync::mpsc;

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
    ) -> Result<u32, StatusCode>;
    /// Returns one status per requested item, in order.
    async fn create_monitored_items(
        &self,
        subscription_id: u32,
        items: Vec<MonitoredItemCreateRequest>,
    ) -> Result<Vec<MonitoredItemCreateResult>, StatusCode>;
    async fn disconnect(&self);
    /// The endpoint the session runs on, as the server described it.
    fn endpoint(&self) -> Option<EndpointDescription> {
        None
    }
}

/// Opens sessions. The driver holds one; a test holds a double.
#[async_trait]
pub trait UaConnector: Send + Sync {
    async fn connect(
        &self,
        config: &SessionConfig,
        identity: &Identity,
    ) -> Result<Arc<dyn UaConnection>, String>;
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
    async fn connect(
        &self,
        config: &SessionConfig,
        identity: &Identity,
    ) -> Result<Arc<dyn UaConnection>, String> {
        // The session's own options win over the client-wide ones; what neither
        // sets falls back to the `async-opcua` default PKI location.
        let client = self.security();
        let pki_dir = config
            .pki_dir
            .clone()
            .or(client.pki_root)
            .unwrap_or_else(|| DEFAULT_PKI_DIR.to_string());
        let certificate = config.certificate_path.clone().or(client.certificate_path);
        let private_key = config.private_key_path.clone().or(client.private_key_path);

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

        let mut client = builder.client().map_err(|errors| {
            format!(
                "security discovery and setup failed with status {}",
                errors.join(", ")
            )
        })?;

        let identity = identity.token()?;
        let connected = if config.security_mode == SecurityMode::None {
            client.connect_to_endpoint_directly(config.unsecured_endpoint(), identity)
        } else {
            if !config.url.starts_with("opc.tcp") {
                errlog_vprintf(format_args!(
                    "OPC UA session {}: fatal - only URLs of type 'opc.tcp' supported\n",
                    config.name
                ));
                return Err("security discovery and setup failed with status fatal".to_string());
            }
            let endpoints = client
                .get_server_endpoints_from_url(config.url.as_str())
                .await
                .map_err(|_| "security discovery and setup failed with status cantConnect")?;
            let endpoint = config
                .select_endpoint(&endpoints)
                .ok_or("security discovery and setup failed with status noMatchingEndpoint")?;
            client.connect_to_endpoint_directly(endpoint, identity)
        };
        let (session, event_loop) = connected.map_err(|e| {
            format!(
                "connect service failed with status {}",
                status_name(e.status())
            )
        })?;

        // The event loop makes the one attempt `session_retry_limit(0)` allows
        // and ends with its status; the session state stays `Connecting`, so
        // its end is the only failure signal.
        let mut handle = event_loop.spawn();
        let ended = tokio::select! {
            true = session.wait_for_connection() => None,
            ended = &mut handle => Some(ended.unwrap_or(StatusCode::BadInternalError)),
        };
        if let Some(status) = ended {
            return Err(format!(
                "connect service failed with status {}",
                status_name(status)
            ));
        }

        Ok(Arc::new(AsyncOpcuaConnection {
            session,
            _event_loop: handle,
        }))
    }
}

struct AsyncOpcuaConnection {
    session: Arc<Session>,
    /// Dropping this aborts the session's event loop, which is what closes the
    /// connection when the driver drops the session.
    _event_loop: tokio::task::JoinHandle<StatusCode>,
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
    ) -> Result<u32, StatusCode> {
        let name = config.name.clone();
        self.session
            .create_subscription(
                Duration::from_secs_f64(config.publishing_interval / 1000.0),
                config.lifetime_count(),
                SubscriptionConfig::MAX_KEEP_ALIVE_COUNT,
                SubscriptionConfig::MAX_NOTIFICATIONS_PER_PUBLISH,
                config.priority,
                true,
                SubscriptionCallbacks::new(
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
                ),
            )
            .await
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
        match s.to_ascii_lowercase().as_str() {
            "best" => Some(SecurityMode::Best),
            "none" => Some(SecurityMode::None),
            "sign" => Some(SecurityMode::Sign),
            "signandencrypt" => Some(SecurityMode::SignAndEncrypt),
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
