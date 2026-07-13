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
use async_opcua::client::{ClientBuilder, DataChangeCallback, IdentityToken, Session};
use async_opcua::types::{
    DataValue, MessageSecurityMode, MonitoredItemCreateRequest, NodeId, ReadValueId, StatusCode,
    TimestampsToReturn, Variant, WriteValue, custom::DataTypeTree,
};
use async_trait::async_trait;
use tokio::sync::mpsc;

use crate::session::SessionConfig;
use crate::subscription::SubscriptionConfig;

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
    async fn read(&self, nodes: &[ReadValueId]) -> Result<Vec<DataValue>, StatusCode>;
    async fn write(&self, values: &[WriteValue]) -> Result<Vec<StatusCode>, StatusCode>;
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
    ) -> Result<Vec<StatusCode>, StatusCode>;
    async fn disconnect(&self);
}

/// Opens sessions. The driver holds one; a test holds a double.
#[async_trait]
pub trait UaConnector: Send + Sync {
    async fn connect(&self, config: &SessionConfig) -> Result<Arc<dyn UaConnection>, String>;
}

/// The `async-opcua` client.
///
/// Accepted-and-documented: the C's `sec-id`/`ident-file` option selects an
/// identity from a credentials file and its `opcuaSaveRejected` command writes
/// rejected server certificates to a directory. `async-opcua` owns its own PKI
/// (`pki_dir`, with `rejected/` and `trusted/` beneath it) and takes the identity
/// directly, so the identity is configured here as an [`Identity`] and the
/// rejected-certificate directory is fixed by the PKI layout instead of being
/// separately settable.
pub struct AsyncOpcuaConnector {
    application_name: String,
    application_uri: String,
}

impl AsyncOpcuaConnector {
    pub fn new() -> Self {
        Self {
            application_name: "EPICS IOC".to_string(),
            application_uri: "urn:EPICS:IOC".to_string(),
        }
    }
}

impl Default for AsyncOpcuaConnector {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl UaConnector for AsyncOpcuaConnector {
    async fn connect(&self, config: &SessionConfig) -> Result<Arc<dyn UaConnection>, String> {
        let mut builder = ClientBuilder::new()
            .application_name(&self.application_name)
            .application_uri(&self.application_uri)
            .product_uri("urn:epics-rs:opcua")
            .session_retry_limit(0)
            .session_name(&config.name)
            .pki_dir(&config.pki_dir)
            .trust_server_certs(config.trust_server_certs)
            .create_sample_keypair(true);
        if let Some(cert) = &config.certificate_path {
            builder = builder.certificate_path(cert);
        }
        if let Some(key) = &config.private_key_path {
            builder = builder.private_key_path(key);
        }

        let mut client = builder
            .client()
            .map_err(|errors| format!("client configuration is invalid: {}", errors.join(", ")))?;

        let endpoint = config.endpoint_description();
        let identity = config.identity.token()?;
        let (session, event_loop) = client
            .connect_to_matching_endpoint(endpoint, identity)
            .await
            .map_err(|e| format!("connecting to {} failed: {e}", config.url))?;

        let handle = event_loop.spawn();
        session
            .wait_for_connection()
            .await
            .then_some(())
            .ok_or_else(|| format!("session to {} did not come up", config.url))?;

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
    async fn read(&self, nodes: &[ReadValueId]) -> Result<Vec<DataValue>, StatusCode> {
        self.session
            .read(nodes, TimestampsToReturn::Both, 0.0)
            .await
    }

    async fn write(&self, values: &[WriteValue]) -> Result<Vec<StatusCode>, StatusCode> {
        self.session.write(values).await
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
        self.session
            .create_subscription(
                Duration::from_secs_f64(config.publishing_interval / 1000.0),
                config.lifetime_count,
                config.max_keep_alive_count,
                config.max_notifications_per_publish,
                config.priority,
                true,
                DataChangeCallback::new(move |value, item| {
                    // The channel is closed only when the driver is shutting the
                    // session down, so a failed send is not worth logging.
                    let _ = sink.send(Notification {
                        client_handle: item.client_handle(),
                        value,
                    });
                }),
            )
            .await
    }

    async fn create_monitored_items(
        &self,
        subscription_id: u32,
        items: Vec<MonitoredItemCreateRequest>,
    ) -> Result<Vec<StatusCode>, StatusCode> {
        let created = self
            .session
            .create_monitored_items(subscription_id, TimestampsToReturn::Both, items)
            .await?;
        Ok(created.iter().map(|item| item.result.status_code).collect())
    }

    async fn disconnect(&self) {
        let _ = self.session.disconnect().await;
    }
}

/// How the client authenticates (`sec-id`, `SessionOpen62541.cpp:159-260`).
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
                .map_err(|e| format!("reading the identity certificate failed: {e}"))?,
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
