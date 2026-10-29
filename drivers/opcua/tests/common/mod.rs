//! A server double behind a real `Registry`, for the tests that drive sessions
//! through the iocsh commands and the IOC's exit.

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use async_opcua::types::{
    AttributeId, DataValue, EndpointDescription, MessageSecurityMode, MonitoredItemCreateRequest,
    MonitoredItemCreateResult, NodeId, ReadValueId, StatusCode, Variant, WriteValue,
};
use async_trait::async_trait;
use parking_lot::Mutex;
use tokio::sync::mpsc;

use opcua::client::{
    CreatedSubscription, Identity, Notification, ServerInfo, SetupError, UaClient, UaConnection,
    UaConnector,
};
use opcua::ioc;
use opcua::registry::Registry;
use opcua::session::SessionConfig;
use opcua::subscription::SubscriptionConfig;

#[derive(Default)]
pub struct Server {
    pub setups: AtomicUsize,
    pub connects: AtomicUsize,
    pub disconnects: AtomicUsize,
    /// Connect attempts never finish — a server that does not answer.
    pub hang: bool,
    /// Connect attempts fail.
    pub refuse: bool,
    /// How long a connect attempt and a disconnect take.
    pub latency: Duration,
    /// What each node holds; a read of any other node fails
    /// `BadNodeIdUnknown`.
    pub nodes: Vec<(NodeId, Variant)>,
    /// The Write service calls, in order.
    pub writes: Mutex<Vec<Vec<WriteValue>>>,
}

struct Connector(Arc<Server>);

#[async_trait]
impl UaConnector for Connector {
    async fn setup(&self, config: &SessionConfig) -> Result<Box<dyn UaClient>, SetupError> {
        self.0.setups.fetch_add(1, Ordering::SeqCst);
        Ok(Box::new(Client(self.0.clone(), config.setup_identity())))
    }
}

struct Client(Arc<Server>, Identity);

#[async_trait]
impl UaClient for Client {
    fn identity(&self) -> &Identity {
        &self.1
    }
    async fn connect(&mut self) -> Result<Arc<dyn UaConnection>, StatusCode> {
        self.0.connects.fetch_add(1, Ordering::SeqCst);
        if self.0.hang {
            std::future::pending::<()>().await;
        }
        tokio::time::sleep(self.0.latency).await;
        if self.0.refuse {
            return Err(StatusCode::BadDisconnect);
        }
        Ok(Arc::new(Connection(self.0.clone())))
    }
}

struct Connection(Arc<Server>);

#[async_trait]
impl UaConnection for Connection {
    async fn read(&self, nodes: &[ReadValueId]) -> (u32, Result<Vec<DataValue>, StatusCode>) {
        let results = nodes
            .iter()
            .map(|node| {
                let Some((_, value)) = self.0.nodes.iter().find(|(id, _)| *id == node.node_id)
                else {
                    return DataValue {
                        status: Some(StatusCode::BadNodeIdUnknown),
                        ..Default::default()
                    };
                };
                if node.attribute_id == AttributeId::DataType as u32 {
                    let scalar = value.scalar_type_id().expect("a scalar node");
                    DataValue::value_only(Variant::NodeId(Box::new(NodeId::new(0, scalar as u32))))
                } else {
                    DataValue {
                        value: Some(value.clone()),
                        status: Some(StatusCode::Good),
                        ..Default::default()
                    }
                }
            })
            .collect();
        (0, Ok(results))
    }
    async fn write(&self, values: &[WriteValue]) -> (u32, Result<Vec<StatusCode>, StatusCode>) {
        self.0.writes.lock().push(values.to_vec());
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
        tokio::time::sleep(self.0.latency).await;
        self.0.disconnects.fetch_add(1, Ordering::SeqCst);
    }
    fn endpoint(&self) -> Option<EndpointDescription> {
        Some(EndpointDescription {
            security_mode: MessageSecurityMode::None,
            security_policy_uri: "http://opcfoundation.org/UA/SecurityPolicy#None".into(),
            ..Default::default()
        })
    }
}

/// A registry with session `S1` on `server`, given `autoconnect` as its option.
pub fn registry(server: &Arc<Server>, autoconnect: &str) -> Arc<Registry> {
    let registry = Registry::new(Arc::new(Connector(server.clone())));
    ioc::session_command(
        &registry,
        Some("S1"),
        Some("opc.tcp://localhost:4999"),
        &[autoconnect],
    );
    registry
}
