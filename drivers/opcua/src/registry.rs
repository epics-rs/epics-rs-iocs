//! The sessions, subscriptions and items an IOC has configured
//! (`Session::sessions`, `Subscription::subscriptions`, `Item::itemsBySession`).
//!
//! iocsh fills it before the records are loaded; the records' device support
//! then looks their session or subscription up here, and the workers are started
//! once — after `iocInit`, when every record has added its item, so the session's
//! initial read covers all of them (`SessionOpen62541::connect` is likewise
//! driven from an init hook).

use std::collections::HashMap;
use std::sync::Arc;

use async_opcua::types::NodeId;
use parking_lot::Mutex;

use crate::client::UaConnector;
use crate::item::{Item, Leaf};
use crate::link::{LinkInfo, LinkTarget, NameResolver, NodeIdentifier};
use crate::session::{self, Control, SessionConfig, SessionHandle, SessionWorker};
use crate::subscription::SubscriptionConfig;

pub struct Registry {
    connector: Arc<dyn UaConnector>,
    inner: Mutex<Inner>,
}

#[derive(Default)]
struct Inner {
    sessions: HashMap<String, Arc<SessionHandle>>,
    /// Workers wait here until [`Registry::start`] spawns them.
    workers: Vec<SessionWorker>,
    /// Subscription name → the session it runs on.
    subscriptions: HashMap<String, String>,
    /// The item of each `opcuaItem` record, by record name — what an element
    /// record's link resolves to.
    item_records: HashMap<String, Arc<Mutex<Item>>>,
}

/// What a record's device support gets when it binds its link.
pub struct Binding {
    pub session: Arc<SessionHandle>,
    pub item: Arc<Mutex<Item>>,
    pub leaf: Arc<Mutex<Leaf>>,
}

impl Registry {
    pub fn new(connector: Arc<dyn UaConnector>) -> Arc<Self> {
        Arc::new(Self {
            connector,
            inner: Mutex::new(Inner::default()),
        })
    }

    /// `opcuaSession` (`iocshIntegration.cpp:88-140`).
    pub fn add_session(&self, config: SessionConfig) -> Result<(), String> {
        let mut inner = self.inner.lock();
        if inner.sessions.contains_key(&config.name) {
            return Err(format!("session '{}' already exists", config.name));
        }
        let name = config.name.clone();
        let (handle, worker) = session::create(config, self.connector.clone());
        inner.sessions.insert(name, handle);
        inner.workers.push(worker);
        Ok(())
    }

    /// `opcuaSubscription` (`iocshIntegration.cpp:159-201`).
    pub fn add_subscription(&self, config: SubscriptionConfig) -> Result<(), String> {
        let mut inner = self.inner.lock();
        if inner.subscriptions.contains_key(&config.name) {
            return Err(format!("subscription '{}' already exists", config.name));
        }
        let session = inner
            .sessions
            .get(&config.session)
            .ok_or_else(|| format!("unknown session '{}'", config.session))?
            .clone();
        inner
            .subscriptions
            .insert(config.name.clone(), config.session.clone());
        session.subscriptions.lock().push(config);
        Ok(())
    }

    /// `opcuaOptions` — the name is a session's or a subscription's
    /// (`iocshIntegration.cpp:212-260`).
    pub fn set_option(&self, name: &str, key: &str, value: &str) -> Result<(), String> {
        let inner = self.inner.lock();
        if let Some(session) = inner.sessions.get(name) {
            return session.set_option(key, value);
        }
        let Some(session_name) = inner.subscriptions.get(name) else {
            return Err(format!("unknown session or subscription '{name}'"));
        };
        let session = inner.sessions[session_name].clone();
        drop(inner);
        let mut subscriptions = session.subscriptions.lock();
        let subscription = subscriptions
            .iter_mut()
            .find(|s| s.name == name)
            .expect("the subscription is on the session it was added to");
        subscription.set_option(key, value)
    }

    /// `opcuaMapNamespace` (`iocshIntegration.cpp:452-490`).
    pub fn map_namespace(&self, session: &str, index: u16, uri: &str) -> Result<(), String> {
        let session = self.session(session)?;
        session
            .config
            .lock()
            .namespace_map
            .insert(index, uri.to_string());
        Ok(())
    }

    /// `opcuaConnect` / `opcuaDisconnect`.
    pub fn control(&self, session: &str, control: Control) -> Result<(), String> {
        self.session(session)?.control(control);
        Ok(())
    }

    pub fn session(&self, name: &str) -> Result<Arc<SessionHandle>, String> {
        self.inner
            .lock()
            .sessions
            .get(name)
            .cloned()
            .ok_or_else(|| format!("unknown session '{name}'"))
    }

    pub fn sessions(&self) -> Vec<Arc<SessionHandle>> {
        self.inner.lock().sessions.values().cloned().collect()
    }

    /// Start every session's worker. Called once, after `iocInit`, so that the
    /// items of every record are in place before the first connection.
    pub fn start(&self) -> usize {
        let workers = std::mem::take(&mut self.inner.lock().workers);
        let started = workers.len();
        for worker in workers {
            tokio::spawn(worker.run());
        }
        started
    }

    /// Bind one record to its item: a new item on a session or subscription, or
    /// a data element of an already-loaded `opcuaItem` record
    /// (`opcua_add_record`, `devOpcua.cpp:88-110`).
    pub fn bind(
        &self,
        record: &str,
        link: LinkInfo,
        notify: tokio::sync::mpsc::Sender<()>,
    ) -> Result<Binding, String> {
        let leaf = Arc::new(Mutex::new(Leaf::new(
            record.to_string(),
            link.clone(),
            notify,
        )));

        if let LinkTarget::ItemRecord(item_record) = &link.target {
            // The item record must already be loaded — the C looks its
            // `RecordConnector` up in the database and refuses the link if it
            // has not been initialized (`linkParser.cpp:226-234`).
            let inner = self.inner.lock();
            let item = inner
                .item_records
                .get(item_record)
                .cloned()
                .ok_or_else(|| {
                    format!(
                        "opcuaItem record '{item_record}' is not loaded yet; \
                     it must be loaded before the records that are its elements"
                    )
                })?;
            let session_name = item.lock().link.session().unwrap_or_default().to_string();
            let session = inner.sessions[&session_name].clone();
            drop(inner);
            item.lock().leaves.push(leaf.clone());
            return Ok(Binding {
                session,
                item,
                leaf,
            });
        }

        let session_name = link
            .session()
            .ok_or_else(|| "the link names no session".to_string())?
            .to_string();
        let session = self.session(&session_name)?;
        let node_id = node_id_of(&link)?;

        let item = {
            let mut items = session.items.lock();
            let client_handle = items.len() as u32;
            let mut item = Item::new(link.clone(), node_id, client_handle);
            item.leaves.push(leaf.clone());
            let item = Arc::new(Mutex::new(item));
            items.push(item.clone());
            item
        };

        if link.is_item_record {
            self.inner
                .lock()
                .item_records
                .insert(record.to_string(), item.clone());
        }

        Ok(Binding {
            session,
            item,
            leaf,
        })
    }
}

fn node_id_of(link: &LinkInfo) -> Result<NodeId, String> {
    match link
        .identifier
        .clone()
        .ok_or_else(|| "the link names no node".to_string())?
    {
        NodeIdentifier::Numeric(id) => Ok(NodeId::new(link.namespace_index, id)),
        NodeIdentifier::String(id) => Ok(NodeId::new(link.namespace_index, id)),
    }
}

impl NameResolver for Registry {
    fn subscription_session(&self, name: &str) -> Option<String> {
        self.inner.lock().subscriptions.get(name).cloned()
    }

    fn is_session(&self, name: &str) -> bool {
        self.inner.lock().sessions.contains_key(name)
    }
}
