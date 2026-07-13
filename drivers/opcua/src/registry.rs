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
///
/// Cloneable because an `opcuaItem` record needs it as well as its device
/// support does — it is the C's `prec->dpvt`, which the record's `special()`
/// reaches through (`opcuaItemRecord.cpp:107-125`).
///
/// There is no session here: a record reaches its session through its item
/// ([`Item::request`]), which is the only thing that knows which session the
/// node is on.
#[derive(Clone, Debug)]
pub struct Binding {
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

    /// Every name `opcuaOptions` and `opcuaShow` can match: the sessions and the
    /// subscriptions (`RegistryKeyNamespace::global`, which the C keeps them
    /// both in).
    pub fn names(&self) -> Vec<String> {
        let inner = self.inner.lock();
        inner
            .sessions
            .keys()
            .chain(inner.subscriptions.keys())
            .cloned()
            .collect()
    }

    /// Start every session's worker. Called once, after `iocInit`, so that the
    /// items of every record are in place before the first connection.
    ///
    /// Deviation, forced by the framework gap in [`Self::bind`]: an element
    /// record whose link names an `opcuaItem` record that is not in the database
    /// is reported here rather than refused at init. The C can refuse it, because
    /// it parses the link as each record is *loaded* (`opcua_add_record` is a
    /// device support extension, `devOpcua.cpp:88`), by which point every record
    /// loaded before it is initialized; this port parses at `iocInit`, in an
    /// order that says nothing about which record was loaded first, so "the item
    /// record has not bound yet" and "there is no such item record" are only
    /// distinguishable once every record has bound.
    pub fn start(&self) -> usize {
        let mut inner = self.inner.lock();
        for (name, item) in &inner.item_records {
            let item = item.lock();
            if item.is_adopted() {
                continue;
            }
            let records: Vec<String> = item
                .leaves
                .iter()
                .map(|leaf| leaf.lock().record.clone())
                .collect();
            log::error!(
                "no opcuaItem record '{name}' with OPCUA device support is loaded; \
                 the records linked to it will never update: {}",
                records.join(", ")
            );
        }

        let workers = std::mem::take(&mut inner.workers);
        let started = workers.len();
        drop(inner);
        for worker in workers {
            tokio::spawn(worker.run());
        }
        started
    }

    /// Bind one record to its item: a new item on a session or subscription, or
    /// a data element of an `opcuaItem` record (`opcua_add_record`,
    /// `devOpcua.cpp:88-110`).
    ///
    /// Why an element record's item is created on demand rather than looked
    /// up: the C binds as each record is *loaded* — `opcua_add_record` is a
    /// device support extension — and can therefore require the `opcuaItem`
    /// record to be already initialized (`linkParser.cpp:226-234`). epics-rs
    /// wires device support in database load order too since PR #29 (it was
    /// `HashMap` order before, which made the C module's example database fail
    /// at random), but this port keeps binding order-independent rather than
    /// reintroducing the C's declaration-order requirement.
    ///
    /// So both an element record and the item record itself reach the item
    /// through [`Self::item_of_record`], and whichever of the two binds first
    /// creates it. The item record's binding is the one that [`Item::adopt`]s it:
    /// gives it its node, its session and its client handle.
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
            let item = self.item_of_record(item_record);
            item.lock().leaves.push(leaf.clone());
            return Ok(Binding { item, leaf });
        }

        let session_name = link
            .session()
            .ok_or_else(|| "the link names no session".to_string())?
            .to_string();
        let session = self.session(&session_name)?;
        let node_id = node_id_of(&link)?;

        let item = if link.is_item_record {
            self.item_of_record(record)
        } else {
            Arc::new(Mutex::new(Item::pending()))
        };
        {
            let mut items = session.items.lock();
            let client_handle = items.len() as u32;
            let mut locked = item.lock();
            locked.adopt(link, node_id, client_handle, session.clone());
            locked.leaves.push(leaf.clone());
            drop(locked);
            items.push(item.clone());
        }

        Ok(Binding { item, leaf })
    }

    /// The item of an `opcuaItem` record, by that record's name — created empty
    /// if neither that record nor one of its element records has bound yet.
    fn item_of_record(&self, record: &str) -> Arc<Mutex<Item>> {
        self.inner
            .lock()
            .item_records
            .entry(record.to_string())
            .or_insert_with(|| Arc::new(Mutex::new(Item::pending())))
            .clone()
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
