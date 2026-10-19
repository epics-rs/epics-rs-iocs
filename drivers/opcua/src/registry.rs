//! The sessions, subscriptions and items an IOC has configured
//! (`Session::sessions`, `Subscription::subscriptions`, `Item::itemsBySession`).
//!
//! iocsh fills it before the records are loaded; the records' device support
//! then looks their session or subscription up here, and the workers are started
//! once — after `iocInit`, when every record has added its item, so the session's
//! initial read covers all of them (`SessionOpen62541::connect` is likewise
//! driven from an init hook).

use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use async_opcua::types::NodeId;
use parking_lot::Mutex;

use crate::client::UaConnector;
use crate::ioc::glob_match;
use crate::item::{Item, Leaf};
use crate::link::{LinkInfo, LinkTarget, NameResolver, NodeIdentifier};
use crate::session::{self, SessionConfig, SessionHandle, SessionWorker};
use crate::subscription::SubscriptionConfig;

pub struct Registry {
    connector: Arc<dyn UaConnector>,
    inner: Mutex<Inner>,
}

#[derive(Default)]
struct Inner {
    /// In creation order, which is the order a glob reports them in; the C's
    /// glob yields a `std::set` of pointers, an order it does not define.
    sessions: Vec<Arc<SessionHandle>>,
    /// Workers wait here until [`Registry::start`] spawns them.
    workers: Vec<SessionWorker>,
    /// (subscription, the session it runs on), in creation order.
    subscriptions: Vec<(String, String)>,
    /// The item of each `opcuaItem` record, by record name — what an element
    /// record's link resolves to.
    item_records: HashMap<String, Arc<Mutex<Item>>>,
    /// Every OPCUA record's type by name, declared as it attaches at load and
    /// so before any init: what C's `dbFindRecord` + `RTYP` check in
    /// `linkParser.cpp:213-232` reads, whatever order the records init in.
    records: HashMap<String, String>,
    /// `opcuaItem` records whose own init failed (C: `dpvt` left NULL).
    failed_item_records: HashSet<String>,
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

    /// `Session::createSession` (`Session.cpp:66-73`): `None` when the name is
    /// taken by a session or a subscription.
    pub fn add_session(&self, config: SessionConfig) -> Option<Arc<SessionHandle>> {
        let mut inner = self.inner.lock();
        if inner.name_in_use(&config.name) {
            return None;
        }
        let (handle, worker) = session::create(config, self.connector.clone());
        inner.sessions.push(handle.clone());
        inner.workers.push(worker);
        Some(handle)
    }

    /// `Subscription::createSubscription` (`Subscription.cpp:28-36`): false when
    /// the name is taken by a session or a subscription, or there is no such
    /// session.
    pub fn add_subscription(&self, config: SubscriptionConfig) -> bool {
        let mut inner = self.inner.lock();
        let Some(session) = inner.session(&config.session).cloned() else {
            return false;
        };
        if inner.name_in_use(&config.name) {
            return false;
        }
        inner
            .subscriptions
            .push((config.name.clone(), config.session.clone()));
        session.subscriptions.lock().push(config);
        true
    }

    /// `RegistryKeyNamespace::global.contains`: sessions and subscriptions
    /// share one namespace.
    pub fn name_in_use(&self, name: &str) -> bool {
        self.inner.lock().name_in_use(name)
    }

    /// `Subscription::find`.
    pub fn has_subscription(&self, name: &str) -> bool {
        self.inner.lock().subscription_session(name).is_some()
    }

    /// `SubscriptionOpen62541::setOption` on the named subscription.
    pub fn set_subscription_option(&self, name: &str, key: &str, value: &str) {
        let Some(session) = self
            .subscription_session(name)
            .and_then(|s| self.session(&s).ok())
        else {
            return;
        };
        let mut subscriptions = session.subscriptions.lock();
        if let Some(subscription) = subscriptions.iter_mut().find(|s| s.name == name) {
            subscription.set_option(key, value);
        }
    }

    pub fn session(&self, name: &str) -> Result<Arc<SessionHandle>, String> {
        self.inner
            .lock()
            .session(name)
            .cloned()
            .ok_or_else(|| format!("unknown session '{name}'"))
    }

    pub fn sessions(&self) -> Vec<Arc<SessionHandle>> {
        self.inner.lock().sessions.clone()
    }

    /// `Session::glob`.
    pub fn glob_sessions(&self, pattern: &str) -> Vec<Arc<SessionHandle>> {
        let inner = self.inner.lock();
        inner
            .sessions
            .iter()
            .filter(|s| glob_match(pattern, &s.name))
            .cloned()
            .collect()
    }

    /// `Subscription::glob`, by name.
    pub fn glob_subscriptions(&self, pattern: &str) -> Vec<String> {
        let inner = self.inner.lock();
        inner
            .subscriptions
            .iter()
            .filter(|(name, _)| glob_match(pattern, name))
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// Start every session's worker. Called once, after `iocInit`, so that the
    /// items of every record are in place before the first connection.
    ///
    /// An element record whose `opcuaItem` record failed its init *after* the
    /// element bound is reported here: init order is not load order in this
    /// port, so only now is the item known never to be adopted.
    pub fn start(&self) -> usize {
        let mut inner = self.inner.lock();
        for (name, item) in &inner.item_records {
            let item = item.lock();
            if item.is_adopted() {
                continue;
            }
            // C's `add_record` refuses each linking record (`devOpcua.cpp:107`).
            for leaf in &item.leaves {
                eprintln!(
                    "{} Error in add_record : opcuaItemRecord '{name}' was not initialized correctly",
                    leaf.lock().record
                );
            }
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
    /// A record with OPCUA device support attached (`set_record_info`).
    pub fn declare_record(&self, name: &str, record_type: &str) {
        self.inner
            .lock()
            .records
            .insert(name.to_string(), record_type.to_string());
    }

    /// An `opcuaItem` record's init failed; element records naming it are
    /// refused from now on.
    pub fn item_record_failed(&self, name: &str) {
        self.inner
            .lock()
            .failed_item_records
            .insert(name.to_string());
    }

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
            {
                let inner = self.inner.lock();
                match inner.records.get(item_record) {
                    None => {
                        return Err(format!(
                            "unknown subscription/session/opcuaItemRecord '{item_record}'"
                        ));
                    }
                    Some(t) if t != "opcuaItem" => {
                        return Err(format!("record '{item_record}' is not of type opcuaItem"));
                    }
                    _ if inner.failed_item_records.contains(item_record) => {
                        return Err(format!(
                            "opcuaItemRecord '{item_record}' was not initialized correctly"
                        ));
                    }
                    _ => {}
                }
            }
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

impl Inner {
    fn session(&self, name: &str) -> Option<&Arc<SessionHandle>> {
        self.sessions.iter().find(|s| s.name == name)
    }

    fn subscription_session(&self, name: &str) -> Option<&str> {
        self.subscriptions
            .iter()
            .find(|(subscription, _)| subscription == name)
            .map(|(_, session)| session.as_str())
    }

    fn name_in_use(&self, name: &str) -> bool {
        self.session(name).is_some() || self.subscription_session(name).is_some()
    }
}

impl NameResolver for Registry {
    fn subscription_session(&self, name: &str) -> Option<String> {
        self.inner
            .lock()
            .subscription_session(name)
            .map(str::to_string)
    }

    fn is_session(&self, name: &str) -> bool {
        self.inner.lock().session(name).is_some()
    }
}
