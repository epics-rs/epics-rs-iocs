//! Items and their data elements (`ItemOpen62541.cpp`,
//! `DataElementOpen62541Leaf.cpp`, `DataElementOpen62541Node.cpp`, `ElementTree.h`).
//!
//! An *item* is one OPC UA node. A *leaf* is one record bound to that node — to
//! the node's whole value, or, when the value is a structure, to one element
//! inside it, addressed by the link's `element=a.b.c` path.
//!
//! The C keeps the reason a record is being processed in two places at once: the
//! `RecordConnector` sets `pconnector->reason` before calling `dbProcess`, and
//! the same reason is also inside the update it pushed onto the queue
//! (`RecordConnector.cpp:56-77` against `Update.h:45`). Here the queue is the
//! only source: the reason a record processes for is the reason of the update it
//! pops. There is no second copy to fall out of step with it.

use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use async_opcua::types::custom::{DataTypeTree, DynamicStructure, EnumTypeInfo, TypeInfoRef};
use async_opcua::types::{
    DataValue, DateTime, ExpandedMessageInfo, ExtensionObject, NodeId, StatusCode, StructureType,
    Variant,
};
use epics_rs::base::calc::cvt::fmt_g;
use epics_rs::base::runtime::log::errlog_vprintf;
use parking_lot::Mutex;
use tokio::sync::mpsc;

use crate::link::{Bini, LinkInfo, NodeIdentifier, TimestampSource};
use crate::queue::{ConnectionStatus, ProcessReason, Update, UpdateQueue};
use crate::session::{Priority, Request, SessionHandle, status_name};
use crate::value::{EnumChoices, type_name};

/// One record's binding to an item (`DataElementOpen62541Leaf` +
/// `RecordConnector`).
#[derive(Debug)]
pub struct Leaf {
    pub record: String,
    /// `RecordConnector::getRecordType`.
    pub record_type: String,
    pub link: LinkInfo,
    /// Incoming updates, one popped per record process cycle.
    pub queue: UpdateQueue,
    /// The value the record last wrote, waiting to go out.
    pub outgoing: Option<Variant>,
    /// Set when `outgoing` has not reached the server yet (`markAsDirty`).
    pub dirty: bool,
    /// The value the element last received, whose type the outgoing value takes.
    pub incoming: Option<Variant>,
    /// The enumeration this element's type defines, if it is one.
    pub choices: Option<Arc<EnumChoices>>,
    pub state: ConnectionStatus,
    /// Wakes the record. The framework processes the record on every pulse
    /// (`io_intr_scan_independent`), which is how a value reaches a record
    /// whatever its SCAN — the C does it with `callbackRequest` + `dbProcess`.
    notify: mpsc::Sender<()>,
}

impl Leaf {
    pub fn new(
        record: String,
        record_type: &str,
        link: LinkInfo,
        notify: mpsc::Sender<()>,
    ) -> Self {
        let queue = UpdateQueue::new(link.client_queue_size as usize, link.discard_oldest);
        Self {
            record,
            record_type: record_type.to_string(),
            link,
            queue,
            outgoing: None,
            dirty: false,
            incoming: None,
            choices: None,
            state: ConnectionStatus::Down,
            notify,
        }
    }

    /// Ask the record to process for a reason no value came with — the C's
    /// `RecordConnector::requestRecordProcessing` (`RecordConnector.cpp:88-113`),
    /// which sets `pcon->reason` and calls `dbProcess`. Here the reason travels
    /// on the queue, like every other reason a record processes for.
    pub fn request_processing(&mut self, reason: ProcessReason) {
        self.push(Update::new(
            reason,
            None,
            StatusCode::Good,
            SystemTime::now(),
        ));
    }

    /// Queue an update and wake the record if this is the one it will see first.
    fn push(&mut self, update: Update) {
        if self.queue.push(update) {
            // A full channel already holds a pulse the record has not consumed,
            // and one pulse drains the whole queue, so dropping it is correct.
            let _ = self.notify.try_send(());
        }
    }
}

/// One OPC UA node (`ItemOpen62541`).
#[derive(Debug)]
pub struct Item {
    /// The link of the record that *addresses the node* — an `opcuaItem` record,
    /// or the single record bound directly to it.
    pub link: LinkInfo,
    /// As configured. Rebuilt into [`Self::node_id`] once the server's namespace
    /// array is known.
    pub configured_node_id: NodeId,
    pub node_id: NodeId,
    /// What `RegisterNodes` returned, if the link asked for it.
    pub registered_node_id: Option<NodeId>,
    /// The node's DataType attribute, read alongside every value
    /// (`SessionOpen62541::processRequests(ReadRequest)` reads DATATYPE and
    /// VALUE as a pair, `no_of_properties_read = 2`).
    pub data_type: Option<NodeId>,
    pub state: ConnectionStatus,
    /// Its place in `SessionHandle::items`, which is what the worker's requests
    /// name it by. Not the monitored item's client handle.
    pub index: u32,
    /// What the server granted when the monitored item was last created; 0
    /// until then (`ItemOpen62541::revisedSamplingInterval`/`revisedQueueSize`).
    pub revised_sampling_interval: f64,
    pub revised_queue_size: u32,
    /// Every record bound to this node.
    pub leaves: Vec<Arc<Mutex<Leaf>>>,
    /// The item's own last value, from which every leaf's element is taken.
    pub last_value: Option<Variant>,
    /// The status and the timestamp of the last thing that happened to the node,
    /// which the `opcuaItem` record shows in STATCODE/STATTEXT and takes its own
    /// timestamp from (`ItemOpen62541::getStatus`, `ItemOpen62541.cpp:251-275`).
    pub last_status: StatusCode,
    pub last_timestamp: Option<SystemTime>,
    /// The server's data type dictionary, needed to rebuild a structure around a
    /// changed element.
    pub type_tree: Option<Arc<DataTypeTree>>,
    /// The element tree was checked against a structure value and what it
    /// cannot supply was reported (`DataElementOpen62541Node::mapped`); a
    /// connection loss drops the mapping.
    mapped: bool,
    /// The session the item's node lives on — `None` until
    /// [`Item::adopt`] puts it there.
    ///
    /// A record only ever reaches its session *through* its item, so this is the
    /// one place the two are tied together, and it is set exactly when the item
    /// enters `SessionHandle::items` (at [`Self::index`]).
    session: Option<Arc<SessionHandle>>,
}

/// What a record asks its item's session to do with the node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    Read,
    Write,
}

impl Item {
    /// An item whose node and session are not known yet.
    ///
    /// An element record's link names its `opcuaItem` record, and that record's
    /// device support is what tells the item which node and session it is on.
    /// The framework wires device support in database load order (epics-rs
    /// PR #29; it was `HashMap` order before), but this port does not depend on
    /// it: an element record declared above its item record still binds first.
    /// The item exists from whichever of the two binds first, and the item
    /// record's binding adopts it.
    pub fn pending() -> Self {
        Self {
            link: LinkInfo::default(),
            configured_node_id: NodeId::null(),
            node_id: NodeId::null(),
            registered_node_id: None,
            data_type: None,
            state: ConnectionStatus::Down,
            index: 0,
            revised_sampling_interval: 0.0,
            revised_queue_size: 0,
            leaves: Vec::new(),
            last_value: None,
            last_status: StatusCode::BadServerNotConnected,
            last_timestamp: None,
            type_tree: None,
            mapped: false,
            session: None,
        }
    }

    /// The record that *addresses the node* has bound: the item is on this
    /// session, at this node, at this index of its items.
    pub fn adopt(
        &mut self,
        link: LinkInfo,
        node_id: NodeId,
        index: u32,
        session: Arc<SessionHandle>,
    ) {
        self.link = link;
        self.configured_node_id = node_id;
        // Built at each connect (`ItemOpen62541::rebuildNodeId`); null until
        // then, as `opcuaShow` prints it.
        self.node_id = NodeId::null();
        self.index = index;
        self.session = Some(session);
    }

    /// Whether the item ever found the record that addresses its node
    /// ([`Item::adopt`]).
    pub fn is_adopted(&self) -> bool {
        self.session.is_some()
    }

    /// Ask the session to read or write the node
    /// (`RecordConnector::requestOpcuaRead`/`requestOpcuaWrite`).
    ///
    /// Framework gap: a record's PRIO lives in `RecordInstance.common`, which
    /// device support cannot reach (`set_record_info` passes the name and the
    /// scan, nothing else), so the C's three priority queues
    /// (`RequestQueueBatcher`, one per `menuPriority`) collapse to one here.
    pub fn request(&self, priority: Priority, action: Action) {
        let Some(session) = &self.session else {
            // The item was never adopted: the `opcuaItem` record its records link
            // to is not in the database. `Registry::start` reports that.
            return;
        };
        let handle = self.index;
        session.request(
            priority,
            match action {
                Action::Read => Request::Read { handle },
                Action::Write => Request::Write { handle },
            },
        );
    }

    /// The node id to use on the wire: the registered one when the server gave
    /// us one (`ItemOpen62541::getNodeId`).
    pub fn wire_node_id(&self) -> &NodeId {
        self.registered_node_id.as_ref().unwrap_or(&self.node_id)
    }

    pub fn is_monitored(&self) -> bool {
        self.link.monitor && self.link.subscription().is_some()
    }

    /// A value arrived from the server (`ItemOpen62541::setIncomingData`).
    ///
    /// Every leaf takes the part of it its element path addresses, along with
    /// the timestamp its link asks for, and queues it.
    pub fn set_incoming_data(&mut self, value: DataValue, reason: ProcessReason) {
        let status = value.status.unwrap_or(StatusCode::Good);
        let data = value.value.clone().unwrap_or(Variant::Empty);
        self.last_value = Some(data.clone());
        self.last_status = status;
        self.map_elements(&data);
        let data_time = self.data_timestamp(&data);
        self.last_timestamp = Some(timestamp_for(&self.link, &value, data_time));

        for leaf in &self.leaves {
            let mut leaf = leaf.lock();
            // `bini=ignore` throws the initial read away, but the item still
            // needs it to learn the node's type for later writes
            // (`ItemOpen62541.cpp:150-170`).
            let element = element_of(&data, &leaf.link.element_path);
            if let Some(element) = &element {
                leaf.incoming = Some(element.clone());
            }

            let timestamp = timestamp_for(&leaf.link, &value, data_time);
            if leaf.state == ConnectionStatus::InitialRead {
                match leaf.link.bini {
                    // The initial value is thrown away; the item still needed it
                    // to learn the node's type (`ItemOpen62541.cpp:150-170`).
                    Bini::Ignore => {
                        leaf.state = ConnectionStatus::Up;
                        continue;
                    }
                    // The record's own value goes to the server instead of the
                    // one just read (`useReadValue` is false here, and
                    // `manageStateAndBiniProcessing`, `devOpcua.cpp:186-199`,
                    // asks the record to process for a write). The write request
                    // is the update the record pops, so the reason it processes
                    // for still comes from the queue and nowhere else.
                    Bini::Write => {
                        leaf.state = ConnectionStatus::InitialWrite;
                        leaf.push(Update::new(
                            ProcessReason::WriteRequest,
                            None,
                            status,
                            timestamp,
                        ));
                        continue;
                    }
                    Bini::Read => {}
                }
            }

            let update = match element {
                Some(element) => Update::new(reason, Some(element), status, timestamp),
                // The element path names something the structure does not have.
                None => Update::new(ProcessReason::ReadFailure, None, status, timestamp),
            };
            leaf.push(update);
            if leaf.state == ConnectionStatus::InitialRead {
                leaf.state = ConnectionStatus::Up;
            }
        }
    }

    /// An event with no value — a read failure, a write result, a connection
    /// loss (`ItemOpen62541::setIncomingEvent`).
    pub fn set_incoming_event(&mut self, reason: ProcessReason, status: StatusCode) {
        self.last_status = status;
        self.last_timestamp = Some(SystemTime::now());
        if reason == ProcessReason::ConnectionLoss {
            self.state = ConnectionStatus::Down;
            self.last_value = None;
            self.mapped = false;
        }
        for leaf in &self.leaves {
            let mut leaf = leaf.lock();
            if reason == ProcessReason::ConnectionLoss {
                leaf.state = ConnectionStatus::Down;
                leaf.choices = None;
                leaf.incoming = None;
            }
            leaf.push(Update::new(reason, None, status, SystemTime::now()));
        }
    }

    pub fn set_state(&mut self, state: ConnectionStatus) {
        self.state = state;
        for leaf in &self.leaves {
            leaf.lock().state = state;
        }
    }

    /// The value to write to the node: the item's last value with every dirty
    /// leaf's element replaced (`DataElementOpen62541Node::getOutgoingData`).
    ///
    /// For a leaf bound to the node itself this is just that leaf's value. For a
    /// structure it is a read-modify-write of the last value received, which is
    /// what the C's node elements assemble from their dirty children.
    pub fn take_outgoing(&mut self) -> Option<Variant> {
        if let Some(base) = self.last_value.clone() {
            self.map_elements(&base);
        }
        let mut result: Option<Variant> = None;
        for leaf in &self.leaves {
            let mut leaf = leaf.lock();
            if !leaf.dirty {
                continue;
            }
            let Some(value) = leaf.outgoing.take() else {
                leaf.dirty = false;
                continue;
            };
            leaf.dirty = false;

            if leaf.link.element_path.is_empty() {
                result = Some(value);
                continue;
            }
            let base = result
                .take()
                .or_else(|| self.last_value.clone())
                .unwrap_or(Variant::Empty);
            // Without a dictionary the value stays encoded, which the mapping
            // reported; C then writes the structure back unchanged.
            let Some(tree) = &self.type_tree else {
                result = Some(base);
                continue;
            };
            match with_element(&base, &leaf.link.element_path, value, tree) {
                Ok(updated) => result = Some(updated),
                Err(e) => {
                    if let ElementError::Encode(e) = e {
                        eprintln!("Item {}: {e}", self.node_id);
                    }
                    result = Some(base);
                }
            }
        }
        result
    }

    pub fn has_dirty_leaf(&self) -> bool {
        self.leaves.iter().any(|l| l.lock().dirty)
    }

    /// Give every leaf the enumeration its element's type defines, if it is one
    /// (`DataElementOpen62541::setEnumChoices`, driven by the node's DataType
    /// attribute and the server's type dictionary).
    ///
    /// The choices are what turns an Int32 on the wire into a state string in an
    /// mbbi, and what an mbbo's value is checked against on the way out.
    pub fn resolve_choices(&mut self) {
        let (Some(tree), Some(data_type)) = (self.type_tree.clone(), self.data_type.clone()) else {
            return;
        };
        for leaf in &self.leaves {
            let mut leaf = leaf.lock();
            if leaf.choices.is_some() {
                continue;
            }
            let type_id = element_type_id(&tree, &data_type, &leaf.link.element_path);
            leaf.choices = type_id
                .and_then(|id| tree.get_type(&id))
                .and_then(|info| match info {
                    TypeInfoRef::Enum(e) => Some(Arc::new(enum_choices(e))),
                    _ => None,
                });
        }
    }

    /// The time in the item's `@element`, if it names a DateTime element.
    fn data_timestamp(&self, data: &Variant) -> Option<SystemTime> {
        let element = &self.link.timestamp_element;
        if element.is_empty() {
            return None;
        }
        match element_of(data, std::slice::from_ref(element)) {
            Some(Variant::DateTime(dt)) => Some(system_time_of(*dt)),
            _ => None,
        }
    }

    /// `DataElementOpen62541Node::createMap` over the whole element tree, on
    /// the first value after a connect: report the `@element` timestamp source
    /// and every element the structure cannot supply.
    ///
    /// C's encoded-extension-object line has no counterpart: `async-opcua`
    /// fails the whole decode of a structure its dictionary lacks.
    fn map_elements(&mut self, data: &Variant) {
        if self.mapped {
            return;
        }
        self.mapped = true;
        self.report_timestamp_element(data);
        for line in self.element_report(data) {
            eprintln!("{line}");
        }
    }

    /// What `createMap` prints, node by node, for the elements `data` cannot
    /// supply.
    pub fn element_report(&self, data: &Variant) -> Vec<String> {
        let paths = self.element_paths();
        let mut report = Vec::new();
        for node in element_nodes(&paths) {
            // A node under one already reported has no value to check.
            let Some(value) = element_of(data, node) else {
                continue;
            };
            let mut children: Vec<&str> = Vec::new();
            for path in paths
                .iter()
                .filter(|p| p.len() > node.len() && p.starts_with(node))
            {
                if !children.contains(&path[node.len()].as_str()) {
                    children.push(&path[node.len()]);
                }
            }
            let Some(structure) = as_structure(&value) else {
                // A built-in structure type, whose elements this port cannot
                // address; reading one is a read failure.
                if matches!(value, Variant::ExtensionObject(_)) {
                    continue;
                }
                let kind = match &value {
                    Variant::Array(a) => format!("{:?}", a.value_type),
                    other => type_name(other),
                };
                report.push(format!(
                    "Error: {} is no structured data but a {kind}",
                    self.element_name(node)
                ));
                continue;
            };
            let type_id = structure.full_data_type_id().node_id;
            let type_name = self
                .type_tree
                .as_ref()
                .and_then(|tree| tree.get_struct_type(&type_id))
                .map_or_else(|| type_id.to_string(), |info| info.name.clone());
            for child in children {
                if structure.get_field_by_name(child).is_none() {
                    report.push(format!(
                        "Item {}: element {child} not found in {type_name}",
                        self.node_id
                    ));
                }
            }
        }
        report
    }

    /// The `@element` half of `createMap` (`DataElementOpen62541Node.cpp:146-169`).
    fn report_timestamp_element(&self, data: &Variant) {
        let element = &self.link.timestamp_element;
        if element.is_empty() {
            return;
        }
        let record = self.addressing_record();
        match element_of(data, std::slice::from_ref(element)) {
            Some(Variant::DateTime(_)) => {}
            Some(other) => {
                let kind = match &other {
                    Variant::Array(a) => format!("{:?}[]", a.value_type),
                    other => type_name(other),
                };
                errlog_vprintf(format_args!(
                    "{record}: timestamp element {element} has invalid type {kind} - using source timestamp\n"
                ));
            }
            None => errlog_vprintf(format_args!(
                "{record}: timestamp element {element} not found - using source timestamp\n"
            )),
        }
    }

    /// The element paths of the records bound to the node; an `opcuaItem`
    /// record is no element of its own tree.
    fn element_paths(&self) -> Vec<Vec<String>> {
        self.leaves
            .iter()
            .filter_map(|leaf| {
                let leaf = leaf.lock();
                (!leaf.link.is_item_record).then(|| leaf.link.element_path.clone())
            })
            .collect()
    }

    /// A node's full name as C's `DataElementOpen62541` prints it:
    /// `<node id>`, then ` element a.b[0]`.
    fn element_name(&self, path: &[String]) -> String {
        let mut name = self.node_id.to_string();
        for (i, step) in path.iter().enumerate() {
            if i == 0 {
                name += " element ";
            } else if !step.starts_with('[') {
                name.push('.');
            }
            name += step;
        }
        name
    }

    /// `ItemOpen62541::show`: the item's line, then from level 1 its data tree.
    pub fn show(&self, level: i32) -> String {
        let mut out = self.show_line() + "\n";
        if level >= 1
            && let Some(root) = self.data_tree()
        {
            root.show(self.mapped, 1, &mut out);
        }
        out
    }

    /// `ElementTree::addLeaf` over the element records, in the order they
    /// bound: a record bound to the whole node is the root leaf, otherwise a
    /// `[ROOT]` node holds a node per path step and the record's leaf last.
    fn data_tree(&self) -> Option<Element> {
        let mut root = None;
        for leaf in &self.leaves {
            let path = {
                let locked = leaf.lock();
                if locked.link.is_item_record {
                    continue;
                }
                locked.link.element_path.clone()
            };
            let Some((name, nodes)) = path.split_last() else {
                root.get_or_insert_with(|| Element::Leaf {
                    name: "[ROOT]".to_string(),
                    leaf: leaf.clone(),
                });
                continue;
            };
            let mut children = match root.get_or_insert_with(|| Element::node("[ROOT]")) {
                Element::Node { children, .. } => children,
                Element::Leaf { .. } => continue,
            };
            for step in nodes {
                let at = match children
                    .iter()
                    .position(|c| matches!(c, Element::Node { name, .. } if name == step))
                {
                    Some(at) => at,
                    None => {
                        children.push(Element::node(step));
                        children.len() - 1
                    }
                };
                let Element::Node { children: next, .. } = &mut children[at] else {
                    unreachable!("the position matched a node");
                };
                children = next;
            }
            children.push(Element::Leaf {
                name: name.clone(),
                leaf: leaf.clone(),
            });
        }
        root
    }

    /// `ItemOpen62541::show(0)`'s line.
    pub fn show_line(&self) -> String {
        let link = &self.link;
        let mut out = String::from("item ns=");
        if self.node_id.namespace != link.namespace_index {
            out += &format!("{}({})", self.node_id.namespace, link.namespace_index);
        } else {
            out += &link.namespace_index.to_string();
        }
        match &link.identifier {
            Some(NodeIdentifier::Numeric(n)) => out += &format!(";i={n}"),
            Some(NodeIdentifier::String(s)) => out += &format!(";s={s}"),
            None => out += ";s=",
        }
        out += &format!(
            " record={} state={} status={} dataDirty={} context={}@{} sampling={}({}) \
             deadband={} qsize={}({}) cqsize={} discard={} timestamp={}",
            self.addressing_record(),
            self.state.as_str(),
            status_name(self.last_status),
            yn(self.has_dirty_leaf()),
            link.subscription().unwrap_or(""),
            self.session.as_ref().map_or("", |s| s.name.as_str()),
            stream_double(self.revised_sampling_interval),
            stream_double(link.sampling_interval),
            stream_double(link.deadband),
            self.revised_queue_size,
            link.queue_size,
            link.client_queue_size,
            if link.discard_oldest { "old" } else { "new" },
            link.timestamp.as_str(),
        );
        if link.timestamp == TimestampSource::Data {
            out += &format!("@{}", link.timestamp_element);
        }
        out += &format!(
            " bini={} output={} monitor={} registered=",
            link.bini.as_str(),
            yn(link.is_output),
            yn(link.monitor),
        );
        match &self.registered_node_id {
            Some(id) => out += &id.to_string(),
            None => out += "-",
        }
        out += &format!("({})", yn(link.register_node));
        let data_leaves = self.element_paths();
        let nodes = element_nodes(&data_leaves);
        if !(nodes.is_empty() && data_leaves.len() == 1) {
            out += &format!(" dataNodes={} dataLeafs={}", nodes.len(), data_leaves.len());
        }
        out
    }

    /// The record whose link addresses the item's node (C `recConnector`).
    pub fn addressing_record(&self) -> String {
        self.leaves
            .iter()
            .find_map(|leaf| {
                let leaf = leaf.lock();
                leaf.link
                    .item_record()
                    .is_none()
                    .then(|| leaf.record.clone())
            })
            .unwrap_or_default()
    }
}

/// The data type of what an element path addresses: the node's own data type for
/// an empty path, else the type of the structure field the path walks to.
fn element_type_id(tree: &DataTypeTree, data_type: &NodeId, path: &[String]) -> Option<NodeId> {
    let mut current = data_type.clone();
    for name in path {
        let structure = tree.get_struct_type(&current)?;
        let index = *structure.index_by_name.get(name)?;
        current = structure.fields.get(index)?.type_id.clone();
    }
    Some(current)
}

/// An OPC UA enumeration's `(value, name)` pairs, as the record's state values
/// and strings. A value outside `epicsUInt32` cannot be a state value, so it is
/// dropped rather than wrapped.
fn enum_choices(info: &EnumTypeInfo) -> EnumChoices {
    info.variants
        .iter()
        .filter_map(|(value, field)| {
            let value = u32::try_from(*value).ok()?;
            Some((value, field.name.as_ref().to_string()))
        })
        .collect()
}

fn system_time_of(dt: DateTime) -> SystemTime {
    let chrono = dt.as_chrono();
    let secs = chrono.timestamp();
    let nanos = chrono.timestamp_subsec_nanos();
    if secs >= 0 {
        UNIX_EPOCH + Duration::new(secs as u64, nanos)
    } else {
        UNIX_EPOCH - Duration::new((-secs) as u64, 0) + Duration::from_nanos(u64::from(nanos))
    }
}

/// `ItemOpen62541::getStatus` / `uaToEpicsTime` — which of the value's
/// timestamps the record takes. An unusable `@element` falls back to the
/// source timestamp, as C's `tsData = tsSource` does.
fn timestamp_for(link: &LinkInfo, value: &DataValue, data_time: Option<SystemTime>) -> SystemTime {
    let pick = |dt: Option<DateTime>| dt.map(system_time_of);
    match link.timestamp {
        TimestampSource::Server => pick(value.server_timestamp),
        TimestampSource::Source => pick(value.source_timestamp),
        TimestampSource::Data => data_time.or_else(|| pick(value.source_timestamp)),
    }
    .unwrap_or_else(SystemTime::now)
}

/// Walk an element path into a structured value.
///
/// An empty path is the value itself. Each further name selects a field of a
/// structure (or the active field of a union) — the `element=a.b.c` link option.
pub fn element_of(value: &Variant, path: &[String]) -> Option<Variant> {
    let mut current = value.clone();
    for name in path {
        if name.is_empty() {
            continue;
        }
        let structure = as_structure(&current)?;
        current = structure.get_field_by_name(name)?.clone();
    }
    Some(current)
}

fn as_structure(value: &Variant) -> Option<&DynamicStructure> {
    match value {
        Variant::ExtensionObject(obj) => obj.inner_as::<DynamicStructure>(),
        _ => None,
    }
}

/// Why [`with_element`] left a structure unchanged.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ElementError {
    /// The structure has no such element; the mapping reported it.
    Unmapped,
    /// The rebuilt structure did not encode.
    Encode(String),
}

/// Replace what an element path addresses, leaving the rest of the structure as
/// the server last sent it — the read-modify-write the C's node elements do when
/// they assemble outgoing data from their dirty children.
///
/// `DynamicStructure` keeps its type definition and type tree private, so the
/// structure is rebuilt from the tree the session loaded, keyed by the value's
/// own data type id.
pub fn with_element(
    base: &Variant,
    path: &[String],
    value: Variant,
    tree: &Arc<DataTypeTree>,
) -> Result<Variant, ElementError> {
    let Some((name, rest)) = path.split_first() else {
        return Ok(value);
    };
    let structure = as_structure(base).ok_or(ElementError::Unmapped)?;
    let type_id = structure.full_data_type_id().node_id;
    let type_def = tree
        .get_struct_type(&type_id)
        .ok_or(ElementError::Unmapped)?;
    let index = *type_def
        .index_by_name
        .get(name)
        .ok_or(ElementError::Unmapped)?;

    let updated = if matches!(type_def.structure_type, StructureType::Union) {
        // A union holds one field at a time; writing a member selects it.
        let field = structure
            .get_field_by_name(name)
            .cloned()
            .unwrap_or(Variant::Empty);
        DynamicStructure::new_union(
            type_def.clone(),
            tree.clone(),
            with_element(&field, rest, value, tree)?,
            index as u32 + 1,
        )
    } else {
        let mut fields: Vec<Variant> = structure.values().to_vec();
        let field = fields.get(index).cloned().ok_or(ElementError::Unmapped)?;
        fields[index] = with_element(&field, rest, value, tree)?;
        DynamicStructure::new_struct(type_def.clone(), tree.clone(), fields)
    }
    .map_err(|e| ElementError::Encode(e.to_string()))?;

    Ok(Variant::ExtensionObject(ExtensionObject::from_message(
        updated,
    )))
}

/// The structure nodes of C's element tree for these leaf paths: `[ROOT]` and
/// one node per further step, in tree order.
/// One element of an item's data tree, as `opcuaShow` walks it.
enum Element {
    Node {
        name: String,
        children: Vec<Element>,
    },
    Leaf {
        name: String,
        leaf: Arc<Mutex<Leaf>>,
    },
}

impl Element {
    fn node(name: &str) -> Self {
        Element::Node {
            name: name.to_string(),
            children: Vec::new(),
        }
    }

    /// `DataElementOpen62541Node::show` / `DataElementOpen62541Leaf::show`.
    /// The port keeps one `mapped` flag per item, where the C keeps one per
    /// node; every node of a tree maps on the same update.
    fn show(&self, mapped: bool, indent: usize, out: &mut String) {
        let ind = " ".repeat(indent * 2);
        match self {
            Element::Node { name, children } => {
                *out += &format!(
                    "{ind}node={name} children={} mapped={}\n",
                    children.len(),
                    yn(mapped)
                );
                for child in children {
                    child.show(mapped, indent + 1, out);
                }
            }
            Element::Leaf { name, leaf } => {
                let leaf = leaf.lock();
                *out += &format!(
                    "{ind}leaf={name} record({})={} type={} timestamp={} bini={} monitor={}\n",
                    leaf.record_type,
                    leaf.record,
                    leaf.incoming
                        .as_ref()
                        .map_or_else(|| "Null".to_string(), type_name),
                    leaf.link.timestamp.as_str(),
                    leaf.link.bini.as_str(),
                    yn(leaf.link.monitor),
                );
            }
        }
    }
}

fn element_nodes(paths: &[Vec<String>]) -> Vec<&[String]> {
    let mut nodes: Vec<&[String]> = paths
        .iter()
        .filter(|path| !path.is_empty())
        .flat_map(|path| (0..path.len()).map(move |n| &path[..n]))
        .collect();
    nodes.sort();
    nodes.dedup();
    nodes
}

fn yn(flag: bool) -> &'static str {
    if flag { "y" } else { "n" }
}

/// A double as `std::cout << v` prints it: `%g` at the default precision 6.
pub(crate) fn stream_double(v: f64) -> String {
    fmt_g(v, 6, false, false)
}
