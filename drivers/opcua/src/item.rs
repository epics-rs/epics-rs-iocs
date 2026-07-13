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

use async_opcua::types::custom::{DataTypeTree, DynamicStructure};
use async_opcua::types::{
    DataValue, DateTime, ExpandedMessageInfo, ExtensionObject, NodeId, StatusCode, StructureType,
    Variant,
};
use parking_lot::Mutex;
use tokio::sync::mpsc;

use crate::link::{Bini, LinkInfo, TimestampSource};
use crate::queue::{ConnectionStatus, ProcessReason, Update, UpdateQueue};
use crate::value::EnumChoices;

/// One record's binding to an item (`DataElementOpen62541Leaf` +
/// `RecordConnector`).
#[derive(Debug)]
pub struct Leaf {
    pub record: String,
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
    pub fn new(record: String, link: LinkInfo, notify: mpsc::Sender<()>) -> Self {
        let queue = UpdateQueue::new(link.client_queue_size as usize, link.discard_oldest);
        Self {
            record,
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
    /// The client handle its monitored item was created with.
    pub client_handle: u32,
    /// Every record bound to this node.
    pub leaves: Vec<Arc<Mutex<Leaf>>>,
    /// The item's own last value, from which every leaf's element is taken.
    pub last_value: Option<Variant>,
    /// The server's data type dictionary, needed to rebuild a structure around a
    /// changed element.
    pub type_tree: Option<Arc<DataTypeTree>>,
}

impl Item {
    pub fn new(link: LinkInfo, node_id: NodeId, client_handle: u32) -> Self {
        Self {
            link,
            configured_node_id: node_id.clone(),
            node_id,
            registered_node_id: None,
            data_type: None,
            state: ConnectionStatus::Down,
            client_handle,
            leaves: Vec::new(),
            last_value: None,
            type_tree: None,
        }
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

        for leaf in &self.leaves {
            let mut leaf = leaf.lock();
            // `bini=ignore` throws the initial read away, but the item still
            // needs it to learn the node's type for later writes
            // (`ItemOpen62541.cpp:150-170`).
            let element = element_of(&data, &leaf.link.element_path);
            if let Some(element) = &element {
                leaf.incoming = Some(element.clone());
            }

            if leaf.state == ConnectionStatus::InitialRead && leaf.link.bini == Bini::Ignore {
                leaf.state = ConnectionStatus::Up;
                continue;
            }

            let timestamp = self.timestamp_for(&leaf.link, &value, &data);
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
        if reason == ProcessReason::ConnectionLoss {
            self.state = ConnectionStatus::Down;
            self.last_value = None;
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
            let Some(tree) = &self.type_tree else {
                log::error!(
                    "{}: cannot write element: the server's type tree is not loaded",
                    leaf.record
                );
                result = Some(base);
                continue;
            };
            match with_element(&base, &leaf.link.element_path, value, tree) {
                Ok(updated) => result = Some(updated),
                Err(e) => {
                    log::error!("{}: cannot write element: {e}", leaf.record);
                    result = Some(base);
                }
            }
        }
        result
    }

    pub fn has_dirty_leaf(&self) -> bool {
        self.leaves.iter().any(|l| l.lock().dirty)
    }

    /// `ItemOpen62541::getStatus` / `uaToEpicsTime` — which of the value's
    /// timestamps the record takes.
    fn timestamp_for(&self, link: &LinkInfo, value: &DataValue, data: &Variant) -> SystemTime {
        let pick = |dt: Option<DateTime>| dt.map(system_time_of);
        match link.timestamp {
            TimestampSource::Server => pick(value.server_timestamp),
            TimestampSource::Source => pick(value.source_timestamp),
            TimestampSource::Data => {
                let element = &self.link.timestamp_element;
                match element_of(data, std::slice::from_ref(element)) {
                    Some(Variant::DateTime(dt)) => Some(system_time_of(*dt)),
                    _ => {
                        log::warn!(
                            "item {}: element '{element}' is not a DateTime; using the server time",
                            self.node_id
                        );
                        pick(value.server_timestamp)
                    }
                }
            }
        }
        .unwrap_or_else(SystemTime::now)
    }
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
) -> Result<Variant, String> {
    let Some((name, rest)) = path.split_first() else {
        return Ok(value);
    };
    let structure =
        as_structure(base).ok_or_else(|| format!("'{name}' is not inside a structure"))?;
    let type_id = structure.full_data_type_id().node_id;
    let type_def = tree
        .get_struct_type(&type_id)
        .ok_or_else(|| format!("the type of element '{name}' is not in the server's type tree"))?;
    let index = *type_def
        .index_by_name
        .get(name)
        .ok_or_else(|| format!("the structure has no element '{name}'"))?;

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
        let field = fields
            .get(index)
            .cloned()
            .ok_or_else(|| format!("the structure has no element '{name}'"))?;
        fields[index] = with_element(&field, rest, value, tree)?;
        DynamicStructure::new_struct(type_def.clone(), tree.clone(), fields)
    }
    .map_err(|e| e.to_string())?;

    Ok(Variant::ExtensionObject(ExtensionObject::from_message(
        updated,
    )))
}
