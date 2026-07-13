//! EPICS Device Support for OPC UA.
//!
//! Port of `epics-modules/opcua` — the client-library-agnostic layer under
//! `devOpcuaSup/` (link grammar, record device support, item/element tree,
//! session and subscription lifecycle) — onto the pure-Rust `async-opcua`
//! client, which takes the place of both C client backends (Unified Automation
//! SDK and open62541). The open62541 backend is the behavioural reference.

pub mod client;
pub mod defaults;
pub mod device_support;
pub mod item;
pub mod link;
pub mod queue;
pub mod record;
pub mod registry;
pub mod session;
pub mod subscription;
pub mod value;
