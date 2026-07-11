//! Per-address model table, shared between `LoveInit` (which creates the
//! port and its [`LoveDriver`](crate::driver::LoveDriver)) and `LoveConfig`
//! (a *separate* iocsh command, called afterwards, that must reach back into
//! the already-running driver instance and set the model for one address).
//!
//! C `drvLoveConfig` does this by walking a global linked list of `Port*`
//! structs (`pports`) to find the instance by name and mutating
//! `pport->instr[addr-1].modidx` directly. asyn-rs 0.22.1's port runtime
//! gives external code only a message-passing `PortHandle`/
//! `PortRuntimeHandle` back for a registered port name — the driver itself
//! is moved into its actor and is not reachable (no `Arc<Mutex<dyn
//! PortDriver>>`, no `Any` downcast, no custom-command extension point in
//! the actor's request enum). This module is the Rust substitute: a
//! name-keyed registry of `Arc<Mutex<[Model; K_INSTRMAX]>>` tables,
//! constructed by `LoveInit` *before* `create_port_runtime`, with one clone
//! held by the driver and another registered here for `LoveConfig` to find.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

/// C `#define K_INSTRMAX (256)` — controller addresses are 1..=256, indexed
/// `instr[addr-1]`.
pub const K_INSTRMAX: usize = 256;

/// C `typedef enum {model1600,model16A} Model` — `model1600` is `0`, the
/// zero-init default (C `Instr[]` comes from `callocMustSucceed`, so every
/// address starts as `model1600` until `drvLoveConfig` runs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Model {
    #[default]
    Model1600 = 0,
    Model16A = 1,
}

impl Model {
    /// C `epicsStrCaseCmp(model,"1600")`/`epicsStrCaseCmp(model,"16A")` in
    /// `drvLoveConfig` — `None` for any other string (C prints "unsupported
    /// model" and returns `-1`).
    pub fn parse(s: &str) -> Option<Model> {
        if s.eq_ignore_ascii_case("1600") {
            Some(Model::Model1600)
        } else if s.eq_ignore_ascii_case("16A") {
            Some(Model::Model16A)
        } else {
            None
        }
    }
}

pub type ModelTable = Arc<Mutex<[Model; K_INSTRMAX]>>;

fn registry() -> &'static Mutex<HashMap<String, ModelTable>> {
    static REGISTRY: OnceLock<Mutex<HashMap<String, ModelTable>>> = OnceLock::new();
    REGISTRY.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Register `table` under `port_name`, called once by `LoveInit`.
pub fn register(port_name: &str, table: ModelTable) {
    registry()
        .lock()
        .unwrap()
        .insert(port_name.to_string(), table);
}

/// Look up the model table for an already-`LoveInit`-ed port, called by
/// `LoveConfig`. C: the `pports` linked-list walk in `drvLoveConfig` that
/// fails with `"drvLoveConfig::failure to locate port %s\n"` when no match
/// is found.
pub fn lookup(port_name: &str) -> Option<ModelTable> {
    registry().lock().unwrap().get(port_name).cloned()
}
