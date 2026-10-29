//! The gate that holds a poll thread's parameter flushes until the records
//! exist.
//!
//! Every `*Config` command runs from st.cmd *before* `iocInit()`, and each one
//! starts its driver's poll thread. A `callParamCallbacks` before the records
//! subscribe consumes the changed flags it found (`ParamList::take_changed`
//! clears them), and `set_int32` / `set_float64` raise the flag again only
//! when the value differs (C `paramVal::setDouble`, paramVal.cpp:241-252), so
//! a parameter that holds still after that flush — `safety_mode`, the
//! digital-bit words, every dashboard string — would stay UDF for the life of
//! the IOC.
//!
//! The poll threads still start polling at construction, as the C++ pollers
//! do: `RTDEControlConfig` and `URGripperConfig` read the safety word and the
//! robot mode those first polls produce. Only the parameter updates and the
//! COMM-alarm edges wait for `initHookAfterScanInit`, where drvMqtt puts its
//! own first subscribe (drvMqtt.cpp:124,186-189). Every cycle carries the full
//! parameter set, so the first open cycle publishes everything.
//!
//! [`arm`] registers the hook on the caller's thread, so the registration
//! cannot lose a race against `iocInit()`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, OnceLock};

use epics_rs::base::server::ioc_app::{InitHookState, init_hook_register};

/// A one-way gate: closed until `initHookAfterScanInit`, open ever after.
#[derive(Default)]
pub struct IocReady(AtomicBool);

impl IocReady {
    /// Whether `iocInit` has finished its scan pass.
    pub fn is_open(&self) -> bool {
        self.0.load(Ordering::Acquire)
    }

    pub(crate) fn open(&self) {
        self.0.store(true, Ordering::Release);
    }
}

/// The process-wide gate, registering its init hook on first use.
pub fn arm() -> Arc<IocReady> {
    static GATE: OnceLock<Arc<IocReady>> = OnceLock::new();
    GATE.get_or_init(|| {
        let gate = Arc::new(IocReady::default());
        let hooked = gate.clone();
        init_hook_register(Arc::new(move |state| {
            if state == InitHookState::AfterScanInit {
                hooked.open();
            }
        }));
        gate
    })
    .clone()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_gate_stays_shut_until_it_is_opened() {
        let gate = IocReady::default();
        assert!(!gate.is_open());
        gate.open();
        assert!(gate.is_open());
    }
}
