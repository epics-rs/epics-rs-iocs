//! The gate that holds a poll thread's first parameter flush until the
//! records exist.
//!
//! Every `*Config` command runs from st.cmd *before* `iocInit()`, and each one
//! starts its driver's poll thread. That thread's first
//! `callParamCallbacks` therefore fires with no record subscribed to anything,
//! and the flush consumes the changed flags it found (`ParamList::take_changed`
//! clears them). `set_int32` / `set_float64` raise the flag again only when the
//! value actually differs (C `paramVal::setDouble`, paramVal.cpp:241-252), so a
//! parameter that holds still after that first poll — `safety_mode`,
//! `runtime_state`, the digital-bit words, `target_speed_fraction`,
//! `actual_momentum`, the analog I/O, every dashboard string — is never flushed
//! again and its record stays UDF/INVALID for the life of the IOC. Only the
//! continuously-moving values (timestamp, voltages, currents, joint angles)
//! looked healthy.
//!
//! So the poll threads wait for `initHookAfterScanInit` before their first
//! pass, which is where drvMqtt puts its own first subscribe
//! (drvMqtt.cpp:124,186-189).
//!
//! [`arm`] registers the hook on the caller's thread and hands back the handle
//! the poll thread waits on, so the registration cannot lose a race against
//! `iocInit()`: there is no way to obtain something to wait on without having
//! registered the hook first, and `arm` is called from the `*Config` command
//! itself.

use std::sync::{Arc, Condvar, Mutex, OnceLock};

use epics_rs::base::server::ioc_app::{InitHookState, init_hook_register};

/// A one-way gate: closed until `initHookAfterScanInit`, open ever after.
#[derive(Default)]
pub struct IocReady {
    open: Mutex<bool>,
    opened: Condvar,
}

impl IocReady {
    /// Block until `iocInit` has finished its scan pass. Returns at once once
    /// the gate is open.
    pub fn wait(&self) {
        let mut open = self.open.lock().unwrap();
        while !*open {
            open = self.opened.wait(open).unwrap();
        }
    }

    fn open(&self) {
        *self.open.lock().unwrap() = true;
        self.opened.notify_all();
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
        let gate = Arc::new(IocReady::default());
        assert!(!*gate.open.lock().unwrap());
        let waiter = {
            let gate = gate.clone();
            std::thread::spawn(move || gate.wait())
        };
        gate.open();
        waiter.join().expect("waiter returns once the gate opens");
    }

    #[test]
    fn waiting_on_an_open_gate_returns_at_once() {
        let gate = IocReady::default();
        gate.open();
        gate.wait();
    }
}
