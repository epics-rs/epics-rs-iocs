//! Single-slot hand-off from `initScaler974` (which constructs a
//! [`Scaler974Driver`](crate::driver::Scaler974Driver)) to the IOC's
//! `register_dynamic_device_support` closure (which binds it to a
//! `scalerRecord` at iocInit).
//!
//! # Framework gap: a record can't be matched by its OUT link's port name
//! C `devScalerAsyn.c::scaler_init_record` distinguishes multiple boards
//! by parsing each record's own `@asyn(portName,addr)` OUT link. The Rust
//! equivalent, `register_dynamic_device_support`'s
//! `DeviceSupportContext { dtyp, inp, out }`, exposes `out` by borrowing
//! `RecordCommon.out` -- but `db_loader::apply_fields`
//! (`epics-base-rs-0.22.1/src/server/db_loader/mod.rs:1089`) only stores a
//! field there when the record type has NO matching entry in its own
//! `field_list()`. `scaler_rs::records::scaler::ScalerRecord` declares its
//! own `"OUT"` `FieldDesc` (mirroring real `scalerRecord.dbd`'s private
//! `DBLINK out`), so `field(OUT,"...")` always routes to
//! `ScalerRecord::set_field`/`self.out` instead, and `RecordCommon.out` --
//! and therefore `ctx.out` -- is **always empty** for a `scalerRecord`.
//! `ctx.dtyp` is the only field left, and it's the same fixed constant
//! (`"Asyn Scaler"`) for every scaler974 instance: nothing in the current
//! `DeviceSupportContext` can tell two `scalerRecord`s apart at bind time.
//!
//! A call-order (FIFO) fallback was considered and rejected: `PvDatabase`
//! stores records in a plain `HashMap`
//! (`server/database/mod.rs::PvDatabaseInner::records`), and
//! `wire_device_support` walks it via `all_record_names()` -- iteration
//! order is not insertion order and isn't stable across runs, so a FIFO
//! match would silently bind the wrong driver to the wrong record on some
//! fraction of IOC boots. That failure mode is worse than refusing
//! multi-instance outright.
//!
//! Given that, this registry supports **exactly one** pending Scaler974
//! instance at a time: [`register`] fails loudly if a previously
//! registered driver hasn't been claimed yet, rather than silently
//! accepting a second `initScaler974` call whose driver could never be
//! correctly disambiguated from the first. A single scaler974-ioc process
//! hosts one Ortec 974 board. Multiple boards per IOC would need an
//! epics-base-rs change exposing a record's own field storage (not just
//! `RecordCommon`) through `DeviceSupportContext`.

use std::sync::{Mutex, OnceLock};

use epics_rs::scaler::device_support::scaler_asyn::ScalerDriver;

type Slot = Option<(String, Box<dyn ScalerDriver>)>;

fn slot() -> &'static Mutex<Slot> {
    static SLOT: OnceLock<Mutex<Slot>> = OnceLock::new();
    SLOT.get_or_init(|| Mutex::new(None))
}

/// Register `driver` for `port_name`, called once by `initScaler974`.
/// Returns `Err` if a previously registered driver hasn't been claimed
/// yet by a `scalerRecord` binding (see the module doc).
pub fn register(port_name: &str, driver: Box<dyn ScalerDriver>) -> Result<(), String> {
    let mut guard = slot().lock().unwrap();
    if let Some((existing, _)) = guard.as_ref() {
        return Err(format!(
            "scaler974: port '{existing}' is still unclaimed; only one Scaler974 instance \
             can be pending at a time (see registry module doc)"
        ));
    }
    *guard = Some((port_name.to_string(), driver));
    Ok(())
}

/// Take the sole pending driver, called once by the
/// `register_dynamic_device_support` closure when a `scalerRecord` with
/// DTYP "Asyn Scaler" is wired. `None` if no `initScaler974` call is
/// currently pending.
pub fn take() -> Option<Box<dyn ScalerDriver>> {
    slot().lock().unwrap().take().map(|(_, driver)| driver)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex as StdMutex;

    // registry() is a process-global static; serialize tests that touch it.
    static TEST_LOCK: StdMutex<()> = StdMutex::new(());

    struct StubDriver;
    impl ScalerDriver for StubDriver {
        fn reset(&mut self) -> epics_rs::base::error::CaResult<()> {
            Ok(())
        }
        fn read(
            &mut self,
            _counts: &mut [u32; epics_rs::scaler::MAX_SCALER_CHANNELS],
        ) -> epics_rs::base::error::CaResult<()> {
            Ok(())
        }
        fn write_preset(
            &mut self,
            _channel: usize,
            preset: u32,
        ) -> epics_rs::base::error::CaResult<u32> {
            Ok(preset)
        }
        fn arm(&mut self, _start: bool) -> epics_rs::base::error::CaResult<()> {
            Ok(())
        }
        fn done(&mut self) -> bool {
            true
        }
        fn num_channels(&self) -> usize {
            4
        }
    }

    #[test]
    fn register_then_take_round_trips() {
        let _guard = TEST_LOCK.lock().unwrap();
        *slot().lock().unwrap() = None;

        register("SCL1", Box::new(StubDriver)).unwrap();
        assert!(take().is_some());
        assert!(take().is_none());
    }

    #[test]
    fn register_twice_without_take_fails_loudly() {
        let _guard = TEST_LOCK.lock().unwrap();
        *slot().lock().unwrap() = None;

        register("SCL1", Box::new(StubDriver)).unwrap();
        let err = register("SCL2", Box::new(StubDriver)).unwrap_err();
        assert!(err.contains("SCL1"));

        // Clean up so other tests in this process see an empty slot.
        take();
    }
}
