//! On a port without `ASYN_CANBLOCK`, `queueRequest` runs devVacSen's callback
//! inside the record's own pass (asynManager.c:1555-1584), so that pass
//! finishes there and no full callback band can leave the record PACT. One
//! test per binary: the band's size and thread count are set once per
//! process, before `callbackInit`.

use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
use epics_rs::asyn::manager::{PortManager, claim_port_driver};
use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::registry::open_owner;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::background::callback_executor::{
    set_parallel_threads, set_queue_size,
};
use epics_rs::base::runtime::task::{
    CallbackPriority, background_callback_stats, background_init, sleep, spawn_blocking_background,
    test_block_on,
};
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::types::EpicsValue;
use vac::device_support::vac_sen::DTYP;
use vac::ioc::{device_support_factory, vs_record_factory};

/// Holds the low band's one worker until its `FullBand` drops.
#[derive(Clone, Default)]
struct Hold(Arc<(Mutex<bool>, Condvar)>);

impl Hold {
    fn wait(&self) {
        let (open, wake) = &*self.0;
        let mut open = open.lock().unwrap();
        while !*open {
            open = wake.wait(open).unwrap();
        }
    }
}

struct FullBand(Hold);

impl Drop for FullBand {
    fn drop(&mut self) {
        let (open, wake) = &*(self.0).0;
        *open.lock().unwrap() = true;
        wake.notify_all();
    }
}

fn low_overflows() -> u64 {
    background_callback_stats(false).unwrap()[CallbackPriority::Low.index()].num_overflow
}

/// Hold the low band's one worker, then take its one ring slot with ad-hoc
/// requests until one is refused: a `CallbackCell` is never refused.
fn fill_low_band() -> FullBand {
    let hold = Hold::default();
    let (running_tx, running_rx) = std::sync::mpsc::channel();
    let holder = hold.clone();
    drop(spawn_blocking_background(
        CallbackPriority::Low,
        move |_| {
            running_tx.send(()).unwrap();
            holder.wait();
        },
    ));
    running_rx.recv_timeout(Duration::from_secs(5)).unwrap();
    let refused = low_overflows() + 1;
    for _ in 0..64 {
        let filler = hold.clone();
        drop(spawn_blocking_background(
            CallbackPriority::Low,
            move |_| filler.wait(),
        ));
        if low_overflows() >= refused {
            return FullBand(hold);
        }
    }
    panic!("the low band never filled");
}

/// A gauge on a port without `ASYN_CANBLOCK` that answers each command with
/// a GP307 error reply, which ends devVacSen's read loop at once. Counts the
/// commands.
struct Gauge {
    base: PortDriverBase,
    asked: Arc<Mutex<usize>>,
}

impl PortDriver for Gauge {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
        let reply = b"SYNTX ERROR";
        let n = reply.len().min(buf.len());
        buf[..n].copy_from_slice(&reply[..n]);
        OctetReadResult::ok(n, EomReason::END)
    }
    fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        *self.asked.lock().unwrap() += 1;
        Ok(data.len())
    }
}

#[test]
fn a_read_on_a_port_that_cannot_block_ends_while_its_band_is_full() {
    // `callbackSetQueueSize` and `callbackParallelThreads` before
    // `callbackInit`; the record's PRIO is LOW.
    set_queue_size(1);
    set_parallel_threads(1, Some(CallbackPriority::Low));
    background_init();
    let mut base = PortDriverBase::new(
        "VSSYNCFULL",
        1,
        PortFlags {
            can_block: false,
            ..PortFlags::default()
        },
        InterfaceMask::COMMON | InterfaceMask::OCTET,
        InterfaceMask::NONE,
    );
    base.init_connected(true);
    let asked = Arc::new(Mutex::new(0));
    let _port = PortManager::new()
        .register_port(
            claim_port_driver("VSSYNCFULL").unwrap(),
            Gauge {
                base,
                asked: asked.clone(),
            },
            &open_owner(),
        )
        .unwrap();
    let db = format!(
        r#"
record(vs, "VSSYNCFULL:G") {{
    field(DTYP, "{DTYP}")
    field(INP, "@asyn(VSSYNCFULL 0)")
}}
"#
    );
    let (vs, vs_factory) = vs_record_factory();
    test_block_on(async {
        let (db, _) = IocBuilder::new()
            .register_record_type(vs, move || vs_factory())
            .register_dynamic_device_support(&[], device_support_factory())
            .db_string(&db, &HashMap::new())
            .unwrap()
            .build()
            .await
            .unwrap();
        let _full = fill_low_band();
        let before = *asked.lock().unwrap();
        db.put_record_field_from_ca_no_notify("VSSYNCFULL:G", "PROC", EpicsValue::Short(1))
            .await
            .unwrap();
        let pact = || {
            db.get_record("VSSYNCFULL:G")
                .unwrap()
                .read()
                .is_processing()
        };
        for _ in 0..100 {
            if !pact() {
                break;
            }
            sleep(Duration::from_millis(5)).await;
        }
        assert!(*asked.lock().unwrap() > before, "the gauge was never asked");
        assert!(!pact(), "PACT after the exchange");
    });
}
