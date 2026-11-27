//! devVacSen's callback ends its record's pass with `dbScanLock; process;
//! dbScanUnlock` on the port thread (`devVacSen.c:996-998`), not through
//! `callbackRequest`, so a full callback band cannot leave the record PACT.
//! One test per binary: the band's size and thread count are set once per
//! process, before `callbackInit`.

use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::manager::{PortManager, claim_port_driver};
use epics_rs::asyn::registry::open_owner;
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

/// A gauge on a TCP socket that answers each command with a GP307 error
/// reply, which ends devVacSen's read loop at once. Counts the commands.
fn gauge() -> (String, Arc<Mutex<usize>>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let asked = Arc::new(Mutex::new(0));
    let count = asked.clone();
    std::thread::spawn(move || {
        let (mut s, _) = listener.accept().unwrap();
        let mut buf = [0; 256];
        while let Ok(1..) = s.read(&mut buf) {
            *count.lock().unwrap() += 1;
            if s.write_all(b"SYNTX ERROR").is_err() {
                break;
            }
        }
    });
    (addr, asked)
}

#[test]
fn a_read_on_a_blocking_port_completes_while_its_band_is_full() {
    // `callbackSetQueueSize` and `callbackParallelThreads` before
    // `callbackInit`; the record's PRIO is LOW.
    set_queue_size(1);
    set_parallel_threads(1, Some(CallbackPriority::Low));
    background_init();
    let (addr, asked) = gauge();
    let _port = PortManager::new()
        .register_port(
            claim_port_driver("VSFULL").unwrap(),
            DrvAsynIPPort::new("VSFULL", &addr).unwrap(),
            &open_owner(),
        )
        .unwrap();
    let db = format!(
        r#"
record(vs, "VSFULL:G") {{
    field(DTYP, "{DTYP}")
    field(INP, "@asyn(VSFULL 0)")
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
        db.put_record_field_from_ca_no_notify("VSFULL:G", "PROC", EpicsValue::Short(1))
            .await
            .unwrap();
        let pact = || db.get_record("VSFULL:G").unwrap().read().is_processing();
        for _ in 0..400 {
            if *asked.lock().unwrap() > 0 && !pact() {
                break;
            }
            sleep(Duration::from_millis(5)).await;
        }
        assert!(*asked.lock().unwrap() > 0, "the gauge was never asked");
        assert!(!pact(), "PACT after the exchange");
    });
}
