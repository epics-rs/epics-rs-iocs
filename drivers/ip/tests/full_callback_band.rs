//! devXxStrParm's callback ends its record's pass with `dbScanLock;
//! process; dbScanUnlock` on the port thread (`devXxStrParm.c:414-416`), not
//! through `callbackRequest`, so a full callback band cannot leave the record
//! PACT. One test per binary: the band's size and thread count are set once
//! per process, before `callbackInit`.

use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use asyn_rs::drivers::ip_port::DrvAsynIPPort;
use asyn_rs::interpose::eos::EosInterpose;
use asyn_rs::manager::{PortManager, claim_port_driver};
use asyn_rs::port::PortDriver;
use asyn_rs::registry::open_owner;
use epics_rs::base::runtime::background::callback_executor::{
    set_parallel_threads, set_queue_size,
};
use epics_rs::base::runtime::task::{
    CallbackPriority, background_callback_stats, background_init, sleep, spawn_blocking_background,
    test_block_on,
};
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::types::EpicsValue;
use ip_devices::strparm::{DEVICE_ROWS, device_support_factory};

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

/// A device on a TCP socket that sends `line` when connected and keeps
/// what it is sent.
fn device(line: &'static [u8]) -> (String, Arc<Mutex<Vec<u8>>>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let got = Arc::new(Mutex::new(Vec::new()));
    let sink = got.clone();
    std::thread::spawn(move || {
        let (mut s, _) = listener.accept().unwrap();
        s.write_all(line).unwrap();
        let mut buf = [0; 256];
        while let Ok(n @ 1..) = s.read(&mut buf) {
            sink.lock().unwrap().extend_from_slice(&buf[..n]);
        }
    });
    (addr, got)
}

/// An `ASYN_CANBLOCK` octet port on `addr` with the EOS layer
/// `drvAsynIPPortConfigure` stacks, alive while the result is.
fn port(name: &str, addr: &str) -> impl Sized {
    let mut driver = DrvAsynIPPort::new(name, addr).unwrap();
    driver
        .base_mut()
        .install_octet_interpose(Box::new(EosInterpose::default()));
    PortManager::new()
        .register_port(claim_port_driver(name).unwrap(), driver, &open_owner())
        .unwrap()
}

#[test]
fn a_read_and_a_write_on_a_blocking_port_complete_while_their_band_is_full() {
    // `callbackSetQueueSize` and `callbackParallelThreads` before
    // `callbackInit`; both records' PRIO is LOW.
    set_queue_size(1);
    set_parallel_threads(1, Some(CallbackPriority::Low));
    background_init();
    let (ai_addr, _) = device(b"42.5\r\n");
    let (ao_addr, written) = device(b"");
    let _ai_port = port("SPFULL_AI", &ai_addr);
    let _ao_port = port("SPFULL_AO", &ao_addr);
    let db = r#"
record(ai, "SPFULL:AI") {
    field(DTYP, "asyn ai stringParm")
    field(INP, "@asyn(SPFULL_AI 0)")
}
record(ao, "SPFULL:AO") {
    field(DTYP, "asyn ao stringParm")
    field(OUT, "@asyn(SPFULL_AO 0)")
}
"#;
    test_block_on(async {
        let (db, _) = IocBuilder::new()
            .register_dynamic_device_support(&DEVICE_ROWS, device_support_factory())
            .db_string(db, &HashMap::new())
            .unwrap()
            .build()
            .await
            .unwrap();
        let _full = fill_low_band();
        db.put_record_field_from_ca_no_notify("SPFULL:AI", "PROC", EpicsValue::Short(1))
            .await
            .unwrap();
        db.put_record_field_from_ca_no_notify("SPFULL:AO", "VAL", EpicsValue::Double(3.0))
            .await
            .unwrap();
        let pact = |name: &str| db.get_record(name).unwrap().read().is_processing();
        for _ in 0..400 {
            if !pact("SPFULL:AI") && !pact("SPFULL:AO") {
                break;
            }
            sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(
            [pact("SPFULL:AI"), pact("SPFULL:AO")],
            [false, false],
            "PACT of the ai and the ao"
        );
        assert_eq!(
            db.get_pv("SPFULL:AI.VAL").unwrap(),
            EpicsValue::Double(42.5)
        );
    });
    assert_eq!(written.lock().unwrap().as_slice(), b"3.000000\r\n");
}
