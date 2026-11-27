//! On a port without `ASYN_CANBLOCK`, `queueRequest` runs the callback inside
//! the record's own pass (asynManager.c:1555-1584), so that pass finishes
//! there and no full callback band can leave the record PACT. One test per
//! binary: the band's size and thread count are set once per process, before
//! `callbackInit`.

use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use asyn_rs::error::AsynResult;
use asyn_rs::interpose::eos::EosInterpose;
use asyn_rs::interpose::{EomReason, OctetReadResult};
use asyn_rs::manager::{PortManager, claim_port_driver};
use asyn_rs::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
use asyn_rs::registry::open_owner;
use asyn_rs::user::AsynUser;
use epics_rs::base::runtime::background::callback_executor::{
    set_parallel_threads, set_queue_size,
};
use epics_rs::base::runtime::task::{
    CallbackPriority, background_callback_stats, background_init, sleep, spawn_blocking_background,
    test_block_on,
};
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::base::types::EpicsValue;

/// `device(ao,INST_IO,devAoMPC,"asyn MPC")`.
static MPC_AO: [(&str, &[DevSup]); 1] = [(
    "ao",
    &[DevSup {
        name: "devAoMPC",
        choice: ip_devices::mpc::DTYP,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(6, &[2, 4, 5], false),
    }],
)];

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

/// An octet port without `ASYN_CANBLOCK` that answers every read with
/// `reply` and keeps what it is sent.
struct Wire {
    base: PortDriverBase,
    reply: &'static [u8],
    eom: EomReason,
    written: Arc<Mutex<Vec<u8>>>,
}

impl PortDriver for Wire {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
        let n = self.reply.len().min(buf.len());
        buf[..n].copy_from_slice(&self.reply[..n]);
        OctetReadResult::ok(n, self.eom)
    }
    fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        self.written.lock().unwrap().extend_from_slice(data);
        Ok(data.len())
    }
}

/// A registered [`Wire`] on `name`, alive while the first result is; with
/// `eos`, the EOS layer `drvAsynIPPortConfigure` stacks.
fn port(
    name: &str,
    reply: &'static [u8],
    eom: EomReason,
    eos: bool,
) -> (impl Sized, Arc<Mutex<Vec<u8>>>) {
    let mut base = PortDriverBase::new(
        name,
        1,
        PortFlags {
            can_block: false,
            ..PortFlags::default()
        },
        InterfaceMask::COMMON | InterfaceMask::OCTET,
        InterfaceMask::NONE,
    );
    base.init_connected(true);
    if eos {
        base.install_octet_interpose(Box::new(EosInterpose::default()));
    }
    let written = Arc::new(Mutex::new(Vec::new()));
    let driver = Wire {
        base,
        reply,
        eom,
        written: written.clone(),
    };
    let runtime = PortManager::new()
        .register_port(claim_port_driver(name).unwrap(), driver, &open_owner())
        .unwrap();
    (runtime, written)
}

#[test]
fn passes_on_a_port_that_cannot_block_end_while_their_band_is_full() {
    // `callbackSetQueueSize` and `callbackParallelThreads` before
    // `callbackInit`; every record's PRIO is LOW.
    set_queue_size(1);
    set_parallel_threads(1, Some(CallbackPriority::Low));
    background_init();
    let (_ai_port, _) = port("SYNCFULL_AI", b"42.5\r\n", EomReason::END, true);
    let (_ao_port, written) = port("SYNCFULL_AO", b"", EomReason::END, true);
    // An MPC reply whose data is longer than `OK`: `writeX` alarms on it.
    let (_mpc_port, _) = port("SYNCFULL_MPC", b"05 OK 00 TOOBIG 1F", EomReason::EOS, false);
    let db = r#"
record(ai, "SYNCFULL:AI") {
    field(DTYP, "asyn ai stringParm")
    field(INP, "@asyn(SYNCFULL_AI 0)")
}
record(ao, "SYNCFULL:AO") {
    field(DTYP, "asyn ao stringParm")
    field(OUT, "@asyn(SYNCFULL_AO 0)")
}
record(ao, "SYNCFULL:MPC") {
    field(DTYP, "asyn MPC")
    field(OUT, "@asyn(SYNCFULL_MPC 5)SET_SP12 1")
}
"#;
    test_block_on(async {
        let (db, _) = IocBuilder::new()
            .register_dynamic_device_support(
                &ip_devices::strparm::DEVICE_ROWS,
                ip_devices::strparm::device_support_factory(),
            )
            .register_dynamic_device_support(&MPC_AO, ip_devices::mpc::device_support_factory())
            .db_string(db, &HashMap::new())
            .unwrap()
            .build()
            .await
            .unwrap();
        let _full = fill_low_band();
        db.put_record_field_from_ca_no_notify("SYNCFULL:AI", "PROC", EpicsValue::Short(1))
            .await
            .unwrap();
        db.put_record_field_from_ca_no_notify("SYNCFULL:AO", "VAL", EpicsValue::Double(3.0))
            .await
            .unwrap();
        db.put_record_field_from_ca_no_notify("SYNCFULL:MPC", "VAL", EpicsValue::Double(1e-6))
            .await
            .unwrap();
        let pact = |name: &str| db.get_record(name).unwrap().read().is_processing();
        let names = ["SYNCFULL:AI", "SYNCFULL:AO", "SYNCFULL:MPC"];
        for _ in 0..100 {
            if names.iter().all(|n| !pact(n)) {
                break;
            }
            sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(
            names.map(pact),
            [false, false, false],
            "PACT of the strParm ai, the strParm ao and the MPC ao"
        );
        assert_eq!(
            db.get_pv("SYNCFULL:AI.VAL").unwrap(),
            EpicsValue::Double(42.5)
        );
        // READ_ALARM, INVALID.
        assert_eq!(
            [
                db.get_pv("SYNCFULL:MPC.STAT").unwrap(),
                db.get_pv("SYNCFULL:MPC.SEVR").unwrap()
            ],
            [EpicsValue::Short(1), EpicsValue::Short(3)],
            "STAT and SEVR of the MPC ao"
        );
    });
    assert_eq!(written.lock().unwrap().as_slice(), b"3.000000\r\n");
}
