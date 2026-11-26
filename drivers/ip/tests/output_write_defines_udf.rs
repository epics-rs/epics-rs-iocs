//! A write the controller accepted defines the output: C `writeX`'s
//! completing pass sets `pr->udf = 0` (devMPC.c:649-799, devTPG261.c:547-617),
//! so a record processed without a VAL put leaves UDF_ALARM on its next pass.

use std::collections::HashMap;
use std::time::Duration;

use asyn_rs::error::AsynResult;
use asyn_rs::interpose::{EomReason, OctetReadResult};
use asyn_rs::manager::{PortManager, claim_port_driver};
use asyn_rs::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
use asyn_rs::registry::open_owner;
use asyn_rs::user::AsynUser;
use epics_rs::base::runtime::task::{sleep, test_block_on};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::base::types::EpicsValue;

const fn row(name: &'static str, choice: &'static str) -> DevSup {
    DevSup {
        name,
        choice,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(6, &[2, 4], false),
    }
}

static MPC: [(&str, &[DevSup]); 2] = [
    ("bo", &[row("devBoMPC", ip_devices::mpc::DTYP)]),
    ("stringout", &[row("devSoMPC", ip_devices::mpc::DTYP)]),
];
static TPG261: [(&str, &[DevSup]); 1] = [("bo", &[row("devBoTPG261", ip_devices::tpg261::DTYP)])];

/// An octet port that answers every read with `reply`.
struct Wire {
    base: PortDriverBase,
    reply: &'static [u8],
    eom: EomReason,
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
        Ok(data.len())
    }
}

/// A registered [`Wire`] on `name`, alive while the result is.
fn port(name: &str, can_block: bool, reply: &'static [u8], eom: EomReason) -> impl Sized {
    let mut base = PortDriverBase::new(
        name,
        1,
        PortFlags {
            can_block,
            ..PortFlags::default()
        },
        InterfaceMask::COMMON | InterfaceMask::OCTET,
        InterfaceMask::NONE,
    );
    base.init_connected(true);
    PortManager::new()
        .register_port(
            claim_port_driver(name).unwrap(),
            Wire { base, reply, eom },
            &open_owner(),
        )
        .unwrap()
}

async fn process(db: &PvDatabase, name: &str) {
    db.put_record_field_from_ca_no_notify(name, "PROC", EpicsValue::Short(1))
        .await
        .unwrap();
    for _ in 0..200 {
        if !db.get_record(name).unwrap().read().is_processing() {
            return;
        }
        sleep(Duration::from_millis(5)).await;
    }
    panic!("{name} stayed PACT");
}

/// UDF, STAT and SEVR of `name` after two passes without a VAL put.
async fn after_two_passes(db: &PvDatabase, name: &str) -> [EpicsValue; 3] {
    process(db, name).await;
    process(db, name).await;
    ["UDF", "STAT", "SEVR"].map(|f| db.get_pv(format!("{name}.{f}")).unwrap())
}

#[test]
fn an_accepted_write_clears_udf() {
    // A reply of bare `OK` / an ACK: the write landed.
    let _queued = port("UDF_MPC_Q", true, b"05 OK 00", EomReason::EOS);
    let _ran = port("UDF_MPC_RAN", false, b"05 OK 00", EomReason::EOS);
    let _tpg = port("UDF_TPG", true, b"\x06", EomReason::END);
    // A reply with data past `OK`: `writeX` alarms and leaves UDF.
    let _big = port("UDF_MPC_BIG", true, b"05 OK 00 TOOBIG 1F", EomReason::EOS);
    let db = r#"
record(stringout, "UDF:MPC:SO") {
    field(DTYP, "asyn MPC")
    field(OUT, "@asyn(UDF_MPC_Q 5)SET_TSP_TIMED 1")
}
record(bo, "UDF:MPC:BO") {
    field(DTYP, "asyn MPC")
    field(OUT, "@asyn(UDF_MPC_RAN 5)SET_STOP 1")
}
record(bo, "UDF:TPG:BO") {
    field(DTYP, "asyn TPG261")
    field(OUT, "@asyn(UDF_TPG 1)SET_SENSOR")
}
record(bo, "UDF:MPC:BIG") {
    field(DTYP, "asyn MPC")
    field(OUT, "@asyn(UDF_MPC_BIG 5)SET_STOP 1")
}
"#;
    test_block_on(async {
        let (db, _) = IocBuilder::new()
            .register_dynamic_device_support(&MPC, ip_devices::mpc::device_support_factory())
            .register_dynamic_device_support(&TPG261, ip_devices::tpg261::device_support_factory())
            .db_string(db, &HashMap::new())
            .unwrap()
            .build()
            .await
            .unwrap();
        // A bo is defined from init; a CA client can still mark it undefined.
        for name in ["UDF:MPC:BO", "UDF:TPG:BO", "UDF:MPC:BIG"] {
            db.put_record_field_from_ca_no_notify(name, "UDF", EpicsValue::UChar(1))
                .await
                .unwrap();
        }
        let defined = [
            EpicsValue::UChar(0),
            EpicsValue::Short(0),
            EpicsValue::Short(0),
        ];
        // A stringout with no VAL is undefined from init.
        assert_eq!(
            after_two_passes(&db, "UDF:MPC:SO").await,
            defined,
            "MPC stringout on a port that queues"
        );
        assert_eq!(
            after_two_passes(&db, "UDF:MPC:BO").await,
            defined,
            "MPC bo on a port that cannot block"
        );
        assert_eq!(
            after_two_passes(&db, "UDF:TPG:BO").await,
            defined,
            "TPG261 bo"
        );
        let [udf, _, sevr] = after_two_passes(&db, "UDF:MPC:BIG").await;
        assert_eq!(
            [udf, sevr],
            [EpicsValue::UChar(1), EpicsValue::Short(3)],
            "UDF and SEVR of an MPC bo whose reply was too big"
        );
    });
}
