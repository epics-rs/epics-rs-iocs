//! A `queueRequest` the port refuses ends the record's pass at once with
//! `asynStatusToEpicsAlarm`'s alarm (asynEpicsUtils.c:234-266), as
//! devAsynInt32's `processAi` ends one (devAsynInt32.c:836-846); an input
//! keeps its value, and the put that processed the record fails with the
//! pass's -1, as `dbPutField` returns it (dbAccess.c:1279-1284). The port
//! here can block but is disconnected without auto-connect, so every request
//! is refused with `asynDisconnected`.

use std::collections::HashMap;
use std::time::Duration;

use asyn_rs::manager::{PortManager, claim_port_driver};
use asyn_rs::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
use asyn_rs::registry::open_owner;
use epics_rs::base::runtime::task::{sleep, test_block_on};
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::base::types::EpicsValue;

const fn row(name: &'static str, choice: &'static str) -> DevSup {
    DevSup {
        name,
        choice,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(6, &[2, 4, 5], false),
    }
}

static TPG261_AI: [(&str, &[DevSup]); 1] =
    [("ai", &[row("devAiTPG261", ip_devices::tpg261::DTYP)])];
static MPC_AO: [(&str, &[DevSup]); 1] = [("ao", &[row("devAoMPC", ip_devices::mpc::DTYP)])];
static TELEVAC_AI: [(&str, &[DevSup]); 1] =
    [("ai", &[row("devAiTelevac", ip_devices::televac::DTYP)])];
static ND261_AI: [(&str, &[DevSup]); 1] =
    [("ai", &[row("devAiHeidND261", ip_devices::nd261::DTYP)])];
static EUROTHERM_AO: [(&str, &[DevSup]); 1] = [(
    "ao",
    &[row("devAoEurotherm", ip_devices::eurotherm::DEVICES[0].1)],
)];

/// An octet port that can block, is not connected and never connects.
struct Dead(PortDriverBase);

impl PortDriver for Dead {
    fn base(&self) -> &PortDriverBase {
        &self.0
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.0
    }
}

#[test]
fn a_refused_request_ends_the_pass_with_an_alarm() {
    let mut base = PortDriverBase::new(
        "REFUSED",
        1,
        PortFlags {
            can_block: true,
            ..PortFlags::default()
        },
        InterfaceMask::COMMON | InterfaceMask::OCTET,
        InterfaceMask::NONE,
    );
    base.init_connected(false);
    base.init_auto_connect(false);
    let _port = PortManager::new()
        .register_port(
            claim_port_driver("REFUSED").unwrap(),
            Dead(base),
            &open_owner(),
        )
        .unwrap();
    let db = r#"
record(ai, "RF:SP:AI") {
    field(DTYP, "asyn ai stringParm")
    field(INP, "@asyn(REFUSED 0)")
    field(VAL, "7")
}
record(ao, "RF:SP:AO") {
    field(DTYP, "asyn ao stringParm")
    field(OUT, "@asyn(REFUSED 0)")
}
record(ai, "RF:TPG:AI") {
    field(DTYP, "asyn TPG261")
    field(INP, "@asyn(REFUSED 1)GET_PRESSURE")
    field(VAL, "7")
}
record(ao, "RF:MPC:AO") {
    field(DTYP, "asyn MPC")
    field(OUT, "@asyn(REFUSED 5)SET_SP12 1")
}
record(ai, "RF:TV:AI") {
    field(DTYP, "asyn Televac")
    field(INP, "@asyn(REFUSED 0)GET_PRESSURE 1")
    field(VAL, "7")
}
record(ai, "RF:ND:AI") {
    field(DTYP, "asyn ai HeidND261")
    field(INP, "@asyn(REFUSED 0)")
    field(VAL, "7")
}
record(ao, "RF:EU:AO") {
    field(DTYP, "asyn ao Eurotherm")
    field(OUT, "@asyn(REFUSED)")
}
"#;
    test_block_on(async {
        let (db, _) = IocBuilder::new()
            .register_dynamic_device_support(
                &ip_devices::strparm::DEVICE_ROWS,
                ip_devices::strparm::device_support_factory(),
            )
            .register_dynamic_device_support(
                &TPG261_AI,
                ip_devices::tpg261::device_support_factory(),
            )
            .register_dynamic_device_support(&MPC_AO, ip_devices::mpc::device_support_factory())
            .register_dynamic_device_support(
                &TELEVAC_AI,
                ip_devices::televac::device_support_factory(),
            )
            .register_dynamic_device_support(&ND261_AI, ip_devices::nd261::device_support_factory())
            .register_dynamic_device_support(
                &EUROTHERM_AO,
                ip_devices::eurotherm::device_support_factory(),
            )
            .db_string(db, &HashMap::new())
            .unwrap()
            .build()
            .await
            .unwrap();
        let inputs = ["RF:SP:AI", "RF:TPG:AI", "RF:TV:AI", "RF:ND:AI"];
        let outputs = ["RF:SP:AO", "RF:MPC:AO", "RF:EU:AO"];
        let mut put_failed = Vec::new();
        for name in inputs {
            let put = db.put_record_field_from_ca_no_notify(name, "PROC", EpicsValue::Short(1));
            put_failed.push((name, put.await.is_err()));
        }
        for name in outputs {
            let put = db.put_record_field_from_ca_no_notify(name, "VAL", EpicsValue::Double(3.0));
            put_failed.push((name, put.await.is_err()));
        }
        let all: Vec<&str> = inputs.iter().chain(&outputs).copied().collect();
        let pact = |name: &str| db.get_record(name).unwrap().read().is_processing();
        for _ in 0..100 {
            if all.iter().all(|n| !pact(n)) {
                break;
            }
            sleep(Duration::from_millis(5)).await;
        }
        let pv = |name: &str, field: &str| db.get_pv(format!("{name}.{field}")).unwrap();
        // COMM_ALARM, INVALID.
        assert_eq!(
            all.iter()
                .map(|&n| (n, pact(n), pv(n, "STAT"), pv(n, "SEVR")))
                .collect::<Vec<_>>(),
            all.iter()
                .map(|&n| (n, false, EpicsValue::Short(9), EpicsValue::Short(3)))
                .collect::<Vec<_>>(),
            "PACT, STAT and SEVR"
        );
        assert_eq!(
            inputs.map(|n| pv(n, "VAL")),
            inputs.map(|_| EpicsValue::Double(7.0)),
            "VAL of each input"
        );
        assert_eq!(
            put_failed,
            all.iter().map(|&n| (n, true)).collect::<Vec<_>>(),
            "whether each put failed"
        );
    });
}
