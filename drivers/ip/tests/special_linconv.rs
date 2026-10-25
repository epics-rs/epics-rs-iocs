//! ai/ao `special(SPC_LINCONV)` through each ip dset's convert slot
//! (`aiRecord.c:187-198`, `aoRecord.c:254-265`). The expected ESLO/EOFF/ROFF
//! are a C IOC built from base 8f5015b66, asyn and ip d00f687's four dset
//! sources, with the same records and puts.

use std::collections::HashMap;

use epics_rs::base::runtime::task::test_block_on;
use epics_rs::base::server::device_support::DeviceSupport;
use epics_rs::base::server::ioc_app::DeviceSupportContext;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::base::types::EpicsValue;

const FIELDS: &str = r#"field(EGUL, "5") field(EGUF, "10") field(ESLO, "2") field(EOFF, "3") field(ROFF, "4") field(LOPR, "-7") field(ASLO, "1.1")"#;

/// ESLO, EOFF, ROFF, LOPR, ASLO.
type Fields = [f64; 5];

/// `<p>:L` (LINR LINEAR) after `EGUF 20`, `<p>:N` (NO CONVERSION) after
/// `LINR LINEAR`, then `<p>:L` after `EGUL 1`. No port `P` exists, so every
/// init fails and leaves PACT set, as C's does; the puts are `dbPutField`s
/// (`dbpf`), which a put-notify would instead hold until PACT clears.
fn linconv<F>(prefix: &str, rtype: &str, dtyp: &'static str, link: &str, factory: F) -> [Fields; 3]
where
    F: Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static,
{
    let rows: &'static [DevSup] = Box::leak(Box::new([DevSup {
        name: "devIp",
        choice: dtyp,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(6, &[2, 4, 5], false),
    }]));
    let io = if rtype == "ao" { "OUT" } else { "INP" };
    let mut db = String::new();
    for (suffix, linr) in [("L", "LINEAR"), ("N", "NO CONVERSION")] {
        db += &format!(
            "record({rtype}, \"{prefix}:{suffix}\") {{ field(DTYP, \"{dtyp}\") \
             field({io}, \"{link}\") field(LINR, \"{linr}\") {FIELDS} }}\n"
        );
    }
    let rtype = rtype.to_owned();
    test_block_on(async move {
        let (db, _) = IocBuilder::new()
            .register_dynamic_device_support(&[(rtype.as_str(), rows)], factory)
            .db_string(&db, &HashMap::new())
            .unwrap()
            .build()
            .await
            .unwrap();
        let fields = |name: &str| -> Fields {
            let rec = db.get_record(name).unwrap();
            let inst = rec.read();
            ["ESLO", "EOFF", "ROFF", "LOPR", "ASLO"]
                .map(|f| inst.record.get_field(f).and_then(|v| v.to_f64()).unwrap())
        };
        let (l, n) = (format!("{prefix}:L"), format!("{prefix}:N"));
        let put = async |name: &str, field: &str, value: EpicsValue| {
            db.put_record_field_from_ca_no_notify(name, field, value)
                .await
                .unwrap();
        };
        put(&l, "EGUF", EpicsValue::Double(20.0)).await;
        put(&n, "LINR", EpicsValue::Short(2)).await;
        let first = [fields(&l), fields(&n)];
        put(&l, "EGUL", EpicsValue::Double(1.0)).await;
        [first[0], first[1], fields(&l)]
    })
}

/// `convertAi` does nothing, but the filled slot still rebases EOFF to EGUL.
#[test]
fn mks_ai_rebases_eoff_only() {
    let got = linconv(
        "LC:MKS",
        "ai",
        ip_devices::mks::DTYP,
        "@asyn(P 1)",
        ip_devices::mks::device_support_factory(),
    );
    assert_eq!(
        got,
        [
            [2.0, 5.0, 4.0, -7.0, 1.1],
            [2.0, 5.0, 4.0, -7.0, 1.1],
            [2.0, 1.0, 4.0, -7.0, 1.1],
        ]
    );
}

/// `MPCConvert` sets ESLO 1 and ROFF 0 on an ai. On an ao its `aiRecord *`
/// cast writes LOPR 1 and zeroes ASLO's low word (C reads ASLO 1.0999994278
/// afterwards); only the EOFF rebase is kept there.
#[test]
fn mpc_ai_takes_a_unit_slope_and_ao_rebases_eoff_only() {
    let factory = ip_devices::mpc::device_support_factory;
    let ai = linconv(
        "LC:MPCAI",
        "ai",
        ip_devices::mpc::DTYP,
        "@asyn(P 5)GET_PRESSURE 1",
        factory(),
    );
    assert_eq!(
        ai,
        [
            [1.0, 5.0, 0.0, -7.0, 1.1],
            [1.0, 5.0, 0.0, -7.0, 1.1],
            [1.0, 1.0, 0.0, -7.0, 1.1],
        ]
    );
    let ao = linconv(
        "LC:MPCAO",
        "ao",
        ip_devices::mpc::DTYP,
        "@asyn(P 5)SET_SIZE 1",
        factory(),
    );
    assert_eq!(
        ao,
        [
            [2.0, 5.0, 4.0, -7.0, 1.1],
            [2.0, 5.0, 4.0, -7.0, 1.1],
            [2.0, 1.0, 4.0, -7.0, 1.1],
        ]
    );
}

/// `TPG261Convert` is `MPCConvert`'s body, with the same ao cast.
#[test]
fn tpg261_ai_takes_a_unit_slope_and_ao_rebases_eoff_only() {
    let factory = ip_devices::tpg261::device_support_factory;
    let ai = linconv(
        "LC:TPGAI",
        "ai",
        ip_devices::tpg261::DTYP,
        "@asyn(P 1)GET_PRESSURE",
        factory(),
    );
    assert_eq!(
        ai,
        [
            [1.0, 5.0, 0.0, -7.0, 1.1],
            [1.0, 5.0, 0.0, -7.0, 1.1],
            [1.0, 1.0, 0.0, -7.0, 1.1],
        ]
    );
    let ao = linconv(
        "LC:TPGAO",
        "ao",
        ip_devices::tpg261::DTYP,
        "@asyn(P 1)SET_SP1",
        factory(),
    );
    assert_eq!(
        ao,
        [
            [2.0, 5.0, 4.0, -7.0, 1.1],
            [2.0, 5.0, 4.0, -7.0, 1.1],
            [2.0, 1.0, 4.0, -7.0, 1.1],
        ]
    );
}

/// `TelevacConvert` sets ESLO 1 and ROFF 0.
#[test]
fn televac_ai_takes_a_unit_slope() {
    let got = linconv(
        "LC:TV",
        "ai",
        ip_devices::televac::DTYP,
        "@asyn(P 1)",
        ip_devices::televac::device_support_factory(),
    );
    assert_eq!(
        got,
        [
            [1.0, 5.0, 0.0, -7.0, 1.1],
            [1.0, 5.0, 0.0, -7.0, 1.1],
            [1.0, 1.0, 0.0, -7.0, 1.1],
        ]
    );
}
