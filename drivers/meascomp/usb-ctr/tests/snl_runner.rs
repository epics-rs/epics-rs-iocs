//! `USBCTR_SNL` against soft stand-ins for the records `measCompMCS.template`
//! and the spectrum waveforms declare. Each `calc` reading itself plus one
//! counts how often the program processed it.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::types::EpicsValue;
use usb_ctr::snl::{UsbCtrSnlConfig, run};

fn counter(name: &str) -> String {
    format!("record(calc, \"{name}\") {{ field(INPA, \"{name} NPP\") field(CALC, \"A+1\") }}\n")
}

async fn ioc(acquiring: i32) -> Arc<PvDatabase> {
    let mut db = String::new();
    db += "record(bo, \"T:DoReadAll\") {}\n";
    db += "record(bi, \"T:HardwareAcquiring\") {}\n";
    db += "record(bi, \"T:SNL_Connected\") {}\n";
    db += &format!("record(bo, \"T:Acquiring\") {{ field(VAL, \"{acquiring}\") }}\n");
    db += "record(asyn, \"T:Asyn\") {}\n";
    db += &counter("T:AbsTimeWF");
    for n in 1..=3 {
        db += &counter(&format!("T:mca{n}"));
    }
    let (name, factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    let (db, _) = IocBuilder::new()
        .register_record_type(name, move || factory())
        .db_string(&db, &HashMap::new())
        .unwrap()
        .build_served()
        .await
        .unwrap();
    db
}

fn start(db: &Arc<PvDatabase>, macros: &str) -> tokio::task::JoinHandle<()> {
    let config = UsbCtrSnlConfig::from_macros(macros);
    let db = (**db).clone();
    tokio::spawn(async move {
        run(config, db).await.unwrap();
    })
}

async fn value(db: &PvDatabase, pv: &str) -> f64 {
    db.get_pv(pv).unwrap().to_f64().unwrap()
}

/// Poll until every `(pv, value)` holds; on timeout, name what each read.
async fn expect(db: &PvDatabase, want: &[(&str, f64)]) {
    for _ in 0..300 {
        let mut all = true;
        for &(pv, v) in want {
            all &= value(db, pv).await == v;
        }
        if all {
            return;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let mut got = Vec::new();
    for &(pv, _) in want {
        got.push((pv, value(db, pv).await));
    }
    panic!("want {want:?}, got {got:?}");
}

/// A client's put: the record processes and posts its monitor.
async fn ca_put(db: &PvDatabase, pv: &str, v: i32) {
    db.put_record_field_from_ca(pv, "VAL", EpicsValue::Enum(v as u16))
        .await
        .unwrap();
}

const MACROS: &str = "P=T:, R=mca, NUM_COUNTERS=2, FIELD=PROC";

#[epics_rs::base::epics_test]
async fn the_connect_time_monitor_reads_each_counter_and_releases_acquiring() {
    let db = ioc(1).await;
    let task = start(&db, MACROS);
    expect(
        &db,
        &[
            ("T:SNL_Connected", 1.0),
            ("T:mca1", 1.0),
            ("T:mca2", 1.0),
            ("T:AbsTimeWF", 1.0),
            ("T:Acquiring", 0.0),
        ],
    )
    .await;
    // NUM_COUNTERS=2 assigns mca1 and mca2 only.
    assert_eq!(value(&db, "T:mca3").await, 0.0);
    task.abort();
}

#[epics_rs::base::epics_test]
async fn a_read_request_reads_and_is_cleared_without_releasing_acquiring() {
    let db = ioc(0).await;
    let task = start(&db, MACROS);
    expect(&db, &[("T:mca1", 1.0), ("T:AbsTimeWF", 1.0)]).await;
    ca_put(&db, "T:Acquiring", 1).await;
    ca_put(&db, "T:HardwareAcquiring", 1).await;
    ca_put(&db, "T:DoReadAll", 1).await;
    expect(
        &db,
        &[
            ("T:mca1", 2.0),
            ("T:mca2", 2.0),
            ("T:AbsTimeWF", 2.0),
            ("T:DoReadAll", 0.0),
        ],
    )
    .await;
    // A second request is evaluated after the HardwareAcquiring=1 clause.
    ca_put(&db, "T:DoReadAll", 1).await;
    expect(&db, &[("T:mca1", 3.0), ("T:DoReadAll", 0.0)]).await;
    assert_eq!(value(&db, "T:Acquiring").await, 1.0);
    task.abort();
}

#[epics_rs::base::epics_test]
async fn the_end_of_acquisition_reads_and_releases_acquiring() {
    let db = ioc(0).await;
    let task = start(&db, MACROS);
    expect(&db, &[("T:mca1", 1.0), ("T:Acquiring", 0.0)]).await;
    ca_put(&db, "T:Acquiring", 1).await;
    ca_put(&db, "T:HardwareAcquiring", 1).await;
    ca_put(&db, "T:HardwareAcquiring", 0).await;
    expect(
        &db,
        &[
            ("T:mca1", 2.0),
            ("T:mca2", 2.0),
            ("T:AbsTimeWF", 2.0),
            ("T:Acquiring", 0.0),
        ],
    )
    .await;
    task.abort();
}

#[epics_rs::base::epics_test]
async fn an_illegal_num_counters_leaves_the_program_dead() {
    let db = ioc(1).await;
    let config = UsbCtrSnlConfig::from_macros("P=T:, NUM_COUNTERS=10, FIELD=PROC");
    run(config, (*db).clone()).await.unwrap();
    assert_eq!(value(&db, "T:SNL_Connected").await, 0.0);
    assert_eq!(value(&db, "T:Acquiring").await, 1.0);
}

#[epics_rs::base::epics_test]
async fn a_missing_record_is_named() {
    let db = ioc(1).await;
    let config = UsbCtrSnlConfig::from_macros("P=T:, NUM_COUNTERS=4, FIELD=PROC");
    let err = run(config, (*db).clone()).await.unwrap_err().to_string();
    assert_eq!(err, "USBCTR_SNL: not in the database: T:mca4.PROC");
}
