//! Each startup script boots with the records C's iocAerotech loads for its
//! controller, on the script's prefix.

use std::collections::BTreeSet;
use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_fields, assert_same_records,
    booted_records, read_db, read_dbl, renamed, renamed_fields, script_fields,
};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn ioc() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_aerotech-ioc"))
}

/// C's st.cmd loads the Ensemble's m1 and the Soloist's m2 on one motorUtil.
fn c_st_cmd(renames: &[(&str, &str)], drops: &[&str]) -> BTreeSet<String> {
    renamed(
        &read_dbl(&dir().join("tests/iocAerotech.dbl")),
        renames,
        drops,
    )
}

/// `script`'s loads and dbpf lines leave every field as C's `db` leaves it.
fn assert_sets_c_fields(script: &str, db: &str, renames: &[(&str, &str)], drops: &[&str]) {
    let c = renamed_fields(&read_db(&dir().join("tests").join(db)), renames, drops);
    assert_same_fields(&script_fields(&dir().join(script)), &c);
}

const ENSEMBLE_DROPS: &[&str] = &["aerotech:m2"];
const SOLOIST_RENAMES: &[(&str, &str)] = &[("aerotech:", "soloist:")];
const SOLOIST_DROPS: &[&str] = &["aerotech:m1"];
const A3200_RENAMES: &[(&str, &str)] = &[("aerotech:", "a3200:")];
const A3200_DROPS: &[&str] = &["aerotech:Y"];

#[test]
fn st_cmd_loads_the_c_ensemble_records() {
    let c = c_st_cmd(&[], ENSEMBLE_DROPS);
    assert_same_records(&booted_records(ioc(), &dir().join("st.cmd")), &c);
}

#[test]
fn st_cmd_sets_the_c_ensemble_fields() {
    assert_sets_c_fields("st.cmd", "iocAerotech.db", &[], ENSEMBLE_DROPS);
}

#[test]
fn st_soloist_cmd_loads_the_c_soloist_records() {
    let c = c_st_cmd(SOLOIST_RENAMES, SOLOIST_DROPS);
    assert_same_records(&booted_records(ioc(), &dir().join("st-soloist.cmd")), &c);
}

#[test]
fn st_soloist_cmd_sets_the_c_soloist_fields() {
    assert_sets_c_fields(
        "st-soloist.cmd",
        "iocAerotech.db",
        SOLOIST_RENAMES,
        SOLOIST_DROPS,
    );
}

#[test]
fn st_a3200_cmd_loads_the_c_a3200_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/a3.asyn.dbl")),
        A3200_RENAMES,
        A3200_DROPS,
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.a3200.cmd")), &c);
}

#[test]
fn st_a3200_cmd_sets_the_c_a3200_fields() {
    assert_sets_c_fields("st.a3200.cmd", "a3.asyn.db", A3200_RENAMES, A3200_DROPS);
}

/// The binary links asyn.dbd, as C's iocAerotech does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_aerotech-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocAerotech st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    assert_inits_motor_util(ioc(), &dir().join("st.cmd"));
}

/// st-soloist.cmd starts motorUtil after iocInit, as C's iocAerotech st.cmd does.
#[test]
fn st_soloist_cmd_inits_motor_util() {
    assert_inits_motor_util(ioc(), &dir().join("st-soloist.cmd"));
}

/// st.a3200.cmd starts motorUtil after iocInit, as C's iocAerotech
/// a3.asyn.st.cmd.linux-x86 does.
#[test]
fn st_a3200_cmd_inits_motor_util() {
    assert_inits_motor_util(ioc(), &dir().join("st.a3200.cmd"));
}
