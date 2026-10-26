//! st.cmd boots with the records C's iocSmartMotor st.cmd loads, on this
//! IOC's prefix.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_calls, assert_same_fields,
    assert_same_records, booted_records, read_calls, read_db, read_dbl, renamed, renamed_fields,
    script_calls, script_fields,
};

const RENAMES: &[(&str, &str)] = &[("smartMotor:", "SM:")];

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(&read_dbl(&dir.join("tests/st.cmd.dbl")), RENAMES, &[]);
    let ioc = Path::new(env!("CARGO_BIN_EXE_smartmotor-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_fields() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed_fields(&read_db(&dir.join("tests/st.cmd.db")), RENAMES, &[]);
    assert_same_fields(&script_fields(&dir.join("st.cmd")), &c);
}

/// st.cmd makes C's controller calls, in C's order.
#[test]
fn st_cmd_makes_the_c_calls() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_calls(&dir.join("tests/st.cmd.calls"));
    assert_same_calls(&script_calls(&dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocSmartMotor does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_smartmotor-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocSmartMotor st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_smartmotor-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
