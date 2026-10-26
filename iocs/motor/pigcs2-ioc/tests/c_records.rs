//! st.cmd boots with the records C's iocPIGCS2 st.cmd loads, on this IOC's
//! prefix and its one axis.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_fields, assert_same_records,
    booted_records, read_db, read_dbl, renamed, renamed_fields, script_fields,
};

const RENAMES: &[(&str, &str)] = &[("pigcs2:", "pi:")];

const DROPS: &[&str] = &["pigcs2:m2"];

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(&read_dbl(&dir.join("tests/iocPIGCS2.dbl")), RENAMES, DROPS);
    let ioc = Path::new(env!("CARGO_BIN_EXE_pigcs2-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_fields() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed_fields(&read_db(&dir.join("tests/iocPIGCS2.db")), RENAMES, DROPS);
    assert_same_fields(&script_fields(&dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocPIGCS2 does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_pigcs2-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocPIGCS2 st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_pigcs2-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
