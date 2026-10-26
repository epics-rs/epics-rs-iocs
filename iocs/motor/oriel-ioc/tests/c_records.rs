//! st.cmd boots with the records C's iocOriel st.cmd loads, on this IOC's
//! prefix.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_fields, assert_same_records,
    booted_records, read_db, read_dbl, renamed, renamed_fields, script_fields,
};

const RENAMES: &[(&str, &str)] = &[("oriel:", "ORIEL:")];

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_dbl(&dir.join("tests/st.cmd.dbl"));
    let want = renamed(&c, RENAMES, &[]);
    let ioc = Path::new(env!("CARGO_BIN_EXE_oriel-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &want);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_fields() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed_fields(&read_db(&dir.join("tests/st.cmd.db")), RENAMES, &[]);
    assert_same_fields(&script_fields(&dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocOriel does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_oriel-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocOriel st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_oriel-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
