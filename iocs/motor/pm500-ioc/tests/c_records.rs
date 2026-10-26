//! st.cmd boots with the records C's iocNewport st.cmd loads with its
//! PM500.cmd, on this IOC's prefix.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_fields, assert_same_records,
    booted_records, read_db, read_dbl, renamed, renamed_fields, script_fields,
};

const RENAMES: &[(&str, &str)] = &[("newport:", "PM500:")];

const DROPS: &[&str] = &["newport:m32", "newport:m33"];

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(&read_dbl(&dir.join("tests/st.cmd.dbl")), RENAMES, DROPS);
    let ioc = Path::new(env!("CARGO_BIN_EXE_pm500-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_fields() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut c = renamed_fields(&read_db(&dir.join("tests/st.cmd.db")), RENAMES, DROPS);
    // C's MRES is five times the driver's 0.01 um step; st.cmd's row is
    // that step.
    let m31 = &mut c.get_mut("PM500:m31").unwrap().fields;
    m31.insert("MRES".into(), "1e-5".into());
    assert_same_fields(&script_fields(&dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocNewport does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_pm500-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocNewport st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_pm500-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
