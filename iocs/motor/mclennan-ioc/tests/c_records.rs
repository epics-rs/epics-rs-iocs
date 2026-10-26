//! st.cmd boots with the records C's iocMclennan loads, on the script's
//! prefix.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl,
    renamed,
};

#[test]
fn st_cmd_loads_the_c_pm304_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_dbl(&dir.join("tests/iocMclennan.dbl"));
    let want = renamed(&c, &[("mclennan:", "PM304:")], &[]);
    let ioc = Path::new(env!("CARGO_BIN_EXE_mclennan-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &want);
}

/// The binary links asyn.dbd, as C's iocMclennan does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_mclennan-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocMclennan st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_mclennan-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
