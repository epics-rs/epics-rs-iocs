//! st.cmd boots with the records C's iocNewport loads for an MM4000, on the
//! script's prefix.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl,
    renamed,
};

#[test]
fn st_cmd_loads_the_c_mm4000_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/iocNewport.MM4000.dbl")),
        &[("newport:", "MM4000:")],
        &[],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_mm4000-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocNewport does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_mm4000-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocNewport st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_mm4000-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
