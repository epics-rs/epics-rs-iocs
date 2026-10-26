//! st.cmd boots with the records C's iocAcsTech80 loads, on the script's
//! prefix.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl, renamed,
};

#[test]
fn st_cmd_loads_the_c_spiiplus_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/iocAcsTech80.dbl")),
        &[("acsTech80:", "ACS:")],
        &[],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_acstech80-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocAcsTech80 does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_acstech80-ioc")));
}
