//! st.cmd boots with the records C's iocNewport st.cmd.SMC100 loads, on
//! this IOC's prefix.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl, renamed,
};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.SMC100.dbl")),
        &[("IOC:", "SMC100:")],
        &[],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_smc100-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocNewport does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_smc100-ioc")));
}
