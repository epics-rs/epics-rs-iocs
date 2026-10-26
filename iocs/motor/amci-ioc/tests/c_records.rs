//! st.cmd and st.ang1.cmd boot with the records C's iocAMCI loads for its
//! ANF2 and ANG1 examples, on the scripts' prefix.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl, renamed,
};

fn assert_boots_as_c(script: &str, dbl: &str) {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(&read_dbl(&dir.join(dbl)), &[("amci:", "AMCI:")], &[]);
    let ioc = Path::new(env!("CARGO_BIN_EXE_amci-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join(script)), &c);
}

#[test]
fn st_cmd_loads_the_c_anf2_records() {
    assert_boots_as_c("st.cmd", "tests/iocAMCI.dbl");
}

#[test]
fn st_ang1_cmd_loads_the_c_ang1_records() {
    assert_boots_as_c("st.ang1.cmd", "tests/iocAMCI.ANG1.dbl");
}

/// The binary links asyn.dbd, as C's iocAMCI does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_amci-ioc")));
}
