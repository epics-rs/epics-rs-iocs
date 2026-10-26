//! Each startup script boots with the records C's iocAerotech loads for its
//! controller, on the script's prefix and axes.

use std::collections::BTreeSet;
use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl, renamed,
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

#[test]
fn st_cmd_loads_the_c_ensemble_records() {
    let c = c_st_cmd(&[], &["aerotech:m2"]);
    assert_same_records(&booted_records(ioc(), &dir().join("st.cmd")), &c);
}

#[test]
fn st_soloist_cmd_loads_the_c_soloist_records() {
    let c = c_st_cmd(
        &[("aerotech:", "soloist:"), ("aerotech:m2", "soloist:m1")],
        &["aerotech:m1"],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st-soloist.cmd")), &c);
}

#[test]
fn st_a3200_cmd_loads_the_c_a3200_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/a3.asyn.dbl")),
        &[("aerotech:", "a3200:"), ("aerotech:X", "a3200:m1")],
        &["aerotech:Y"],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.a3200.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocAerotech does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_aerotech-ioc")));
}
