//! st.cmd boots with the records C's iocNewport st.cmd.xps5 loads, on this
//! IOC's prefix and its one axis, less those whose parameters this driver
//! lacks.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_fields, assert_same_records, booted_records, read_db,
    read_dbl, renamed, renamed_fields, script_fields,
};

/// XPS_extra.db's records (XPS_MIN_JERK, XPS_MAX_JERK, XPS_STATUS) and the
/// profile-move records (XPS_PROFILE_*, and PROFILE_* with no XpsHead hook).
const UNSERVED: &[&str] = &[
    "IOC:m1MIN_JERK_TIME",
    "IOC:m1MAX_JERK_TIME",
    "IOC:m1XPS_STATUS",
    "IOC:m1READBACK",
    "IOC:Prof1:",
];

const RENAMES: &[(&str, &str)] = &[("IOC:", "XPS:")];

fn drops() -> Vec<&'static str> {
    let axes = [
        "IOC:m2", "IOC:m3", "IOC:m4", "IOC:m5", "IOC:m6", "IOC:m7", "IOC:m8",
    ];
    [&axes[..], UNSERVED].concat()
}

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.xps5.names")),
        RENAMES,
        &drops(),
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_xps-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_fields() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed_fields(
        &read_db(&dir.join("tests/st.cmd.xps5.db")),
        RENAMES,
        &drops(),
    );
    assert_same_fields(&script_fields(&dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocNewport does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_xps-ioc")));
}
