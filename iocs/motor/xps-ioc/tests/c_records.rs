//! st.cmd boots with the records C's iocNewport st.cmd.xps5 loads, on this
//! IOC's prefix and its one axis, less those whose parameters this driver
//! lacks.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl, renamed,
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

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let axes = [
        "IOC:m2", "IOC:m3", "IOC:m4", "IOC:m5", "IOC:m6", "IOC:m7", "IOC:m8",
    ];
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.xps5.names")),
        &[("IOC:", "XPS:"), ("IOC:m1", "XPS:m0")],
        &[&axes[..], UNSERVED].concat(),
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_xps-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocNewport does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_xps-ioc")));
}
