//! st.cmd boots with the records C's iocMotorSim st.cmd loads, on this
//! IOC's prefix and its two axes.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl, renamed,
};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut c = renamed(
        &read_dbl(&dir.join("tests/motorSim.dbl")),
        &[("IOC:", "SIM:")],
        &["IOC:m3", "IOC:m4", "IOC:m5", "IOC:m6", "IOC:m7", "IOC:m8"],
    );
    // motorUtil.db, which C's st.cmd runs motorUtilInit without.
    c.extend(
        ["allstop", "alldone", "moving", "alldoneBlink", "movingDiff"].map(|r| format!("SIM:{r}")),
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_motorsim-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocMotorSim does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_motorsim-ioc")));
}
