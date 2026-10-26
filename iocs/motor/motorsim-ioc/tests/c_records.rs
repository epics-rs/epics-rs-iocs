//! st.cmd boots with the records C's iocMotorSim st.cmd loads, on this
//! IOC's prefix and its two axes.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_fields, assert_same_records,
    booted_records, read_db, read_dbl, renamed, renamed_fields, script_fields,
};

const RENAMES: &[(&str, &str)] = &[("IOC:", "SIM:")];
const DROPS: &[&str] = &["IOC:m3", "IOC:m4", "IOC:m5", "IOC:m6", "IOC:m7", "IOC:m8"];

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut c = renamed(&read_dbl(&dir.join("tests/motorSim.dbl")), RENAMES, DROPS);
    // motorUtil.db, which C's st.cmd runs motorUtilInit without.
    c.extend(
        ["allstop", "alldone", "moving", "alldoneBlink", "movingDiff"].map(|r| format!("SIM:{r}")),
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_motorsim-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_fields() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed_fields(&read_db(&dir.join("tests/motorSim.db")), RENAMES, DROPS);
    assert_same_fields(&script_fields(&dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocMotorSim does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_motorsim-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocMotorSim st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_motorsim-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
