//! Each startup script boots with the records C's iocParker loads for its
//! controller, on the script's prefix and axes.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl,
    renamed,
};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn ioc() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_parker-ioc"))
}

#[test]
fn st_cmd_loads_the_c_oem_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.oem.dbl")),
        &[("IOC:", "OEM:")],
        &["IOC:m2", "IOC:m3"],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.cmd")), &c);
}

#[test]
fn st_acr_cmd_loads_the_c_acr_records() {
    // C's asyn2 sits on ARIES1 beside asyn1, its ARIES2 port's records on
    // the second controller this IOC does not configure.
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.acr.dbl")),
        &[("IOC:", "ACR:")],
        &["IOC:m2", "IOC:ACR2:", "IOC:asyn2"],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.acr.cmd")), &c);
}

#[test]
fn st_pc6k_cmd_loads_the_c_pc6k_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.dbl")),
        &[("parker:", "PC6K:")],
        &["parker:m2"],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st-pc6k.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocParker does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_parker-ioc")));
}

/// st-pc6k.cmd starts motorUtil after iocInit, as C's iocParker st.cmd does.
#[test]
fn st_pc6k_cmd_inits_motor_util() {
    assert_inits_motor_util(ioc(), &dir().join("st-pc6k.cmd"));
}
