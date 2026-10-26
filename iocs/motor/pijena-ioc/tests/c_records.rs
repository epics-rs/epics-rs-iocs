//! st.cmd boots with the records C's iocPiJena st.cmd loads, on this IOC's
//! prefix.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl,
    renamed,
};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.dbl")),
        &[("piJena:", "PIJEDS:")],
        &[],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_pijena-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocPiJena does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_pijena-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocPiJena st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_pijena-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
