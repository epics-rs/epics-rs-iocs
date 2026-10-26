//! st.cmd boots with the records C's iocPI st.cmd loads, on this IOC's
//! prefix and its C-862 axis.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl,
    renamed,
};

#[test]
fn st_cmd_loads_the_c_records() {
    // C's PI_C862.substitutions names its axis m3; C's other four
    // controllers are commented examples here.
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.dbl")),
        &[("pi:m3", "pi:c862")],
        &["pi:m1", "pi:m2", "pi:m4", "pi:m5"],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_pi-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocPI does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_pi-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocPI st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_pi-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
