//! st.cmd boots with the records C's iocThorLabs st.cmd loads, on this
//! IOC's prefix and channels.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl,
    renamed,
};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_dbl(&dir.join("tests/st.cmd.dbl"));
    let mut want = renamed(&c, &[("thorlabs:", "THORLABS:")], &["thorlabs:m1"]);
    // C's MDT695.substitutions loads one channel, m1; the controller's
    // three take its records under this IOC's names.
    for m in ["THORLABS:x", "THORLABS:y", "THORLABS:z"] {
        want.extend(renamed(&c, &[("thorlabs:m1", m)], &["thorlabs:"]));
    }
    let ioc = Path::new(env!("CARGO_BIN_EXE_thorlabs-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &want);
}

/// The binary links asyn.dbd, as C's iocThorLabs does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_thorlabs-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocThorLabs st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_thorlabs-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
