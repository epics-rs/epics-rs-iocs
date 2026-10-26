//! Each script boots with the records C's iocIms st.cmd loads with its
//! controller's cmd file, on the script's prefix.

use std::collections::BTreeSet;
use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl,
    renamed,
};

fn assert_boots(script: &str, want: &BTreeSet<String>) {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_ims-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join(script)), want);
}

fn c_dbl(name: &str) -> BTreeSet<String> {
    read_dbl(
        &Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests")
            .join(name),
    )
}

/// C's MDrive-Eth.cmd loads no motor record; st.cmd adds the axis's
/// basic_asyn_motor.db, as st.cmd.ims loads it.
#[test]
fn st_cmd_loads_the_c_mdrive_eth_records() {
    let mut want = renamed(&c_dbl("iocIms.dbl"), &[("Test-CT", "ims:")], &[]);
    want.extend(["ims:1", "ims:1Direction", "ims:1Offset", "ims:1Resolution"].map(String::from));
    assert_boots("st.cmd", &want);
}

#[test]
fn st_im483sm_cmd_loads_the_c_im483_records() {
    let want = renamed(&c_dbl("iocIms.IM483.dbl"), &[], &[]);
    assert_boots("st-im483sm.cmd", &want);
}

#[test]
fn st_mdrive_cmd_loads_the_c_mdrive_records() {
    let want = renamed(&c_dbl("iocIms.MDrive.dbl"), &[], &[]);
    assert_boots("st-mdrive.cmd", &want);
}

/// The binary links asyn.dbd, as C's iocIms does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_ims-ioc")));
}

/// Each script with a C cmd file starts motorUtil after iocInit, as C's
/// iocIms st.cmd does.
#[test]
fn every_c_script_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_ims-ioc"));
    for script in ["st.cmd", "st-im483sm.cmd", "st-mdrive.cmd"] {
        assert_inits_motor_util(ioc, &dir.join(script));
    }
}
