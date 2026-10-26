//! Each script boots with the records C's iocMicos st.cmd loads with its
//! controller's cmd file alone.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_fields, assert_same_records,
    booted_records, read_db, read_dbl, renamed, script_fields,
};

fn assert_boots_as_c(script: &str, dbl: &str) {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_dbl(&dir.join("tests").join(dbl));
    let ioc = Path::new(env!("CARGO_BIN_EXE_micos-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join(script)), &c);
}

/// `script`'s loads and dbpf lines leave every field as C's `db` leaves it.
fn assert_sets_c_fields(script: &str, db: &str) {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_db(&dir.join("tests").join(db));
    assert_same_fields(&script_fields(&dir.join(script)), &c);
}

#[test]
fn st_cmd_loads_the_c_corvus_records() {
    assert_boots_as_c("st.cmd", "iocMicos.corvus.dbl");
}

#[test]
fn st_cmd_sets_the_c_corvus_fields() {
    assert_sets_c_fields("st.cmd", "iocMicos.corvus.db");
}

#[test]
fn st_hydra_cmd_loads_the_c_hydra_records() {
    assert_boots_as_c("st.hydra.cmd", "iocMicos.hydra.dbl");
}

#[test]
fn st_hydra_cmd_sets_the_c_hydra_fields() {
    assert_sets_c_fields("st.hydra.cmd", "iocMicos.hydra.db");
}

#[test]
fn st_moco_cmd_loads_the_c_moco_records() {
    assert_boots_as_c("st.moco.cmd", "iocMicos.moco.dbl");
}

#[test]
fn st_moco_cmd_sets_the_c_moco_fields() {
    assert_sets_c_fields("st.moco.cmd", "iocMicos.moco.db");
}

/// motorMicos ships no Taurus example: st.taurus.cmd is corvus's layout for
/// one axis.
#[test]
fn st_taurus_cmd_loads_the_c_corvus_records_for_one_axis() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/iocMicos.corvus.dbl")),
        &[
            ("micos:corvusComm", "micos:taurusComm"),
            ("micos:corvusAsyn1", "micos:taurusAsyn1"),
        ],
        &["micos:m2", "micos:m3", "micos:corvusAsyn2"],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_micos-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.taurus.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocMicos does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_micos-ioc")));
}

/// Every script starts motorUtil after iocInit, as C's iocMicos st.cmd does.
#[test]
fn every_script_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_micos-ioc"));
    for script in ["st.cmd", "st.hydra.cmd", "st.moco.cmd", "st.taurus.cmd"] {
        assert_inits_motor_util(ioc, &dir.join(script));
    }
}
