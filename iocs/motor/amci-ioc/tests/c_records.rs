//! st.cmd and st.ang1.cmd boot with the records C's iocAMCI loads for its
//! ANF2 and ANG1 examples, on the scripts' prefix.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_fields, assert_same_records,
    booted_records, read_db, read_dbl, renamed, renamed_fields, script_fields,
};

const RENAMES: &[(&str, &str)] = &[("amci:", "AMCI:")];

fn assert_boots_as_c(script: &str, dbl: &str) {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(&read_dbl(&dir.join(dbl)), RENAMES, &[]);
    let ioc = Path::new(env!("CARGO_BIN_EXE_amci-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join(script)), &c);
}

/// `script`'s loads and dbpf lines leave every field as C's `db` leaves it.
fn assert_sets_c_fields(script: &str, db: &str) {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed_fields(&read_db(&dir.join(db)), RENAMES, &[]);
    assert_same_fields(&script_fields(&dir.join(script)), &c);
}

#[test]
fn st_cmd_loads_the_c_anf2_records() {
    assert_boots_as_c("st.cmd", "tests/iocAMCI.dbl");
}

#[test]
fn st_cmd_sets_the_c_anf2_fields() {
    assert_sets_c_fields("st.cmd", "tests/iocAMCI.db");
}

#[test]
fn st_ang1_cmd_loads_the_c_ang1_records() {
    assert_boots_as_c("st.ang1.cmd", "tests/iocAMCI.ANG1.dbl");
}

#[test]
fn st_ang1_cmd_sets_the_c_ang1_fields() {
    assert_sets_c_fields("st.ang1.cmd", "tests/iocAMCI.ANG1.db");
}

/// The binary links asyn.dbd, as C's iocAMCI does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_amci-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocAMCI st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_amci-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}

/// st.ang1.cmd starts motorUtil after iocInit, as C's iocAMCI st.cmd does.
#[test]
fn st_ang1_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_amci-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.ang1.cmd"));
}
