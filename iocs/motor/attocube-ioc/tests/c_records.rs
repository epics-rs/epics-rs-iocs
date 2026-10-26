//! st.cmd boots with the records C's iocAttocube loads, on the script's prefix.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_fields, assert_same_records,
    booted_records, read_db, read_dbl, renamed, renamed_fields, script_fields,
};

const RENAMES: &[(&str, &str)] = &[("attocube:", "ANC150:")];

#[test]
fn st_cmd_loads_the_c_anc150_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(&read_dbl(&dir.join("tests/iocAttocube.dbl")), RENAMES, &[]);
    let ioc = Path::new(env!("CARGO_BIN_EXE_attocube-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_anc150_fields() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut c = renamed_fields(&read_db(&dir.join("tests/iocAttocube.db")), RENAMES, &[]);
    // C's rows bind the records to the octet port serial1, which has no
    // asynMotor; st.cmd binds them to the drvAsynMotorConfigure port C lacks.
    for value in c.values_mut().flat_map(|r| r.fields.values_mut()) {
        *value = value.replace("@asyn(serial1,", "@asyn(ANC150,");
    }
    assert_same_fields(&script_fields(&dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocAttocube does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_attocube-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocAttocube st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ioc = Path::new(env!("CARGO_BIN_EXE_attocube-ioc"));
    assert_inits_motor_util(ioc, &dir.join("st.cmd"));
}
