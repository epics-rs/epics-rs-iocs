//! Each startup script boots with the records C's iocOmsAsyn st.cmd loads
//! for its controller, on this IOC's prefix.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_calls, assert_same_fields,
    assert_same_records, booted_records, read_calls, read_db, read_dbl, renamed, renamed_fields,
    script_calls, script_fields,
};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn ioc() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_oms-asyn-ioc"))
}

/// `script`'s loads and dbpf lines leave every field as C's `db` leaves it.
fn assert_sets_c_fields(script: &str, db: &str, renames: &[(&str, &str)], drops: &[&str]) {
    let c = renamed_fields(&read_db(&dir().join("tests").join(db)), renames, drops);
    assert_same_fields(&script_fields(&dir().join(script)), &c);
}

/// C's st.cmd loads the MAXv's m1 beside the MAXnet's m2; there is no MAXv.
const MAXNET_RENAMES: &[(&str, &str)] = &[("omsAsyn:", "oms:")];
const MAXNET_DROPS: &[&str] = &["omsAsyn:m1"];
const MXA_RENAMES: &[(&str, &str)] = &[("omsAsyn:", "oms:")];

#[test]
fn st_cmd_loads_the_c_maxnet_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/iocOmsAsyn.dbl")),
        MAXNET_RENAMES,
        MAXNET_DROPS,
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.cmd")), &c);
}

#[test]
fn st_cmd_sets_the_c_maxnet_fields() {
    let mut c = renamed_fields(
        &read_db(&dir().join("tests/iocOmsAsyn.db")),
        MAXNET_RENAMES,
        MAXNET_DROPS,
    );
    // C's row binds the axis to the octet port MAXNET, which has no
    // asynMotor; st.cmd binds it to the omsMAXnetConfig port.
    for value in c.values_mut().flat_map(|r| r.fields.values_mut()) {
        *value = value.replace("@asyn(MAXNET,", "@asyn(MAXnet1,");
    }
    assert_same_fields(&script_fields(&dir().join("st.cmd")), &c);
}

/// st.cmd makes C's controller calls, in C's order.
#[test]
fn st_cmd_makes_the_c_maxnet_calls() {
    let mut c = read_calls(&dir().join("tests/iocOmsAsyn.calls"));
    // There is no MAXv driver in rs (omsMAXvSetup/omsMAXvConfig).
    let before = c.len();
    c.retain(|w| !w[0].starts_with("omsMAXv"));
    assert_eq!(c.len(), before - 2);
    assert_same_calls(&script_calls(&dir().join("st.cmd")), &c);
}

#[test]
fn st_mxa_cmd_loads_the_c_mxa_records() {
    let c = renamed(&read_dbl(&dir().join("tests/MXA.dbl")), MXA_RENAMES, &[]);
    assert_same_records(&booted_records(ioc(), &dir().join("st.mxa.cmd")), &c);
}

#[test]
fn st_mxa_cmd_sets_the_c_mxa_fields() {
    assert_sets_c_fields("st.mxa.cmd", "MXA.db", MXA_RENAMES, &[]);
}

/// st.mxa.cmd makes C's controller calls, in C's order.
#[test]
fn st_mxa_cmd_makes_the_c_mxa_calls() {
    let c = read_calls(&dir().join("tests/MXA.calls"));
    assert_same_calls(&script_calls(&dir().join("st.mxa.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocOmsAsyn does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_oms-asyn-ioc")));
}

/// st.cmd starts motorUtil after iocInit, as C's iocOmsAsyn st.cmd does.
#[test]
fn st_cmd_inits_motor_util() {
    assert_inits_motor_util(ioc(), &dir().join("st.cmd"));
}

/// st.mxa.cmd starts motorUtil after iocInit, as C's iocOmsAsyn st.cmd does.
#[test]
fn st_mxa_cmd_inits_motor_util() {
    assert_inits_motor_util(ioc(), &dir().join("st.mxa.cmd"));
}
