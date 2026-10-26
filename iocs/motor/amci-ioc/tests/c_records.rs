//! st.cmd and st.ang1.cmd boot with the records C's iocAMCI loads for its
//! ANF2 and ANG1 examples, on the scripts' prefix.

use std::path::Path;

use ioc_conventions::{
    args, assert_inits_motor_util, assert_links_asyn_dbd, assert_same_calls, assert_same_fields,
    assert_same_records, booted_records, read_calls, read_db, read_dbl, renamed, renamed_fields,
    script_calls, script_fields,
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

/// st.cmd makes C's controller calls, in C's order.
#[test]
fn st_cmd_makes_the_c_anf2_calls() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_calls(&dir.join("tests/iocAMCI.calls"));
    assert_same_calls(&script_calls(&dir.join("st.cmd")), &c);
}

#[test]
fn st_ang1_cmd_loads_the_c_ang1_records() {
    assert_boots_as_c("st.ang1.cmd", "tests/iocAMCI.ANG1.dbl");
}

#[test]
fn st_ang1_cmd_sets_the_c_ang1_fields() {
    assert_sets_c_fields("st.ang1.cmd", "tests/iocAMCI.ANG1.db");
}

/// st.ang1.cmd makes C's controller calls, in C's order.
#[test]
fn st_ang1_cmd_makes_the_c_ang1_calls() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut c = read_calls(&dir.join("tests/iocAMCI.ANG1.calls"));
    // C's st.cmd still runs poller.cmd.ANF2 after st.cmd.ANG1 replaces
    // motor.cmd.ANF2, which defines no PORT1 or PORT2; those pollers name
    // no controller and fail, so st.ang1.cmd does not make them.
    let pollers = [
        args(r#"ANF2StartPoller("$(PORT1)", 200, 1000)"#),
        args(r#"ANF2StartPoller("$(PORT2)", 200, 1000)"#),
    ];
    let before = c.len();
    c.retain(|w| !pollers.contains(w));
    assert_eq!(c.len(), before - 2);
    assert_same_calls(&script_calls(&dir.join("st.ang1.cmd")), &c);
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
