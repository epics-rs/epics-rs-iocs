//! st.cmd boots with the records C's st.cmd.AG_UC loads, on the script's
//! prefix.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_calls, assert_same_fields, assert_same_records,
    booted_records, read_calls, read_db, read_dbl, renamed, renamed_fields, script_calls,
    script_fields,
};

const RENAMES: &[(&str, &str)] = &[("IOC:", "AGILIS:")];

#[test]
fn st_cmd_loads_the_c_ag_uc_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/iocNewport.AG_UC.dbl")),
        RENAMES,
        &[],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_ag-uc-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_ag_uc_fields() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed_fields(
        &read_db(&dir.join("tests/iocNewport.AG_UC.db")),
        RENAMES,
        &[],
    );
    assert_same_fields(&script_fields(&dir.join("st.cmd")), &c);
}

/// st.cmd makes C's controller calls, in C's order.
#[test]
fn st_cmd_makes_the_c_ag_uc_calls() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_calls(&dir.join("tests/iocNewport.AG_UC.calls"));
    assert_same_calls(&script_calls(&dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocNewport does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_ag-uc-ioc")));
}
