//! st.cmd boots with the records C's iocNewport st.cmd.xps5 loads, on this
//! IOC's prefix.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_calls, assert_same_fields, assert_same_records,
    booted_records, read_calls, read_db, read_dbl, renamed, renamed_fields, script_calls,
    script_fields,
};

const RENAMES: &[(&str, &str)] = &[("IOC:", "XPS:")];

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.xps5.names")),
        RENAMES,
        &[],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_xps-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_fields() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed_fields(&read_db(&dir.join("tests/st.cmd.xps5.db")), RENAMES, &[]);
    assert_same_fields(&script_fields(&dir.join("st.cmd")), &c);
}

/// st.cmd makes C's controller calls, in C's order, on its own XPS host.
#[test]
fn st_cmd_makes_the_c_calls() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut c = read_calls(&dir.join("tests/st.cmd.xps5.calls"));
    for w in c.iter_mut().flatten().filter(|w| *w == "newport-xps4") {
        *w = "192.168.0.254".to_string();
    }
    assert_same_calls(&script_calls(&dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocNewport does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_xps-ioc")));
}
