//! st.cmd boots with the records C's iocNewport st.cmd loads with its
//! MM3000.cmd, on this IOC's prefix and axis.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl, renamed,
};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.dbl")),
        &[("newport:", "MM3000:")],
        &["newport:m2", "newport:m3", "newport:m4"],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_mm3000-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocNewport does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_mm3000-ioc")));
}
