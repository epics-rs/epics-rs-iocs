//! st.cmd boots with the records C's iocKohzu loads, on the script's prefix.

use std::path::Path;

use ioc_conventions::{assert_same_records, booted_records, read_dbl, renamed};

#[test]
fn st_cmd_loads_the_c_sc800_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let want = renamed(
        &read_dbl(&dir.join("tests/iocKohzu.dbl")),
        &[("kohzu:", "SC800:")],
        &[],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_kohzu-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &want);
}
