//! st.cmd boots with the records C's iocNPoint st.cmd loads with its
//! C300.cmd, on this IOC's prefix.

use std::path::Path;

use ioc_conventions::{assert_same_records, booted_records, read_dbl, renamed};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/C300.dbl")),
        &[("nPoint:", "NPOINT:")],
        &[],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_npoint-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}
