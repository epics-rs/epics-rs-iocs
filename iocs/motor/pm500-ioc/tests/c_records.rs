//! st.cmd boots with the records C's iocNewport st.cmd loads with its
//! PM500.cmd, on this IOC's prefix and axis.

use std::path::Path;

use ioc_conventions::{assert_same_records, booted_records, read_dbl, renamed};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.dbl")),
        &[("newport:m31", "PM500:m1"), ("newport:", "PM500:")],
        &["newport:m32", "newport:m33"],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_pm500-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}
