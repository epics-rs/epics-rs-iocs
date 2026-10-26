//! st.cmd loads the records C's iocFaulhaber loads, on the script's prefix.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_mcdc2805_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_dbl(&dir.join("tests/iocFaulhaber.dbl"));
    let want = renamed(&c, &[("faulhaber:", "MCDC:")], &[]);
    assert_same_records(&script_records(&dir.join("st.cmd")), &want);
}
