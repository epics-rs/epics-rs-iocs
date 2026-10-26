//! st.cmd loads the records C's iocMclennan loads, on the script's prefix.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_pm304_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_dbl(&dir.join("tests/iocMclennan.dbl"));
    let want = renamed(&c, &[("mclennan:", "PM304:")], &[]);
    assert_same_records(&script_records(&dir.join("st.cmd")), &want);
}
