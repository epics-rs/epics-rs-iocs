//! st.cmd loads the records C's st.cmd.AG_UC loads, on the script's prefix.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_ag_uc_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/iocNewport.AG_UC.dbl")),
        &[("IOC:", "AGILIS:")],
        &[],
    );
    assert_same_records(&script_records(&dir.join("st.cmd")), &c);
}
