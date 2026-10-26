//! st.cmd loads the records C's iocNewport loads for an MM4000, on the
//! script's prefix.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_mm4000_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/iocNewport.MM4000.dbl")),
        &[("newport:", "MM4000:")],
        &[],
    );
    assert_same_records(&script_records(&dir.join("st.cmd")), &c);
}
