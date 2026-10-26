//! st.cmd loads the records C's iocNewport loads for an HXP, on the
//! script's prefix.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_hxp_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/iocNewport.hxp.dbl")),
        &[("newport:", "HXP:"), ("xxx:hxp:c0:", "HXP:")],
        &[],
    );
    assert_same_records(&script_records(&dir.join("st.cmd")), &c);
}
