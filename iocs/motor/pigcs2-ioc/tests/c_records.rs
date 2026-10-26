//! st.cmd loads the records C's iocPIGCS2 st.cmd loads, on this IOC's
//! prefix and its one axis.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/iocPIGCS2.dbl")),
        &[("pigcs2:", "pi:")],
        &["pigcs2:m2"],
    );
    assert_same_records(&script_records(&dir.join("st.cmd")), &c);
}
