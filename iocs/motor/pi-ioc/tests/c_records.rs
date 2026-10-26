//! st.cmd loads the records C's iocPI st.cmd loads, on this IOC's prefix
//! and its C-862 axis.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_records() {
    // C's PI_C862.substitutions names its axis m3; C's other four
    // controllers are commented examples here.
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.dbl")),
        &[("pi:m3", "pi:c862")],
        &["pi:m1", "pi:m2", "pi:m4", "pi:m5"],
    );
    assert_same_records(&script_records(&dir.join("st.cmd")), &c);
}
