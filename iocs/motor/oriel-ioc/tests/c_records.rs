//! st.cmd loads the records C's iocOriel st.cmd loads, on this IOC's prefix
//! and axes.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = read_dbl(&dir.join("tests/st.cmd.dbl"));
    let mut want = renamed(&c, &[("oriel:", "ORIEL:")], &[]);
    // C's EMC18011.substitutions loads one axis; the controller's other two
    // take its records.
    for m in ["ORIEL:m2", "ORIEL:m3"] {
        want.extend(renamed(&c, &[("oriel:m1", m)], &["oriel:"]));
    }
    assert_same_records(&script_records(&dir.join("st.cmd")), &want);
}
