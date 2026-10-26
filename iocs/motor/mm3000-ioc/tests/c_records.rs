//! st.cmd loads the records C's iocNewport st.cmd loads with its
//! MM3000.cmd, on this IOC's prefix and axis.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.dbl")),
        &[("newport:", "MM3000:")],
        &["newport:m2", "newport:m3", "newport:m4"],
    );
    assert_same_records(&script_records(&dir.join("st.cmd")), &c);
}
