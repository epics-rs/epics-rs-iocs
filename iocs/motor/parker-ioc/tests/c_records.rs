//! Each startup script loads the records C's iocParker loads for its
//! controller, on the script's prefix and axes.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn st_cmd_loads_the_c_oem_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.oem.dbl")),
        &[("IOC:", "OEM:")],
        &["IOC:m2", "IOC:m3"],
    );
    assert_same_records(&script_records(&dir().join("st.cmd")), &c);
}
