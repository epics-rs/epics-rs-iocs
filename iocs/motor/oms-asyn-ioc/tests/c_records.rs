//! Each startup script loads the records C's iocOmsAsyn st.cmd loads for its
//! controller, on this IOC's prefix and axes.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

/// C's st.cmd names the MAXnet's axis m2 and the MAXv's m1; there is no MAXv.
#[test]
fn st_cmd_loads_the_c_maxnet_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/iocOmsAsyn.dbl")),
        &[("omsAsyn:", "oms:"), ("omsAsyn:m2", "oms:m1")],
        &["omsAsyn:m1"],
    );
    assert_same_records(&script_records(&dir().join("st.cmd")), &c);
}

#[test]
fn st_mxa_cmd_loads_the_c_mxa_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/MXA.dbl")),
        &[("omsAsyn:", "oms:")],
        &[],
    );
    assert_same_records(&script_records(&dir().join("st.mxa.cmd")), &c);
}
