//! st.cmd loads the records C's iocNewFocus st.cmd loads, on this IOC's
//! prefix and PMNC87xx axis.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_records() {
    // C's newfocus8742.cmd loads its 874x records with P=$(PREFIX), which
    // st.cmd never sets, so C has none; this IOC configures no 874x.
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/st.cmd.dbl")),
        &[("newFocus:", "NF:")],
        &[
            "newFocus:m2",
            "newFocus:m3",
            "newFocus:m4",
            "newFocus:m5",
            "newFocus:m6",
        ],
    );
    assert_same_records(&script_records(&dir.join("st.cmd")), &c);
}
