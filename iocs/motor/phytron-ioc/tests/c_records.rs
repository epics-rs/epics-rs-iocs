//! Each startup script loads the records C's iocPhytron loads for its
//! controller, on the script's prefix and axes.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn st_mcc_cmd_loads_the_c_mcc_records() {
    // C's I/O rows carry no prefix; this IOC's are under its P.
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.mcc.dbl")),
        &[
            ("TEST:", "phytron:"),
            ("phyA", "phytron:phyA"),
            ("phyD", "phytron:phyD"),
            ("phy:", "phytron:phy:"),
        ],
        &["TEST:m2"],
    );
    assert_same_records(&script_records(&dir().join("st.mcc.cmd")), &c);
}
